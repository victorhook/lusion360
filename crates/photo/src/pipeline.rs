//! Photos/videos → dense triangle mesh, by driving COLMAP (structure from motion) and
//! OpenMVS (multi-view stereo, meshing, refinement) as subprocesses.
//!
//! Everything runs on the CPU (COLMAP's dense stereo needs CUDA; OpenMVS does not).
//! The work folder keeps every intermediate result; a stage that already finished is
//! skipped when the same folder is run again, so "Retry" after a failure is cheap.

use crate::frames::{self, Inputs};
use crate::tools::Toolchain;
use anyhow::{anyhow, bail, Context, Result};
use glam::{Quat, Vec3};
use lusion_meshio::TriMesh;
use std::collections::{HashMap, VecDeque};
use std::ffi::OsString;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::time::{Duration, Instant};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Quality {
    /// Quick look: small images, coarse depth maps.
    Draft,
    Normal,
    /// Large images + mesh refinement (slow on CPU).
    High,
}

impl Quality {
    pub const ALL: [Quality; 3] = [Quality::Draft, Quality::Normal, Quality::High];
    pub fn label(self) -> &'static str {
        match self {
            Quality::Draft => "Draft",
            Quality::Normal => "Normal",
            Quality::High => "High",
        }
    }
    pub fn hint(self) -> &'static str {
        match self {
            Quality::Draft => "Fast preview (≈ 2–10 min). Coarse detail.",
            Quality::Normal => "Good for most prints (≈ 5–30 min).",
            Quality::High => "Fine detail + mesh refinement. Slow on CPU (≈ 20–90 min).",
        }
    }
    /// Longest image side used for features and depth maps.
    fn max_image_size(self) -> u32 {
        match self {
            Quality::Draft => 1600,
            Quality::Normal => 2400,
            Quality::High => 3200,
        }
    }
    /// OpenMVS --resolution-level (each level halves the depth-map resolution).
    fn densify_level(self) -> u32 {
        1
    }
    fn refine(self) -> bool {
        self == Quality::High
    }
}

#[derive(Clone, Debug)]
pub struct Settings {
    pub quality: Quality,
    /// Frames kept per video.
    pub frames_per_video: usize,
    /// Override the quality's refinement choice.
    pub refine: Option<bool>,
    /// CPU threads for the tools (0 = all but two, so the desktop stays usable).
    pub threads: usize,
}

impl Settings {
    pub fn thread_count(&self) -> usize {
        if self.threads > 0 {
            return self.threads;
        }
        let n = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
        n.saturating_sub(2).max(1)
    }
}

impl Default for Settings {
    fn default() -> Self {
        Settings { quality: Quality::Normal, frames_per_video: 100, refine: None, threads: 0 }
    }
}

pub const STAGES: [&str; 8] = [
    "Preparing images",
    "Finding features",
    "Matching images",
    "Solving camera positions",
    "Undistorting images",
    "Dense point cloud",
    "Building mesh",
    "Refining mesh",
];

#[derive(Clone, Copy, Debug)]
pub struct CameraPose {
    /// Camera centre in scan coordinates.
    pub center: Vec3,
    /// World direction of the image's "up" (−y of the camera).
    pub up: Vec3,
    /// Viewing direction.
    pub forward: Vec3,
}

#[derive(Clone, Debug)]
pub struct ScanResult {
    pub mesh: TriMesh,
    pub mesh_path: PathBuf,
    pub workdir: PathBuf,
    pub cameras: Vec<CameraPose>,
    pub images_total: usize,
    pub images_registered: usize,
}

pub enum Event {
    Stage(usize),
    /// 0..1 within the current stage, if known.
    Progress(f32),
    Log(String),
    Finished(Result<ScanResult, String>),
}

pub struct Job {
    pub rx: Receiver<Event>,
    cancel: Arc<AtomicBool>,
    pub workdir: PathBuf,
}

impl Job {
    pub fn cancel(&self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
    pub fn poll(&self) -> Vec<Event> {
        self.rx.try_iter().collect()
    }
}

/// Default location for work folders: ~/.cache/lusion360/scans/<name>-<time>.
pub fn default_workdir(name: &str) -> PathBuf {
    let base = std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
        .unwrap_or_else(std::env::temp_dir);
    let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    let clean: String = name.chars().map(|c| if c.is_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
    base.join("lusion360").join("scans").join(format!("{}-{t}", if clean.is_empty() { "scan" } else { &clean }))
}

/// Start a reconstruction in a background thread.
pub fn start(inputs: Inputs, workdir: PathBuf, settings: Settings, tools: Toolchain) -> Job {
    let (tx, rx) = channel();
    let cancel = Arc::new(AtomicBool::new(false));
    let c2 = cancel.clone();
    let wd = workdir.clone();
    std::thread::Builder::new()
        .name("photogrammetry".into())
        .spawn(move || {
            let mut r = Runner::new(tx.clone(), c2, wd, settings, tools);
            let res = r.run(&inputs).map_err(|e| format!("{e:#}"));
            if let Err(e) = &res {
                r.log(format!("FAILED: {e}"));
            }
            let _ = tx.send(Event::Finished(res));
        })
        .expect("spawning photogrammetry thread");
    Job { rx, cancel, workdir }
}

/// Run synchronously (CLI); `on_event` sees every event except `Finished`.
pub fn run_blocking(inputs: Inputs, workdir: PathBuf, settings: Settings, tools: Toolchain, mut on_event: impl FnMut(Event)) -> Result<ScanResult> {
    let job = start(inputs, workdir, settings, tools);
    for ev in job.rx.iter() {
        match ev {
            Event::Finished(r) => return r.map_err(|e| anyhow!(e)),
            other => on_event(other),
        }
    }
    bail!("photogrammetry thread ended unexpectedly")
}

struct Runner {
    tx: Sender<Event>,
    cancel: Arc<AtomicBool>,
    wd: PathBuf,
    settings: Settings,
    tools: Toolchain,
    logfile: Option<std::fs::File>,
    help_cache: HashMap<String, String>,
    images_total: usize,
}

/// How to turn a subprocess output line into stage progress.
#[derive(Clone, Copy)]
enum Meter {
    None,
    /// "[i/n]" style counters (COLMAP extraction, sequential matching)
    Bracket,
    /// "Matching block [i/n, j/m]"
    Blocks,
    /// "Registering image #k (n)" with n out of `total`
    Registered(usize),
    /// "(12.5%" style percentages (OpenMVS)
    Percent,
}

fn parse_progress(m: Meter, line: &str) -> Option<f32> {
    fn nums(s: &str) -> Vec<f64> {
        s.split(|c: char| !(c.is_ascii_digit() || c == '.')).filter_map(|t| t.parse().ok()).collect()
    }
    match m {
        Meter::None => None,
        Meter::Bracket => {
            let a = line.find('[')?;
            let b = line[a..].find(']')? + a;
            let n = nums(&line[a + 1..b]);
            (n.len() >= 2 && n[1] > 0.0).then(|| (n[0] / n[1]) as f32)
        }
        Meter::Blocks => {
            let a = line.find("block [")? + 6;
            let b = line[a..].find(']')? + a;
            let n = nums(&line[a + 1..b]);
            if n.len() >= 4 && n[1] > 0.0 && n[3] > 0.0 {
                Some((((n[0] - 1.0) * n[3] + n[2]) / (n[1] * n[3])) as f32)
            } else {
                None
            }
        }
        Meter::Registered(total) => {
            let a = line.find("Registering image #")?;
            let rest = &line[a..];
            let p = rest.find('(')?;
            let q = rest[p..].find(')')? + p;
            let k: f64 = rest[p + 1..q].trim().parse().ok()?;
            (total > 0).then(|| (k / total as f64) as f32)
        }
        Meter::Percent => {
            let p = line.find('%')?;
            let start = line[..p].rfind(|c: char| !(c.is_ascii_digit() || c == '.')).map(|i| i + 1).unwrap_or(0);
            line[start..p].parse::<f32>().ok().map(|v| v / 100.0)
        }
    }
}

impl Runner {
    fn new(tx: Sender<Event>, cancel: Arc<AtomicBool>, wd: PathBuf, settings: Settings, tools: Toolchain) -> Self {
        Runner { tx, cancel, wd, settings, tools, logfile: None, help_cache: HashMap::new(), images_total: 0 }
    }

    fn log(&mut self, s: impl Into<String>) {
        let s = s.into();
        if let Some(f) = self.logfile.as_mut() {
            let _ = writeln!(f, "{s}");
        }
        let _ = self.tx.send(Event::Log(s));
    }

    fn stage(&mut self, i: usize) {
        self.log(format!("== {} ==", STAGES[i]));
        let _ = self.tx.send(Event::Stage(i));
        let _ = self.tx.send(Event::Progress(0.0));
    }

    fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::Relaxed)
    }

    fn done_marker(&self, i: usize) -> PathBuf {
        self.wd.join(format!(".done-{i}"))
    }
    fn is_done(&self, i: usize) -> bool {
        self.done_marker(i).exists()
    }
    fn mark_done(&self, i: usize) {
        let _ = std::fs::write(self.done_marker(i), b"");
    }

    /// `colmap <cmd> -h` text, to pick option names that differ between COLMAP versions.
    fn colmap_help(&mut self, cmd: &str) -> String {
        if let Some(h) = self.help_cache.get(cmd) {
            return h.clone();
        }
        let colmap = self.tools.colmap.clone().unwrap_or_default();
        let text = Command::new(colmap)
            .args([cmd, "-h"])
            .env("QT_QPA_PLATFORM", "offscreen")
            .output()
            .map(|o| String::from_utf8_lossy(&o.stdout).to_string() + &String::from_utf8_lossy(&o.stderr))
            .unwrap_or_default();
        self.help_cache.insert(cmd.into(), text.clone());
        text
    }

    /// The first of `names` that `colmap <cmd>` accepts.
    fn colmap_opt(&mut self, cmd: &str, names: &[&str]) -> Option<String> {
        let h = self.colmap_help(cmd);
        names.iter().find(|n| h.contains(*n)).map(|n| format!("--{n}"))
    }

    /// Run a program, streaming its output to the log and progress to the UI.
    fn exec(&mut self, prog: &Path, args: Vec<OsString>, cwd: &Path, meter: Meter) -> Result<()> {
        let pretty = format!(
            "$ {} {}",
            prog.file_name().unwrap_or_default().to_string_lossy(),
            args.iter().map(|a| a.to_string_lossy().to_string()).collect::<Vec<_>>().join(" ")
        );
        self.log(pretty);
        // be nice to the desktop: reconstruction saturates every core for a long time
        let mut cmd = if cfg!(unix) && crate::tools::in_path("nice").is_some() {
            let mut c = Command::new("nice");
            c.args(["-n", "10"]).arg(prog);
            c
        } else {
            Command::new(prog)
        };
        cmd.args(&args)
            .current_dir(cwd)
            .env("QT_QPA_PLATFORM", "offscreen")
            .env("OMP_NUM_THREADS", self.settings.thread_count().to_string())
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped());
        let mut child = cmd.spawn().with_context(|| format!("starting {}", prog.display()))?;
        let (ltx, lrx) = channel::<String>();
        let mut readers = Vec::new();
        for stream in [child.stdout.take().map(|s| Box::new(s) as Box<dyn Read + Send>), child.stderr.take().map(|s| Box::new(s) as Box<dyn Read + Send>)]
            .into_iter()
            .flatten()
        {
            let ltx = ltx.clone();
            readers.push(std::thread::spawn(move || {
                let mut stream = stream;
                let mut buf = [0u8; 4096];
                let mut line = Vec::new();
                loop {
                    match stream.read(&mut buf) {
                        Ok(0) | Err(_) => break,
                        Ok(n) => {
                            for &b in &buf[..n] {
                                if b == b'\n' || b == b'\r' {
                                    if !line.is_empty() {
                                        let _ = ltx.send(String::from_utf8_lossy(&line).to_string());
                                        line.clear();
                                    }
                                } else {
                                    line.push(b);
                                }
                            }
                        }
                    }
                }
                if !line.is_empty() {
                    let _ = ltx.send(String::from_utf8_lossy(&line).to_string());
                }
            }));
        }
        drop(ltx);
        let mut tail: VecDeque<String> = VecDeque::new();
        let mut last_progress = Instant::now();
        let status = loop {
            if self.cancelled() {
                let _ = child.kill();
                let _ = child.wait();
                bail!("cancelled");
            }
            match lrx.recv_timeout(Duration::from_millis(100)) {
                Ok(line) => {
                    // glog prefix "I20241117 12:00:00.123 1234 file.cc:12] " is noise in the UI
                    let clean = match line.find("] ") {
                        Some(i) if line.starts_with(['I', 'W', 'E', 'F']) && i < 60 => line[i + 2..].to_string(),
                        _ => line.clone(),
                    };
                    if let Some(p) = parse_progress(meter, &clean) {
                        if last_progress.elapsed() > Duration::from_millis(50) || p >= 1.0 {
                            let _ = self.tx.send(Event::Progress(p.clamp(0.0, 1.0)));
                            last_progress = Instant::now();
                        }
                    }
                    tail.push_back(clean.clone());
                    if tail.len() > 30 {
                        tail.pop_front();
                    }
                    self.log(clean);
                }
                Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {
                    if let Some(st) = child.try_wait()? {
                        // drain the rest
                        for line in lrx.try_iter() {
                            self.log(line);
                        }
                        break st;
                    }
                }
                Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break child.wait()?,
            }
        };
        for r in readers {
            let _ = r.join();
        }
        if !status.success() {
            let name = prog.file_name().unwrap_or_default().to_string_lossy().to_string();
            let hint: Vec<String> = tail.iter().rev().take(6).rev().cloned().collect();
            bail!("{name} failed ({status}):\n{}", hint.join("\n"));
        }
        Ok(())
    }

    fn colmap(&self) -> Result<PathBuf> {
        self.tools.colmap.clone().ok_or_else(|| anyhow!("COLMAP not found"))
    }
    fn mvs(&self, name: &str) -> Result<PathBuf> {
        self.tools.openmvs_tool(name).ok_or_else(|| anyhow!("OpenMVS {name} not found"))
    }

    fn run(&mut self, inputs: &Inputs) -> Result<ScanResult> {
        std::fs::create_dir_all(&self.wd).with_context(|| format!("creating {}", self.wd.display()))?;
        self.logfile = std::fs::OpenOptions::new().create(true).append(true).open(self.wd.join("log.txt")).ok();
        let missing = self.tools.missing(!inputs.videos.is_empty());
        if !missing.is_empty() {
            bail!("missing tools:\n  {}", missing.join("\n  "));
        }
        self.log(format!("work folder: {}", self.wd.display()));
        self.log(format!("{} | OpenMVS: {}", self.tools.colmap_version, self.tools.openmvs.as_ref().map(|p| p.display().to_string()).unwrap_or_default()));
        let q = self.settings.quality;
        let max_size = q.max_image_size();
        let images = self.wd.join("images");
        let db = self.wd.join("database.db");
        let sparse = self.wd.join("sparse");
        let dense = self.wd.join("dense");
        let mvs = self.wd.join("mvs");
        let colmap = self.colmap()?;

        // ---- 0: images
        self.stage(0);
        if !self.is_done(0) {
            let _ = std::fs::remove_dir_all(&images);
            std::fs::create_dir_all(&images)?;
            let mut n = 0;
            if !inputs.images.is_empty() {
                let mut logs = Vec::new();
                n += frames::place_photos(&inputs.images, &images, &mut |s| logs.push(s))?;
                for l in logs {
                    self.log(l);
                }
            }
            let (ffmpeg, ffprobe) = (self.tools.ffmpeg.clone(), self.tools.ffprobe.clone());
            for (k, v) in inputs.videos.iter().enumerate() {
                let (Some(ff), Some(fp)) = (&ffmpeg, &ffprobe) else { bail!("ffmpeg not found") };
                let mut logs = Vec::new();
                let got = frames::extract_video_frames(ff, fp, v, &images.join(format!("video{k}")), self.settings.frames_per_video, max_size, &self.cancel, &mut |s| logs.push(s))?;
                for l in logs {
                    self.log(l);
                }
                self.log(format!("kept {got} sharp frames from {}", v.display()));
                n += got;
                let _ = self.tx.send(Event::Progress((k + 1) as f32 / inputs.videos.len() as f32));
            }
            if n < 3 {
                bail!("need at least 3 images (got {n})");
            }
            std::fs::write(self.wd.join("image_count"), n.to_string())?;
            self.mark_done(0);
        }
        self.images_total = std::fs::read_to_string(self.wd.join("image_count")).ok().and_then(|s| s.trim().parse().ok()).unwrap_or(0);
        self.log(format!("{} images", self.images_total));
        let video_only = inputs.images.is_empty();

        // ---- 1: features
        self.stage(1);
        if !self.is_done(1) {
            let _ = std::fs::remove_file(&db);
            let mut args: Vec<OsString> = vec!["feature_extractor".into(), "--database_path".into(), db.clone().into(), "--image_path".into(), images.clone().into()];
            args.extend(["--ImageReader.single_camera_per_folder".into(), "1".into()]);
            if let Some(o) = self.colmap_opt("feature_extractor", &["FeatureExtraction.use_gpu", "SiftExtraction.use_gpu"]) {
                args.extend([o.into(), if self.tools.colmap_cuda { "1" } else { "0" }.into()]);
            }
            if let Some(o) = self.colmap_opt("feature_extractor", &["FeatureExtraction.max_image_size", "SiftExtraction.max_image_size"]) {
                args.extend([o.into(), max_size.to_string().into()]);
            }
            if let Some(o) = self.colmap_opt("feature_extractor", &["FeatureExtraction.num_threads", "SiftExtraction.num_threads"]) {
                args.extend([o.into(), self.settings.thread_count().to_string().into()]);
            }
            // CPU matching cost grows with features²: 8192 (the default) makes exhaustive
            // matching take many minutes without a GPU
            if !self.tools.colmap_cuda {
                if let Some(o) = self.colmap_opt("feature_extractor", &["FeatureExtraction.max_num_features", "SiftExtraction.max_num_features"]) {
                    let n = match q {
                        Quality::Draft => 3000,
                        Quality::Normal => 4096,
                        Quality::High => 6000,
                    };
                    args.extend([o.into(), n.to_string().into()]);
                }
            }
            // better features on low-texture objects (slower, but matching dominates anyway)
            if let Some(o) = self.colmap_opt("feature_extractor", &["SiftExtraction.estimate_affine_shape"]) {
                if q == Quality::High {
                    args.extend([o.into(), "1".into()]);
                }
            }
            self.exec(&colmap, args, &self.wd.clone(), Meter::Bracket)?;
            self.mark_done(1);
        }
        if self.cancelled() {
            bail!("cancelled");
        }

        // ---- 2: matching
        self.stage(2);
        if !self.is_done(2) {
            let sequential = video_only && self.images_total > 40;
            let cmd = if sequential { "sequential_matcher" } else { "exhaustive_matcher" };
            let mut args: Vec<OsString> = vec![cmd.into(), "--database_path".into(), db.clone().into()];
            if let Some(o) = self.colmap_opt(cmd, &["FeatureMatching.use_gpu", "SiftMatching.use_gpu"]) {
                args.extend([o.into(), if self.tools.colmap_cuda { "1" } else { "0" }.into()]);
            }
            if let Some(o) = self.colmap_opt(cmd, &["FeatureMatching.num_threads", "SiftMatching.num_threads"]) {
                args.extend([o.into(), self.settings.thread_count().to_string().into()]);
            }
            if !sequential {
                // small blocks → visible progress (one 50-image block reports nothing until done)
                let bs = (self.images_total / 6).clamp(8, 50);
                args.extend(["--ExhaustiveMatching.block_size".into(), bs.to_string().into()]);
            }
            if sequential {
                args.extend(["--SequentialMatching.overlap".into(), "20".into(), "--SequentialMatching.quadratic_overlap".into(), "1".into()]);
            }
            self.exec(&colmap, args, &self.wd.clone(), if sequential { Meter::Bracket } else { Meter::Blocks })?;
            self.mark_done(2);
        }

        // ---- 3: mapper
        self.stage(3);
        if !self.is_done(3) {
            let _ = std::fs::remove_dir_all(&sparse);
            std::fs::create_dir_all(&sparse)?;
            let mut args: Vec<OsString> = vec![
                "mapper".into(),
                "--database_path".into(),
                db.clone().into(),
                "--image_path".into(),
                images.clone().into(),
                "--output_path".into(),
                sparse.clone().into(),
            ];
            args.extend(["--Mapper.num_threads".into(), self.settings.thread_count().to_string().into()]);
            let total = self.images_total;
            self.exec(&colmap, args, &self.wd.clone(), Meter::Registered(total))?;
            self.mark_done(3);
        }
        // largest model (COLMAP may split the scene into several)
        let model = std::fs::read_dir(&sparse)?
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.join("images.bin").exists())
            .max_by_key(|p| std::fs::metadata(p.join("images.bin")).map(|m| m.len()).unwrap_or(0))
            .ok_or_else(|| anyhow!("COLMAP could not solve the camera positions. Try more photos with more overlap (each spot of the object should appear in 3+ photos), sharper images, or a textured background."))?;

        // ---- 4: undistort (+ text model for camera poses)
        self.stage(4);
        if !self.is_done(4) {
            let _ = std::fs::remove_dir_all(&dense);
            let args: Vec<OsString> = vec![
                "image_undistorter".into(),
                "--image_path".into(),
                images.clone().into(),
                "--input_path".into(),
                model.clone().into(),
                "--output_path".into(),
                dense.clone().into(),
                "--output_type".into(),
                "COLMAP".into(),
                "--max_image_size".into(),
                max_size.to_string().into(),
            ];
            self.exec(&colmap, args, &self.wd.clone(), Meter::Bracket)?;
            // OpenMVS' importer reads the text format
            let args: Vec<OsString> = vec![
                "model_converter".into(),
                "--input_path".into(),
                dense.join("sparse").into(),
                "--output_path".into(),
                dense.join("sparse").into(),
                "--output_type".into(),
                "TXT".into(),
            ];
            self.exec(&colmap, args, &self.wd.clone(), Meter::None)?;
            self.mark_done(4);
        }
        let cameras = read_colmap_cameras(&dense.join("sparse/images.txt")).unwrap_or_default();
        let registered = cameras.len();
        self.log(format!("{registered} of {} images have a solved camera", self.images_total));
        if registered < 3 {
            bail!("only {registered} images could be placed — need more overlapping photos");
        }
        if registered * 2 < self.images_total {
            self.log(format!("warning: only {registered}/{} images were used; parts of the object may be missing", self.images_total));
        }

        // ---- 5: densify
        std::fs::create_dir_all(&mvs)?;
        self.stage(5);
        let scene = mvs.join("scene.mvs");
        let scene_dense = mvs.join("scene_dense.mvs");
        if !self.is_done(5) {
            let args: Vec<OsString> = vec![
                "-i".into(),
                dense.clone().into(),
                "-o".into(),
                scene.clone().into(),
                "--image-folder".into(),
                dense.join("images").into(),
                "-w".into(),
                mvs.clone().into(),
            ];
            self.exec(&self.mvs("InterfaceCOLMAP")?, args, &mvs, Meter::None)?;
            let args: Vec<OsString> = vec![
                "-i".into(),
                scene.clone().into(),
                "-o".into(),
                scene_dense.clone().into(),
                "-w".into(),
                mvs.clone().into(),
                "--resolution-level".into(),
                q.densify_level().to_string().into(),
                "--max-resolution".into(),
                max_size.to_string().into(),
                "--archive-type".into(),
                "-1".into(),
                "-v".into(),
                "2".into(),
                "--max-threads".into(),
                self.settings.thread_count().to_string().into(),
            ];
            self.exec(&self.mvs("DensifyPointCloud")?, args, &mvs, Meter::Percent)?;
            self.mark_done(5);
        }

        // ---- 6: mesh
        self.stage(6);
        let mesh_ply = mvs.join("scene_mesh.ply");
        if !self.is_done(6) {
            let args: Vec<OsString> = vec![
                "-i".into(),
                scene_dense.clone().into(),
                "-o".into(),
                mvs.join("scene_mesh.mvs").into(),
                "-w".into(),
                mvs.clone().into(),
                "--archive-type".into(),
                "-1".into(),
                "-v".into(),
                "2".into(),
                "--max-threads".into(),
                self.settings.thread_count().to_string().into(),
            ];
            self.exec(&self.mvs("ReconstructMesh")?, args, &mvs, Meter::Percent)?;
            self.mark_done(6);
        }

        // ---- 7: refine
        let refine = self.settings.refine.unwrap_or(q.refine());
        let refined_ply = mvs.join("scene_refined.ply");
        if refine {
            self.stage(7);
            if !self.is_done(7) {
                let args: Vec<OsString> = vec![
                    "-i".into(),
                    scene_dense.clone().into(),
                    "-m".into(),
                    mesh_ply.clone().into(),
                    "-o".into(),
                    mvs.join("scene_refined.mvs").into(),
                    "-w".into(),
                    mvs.clone().into(),
                    "--resolution-level".into(),
                    "1".into(),
                    "--max-face-area".into(),
                    "16".into(),
                    "--archive-type".into(),
                    "-1".into(),
                    "-v".into(),
                    "2".into(),
                    "--max-threads".into(),
                    self.settings.thread_count().to_string().into(),
                ];
                self.exec(&self.mvs("RefineMesh")?, args, &mvs, Meter::Percent)?;
                self.mark_done(7);
            }
        }
        let out = if refine && refined_ply.exists() { refined_ply } else { mesh_ply };
        if !out.exists() {
            bail!("OpenMVS produced no mesh ({} missing)", out.display());
        }
        self.log(format!("loading {}", out.display()));
        let mut meshes = lusion_meshio::load_mesh_file(&out)?;
        let mesh = meshes.pop().filter(|m| !m.indices.is_empty()).ok_or_else(|| anyhow!("the reconstructed mesh is empty"))?;
        self.log(format!("mesh: {} triangles", mesh.indices.len() / 3));
        let freed = cleanup_intermediates(&self.wd);
        self.log(format!("removed {:.0} MB of intermediate files", freed as f64 / 1e6));
        Ok(ScanResult { mesh, mesh_path: out, workdir: self.wd.clone(), cameras, images_total: self.images_total, images_registered: registered })
    }
}

fn dir_size(p: &Path) -> u64 {
    match std::fs::symlink_metadata(p) {
        Ok(m) if m.is_dir() => std::fs::read_dir(p).map(|rd| rd.flatten().map(|e| dir_size(&e.path())).sum()).unwrap_or(0),
        Ok(m) => m.len(),
        Err(_) => 0,
    }
}

/// After a successful run: drop depth maps, undistorted image copies and point clouds
/// (gigabytes), keep the mesh, camera poses and log. Returns the bytes freed.
pub fn cleanup_intermediates(wd: &Path) -> u64 {
    let mut freed = 0;
    for d in [wd.join("dense/images"), wd.join("dense/stereo"), wd.join("images")] {
        freed += dir_size(&d);
        let _ = std::fs::remove_dir_all(&d);
    }
    if let Ok(rd) = std::fs::read_dir(wd.join("mvs")) {
        for e in rd.flatten() {
            let p = e.path();
            let name = p.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let keep = name == "scene_mesh.ply" || name == "scene_refined.ply";
            if !keep && (name.ends_with(".dmap") || name.ends_with(".ply") || name.ends_with(".mvs") || name.ends_with(".log")) {
                freed += dir_size(&p);
                let _ = std::fs::remove_file(&p);
            }
        }
    }
    // stages can't be resumed any more; a re-run starts from scratch
    for i in 0..STAGES.len() {
        let _ = std::fs::remove_file(wd.join(format!(".done-{i}")));
    }
    freed
}

/// Rough share of the total run time per stage (CPU): dense matching dominates.
pub const STAGE_WEIGHTS: [f32; 8] = [1.0, 6.0, 12.0, 8.0, 2.0, 50.0, 12.0, 30.0];

/// Overall progress 0..1 given the current stage and its progress.
pub fn overall_progress(stage: usize, within: f32, refine: bool) -> f32 {
    let n = if refine { STAGES.len() } else { STAGES.len() - 1 };
    let total: f32 = STAGE_WEIGHTS[..n].iter().sum();
    let done: f32 = STAGE_WEIGHTS[..stage.min(n)].iter().sum();
    ((done + STAGE_WEIGHTS[stage.min(n - 1)] * within.clamp(0.0, 1.0)) / total).clamp(0.0, 1.0)
}

/// Camera poses from a COLMAP `images.txt`.
pub fn read_colmap_cameras(path: &Path) -> Result<Vec<CameraPose>> {
    let text = std::fs::read_to_string(path)?;
    let mut out = Vec::new();
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    while let Some(l) = lines.next() {
        let f: Vec<&str> = l.split_whitespace().collect();
        if f.len() < 10 {
            continue;
        }
        let v: Vec<f32> = f[1..8].iter().filter_map(|s| s.parse().ok()).collect();
        if v.len() != 7 {
            continue;
        }
        // world → camera rotation
        let r = Quat::from_xyzw(v[1], v[2], v[3], v[0]).normalize();
        let t = Vec3::new(v[4], v[5], v[6]);
        let rt = r.inverse();
        out.push(CameraPose { center: -(rt * t), up: rt * Vec3::NEG_Y, forward: rt * Vec3::Z });
        let _ = lines.next(); // 2D points line
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn progress_parsing() {
        assert_eq!(parse_progress(Meter::Bracket, "Processed file [3/12]"), Some(0.25));
        let b = parse_progress(Meter::Blocks, "Matching block [2/2, 1/2]").unwrap();
        assert!((b - 0.75).abs() < 1e-6);
        assert_eq!(parse_progress(Meter::Registered(10), "Registering image #7 (5)"), Some(0.5));
        let p = parse_progress(Meter::Percent, "Estimated depth-maps 12 (35.29%, 1m2s, ETA 1m54s)...").unwrap();
        assert!((p - 0.3529).abs() < 1e-4);
    }

    #[test]
    fn weighted_progress() {
        assert_eq!(overall_progress(0, 0.0, false), 0.0);
        assert!((overall_progress(6, 1.0, false) - 1.0).abs() < 1e-6);
        // half-way through dense ≈ well under the stage-count estimate (5.5/7)
        let mid = overall_progress(5, 0.5, false);
        assert!(mid > 0.4 && mid < 0.7, "{mid}");
        assert!(overall_progress(6, 1.0, true) < 1.0);
    }
}
