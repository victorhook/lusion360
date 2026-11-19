//! Photogrammetry: photos / video → printable mesh body.
//!
//! Three phases in one window:
//! 1. **Setup** — choose photos, folders or videos; checks that COLMAP / OpenMVS are installed.
//! 2. **Running** — the reconstruction runs in the background (minutes); progress per stage.
//! 3. **Prepare** — stand the scan on the table, cut away the background, set the real size,
//!    make it a closed solid. A live preview is shown in the viewport as an (uncommitted)
//!    mesh feature; "Insert" commits it as one undo step.

use crate::app::App;
use crate::icons as ic;
use crate::theme;
use egui::{Color32, RichText};
use lusion_core::doc::*;
use lusion_photo::pipeline::{self, Event, Job, STAGES};
use lusion_photo::prepare::{AlignSource, Alignment, PrepSettings, Prepared, SizeAxis};
use lusion_photo::{Inputs, Mesh, Quality, Settings, Toolchain};
use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{channel, Receiver};
use std::sync::Arc;
use std::time::Instant;

#[derive(PartialEq)]
enum Phase {
    Setup,
    Running,
    Prepare,
}

/// Background preparation result.
enum PrepMsg {
    Aligned(Alignment),
    /// Fast preview (no solid yet) or the final result.
    Result { gen: u64, final_: bool, res: Result<Prepared, String> },
}

pub struct ScanUi {
    phase: Phase,
    // setup
    paths: Vec<PathBuf>,
    inputs: Inputs,
    settings: Settings,
    tools: Toolchain,
    name: String,
    // running
    job: Option<Job>,
    stage: usize,
    progress: f32,
    started: Instant,
    stage_started: Instant,
    log: VecDeque<String>,
    error: Option<String>,
    workdir: Option<PathBuf>,
    // prepare
    raw: Option<Arc<Mesh>>,
    cams: Arc<Vec<pipeline::CameraPose>>,
    align: Option<Alignment>,
    auto_align: Option<Alignment>,
    prep: PrepSettings,
    show_all: bool,
    prep_rx: Option<Receiver<PrepMsg>>,
    gen: Arc<AtomicU64>,
    applied: Option<(PrepSettings, bool, u64)>,
    changed_at: Option<Instant>,
    busy: bool,
    result: Option<Prepared>,
    result_final: bool,
    prep_error: Option<String>,
    preview: Option<FeatureId>,
    /// `doc.next_id` before the preview feature was added (restored on discard).
    prev_next_id: u32,
    fitted: bool,
}

impl ScanUi {
    fn new() -> Self {
        ScanUi {
            phase: Phase::Setup,
            paths: vec![],
            inputs: Inputs::default(),
            settings: Settings::default(),
            tools: Toolchain::detect(),
            name: "Scan".into(),
            job: None,
            stage: 0,
            progress: 0.0,
            started: Instant::now(),
            stage_started: Instant::now(),
            log: VecDeque::new(),
            error: None,
            workdir: None,
            raw: None,
            cams: Arc::new(vec![]),
            align: None,
            auto_align: None,
            prep: PrepSettings::default(),
            show_all: false,
            prep_rx: None,
            gen: Arc::new(AtomicU64::new(0)),
            applied: None,
            changed_at: None,
            busy: false,
            result: None,
            result_final: false,
            prep_error: None,
            preview: None,
            prev_next_id: 0,
            fitted: false,
        }
    }

    fn add_paths(&mut self, ps: impl IntoIterator<Item = PathBuf>) {
        for p in ps {
            if !self.paths.contains(&p) {
                self.paths.push(p);
            }
        }
        self.inputs = Inputs::collect(&self.paths);
        if let Some(first) = self.paths.first() {
            let stem = if first.is_dir() { first.file_name() } else { first.parent().and_then(|d| d.file_name()).or(first.file_stem()) };
            self.name = stem.map(|s| s.to_string_lossy().to_string()).unwrap_or_else(|| "Scan".into());
        }
    }

    pub fn running(&self) -> bool {
        self.job.is_some()
    }
}

/// Open the photogrammetry window.
pub fn open(app: &mut App) {
    if app.scan.is_none() {
        app.scan = Some(Box::new(ScanUi::new()));
    }
}

/// Files dropped while the setup page is showing go to the scan, not the design.
pub fn take_drop(app: &mut App, paths: &[PathBuf]) -> bool {
    let Some(s) = app.scan.as_mut() else { return false };
    if s.phase != Phase::Setup {
        return false;
    }
    let ok: Vec<PathBuf> = paths
        .iter()
        .filter(|p| p.is_dir() || lusion_photo::frames::is_image(p) || lusion_photo::frames::is_video(p))
        .cloned()
        .collect();
    if ok.is_empty() {
        return false;
    }
    s.add_paths(ok);
    true
}

fn fmt_dur(secs: f32) -> String {
    let s = secs as u64;
    if s >= 3600 {
        format!("{}h {:02}m", s / 3600, (s / 60) % 60)
    } else if s >= 60 {
        format!("{}m {:02}s", s / 60, s % 60)
    } else {
        format!("{s}s")
    }
}

static STARTUP: std::sync::Mutex<Option<Vec<PathBuf>>> = std::sync::Mutex::new(None);

/// Inputs from the command line (`--scan …`), picked up on the first frame.
pub fn set_startup(paths: Vec<PathBuf>) {
    if let Ok(mut g) = STARTUP.lock() {
        *g = Some(paths);
    }
}

fn is_mesh_file(p: &std::path::Path) -> bool {
    let e = p.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    lusion_meshio::supported_import_extensions().contains(&e.as_str())
}

pub fn ui(app: &mut App, ctx: &egui::Context) {
    if let Some(paths) = STARTUP.lock().ok().and_then(|mut g| g.take()) {
        open(app);
        if let Some(mut s) = app.scan.take() {
            match paths.as_slice() {
                [one] if is_mesh_file(one) => load_mesh_for_prepare(app, &mut s, one),
                _ => s.add_paths(paths),
            }
            app.scan = Some(s);
        }
    }
    let Some(mut s) = app.scan.take() else { return };
    poll_job(app, &mut s);
    poll_prep(app, &mut s, ctx);
    if s.running() || s.busy {
        ctx.request_repaint_after(std::time::Duration::from_millis(150));
    }
    let mut keep = true;
    let title = match s.phase {
        Phase::Setup => format!("{} Scan an object (photogrammetry)", ic::CAMERA),
        Phase::Running => format!("{} Scanning — {}", ic::CAMERA, s.name),
        Phase::Prepare => format!("{} Prepare scan for printing", ic::CAMERA),
    };
    #[allow(deprecated)]
    let mut win = egui::Window::new(title).id(egui::Id::new("scan_window")).collapsible(true).resizable(false);
    win = match s.phase {
        // below the view cube; scrolls if the screen is short
        Phase::Prepare => win.anchor(egui::Align2::RIGHT_TOP, [-16.0, 260.0]),
        _ => win.default_pos(ctx.content_rect().center() - egui::vec2(230.0, 260.0)),
    };
    win.show(ctx, |ui| {
        ui.set_width(if s.phase == Phase::Prepare { 330.0 } else { 460.0 });
        match s.phase {
            Phase::Setup => keep = setup_ui(app, &mut s, ui),
            Phase::Running => keep = running_ui(app, &mut s, ui),
            Phase::Prepare => {
                let h = (ctx.content_rect().height() - 330.0).max(240.0);
                egui::ScrollArea::vertical().max_height(h).show(ui, |ui| keep = prepare_ui(app, &mut s, ui));
            }
        }
    });
    if keep {
        app.scan = Some(s);
    } else {
        if let Some(j) = &s.job {
            j.cancel();
        }
        remove_preview(app, &mut s);
    }
}

// ---------------------------------------------------------------- setup

fn tool_row(ui: &mut egui::Ui, name: &str, ok: bool, detail: &str) {
    ui.horizontal(|ui| {
        if ok {
            ui.label(RichText::new(ic::CHECK_CIRCLE).color(theme::OK));
        } else {
            ui.label(RichText::new(ic::X_CIRCLE).color(theme::ERR));
        }
        ui.label(RichText::new(name).strong());
        ui.label(RichText::new(detail).small().color(theme::TEXT_DIM));
    });
}

fn setup_ui(app: &mut App, s: &mut ScanUi, ui: &mut egui::Ui) -> bool {
    let mut keep = true;
    ui.label("Turn photos or a video of a real object into a printable 3D model.");
    ui.add_space(6.0);

    // inputs
    egui::Frame::new().fill(theme::PANEL).corner_radius(6).inner_margin(8).show(ui, |ui| {
        ui.set_width(ui.available_width());
        ui.horizontal(|ui| {
            if ui.button(format!("{} Photos…", ic::IMAGE)).clicked() {
                let exts: Vec<&str> = lusion_photo::frames::IMAGE_EXTS.iter().flat_map(|e| [*e]).collect();
                if let Some(ps) = rfd::FileDialog::new().add_filter("Photos", &exts).pick_files() {
                    s.add_paths(ps);
                }
            }
            if ui.button(format!("{} Folder…", ic::FOLDER_OPEN)).clicked() {
                if let Some(d) = rfd::FileDialog::new().pick_folder() {
                    s.add_paths([d]);
                }
            }
            if ui.button(format!("{} Video…", ic::FILM_STRIP)).clicked() {
                if let Some(ps) = rfd::FileDialog::new().add_filter("Video", lusion_photo::frames::VIDEO_EXTS).pick_files() {
                    s.add_paths(ps);
                }
            }
            if !s.paths.is_empty() && ui.button(RichText::new(ic::TRASH).color(theme::TEXT_DIM)).on_hover_text("Clear").clicked() {
                s.paths.clear();
                s.inputs = Inputs::default();
            }
        });
        ui.add_space(4.0);
        if s.inputs.is_empty() {
            ui.label(RichText::new("…or drop photos, a folder or a video here.").color(theme::TEXT_DIM));
        } else {
            ui.label(RichText::new(format!("{} {}", ic::CHECK, s.inputs.summary())).strong().color(theme::OK));
            let n = s.inputs.images.len();
            if s.inputs.videos.is_empty() && n < 20 {
                ui.colored_label(theme::WARN, format!("{} Few photos — 40–150 with lots of overlap work best", ic::WARNING));
            }
            if !s.inputs.skipped.is_empty() {
                ui.label(RichText::new(format!("{} unsupported files ignored (HEIC? export as JPEG)", s.inputs.skipped.len())).small().color(theme::TEXT_DIM));
            }
        }
    });
    ui.add_space(8.0);

    ui.horizontal(|ui| {
        ui.label("Quality");
        for q in Quality::ALL {
            ui.selectable_value(&mut s.settings.quality, q, q.label());
        }
    });
    ui.label(RichText::new(s.settings.quality.hint()).small().color(theme::TEXT_DIM));
    if !s.inputs.videos.is_empty() {
        ui.horizontal(|ui| {
            ui.label("Frames per video");
            ui.add(egui::Slider::new(&mut s.settings.frames_per_video, 30..=250));
        });
    }
    egui::CollapsingHeader::new("Advanced").show(ui, |ui| {
        let max = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(4);
        let mut n = s.settings.thread_count();
        ui.horizontal(|ui| {
            ui.label("CPU threads");
            if ui.add(egui::Slider::new(&mut n, 1..=max)).on_hover_text("Fewer = slower scan, snappier desktop. Default leaves two cores free.").changed() {
                s.settings.threads = n;
            }
        });
        let mut refine = s.settings.refine.unwrap_or(s.settings.quality == Quality::High);
        if ui.checkbox(&mut refine, "Refine mesh against the photos").on_hover_text("Sharper detail; adds a slow CPU step").changed() {
            s.settings.refine = Some(refine);
        }
    });
    ui.add_space(6.0);
    egui::CollapsingHeader::new(format!("{} How to take good scan photos", ic::LIGHTBULB)).show(ui, |ui| {
        for tip in [
            "Walk all the way around the object, 2–3 rings at different heights.",
            "Every spot should be in 3+ photos; move ~10° between shots.",
            "Fill the frame with the object, keep it sharp, don't zoom.",
            "Even, diffuse light. Shiny, transparent or plain-white things scan badly (a dusting of chalk/talc spray helps).",
            "Put it on a textured surface (newspaper, patterned cloth) — that helps solve the cameras, and the table gets cut away automatically.",
            "Video: move slowly; 1–2 min of 4K/1080p is plenty.",
        ] {
            ui.label(format!("• {tip}"));
        }
    });

    // tools
    ui.add_space(6.0);
    let t = s.tools.clone();
    let need_video = !s.inputs.videos.is_empty();
    egui::CollapsingHeader::new(if t.ready(need_video) { format!("{} Tools found", ic::CHECK_CIRCLE) } else { format!("{} Tools missing", ic::WARNING) })
        .default_open(!t.ready(need_video))
        .show(ui, |ui| {
            tool_row(ui, "COLMAP", t.colmap.is_some(), if t.colmap.is_some() { if t.colmap_cuda { "camera solving (CUDA)" } else { "camera solving (CPU)" } } else { "sudo apt install colmap" });
            tool_row(ui, "OpenMVS", t.openmvs.is_some(), if t.openmvs.is_some() { "dense mesh (CPU)" } else { "run tools/photogrammetry/build-openmvs.sh" });
            tool_row(ui, "ffmpeg", t.ffmpeg.is_some(), if t.ffmpeg.is_some() { "video frames" } else { "sudo apt install ffmpeg (only for video)" });
            if ui.small_button("Check again").clicked() {
                s.tools = Toolchain::detect();
            }
        });
    ui.add_space(10.0);
    ui.horizontal(|ui| {
        let can = !s.inputs.is_empty() && s.tools.ready(need_video);
        if ui
            .add_enabled(can, egui::Button::new(RichText::new(format!("{} Start scan", ic::PLAY)).strong()).fill(theme::ACCENT_DIM).min_size(egui::vec2(130.0, 30.0)))
            .clicked()
        {
            let wd = pipeline::default_workdir(&s.name);
            s.workdir = Some(wd.clone());
            start_job(s, wd);
        }
        if ui.button(format!("{} Clean up a mesh file…", ic::BROOM)).on_hover_text("Prepare an existing scan (PLY/OBJ/STL…) for printing").clicked() {
            let exts = lusion_meshio::supported_import_extensions();
            if let Some(p) = rfd::FileDialog::new().add_filter("Meshes", exts).pick_file() {
                load_mesh_for_prepare(app, s, &p);
            }
        }
        if ui.button("Close").clicked() {
            keep = false;
        }
    });
    keep
}

/// Load an existing mesh (e.g. a previous scan) straight into the prepare step. If it sits
/// in one of our work folders, the camera poses next to it are used for orientation.
fn load_mesh_for_prepare(app: &mut App, s: &mut ScanUi, p: &std::path::Path) {
    match lusion_meshio::load_mesh_file(p) {
        Ok(ms) => {
            let mut all = lusion_meshio::TriMesh::default();
            for m in ms {
                let base = all.positions.len() as u32;
                all.positions.extend_from_slice(&m.positions);
                all.indices.extend(m.indices.iter().map(|i| i + base));
            }
            s.name = p.file_stem().map(|x| x.to_string_lossy().to_string()).unwrap_or_else(|| "Scan".into());
            let cams = p.parent().map(|d| d.join("../dense/sparse/images.txt")).and_then(|f| pipeline::read_colmap_cameras(&f).ok()).unwrap_or_default();
            enter_prepare(app, s, Mesh::from_trimesh(&all), cams);
        }
        Err(e) => app.error(format!("Could not read {}: {e:#}", p.display())),
    }
}

fn start_job(s: &mut ScanUi, wd: PathBuf) {
    s.log.clear();
    s.error = None;
    s.stage = 0;
    s.progress = 0.0;
    s.started = Instant::now();
    s.stage_started = Instant::now();
    s.job = Some(pipeline::start(s.inputs.clone(), wd, s.settings.clone(), s.tools.clone()));
    s.phase = Phase::Running;
}

// ---------------------------------------------------------------- running

fn poll_job(app: &mut App, s: &mut ScanUi) {
    let Some(job) = &s.job else { return };
    let mut finished = None;
    for ev in job.poll() {
        match ev {
            Event::Stage(i) => {
                s.stage = i;
                s.stage_started = Instant::now();
                s.progress = 0.0;
            }
            Event::Progress(p) => s.progress = p,
            Event::Log(l) => {
                s.log.push_back(l);
                while s.log.len() > 400 {
                    s.log.pop_front();
                }
            }
            Event::Finished(r) => finished = Some(r),
        }
    }
    if let Some(r) = finished {
        s.job = None;
        match r {
            Ok(res) => {
                app.info(format!("Scan finished in {} — {} of {} images used", fmt_dur(s.started.elapsed().as_secs_f32()), res.images_registered, res.images_total));
                let raw = Mesh::from_trimesh(&res.mesh);
                enter_prepare(app, s, raw, res.cameras);
            }
            Err(e) if e == "cancelled" => {
                // nothing to resume from a deliberate cancel: free the disk space
                if let Some(wd) = s.workdir.take() {
                    let _ = std::fs::remove_dir_all(wd);
                }
                s.phase = Phase::Setup;
            }
            Err(e) => s.error = Some(e),
        }
    }
}

fn running_ui(_app: &mut App, s: &mut ScanUi, ui: &mut egui::Ui) -> bool {
    let mut keep = true;
    let refine = s.settings.refine.unwrap_or(s.settings.quality == Quality::High);
    let n_stages = if refine { STAGES.len() } else { STAGES.len() - 1 };
    for (i, name) in STAGES.iter().enumerate().take(n_stages) {
        ui.horizontal(|ui| {
            let (icon, col) = if i < s.stage || (i == s.stage && s.job.is_none() && s.error.is_none()) {
                (ic::CHECK_CIRCLE, theme::OK)
            } else if i == s.stage && s.error.is_some() {
                (ic::X_CIRCLE, theme::ERR)
            } else if i == s.stage {
                (ic::CIRCLE_NOTCH, theme::ACCENT)
            } else {
                (ic::CIRCLE, theme::TEXT_DIM)
            };
            if i == s.stage && s.job.is_some() {
                ui.add(egui::Spinner::new().size(14.0).color(theme::ACCENT));
            } else {
                ui.label(RichText::new(icon).color(col));
            }
            let txt = RichText::new(*name);
            ui.label(if i == s.stage { txt.strong() } else if i > s.stage { txt.color(theme::TEXT_DIM) } else { txt });
            if i == s.stage && s.job.is_some() {
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    ui.label(RichText::new(fmt_dur(s.stage_started.elapsed().as_secs_f32())).small().color(theme::TEXT_DIM));
                });
            }
        });
    }
    ui.add_space(6.0);
    if s.job.is_some() {
        let overall = pipeline::overall_progress(s.stage, s.progress, refine);
        ui.add(egui::ProgressBar::new(overall).desired_width(ui.available_width()).text(format!("{:.0}% · {} elapsed", overall * 100.0, fmt_dur(s.started.elapsed().as_secs_f32()))));
        ui.label(RichText::new("Runs in the background — you can keep working (collapse this window with the arrow in its title).").small().color(theme::TEXT_DIM));
    }
    if let Some(e) = &s.error {
        ui.add_space(4.0);
        egui::Frame::new().fill(Color32::from_rgb(0x3a, 0x20, 0x22)).corner_radius(6).inner_margin(8).show(ui, |ui| {
            ui.set_width(ui.available_width());
            ui.label(RichText::new(format!("{} Scan failed", ic::WARNING)).strong().color(theme::ERR));
            ui.label(RichText::new(e).small().monospace());
        });
    }
    ui.add_space(4.0);
    egui::CollapsingHeader::new("Log").default_open(s.error.is_some()).show(ui, |ui| {
        egui::ScrollArea::vertical().max_height(180.0).stick_to_bottom(true).show(ui, |ui| {
            for l in &s.log {
                ui.label(RichText::new(l).small().monospace().color(theme::TEXT_DIM));
            }
        });
        if let Some(wd) = &s.workdir {
            ui.label(RichText::new(format!("Work folder: {}", wd.display())).small().color(theme::TEXT_DIM));
        }
    });
    ui.add_space(8.0);
    ui.horizontal(|ui| {
        if s.job.is_some() {
            if ui.add(egui::Button::new(format!("{} Cancel", ic::STOP)).min_size(egui::vec2(100.0, 28.0))).clicked() {
                if let Some(j) = &s.job {
                    j.cancel();
                }
            }
        } else {
            if s.error.is_some() {
                if let Some(wd) = s.workdir.clone() {
                    if ui.button(format!("{} Retry", ic::ARROW_CLOCKWISE)).on_hover_text("Continues from the last finished step").clicked() {
                        start_job(s, wd);
                    }
                }
            }
            if ui.button("Back").clicked() {
                s.phase = Phase::Setup;
                s.error = None;
            }
            if ui.button("Close").clicked() {
                keep = false;
            }
            if !keep || s.phase == Phase::Setup {
                // leaving a failed run: its partial results (often GBs) are no longer needed
                if let Some(wd) = s.workdir.take() {
                    let _ = std::fs::remove_dir_all(wd);
                }
            }
        }
    });
    keep
}

// ---------------------------------------------------------------- prepare

fn enter_prepare(app: &mut App, s: &mut ScanUi, raw: Mesh, cams: Vec<pipeline::CameraPose>) {
    s.raw = Some(Arc::new(raw));
    s.cams = Arc::new(cams);
    s.align = None;
    s.auto_align = None;
    s.result = None;
    s.applied = None;
    s.changed_at = Some(Instant::now());
    s.phase = Phase::Prepare;
    s.fitted = false;
    app.end_interaction();
}

/// Kick off background preparation when settings changed (debounced).
fn poll_prep(app: &mut App, s: &mut ScanUi, ctx: &egui::Context) {
    if s.phase != Phase::Prepare {
        return;
    }
    let Some(raw) = s.raw.clone() else { return };
    if let Some(rx) = &s.prep_rx {
        let msgs: Vec<PrepMsg> = rx.try_iter().collect();
        for m in msgs {
            match m {
                PrepMsg::Aligned(a) => {
                    if s.auto_align.is_none() {
                        s.auto_align = Some(a.clone());
                    }
                    if s.align.is_none() {
                        s.align = Some(a);
                    }
                }
                PrepMsg::Result { gen, final_, res } => {
                    if gen != s.gen.load(Ordering::Relaxed) {
                        continue;
                    }
                    if final_ {
                        s.busy = false;
                    }
                    match res {
                        Ok(p) => {
                            s.prep_error = None;
                            show_preview(app, s, &p);
                            s.result = Some(p);
                            s.result_final = final_;
                        }
                        Err(e) => {
                            s.prep_error = Some(e);
                            s.busy = false;
                        }
                    }
                }
            }
        }
    }
    let key = (s.prep.clone(), s.show_all, 0u64);
    let stale = s.applied.as_ref().map(|a| a.0 != key.0 || a.1 != key.1).unwrap_or(true);
    if !stale {
        return;
    }
    // debounce slider drags
    let Some(t) = s.changed_at else {
        s.changed_at = Some(Instant::now());
        return;
    };
    if t.elapsed().as_millis() < 250 {
        ctx.request_repaint_after(std::time::Duration::from_millis(60));
        return;
    }
    s.applied = Some(key);
    s.changed_at = None;
    s.busy = true;
    let gen = s.gen.fetch_add(1, Ordering::Relaxed) + 1;
    let latest = s.gen.clone();
    let (tx, rx) = channel();
    s.prep_rx = Some(rx);
    let cams = s.cams.clone();
    let align = s.align.clone();
    let mut settings = s.prep.clone();
    if s.show_all {
        settings = PrepSettings { remove_ground: false, crop: 9.0, keep_largest: false, solidify: false, base_cut_mm: 0.0, ..settings };
    }
    std::thread::spawn(move || {
        let align = match align {
            Some(a) => a,
            None => {
                let a = Alignment::auto(&raw, &cams);
                let _ = tx.send(PrepMsg::Aligned(a.clone()));
                a
            }
        };
        let quiet = |_: &str| {};
        // instant feedback without the (slower) solid rebuild
        let quick = PrepSettings { solidify: false, max_triangles: settings.max_triangles.min(300_000), ..settings.clone() };
        let r = lusion_photo::prepare::prepare(&raw, &align, &quick, &quiet).map_err(|e| format!("{e:#}"));
        let failed = r.is_err();
        let _ = tx.send(PrepMsg::Result { gen, final_: !settings.solidify || failed, res: r });
        if !settings.solidify || failed || latest.load(Ordering::Relaxed) != gen {
            return;
        }
        let r = lusion_photo::prepare::prepare(&raw, &align, &settings, &quiet).map_err(|e| format!("{e:#}"));
        let _ = tx.send(PrepMsg::Result { gen, final_: true, res: r });
    });
}

fn mesh_feature(p: &Prepared, name: &str) -> FeatureKind {
    FeatureKind::ImportMesh(ImportMesh {
        source: format!("photogrammetry: {name}"),
        positions: Arc::new(p.mesh.flat_positions()),
        indices: Arc::new(p.mesh.flat_indices()),
    })
}

fn show_preview(app: &mut App, s: &mut ScanUi, p: &Prepared) {
    let kind = mesh_feature(p, &s.name);
    match s.preview.and_then(|f| app.doc.feature_mut(f)) {
        Some(f) => f.kind = kind,
        None => {
            s.prev_next_id = app.doc.next_id;
            let name = s.name.clone();
            s.preview = Some(app.doc.insert_feature(name, app.active_comp, kind));
        }
    }
    if !s.fitted {
        s.fitted = true;
        app.fit_pending = true;
    }
}

/// Take the preview feature out of the document (undoing its id allocation if possible).
fn remove_preview(app: &mut App, s: &mut ScanUi) {
    if let Some(fid) = s.preview.take() {
        app.doc.delete_feature(fid);
        if app.doc.next_id == fid + 1 && s.prev_next_id == fid {
            app.doc.next_id = fid;
        }
    }
}

fn prepare_ui(app: &mut App, s: &mut ScanUi, ui: &mut egui::Ui) -> bool {
    let mut keep = true;
    let before = s.prep.clone();
    let before_all = s.show_all;
    let section = |ui: &mut egui::Ui, t: &str| {
        ui.add_space(6.0);
        ui.label(RichText::new(t).small().strong().color(theme::TEXT_DIM));
    };

    // orientation
    section(ui, "ORIENTATION");
    match s.align.clone() {
        None => {
            ui.horizontal(|ui| {
                ui.add(egui::Spinner::new());
                ui.label("Finding the table…");
            });
        }
        Some(a) => {
            let (icon, col) = match a.source {
                AlignSource::Ground => (ic::CHECK_CIRCLE, theme::OK),
                _ => (ic::WARNING, theme::WARN),
            };
            ui.label(RichText::new(format!("{icon} {}", capitalise(&a.describe()))).color(col));
            let mut rot = None;
            ui.horizontal(|ui| {
                use std::f32::consts::FRAC_PI_2;
                if ui.button(format!("{} Flip", ic::FLIP_VERTICAL)).on_hover_text("Upside down").clicked() {
                    rot = Some(glam::Quat::from_rotation_x(std::f32::consts::PI));
                }
                if ui.button(format!("{} Tip X", ic::ARROW_CLOCKWISE)).on_hover_text("Rotate 90° about X").clicked() {
                    rot = Some(glam::Quat::from_rotation_x(FRAC_PI_2));
                }
                if ui.button(format!("{} Tip Y", ic::ARROW_CLOCKWISE)).on_hover_text("Rotate 90° about Y").clicked() {
                    rot = Some(glam::Quat::from_rotation_y(FRAC_PI_2));
                }
                if s.auto_align.is_some() && ui.button("Auto").on_hover_text("Back to the detected orientation").clicked() {
                    s.align = s.auto_align.clone();
                    s.applied = None;
                }
            });
            if let Some(q) = rot {
                s.align = Some(a.rotated(q));
                s.applied = None;
                s.fitted = false;
            }
        }
    }
    let has_floor = s.align.as_ref().map(|a| a.floor.is_some()).unwrap_or(false);
    let has_ring = s.align.as_ref().map(|a| a.ring_radius.is_some()).unwrap_or(false);

    section(ui, "CLEAN UP");
    ui.add_enabled(has_floor, egui::Checkbox::new(&mut s.prep.remove_ground, "Remove the table"));
    ui.checkbox(&mut s.prep.keep_largest, "Keep only the main object");
    if has_ring {
        ui.horizontal(|ui| {
            ui.label("Crop");
            ui.add(egui::Slider::new(&mut s.prep.crop, 0.2..=1.5).custom_formatter(|v, _| if v >= 1.5 { "off".into() } else { format!("{:.0}%", v * 100.0) }))
                .on_hover_text("Keep what is within this share of the distance to the cameras");
        });
    }
    ui.checkbox(&mut s.show_all, "Show the raw scan (before clean-up)");

    section(ui, "SIZE");
    ui.horizontal(|ui| {
        egui::ComboBox::from_id_salt("scan_axis").selected_text(s.prep.size_axis.label()).width(120.0).show_ui(ui, |ui| {
            for a in SizeAxis::ALL {
                ui.selectable_value(&mut s.prep.size_axis, a, a.label());
            }
        });
        ui.add(egui::DragValue::new(&mut s.prep.size_mm).range(1.0..=1000.0).speed(0.5).suffix(" mm"));
    });
    ui.label(RichText::new("Photos have no scale: measure the real object and type it here.").small().color(theme::TEXT_DIM));
    ui.horizontal(|ui| {
        ui.label("Flatten base");
        ui.add(egui::DragValue::new(&mut s.prep.base_cut_mm).range(0.0..=50.0).speed(0.1).suffix(" mm")).on_hover_text("Cut this much off the bottom for a bigger flat base");
    });

    section(ui, "SOLID FOR PRINTING");
    ui.checkbox(&mut s.prep.solidify, "Make watertight solid").on_hover_text("Rebuild as a closed, manifold mesh with a flat base");
    ui.add_enabled_ui(s.prep.solidify, |ui| {
        ui.horizontal(|ui| {
            ui.label("Detail");
            ui.add(egui::Slider::new(&mut s.prep.detail_mm, 0.1..=1.5).step_by(0.05).suffix(" mm")).on_hover_text("Smaller = finer, slower. 0.2–0.3 mm suits a 0.4 mm nozzle.");
        });
        ui.horizontal(|ui| {
            ui.label("Close holes");
            ui.add(egui::Slider::new(&mut s.prep.close_mm, 0.0..=10.0).step_by(0.25).suffix(" mm")).on_hover_text("Holes up to about twice this are sealed (also fills narrow gaps)");
        });
        ui.horizontal(|ui| {
            ui.label("Smooth");
            ui.add(egui::Slider::new(&mut s.prep.smooth, 0..=8));
        });
    });
    ui.horizontal(|ui| {
        ui.label("Max triangles");
        let mut k = (s.prep.max_triangles / 1000) as u32;
        if ui.add(egui::Slider::new(&mut k, 50..=2000).logarithmic(true).suffix("k")).changed() {
            s.prep.max_triangles = k as usize * 1000;
        }
    });

    if s.prep != before || s.show_all != before_all {
        s.changed_at = Some(Instant::now());
        // the preview changes size: frame it again once the new result arrives
        if s.prep.size_mm != before.size_mm || s.prep.size_axis != before.size_axis || s.show_all != before_all {
            s.fitted = false;
        }
    }

    // result
    ui.add_space(8.0);
    egui::Frame::new().fill(theme::PANEL).corner_radius(6).inner_margin(8).show(ui, |ui| {
        ui.set_width(ui.available_width());
        if let Some(e) = &s.prep_error {
            ui.colored_label(theme::ERR, format!("{} {e}", ic::WARNING));
        }
        match &s.result {
            None => {
                ui.horizontal(|ui| {
                    ui.add(egui::Spinner::new());
                    ui.label("Preparing…");
                });
            }
            Some(p) => {
                let st = &p.stats;
                ui.horizontal(|ui| {
                    ui.label(RichText::new(format!("{:.1} × {:.1} × {:.1} mm", st.size.x, st.size.y, st.size.z)).strong());
                    if s.busy {
                        ui.add(egui::Spinner::new().size(12.0));
                        ui.label(RichText::new(if s.prep.solidify && !s.result_final { "building solid…" } else { "updating…" }).small().color(theme::TEXT_DIM));
                    }
                });
                let tris = if st.triangles >= 10_000 { format!("{}k", st.triangles / 1000) } else { st.triangles.to_string() };
                ui.label(RichText::new(format!("{tris} triangles · {} part{}", st.components, if st.components == 1 { "" } else { "s" })).small().color(theme::TEXT_DIM));
                if s.show_all {
                    ui.label(RichText::new("Raw scan shown — untick to see the result").color(theme::WARN));
                } else if st.watertight() {
                    ui.label(RichText::new(format!("{} Watertight · {:.1} cm³", ic::CHECK_CIRCLE, st.volume / 1000.0)).color(theme::OK));
                } else if s.result_final {
                    ui.label(RichText::new(format!("{} Open mesh ({} hole edges) — slicers may complain", ic::WARNING, st.boundary_edges)).color(theme::WARN));
                }
                for w in &p.warnings {
                    ui.colored_label(theme::WARN, format!("{} {w}", ic::WARNING));
                }
            }
        }
    });

    ui.add_space(10.0);
    let ready = s.result.is_some() && s.result_final && !s.busy && !s.show_all && s.preview.is_some();
    let mut insert = false;
    let mut print = false;
    ui.horizontal(|ui| {
        if ui.add_enabled(ready, egui::Button::new(RichText::new(format!("{} Insert", ic::CHECK)).strong()).fill(theme::ACCENT_DIM).min_size(egui::vec2(90.0, 28.0))).clicked() {
            insert = true;
        }
        if ui.add_enabled(ready, egui::Button::new(format!("{} Insert & print…", ic::PRINTER)).min_size(egui::vec2(120.0, 28.0))).clicked() {
            insert = true;
            print = true;
        }
        if ui.button("Discard").clicked() {
            keep = false;
        }
    });
    if insert {
        if let Some(fid) = s.preview.take() {
            // one undo step: "before" is the document without the preview feature
            let mut before = app.doc.clone();
            before.delete_feature(fid);
            if before.next_id == fid + 1 && s.prev_next_id == fid {
                before.next_id = fid;
            }
            app.commit(before);
            app.info(format!("Inserted scan \"{}\"", s.name));
            if print {
                crate::print::export_bodies_for_print(app, vec![BodyId::new(fid, 0)]);
            }
        }
        keep = false;
    }
    keep
}

fn capitalise(s: &str) -> String {
    let mut c = s.chars();
    match c.next() {
        Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
        None => String::new(),
    }
}

/// The tab holding this scan is being closed: stop the tools and free the work dir.
pub fn discard(app: &mut App) {
    let Some(mut s) = app.scan.take() else { return };
    remove_preview(app, &mut s);
    if s.phase == Phase::Prepare {
        return; // finished scan: keep its results for "Clean up a mesh file…"
    }
    let wd = s.workdir.take();
    match s.job.take() {
        Some(j) => {
            j.cancel();
            // delete once the tools have actually stopped writing
            std::thread::spawn(move || {
                for ev in j.rx.iter() {
                    if matches!(ev, Event::Finished(_)) {
                        break;
                    }
                }
                if let Some(wd) = wd {
                    let _ = std::fs::remove_dir_all(wd);
                }
            });
        }
        None => {
            if let Some(wd) = wd {
                let _ = std::fs::remove_dir_all(wd);
            }
        }
    }
}
