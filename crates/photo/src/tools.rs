//! Locating the external photogrammetry programs (COLMAP, OpenMVS, ffmpeg).

use std::path::{Path, PathBuf};
use std::process::Command;

/// The OpenMVS programs the pipeline uses.
pub const OPENMVS_TOOLS: [&str; 4] = ["InterfaceCOLMAP", "DensifyPointCloud", "ReconstructMesh", "RefineMesh"];

#[derive(Clone, Debug, Default)]
pub struct Toolchain {
    pub colmap: Option<PathBuf>,
    /// Directory holding the OpenMVS executables.
    pub openmvs: Option<PathBuf>,
    pub ffmpeg: Option<PathBuf>,
    pub ffprobe: Option<PathBuf>,
    /// `colmap -h` banner mentions CUDA support.
    pub colmap_cuda: bool,
    /// First line of `colmap -h`, e.g. "COLMAP 3.9.1 -- Structure-from-Motion and Multi-View Stereo".
    pub colmap_version: String,
}

pub fn in_path(exe: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    std::env::split_paths(&path).find_map(|dir| {
        [dir.join(exe), dir.join(format!("{exe}.exe"))].into_iter().find(|c| c.is_file())
    })
}

fn exe_in(dir: &Path, exe: &str) -> bool {
    dir.join(exe).is_file() || dir.join(format!("{exe}.exe")).is_file()
}

fn home() -> PathBuf {
    std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE")).map(PathBuf::from).unwrap_or_default()
}

impl Toolchain {
    /// Look in the environment overrides, PATH and the usual install prefixes.
    pub fn detect() -> Self {
        let colmap = std::env::var_os("LUSION_COLMAP").map(PathBuf::from).filter(|p| p.is_file()).or_else(|| in_path("colmap"));
        let mut dirs: Vec<PathBuf> = Vec::new();
        if let Some(d) = std::env::var_os("LUSION_OPENMVS_DIR") {
            dirs.push(PathBuf::from(d));
        }
        dirs.push(home().join(".local/opt/openmvs/bin/OpenMVS"));
        dirs.push(home().join(".local/bin/OpenMVS"));
        dirs.push(PathBuf::from("/usr/local/bin/OpenMVS"));
        dirs.push(PathBuf::from("/usr/bin/OpenMVS"));
        dirs.push(PathBuf::from("/opt/openmvs/bin/OpenMVS"));
        if let Some(p) = in_path("DensifyPointCloud") {
            if let Some(d) = p.parent() {
                dirs.push(d.to_path_buf());
            }
        }
        let openmvs = dirs.into_iter().find(|d| OPENMVS_TOOLS.iter().all(|t| exe_in(d, t)));
        let mut tc = Toolchain { colmap, openmvs, ffmpeg: in_path("ffmpeg"), ffprobe: in_path("ffprobe"), ..Default::default() };
        if let Some(c) = &tc.colmap {
            if let Ok(out) = Command::new(c).arg("-h").env("QT_QPA_PLATFORM", "offscreen").output() {
                let text = String::from_utf8_lossy(&out.stdout).to_string() + &String::from_utf8_lossy(&out.stderr);
                tc.colmap_version = text.lines().find(|l| l.contains("COLMAP")).unwrap_or("").trim().to_string();
                tc.colmap_cuda = text.contains("with CUDA");
            }
        }
        tc
    }

    pub fn openmvs_tool(&self, name: &str) -> Option<PathBuf> {
        let d = self.openmvs.as_ref()?;
        [d.join(name), d.join(format!("{name}.exe"))].into_iter().find(|p| p.is_file())
    }

    /// Human-readable list of what is missing for a run (`video`: inputs include videos).
    pub fn missing(&self, video: bool) -> Vec<String> {
        let mut v = Vec::new();
        if self.colmap.is_none() {
            v.push("COLMAP (camera solving) — `sudo apt install colmap`".into());
        }
        if self.openmvs.is_none() {
            v.push("OpenMVS (dense mesh) — run tools/photogrammetry/build-openmvs.sh".into());
        }
        if video && (self.ffmpeg.is_none() || self.ffprobe.is_none()) {
            v.push("ffmpeg (video frames) — `sudo apt install ffmpeg`".into());
        }
        v
    }

    pub fn ready(&self, video: bool) -> bool {
        self.missing(video).is_empty()
    }
}
