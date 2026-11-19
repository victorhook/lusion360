//! Turning the user's inputs (photos, folders, videos) into one image folder for COLMAP.
//!
//! Photos are linked (not re-encoded, so EXIF focal lengths survive) into one sub-folder per
//! image resolution; COLMAP then shares one camera model per folder, which is what a phone
//! or camera at a fixed zoom really is. Videos are sampled with ffmpeg and only the sharpest
//! frame of every small group is kept (motion blur is the #1 cause of failed video scans).

use anyhow::{bail, Context, Result};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};

pub const IMAGE_EXTS: &[&str] = &["jpg", "jpeg", "png", "tif", "tiff", "bmp"];
pub const VIDEO_EXTS: &[&str] = &["mp4", "mov", "m4v", "mkv", "avi", "webm", "mts", "3gp"];

fn ext(p: &Path) -> String {
    p.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase()
}

pub fn is_image(p: &Path) -> bool {
    IMAGE_EXTS.contains(&ext(p).as_str())
}

pub fn is_video(p: &Path) -> bool {
    VIDEO_EXTS.contains(&ext(p).as_str())
}

#[derive(Clone, Debug, Default)]
pub struct Inputs {
    pub images: Vec<PathBuf>,
    pub videos: Vec<PathBuf>,
    /// Files that were skipped (unsupported type), for the UI.
    pub skipped: Vec<PathBuf>,
}

impl Inputs {
    /// Expand folders (one level deep plus nested folders) into image and video files.
    pub fn collect(paths: &[PathBuf]) -> Inputs {
        let mut inp = Inputs::default();
        let mut stack: Vec<PathBuf> = paths.to_vec();
        while let Some(p) = stack.pop() {
            if p.is_dir() {
                if let Ok(rd) = std::fs::read_dir(&p) {
                    let mut kids: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
                    kids.sort();
                    // skip our own work folders if someone scans a folder twice
                    kids.retain(|k| k.file_name().map(|n| n != "lusion-scan").unwrap_or(true));
                    stack.extend(kids.into_iter().rev());
                }
            } else if is_image(&p) {
                inp.images.push(p);
            } else if is_video(&p) {
                inp.videos.push(p);
            } else if p.is_file() {
                inp.skipped.push(p);
            }
        }
        inp.images.sort();
        inp.images.dedup();
        inp.videos.sort();
        inp.videos.dedup();
        inp
    }

    pub fn is_empty(&self) -> bool {
        self.images.is_empty() && self.videos.is_empty()
    }

    pub fn summary(&self) -> String {
        match (self.images.len(), self.videos.len()) {
            (0, 0) => "nothing selected".into(),
            (i, 0) => format!("{i} photo{}", if i == 1 { "" } else { "s" }),
            (0, v) => format!("{v} video{}", if v == 1 { "" } else { "s" }),
            (i, v) => format!("{i} photo{} + {v} video{}", if i == 1 { "" } else { "s" }, if v == 1 { "" } else { "s" }),
        }
    }
}

fn link_or_copy(src: &Path, dst: &Path) -> Result<()> {
    let src = src.canonicalize().unwrap_or_else(|_| src.to_path_buf());
    #[cfg(unix)]
    {
        if std::os::unix::fs::symlink(&src, dst).is_ok() {
            return Ok(());
        }
    }
    if std::fs::hard_link(&src, dst).is_ok() {
        return Ok(());
    }
    std::fs::copy(&src, dst).with_context(|| format!("copying {}", src.display()))?;
    Ok(())
}

/// Place photos into `images/photos_<W>x<H>/` (one camera per resolution).
/// Returns the number of images placed.
pub fn place_photos(images: &[PathBuf], out: &Path, log: &mut dyn FnMut(String)) -> Result<usize> {
    let mut n = 0;
    for (i, src) in images.iter().enumerate() {
        let (w, h) = match image::image_dimensions(src) {
            Ok(d) => d,
            Err(e) => {
                log(format!("skipping {} ({e})", src.display()));
                continue;
            }
        };
        let dir = out.join(format!("photos_{w}x{h}"));
        std::fs::create_dir_all(&dir)?;
        let name = src.file_name().and_then(|s| s.to_str()).unwrap_or("img.jpg");
        // prefix with the index so equally named files from different folders don't clash
        let dst = dir.join(format!("{i:05}_{name}"));
        if !dst.exists() {
            link_or_copy(src, &dst)?;
        }
        n += 1;
    }
    Ok(n)
}

pub fn video_duration(ffprobe: &Path, video: &Path) -> Option<f64> {
    let out = Command::new(ffprobe)
        .args(["-v", "error", "-show_entries", "format=duration", "-of", "default=noprint_wrappers=1:nokey=1"])
        .arg(video)
        .output()
        .ok()?;
    String::from_utf8_lossy(&out.stdout).trim().parse::<f64>().ok().filter(|d| d.is_finite() && *d > 0.0)
}

/// Variance of the Laplacian of a downscaled grey image: higher = sharper.
pub fn sharpness(path: &Path) -> f64 {
    let Ok(img) = image::open(path) else { return 0.0 };
    let g = img.thumbnail(640, 640).to_luma8();
    let (w, h) = g.dimensions();
    if w < 3 || h < 3 {
        return 0.0;
    }
    let px = |x: u32, y: u32| g.get_pixel(x, y)[0] as f64;
    let (mut s, mut s2, mut n) = (0.0, 0.0, 0.0);
    for y in 1..h - 1 {
        for x in 1..w - 1 {
            let l = 4.0 * px(x, y) - px(x - 1, y) - px(x + 1, y) - px(x, y - 1) - px(x, y + 1);
            s += l;
            s2 += l * l;
            n += 1.0;
        }
    }
    let m = s / n;
    s2 / n - m * m
}

/// Extract about `target` sharp frames from a video into `dir`.
/// `max_dim` bounds the longest side of the frames (0 = keep).
pub fn extract_video_frames(
    ffmpeg: &Path,
    ffprobe: &Path,
    video: &Path,
    dir: &Path,
    target: usize,
    max_dim: u32,
    cancel: &AtomicBool,
    log: &mut dyn FnMut(String),
) -> Result<usize> {
    std::fs::create_dir_all(dir)?;
    let dur = video_duration(ffprobe, video).unwrap_or(30.0);
    // sample 3 candidates per kept frame, pick the sharpest of each triple
    const GROUP: usize = 3;
    let fps = ((target * GROUP) as f64 / dur).clamp(0.2, 30.0);
    log(format!("{}: {:.0} s, sampling at {:.2} fps", video.display(), dur, fps));
    let mut vf = format!("fps={fps:.4}");
    if max_dim > 0 {
        vf += &format!(",scale='if(gt(iw,ih),min(iw,{max_dim}),-2)':'if(gt(iw,ih),-2,min(ih,{max_dim}))'");
    }
    let tmp = dir.join("candidates");
    std::fs::create_dir_all(&tmp)?;
    let mut child = Command::new(ffmpeg)
        .args(["-hide_banner", "-loglevel", "error", "-nostdin", "-y", "-i"])
        .arg(video)
        .args(["-vf", &vf, "-q:v", "2"])
        .arg(tmp.join("c_%05d.jpg"))
        .spawn()
        .context("starting ffmpeg")?;
    loop {
        if cancel.load(Ordering::Relaxed) {
            let _ = child.kill();
            let _ = child.wait();
            bail!("cancelled");
        }
        if let Some(st) = child.try_wait()? {
            if !st.success() {
                bail!("ffmpeg failed on {}", video.display());
            }
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(100));
    }
    let mut cands: Vec<PathBuf> = std::fs::read_dir(&tmp)?.flatten().map(|e| e.path()).filter(|p| is_image(p)).collect();
    cands.sort();
    if cands.is_empty() {
        bail!("no frames could be read from {}", video.display());
    }
    use rayon::prelude::*;
    let scores: Vec<f64> = cands.par_iter().map(|p| sharpness(p)).collect();
    let mut kept = 0;
    for (g, chunk) in cands.chunks(GROUP).enumerate() {
        let base = g * GROUP;
        let best = (0..chunk.len()).max_by(|&a, &b| scores[base + a].total_cmp(&scores[base + b])).unwrap_or(0);
        let dst = dir.join(format!("frame_{g:05}.jpg"));
        std::fs::rename(&chunk[best], &dst)?;
        kept += 1;
    }
    let _ = std::fs::remove_dir_all(&tmp);
    Ok(kept)
}
