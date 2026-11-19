//! Headless photogrammetry: `lusion-scan run <photos|folders|videos>... [-o out.ply]`
//! and `lusion-scan prepare <mesh> -o out.3mf [--height 80]`.

use anyhow::{bail, Context, Result};
use lusion_photo::pipeline::{self, Event, STAGES};
use lusion_photo::*;
use std::path::PathBuf;

fn usage() -> ! {
    eprintln!(
        "usage:
  lusion-scan tools
  lusion-scan run <photo|folder|video>... [-o mesh.ply] [--quality draft|normal|high] [--workdir DIR] [--frames N] [--threads N]
  lusion-scan prepare <mesh.ply|stl|obj> -o out.(stl|3mf|obj) [--cameras images.txt] [--height MM | --longest MM]
          [--detail MM] [--close MM] [--base-cut MM] [--no-solid] [--no-ground] [--max-tris N] [--smooth N]"
    );
    std::process::exit(2)
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some(cmd) = args.first() else { usage() };
    let mut pos: Vec<String> = Vec::new();
    let mut opts: std::collections::HashMap<String, String> = Default::default();
    let mut flags: std::collections::HashSet<String> = Default::default();
    let mut it = args[1..].iter();
    while let Some(a) = it.next() {
        if let Some(k) = a.strip_prefix("--").or_else(|| a.strip_prefix('-')) {
            if k.starts_with("no-") {
                flags.insert(k.to_string());
            } else {
                opts.insert(k.to_string(), it.next().cloned().unwrap_or_else(|| usage()));
            }
        } else {
            pos.push(a.clone());
        }
    }
    let num = |k: &str| -> Option<f32> { opts.get(k).and_then(|v| v.parse().ok()) };
    match cmd.as_str() {
        "tools" => {
            let t = Toolchain::detect();
            println!("colmap:  {:?} ({}; CUDA: {})", t.colmap, t.colmap_version, t.colmap_cuda);
            println!("openmvs: {:?}", t.openmvs);
            println!("ffmpeg:  {:?}", t.ffmpeg);
            for m in t.missing(true) {
                println!("missing: {m}");
            }
        }
        "run" => {
            let inputs = Inputs::collect(&pos.iter().map(PathBuf::from).collect::<Vec<_>>());
            if inputs.is_empty() {
                bail!("no photos or videos found");
            }
            let quality = match opts.get("quality").map(|s| s.as_str()) {
                Some("draft") => Quality::Draft,
                Some("high") => Quality::High,
                _ => Quality::Normal,
            };
            let mut settings = Settings { quality, ..Default::default() };
            if let Some(n) = num("threads") {
                settings.threads = n as usize;
            }
            if let Some(n) = num("frames") {
                settings.frames_per_video = n as usize;
            }
            let wd = opts.get("workdir").map(PathBuf::from).unwrap_or_else(|| pipeline::default_workdir("scan"));
            eprintln!("{} → {}", inputs.summary(), wd.display());
            let t0 = std::time::Instant::now();
            let verbose = std::env::var_os("LUSION_SCAN_VERBOSE").is_some();
            let res = pipeline::run_blocking(inputs, wd, settings, Toolchain::detect(), |ev| match ev {
                Event::Stage(i) => eprintln!("[{:>6.1}s] {}", t0.elapsed().as_secs_f32(), STAGES[i]),
                Event::Log(l) if verbose => eprintln!("    {l}"),
                _ => {}
            })?;
            eprintln!(
                "[{:>6.1}s] done: {} triangles, {}/{} images used, mesh {}",
                t0.elapsed().as_secs_f32(),
                res.mesh.indices.len() / 3,
                res.images_registered,
                res.images_total,
                res.mesh_path.display()
            );
            if let Some(o) = opts.get("o") {
                let m = Mesh::from_trimesh(&res.mesh);
                save(&m, &PathBuf::from(o))?;
            }
        }
        "prepare" => {
            let input = pos.first().unwrap_or_else(|| usage());
            let out = PathBuf::from(opts.get("o").unwrap_or_else(|| usage()));
            let mut meshes = lusion_meshio::load_mesh_file(&PathBuf::from(input)).context("loading mesh")?;
            let raw = Mesh::from_trimesh(&meshes.pop().context("empty file")?);
            let cams = match opts.get("cameras") {
                Some(p) => pipeline::read_colmap_cameras(&PathBuf::from(p))?,
                None => vec![],
            };
            let t0 = std::time::Instant::now();
            let align = Alignment::auto(&raw, &cams);
            eprintln!("alignment: {} ({:.2}s)", align.describe(), t0.elapsed().as_secs_f32());
            let mut s = PrepSettings::default();
            if let Some(h) = num("height") {
                s.size_axis = SizeAxis::Height;
                s.size_mm = h;
            }
            if let Some(h) = num("longest") {
                s.size_axis = SizeAxis::Longest;
                s.size_mm = h;
            }
            if let Some(v) = num("detail") {
                s.detail_mm = v;
            }
            if let Some(v) = num("close") {
                s.close_mm = v;
            }
            if let Some(v) = num("base-cut") {
                s.base_cut_mm = v;
            }
            if let Some(v) = num("max-tris") {
                s.max_triangles = v as usize;
            }
            if let Some(v) = num("smooth") {
                s.smooth = v as u32;
            }
            s.solidify = !flags.contains("no-solid");
            s.remove_ground = !flags.contains("no-ground");
            let p = prepare::prepare(&raw, &align, &s, &|msg| eprintln!("  {msg}"))?;
            let st = &p.stats;
            eprintln!(
                "result: {} tris, {:.1} × {:.1} × {:.1} mm, watertight: {} (boundary {}, non-manifold {}), parts {}, volume {:.1} cm³ ({:.2}s)",
                st.triangles, st.size.x, st.size.y, st.size.z, st.watertight(), st.boundary_edges, st.nonmanifold_edges, st.components, st.volume / 1000.0,
                t0.elapsed().as_secs_f32()
            );
            for w in &p.warnings {
                eprintln!("warning: {w}");
            }
            save(&p.mesh, &out)?;
        }
        _ => usage(),
    }
    Ok(())
}

fn save(m: &Mesh, path: &PathBuf) -> Result<()> {
    let tm = m.to_trimesh(path.file_stem().and_then(|s| s.to_str()).unwrap_or("scan"));
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
    match ext.as_str() {
        "stl" => lusion_meshio::save_stl(path, &[tm])?,
        "3mf" => lusion_meshio::save_3mf(path, &[tm])?,
        "obj" => lusion_meshio::save_obj(path, &[tm])?,
        "ply" => save_ply(path, &tm)?,
        _ => bail!("unsupported output format .{ext}"),
    }
    eprintln!("wrote {}", path.display());
    Ok(())
}

fn save_ply(path: &PathBuf, tm: &lusion_meshio::TriMesh) -> Result<()> {
    use std::io::Write;
    let mut f = std::io::BufWriter::new(std::fs::File::create(path)?);
    write!(f, "ply\nformat binary_little_endian 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar uint vertex_indices\nend_header\n", tm.positions.len(), tm.indices.len() / 3)?;
    for p in &tm.positions {
        for c in p {
            f.write_all(&c.to_le_bytes())?;
        }
    }
    for t in tm.indices.chunks_exact(3) {
        f.write_all(&[3u8])?;
        for i in t {
            f.write_all(&i.to_le_bytes())?;
        }
    }
    Ok(())
}
