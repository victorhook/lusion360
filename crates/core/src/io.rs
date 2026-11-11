//! File import/export.

use crate::doc::*;
use crate::model::{Body, ModelState};
use crate::tess::BodyMesh;
use anyhow::{anyhow, bail, Context, Result};
use lusion_kernel::Shape;
use std::collections::HashMap;
use std::path::Path;
use std::sync::Arc;

pub const NATIVE_EXT: &str = "l360";

pub fn ext_of(path: &Path) -> String {
    path.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase()
}

pub fn cad_import_extensions() -> &'static [&'static str] {
    &["step", "stp", "brep", "brp"]
}

pub fn all_open_extensions() -> Vec<&'static str> {
    let mut v = vec![NATIVE_EXT];
    v.extend_from_slice(cad_import_extensions());
    v.extend_from_slice(lusion_meshio::supported_import_extensions());
    v
}

pub fn export_extensions() -> &'static [&'static str] {
    &["step", "stl", "3mf", "obj", "brep"]
}

pub fn load_native(path: &Path) -> Result<Document> {
    let s = std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    Document::from_json(&s).with_context(|| format!("parsing {}", path.display()))
}

pub fn save_native(doc: &Document, path: &Path) -> Result<()> {
    let json = doc.to_json()?;
    // atomic-ish write: temp file then rename
    let tmp = path.with_extension("l360.tmp");
    std::fs::write(&tmp, json).with_context(|| format!("writing {}", tmp.display()))?;
    std::fs::rename(&tmp, path).with_context(|| format!("saving {}", path.display()))?;
    Ok(())
}

fn stem(path: &Path) -> String {
    path.file_stem().and_then(|s| s.to_str()).unwrap_or("Imported").to_string()
}

/// Read a foreign file and append import features to `doc` in `component`.
/// Returns the number of features added.
pub fn import_into(doc: &mut Document, path: &Path, component: CompId) -> Result<usize> {
    let ext = ext_of(path);
    let p = path.to_str().ok_or_else(|| anyhow!("non UTF-8 path"))?;
    match ext.as_str() {
        "step" | "stp" | "brep" | "brp" => {
            let shape = if ext.starts_with('b') { Shape::read_brep_file(p) } else { Shape::read_step(p) }
                .map_err(|e| anyhow!(e.0))?;
            let brep = shape.to_brep_string().map_err(|e| anyhow!(e.0))?;
            let name = stem(path);
            doc.insert_feature(
                name,
                component,
                FeatureKind::ImportShape(ImportShape { source: path.display().to_string(), brep: Arc::new(brep) }),
            );
            Ok(1)
        }
        e if lusion_meshio::supported_import_extensions().contains(&e) => {
            let mut meshes = lusion_meshio::load_mesh_file(path)?;
            if meshes.is_empty() {
                bail!("{} contains no triangles", path.display());
            }
            // files with very many objects (e.g. OBJ groups) become one body
            if meshes.len() > 32 {
                let mut merged = lusion_meshio::TriMesh { name: stem(path), ..Default::default() };
                for m in meshes {
                    let base = merged.positions.len() as u32;
                    merged.positions.extend_from_slice(&m.positions);
                    merged.indices.extend(m.indices.iter().map(|i| i + base));
                }
                meshes = vec![merged];
            }
            let n = meshes.len();
            for m in meshes {
                let name = if m.name.is_empty() { stem(path) } else { m.name.clone() };
                doc.insert_feature(
                    name,
                    component,
                    FeatureKind::ImportMesh(ImportMesh {
                        source: path.display().to_string(),
                        positions: Arc::new(m.positions.iter().flatten().copied().collect()),
                        indices: Arc::new(m.indices),
                    }),
                );
            }
            Ok(n)
        }
        _ => bail!("unsupported file type '.{ext}'"),
    }
}

/// Open any supported file as a document (native or imported).
pub fn open_any(path: &Path) -> Result<Document> {
    if ext_of(path) == NATIVE_EXT {
        return load_native(path);
    }
    let mut doc = Document::default();
    doc.components[0].name = stem(path);
    import_into(&mut doc, path, ROOT)?;
    Ok(doc)
}

fn body_trimesh(b: &Body, fine: bool) -> Result<lusion_meshio::TriMesh> {
    let m = if let Some(im) = &b.mesh {
        BodyMesh::from_import(im)
    } else {
        let diag = b.shape.bbox().map(|(a, c)| (c - a).length()).unwrap_or(1.0);
        let lin = if fine { (diag * 0.0002).clamp(0.001, 0.05) } else { (diag * 0.001).clamp(0.005, 0.5) };
        let raw = b.shape.mesh(lin, if fine { 0.1 } else { 0.3 }).map_err(|e| anyhow!(e.0))?;
        BodyMesh {
            positions: raw.positions.chunks(3).map(|c| [c[0], c[1], c[2]]).collect(),
            normals: raw.normals.chunks(3).map(|c| [c[0], c[1], c[2]]).collect(),
            indices: raw.indices,
            ..Default::default()
        }
    };
    let (positions, indices) = weld(&m.positions, &m.indices);
    let normals = vertex_normals(&positions, &indices);
    Ok(lusion_meshio::TriMesh { name: format!("body{}", b.id.0), positions, normals, indices, color: None })
}

/// Merge vertices closer than a small tolerance (B-rep tessellations have a
/// separate copy of every boundary/seam node per face) and drop triangles that
/// become degenerate. Result: a closed, manifold mesh for closed solids.
pub fn weld(pos: &[[f32; 3]], idx: &[u32]) -> (Vec<[f32; 3]>, Vec<u32>) {
    let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
    for p in pos {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diag = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2) + (hi[2] - lo[2]).powi(2)).sqrt().max(1e-3);
    let tol = (diag * 2e-6).max(1e-6);
    let cell = tol * 2.0;
    let key = |p: &[f32; 3]| [(p[0] / cell).floor() as i64, (p[1] / cell).floor() as i64, (p[2] / cell).floor() as i64];
    let mut grid: HashMap<[i64; 3], Vec<u32>> = HashMap::new();
    let mut out: Vec<[f32; 3]> = Vec::new();
    let mut remap: Vec<u32> = Vec::with_capacity(pos.len());
    for p in pos {
        let k = key(p);
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(list) = grid.get(&[k[0] + dx, k[1] + dy, k[2] + dz]) {
                        for &i in list {
                            let q = out[i as usize];
                            if (q[0] - p[0]).abs() <= tol && (q[1] - p[1]).abs() <= tol && (q[2] - p[2]).abs() <= tol {
                                found = Some(i);
                                break 'search;
                            }
                        }
                    }
                }
            }
        }
        let id = match found {
            Some(i) => i,
            None => {
                out.push(*p);
                let i = (out.len() - 1) as u32;
                grid.entry(k).or_default().push(i);
                i
            }
        };
        remap.push(id);
    }
    let mut tris = Vec::with_capacity(idx.len());
    for t in idx.chunks_exact(3) {
        let (a, b, c) = (remap[t[0] as usize], remap[t[1] as usize], remap[t[2] as usize]);
        if a != b && b != c && a != c {
            tris.extend_from_slice(&[a, b, c]);
        }
    }
    (out, tris)
}

fn vertex_normals(pos: &[[f32; 3]], idx: &[u32]) -> Vec<[f32; 3]> {
    let mut n = vec![[0.0f32; 3]; pos.len()];
    for t in idx.chunks_exact(3) {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let f = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        for &i in t {
            for k in 0..3 {
                n[i as usize][k] += f[k];
            }
        }
    }
    for v in n.iter_mut() {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        if l > 0.0 {
            *v = [v[0] / l, v[1] / l, v[2] / l];
        } else {
            *v = [0.0, 0.0, 1.0];
        }
    }
    n
}

/// Export bodies to `path`; format chosen by extension.
pub fn export(path: &Path, bodies: &[&Body], names: &[String]) -> Result<()> {
    if bodies.is_empty() {
        bail!("nothing to export");
    }
    let ext = ext_of(path);
    let p = path.to_str().ok_or_else(|| anyhow!("non UTF-8 path"))?;
    match ext.as_str() {
        "step" | "stp" | "brep" => {
            let shapes: Vec<Shape> = bodies.iter().filter(|b| !b.is_mesh()).map(|b| b.shape.clone()).collect();
            if shapes.is_empty() {
                bail!("mesh bodies cannot be exported as {ext}; use STL/3MF/OBJ");
            }
            let c = if shapes.len() == 1 { shapes[0].clone() } else { Shape::compound(&shapes) };
            if ext == "brep" {
                std::fs::write(path, c.to_brep_string().map_err(|e| anyhow!(e.0))?)?;
            } else {
                c.write_step(p).map_err(|e| anyhow!(e.0))?;
            }
        }
        "stl" | "obj" | "3mf" => {
            let mut meshes = Vec::new();
            for (i, b) in bodies.iter().enumerate() {
                let mut m = body_trimesh(b, true)?;
                if let Some(n) = names.get(i) {
                    m.name = n.clone();
                }
                meshes.push(m);
            }
            match ext.as_str() {
                "stl" => lusion_meshio::save_stl(path, &meshes)?,
                "obj" => lusion_meshio::save_obj(path, &meshes)?,
                _ => lusion_meshio::save_3mf(path, &meshes)?,
            }
        }
        _ => bail!("unsupported export type '.{ext}'"),
    }
    Ok(())
}

/// Bodies visible in the given state (component + body visibility).
pub fn visible_bodies<'a>(doc: &Document, st: &'a ModelState) -> Vec<&'a Body> {
    st.bodies
        .values()
        .filter(|b| {
            let props = doc.body_props(b.id);
            props.visible.unwrap_or(true) && doc.component_visible(props.component.unwrap_or(b.component))
        })
        .collect()
}
