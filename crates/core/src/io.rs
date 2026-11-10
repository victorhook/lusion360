//! File import/export.

use crate::doc::*;
use crate::model::{Body, ModelState};
use crate::tess::BodyMesh;
use anyhow::{anyhow, bail, Context, Result};
use lusion_kernel::Shape;
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
            let meshes = lusion_meshio::load_mesh_file(path)?;
            if meshes.is_empty() {
                bail!("{} contains no triangles", path.display());
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
    Ok(lusion_meshio::TriMesh {
        name: format!("body{}", b.id.0),
        positions: m.positions,
        normals: m.normals,
        indices: m.indices,
        color: None,
    })
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
