//! Triangle-mesh file import/export for Lusion360.
//!
//! Import: STL (ascii/binary), OBJ (+MTL diffuse colour), glTF/GLB, 3MF, PLY, OFF.
//! Export: binary STL, OBJ, 3MF.
//!
//! All positions are in millimetres. Loaders always fill per-vertex normals.

use anyhow::{bail, Context, Result};
use std::path::Path;

mod gltf_io;
mod hash;
mod normals;
mod obj;
mod off;
mod ply;
mod stl;
mod threemf;

#[cfg(test)]
mod tests;

/// Default crease angle used by loaders when a file carries no normals.
pub(crate) const DEFAULT_CREASE_DEG: f32 = 30.0;

#[derive(Clone, Debug, Default)]
pub struct TriMesh {
    pub name: String,
    /// Millimetres.
    pub positions: Vec<[f32; 3]>,
    /// Per-vertex, same len as positions (always filled by loaders).
    pub normals: Vec<[f32; 3]>,
    /// Triangle list.
    pub indices: Vec<u32>,
    /// Linear RGBA if the file specifies one for the object/material.
    pub color: Option<[f32; 4]>,
}

impl TriMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn bounds(&self) -> Option<([f32; 3], [f32; 3])> {
        let mut it = self
            .positions
            .iter()
            .filter(|p| p.iter().all(|c| c.is_finite()));
        let first = *it.next()?;
        let (mut lo, mut hi) = (first, first);
        for p in it {
            for k in 0..3 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        Some((lo, hi))
    }

    /// Recompute normals: flat-shaded look for hard edges but smooth across small angles:
    /// split vertices where the dihedral angle between adjacent face normals exceeds `crease_deg`.
    ///
    /// Vertices with identical positions are treated as shared (the mesh is welded first),
    /// so this also works on "triangle soup" input. Triangles with out-of-range indices are
    /// dropped. `positions`, `normals` and `indices` are all rewritten.
    pub fn compute_normals(&mut self, crease_deg: f32) {
        normals::compute_normals(self, crease_deg);
    }
}

pub fn supported_import_extensions() -> &'static [&'static str] {
    &["stl", "obj", "gltf", "glb", "3mf", "ply", "off"]
}

/// Load a mesh file, dispatching on its (case-insensitive) extension.
pub fn load_mesh_file(path: &Path) -> Result<Vec<TriMesh>> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_ascii_lowercase())
        .with_context(|| format!("{}: file has no extension", path.display()))?;
    let bytes =
        std::fs::read(path).with_context(|| format!("failed to read {}", path.display()))?;
    let base_dir = path.parent();
    let stem = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    let mut meshes = load_impl(&bytes, &ext, base_dir)
        .with_context(|| format!("failed to load {}", path.display()))?;
    for (i, m) in meshes.iter_mut().enumerate() {
        if m.name.trim().is_empty() {
            m.name = if i == 0 {
                stem.clone()
            } else {
                format!("{stem}_{i}")
            };
        }
    }
    Ok(meshes)
}

/// Load a mesh from memory. `ext` is the file extension (with or without a leading dot,
/// case-insensitive). glTF files referencing external buffers cannot be loaded this way.
pub fn load_mesh_bytes(bytes: &[u8], ext: &str) -> Result<Vec<TriMesh>> {
    let ext = ext.trim_start_matches('.').to_ascii_lowercase();
    load_impl(bytes, &ext, None)
}

fn load_impl(bytes: &[u8], ext: &str, base_dir: Option<&Path>) -> Result<Vec<TriMesh>> {
    let meshes = match ext {
        "stl" => stl::load(bytes)?,
        "obj" => obj::load(bytes, base_dir)?,
        "gltf" | "glb" => gltf_io::load(bytes, base_dir)?,
        "3mf" => threemf::load(bytes)?,
        "ply" => ply::load(bytes)?,
        "off" => off::load(bytes)?,
        other => bail!(
            "unsupported mesh format '.{other}' (supported: {})",
            supported_import_extensions().join(", ")
        ),
    };
    Ok(meshes
        .into_iter()
        .filter(|m| !m.indices.is_empty())
        .collect())
}

/// Binary STL, all meshes merged.
pub fn save_stl(path: &Path, meshes: &[TriMesh]) -> Result<()> {
    let bytes = stl::write_binary(meshes)?;
    std::fs::write(path, bytes).with_context(|| format!("failed to write {}", path.display()))
}

/// Wavefront OBJ: one `o` per mesh, with normals.
pub fn save_obj(path: &Path, meshes: &[TriMesh]) -> Result<()> {
    let bytes = obj::write(meshes)?;
    std::fs::write(path, bytes).with_context(|| format!("failed to write {}", path.display()))
}

/// 3MF package (core spec), unit="millimeter", one object + build item per mesh.
/// Vertices are welded by exact position and degenerate triangles dropped.
pub fn save_3mf(path: &Path, meshes: &[TriMesh]) -> Result<()> {
    let bytes = threemf::write(meshes)?;
    std::fs::write(path, bytes).with_context(|| format!("failed to write {}", path.display()))
}

// ---------------------------------------------------------------------------------------
// Small shared helpers
// ---------------------------------------------------------------------------------------

pub(crate) fn srgb_to_linear(c: f32) -> f32 {
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

pub(crate) fn linear_to_srgb(c: f32) -> f32 {
    let c = c.clamp(0.0, 1.0);
    if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

/// Unit face normal of a triangle (zero for degenerate triangles).
pub(crate) fn face_normal(a: [f32; 3], b: [f32; 3], c: [f32; 3]) -> [f32; 3] {
    let u = sub(b, a);
    let v = sub(c, a);
    normalize(cross(u, v))
}

#[inline]
pub(crate) fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub(crate) fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub(crate) fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub(crate) fn normalize(v: [f32; 3]) -> [f32; 3] {
    let l2 = dot(v, v);
    if l2 > 0.0 && l2.is_finite() {
        let inv = 1.0 / l2.sqrt();
        [v[0] * inv, v[1] * inv, v[2] * inv]
    } else {
        [0.0, 0.0, 0.0]
    }
}

/// Fan-triangulate a polygon given as a slice of vertex indices, appending to `out`.
pub(crate) fn fan(poly: &[u32], out: &mut Vec<u32>) {
    if poly.len() < 3 {
        return;
    }
    for i in 1..poly.len() - 1 {
        out.extend_from_slice(&[poly[0], poly[i], poly[i + 1]]);
    }
}

/// Drop triangles that reference out-of-range vertices.
pub(crate) fn sanitize_indices(indices: &mut Vec<u32>, vertex_count: usize) {
    indices.truncate(indices.len() / 3 * 3);
    if indices.iter().all(|&i| (i as usize) < vertex_count) {
        return;
    }
    let mut w = 0;
    for t in 0..indices.len() / 3 {
        let tri = [indices[3 * t], indices[3 * t + 1], indices[3 * t + 2]];
        if tri.iter().all(|&i| (i as usize) < vertex_count) {
            indices[w..w + 3].copy_from_slice(&tri);
            w += 3;
        }
    }
    indices.truncate(w);
}
