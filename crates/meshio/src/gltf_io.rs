//! glTF 2.0 / GLB reader (via the `gltf` crate, without its image-decoding `import` feature).

use anyhow::{anyhow, bail, Context, Result};
use base64::Engine as _;
use std::path::Path;

use crate::{normalize, TriMesh, DEFAULT_CREASE_DEG};

/// glTF is in metres.
const M_TO_MM: f32 = 1000.0;

type Mat4 = [[f32; 4]; 4]; // column-major, m[col][row]

const IDENTITY: Mat4 = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
];

fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut r = [[0f32; 4]; 4];
    for (c, col) in r.iter_mut().enumerate() {
        for (row, v) in col.iter_mut().enumerate() {
            *v = (0..4).map(|k| a[k][row] * b[c][k]).sum();
        }
    }
    r
}

fn xform_point(m: &Mat4, p: [f32; 3]) -> [f32; 3] {
    let mut r = [0f32; 3];
    for (row, v) in r.iter_mut().enumerate() {
        *v = m[0][row] * p[0] + m[1][row] * p[1] + m[2][row] * p[2] + m[3][row];
    }
    r
}

/// Cofactor matrix of the upper 3x3 (== det * inverse-transpose), as rows, and the determinant.
fn normal_matrix(m: &Mat4) -> ([[f32; 3]; 3], f32) {
    let a = |r: usize, c: usize| m[c][r];
    let cof = [
        [
            a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1),
            a(1, 2) * a(2, 0) - a(1, 0) * a(2, 2),
            a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0),
        ],
        [
            a(0, 2) * a(2, 1) - a(0, 1) * a(2, 2),
            a(0, 0) * a(2, 2) - a(0, 2) * a(2, 0),
            a(0, 1) * a(2, 0) - a(0, 0) * a(2, 1),
        ],
        [
            a(0, 1) * a(1, 2) - a(0, 2) * a(1, 1),
            a(0, 2) * a(1, 0) - a(0, 0) * a(1, 2),
            a(0, 0) * a(1, 1) - a(0, 1) * a(1, 0),
        ],
    ];
    let det = a(0, 0) * cof[0][0] + a(0, 1) * cof[0][1] + a(0, 2) * cof[0][2];
    (cof, det)
}

fn percent_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() {
            let hex = |c: u8| (c as char).to_digit(16);
            if let (Some(h), Some(l)) = (hex(b[i + 1]), hex(b[i + 2])) {
                out.push((h * 16 + l) as u8);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn decode_data_uri(uri: &str) -> Result<Vec<u8>> {
    let comma = uri.find(',').context("malformed data URI")?;
    let (meta, data) = (&uri[..comma], &uri[comma + 1..]);
    if meta.ends_with(";base64") {
        base64::engine::general_purpose::STANDARD
            .decode(data.trim())
            .context("invalid base64 in data URI")
    } else {
        Ok(percent_decode(data).into_bytes())
    }
}

pub(crate) fn load(bytes: &[u8], base_dir: Option<&Path>) -> Result<Vec<TriMesh>> {
    let gltf = gltf::Gltf::from_slice(bytes).map_err(|e| anyhow!("invalid glTF: {e}"))?;
    let mut blob = gltf.blob.clone();
    let doc = &gltf.document;

    let mut buffers: Vec<Vec<u8>> = Vec::new();
    for buf in doc.buffers() {
        let data = match buf.source() {
            gltf::buffer::Source::Bin => blob
                .take()
                .context("glTF buffer refers to a missing GLB binary chunk")?,
            gltf::buffer::Source::Uri(uri) if uri.starts_with("data:") => {
                decode_data_uri(uri).with_context(|| format!("glTF buffer {}", buf.index()))?
            }
            gltf::buffer::Source::Uri(uri) => {
                let dir = base_dir.with_context(|| {
                    format!("glTF buffer '{uri}' is an external file; load from a path instead")
                })?;
                let p = dir.join(percent_decode(uri));
                std::fs::read(&p)
                    .with_context(|| format!("failed to read glTF buffer {}", p.display()))?
            }
        };
        if data.len() < buf.length() {
            bail!(
                "glTF buffer {} is {} bytes but declares {}",
                buf.index(),
                data.len(),
                buf.length()
            );
        }
        buffers.push(data);
    }

    // The gltf crate's accessors are bounds-checked, but guard against any arithmetic panic
    // on malformed input.
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| walk(doc, &buffers)));
    match result {
        Ok(r) => r,
        Err(_) => bail!("malformed glTF (internal error while reading accessors)"),
    }
}

fn walk(doc: &gltf::Document, buffers: &[Vec<u8>]) -> Result<Vec<TriMesh>> {
    let roots: Vec<gltf::Node> = match doc.default_scene().or_else(|| doc.scenes().next()) {
        Some(scene) => scene.nodes().collect(),
        None => {
            let mut is_child = vec![false; doc.nodes().len()];
            for n in doc.nodes() {
                for c in n.children() {
                    is_child[c.index()] = true;
                }
            }
            doc.nodes().filter(|n| !is_child[n.index()]).collect()
        }
    };

    let mut out = Vec::new();
    let mut stack: Vec<(gltf::Node, Mat4, usize)> =
        roots.into_iter().rev().map(|n| (n, IDENTITY, 0)).collect();
    let mut visits = 0usize;
    while let Some((node, parent, depth)) = stack.pop() {
        visits += 1;
        if depth > 256 || visits > 1_000_000 {
            bail!("glTF node hierarchy too deep or cyclic");
        }
        let world = mul(&parent, &node.transform().matrix());
        if let Some(mesh) = node.mesh() {
            load_mesh_instance(&node, &mesh, &world, buffers, &mut out)?;
        }
        for c in node.children().collect::<Vec<_>>().into_iter().rev() {
            stack.push((c, world, depth + 1));
        }
    }
    if out.is_empty() {
        bail!(
            "glTF contains no triangle meshes (compressed meshes such as Draco are not supported)"
        );
    }
    Ok(out)
}

fn load_mesh_instance(
    node: &gltf::Node,
    mesh: &gltf::Mesh,
    world: &Mat4,
    buffers: &[Vec<u8>],
    out: &mut Vec<TriMesh>,
) -> Result<()> {
    let (cof, det) = normal_matrix(world);
    let flip = det < 0.0;
    let nsign = if flip { -1.0 } else { 1.0 };
    let base_name = node
        .name()
        .or(mesh.name())
        .map(str::to_string)
        .unwrap_or_else(|| format!("mesh{}", mesh.index()));

    // Group primitives by material: one TriMesh per (node, material).
    let mut groups: Vec<(Option<usize>, TriMesh)> = Vec::new();
    for prim in mesh.primitives() {
        use gltf::mesh::Mode;
        let mode = prim.mode();
        if !matches!(
            mode,
            Mode::Triangles | Mode::TriangleStrip | Mode::TriangleFan
        ) {
            continue;
        }
        let reader = prim.reader(|b| buffers.get(b.index()).map(|v| v.as_slice()));
        let Some(pos_iter) = reader.read_positions() else {
            continue;
        };
        let positions: Vec<[f32; 3]> = pos_iter
            .map(|p| {
                let w = xform_point(world, p);
                [w[0] * M_TO_MM, w[1] * M_TO_MM, w[2] * M_TO_MM]
            })
            .collect();
        let n = positions.len();
        let raw: Vec<u32> = match reader.read_indices() {
            Some(it) => it.into_u32().collect(),
            None => (0..n as u32).collect(),
        };
        let mut indices = Vec::with_capacity(raw.len());
        match mode {
            Mode::Triangles => indices.extend_from_slice(&raw[..raw.len() / 3 * 3]),
            Mode::TriangleStrip => {
                for i in 2..raw.len() {
                    if i % 2 == 0 {
                        indices.extend_from_slice(&[raw[i - 2], raw[i - 1], raw[i]]);
                    } else {
                        indices.extend_from_slice(&[raw[i - 1], raw[i - 2], raw[i]]);
                    }
                }
            }
            _ => {
                if raw.len() >= 3 {
                    crate::fan(&raw, &mut indices);
                }
            }
        }
        crate::sanitize_indices(&mut indices, n);
        if flip {
            for t in indices.as_chunks_mut::<3>().0 {
                t.swap(1, 2);
            }
        }
        let normals: Option<Vec<[f32; 3]>> = reader.read_normals().map(|it| {
            it.map(|v| {
                let r = [
                    cof[0][0] * v[0] + cof[0][1] * v[1] + cof[0][2] * v[2],
                    cof[1][0] * v[0] + cof[1][1] * v[1] + cof[1][2] * v[2],
                    cof[2][0] * v[0] + cof[2][1] * v[1] + cof[2][2] * v[2],
                ];
                normalize([r[0] * nsign, r[1] * nsign, r[2] * nsign])
            })
            .collect()
        });
        let mut tm = TriMesh {
            positions,
            indices,
            ..Default::default()
        };
        match normals {
            Some(nr) if nr.len() == n && !nr.contains(&[0.0; 3]) => tm.normals = nr,
            _ => tm.compute_normals(DEFAULT_CREASE_DEG),
        }
        if tm.indices.is_empty() {
            continue;
        }

        let mat = prim.material();
        let key = mat.index();
        let gi = match groups.iter().position(|(k, _)| *k == key) {
            Some(i) => i,
            None => {
                let color = key.map(|_| mat.pbr_metallic_roughness().base_color_factor());
                groups.push((
                    key,
                    TriMesh {
                        color,
                        ..Default::default()
                    },
                ));
                groups.len() - 1
            }
        };
        let g = &mut groups[gi].1;
        let off = g.positions.len() as u32;
        g.positions.extend_from_slice(&tm.positions);
        g.normals.extend_from_slice(&tm.normals);
        g.indices.extend(tm.indices.iter().map(|i| i + off));
    }

    let multi = groups.len() > 1;
    for (key, mut g) in groups {
        g.name = match (multi, key) {
            (true, Some(k)) => {
                let mname = mesh
                    .primitives()
                    .find(|p| p.material().index() == Some(k))
                    .and_then(|p| p.material().name().map(str::to_string))
                    .unwrap_or_else(|| format!("material{k}"));
                format!("{base_name} ({mname})")
            }
            _ => base_name.clone(),
        };
        out.push(g);
    }
    Ok(())
}
