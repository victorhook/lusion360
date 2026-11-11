//! Wavefront OBJ reader (+ MTL diffuse colours) and writer.

use anyhow::{bail, Context, Result};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::Path;

use crate::hash::{map_with_capacity, FxHashMap};
use crate::{normalize, TriMesh, DEFAULT_CREASE_DEG};

const NO_NORMAL: u32 = u32::MAX;

#[derive(Default)]
struct Builder {
    name: String,
    material: String,
    /// Triangle corners: (position index, normal index or NO_NORMAL).
    corners: Vec<[u32; 2]>,
}

fn parse_f32(tok: Option<&str>, line: usize, what: &str) -> Result<f32> {
    let t = tok.with_context(|| format!("OBJ line {line}: missing {what} component"))?;
    t.parse::<f32>()
        .with_context(|| format!("OBJ line {line}: invalid number '{t}'"))
}

/// Resolve a 1-based (or negative, relative) OBJ index against `len` elements.
fn resolve(s: &str, len: usize) -> Option<u32> {
    let i: i64 = s.parse().ok()?;
    let r = if i > 0 {
        i - 1
    } else if i < 0 {
        len as i64 + i
    } else {
        return None;
    };
    (r >= 0 && (r as usize) < len).then_some(r as u32)
}

pub(crate) fn load(bytes: &[u8], base_dir: Option<&Path>) -> Result<Vec<TriMesh>> {
    let text = String::from_utf8_lossy(bytes);
    let mut v: Vec<[f32; 3]> = Vec::new();
    let mut vn: Vec<[f32; 3]> = Vec::new();
    let mut materials: HashMap<String, [f32; 4]> = HashMap::new();
    let mut builders: Vec<Builder> = Vec::new();
    let mut lookup: HashMap<(String, String), usize> = HashMap::new();
    let mut cur_name = String::new();
    let mut cur_mtl = String::new();
    let mut cur: Option<usize> = None;
    let mut poly: Vec<[u32; 2]> = Vec::with_capacity(8);
    let mut skipped_faces = 0usize;

    for (ln, raw) in text.lines().enumerate() {
        let line_no = ln + 1;
        let line = match raw.find('#') {
            Some(p) => &raw[..p],
            None => raw,
        }
        .trim();
        if line.is_empty() {
            continue;
        }
        let (kw, rest) = match line.find(|c: char| c.is_ascii_whitespace()) {
            Some(p) => (&line[..p], line[p..].trim()),
            None => (line, ""),
        };
        match kw {
            "v" => {
                let mut it = rest.split_ascii_whitespace();
                let p = [
                    parse_f32(it.next(), line_no, "vertex")?,
                    parse_f32(it.next(), line_no, "vertex")?,
                    parse_f32(it.next(), line_no, "vertex")?,
                ];
                v.push(p);
            }
            "vn" => {
                let mut it = rest.split_ascii_whitespace();
                let n = [
                    parse_f32(it.next(), line_no, "normal")?,
                    parse_f32(it.next(), line_no, "normal")?,
                    parse_f32(it.next(), line_no, "normal")?,
                ];
                vn.push(n);
            }
            "f" | "fo" => {
                poly.clear();
                let mut bad = false;
                for tok in rest.split_ascii_whitespace() {
                    let mut parts = tok.split('/');
                    let pi = parts.next().and_then(|s| resolve(s, v.len()));
                    let _vt = parts.next();
                    let ni = parts
                        .next()
                        .and_then(|s| resolve(s, vn.len()))
                        .unwrap_or(NO_NORMAL);
                    match pi {
                        Some(pi) => poly.push([pi, ni]),
                        None => {
                            bad = true;
                            break;
                        }
                    }
                }
                if bad || poly.len() < 3 {
                    skipped_faces += 1;
                    continue;
                }
                let bi = match cur {
                    Some(i) => i,
                    None => {
                        let key = (cur_name.clone(), cur_mtl.clone());
                        let i = *lookup.entry(key).or_insert_with(|| {
                            builders.push(Builder {
                                name: cur_name.clone(),
                                material: cur_mtl.clone(),
                                corners: Vec::new(),
                            });
                            builders.len() - 1
                        });
                        cur = Some(i);
                        i
                    }
                };
                let corners = &mut builders[bi].corners;
                for i in 1..poly.len() - 1 {
                    corners.extend_from_slice(&[poly[0], poly[i], poly[i + 1]]);
                }
            }
            "o" | "g" => {
                if rest != cur_name {
                    cur_name = rest.to_string();
                    cur = None;
                }
            }
            "usemtl" => {
                if rest != cur_mtl {
                    cur_mtl = rest.to_string();
                    cur = None;
                }
            }
            "mtllib" => {
                if let Some(dir) = base_dir {
                    load_mtllib(dir, rest, &mut materials);
                }
            }
            _ => {} // vt, s, l, p, curves, ... ignored
        }
    }

    if builders.iter().all(|b| b.corners.is_empty()) {
        if skipped_faces > 0 {
            bail!("OBJ: all {skipped_faces} faces reference invalid vertex indices");
        }
        bail!("OBJ contains no faces");
    }

    let mut out = Vec::with_capacity(builders.len());
    for b in builders {
        if b.corners.is_empty() {
            continue;
        }
        let color = materials.get(&b.material).copied();
        let name = if b.name.is_empty() {
            b.material.clone()
        } else {
            b.name.clone()
        };
        let has_normals = b.corners.iter().all(|c| c[1] != NO_NORMAL);
        let mut map: FxHashMap<u64, u32> = map_with_capacity(b.corners.len() / 2 + 8);
        let mut mesh = TriMesh {
            name,
            color,
            ..Default::default()
        };
        mesh.indices.reserve(b.corners.len());
        for c in &b.corners {
            let key = if has_normals {
                ((c[0] as u64) << 32) | c[1] as u64
            } else {
                c[0] as u64
            };
            let next = mesh.positions.len() as u32;
            let i = *map.entry(key).or_insert(next);
            if i == next {
                mesh.positions.push(v[c[0] as usize]);
                if has_normals {
                    mesh.normals.push(normalize(vn[c[1] as usize]));
                }
            }
            mesh.indices.push(i);
        }
        if !has_normals || mesh.normals.contains(&[0.0; 3]) {
            mesh.compute_normals(DEFAULT_CREASE_DEG);
        }
        out.push(mesh);
    }
    Ok(out)
}

fn load_mtllib(dir: &Path, rest: &str, materials: &mut HashMap<String, [f32; 4]>) {
    let mut candidates = vec![rest.to_string()];
    candidates.extend(rest.split_ascii_whitespace().map(str::to_string));
    for c in candidates {
        if let Ok(bytes) = std::fs::read(dir.join(&c)) {
            parse_mtl(&String::from_utf8_lossy(&bytes), materials);
            if !rest.contains(' ') {
                break;
            }
        }
    }
}

/// Parse `newmtl`/`Kd`/`d`/`Tr` from an MTL file. Kd is taken as-is (treated as linear).
pub(crate) fn parse_mtl(text: &str, materials: &mut HashMap<String, [f32; 4]>) {
    let mut cur: Option<String> = None;
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        let mut it = line.split_ascii_whitespace();
        let Some(kw) = it.next() else { continue };
        match kw {
            "newmtl" => {
                let name = line["newmtl".len()..].trim().to_string();
                materials
                    .entry(name.clone())
                    .or_insert([0.8, 0.8, 0.8, 1.0]);
                cur = Some(name);
            }
            "Kd" => {
                let vals: Vec<f32> = it.filter_map(|t| t.parse().ok()).collect();
                if let (Some(name), 3..) = (&cur, vals.len()) {
                    if let Some(m) = materials.get_mut(name) {
                        m[0] = vals[0];
                        m[1] = vals[1];
                        m[2] = vals[2];
                    }
                }
            }
            "d" | "Tr" => {
                if let (Some(name), Some(a)) = (&cur, it.next().and_then(|t| t.parse::<f32>().ok()))
                {
                    if let Some(m) = materials.get_mut(name) {
                        m[3] = if kw == "d" { a } else { 1.0 - a }.clamp(0.0, 1.0);
                    }
                }
            }
            _ => {}
        }
    }
}

pub(crate) fn write(meshes: &[TriMesh]) -> Result<Vec<u8>> {
    let mut s = String::with_capacity(
        64 + meshes
            .iter()
            .map(|m| m.positions.len() * 70 + m.indices.len() * 8)
            .sum::<usize>(),
    );
    s.push_str("# Wavefront OBJ exported by Lusion360 (units: mm)\n");
    let (mut vbase, mut nbase) = (1usize, 1usize);
    for (mi, m) in meshes.iter().enumerate() {
        let computed;
        let m = if m.normals.len() == m.positions.len() {
            m
        } else {
            let mut c = m.clone();
            c.compute_normals(DEFAULT_CREASE_DEG);
            computed = c;
            &computed
        };
        let name: String = m
            .name
            .chars()
            .map(|c| {
                if c.is_whitespace() || c == '#' {
                    '_'
                } else {
                    c
                }
            })
            .collect();
        let name = if name.is_empty() {
            format!("mesh_{}", mi + 1)
        } else {
            name
        };
        let _ = writeln!(s, "o {name}");
        // Positions are written once per distinct point so faces share
        // vertices (slicers need a watertight, manifold index structure);
        // normals are indexed separately so creases stay sharp.
        let mut uniq: std::collections::HashMap<[u32; 3], usize> = std::collections::HashMap::new();
        let mut vmap: Vec<usize> = Vec::with_capacity(m.positions.len());
        let mut nv = 0usize;
        for p in &m.positions {
            let key = [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            let id = *uniq.entry(key).or_insert_with(|| {
                let _ = writeln!(s, "v {} {} {}", p[0], p[1], p[2]);
                nv += 1;
                nv - 1
            });
            vmap.push(id);
        }
        for n in &m.normals {
            let _ = writeln!(s, "vn {} {} {}", n[0], n[1], n[2]);
        }
        let np = m.positions.len();
        for t in m.indices.as_chunks::<3>().0 {
            if t.iter().any(|&i| i as usize >= np) {
                continue;
            }
            let v = |i: u32| vmap[i as usize] + vbase;
            let n = |i: u32| i as usize + nbase;
            if v(t[0]) == v(t[1]) || v(t[1]) == v(t[2]) || v(t[0]) == v(t[2]) {
                continue; // degenerate after welding
            }
            let _ = writeln!(s, "f {}//{} {}//{} {}//{}", v(t[0]), n(t[0]), v(t[1]), n(t[1]), v(t[2]), n(t[2]));
        }
        vbase += nv;
        nbase += np;
    }
    Ok(s.into_bytes())
}
