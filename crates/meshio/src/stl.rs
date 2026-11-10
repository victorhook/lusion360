//! STL (ascii + binary) reader and binary writer.

use anyhow::{bail, Context, Result};

use crate::hash::Welder;
use crate::{face_normal, fan, srgb_to_linear, TriMesh, DEFAULT_CREASE_DEG};

pub(crate) fn load(bytes: &[u8]) -> Result<Vec<TriMesh>> {
    let starts_solid = trim_ascii_start(bytes).starts_with(b"solid");
    if bytes.len() < 84 {
        if starts_solid {
            return load_ascii(bytes);
        }
        bail!("file too small to be an STL ({} bytes)", bytes.len());
    }
    let n = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as u64;
    let len = bytes.len() as u64;
    let expected = 84 + 50 * n;
    if expected == len {
        return load_binary(bytes, n as usize);
    }
    if starts_solid {
        match load_ascii(bytes) {
            Ok(m) => return Ok(m),
            Err(e) if expected > len => return Err(e),
            Err(_) => {} // fall through: binary file whose header starts with "solid"
        }
    }
    if expected <= len && n > 0 {
        // Binary with trailing garbage.
        return load_binary(bytes, n as usize);
    }
    if (len - 84).is_multiple_of(50) {
        // Header triangle count is wrong (seen in the wild); trust the file size.
        return load_binary(bytes, ((len - 84) / 50) as usize);
    }
    bail!(
        "truncated binary STL: header declares {n} triangles ({expected} bytes) but file has {len} bytes"
    )
}

fn trim_ascii_start(b: &[u8]) -> &[u8] {
    let i = b
        .iter()
        .position(|c| !c.is_ascii_whitespace())
        .unwrap_or(b.len());
    &b[i..]
}

#[inline]
fn rd_f32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}

fn load_binary(bytes: &[u8], n: usize) -> Result<Vec<TriMesh>> {
    let data = &bytes[84..];
    let n = n.min(data.len() / 50);
    let mut welder = Welder::with_capacity(n / 2 + 16);
    let mut indices = Vec::with_capacity(n * 3);
    for rec in data.as_chunks::<50>().0.iter().take(n) {
        let mut tri = [0u32; 3];
        let mut finite = true;
        for (v, slot) in tri.iter_mut().enumerate() {
            let o = 12 + 12 * v;
            let p = [rd_f32(rec, o), rd_f32(rec, o + 4), rd_f32(rec, o + 8)];
            finite &= p.iter().all(|c| c.is_finite());
            *slot = welder.index(p);
        }
        if finite {
            indices.extend_from_slice(&tri);
        }
    }
    if indices.is_empty() {
        bail!("binary STL contains no (valid) triangles");
    }
    let header = &bytes[..80];
    let color = header
        .windows(6)
        .position(|w| w == b"COLOR=")
        .filter(|&p| p + 10 <= 80)
        .map(|p| {
            let c = &header[p + 6..p + 10];
            [
                srgb_to_linear(c[0] as f32 / 255.0),
                srgb_to_linear(c[1] as f32 / 255.0),
                srgb_to_linear(c[2] as f32 / 255.0),
                c[3] as f32 / 255.0,
            ]
        });
    let mut mesh = TriMesh {
        name: String::new(),
        positions: welder.positions,
        normals: Vec::new(),
        indices,
        color,
    };
    mesh.compute_normals(DEFAULT_CREASE_DEG);
    Ok(vec![mesh])
}

struct Cursor<'a> {
    b: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    #[inline]
    fn token(&mut self) -> Option<&'a [u8]> {
        let b = self.b;
        let mut i = self.pos;
        while i < b.len() && b[i].is_ascii_whitespace() {
            i += 1;
        }
        if i >= b.len() {
            self.pos = i;
            return None;
        }
        let s = i;
        while i < b.len() && !b[i].is_ascii_whitespace() {
            i += 1;
        }
        self.pos = i;
        Some(&b[s..i])
    }

    fn rest_of_line(&mut self) -> &'a [u8] {
        let b = self.b;
        let s = self.pos;
        let mut i = s;
        while i < b.len() && b[i] != b'\n' {
            i += 1;
        }
        self.pos = i;
        b[s..i].trim_ascii()
    }

    fn line(&self) -> usize {
        1 + self.b[..self.pos.min(self.b.len())]
            .iter()
            .filter(|&&c| c == b'\n')
            .count()
    }
}

fn load_ascii(bytes: &[u8]) -> Result<Vec<TriMesh>> {
    struct Cur {
        name: String,
        welder: Welder,
        indices: Vec<u32>,
    }
    let new_cur = |name: String| Cur {
        name,
        welder: Welder::with_capacity(1024),
        indices: Vec::new(),
    };
    let finish = |c: Cur, out: &mut Vec<TriMesh>| {
        if !c.indices.is_empty() {
            let mut m = TriMesh {
                name: c.name,
                positions: c.welder.positions,
                normals: Vec::new(),
                indices: c.indices,
                color: None,
            };
            m.compute_normals(DEFAULT_CREASE_DEG);
            out.push(m);
        }
    };

    let mut out = Vec::new();
    let mut cur: Option<Cur> = None;
    let mut facet: Vec<u32> = Vec::with_capacity(4);
    let mut cs = Cursor { b: bytes, pos: 0 };
    while let Some(tok) = cs.token() {
        match tok {
            b"solid" => {
                if let Some(c) = cur.take() {
                    finish(c, &mut out);
                }
                let name = String::from_utf8_lossy(cs.rest_of_line()).into_owned();
                cur = Some(new_cur(name));
            }
            b"endsolid" => {
                cs.rest_of_line();
                let c = cur.get_or_insert_with(|| new_cur(String::new()));
                fan(&facet, &mut c.indices);
                facet.clear();
                if let Some(c) = cur.take() {
                    finish(c, &mut out);
                }
            }
            b"vertex" => {
                let mut p = [0f32; 3];
                for v in p.iter_mut() {
                    let t = cs.token().with_context(|| {
                        format!(
                            "ASCII STL line {}: unexpected end of file in vertex",
                            cs.line()
                        )
                    })?;
                    *v = std::str::from_utf8(t)
                        .ok()
                        .and_then(|s| s.parse().ok())
                        .with_context(|| {
                            format!(
                                "ASCII STL line {}: invalid number '{}'",
                                cs.line(),
                                String::from_utf8_lossy(t)
                            )
                        })?;
                }
                let c = cur.get_or_insert_with(|| new_cur(String::new()));
                facet.push(c.welder.index(p));
            }
            b"endloop" | b"endfacet" => {
                let c = cur.get_or_insert_with(|| new_cur(String::new()));
                fan(&facet, &mut c.indices);
                facet.clear();
            }
            _ => {}
        }
    }
    if let Some(mut c) = cur.take() {
        fan(&facet, &mut c.indices);
        finish(c, &mut out);
    }
    if out.is_empty() {
        bail!("ASCII STL contains no triangles");
    }
    Ok(out)
}

pub(crate) fn write_binary(meshes: &[TriMesh]) -> Result<Vec<u8>> {
    let total: usize = meshes.iter().map(|m| m.triangle_count()).sum();
    let n = u32::try_from(total).context("too many triangles for binary STL")?;
    let mut out = Vec::with_capacity(84 + 50 * total);
    let mut header = [0u8; 80];
    let text = b"Binary STL exported by Lusion360 (units: mm)";
    header[..text.len()].copy_from_slice(text);
    out.extend_from_slice(&header);
    out.extend_from_slice(&n.to_le_bytes());
    let mut written = 0u32;
    for m in meshes {
        let np = m.positions.len();
        for t in m.indices.as_chunks::<3>().0 {
            if t.iter().any(|&i| i as usize >= np) {
                continue;
            }
            let (a, b, c) = (
                m.positions[t[0] as usize],
                m.positions[t[1] as usize],
                m.positions[t[2] as usize],
            );
            for v in face_normal(a, b, c).iter().chain(&a).chain(&b).chain(&c) {
                out.extend_from_slice(&v.to_le_bytes());
            }
            out.extend_from_slice(&[0, 0]);
            written += 1;
        }
    }
    out[80..84].copy_from_slice(&written.to_le_bytes());
    Ok(out)
}
