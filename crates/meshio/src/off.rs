//! OFF (Object File Format, ascii) reader. Supports the OFF/COFF/NOFF/CNOFF/STOFF header
//! variants by only reading the first three numbers of each vertex line.

use anyhow::{bail, Context, Result};

use crate::{TriMesh, DEFAULT_CREASE_DEG};

pub(crate) fn load(bytes: &[u8]) -> Result<Vec<TriMesh>> {
    let text = String::from_utf8_lossy(bytes);
    // Non-empty, comment-stripped lines with their 1-based line numbers.
    let mut lines = text.lines().enumerate().filter_map(|(i, l)| {
        let l = l.split('#').next().unwrap_or("").trim();
        (!l.is_empty()).then_some((i + 1, l))
    });

    let (_, header) = lines.next().context("OFF: empty file")?;
    let mut toks = header.split_ascii_whitespace();
    let magic = toks.next().unwrap_or("");
    if !magic.ends_with("OFF") {
        bail!("OFF: missing 'OFF' header (found '{magic}')");
    }
    if magic.starts_with('4') || magic.contains("n") {
        bail!("OFF: unsupported variant '{magic}'");
    }
    let mut counts: Vec<&str> = toks.collect();
    let mut counts_line = 1;
    if counts.is_empty() {
        let (ln, l) = lines.next().context("OFF: missing vertex/face counts")?;
        counts = l.split_ascii_whitespace().collect();
        counts_line = ln;
    }
    let parse_count = |i: usize| -> Result<usize> {
        counts
            .get(i)
            .and_then(|s| s.parse().ok())
            .with_context(|| format!("OFF line {counts_line}: invalid vertex/face counts"))
    };
    let nv = parse_count(0)?;
    let nf = parse_count(1)?;

    let mut positions = Vec::with_capacity(nv.min(1 << 24));
    for _ in 0..nv {
        let (ln, l) = lines
            .next()
            .context("OFF: unexpected end of file in vertex list")?;
        let mut it = l.split_ascii_whitespace();
        let mut p = [0f32; 3];
        for c in p.iter_mut() {
            *c = it
                .next()
                .and_then(|t| t.parse().ok())
                .with_context(|| format!("OFF line {ln}: invalid vertex"))?;
        }
        positions.push(p);
    }

    let mut indices = Vec::with_capacity(nf.min(1 << 24) * 3);
    let mut poly: Vec<u32> = Vec::with_capacity(8);
    for _ in 0..nf {
        let (ln, l) = lines
            .next()
            .context("OFF: unexpected end of file in face list")?;
        let mut it = l.split_ascii_whitespace();
        let n: usize = it
            .next()
            .and_then(|t| t.parse().ok())
            .with_context(|| format!("OFF line {ln}: invalid face"))?;
        poly.clear();
        for _ in 0..n {
            let i: u32 = it
                .next()
                .and_then(|t| t.parse().ok())
                .with_context(|| format!("OFF line {ln}: invalid face index"))?;
            if i as usize >= nv {
                bail!("OFF line {ln}: vertex index {i} out of range (have {nv} vertices)");
            }
            poly.push(i);
        }
        crate::fan(&poly, &mut indices);
    }
    if indices.is_empty() {
        bail!("OFF contains no faces");
    }
    let mut mesh = TriMesh {
        positions,
        indices,
        ..Default::default()
    };
    mesh.compute_normals(DEFAULT_CREASE_DEG);
    Ok(vec![mesh])
}
