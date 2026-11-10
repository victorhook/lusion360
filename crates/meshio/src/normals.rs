//! Crease-aware vertex normal computation.
//!
//! Algorithm (O(n log d) where d is the vertex valence, i.e. ~O(n)):
//! 1. weld vertices by exact position,
//! 2. compute unit face normals and per-corner angle weights,
//! 3. build vertex -> corners adjacency (CSR),
//! 4. for each vertex, union incident faces that share an edge through that vertex and whose
//!    normals differ by less than the crease angle (local union-find), then emit one output
//!    vertex per cluster with the angle-weighted average normal.

use crate::hash::Welder;
use crate::{cross, dot, normalize, sanitize_indices, sub, TriMesh};

pub(crate) fn compute_normals(mesh: &mut TriMesh, crease_deg: f32) {
    sanitize_indices(&mut mesh.indices, mesh.positions.len());
    let ntri = mesh.indices.len() / 3;
    if ntri == 0 {
        mesh.normals = vec![[0.0, 0.0, 1.0]; mesh.positions.len()];
        return;
    }

    // 1. Weld by exact position.
    let mut welder = Welder::with_capacity(mesh.positions.len());
    let canon_of: Vec<u32> = mesh.positions.iter().map(|&p| welder.index(p)).collect();
    let pos = welder.positions;
    let nc = pos.len();
    let idx: Vec<u32> = mesh.indices.iter().map(|&i| canon_of[i as usize]).collect();
    drop(canon_of);

    // 2. Face normals and corner angle weights.
    let mut fnorm = Vec::with_capacity(ntri);
    let mut cweight = vec![0f32; 3 * ntri];
    for t in 0..ntri {
        let a = pos[idx[3 * t] as usize];
        let b = pos[idx[3 * t + 1] as usize];
        let c = pos[idx[3 * t + 2] as usize];
        let e0 = sub(b, a);
        let e1 = sub(c, b);
        let e2 = sub(a, c);
        let n = cross(e0, sub(c, a));
        let twice_area = dot(n, n).sqrt();
        fnorm.push(normalize(n));
        if twice_area > 0.0 && twice_area.is_finite() {
            // angle at a corner = atan2(|u x v|, u . v); |u x v| == 2*area for all corners.
            cweight[3 * t] = twice_area.atan2(-dot(e0, e2));
            cweight[3 * t + 1] = twice_area.atan2(-dot(e1, e0));
            cweight[3 * t + 2] = twice_area.atan2(-dot(e2, e1));
        }
    }

    // 3. vertex -> corners (CSR).
    let mut start = vec![0u32; nc + 1];
    for &v in &idx {
        start[v as usize + 1] += 1;
    }
    for i in 0..nc {
        start[i + 1] += start[i];
    }
    let mut fill = start.clone();
    let mut corners = vec![0u32; idx.len()];
    for (corner, &v) in idx.iter().enumerate() {
        let slot = &mut fill[v as usize];
        corners[*slot as usize] = corner as u32;
        *slot += 1;
    }
    drop(fill);

    // 4. Per-vertex clustering.
    let cos_t = if crease_deg >= 180.0 {
        -2.0
    } else {
        crease_deg.to_radians().cos()
    };
    let mut out_pos: Vec<[f32; 3]> = Vec::with_capacity(nc + nc / 4);
    let mut out_nrm: Vec<[f32; 3]> = Vec::with_capacity(nc + nc / 4);
    let mut new_idx = vec![0u32; idx.len()];
    let mut pairs: Vec<u64> = Vec::new();
    let mut parent: Vec<u32> = Vec::new();
    let mut root_out: Vec<u32> = Vec::new();

    fn find(parent: &mut [u32], mut i: u32) -> u32 {
        while parent[i as usize] != i {
            let gp = parent[parent[i as usize] as usize];
            parent[i as usize] = gp;
            i = gp;
        }
        i
    }

    for c in 0..nc {
        let cs = &corners[start[c] as usize..start[c + 1] as usize];
        let d = cs.len();
        if d == 0 {
            continue;
        }
        parent.clear();
        parent.extend(0..d as u32);
        if d > 1 {
            pairs.clear();
            for (i, &corner) in cs.iter().enumerate() {
                let f = (corner / 3) as usize;
                let k = (corner % 3) as usize;
                let w1 = idx[3 * f + (k + 1) % 3];
                let w2 = idx[3 * f + (k + 2) % 3];
                if w1 as usize != c {
                    pairs.push(((w1 as u64) << 32) | i as u64);
                }
                if w2 as usize != c && w2 != w1 {
                    pairs.push(((w2 as u64) << 32) | i as u64);
                }
            }
            pairs.sort_unstable();
            let mut j = 0;
            while j < pairs.len() {
                let w = pairs[j] >> 32;
                let mut k = j + 1;
                while k < pairs.len() && pairs[k] >> 32 == w {
                    k += 1;
                }
                // Exactly two faces share edge (c, w): manifold edge -> maybe smooth.
                if k - j == 2 {
                    let i1 = (pairs[j] & 0xffff_ffff) as u32;
                    let i2 = (pairs[j + 1] & 0xffff_ffff) as u32;
                    let f1 = (cs[i1 as usize] / 3) as usize;
                    let f2 = (cs[i2 as usize] / 3) as usize;
                    if dot(fnorm[f1], fnorm[f2]) >= cos_t {
                        let r1 = find(&mut parent, i1);
                        let r2 = find(&mut parent, i2);
                        if r1 != r2 {
                            parent[r1 as usize] = r2;
                        }
                    }
                }
                j = k;
            }
        }

        root_out.clear();
        root_out.resize(d, u32::MAX);
        let first_out = out_pos.len();
        for (i, &corner) in cs.iter().enumerate() {
            let r = find(&mut parent, i as u32) as usize;
            if root_out[r] == u32::MAX {
                root_out[r] = out_pos.len() as u32;
                out_pos.push(pos[c]);
                out_nrm.push([0.0; 3]);
            }
            let o = root_out[r];
            let n = fnorm[(corner / 3) as usize];
            let w = cweight[corner as usize];
            let acc = &mut out_nrm[o as usize];
            acc[0] += n[0] * w;
            acc[1] += n[1] * w;
            acc[2] += n[2] * w;
            new_idx[corner as usize] = o;
        }
        let mut fallback: Option<[f32; 3]> = None;
        for acc in &mut out_nrm[first_out..] {
            let n = normalize(*acc);
            *acc = if n != [0.0; 3] {
                n
            } else {
                *fallback.get_or_insert_with(|| {
                    let mut s = [0.0f32; 3];
                    for &corner in cs {
                        let n = fnorm[(corner / 3) as usize];
                        s = [s[0] + n[0], s[1] + n[1], s[2] + n[2]];
                    }
                    let s = normalize(s);
                    if s == [0.0; 3] {
                        [0.0, 0.0, 1.0]
                    } else {
                        s
                    }
                })
            };
        }
    }

    mesh.positions = out_pos;
    mesh.normals = out_nrm;
    mesh.indices = new_idx;
}
