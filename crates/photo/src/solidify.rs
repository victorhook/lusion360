//! Rebuild a scan as a closed, manifold solid (what a slicer needs).
//!
//! 1. Sample the unsigned distance to the input surface on a grid (narrow band).
//! 2. "Close" the surface: grid points within `close` of it block a flood fill that starts
//!    at the grid boundary; the plane z = 0 is a wall too, so an open bottom (the side that
//!    stood on the table) gets a flat base.
//! 3. Whatever the flood cannot reach is inside. Shrinking the outside region back by
//!    `close` (morphological closing) keeps the original size while holes up to ~2·close
//!    stay sealed.
//! 4. Signed field = ±distance-to-surface near the scan, distance-to-closing elsewhere;
//!    marching tetrahedra on it. Tetrahedra (unlike marching cubes) have no ambiguous
//!    cases, so the output is always a closed 2-manifold.

use crate::mesh::Mesh;
use glam::Vec3;
use rayon::prelude::*;
use std::collections::HashMap;
use std::sync::atomic::{AtomicU32, Ordering};

pub struct SolidifyResult {
    pub mesh: Mesh,
    /// Cell size actually used (grown if the requested one needed too much memory).
    pub cell: f32,
    pub dims: [usize; 3],
}

struct Grid {
    o: Vec3,
    h: f32,
    n: [usize; 3],
}

impl Grid {
    #[inline]
    fn idx(&self, i: usize, j: usize, k: usize) -> usize {
        i + self.n[0] * (j + self.n[1] * k)
    }
    #[inline]
    fn pos(&self, i: usize, j: usize, k: usize) -> Vec3 {
        self.o + Vec3::new(i as f32, j as f32, k as f32) * self.h
    }
    fn len(&self) -> usize {
        self.n[0] * self.n[1] * self.n[2]
    }
}

/// Exact distance from `p` to triangle `abc` (Ericson, Real-Time Collision Detection 5.1.5).
pub fn point_tri_dist(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return ap.length();
    }
    let bp = p - b;
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return bp.length();
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (p - (a + ab * v)).length();
    }
    let cp = p - c;
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return cp.length();
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (p - (a + ac * w)).length();
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (p - (b + (c - b) * w)).length();
    }
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (p - (a + ab * v + ac * w)).length()
}

/// 1-D squared Euclidean distance transform (Felzenszwalb & Huttenlocher) of `f` in place.
fn edt_1d(f: &mut [f32], v: &mut [usize], z: &mut [f32], d: &mut [f32]) {
    let n = f.len();
    if n == 0 {
        return;
    }
    let inf = f32::INFINITY;
    let mut k = 0usize;
    v[0] = 0;
    z[0] = -inf;
    z[1] = inf;
    for q in 1..n {
        if f[q] == inf {
            continue;
        }
        loop {
            let p = v[k];
            if f[p] == inf {
                // first finite sample replaces an infinite one
                v[k] = q;
                z[k] = -inf;
                z[k + 1] = inf;
                break;
            }
            let s = ((f[q] + (q * q) as f32) - (f[p] + (p * p) as f32)) / (2.0 * (q as f32 - p as f32));
            if s <= z[k] {
                if k == 0 {
                    v[0] = q;
                    z[0] = -inf;
                    z[1] = inf;
                    break;
                }
                k -= 1;
                continue;
            }
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = inf;
            break;
        }
    }
    if f[v[0]] == inf {
        return; // all infinite
    }
    k = 0;
    for q in 0..n {
        while z[k + 1] < q as f32 {
            k += 1;
        }
        let p = v[k];
        let dq = q as f32 - p as f32;
        d[q] = dq * dq + f[p];
    }
    f.copy_from_slice(&d[..n]);
}

/// Squared distance (in cells²) to the nearest `seed` point, for the whole grid.
fn edt(g: &Grid, seed: impl Fn(usize) -> bool + Sync) -> Vec<f32> {
    let [nx, ny, nz] = g.n;
    let mut f: Vec<f32> = (0..g.len()).into_par_iter().map(|i| if seed(i) { 0.0 } else { f32::INFINITY }).collect();
    let maxn = nx.max(ny).max(nz);
    // x lines are contiguous
    f.par_chunks_mut(nx).for_each(|line| {
        let (mut v, mut z, mut d) = (vec![0usize; maxn], vec![0f32; maxn + 1], vec![0f32; maxn]);
        edt_1d(line, &mut v, &mut z, &mut d);
    });
    // y lines: within each z slab
    f.par_chunks_mut(nx * ny).for_each(|slab| {
        let (mut v, mut z, mut d) = (vec![0usize; maxn], vec![0f32; maxn + 1], vec![0f32; maxn]);
        let mut line = vec![0f32; ny];
        for i in 0..nx {
            for j in 0..ny {
                line[j] = slab[i + nx * j];
            }
            edt_1d(&mut line, &mut v, &mut z, &mut d);
            for j in 0..ny {
                slab[i + nx * j] = line[j];
            }
        }
    });
    // z lines: gather columns per (i, j) row block
    let plane = nx * ny;
    let cols: Vec<Vec<f32>> = (0..plane)
        .into_par_iter()
        .map(|c| {
            let (mut v, mut z, mut d) = (vec![0usize; maxn], vec![0f32; maxn + 1], vec![0f32; maxn]);
            let mut line: Vec<f32> = (0..nz).map(|k| f[c + plane * k]).collect();
            edt_1d(&mut line, &mut v, &mut z, &mut d);
            line
        })
        .collect();
    for (c, line) in cols.into_iter().enumerate() {
        for (k, val) in line.into_iter().enumerate() {
            f[c + plane * k] = val;
        }
    }
    f
}

/// Rebuild `m` as a closed solid. `cell` is the grid spacing (mm), `close` the hole-closing
/// radius (mm). The plane z = 0 acts as a floor; nothing below it survives.
/// `max_points` bounds memory (≈ 13 bytes per grid point).
pub fn solidify(m: &Mesh, cell: f32, close: f32, max_points: usize, progress: &dyn Fn(&str)) -> Option<SolidifyResult> {
    let (lo, hi) = m.bounds()?;
    let mut h = cell.max(1e-4);
    let close = close.max(0.0);
    let grid = loop {
        let pad = close + 3.0 * h;
        let o = Vec3::new(lo.x - pad, lo.y - pad, lo.z.min(0.0) - 2.0 * h);
        let top = hi + Vec3::splat(pad);
        let n = [((top.x - o.x) / h).ceil() as usize + 1, ((top.y - o.y) / h).ceil() as usize + 1, ((top.z - o.z) / h).ceil() as usize + 1];
        let total = n[0] * n[1] * n[2];
        if total <= max_points {
            // put z = 0 exactly on a grid plane
            let o = Vec3::new(o.x, o.y, -(((-o.z) / h).round()) * h);
            break Grid { o, h, n };
        }
        h *= (total as f32 / max_points as f32).cbrt() * 1.01;
    };
    let [nx, ny, nz] = grid.n;
    progress(&format!("solid grid {nx}×{ny}×{nz} at {:.2} mm", h));

    // 1. narrow-band unsigned distance
    let band = 1.5 * h;
    let udf: Vec<AtomicU32> = (0..grid.len()).map(|_| AtomicU32::new(f32::INFINITY.to_bits())).collect();
    m.t.par_iter().for_each(|t| {
        let (a, b, c) = (m.p[t[0] as usize], m.p[t[1] as usize], m.p[t[2] as usize]);
        let tlo = a.min(b).min(c) - Vec3::splat(band);
        let thi = a.max(b).max(c) + Vec3::splat(band);
        let i0 = (((tlo - grid.o) / h).floor().max(Vec3::ZERO)).as_uvec3();
        let i1 = ((thi - grid.o) / h).ceil().as_uvec3();
        for k in i0.z as usize..=(i1.z as usize).min(nz - 1) {
            for j in i0.y as usize..=(i1.y as usize).min(ny - 1) {
                for i in i0.x as usize..=(i1.x as usize).min(nx - 1) {
                    let d = point_tri_dist(grid.pos(i, j, k), a, b, c);
                    if d < band {
                        // non-negative floats order like their bit patterns
                        udf[grid.idx(i, j, k)].fetch_min(d.to_bits(), Ordering::Relaxed);
                    }
                }
            }
        }
    });
    let udf: Vec<f32> = udf.into_iter().map(|a| f32::from_bits(a.into_inner())).collect();
    let floor_k = ((0.0 - grid.o.z) / h).round() as usize; // grid plane at z = 0

    // 2. blocked = near the surface (dilated by `close`) or at/below the floor
    let rc = close / h; // closing radius in cells
    let near = 0.6f32; // a 6-connected step can't cross the surface without passing within h/2
    let blocked: Vec<bool> = if rc > near {
        let ds = edt(&grid, |i| udf[i] <= near * h);
        (0..grid.len()).into_par_iter().map(|i| ds[i] <= rc * rc || i / (nx * ny) <= floor_k).collect()
    } else {
        (0..grid.len()).into_par_iter().map(|i| udf[i] <= near * h || i / (nx * ny) <= floor_k).collect()
    };

    // 3. flood fill the outside from the grid boundary
    let mut outside = vec![false; grid.len()];
    let mut stack: Vec<u32> = Vec::new();
    let seed = |i: usize, outside: &mut Vec<bool>, stack: &mut Vec<u32>| {
        if !blocked[i] && !outside[i] {
            outside[i] = true;
            stack.push(i as u32);
        }
    };
    for k in 0..nz {
        for j in 0..ny {
            for i in 0..nx {
                if i == 0 || j == 0 || k == 0 || i == nx - 1 || j == ny - 1 || k == nz - 1 {
                    seed(grid.idx(i, j, k), &mut outside, &mut stack);
                }
            }
        }
    }
    while let Some(c) = stack.pop() {
        let c = c as usize;
        let (i, j, k) = (c % nx, (c / nx) % ny, c / (nx * ny));
        if i > 0 {
            seed(c - 1, &mut outside, &mut stack);
        }
        if i + 1 < nx {
            seed(c + 1, &mut outside, &mut stack);
        }
        if j > 0 {
            seed(c - nx, &mut outside, &mut stack);
        }
        if j + 1 < ny {
            seed(c + nx, &mut outside, &mut stack);
        }
        if k > 0 {
            seed(c - nx * ny, &mut outside, &mut stack);
        }
        if k + 1 < nz {
            seed(c + nx * ny, &mut outside, &mut stack);
        }
    }
    drop(blocked);

    // 4. shrink the outside back by the closing radius → solid; build the signed field.
    // The flood stops `near` cells short of the surface too, so erode by rc + near: an
    // outside point at distance δ from the surface is (rc + near − δ/h) cells from the flood.
    // (+0.4: the flood front is quantised to grid points, on average ~half a cell further out.
    // This only places surfaces bridging holes; near the scan the exact sign below decides.)
    let thresh = rc.max(0.0) + near + 0.4;
    let d_o = edt(&grid, |i| outside[i]);
    let coarse_inside = |i: usize| d_o[i].sqrt() > thresh;

    // Exact sign near the surface. The coarse classification above is only accurate to about
    // a cell, and using it right at the surface makes stair-step contour rings. Instead:
    // grid edges the scan crosses are "cut"; points ≥ `band` from the surface keep their
    // coarse label (reliable there), and labels grow into the band along uncut edges,
    // nearest seed first — so they never jump the surface, and at holes a wrong label can
    // only creep a step or two.
    progress("classifying surface");
    let cuts = cut_edges(&grid, m);
    let mut label = vec![0i8; grid.len()]; // +1 outside, −1 inside, 0 unknown (band)
    let mut queue: std::collections::VecDeque<u32> = std::collections::VecDeque::new();
    // seeds: away from the surface and clearly on one side of the closing threshold
    for i in 0..grid.len() {
        if udf[i] >= band && (d_o[i].sqrt() - thresh).abs() >= 1.0 {
            label[i] = if coarse_inside(i) { -1 } else { 1 };
        }
    }
    let plane = nx * ny;
    let neighbours = |c: usize| -> [(Option<usize>, bool); 6] {
        let (i, j, k) = (c % nx, (c / nx) % ny, c / plane);
        // (neighbour, edge cut?)  — edge bits are stored at the lower-index endpoint
        [
            (if i + 1 < nx { Some(c + 1) } else { None }, cuts[c] & 1 != 0),
            (if i > 0 { Some(c - 1) } else { None }, i > 0 && cuts[c - 1] & 1 != 0),
            (if j + 1 < ny { Some(c + nx) } else { None }, cuts[c] & 2 != 0),
            (if j > 0 { Some(c - nx) } else { None }, j > 0 && cuts[c - nx] & 2 != 0),
            (if k + 1 < nz { Some(c + plane) } else { None }, cuts[c] & 4 != 0),
            (if k > 0 { Some(c - plane) } else { None }, k > 0 && cuts[c - plane] & 4 != 0),
        ]
    };
    for i in 0..grid.len() {
        if label[i] != 0 && neighbours(i).iter().any(|&(n, cut)| !cut && n.map(|n| label[n] == 0).unwrap_or(false)) {
            queue.push_back(i as u32);
        }
    }
    while let Some(c) = queue.pop_front() {
        let c = c as usize;
        let l = label[c];
        for (n, cut) in neighbours(c) {
            if let Some(n) = n {
                if !cut && label[n] == 0 {
                    label[n] = l;
                    queue.push_back(n as u32);
                }
            }
        }
    }
    let is_inside = |i: usize| match label[i] {
        0 => coarse_inside(i),
        l => l < 0,
    };
    // The scan is the solid's boundary only where the sides differ across a cut edge.
    // (Inside a sealed crease or hole the scan surface is buried; there the boundary is the
    // closing patch.) Use the exact scan distance at those points and their band neighbours.
    let mut exact = vec![false; grid.len()];
    for i in 0..grid.len() {
        if udf[i] >= band {
            continue;
        }
        let me = is_inside(i);
        if neighbours(i).iter().any(|&(n, cut)| cut && n.map(|n| is_inside(n) != me).unwrap_or(false)) {
            exact[i] = true;
        }
    }
    let seeds: Vec<usize> = (0..grid.len()).filter(|&i| exact[i]).collect();
    for i in seeds {
        for (n, _) in neighbours(i) {
            if let Some(n) = n {
                if udf[n] < band {
                    exact[n] = true;
                }
            }
        }
    }
    drop(cuts);

    let field: Vec<f32> = (0..grid.len())
        .into_par_iter()
        .map(|i| {
            let mag = if exact[i] { udf[i].max(1e-6) } else { ((thresh - d_o[i].sqrt()).abs() * h).clamp(1e-3 * h, 3.0 * h) };
            let f = if is_inside(i) { -mag } else { mag };
            let z = grid.o.z + (i / (nx * ny)) as f32 * h;
            // floor slightly above 0 so base vertices never sit exactly on grid points
            let f = f.max(1e-3 * h - z);
            if f == 0.0 {
                1e-6
            } else {
                f
            }
        })
        .collect();
    drop(d_o);
    drop(outside);
    drop(udf);
    progress("extracting surface");
    let (mut mesh, on_scan) = marching_tets(&grid, &field, &exact);
    // Patches that close holes and gaps come from the grid-quantised closing distance and
    // look blocky; relax them towards a smooth membrane, with the real scan surface pinned.
    relax_fill(&mut mesh, &on_scan, 0.01 * h, 20);
    Some(SolidifyResult { mesh, cell: h, dims: grid.n })
}

/// Bit k of entry `c` is set when the grid edge from point `c` to its +x / +y / +z
/// neighbour (k = 0, 1, 2) crosses a triangle of `m` (inclusive tests: over-cutting is safe).
fn cut_edges(g: &Grid, m: &Mesh) -> Vec<u8> {
    use std::sync::atomic::AtomicU8;
    let [nx, ny, nz] = g.n;
    let h = g.h;
    let cuts: Vec<AtomicU8> = (0..g.len()).map(|_| AtomicU8::new(0)).collect();
    m.t.par_iter().for_each(|t| {
        let (a, b, c) = (m.p[t[0] as usize], m.p[t[1] as usize], m.p[t[2] as usize]);
        let lo = (((a.min(b).min(c) - g.o) / h).floor() - Vec3::ONE).max(Vec3::ZERO).as_uvec3();
        let hi = ((a.max(b).max(c) - g.o) / h).ceil().as_uvec3();
        let e1 = b - a;
        let e2 = c - a;
        for k in lo.z as usize..=(hi.z as usize).min(nz - 1) {
            for j in lo.y as usize..=(hi.y as usize).min(ny - 1) {
                for i in lo.x as usize..=(hi.x as usize).min(nx - 1) {
                    let p = g.pos(i, j, k);
                    let idx = g.idx(i, j, k);
                    for (bit, dir) in [(1u8, Vec3::X), (2, Vec3::Y), (4, Vec3::Z)] {
                        // Möller–Trumbore, segment p → p + h·dir
                        let d = dir * h;
                        let pv = d.cross(e2);
                        let det = e1.dot(pv);
                        if det.abs() < 1e-12 {
                            continue;
                        }
                        let inv = 1.0 / det;
                        let tv = p - a;
                        let u = tv.dot(pv) * inv;
                        if !(-1e-6..=1.0 + 1e-6).contains(&u) {
                            continue;
                        }
                        let qv = tv.cross(e1);
                        let v = d.dot(qv) * inv;
                        if v < -1e-6 || u + v > 1.0 + 1e-6 {
                            continue;
                        }
                        let s = e2.dot(qv) * inv;
                        if (-1e-6..=1.0 + 1e-6).contains(&s) {
                            cuts[idx].fetch_or(bit, Ordering::Relaxed);
                        }
                    }
                }
            }
        }
    });
    cuts.into_iter().map(|a| a.into_inner()).collect()
}

/// Kuhn subdivision of a cube into 6 tetrahedra around the 0–7 diagonal; corner bit
/// order x = 1, y = 2, z = 4. Consistent between neighbouring cells.
const TETS: [[usize; 4]; 6] = [[0, 1, 3, 7], [0, 3, 2, 7], [0, 2, 6, 7], [0, 6, 4, 7], [0, 4, 5, 7], [0, 5, 1, 7]];

/// Returns the mesh and, per vertex, whether it lies on the original scan surface (both
/// ends of its grid edge were near the scan) rather than on a hole-closing patch.
fn marching_tets(g: &Grid, f: &[f32], exact: &[bool]) -> (Mesh, Vec<bool>) {
    let [nx, ny, nz] = g.n;
    // value 0 counts as outside so every crossing is strict
    let inside = |i: usize| f[i] < 0.0;
    // per z-slab: triangles as triples of grid-edge keys
    let slabs: Vec<Vec<[(u32, u32); 3]>> = (0..nz - 1)
        .into_par_iter()
        .map(|k| {
            let mut tris = Vec::new();
            for j in 0..ny - 1 {
                for i in 0..nx - 1 {
                    let c: [usize; 8] = std::array::from_fn(|b| g.idx(i + (b & 1), j + ((b >> 1) & 1), k + ((b >> 2) & 1)));
                    let ins: [bool; 8] = c.map(inside);
                    if ins.iter().all(|&b| b) || ins.iter().all(|&b| !b) {
                        continue;
                    }
                    for tet in TETS {
                        let v = tet.map(|t| c[t]);
                        let inn: Vec<usize> = v.iter().copied().filter(|&x| inside(x)).collect();
                        let out: Vec<usize> = v.iter().copied().filter(|&x| !inside(x)).collect();
                        let e = |a: usize, b: usize| (a.min(b) as u32, a.max(b) as u32);
                        match inn.len() {
                            1 => tris.push(orient(g, f, [e(inn[0], out[0]), e(inn[0], out[1]), e(inn[0], out[2])])),
                            3 => tris.push(orient(g, f, [e(out[0], inn[0]), e(out[0], inn[1]), e(out[0], inn[2])])),
                            2 => {
                                let (a, b, c2, d) = (inn[0], inn[1], out[0], out[1]);
                                let q = [e(a, c2), e(a, d), e(b, d), e(b, c2)];
                                tris.push(orient(g, f, [q[0], q[1], q[2]]));
                                tris.push(orient(g, f, [q[0], q[2], q[3]]));
                            }
                            _ => {}
                        }
                    }
                }
            }
            tris
        })
        .collect();
    let mut map: HashMap<(u32, u32), u32> = HashMap::new();
    let mut m = Mesh::default();
    let mut on_scan = Vec::new();
    for slab in slabs {
        for tri in slab {
            let ids = tri.map(|key| {
                *map.entry(key).or_insert_with(|| {
                    m.p.push(edge_point(g, f, key));
                    on_scan.push(exact[key.0 as usize] && exact[key.1 as usize]);
                    (m.p.len() - 1) as u32
                })
            });
            if ids[0] != ids[1] && ids[1] != ids[2] && ids[0] != ids[2] {
                m.t.push(ids);
            }
        }
    }
    (m, on_scan)
}

/// Laplacian relaxation of the non-scan vertices (scan vertices fixed). Vertices on the
/// floor (z ≤ `floor_z`) stay on it.
fn relax_fill(m: &mut Mesh, on_scan: &[bool], floor_z: f32, iterations: u32) {
    if on_scan.iter().all(|&b| b) {
        return;
    }
    let (start, nb) = m.neighbours();
    let free: Vec<usize> = (0..m.p.len()).filter(|&i| !on_scan[i]).collect();
    let on_floor: Vec<bool> = m.p.iter().map(|p| p.z <= floor_z).collect();
    for _ in 0..iterations {
        let upd: Vec<(usize, Vec3)> = free
            .par_iter()
            .map(|&i| {
                let (s, e) = (start[i] as usize, start[i + 1] as usize);
                if s == e {
                    return (i, m.p[i]);
                }
                let avg = nb[s..e].iter().map(|&j| m.p[j as usize]).sum::<Vec3>() / (e - s) as f32;
                let mut v = m.p[i] + (avg - m.p[i]) * 0.6;
                if on_floor[i] {
                    v.z = m.p[i].z;
                }
                (i, v)
            })
            .collect();
        for (i, v) in upd {
            m.p[i] = v;
        }
    }
}

fn grid_pos(g: &Grid, i: u32) -> Vec3 {
    let i = i as usize;
    let [nx, ny, _] = g.n;
    g.pos(i % nx, (i / nx) % ny, i / (nx * ny))
}

fn edge_point(g: &Grid, f: &[f32], (a, b): (u32, u32)) -> Vec3 {
    let (fa, fb) = (f[a as usize], f[b as usize]);
    let t = (fa / (fa - fb)).clamp(0.0, 1.0);
    let (pa, pb) = (grid_pos(g, a), grid_pos(g, b));
    pa + (pb - pa) * t
}

/// Order the triangle so its normal points from inside (f < 0) to outside.
fn orient(g: &Grid, f: &[f32], t: [(u32, u32); 3]) -> [(u32, u32); 3] {
    let p = t.map(|k| edge_point(g, f, k));
    let n = (p[1] - p[0]).cross(p[2] - p[0]);
    // direction inside → outside along the first edge
    let (a, b) = t[0];
    let (pin, pout) = if f[a as usize] < 0.0 { (grid_pos(g, a), grid_pos(g, b)) } else { (grid_pos(g, b), grid_pos(g, a)) };
    let mut dir = pout - pin;
    // edges of one triangle may all be parallel to the triangle in degenerate spots: use the sum
    for &(a, b) in &t[1..] {
        let (pi, po) = if f[a as usize] < 0.0 { (grid_pos(g, a), grid_pos(g, b)) } else { (grid_pos(g, b), grid_pos(g, a)) };
        dir += po - pi;
    }
    if n.dot(dir) < 0.0 {
        [t[0], t[2], t[1]]
    } else {
        t
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere(r: f32, n: usize) -> Mesh {
        // UV sphere
        let mut m = Mesh::default();
        for i in 0..=n {
            let th = std::f32::consts::PI * i as f32 / n as f32;
            for j in 0..2 * n {
                let ph = std::f32::consts::PI * j as f32 / n as f32;
                m.p.push(Vec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()) * r + Vec3::new(0.0, 0.0, r + 1.0));
            }
        }
        let w = 2 * n as u32;
        for i in 0..n as u32 {
            for j in 0..w {
                let a = i * w + j;
                let b = i * w + (j + 1) % w;
                let c = a + w;
                let d = b + w;
                m.t.push([a, c, b]);
                m.t.push([b, c, d]);
            }
        }
        m.remove_degenerate();
        m
    }

    #[test]
    fn edt_line() {
        let mut f = vec![f32::INFINITY, 0.0, f32::INFINITY, f32::INFINITY, 0.0];
        let n = f.len();
        edt_1d(&mut f, &mut vec![0; n], &mut vec![0.0; n + 1], &mut vec![0.0; n]);
        assert_eq!(f, vec![1.0, 0.0, 1.0, 1.0, 0.0]);
    }

    #[test]
    fn sphere_with_hole_becomes_watertight() {
        let mut m = sphere(10.0, 32);
        // well clear of the floor (closing also bridges narrow gaps to the floor)
        m.translate(Vec3::new(0.0, 0.0, 10.0));
        let v = m.stats().volume; // faceted sphere, slightly below 4/3 π r³
        // punch a ~6 mm hole at the top; closing radius 4 mm seals holes up to ~8 mm
        m.clip_halfspace(-Vec3::Z, -30.5);
        assert!(m.stats().boundary_edges > 0);
        let r = solidify(&m, 0.5, 4.0, 4_000_000, &|_| {}).unwrap();
        let s = r.mesh.stats();
        assert!(s.watertight(), "{s:?}");
        assert_eq!(s.components, 1);
        assert!((s.volume - v).abs() / v < 0.015, "volume {} vs {v}", s.volume);
    }

    #[test]
    fn open_bottom_gets_floor() {
        // hemisphere-ish cup standing on z = 0, open at the bottom
        let mut m = sphere(10.0, 32);
        m.translate(Vec3::new(0.0, 0.0, -11.0)); // centre at origin
        m.clip_halfspace(Vec3::Z, 0.0);
        let r = solidify(&m, 0.5, 0.0, 4_000_000, &|_| {}).unwrap();
        let s = r.mesh.stats();
        assert!(s.watertight(), "{s:?}");
        let v = 2.0 / 3.0 * std::f64::consts::PI * 1000.0;
        assert!((s.volume - v).abs() / v < 0.03, "volume {} vs {v}", s.volume);
        let (lo, _) = r.mesh.bounds().unwrap();
        assert!(lo.z.abs() < 0.05, "base at {}", lo.z);
    }
}

