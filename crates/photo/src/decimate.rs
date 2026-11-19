//! Quadric-error edge-collapse simplification (Garland & Heckbert) that never breaks a
//! closed manifold: every collapse must pass the link condition and must not flip or
//! degenerate any triangle. Slower than meshoptimizer, but a watertight scan stays watertight.

use crate::mesh::Mesh;
use glam::DVec3;
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Symmetric 4×4 quadric, upper triangle.
#[derive(Clone, Copy, Default)]
struct Quadric([f64; 10]);

impl Quadric {
    fn plane(n: DVec3, d: f64, w: f64) -> Self {
        let (a, b, c) = (n.x, n.y, n.z);
        Quadric([a * a * w, a * b * w, a * c * w, a * d * w, b * b * w, b * c * w, b * d * w, c * c * w, c * d * w, d * d * w])
    }
    fn add(&mut self, o: &Quadric) {
        for i in 0..10 {
            self.0[i] += o.0[i];
        }
    }
    fn eval(&self, p: DVec3) -> f64 {
        let q = &self.0;
        let (x, y, z) = (p.x, p.y, p.z);
        q[0] * x * x + 2.0 * q[1] * x * y + 2.0 * q[2] * x * z + 2.0 * q[3] * x + q[4] * y * y + 2.0 * q[5] * y * z + 2.0 * q[6] * y + q[7] * z * z + 2.0 * q[8] * z + q[9]
    }
    /// Point minimising the error, if the system is well conditioned.
    fn optimum(&self) -> Option<DVec3> {
        let q = &self.0;
        let m = glam::DMat3::from_cols(DVec3::new(q[0], q[1], q[2]), DVec3::new(q[1], q[4], q[5]), DVec3::new(q[2], q[5], q[7]));
        let det = m.determinant();
        let scale = (q[0] + q[4] + q[7]).abs().max(1e-30);
        if det.abs() < 1e-9 * scale * scale * scale {
            return None;
        }
        let p = m.inverse() * -DVec3::new(q[3], q[6], q[8]);
        p.is_finite().then_some(p)
    }
}

#[derive(PartialEq)]
struct Cand {
    cost: f64,
    u: u32,
    v: u32,
    stamp: u64,
}
impl Eq for Cand {}
impl PartialOrd for Cand {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Cand {
    // min-heap on cost
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost.total_cmp(&self.cost)
    }
}

/// Simplify to at most `target` triangles (stops early if no legal collapse remains).
pub fn decimate(m: &mut Mesh, target: usize) {
    if m.t.len() <= target {
        return;
    }
    let nv = m.p.len();
    let mut pos: Vec<DVec3> = m.p.iter().map(|p| p.as_dvec3()).collect();
    let mut tris: Vec<[u32; 3]> = m.t.clone();
    let mut alive = vec![true; tris.len()];
    let mut vt: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (i, t) in tris.iter().enumerate() {
        for &v in t {
            vt[v as usize].push(i as u32);
        }
    }
    let mut quad = vec![Quadric::default(); nv];
    for t in &tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let n = (b - a).cross(c - a);
        let area = n.length();
        if area <= 0.0 {
            continue;
        }
        let n = n / area;
        let q = Quadric::plane(n, -n.dot(a), area * 0.5);
        for &v in t {
            quad[v as usize].add(&q);
        }
    }
    let mut version = vec![0u32; nv];
    let mut removed = vec![false; nv];
    let stamp = |version: &[u32], u: u32, v: u32| ((version[u as usize] as u64) << 32) | version[v as usize] as u64;

    let neighbours = |vt: &[Vec<u32>], tris: &[[u32; 3]], alive: &[bool], v: u32, out: &mut Vec<u32>| {
        out.clear();
        for &t in &vt[v as usize] {
            if !alive[t as usize] {
                continue;
            }
            for &w in &tris[t as usize] {
                if w != v && !out.contains(&w) {
                    out.push(w);
                }
            }
        }
    };

    // where to put the merged vertex, and the cost
    let place = |quad: &[Quadric], pos: &[DVec3], u: u32, v: u32| -> (DVec3, f64) {
        let mut q = quad[u as usize];
        q.add(&quad[v as usize]);
        let (pu, pv) = (pos[u as usize], pos[v as usize]);
        let mut best = (pu, q.eval(pu));
        for p in [pv, (pu + pv) * 0.5].into_iter().chain(q.optimum().filter(|p| (*p - (pu + pv) * 0.5).length() <= (pu - pv).length() * 2.0)) {
            let e = q.eval(p);
            if e < best.1 {
                best = (p, e);
            }
        }
        (best.0, best.1.max(0.0))
    };

    let mut heap = BinaryHeap::with_capacity(tris.len() * 2);
    for t in &tris {
        for k in 0..3 {
            let (u, v) = (t[k], t[(k + 1) % 3]);
            if u < v {
                let (_, cost) = place(&quad, &pos, u, v);
                heap.push(Cand { cost, u, v, stamp: stamp(&version, u, v) });
            }
        }
    }

    let mut live = tris.len();
    let (mut nu, mut nvv) = (Vec::new(), Vec::new());
    while live > target {
        let Some(c) = heap.pop() else { break };
        let (u, v) = (c.u, c.v);
        if removed[u as usize] || removed[v as usize] || c.stamp != stamp(&version, u, v) {
            continue;
        }
        // link condition: common neighbours must be exactly the apexes of the shared triangles
        neighbours(&vt, &tris, &alive, u, &mut nu);
        neighbours(&vt, &tris, &alive, v, &mut nvv);
        if !nu.contains(&v) {
            continue;
        }
        let shared: Vec<u32> = vt[u as usize].iter().copied().filter(|&t| alive[t as usize] && tris[t as usize].contains(&v)).collect();
        let common = nu.iter().filter(|w| nvv.contains(w)).count();
        if shared.len() != 2 || common != 2 {
            continue;
        }
        let (p, _) = place(&quad, &pos, u, v);
        // no flipped or degenerate triangles around u and v
        let mut ok = true;
        for &x in [u, v].iter() {
            for &t in &vt[x as usize] {
                if !alive[t as usize] || shared.contains(&t) {
                    continue;
                }
                let tr = tris[t as usize];
                let pts_old = tr.map(|i| pos[i as usize]);
                let pts_new = tr.map(|i| if i == u || i == v { p } else { pos[i as usize] });
                let n0 = (pts_old[1] - pts_old[0]).cross(pts_old[2] - pts_old[0]);
                let n1 = (pts_new[1] - pts_new[0]).cross(pts_new[2] - pts_new[0]);
                let l0 = n0.length();
                let l1 = n1.length();
                if l1 <= l0 * 1e-4 || n0.dot(n1) < 0.2 * l0 * l1 {
                    ok = false;
                    break;
                }
            }
            if !ok {
                break;
            }
        }
        if !ok {
            continue;
        }
        // collapse v into u
        for &t in &shared {
            alive[t as usize] = false;
            live -= 1;
        }
        let vts = std::mem::take(&mut vt[v as usize]);
        for t in vts {
            if !alive[t as usize] {
                continue;
            }
            for i in tris[t as usize].iter_mut() {
                if *i == v {
                    *i = u;
                }
            }
            vt[u as usize].push(t);
        }
        vt[u as usize].retain(|&t| alive[t as usize]);
        removed[v as usize] = true;
        pos[u as usize] = p;
        let qv = quad[v as usize];
        quad[u as usize].add(&qv);
        version[u as usize] += 1;
        neighbours(&vt, &tris, &alive, u, &mut nu);
        for &w in &nu {
            let (a, b) = (u.min(w), u.max(w));
            let (_, cost) = place(&quad, &pos, a, b);
            heap.push(Cand { cost, u: a, v: b, stamp: stamp(&version, a, b) });
        }
    }
    m.p = pos.iter().map(|p| p.as_vec3()).collect();
    m.t = tris.into_iter().zip(alive).filter(|(_, a)| *a).map(|(t, _)| t).collect();
    m.compact();
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::Vec3;

    #[test]
    fn solid_stays_watertight() {
        let mut m = Mesh::default();
        let n = 48;
        for i in 0..=n {
            let th = std::f32::consts::PI * i as f32 / n as f32;
            for j in 0..2 * n {
                let ph = std::f32::consts::PI * j as f32 / n as f32;
                m.p.push(Vec3::new(th.sin() * ph.cos(), th.sin() * ph.sin(), th.cos()) * 10.0 + Vec3::new(0.0, 0.0, 11.0));
            }
        }
        let w = 2 * n as u32;
        for i in 0..n as u32 {
            for j in 0..w {
                let a = i * w + j;
                let b = i * w + (j + 1) % w;
                m.t.push([a, a + w, b]);
                m.t.push([b, a + w, b + w]);
            }
        }
        m.remove_degenerate();
        let mut s = crate::solidify::solidify(&m, 0.2, 1.0, 20_000_000, &|_| {}).unwrap().mesh;
        let before = s.stats();
        assert!(before.watertight());
        let t0 = std::time::Instant::now();
        decimate(&mut s, before.triangles / 20);
        let after = s.stats();
        eprintln!("{} → {} tris in {:.2}s", before.triangles, after.triangles, t0.elapsed().as_secs_f32());
        assert!(after.watertight(), "{after:?}");
        assert!(after.triangles <= before.triangles / 20 + 2);
        assert!((after.volume - before.volume).abs() / before.volume < 0.01, "{} vs {}", after.volume, before.volume);
    }
}
