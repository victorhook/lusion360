//! A plain indexed triangle mesh with the clean-up operations scans need.

use glam::{Mat4, Vec3};
use lusion_meshio::TriMesh;
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub p: Vec<Vec3>,
    pub t: Vec<[u32; 3]>,
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshStats {
    pub triangles: usize,
    pub vertices: usize,
    /// Edges used by one triangle (holes).
    pub boundary_edges: usize,
    /// Edges used by more than two triangles.
    pub nonmanifold_edges: usize,
    pub components: usize,
    /// Signed volume (mm³ once scaled); meaningful when watertight.
    pub volume: f64,
    pub size: Vec3,
}

impl MeshStats {
    pub fn watertight(&self) -> bool {
        self.boundary_edges == 0 && self.nonmanifold_edges == 0 && self.triangles > 0
    }
}

/// Union–find over `n` elements.
pub struct Dsu(Vec<u32>);
impl Dsu {
    pub fn new(n: usize) -> Self {
        Dsu((0..n as u32).collect())
    }
    pub fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let up = self.0[self.0[x as usize] as usize];
            self.0[x as usize] = up;
            x = up;
        }
        x
    }
    pub fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b) as usize] = a.min(b);
        }
    }
}

impl Mesh {
    /// From a loaded mesh; vertices with bit-identical positions are merged (loaders split
    /// vertices along creases for shading).
    pub fn from_trimesh(tm: &TriMesh) -> Mesh {
        let mut map: HashMap<[u32; 3], u32> = HashMap::with_capacity(tm.positions.len());
        let mut p = Vec::with_capacity(tm.positions.len());
        let remap: Vec<u32> = tm
            .positions
            .iter()
            .map(|q| {
                let key = [q[0].to_bits(), q[1].to_bits(), q[2].to_bits()];
                *map.entry(key).or_insert_with(|| {
                    p.push(Vec3::from_array(*q));
                    (p.len() - 1) as u32
                })
            })
            .collect();
        let n = remap.len() as u32;
        let t = tm
            .indices
            .chunks_exact(3)
            .filter(|c| c.iter().all(|&i| i < n))
            .map(|c| [remap[c[0] as usize], remap[c[1] as usize], remap[c[2] as usize]])
            .collect();
        let mut m = Mesh { p, t };
        m.remove_degenerate();
        m
    }

    pub fn from_flat(positions: &[f32], indices: &[u32]) -> Mesh {
        let tm = TriMesh {
            positions: positions.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect(),
            indices: indices.to_vec(),
            ..Default::default()
        };
        Mesh::from_trimesh(&tm)
    }

    pub fn to_trimesh(&self, name: &str) -> TriMesh {
        let mut tm = TriMesh {
            name: name.into(),
            positions: self.p.iter().map(|v| v.to_array()).collect(),
            normals: vec![],
            indices: self.t.iter().flatten().copied().collect(),
            color: None,
        };
        tm.normals = vertex_normals(&tm.positions, &tm.indices);
        tm
    }

    pub fn flat_positions(&self) -> Vec<f32> {
        self.p.iter().flat_map(|v| v.to_array()).collect()
    }
    pub fn flat_indices(&self) -> Vec<u32> {
        self.t.iter().flatten().copied().collect()
    }

    pub fn is_empty(&self) -> bool {
        self.t.is_empty()
    }

    /// Bounds of the referenced vertices.
    pub fn bounds(&self) -> Option<(Vec3, Vec3)> {
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        for t in &self.t {
            for &i in t {
                let v = self.p[i as usize];
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
        (lo.x <= hi.x).then_some((lo, hi))
    }

    pub fn size(&self) -> Vec3 {
        self.bounds().map(|(a, b)| b - a).unwrap_or(Vec3::ZERO)
    }

    pub fn transform(&mut self, m: &Mat4) {
        let flip = m.determinant() < 0.0;
        for v in &mut self.p {
            *v = m.transform_point3(*v);
        }
        if flip {
            for t in &mut self.t {
                t.swap(1, 2);
            }
        }
    }

    pub fn translate(&mut self, d: Vec3) {
        for v in &mut self.p {
            *v += d;
        }
    }

    pub fn scale(&mut self, s: f32) {
        for v in &mut self.p {
            *v *= s;
        }
    }

    pub fn tri_area(&self, t: &[u32; 3]) -> f32 {
        let (a, b, c) = (self.p[t[0] as usize], self.p[t[1] as usize], self.p[t[2] as usize]);
        (b - a).cross(c - a).length() * 0.5
    }

    pub fn area(&self) -> f64 {
        self.t.iter().map(|t| self.tri_area(t) as f64).sum()
    }

    /// Drop triangles with repeated vertices, zero area or duplicates; drop unused vertices.
    /// A pair of back-to-back triangles (same vertices, opposite winding) is a zero-volume
    /// fin: both go, which keeps a closed mesh closed.
    pub fn remove_degenerate(&mut self) {
        let p = &self.p;
        self.t.retain(|t| {
            if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
                return false;
            }
            let (a, b, c) = (p[t[0] as usize], p[t[1] as usize], p[t[2] as usize]);
            a.is_finite() && b.is_finite() && c.is_finite() && (b - a).cross(c - a).length_squared() > 0.0
        });
        // canonical key: sorted vertices + winding parity
        let key = |t: &[u32; 3]| {
            let mut k = *t;
            k.sort_unstable();
            // even permutation of the sorted order ⇔ same winding
            let r = t.iter().position(|&v| v == k[0]).unwrap();
            let even = t[(r + 1) % 3] == k[1];
            (k, even)
        };
        let mut count: HashMap<[u32; 3], (u32, u32)> = HashMap::with_capacity(self.t.len());
        for t in &self.t {
            let (k, even) = key(t);
            let e = count.entry(k).or_insert((0, 0));
            if even {
                e.0 += 1;
            } else {
                e.1 += 1;
            }
        }
        let mut emitted: HashMap<[u32; 3], (u32, u32)> = HashMap::new();
        self.t.retain(|t| {
            let (k, even) = key(t);
            let (ne, no) = count[&k];
            if ne + no == 1 {
                return true;
            }
            // opposite-winding copies cancel pairwise; keep at most one of the remainder
            let keep_even = ne.saturating_sub(no).min(1);
            let keep_odd = no.saturating_sub(ne).min(1);
            let em = emitted.entry(k).or_insert((0, 0));
            if even && em.0 < keep_even {
                em.0 += 1;
                true
            } else if !even && em.1 < keep_odd {
                em.1 += 1;
                true
            } else {
                false
            }
        });
        self.compact();
    }

    /// Remove vertices no triangle uses.
    pub fn compact(&mut self) {
        let mut remap = vec![u32::MAX; self.p.len()];
        let mut p = Vec::with_capacity(self.p.len());
        for t in &mut self.t {
            for i in t.iter_mut() {
                let r = &mut remap[*i as usize];
                if *r == u32::MAX {
                    p.push(self.p[*i as usize]);
                    *r = (p.len() - 1) as u32;
                }
                *i = *r;
            }
        }
        self.p = p;
    }

    /// Connected component per triangle (vertex-connected), and the component count.
    pub fn components(&self) -> (Vec<u32>, usize) {
        let mut d = Dsu::new(self.p.len());
        for t in &self.t {
            d.union(t[0], t[1]);
            d.union(t[1], t[2]);
        }
        let mut ids: HashMap<u32, u32> = HashMap::new();
        let labels = self
            .t
            .iter()
            .map(|t| {
                let r = d.find(t[0]);
                let n = ids.len() as u32;
                *ids.entry(r).or_insert(n)
            })
            .collect();
        (labels, ids.len())
    }

    /// Keep the largest component (by area) and any others at least `min_frac` of its size.
    pub fn keep_largest(&mut self, min_frac: f32) {
        let (labels, n) = self.components();
        if n <= 1 {
            return;
        }
        let mut area = vec![0f64; n];
        for (t, &l) in self.t.iter().zip(&labels) {
            area[l as usize] += self.tri_area(t) as f64;
        }
        let best = area.iter().cloned().fold(0.0, f64::max);
        let keep: Vec<bool> = area.iter().map(|&a| a >= best * min_frac as f64 && a > 0.0).collect();
        let mut i = 0;
        self.t.retain(|_| {
            let k = keep[labels[i] as usize];
            i += 1;
            k
        });
        self.compact();
    }

    /// Keep only the part with `n·x >= d` (triangles crossing the plane are split exactly).
    pub fn clip_halfspace(&mut self, n: Vec3, d: f32) {
        let side: Vec<f32> = self.p.iter().map(|v| n.dot(*v) - d).collect();
        let mut cache: HashMap<(u32, u32), u32> = HashMap::new();
        let mut out = Vec::with_capacity(self.t.len());
        let tris = std::mem::take(&mut self.t);
        for t in tris {
            let inside = t.map(|i| side[i as usize] >= 0.0);
            match inside.iter().filter(|&&b| b).count() {
                3 => out.push(t),
                0 => {}
                _ => {
                    // Sutherland–Hodgman on one triangle
                    let mut poly: Vec<u32> = Vec::with_capacity(4);
                    for k in 0..3 {
                        let (a, b) = (t[k], t[(k + 1) % 3]);
                        let (ia, ib) = (inside[k], inside[(k + 1) % 3]);
                        if ia {
                            poly.push(a);
                        }
                        if ia != ib {
                            let key = (a.min(b), a.max(b));
                            let v = *cache.entry(key).or_insert_with(|| {
                                let (sa, sb) = (side[key.0 as usize], side[key.1 as usize]);
                                let s = sa / (sa - sb);
                                let pa = self.p[key.0 as usize];
                                let pb = self.p[key.1 as usize];
                                self.p.push(pa + (pb - pa) * s);
                                (self.p.len() - 1) as u32
                            });
                            poly.push(v);
                        }
                    }
                    for k in 1..poly.len().saturating_sub(1) {
                        out.push([poly[0], poly[k], poly[k + 1]]);
                    }
                }
            }
        }
        self.t = out;
        self.remove_degenerate();
    }

    /// Drop triangles whose centroid is further than `r` from the vertical axis through `c`.
    pub fn crop_cylinder(&mut self, c: Vec3, r: f32) {
        let p = &self.p;
        self.t.retain(|t| {
            let m = (p[t[0] as usize] + p[t[1] as usize] + p[t[2] as usize]) / 3.0;
            (m.truncate() - c.truncate()).length() <= r
        });
        self.compact();
    }

    /// Edge statistics, component count, volume and size.
    pub fn stats(&self) -> MeshStats {
        let mut edges: Vec<u64> = Vec::with_capacity(self.t.len() * 3);
        for t in &self.t {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                edges.push(((a.min(b) as u64) << 32) | a.max(b) as u64);
            }
        }
        edges.sort_unstable();
        let (mut boundary, mut nonmanifold) = (0, 0);
        let mut i = 0;
        while i < edges.len() {
            let mut j = i + 1;
            while j < edges.len() && edges[j] == edges[i] {
                j += 1;
            }
            match j - i {
                1 => boundary += 1,
                2 => {}
                _ => nonmanifold += 1,
            }
            i = j;
        }
        let mut vol = 0.0f64;
        for t in &self.t {
            let (a, b, c) = (self.p[t[0] as usize].as_dvec3(), self.p[t[1] as usize].as_dvec3(), self.p[t[2] as usize].as_dvec3());
            vol += a.dot(b.cross(c)) / 6.0;
        }
        MeshStats {
            triangles: self.t.len(),
            vertices: self.p.len(),
            boundary_edges: boundary,
            nonmanifold_edges: nonmanifold,
            components: self.components().1,
            volume: vol,
            size: self.size(),
        }
    }

    /// Vertex adjacency (CSR).
    pub(crate) fn neighbours(&self) -> (Vec<u32>, Vec<u32>) {
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(self.t.len() * 6);
        for t in &self.t {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                pairs.push((a, b));
                pairs.push((b, a));
            }
        }
        pairs.sort_unstable();
        pairs.dedup();
        let mut start = vec![0u32; self.p.len() + 1];
        for &(a, _) in &pairs {
            start[a as usize + 1] += 1;
        }
        for i in 0..self.p.len() {
            start[i + 1] += start[i];
        }
        (start, pairs.into_iter().map(|(_, b)| b).collect())
    }

    /// Taubin λ|μ smoothing (no shrinking). Vertices with `z <= pin_z` keep their height,
    /// so a flat printing base stays flat.
    pub fn taubin(&mut self, iterations: u32, pin_z: Option<f32>) {
        if iterations == 0 || self.t.is_empty() {
            return;
        }
        let (start, nb) = self.neighbours();
        let pinned: Vec<bool> = self.p.iter().map(|v| pin_z.map(|z| v.z <= z + 1e-4).unwrap_or(false)).collect();
        let mut tmp = self.p.clone();
        for _ in 0..iterations {
            for &f in &[0.5f32, -0.53] {
                for i in 0..self.p.len() {
                    let (s, e) = (start[i] as usize, start[i + 1] as usize);
                    if s == e {
                        tmp[i] = self.p[i];
                        continue;
                    }
                    let mut avg = Vec3::ZERO;
                    for &j in &nb[s..e] {
                        avg += self.p[j as usize];
                    }
                    avg /= (e - s) as f32;
                    let mut v = self.p[i] + (avg - self.p[i]) * f;
                    if pinned[i] {
                        v.z = self.p[i].z;
                    }
                    tmp[i] = v;
                }
                std::mem::swap(&mut self.p, &mut tmp);
            }
        }
    }

    /// Quadric simplification to at most `target` triangles; keeps closed meshes closed.
    pub fn decimate(&mut self, target: usize) {
        crate::decimate::decimate(self, target);
    }
}

pub fn vertex_normals(pos: &[[f32; 3]], idx: &[u32]) -> Vec<[f32; 3]> {
    let mut n = vec![Vec3::ZERO; pos.len()];
    for t in idx.chunks_exact(3) {
        let (a, b, c) = (Vec3::from_array(pos[t[0] as usize]), Vec3::from_array(pos[t[1] as usize]), Vec3::from_array(pos[t[2] as usize]));
        let f = (b - a).cross(c - a);
        for &i in t {
            n[i as usize] += f;
        }
    }
    n.into_iter().map(|v| v.try_normalize().unwrap_or(Vec3::Z).to_array()).collect()
}

/// Small deterministic RNG (xorshift64*).
pub struct Rng(u64);
impl Rng {
    pub fn new(seed: u64) -> Self {
        Rng(seed.max(1))
    }
    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    pub fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }
}

/// Eigen-decomposition of a symmetric 3×3 matrix (Jacobi). Returns (values, vectors as columns),
/// sorted ascending.
pub fn sym_eigen(m: [[f64; 3]; 3]) -> ([f64; 3], [glam::DVec3; 3]) {
    let mut a = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for _ in 0..50 {
        let (mut p, mut q, mut best) = (0, 1, 0.0);
        for i in 0..3 {
            for j in i + 1..3 {
                if a[i][j].abs() > best {
                    best = a[i][j].abs();
                    p = i;
                    q = j;
                }
            }
        }
        if best < 1e-15 {
            break;
        }
        let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
        let t = if theta == 0.0 { 1.0 } else { t };
        let c = 1.0 / (t * t + 1.0).sqrt();
        let s = t * c;
        for k in 0..3 {
            let (akp, akq) = (a[k][p], a[k][q]);
            a[k][p] = c * akp - s * akq;
            a[k][q] = s * akp + c * akq;
        }
        for k in 0..3 {
            let (apk, aqk) = (a[p][k], a[q][k]);
            a[p][k] = c * apk - s * aqk;
            a[q][k] = s * apk + c * aqk;
        }
        for k in 0..3 {
            let (vkp, vkq) = (v[k][p], v[k][q]);
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let vals = idx.map(|i| a[i][i]);
    let vecs = idx.map(|i| glam::DVec3::new(v[0][i], v[1][i], v[2][i]));
    (vals, vecs)
}

#[cfg(test)]
mod tests {
    use super::*;

    pub fn cube(s: f32) -> Mesh {
        let p = (0..8).map(|i| Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32) * s).collect();
        let t = vec![
            [0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4],
            [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5],
        ];
        Mesh { p, t }
    }

    #[test]
    fn cube_is_watertight() {
        let s = cube(2.0).stats();
        assert!(s.watertight(), "{s:?}");
        assert!((s.volume - 8.0).abs() < 1e-5, "{}", s.volume);
    }

    #[test]
    fn clip_splits() {
        let mut m = cube(2.0);
        m.clip_halfspace(Vec3::Z, 0.5);
        let s = m.stats();
        assert!((s.size.z - 1.5).abs() < 1e-5);
        assert!(s.boundary_edges > 0); // open at the cut
    }

    #[test]
    fn eigen() {
        let (vals, vecs) = sym_eigen([[2.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 1.0]]);
        assert_eq!(vals, [1.0, 2.0, 5.0]);
        assert!((vecs[0].z.abs() - 1.0).abs() < 1e-9);
    }
}
