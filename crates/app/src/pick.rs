//! Picking: ray casts against display meshes (with a BVH) and screen-space
//! tests for edges.

use crate::camera::Ray;
use glam::{Vec2, Vec3};
use lusion_core::tess::BodyMesh;
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Clone, Copy)]
struct Node {
    min: Vec3,
    max: Vec3,
    /// leaf: first tri index into `tris`; inner: left child (right = left+1)
    start: u32,
    count: u32,
}

pub struct Bvh {
    nodes: Vec<Node>,
    tris: Vec<u32>,
}

fn tri(m: &BodyMesh, t: u32) -> [Vec3; 3] {
    let i = 3 * t as usize;
    [
        Vec3::from_array(m.positions[m.indices[i] as usize]),
        Vec3::from_array(m.positions[m.indices[i + 1] as usize]),
        Vec3::from_array(m.positions[m.indices[i + 2] as usize]),
    ]
}

impl Bvh {
    pub fn build(m: &BodyMesh) -> Bvh {
        let n = m.indices.len() / 3;
        let mut tris: Vec<u32> = (0..n as u32).collect();
        let cents: Vec<Vec3> = (0..n as u32).map(|t| {
            let [a, b, c] = tri(m, t);
            (a + b + c) / 3.0
        }).collect();
        let mut nodes = vec![Node { min: Vec3::ZERO, max: Vec3::ZERO, start: 0, count: n as u32 }];
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let (start, count) = (nodes[ni].start as usize, nodes[ni].count as usize);
            let mut lo = Vec3::splat(f32::INFINITY);
            let mut hi = Vec3::splat(f32::NEG_INFINITY);
            for &t in &tris[start..start + count] {
                for p in tri(m, t) {
                    lo = lo.min(p);
                    hi = hi.max(p);
                }
            }
            nodes[ni].min = lo;
            nodes[ni].max = hi;
            if count <= 8 {
                continue;
            }
            let ext = hi - lo;
            let axis = if ext.x >= ext.y && ext.x >= ext.z { 0 } else if ext.y >= ext.z { 1 } else { 2 };
            let slice = &mut tris[start..start + count];
            let mid = count / 2;
            slice.select_nth_unstable_by(mid, |a, b| {
                cents[*a as usize][axis].total_cmp(&cents[*b as usize][axis])
            });
            let left = nodes.len();
            nodes.push(Node { min: lo, max: hi, start: start as u32, count: mid as u32 });
            nodes.push(Node { min: lo, max: hi, start: (start + mid) as u32, count: (count - mid) as u32 });
            nodes[ni].start = left as u32;
            nodes[ni].count = 0;
            stack.push(left);
            stack.push(left + 1);
        }
        Bvh { nodes, tris }
    }

    /// Closest hit: (t, triangle index).
    pub fn raycast(&self, m: &BodyMesh, ray: &Ray) -> Option<(f32, u32)> {
        if self.nodes.is_empty() || m.indices.is_empty() {
            return None;
        }
        let inv = Vec3::ONE / ray.d;
        let mut best: Option<(f32, u32)> = None;
        let mut stack = vec![0usize];
        while let Some(ni) = stack.pop() {
            let nd = &self.nodes[ni];
            let tmax = best.map(|b| b.0).unwrap_or(f32::INFINITY);
            if !slab(ray.o, inv, nd.min, nd.max, tmax) {
                continue;
            }
            if nd.count > 0 {
                for &t in &self.tris[nd.start as usize..(nd.start + nd.count) as usize] {
                    let [a, b, c] = tri(m, t);
                    if let Some(d) = ray_tri(ray.o, ray.d, a, b, c) {
                        if best.map(|b| d < b.0).unwrap_or(true) {
                            best = Some((d, t));
                        }
                    }
                }
            } else {
                stack.push(nd.start as usize);
                stack.push(nd.start as usize + 1);
            }
        }
        best
    }
}

fn slab(o: Vec3, inv: Vec3, lo: Vec3, hi: Vec3, tmax: f32) -> bool {
    let t1 = (lo - o) * inv;
    let t2 = (hi - o) * inv;
    let tmin_v = t1.min(t2);
    let tmax_v = t1.max(t2);
    let tn = tmin_v.x.max(tmin_v.y).max(tmin_v.z);
    let tf = tmax_v.x.min(tmax_v.y).min(tmax_v.z);
    tf >= tn.max(-1e30) && tn <= tmax && tf >= -1e-4 * tmax.min(1e30).max(1.0)
}

pub fn ray_tri(o: Vec3, d: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    let e1 = b - a;
    let e2 = c - a;
    let p = d.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-12 {
        return None;
    }
    let inv = 1.0 / det;
    let s = o - a;
    let u = s.dot(p) * inv;
    if !(-1e-6..=1.0 + 1e-6).contains(&u) {
        return None;
    }
    let q = s.cross(e1);
    let v = d.dot(q) * inv;
    if v < -1e-6 || u + v > 1.0 + 1e-6 {
        return None;
    }
    let t = e2.dot(q) * inv;
    (t > 0.0).then_some(t)
}

/// BVH cache keyed by mesh identity.
#[derive(Default)]
pub struct PickCache {
    bvh: HashMap<usize, (Arc<Bvh>, u64)>,
    frame: u64,
}

impl PickCache {
    pub fn bvh(&mut self, m: &Arc<BodyMesh>) -> Arc<Bvh> {
        let key = Arc::as_ptr(m) as usize;
        let f = self.frame;
        if let Some((b, last)) = self.bvh.get_mut(&key) {
            *last = f;
            return b.clone();
        }
        let b = Arc::new(Bvh::build(m));
        self.bvh.insert(key, (b.clone(), f));
        b
    }
    pub fn tick(&mut self) {
        self.frame += 1;
        let f = self.frame;
        if self.bvh.len() > 64 {
            self.bvh.retain(|_, (_, l)| f - *l < 600);
        }
    }
}

/// Distance from point `p` to segment `a`-`b` in 2D, plus the parameter.
pub fn seg_dist(p: Vec2, a: Vec2, b: Vec2) -> (f32, f32) {
    let d = b - a;
    let l2 = d.length_squared();
    let t = if l2 < 1e-12 { 0.0 } else { ((p - a).dot(d) / l2).clamp(0.0, 1.0) };
    ((p - (a + d * t)).length(), t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::Ray;

    #[test]
    fn bvh_hits_bracket_top() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/bracket.l360");
        let Ok(doc) = lusion_core::io::load_native(std::path::Path::new(path)) else { return };
        let mut m = lusion_core::model::Model::new();
        m.update(&doc);
        let b = m.current(&doc).bodies.values().next().unwrap();
        let mesh = BodyMesh::from_shape(&b.shape).unwrap();
        let bvh = Bvh::build(&mesh);
        let mut misses = 0;
        let mut brute_misses = 0;
        for i in 0..20 {
            for j in 0..10 {
                let x = -35.0 + i as f32 * 3.5;
                let y = -22.0 + j as f32 * 4.5;
                let o = Vec3::new(120.0, -150.0, 110.0);
                let ray = Ray { o, d: (Vec3::new(x, y, 6.0) - o).normalize() };
                let hit = bvh.raycast(&mesh, &ray);
                // brute force
                let mut best: Option<f32> = None;
                for t in 0..mesh.indices.len() / 3 {
                    let [a, b2, c] = tri(&mesh, t as u32);
                    if let Some(d) = ray_tri(ray.o, ray.d, a, b2, c) {
                        best = Some(best.map_or(d, |x: f32| x.min(d)));
                    }
                }
                if best.is_none() {
                    brute_misses += 1;
                }
                if hit.map(|h| h.0) != best {
                    misses += 1;
                }
            }
        }
        assert!(brute_misses < 10, "brute force misses {brute_misses}");
        assert_eq!(misses, 0, "bvh disagrees with brute force");
    }
}
