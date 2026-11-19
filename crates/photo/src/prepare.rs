//! Raw reconstruction → printable part.
//!
//! The reconstruction is in an arbitrary frame and scale and contains the table and the
//! room around the object. [`Alignment::auto`] finds the table (the large plane the cameras
//! all look down on) and the point the cameras orbit; [`prepare`] then stands the object on
//! z = 0, cuts the table away, keeps the object, scales it to a real size and rebuilds it
//! as a closed solid.

use crate::mesh::{sym_eigen, Mesh, MeshStats, Rng};
use crate::pipeline::CameraPose;
use crate::solidify;
use anyhow::{bail, Result};
use glam::{DMat3, DVec3, Mat4, Quat, Vec2, Vec3};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AlignSource {
    /// Found the table plane.
    Ground,
    /// No table; "up" estimated from where the cameras were.
    Cameras,
    /// Nothing to go on: the file's own axes.
    None,
}

/// How to bring the raw scan into a Z-up frame.
#[derive(Clone, Debug)]
pub struct Alignment {
    /// Scan → aligned rotation (up = +Z).
    pub rot: Quat,
    /// Height of the table plane in the rotated frame (scan units), if one was found.
    pub floor: Option<f32>,
    /// Point the cameras orbit, in the rotated frame.
    pub center: Option<Vec3>,
    /// Median horizontal distance of the cameras from `center` (scan units).
    pub ring_radius: Option<f32>,
    /// Roughness of the table surface (scan units); the table is cut this far above its plane.
    pub noise: f32,
    pub source: AlignSource,
}

impl Alignment {
    pub fn identity(m: &Mesh) -> Self {
        let diag = m.size().length().max(1e-6);
        Alignment { rot: Quat::IDENTITY, floor: None, center: None, ring_radius: None, noise: diag * 0.004, source: AlignSource::None }
    }

    pub fn describe(&self) -> String {
        match self.source {
            AlignSource::Ground => "stood on the detected table".into(),
            AlignSource::Cameras => "up direction estimated from camera positions (no table found)".into(),
            AlignSource::None => "file axes (no orientation found)".into(),
        }
    }

    /// Apply a user correction (e.g. a 90° tip). The table plane is no longer valid.
    pub fn rotated(&self, extra: Quat) -> Self {
        let mut a = self.clone();
        a.rot = (extra * self.rot).normalize();
        a.floor = None;
        a.center = self.center.map(|c| extra * c);
        if a.source == AlignSource::Ground {
            a.source = AlignSource::Cameras;
        }
        a
    }

    /// Detect the table and the orbit centre.
    pub fn auto(m: &Mesh, cams: &[CameraPose]) -> Self {
        let mut a = Alignment::identity(m);
        if m.p.is_empty() {
            return a;
        }
        let diag = m.size().length().max(1e-6);
        let thr = diag * 0.004;
        a.noise = thr;
        let mut rng = Rng::new(0x5eed);
        let n_samples = m.p.len().min(60_000);
        let mut pts: Vec<Vec3> = (0..n_samples).map(|_| m.p[rng.below(m.p.len())]).collect();
        let total = pts.len();

        // camera prior: where the cameras look at and which way is "towards the cameras"
        let target = orbit_center(cams);

        // sequential RANSAC: a few dominant planes, the table is the one with every camera above it
        let mut best: Option<(Vec3, f32, usize, f32)> = None;
        for _round in 0..4 {
            if pts.len() < 100 {
                break;
            }
            let Some((n, d, inl)) = ransac_plane(&pts, thr, 400, &mut rng) else { break };
            let count = inl.len();
            if count < total / 20 {
                break;
            }
            let (n, d, sigma) = refine_plane(&pts, &inl).unwrap_or((n, d, thr / 3.0));
            // orient towards the cameras (or towards the bulk of the other points)
            let (n, d, frac) = if !cams.is_empty() {
                let above = cams.iter().filter(|c| n.dot(c.center) + d > 0.0).count();
                let frac = above as f32 / cams.len() as f32;
                if frac >= 0.5 {
                    (n, d, frac)
                } else {
                    (-n, -d, 1.0 - frac)
                }
            } else {
                let mask: std::collections::HashSet<usize> = inl.iter().copied().collect();
                let above = pts.iter().enumerate().filter(|(i, p)| !mask.contains(i) && n.dot(**p) + d > thr).count();
                let below = pts.iter().enumerate().filter(|(i, p)| !mask.contains(i) && n.dot(**p) + d < -thr).count();
                if above >= below {
                    (n, d, 1.0)
                } else {
                    (-n, -d, 1.0)
                }
            };
            // the object must stand on it: the orbit target is above, close to the plane
            let target_ok = target.map(|t| n.dot(t) + d > -thr * 2.0).unwrap_or(true);
            if frac >= 0.9 && target_ok && best.map(|b| count > b.2).unwrap_or(true) {
                best = Some((n, d, count, sigma));
            }
            let set: std::collections::HashSet<usize> = inl.into_iter().collect();
            let mut k = 0;
            pts.retain(|_| {
                let keep = !set.contains(&k);
                k += 1;
                keep
            });
        }

        let up = if let Some((n, d, _, sigma)) = best {
            a.source = AlignSource::Ground;
            // cut just above the table's own roughness
            a.noise = (3.0 * sigma).max(diag * 1e-4);
            a.rot = Quat::from_rotation_arc(n, Vec3::Z);
            // plane point -n·d rotates onto z = -d
            a.floor = Some(-d);
            n
        } else if !cams.is_empty() {
            a.source = AlignSource::Cameras;
            let mean_up = cams.iter().map(|c| c.up).sum::<Vec3>() / cams.len() as f32;
            let centroid = m.p.iter().copied().sum::<Vec3>() / m.p.len() as f32;
            let mean_c = cams.iter().map(|c| c.center).sum::<Vec3>() / cams.len() as f32;
            let up = if mean_up.length() > 0.7 { mean_up.normalize() } else { (mean_c - target.unwrap_or(centroid)).normalize_or(Vec3::Z) };
            a.rot = Quat::from_rotation_arc(up, Vec3::Z);
            up
        } else {
            Vec3::Z
        };
        let _ = up;
        if let Some(t) = target {
            let c = a.rot * t;
            a.center = Some(c);
            let mut dists: Vec<f32> = cams.iter().map(|cam| ((a.rot * cam.center).truncate() - c.truncate()).length()).collect();
            dists.sort_by(f32::total_cmp);
            a.ring_radius = dists.get(dists.len() / 2).copied();
        }
        // yaw: longest horizontal extent of the object along X (looks tidy on the bed)
        a
    }

    pub fn matrix(&self) -> Mat4 {
        Mat4::from_quat(self.rot)
    }
}

/// Least-squares point closest to all camera viewing rays.
fn orbit_center(cams: &[CameraPose]) -> Option<Vec3> {
    if cams.len() < 3 {
        return None;
    }
    let mut a = DMat3::ZERO;
    let mut b = DVec3::ZERO;
    for c in cams {
        let d = c.forward.as_dvec3().normalize();
        let m = DMat3::IDENTITY - DMat3::from_cols(d * d.x, d * d.y, d * d.z);
        a += m;
        b += m * c.center.as_dvec3();
    }
    if a.determinant().abs() < 1e-9 {
        return None;
    }
    let x = a.inverse() * b;
    x.is_finite().then(|| x.as_vec3())
}

fn ransac_plane(pts: &[Vec3], thr: f32, iters: usize, rng: &mut Rng) -> Option<(Vec3, f32, Vec<usize>)> {
    let mut best: Option<(Vec3, f32, usize)> = None;
    for _ in 0..iters {
        let (a, b, c) = (pts[rng.below(pts.len())], pts[rng.below(pts.len())], pts[rng.below(pts.len())]);
        let Some(n) = (b - a).cross(c - a).try_normalize() else { continue };
        let d = -n.dot(a);
        // score on a subset for speed
        let step = (pts.len() / 5000).max(1);
        let count = pts.iter().step_by(step).filter(|p| (n.dot(**p) + d).abs() < thr).count();
        if best.map(|b| count > b.2).unwrap_or(true) {
            best = Some((n, d, count));
        }
    }
    let (n, d, _) = best?;
    let inl: Vec<usize> = (0..pts.len()).filter(|&i| (n.dot(pts[i]) + d).abs() < thr).collect();
    Some((n, d, inl))
}

/// Total-least-squares plane through the inliers, with the residual standard deviation.
fn refine_plane(pts: &[Vec3], inl: &[usize]) -> Option<(Vec3, f32, f32)> {
    if inl.len() < 3 {
        return None;
    }
    let c = inl.iter().map(|&i| pts[i].as_dvec3()).sum::<DVec3>() / inl.len() as f64;
    let mut m = [[0.0f64; 3]; 3];
    for &i in inl {
        let d = pts[i].as_dvec3() - c;
        let v = [d.x, d.y, d.z];
        for r in 0..3 {
            for k in 0..3 {
                m[r][k] += v[r] * v[k];
            }
        }
    }
    let (vals, vecs) = sym_eigen(m);
    let n = vecs[0].normalize().as_vec3();
    let sigma = (vals[0].max(0.0) / inl.len() as f64).sqrt() as f32;
    Some((n, -n.dot(c.as_vec3()), sigma))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SizeAxis {
    Height,
    /// Longest side.
    Longest,
    /// X (after the automatic yaw this is the longer horizontal side).
    Width,
    Depth,
}

impl SizeAxis {
    pub const ALL: [SizeAxis; 4] = [SizeAxis::Height, SizeAxis::Longest, SizeAxis::Width, SizeAxis::Depth];
    pub fn label(self) -> &'static str {
        match self {
            SizeAxis::Height => "Height",
            SizeAxis::Longest => "Longest side",
            SizeAxis::Width => "Width (X)",
            SizeAxis::Depth => "Depth (Y)",
        }
    }
    fn of(self, s: Vec3) -> f32 {
        match self {
            SizeAxis::Height => s.z,
            SizeAxis::Longest => s.max_element(),
            SizeAxis::Width => s.x,
            SizeAxis::Depth => s.y,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct PrepSettings {
    /// Cut away the detected table (and anything below it).
    pub remove_ground: bool,
    /// Keep only what is within this fraction of the camera ring radius (1.0+ = everything).
    pub crop: f32,
    /// Keep only the biggest connected piece (drops floating bits of background).
    pub keep_largest: bool,
    pub size_axis: SizeAxis,
    pub size_mm: f32,
    /// Extra flat cut from the bottom, in final millimetres (a bigger, flatter base).
    pub base_cut_mm: f32,
    /// Rebuild as a closed solid.
    pub solidify: bool,
    /// Solid grid spacing (mm).
    pub detail_mm: f32,
    /// Holes up to about twice this get closed (mm).
    pub close_mm: f32,
    /// Taubin smoothing passes after solidifying.
    pub smooth: u32,
    pub max_triangles: usize,
    /// Grid size limit for solidifying (memory ≈ 13 bytes per point).
    pub max_grid_points: usize,
}

impl Default for PrepSettings {
    fn default() -> Self {
        PrepSettings {
            remove_ground: true,
            crop: 0.8,
            keep_largest: true,
            size_axis: SizeAxis::Height,
            size_mm: 60.0,
            base_cut_mm: 0.0,
            solidify: true,
            detail_mm: 0.3,
            close_mm: 1.5,
            smooth: 2,
            max_triangles: 400_000,
            max_grid_points: 40_000_000,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Prepared {
    pub mesh: Mesh,
    pub stats: MeshStats,
    /// Millimetres per scan unit.
    pub scale: f32,
    /// Grid spacing used when solidifying.
    pub cell: Option<f32>,
    pub warnings: Vec<String>,
}

/// 2-D convex hull (Andrew's monotone chain), counter-clockwise.
fn hull2(mut pts: Vec<Vec2>) -> Vec<Vec2> {
    pts.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    pts.dedup();
    if pts.len() < 3 {
        return pts;
    }
    let cross = |o: Vec2, a: Vec2, b: Vec2| (a - o).perp_dot(b - o);
    let mut h: Vec<Vec2> = Vec::with_capacity(pts.len() * 2);
    for pass in 0..2 {
        let start = h.len();
        let iter: Box<dyn Iterator<Item = &Vec2>> = if pass == 0 { Box::new(pts.iter()) } else { Box::new(pts.iter().rev()) };
        for &p in iter {
            while h.len() >= start + 2 && cross(h[h.len() - 2], h[h.len() - 1], p) <= 0.0 {
                h.pop();
            }
            h.push(p);
        }
        h.pop();
    }
    h
}

/// Rotate about Z so the footprint's minimum-area bounding rectangle is axis-aligned,
/// with the longer side along X (tidy on the bed, and makes "Width"/"Depth" meaningful).
fn yaw_to_x(m: &mut Mesh) {
    let h = hull2(m.p.iter().map(|p| p.truncate()).collect());
    if h.len() < 3 {
        return;
    }
    let mut best = (f32::INFINITY, 0.0f32);
    for i in 0..h.len() {
        let e = h[(i + 1) % h.len()] - h[i];
        if e.length_squared() == 0.0 {
            continue;
        }
        let ang = e.y.atan2(e.x);
        let (s, c) = (-ang).sin_cos();
        let (mut lo, mut hi) = (Vec2::splat(f32::INFINITY), Vec2::splat(f32::NEG_INFINITY));
        for p in &h {
            let q = Vec2::new(c * p.x - s * p.y, s * p.x + c * p.y);
            lo = lo.min(q);
            hi = hi.max(q);
        }
        let area = (hi.x - lo.x) * (hi.y - lo.y);
        if area < best.0 {
            best = (area, ang);
        }
    }
    m.transform(&Mat4::from_rotation_z(-best.1));
    let s = m.size();
    if s.y > s.x {
        m.transform(&Mat4::from_rotation_z(std::f32::consts::FRAC_PI_2));
    }
}

fn center_on_bed(m: &mut Mesh) {
    if let Some((lo, hi)) = m.bounds() {
        let c = (lo + hi) * 0.5;
        m.translate(Vec3::new(-c.x, -c.y, -lo.z));
    }
}

/// Everything except solidifying is fast (well under a second for a few million triangles).
pub fn prepare(raw: &Mesh, a: &Alignment, s: &PrepSettings, log: &dyn Fn(&str)) -> Result<Prepared> {
    let mut warnings = Vec::new();
    let mut m = raw.clone();
    m.transform(&a.matrix());
    if let (true, Some(fz)) = (s.remove_ground, a.floor) {
        m.clip_halfspace(Vec3::Z, fz + a.noise);
    }
    if let (Some(c), Some(r)) = (a.center, a.ring_radius) {
        if s.crop < 1.5 {
            m.crop_cylinder(c, r * s.crop);
        }
    }
    m.keep_largest(if s.keep_largest { 0.05 } else { 0.002 });
    if m.is_empty() {
        bail!("nothing left after removing the background — widen the crop or turn off table removal");
    }
    yaw_to_x(&mut m);
    center_on_bed(&mut m);

    // scale (the base cut is in final mm, so iterate: cutting changes the measured size)
    let measure = |m: &Mesh| s.size_axis.of(m.size()).max(1e-9);
    let target = s.size_mm.max(0.01);
    let mut sc = target / measure(&m);
    if s.base_cut_mm > 0.0 {
        for _ in 0..4 {
            let mut tmp = m.clone();
            tmp.clip_halfspace(Vec3::Z, s.base_cut_mm / sc);
            if tmp.is_empty() {
                bail!("the base cut removes the whole object");
            }
            sc = target / measure(&tmp);
        }
        m.clip_halfspace(Vec3::Z, s.base_cut_mm / sc);
    }
    m.scale(sc);
    center_on_bed(&mut m);
    log(&format!("scaled ×{sc:.4} → {:.1} × {:.1} × {:.1} mm", m.size().x, m.size().y, m.size().z));

    let mut cell = None;
    if s.solidify {
        if m.t.len() > 1_500_000 {
            log("simplifying input before solidifying");
            m.decimate(1_000_000);
        }
        let r = solidify::solidify(&m, s.detail_mm, s.close_mm, s.max_grid_points, log)
            .ok_or_else(|| anyhow::anyhow!("solidify failed (empty mesh)"))?;
        if r.cell > s.detail_mm * 1.05 {
            warnings.push(format!("detail limited to {:.2} mm by the memory budget", r.cell));
        }
        cell = Some(r.cell);
        m = r.mesh;
        let before = m.t.len();
        m.keep_largest(0.01);
        if m.t.len() < before {
            log("removed small loose pieces");
        }
        if m.is_empty() {
            bail!("the solid came out empty");
        }
        m.taubin(s.smooth, Some(0.01 * r.cell));
        // the shell can come out thin if the scan has a large hole the closing could not seal
        let st = m.stats();
        let bb = st.size.x as f64 * st.size.y as f64 * st.size.z as f64;
        if bb > 0.0 && st.volume / bb < 0.02 {
            warnings.push("the solid is very thin — the scan probably has a large hole. Increase \"Close holes\" or cut the base.".into());
        }
    }
    if m.t.len() > s.max_triangles {
        m.decimate(s.max_triangles);
    }
    let stats = m.stats();
    if s.solidify && !stats.watertight() {
        warnings.push(format!("not watertight ({} open, {} non-manifold edges)", stats.boundary_edges, stats.nonmanifold_edges));
    }
    if stats.size.x > 256.0 || stats.size.y > 256.0 || stats.size.z > 256.0 {
        warnings.push("larger than a 256 mm print bed".into());
    }
    Ok(Prepared { mesh: m, stats, scale: sc, cell, warnings })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A box "object" standing on a big flat "table", in a tilted frame, with cameras above.
    fn scene() -> (Mesh, Vec<CameraPose>, Quat) {
        let mut m = Mesh::default();
        // table: 40×40 grid at z = 0
        let n = 40;
        for j in 0..=n {
            for i in 0..=n {
                m.p.push(Vec3::new(i as f32 - 20.0, j as f32 - 20.0, 0.0));
            }
        }
        for j in 0..n as u32 {
            for i in 0..n as u32 {
                let a = j * (n as u32 + 1) + i;
                m.t.push([a, a + 1, a + n as u32 + 2]);
                m.t.push([a, a + n as u32 + 2, a + n as u32 + 1]);
            }
        }
        // object: open-bottom box 4×2×3 on the table
        let base = m.p.len() as u32;
        let s = Vec3::new(4.0, 2.0, 3.0);
        for k in 0..8 {
            m.p.push(Vec3::new(((k & 1) as f32 - 0.5) * s.x, (((k >> 1) & 1) as f32 - 0.5) * s.y, ((k >> 2) & 1) as f32 * s.z));
        }
        for t in [[4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]] {
            m.t.push([base + t[0], base + t[1], base + t[2]]);
        }
        let cams: Vec<CameraPose> = (0..12)
            .map(|k| {
                let a = k as f32 / 12.0 * std::f32::consts::TAU;
                let c = Vec3::new(a.cos() * 10.0, a.sin() * 10.0, 6.0);
                let fwd = (Vec3::new(0.0, 0.0, 1.0) - c).normalize();
                CameraPose { center: c, up: Vec3::Z, forward: fwd }
            })
            .collect();
        // tilt everything into an arbitrary frame
        let q = Quat::from_euler(glam::EulerRot::XYZ, 0.7, -0.4, 1.9);
        m.transform(&Mat4::from_quat(q));
        let cams = cams.into_iter().map(|c| CameraPose { center: q * c.center, up: q * c.up, forward: q * c.forward }).collect();
        (m, cams, q)
    }

    #[test]
    fn finds_table_and_isolates_object() {
        let (m, cams, _) = scene();
        let a = Alignment::auto(&m, &cams);
        assert_eq!(a.source, AlignSource::Ground);
        let s = PrepSettings { size_axis: SizeAxis::Height, size_mm: 30.0, solidify: false, ..Default::default() };
        let p = prepare(&m, &a, &s, &|_| {}).unwrap();
        let sz = p.stats.size;
        // object only (table removed): 4×2×3 scaled to height 30
        assert!((sz.z - 30.0).abs() < 0.01, "{sz:?}");
        assert!((sz.x - 40.0).abs() < 0.2 && (sz.y - 20.0).abs() < 0.2, "{sz:?}");
    }

    #[test]
    fn solid_print_ready() {
        let (m, cams, _) = scene();
        let a = Alignment::auto(&m, &cams);
        let s = PrepSettings { size_mm: 30.0, detail_mm: 0.5, close_mm: 1.0, ..Default::default() };
        let p = prepare(&m, &a, &s, &|_| {}).unwrap();
        assert!(p.stats.watertight(), "{:?}", p.stats);
        assert_eq!(p.stats.components, 1);
        let (lo, _) = p.mesh.bounds().unwrap();
        assert!(lo.z.abs() < 0.05);
        // 40×20×30 box ≈ 24 cm³
        assert!((p.stats.volume / 1000.0 - 24.0).abs() < 2.5, "{}", p.stats.volume);
    }
}
