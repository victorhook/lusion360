//! The 3D viewport: camera navigation, picking, scene assembly.

use crate::app::{App, Filter, Hit, Mode, Pick};
use crate::camera::Ray;
use crate::render::{BodyDraw, FillVert, LineSeg, PointInst, Scene, ViewportCallback};
use crate::theme;
use eframe::egui_wgpu;
use egui::{Color32, PointerButton, Pos2, Rect, Sense, Vec2 as EVec2};
use glam::{DVec3, Vec2, Vec3};
use lusion_core::doc::*;
use lusion_core::model::ModelState;
use lusion_core::tess::BodyMesh;
use lusion_kernel::SurfaceKind;
use std::sync::Arc;

/// Per-frame viewport info shared with sketch mode / commands.
#[derive(Clone, Copy)]
pub struct View {
    pub rect: Rect,
    pub size: Vec2,
    pub mouse: Option<Vec2>,
    pub ppp: f32,
}

impl View {
    pub fn to_screen(&self, p: Vec2) -> Pos2 {
        self.rect.min + EVec2::new(p.x, p.y)
    }
}

pub fn face_range(m: &BodyMesh, face: u32) -> std::ops::Range<usize> {
    let a = m.tri_face.partition_point(|&f| f < face);
    let b = m.tri_face.partition_point(|&f| f <= face);
    a..b
}

pub struct VisBody {
    pub id: BodyId,
    pub mesh: Arc<BodyMesh>,
    pub is_mesh: bool,
}

pub fn visible_bodies(app: &mut App, st: &ModelState) -> Vec<VisBody> {
    let mut out = Vec::new();
    for b in st.bodies.values() {
        if !app.body_visible(st, b.id) {
            continue;
        }
        if let Some(m) = app.meshes.get(b) {
            out.push(VisBody { id: b.id, mesh: m, is_mesh: b.is_mesh() });
        }
    }
    out
}

/// Sketches (other than the one being edited) that are shown.
pub fn visible_sketches(app: &App, st: &ModelState) -> Vec<FeatureId> {
    let editing = match &app.mode {
        Mode::Sketch(s) => Some(s.fid),
        _ => None,
    };
    let mut v: Vec<FeatureId> = st
        .sketches
        .keys()
        .copied()
        .filter(|fid| Some(*fid) != editing)
        .filter(|fid| {
            app.doc
                .feature(*fid)
                .map(|f| f.visible && app.doc.component_visible(f.component))
                .unwrap_or(false)
        })
        .collect();
    v.sort();
    v
}

const ORIGIN_PLANES: [OriginPlane; 3] = [OriginPlane::XY, OriginPlane::XZ, OriginPlane::YZ];

pub fn origin_plane_frame(p: OriginPlane) -> lusion_core::sketch::Frame {
    use lusion_core::sketch::Frame;
    match p {
        OriginPlane::XY => Frame::new(DVec3::ZERO, DVec3::Z, DVec3::X),
        OriginPlane::XZ => Frame::new(DVec3::ZERO, DVec3::NEG_Y, DVec3::X),
        OriginPlane::YZ => Frame::new(DVec3::ZERO, DVec3::X, DVec3::Y),
    }
}

pub fn plane_size(app: &App) -> f32 {
    (app.cam.cam.distance * 0.25).clamp(5.0, 1.0e5)
}

fn show_planes(app: &App) -> bool {
    app.show_origin || app.cmd.as_ref().map(|c| c.wants_planes()).unwrap_or(false)
}

// ------------------------------------------------------------------ picking

pub fn pick(app: &mut App, st: &ModelState, ray: &Ray, view: &View, filter: Filter) -> Option<Hit> {
    let mouse = view.mouse?;
    let bodies = visible_bodies(app, st);
    // faces
    let mut face_hit: Option<Hit> = None;
    for vb in &bodies {
        let bvh = app.picks.bvh(&vb.mesh);
        if let Some((t, tri)) = bvh.raycast(&vb.mesh, ray) {
            if face_hit.map(|h| t < h.depth).unwrap_or(true) {
                let pick = if vb.is_mesh {
                    Pick::Body(vb.id)
                } else {
                    Pick::Face { body: vb.id, face: vb.mesh.tri_face[tri as usize] }
                };
                face_hit = Some(Hit { pick, point: ray.at(t), depth: t });
            }
        }
    }
    let occl = face_hit.map(|h| h.depth).unwrap_or(f32::INFINITY);
    let tol = 1e-3 * app.cam.cam.distance;

    // edges (screen space)
    if filter.edges {
        let mut best: Option<(f32, Hit)> = None;
        for vb in bodies.iter().filter(|b| !b.is_mesh) {
            for (ei, poly) in vb.mesh.edges.iter().enumerate() {
                let mut prev: Option<(Vec2, Vec3)> = None;
                for p in poly {
                    let w = Vec3::from_array(*p);
                    let s = app.cam.project(w, view.size);
                    if let (Some((ps, pw)), Some(s)) = (prev, s) {
                        let (d, t) = crate::pick::seg_dist(mouse, ps, s);
                        if d < 7.0 && best.map(|b| d < b.0).unwrap_or(true) {
                            let wp = pw.lerp(w, t);
                            let depth = (wp - ray.o).dot(ray.d);
                            if depth <= occl + tol * 4.0 {
                                best = Some((d, Hit { pick: Pick::Edge { body: vb.id, edge: ei as u32 }, point: wp, depth }));
                            }
                        }
                    }
                    prev = s.map(|s| (s, w));
                }
            }
        }
        if let Some((_, h)) = best {
            return Some(h);
        }
    }
    // sketch lines (e.g. revolve axis)
    if filter.sketch_lines {
        let mut best: Option<(f32, Hit)> = None;
        for sid in visible_sketches(app, st) {
            let ss = &st.sketches[&sid];
            for c in &ss.sketch.curves {
                if let Some((a, b)) = ss.sketch.line_pts(c.id) {
                    let (wa, wb) = (ss.frame.to3(a).as_vec3(), ss.frame.to3(b).as_vec3());
                    if let (Some(sa), Some(sb)) = (app.cam.project(wa, view.size), app.cam.project(wb, view.size)) {
                        let (d, t) = crate::pick::seg_dist(mouse, sa, sb);
                        if d < 7.0 && best.map(|b| d < b.0).unwrap_or(true) {
                            let wp = wa.lerp(wb, t);
                            best = Some((d, Hit { pick: Pick::SketchLine { sketch: sid, curve: c.id }, point: wp, depth: (wp - ray.o).dot(ray.d) }));
                        }
                    }
                }
            }
        }
        if let Some((_, h)) = best {
            return Some(h);
        }
    }
    if filter.axes {
        let s = plane_size(app) * 1.5;
        for (i, dir) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
            if let (Some(a), Some(b)) = (app.cam.project(-*dir * s, view.size), app.cam.project(*dir * s, view.size)) {
                if crate::pick::seg_dist(mouse, a, b).0 < 6.0 {
                    return Some(Hit { pick: Pick::Axis(i as u8), point: Vec3::ZERO, depth: 0.0 });
                }
            }
        }
    }
    // profiles
    if filter.profiles {
        let mut best: Option<(f32, f64, Hit)> = None;
        for sid in visible_sketches(app, st) {
            let ss = &st.sketches[&sid];
            let n = ss.frame.n.as_vec3();
            let Some(t) = ray.plane(ss.frame.origin.as_vec3(), n) else { continue };
            if t <= 0.0 || t > occl + tol * 4.0 {
                continue;
            }
            let p2 = ss.frame.to2(ray.at(t).as_dvec3());
            for (i, p) in ss.profiles.iter().enumerate() {
                if p.contains(p2) && best.map(|b| p.area < b.1 || t < b.0 - tol).unwrap_or(true) {
                    best = Some((t, p.area, Hit { pick: Pick::Profile { sketch: sid, index: i }, point: ray.at(t), depth: t }));
                }
            }
        }
        if let Some((_, _, h)) = best {
            return Some(h);
        }
    }
    if let Some(h) = face_hit {
        match h.pick {
            Pick::Face { body, face } if filter.faces => {
                if filter.planar_only {
                    let planar = st
                        .bodies
                        .get(&body)
                        .and_then(|b| b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok()))
                        .map(|i| i.kind == SurfaceKind::Plane)
                        .unwrap_or(false);
                    if planar {
                        return Some(h);
                    }
                } else {
                    return Some(h);
                }
            }
            Pick::Face { body, .. } if filter.bodies => return Some(Hit { pick: Pick::Body(body), ..h }),
            Pick::Body(_) if filter.bodies || filter.faces => return Some(h),
            _ => {}
        }
        if face_hit.is_some() && !filter.planes {
            return None;
        }
    }
    if filter.planes {
        let mut best: Option<Hit> = None;
        let mut consider = |pick: Pick, f: &lusion_core::sketch::Frame, half: f32| {
            let n = f.n.as_vec3();
            if let Some(t) = ray.plane(f.origin.as_vec3(), n) {
                if t > 0.0 && t < occl {
                    let p = f.to2(ray.at(t).as_dvec3());
                    if p.x.abs() as f32 <= half && p.y.abs() as f32 <= half && best.map(|b| t < b.depth).unwrap_or(true) {
                        best = Some(Hit { pick, point: ray.at(t), depth: t });
                    }
                }
            }
        };
        if show_planes(app) {
            let s = plane_size(app);
            for p in ORIGIN_PLANES {
                let f = origin_plane_frame(p);
                // origin planes are drawn as squares in the positive quadrant
                let f2 = lusion_core::sketch::Frame { origin: f.origin + (f.x + f.y) * (s as f64 * 0.5), ..f };
                consider(Pick::Origin(p), &f2, s * 0.5);
            }
        }
        for (fid, f) in &st.planes {
            if app.doc.feature(*fid).map(|f| f.visible).unwrap_or(false) {
                consider(Pick::Plane(*fid), f, plane_size(app) * 0.5);
            }
        }
        if best.is_some() {
            return best;
        }
    }
    None
}

// ------------------------------------------------------------------ scene

pub fn push_line(v: &mut Vec<LineSeg>, a: Vec3, b: Vec3, color: [f32; 4], width: f32) {
    v.push(LineSeg { p0: a.to_array(), p1: b.to_array(), color, width });
}

pub fn push_polyline(v: &mut Vec<LineSeg>, pts: &[Vec3], color: [f32; 4], width: f32) {
    for w in pts.windows(2) {
        push_line(v, w[0], w[1], color, width);
    }
}

pub fn push_dashed(v: &mut Vec<LineSeg>, pts: &[Vec3], color: [f32; 4], width: f32, dash: f32) {
    let mut acc = 0.0f32;
    for w in pts.windows(2) {
        let len = (w[1] - w[0]).length();
        if len < 1e-9 {
            continue;
        }
        let mut s = 0.0;
        while s < len {
            let phase = (acc / dash) as i64 % 2 == 0;
            let step = (dash - acc % dash).min(len - s);
            if phase {
                push_line(v, w[0].lerp(w[1], s / len), w[0].lerp(w[1], (s + step) / len), color, width);
            }
            s += step;
            acc += step;
        }
    }
}

fn push_face_fill(scene: &mut Scene, m: &BodyMesh, face: u32, color: [f32; 4]) {
    let r = face_range(m, face);
    for t in r {
        for k in 0..3 {
            let p = m.positions[m.indices[3 * t + k] as usize];
            scene.fills.push(FillVert { p, color });
        }
    }
}

fn push_tris2(scene: &mut Scene, f: &lusion_core::sketch::Frame, tris: &[[glam::DVec2; 3]], color: [f32; 4]) {
    for t in tris {
        for p in t {
            scene.fills.push(FillVert { p: f.to3(*p).as_vec3().to_array(), color });
        }
    }
}

fn push_square(scene: &mut Scene, f: &lusion_core::sketch::Frame, center: glam::DVec2, half: f64, fill: [f32; 4], edge: [f32; 4]) {
    let c = [
        center + glam::DVec2::new(-half, -half),
        center + glam::DVec2::new(half, -half),
        center + glam::DVec2::new(half, half),
        center + glam::DVec2::new(-half, half),
    ];
    push_tris2(scene, f, &[[c[0], c[1], c[2]], [c[0], c[2], c[3]]], fill);
    let pts: Vec<Vec3> = (0..5).map(|i| f.to3(c[i % 4]).as_vec3()).collect();
    push_polyline(&mut scene.lines, &pts, edge, 1.2);
}

pub fn highlight(app: &mut App, scene: &mut Scene, st: &ModelState, pick: Pick, strong: bool) {
    let (fc, ec, pc) = if strong { (theme::SELECT, theme::SELECT_EDGE, theme::PROFILE_SEL) } else { (theme::HOVER, theme::HOVER_EDGE, theme::PROFILE_HOVER) };
    match pick {
        Pick::Face { body, face } => {
            if let Some(b) = st.bodies.get(&body) {
                if let Some(m) = app.meshes.get(b) {
                    push_face_fill(scene, &m, face, fc);
                }
            }
        }
        Pick::Body(body) => {
            if let Some(b) = st.bodies.get(&body) {
                if let Some(m) = app.meshes.get(b) {
                    let n = m.tri_face.iter().copied().max().unwrap_or(0);
                    for f in 0..=n {
                        push_face_fill(scene, &m, f, [fc[0], fc[1], fc[2], fc[3] * 0.6]);
                    }
                }
            }
        }
        Pick::Edge { body, edge } => {
            if let Some(b) = st.bodies.get(&body) {
                if let Some(m) = app.meshes.get(b) {
                    if let Some(poly) = m.edges.get(edge as usize) {
                        let pts: Vec<Vec3> = poly.iter().map(|p| Vec3::from_array(*p)).collect();
                        push_polyline(&mut scene.lines, &pts, ec, 4.0);
                    }
                }
            }
        }
        Pick::Profile { sketch, index } => {
            if let Some(ss) = st.sketches.get(&sketch) {
                if let Some(p) = ss.profiles.get(index) {
                    push_tris2(scene, &ss.frame, &p.tris, pc);
                }
            }
        }
        Pick::Origin(p) => {
            let f = origin_plane_frame(p);
            let s = plane_size(app) as f64;
            push_square(scene, &f, glam::DVec2::splat(s * 0.5), s * 0.5, [fc[0], fc[1], fc[2], 0.35], ec);
        }
        Pick::Plane(fid) => {
            if let Some(f) = st.planes.get(&fid) {
                let s = plane_size(app) as f64;
                push_square(scene, f, glam::DVec2::ZERO, s * 0.5, [fc[0], fc[1], fc[2], 0.35], ec);
            }
        }
        Pick::Axis(i) => {
            let s = plane_size(app) * 1.5;
            let d = [Vec3::X, Vec3::Y, Vec3::Z][i as usize];
            push_line(&mut scene.overlay_lines, -d * s, d * s, ec, 3.0);
        }
        Pick::SketchLine { sketch, curve } => {
            if let Some(ss) = st.sketches.get(&sketch) {
                if let Some((a, b)) = ss.sketch.line_pts(curve) {
                    push_line(&mut scene.overlay_lines, ss.frame.to3(a).as_vec3(), ss.frame.to3(b).as_vec3(), ec, 3.5);
                }
            }
        }
    }
}

fn grid(app: &App, scene: &mut Scene, frame: &lusion_core::sketch::Frame, strength: f32) {
    let d = app.cam.cam.distance.max(1e-3);
    let step = 10f32.powf((d / 3.0).log10().floor()) * 0.5;
    let step = step.max(0.01);
    let n = 24i32;
    let t = frame.to2(app.cam.cam.target.as_dvec3());
    let snap = step * 10.0;
    let cx = (t.x as f32 / snap).round() * snap;
    let cy = (t.y as f32 / snap).round() * snap;
    let base = if app.light_bg { [0.22, 0.25, 0.30] } else { [0.62, 0.68, 0.76] };
    let ext = n as f32 * step;
    for i in -n..=n {
        let fade = (1.0 - (i.abs() as f32 / n as f32)).powf(1.5);
        for axis in 0..2 {
            let c0 = if axis == 0 { cx } else { cy };
            let v = c0 + i as f32 * step;
            let major = ((v / step).round() as i64).rem_euclid(10) == 0;
            let a = if major { 0.16 } else { 0.055 } * strength * fade;
            if a < 0.004 {
                continue;
            }
            let col = [base[0], base[1], base[2], a];
            // fade the far ends by splitting into segments
            let segs = 6;
            for k in 0..segs {
                let u0 = -ext + 2.0 * ext * k as f32 / segs as f32;
                let u1 = -ext + 2.0 * ext * (k + 1) as f32 / segs as f32;
                let um = (u0 + u1) * 0.5;
                let f2 = (1.0 - (um / ext).abs()).powf(1.2);
                let col = [col[0], col[1], col[2], col[3] * f2];
                let (p0, p1) = if axis == 0 {
                    (glam::DVec2::new(v as f64, (cy + u0) as f64), glam::DVec2::new(v as f64, (cy + u1) as f64))
                } else {
                    (glam::DVec2::new((cx + u0) as f64, v as f64), glam::DVec2::new((cx + u1) as f64, v as f64))
                };
                push_line(&mut scene.lines, frame.to3(p0).as_vec3(), frame.to3(p1).as_vec3(), col, 1.0);
            }
        }
    }
}

fn build_scene(app: &mut App, view: &View) -> Scene {
    let aspect = view.size.x / view.size.y.max(1.0);
    let st = app.display_state().clone();
    let pst = app.pick_state().clone();
    let bounds = app.scene_bounds();
    let radius = bounds.map(|(a, b)| (b - a).length() * 0.5 + (a + b).length() * 0.5).unwrap_or(100.0).max(10.0);
    app.cam.scene_radius = radius;
    let cam = app.cam.cam;
    let light = (cam.right() * -0.35 + cam.up() * 0.55 - cam.forward() * 0.75).normalize();
    let mut scene = Scene {
        viewport_px: [view.size.x * view.ppp, view.size.y * view.ppp],
        view_proj: cam.view_proj(aspect, radius),
        eye: cam.eye(),
        fwd: cam.forward(),
        cam_dist: cam.distance,
        light,
        ortho: cam.ortho,
        bg_top: if app.light_bg { theme::BG_TOP_LIGHT } else { theme::BG_TOP },
        bg_bottom: if app.light_bg { theme::BG_BOTTOM_LIGHT } else { theme::BG_BOTTOM },
        ..Default::default()
    };

    let in_sketch = matches!(app.mode, Mode::Sketch(_));
    // bodies
    for vb in visible_bodies(app, &st) {
        let props = app.doc.body_props(vb.id);
        let comp_color = st
            .bodies
            .get(&vb.id)
            .and_then(|b| app.doc.component(app.body_component(&st, b.id)))
            .and_then(|c| c.color);
        let mut color = match (props.color, comp_color) {
            (Some(c), _) | (None, Some(c)) => [c[0], c[1], c[2], 1.0],
            _ => {
                if vb.is_mesh {
                    theme::MESH_BODY
                } else {
                    theme::BODY
                }
            }
        };
        // whole-body hover/selection: tint on the GPU instead of overlay geometry
        let sel_body = app.selection.contains(&Pick::Body(vb.id))
            || app.cmd.as_ref().map(|c| c.selected_picks(app).contains(&Pick::Body(vb.id))).unwrap_or(false);
        let hov_body = matches!(app.hover, Some(Hit { pick: Pick::Body(b), .. }) if b == vb.id);
        let tint = |c: [f32; 4], t: [f32; 4], k: f32| [c[0] + (t[0] - c[0]) * k, c[1] + (t[1] - c[1]) * k, c[2] + (t[2] - c[2]) * k, c[3]];
        if sel_body {
            color = tint(color, theme::SELECT, 0.45);
        } else if hov_body {
            color = tint(color, theme::HOVER, 0.25);
        }
        if in_sketch {
            color[3] = 0.55;
        }
        let edge_color = if app.light_bg { [0.1, 0.1, 0.12, 1.0] } else { theme::BODY_EDGE };
        scene.bodies.push(BodyDraw { mesh: vb.mesh.clone(), color, edge_color, show_edges: app.show_edges, face_hl: vec![], edge_hl: vec![] });
    }
    // soft contact shadow on the ground plane
    if let Some((lo, hi)) = bounds {
        let eye = cam.eye();
        if !in_sketch && eye.z > lo.z && !st.bodies.is_empty() {
            let c = (lo + hi) * 0.5;
            let ext = (hi - lo) * 0.5;
            let z = lo.z - (hi - lo).length() * 0.002;
            let m = ext.x.max(ext.y) * 0.35 + 3.0;
            let strength = if app.light_bg { 0.30 } else { 0.55 };
            let n = 64;
            let pt = |k: usize, rx: f32, ry: f32, a: f32| {
                let t = k as f32 / n as f32 * std::f32::consts::TAU;
                FillVert { p: [c.x + rx * t.cos(), c.y + ry * t.sin(), z], color: [0.0, 0.0, 0.0, a] }
            };
            let center = FillVert { p: [c.x, c.y, z], color: [0.0, 0.0, 0.0, strength] };
            for k in 0..n {
                let (i0, i1) = (pt(k, ext.x, ext.y, strength), pt(k + 1, ext.x, ext.y, strength));
                let (o0, o1) = (pt(k, ext.x + m, ext.y + m, 0.0), pt(k + 1, ext.x + m, ext.y + m, 0.0));
                scene.fills.extend_from_slice(&[center, i0, i1, i0, o0, o1, i0, o1, i1]);
            }
        }
    }
    // grid
    if !in_sketch {
        grid(app, &mut scene, &origin_plane_frame(OriginPlane::XY), 1.0);
        let s = plane_size(app) * 0.6;
        push_line(&mut scene.lines, Vec3::ZERO, Vec3::X * s, [0.9, 0.3, 0.3, 0.9], 1.6);
        push_line(&mut scene.lines, Vec3::ZERO, Vec3::Y * s, [0.35, 0.8, 0.35, 0.9], 1.6);
        push_line(&mut scene.lines, Vec3::ZERO, Vec3::Z * s, [0.35, 0.55, 1.0, 0.9], 1.6);
    }
    // origin planes
    if show_planes(app) {
        let s = plane_size(app) as f64;
        for (p, col) in [(OriginPlane::XY, [0.3, 0.5, 1.0]), (OriginPlane::XZ, [0.3, 0.85, 0.4]), (OriginPlane::YZ, [0.95, 0.4, 0.35])] {
            let f = origin_plane_frame(p);
            push_square(&mut scene, &f, glam::DVec2::splat(s * 0.5), s * 0.5, [col[0], col[1], col[2], 0.10], [col[0], col[1], col[2], 0.7]);
        }
    }
    // construction planes
    for (fid, f) in &st.planes {
        if app.doc.feature(*fid).map(|f| f.visible).unwrap_or(false) {
            let s = plane_size(app) as f64;
            push_square(&mut scene, f, glam::DVec2::ZERO, s * 0.5, [0.95, 0.75, 0.3, 0.10], [0.95, 0.75, 0.3, 0.8]);
        }
    }
    // sketches (not being edited)
    for sid in visible_sketches(app, &st) {
        let ss = st.sketches[&sid].clone();
        let tol = (cam.distance * 0.001) as f64;
        for c in &ss.sketch.curves {
            let pts: Vec<Vec3> = ss.sketch.curve_polyline(c, tol).iter().map(|p| ss.frame.to3(*p).as_vec3()).collect();
            if c.construction {
                push_dashed(&mut scene.lines, &pts, [0.95, 0.6, 0.25, 0.6], 1.2, cam.distance * 0.008);
            } else {
                push_polyline(&mut scene.lines, &pts, theme::SK_INACTIVE, 1.6);
            }
        }
        for t in &ss.sketch.texts {
            for c in ss.sketch.text_outlines(t, &app.model.params) {
                let mut pts: Vec<Vec3> = c.iter().map(|p| ss.frame.to3(*p).as_vec3()).collect();
                if let Some(f0) = pts.first().copied() {
                    pts.push(f0);
                }
                push_polyline(&mut scene.lines, &pts, theme::SK_INACTIVE, 1.2);
            }
        }
        if !in_sketch {
            for p in &ss.profiles {
                push_tris2(&mut scene, &ss.frame, &p.tris, theme::PROFILE);
            }
        }
    }
    // selection + hover highlights (model mode and commands)
    if !in_sketch {
        let mut sel = app.selection.clone();
        if let Some(c) = &app.cmd {
            sel.extend(c.selected_picks(app));
        }
        for p in sel {
            if !matches!(p, Pick::Body(_)) {
                highlight(app, &mut scene, &pst, p, true);
            }
        }
        if let Some(h) = app.hover {
            if !matches!(h.pick, Pick::Body(_)) {
                highlight(app, &mut scene, &pst, h.pick, false);
            }
        }
    }
    // timeline hover: highlight what the feature made
    if let Some(fid) = app.timeline_hover {
        let prefix = format!("f{fid}.");
        let ids: Vec<BodyId> = st.bodies.keys().copied().collect();
        for id in ids {
            let b = &st.bodies[&id];
            if !app.body_visible(&st, id) {
                continue;
            }
            let faces: Vec<u32> = b.face_names.iter().enumerate().filter(|(_, n)| n.starts_with(&prefix)).map(|(i, _)| i as u32).collect();
            if let Some(m) = app.meshes.get(b) {
                for f in faces {
                    push_face_fill(&mut scene, &m, f, [1.0, 0.72, 0.28, 0.45]);
                }
            }
        }
        if let Some(ss) = st.sketches.get(&fid) {
            for c in &ss.sketch.curves {
                let pts: Vec<Vec3> = ss.sketch.curve_polyline(c, 0.05).iter().map(|p| ss.frame.to3(*p).as_vec3()).collect();
                push_polyline(&mut scene.overlay_lines, &pts, [1.0, 0.72, 0.28, 1.0], 2.5);
            }
        }
        if let Some(f) = st.planes.get(&fid) {
            let s = plane_size(app) as f64;
            push_square(&mut scene, f, glam::DVec2::ZERO, s * 0.5, [1.0, 0.72, 0.28, 0.25], [1.0, 0.72, 0.28, 1.0]);
        }
    }
    if let Some(c) = app.cmd.take() {
        c.draw(app, &mut scene, view);
        app.cmd = Some(c);
    }
    if in_sketch {
        crate::sketch_mode::draw(app, &mut scene, view);
    }
    scene
}

// ------------------------------------------------------------------ main entry

pub fn show(app: &mut App, ui: &mut egui::Ui) {
    let rect = ui.available_rect_before_wrap();
    let resp = ui.allocate_rect(rect, Sense::click_and_drag());
    let ppp = ui.ctx().pixels_per_point();
    let mouse = resp.hover_pos().map(|p| Vec2::new(p.x - rect.min.x, p.y - rect.min.y));
    let view = View { rect, size: Vec2::new(rect.width(), rect.height()), mouse, ppp };

    navigate(app, ui, &resp, &view);
    let mut consumed = false;
    if let Some(mut c) = app.cmd.take() {
        consumed = c.handle_drag(app, ui, &view);
        app.cmd = Some(c);
    }

    // hover + clicks
    let ray = mouse.map(|m| app.cam.ray(m, view.size));
    match &app.mode {
        Mode::Sketch(_) => {
            crate::sketch_mode::input(app, ui, &resp, &view, ray);
        }
        Mode::Model => {
            let filter = match &app.cmd {
                Some(c) => c.filter(),
                None => Filter::default_model(),
            };
            let st = app.pick_state().clone();
            let navigating = resp.dragged_by(PointerButton::Middle) || resp.dragged_by(PointerButton::Secondary);
            app.hover = match (ray, navigating) {
                (Some(r), false) => pick(app, &st, &r, &view, filter),
                _ => None,
            };
            if resp.clicked_by(PointerButton::Primary) && !consumed {
                let shift = ui.input(|i| i.modifiers.shift || i.modifiers.command);
                let hit = app.hover;
                if let Some(mut c) = app.cmd.take() {
                    if let Some(h) = hit {
                        c.on_pick(app, h);
                    }
                    app.cmd = Some(c);
                } else {
                    match hit {
                        Some(h) => {
                            if shift {
                                if let Some(i) = app.selection.iter().position(|p| *p == h.pick) {
                                    app.selection.remove(i);
                                } else {
                                    app.selection.push(h.pick);
                                }
                            } else {
                                app.selection = vec![h.pick];
                            }
                        }
                        None => {
                            if !shift {
                                app.selection.clear();
                            }
                        }
                    }
                }
            }
            if resp.double_clicked_by(PointerButton::Primary) && app.cmd.is_none() {
                if let Some(h) = app.hover {
                    match h.pick {
                        Pick::Profile { .. } => crate::commands::start(app, crate::commands::CmdKind::Extrude),
                        Pick::Face { .. } => crate::commands::start(app, crate::commands::CmdKind::Extrude),
                        _ => {}
                    }
                }
            }
            context_menu(app, &resp);
        }
    }

    if resp.hovered() {
        let icon = match &app.mode {
            Mode::Sketch(s) if s.tool != crate::sketch_mode::SkTool::Select => egui::CursorIcon::Crosshair,
            _ if app.hover.is_some() => egui::CursorIcon::PointingHand,
            _ => egui::CursorIcon::Default,
        };
        if resp.dragged_by(PointerButton::Secondary) || resp.dragged_by(PointerButton::Middle) {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
        } else {
            ui.ctx().set_cursor_icon(icon);
        }
    }
    let scene = build_scene(app, &view);
    ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, ViewportCallback { scene: Arc::new(scene) }));

    // 2D overlays
    if let Mode::Sketch(_) = app.mode {
        crate::sketch_mode::overlay(app, ui, &view);
    }
    if let Some(mut c) = app.cmd.take() {
        c.overlay(app, ui, &view);
        app.cmd = Some(c);
    }
    view_controls(app, ui, &view);
    welcome(app, ui, &view);
    axis_triad(app, ui, &view);
    hover_tooltip(app, ui, &view);
}

fn navigate(app: &mut App, ui: &egui::Ui, resp: &egui::Response, view: &View) {
    let (scroll, shift, mods) = ui.input(|i| (i.smooth_scroll_delta.y, i.modifiers.shift, i.modifiers));
    let _ = mods;
    if resp.hovered() && scroll.abs() > 0.0 {
        if let Some(m) = view.mouse {
            app.cam.zoom_at((-scroll * 0.0018).exp(), m, view.size);
        }
    }
    // pivot = point under the cursor when a drag starts
    let start = resp.drag_started_by(PointerButton::Secondary) || resp.drag_started_by(PointerButton::Middle);
    if start {
        let pivot = view.mouse.and_then(|m| {
            let ray = app.cam.ray(m, view.size);
            let st = app.display_state().clone();
            let bodies = visible_bodies(app, &st);
            let mut best: Option<f32> = None;
            for vb in &bodies {
                let bvh = app.picks.bvh(&vb.mesh);
                if let Some((t, _)) = bvh.raycast(&vb.mesh, &ray) {
                    if best.map(|b| t < b).unwrap_or(true) {
                        best = Some(t);
                    }
                }
            }
            best.map(|t| ray.at(t))
        });
        ui.ctx().data_mut(|d| d.insert_temp(egui::Id::new("orbit_pivot"), pivot.unwrap_or(app.cam.cam.target)));
    }
    let pivot: Vec3 = ui.ctx().data(|d| d.get_temp(egui::Id::new("orbit_pivot"))).unwrap_or(app.cam.cam.target);
    let d = resp.drag_delta();
    let delta = Vec2::new(d.x, d.y);
    if resp.dragged_by(PointerButton::Secondary) || (resp.dragged_by(PointerButton::Middle) && shift) {
        app.cam.orbit(delta, pivot);
    } else if resp.dragged_by(PointerButton::Middle) {
        app.cam.pan(delta, view.size.y);
    }
}

fn context_menu(app: &mut App, resp: &egui::Response) {
    if resp.secondary_clicked() {
        let t = app.hover.map(|h| h.pick);
        resp.ctx.data_mut(|d| d.insert_temp(egui::Id::new("ctx_target"), t));
    }
    resp.context_menu(|ui| {
        let target: Option<Pick> = ui.ctx().data(|d| d.get_temp(egui::Id::new("ctx_target"))).flatten();
        ui.set_min_width(190.0);
        let body = match target {
            Some(Pick::Face { body, .. }) | Some(Pick::Edge { body, .. }) | Some(Pick::Body(body)) => Some(body),
            _ => None,
        };
        if let Some(Pick::Face { .. }) | Some(Pick::Origin(_)) | Some(Pick::Plane(_)) = target {
            if ui.button(format!("{}  New sketch here", crate::icons::PENCIL_SIMPLE_LINE)).clicked() {
                crate::commands::new_sketch_on(app, target.unwrap());
                ui.close();
            }
        }
        if let Some(Pick::Face { .. }) | Some(Pick::Profile { .. }) = target {
            if ui.button(format!("{}  Extrude", crate::icons::ARROW_FAT_UP)).clicked() {
                app.selection = vec![target.unwrap()];
                crate::commands::start(app, crate::commands::CmdKind::Extrude);
                ui.close();
            }
        }
        if let Some(Pick::Edge { .. }) = target {
            if ui.button(format!("{}  Fillet", crate::icons::BEZIER_CURVE)).clicked() {
                app.selection = vec![target.unwrap()];
                crate::commands::start(app, crate::commands::CmdKind::Fillet);
                ui.close();
            }
        }
        if let Some(b) = body {
            ui.separator();
            ui.label(egui::RichText::new(app.body_name(b)).color(theme::TEXT_DIM));
            if ui.button(format!("{}  Hide body", crate::icons::EYE_SLASH)).clicked() {
                let before = app.doc.clone();
                app.doc.bodies.entry(b).or_default().visible = Some(false);
                app.commit(before);
                ui.close();
            }
            if ui.button(format!("{}  Rename body…", crate::icons::TEXT_AA)).clicked() {
                app.rename = Some((crate::app::RenameTarget::Body(b), app.body_name(b)));
                ui.close();
            }
            if ui.button(format!("{}  Export body…", crate::icons::EXPORT)).clicked() {
                app.export(Some(vec![b]));
                ui.close();
            }
            if ui.button(format!("{}  Move to new component", crate::icons::PUZZLE_PIECE)).clicked() {
                let before = app.doc.clone();
                let parent = app.active_comp;
                let name = format!("Component{}", app.doc.components.len());
                let c = app.doc.add_component(&name, parent);
                app.doc.bodies.entry(b).or_default().component = Some(c);
                app.commit(before);
                ui.close();
            }
        }
        ui.separator();
        if ui.button(format!("{}  Fit all", crate::icons::CORNERS_OUT)).clicked() {
            app.fit_all(true);
            ui.close();
        }
        if ui.button(format!("{}  Home view", crate::icons::HOUSE)).clicked() {
            app.home_view();
            ui.close();
        }
        if ui.button(format!("{}  New sketch", crate::icons::PENCIL_SIMPLE_LINE)).clicked() {
            crate::commands::start(app, crate::commands::CmdKind::NewSketch);
            ui.close();
        }
    });
}

const CUBE_FACES: [(Vec3, &str, f32, f32); 6] = [
    (Vec3::Z, "TOP", 0.0, 90.0),
    (Vec3::NEG_Z, "BOTTOM", 0.0, -90.0),
    (Vec3::NEG_Y, "FRONT", 0.0, 0.0),
    (Vec3::Y, "BACK", 180.0, 0.0),
    (Vec3::X, "RIGHT", 90.0, 0.0),
    (Vec3::NEG_X, "LEFT", -90.0, 0.0),
];

fn view_cube(app: &mut App, ui: &mut egui::Ui, view: &View) {
    let size = 34.0;
    let center = view.rect.right_top() + EVec2::new(-74.0, 72.0);
    let cam = app.cam.cam;
    let proj = |p: Vec3| center + EVec2::new(p.dot(cam.right()), -p.dot(cam.up())) * size;
    let area = egui::Rect::from_center_size(center, egui::vec2(size * 3.4, size * 3.4));
    let resp = ui.interact(area, ui.id().with("view_cube"), Sense::click());
    let mouse = resp.hover_pos();
    let painter = ui.painter_at(view.rect);
    // back-to-front
    let mut faces: Vec<(Vec3, &str, f32, f32)> = CUBE_FACES.to_vec();
    faces.sort_by(|a, b| a.0.dot(cam.forward()).partial_cmp(&b.0.dot(cam.forward())).unwrap().reverse());
    let mut hovered: Option<(f32, f32)> = None;
    let mut polys = Vec::new();
    for (n, label, yaw, pitch) in faces {
        let facing = -n.dot(cam.forward());
        if facing <= 0.02 {
            continue;
        }
        let (u, v) = if n.z.abs() > 0.5 { (Vec3::X, Vec3::Y * n.z.signum()) } else { (Vec3::Z.cross(n), Vec3::Z) };
        let pts: Vec<Pos2> = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
            .iter()
            .map(|(a, b)| proj(n + u * *a + v * *b))
            .collect();
        let inside = mouse.map(|m| point_in_poly(m, &pts)).unwrap_or(false);
        if inside {
            hovered = Some((yaw, pitch));
        }
        polys.push((pts, label, facing, inside, proj(n)));
    }
    // only the front-most hovered face counts
    let mut hover_label = None;
    if mouse.is_some() {
        for (pts, label, _, inside, _) in polys.iter().rev() {
            if *inside {
                hover_label = Some(*label);
                let _ = pts;
                break;
            }
        }
    }
    for (pts, label, facing, _, c) in &polys {
        let hot = hover_label == Some(*label);
        let shade = (150.0 + 70.0 * facing) as u8;
        let fill = if hot { theme::ACCENT } else { Color32::from_rgba_unmultiplied(shade / 3 + 20, shade / 3 + 24, shade / 3 + 30, 235) };
        painter.add(egui::Shape::convex_polygon(pts.clone(), fill, egui::Stroke::new(1.0, Color32::from_rgb(90, 100, 115))));
        if *facing > 0.35 {
            painter.text(*c, egui::Align2::CENTER_CENTER, *label, egui::FontId::proportional(9.0 + 2.0 * facing), Color32::from_gray((170.0 + 85.0 * facing) as u8));
        }
    }
    if resp.clicked() {
        if let Some(l) = hover_label {
            if let Some((_, _, yaw, pitch)) = CUBE_FACES.iter().find(|f| f.1 == l) {
                app.cam.set_orientation(crate::camera::Camera::view_rot(yaw.to_radians(), pitch.to_radians()));
            }
        }
    }
    let _ = hovered;
}

fn point_in_poly(p: Pos2, pts: &[Pos2]) -> bool {
    let mut inside = false;
    let n = pts.len();
    let mut j = n - 1;
    for i in 0..n {
        let (a, b) = (pts[i], pts[j]);
        if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn view_controls(app: &mut App, ui: &mut egui::Ui, view: &View) {
    view_cube(app, ui, view);
    let w = 30.0;
    let pos = view.rect.right_top() + EVec2::new(-(w + 2.0) * 4.0 - 26.0, 140.0);
    egui::Area::new(egui::Id::new("view_controls")).fixed_pos(pos).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
        egui::Frame::new().fill(Color32::from_rgba_unmultiplied(30, 33, 38, 200)).corner_radius(8).inner_margin(3).show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(2.0, 2.0);
                let b = |ui: &mut egui::Ui, icon: &str, tip: &str| {
                    ui.add_sized([w, 24.0], egui::Button::new(egui::RichText::new(icon).size(15.0)).frame(false)).on_hover_text(tip).clicked()
                };
                if b(ui, crate::icons::HOUSE, "Home view (Home / F6)") {
                    app.home_view();
                }
                if b(ui, crate::icons::CORNERS_OUT, "Fit all") {
                    app.fit_all(true);
                }
                let icon = if app.cam.cam.ortho { crate::icons::CUBE } else { crate::icons::PERSPECTIVE };
                if b(ui, icon, if app.cam.cam.ortho { "Orthographic (click for perspective)" } else { "Perspective (click for orthographic)" }) {
                    app.cam.cam.ortho = !app.cam.cam.ortho;
                }
                if let Mode::Sketch(s) = &app.mode {
                    let fid = s.fid;
                    if b(ui, crate::icons::FRAME_CORNERS, "Look at sketch") {
                        crate::sketch_mode::look_at(app, fid);
                    }
                }
            });
        });
    });
}

fn welcome(app: &mut App, ui: &mut egui::Ui, view: &View) {
    if !app.doc.features.is_empty() || app.cmd.is_some() || !matches!(app.mode, Mode::Model) {
        return;
    }
    let pos = view.rect.center() - EVec2::new(170.0, 150.0);
    egui::Area::new(egui::Id::new("welcome")).fixed_pos(pos).order(egui::Order::Middle).show(ui.ctx(), |ui| {
        egui::Frame::new()
            .fill(Color32::from_rgba_unmultiplied(28, 31, 36, 235))
            .stroke(egui::Stroke::new(1.0, Color32::from_rgb(55, 62, 72)))
            .corner_radius(12)
            .inner_margin(18)
            .show(ui, |ui| {
                ui.set_width(300.0);
                ui.label(egui::RichText::new(format!("{} Lusion360", crate::icons::CUBE)).size(22.0).strong().color(theme::ACCENT));
                ui.label(egui::RichText::new("Fast, local, parametric.").color(theme::TEXT_DIM));
                ui.add_space(10.0);
                let big = |ui: &mut egui::Ui, icon: &str, t: &str| {
                    ui.add_sized([300.0, 34.0], egui::Button::new(egui::RichText::new(format!("{icon}   {t}")).size(15.0))).clicked()
                };
                if big(ui, crate::icons::PENCIL_SIMPLE_LINE, "Start a sketch") {
                    crate::commands::start(app, crate::commands::CmdKind::NewSketch);
                }
                if big(ui, crate::icons::FOLDER_OPEN, "Open design or 3D file…") {
                    app.request(crate::app::PendingAction::Open(None));
                }
                if big(ui, crate::icons::DOWNLOAD_SIMPLE, "Import STEP / STL / 3MF / OBJ…") {
                    app.import_file();
                }
                if !app.recent.is_empty() {
                    ui.add_space(8.0);
                    ui.label(egui::RichText::new("Recent").small().color(theme::TEXT_DIM));
                    for p in app.recent.clone().into_iter().take(5) {
                        let name = p.file_name().unwrap_or_default().to_string_lossy().to_string();
                        if ui.link(name).on_hover_text(p.display().to_string()).clicked() {
                            app.request(crate::app::PendingAction::Open(Some(p)));
                        }
                    }
                }
                ui.add_space(8.0);
                ui.label(egui::RichText::new("Tip: drop files onto the window · press S to search commands").small().color(theme::TEXT_DIM));
            });
    });
}

fn axis_triad(app: &App, ui: &egui::Ui, view: &View) {
    let c = view.rect.left_bottom() + EVec2::new(46.0, -46.0);
    let cam = app.cam.cam;
    let painter = ui.painter_at(view.rect);
    let mut axes = [(Vec3::X, "X", Color32::from_rgb(235, 80, 80)), (Vec3::Y, "Y", Color32::from_rgb(90, 205, 90)), (Vec3::Z, "Z", Color32::from_rgb(90, 140, 255))];
    // draw back-to-front
    axes.sort_by(|a, b| a.0.dot(cam.forward()).partial_cmp(&b.0.dot(cam.forward())).unwrap().reverse());
    for (d, l, col) in axes {
        let x = d.dot(cam.right());
        let y = d.dot(cam.up());
        let tip = c + EVec2::new(x, -y) * 30.0;
        painter.line_segment([c, tip], egui::Stroke::new(2.0, col));
        painter.circle_filled(tip, 8.0, col.gamma_multiply(0.9));
        painter.text(tip, egui::Align2::CENTER_CENTER, l, egui::FontId::proportional(10.0), Color32::BLACK);
    }
}

fn hover_tooltip(app: &App, ui: &egui::Ui, view: &View) {
    let _ = (app, ui, view);
}
