//! Feature commands ("dialogs"): selection-driven creation and editing of
//! timeline features with live preview.

use crate::app::{App, Filter, Hit, Mode, Pick};
use crate::icons as ic;
use crate::render::Scene;
use crate::theme;
use crate::viewport::{push_line, push_polyline, View};
use egui::{Color32, RichText};
use glam::{DVec3, Vec2, Vec3};
use lusion_core::doc::*;
use lusion_core::model::{find_profile, ModelState};
use lusion_core::sketch::Sketch;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CmdKind {
    NewSketch,
    Extrude,
    Revolve,
    Fillet,
    Chamfer,
    Shell,
    Plane,
    Combine,
    Move,
    Pattern,
    Mirror,
    Split,
    Hole,
    Thread,
    Sweep,
    Loft,
    Joint,
    Measure,
    Section,
}

impl CmdKind {
    /// Inspection tools: no timeline feature, just a panel.
    pub fn is_tool(self) -> bool {
        matches!(self, CmdKind::Measure | CmdKind::Section)
    }
    pub fn title(self) -> &'static str {
        match self {
            CmdKind::NewSketch => "Create Sketch",
            CmdKind::Extrude => "Extrude",
            CmdKind::Revolve => "Revolve",
            CmdKind::Fillet => "Fillet",
            CmdKind::Chamfer => "Chamfer",
            CmdKind::Shell => "Shell",
            CmdKind::Plane => "Construction Plane",
            CmdKind::Combine => "Combine",
            CmdKind::Move => "Move / Copy",
            CmdKind::Pattern => "Pattern",
            CmdKind::Mirror => "Mirror",
            CmdKind::Split => "Split Body",
            CmdKind::Hole => "Hole",
            CmdKind::Thread => "Thread",
            CmdKind::Sweep => "Sweep",
            CmdKind::Loft => "Loft",
            CmdKind::Joint => "Joint",
            CmdKind::Measure => "Measure",
            CmdKind::Section => "Section View",
        }
    }
    pub fn icon(self) -> &'static str {
        match self {
            CmdKind::NewSketch => ic::PENCIL_SIMPLE_LINE,
            CmdKind::Extrude => ic::ARROW_FAT_UP,
            CmdKind::Revolve => ic::ARROWS_CLOCKWISE,
            CmdKind::Fillet => ic::BEZIER_CURVE,
            CmdKind::Chamfer => ic::POLYGON,
            CmdKind::Shell => ic::CUBE_TRANSPARENT,
            CmdKind::Plane => ic::SQUARE_HALF,
            CmdKind::Combine => ic::UNITE,
            CmdKind::Move => ic::ARROWS_OUT_CARDINAL,
            CmdKind::Pattern => ic::DOTS_NINE,
            CmdKind::Mirror => ic::FLIP_HORIZONTAL,
            CmdKind::Split => ic::KNIFE,
            CmdKind::Hole => ic::RECORD,
            CmdKind::Thread => ic::SPIRAL,
            CmdKind::Sweep => ic::PATH,
            CmdKind::Loft => ic::STACK,
            CmdKind::Joint => ic::LINK,
            CmdKind::Measure => ic::RULER,
            CmdKind::Section => ic::KNIFE,
        }
    }
}

/// Construction plane variants offered by the Plane command.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PlaneKind {
    #[default]
    Offset,
    Angle,
    Midplane,
    NormalToEdge,
}

pub struct Command {
    pub kind: CmdKind,
    pub fid: Option<FeatureId>,
    pub editing: bool,
    pub before: Document,
    pub prev_marker: usize,
    pub stage: u8,
    gdrag: Option<GDrag>,
    ghover: Option<u8>,
    /// automatic Join/Cut switching (until the user picks an operation)
    auto_op: Option<Operation>,
    /// guard against re-expanding text picks
    expanding: bool,
    /// Measure: picked things (max 2)
    measure: Vec<Hit>,
    /// Plane: which variant new planes use
    plane_kind: PlaneKind,
    /// Joint: first pick (on the moving part) until the second is made
    joint_a: Option<JointRef>,
    /// Measure: results for the current picks (computed on pick)
    measure_res: Option<MeasureResult>,
}

#[derive(Clone, Debug, Default)]
pub struct MeasureResult {
    /// (label, value) rows for the panel
    pub rows: Vec<(String, String)>,
    pub distance: f64,
    /// closest points between the two picks
    pub points: Option<(Vec3, Vec3)>,
}

fn measure_two(_app: &App, c: &Command) -> Option<MeasureResult> {
    c.measure_res.clone().filter(|m| m.points.is_some())
}

fn pick_shape(st: &ModelState, p: Pick) -> Option<lusion_kernel::Shape> {
    match p {
        Pick::Face { body, face } => st.bodies.get(&body)?.shape.faces().get(face as usize).cloned(),
        Pick::Edge { body, edge } => st.bodies.get(&body)?.shape.edges().get(edge as usize).cloned(),
        Pick::Body(b) => st.bodies.get(&b).filter(|b| !b.is_mesh()).map(|b| b.shape.clone()),
        _ => None,
    }
}

fn body_of_pick(p: Pick) -> Option<BodyId> {
    match p {
        Pick::Face { body, .. } | Pick::Edge { body, .. } | Pick::Body(body) => Some(body),
        _ => None,
    }
}

fn compute_measure(app: &App, st: &ModelState, picks: &[Hit]) -> MeasureResult {
    let f = |v: f64| lusion_core::expr::fmt_num((v * 1000.0).round() / 1000.0);
    let mut r = MeasureResult::default();
    let describe = |r: &mut MeasureResult, p: Pick, prefix: &str| {
        match p {
            Pick::Face { body, face } => {
                if let Some(i) = st.bodies.get(&body).and_then(|b| b.shape.faces().get(face as usize).and_then(|x| x.face_info().ok())) {
                    use lusion_kernel::SurfaceKind as K;
                    let kind = match i.kind {
                        K::Plane => "Planar face".to_string(),
                        K::Cylinder => format!("Cylinder  Ø {} (R {})", f(i.radius * 2.0), f(i.radius)),
                        K::Cone => "Conical face".into(),
                        K::Sphere => format!("Sphere  Ø {}", f(i.radius * 2.0)),
                        K::Torus => "Toroidal face".into(),
                        _ => "Freeform face".into(),
                    };
                    r.rows.push((format!("{prefix}Face"), kind));
                    r.rows.push((format!("{prefix}Area"), format!("{} mm²", f(i.area))));
                    if i.kind == K::Plane {
                        r.rows.push((format!("{prefix}Normal"), format!("{}, {}, {}", f(i.normal.x), f(i.normal.y), f(i.normal.z))));
                    }
                }
            }
            Pick::Edge { body, edge } => {
                if let Some(i) = st.bodies.get(&body).and_then(|b| b.shape.edges().get(edge as usize).and_then(|x| x.edge_info().ok())) {
                    use lusion_kernel::CurveKind as K;
                    let kind = match i.kind {
                        K::Line => "Straight edge".to_string(),
                        K::Circle => format!("{}  Ø {} (R {})", if i.closed { "Circle" } else { "Arc" }, f(i.radius * 2.0), f(i.radius)),
                        _ => "Curved edge".into(),
                    };
                    r.rows.push((format!("{prefix}Edge"), kind));
                    r.rows.push((format!("{prefix}Length"), format!("{} mm", f(i.length))));
                }
            }
            _ => {}
        }
    };
    match picks {
        [a] => {
            describe(&mut r, a.pick, "");
            if let Some(b) = body_of_pick(a.pick).and_then(|b| st.bodies.get(&b)) {
                r.rows.push(("Body".into(), app.body_name(b.id)));
                if let Some((v, area, c)) = b.shape.props() {
                    r.rows.push(("Volume".into(), format!("{} cm³", f(v / 1000.0))));
                    r.rows.push(("Surface".into(), format!("{} cm²", f(area / 100.0))));
                    r.rows.push(("Mass (PLA, solid)".into(), format!("≈ {} g", f(v / 1000.0 * 1.24))));
                    r.rows.push(("Centre of mass".into(), format!("{}, {}, {}", f(c.x), f(c.y), f(c.z))));
                }
                if let Some((lo, hi)) = b.shape.bbox() {
                    let d = hi - lo;
                    r.rows.push(("Size (X × Y × Z)".into(), format!("{} × {} × {} mm", f(d.x), f(d.y), f(d.z))));
                }
            }
            r.rows.push(("Point".into(), format!("{}, {}, {}", f(a.point.x as f64), f(a.point.y as f64), f(a.point.z as f64))));
        }
        [a, b] => {
            if let (Some(sa), Some(sb)) = (pick_shape(st, a.pick), pick_shape(st, b.pick)) {
                if let Some((d, pa, pb)) = lusion_kernel::distance(&sa, &sb) {
                    r.distance = d;
                    r.points = Some((pa.as_vec3(), pb.as_vec3()));
                    r.rows.push(("Distance".into(), format!("{} mm", f(d))));
                    let dv = pb - pa;
                    r.rows.push(("ΔX / ΔY / ΔZ".into(), format!("{} / {} / {}", f(dv.x.abs()), f(dv.y.abs()), f(dv.z.abs()))));
                }
                // angle between planes / straight edges / plane and edge
                let dir = |s: &lusion_kernel::Shape| -> Option<(glam::DVec3, bool)> {
                    if let Ok(i) = s.face_info() {
                        return (i.kind == lusion_kernel::SurfaceKind::Plane).then_some((i.normal, true));
                    }
                    let i = s.edge_info().ok()?;
                    (i.kind == lusion_kernel::CurveKind::Line).then(|| ((i.end - i.start).normalize(), false))
                };
                if let (Some((da, pa)), Some((db, pb))) = (dir(&sa), dir(&sb)) {
                    let mut ang = da.angle_between(db).to_degrees();
                    if pa != pb {
                        ang = (90.0 - ang).abs();
                    } else if ang > 90.0 {
                        ang = 180.0 - ang;
                    }
                    r.rows.push(("Angle".into(), format!("{}°", f(ang))));
                    if pa && pb && ang < 1e-6 {
                        if let (Ok(ia), Ok(ib)) = (sa.face_info(), sb.face_info()) {
                            r.rows.push(("Plane gap".into(), format!("{} mm", f(ia.normal.dot(ib.center - ia.center).abs()))));
                        }
                    }
                }
            }
            if r.rows.is_empty() {
                let d = a.point.distance(b.point) as f64;
                r.distance = d;
                r.points = Some((a.point, b.point));
                r.rows.push(("Distance (points)".into(), format!("{} mm", f(d))));
            }
        }
        _ => {}
    }
    r
}

// ------------------------------------------------------------------ manipulators

#[derive(Clone, Copy, Debug)]
enum GKind {
    /// arrow from `base` to `tip`; values measured along `dir` from `origin`
    Linear { origin: Vec3, dir: Vec3, base: Vec3, tip: Vec3 },
    /// rotation ring
    Ring { center: Vec3, normal: Vec3, radius: f32 },
}

#[derive(Clone, Debug)]
struct Handle {
    id: u8,
    kind: GKind,
    value: f64,
    color: [f32; 4],
    label: Option<String>,
}

#[derive(Clone, Copy, Debug)]
struct GDrag {
    id: u8,
    kind: GKind,
    start_value: f64,
    start_param: f64,
}

const AXIS_COLORS: [[f32; 4]; 3] = [[0.92, 0.32, 0.32, 1.0], [0.36, 0.82, 0.38, 1.0], [0.35, 0.58, 1.0, 1.0]];
const HANDLE_BLUE: [f32; 4] = [0.25, 0.65, 1.0, 1.0];

/// World size of one screen pixel at `p`.
fn px_at(app: &App, view: &View, p: Vec3) -> f32 {
    let c = &app.cam.cam;
    c.px_size(view.size.y) * if c.ortho { 1.0 } else { (p - c.eye()).length() / c.distance.max(1e-6) }
}

/// Snap to a "nice" step (1/2/5 x 10^n) worth ~3 pixels.
fn nice_snap(v: f64, px: f32) -> f64 {
    let raw = (px as f64 * 3.0).max(1e-4);
    let mag = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag);
    (v / step).round() * step
}

/// Parameter along line (origin, dir) closest to the mouse ray.
fn line_param(ray: &crate::camera::Ray, origin: Vec3, dir: Vec3) -> Option<f64> {
    let w0 = origin - ray.o;
    let (a, b, c, d, e) = (dir.dot(dir), dir.dot(ray.d), ray.d.dot(ray.d), dir.dot(w0), ray.d.dot(w0));
    let den = a * c - b * b;
    (den.abs() > 1e-9).then(|| ((b * e - c * d) / den) as f64)
}

/// Angle (degrees) of the mouse ray's hit on a ring's plane.
fn ring_angle(ray: &crate::camera::Ray, center: Vec3, normal: Vec3) -> Option<f64> {
    let t = ray.plane(center, normal)?;
    let v = ray.at(t) - center;
    let u = normal.any_orthonormal_vector();
    let w = normal.cross(u);
    Some((v.dot(w).atan2(v.dot(u)) as f64).to_degrees())
}

fn push_cone(scene: &mut Scene, app: &App, base: Vec3, dir: Vec3, h: f32, r: f32, color: [f32; 4]) {
    let apex = base + dir * h;
    let u = dir.any_orthonormal_vector();
    let v = dir.cross(u);
    let segs = 18;
    for i in 0..segs {
        let t0 = i as f32 / segs as f32 * std::f32::consts::TAU;
        let t1 = (i + 1) as f32 / segs as f32 * std::f32::consts::TAU;
        let p0 = base + (u * t0.cos() + v * t0.sin()) * r;
        let p1 = base + (u * t1.cos() + v * t1.sin()) * r;
        let side = (u * t0.cos() + v * t0.sin()).dot(-app.cam.cam.forward()).max(0.0);
        let c = [color[0] * (0.75 + 0.25 * side), color[1] * (0.75 + 0.25 * side), color[2] * (0.75 + 0.25 * side), 0.97];
        for p in [apex, p0, p1] {
            scene.overlay_fills.push(crate::render::FillVert { p: p.to_array(), color: c });
        }
        for p in [base, p1, p0] {
            scene.overlay_fills.push(crate::render::FillVert { p: p.to_array(), color: [color[0] * 0.5, color[1] * 0.5, color[2] * 0.5, 0.97] });
        }
    }
}

/// Outward direction at an edge: bisector of the adjacent faces' normals near `mid`.
fn edge_outward(app: &mut App, st: &ModelState, body: BodyId, edge: usize, mid: Vec3) -> Vec3 {
    let Some(b) = st.bodies.get(&body) else { return Vec3::Z };
    let Some((f1, f2)) = b.shape.edge_faces().get(edge).copied() else { return Vec3::Z };
    let Some(mesh) = app.meshes.get(b) else { return Vec3::Z };
    let normal_near = |f: i32| -> Option<Vec3> {
        if f < 0 {
            return None;
        }
        let r = crate::viewport::face_range(&mesh, f as u32);
        let mut best: Option<(f32, Vec3)> = None;
        for t in r {
            for k in 0..3 {
                let vi = mesh.indices[3 * t + k] as usize;
                let d = (Vec3::from_array(mesh.positions[vi]) - mid).length_squared();
                if best.map(|b| d < b.0).unwrap_or(true) {
                    best = Some((d, Vec3::from_array(mesh.normals[vi])));
                }
            }
        }
        best.map(|b| b.1)
    };
    let n = normal_near(f1).unwrap_or(Vec3::ZERO) + normal_near(f2).unwrap_or(Vec3::ZERO);
    if n.length() < 1e-6 {
        Vec3::Z
    } else {
        n.normalize()
    }
}

// ------------------------------------------------------------------ refs <-> picks

pub fn profile_ref(st: &ModelState, sketch: FeatureId, index: usize) -> Option<ProfileRef> {
    let p = st.sketches.get(&sketch)?.profiles.get(index)?;
    Some(ProfileRef::Sketch { sketch, curves: p.curves.clone(), hint: p.hint.to_array() })
}

pub fn face_ref(st: &ModelState, body: BodyId, face: u32) -> Option<FaceRef> {
    let b = st.bodies.get(&body)?;
    let name = b.face_names.get(face as usize)?.clone();
    let hint = b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok()).map(|i| i.center).unwrap_or_default();
    Some(FaceRef { body, name, hint: hint.to_array() })
}

pub fn edge_ref(st: &ModelState, body: BodyId, edge: u32) -> Option<EdgeRef> {
    let b = st.bodies.get(&body)?;
    let name = b.edge_names.get(edge as usize)?.clone();
    let hint = b.shape.edges().get(edge as usize).and_then(|e| e.edge_info().ok()).map(|i| i.mid).unwrap_or_default();
    Some(EdgeRef { body, name, hint: hint.to_array() })
}

fn face_pick(st: &ModelState, r: &FaceRef) -> Option<Pick> {
    let b = st.bodies.get(&r.body)?;
    Some(Pick::Face { body: r.body, face: b.face_index(&r.name)? as u32 })
}

fn edge_pick(st: &ModelState, r: &EdgeRef) -> Option<Pick> {
    let b = st.bodies.get(&r.body)?;
    Some(Pick::Edge { body: r.body, edge: b.edge_index(&r.name)? as u32 })
}

fn profile_pick(st: &ModelState, r: &ProfileRef) -> Option<Pick> {
    match r {
        ProfileRef::Sketch { sketch, curves, hint } => {
            let ss = st.sketches.get(sketch)?;
            let p = find_profile(ss, curves, glam::DVec2::from_array(*hint))?;
            let index = ss.profiles.iter().position(|q| std::ptr::eq(q, p))?;
            Some(Pick::Profile { sketch: *sketch, index })
        }
        ProfileRef::Face(f) => face_pick(st, f),
    }
}

fn same_profile(a: &ProfileRef, b: &ProfileRef) -> bool {
    match (a, b) {
        (ProfileRef::Sketch { sketch: s1, curves: c1, hint: h1 }, ProfileRef::Sketch { sketch: s2, curves: c2, hint: h2 }) => {
            s1 == s2 && c1 == c2 && (glam::DVec2::from_array(*h1) - glam::DVec2::from_array(*h2)).length() < 1e-6
        }
        (ProfileRef::Face(a), ProfileRef::Face(b)) => a.body == b.body && a.name == b.name,
        _ => false,
    }
}

/// Edges connected to `start` through tangent-continuous joints (by polyline end directions).
fn tangent_chain(m: &lusion_core::tess::BodyMesh, start: usize) -> Vec<usize> {
    use glam::Vec3 as V;
    let ends = |i: usize| -> Option<[(V, V); 2]> {
        let p = m.edges.get(i)?;
        if p.len() < 2 {
            return None;
        }
        let a = V::from_array(p[0]);
        let a1 = V::from_array(p[1]);
        let b = V::from_array(p[p.len() - 1]);
        let b1 = V::from_array(p[p.len() - 2]);
        // (position, outward direction at that end)
        Some([(a, (a - a1).normalize_or_zero()), (b, (b - b1).normalize_or_zero())])
    };
    let scale = V::from_array(m.bbox.1).distance(V::from_array(m.bbox.0)).max(1e-3);
    let tol = scale * 1e-5;
    let mut chain = vec![start];
    let mut stack = vec![start];
    while let Some(e) = stack.pop() {
        let Some(ee) = ends(e) else { continue };
        for (p, d) in ee {
            for j in 0..m.edges.len() {
                if chain.contains(&j) {
                    continue;
                }
                let Some(ej) = ends(j) else { continue };
                for (q, dq) in ej {
                    // joined end-to-end and continuing in the same direction
                    if p.distance(q) < tol && d.dot(-dq) > 0.985 {
                        chain.push(j);
                        stack.push(j);
                    }
                }
            }
        }
        if chain.len() > 500 {
            break;
        }
    }
    chain
}

// ------------------------------------------------------------------ lifecycle

pub fn start(app: &mut App, kind: CmdKind) {
    if app.cmd.is_some() {
        cancel(app);
    }
    if let Mode::Sketch(_) = app.mode {
        crate::sketch_mode::finish(app);
    }
    let mut c = Command { kind, fid: None, editing: false, before: app.doc.clone(), prev_marker: app.doc.marker, stage: 0, gdrag: None, ghover: None, auto_op: None, expanding: false, measure: vec![], plane_kind: PlaneKind::Offset, joint_a: None, measure_res: None };
    let sel = std::mem::take(&mut app.selection);
    app.cmd = None;
    // pre-selection
    match kind {
        CmdKind::NewSketch => {
            if let Some(p) = sel.iter().find(|p| matches!(p, Pick::Face { .. } | Pick::Origin(_) | Pick::Plane(_))) {
                new_sketch_on(app, *p);
                return;
            }
        }
        _ => {
            for p in &sel {
                c.on_pick(app, Hit { pick: *p, point: Vec3::ZERO, depth: 0.0 });
            }
        }
    }
    if kind == CmdKind::Section && app.section.is_none() {
        // cut away the half facing the camera, through the middle of the model
        let (lo, hi) = app.scene_bounds().unwrap_or((Vec3::splat(-10.0), Vec3::splat(10.0)));
        let toward = -app.cam.cam.forward();
        let axes = [Vec3::X, Vec3::Y, Vec3::Z, Vec3::NEG_X, Vec3::NEG_Y, Vec3::NEG_Z];
        let n = axes.into_iter().max_by(|a, b| a.dot(toward).total_cmp(&b.dot(toward))).unwrap();
        app.section = Some(crate::app::SectionView { normal: n, origin: (lo + hi) * 0.5, offset: 0.0 });
    }
    app.cmd = Some(c);
}

/// ISO metric coarse pitches.
pub const ISO_COARSE: [(f64, f64); 14] =
    [(1.6, 0.35), (2.0, 0.4), (2.5, 0.45), (3.0, 0.5), (4.0, 0.7), (5.0, 0.8), (6.0, 1.0), (8.0, 1.25), (10.0, 1.5), (12.0, 1.75), (14.0, 2.0), (16.0, 2.0), (20.0, 2.5), (24.0, 3.0)];

/// Guess the thread for a cylinder of radius `r` (external: the rod, internal: the tap-drilled hole).
fn guess_thread(r: f64, external: bool) -> (f64, f64) {
    let score = |(d, p): (f64, f64)| if external { (d - 2.0 * r).abs() } else { (d - 1.0825 * p - 2.0 * r).abs() };
    ISO_COARSE.into_iter().min_by(|a, b| score(*a).total_cmp(&score(*b))).unwrap_or((6.0, 1.0))
}

/// Hole presets for printed parts: (label, kind, diameter, cb/cs diameter, cb depth or cs angle).
pub fn hole_presets() -> Vec<(String, HoleKind, f64, f64, f64)> {
    let sizes: [(&str, f64, f64, f64, f64, f64, f64, f64); 7] = [
        // name, clearance, tap, socket head cb Ø, cb depth, csk Ø, insert Ø, insert depth
        ("M2", 2.4, 1.6, 4.5, 2.5, 4.4, 3.2, 3.5),
        ("M2.5", 2.9, 2.05, 5.5, 3.0, 5.4, 3.6, 4.0),
        ("M3", 3.4, 2.5, 6.5, 3.5, 6.5, 4.2, 5.0),
        ("M4", 4.5, 3.3, 8.0, 4.5, 8.6, 5.6, 6.0),
        ("M5", 5.5, 4.2, 9.5, 5.5, 10.5, 6.4, 7.0),
        ("M6", 6.6, 5.0, 11.0, 6.5, 12.5, 8.0, 9.0),
        ("M8", 9.0, 6.8, 14.0, 8.5, 16.5, 9.6, 11.0),
    ];
    let mut v = Vec::new();
    for (n, clr, tap, cb, cbd, cs, ins, insd) in sizes {
        v.push((format!("{n} clearance"), HoleKind::Simple, clr, 0.0, 0.0));
        v.push((format!("{n} socket head (counterbore)"), HoleKind::Counterbore, clr, cb, cbd));
        v.push((format!("{n} flat head (countersink)"), HoleKind::Countersink, clr, cs, 90.0));
        v.push((format!("{n} tap drill (for a thread)"), HoleKind::Simple, tap, 0.0, 0.0));
        v.push((format!("{n} heat-set insert"), HoleKind::Simple, ins, 0.0, insd));
    }
    v
}

fn plane_ref_of(st: &ModelState, pick: Pick) -> Option<PlaneRef> {
    match pick {
        Pick::Origin(o) => Some(PlaneRef::Origin(o)),
        Pick::Plane(f) => Some(PlaneRef::Plane(f)),
        Pick::Face { body, face } => face_ref(st, body, face).map(PlaneRef::Face),
        _ => None,
    }
}

fn axis_ref_of(st: &ModelState, pick: Pick) -> Option<AxisRef> {
    match pick {
        Pick::Axis(0) => Some(AxisRef::X),
        Pick::Axis(1) => Some(AxisRef::Y),
        Pick::Axis(_) => Some(AxisRef::Z),
        Pick::SketchLine { sketch, curve } => Some(AxisRef::SketchLine { sketch, curve }),
        Pick::Edge { body, edge } => edge_ref(st, body, edge).map(AxisRef::Edge),
        _ => None,
    }
}

/// World plane (point, normal) of a plane-like pick.
fn plane_of_pick(app: &App, st: &ModelState, pick: Pick) -> Option<(Vec3, Vec3)> {
    match pick {
        Pick::Origin(o) => Some(match o {
            OriginPlane::XY => (Vec3::ZERO, Vec3::Z),
            OriginPlane::XZ => (Vec3::ZERO, Vec3::NEG_Y),
            OriginPlane::YZ => (Vec3::ZERO, Vec3::X),
        }),
        Pick::Plane(f) => st.planes.get(&f).map(|fr| (fr.origin.as_vec3(), fr.n.as_vec3())),
        Pick::Face { body, face } => {
            let i = st.bodies.get(&body)?.shape.faces().get(face as usize)?.face_info().ok()?;
            let _ = app;
            (i.kind == lusion_kernel::SurfaceKind::Plane).then(|| (i.center.as_vec3(), i.normal.as_vec3()))
        }
        _ => None,
    }
}

/// Snap a click on a planar face to circle centres of its edges or the face centre.
fn snap_on_face(app: &App, st: &ModelState, body: BodyId, face: u32, p: Vec3) -> Vec3 {
    let tol = app.cam.cam.distance * 0.02;
    let Some(b) = st.bodies.get(&body) else { return p };
    let Some(f) = b.shape.faces().get(face as usize).cloned() else { return p };
    let mut cands: Vec<Vec3> = f.edges().iter().filter_map(|e| e.edge_info().ok()).filter(|i| i.kind == lusion_kernel::CurveKind::Circle).map(|i| i.center.as_vec3()).collect();
    if let Ok(i) = f.face_info() {
        cands.push(i.center.as_vec3());
    }
    cands.into_iter().filter(|c| c.distance(p) < tol).min_by(|a, b| a.distance(p).total_cmp(&b.distance(p))).unwrap_or(p)
}

pub fn edit(app: &mut App, fid: FeatureId) {
    let Some(idx) = app.doc.feature_index(fid) else { return };
    let kind = match &app.doc.features[idx].kind {
        FeatureKind::Sketch(_) => {
            crate::sketch_mode::enter(app, fid);
            return;
        }
        FeatureKind::Extrude(_) => CmdKind::Extrude,
        FeatureKind::Revolve(_) => CmdKind::Revolve,
        FeatureKind::Fillet(_) => CmdKind::Fillet,
        FeatureKind::Chamfer(_) => CmdKind::Chamfer,
        FeatureKind::Shell(_) => CmdKind::Shell,
        FeatureKind::Plane(_) => CmdKind::Plane,
        FeatureKind::Combine(_) => CmdKind::Combine,
        FeatureKind::Move(_) => CmdKind::Move,
        FeatureKind::Pattern(_) => CmdKind::Pattern,
        FeatureKind::Mirror(_) => CmdKind::Mirror,
        FeatureKind::Split(_) => CmdKind::Split,
        FeatureKind::Hole(_) => CmdKind::Hole,
        FeatureKind::Thread(_) => CmdKind::Thread,
        FeatureKind::Sweep(_) => CmdKind::Sweep,
        FeatureKind::Loft(_) => CmdKind::Loft,
        FeatureKind::Joint(_) => CmdKind::Joint,
        _ => {
            app.info("This feature has no editable parameters");
            return;
        }
    };
    if app.cmd.is_some() {
        cancel(app);
    }
    if let Mode::Sketch(_) = app.mode {
        crate::sketch_mode::finish(app);
    }
    let before = app.doc.clone();
    let prev_marker = app.doc.marker;
    app.doc.marker = idx + 1;
    app.selection.clear();
    let plane_kind = match &app.doc.features[idx].kind {
        FeatureKind::Plane(p) => match p.mode {
            PlaneMode::Offset => PlaneKind::Offset,
            PlaneMode::Angle { .. } => PlaneKind::Angle,
            PlaneMode::Midplane { .. } => PlaneKind::Midplane,
            PlaneMode::NormalToEdge { .. } => PlaneKind::NormalToEdge,
        },
        _ => PlaneKind::Offset,
    };
    app.cmd = Some(Command { kind, fid: Some(fid), editing: true, before, prev_marker, stage: 0, gdrag: None, ghover: None, auto_op: None, expanding: false, measure: vec![], plane_kind, joint_a: None, measure_res: None });
}

pub fn ok(app: &mut App) {
    let Some(c) = app.cmd.take() else { return };
    if c.editing && c.prev_marker > app.doc.marker {
        app.doc.marker = c.prev_marker;
    }
    if !c.editing {
        // like Fusion: sketches consumed by a feature get hidden
        let used: Vec<FeatureId> = match c.fid.and_then(|f| app.doc.feature(f)).map(|f| &f.kind) {
            Some(FeatureKind::Extrude(e)) => e.profiles.iter().filter_map(sketch_of).collect(),
            Some(FeatureKind::Revolve(r)) => r.profiles.iter().filter_map(sketch_of).collect(),
            _ => vec![],
        };
        for sid in used {
            if let Some(f) = app.doc.feature_mut(sid) {
                f.visible = false;
            }
        }
    }
    app.commit(c.before);
}

fn sketch_of(p: &ProfileRef) -> Option<FeatureId> {
    match p {
        ProfileRef::Sketch { sketch, .. } => Some(*sketch),
        _ => None,
    }
}

/// If we look straight down the extrude direction, tilt the camera so the
/// extrusion is visible.
fn tilt_view_for(app: &mut App, n: DVec3) {
    let cam = app.cam.cam;
    let fwd = cam.forward();
    if fwd.dot(n.as_vec3()).abs() < 0.92 {
        return;
    }
    let nf = (fwd + cam.up() * 0.85 + cam.right() * 0.55).normalize();
    let world_up = if nf.cross(Vec3::Z).length() > 0.2 { Vec3::Z } else { cam.up() };
    let right = nf.cross(world_up).normalize();
    let up = right.cross(nf);
    let rot = glam::Quat::from_mat3(&glam::Mat3::from_cols(right, up, -nf)).normalize();
    app.cam.set_orientation(rot);
}

pub fn cancel(app: &mut App) {
    if let Some(c) = app.cmd.take() {
        app.doc = c.before;
    }
}

/// Start a sketch on a plane/face pick and enter sketch mode.
pub fn new_sketch_on(app: &mut App, pick: Pick) {
    let st = app.pick_state().clone();
    if app.cmd.is_some() {
        cancel(app);
    }
    let plane = match pick {
        Pick::Origin(p) => PlaneRef::Origin(p),
        Pick::Plane(fid) => PlaneRef::Plane(fid),
        Pick::Face { body, face } => {
            let Some(fr) = face_ref(&st, body, face) else { return };
            let planar = st
                .bodies
                .get(&body)
                .and_then(|b| b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok()))
                .map(|i| i.kind == lusion_kernel::SurfaceKind::Plane)
                .unwrap_or(false);
            if !planar {
                app.error("Sketches need a planar face");
                return;
            }
            PlaneRef::Face(fr)
        }
        _ => return,
    };
    app.cmd = None;
    let before = app.doc.clone();
    let mut sk = Sketch::new(plane);
    // the projected origin, fixed
    let o = sk.add_point(glam::DVec2::ZERO);
    sk.point_mut(o).unwrap().fixed = true;
    let name = app.doc.next_name("Sketch");
    let comp = app.active_comp;
    let fid = app.doc.insert_feature(name, comp, FeatureKind::Sketch(sk));
    app.model.update(&app.doc);
    crate::sketch_mode::enter_with_before(app, fid, before, true);
}

// ------------------------------------------------------------------ behaviour

impl Command {
    /// Model state before this command's feature (what the user picks from).
    /// Use this instead of `self.pre_state(app)`: the command is taken out of
    /// `app` while its methods run.
    fn pre_state<'a>(&self, app: &'a App) -> &'a ModelState {
        match self.fid.and_then(|f| app.doc.feature_index(f)) {
            Some(i) => app.model.state_after(i),
            None => app.model.current(&app.doc),
        }
    }

    pub fn wants_planes(&self) -> bool {
        matches!(self.kind, CmdKind::NewSketch | CmdKind::Plane)
            || (matches!(self.kind, CmdKind::Mirror | CmdKind::Split) && self.stage == 1)
    }

    pub fn filter(&self) -> Filter {
        let mut f = Filter::default();
        match self.kind {
            CmdKind::NewSketch => {
                f.faces = true;
                f.planar_only = true;
                f.planes = true;
            }
            CmdKind::Extrude => {
                f.profiles = true;
                f.faces = true;
                f.planar_only = true;
            }
            CmdKind::Revolve => {
                if self.stage == 0 {
                    f.profiles = true;
                    f.faces = true;
                    f.planar_only = true;
                } else {
                    f.sketch_lines = true;
                    f.edges = true;
                    f.axes = true;
                }
            }
            CmdKind::Fillet | CmdKind::Chamfer => {
                f.edges = true;
                f.faces = true;
            }
            CmdKind::Shell => {
                f.faces = true;
            }
            CmdKind::Plane => match self.plane_kind {
                PlaneKind::NormalToEdge => f.edges = true,
                PlaneKind::Angle => {
                    f.faces = true;
                    f.planar_only = true;
                    f.planes = true;
                    f.edges = true;
                    f.sketch_lines = true;
                    f.axes = true;
                }
                _ => {
                    f.faces = true;
                    f.planar_only = true;
                    f.planes = true;
                }
            },
            CmdKind::Split => {
                if self.stage == 0 {
                    f.bodies = true;
                } else {
                    f.faces = true;
                    f.planar_only = true;
                    f.planes = true;
                }
            }
            CmdKind::Hole => {
                f.faces = true;
                f.planar_only = true;
            }
            CmdKind::Thread => f.faces = true,
            CmdKind::Sweep => {
                if self.stage == 0 {
                    f.profiles = true;
                    f.faces = true;
                    f.planar_only = true;
                } else {
                    f.edges = true;
                    f.sketch_curves = true;
                }
            }
            CmdKind::Loft => {
                f.profiles = true;
                f.faces = true;
                f.planar_only = true;
            }
            CmdKind::Joint => {
                f.faces = true;
                f.edges = true;
            }
            CmdKind::Measure => {
                f.faces = true;
                f.edges = true;
                f.bodies = true;
            }
            CmdKind::Section => {
                f.faces = true;
                f.planar_only = true;
                f.planes = true;
            }
            CmdKind::Combine => {
                f.bodies = true;
            }
            CmdKind::Move => match self.stage {
                1 => {
                    f.edges = true;
                    f.sketch_lines = true;
                    f.axes = true;
                }
                2 => {
                    f.faces = true;
                    f.edges = true;
                }
                _ => f.bodies = true,
            },
            CmdKind::Pattern => {
                if self.stage == 0 {
                    f.bodies = true;
                } else {
                    f.edges = true;
                    f.sketch_lines = true;
                    f.axes = true;
                }
            }
            CmdKind::Mirror => {
                if self.stage == 0 {
                    f.bodies = true;
                } else {
                    f.faces = true;
                    f.planar_only = true;
                    f.planes = true;
                }
            }
        }
        f
    }

    fn feature<'a>(&self, app: &'a App) -> Option<&'a FeatureKind> {
        self.fid.and_then(|f| app.doc.feature(f)).map(|f| &f.kind)
    }

    fn feature_mut<'a>(&self, app: &'a mut App) -> Option<&'a mut FeatureKind> {
        self.fid.and_then(|f| app.doc.feature_mut(f)).map(|f| &mut f.kind)
    }

    fn insert(&mut self, app: &mut App, kind: FeatureKind) {
        let base = match &kind {
            FeatureKind::Plane(_) => "Plane",
            k => k.type_name(),
        };
        let name = app.doc.next_name(base);
        let comp = app.active_comp;
        self.fid = Some(app.doc.insert_feature(name, comp, kind));
    }

    /// Picks that show as selected while the command runs.
    pub fn selected_picks(&self, app: &App) -> Vec<Pick> {
        let st = self.pre_state(app);
        if self.kind == CmdKind::Measure {
            return self.measure.iter().map(|h| h.pick).collect();
        }
        let joint_pick = |r: &JointRef| match r {
            JointRef::Face(f) => face_pick(st, f),
            JointRef::Edge(e) => edge_pick(st, e),
        };
        let Some(k) = self.feature(app) else {
            return self.joint_a.as_ref().and_then(joint_pick).into_iter().collect();
        };
        let mut v = Vec::new();
        let plane_pick = |p: &PlaneRef| match p {
            PlaneRef::Origin(o) => Some(Pick::Origin(*o)),
            PlaneRef::Plane(f) => Some(Pick::Plane(*f)),
            PlaneRef::Face(f) => face_pick(st, f),
        };
        let axis_pick = |a: &AxisRef| match a {
            AxisRef::X => Some(Pick::Axis(0)),
            AxisRef::Y => Some(Pick::Axis(1)),
            AxisRef::Z => Some(Pick::Axis(2)),
            AxisRef::SketchLine { sketch, curve } => Some(Pick::SketchLine { sketch: *sketch, curve: *curve }),
            AxisRef::Edge(e) => edge_pick(st, e),
        };
        match k {
            FeatureKind::Plane(p) if p.mode != PlaneMode::Offset => match &p.mode {
                PlaneMode::Angle { axis, .. } => {
                    v.extend(plane_pick(&p.base));
                    v.extend(axis_pick(axis));
                }
                PlaneMode::Midplane { other } => {
                    v.extend(plane_pick(&p.base));
                    v.extend(plane_pick(other));
                }
                PlaneMode::NormalToEdge { edge, .. } => v.extend(edge_pick(st, edge)),
                PlaneMode::Offset => {}
            },
            FeatureKind::Split(sp) => {
                v.extend(sp.bodies.iter().map(|b| Pick::Body(*b)));
                v.extend(plane_pick(&sp.tool));
            }
            FeatureKind::Hole(h) => v.extend(face_pick(st, &h.face)),
            FeatureKind::Thread(t) => v.extend(face_pick(st, &t.face)),
            FeatureKind::Sweep(sw) => {
                v.extend(sw.profiles.iter().filter_map(|p| profile_pick(st, p)));
                for p in &sw.path {
                    match p {
                        PathRef::SketchCurve { sketch, curve } => v.push(Pick::SketchLine { sketch: *sketch, curve: *curve }),
                        PathRef::Edge(e) => v.extend(edge_pick(st, e)),
                    }
                }
            }
            FeatureKind::Loft(l) => v.extend(l.profiles.iter().filter_map(|p| profile_pick(st, p))),
            FeatureKind::Joint(j) => {
                v.extend(joint_pick(&j.a));
                v.extend(joint_pick(&j.b));
            }
            FeatureKind::Extrude(e) => v.extend(e.profiles.iter().filter_map(|p| profile_pick(st, p))),
            FeatureKind::Revolve(r) => {
                v.extend(r.profiles.iter().filter_map(|p| profile_pick(st, p)));
                match &r.axis {
                    AxisRef::X => v.push(Pick::Axis(0)),
                    AxisRef::Y => v.push(Pick::Axis(1)),
                    AxisRef::Z => v.push(Pick::Axis(2)),
                    AxisRef::SketchLine { sketch, curve } => v.push(Pick::SketchLine { sketch: *sketch, curve: *curve }),
                    AxisRef::Edge(e) => v.extend(edge_pick(st, e)),
                }
            }
            FeatureKind::Fillet(f) => v.extend(f.edges.iter().filter_map(|e| edge_pick(st, e))),
            FeatureKind::Chamfer(f) => v.extend(f.edges.iter().filter_map(|e| edge_pick(st, e))),
            FeatureKind::Shell(s) => v.extend(s.faces.iter().filter_map(|f| face_pick(st, f))),
            FeatureKind::Plane(p) => match &p.base {
                PlaneRef::Origin(o) => v.push(Pick::Origin(*o)),
                PlaneRef::Plane(f) => v.push(Pick::Plane(*f)),
                PlaneRef::Face(f) => v.extend(face_pick(st, f)),
            },
            FeatureKind::Combine(c) => {
                v.extend(c.target.map(Pick::Body));
                v.extend(c.tools.iter().map(|b| Pick::Body(*b)));
            }
            FeatureKind::Move(m) => v.extend(m.bodies.iter().map(|b| Pick::Body(*b))),
            FeatureKind::Pattern(p) => {
                v.extend(p.bodies.iter().map(|b| Pick::Body(*b)));
                match &p.axis {
                    AxisRef::X => v.push(Pick::Axis(0)),
                    AxisRef::Y => v.push(Pick::Axis(1)),
                    AxisRef::Z => v.push(Pick::Axis(2)),
                    AxisRef::SketchLine { sketch, curve } => v.push(Pick::SketchLine { sketch: *sketch, curve: *curve }),
                    AxisRef::Edge(e) => v.extend(edge_pick(st, e)),
                }
            }
            FeatureKind::Mirror(m) => {
                v.extend(m.bodies.iter().map(|b| Pick::Body(*b)));
                match &m.plane {
                    PlaneRef::Origin(o) => v.push(Pick::Origin(*o)),
                    PlaneRef::Plane(f) => v.push(Pick::Plane(*f)),
                    PlaneRef::Face(f) => v.extend(face_pick(st, f)),
                }
            }
            _ => {}
        }
        v
    }

    pub fn on_pick(&mut self, app: &mut App, hit: Hit) {
        let st = self.pre_state(app).clone();
        match self.kind {
            CmdKind::NewSketch => {
                if matches!(hit.pick, Pick::Face { .. } | Pick::Origin(_) | Pick::Plane(_)) {
                    new_sketch_on(app, hit.pick);
                    // new_sketch_on cleared the command; keep it cleared
                    self.kind = CmdKind::NewSketch;
                    self.stage = 99;
                }
            }
            CmdKind::Extrude | CmdKind::Revolve => {
                if self.kind == CmdKind::Revolve && self.stage == 1 {
                    let axis = match hit.pick {
                        Pick::Axis(0) => Some(AxisRef::X),
                        Pick::Axis(1) => Some(AxisRef::Y),
                        Pick::Axis(_) => Some(AxisRef::Z),
                        Pick::SketchLine { sketch, curve } => Some(AxisRef::SketchLine { sketch, curve }),
                        Pick::Edge { body, edge } => edge_ref(&st, body, edge).map(AxisRef::Edge),
                        _ => None,
                    };
                    if let (Some(a), Some(FeatureKind::Revolve(r))) = (axis, self.feature_mut(app)) {
                        r.axis = a;
                    }
                    return;
                }
                // clicking any letter of a text selects the whole text
                if let (Pick::Profile { sketch, index }, false) = (hit.pick, self.expanding) {
                    if let Some(ss) = st.sketches.get(&sketch) {
                        let Some(p) = ss.profiles.get(index) else { return };
                        let tids: Vec<u32> = ss.sketch.texts.iter().map(|t| t.id).filter(|t| p.curves.contains(t)).collect();
                        if let Some(&tid) = tids.first() {
                            let all = lusion_core::model::text_fill_profiles(ss, tid, &app.model.params);
                            if all.len() > 1 {
                                let refs: Vec<ProfileRef> = all.iter().filter_map(|i| profile_ref(&st, sketch, *i)).collect();
                                let first = refs[0].clone();
                                if self.fid.is_none() {
                                    self.expanding = true;
                                    self.on_pick(app, Hit { pick: Pick::Profile { sketch, index: all[0] }, ..hit });
                                    self.expanding = false;
                                }
                                let list = match self.feature_mut(app) {
                                    Some(FeatureKind::Extrude(e)) => &mut e.profiles,
                                    Some(FeatureKind::Revolve(r)) => &mut r.profiles,
                                    _ => return,
                                };
                                let already = refs.iter().all(|r| list.iter().any(|x| same_profile(x, r)));
                                if already && list.len() > refs.len() {
                                    list.retain(|x| !refs.iter().any(|r| same_profile(x, r)));
                                } else {
                                    for r in refs {
                                        if !list.iter().any(|x| same_profile(x, &r)) {
                                            list.push(r);
                                        }
                                    }
                                }
                                let _ = first;
                                return;
                            }
                        }
                    }
                }
                let pr = match hit.pick {
                    Pick::Profile { sketch, index } => profile_ref(&st, sketch, index),
                    Pick::Face { body, face } => face_ref(&st, body, face).map(ProfileRef::Face),
                    _ => None,
                };
                let Some(pr) = pr else { return };
                if self.fid.is_none() {
                    let is_face = matches!(pr, ProfileRef::Face(_));
                    let on_body = match &pr {
                        ProfileRef::Sketch { sketch, .. } => matches!(
                            app.doc.feature(*sketch).map(|f| &f.kind),
                            Some(FeatureKind::Sketch(Sketch { plane: PlaneRef::Face(_), .. }))
                        ),
                        _ => true,
                    };
                    let op = if is_face || on_body { Operation::Join } else { Operation::NewBody };
                    self.auto_op = Some(op);
                    let kind = if self.kind == CmdKind::Extrude {
                        FeatureKind::Extrude(Extrude {
                            profiles: vec![pr],
                            distance: "10".into(),
                            distance2: "10".into(),
                            mode: ExtentMode::OneSide,
                            start_offset: String::new(),
                            op,
                            targets: vec![],
                        })
                    } else {
                        // default axis: a construction line of the sketch if any, else Z
                        let mut axis = AxisRef::Z;
                        if let ProfileRef::Sketch { sketch, .. } = &pr {
                            if let Some(ss) = st.sketches.get(sketch) {
                                if let Some(c) = ss.sketch.curves.iter().find(|c| c.construction && ss.sketch.line_pts(c.id).is_some()) {
                                    axis = AxisRef::SketchLine { sketch: *sketch, curve: c.id };
                                }
                            }
                        }
                        FeatureKind::Revolve(Revolve { profiles: vec![pr], axis, angle: "360".into(), op, targets: vec![] })
                    };
                    self.insert(app, kind);
                    app.model.update(&app.doc);
                    if let Some((_, n, _)) = self.extrude_axis(app) {
                        tilt_view_for(app, n);
                    }
                    return;
                }
                let list = match self.feature_mut(app) {
                    Some(FeatureKind::Extrude(e)) => &mut e.profiles,
                    Some(FeatureKind::Revolve(r)) => &mut r.profiles,
                    _ => return,
                };
                if let Some(i) = list.iter().position(|p| same_profile(p, &pr)) {
                    list.remove(i);
                } else {
                    list.push(pr);
                }
            }
            CmdKind::Fillet | CmdKind::Chamfer => {
                let mut refs: Vec<EdgeRef> = Vec::new();
                match hit.pick {
                    Pick::Edge { body, edge } => {
                        // Fusion-style: take the whole tangent chain
                        let chain = st.bodies.get(&body).and_then(|b| app.meshes.get(b)).map(|m| tangent_chain(&m, edge as usize)).unwrap_or_else(|| vec![edge as usize]);
                        for e in chain {
                            refs.extend(edge_ref(&st, body, e as u32));
                        }
                    }
                    Pick::Face { body, face } => {
                        // all edges of the face
                        if let Some(b) = st.bodies.get(&body) {
                            let ef = b.shape.edge_faces();
                            for (i, (a, c)) in ef.iter().enumerate() {
                                if *a == face as i32 || *c == face as i32 {
                                    refs.extend(edge_ref(&st, body, i as u32));
                                }
                            }
                        }
                    }
                    _ => {}
                }
                if refs.is_empty() {
                    return;
                }
                if self.fid.is_none() {
                    let kind = if self.kind == CmdKind::Fillet {
                        FeatureKind::Fillet(Fillet { edges: refs, radius: "2".into() })
                    } else {
                        FeatureKind::Chamfer(Chamfer { edges: refs, distance: "1".into() })
                    };
                    self.insert(app, kind);
                    return;
                }
                let list = match self.feature_mut(app) {
                    Some(FeatureKind::Fillet(f)) => &mut f.edges,
                    Some(FeatureKind::Chamfer(f)) => &mut f.edges,
                    _ => return,
                };
                let is_edge_pick = matches!(hit.pick, Pick::Edge { .. });
                let all_present = refs.iter().all(|r| list.iter().any(|e| e.body == r.body && e.name == r.name));
                if is_edge_pick && all_present {
                    list.retain(|e| !refs.iter().any(|r| r.body == e.body && r.name == e.name));
                } else {
                    for r in refs {
                        if !list.iter().any(|e| e.body == r.body && e.name == r.name) {
                            list.push(r);
                        }
                    }
                }
            }
            CmdKind::Shell => {
                let Pick::Face { body, face } = hit.pick else { return };
                let Some(fr) = face_ref(&st, body, face) else { return };
                if self.fid.is_none() {
                    self.insert(app, FeatureKind::Shell(Shell { body: Some(body), faces: vec![fr], thickness: "2".into() }));
                    return;
                }
                if let Some(FeatureKind::Shell(s)) = self.feature_mut(app) {
                    if s.body != Some(body) {
                        s.body = Some(body);
                        s.faces.clear();
                    }
                    if let Some(i) = s.faces.iter().position(|f| f.name == fr.name) {
                        s.faces.remove(i);
                    } else {
                        s.faces.push(fr);
                    }
                }
            }
            CmdKind::Plane if self.plane_kind == PlaneKind::Offset => {
                let base = match hit.pick {
                    Pick::Origin(o) => PlaneRef::Origin(o),
                    Pick::Plane(f) => PlaneRef::Plane(f),
                    Pick::Face { body, face } => match face_ref(&st, body, face) {
                        Some(f) => PlaneRef::Face(f),
                        None => return,
                    },
                    _ => return,
                };
                if self.fid.is_none() {
                    self.insert(app, FeatureKind::Plane(OffsetPlane { base, offset: "10".into(), mode: PlaneMode::Offset }));
                } else if let Some(FeatureKind::Plane(p)) = self.feature_mut(app) {
                    p.base = base;
                }
            }
            CmdKind::Combine => {
                let b = match hit.pick {
                    Pick::Body(b) | Pick::Face { body: b, .. } => b,
                    _ => return,
                };
                if self.fid.is_none() {
                    self.insert(app, FeatureKind::Combine(Combine { target: Some(b), tools: vec![], op: Operation::Join, keep_tools: false }));
                    self.stage = 1;
                    return;
                }
                if let Some(FeatureKind::Combine(c)) = self.feature_mut(app) {
                    if self.stage == 0 {
                        c.target = Some(b);
                        c.tools.retain(|t| *t != b);
                    } else if c.target != Some(b) {
                        if let Some(i) = c.tools.iter().position(|t| *t == b) {
                            c.tools.remove(i);
                        } else {
                            c.tools.push(b);
                        }
                    }
                }
            }
            CmdKind::Pattern | CmdKind::Mirror => {
                if self.stage == 1 {
                    if self.kind == CmdKind::Pattern {
                        let axis = match hit.pick {
                            Pick::Axis(0) => Some(AxisRef::X),
                            Pick::Axis(1) => Some(AxisRef::Y),
                            Pick::Axis(_) => Some(AxisRef::Z),
                            Pick::SketchLine { sketch, curve } => Some(AxisRef::SketchLine { sketch, curve }),
                            Pick::Edge { body, edge } => edge_ref(&st, body, edge).map(AxisRef::Edge),
                            _ => None,
                        };
                        if let (Some(a), Some(FeatureKind::Pattern(p))) = (axis, self.feature_mut(app)) {
                            p.axis = a;
                        }
                    } else {
                        let plane = match hit.pick {
                            Pick::Origin(o) => Some(PlaneRef::Origin(o)),
                            Pick::Plane(f) => Some(PlaneRef::Plane(f)),
                            Pick::Face { body, face } => face_ref(&st, body, face).map(PlaneRef::Face),
                            _ => None,
                        };
                        if let (Some(pl), Some(FeatureKind::Mirror(m))) = (plane, self.feature_mut(app)) {
                            m.plane = pl;
                        }
                    }
                    return;
                }
                let b = match hit.pick {
                    Pick::Body(b) | Pick::Face { body: b, .. } => b,
                    _ => return,
                };
                if self.fid.is_none() {
                    let kind = if self.kind == CmdKind::Pattern {
                        FeatureKind::Pattern(Pattern {
                            bodies: vec![b],
                            kind: PatternKind::Circular,
                            axis: AxisRef::Z,
                            count: "6".into(),
                            spacing: "20".into(),
                            angle: "360".into(),
                            axis2: None,
                            count2: "2".into(),
                            spacing2: "20".into(),
                            join: false,
                        })
                    } else {
                        FeatureKind::Mirror(Mirror { bodies: vec![b], plane: PlaneRef::Origin(OriginPlane::YZ), join: false })
                    };
                    self.insert(app, kind);
                    return;
                }
                let list = match self.feature_mut(app) {
                    Some(FeatureKind::Pattern(p)) => &mut p.bodies,
                    Some(FeatureKind::Mirror(m)) => &mut m.bodies,
                    _ => return,
                };
                if let Some(i) = list.iter().position(|t| *t == b) {
                    list.remove(i);
                } else {
                    list.push(b);
                }
            }
            CmdKind::Plane if self.plane_kind != PlaneKind::Offset => {
                let fresh = |mode: PlaneMode| FeatureKind::Plane(OffsetPlane { base: PlaneRef::Origin(OriginPlane::XY), offset: "0".into(), mode });
                match self.plane_kind {
                    PlaneKind::Angle => {
                        if let Some(a) = axis_ref_of(&st, hit.pick) {
                            if self.fid.is_none() {
                                self.insert(app, fresh(PlaneMode::Angle { axis: a, angle: "45".into() }));
                            } else if let Some(FeatureKind::Plane(OffsetPlane { mode: PlaneMode::Angle { axis, .. }, .. })) = self.feature_mut(app) {
                                *axis = a;
                            }
                        } else if let (Some(base), Some(FeatureKind::Plane(p))) = (plane_ref_of(&st, hit.pick), self.feature_mut(app)) {
                            p.base = base;
                        }
                    }
                    PlaneKind::Midplane => {
                        let Some(pr) = plane_ref_of(&st, hit.pick) else { return };
                        if self.fid.is_none() {
                            // first pick: base; the feature appears with the second
                            self.insert(app, FeatureKind::Plane(OffsetPlane { base: pr.clone(), offset: "0".into(), mode: PlaneMode::Midplane { other: pr } }));
                            self.stage = 1;
                        } else if let Some(FeatureKind::Plane(p)) = self.feature_mut(app) {
                            if self.stage == 0 {
                                p.base = pr;
                                self.stage = 1;
                            } else if let PlaneMode::Midplane { other } = &mut p.mode {
                                *other = pr;
                                self.stage = 0;
                            }
                        }
                    }
                    PlaneKind::NormalToEdge => {
                        let Pick::Edge { body, edge } = hit.pick else { return };
                        let Some(er) = edge_ref(&st, body, edge) else { return };
                        if self.fid.is_none() {
                            self.insert(app, fresh(PlaneMode::NormalToEdge { edge: er, at: "50".into() }));
                        } else if let Some(FeatureKind::Plane(OffsetPlane { mode: PlaneMode::NormalToEdge { edge: e, .. }, .. })) = self.feature_mut(app) {
                            *e = er;
                        }
                    }
                    PlaneKind::Offset => {}
                }
            }
            CmdKind::Plane => {}
            CmdKind::Split => {
                if self.stage == 1 {
                    if let (Some(pl), Some(FeatureKind::Split(sp))) = (plane_ref_of(&st, hit.pick), self.feature_mut(app)) {
                        sp.tool = pl;
                    }
                    return;
                }
                let b = match hit.pick {
                    Pick::Body(b) | Pick::Face { body: b, .. } => b,
                    _ => return,
                };
                if self.fid.is_none() {
                    // default tool: an origin plane that passes through the body (prefer the one facing the camera)
                    let bb = st.bodies.get(&b).and_then(|x| x.shape.bbox());
                    let fwd = app.cam.cam.forward().abs();
                    let mut cands = [(OriginPlane::XY, 2usize, fwd.z), (OriginPlane::XZ, 1, fwd.y), (OriginPlane::YZ, 0, fwd.x)];
                    cands.sort_by(|a, b| b.2.total_cmp(&a.2));
                    let crosses = |axis: usize| bb.map(|(lo, hi)| lo[axis] < -1e-6 && hi[axis] > 1e-6).unwrap_or(false);
                    let tool = cands.iter().find(|c| crosses(c.1)).map(|c| c.0).unwrap_or(cands[0].0);
                    self.insert(app, FeatureKind::Split(Split { bodies: vec![b], tool: PlaneRef::Origin(tool), keep: SplitKeep::Both }));
                    self.stage = 1;
                    return;
                }
                if let Some(FeatureKind::Split(sp)) = self.feature_mut(app) {
                    if let Some(i) = sp.bodies.iter().position(|x| *x == b) {
                        sp.bodies.remove(i);
                    } else {
                        sp.bodies.push(b);
                    }
                }
            }
            CmdKind::Hole => {
                let Pick::Face { body, face } = hit.pick else { return };
                let Some(fr) = face_ref(&st, body, face) else { return };
                let Some(info) = st.bodies.get(&body).and_then(|b| b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok())) else { return };
                if info.kind != lusion_kernel::SurfaceKind::Plane {
                    return;
                }
                let frame = lusion_core::model::face_frame(info.origin, info.normal);
                let p = snap_on_face(app, &st, body, face, hit.point);
                let p2 = frame.to2(p.as_dvec3()).to_array();
                if self.fid.is_none() {
                    self.insert(
                        app,
                        FeatureKind::Hole(Hole {
                            face: fr,
                            points: vec![p2],
                            sketch: None,
                            kind: HoleKind::Simple,
                            diameter: "3.4".into(),
                            depth: "10".into(),
                            through: true,
                            cb_diameter: "6.5".into(),
                            cb_depth: "3.5".into(),
                            cs_diameter: "6.5".into(),
                            cs_angle: "90".into(),
                            drill_point: false,
                        }),
                    );
                    return;
                }
                if let Some(FeatureKind::Hole(h)) = self.feature_mut(app) {
                    if h.face.body != fr.body || h.face.name != fr.name {
                        h.face = fr;
                        h.points.clear();
                    }
                    // clicking an existing hole again removes it
                    if let Some(i) = h.points.iter().position(|q| (glam::DVec2::from_array(*q) - glam::DVec2::from_array(p2)).length() < 0.5) {
                        h.points.remove(i);
                    } else {
                        h.points.push(p2);
                    }
                }
            }
            CmdKind::Thread => {
                let Pick::Face { body, face } = hit.pick else { return };
                let Some(b) = st.bodies.get(&body) else { return };
                let Some(f) = b.shape.faces().get(face as usize).cloned() else { return };
                let Ok(info) = f.face_info() else { return };
                if info.kind != lusion_kernel::SurfaceKind::Cylinder {
                    app.info("Threads go on cylindrical faces (a rod or a hole)");
                    return;
                }
                let Some(fr) = face_ref(&st, body, face) else { return };
                let external = f.point_normal().map(|(p, n)| {
                    let a = info.normal.normalize();
                    let r = (p - info.origin) - a * a.dot(p - info.origin);
                    r.dot(n) > 0.0
                }).unwrap_or(true);
                let (d, pitch) = guess_thread(info.radius, external);
                let fmt = lusion_core::expr::fmt_num;
                if self.fid.is_none() {
                    self.insert(app, FeatureKind::Thread(Thread { face: fr, size: fmt(d), pitch: fmt(pitch), length: String::new(), clearance: "0.1".into(), left: false, flip: false }));
                } else if let Some(FeatureKind::Thread(t)) = self.feature_mut(app) {
                    t.face = fr;
                    t.size = fmt(d);
                    t.pitch = fmt(pitch);
                }
            }
            CmdKind::Sweep | CmdKind::Loft => {
                if self.kind == CmdKind::Sweep && self.stage == 1 {
                    let pr = match hit.pick {
                        Pick::SketchLine { sketch, curve } => PathRef::SketchCurve { sketch, curve },
                        Pick::Edge { body, edge } => match edge_ref(&st, body, edge) {
                            Some(e) => PathRef::Edge(e),
                            None => return,
                        },
                        _ => return,
                    };
                    if let Some(FeatureKind::Sweep(sw)) = self.feature_mut(app) {
                        if let Some(i) = sw.path.iter().position(|p| *p == pr) {
                            sw.path.remove(i);
                        } else {
                            sw.path.push(pr);
                        }
                    }
                    return;
                }
                let pr = match hit.pick {
                    Pick::Profile { sketch, index } => profile_ref(&st, sketch, index),
                    Pick::Face { body, face } => face_ref(&st, body, face).map(ProfileRef::Face),
                    _ => None,
                };
                let Some(pr) = pr else { return };
                if self.fid.is_none() {
                    let kind = if self.kind == CmdKind::Sweep {
                        FeatureKind::Sweep(Sweep { profiles: vec![pr], path: vec![], op: Operation::NewBody, targets: vec![], frenet: false })
                    } else {
                        FeatureKind::Loft(Loft { profiles: vec![pr], ruled: false, op: Operation::NewBody, targets: vec![] })
                    };
                    self.insert(app, kind);
                    if self.kind == CmdKind::Sweep {
                        self.stage = 1;
                    }
                    return;
                }
                let list = match self.feature_mut(app) {
                    Some(FeatureKind::Sweep(x)) => &mut x.profiles,
                    Some(FeatureKind::Loft(x)) => &mut x.profiles,
                    _ => return,
                };
                if let Some(i) = list.iter().position(|p| same_profile(p, &pr)) {
                    list.remove(i);
                } else {
                    list.push(pr);
                }
            }
            CmdKind::Joint => {
                let jr = match hit.pick {
                    Pick::Face { body, face } => face_ref(&st, body, face).map(JointRef::Face),
                    Pick::Edge { body, edge } => edge_ref(&st, body, edge).map(JointRef::Edge),
                    _ => None,
                };
                let Some(jr) = jr else { return };
                let body_of = |r: &JointRef| match r {
                    JointRef::Face(f) => f.body,
                    JointRef::Edge(e) => e.body,
                };
                if self.fid.is_some() {
                    // re-pick: stage 0 = moving attachment, 1 = target
                    if let Some(FeatureKind::Joint(j)) = self.feature_mut(app) {
                        if self.stage == 0 {
                            j.a = jr;
                        } else {
                            j.b = jr;
                        }
                    }
                    return;
                }
                let Some(a) = self.joint_a.take() else {
                    self.joint_a = Some(jr);
                    self.stage = 1;
                    return;
                };
                let (ba, bb) = (body_of(&a), body_of(&jr));
                if ba == bb {
                    self.joint_a = Some(a);
                    app.info("Pick the second attachment on another part");
                    return;
                }
                // the moving part needs its own component (not one containing the target)
                let mut comp = app.body_component(&st, ba);
                let target_comp = app.body_component(&st, bb);
                if comp == ROOT || app.doc.is_ancestor(comp, target_comp) {
                    let name = format!("Part{}", app.doc.components.len());
                    comp = app.doc.add_component(&name, ROOT);
                    app.doc.bodies.entry(ba).or_default().component = Some(comp);
                }
                self.insert(app, FeatureKind::Joint(Joint { moving: comp, a, b: jr, kind: JointKind::Rigid, angle: "0".into(), offset: "0".into(), flip: false }));
                self.stage = 0;
            }
            CmdKind::Measure => {
                if self.measure.len() >= 2 {
                    self.measure.clear();
                }
                self.measure.push(hit);
                self.measure_res = Some(compute_measure(app, &st, &self.measure));
            }
            CmdKind::Section => {
                if let Some((p, n)) = plane_of_pick(app, &st, hit.pick) {
                    // keep the half that faces away from the picked plane's normal
                    app.section = Some(crate::app::SectionView { normal: n, origin: p, offset: 0.0 });
                }
            }
            CmdKind::Move if self.stage == 1 && self.fid.is_some() => {
                // rotation axis
                let axis = match hit.pick {
                    Pick::Axis(0) => Some(AxisRef::X),
                    Pick::Axis(1) => Some(AxisRef::Y),
                    Pick::Axis(_) => Some(AxisRef::Z),
                    Pick::SketchLine { sketch, curve } => Some(AxisRef::SketchLine { sketch, curve }),
                    Pick::Edge { body, edge } => edge_ref(&st, body, edge).map(AxisRef::Edge),
                    _ => None,
                };
                if let (Some(a), Some(FeatureKind::Move(m))) = (axis, self.feature_mut(app)) {
                    m.axis = a;
                }
                self.stage = 0;
            }
            CmdKind::Move if self.stage == 2 && self.fid.is_some() => {
                // pivot: face centre, edge midpoint (circle centre for round edges)
                let p = match hit.pick {
                    Pick::Face { body, face } => st.bodies.get(&body).and_then(|b| b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok())).map(|i| i.center),
                    Pick::Edge { body, edge } => st.bodies.get(&body).and_then(|b| b.shape.edges().get(edge as usize).and_then(|e| e.edge_info().ok())).map(|i| if i.kind == lusion_kernel::CurveKind::Circle { i.center } else { i.mid }),
                    _ => None,
                };
                if let (Some(p), Some(FeatureKind::Move(m))) = (p, self.feature_mut(app)) {
                    m.pivot = Some(p.to_array());
                }
                self.stage = 0;
            }
            CmdKind::Move => {
                let b = match hit.pick {
                    Pick::Body(b) | Pick::Face { body: b, .. } => b,
                    _ => return,
                };
                if self.fid.is_none() {
                    self.insert(
                        app,
                        FeatureKind::Move(MoveBodies { bodies: vec![b], dx: "0".into(), dy: "0".into(), dz: "0".into(), rz: "0".into(), copy: false, pivot: None, axis: AxisRef::Z }),
                    );
                    return;
                }
                if let Some(FeatureKind::Move(m)) = self.feature_mut(app) {
                    if let Some(i) = m.bodies.iter().position(|t| *t == b) {
                        m.bodies.remove(i);
                    } else {
                        m.bodies.push(b);
                    }
                }
            }
        }
    }

    /// Fusion-style: pulling into the body cuts, pulling out joins.
    fn apply_auto_op(&self, app: &App, e: &mut Extrude) {
        let Some(base) = self.auto_op else { return };
        if base == Operation::NewBody || !matches!(e.mode, ExtentMode::OneSide | ExtentMode::ThroughAll) {
            return;
        }
        let d = app.model.params.eval(&e.distance).unwrap_or(0.0);
        e.op = if d < 0.0 { Operation::Cut } else { Operation::Join };
    }

    /// Anchor + direction of the extrude drag arrow.
    fn extrude_axis(&self, app: &App) -> Option<(DVec3, DVec3, f64)> {
        let Some(FeatureKind::Extrude(e)) = self.feature(app) else { return None };
        let st = self.pre_state(app);
        let (anchor, n) = match e.profiles.first()? {
            ProfileRef::Sketch { sketch, curves, hint } => {
                let ss = st.sketches.get(sketch)?;
                let p = find_profile(ss, curves, glam::DVec2::from_array(*hint))?;
                (ss.frame.to3(p.hint), ss.frame.n)
            }
            ProfileRef::Face(f) => {
                let b = st.bodies.get(&f.body)?;
                let i = b.face_index(&f.name)?;
                let info = b.shape.faces().get(i)?.face_info().ok()?;
                (info.center, info.normal)
            }
        };
        let start = app.model.params.eval(if e.start_offset.trim().is_empty() { "0" } else { &e.start_offset }).unwrap_or(0.0);
        let d = app.model.params.eval(&e.distance).unwrap_or(0.0);
        Some((anchor + n * start, n, d))
    }

    /// In-viewport manipulators for the current command.
    fn handles(&self, app: &mut App, view: &View) -> Vec<Handle> {
        let mut out = Vec::new();
        match self.kind {
            CmdKind::Section => {
                if let Some(sv) = app.section {
                    let n = sv.normal.normalize_or_zero();
                    let o = sv.origin;
                    let tip = o + n * sv.offset;
                    let px = px_at(app, view, tip);
                    out.push(Handle {
                        id: 0,
                        kind: GKind::Linear { origin: o, dir: n, base: tip, tip: tip + n * px * 60.0 },
                        value: sv.offset as f64,
                        color: HANDLE_BLUE,
                        label: None,
                    });
                }
            }
            CmdKind::Plane => {
                let Some(fid) = self.fid else { return out };
                let Some(FeatureKind::Plane(p)) = self.feature(app).cloned() else { return out };
                let st = app.model.current(&app.doc);
                let Some(fr) = st.planes.get(&fid).copied() else { return out };
                let d = app.model.params.eval(if p.offset.trim().is_empty() { "0" } else { &p.offset }).unwrap_or(0.0);
                let (o, n) = ((fr.origin - fr.n * d).as_vec3(), fr.n.as_vec3());
                let tip = fr.origin.as_vec3();
                let shown = (d as f32).abs().max(px_at(app, view, tip) * 30.0) * if d < 0.0 { -1.0 } else { 1.0 };
                out.push(Handle {
                    id: 0,
                    kind: GKind::Linear { origin: o, dir: n, base: o, tip: o + n * shown },
                    value: d,
                    color: HANDLE_BLUE,
                    label: Some(format!("{} mm", lusion_core::expr::fmt_num(d))),
                });
            }
            CmdKind::Joint => {
                let Some(FeatureKind::Joint(j)) = self.feature(app).cloned() else { return out };
                let st = self.pre_state(app);
                let Some(fb) = lusion_core::model::joint_frame(st, &j.b) else { return out };
                let ev = |e: &str| app.model.params.eval(if e.trim().is_empty() { "0" } else { e }).unwrap_or(0.0);
                let (ang, off) = (ev(&j.angle), ev(&j.offset));
                let n = fb.n.as_vec3();
                let c = (fb.origin + fb.n * off).as_vec3();
                let px = px_at(app, view, c);
                if j.kind != JointKind::Slider {
                    out.push(Handle { id: 1, kind: GKind::Ring { center: c, normal: n, radius: px * 70.0 }, value: ang, color: [1.0, 0.62, 0.25, 1.0], label: None });
                }
                if j.kind != JointKind::Revolute {
                    let o = fb.origin.as_vec3();
                    out.push(Handle { id: 2, kind: GKind::Linear { origin: o, dir: n, base: c, tip: c + n * px * 60.0 }, value: off, color: HANDLE_BLUE, label: None });
                }
            }
            CmdKind::Extrude => {
                if let Some((anchor, n, d)) = self.extrude_axis(app) {
                    let (o, dir) = (anchor.as_vec3(), n.as_vec3());
                    out.push(Handle {
                        id: 0,
                        kind: GKind::Linear { origin: o, dir, base: o, tip: o + dir * d as f32 },
                        value: d,
                        color: HANDLE_BLUE,
                        label: Some(format!("{} mm", lusion_core::expr::fmt_num(d))),
                    });
                }
            }
            CmdKind::Fillet | CmdKind::Chamfer => {
                let (first, size, fillet) = match self.feature(app) {
                    Some(FeatureKind::Fillet(f)) => (f.edges.first().cloned(), f.radius.clone(), true),
                    Some(FeatureKind::Chamfer(f)) => (f.edges.first().cloned(), f.distance.clone(), false),
                    _ => return out,
                };
                let Some(er) = first else { return out };
                let st = self.pre_state(app).clone();
                let Some(body) = st.bodies.get(&er.body) else { return out };
                let Some(ei) = body.edge_index(&er.name) else { return out };
                let Some(info) = body.shape.edges().get(ei).and_then(|e| e.edge_info().ok()) else { return out };
                let mid = info.mid.as_vec3();
                let dir = edge_outward(app, &st, er.body, ei, mid);
                let v = app.model.params.eval(&size).unwrap_or(1.0);
                // show the arrow at least 30 px long so it can be grabbed
                let shown = (v as f32).max(px_at(app, view, mid) * 30.0);
                out.push(Handle {
                    id: 0,
                    kind: GKind::Linear { origin: mid, dir, base: mid, tip: mid + dir * shown },
                    value: v,
                    color: HANDLE_BLUE,
                    label: Some(if fillet { format!("R {} mm", lusion_core::expr::fmt_num(v)) } else { format!("{} mm", lusion_core::expr::fmt_num(v)) }),
                });
            }
            CmdKind::Move => {
                let Some(FeatureKind::Move(m)) = self.feature(app).cloned() else { return out };
                let st = self.pre_state(app).clone();
                let pivot = m.pivot.map(|p| DVec3::from_array(p)).unwrap_or_else(|| lusion_core::model::move_pivot(&st, &m.bodies));
                let ev = |e: &str| app.model.params.eval(if e.trim().is_empty() { "0" } else { e }).unwrap_or(0.0);
                let d = [ev(&m.dx), ev(&m.dy), ev(&m.dz)];
                let ang = ev(&m.rz);
                let gc = (pivot + DVec3::from_array(d)).as_vec3();
                let px = px_at(app, view, gc);
                let len = px * 95.0;
                for (i, dir) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
                    out.push(Handle {
                        id: i as u8,
                        kind: GKind::Linear { origin: gc - *dir * d[i] as f32, dir: *dir, base: gc + *dir * px * 14.0, tip: gc + *dir * len },
                        value: d[i],
                        color: AXIS_COLORS[i],
                        label: None,
                    });
                }
                let axis_idx = match m.axis {
                    AxisRef::X => Some(0),
                    AxisRef::Y => Some(1),
                    AxisRef::Z => Some(2),
                    _ => None,
                };
                for (i, n) in [Vec3::X, Vec3::Y, Vec3::Z].iter().enumerate() {
                    out.push(Handle {
                        id: 3 + i as u8,
                        kind: GKind::Ring { center: gc, normal: *n, radius: px * 72.0 },
                        value: if axis_idx == Some(i) { ang } else { 0.0 },
                        color: AXIS_COLORS[i],
                        label: None,
                    });
                }
            }
            _ => {}
        }
        out
    }

    fn set_handle(&mut self, app: &mut App, id: u8, v: f64) {
        let txt = lusion_core::expr::fmt_num(v);
        if self.kind == CmdKind::Section {
            if let Some(sv) = app.section.as_mut() {
                sv.offset = v as f32;
            }
            return;
        }
        match self.feature_mut(app) {
            Some(FeatureKind::Plane(p)) => p.offset = txt,
            Some(FeatureKind::Joint(j)) => {
                if id == 1 {
                    j.angle = txt;
                } else {
                    j.offset = txt;
                }
            }
            Some(FeatureKind::Extrude(ex)) => {
                if ex.distance != txt {
                    ex.distance = txt;
                }
            }
            Some(FeatureKind::Fillet(f)) => f.radius = lusion_core::expr::fmt_num(v.max(0.01)),
            Some(FeatureKind::Chamfer(f)) => f.distance = lusion_core::expr::fmt_num(v.max(0.01)),
            Some(FeatureKind::Move(m)) => match id {
                0 => m.dx = txt,
                1 => m.dy = txt,
                2 => m.dz = txt,
                _ => {
                    let axis = [AxisRef::X, AxisRef::Y, AxisRef::Z][(id - 3) as usize].clone();
                    if m.axis != axis {
                        m.axis = axis;
                    }
                    m.rz = txt;
                }
            },
            _ => {}
        }
        if self.kind == CmdKind::Extrude {
            let mut ex2 = match self.feature(app) {
                Some(FeatureKind::Extrude(e)) => e.clone(),
                _ => return,
            };
            self.apply_auto_op(app, &mut ex2);
            if let Some(FeatureKind::Extrude(ex)) = self.feature_mut(app) {
                if ex.op != ex2.op {
                    ex.op = ex2.op;
                }
            }
        }
    }

    /// Screen distance from the mouse to a handle.
    fn handle_dist(app: &App, view: &View, h: &Handle, mouse: Vec2) -> f32 {
        match h.kind {
            GKind::Linear { base, tip, .. } => {
                let (Some(a), Some(b)) = (app.cam.project(base, view.size), app.cam.project(tip, view.size)) else { return f32::INFINITY };
                // the arrow head extends beyond the tip
                let ext = b + (b - a).normalize_or_zero() * 22.0;
                crate::pick::seg_dist(mouse, a, ext).0
            }
            GKind::Ring { center, normal, radius } => {
                let u = normal.any_orthonormal_vector();
                let w = normal.cross(u);
                let mut best = f32::INFINITY;
                let mut prev: Option<Vec2> = None;
                for k in 0..=64 {
                    let t = k as f32 / 64.0 * std::f32::consts::TAU;
                    let p = app.cam.project(center + (u * t.cos() + w * t.sin()) * radius, view.size);
                    if let (Some(a), Some(b)) = (prev, p) {
                        best = best.min(crate::pick::seg_dist(mouse, a, b).0);
                    }
                    prev = p;
                }
                best
            }
        }
    }

    /// Drag handling for in-viewport manipulators. Returns true if the pointer is consumed.
    pub fn handle_drag(&mut self, app: &mut App, ui: &egui::Ui, view: &View) -> bool {
        let handles = self.handles(app, view);
        if handles.is_empty() {
            self.gdrag = None;
            self.ghover = None;
            return false;
        }
        let Some(mouse) = view.mouse else {
            return self.gdrag.is_some();
        };
        let (pressed, down, released, fine) =
            ui.input(|i| (i.pointer.primary_pressed(), i.pointer.primary_down(), i.pointer.primary_released(), i.modifiers.shift));
        let ray = app.cam.ray(mouse, view.size);
        if self.gdrag.is_none() {
            let near = handles
                .iter()
                // arrows win over rings when both are close
                .map(|h| (h, Self::handle_dist(app, view, h, mouse) + if matches!(h.kind, GKind::Ring { .. }) { 5.0 } else { 0.0 }))
                .filter(|(_, d)| *d < 12.0)
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .map(|(h, _)| h.clone());
            self.ghover = near.as_ref().map(|h| h.id);
            if pressed {
                if let Some(h) = near {
                    let start_param = match h.kind {
                        GKind::Linear { origin, dir, .. } => line_param(&ray, origin, dir),
                        GKind::Ring { center, normal, .. } => ring_angle(&ray, center, normal),
                    };
                    if let Some(sp) = start_param {
                        self.gdrag = Some(GDrag { id: h.id, kind: h.kind, start_value: h.value, start_param: sp });
                        return true;
                    }
                }
            }
            return false;
        }
        let g = self.gdrag.unwrap();
        if down {
            match g.kind {
                GKind::Linear { origin, dir, tip, .. } => {
                    if let Some(sp) = line_param(&ray, origin, dir) {
                        let v = nice_snap(g.start_value + (sp - g.start_param), px_at(app, view, tip));
                        self.set_handle(app, g.id, v);
                    }
                }
                GKind::Ring { center, normal, .. } => {
                    if let Some(a) = ring_angle(&ray, center, normal) {
                        let mut delta = a - g.start_param;
                        while delta > 180.0 {
                            delta -= 360.0;
                        }
                        while delta < -180.0 {
                            delta += 360.0;
                        }
                        let step = if fine { 1.0 } else { 5.0 };
                        let v = ((g.start_value + delta) / step).round() * step;
                        self.set_handle(app, g.id, v);
                        // keep the start reference so large rotations accumulate
                        if let Some(gd) = self.gdrag.as_mut() {
                            gd.start_value = v;
                            gd.start_param = a - (g.start_value + delta - v);
                        }
                    }
                }
            }
            return true;
        }
        if released || !down {
            self.gdrag = None;
            return true;
        }
        true
    }

    pub fn draw(&self, app: &mut App, scene: &mut Scene, view: &View) {
        let handles = self.handles(app, view);
        let active = self.gdrag.map(|g| g.id).or(self.ghover);
        for h in &handles {
            let hot = active == Some(h.id);
            let c = if hot { [1.0, 0.85, 0.30, 1.0] } else { h.color };
            match h.kind {
                GKind::Linear { origin, dir, base, tip } => {
                    let px = px_at(app, view, tip);
                    push_line(&mut scene.overlay_lines, base, tip, c, if hot { 3.5 } else { 2.2 });
                    let d = if (tip - base).dot(dir) < 0.0 { -dir } else { dir };
                    push_cone(scene, app, tip, d, px * if hot { 28.0 } else { 24.0 }, px * if hot { 10.0 } else { 8.0 }, c);
                    if self.kind != CmdKind::Move {
                        scene.points.push(crate::render::PointInst { p: base.to_array(), color: [0.9, 0.9, 0.95, 1.0], size: 7.0 });
                    }
                    let _ = origin;
                }
                GKind::Ring { center, normal, radius } => {
                    let u = normal.any_orthonormal_vector();
                    let w = normal.cross(u);
                    let pts: Vec<Vec3> = (0..=72).map(|k| {
                        let t = k as f32 / 72.0 * std::f32::consts::TAU;
                        center + (u * t.cos() + w * t.sin()) * radius
                    }).collect();
                    push_polyline(&mut scene.overlay_lines, &pts, [c[0], c[1], c[2], if hot { 1.0 } else { 0.75 }], if hot { 3.5 } else { 2.0 });
                }
            }
        }
        self.draw_extras(app, scene, view);
        if self.kind == CmdKind::Move && !handles.is_empty() {
            if let GKind::Ring { center, .. } = handles[3].kind {
                scene.points.push(crate::render::PointInst { p: center.to_array(), color: [1.0, 1.0, 1.0, 1.0], size: 9.0 });
            }
        }
    }

    /// Command-specific scene decorations (hole markers, measurements, section plane).
    fn draw_extras(&self, app: &mut App, scene: &mut Scene, view: &View) {
        match self.kind {
            CmdKind::Hole => {
                let Some(FeatureKind::Hole(h)) = self.feature(app).cloned() else { return };
                let st = self.pre_state(app).clone();
                let Some(face) = face_pick(&st, &h.face) else { return };
                let Pick::Face { body, face } = face else { return };
                let Some(info) = st.bodies.get(&body).and_then(|b| b.shape.faces().get(face as usize).and_then(|f| f.face_info().ok())) else { return };
                let frame = lusion_core::model::face_frame(info.origin, info.normal);
                for p in &h.points {
                    let w = frame.to3(glam::DVec2::from_array(*p)).as_vec3();
                    scene.points.push(crate::render::PointInst { p: w.to_array(), color: [1.0, 0.75, 0.25, 1.0], size: 9.0 });
                }
            }
            CmdKind::Measure => {
                for h in &self.measure {
                    scene.points.push(crate::render::PointInst { p: h.point.to_array(), color: [1.0, 0.85, 0.3, 1.0], size: 8.0 });
                }
                if let Some(m) = measure_two(app, self) {
                    if let Some((a, b)) = m.points {
                        push_line(&mut scene.overlay_lines, a, b, [1.0, 0.85, 0.3, 1.0], 2.5);
                        for p in [a, b] {
                            scene.points.push(crate::render::PointInst { p: p.to_array(), color: [1.0, 1.0, 1.0, 1.0], size: 7.0 });
                        }
                    }
                }
            }
            CmdKind::Section => {
                let Some(sv) = app.section else { return };
                let n = sv.normal.normalize_or_zero();
                let c = sv.origin + n * sv.offset;
                let size = app.scene_bounds().map(|(a, b)| (b - a).length() * 0.6).unwrap_or(50.0);
                let u = n.any_orthonormal_vector();
                let w = n.cross(u);
                let corners = [c + (u + w) * size, c + (u - w) * size, c + (-u - w) * size, c + (-u + w) * size];
                let col = [0.35, 0.65, 1.0, 0.10];
                for t in [[0, 1, 2], [0, 2, 3]] {
                    for k in t {
                        scene.fills.push(crate::render::FillVert { p: corners[k].to_array(), color: col });
                    }
                }
                let mut ring = corners.to_vec();
                ring.push(corners[0]);
                push_polyline(&mut scene.lines, &ring, [0.45, 0.72, 1.0, 0.8], 1.5);
                let _ = view;
            }
            _ => {}
        }
    }

    pub fn overlay(&mut self, app: &mut App, ui: &egui::Ui, view: &View) {
        if self.kind == CmdKind::Measure {
            if let Some(m) = measure_two(app, self) {
                if let Some((a, b)) = m.points {
                    if let Some(p) = app.cam.project((a + b) * 0.5, view.size) {
                        let pos = view.to_screen(p + Vec2::new(10.0, -10.0));
                        let galley = ui.painter().layout_no_wrap(format!("{} mm", lusion_core::expr::fmt_num(m.distance)), egui::FontId::proportional(14.0), Color32::WHITE);
                        let r = egui::Rect::from_min_size(pos - egui::vec2(4.0, galley.size().y + 2.0), galley.size() + egui::vec2(8.0, 4.0));
                        ui.painter().rect_filled(r, 4.0, Color32::from_rgba_unmultiplied(60, 45, 10, 230));
                        ui.painter().galley(r.min + egui::vec2(4.0, 2.0), galley, Color32::WHITE);
                    }
                }
            }
        }
        let handles = self.handles(app, view);
        for h in &handles {
            let text = match (&h.label, self.gdrag) {
                (Some(l), _) => Some(l.clone()),
                (None, Some(g)) if g.id == h.id => Some(if matches!(h.kind, GKind::Ring { .. }) { format!("{}°", lusion_core::expr::fmt_num(h.value)) } else { format!("{} mm", lusion_core::expr::fmt_num(h.value)) }),
                _ => None,
            };
            let Some(text) = text else { continue };
            let at = match h.kind {
                GKind::Linear { tip, .. } => tip,
                GKind::Ring { center, normal, radius } => center + normal.any_orthonormal_vector() * radius,
            };
            if let Some(p) = app.cam.project(at, view.size) {
                let pos = view.to_screen(p + Vec2::new(16.0, -16.0));
                let galley = ui.painter().layout_no_wrap(text, egui::FontId::proportional(13.0), Color32::WHITE);
                let r = egui::Rect::from_min_size(pos - egui::vec2(4.0, galley.size().y + 2.0), galley.size() + egui::vec2(8.0, 4.0));
                ui.painter().rect_filled(r, 4.0, Color32::from_rgba_unmultiplied(20, 24, 30, 210));
                ui.painter().galley(r.min + egui::vec2(4.0, 2.0), galley, Color32::WHITE);
            }
        }
    }
}

// ------------------------------------------------------------------ dialog panel

fn expr_field(ui: &mut egui::Ui, app: &App, label: &str, value: &mut String, unit: &str) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.add_sized([92.0, 22.0], egui::Label::new(label));
        let res = if value.trim().is_empty() { Ok(0.0) } else { app.model.params.eval(value) };
        let mut te = egui::TextEdit::singleline(value).desired_width(100.0);
        if res.is_err() {
            te = te.text_color(theme::ERR);
        }
        let r = ui.add(te);
        changed |= r.changed();
        match res {
            Ok(v) if value.trim().parse::<f64>().is_err() && !value.trim().is_empty() => {
                ui.label(RichText::new(format!("= {} {unit}", lusion_core::expr::fmt_num(v))).color(theme::TEXT_DIM));
            }
            Ok(_) => {
                ui.label(RichText::new(unit).color(theme::TEXT_DIM));
            }
            Err(e) => {
                ui.label(RichText::new(ic::WARNING).color(theme::ERR)).on_hover_text(e);
            }
        }
    });
    changed
}

fn op_selector(ui: &mut egui::Ui, op: &mut Operation) {
    ui.horizontal(|ui| {
        ui.add_sized([92.0, 22.0], egui::Label::new("Operation"));
        for o in Operation::ALL {
            let icon = match o {
                Operation::NewBody => ic::CUBE,
                Operation::Join => ic::UNITE,
                Operation::Cut => ic::SUBTRACT,
                Operation::Intersect => ic::INTERSECT,
            };
            ui.selectable_value(op, o, RichText::new(icon).size(16.0)).on_hover_text(o.label());
        }
    });
}

pub fn command_panel(app: &mut App, ui: &mut egui::Ui) {
    let Some(mut c) = app.cmd.take() else { return };
    if c.stage == 99 {
        // command finished itself (e.g. new sketch)
        return;
    }
    let mut finish: Option<bool> = None;
    let frame = egui::Frame::new()
        .fill(egui::Color32::from_rgba_unmultiplied(0x25, 0x29, 0x2f, 245))
        .stroke(egui::Stroke::new(1.0, egui::Color32::from_rgb(0x3a, 0x42, 0x4d)))
        .corner_radius(10)
        .shadow(egui::epaint::Shadow { offset: [0, 6], blur: 18, spread: 0, color: egui::Color32::from_black_alpha(110) })
        .inner_margin(12);
    egui::Area::new(egui::Id::new("command_panel")).anchor(egui::Align2::RIGHT_TOP, [-16.0, 285.0]).order(egui::Order::Foreground).show(ui.ctx(), |ui| frame.show(ui, |ui| {
        ui.set_width(290.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new(c.kind.icon()).size(20.0).color(theme::ACCENT));
            ui.label(RichText::new(if c.editing { format!("Edit {}", c.kind.title()) } else { c.kind.title().to_string() }).size(16.0).strong());
        });
        ui.separator();
        let before = app.doc.clone();
        let hint = dialog_body(&mut c, app, ui);
        if app.doc != before {
            // live preview is automatic (model updates next frame)
            ui.ctx().request_repaint();
        }
        ui.add_space(8.0);
        if let Some(h) = hint {
            ui.label(RichText::new(h).color(theme::TEXT_DIM).italics());
        }
        if let Some(fid) = c.fid {
            if let Some(i) = app.doc.feature_index(fid) {
                if let Some(s) = app.model.feature_status(i) {
                    if let Some(e) = &s.error {
                        ui.colored_label(theme::ERR, format!("{} {e}", ic::WARNING));
                    }
                    for w in &s.warnings {
                        ui.colored_label(theme::WARN, format!("{} {w}", ic::WARNING));
                    }
                    ui.label(RichText::new(format!("computed in {:.1} ms", s.millis)).small().color(theme::TEXT_DIM));
                }
            }
        }
        ui.add_space(10.0);
        ui.horizontal(|ui| {
            if c.kind.is_tool() {
                if ui.add(egui::Button::new(format!("{} Close", ic::X)).min_size(egui::vec2(90.0, 28.0))).clicked() {
                    finish = Some(false);
                }
                return;
            }
            let can_ok = c.fid.is_some();
            if ui.add_enabled(can_ok, egui::Button::new(RichText::new(format!("{} OK", ic::CHECK)).strong()).min_size(egui::vec2(90.0, 28.0))).clicked() {
                finish = Some(true);
            }
            if ui.add(egui::Button::new(format!("{} Cancel", ic::X)).min_size(egui::vec2(90.0, 28.0))).clicked() {
                finish = Some(false);
            }
        });
        if !c.kind.is_tool() {
            ui.label(RichText::new("Enter = OK · Esc = Cancel").small().color(theme::TEXT_DIM));
        }
    }));
    let wants_kb = ui.ctx().egui_wants_keyboard_input();
    ui.input(|i| {
        if i.key_pressed(egui::Key::Escape) {
            finish = Some(false);
        }
        if i.key_pressed(egui::Key::Enter) && c.fid.is_some() && (!wants_kb || i.modifiers.command) {
            finish = Some(true);
        }
    });
    if ui.input(|i| i.key_pressed(egui::Key::Enter)) && wants_kb && c.fid.is_some() {
        // Enter inside a text field: accept the field; a second Enter confirms
    }
    app.cmd = Some(c);
    match finish {
        Some(true) => ok(app),
        Some(false) => cancel(app),
        None => {}
    }
}

/// Draws the dialog fields; returns a hint line.
fn dialog_body(c: &mut Command, app: &mut App, ui: &mut egui::Ui) -> Option<String> {
    let fid = c.fid;
    match c.kind {
        CmdKind::NewSketch => {
            app.show_origin = app.show_origin;
            Some("Select a plane or a planar face to sketch on.".into())
        }
        CmdKind::Extrude => {
            let Some(fid) = fid else { return Some("Select profiles or planar faces.".into()) };
            let mut e = match app.doc.feature(fid).map(|f| &f.kind) {
                Some(FeatureKind::Extrude(e)) => e.clone(),
                _ => return None,
            };
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Profiles"));
                ui.label(RichText::new(format!("{} selected", e.profiles.len())).strong());
                if ui.small_button(ic::X).on_hover_text("Clear").clicked() {
                    e.profiles.clear();
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Direction"));
                egui::ComboBox::from_id_salt("ext_mode")
                    .selected_text(match e.mode {
                        ExtentMode::OneSide => "One side",
                        ExtentMode::Symmetric => "Symmetric",
                        ExtentMode::TwoSides => "Two sides",
                        ExtentMode::ThroughAll => "Through all",
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut e.mode, ExtentMode::OneSide, "One side");
                        ui.selectable_value(&mut e.mode, ExtentMode::Symmetric, "Symmetric");
                        ui.selectable_value(&mut e.mode, ExtentMode::TwoSides, "Two sides");
                        ui.selectable_value(&mut e.mode, ExtentMode::ThroughAll, "Through all");
                    });
            });
            if e.mode != ExtentMode::ThroughAll {
                expr_field(ui, app, "Distance", &mut e.distance, "mm");
            }
            ui.horizontal(|ui| {
                ui.add_space(96.0);
                if ui.small_button(format!("{} Flip", ic::ARROWS_DOWN_UP)).clicked() {
                    e.distance = if let Some(s) = e.distance.trim().strip_prefix('-') { s.trim().to_string() } else { format!("-{}", e.distance.trim()) };
                }
            });
            if e.mode == ExtentMode::TwoSides {
                expr_field(ui, app, "Distance 2", &mut e.distance2, "mm");
            }
            expr_field(ui, app, "Start offset", &mut e.start_offset, "mm");
            let op_before = e.op;
            op_selector(ui, &mut e.op);
            if e.op != op_before {
                c.auto_op = None;
            }
            c.apply_auto_op(app, &mut e);
            if let Some(FeatureKind::Extrude(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != e {
                    *x = e;
                }
            }
            Some("Click profiles/faces to add or remove them. Drag the arrow to set the distance.".into())
        }
        CmdKind::Revolve => {
            let Some(fid) = fid else { return Some("Select profiles to revolve.".into()) };
            let mut r = match app.doc.feature(fid).map(|f| &f.kind) {
                Some(FeatureKind::Revolve(r)) => r.clone(),
                _ => return None,
            };
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Profiles"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", r.profiles.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Axis"));
                let label = match &r.axis {
                    AxisRef::X => "X axis".to_string(),
                    AxisRef::Y => "Y axis".into(),
                    AxisRef::Z => "Z axis".into(),
                    AxisRef::SketchLine { .. } => "Sketch line".into(),
                    AxisRef::Edge(_) => "Edge".into(),
                };
                if ui.selectable_label(c.stage == 1, label).on_hover_text("Click, then pick a line/edge/axis").clicked() {
                    c.stage = 1;
                }
                if ui.small_button("X").clicked() {
                    r.axis = AxisRef::X;
                }
                if ui.small_button("Y").clicked() {
                    r.axis = AxisRef::Y;
                }
                if ui.small_button("Z").clicked() {
                    r.axis = AxisRef::Z;
                }
            });
            expr_field(ui, app, "Angle", &mut r.angle, "deg");
            op_selector(ui, &mut r.op);
            if let Some(FeatureKind::Revolve(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != r {
                    *x = r;
                }
            }
            Some(if c.stage == 0 { "Click profiles to add/remove." } else { "Pick the axis: a sketch line, straight edge or origin axis." }.into())
        }
        CmdKind::Fillet | CmdKind::Chamfer => {
            let Some(fid) = fid else { return Some("Select edges (or a face to take all its edges).".into()) };
            let (n, mut v, label) = match app.doc.feature(fid).map(|f| &f.kind) {
                Some(FeatureKind::Fillet(f)) => (f.edges.len(), f.radius.clone(), "Radius"),
                Some(FeatureKind::Chamfer(f)) => (f.edges.len(), f.distance.clone(), "Distance"),
                _ => return None,
            };
            let mut clear = false;
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Edges"));
                ui.label(RichText::new(format!("{n} selected")).strong());
                clear = ui.small_button(ic::X).on_hover_text("Clear").clicked();
            });
            expr_field(ui, app, label, &mut v, "mm");
            match app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                Some(FeatureKind::Fillet(f)) => {
                    if f.radius != v {
                        f.radius = v;
                    }
                    if clear {
                        f.edges.clear();
                    }
                }
                Some(FeatureKind::Chamfer(f)) => {
                    if f.distance != v {
                        f.distance = v;
                    }
                    if clear {
                        f.edges.clear();
                    }
                }
                _ => {}
            }
            Some("Click edges to add/remove (a face adds all its edges). Drag the arrow or type a value.".into())
        }
        CmdKind::Shell => {
            let Some(fid) = fid else { return Some("Select the faces to remove.".into()) };
            let Some(FeatureKind::Shell(s)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut s = s.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Open faces"));
                ui.label(RichText::new(format!("{} selected", s.faces.len())).strong());
            });
            expr_field(ui, app, "Thickness", &mut s.thickness, "mm");
            if let Some(FeatureKind::Shell(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != s {
                    *x = s;
                }
            }
            Some("Click faces of one body to remove them.".into())
        }
        CmdKind::Plane => {
            if fid.is_none() {
                ui.horizontal(|ui| {
                    ui.add_sized([92.0, 22.0], egui::Label::new("Type"));
                    egui::ComboBox::from_id_salt("plane_kind")
                        .selected_text(match c.plane_kind {
                            PlaneKind::Offset => "Offset",
                            PlaneKind::Angle => "At angle",
                            PlaneKind::Midplane => "Midplane",
                            PlaneKind::NormalToEdge => "Normal to edge",
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut c.plane_kind, PlaneKind::Offset, "Offset");
                            ui.selectable_value(&mut c.plane_kind, PlaneKind::Angle, "At angle");
                            ui.selectable_value(&mut c.plane_kind, PlaneKind::Midplane, "Midplane");
                            ui.selectable_value(&mut c.plane_kind, PlaneKind::NormalToEdge, "Normal to edge");
                        });
                });
                return Some(
                    match c.plane_kind {
                        PlaneKind::Offset => "Select a plane or planar face to offset from.",
                        PlaneKind::Angle => "Pick the hinge line: an edge, sketch line or axis (then optionally a reference plane).",
                        PlaneKind::Midplane => "Pick two planes or planar faces.",
                        PlaneKind::NormalToEdge => "Pick an edge; the plane is perpendicular to it.",
                    }
                    .into(),
                );
            }
            let fid = fid.unwrap();
            let Some(FeatureKind::Plane(p)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut p = p.clone();
            let hint = match &mut p.mode {
                PlaneMode::Offset => "Click another plane/face to change the base. Drag the arrow to offset.",
                PlaneMode::Angle { angle, .. } => {
                    expr_field(ui, app, "Angle", angle, "deg");
                    "Click a line/edge/axis for the hinge, a plane/face for the reference."
                }
                PlaneMode::Midplane { .. } => {
                    ui.horizontal(|ui| {
                        ui.add_sized([92.0, 22.0], egui::Label::new("Picking"));
                        if ui.selectable_label(c.stage == 0, "First").clicked() {
                            c.stage = 0;
                        }
                        if ui.selectable_label(c.stage == 1, "Second").clicked() {
                            c.stage = 1;
                        }
                    });
                    "Pick the two planes or faces (parallel: halfway, angled: bisector)."
                }
                PlaneMode::NormalToEdge { at, .. } => {
                    expr_field(ui, app, "Position", at, "%");
                    "Position along the edge, 0 to 100 %."
                }
            };
            expr_field(ui, app, "Offset", &mut p.offset, "mm");
            if let Some(FeatureKind::Plane(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != p {
                    *x = p;
                }
            }
            Some(hint.into())
        }
        CmdKind::Split => {
            let Some(fid) = fid else { return Some("Select the bodies to split.".into()) };
            let Some(FeatureKind::Split(sp)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut sp = sp.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Bodies"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", sp.bodies.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Cut with"));
                let label = match &sp.tool {
                    PlaneRef::Origin(OriginPlane::XY) => "XY plane",
                    PlaneRef::Origin(OriginPlane::XZ) => "XZ plane",
                    PlaneRef::Origin(OriginPlane::YZ) => "YZ plane",
                    PlaneRef::Plane(_) => "Plane",
                    PlaneRef::Face(_) => "Face",
                };
                if ui.selectable_label(c.stage == 1, label).on_hover_text("Click, then pick a plane or planar face").clicked() {
                    c.stage = 1;
                }
                for (l, o) in [("XY", OriginPlane::XY), ("XZ", OriginPlane::XZ), ("YZ", OriginPlane::YZ)] {
                    if ui.small_button(l).clicked() {
                        sp.tool = PlaneRef::Origin(o);
                    }
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Keep"));
                ui.selectable_value(&mut sp.keep, SplitKeep::Both, "Both");
                ui.selectable_value(&mut sp.keep, SplitKeep::Front, "Front side");
                ui.selectable_value(&mut sp.keep, SplitKeep::Back, "Back side");
            });
            if let Some(FeatureKind::Split(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != sp {
                    *x = sp;
                }
            }
            Some(if c.stage == 0 { "Click bodies to add/remove." } else { "Pick the cutting plane (a construction plane or planar face). Tip: make a Plane (Offset) for any height." }.into())
        }
        CmdKind::Hole => {
            let Some(fid) = fid else { return Some("Click on a planar face where the hole goes (snaps to circle centres).".into()) };
            let Some(FeatureKind::Hole(h)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut h = h.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Preset"));
                egui::ComboBox::from_id_salt("hole_preset").selected_text("Choose…").height(320.0).show_ui(ui, |ui| {
                    for (label, kind, d, cd, x) in hole_presets() {
                        if ui.selectable_label(false, &label).clicked() {
                            let fmt = lusion_core::expr::fmt_num;
                            h.kind = kind;
                            h.diameter = fmt(d);
                            match kind {
                                HoleKind::Counterbore => {
                                    h.cb_diameter = fmt(cd);
                                    h.cb_depth = fmt(x);
                                }
                                HoleKind::Countersink => {
                                    h.cs_diameter = fmt(cd);
                                    h.cs_angle = fmt(x);
                                }
                                HoleKind::Simple => {
                                    if x > 0.0 {
                                        h.through = false;
                                        h.depth = fmt(x);
                                    }
                                }
                            }
                        }
                    }
                });
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Type"));
                ui.selectable_value(&mut h.kind, HoleKind::Simple, "Simple");
                ui.selectable_value(&mut h.kind, HoleKind::Counterbore, "Counterbore");
                ui.selectable_value(&mut h.kind, HoleKind::Countersink, "Countersink");
            });
            expr_field(ui, app, "Diameter", &mut h.diameter, "mm");
            ui.horizontal(|ui| {
                ui.add_space(96.0);
                ui.checkbox(&mut h.through, "Through all");
                if !h.through {
                    ui.checkbox(&mut h.drill_point, "Drill point");
                }
            });
            if !h.through {
                expr_field(ui, app, "Depth", &mut h.depth, "mm");
            }
            match h.kind {
                HoleKind::Counterbore => {
                    expr_field(ui, app, "Bore Ø", &mut h.cb_diameter, "mm");
                    expr_field(ui, app, "Bore depth", &mut h.cb_depth, "mm");
                }
                HoleKind::Countersink => {
                    expr_field(ui, app, "Sink Ø", &mut h.cs_diameter, "mm");
                    expr_field(ui, app, "Sink angle", &mut h.cs_angle, "deg");
                }
                HoleKind::Simple => {}
            }
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Positions"));
                ui.label(RichText::new(format!("{} clicked", h.points.len())).strong());
                if !h.points.is_empty() && ui.small_button(ic::X).on_hover_text("Clear").clicked() {
                    h.points.clear();
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Sketch points"));
                let st = app.model.current(&app.doc);
                let sketches: Vec<(FeatureId, String)> = app
                    .doc
                    .features
                    .iter()
                    .filter(|f| matches!(f.kind, FeatureKind::Sketch(_)) && st.sketches.contains_key(&f.id))
                    .map(|f| (f.id, f.name.clone()))
                    .collect();
                let cur = h.sketch.and_then(|s| sketches.iter().find(|x| x.0 == s)).map(|x| x.1.clone()).unwrap_or_else(|| "None".into());
                egui::ComboBox::from_id_salt("hole_sketch").selected_text(cur).show_ui(ui, |ui| {
                    ui.selectable_value(&mut h.sketch, None, "None");
                    for (id, name) in &sketches {
                        ui.selectable_value(&mut h.sketch, Some(*id), name);
                    }
                })
                .response
                .on_hover_text("Drill at every standalone point of a sketch (fully parametric placement)");
            });
            if let Some(FeatureKind::Hole(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != h {
                    *x = h;
                }
            }
            Some("Click the face to add holes (click a hole again to remove it).".into())
        }
        CmdKind::Thread => {
            let Some(fid) = fid else { return Some("Select a cylindrical face: a rod (external) or a hole (internal).".into()) };
            let Some(FeatureKind::Thread(t)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut t = t.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Size"));
                let cur = app.model.params.eval(&t.size).ok();
                let label = cur.map(|d| format!("M{}", lusion_core::expr::fmt_num(d))).unwrap_or_else(|| "custom".into());
                egui::ComboBox::from_id_salt("thread_size").selected_text(label).height(300.0).show_ui(ui, |ui| {
                    for (d, p) in ISO_COARSE {
                        if ui.selectable_label(cur == Some(d), format!("M{} × {}", lusion_core::expr::fmt_num(d), lusion_core::expr::fmt_num(p))).clicked() {
                            t.size = lusion_core::expr::fmt_num(d);
                            t.pitch = lusion_core::expr::fmt_num(p);
                        }
                    }
                });
            });
            expr_field(ui, app, "Diameter", &mut t.size, "mm");
            expr_field(ui, app, "Pitch", &mut t.pitch, "mm");
            expr_field(ui, app, "Length", &mut t.length, "mm");
            expr_field(ui, app, "Clearance", &mut t.clearance, "mm");
            ui.horizontal(|ui| {
                ui.add_space(96.0);
                ui.checkbox(&mut t.left, "Left-hand");
                ui.checkbox(&mut t.flip, "Start at other end");
            });
            if let Some(FeatureKind::Thread(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != t {
                    *x = t;
                }
            }
            Some("Modelled ISO thread, printable. Empty length = whole face. Clearance loosens the fit for printing (0.1–0.2 mm). For tapped holes use a Hole with the tap-drill preset.".into())
        }
        CmdKind::Sweep => {
            let Some(fid) = fid else { return Some("Select the profile to sweep.".into()) };
            let Some(FeatureKind::Sweep(sw)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut sw = sw.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Profiles"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", sw.profiles.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Path"));
                if ui.selectable_label(c.stage == 1, format!("{} curves", sw.path.len())).on_hover_text("Click, then pick sketch curves or edges in order").clicked() {
                    c.stage = 1;
                }
                if !sw.path.is_empty() && ui.small_button(ic::X).on_hover_text("Clear path").clicked() {
                    sw.path.clear();
                }
            });
            ui.checkbox(&mut sw.frenet, "Frenet frame (for helical paths)");
            op_selector(ui, &mut sw.op);
            if let Some(FeatureKind::Sweep(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != sw {
                    *x = sw;
                }
            }
            Some(if c.stage == 0 { "Click profiles to add/remove." } else { "Pick the path: connected sketch curves or edges." }.into())
        }
        CmdKind::Loft => {
            let Some(fid) = fid else { return Some("Select the first profile.".into()) };
            let Some(FeatureKind::Loft(l)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut l = l.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Sections"));
                ui.label(RichText::new(format!("{} in order", l.profiles.len())).strong());
                if l.profiles.len() > 1 && ui.small_button(ic::ARROWS_DOWN_UP).on_hover_text("Reverse order").clicked() {
                    l.profiles.reverse();
                }
            });
            ui.checkbox(&mut l.ruled, "Straight sections (ruled)");
            op_selector(ui, &mut l.op);
            if let Some(FeatureKind::Loft(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != l {
                    *x = l;
                }
            }
            Some("Click profiles in order (one per sketch or plane). Click again to remove.".into())
        }
        CmdKind::Joint => {
            let Some(fid) = fid else {
                return Some(if c.joint_a.is_none() { "1/2: pick a face or edge on the part that moves." } else { "2/2: pick where it attaches (face or edge on another part)." }.into());
            };
            let Some(FeatureKind::Joint(j)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut j = j.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Moving part"));
                ui.label(RichText::new(app.doc.component(j.moving).map(|c| c.name.clone()).unwrap_or_default()).strong());
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Re-pick"));
                if ui.selectable_label(c.stage == 0, "Moving").clicked() {
                    c.stage = 0;
                }
                if ui.selectable_label(c.stage == 1, "Target").clicked() {
                    c.stage = 1;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Motion"));
                ui.selectable_value(&mut j.kind, JointKind::Rigid, "Rigid");
                ui.selectable_value(&mut j.kind, JointKind::Revolute, "Revolute");
                ui.selectable_value(&mut j.kind, JointKind::Slider, "Slider");
            });
            expr_field(ui, app, "Angle", &mut j.angle, "deg");
            expr_field(ui, app, "Offset", &mut j.offset, "mm");
            ui.horizontal(|ui| {
                ui.add_space(96.0);
                ui.checkbox(&mut j.flip, "Flip");
            });
            if let Some(FeatureKind::Joint(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != j {
                    *x = j;
                }
            }
            Some("Faces mate face to face; circular edges line up their centres. Drag the ring to rotate, the arrow to slide.".into())
        }
        CmdKind::Measure => {
            match &c.measure_res {
                Some(m) if !m.rows.is_empty() => {
                    egui::Grid::new("measure").num_columns(2).spacing([12.0, 4.0]).show(ui, |ui| {
                        for (k, v) in &m.rows {
                            ui.label(RichText::new(k).color(theme::TEXT_DIM));
                            let r = ui.label(RichText::new(v).strong().monospace());
                            if r.on_hover_text("Click to copy").clicked() {
                                ui.ctx().copy_text(v.clone());
                            }
                            ui.end_row();
                        }
                    });
                }
                _ => {
                    ui.label(RichText::new("Nothing measured yet").color(theme::TEXT_DIM));
                }
            }
            if !c.measure.is_empty() && ui.small_button(format!("{} Clear", ic::X)).clicked() {
                c.measure.clear();
                c.measure_res = None;
            }
            Some("Click a face, edge or body for its size; click a second one for distance and angle.".into())
        }
        CmdKind::Section => {
            let mut off = false;
            if let Some(sv) = app.section.as_mut() {
                ui.horizontal(|ui| {
                    ui.add_sized([92.0, 22.0], egui::Label::new("Cut normal"));
                    for (l, n) in [("X", Vec3::X), ("Y", Vec3::Y), ("Z", Vec3::Z)] {
                        if ui.selectable_label(sv.normal.abs().abs_diff_eq(n, 1e-4), l).clicked() {
                            sv.normal = n;
                            sv.offset = 0.0;
                        }
                    }
                    if ui.small_button(format!("{} Flip", ic::ARROWS_DOWN_UP)).clicked() {
                        sv.normal = -sv.normal;
                        sv.offset = -sv.offset;
                    }
                });
                ui.horizontal(|ui| {
                    ui.add_sized([92.0, 22.0], egui::Label::new("Offset"));
                    ui.add(egui::DragValue::new(&mut sv.offset).speed(0.25).suffix(" mm"));
                });
                off = ui.button(format!("{} Turn section off", ic::EYE)).clicked();
            }
            if off {
                app.section = None;
            }
            Some("Drag the arrow to move the cut. Pick a plane or planar face to cut along it. The section stays on after closing.".into())
        }
        CmdKind::Combine => {
            let Some(fid) = fid else { return Some("Select the target body.".into()) };
            let Some(FeatureKind::Combine(cb)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut cb = cb.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Target"));
                let t = cb.target.map(|b| app.body_name(b)).unwrap_or_else(|| "—".into());
                if ui.selectable_label(c.stage == 0, t).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Tools"));
                if ui.selectable_label(c.stage == 1, format!("{} bodies", cb.tools.len())).clicked() {
                    c.stage = 1;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Operation"));
                ui.selectable_value(&mut cb.op, Operation::Join, RichText::new(ic::UNITE).size(16.0)).on_hover_text("Join");
                ui.selectable_value(&mut cb.op, Operation::Cut, RichText::new(ic::SUBTRACT).size(16.0)).on_hover_text("Cut");
                ui.selectable_value(&mut cb.op, Operation::Intersect, RichText::new(ic::INTERSECT).size(16.0)).on_hover_text("Intersect");
            });
            ui.checkbox(&mut cb.keep_tools, "Keep tool bodies");
            if let Some(FeatureKind::Combine(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != cb {
                    *x = cb;
                }
            }
            Some(if c.stage == 0 { "Pick the target body." } else { "Pick tool bodies." }.into())
        }
        CmdKind::Pattern => {
            let Some(fid) = fid else { return Some("Select bodies to pattern.".into()) };
            let Some(FeatureKind::Pattern(p)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut p = p.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Bodies"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", p.bodies.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Type"));
                ui.selectable_value(&mut p.kind, PatternKind::Circular, format!("{} Circular", ic::CIRCLE_DASHED));
                ui.selectable_value(&mut p.kind, PatternKind::Linear, format!("{} Linear", ic::DOTS_THREE));
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new(if p.kind == PatternKind::Circular { "Axis" } else { "Direction" }));
                let label = match &p.axis {
                    AxisRef::X => "X",
                    AxisRef::Y => "Y",
                    AxisRef::Z => "Z",
                    AxisRef::SketchLine { .. } => "Sketch line",
                    AxisRef::Edge(_) => "Edge",
                };
                if ui.selectable_label(c.stage == 1, label).on_hover_text("Click, then pick an edge / line / axis").clicked() {
                    c.stage = 1;
                }
                for (l, a) in [("X", AxisRef::X), ("Y", AxisRef::Y), ("Z", AxisRef::Z)] {
                    if ui.small_button(l).clicked() {
                        p.axis = a;
                    }
                }
            });
            expr_field(ui, app, "Count", &mut p.count, "");
            if p.kind == PatternKind::Circular {
                expr_field(ui, app, "Total angle", &mut p.angle, "deg");
            } else {
                expr_field(ui, app, "Spacing", &mut p.spacing, "mm");
                let mut grid = p.axis2.is_some();
                if ui.checkbox(&mut grid, "Second direction (grid)").changed() {
                    p.axis2 = if grid { Some(AxisRef::Y) } else { None };
                }
                if let Some(a2) = &mut p.axis2 {
                    ui.horizontal(|ui| {
                        ui.add_sized([92.0, 22.0], egui::Label::new("Direction 2"));
                        for (l, a) in [("X", AxisRef::X), ("Y", AxisRef::Y), ("Z", AxisRef::Z)] {
                            if ui.selectable_label(*a2 == a, l).clicked() {
                                *a2 = a;
                            }
                        }
                    });
                    expr_field(ui, app, "Count 2", &mut p.count2, "");
                    expr_field(ui, app, "Spacing 2", &mut p.spacing2, "mm");
                }
            }
            ui.checkbox(&mut p.join, "Join copies into the original");
            if let Some(FeatureKind::Pattern(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != p {
                    *x = p;
                }
            }
            Some(if c.stage == 0 { "Click bodies to add/remove." } else { "Pick the axis/direction." }.into())
        }
        CmdKind::Mirror => {
            let Some(fid) = fid else { return Some("Select bodies to mirror.".into()) };
            let Some(FeatureKind::Mirror(m)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut m = m.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Bodies"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", m.bodies.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Plane"));
                let label = match &m.plane {
                    PlaneRef::Origin(OriginPlane::XY) => "XY",
                    PlaneRef::Origin(OriginPlane::XZ) => "XZ",
                    PlaneRef::Origin(OriginPlane::YZ) => "YZ",
                    PlaneRef::Plane(_) => "Plane",
                    PlaneRef::Face(_) => "Face",
                };
                if ui.selectable_label(c.stage == 1, label).on_hover_text("Click, then pick a plane or planar face").clicked() {
                    c.stage = 1;
                }
                for (l, o) in [("XY", OriginPlane::XY), ("XZ", OriginPlane::XZ), ("YZ", OriginPlane::YZ)] {
                    if ui.small_button(l).clicked() {
                        m.plane = PlaneRef::Origin(o);
                    }
                }
            });
            ui.checkbox(&mut m.join, "Join with the original");
            if let Some(FeatureKind::Mirror(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != m {
                    *x = m;
                }
            }
            Some(if c.stage == 0 { "Click bodies to add/remove." } else { "Pick the mirror plane." }.into())
        }
        CmdKind::Move => {
            let Some(fid) = fid else { return Some("Select bodies to move.".into()) };
            let Some(FeatureKind::Move(m)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut m = m.clone();
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Bodies"));
                if ui.selectable_label(c.stage == 0, format!("{} selected", m.bodies.len())).clicked() {
                    c.stage = 0;
                }
            });
            ui.add_space(4.0);
            ui.label(RichText::new("Translate").small().strong().color(theme::TEXT_DIM));
            expr_field(ui, app, "X", &mut m.dx, "mm");
            expr_field(ui, app, "Y", &mut m.dy, "mm");
            expr_field(ui, app, "Z", &mut m.dz, "mm");
            ui.add_space(4.0);
            ui.label(RichText::new("Rotate").small().strong().color(theme::TEXT_DIM));
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Axis"));
                for (l, a) in [("X", AxisRef::X), ("Y", AxisRef::Y), ("Z", AxisRef::Z)] {
                    if ui.selectable_label(m.axis == a, l).clicked() {
                        m.axis = a;
                    }
                }
                let picked = matches!(m.axis, AxisRef::Edge(_) | AxisRef::SketchLine { .. });
                if ui.selectable_label(c.stage == 1 || picked, format!("{} Pick", ic::CURSOR_CLICK)).on_hover_text("Pick an edge or sketch line as the rotation axis").clicked() {
                    c.stage = 1;
                }
            });
            expr_field(ui, app, "Angle", &mut m.rz, "deg");
            ui.horizontal(|ui| {
                ui.add_sized([92.0, 22.0], egui::Label::new("Pivot"));
                let txt = match m.pivot {
                    Some(p) => format!("{}, {}, {}", lusion_core::expr::fmt_num(p[0]), lusion_core::expr::fmt_num(p[1]), lusion_core::expr::fmt_num(p[2])),
                    None => "body centre".into(),
                };
                ui.label(RichText::new(txt).color(theme::TEXT_DIM));
            });
            ui.horizontal(|ui| {
                ui.add_space(96.0);
                if ui.selectable_label(c.stage == 2, format!("{} Pick point", ic::CROSSHAIR)).on_hover_text("Click a face (its centre) or an edge (midpoint / circle centre)").clicked() {
                    c.stage = 2;
                }
                if m.pivot.is_some() && ui.small_button("Reset").clicked() {
                    m.pivot = None;
                }
            });
            ui.add_space(4.0);
            ui.checkbox(&mut m.copy, "Create copy");
            if let Some(FeatureKind::Move(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != m {
                    *x = m;
                }
            }
            Some(match c.stage {
                1 => "Pick the rotation axis: an edge, sketch line or origin axis.",
                2 => "Pick the pivot: a face (centre) or an edge (midpoint / circle centre).",
                _ => "Drag the arrows to move and the rings to rotate (Shift = 1° steps). Click bodies to add/remove.",
            }
            .into())
        }
    }
}
