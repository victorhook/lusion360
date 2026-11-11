//! Feature commands ("dialogs"): selection-driven creation and editing of
//! timeline features with live preview.

use crate::app::{App, Filter, Hit, Mode, Pick};
use crate::icons as ic;
use crate::render::Scene;
use crate::theme;
use crate::viewport::{push_line, View};
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
}

impl CmdKind {
    pub fn title(self) -> &'static str {
        match self {
            CmdKind::NewSketch => "Create Sketch",
            CmdKind::Extrude => "Extrude",
            CmdKind::Revolve => "Revolve",
            CmdKind::Fillet => "Fillet",
            CmdKind::Chamfer => "Chamfer",
            CmdKind::Shell => "Shell",
            CmdKind::Plane => "Offset Plane",
            CmdKind::Combine => "Combine",
            CmdKind::Move => "Move / Copy",
            CmdKind::Pattern => "Pattern",
            CmdKind::Mirror => "Mirror",
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
        }
    }
}

pub struct Command {
    pub kind: CmdKind,
    pub fid: Option<FeatureId>,
    pub editing: bool,
    pub before: Document,
    pub prev_marker: usize,
    pub stage: u8,
    drag: Option<f64>,
    /// automatic Join/Cut switching (until the user picks an operation)
    auto_op: Option<Operation>,
    /// guard against re-expanding text picks
    expanding: bool,
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
    let mut c = Command { kind, fid: None, editing: false, before: app.doc.clone(), prev_marker: app.doc.marker, stage: 0, drag: None, auto_op: None, expanding: false };
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
    if matches!(kind, CmdKind::Plane) && c.fid.is_none() {
        // default: offset from XY until something else is picked
    }
    app.cmd = Some(c);
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
    app.cmd = Some(Command { kind, fid: Some(fid), editing: true, before, prev_marker, stage: 0, drag: None, auto_op: None, expanding: false });
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
    pub fn wants_planes(&self) -> bool {
        matches!(self.kind, CmdKind::NewSketch | CmdKind::Plane) || (self.kind == CmdKind::Mirror && self.stage == 1)
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
            CmdKind::Plane => {
                f.faces = true;
                f.planar_only = true;
                f.planes = true;
            }
            CmdKind::Combine | CmdKind::Move => {
                f.bodies = true;
            }
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
        let st = app.pick_state();
        let Some(k) = self.feature(app) else { return vec![] };
        let mut v = Vec::new();
        match k {
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
        let st = app.pick_state().clone();
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
            CmdKind::Plane => {
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
                    self.insert(app, FeatureKind::Plane(OffsetPlane { base, offset: "10".into() }));
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
            CmdKind::Move => {
                let b = match hit.pick {
                    Pick::Body(b) | Pick::Face { body: b, .. } => b,
                    _ => return,
                };
                if self.fid.is_none() {
                    self.insert(
                        app,
                        FeatureKind::Move(MoveBodies { bodies: vec![b], dx: "0".into(), dy: "0".into(), dz: "0".into(), rz: "0".into(), copy: false }),
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
        let st = app.pick_state();
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

    /// Drag handling for in-viewport manipulators. Returns true if the pointer is consumed.
    pub fn handle_drag(&mut self, app: &mut App, ui: &egui::Ui, view: &View) -> bool {
        let Some((anchor, n, d)) = self.extrude_axis(app) else { return false };
        let Some(mouse) = view.mouse else {
            return self.drag.is_some();
        };
        let tip = (anchor + n * d).as_vec3();
        let Some(tip_s) = app.cam.project(tip, view.size) else { return false };
        let (pressed, down, released) = ui.input(|i| (i.pointer.primary_pressed(), i.pointer.primary_down(), i.pointer.primary_released()));
        if pressed && (tip_s - mouse).length() < 26.0 {
            self.drag = Some(d);
        }
        if self.drag.is_some() && down {
            // closest point on the extrude axis to the mouse ray
            let ray = app.cam.ray(mouse, view.size);
            let (p0, u) = (anchor.as_vec3(), n.as_vec3());
            let w0 = p0 - ray.o;
            let (a, b, c, dd, e) = (u.dot(u), u.dot(ray.d), ray.d.dot(ray.d), u.dot(w0), ray.d.dot(w0));
            let den = a * c - b * b;
            if den.abs() > 1e-9 {
                let s = (b * e - c * dd) / den;
                // snap to a "nice" step (1/2/5 x 10^n) worth ~3 pixels at the handle
                let px = app.cam.cam.px_size(view.size.y) * if app.cam.cam.ortho { 1.0 } else { (tip - app.cam.cam.eye()).length() / app.cam.cam.distance.max(1e-6) };
                let raw = (px * 3.0).max(1e-4);
                let mag = 10f32.powf(raw.log10().floor());
                let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag);
                let v = ((s / step).round() * step) as f64;
                if let Some(FeatureKind::Extrude(ex)) = self.feature_mut(app) {
                    let txt = lusion_core::expr::fmt_num(v);
                    if ex.distance != txt {
                        ex.distance = txt;
                    }
                }
                let mut ex2 = match self.feature(app) {
                    Some(FeatureKind::Extrude(e)) => e.clone(),
                    _ => return true,
                };
                self.apply_auto_op(app, &mut ex2);
                if let Some(FeatureKind::Extrude(ex)) = self.feature_mut(app) {
                    if ex.op != ex2.op {
                        ex.op = ex2.op;
                    }
                }
            }
            return true;
        }
        if released && self.drag.is_some() {
            self.drag = None;
            return true;
        }
        self.drag.is_some()
    }

    pub fn draw(&self, app: &mut App, scene: &mut Scene, view: &View) {
        if let Some((anchor, n, d)) = self.extrude_axis(app) {
            let a = anchor.as_vec3();
            let tip = (anchor + n * d).as_vec3();
            push_line(&mut scene.overlay_lines, a, tip, [0.25, 0.65, 1.0, 0.9], 2.0);
            // cone arrowhead, sized in screen pixels
            let px = app.cam.cam.px_size(view.size.y) * if app.cam.cam.ortho { 1.0 } else { (tip - app.cam.cam.eye()).length() / app.cam.cam.distance.max(1e-6) };
            let dir = (n.as_vec3() * if d < 0.0 { -1.0 } else { 1.0 }).normalize();
            let (h, r) = (px * 26.0, px * 9.0);
            let base = tip;
            let apex = base + dir * h;
            let u = dir.any_orthonormal_vector();
            let v = dir.cross(u);
            let segs = 20;
            for i in 0..segs {
                let t0 = i as f32 / segs as f32 * std::f32::consts::TAU;
                let t1 = (i + 1) as f32 / segs as f32 * std::f32::consts::TAU;
                let p0 = base + (u * t0.cos() + v * t0.sin()) * r;
                let p1 = base + (u * t1.cos() + v * t1.sin()) * r;
                // shade sides by facing for a bit of depth
                let side = (u * t0.cos() + v * t0.sin()).dot(-app.cam.cam.forward()).max(0.0);
                let c = [0.20 + 0.25 * side, 0.55 + 0.25 * side, 1.0, 0.97];
                for p in [apex, p0, p1] {
                    scene.fills.push(crate::render::FillVert { p: p.to_array(), color: c });
                }
                for p in [base, p1, p0] {
                    scene.fills.push(crate::render::FillVert { p: p.to_array(), color: [0.12, 0.35, 0.75, 0.97] });
                }
            }
            scene.points.push(crate::render::PointInst { p: a.to_array(), color: [0.9, 0.9, 0.95, 1.0], size: 7.0 });
        }
    }

    pub fn overlay(&mut self, app: &mut App, ui: &egui::Ui, view: &View) {
        if let Some((anchor, n, d)) = self.extrude_axis(app) {
            if let Some(p) = app.cam.project((anchor + n * d).as_vec3(), view.size) {
                let pos = view.to_screen(p + Vec2::new(14.0, -14.0));
                ui.painter().text(
                    pos,
                    egui::Align2::LEFT_BOTTOM,
                    format!("{} mm", lusion_core::expr::fmt_num(d)),
                    egui::FontId::proportional(13.0),
                    Color32::WHITE,
                );
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
            let can_ok = c.fid.is_some();
            if ui.add_enabled(can_ok, egui::Button::new(RichText::new(format!("{} OK", ic::CHECK)).strong()).min_size(egui::vec2(90.0, 28.0))).clicked() {
                finish = Some(true);
            }
            if ui.add(egui::Button::new(format!("{} Cancel", ic::X)).min_size(egui::vec2(90.0, 28.0))).clicked() {
                finish = Some(false);
            }
        });
        ui.label(RichText::new("Enter = OK · Esc = Cancel").small().color(theme::TEXT_DIM));
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
            Some("Click edges to add/remove. Clicking a face adds all of its edges.".into())
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
            let Some(fid) = fid else { return Some("Select a plane or planar face to offset from.".into()) };
            let Some(FeatureKind::Plane(p)) = app.doc.feature(fid).map(|f| &f.kind) else { return None };
            let mut p = p.clone();
            expr_field(ui, app, "Offset", &mut p.offset, "mm");
            if let Some(FeatureKind::Plane(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != p {
                    *x = p;
                }
            }
            None
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
            ui.label(format!("{} bodies", m.bodies.len()));
            expr_field(ui, app, "X", &mut m.dx, "mm");
            expr_field(ui, app, "Y", &mut m.dy, "mm");
            expr_field(ui, app, "Z", &mut m.dz, "mm");
            expr_field(ui, app, "Rotate Z", &mut m.rz, "deg");
            ui.checkbox(&mut m.copy, "Create copy");
            if let Some(FeatureKind::Move(x)) = app.doc.feature_mut(fid).map(|f| &mut f.kind) {
                if *x != m {
                    *x = m;
                }
            }
            None
        }
    }
}
