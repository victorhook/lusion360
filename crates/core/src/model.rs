//! Timeline evaluation: turns the parametric `Document` into geometry.
//!
//! Each feature is evaluated on top of the state left by the previous one.
//! States are cached per feature, so editing feature *k* only recomputes
//! features *k..marker*. Faces carry persistent names derived from the
//! operation history (see `propagate`), which is how later features keep
//! referring to "the same" face/edge after upstream edits.

use crate::doc::*;
use crate::expr::ParamValues;
use crate::sketch::{CurveGeom, Frame, SolveReport, Sketch};
use glam::{DMat4, DQuat, DVec2, DVec3};
use lusion_kernel::{self as k, BoolKind, CurveKind, Op, Shape, ShapeKind, SurfaceKind};
use std::collections::{BTreeMap, HashMap, HashSet};
use std::hash::{Hash, Hasher};
use std::sync::Arc;
use std::time::Instant;

#[derive(Clone)]
pub struct Body {
    pub id: BodyId,
    pub component: CompId,
    /// B-rep (null for mesh-only bodies)
    pub shape: Shape,
    /// persistent name per face (index = `shape.faces()` order)
    pub face_names: Arc<Vec<String>>,
    /// persistent name per edge (index = `shape.edges()` order)
    pub edge_names: Arc<Vec<String>>,
    pub mesh: Option<Arc<ImportMesh>>,
}

impl Body {
    pub fn is_mesh(&self) -> bool {
        self.mesh.is_some()
    }
    pub fn face_index(&self, name: &str) -> Option<usize> {
        self.face_names.iter().position(|n| n == name)
    }
    pub fn edge_index(&self, name: &str) -> Option<usize> {
        self.edge_names.iter().position(|n| n == name)
    }
}

pub struct Profile {
    pub face: Shape,
    pub curves: Vec<u32>,
    pub hint: DVec2,
    pub tris: Vec<[DVec2; 3]>,
    pub area: f64,
    /// edge hash → persistent name ("s<sketch>.c<curve>")
    pub edge_names: HashMap<u64, String>,
}

impl Profile {
    pub fn contains(&self, p: DVec2) -> bool {
        self.tris.iter().any(|t| point_in_tri(p, t))
    }
}

pub fn point_in_tri(p: DVec2, t: &[DVec2; 3]) -> bool {
    let d1 = (t[1] - t[0]).perp_dot(p - t[0]);
    let d2 = (t[2] - t[1]).perp_dot(p - t[1]);
    let d3 = (t[0] - t[2]).perp_dot(p - t[2]);
    let neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(neg && pos)
}

pub struct SolvedSketch {
    pub frame: Frame,
    pub sketch: Sketch,
    pub report: SolveReport,
    pub profiles: Vec<Profile>,
}

#[derive(Clone, Default)]
pub struct ModelState {
    pub bodies: BTreeMap<BodyId, Body>,
    pub sketches: HashMap<FeatureId, Arc<SolvedSketch>>,
    pub planes: HashMap<FeatureId, Frame>,
}

#[derive(Clone, Debug, Default)]
pub struct FeatureStatus {
    pub error: Option<String>,
    pub warnings: Vec<String>,
    pub millis: f64,
}

#[derive(Default)]
pub struct Model {
    states: Vec<ModelState>,
    pub status: Vec<FeatureStatus>,
    fingerprints: Vec<u64>,
    param_fp: u64,
    pub params: ParamValues,
    empty: ModelState,
    /// total milliseconds spent in the last update
    pub last_update_ms: f64,
}

fn fingerprint(f: &Feature) -> u64 {
    let mut h = std::collections::hash_map::DefaultHasher::new();
    f.id.hash(&mut h);
    f.suppressed.hash(&mut h);
    f.component.hash(&mut h);
    match &f.kind {
        FeatureKind::ImportShape(i) => {
            (Arc::as_ptr(&i.brep) as usize).hash(&mut h);
            i.brep.len().hash(&mut h);
        }
        FeatureKind::ImportMesh(m) => {
            (Arc::as_ptr(&m.positions) as usize).hash(&mut h);
            (Arc::as_ptr(&m.indices) as usize).hash(&mut h);
        }
        other => {
            // Sketch dimension "value" fields are outputs; strip by serializing a clone
            let mut kind = other.clone();
            if let FeatureKind::Sketch(s) = &mut kind {
                for c in &mut s.constraints {
                    if let crate::sketch::Constraint::Dimension { value, .. } = &mut c.c {
                        *value = 0.0;
                    }
                }
            }
            serde_json::to_string(&kind).unwrap_or_default().hash(&mut h);
        }
    }
    h.finish()
}

impl Model {
    pub fn new() -> Self {
        Self::default()
    }

    /// Bring cached states up to date with `doc` (up to the timeline marker).
    /// Returns true if anything was recomputed.
    pub fn update(&mut self, doc: &Document) -> bool {
        let t0 = Instant::now();
        let pfp = {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            serde_json::to_string(&doc.parameters).unwrap_or_default().hash(&mut h);
            h.finish()
        };
        if pfp != self.param_fp {
            self.param_fp = pfp;
            self.params = ParamValues::evaluate(&doc.parameters);
            self.truncate(0);
        }
        let n = self.states.len().min(doc.features.len());
        let mut first_bad = n;
        for i in 0..n {
            if fingerprint(&doc.features[i]) != self.fingerprints[i] {
                first_bad = i;
                break;
            }
        }
        if first_bad < self.states.len() {
            self.truncate(first_bad);
        }
        let marker = doc.marker.min(doc.features.len());
        let mut changed = false;
        while self.states.len() < marker {
            let i = self.states.len();
            let f = &doc.features[i];
            let prev = if i == 0 { self.empty.clone() } else { self.states[i - 1].clone() };
            let mut st = prev.clone();
            let mut status = FeatureStatus::default();
            let ft = Instant::now();
            if !f.suppressed {
                let mut ctx = Ctx { doc, params: &self.params, warnings: vec![] };
                match ctx.eval(f, &mut st) {
                    Ok(()) => {}
                    Err(e) => {
                        status.error = Some(e);
                        st = prev;
                    }
                }
                status.warnings = ctx.warnings;
            }
            status.millis = ft.elapsed().as_secs_f64() * 1000.0;
            self.states.push(st);
            self.status.push(status);
            self.fingerprints.push(fingerprint(f));
            changed = true;
        }
        self.last_update_ms = t0.elapsed().as_secs_f64() * 1000.0;
        changed
    }

    fn truncate(&mut self, n: usize) {
        self.states.truncate(n);
        self.status.truncate(n);
        self.fingerprints.truncate(n);
    }

    pub fn invalidate(&mut self) {
        self.truncate(0);
        self.param_fp = 0;
    }

    /// State after the first `n` features (n = marker gives the current model).
    pub fn state_after(&self, n: usize) -> &ModelState {
        if n == 0 || self.states.is_empty() {
            return &self.empty;
        }
        &self.states[(n - 1).min(self.states.len() - 1)]
    }

    pub fn current(&self, doc: &Document) -> &ModelState {
        self.state_after(doc.marker)
    }

    pub fn feature_status(&self, idx: usize) -> Option<&FeatureStatus> {
        self.status.get(idx)
    }
}

// ------------------------------------------------------------------ naming

struct Named {
    shape: Shape,
    names: Vec<String>,
}

fn face_center(f: &Shape) -> DVec3 {
    f.face_info().map(|i| i.center).unwrap_or(DVec3::ZERO)
}

fn cmp_vec(a: DVec3, b: DVec3) -> std::cmp::Ordering {
    let r = |v: f64| (v * 1e4).round() as i64;
    (r(a.x), r(a.y), r(a.z)).cmp(&(r(b.x), r(b.y), r(b.z)))
}

/// Make names unique by suffixing duplicates (ordered by geometry).
fn dedupe(names: &mut [String], pos: impl Fn(usize) -> DVec3) {
    let mut groups: HashMap<String, Vec<usize>> = HashMap::new();
    for (i, n) in names.iter().enumerate() {
        groups.entry(n.clone()).or_default().push(i);
    }
    for (name, mut idx) in groups {
        if idx.len() < 2 {
            continue;
        }
        let keyed: Vec<(usize, DVec3)> = idx.iter().map(|&i| (i, pos(i))).collect();
        idx.sort_by(|a, b| {
            let pa = keyed.iter().find(|k| k.0 == *a).unwrap().1;
            let pb = keyed.iter().find(|k| k.0 == *b).unwrap().1;
            cmp_vec(pa, pb)
        });
        for (k, i) in idx.into_iter().enumerate() {
            if k > 0 {
                names[i] = format!("{name}~{k}");
            }
        }
    }
}

struct FaceIndex {
    faces: Vec<Shape>,
    by_hash: HashMap<u64, Vec<usize>>,
}

impl FaceIndex {
    fn new(faces: Vec<Shape>) -> Self {
        let mut by_hash: HashMap<u64, Vec<usize>> = HashMap::new();
        for (i, f) in faces.iter().enumerate() {
            by_hash.entry(f.hash()).or_default().push(i);
        }
        FaceIndex { faces, by_hash }
    }
    fn find(&self, s: &Shape) -> Option<usize> {
        self.by_hash.get(&s.hash())?.iter().copied().find(|&i| self.faces[i].same(s))
    }
}

/// Carry face names from `inputs` through `op` onto `result`.
/// `generated` lists (input sub-shape, name) pairs whose generated faces get that name.
fn propagate(op: &Op, inputs: &[&Named], generated: &[(Shape, String)], result: Shape, fallback: &str) -> Named {
    let idx = FaceIndex::new(result.faces());
    let mut names: Vec<Option<String>> = vec![None; idx.faces.len()];
    for inp in inputs {
        for (i, f) in inp.shape.faces().iter().enumerate() {
            let name = &inp.names[i];
            let mods = op.modified(f);
            if !mods.is_empty() {
                for m in mods.iter().filter(|m| m.kind() == ShapeKind::Face) {
                    if let Some(k) = idx.find(m) {
                        names[k].get_or_insert_with(|| name.clone());
                    }
                }
            } else if let Some(k) = idx.find(f) {
                names[k].get_or_insert_with(|| name.clone());
            }
        }
    }
    for (s, name) in generated {
        for g in op.generated(s).iter().filter(|g| g.kind() == ShapeKind::Face) {
            if let Some(k) = idx.find(g) {
                names[k].get_or_insert_with(|| name.clone());
            }
        }
    }
    let mut names: Vec<String> = names.into_iter().map(|n| n.unwrap_or_else(|| fallback.to_string())).collect();
    let faces = &idx.faces;
    dedupe(&mut names, |i| face_center(&faces[i]));
    Named { shape: result, names }
}

/// Restrict a named compound to one of its solids.
fn sub_named(parent: &Named, part: Shape) -> Named {
    let idx = FaceIndex::new(parent.shape.faces());
    let names = part
        .faces()
        .iter()
        .map(|f| idx.find(f).map(|i| parent.names[i].clone()).unwrap_or_else(|| "?".into()))
        .collect();
    Named { shape: part, names }
}

pub fn compute_edge_names(shape: &Shape, face_names: &[String]) -> Vec<String> {
    let ef = shape.edge_faces();
    let edges = shape.edges();
    let mut names: Vec<String> = ef
        .iter()
        .map(|&(a, b)| {
            let na = face_names.get(a.max(0) as usize).map(|s| s.as_str()).unwrap_or("?");
            let nb = if b >= 0 { face_names.get(b as usize).map(|s| s.as_str()).unwrap_or("?") } else { na };
            if a < 0 {
                "free".to_string()
            } else if na <= nb {
                format!("{na}|{nb}")
            } else {
                format!("{nb}|{na}")
            }
        })
        .collect();
    dedupe(&mut names, |i| edges[i].edge_info().map(|e| e.mid).unwrap_or(DVec3::ZERO));
    names
}

fn make_body(id: BodyId, component: CompId, named: Named) -> Body {
    let edge_names = compute_edge_names(&named.shape, &named.names);
    Body {
        id,
        component,
        shape: named.shape,
        face_names: Arc::new(named.names),
        edge_names: Arc::new(edge_names),
        mesh: None,
    }
}

fn body_named(b: &Body) -> Named {
    Named { shape: b.shape.clone(), names: (*b.face_names).clone() }
}

// ------------------------------------------------------------------ evaluation

struct Ctx<'a> {
    doc: &'a Document,
    params: &'a ParamValues,
    warnings: Vec<String>,
}

fn origin_frame(p: OriginPlane) -> Frame {
    match p {
        OriginPlane::XY => Frame::new(DVec3::ZERO, DVec3::Z, DVec3::X),
        OriginPlane::XZ => Frame::new(DVec3::ZERO, DVec3::NEG_Y, DVec3::X),
        OriginPlane::YZ => Frame::new(DVec3::ZERO, DVec3::X, DVec3::Y),
    }
}

/// Default sketch frame on a planar face: origin = projection of the world
/// origin, x axis horizontal where possible.
pub fn face_frame(origin_on_plane: DVec3, n: DVec3) -> Frame {
    let n = n.normalize();
    let o = n * n.dot(origin_on_plane);
    let xh = if n.z.abs() > 0.9 { DVec3::X } else { DVec3::Z.cross(n) };
    Frame::new(o, n, xh)
}

struct ResolvedProfiles {
    faces: Vec<Shape>,
    normal: DVec3,
    edge_names: HashMap<u64, String>,
    default_target: Option<BodyId>,
}

impl Ctx<'_> {
    fn eval_expr(&self, what: &str, src: &str) -> Result<f64, String> {
        let src = if src.trim().is_empty() { "0" } else { src };
        self.params.eval(src).map_err(|e| format!("{what}: {e}"))
    }

    fn eval(&mut self, f: &Feature, st: &mut ModelState) -> Result<(), String> {
        match &f.kind {
            FeatureKind::Sketch(s) => self.sketch(f, s, st),
            FeatureKind::Extrude(e) => self.extrude(f, e, st),
            FeatureKind::Revolve(r) => self.revolve(f, r, st),
            FeatureKind::Fillet(x) => self.fillet_chamfer(f, &x.edges, &x.radius, true, st),
            FeatureKind::Chamfer(x) => self.fillet_chamfer(f, &x.edges, &x.distance, false, st),
            FeatureKind::Shell(s) => self.shell(f, s, st),
            FeatureKind::Plane(p) => {
                let base = self.resolve_plane(st, &p.base)?;
                let d = self.eval_expr("offset", &p.offset)?;
                st.planes.insert(f.id, base.0.offset(d));
                Ok(())
            }
            FeatureKind::Combine(c) => self.combine(f, c, st),
            FeatureKind::Move(m) => self.move_bodies(f, m, st),
            FeatureKind::Pattern(p) => self.pattern(f, p, st),
            FeatureKind::Mirror(m) => self.mirror(f, m, st),
            FeatureKind::ImportShape(i) => {
                let shape = Shape::from_brep_string(&i.brep).map_err(|e| e.0)?;
                let mut solids = shape.solids();
                if solids.is_empty() {
                    solids = vec![shape];
                }
                for (k, s) in solids.into_iter().enumerate() {
                    let names = (0..s.faces().len()).map(|j| format!("f{}.i{}", f.id, j)).collect();
                    let id = BodyId::new(f.id, k as u32);
                    st.bodies.insert(id, make_body(id, f.component, Named { shape: s, names }));
                }
                Ok(())
            }
            FeatureKind::ImportMesh(m) => {
                let id = BodyId::new(f.id, 0);
                st.bodies.insert(
                    id,
                    Body {
                        id,
                        component: f.component,
                        shape: Shape::null(),
                        face_names: Arc::new(vec![]),
                        edge_names: Arc::new(vec![]),
                        mesh: Some(Arc::new(m.clone())),
                    },
                );
                Ok(())
            }
        }
    }

    // ---- references

    fn resolve_plane(&mut self, st: &ModelState, p: &PlaneRef) -> Result<(Frame, Option<BodyId>), String> {
        match p {
            PlaneRef::Origin(o) => Ok((origin_frame(*o), None)),
            PlaneRef::Plane(fid) => {
                st.planes.get(fid).copied().map(|f| (f, None)).ok_or_else(|| "construction plane is missing".into())
            }
            PlaneRef::Face(fr) => {
                let (body, face) = self.resolve_face(st, fr)?;
                let info = face.face_info().map_err(|e| e.0)?;
                if info.kind != SurfaceKind::Plane {
                    return Err("sketch face is not planar".into());
                }
                Ok((face_frame(info.origin, info.normal), Some(body.id)))
            }
        }
    }

    fn resolve_face<'s>(&mut self, st: &'s ModelState, fr: &FaceRef) -> Result<(&'s Body, Shape), String> {
        let body = st.bodies.get(&fr.body).ok_or_else(|| "referenced body no longer exists".to_string())?;
        let faces = body.shape.faces();
        if let Some(i) = body.face_index(&fr.name) {
            return Ok((body, faces[i].clone()));
        }
        // fallback: nearest face centre
        let hint = DVec3::from_array(fr.hint);
        let best = faces
            .iter()
            .enumerate()
            .map(|(i, f)| (i, (face_center(f) - hint).length()))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        match best {
            Some((i, _)) => {
                self.warnings.push(format!("face '{}' was lost; re-attached by position", fr.name));
                Ok((body, faces[i].clone()))
            }
            None => Err("referenced face no longer exists".into()),
        }
    }

    fn resolve_edge(&mut self, body: &Body, er: &EdgeRef) -> Result<Shape, String> {
        let edges = body.shape.edges();
        if let Some(i) = body.edge_index(&er.name) {
            return Ok(edges[i].clone());
        }
        let hint = DVec3::from_array(er.hint);
        let best = edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| e.edge_info().ok().map(|inf| (i, (inf.mid - hint).length())))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
        match best {
            Some((i, d)) if d < 1e3 => {
                self.warnings.push(format!("edge '{}' was lost; re-attached by position", er.name));
                Ok(edges[i].clone())
            }
            _ => Err("referenced edge no longer exists".into()),
        }
    }

    fn resolve_profiles(&mut self, st: &ModelState, refs: &[ProfileRef]) -> Result<ResolvedProfiles, String> {
        if refs.is_empty() {
            return Err("no profile selected".into());
        }
        let mut out = ResolvedProfiles { faces: vec![], normal: DVec3::Z, edge_names: HashMap::new(), default_target: None };
        let mut normal: Option<DVec3> = None;
        for r in refs {
            match r {
                ProfileRef::Sketch { sketch, curves, hint } => {
                    let ss = st.sketches.get(sketch).ok_or("profile sketch is missing (suppressed or rolled back?)")?;
                    let p = find_profile(ss, curves, DVec2::from_array(*hint))
                        .ok_or("a selected profile no longer exists (sketch changed?)")?;
                    out.faces.push(p.face.clone());
                    out.edge_names.extend(p.edge_names.iter().map(|(k, v)| (*k, v.clone())));
                    normal.get_or_insert(ss.frame.n);
                    if out.default_target.is_none() {
                        if let Some(FeatureKind::Sketch(s)) = self.doc.feature(*sketch).map(|f| &f.kind) {
                            if let PlaneRef::Face(fr) = &s.plane {
                                out.default_target = Some(fr.body);
                            }
                        }
                    }
                }
                ProfileRef::Face(fr) => {
                    let (body, face) = self.resolve_face(st, fr)?;
                    let info = face.face_info().map_err(|e| e.0)?;
                    if info.kind != SurfaceKind::Plane {
                        return Err("only planar faces can be extruded".into());
                    }
                    let eidx: HashMap<u64, usize> =
                        body.shape.edges().iter().enumerate().map(|(i, e)| (e.hash(), i)).collect();
                    for e in face.edges() {
                        if let Some(&i) = eidx.get(&e.hash()) {
                            out.edge_names.insert(e.hash(), body.edge_names[i].clone());
                        }
                    }
                    normal.get_or_insert(info.normal);
                    out.default_target.get_or_insert(body.id);
                    out.faces.push(face);
                }
            }
        }
        out.normal = normal.unwrap_or(DVec3::Z);
        Ok(out)
    }

    // ---- features

    fn sketch(&mut self, f: &Feature, s: &Sketch, st: &mut ModelState) -> Result<(), String> {
        let (frame, _) = self.resolve_plane(st, &s.plane)?;
        let mut solved = s.clone();
        let report = solved.solve(self.params, &[], true);
        if !report.converged {
            self.warnings.push("sketch constraints could not all be satisfied".into());
        }
        for (_, e) in &report.failed {
            if !self.warnings.contains(e) {
                self.warnings.push(e.clone());
            }
        }
        let profiles = match compute_profiles(&solved, &frame, f.id) {
            Ok(p) => p,
            Err(e) => {
                self.warnings.push(format!("profile detection failed: {e}"));
                vec![]
            }
        };
        st.sketches.insert(f.id, Arc::new(SolvedSketch { frame, sketch: solved, report, profiles }));
        Ok(())
    }

    fn extrude(&mut self, f: &Feature, e: &Extrude, st: &mut ModelState) -> Result<(), String> {
        let prof = self.resolve_profiles(st, &e.profiles)?;
        let d = self.eval_expr("distance", &e.distance)?;
        let d2 = self.eval_expr("distance 2", &e.distance2)?;
        let start = self.eval_expr("start offset", &e.start_offset)?;
        let (off, len) = match e.mode {
            ExtentMode::OneSide => (start, d),
            ExtentMode::Symmetric => (start - d, 2.0 * d),
            ExtentMode::TwoSides => (start - d2, d + d2),
        };
        if len.abs() < 1e-9 {
            return Err("extrude distance is zero".into());
        }
        let n = prof.normal.normalize();
        let mut profile = if prof.faces.len() == 1 { prof.faces[0].clone() } else { Shape::compound(&prof.faces) };
        let mut edge_names = prof.edge_names;
        if off.abs() > 1e-12 {
            let mv = k::transform(&profile, &DMat4::from_translation(n * off)).map_err(|e| e.0)?;
            let mut moved = HashMap::new();
            for edge in profile.edges() {
                if let Some(name) = edge_names.get(&edge.hash()) {
                    for m in mv.modified(&edge) {
                        moved.insert(m.hash(), name.clone());
                    }
                }
            }
            edge_names = moved;
            profile = mv.shape();
        }
        let op = k::prism(&profile, n * len).map_err(|e| e.0)?;
        let tool = self.name_sweep(f.id, &op, &profile, &edge_names);
        self.apply_tool(f, e.op, &e.targets, prof.default_target, tool, st)
    }

    /// Name the faces of an extrusion / revolution result.
    fn name_sweep(&self, fid: FeatureId, op: &Op, profile: &Shape, edge_names: &HashMap<u64, String>) -> Named {
        let result = op.shape();
        let idx = FaceIndex::new(result.faces());
        let mut names: Vec<Option<String>> = vec![None; idx.faces.len()];
        let first = op.first();
        let last = op.last();
        for (cap, label) in [(&first, "start"), (&last, "end")] {
            if cap.is_null() {
                continue;
            }
            for fc in cap.faces() {
                if let Some(k) = idx.find(&fc) {
                    names[k].get_or_insert_with(|| format!("f{fid}.{label}"));
                }
            }
        }
        for edge in profile.edges() {
            let en = edge_names.get(&edge.hash()).cloned().unwrap_or_else(|| "?".into());
            for g in op.generated(&edge) {
                if g.kind() == ShapeKind::Face {
                    if let Some(k) = idx.find(&g) {
                        names[k].get_or_insert_with(|| format!("f{fid}.side({en})"));
                    }
                }
            }
        }
        let mut names: Vec<String> = names.into_iter().map(|n| n.unwrap_or_else(|| format!("f{fid}.new"))).collect();
        let faces = &idx.faces;
        dedupe(&mut names, |i| face_center(&faces[i]));
        Named { shape: result, names }
    }

    /// Combine a freshly built tool solid with the model according to `op`.
    fn apply_tool(
        &mut self,
        f: &Feature,
        op: Operation,
        targets: &[BodyId],
        default_target: Option<BodyId>,
        tool: Named,
        st: &mut ModelState,
    ) -> Result<(), String> {
        let tool_solids = tool.shape.solids();
        if tool_solids.is_empty() {
            return Err("operation produced no solid".into());
        }
        let fallback = format!("f{}.new", f.id);
        // Explicit targets, or automatic: bodies overlapping the tool.
        let pick_targets = |st: &ModelState, same_comp: bool| -> Vec<BodyId> {
            if !targets.is_empty() {
                return targets.iter().copied().filter(|b| st.bodies.contains_key(b)).collect();
            }
            let tb = tool.shape.bbox();
            let mut v: Vec<BodyId> = st
                .bodies
                .values()
                .filter(|b| !b.is_mesh() && (!same_comp || b.component == f.component))
                .filter(|b| match (b.shape.bbox(), tb) {
                    (Some((a0, a1)), Some((b0, b1))) => {
                        let eps = 1e-6;
                        a0.x <= b1.x + eps
                            && b0.x <= a1.x + eps
                            && a0.y <= b1.y + eps
                            && b0.y <= a1.y + eps
                            && a0.z <= b1.z + eps
                            && b0.z <= a1.z + eps
                    }
                    _ => false,
                })
                .map(|b| b.id)
                .collect();
            if let Some(d) = default_target {
                if st.bodies.contains_key(&d) && !v.contains(&d) && op == Operation::Join {
                    v.insert(0, d);
                } else if let Some(pos) = v.iter().position(|b| *b == d) {
                    v.swap(0, pos);
                }
            }
            v
        };
        match op {
            Operation::NewBody => {
                self.add_new_bodies(f, tool, st, &fallback)?;
            }
            Operation::Join => {
                let mut tg = pick_targets(st, true);
                if tg.is_empty() {
                    tg = pick_targets(st, false);
                }
                if tg.is_empty() {
                    self.warnings.push("nothing to join; created a new body".into());
                    return self.add_new_bodies(f, tool, st, &fallback);
                }
                let first = st.bodies[&tg[0]].clone();
                let first_n = body_named(&first);
                let others: Vec<Named> = tg[1..].iter().map(|b| body_named(&st.bodies[b])).collect();
                let mut tools: Vec<&Shape> = others.iter().map(|n| &n.shape).collect();
                tools.push(&tool.shape);
                let bop = k::boolean(BoolKind::Fuse, &[&first_n.shape], &tools, true).map_err(|e| e.0)?;
                let mut inputs: Vec<&Named> = vec![&first_n];
                inputs.extend(others.iter());
                inputs.push(&tool);
                let res = propagate(&bop, &inputs, &[], bop.shape(), &fallback);
                for b in &tg[1..] {
                    st.bodies.remove(b);
                }
                st.bodies.insert(first.id, make_body(first.id, first.component, res));
            }
            Operation::Cut | Operation::Intersect => {
                let tg = pick_targets(st, false);
                if tg.is_empty() {
                    return Err(format!("nothing to {}", if op == Operation::Cut { "cut" } else { "intersect" }));
                }
                let kind = if op == Operation::Cut { BoolKind::Cut } else { BoolKind::Common };
                for b in tg {
                    let body = st.bodies[&b].clone();
                    let bn = body_named(&body);
                    let bop = k::boolean(kind, &[&bn.shape], &[&tool.shape], true).map_err(|e| e.0)?;
                    let res = bop.shape();
                    if res.solids().is_empty() {
                        st.bodies.remove(&b);
                        continue;
                    }
                    let named = propagate(&bop, &[&bn, &tool], &[], res, &fallback);
                    st.bodies.insert(b, make_body(b, body.component, named));
                }
            }
        }
        Ok(())
    }

    fn add_new_bodies(&mut self, f: &Feature, tool: Named, st: &mut ModelState, fallback: &str) -> Result<(), String> {
        let solids = tool.shape.solids();
        let merged = if solids.len() > 1 {
            let refs: Vec<&Shape> = solids.iter().collect();
            let bop = k::boolean(BoolKind::Fuse, &refs[..1], &refs[1..], true).map_err(|e| e.0)?;
            propagate(&bop, &[&tool], &[], bop.shape(), fallback)
        } else {
            tool
        };
        let parts = merged.shape.solids();
        let mut k = 0;
        while st.bodies.contains_key(&BodyId::new(f.id, k)) {
            k += 1;
        }
        for part in parts {
            let id = BodyId::new(f.id, k);
            k += 1;
            st.bodies.insert(id, make_body(id, f.component, sub_named(&merged, part)));
        }
        Ok(())
    }

    fn revolve(&mut self, f: &Feature, r: &Revolve, st: &mut ModelState) -> Result<(), String> {
        let prof = self.resolve_profiles(st, &r.profiles)?;
        let (o, dir) = self.resolve_axis(st, &r.axis)?;
        let ang = self.eval_expr("angle", &r.angle)?;
        let profile = if prof.faces.len() == 1 { prof.faces[0].clone() } else { Shape::compound(&prof.faces) };
        let op = k::revolve(&profile, o, dir, ang.to_radians()).map_err(|e| e.0)?;
        let tool = self.name_sweep(f.id, &op, &profile, &prof.edge_names);
        self.apply_tool(f, r.op, &r.targets, prof.default_target, tool, st)
    }

    fn resolve_axis(&mut self, st: &ModelState, axis: &AxisRef) -> Result<(DVec3, DVec3), String> {
        Ok(match axis {
            AxisRef::X => (DVec3::ZERO, DVec3::X),
            AxisRef::Y => (DVec3::ZERO, DVec3::Y),
            AxisRef::Z => (DVec3::ZERO, DVec3::Z),
            AxisRef::SketchLine { sketch, curve } => {
                let ss = st.sketches.get(sketch).ok_or("axis sketch is missing")?;
                let (a, b) = ss.sketch.line_pts(*curve).ok_or("axis line no longer exists")?;
                let (a3, b3) = (ss.frame.to3(a), ss.frame.to3(b));
                (a3, (b3 - a3).normalize())
            }
            AxisRef::Edge(er) => {
                let body = st.bodies.get(&er.body).ok_or("axis body is missing")?.clone();
                let e = self.resolve_edge(&body, er)?;
                let inf = e.edge_info().map_err(|e| e.0)?;
                match inf.kind {
                    CurveKind::Line => (inf.start, (inf.end - inf.start).normalize()),
                    CurveKind::Circle => (inf.center, inf.axis.normalize()),
                    _ => return Err("axis edge must be straight or circular".into()),
                }
            }
        })
    }

    /// Put `copies` (already named) into the model: fused into `orig` or as new bodies.
    fn place_copies(&mut self, f: &Feature, orig: &Body, copies: Vec<Named>, join: bool, next: &mut u32, st: &mut ModelState) -> Result<(), String> {
        if copies.is_empty() {
            return Ok(());
        }
        if join {
            let on = body_named(orig);
            let tools: Vec<&Shape> = copies.iter().map(|c| &c.shape).collect();
            let bop = k::boolean(BoolKind::Fuse, &[&on.shape], &tools, true).map_err(|e| e.0)?;
            let mut inputs = vec![&on];
            inputs.extend(copies.iter());
            let res = propagate(&bop, &inputs, &[], bop.shape(), &format!("f{}.new", f.id));
            st.bodies.insert(orig.id, make_body(orig.id, orig.component, res));
        } else {
            for c in copies {
                let id = BodyId::new(f.id, *next);
                *next += 1;
                st.bodies.insert(id, make_body(id, orig.component, c));
            }
        }
        Ok(())
    }

    fn copy_named(&self, body: &Body, op: &Op, tag: &str) -> Named {
        let base = body_named(body);
        let mut n = propagate(op, &[&base], &[], op.shape(), "?");
        for name in n.names.iter_mut() {
            *name = format!("{name}@{tag}");
        }
        n
    }

    fn pattern(&mut self, f: &Feature, p: &Pattern, st: &mut ModelState) -> Result<(), String> {
        let n = self.eval_expr("count", &p.count)?.round() as i64;
        if !(1..=1000).contains(&n) {
            return Err("count must be between 1 and 1000".into());
        }
        if p.bodies.is_empty() {
            return Err("no bodies selected".into());
        }
        let (o, dir) = self.resolve_axis(st, &p.axis)?;
        let mut steps: Vec<DMat4> = Vec::new();
        match p.kind {
            PatternKind::Linear => {
                let d = self.eval_expr("spacing", &p.spacing)?;
                let (n2, d2, dir2) = match &p.axis2 {
                    Some(a2) => {
                        let (_, d2v) = self.resolve_axis(st, a2)?;
                        (self.eval_expr("count 2", &p.count2)?.round().max(1.0) as i64, self.eval_expr("spacing 2", &p.spacing2)?, d2v)
                    }
                    None => (1, 0.0, DVec3::ZERO),
                };
                for j in 0..n2 {
                    for i in 0..n {
                        if i == 0 && j == 0 {
                            continue;
                        }
                        steps.push(DMat4::from_translation(dir * d * i as f64 + dir2 * d2 * j as f64));
                    }
                }
            }
            PatternKind::Circular => {
                let total = if p.angle.trim().is_empty() { 360.0 } else { self.eval_expr("angle", &p.angle)? };
                let full = (total.abs() - 360.0).abs() < 1e-9;
                let step = if full { total / n as f64 } else { total / (n - 1).max(1) as f64 };
                for i in 1..n {
                    let q = DQuat::from_axis_angle(dir, (step * i as f64).to_radians());
                    steps.push(DMat4::from_translation(o) * DMat4::from_quat(q) * DMat4::from_translation(-o));
                }
            }
        }
        let mut next = 0;
        for bid in &p.bodies {
            let body = st.bodies.get(bid).ok_or("pattern body no longer exists")?.clone();
            if body.is_mesh() {
                return Err("mesh bodies cannot be patterned".into());
            }
            let mut copies = Vec::new();
            for (i, m) in steps.iter().enumerate() {
                let op = k::transform(&body.shape, m).map_err(|e| e.0)?;
                copies.push(self.copy_named(&body, &op, &format!("{}.{}", f.id, i + 1)));
            }
            self.place_copies(f, &body, copies, p.join, &mut next, st)?;
        }
        Ok(())
    }

    fn mirror(&mut self, f: &Feature, m: &Mirror, st: &mut ModelState) -> Result<(), String> {
        if m.bodies.is_empty() {
            return Err("no bodies selected".into());
        }
        let (frame, _) = self.resolve_plane(st, &m.plane)?;
        let mut next = 0;
        for bid in &m.bodies {
            let body = st.bodies.get(bid).ok_or("mirror body no longer exists")?.clone();
            if body.is_mesh() {
                return Err("mesh bodies cannot be mirrored".into());
            }
            let op = k::mirror(&body.shape, frame.origin, frame.n).map_err(|e| e.0)?;
            let copy = self.copy_named(&body, &op, &format!("{}m", f.id));
            self.place_copies(f, &body, vec![copy], m.join, &mut next, st)?;
        }
        Ok(())
    }

    fn fillet_chamfer(
        &mut self,
        f: &Feature,
        edges: &[EdgeRef],
        size: &str,
        fillet: bool,
        st: &mut ModelState,
    ) -> Result<(), String> {
        let v = self.eval_expr(if fillet { "radius" } else { "distance" }, size)?;
        if edges.is_empty() {
            return Err("no edges selected".into());
        }
        let mut by_body: BTreeMap<BodyId, Vec<&EdgeRef>> = BTreeMap::new();
        for e in edges {
            by_body.entry(e.body).or_default().push(e);
        }
        let fallback = format!("f{}.{}", f.id, if fillet { "fillet" } else { "chamfer" });
        for (bid, refs) in by_body {
            let body = st.bodies.get(&bid).ok_or("edge's body no longer exists")?.clone();
            let mut shapes = Vec::new();
            let mut gen = Vec::new();
            for r in refs {
                let e = self.resolve_edge(&body, r)?;
                gen.push((e.clone(), format!("{fallback}({})", r.name)));
                shapes.push(e);
            }
            let op = if fillet { k::fillet(&body.shape, &shapes, v) } else { k::chamfer(&body.shape, &shapes, v) }
                .map_err(|e| e.0)?;
            let named = propagate(&op, &[&body_named(&body)], &gen, op.shape(), &fallback);
            st.bodies.insert(bid, make_body(bid, body.component, named));
        }
        Ok(())
    }

    fn shell(&mut self, f: &Feature, s: &Shell, st: &mut ModelState) -> Result<(), String> {
        let t = self.eval_expr("thickness", &s.thickness)?;
        if t <= 0.0 {
            return Err("thickness must be positive".into());
        }
        let bid = s.body.or_else(|| s.faces.first().map(|f| f.body)).ok_or("no body selected")?;
        let body = st.bodies.get(&bid).ok_or("body no longer exists")?.clone();
        let mut faces = Vec::new();
        for fr in &s.faces {
            faces.push(self.resolve_face(st, fr)?.1);
        }
        let op = k::shell(&body.shape, &faces, t).map_err(|e| e.0)?;
        let named = propagate(&op, &[&body_named(&body)], &[], op.shape(), &format!("f{}.inner", f.id));
        st.bodies.insert(bid, make_body(bid, body.component, named));
        Ok(())
    }

    fn combine(&mut self, f: &Feature, c: &Combine, st: &mut ModelState) -> Result<(), String> {
        let tid = c.target.ok_or("no target body")?;
        let target = st.bodies.get(&tid).ok_or("target body no longer exists")?.clone();
        let tools: Vec<Body> = c.tools.iter().filter(|b| **b != tid).filter_map(|b| st.bodies.get(b).cloned()).collect();
        if tools.is_empty() {
            return Err("no tool bodies".into());
        }
        let kind = match c.op {
            Operation::Cut => BoolKind::Cut,
            Operation::Intersect => BoolKind::Common,
            _ => BoolKind::Fuse,
        };
        let tn = body_named(&target);
        let tool_named: Vec<Named> = tools.iter().map(body_named).collect();
        let tool_shapes: Vec<&Shape> = tool_named.iter().map(|n| &n.shape).collect();
        let op = k::boolean(kind, &[&tn.shape], &tool_shapes, true).map_err(|e| e.0)?;
        let mut inputs = vec![&tn];
        inputs.extend(tool_named.iter());
        let named = propagate(&op, &inputs, &[], op.shape(), &format!("f{}.new", f.id));
        if named.shape.solids().is_empty() {
            st.bodies.remove(&tid);
        } else {
            st.bodies.insert(tid, make_body(tid, target.component, named));
        }
        if !c.keep_tools {
            for t in &tools {
                st.bodies.remove(&t.id);
            }
        }
        Ok(())
    }

    fn move_bodies(&mut self, f: &Feature, m: &MoveBodies, st: &mut ModelState) -> Result<(), String> {
        let d = DVec3::new(self.eval_expr("x", &m.dx)?, self.eval_expr("y", &m.dy)?, self.eval_expr("z", &m.dz)?);
        let rz = self.eval_expr("rotation", &m.rz)?;
        let mut k_idx = 0;
        for bid in &m.bodies {
            let body = st.bodies.get(bid).ok_or("body no longer exists")?.clone();
            if body.is_mesh() {
                self.warnings.push("mesh bodies cannot be moved yet".into());
                continue;
            }
            let c = body.shape.bbox().map(|(a, b)| (a + b) * 0.5).unwrap_or(DVec3::ZERO);
            let mat = DMat4::from_translation(d)
                * DMat4::from_translation(c)
                * DMat4::from_quat(DQuat::from_rotation_z(rz.to_radians()))
                * DMat4::from_translation(-c);
            let op = k::transform(&body.shape, &mat).map_err(|e| e.0)?;
            let named = propagate(&op, &[&body_named(&body)], &[], op.shape(), &format!("f{}.moved", f.id));
            if m.copy {
                let id = BodyId::new(f.id, k_idx);
                k_idx += 1;
                st.bodies.insert(id, make_body(id, body.component, named));
            } else {
                st.bodies.insert(*bid, make_body(*bid, body.component, named));
            }
        }
        Ok(())
    }
}

pub fn find_profile<'a>(ss: &'a SolvedSketch, curves: &[u32], hint: DVec2) -> Option<&'a Profile> {
    let mut want: Vec<u32> = curves.to_vec();
    want.sort_unstable();
    want.dedup();
    let exact: Vec<&Profile> = ss.profiles.iter().filter(|p| p.curves == want).collect();
    match exact.len() {
        1 => return Some(exact[0]),
        n if n > 1 => {
            return exact
                .iter()
                .copied()
                .find(|p| p.contains(hint))
                .or_else(|| exact.iter().copied().min_by(|a, b| {
                    (a.hint - hint).length().partial_cmp(&(b.hint - hint).length()).unwrap()
                }))
        }
        _ => {}
    }
    // fallback: whichever (smallest) region contains the hint point
    ss.profiles
        .iter()
        .filter(|p| p.contains(hint))
        .min_by(|a, b| a.area.partial_cmp(&b.area).unwrap())
}

/// Closed regions of a solved sketch.
pub fn compute_profiles(s: &Sketch, frame: &Frame, sid: FeatureId) -> Result<Vec<Profile>, String> {
    let mut edges = Vec::new();
    for c in s.curves.iter().filter(|c| !c.construction) {
        let e = match c.geom {
            CurveGeom::Line { a, b } => {
                let (pa, pb) = (s.pos(a), s.pos(b));
                if (pa - pb).length() < 1e-9 {
                    continue;
                }
                k::line(frame.to3(pa), frame.to3(pb))
            }
            CurveGeom::Circle { c, r } => {
                if r < 1e-9 {
                    continue;
                }
                k::circle(frame.to3(s.pos(c)), frame.n, r)
            }
            CurveGeom::Arc { c, a, b } => {
                if (s.pos(a) - s.pos(c)).length() < 1e-9 || (s.pos(a) - s.pos(b)).length() < 1e-9 {
                    continue;
                }
                k::arc(frame.to3(s.pos(c)), frame.n, frame.to3(s.pos(a)), frame.to3(s.pos(b)))
            }
        };
        match e {
            Ok(e) => edges.push(e),
            Err(_) => continue,
        }
    }
    if edges.is_empty() {
        return Ok(vec![]);
    }
    let regions = k::planar_regions(&edges).map_err(|e| e.0)?;
    let scale = s.bounds().map(|(a, b)| (b - a).length()).unwrap_or(1.0).max(1e-3);
    let mut out = Vec::new();
    for face in regions {
        let mesh = match face.mesh(scale * 0.002, 0.25) {
            Ok(m) => m,
            Err(_) => continue,
        };
        let p2: Vec<DVec2> = mesh
            .positions
            .chunks(3)
            .map(|c| frame.to2(DVec3::new(c[0] as f64, c[1] as f64, c[2] as f64)))
            .collect();
        let tris: Vec<[DVec2; 3]> = mesh
            .indices
            .chunks(3)
            .map(|t| [p2[t[0] as usize], p2[t[1] as usize], p2[t[2] as usize]])
            .collect();
        let hint = tris
            .iter()
            .max_by(|a, b| tri_area(a).partial_cmp(&tri_area(b)).unwrap())
            .map(|t| (t[0] + t[1] + t[2]) / 3.0)
            .unwrap_or(DVec2::ZERO);
        let mut curves = Vec::new();
        let mut edge_names = HashMap::new();
        for e in face.edges() {
            let Ok(info) = e.edge_info() else { continue };
            let m2 = frame.to2(info.mid);
            let best = s
                .curves
                .iter()
                .filter(|c| !c.construction)
                .map(|c| (c.id, s.curve_distance(c, m2)))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            if let Some((cid, d)) = best {
                if d < scale * 1e-5 + 1e-6 {
                    curves.push(cid);
                    edge_names.insert(e.hash(), format!("s{sid}.c{cid}"));
                }
            }
        }
        curves.sort_unstable();
        curves.dedup();
        let area = face.face_info().map(|i| i.area).unwrap_or(0.0);
        out.push(Profile { face, curves, hint, tris, area, edge_names });
    }
    Ok(out)
}

fn tri_area(t: &[DVec2; 3]) -> f64 {
    ((t[1] - t[0]).perp_dot(t[2] - t[0])).abs() * 0.5
}

/// All curve ids referenced by any feature's profile selections (for UI hints).
pub fn referenced_bodies(doc: &Document) -> HashSet<BodyId> {
    let mut s = HashSet::new();
    for f in &doc.features {
        if let FeatureKind::Extrude(e) = &f.kind {
            s.extend(e.targets.iter().copied());
        }
    }
    s
}
