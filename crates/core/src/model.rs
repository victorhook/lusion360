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
    /// tool solid of extrude/revolve features (for previews), with its operation
    pub tools: HashMap<FeatureId, (Shape, Operation)>,
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
    /// feature id per cached state (to tell edits from structural changes)
    ids: Vec<FeatureId>,
    param_fp: u64,
    pub params: ParamValues,
    empty: ModelState,
    /// total milliseconds spent in the last update
    pub last_update_ms: f64,
    /// last evaluation time per feature (ms), used to decide sync vs background
    cost: HashMap<FeatureId, f64>,
    /// background recompute in flight
    job: Option<Job>,
    /// a background job crashed: stay synchronous for the next update
    job_failed: bool,
    /// display meshes produced by background jobs (shape, mesh, is_tool) for the mesh cache
    pub premeshed: Vec<(Shape, crate::tess::BodyMesh, bool)>,
}

/// `Shape` is `Send` but not `Sync` (OCCT handles are reference counted with
/// atomics, but geometry caches are not locked). States handed to a worker get
/// deep-copied bodies; sketch profiles and tool shapes are only read there.
struct SendBox<T>(T);
unsafe impl<T> Send for SendBox<T> {}

struct JobOut {
    states: Vec<ModelState>,
    status: Vec<FeatureStatus>,
    fingerprints: Vec<u64>,
    ids: Vec<FeatureId>,
    meshes: Vec<(Shape, crate::tess::BodyMesh, bool)>,
}

struct Job {
    start: usize,
    /// fingerprints of features 0..start at launch (must still match to install)
    prefix: Vec<u64>,
    param_fp: u64,
    params: ParamValues,
    /// deep-copy hash → original shape, to restore untouched bodies on install
    originals: HashMap<u64, Shape>,
    rx: std::sync::mpsc::Receiver<SendBox<JobOut>>,
}

/// Evaluate one feature on top of `prev`.
fn eval_feature(doc: &Document, params: &ParamValues, f: &Feature, prev: &ModelState) -> (ModelState, FeatureStatus) {
    let mut st = prev.clone();
    let mut status = FeatureStatus::default();
    let ft = Instant::now();
    if !f.suppressed {
        let mut ctx = Ctx { doc, params, warnings: vec![] };
        // a kernel/logic panic in one feature must not take down the app
        let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| ctx.eval(f, &mut st)));
        match r {
            Ok(Ok(())) => {}
            Ok(Err(e)) => {
                status.error = Some(e);
                st = prev.clone();
            }
            Err(_) => {
                status.error = Some("internal error while computing this feature".into());
                st = prev.clone();
            }
        }
        status.warnings = ctx.warnings;
    }
    status.millis = ft.elapsed().as_secs_f64() * 1000.0;
    (st, status)
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

    fn param_hash(doc: &Document) -> u64 {
        let mut h = std::collections::hash_map::DefaultHasher::new();
        serde_json::to_string(&doc.parameters).unwrap_or_default().hash(&mut h);
        h.finish()
    }

    /// True while a background recompute runs (the shown states are stale).
    pub fn busy(&self) -> bool {
        self.job.is_some()
    }

    /// Install a finished background job. Returns true if states changed.
    fn poll_job(&mut self) -> bool {
        let Some(job) = &self.job else { return false };
        match job.rx.try_recv() {
            Err(std::sync::mpsc::TryRecvError::Empty) => false,
            Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                self.job = None;
                self.job_failed = true;
                false
            }
            Ok(SendBox(mut out)) => {
                let job = self.job.take().unwrap();
                // the prefix it was built on must still be cached
                if self.fingerprints.len() < job.start || self.fingerprints[..job.start] != job.prefix[..] {
                    return false;
                }
                // untouched bodies: back to the original shapes (keeps mesh/pick caches warm)
                for st in &mut out.states {
                    for b in st.bodies.values_mut() {
                        if b.shape.is_null() {
                            continue;
                        }
                        if let Some(orig) = job.originals.get(&b.shape.hash()) {
                            b.shape = orig.clone();
                        }
                    }
                }
                for (id, s) in out.ids.iter().zip(&out.status) {
                    self.cost.insert(*id, s.millis);
                }
                self.truncate(job.start);
                self.states.extend(out.states);
                self.status.extend(out.status);
                self.fingerprints.extend(out.fingerprints);
                self.ids.extend(out.ids);
                if job.param_fp != self.param_fp {
                    self.param_fp = job.param_fp;
                    self.params = job.params;
                }
                self.premeshed.extend(out.meshes);
                true
            }
        }
    }

    /// Like `update`, but slow recomputes run on a worker thread while the
    /// previous result stays visible (for live previews while dragging).
    /// Returns true if the visible states changed.
    pub fn update_interactive(&mut self, doc: &Document, budget_ms: f64) -> bool {
        let mut changed = self.poll_job();
        if self.job.is_some() {
            return changed;
        }
        if self.job_failed {
            self.job_failed = false;
            return self.update(doc) || changed;
        }
        let pfp = Self::param_hash(doc);
        let n = self.states.len().min(doc.features.len());
        let mut first_bad = if pfp != self.param_fp { 0 } else { n };
        for i in 0..first_bad {
            if fingerprint(&doc.features[i]) != self.fingerprints[i] {
                first_bad = i;
                break;
            }
        }
        let marker = doc.marker.min(doc.features.len());
        if first_bad >= marker && self.states.len() >= marker {
            return changed;
        }
        // background only for edits of existing features with stale states to show meanwhile
        let same_shape = self.states.len() >= marker && (first_bad..marker).all(|i| self.ids.get(i) == Some(&doc.features[i].id));
        let est: f64 = (first_bad..marker).map(|i| self.cost.get(&doc.features[i].id).copied().unwrap_or(0.0)).sum();
        if !same_shape || est < budget_ms {
            changed |= self.update(doc);
            return changed;
        }
        self.launch(doc, first_bad, marker, pfp);
        changed
    }

    fn launch(&mut self, doc: &Document, start: usize, end: usize, pfp: u64) {
        let params = if pfp != self.param_fp { ParamValues::evaluate(&doc.parameters) } else { self.params.clone() };
        let mut base = self.state_after(start).clone();
        let mut originals = HashMap::new();
        for b in base.bodies.values_mut() {
            if b.shape.is_null() {
                continue;
            }
            let copy = b.shape.deep_copy();
            originals.insert(copy.hash(), b.shape.clone());
            b.shape = copy;
        }
        let copies: HashSet<u64> = originals.keys().copied().collect();
        let doc = doc.clone();
        let (tx, rx) = std::sync::mpsc::channel();
        let input = SendBox((base, params.clone()));
        std::thread::Builder::new()
            .name("recompute".into())
            .spawn(move || {
                let input = input;
                let SendBox((base, params)) = input;
                let mut out = JobOut { states: vec![], status: vec![], fingerprints: vec![], ids: vec![], meshes: vec![] };
                let mut prev = base;
                for f in &doc.features[start..end] {
                    let (st, status) = eval_feature(&doc, &params, f, &prev);
                    out.states.push(st.clone());
                    out.status.push(status);
                    out.fingerprints.push(fingerprint(f));
                    out.ids.push(f.id);
                    prev = st;
                }
                // tessellate what changed here, not on the UI thread
                for b in prev.bodies.values() {
                    if b.shape.is_null() || b.mesh.is_some() || copies.contains(&b.shape.hash()) {
                        continue;
                    }
                    if let Ok(m) = crate::tess::BodyMesh::from_shape(&b.shape) {
                        out.meshes.push((b.shape.clone(), m, false));
                    }
                }
                for f in &doc.features[start..end] {
                    if let Some((t, _)) = prev.tools.get(&f.id) {
                        if let Ok(m) = crate::tess::BodyMesh::from_shape(t) {
                            out.meshes.push((t.clone(), m, true));
                        }
                    }
                }
                let _ = tx.send(SendBox(out));
            })
            .ok();
        self.job = Some(Job { start, prefix: self.fingerprints[..start].to_vec(), param_fp: pfp, params, originals, rx });
    }

    /// Bring cached states up to date with `doc` (up to the timeline marker).
    /// Returns true if anything was recomputed.
    pub fn update(&mut self, doc: &Document) -> bool {
        let t0 = Instant::now();
        // a finished background result may still be useful; a running one is dropped
        let mut changed = self.poll_job();
        self.job = None;
        let pfp = Self::param_hash(doc);
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
        while self.states.len() < marker {
            let i = self.states.len();
            let f = &doc.features[i];
            let (st, status) = {
                let prev = if i == 0 { &self.empty } else { &self.states[i - 1] };
                eval_feature(doc, &self.params, f, prev)
            };
            self.cost.insert(f.id, status.millis);
            self.states.push(st);
            self.status.push(status);
            self.fingerprints.push(fingerprint(f));
            self.ids.push(f.id);
            changed = true;
        }
        self.last_update_ms = t0.elapsed().as_secs_f64() * 1000.0;
        changed
    }

    fn truncate(&mut self, n: usize) {
        self.states.truncate(n);
        self.status.truncate(n);
        self.fingerprints.truncate(n);
        self.ids.truncate(n);
    }

    pub fn invalidate(&mut self) {
        self.job = None;
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
                let fr = self.plane_frame(st, p)?;
                st.planes.insert(f.id, fr);
                Ok(())
            }
            FeatureKind::Split(x) => self.split(f, x, st),
            FeatureKind::Hole(x) => self.hole(f, x, st),
            FeatureKind::Thread(x) => self.thread(f, x, st),
            FeatureKind::Sweep(x) => self.sweep(f, x, st),
            FeatureKind::Loft(x) => self.loft(f, x, st),
            FeatureKind::Joint(x) => self.joint(f, x, st),
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
            // twins (`name~k`) can swap order when geometry moves: prefer the one nearest the hint
            let base = fr.name.split('~').next().unwrap_or(&fr.name);
            let twins: Vec<usize> = body.face_names.iter().enumerate().filter(|(_, n)| n.split('~').next() == Some(base)).map(|(j, _)| j).collect();
            let hint = DVec3::from_array(fr.hint);
            if twins.len() > 1 && hint != DVec3::ZERO {
                let best = twins.iter().copied().min_by(|a, b| (face_center(&faces[*a]) - hint).length().total_cmp(&(face_center(&faces[*b]) - hint).length())).unwrap_or(i);
                return Ok((body, faces[best].clone()));
            }
            return Ok((body, faces[i].clone()));
        }
        // fallback: nearest face centre
        let hint = DVec3::from_array(fr.hint);
        let best = faces
            .iter()
            .enumerate()
            .map(|(i, f)| (i, (face_center(f) - hint).length()))
            .min_by(|a, b| a.1.total_cmp(&b.1));
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
            let base = er.name.split('~').next().unwrap_or(&er.name);
            let twins: Vec<usize> = body.edge_names.iter().enumerate().filter(|(_, n)| n.split('~').next() == Some(base)).map(|(j, _)| j).collect();
            let hint = DVec3::from_array(er.hint);
            if twins.len() > 1 && hint != DVec3::ZERO {
                let mid = |j: usize| edges[j].edge_info().map(|e| e.mid).unwrap_or(DVec3::splat(f64::INFINITY));
                let best = twins.iter().copied().min_by(|a, b| (mid(*a) - hint).length().total_cmp(&(mid(*b) - hint).length())).unwrap_or(i);
                return Ok(edges[best].clone());
            }
            return Ok(edges[i].clone());
        }
        let hint = DVec3::from_array(er.hint);
        let best = edges
            .iter()
            .enumerate()
            .filter_map(|(i, e)| e.edge_info().ok().map(|inf| (i, (inf.mid - hint).length())))
            .min_by(|a, b| a.1.total_cmp(&b.1));
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
        let profiles = match compute_profiles(&solved, &frame, f.id, self.params) {
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
            ExtentMode::ThroughAll => {
                // long enough to pass through every body
                let mut lo = DVec3::splat(f64::INFINITY);
                let mut hi = DVec3::splat(f64::NEG_INFINITY);
                for b in st.bodies.values() {
                    if let Some((a, c)) = b.shape.bbox() {
                        lo = lo.min(a);
                        hi = hi.max(c);
                    }
                }
                let reach = if lo.x <= hi.x { (hi - lo).length() * 2.0 + 10.0 } else { 1000.0 };
                (start, if d < 0.0 { -reach } else { reach })
            }
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
        st.tools.insert(f.id, (tool.shape.clone(), e.op));
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
        st.tools.insert(f.id, (tool.shape.clone(), r.op));
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
                        let n2 = self.eval_expr("count 2", &p.count2)?.round() as i64;
                        if !(1..=1000).contains(&n2) || n * n2 > 10_000 {
                            return Err("too many copies (max 10 000)".into());
                        }
                        (n2, self.eval_expr("spacing 2", &p.spacing2)?, d2v)
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
        let ang = self.eval_expr("rotation", &m.rz)?;
        let pivot = m.pivot.map(DVec3::from_array).unwrap_or_else(|| move_pivot(st, &m.bodies));
        let (ao, adir) = match &m.axis {
            AxisRef::X => (pivot, DVec3::X),
            AxisRef::Y => (pivot, DVec3::Y),
            AxisRef::Z => (pivot, DVec3::Z),
            other => self.resolve_axis(st, other)?,
        };
        let rot = DMat4::from_translation(ao) * DMat4::from_quat(DQuat::from_axis_angle(adir.normalize(), ang.to_radians())) * DMat4::from_translation(-ao);
        let mat = DMat4::from_translation(d) * rot;
        let mut k_idx = 0;
        for bid in &m.bodies {
            let body = st.bodies.get(bid).ok_or("body no longer exists")?.clone();
            if body.is_mesh() {
                self.warnings.push("mesh bodies cannot be moved yet".into());
                continue;
            }
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

    // ---- construction planes

    fn plane_frame(&mut self, st: &ModelState, p: &OffsetPlane) -> Result<Frame, String> {
        let d = self.eval_expr("offset", &p.offset)?;
        let fr = match &p.mode {
            PlaneMode::Offset => self.resolve_plane(st, &p.base)?.0,
            PlaneMode::Angle { axis, angle } => {
                let base = self.resolve_plane(st, &p.base)?.0;
                let (o, a) = self.resolve_axis(st, axis)?;
                let a = a.normalize();
                let mut n0 = base.n - a * base.n.dot(a);
                if n0.length() < 1e-9 {
                    n0 = a.any_orthonormal_vector();
                }
                let ang = self.eval_expr("angle", angle)?.to_radians();
                let n = DQuat::from_axis_angle(a, ang) * n0.normalize();
                Frame::new(o, n, a)
            }
            PlaneMode::Midplane { other } => {
                let a = self.resolve_plane(st, &p.base)?.0;
                let b = self.resolve_plane(st, other)?.0;
                let bn = if a.n.dot(b.n) < 0.0 { -b.n } else { b.n };
                if a.n.cross(bn).length() < 1e-9 {
                    let mid = a.origin + a.n * (a.n.dot(b.origin - a.origin) * 0.5);
                    Frame::new(mid, a.n, a.x)
                } else {
                    // bisector through the intersection line
                    let u = a.n.cross(bn);
                    let (d1, d2) = (a.n.dot(a.origin), bn.dot(b.origin));
                    let pt = (a.n * 0.0 + bn.cross(u) * d1 + u.cross(a.n) * d2) / u.length_squared();
                    let n = (a.n - bn).normalize();
                    Frame::new(pt, n, u)
                }
            }
            PlaneMode::NormalToEdge { edge, at } => {
                let body = st.bodies.get(&edge.body).ok_or("edge body is missing")?.clone();
                let e = self.resolve_edge(&body, edge)?;
                let t = self.eval_expr("position", at)? / 100.0;
                let (pt, tan) = e.edge_at(t).ok_or("cannot evaluate the edge")?;
                Frame::new(pt, tan, tan.any_orthonormal_vector())
            }
        };
        Ok(fr.offset(d))
    }

    // ---- split

    fn split(&mut self, f: &Feature, sp: &Split, st: &mut ModelState) -> Result<(), String> {
        if sp.bodies.is_empty() {
            return Err("no bodies selected".into());
        }
        let (fr, _) = self.resolve_plane(st, &sp.tool)?;
        let mut next = 0u32;
        let mut cut_any = false;
        for bid in &sp.bodies {
            let body = st.bodies.get(bid).ok_or("body no longer exists")?.clone();
            if body.is_mesh() {
                self.warnings.push("mesh bodies cannot be split".into());
                continue;
            }
            let Some((lo, hi)) = body.shape.bbox() else { continue };
            let c = (lo + hi) * 0.5;
            let centre = c - fr.n * fr.n.dot(c - fr.origin);
            let tool = k::plane_face(centre, fr.n, (hi - lo).length() + 10.0).map_err(|e| e.0)?;
            let op = k::split(&body.shape, &[&tool]).map_err(|e| e.0)?;
            let bn = body_named(&body);
            let tn = Named { shape: tool.clone(), names: vec![format!("f{}.cut", f.id)] };
            let named = propagate(&op, &[&bn, &tn], &[], op.shape(), &format!("f{}.cut", f.id));
            let parts = named.shape.solids();
            if parts.len() < 2 {
                continue;
            }
            cut_any = true;
            let side = |p: &Shape| p.props().map(|(_, _, c)| fr.n.dot(c - fr.origin)).unwrap_or(0.0);
            let mut kept: Vec<Shape> = parts
                .into_iter()
                .filter(|p| match sp.keep {
                    SplitKeep::Both => true,
                    SplitKeep::Front => side(p) > 0.0,
                    SplitKeep::Back => side(p) <= 0.0,
                })
                .collect();
            // the original id goes to the back-most piece (stable for downstream references)
            kept.sort_by(|a, b| side(a).total_cmp(&side(b)));
            st.bodies.remove(bid);
            for (i, part) in kept.into_iter().enumerate() {
                let id = if i == 0 {
                    *bid
                } else {
                    let id = BodyId::new(f.id, next);
                    next += 1;
                    id
                };
                st.bodies.insert(id, make_body(id, body.component, sub_named(&named, part)));
            }
        }
        if !cut_any {
            return Err("the plane does not cross the selected bodies".into());
        }
        Ok(())
    }

    // ---- holes

    fn hole(&mut self, f: &Feature, h: &Hole, st: &mut ModelState) -> Result<(), String> {
        let (body, face) = self.resolve_face(st, &h.face)?;
        let body = body.clone();
        let info = face.face_info().map_err(|e| e.0)?;
        if info.kind != SurfaceKind::Plane {
            return Err("holes start on a planar face".into());
        }
        let n = info.normal.normalize();
        let frame = face_frame(info.origin, n);
        let mut centres: Vec<DVec3> = h.points.iter().map(|p| frame.to3(DVec2::from_array(*p))).collect();
        if let Some(sid) = h.sketch {
            let ss = st.sketches.get(&sid).ok_or("hole sketch is missing")?;
            let used: HashSet<u32> = ss.sketch.curves.iter().flat_map(|c| c.points()).collect();
            for p in &ss.sketch.points {
                if used.contains(&p.id) || p.fixed {
                    continue;
                }
                let p3 = ss.frame.to3(p.pos());
                centres.push(p3 - n * n.dot(p3 - info.origin));
            }
        }
        if centres.is_empty() {
            return Err("no hole positions (click on the face or pick a sketch with points)".into());
        }
        let r = self.eval_expr("diameter", &h.diameter)? / 2.0;
        if r <= 0.0 {
            return Err("diameter must be positive".into());
        }
        let depth = if h.through {
            body.shape.bbox().map(|(a, b)| (b - a).length() * 2.0 + 10.0).unwrap_or(1000.0)
        } else {
            self.eval_expr("depth", &h.depth)?
        };
        if depth <= 0.0 {
            return Err("depth must be positive".into());
        }
        let e = (r * 0.05).max(0.05);
        // half profile (radius, depth below the face), labelled per segment
        let mut prof: Vec<(f64, f64, &str)> = vec![(0.0, -e, "top")];
        match h.kind {
            HoleKind::Simple => {
                prof.push((r, -e, "wall"));
            }
            HoleKind::Counterbore => {
                let cr = self.eval_expr("counterbore diameter", &h.cb_diameter)? / 2.0;
                let cd = self.eval_expr("counterbore depth", &h.cb_depth)?;
                if cr <= r || cd <= 0.0 || cd >= depth {
                    return Err("counterbore must be wider than the hole and shallower than its depth".into());
                }
                prof.push((cr, -e, "cbwall"));
                prof.push((cr, cd, "cbfloor"));
                prof.push((r, cd, "wall"));
            }
            HoleKind::Countersink => {
                let cr = self.eval_expr("countersink diameter", &h.cs_diameter)? / 2.0;
                let ang = self.eval_expr("countersink angle", &h.cs_angle)?;
                if cr <= r || !(1.0..179.0).contains(&ang) {
                    return Err("countersink must be wider than the hole (angle 1..179°)".into());
                }
                let cd = (cr - r) / (ang.to_radians() / 2.0).tan();
                prof.push((cr, -e, "csrim"));
                prof.push((cr, 0.0, "cs"));
                prof.push((r, cd, "wall"));
            }
        }
        prof.push((r, depth, "floor"));
        if h.drill_point && !h.through {
            prof.push((0.0, depth + r / 59f64.to_radians().tan(), "axis"));
        } else {
            prof.push((0.0, depth, "axis"));
        }
        let w = -n;
        let u = frame.x;
        let mut tools: Vec<Named> = Vec::new();
        for (hi, c) in centres.iter().enumerate() {
            let pts: Vec<DVec3> = prof.iter().map(|(rr, z, _)| *c + u * *rr + w * *z).collect();
            let mut edges = Vec::new();
            let mut names = HashMap::new();
            for i in 0..pts.len() {
                let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
                if (a - b).length() < 1e-9 {
                    continue;
                }
                let ed = k::line(a, b).map_err(|e| e.0)?;
                names.insert(ed.hash(), format!("h{hi}.{}", prof[i].2));
                edges.push(ed);
            }
            let region = k::planar_regions(&edges).map_err(|e| e.0)?;
            let face = region.into_iter().next().ok_or("hole profile is degenerate")?;
            // planar_regions rebuilds edges: map names by midpoint
            let mut fnames = HashMap::new();
            for fe in face.edges() {
                let Ok(fi) = fe.edge_info() else { continue };
                for (i, ed) in edges.iter().enumerate() {
                    let _ = i;
                    if let Ok(ei) = ed.edge_info() {
                        if (ei.mid - fi.mid).length() < 1e-7 {
                            if let Some(nm) = names.get(&ed.hash()) {
                                fnames.insert(fe.hash(), nm.clone());
                            }
                        }
                    }
                }
            }
            let op = k::revolve(&face, *c, w, std::f64::consts::TAU).map_err(|e| e.0)?;
            tools.push(self.name_sweep(f.id, &op, &face, &fnames));
        }
        let tool = if tools.len() == 1 {
            tools.pop().unwrap()
        } else {
            let shapes: Vec<&Shape> = tools.iter().map(|t| &t.shape).collect();
            let shape = Shape::compound(shapes);
            let names = tools.iter().flat_map(|t| t.names.iter().cloned()).collect();
            Named { shape, names }
        };
        st.tools.insert(f.id, (tool.shape.clone(), Operation::Cut));
        self.apply_tool(f, Operation::Cut, &[body.id], Some(body.id), tool, st)
    }

    // ---- threads

    fn thread(&mut self, f: &Feature, t: &Thread, st: &mut ModelState) -> Result<(), String> {
        let (body, face) = self.resolve_face(st, &t.face)?;
        let body = body.clone();
        let info = face.face_info().map_err(|e| e.0)?;
        if info.kind != SurfaceKind::Cylinder {
            return Err("threads go on a cylindrical face".into());
        }
        let axis = info.normal.normalize();
        let o = info.origin;
        let (pt, nrm) = face.point_normal().ok_or("cannot evaluate the face")?;
        let radial = (pt - o) - axis * axis.dot(pt - o);
        let external = radial.dot(nrm) > 0.0;
        // axial extent of the face
        let (mut smin, mut smax) = (f64::INFINITY, f64::NEG_INFINITY);
        for e in face.edges() {
            for p in e.edge_polyline(0.01, 0.2) {
                let s = axis.dot(p - o);
                smin = smin.min(s);
                smax = smax.max(s);
            }
        }
        if !(smax > smin) {
            return Err("cannot measure the cylinder".into());
        }
        let d = self.eval_expr("size", &t.size)?;
        let p = self.eval_expr("pitch", &t.pitch)?;
        let c = self.eval_expr("clearance", &t.clearance)?;
        if d <= 0.0 || p <= 0.0 || p > d {
            return Err("size and pitch must be positive (pitch < size)".into());
        }
        let full = smax - smin;
        let len = if t.length.trim().is_empty() { full } else { self.eval_expr("length", &t.length)?.clamp(p, full) };
        // run out past the starting end so the groove is open there
        let (a, b) = if !t.flip { (smin - 1.5 * p, smin + len) } else { (smax - len, smax + 1.5 * p) };
        let tan30 = 30f64.to_radians().tan();
        // trapezoid (r_in, half width at r_in, r_out, half width at r_out); ISO 68 basic profile
        let (r_in, w_in, r_out, w_out) = if external {
            let rmaj = d / 2.0 - c;
            let h = 0.6134 * p;
            // small run-out past the surface; the groove must stay narrower than the pitch
            let ext = 0.08 * p;
            let top = 0.875 * p / 2.0 + ext * tan30;
            let bot = (0.875 * p - 2.0 * h * tan30) / 2.0;
            if info.radius < rmaj - h {
                self.warnings.push("the cylinder is thinner than the thread's root diameter".into());
            }
            (rmaj - h, bot, rmaj + ext, top)
        } else {
            let rmin = (d - 1.0825 * p) / 2.0 + c;
            let h = 0.5413 * p + c;
            let ext = 0.15 * p;
            let top = 0.75 * p / 2.0 + ext * tan30;
            let bot = ((0.75 * p - 2.0 * h * tan30) / 2.0).max(0.02 * p);
            if info.radius > rmin + 0.25 * h {
                self.warnings.push(format!("the hole (Ø{:.2}) is larger than the thread's minor diameter (Ø{:.2}); threads will be shallow", info.radius * 2.0, rmin * 2.0));
            }
            (rmin - ext, top, rmin + h, bot)
        };
        let xdir = if radial.length() > 1e-9 { radial.normalize() } else { axis.any_orthonormal_vector() };
        let shape = k::thread(o + axis * a, axis, xdir, r_in, w_in, r_out, w_out, p, b - a, t.left).map_err(|e| e.0)?;
        let names = vec![format!("f{}.thread", f.id); shape.faces().len()];
        let tool = Named { shape, names };
        st.tools.insert(f.id, (tool.shape.clone(), Operation::Cut));
        self.apply_tool(f, Operation::Cut, &[body.id], Some(body.id), tool, st)
    }

    // ---- sweep / loft

    fn path_edges(&mut self, st: &ModelState, path: &[PathRef]) -> Result<Vec<Shape>, String> {
        let mut out = Vec::new();
        for pr in path {
            match pr {
                PathRef::SketchCurve { sketch, curve } => {
                    let ss = st.sketches.get(sketch).ok_or("path sketch is missing")?;
                    let e = curve_edge(&ss.sketch, *curve, &ss.frame).ok_or("path curve no longer exists")?.map_err(|e| e.0)?;
                    out.push(e);
                }
                PathRef::Edge(er) => {
                    let body = st.bodies.get(&er.body).ok_or("path body is missing")?.clone();
                    out.push(self.resolve_edge(&body, er)?);
                }
            }
        }
        if out.is_empty() {
            return Err("no path selected".into());
        }
        Ok(out)
    }

    fn sweep(&mut self, f: &Feature, sw: &Sweep, st: &mut ModelState) -> Result<(), String> {
        let prof = self.resolve_profiles(st, &sw.profiles)?;
        let path = self.path_edges(st, &sw.path)?;
        let mut parts = Vec::new();
        for face in &prof.faces {
            let op = k::sweep(face, &path, sw.frenet).map_err(|e| e.0)?;
            parts.push(self.name_sweep(f.id, &op, face, &prof.edge_names));
        }
        let tool = if parts.len() == 1 {
            parts.pop().unwrap()
        } else {
            let shape = Shape::compound(parts.iter().map(|p| &p.shape));
            Named { shape, names: parts.iter().flat_map(|p| p.names.iter().cloned()).collect() }
        };
        st.tools.insert(f.id, (tool.shape.clone(), sw.op));
        self.apply_tool(f, sw.op, &sw.targets, prof.default_target, tool, st)
    }

    fn loft(&mut self, f: &Feature, l: &Loft, st: &mut ModelState) -> Result<(), String> {
        if l.profiles.len() < 2 {
            return Err("pick at least two profiles".into());
        }
        let mut faces = Vec::new();
        let mut names = HashMap::new();
        let mut target = None;
        for pr in &l.profiles {
            let p = self.resolve_profiles(st, std::slice::from_ref(pr))?;
            faces.push(p.faces[0].clone());
            names.extend(p.edge_names);
            target = target.or(p.default_target);
        }
        let op = k::loft(&faces, l.ruled).map_err(|e| e.0)?;
        let all = Shape::compound(faces.iter());
        let tool = self.name_sweep(f.id, &op, &all, &names);
        st.tools.insert(f.id, (tool.shape.clone(), l.op));
        self.apply_tool(f, l.op, &l.targets, target, tool, st)
    }

    // ---- joints

    fn joint_frame(&mut self, st: &ModelState, r: &JointRef) -> Result<(Frame, BodyId), String> {
        match r {
            JointRef::Face(fr) => {
                let (body, face) = self.resolve_face(st, fr)?;
                let bid = body.id;
                let i = face.face_info().map_err(|e| e.0)?;
                Ok((
                    match i.kind {
                        SurfaceKind::Plane => Frame::new(i.center, i.normal, i.xdir),
                        _ => {
                            let a = i.normal.normalize();
                            Frame::new(i.origin + a * a.dot(i.center - i.origin), a, i.xdir)
                        }
                    },
                    bid,
                ))
            }
            JointRef::Edge(er) => {
                let body = st.bodies.get(&er.body).ok_or("joint body is missing")?.clone();
                let e = self.resolve_edge(&body, er)?;
                let i = e.edge_info().map_err(|e| e.0)?;
                Ok((
                    match i.kind {
                        CurveKind::Circle => Frame::new(i.center, i.axis, i.start - i.center),
                        _ => {
                            let d = (i.end - i.start).normalize_or_zero();
                            Frame::new(i.mid, d, d.any_orthonormal_vector())
                        }
                    },
                    body.id,
                ))
            }
        }
    }

    fn joint(&mut self, f: &Feature, j: &Joint, st: &mut ModelState) -> Result<(), String> {
        let (fa, ba) = self.joint_frame(st, &j.a)?;
        let (fb, bb) = self.joint_frame(st, &j.b)?;
        let comp_of = |doc: &Document, b: &Body| doc.body_props(b.id).component.unwrap_or(b.component);
        let moving: Vec<BodyId> = st.bodies.values().filter(|b| self.doc.is_ancestor(j.moving, comp_of(self.doc, b))).map(|b| b.id).collect();
        if moving.is_empty() {
            return Err("the moving component has no bodies".into());
        }
        if !moving.contains(&ba) {
            return Err("the first pick must be on the moving component".into());
        }
        if moving.contains(&bb) {
            return Err("the second pick must be on another component".into());
        }
        let ang = self.eval_expr("angle", &j.angle)?.to_radians();
        let off = self.eval_expr("offset", &j.offset)?;
        let nt = if j.flip { fb.n } else { -fb.n };
        let xt = DQuat::from_axis_angle(fb.n, ang) * fb.x;
        let target = Frame::new(fb.origin + fb.n * off, nt, xt);
        let ma = glam::DMat3::from_cols(fa.x, fa.y, fa.n);
        let mt = glam::DMat3::from_cols(target.x, target.y, target.n);
        let rot = mt * ma.transpose();
        let mat = DMat4::from_translation(target.origin) * DMat4::from_mat3(rot) * DMat4::from_translation(-fa.origin);
        for bid in moving {
            let body = st.bodies[&bid].clone();
            if body.is_mesh() {
                continue;
            }
            let op = k::transform(&body.shape, &mat).map_err(|e| e.0)?;
            let named = propagate(&op, &[&body_named(&body)], &[], op.shape(), &format!("f{}.moved", f.id));
            st.bodies.insert(bid, make_body(bid, body.component, named));
        }
        Ok(())
    }
}

/// Profiles that make up the filled glyphs of text `tid` (excludes counters
/// like the inside of an "o"), by even-odd depth among the glyph contours.
pub fn text_fill_profiles(ss: &SolvedSketch, tid: u32, params: &ParamValues) -> Vec<usize> {
    let Some(t) = ss.sketch.texts.iter().find(|t| t.id == tid) else { return vec![] };
    let contours = ss.sketch.text_outlines(t, params);
    let inside = |p: DVec2, poly: &[DVec2]| {
        let mut c = false;
        let n = poly.len();
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (poly[i], poly[j]);
            if (a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x {
                c = !c;
            }
            j = i;
        }
        c
    };
    ss.profiles
        .iter()
        .enumerate()
        .filter(|(_, p)| p.curves.contains(&tid))
        .filter(|(_, p)| contours.iter().filter(|c| inside(p.hint, c)).count() % 2 == 1)
        .map(|(i, _)| i)
        .collect()
}

/// Default rotation centre of a Move: centre of the bodies' combined bounding box.
pub fn move_pivot(st: &ModelState, bodies: &[BodyId]) -> DVec3 {
    let mut lo = DVec3::splat(f64::INFINITY);
    let mut hi = DVec3::splat(f64::NEG_INFINITY);
    for b in bodies.iter().filter_map(|b| st.bodies.get(b)) {
        if let Some((a, c)) = b.shape.bbox() {
            lo = lo.min(a);
            hi = hi.max(c);
        }
    }
    if lo.x <= hi.x {
        (lo + hi) * 0.5
    } else {
        DVec3::ZERO
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
                    (a.hint - hint).length().total_cmp(&(b.hint - hint).length())
                }))
        }
        _ => {}
    }
    // fallback: whichever (smallest) region contains the hint point
    ss.profiles
        .iter()
        .filter(|p| p.contains(hint))
        .min_by(|a, b| a.area.total_cmp(&b.area))
}

/// 3D edge of a sketch curve (None if the curve is missing or degenerate).
pub fn curve_edge(s: &Sketch, cid: u32, frame: &Frame) -> Option<k::KResult<Shape>> {
    let c = s.curve(cid)?;
    Some(match &c.geom {
        CurveGeom::Line { a, b } => {
            let (pa, pb) = (s.pos(*a), s.pos(*b));
            if (pa - pb).length() < 1e-9 {
                return None;
            }
            k::line(frame.to3(pa), frame.to3(pb))
        }
        CurveGeom::Circle { c, r } => {
            if *r < 1e-9 {
                return None;
            }
            k::circle(frame.to3(s.pos(*c)), frame.n, *r)
        }
        CurveGeom::Arc { c, a, b } => {
            if (s.pos(*a) - s.pos(*c)).length() < 1e-9 || (s.pos(*a) - s.pos(*b)).length() < 1e-9 {
                return None;
            }
            k::arc(frame.to3(s.pos(*c)), frame.n, frame.to3(s.pos(*a)), frame.to3(s.pos(*b)))
        }
    })
}

/// Closed regions of a solved sketch.
pub fn compute_profiles(s: &Sketch, frame: &Frame, sid: FeatureId, params: &ParamValues) -> Result<Vec<Profile>, String> {
    let mut edges = Vec::new();
    for c in s.curves.iter().filter(|c| !c.construction) {
        if let Some(Ok(e)) = curve_edge(s, c.id, frame) {
            edges.push(e);
        }
    }
    // text outlines become closed polygons of line edges
    let mut text_segs: Vec<(u32, DVec2, DVec2)> = Vec::new();
    for t in &s.texts {
        for c in s.text_outlines(t, params) {
            for i in 0..c.len() {
                let (a, b) = (c[i], c[(i + 1) % c.len()]);
                if (a - b).length() < 1e-9 {
                    continue;
                }
                if let Ok(e) = k::line(frame.to3(a), frame.to3(b)) {
                    edges.push(e);
                    text_segs.push((t.id, a, b));
                }
            }
        }
    }
    if edges.is_empty() {
        return Ok(vec![]);
    }
    let regions = k::planar_regions(&edges).map_err(|e| e.0)?;
    let scale = s.bounds().map(|(a, b)| (b - a).length()).unwrap_or(1.0).max(1e-3);
    let mut out = Vec::new();
    for face in regions {
        let tris = triangulate_face(&face, frame, scale);
        let hint = tris
            .iter()
            .max_by(|a, b| tri_area(a).total_cmp(&tri_area(b)))
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
                .min_by(|a, b| a.1.total_cmp(&b.1));
            let tol = scale * 1e-5 + 1e-6;
            match best {
                Some((cid, d)) if d < tol => {
                    curves.push(cid);
                    edge_names.insert(e.hash(), format!("s{sid}.c{cid}"));
                }
                _ => {
                    if let Some((tid, _, _)) = text_segs.iter().find(|(_, a, b)| crate::sketch::dist_point_segment(m2, *a, *b) < tol) {
                        curves.push(*tid);
                        edge_names.insert(e.hash(), format!("s{sid}.t{tid}"));
                    }
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

/// Triangulate a planar region in sketch coordinates (earcut on its boundary
/// loops; falls back to OCCT meshing).
fn triangulate_face(face: &Shape, frame: &Frame, scale: f64) -> Vec<[DVec2; 3]> {
    let loops = face.face_loops(scale * 0.002, 0.2);
    if !loops.is_empty() && loops.iter().all(|l| l.len() >= 3) {
        let mut data: Vec<f64> = Vec::new();
        let mut holes: Vec<usize> = Vec::new();
        let mut pts: Vec<DVec2> = Vec::new();
        for (i, l) in loops.iter().enumerate() {
            if i > 0 {
                holes.push(pts.len());
            }
            for p in l {
                let q = frame.to2(*p);
                pts.push(q);
                data.push(q.x);
                data.push(q.y);
            }
        }
        if let Ok(idx) = earcutr::earcut(&data, &holes, 2) {
            if !idx.is_empty() {
                return idx.chunks(3).map(|t| [pts[t[0]], pts[t[1]], pts[t[2]]]).collect();
            }
        }
    }
    match face.mesh(scale * 0.004, 0.3) {
        Ok(mesh) => {
            let p2: Vec<DVec2> = mesh.positions.chunks(3).map(|c| frame.to2(DVec3::new(c[0] as f64, c[1] as f64, c[2] as f64))).collect();
            mesh.indices.chunks(3).map(|t| [p2[t[0] as usize], p2[t[1] as usize], p2[t[2] as usize]]).collect()
        }
        Err(_) => vec![],
    }
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
