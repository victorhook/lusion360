//! Application state and the top-level frame loop.

use crate::camera::CameraCtl;
use crate::commands::Command;
use crate::pick::PickCache;
use crate::sketch_mode::SketchSession;
use crate::theme;
use glam::Vec3;
use lusion_core::doc::*;
use lusion_core::io;
use lusion_core::model::{Model, ModelState};
use lusion_core::tess::MeshCache;
use std::path::PathBuf;
use std::time::Instant;

pub enum Mode {
    Model,
    Sketch(Box<SketchSession>),
}

/// Something under the cursor / in the selection (model mode).
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Pick {
    Face { body: BodyId, face: u32 },
    Edge { body: BodyId, edge: u32 },
    Body(BodyId),
    Profile { sketch: FeatureId, index: usize },
    Origin(OriginPlane),
    Plane(FeatureId),
    Axis(u8),
    SketchLine { sketch: FeatureId, curve: u32 },
}

#[derive(Clone, Copy, Debug)]
pub struct Hit {
    pub pick: Pick,
    pub point: Vec3,
    pub depth: f32,
}

/// What can be picked right now.
#[derive(Clone, Copy, Default, Debug)]
pub struct Filter {
    pub faces: bool,
    pub planar_only: bool,
    pub edges: bool,
    pub bodies: bool,
    pub profiles: bool,
    pub planes: bool,
    pub axes: bool,
    pub sketch_lines: bool,
    /// any sketch curve (lines, arcs, circles, splines), reported as `Pick::SketchLine`
    pub sketch_curves: bool,
}

impl Filter {
    pub fn default_model() -> Self {
        Filter { faces: true, edges: true, profiles: true, planes: true, ..Default::default() }
    }
}

pub struct Toast {
    pub text: String,
    pub color: egui::Color32,
    pub until: f64,
}

/// Section view: everything on the positive side of the plane is cut away.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SectionView {
    pub normal: glam::Vec3,
    /// a point the plane passes through at offset 0
    pub origin: glam::Vec3,
    pub offset: f32,
}

impl SectionView {
    /// Plane as (n.xyz, d) for the shaders.
    pub fn plane(&self) -> [f32; 4] {
        let n = self.normal.normalize_or_zero();
        let p = self.origin + n * self.offset;
        [n.x, n.y, n.z, n.dot(p)]
    }
    pub fn cuts(&self, p: glam::Vec3) -> bool {
        let pl = self.plane();
        glam::Vec3::new(pl[0], pl[1], pl[2]).dot(p) > pl[3] + 1e-4
    }
}

pub struct App {
    pub doc: Document,
    /// see-through bodies
    pub xray: bool,
    pub section: Option<SectionView>,
    pub model: Model,
    pub meshes: MeshCache,
    pub picks: PickCache,
    pub path: Option<PathBuf>,
    pub saved_doc: Document,
    pub undo: Vec<Document>,
    pub redo: Vec<Document>,
    pub cam: CameraCtl,
    pub mode: Mode,
    pub cmd: Option<Command>,
    pub selection: Vec<Pick>,
    pub hover: Option<Hit>,
    pub active_comp: CompId,
    pub toasts: Vec<Toast>,
    pub show_params: bool,
    pub palette: Option<(String, usize)>,
    pub project_dir: Option<PathBuf>,
    pub recent: Vec<PathBuf>,
    pub show_origin: bool,
    pub light_bg: bool,
    pub show_edges: bool,
    pub timeline_sel: Option<FeatureId>,
    pub rename: Option<(RenameTarget, String)>,
    pub confirm_discard: Option<PendingAction>,
    pub last_frame: Instant,
    pub fit_pending: bool,
    pub status_hint: String,
    pub frame_ms: f32,
    pub left_tab: LeftTab,
    pub allow_close: bool,
    pub about: bool,
    pub print: Option<crate::print::PrintDialog>,
    pub slicers: Option<Vec<crate::print::Slicer>>,
    pub slicer_pref: Option<String>,
    pub timeline_hover: Option<FeatureId>,
    pub hover_anim: Option<(Pick, Instant)>,
    pub last_autosave: Instant,
    pub autosaved_doc: Option<Document>,
    pub recovery: Option<PathBuf>,
    pub drag_out: Option<crate::dragdrop::DragSession>,
    pub drag_format: crate::dragdrop::DragFormat,
    #[cfg(target_os = "linux")]
    pub xdnd: Option<Option<crate::dragout::DragOut>>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LeftTab {
    Browser,
    Project,
}

#[derive(Clone, Debug, PartialEq)]
pub enum RenameTarget {
    Feature(FeatureId),
    Component(CompId),
    Body(BodyId),
}

#[derive(Clone, Debug)]
pub enum PendingAction {
    New,
    Open(Option<PathBuf>),
    Quit,
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, initial: Option<PathBuf>) -> Self {
        theme::apply(&cc.egui_ctx);
        cc.egui_ctx.set_fonts(theme::fonts());
        if let Some(rs) = cc.wgpu_render_state.as_ref() {
            let r = crate::render::Renderer::new(rs);
            rs.renderer.write().callback_resources.insert(r);
        }
        let recent: Vec<PathBuf> = cc
            .storage
            .and_then(|s| s.get_string("recent"))
            .map(|s| s.lines().map(PathBuf::from).filter(|p| p.exists()).collect())
            .unwrap_or_default();
        let light_bg = cc.storage.and_then(|s| s.get_string("light_bg")).map(|s| s == "1").unwrap_or(false);
        let mut app = App {
            doc: Document::default(),
            xray: false,
            section: None,
            model: Model::new(),
            meshes: MeshCache::default(),
            picks: PickCache::default(),
            path: None,
            saved_doc: Document::default(),
            undo: vec![],
            redo: vec![],
            cam: CameraCtl::new(),
            mode: Mode::Model,
            cmd: None,
            selection: vec![],
            hover: None,
            active_comp: ROOT,
            toasts: vec![],
            show_params: false,
            palette: None,
            project_dir: None,
            recent,
            show_origin: false,
            light_bg,
            show_edges: true,
            timeline_sel: None,
            rename: None,
            confirm_discard: None,
            last_frame: Instant::now(),
            fit_pending: false,
            status_hint: String::new(),
            frame_ms: 0.0,
            left_tab: LeftTab::Browser,
            allow_close: false,
            about: false,
            print: None,
            slicers: None,
            timeline_hover: None,
            hover_anim: None,
            last_autosave: Instant::now(),
            autosaved_doc: None,
            recovery: find_orphan_autosave(),
            drag_out: None,
            drag_format: cc.storage.and_then(|s| s.get_string("drag_format")).and_then(|s| crate::dragdrop::DragFormat::from_key(&s)).unwrap_or(crate::dragdrop::DragFormat::Stl),
            #[cfg(target_os = "linux")]
            xdnd: None,
            slicer_pref: cc.storage.and_then(|s| s.get_string("slicer")).filter(|s| !s.is_empty()),
        };
        if let Some(p) = initial {
            if p.is_dir() {
                app.project_dir = Some(p);
                app.left_tab = LeftTab::Project;
            } else {
                app.open_path(p);
            }
        }
        app
    }

    // ------------------------------------------------------------------ doc helpers

    pub fn dirty(&self) -> bool {
        self.doc != self.saved_doc
    }

    /// Record `before` as an undo step (if the document changed).
    pub fn commit(&mut self, before: Document) {
        crash_snapshot(&self.doc, self.dirty());
        if before != self.doc {
            self.undo.push(before);
            if self.undo.len() > 200 {
                self.undo.remove(0);
            }
            self.redo.clear();
        }
    }

    pub fn undo(&mut self) {
        if self.cmd.is_some() {
            self.cancel_command();
            return;
        }
        if let Some(prev) = self.undo.pop() {
            let cur = std::mem::replace(&mut self.doc, prev);
            self.redo.push(cur);
            self.after_doc_swap();
        }
    }

    pub fn redo(&mut self) {
        if self.cmd.is_some() {
            self.cancel_command();
            return;
        }
        if let Some(next) = self.redo.pop() {
            let cur = std::mem::replace(&mut self.doc, next);
            self.undo.push(cur);
            self.after_doc_swap();
        }
    }

    fn after_doc_swap(&mut self) {
        self.selection.clear();
        self.hover = None;
        self.timeline_sel = None;
        crash_snapshot(&self.doc, self.dirty());
        if let Mode::Sketch(s) = &self.mode {
            if self.doc.feature(s.fid).is_none() {
                self.mode = Mode::Model;
            }
        }
        if self.doc.component(self.active_comp).is_none() {
            self.active_comp = ROOT;
        }
    }

    /// Called after a caught panic: drop transient interaction state.
    pub fn recover_from_panic(&mut self) {
        if let Some(c) = self.cmd.take() {
            // restore the pre-command document
            self.doc = c.before;
        }
        if let Mode::Sketch(_) = self.mode {
            let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| crate::sketch_mode::finish(self)));
            self.mode = Mode::Model;
        }
        self.hover = None;
        self.selection.clear();
        self.error("Internal error — the last action was cancelled. Your design is safe (please report this).");
    }

    /// Visibility-type change: applied to the live document and to a running
    /// command's snapshot (so cancelling the command keeps it).
    pub fn view_change(&mut self, f: impl Fn(&mut Document)) {
        if let Some(c) = self.cmd.as_mut() {
            f(&mut c.before);
            f(&mut self.doc);
        } else {
            let before = self.doc.clone();
            f(&mut self.doc);
            self.commit(before);
        }
    }

    /// Drop selections that no longer exist (after edits, undo, parameter changes).
    fn prune_selection(&mut self) {
        let st = self.display_state();
        let valid = |p: &Pick| match *p {
            Pick::Face { body, face } => st.bodies.get(&body).map(|b| (face as usize) < b.face_names.len()).unwrap_or(false),
            Pick::Edge { body, edge } => st.bodies.get(&body).map(|b| (edge as usize) < b.edge_names.len()).unwrap_or(false),
            Pick::Body(b) => st.bodies.contains_key(&b),
            Pick::Profile { sketch, index } => st.sketches.get(&sketch).map(|s| index < s.profiles.len()).unwrap_or(false),
            Pick::Plane(f) => st.planes.contains_key(&f),
            Pick::SketchLine { sketch, curve } => st.sketches.get(&sketch).map(|s| s.sketch.line_pts(curve).is_some()).unwrap_or(false),
            Pick::Origin(_) | Pick::Axis(_) => true,
        };
        let keep: Vec<Pick> = self.selection.iter().copied().filter(|p| valid(p)).collect();
        if keep.len() != self.selection.len() {
            self.selection = keep;
        }
    }

    /// Cancel an open command dialog and finish an open sketch.
    pub fn end_interaction(&mut self) {
        if self.cmd.is_some() {
            self.cancel_command();
        }
        if let Mode::Sketch(_) = self.mode {
            crate::sketch_mode::finish(self);
        }
    }

    pub fn cancel_command(&mut self) {
        crate::commands::cancel(self);
    }

    pub fn toast(&mut self, text: impl Into<String>, color: egui::Color32) {
        let now = self.now();
        self.toasts.push(Toast { text: text.into(), color, until: now + 4.0 });
    }
    pub fn info(&mut self, text: impl Into<String>) {
        self.toast(text, theme::ACCENT);
    }
    pub fn error(&mut self, text: impl Into<String>) {
        let t = text.into();
        log::error!("{t}");
        self.toast(t, theme::ERR);
    }
    pub fn now(&self) -> f64 {
        static START: std::sync::OnceLock<Instant> = std::sync::OnceLock::new();
        START.get_or_init(Instant::now).elapsed().as_secs_f64()
    }

    /// State that is displayed.
    pub fn display_state(&self) -> &ModelState {
        self.model.current(&self.doc)
    }

    /// State used for picking (before the feature being edited).
    pub fn pick_state(&self) -> &ModelState {
        if let Some(c) = &self.cmd {
            if let Some(fid) = c.fid {
                if let Some(i) = self.doc.feature_index(fid) {
                    return self.model.state_after(i);
                }
            }
        }
        if let Mode::Sketch(s) = &self.mode {
            if let Some(i) = self.doc.feature_index(s.fid) {
                return self.model.state_after(i);
            }
        }
        self.model.current(&self.doc)
    }

    pub fn body_name(&self, id: BodyId) -> String {
        if let Some(n) = self.doc.body_props(id).name {
            return n;
        }
        let n = self.display_state().bodies.keys().position(|b| *b == id).map(|i| i + 1).unwrap_or(1);
        format!("Body{n}")
    }

    pub fn body_component(&self, st: &ModelState, id: BodyId) -> CompId {
        self.doc.body_props(id).component.or_else(|| st.bodies.get(&id).map(|b| b.component)).unwrap_or(ROOT)
    }

    pub fn body_visible(&self, st: &ModelState, id: BodyId) -> bool {
        let p = self.doc.body_props(id);
        p.visible.unwrap_or(true) && self.doc.component_visible(self.body_component(st, id))
    }

    // ------------------------------------------------------------------ files

    pub fn title(&self) -> String {
        let name = self
            .path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "Untitled".into());
        format!("{}{} — Lusion360", name, if self.dirty() { " •" } else { "" })
    }

    pub fn request(&mut self, action: PendingAction) {
        if self.dirty() {
            self.confirm_discard = Some(action);
        } else {
            self.perform(action);
        }
    }

    pub fn perform(&mut self, action: PendingAction) {
        match action {
            PendingAction::New => self.new_doc(),
            PendingAction::Open(Some(p)) => self.open_path(p),
            PendingAction::Open(None) => {
                if let Some(p) = open_dialog() {
                    self.open_path(p);
                }
            }
            PendingAction::Quit => {
                self.clear_autosave();
                self.allow_close = true;
            }
        }
    }

    pub fn new_doc(&mut self) {
        self.cancel_command();
        self.mode = Mode::Model;
        self.doc = Document::default();
        self.saved_doc = self.doc.clone();
        self.model = Model::new();
        self.undo.clear();
        self.redo.clear();
        self.path = None;
        self.selection.clear();
        self.timeline_sel = None;
        self.active_comp = ROOT;
        self.cam = CameraCtl::new();
    }

    pub fn open_path(&mut self, p: PathBuf) {
        let t0 = Instant::now();
        match io::open_any(&p) {
            Ok(doc) => {
                self.cancel_command();
                self.mode = Mode::Model;
                let native = io::ext_of(&p) == io::NATIVE_EXT;
                self.doc = doc;
                self.model = Model::new();
                self.undo.clear();
                self.redo.clear();
                self.selection.clear();
                self.timeline_sel = None;
                self.active_comp = ROOT;
                if native {
                    self.saved_doc = self.doc.clone();
                    self.path = Some(p.clone());
                } else {
                    // viewing a foreign file: untitled design containing the import
                    self.saved_doc = self.doc.clone();
                    self.path = None;
                }
                self.model.update(&self.doc);
                self.fit_pending = true;
                self.push_recent(&p);
                let ms = t0.elapsed().as_secs_f64() * 1000.0;
                self.info(format!("Opened {} ({ms:.0} ms)", p.file_name().unwrap_or_default().to_string_lossy()));
            }
            Err(e) => self.error(format!("Could not open {}: {e:#}", p.display())),
        }
    }

    fn push_recent(&mut self, p: &std::path::Path) {
        let p = p.to_path_buf();
        self.recent.retain(|r| r != &p);
        self.recent.insert(0, p);
        self.recent.truncate(12);
    }

    pub fn save(&mut self, save_as: bool) -> bool {
        let path = if save_as || self.path.is_none() {
            let mut d = rfd::FileDialog::new().add_filter("Lusion360 design", &[io::NATIVE_EXT]).set_file_name("design.l360");
            if let Some(dir) = self.project_dir.as_ref() {
                d = d.set_directory(dir);
            }
            match d.save_file() {
                Some(mut p) => {
                    if io::ext_of(&p) != io::NATIVE_EXT {
                        p.set_extension(io::NATIVE_EXT);
                    }
                    p
                }
                None => return false,
            }
        } else {
            self.path.clone().unwrap()
        };
        // never write a half-finished command or a rolled-back sketch timeline
        let mut to_save = match &self.cmd {
            Some(c) => c.before.clone(),
            None => self.doc.clone(),
        };
        if let Mode::Sketch(s) = &self.mode {
            if let Some(idx) = to_save.feature_index(s.fid) {
                to_save.marker = s.prev_marker.max(idx + 1).min(to_save.features.len());
            }
        }
        match io::save_native(&to_save, &path) {
            Ok(()) => {
                self.saved_doc = self.doc.clone();
                crash_snapshot(&self.doc, false);
                self.path = Some(path.clone());
                self.push_recent(&path);
                self.clear_autosave();
                self.info(format!("Saved {}", path.display()));
                true
            }
            Err(e) => {
                self.error(format!("Save failed: {e:#}"));
                false
            }
        }
    }

    pub fn import_file(&mut self) {
        let exts = {
            let mut v: Vec<&str> = io::cad_import_extensions().to_vec();
            v.extend_from_slice(lusion_meshio::supported_import_extensions());
            v
        };
        let Some(p) = rfd::FileDialog::new().add_filter("3D files", &exts).pick_file() else { return };
        self.end_interaction();
        let before = self.doc.clone();
        match io::import_into(&mut self.doc, &p, self.active_comp) {
            Ok(n) => {
                self.commit(before);
                self.fit_pending = true;
                self.info(format!("Imported {} object(s) from {}", n, p.display()));
            }
            Err(e) => {
                self.doc = before;
                self.error(format!("Import failed: {e:#}"));
            }
        }
    }

    pub fn export(&mut self, only: Option<Vec<BodyId>>) {
        let Some(mut p) = rfd::FileDialog::new()
            .add_filter("STEP", &["step", "stp"])
            .add_filter("STL (3D printing)", &["stl"])
            .add_filter("3MF (3D printing)", &["3mf"])
            .add_filter("OBJ", &["obj"])
            .add_filter("OpenCASCADE BREP", &["brep"])
            .set_file_name("export.step")
            .save_file()
        else {
            return;
        };
        if io::ext_of(&p).is_empty() {
            p.set_extension("step");
        }
        let st = self.display_state();
        let bodies: Vec<_> = match &only {
            Some(ids) => ids.iter().filter_map(|i| st.bodies.get(i)).collect(),
            None => io::visible_bodies(&self.doc, st),
        };
        let names: Vec<String> = bodies.iter().map(|b| self.body_name(b.id)).collect();
        match io::export(&p, &bodies, &names) {
            Ok(()) => self.info(format!("Exported {} body(ies) to {}", bodies.len(), p.display())),
            Err(e) => self.error(format!("Export failed: {e:#}")),
        }
    }

    // ------------------------------------------------------------------ fitting

    pub fn scene_bounds(&mut self) -> Option<(Vec3, Vec3)> {
        let mut lo = Vec3::splat(f32::INFINITY);
        let mut hi = Vec3::splat(f32::NEG_INFINITY);
        let st = self.model.current(&self.doc).clone();
        for b in st.bodies.values() {
            if !self.body_visible(&st, b.id) {
                continue;
            }
            if let Some(m) = self.meshes.get(b) {
                lo = lo.min(Vec3::from_array(m.bbox.0));
                hi = hi.max(Vec3::from_array(m.bbox.1));
            }
        }
        for ss in st.sketches.values() {
            if let Some((a, b)) = ss.sketch.bounds() {
                for p in [a, b] {
                    let w = ss.frame.to3(p).as_vec3();
                    lo = lo.min(w);
                    hi = hi.max(w);
                }
            }
        }
        (lo.x <= hi.x).then_some((lo, hi))
    }

    pub fn fit_all(&mut self, animate: bool) {
        match self.scene_bounds() {
            Some((lo, hi)) => {
                let c = (lo + hi) * 0.5;
                let r = ((hi - lo).length() * 0.5).max(1.0);
                self.cam.fit(c, r, animate);
            }
            None => self.cam.fit(Vec3::ZERO, 60.0, animate),
        }
    }

    pub fn home_view(&mut self) {
        let rot = crate::camera::Camera::view_rot(45f32.to_radians(), 30f32.to_radians());
        self.cam.set_orientation(rot);
        let target_rot = rot;
        self.fit_all(true);
        // fit_all may have replaced the animation target; keep the home orientation
        let mut to = self.cam.cam;
        to.rot = target_rot;
        if let Some((lo, hi)) = self.scene_bounds() {
            to.target = (lo + hi) * 0.5;
            let r = ((hi - lo).length() * 0.5).max(1.0);
            to.distance = (r / (to.fov_y * 0.5).sin()) * 1.1;
        }
        self.cam.animate_to(to);
    }
}

static CRASH_DOC: std::sync::Mutex<Option<Document>> = std::sync::Mutex::new(None);

/// Remember the latest document for the panic hook.
pub fn crash_snapshot(doc: &Document, dirty: bool) {
    if let Ok(mut g) = CRASH_DOC.lock() {
        *g = dirty.then(|| doc.clone());
    }
}

/// On panic, write the last known document to the recovery file.
pub fn install_panic_hook() {
    let prev = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if let (Ok(g), Some(p)) = (CRASH_DOC.try_lock(), autosave_path()) {
            if let Some(doc) = g.as_ref() {
                if let Some(dir) = p.parent() {
                    let _ = std::fs::create_dir_all(dir);
                }
                let _ = io::save_native(doc, &p);
                eprintln!("Lusion360 crashed; your work was saved to {}", p.display());
            }
        }
        prev(info);
    }));
}

/// Where crash-recovery snapshots go.
pub fn autosave_path() -> Option<PathBuf> {
    let base = if cfg!(windows) {
        std::env::var_os("APPDATA").map(PathBuf::from)
    } else {
        std::env::var_os("XDG_DATA_HOME").map(PathBuf::from).or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/share")))
    }?;
    Some(base.join("lusion360").join(format!("autosave-{}.l360", std::process::id())))
}

/// An autosave left behind by an instance that is no longer running.
pub fn find_orphan_autosave() -> Option<PathBuf> {
    let dir = autosave_path()?.parent()?.to_path_buf();
    let me = std::process::id();
    let mut found: Vec<(std::time::SystemTime, PathBuf)> = Vec::new();
    for e in std::fs::read_dir(&dir).ok()?.flatten() {
        let name = e.file_name().to_string_lossy().to_string();
        let Some(pid) = name.strip_prefix("autosave-").and_then(|r| r.strip_suffix(".l360")).and_then(|p| p.parse::<u32>().ok()) else {
            continue;
        };
        if pid == me {
            continue;
        }
        // still running? (Linux: /proc/<pid>)
        if cfg!(target_os = "linux") && std::path::Path::new(&format!("/proc/{pid}")).exists() {
            continue;
        }
        if let Ok(t) = e.metadata().and_then(|m| m.modified()) {
            found.push((t, e.path()));
        }
    }
    found.sort();
    found.pop().map(|(_, p)| p)
}

impl App {
    /// Snapshot unsaved work every 20 s (in a background thread).
    fn autosave(&mut self) {
        if self.last_autosave.elapsed().as_secs() < 20 || self.recovery.is_some() {
            return;
        }
        self.last_autosave = Instant::now();
        let Some(path) = autosave_path() else { return };
        if !self.dirty() {
            if self.autosaved_doc.is_some() {
                let _ = std::fs::remove_file(&path);
                self.autosaved_doc = None;
            }
            return;
        }
        if self.autosaved_doc.as_ref() == Some(&self.doc) {
            return;
        }
        let doc = self.doc.clone();
        self.autosaved_doc = Some(doc.clone());
        std::thread::spawn(move || {
            if let Some(dir) = path.parent() {
                let _ = std::fs::create_dir_all(dir);
            }
            if let Err(e) = io::save_native(&doc, &path) {
                log::warn!("autosave failed: {e:#}");
            }
        });
    }

    pub fn clear_autosave(&mut self) {
        if let Some(p) = autosave_path() {
            let _ = std::fs::remove_file(p);
        }
        self.autosaved_doc = None;
    }
}

pub fn open_dialog() -> Option<PathBuf> {
    let all = io::all_open_extensions();
    let mut mesh: Vec<&str> = lusion_meshio::supported_import_extensions().to_vec();
    mesh.sort();
    rfd::FileDialog::new()
        .add_filter("All supported", &all)
        .add_filter("Lusion360 design", &[io::NATIVE_EXT])
        .add_filter("STEP / BREP", io::cad_import_extensions())
        .add_filter("Meshes", &mesh)
        .pick_file()
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let now = Instant::now();
        let dt = (now - self.last_frame).as_secs_f32().min(0.1);
        self.frame_ms = self.frame_ms * 0.9 + dt * 1000.0 * 0.1;
        self.last_frame = now;

        self.timeline_hover = None;
        self.autosave();
        // keep the model up to date
        // slow recomputes (fillets on big bodies...) run in the background while
        // the last result stays on screen, so dragging stays smooth
        if self.model.update_interactive(&self.doc, 14.0) {
            self.prune_selection();
        }
        for (shape, m, tool) in self.model.premeshed.drain(..) {
            self.meshes.insert_premeshed(&shape, m, tool);
        }
        if self.model.busy() {
            ctx.request_repaint_after(std::time::Duration::from_millis(8));
        }
        self.meshes.tick();
        self.picks.tick();
        if self.fit_pending {
            self.fit_pending = false;
            self.fit_all(false);
        }
        if self.cam.tick(dt) {
            ctx.request_repaint();
        }
        if self.meshes.busy() {
            // poll background tessellation
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }

        ctx.send_viewport_cmd(egui::ViewportCommand::Title(self.title()));
        self.handle_close(&ctx);
        self.handle_dropped_files(&ctx);
        // before the shortcuts: Esc cancels a drag-out instead of the current tool
        crate::dragdrop::update(self, &ctx, frame);
        self.global_shortcuts(&ctx);

        crate::panels::top_bar(self, ui);
        crate::panels::status_bar(self, ui);
        crate::panels::timeline(self, ui);
        crate::panels::left_panel(self, ui);
        if std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| crate::commands::command_panel(self, ui))).is_err() {
            self.recover_from_panic();
        }
        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            // a bug in an interaction must not take the whole app (and the user's work) down
            let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| crate::viewport::show(self, ui)));
            if r.is_err() {
                self.recover_from_panic();
            }
        });
        crate::panels::dialogs(self, &ctx);
        crate::print::dialog(self, &ctx);
        crate::panels::toasts(self, &ctx);
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        let recent: Vec<String> = self.recent.iter().map(|p| p.display().to_string()).collect();
        storage.set_string("recent", recent.join("\n"));
        storage.set_string("light_bg", if self.light_bg { "1" } else { "0" }.into());
        storage.set_string("slicer", self.slicer_pref.clone().unwrap_or_default());
        storage.set_string("drag_format", self.drag_format.ext().into());
    }
}

impl App {
    fn handle_close(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.viewport().close_requested()) && !self.allow_close && self.dirty() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.confirm_discard = Some(PendingAction::Quit);
        }
        if self.allow_close {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    fn handle_dropped_files(&mut self, ctx: &egui::Context) {
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).filter(|p| !p.as_os_str().is_empty()).collect());
        for p in dropped {
            if io::ext_of(&p) == io::NATIVE_EXT || self.doc.features.is_empty() {
                self.request(PendingAction::Open(Some(p)));
            } else {
                self.end_interaction();
                let before = self.doc.clone();
                match io::import_into(&mut self.doc, &p, self.active_comp) {
                    Ok(_) => {
                        self.commit(before);
                        self.fit_pending = true;
                    }
                    Err(e) => {
                        self.doc = before;
                        self.error(format!("Import failed: {e:#}"));
                    }
                }
            }
        }
    }

    fn global_shortcuts(&mut self, ctx: &egui::Context) {
        if ctx.egui_wants_keyboard_input() {
            return;
        }
        use egui::{Key, KeyboardShortcut as KS, Modifiers as M};
        let mut fire = |k: KS| ctx.input_mut(|i| i.consume_shortcut(&k));
        if fire(KS::new(M::COMMAND | M::SHIFT, Key::Z)) || fire(KS::new(M::COMMAND, Key::Y)) {
            self.redo();
        } else if fire(KS::new(M::COMMAND, Key::Z)) {
            self.undo();
        }
        if fire(KS::new(M::COMMAND | M::SHIFT, Key::S)) {
            self.save(true);
        } else if fire(KS::new(M::COMMAND, Key::S)) {
            self.save(false);
        }
        if fire(KS::new(M::COMMAND, Key::O)) {
            self.request(PendingAction::Open(None));
        }
        if fire(KS::new(M::COMMAND, Key::N)) {
            self.request(PendingAction::New);
        }
        if fire(KS::new(M::COMMAND, Key::I)) {
            self.import_file();
        }
        if fire(KS::new(M::COMMAND, Key::E)) {
            self.export(None);
        }
        if fire(KS::new(M::NONE, Key::Home)) || fire(KS::new(M::NONE, Key::F6)) {
            self.home_view();
        }
        let model_idle = matches!(self.mode, Mode::Model) && self.cmd.is_none();
        if model_idle && fire(KS::new(M::NONE, Key::E)) {
            crate::commands::start(self, crate::commands::CmdKind::Extrude);
        }
        if model_idle && fire(KS::new(M::NONE, Key::H)) {
            crate::commands::start(self, crate::commands::CmdKind::Hole);
        }
        if model_idle && fire(KS::new(M::NONE, Key::I)) {
            crate::commands::start(self, crate::commands::CmdKind::Measure);
        }
        if matches!(self.mode, Mode::Model) && fire(KS::new(M::ALT, Key::X)) {
            self.xray = !self.xray;
        }
        if model_idle && fire(KS::new(M::NONE, Key::Escape)) {
            self.selection.clear();
        }
        if model_idle && fire(KS::new(M::NONE, Key::Delete)) {
            let st = self.display_state().clone();
            let bodies: Vec<BodyId> = self
                .selection
                .iter()
                .filter_map(|p| match p {
                    Pick::Body(b) | Pick::Face { body: b, .. } | Pick::Edge { body: b, .. } => Some(*b),
                    _ => None,
                })
                .filter(|b| st.bodies.contains_key(b))
                .collect();
            if !bodies.is_empty() {
                let before = self.doc.clone();
                for b in bodies {
                    self.doc.bodies.entry(b).or_default().visible = Some(false);
                }
                self.commit(before);
                self.selection.clear();
                self.info("Hidden — show again from the browser");
            } else if let Some(fid) = self.timeline_sel {
                crate::panels::feature_action(self, fid, "delete");
            }
        }
        if fire(KS::new(M::NONE, Key::F)) && matches!(self.mode, Mode::Model) && self.cmd.is_none() {
            // F: fillet in model mode (Fusion), handled with selection
            crate::commands::start(self, crate::commands::CmdKind::Fillet);
        }
        if fire(KS::new(M::NONE, Key::S)) && self.palette.is_none() {
            self.palette = Some((String::new(), 0));
        }
    }
}
