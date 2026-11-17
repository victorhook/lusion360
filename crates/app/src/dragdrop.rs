//! Drag and drop: files in (drop overlay) and bodies out (to slicers / file managers).

use crate::app::App;
use crate::icons as ic;
use crate::theme;
use egui::{Color32, RichText};
use lusion_core::doc::BodyId;
use crate::dragicon::Img;
use lusion_core::tess::BodyMesh;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// File format used when dragging bodies out.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum DragFormat {
    Stl,
    ThreeMf,
    Step,
}

impl DragFormat {
    pub const ALL: [DragFormat; 3] = [DragFormat::Stl, DragFormat::ThreeMf, DragFormat::Step];

    pub fn ext(self) -> &'static str {
        match self {
            DragFormat::Stl => "stl",
            DragFormat::ThreeMf => "3mf",
            DragFormat::Step => "step",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            DragFormat::Stl => "STL",
            DragFormat::ThreeMf => "3MF",
            DragFormat::Step => "STEP",
        }
    }

    pub fn from_key(s: &str) -> Option<DragFormat> {
        Self::ALL.into_iter().find(|f| f.ext() == s)
    }

    /// Shift = 3MF (or STL when 3MF is the default), Ctrl = STEP (or STL when STEP is the default).
    pub fn with_mods(self, shift: bool, ctrl: bool) -> DragFormat {
        if ctrl {
            if self == DragFormat::Step { DragFormat::Stl } else { DragFormat::Step }
        } else if shift {
            if self == DragFormat::ThreeMf { DragFormat::Stl } else { DragFormat::ThreeMf }
        } else {
            self
        }
    }

    /// "Shift 3MF · Ctrl STEP" for the given default.
    pub fn alternatives(self) -> String {
        format!("Shift {} · Ctrl {}", self.with_mods(true, false).label(), self.with_mods(false, true).label())
    }
}

struct Job {
    path: PathBuf,
    ready: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
}

pub struct DragSession {
    stem: String,
    /// untouched deep copies; every export job gets its own copy of these
    src: Vec<lusion_core::model::Body>,
    names: Vec<String>,
    jobs: HashMap<DragFormat, Job>,
    fmt: DragFormat,
    thumb: Arc<Mutex<Option<Img>>>,
    thumb_tex: Option<egui::TextureHandle>,
    /// what the X11 drag icon currently shows
    #[cfg_attr(not(target_os = "linux"), allow(dead_code))]
    icon_key: Option<(DragFormat, u8, bool)>,
    #[cfg(target_os = "linux")]
    over: crate::dragout::Over,
    #[cfg_attr(not(target_os = "linux"), allow(dead_code))]
    started: bool,
}

impl DragSession {
    fn file_name(&self, f: DragFormat) -> String {
        format!("{}.{}", self.stem, f.ext())
    }

    /// Start exporting `f` in the background (once).
    fn ensure_job(&mut self, f: DragFormat) {
        if self.jobs.contains_key(&f) {
            return;
        }
        let path = cache_dir().join(self.file_name(f));
        // deep copies: OCCT meshing writes into the shape, threads must not share one
        let copies: Vec<lusion_core::model::Body> = self
            .src
            .iter()
            .map(|b| {
                let mut c = b.clone();
                if !c.shape.is_null() {
                    c.shape = c.shape.deep_copy();
                }
                c
            })
            .collect();
        let names = self.names.clone();
        let ready = Arc::new(AtomicBool::new(false));
        let failed = Arc::new(AtomicBool::new(false));
        {
            let (path, ready, failed) = (path.clone(), ready.clone(), failed.clone());
            std::thread::spawn(move || {
                let refs: Vec<&lusion_core::model::Body> = copies.iter().collect();
                if let Err(e) = lusion_core::io::export(&path, &refs, &names) {
                    log::error!("drag export failed: {e:#}");
                    failed.store(true, Ordering::SeqCst);
                }
                ready.store(true, Ordering::SeqCst);
            });
        }
        self.jobs.insert(f, Job { path, ready, failed });
    }

    fn path(&self) -> PathBuf {
        self.jobs.get(&self.fmt).map(|j| j.path.clone()).unwrap_or_default()
    }
}

fn cache_dir() -> PathBuf {
    let base = std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
        .unwrap_or_else(std::env::temp_dir);
    base.join("lusion360").join("drag")
}

fn sanitize(s: &str) -> String {
    let t: String = s.chars().map(|c| if c.is_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
    let t = t.trim_matches('_').to_string();
    if t.is_empty() {
        "part".into()
    } else {
        t
    }
}

/// Display colour of a body (as in the viewport, without hover/selection tints).
fn body_color(app: &App, st: &lusion_core::model::ModelState, b: &lusion_core::model::Body) -> [f32; 3] {
    let props = app.doc.body_props(b.id);
    let comp = app.doc.component(app.body_component(st, b.id)).and_then(|c| c.color);
    match (props.color, comp) {
        (Some(c), _) | (None, Some(c)) => [c[0], c[1], c[2]],
        _ => {
            let c = if b.is_mesh() { theme::MESH_BODY } else { theme::BODY };
            [c[0], c[1], c[2]]
        }
    }
}

/// Start dragging `bodies` (exported in the background in the default format).
pub fn begin(app: &mut App, bodies: Vec<BodyId>) {
    if bodies.is_empty() || app.drag_out.is_some() {
        return;
    }
    let st = app.display_state().clone();
    let design = app
        .path
        .as_ref()
        .and_then(|p| p.file_stem())
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| app.doc.components[0].name.clone());
    let stem = if bodies.len() == 1 { format!("{}-{}", sanitize(&design), sanitize(&app.body_name(bodies[0]))) } else { sanitize(&design) };
    let dir = cache_dir();
    let _ = std::fs::create_dir_all(&dir);
    // keep the cache small: drop files older than a day
    if let Ok(rd) = std::fs::read_dir(&dir) {
        for e in rd.flatten() {
            if e.metadata().and_then(|m| m.modified()).ok().and_then(|t| t.elapsed().ok()).map(|d| d.as_secs() > 86400).unwrap_or(false) {
                let _ = std::fs::remove_file(e.path());
            }
        }
    }
    let src: Vec<lusion_core::model::Body> = bodies
        .iter()
        .filter_map(|b| st.bodies.get(b))
        .map(|b| {
            let mut c = b.clone();
            if !c.shape.is_null() {
                c.shape = c.shape.deep_copy();
            }
            c
        })
        .collect();
    let names: Vec<String> = bodies.iter().map(|b| app.body_name(*b)).collect();
    // thumbnail from the display meshes, seen like the current view
    let meshes: Vec<(Arc<BodyMesh>, [f32; 3])> = bodies
        .iter()
        .filter_map(|b| st.bodies.get(b))
        .filter_map(|b| Some((app.meshes.get(b)?, body_color(app, &st, b))))
        .collect();
    let cam = app.cam.cam;
    let (right, up, fwd) = (cam.right(), cam.up(), cam.forward());
    let thumb = Arc::new(Mutex::new(None));
    {
        let thumb = thumb.clone();
        std::thread::spawn(move || {
            let t = crate::dragicon::thumbnail(&meshes, right, up, fwd, crate::dragicon::ICON_H - 12);
            *thumb.lock().unwrap() = Some(t);
        });
    }
    let mut s = DragSession {
        stem,
        src,
        names,
        jobs: HashMap::new(),
        fmt: app.drag_format,
        thumb,
        thumb_tex: None,
        icon_key: None,
        #[cfg(target_os = "linux")]
        over: crate::dragout::Over::Inside,
        started: false,
    };
    s.ensure_job(s.fmt);
    app.drag_out = Some(s);
}

/// Per-frame: drive an active drag-out and draw drop overlays.
pub fn update(app: &mut App, ctx: &egui::Context, frame: &eframe::Frame) {
    file_drop_overlay(ctx);
    let Some(mut s) = app.drag_out.take() else { return };
    let (down, pos, esc, emods) = ctx.input(|i| (i.pointer.primary_down(), i.pointer.latest_pos(), i.modifiers.is_none() && i.key_pressed(egui::Key::Escape), i.modifiers));
    ctx.request_repaint();
    if esc {
        ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Escape));
        app.info("Drag cancelled");
        #[cfg(target_os = "linux")]
        if let Some(Some(x)) = app.xdnd.as_ref() {
            x.cancel();
        }
        return;
    }
    if s.thumb_tex.is_none() {
        if let Some(t) = s.thumb.lock().unwrap().as_ref() {
            let img = egui::ColorImage::from_rgba_unmultiplied([t.w, t.h], &t.rgba8());
            s.thumb_tex = Some(ctx.load_texture("drag_thumb", img, egui::TextureOptions::LINEAR));
        }
    }
    #[cfg(target_os = "linux")]
    {
        let _ = emods;
        if app.xdnd.is_none() {
            // only on a real X11 window (on native Wayland XWayland would not see the drag)
            app.xdnd = Some(own_window(frame).and_then(|w| crate::dragout::DragOut::new(Some(w))));
            if app.xdnd.as_ref().map(|x| x.is_none()).unwrap_or(false) {
                app.info("Dragging out of the app needs an X11 session (use File > Export or Print)");
            }
        }
        let Some(Some(x)) = app.xdnd.as_ref() else {
            return;
        };
        if !s.started {
            x.begin(&[s.path()]);
            s.started = true;
        }
        if down {
            let (over, mods) = x.update();
            s.over = over;
            let f = app.drag_format.with_mods(mods.shift, mods.ctrl);
            if f != s.fmt {
                s.fmt = f;
                s.ensure_job(f);
                x.set_files(&[s.path()]);
            }
            // drag icon outside the window
            use crate::dragout::Over;
            let state = match over {
                Over::Inside => 0u8,
                Over::Refused => 1,
                Over::Accepting => 2,
            };
            let thumb = s.thumb.lock().unwrap().clone();
            let key = (s.fmt, state, thumb.is_some());
            if s.icon_key != Some(key) {
                s.icon_key = Some(key);
                let rgb = |c: Color32| [c.r() as f32 / 255.0, c.g() as f32 / 255.0, c.b() as f32 / 255.0];
                let (hint, accent, hc) = match over {
                    Over::Accepting => (format!("Release to drop · {}", s.fmt.label()), rgb(theme::OK), rgb(theme::OK)),
                    Over::Refused => ("This window does not take files".to_string(), rgb(theme::WARN), rgb(theme::WARN)),
                    Over::Inside => (app.drag_format.alternatives(), rgb(theme::ACCENT), rgb(theme::TEXT_DIM)),
                };
                let img = crate::dragicon::card(thumb.as_ref(), &s.file_name(s.fmt), &hint, accent, hc, x.rounded_icons());
                x.set_icon(img.w as u16, img.h as u16, &img.argb());
            }
            x.show_icon(true);
        } else {
            // released: make sure the file exists before handing it over
            let (ready, failed) = s.jobs.get(&s.fmt).map(|j| (j.ready.clone(), j.failed.clone())).unwrap();
            let t0 = std::time::Instant::now();
            while !ready.load(Ordering::SeqCst) && t0.elapsed().as_secs() < 15 {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if failed.load(Ordering::SeqCst) {
                x.cancel();
                app.error(format!("Could not export the dragged bodies as {}", s.fmt.label()));
            } else if x.release() {
                app.info(format!("Dropped {}", s.file_name(s.fmt)));
            }
            return;
        }
    }
    #[cfg(not(target_os = "linux"))]
    {
        let _ = frame;
        if !down {
            return;
        }
        let f = app.drag_format.with_mods(emods.shift, emods.command);
        if f != s.fmt {
            s.fmt = f;
            s.ensure_job(f);
        }
    }
    // in-app ghost (the X11 icon takes over outside the window)
    if let Some(p) = pos.filter(|_| !pointer_outside(ctx)) {
        #[cfg(target_os = "linux")]
        let (txt, col, stroke) = match s.over {
            crate::dragout::Over::Accepting => (format!("release to drop · {}", s.fmt.label()), theme::OK, theme::OK),
            crate::dragout::Over::Refused => ("this window does not take files".to_string(), theme::WARN, theme::WARN),
            crate::dragout::Over::Inside => ("drag outside the window, e.g. onto Bambu Studio".to_string(), theme::TEXT_DIM, theme::ACCENT),
        };
        #[cfg(not(target_os = "linux"))]
        let (txt, col, stroke) = ("drag-out is Linux/X11 only for now".to_string(), theme::TEXT_DIM, theme::ACCENT);
        let name = s.file_name(s.fmt);
        let alts = app.drag_format.alternatives();
        egui::Area::new(egui::Id::new("drag_ghost")).fixed_pos(p + egui::vec2(18.0, 16.0)).order(egui::Order::Tooltip).interactable(false).show(ctx, |ui| {
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(27, 31, 37, 240))
                .stroke(egui::Stroke::new(1.5, stroke))
                .corner_radius(10)
                .inner_margin(egui::Margin::same(6))
                .show(ui, |ui| {
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                    ui.horizontal(|ui| {
                        let sz = egui::vec2(52.0, 52.0);
                        match &s.thumb_tex {
                            Some(t) => {
                                ui.add(egui::Image::new(t).fit_to_exact_size(sz).corner_radius(6));
                            }
                            None => {
                                let (r, _) = ui.allocate_exact_size(sz, egui::Sense::hover());
                                ui.painter().rect_filled(r, 6.0, Color32::from_rgb(0x29, 0x2e, 0x37));
                                ui.painter().text(r.center(), egui::Align2::CENTER_CENTER, ic::CUBE, egui::FontId::proportional(22.0), theme::TEXT_DIM);
                            }
                        }
                        ui.add_space(4.0);
                        ui.vertical(|ui| {
                            ui.add_space(2.0);
                            ui.label(RichText::new(name).strong().color(Color32::WHITE));
                            ui.label(RichText::new(txt).small().color(col));
                            ui.label(RichText::new(alts).small().color(theme::TEXT_DIM));
                        });
                        ui.add_space(6.0);
                    });
                });
        });
    }
    app.drag_out = Some(s);
}

#[cfg(target_os = "linux")]
fn own_window(frame: &eframe::Frame) -> Option<u32> {
    use raw_window_handle::{HasWindowHandle, RawWindowHandle};
    match frame.window_handle().ok()?.as_raw() {
        RawWindowHandle::Xlib(h) => Some(h.window as u32),
        RawWindowHandle::Xcb(h) => Some(h.window.get()),
        _ => None,
    }
}

/// Files dragged over the window: show where they will go.
fn file_drop_overlay(ctx: &egui::Context) {
    let files: Vec<String> = ctx.input(|i| {
        i.raw
            .hovered_files
            .iter()
            .map(|f| f.path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().to_string()).unwrap_or_else(|| "file".into()))
            .collect()
    });
    if files.is_empty() {
        return;
    }
    let rect = ctx.content_rect();
    let painter = ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("file_drop")));
    painter.rect_filled(rect, 0.0, Color32::from_rgba_unmultiplied(15, 40, 80, 120));
    painter.rect_stroke(rect.shrink(10.0), 14.0, egui::Stroke::new(3.0, theme::ACCENT), egui::StrokeKind::Inside);
    painter.text(rect.center() - egui::vec2(0.0, 16.0), egui::Align2::CENTER_CENTER, format!("{}  Drop to open or import", ic::DOWNLOAD_SIMPLE), egui::FontId::proportional(26.0), Color32::WHITE);
    painter.text(rect.center() + egui::vec2(0.0, 18.0), egui::Align2::CENTER_CENTER, files.join(", "), egui::FontId::proportional(14.0), Color32::from_gray(210));
    painter.text(rect.center() + egui::vec2(0.0, 42.0), egui::Align2::CENTER_CENTER, ".l360 opens · STEP / STL / 3MF / OBJ / glTF are added to the design", egui::FontId::proportional(12.0), Color32::from_gray(170));
}

/// All visible bodies (what "Print" would export).
pub fn visible_bodies(app: &App) -> Vec<BodyId> {
    let st = app.display_state();
    let mut v: Vec<BodyId> = st.bodies.keys().copied().filter(|b| app.body_visible(st, *b)).collect();
    v.sort();
    v
}

/// Bodies to drag when the user grabs `body` in the viewport: the whole selection
/// if `body` is part of it, otherwise just `body`.
pub fn bodies_for_grab(app: &App, body: BodyId) -> Vec<BodyId> {
    use crate::app::Pick;
    let sel: Vec<BodyId> = app
        .selection
        .iter()
        .filter_map(|p| match p {
            Pick::Body(b) | Pick::Face { body: b, .. } | Pick::Edge { body: b, .. } => Some(*b),
            _ => None,
        })
        .collect();
    if sel.contains(&body) {
        let mut v = sel;
        v.sort();
        v.dedup();
        v
    } else {
        vec![body]
    }
}

/// Pointer is outside the window (while a button is held X11 keeps sending us motion).
pub fn pointer_outside(ctx: &egui::Context) -> bool {
    let r = ctx.content_rect();
    ctx.input(|i| i.pointer.latest_pos()).map(|p| !r.shrink(2.0).contains(p)).unwrap_or(true)
}
