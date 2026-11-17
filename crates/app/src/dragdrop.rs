//! Drag and drop: files in (drop overlay) and bodies out (to slicers / file managers).

use crate::app::App;
use crate::icons as ic;
use crate::theme;
use egui::{Color32, RichText};
use lusion_core::doc::BodyId;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

pub struct DragSession {
    pub label: String,
    pub path: PathBuf,
    ready: Arc<AtomicBool>,
    failed: Arc<AtomicBool>,
    #[cfg(target_os = "linux")]
    over: crate::dragout::Over,
    started: bool,
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

/// Start dragging `bodies` (exported to a 3MF in the background).
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
    let name = if bodies.len() == 1 { format!("{}-{}", sanitize(&design), sanitize(&app.body_name(bodies[0]))) } else { sanitize(&design) };
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
    let path = dir.join(format!("{name}.3mf"));
    // deep copies: the export thread must not share OCCT shapes with the UI thread
    let copies: Vec<lusion_core::model::Body> = bodies
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
    let label = path.file_name().map(|s| s.to_string_lossy().to_string()).unwrap_or_default();
    app.drag_out = Some(DragSession {
        label,
        path,
        ready,
        failed,
        #[cfg(target_os = "linux")]
        over: crate::dragout::Over::Inside,
        started: false,
    });
}

/// Per-frame: drive an active drag-out and draw drop overlays.
pub fn update(app: &mut App, ctx: &egui::Context, frame: &eframe::Frame) {
    file_drop_overlay(ctx);
    let Some(mut s) = app.drag_out.take() else { return };
    let (down, pos, esc) = ctx.input(|i| (i.pointer.primary_down(), i.pointer.latest_pos(), i.key_pressed(egui::Key::Escape)));
    ctx.request_repaint();
    if esc {
        #[cfg(target_os = "linux")]
        if let Some(Some(x)) = app.xdnd.as_ref() {
            x.cancel();
        }
        return;
    }
    #[cfg(target_os = "linux")]
    {
        if app.xdnd.is_none() {
            // only on a real X11 window (on native Wayland XWayland would not see the drag)
            app.xdnd = Some(own_window(frame).and_then(|w| crate::dragout::DragOut::new(Some(w))));
            if app.xdnd.as_ref().map(|x| x.is_none()).unwrap_or(false) {
                app.info("Dragging out of the app needs an X11 session (use File → Export or Print)");
            }
        }
        let Some(Some(x)) = app.xdnd.as_ref() else {
            return;
        };
        if !s.started {
            x.begin(std::slice::from_ref(&s.path));
            s.started = true;
        }
        if down {
            s.over = x.update();
        } else {
            // released: make sure the file exists before handing it over
            let t0 = std::time::Instant::now();
            while !s.ready.load(Ordering::SeqCst) && t0.elapsed().as_secs() < 10 {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if s.failed.load(Ordering::SeqCst) {
                x.cancel();
                app.error("Could not export the dragged bodies");
            } else if x.release() {
                app.info(format!("Dropped {}", s.label));
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
    }
    // ghost label following the cursor (only visible while inside our window)
    if let Some(p) = pos {
        #[cfg(target_os = "linux")]
        let (txt, col) = match s.over {
            crate::dragout::Over::Accepting => ("release to drop", theme::OK),
            crate::dragout::Over::Refused => ("target does not accept files", theme::WARN),
            crate::dragout::Over::Inside => ("drag outside the window — e.g. onto Bambu Studio", theme::TEXT_DIM),
        };
        #[cfg(not(target_os = "linux"))]
        let (txt, col) = ("drag-out is Linux/X11 only for now", theme::TEXT_DIM);
        egui::Area::new(egui::Id::new("drag_ghost")).fixed_pos(p + egui::vec2(16.0, 12.0)).order(egui::Order::Tooltip).interactable(false).show(ctx, |ui| {
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(28, 32, 38, 235))
                .stroke(egui::Stroke::new(1.0, theme::ACCENT))
                .corner_radius(6)
                .inner_margin(egui::Margin::symmetric(10, 6))
                .show(ui, |ui| {
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                    ui.label(RichText::new(format!("{} {}", ic::PRINTER, s.label)).strong());
                    ui.label(RichText::new(txt).small().color(col));
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
