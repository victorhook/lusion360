//! One-click export for 3D printing (3MF) with optional hand-off to a slicer.

use crate::app::App;
use crate::icons as ic;
use crate::theme;
use egui::RichText;
use lusion_core::doc::BodyId;
use std::path::PathBuf;

pub struct PrintDialog {
    pub bodies: Option<Vec<BodyId>>,
    pub path: Option<PathBuf>,
}

#[derive(Clone, Debug)]
pub struct Slicer {
    pub name: String,
    /// program + leading args; the 3MF path is appended
    pub cmd: Vec<String>,
}

fn in_path(exe: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    for dir in std::env::split_paths(&path) {
        for cand in [dir.join(exe), dir.join(format!("{exe}.exe"))] {
            if cand.is_file() {
                return Some(cand);
            }
        }
    }
    None
}

/// Slicers installed on this machine (PATH, flatpak, common AppImage spots).
pub fn detect_slicers() -> Vec<Slicer> {
    let mut v = Vec::new();
    let bins = [
        ("Bambu Studio", "bambu-studio"),
        ("Bambu Studio", "BambuStudio"),
        ("OrcaSlicer", "orca-slicer"),
        ("OrcaSlicer", "OrcaSlicer"),
        ("PrusaSlicer", "prusa-slicer"),
        ("PrusaSlicer", "PrusaSlicer"),
        ("Cura", "cura"),
    ];
    for (name, exe) in bins {
        if let Some(p) = in_path(exe) {
            if !v.iter().any(|s: &Slicer| s.name == name) {
                v.push(Slicer { name: name.into(), cmd: vec![p.display().to_string()] });
            }
        }
    }
    let home = std::env::var("HOME").unwrap_or_default();
    let flatpaks = [
        ("Bambu Studio", "com.bambulab.BambuStudio"),
        ("OrcaSlicer", "io.github.softfever.OrcaSlicer"),
        ("PrusaSlicer", "com.prusa3d.PrusaSlicer"),
    ];
    for (name, id) in flatpaks {
        let sys = PathBuf::from("/var/lib/flatpak/app").join(id);
        let user = PathBuf::from(&home).join(".local/share/flatpak/app").join(id);
        if (sys.exists() || user.exists()) && !v.iter().any(|s: &Slicer| s.name == name) {
            v.push(Slicer { name: name.into(), cmd: vec!["flatpak".into(), "run".into(), id.into()] });
        }
    }
    // AppImages in ~/Applications or ~/Downloads
    for dir in ["Applications", "Downloads", "bin", "opt"] {
        if let Ok(rd) = std::fs::read_dir(PathBuf::from(&home).join(dir)) {
            for e in rd.flatten() {
                let n = e.file_name().to_string_lossy().to_lowercase();
                if n.ends_with(".appimage") {
                    let name = if n.contains("bambu") {
                        "Bambu Studio"
                    } else if n.contains("orca") {
                        "OrcaSlicer"
                    } else if n.contains("prusa") {
                        "PrusaSlicer"
                    } else {
                        continue;
                    };
                    if !v.iter().any(|s: &Slicer| s.name == name) {
                        v.push(Slicer { name: name.into(), cmd: vec![e.path().display().to_string()] });
                    }
                }
            }
        }
    }
    v
}

pub fn export_for_print(app: &mut App) {
    app.print = Some(PrintDialog { bodies: None, path: None });
}

pub fn export_bodies_for_print(app: &mut App, bodies: Vec<BodyId>) {
    app.print = Some(PrintDialog { bodies: Some(bodies), path: None });
}

fn default_name(app: &App) -> String {
    app.path
        .as_ref()
        .and_then(|p| p.file_stem())
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| app.doc.components[0].name.clone())
        .replace(' ', "_")
}

pub fn dialog(app: &mut App, ctx: &egui::Context) {
    let Some(mut d) = app.print.take() else { return };
    if app.slicers.is_none() {
        app.slicers = Some(detect_slicers());
    }
    let slicers = app.slicers.clone().unwrap_or_default();
    // default: open in the first slicer found (Bambu Studio preferred)
    if app.slicer_pref.is_none() {
        app.slicer_pref = Some(slicers.first().map(|s| s.name.clone()).unwrap_or_else(|| "none".into()));
    }
    let st = app.display_state().clone();
    let bodies: Vec<BodyId> = match &d.bodies {
        Some(b) => b.clone(),
        None => lusion_core::io::visible_bodies(&app.doc, &st).iter().map(|b| b.id).collect(),
    };
    let mut keep = true;
    let mut go = false;
    egui::Window::new(format!("{} Export for 3D printing", ic::PRINTER))
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
        .show(ctx, |ui| {
            ui.set_min_width(380.0);
            let (mut lo, mut hi) = (glam::DVec3::splat(f64::INFINITY), glam::DVec3::splat(f64::NEG_INFINITY));
            for b in &bodies {
                if let Some(body) = st.bodies.get(b) {
                    if let Some((a, c)) = body.shape.bbox() {
                        lo = lo.min(a);
                        hi = hi.max(c);
                    } else if let Some(m) = &body.mesh {
                        for p in m.positions.chunks(3) {
                            let v = glam::DVec3::new(p[0] as f64, p[1] as f64, p[2] as f64);
                            lo = lo.min(v);
                            hi = hi.max(v);
                        }
                    }
                }
            }
            ui.label(RichText::new(if bodies.len() == 1 { "1 body".to_string() } else { format!("{} bodies", bodies.len()) }).strong());
            if lo.x <= hi.x {
                let s = hi - lo;
                ui.label(RichText::new(format!("Size: {:.1} × {:.1} × {:.1} mm", s.x, s.y, s.z)).color(theme::TEXT_DIM));
                if s.x > 256.0 || s.y > 256.0 || s.z > 256.0 {
                    ui.colored_label(theme::WARN, format!("{} Larger than a 256 mm bed (X1/P1/A1)", ic::WARNING));
                }
            }
            ui.add_space(6.0);
            ui.label("Format: 3MF (millimetres) — opens directly in Bambu Studio, OrcaSlicer, PrusaSlicer and Cura.");
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                ui.label("Open in:");
                let cur = match app.slicer_pref.as_deref() {
                    None | Some("none") => "Don't open".to_string(),
                    Some(s) => s.to_string(),
                };
                egui::ComboBox::from_id_salt("slicer").selected_text(cur).show_ui(ui, |ui| {
                    if ui.selectable_label(app.slicer_pref.as_deref() == Some("none"), "Don't open").clicked() {
                        app.slicer_pref = Some("none".into());
                    }
                    for s in &slicers {
                        if ui.selectable_label(app.slicer_pref.as_deref() == Some(s.name.as_str()), &s.name).clicked() {
                            app.slicer_pref = Some(s.name.clone());
                        }
                    }
                    if ui.selectable_label(app.slicer_pref.as_deref() == Some("System default"), "System default app").clicked() {
                        app.slicer_pref = Some("System default".into());
                    }
                });
            });
            if slicers.is_empty() {
                ui.label(RichText::new("No slicer found in PATH / flatpak / ~/Applications. \"System default app\" uses xdg-open.").small().color(theme::TEXT_DIM));
            }
            ui.add_space(10.0);
            ui.horizontal(|ui| {
                if ui.add(egui::Button::new(RichText::new(format!("{} Export 3MF…", ic::EXPORT)).strong()).fill(theme::ACCENT_DIM).min_size(egui::vec2(130.0, 28.0))).clicked() {
                    go = true;
                }
                if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    keep = false;
                }
            });
        });
    if go {
        keep = false;
        let mut dlg = rfd::FileDialog::new().add_filter("3MF", &["3mf"]).set_file_name(format!("{}.3mf", default_name(app)));
        if let Some(p) = app.path.as_ref().and_then(|p| p.parent()) {
            dlg = dlg.set_directory(p);
        } else if let Some(p) = &app.project_dir {
            dlg = dlg.set_directory(p);
        }
        if let Some(mut path) = dlg.save_file() {
            if lusion_core::io::ext_of(&path) != "3mf" {
                path.set_extension("3mf");
            }
            let refs: Vec<&lusion_core::model::Body> = bodies.iter().filter_map(|b| st.bodies.get(b)).collect();
            let names: Vec<String> = refs.iter().map(|b| app.body_name(b.id)).collect();
            match lusion_core::io::export(&path, &refs, &names) {
                Ok(()) => {
                    app.info(format!("Exported {}", path.display()));
                    d.path = Some(path.clone());
                    open_in_slicer(app, &slicers, &path);
                }
                Err(e) => app.error(format!("3MF export failed: {e:#}")),
            }
        }
    }
    if keep {
        app.print = Some(d);
    }
}

fn open_in_slicer(app: &mut App, slicers: &[Slicer], path: &std::path::Path) {
    let Some(pref) = app.slicer_pref.clone().filter(|p| p != "none") else { return };
    let cmd: Vec<String> = if pref == "System default" {
        if cfg!(windows) {
            vec!["cmd".into(), "/C".into(), "start".into(), "".into()]
        } else {
            vec!["xdg-open".into()]
        }
    } else {
        match slicers.iter().find(|s| s.name == pref) {
            Some(s) => s.cmd.clone(),
            None => return,
        }
    };
    let mut c = std::process::Command::new(&cmd[0]);
    c.args(&cmd[1..]).arg(path);
    match c.spawn() {
        Ok(_) => app.info(format!("Opening in {pref}…")),
        Err(e) => app.error(format!("Could not start {pref}: {e}")),
    }
}
