//! Toolbar, browser tree, timeline, status bar, dialogs.

use crate::app::{App, LeftTab, Mode, PendingAction, Pick, RenameTarget};
use crate::commands::{self, CmdKind};
use crate::icons as ic;
use crate::sketch_mode;
use crate::theme;
use egui::{Color32, RichText};
use lusion_core::doc::*;
use lusion_core::expr::{fmt_num, Parameter};

fn tool_button(ui: &mut egui::Ui, icon: &str, label: &str, tip: &str) -> egui::Response {
    tool_button_active(ui, icon, label, tip, false)
}

/// Ribbon button; `active` draws it highlighted (the running command).
fn tool_button_active(ui: &mut egui::Ui, icon: &str, label: &str, tip: &str, active: bool) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(52.0, 46.0), egui::Sense::click());
    let p = ui.painter();
    let hovered = resp.hovered();
    if active {
        p.rect_filled(rect.shrink(1.0), 6.0, theme::ACCENT_DIM);
        p.rect_stroke(rect.shrink(1.0), 6.0, egui::Stroke::new(1.0, theme::ACCENT), egui::StrokeKind::Inside);
    } else if hovered {
        p.rect_filled(rect.shrink(1.0), 6.0, Color32::from_rgb(0x2c, 0x32, 0x3a));
    }
    let fg = if active { Color32::WHITE } else if hovered { Color32::from_rgb(0xe8, 0xee, 0xf6) } else { Color32::from_rgb(0xc8, 0xd0, 0xda) };
    p.text(rect.center_top() + egui::vec2(0.0, 16.0), egui::Align2::CENTER_CENTER, icon, egui::FontId::proportional(21.0), fg);
    p.text(rect.center_bottom() - egui::vec2(0.0, 8.0), egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(10.0), if active { Color32::WHITE } else { theme::TEXT_DIM });
    if hovered {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    resp.on_hover_text(tip)
}

pub fn top_bar(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::top("menu").frame(egui::Frame::new().fill(Color32::from_rgb(0x17, 0x19, 0x1d)).inner_margin(egui::Margin::symmetric(8, 3))).show(ui, |ui| {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.label(RichText::new(format!("{} Lusion360", ic::CUBE)).strong().color(theme::ACCENT));
            ui.add_space(8.0);
            ui.menu_button("File", |ui| {
                ui.set_min_width(240.0);
                if ui.button(format!("{}  New design            Ctrl+N", ic::FILE_PLUS)).clicked() {
                    app.request(PendingAction::New);
                    ui.close();
                }
                if ui.button(format!("{}  Open…                    Ctrl+O", ic::FOLDER_OPEN)).clicked() {
                    app.request(PendingAction::Open(None));
                    ui.close();
                }
                ui.menu_button(format!("{}  Open recent", ic::CLOCK_COUNTER_CLOCKWISE), |ui| {
                    if app.recent.is_empty() {
                        ui.label("Nothing yet");
                    }
                    for p in app.recent.clone() {
                        if ui.button(p.file_name().unwrap_or_default().to_string_lossy()).on_hover_text(p.display().to_string()).clicked() {
                            app.request(PendingAction::Open(Some(p)));
                            ui.close();
                        }
                    }
                });
                if ui.button(format!("{}  Open folder as project…", ic::FOLDER)).clicked() {
                    if let Some(d) = rfd::FileDialog::new().pick_folder() {
                        app.project_dir = Some(d);
                        app.left_tab = LeftTab::Project;
                    }
                    ui.close();
                }
                ui.separator();
                if ui.button(format!("{}  Save                        Ctrl+S", ic::FLOPPY_DISK)).clicked() {
                    app.save(false);
                    ui.close();
                }
                if ui.button(format!("{}  Save as…           Ctrl+Shift+S", ic::FLOPPY_DISK_BACK)).clicked() {
                    app.save(true);
                    ui.close();
                }
                ui.separator();
                if ui.button(format!("{}  Import into design…  Ctrl+I", ic::DOWNLOAD_SIMPLE)).clicked() {
                    app.import_file();
                    ui.close();
                }
                if ui.button(format!("{}  Export…                   Ctrl+E", ic::EXPORT)).clicked() {
                    app.export(None);
                    ui.close();
                }
                if ui.button(format!("{}  Export for 3D printing (3MF)…", ic::PRINTER)).clicked() {
                    crate::print::export_for_print(app);
                    ui.close();
                }
                ui.separator();
                if ui.button(format!("{}  Quit", ic::SIGN_OUT)).clicked() {
                    app.request(PendingAction::Quit);
                    ui.close();
                }
            });
            ui.menu_button("Edit", |ui| {
                if ui.add_enabled(!app.undo.is_empty(), egui::Button::new(format!("{}  Undo   Ctrl+Z", ic::ARROW_U_UP_LEFT))).clicked() {
                    app.undo();
                    ui.close();
                }
                if ui.add_enabled(!app.redo.is_empty(), egui::Button::new(format!("{}  Redo   Ctrl+Y", ic::ARROW_U_UP_RIGHT))).clicked() {
                    app.redo();
                    ui.close();
                }
                ui.separator();
                if ui.button(format!("{}  Parameters…", ic::FUNCTION)).clicked() {
                    app.show_params = true;
                    ui.close();
                }
                if ui.button(format!("{}  Command search   S", ic::MAGNIFYING_GLASS)).clicked() {
                    app.palette = Some((String::new(), 0));
                    ui.close();
                }
            });
            ui.menu_button("View", |ui| {
                ui.checkbox(&mut app.show_origin, "Show origin planes");
                ui.checkbox(&mut app.show_edges, "Show edges");
                ui.checkbox(&mut app.light_bg, "Light background");
                ui.checkbox(&mut app.cam.cam.ortho, "Orthographic");
                ui.separator();
                if ui.button("Home view   (Home)").clicked() {
                    app.home_view();
                    ui.close();
                }
                if ui.button("Fit all").clicked() {
                    app.fit_all(true);
                    ui.close();
                }
            });
            ui.menu_button("Help", |ui| {
                if ui.button("Keyboard & mouse…").clicked() {
                    app.about = true;
                    ui.close();
                }
            });
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(format!("{:.0} fps", 1000.0 / app.frame_ms.max(0.1))).small().color(theme::TEXT_DIM));
                if ui.add(egui::Button::new(RichText::new(format!("{} Print", ic::PRINTER))).fill(theme::ACCENT_DIM)).on_hover_text("Export visible bodies as 3MF for Bambu Studio / PrusaSlicer / Orca").clicked() {
                    crate::print::export_for_print(app);
                }
                if ui.add_enabled(!app.redo.is_empty(), egui::Button::new(ic::ARROW_U_UP_RIGHT).frame(false)).on_hover_text("Redo").clicked() {
                    app.redo();
                }
                if ui.add_enabled(!app.undo.is_empty(), egui::Button::new(ic::ARROW_U_UP_LEFT).frame(false)).on_hover_text("Undo").clicked() {
                    app.undo();
                }
            });
        });
    });

    // tool ribbon
    egui::Panel::top("ribbon").frame(egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::symmetric(10, 4))).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 2.0;
            if let Mode::Sketch(_) = app.mode {
                ui.label(RichText::new("SKETCH").small().strong().color(theme::ACCENT));
                ui.add_space(6.0);
                sketch_mode::toolbar(app, ui);
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui
                        .add(egui::Button::new(RichText::new(format!("{}  Finish Sketch", ic::CHECK_CIRCLE)).strong().color(Color32::WHITE)).fill(Color32::from_rgb(0x2f, 0x8f, 0x4e)).min_size(egui::vec2(130.0, 30.0)))
                        .on_hover_text("Esc with nothing selected also finishes")
                        .clicked()
                    {
                        sketch_mode::finish(app);
                    }
                    if let Mode::Sketch(s) = &app.mode {
                        let dof = s.report.dof;
                        let (txt, col) = if !s.report.converged {
                            ("conflict".to_string(), theme::ERR)
                        } else if dof == 0 {
                            ("fully constrained".to_string(), theme::OK)
                        } else {
                            (format!("{dof} DOF free"), theme::TEXT_DIM)
                        };
                        ui.label(RichText::new(txt).color(col));
                    }
                });
                return;
            }
            ui.label(RichText::new("CREATE").small().strong().color(theme::TEXT_DIM));
            let items: [(CmdKind, &str, &str); 3] = [
                (CmdKind::NewSketch, "Sketch", "Create a sketch on a plane or face"),
                (CmdKind::Extrude, "Extrude", "Extrude profiles or faces (E)"),
                (CmdKind::Revolve, "Revolve", "Revolve profiles around an axis"),
            ];
            for (k, l, t) in items {
                let active = app.cmd.as_ref().map(|c| c.kind == k).unwrap_or(false);
                if tool_button_active(ui, k.icon(), l, t, active).clicked() {
                    commands::start(app, k);
                }
            }
            let plane_active = app.cmd.as_ref().map(|c| c.kind == CmdKind::Plane).unwrap_or(false);
            if tool_button_active(ui, CmdKind::Plane.icon(), "Plane", "Offset construction plane", plane_active).clicked() {
                commands::start(app, CmdKind::Plane);
            }
            ui.separator();
            ui.label(RichText::new("MODIFY").small().strong().color(theme::TEXT_DIM));
            let items: [(CmdKind, &str, &str); 7] = [
                (CmdKind::Fillet, "Fillet", "Round edges (F)"),
                (CmdKind::Chamfer, "Chamfer", "Bevel edges"),
                (CmdKind::Shell, "Shell", "Hollow out a body"),
                (CmdKind::Combine, "Combine", "Join / cut / intersect bodies"),
                (CmdKind::Move, "Move", "Move or copy bodies"),
                (CmdKind::Pattern, "Pattern", "Circular / linear pattern of bodies"),
                (CmdKind::Mirror, "Mirror", "Mirror bodies across a plane"),
            ];
            for (k, l, t) in items {
                let active = app.cmd.as_ref().map(|c| c.kind == k).unwrap_or(false);
                if tool_button_active(ui, k.icon(), l, t, active).clicked() {
                    commands::start(app, k);
                }
            }
            ui.separator();
            ui.label(RichText::new("INSERT").small().strong().color(theme::TEXT_DIM));
            if tool_button(ui, ic::DOWNLOAD_SIMPLE, "Import", "Insert STEP / STL / OBJ / 3MF … (Ctrl+I)").clicked() {
                app.import_file();
            }
            if tool_button_active(ui, ic::FUNCTION, "Params", "User parameters", app.show_params).clicked() {
                app.show_params = true;
            }
            ui.separator();
            ui.label(RichText::new("OUTPUT").small().strong().color(theme::TEXT_DIM));
            if tool_button(ui, ic::EXPORT, "Export", "STEP / STL / 3MF / OBJ (Ctrl+E)").clicked() {
                app.export(None);
            }
            if tool_button_active(ui, ic::PRINTER, "3D Print", "Export 3MF for your slicer", app.print.is_some()).clicked() {
                crate::print::export_for_print(app);
            }
        });
    });
}

pub fn status_bar(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::bottom("status").frame(egui::Frame::new().fill(Color32::from_rgb(0x17, 0x19, 0x1d)).inner_margin(egui::Margin::symmetric(10, 3))).show(ui, |ui| {
        ui.horizontal(|ui| {
            let hint = match (&app.mode, &app.cmd) {
                (Mode::Sketch(s), _) => s.tool.hint().to_string(),
                (_, Some(c)) => format!("{} — pick in the viewport", c.kind.title()),
                _ => match app.selection.len() {
                    0 => "Select faces, edges or profiles · double-click a profile to extrude · S for command search".into(),
                    n => format!("{n} selected · E extrude · F fillet · Del hide"),
                },
            };
            ui.label(RichText::new(hint).color(theme::TEXT_DIM));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(RichText::new(format!("recompute {:.1} ms", app.model.last_update_ms)).small().color(theme::TEXT_DIM));
                ui.separator();
                let st = app.display_state();
                let nb = st.bodies.len();
                ui.label(RichText::new(format!("{nb} bodies")).small().color(theme::TEXT_DIM));
                if let Some(info) = selection_info(app) {
                    ui.separator();
                    ui.label(RichText::new(info).small().color(theme::ACCENT));
                }
                if let Some(h) = app.hover {
                    ui.separator();
                    ui.label(RichText::new(format!("{:.2}, {:.2}, {:.2}", h.point.x, h.point.y, h.point.z)).small().monospace().color(theme::TEXT_DIM));
                }
            });
        });
    });
}

/// Quick measurements of the current selection.
fn selection_info(app: &App) -> Option<String> {
    let st = app.display_state();
    match app.selection.as_slice() {
        [Pick::Face { body, face }] => {
            let f = st.bodies.get(body)?.shape.faces().get(*face as usize)?.face_info().ok()?;
            let kind = match f.kind {
                lusion_kernel::SurfaceKind::Plane => "Plane".to_string(),
                lusion_kernel::SurfaceKind::Cylinder => format!("Cylinder Ø{}", fmt_num(f.radius * 2.0)),
                lusion_kernel::SurfaceKind::Cone => "Cone".into(),
                lusion_kernel::SurfaceKind::Sphere => format!("Sphere R{}", fmt_num(f.radius)),
                lusion_kernel::SurfaceKind::Torus => "Torus".into(),
                lusion_kernel::SurfaceKind::Other => "Surface".into(),
            };
            Some(format!("{kind} · area {} mm²", fmt_num((f.area * 100.0).round() / 100.0)))
        }
        [Pick::Edge { body, edge }] => {
            let e = st.bodies.get(body)?.shape.edges().get(*edge as usize)?.edge_info().ok()?;
            let r = (e.length * 1000.0).round() / 1000.0;
            Some(match e.kind {
                lusion_kernel::CurveKind::Circle => format!("Arc R{} · length {} mm", fmt_num(e.radius), fmt_num(r)),
                _ => format!("Edge length {} mm", fmt_num(r)),
            })
        }
        [Pick::Body(b)] => {
            let body = st.bodies.get(b)?;
            if body.is_mesh() {
                return Some(format!("Mesh · {} triangles", body.mesh.as_ref()?.indices.len() / 3));
            }
            let v = body.shape.volume().ok()?;
            let (lo, hi) = body.shape.bbox()?;
            let d = hi - lo;
            Some(format!("Volume {} cm³ · {}×{}×{} mm", fmt_num((v / 1000.0 * 100.0).round() / 100.0), fmt_num(d.x.round()), fmt_num(d.y.round()), fmt_num(d.z.round())))
        }
        [Pick::Face { body: b1, face: f1 }, Pick::Face { body: b2, face: f2 }] => {
            let a = st.bodies.get(b1)?.shape.faces().get(*f1 as usize)?.face_info().ok()?;
            let b = st.bodies.get(b2)?.shape.faces().get(*f2 as usize)?.face_info().ok()?;
            if a.kind == lusion_kernel::SurfaceKind::Plane && b.kind == lusion_kernel::SurfaceKind::Plane && a.normal.cross(b.normal).length() < 1e-6 {
                let d = (b.origin - a.origin).dot(a.normal).abs();
                Some(format!("Distance between planes {} mm", fmt_num((d * 1000.0).round() / 1000.0)))
            } else {
                let ang = a.normal.angle_between(b.normal).to_degrees();
                Some(format!("Angle {}°", fmt_num((ang * 100.0).round() / 100.0)))
            }
        }
        _ => None,
    }
}

fn feature_icon(k: &FeatureKind) -> &'static str {
    match k {
        FeatureKind::Sketch(_) => ic::PENCIL_SIMPLE_LINE,
        FeatureKind::Extrude(_) => ic::ARROW_FAT_UP,
        FeatureKind::Revolve(_) => ic::ARROWS_CLOCKWISE,
        FeatureKind::Fillet(_) => ic::BEZIER_CURVE,
        FeatureKind::Chamfer(_) => ic::POLYGON,
        FeatureKind::Shell(_) => ic::CUBE_TRANSPARENT,
        FeatureKind::Plane(_) => ic::SQUARE_HALF,
        FeatureKind::Combine(_) => ic::UNITE,
        FeatureKind::Move(_) => ic::ARROWS_OUT_CARDINAL,
        FeatureKind::Pattern(_) => ic::DOTS_NINE,
        FeatureKind::Mirror(_) => ic::FLIP_HORIZONTAL,
        FeatureKind::ImportShape(_) => ic::FILE_ARROW_DOWN,
        FeatureKind::ImportMesh(_) => ic::TRIANGLE,
    }
}

pub fn timeline(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::bottom("timeline").frame(egui::Frame::new().fill(theme::PANEL).inner_margin(egui::Margin::symmetric(10, 6))).show(ui, |ui| {
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 3.0;
            let n = app.doc.features.len();
            let editing = app.cmd.is_some() || matches!(app.mode, Mode::Sketch(_));
            let mut set_marker: Option<usize> = None;
            if ui.add_enabled(!editing, egui::Button::new(ic::SKIP_BACK).frame(false)).on_hover_text("Roll back to start").clicked() {
                set_marker = Some(0);
            }
            if ui.add_enabled(!editing && app.doc.marker > 0, egui::Button::new(ic::CARET_LEFT).frame(false)).on_hover_text("Step back").clicked() {
                set_marker = Some(app.doc.marker - 1);
            }
            if ui.add_enabled(!editing && app.doc.marker < n, egui::Button::new(ic::CARET_RIGHT).frame(false)).on_hover_text("Step forward").clicked() {
                set_marker = Some(app.doc.marker + 1);
            }
            if ui.add_enabled(!editing, egui::Button::new(ic::SKIP_FORWARD).frame(false)).on_hover_text("Roll to end").clicked() {
                set_marker = Some(n);
            }
            ui.separator();
            egui::ScrollArea::horizontal().id_salt("tl_scroll").show(ui, |ui| {
                ui.horizontal(|ui| {
                    let marker = app.doc.marker;
                    let mut action: Option<(FeatureId, &str)> = None;
                    let mut rects: Vec<egui::Rect> = Vec::new();
                    let mut marker_resp: Option<egui::Response> = None;
                    for (i, f) in app.doc.features.clone().iter().enumerate() {
                        if i == marker {
                            marker_resp = Some(marker_bar(ui, editing));
                        }
                        let status = app.model.feature_status(i).cloned();
                        let rolled = i >= marker;
                        let mut col = Color32::from_rgb(0x2e, 0x34, 0x3c);
                        let mut fg = Color32::from_rgb(0xd8, 0xde, 0xe6);
                        if let Some(s) = &status {
                            if s.error.is_some() {
                                col = Color32::from_rgb(0x6a, 0x26, 0x26);
                            } else if !s.warnings.is_empty() {
                                col = Color32::from_rgb(0x5e, 0x4b, 0x1c);
                            }
                        }
                        if f.suppressed || rolled {
                            fg = Color32::from_rgb(0x70, 0x76, 0x80);
                            col = col.gamma_multiply(0.6);
                        }
                        let sel = app.timeline_sel == Some(f.id);
                        let mut b = egui::Button::new(RichText::new(feature_icon(&f.kind)).size(17.0).color(fg)).fill(col).min_size(egui::vec2(30.0, 28.0));
                        if sel {
                            b = b.stroke(egui::Stroke::new(1.5, theme::ACCENT));
                        }
                        let r = ui.add(b);
                        rects.push(r.rect);
                        let mut tip = format!("{} ({})", f.name, f.kind.type_name());
                        if let Some(s) = &status {
                            if let Some(e) = &s.error {
                                tip += &format!("\n⚠ {e}");
                            }
                            for w in &s.warnings {
                                tip += &format!("\n⚠ {w}");
                            }
                            tip += &format!("\n{:.1} ms", s.millis);
                        }
                        if f.suppressed {
                            tip += "\n(suppressed)";
                        }
                        tip += "\nDouble-click to edit · right-click for more";
                        let r = r.on_hover_text(tip);
                        if r.hovered() {
                            app.timeline_hover = Some(f.id);
                        }
                        if r.clicked() {
                            app.timeline_sel = Some(f.id);
                        }
                        if r.double_clicked() {
                            action = Some((f.id, "edit"));
                        }
                        r.context_menu(|ui| {
                            ui.label(RichText::new(&f.name).strong());
                            if ui.button(format!("{}  Edit", ic::PENCIL_SIMPLE)).clicked() {
                                action = Some((f.id, "edit"));
                                ui.close();
                            }
                            if ui.button(format!("{}  Rename", ic::TEXT_AA)).clicked() {
                                action = Some((f.id, "rename"));
                                ui.close();
                            }
                            if ui.button(format!("{}  Roll to here", ic::CARET_LINE_RIGHT)).clicked() {
                                action = Some((f.id, "roll"));
                                ui.close();
                            }
                            if ui.button(format!("{}  {}", ic::PROHIBIT, if f.suppressed { "Unsuppress" } else { "Suppress" })).clicked() {
                                action = Some((f.id, "suppress"));
                                ui.close();
                            }
                            ui.separator();
                            if ui.add_enabled(i > 0, egui::Button::new(format!("{}  Move earlier", ic::ARROW_LEFT))).clicked() {
                                action = Some((f.id, "left"));
                                ui.close();
                            }
                            if ui.add_enabled(i + 1 < n, egui::Button::new(format!("{}  Move later", ic::ARROW_RIGHT))).clicked() {
                                action = Some((f.id, "right"));
                                ui.close();
                            }
                            ui.separator();
                            if ui.button(RichText::new(format!("{}  Delete", ic::TRASH)).color(theme::ERR)).clicked() {
                                action = Some((f.id, "delete"));
                                ui.close();
                            }
                        });
                    }
                    if marker >= app.doc.features.len() {
                        marker_resp = Some(marker_bar(ui, editing));
                    }
                    // drag the marker to roll the timeline back/forward
                    if let Some(mr) = marker_resp {
                        if mr.dragged() {
                            if let Some(p) = ui.ctx().pointer_interact_pos() {
                                let idx = rects.iter().filter(|r| r.center().x < p.x).count();
                                if idx != app.doc.marker {
                                    set_marker = Some(idx);
                                }
                            }
                        }
                    }
                    if let Some((fid, a)) = action {
                        feature_action(app, fid, a);
                    }
                });
            });
            if let Some(m) = set_marker {
                let before = app.doc.clone();
                app.doc.marker = m.min(n);
                app.commit(before);
            }
        });
    });
}

fn marker_bar(ui: &mut egui::Ui, locked: bool) -> egui::Response {
    let sense = if locked { egui::Sense::hover() } else { egui::Sense::drag() };
    let (r, resp) = ui.allocate_exact_size(egui::vec2(10.0, 28.0), sense);
    let hot = resp.hovered() || resp.dragged();
    let col = if hot { egui::Color32::from_rgb(0x7c, 0xbd, 0xff) } else { theme::ACCENT };
    ui.painter().rect_filled(egui::Rect::from_center_size(r.center(), egui::vec2(if hot { 4.0 } else { 3.0 }, 30.0)), 1.5, col);
    ui.painter().circle_filled(egui::pos2(r.center().x, r.top() - 1.0), if hot { 5.5 } else { 4.0 }, col);
    if !locked {
        if hot {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeHorizontal);
        }
        resp.on_hover_text("Drag to roll the timeline back / forward")
    } else {
        resp
    }
}

pub fn feature_action(app: &mut App, fid: FeatureId, a: &str) {
    if a != "edit" && a != "rename" {
        app.end_interaction();
    }
    let Some(i) = app.doc.feature_index(fid) else { return };
    match a {
        "edit" => commands::edit(app, fid),
        "rename" => {
            let name = app.doc.features[i].name.clone();
            app.rename = Some((RenameTarget::Feature(fid), name));
        }
        "roll" => {
            let before = app.doc.clone();
            app.doc.marker = i + 1;
            app.commit(before);
        }
        "suppress" => {
            let before = app.doc.clone();
            app.doc.features[i].suppressed = !app.doc.features[i].suppressed;
            app.commit(before);
        }
        "left" | "right" => {
            let j = if a == "left" { i.wrapping_sub(1) } else { i + 1 };
            if j < app.doc.features.len() {
                let before = app.doc.clone();
                app.doc.features.swap(i, j);
                app.commit(before);
            }
        }
        "delete" => {
            let before = app.doc.clone();
            app.doc.delete_feature(fid);
            app.commit(before);
            app.timeline_sel = None;
        }
        _ => {}
    }
}

// ------------------------------------------------------------------ browser

pub fn left_panel(app: &mut App, ui: &mut egui::Ui) {
    egui::Panel::left("browser").resizable(true).default_size(250.0).min_size(180.0).show(ui, |ui| {
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.selectable_value(&mut app.left_tab, LeftTab::Browser, RichText::new(format!("{} Browser", ic::TREE_STRUCTURE)));
            ui.selectable_value(&mut app.left_tab, LeftTab::Project, RichText::new(format!("{} Project", ic::FOLDER)));
        });
        ui.separator();
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| match app.left_tab {
            LeftTab::Browser => browser(app, ui),
            LeftTab::Project => project(app, ui),
        });
    });
}

fn eye(ui: &mut egui::Ui, visible: bool) -> bool {
    ui.add(egui::Button::new(RichText::new(if visible { ic::EYE } else { ic::EYE_SLASH }).color(if visible { theme::TEXT_DIM } else { Color32::from_rgb(0x60, 0x66, 0x70) })).frame(false))
        .on_hover_text(if visible { "Hide" } else { "Show" })
        .clicked()
}

fn browser(app: &mut App, ui: &mut egui::Ui) {
    // origin
    ui.horizontal(|ui| {
        if eye(ui, app.show_origin) {
            app.show_origin = !app.show_origin;
        }
        ui.label(format!("{} Origin", ic::CROSSHAIR));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            if ui.small_button(format!("{} Component", ic::PLUS)).on_hover_text("New component inside the active one").clicked() {
                let before = app.doc.clone();
                let name = format!("Component{}", app.doc.components.len());
                let parent = app.active_comp;
                let c = app.doc.add_component(&name, parent);
                app.active_comp = c;
                app.commit(before);
                app.info(format!("{name} created and activated — new features go into it"));
            }
        });
    });
    component_tree(app, ui, ROOT, 0);
    ui.add_space(8.0);
    ui.label(RichText::new("Tip: drag bodies onto a component to move them.").small().color(theme::TEXT_DIM));
}

fn component_tree(app: &mut App, ui: &mut egui::Ui, cid: CompId, depth: usize) {
    let Some(comp) = app.doc.component(cid).cloned() else { return };
    let st = app.display_state().clone();
    let id = ui.make_persistent_id(("comp", cid));
    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true)
        .show_header(ui, |ui| {
            if eye(ui, comp.visible) {
                app.view_change(|d| {
                    if let Some(c) = d.component_mut(cid) {
                        c.visible = !c.visible;
                    }
                });
            }
            let active = app.active_comp == cid;
            let icon = if depth == 0 { ic::CUBE } else { ic::PUZZLE_PIECE };
            let mut text = RichText::new(format!("{icon} {}", comp.name));
            if active {
                text = text.strong().color(theme::ACCENT);
            }
            let r = ui.add(egui::Label::new(text).sense(egui::Sense::click())).on_hover_text("Click to activate · right-click for options");
            // drop target for bodies dragged from the browser
            if r.dnd_hover_payload::<BodyId>().is_some() {
                ui.painter().rect_stroke(r.rect.expand(2.0), 4.0, egui::Stroke::new(1.5, theme::ACCENT), egui::StrokeKind::Outside);
            }
            if let Some(b) = r.dnd_release_payload::<BodyId>() {
                let before = app.doc.clone();
                app.doc.bodies.entry(*b).or_default().component = Some(cid);
                app.commit(before);
            }
            if r.clicked() {
                app.active_comp = cid;
            }
            if r.double_clicked() {
                app.rename = Some((RenameTarget::Component(cid), comp.name.clone()));
            }
            r.context_menu(|ui| {
                if ui.button(format!("{}  Activate", ic::RADIO_BUTTON)).clicked() {
                    app.active_comp = cid;
                    ui.close();
                }
                if ui.button(format!("{}  New component", ic::PLUS)).clicked() {
                    let before = app.doc.clone();
                    let name = format!("Component{}", app.doc.components.len());
                    let n = app.doc.add_component(&name, cid);
                    app.active_comp = n;
                    app.commit(before);
                    ui.close();
                }
                if ui.button(format!("{}  Rename", ic::TEXT_AA)).clicked() {
                    app.rename = Some((RenameTarget::Component(cid), comp.name.clone()));
                    ui.close();
                }
                ui.menu_button(format!("{}  Colour", ic::PALETTE), |ui| {
                    for (name, c) in PALETTE {
                        if ui.button(*name).clicked() {
                            let before = app.doc.clone();
                            if let Some(x) = app.doc.component_mut(cid) {
                                x.color = *c;
                            }
                            app.commit(before);
                            ui.close();
                        }
                    }
                });
                if ui.button(format!("{}  Export component…", ic::EXPORT)).clicked() {
                    let ids: Vec<BodyId> = st.bodies.keys().copied().filter(|b| app.doc.is_ancestor(cid, app.body_component(&st, *b))).collect();
                    app.export(Some(ids));
                    ui.close();
                }
                if depth > 0 && ui.button(RichText::new(format!("{}  Delete component", ic::TRASH)).color(theme::ERR)).clicked() {
                    let before = app.doc.clone();
                    let parent = comp.parent.unwrap_or(ROOT);
                    // move contents to the parent
                    for f in app.doc.features.iter_mut().filter(|f| f.component == cid) {
                        f.component = parent;
                    }
                    for c in app.doc.components.iter_mut().filter(|c| c.parent == Some(cid)) {
                        c.parent = Some(parent);
                    }
                    for b in app.doc.bodies.values_mut().filter(|b| b.component == Some(cid)) {
                        b.component = Some(parent);
                    }
                    app.doc.components.retain(|c| c.id != cid);
                    if app.active_comp == cid {
                        app.active_comp = parent;
                    }
                    app.commit(before);
                    ui.close();
                }
            });
        })
        .body(|ui| {
            // bodies
            let bodies: Vec<BodyId> = st.bodies.keys().copied().filter(|b| app.body_component(&st, *b) == cid).collect();
            if !bodies.is_empty() {
                let hid = ui.make_persistent_id(("bodies", cid));
                egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), hid, true)
                    .show_header(ui, |ui| {
                        ui.label(RichText::new(format!("{} Bodies", ic::FOLDER_SIMPLE)).color(theme::TEXT_DIM));
                    })
                    .body(|ui| {
                        for b in bodies {
                            ui.horizontal(|ui| {
                                let vis = app.doc.body_props(b).visible.unwrap_or(true);
                                if eye(ui, vis) {
                                    app.view_change(|d| d.bodies.entry(b).or_default().visible = Some(!vis));
                                }
                                let selected = app.selection.contains(&Pick::Body(b));
                                let is_mesh = st.bodies.get(&b).map(|x| x.is_mesh()).unwrap_or(false);
                                let icon = if is_mesh { ic::TRIANGLE } else { ic::CUBE };
                                let color = app.doc.body_props(b).color;
                                let r = ui.selectable_label(selected, format!("{icon} {}", app.body_name(b))).interact(egui::Sense::click_and_drag());
                                if r.drag_started() {
                                    r.dnd_set_drag_payload(b);
                                }
                                if r.dragged() {
                                    ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
                                }
                                if let Some(c) = color {
                                    let rc = r.rect;
                                    ui.painter().circle_filled(egui::pos2(rc.right() + 6.0, rc.center().y), 4.0, Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8));
                                }
                                if r.clicked() {
                                    app.selection = vec![Pick::Body(b)];
                                }
                                if r.double_clicked() {
                                    app.rename = Some((RenameTarget::Body(b), app.body_name(b)));
                                }
                                r.context_menu(|ui| body_menu(app, ui, b));
                            });
                        }
                    });
            }
            // sketches / planes
            let feats: Vec<Feature> = app
                .doc
                .features
                .iter()
                .filter(|f| f.component == cid && matches!(f.kind, FeatureKind::Sketch(_) | FeatureKind::Plane(_)))
                .cloned()
                .collect();
            if !feats.is_empty() {
                let hid = ui.make_persistent_id(("sketches", cid));
                egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), hid, true)
                    .show_header(ui, |ui| {
                        ui.label(RichText::new(format!("{} Sketches & planes", ic::FOLDER_SIMPLE)).color(theme::TEXT_DIM));
                    })
                    .body(|ui| {
                        for f in feats {
                            ui.horizontal(|ui| {
                                if eye(ui, f.visible) {
                                    let fid = f.id;
                                    app.view_change(|d| {
                                        if let Some(x) = d.feature_mut(fid) {
                                            x.visible = !x.visible;
                                        }
                                    });
                                }
                                let r = ui.selectable_label(app.timeline_sel == Some(f.id), format!("{} {}", feature_icon(&f.kind), f.name));
                                if r.clicked() {
                                    app.timeline_sel = Some(f.id);
                                }
                                if r.double_clicked() {
                                    commands::edit(app, f.id);
                                }
                                r.context_menu(|ui| {
                                    if ui.button("Edit").clicked() {
                                        commands::edit(app, f.id);
                                        ui.close();
                                    }
                                    if ui.button("Rename").clicked() {
                                        app.rename = Some((RenameTarget::Feature(f.id), f.name.clone()));
                                        ui.close();
                                    }
                                    if ui.button(RichText::new("Delete").color(theme::ERR)).clicked() {
                                        feature_action(app, f.id, "delete");
                                        ui.close();
                                    }
                                });
                            });
                        }
                    });
            }
            let children: Vec<CompId> = app.doc.children(cid).map(|c| c.id).collect();
            for c in children {
                component_tree(app, ui, c, depth + 1);
            }
        });
}

pub const PALETTE: &[(&str, Option<[f32; 3]>)] = &[
    ("Default", None),
    ("Steel blue", Some([0.45, 0.62, 0.85])),
    ("Orange", Some([0.95, 0.55, 0.20])),
    ("Green", Some([0.40, 0.75, 0.45])),
    ("Red", Some([0.85, 0.32, 0.30])),
    ("Yellow", Some([0.95, 0.82, 0.30])),
    ("Purple", Some([0.62, 0.45, 0.85])),
    ("Graphite", Some([0.30, 0.32, 0.36])),
    ("White", Some([0.95, 0.95, 0.95])),
];

fn body_menu(app: &mut App, ui: &mut egui::Ui, b: BodyId) {
    if ui.button(format!("{}  Rename", ic::TEXT_AA)).clicked() {
        app.rename = Some((RenameTarget::Body(b), app.body_name(b)));
        ui.close();
    }
    ui.menu_button(format!("{}  Colour", ic::PALETTE), |ui| {
        for (name, c) in PALETTE {
            if ui.button(*name).clicked() {
                let before = app.doc.clone();
                app.doc.bodies.entry(b).or_default().color = *c;
                app.commit(before);
                ui.close();
            }
        }
    });
    if ui.button(format!("{}  Export…", ic::EXPORT)).clicked() {
        app.export(Some(vec![b]));
        ui.close();
    }
    if ui.button(format!("{}  3D print (3MF)…", ic::PRINTER)).clicked() {
        crate::print::export_bodies_for_print(app, vec![b]);
        ui.close();
    }
    ui.menu_button(format!("{}  Move to component", ic::PUZZLE_PIECE), |ui| {
        let comps = app.doc.components.clone();
        for c in comps {
            if ui.button(&c.name).clicked() {
                let before = app.doc.clone();
                app.doc.bodies.entry(b).or_default().component = Some(c.id);
                app.commit(before);
                ui.close();
            }
        }
        ui.separator();
        if ui.button("New component").clicked() {
            let before = app.doc.clone();
            let name = format!("Component{}", app.doc.components.len());
            let c = app.doc.add_component(&name, app.active_comp);
            app.doc.bodies.entry(b).or_default().component = Some(c);
            app.commit(before);
            ui.close();
        }
    });
}

fn project(app: &mut App, ui: &mut egui::Ui) {
    let Some(dir) = app.project_dir.clone() else {
        ui.label("Open a folder to browse its designs and 3D files.");
        if ui.button(format!("{} Open folder…", ic::FOLDER_OPEN)).clicked() {
            if let Some(d) = rfd::FileDialog::new().pick_folder() {
                app.project_dir = Some(d);
            }
        }
        return;
    };
    ui.label(RichText::new(dir.display().to_string()).small().color(theme::TEXT_DIM));
    ui.horizontal(|ui| {
        if ui.small_button(format!("{} Change", ic::FOLDER_OPEN)).clicked() {
            if let Some(d) = rfd::FileDialog::new().set_directory(&dir).pick_folder() {
                app.project_dir = Some(d);
            }
        }
        if ui.small_button(format!("{} New design here", ic::FILE_PLUS)).clicked() {
            app.request(PendingAction::New);
        }
    });
    ui.separator();
    let exts = lusion_core::io::all_open_extensions();
    let mut entries: Vec<std::path::PathBuf> = std::fs::read_dir(&dir)
        .map(|rd| rd.filter_map(|e| e.ok()).map(|e| e.path()).collect())
        .unwrap_or_default();
    entries.sort();
    let mut dirs = vec![];
    let mut files = vec![];
    for p in entries {
        if p.is_dir() {
            if !p.file_name().map(|n| n.to_string_lossy().starts_with('.')).unwrap_or(true) {
                dirs.push(p);
            }
        } else if exts.contains(&lusion_core::io::ext_of(&p).as_str()) {
            files.push(p);
        }
    }
    if let Some(parent) = dir.parent() {
        if ui.selectable_label(false, format!("{} ..", ic::ARROW_ELBOW_LEFT_UP)).clicked() {
            app.project_dir = Some(parent.to_path_buf());
        }
    }
    for d in dirs {
        if ui.selectable_label(false, format!("{} {}", ic::FOLDER, d.file_name().unwrap().to_string_lossy())).clicked() {
            app.project_dir = Some(d);
        }
    }
    for f in files {
        let ext = lusion_core::io::ext_of(&f);
        let icon = if ext == "l360" { ic::CUBE } else if ext == "step" || ext == "stp" { ic::FILE_ARROW_DOWN } else { ic::TRIANGLE };
        let current = app.path.as_ref() == Some(&f);
        let r = ui.selectable_label(current, format!("{icon} {}", f.file_name().unwrap().to_string_lossy()));
        if r.clicked() && !current {
            app.request(PendingAction::Open(Some(f.clone())));
        }
        r.context_menu(|ui| {
            if ui.button("Open").clicked() {
                app.request(PendingAction::Open(Some(f.clone())));
                ui.close();
            }
            if ext != "l360" && ui.button("Insert into current design").clicked() {
                let before = app.doc.clone();
                match lusion_core::io::import_into(&mut app.doc, &f, app.active_comp) {
                    Ok(_) => app.commit(before),
                    Err(e) => {
                        app.doc = before;
                        app.error(format!("{e:#}"));
                    }
                }
                ui.close();
            }
        });
    }
}

// ------------------------------------------------------------------ dialogs

pub fn dialogs(app: &mut App, ctx: &egui::Context) {
    if let Some(p) = app.recovery.clone() {
        let mut done = false;
        egui::Window::new(format!("{} Recover unsaved work?", ic::LIFEBUOY)).collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
            let age = std::fs::metadata(&p).and_then(|m| m.modified()).ok().and_then(|t| t.elapsed().ok()).map(|d| d.as_secs() / 60);
            ui.label("Lusion360 found an autosaved design that was never saved (the app may have crashed).");
            if let Some(m) = age {
                ui.label(RichText::new(format!("Autosaved {m} min ago")).color(theme::TEXT_DIM));
            }
            ui.add_space(6.0);
            ui.horizontal(|ui| {
                if ui.button(RichText::new(format!("{} Recover", ic::ARROW_COUNTER_CLOCKWISE)).strong()).clicked() {
                    match lusion_core::io::load_native(&p) {
                        Ok(d) => {
                            app.new_doc();
                            app.doc = d;
                            let _ = std::fs::remove_file(&p);
                            app.fit_pending = true;
                            app.info("Recovered autosaved design — save it to keep it");
                        }
                        Err(e) => app.error(format!("Recovery failed: {e:#}")),
                    }
                    done = true;
                }
                if ui.button("Discard").clicked() {
                    let _ = std::fs::remove_file(&p);
                    done = true;
                }
            });
        });
        if done {
            app.recovery = None;
        }
    }
    // unsaved changes
    if let Some(action) = app.confirm_discard.clone() {
        let mut close = false;
        egui::Window::new("Unsaved changes").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0]).show(ctx, |ui| {
            ui.label("The design has unsaved changes.");
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button(format!("{} Save", ic::FLOPPY_DISK)).clicked() {
                    close = true;
                    if app.save(false) {
                        app.perform(action.clone());
                    }
                }
                if ui.button(RichText::new("Discard").color(theme::ERR)).clicked() {
                    close = true;
                    app.perform(action.clone());
                }
                if ui.button("Cancel").clicked() {
                    close = true;
                }
            });
        });
        if close {
            app.confirm_discard = None;
        }
    }
    // rename
    if let Some((target, mut name)) = app.rename.take() {
        let mut keep = true;
        let mut apply = false;
        egui::Window::new("Rename").collapsible(false).resizable(false).anchor(egui::Align2::CENTER_CENTER, [0.0, -100.0]).show(ctx, |ui| {
            let r = ui.text_edit_singleline(&mut name);
            r.request_focus();
            if r.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                apply = true;
            }
            ui.horizontal(|ui| {
                if ui.button("OK").clicked() {
                    apply = true;
                }
                if ui.button("Cancel").clicked() || ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    keep = false;
                }
            });
        });
        if apply && !name.trim().is_empty() {
            let before = app.doc.clone();
            match &target {
                RenameTarget::Feature(f) => {
                    if let Some(x) = app.doc.feature_mut(*f) {
                        x.name = name.trim().into();
                    }
                }
                RenameTarget::Component(c) => {
                    if let Some(x) = app.doc.component_mut(*c) {
                        x.name = name.trim().into();
                    }
                }
                RenameTarget::Body(b) => {
                    app.doc.bodies.entry(*b).or_default().name = Some(name.trim().into());
                }
            }
            app.commit(before);
        } else if keep && !apply {
            app.rename = Some((target, name));
        }
    }
    if app.show_params {
        params_window(app, ctx);
    }
    if app.palette.is_some() {
        palette(app, ctx);
    }
    if app.about {
        egui::Window::new("Keyboard & mouse").open(&mut app.about).collapsible(false).show(ctx, |ui| {
            egui::Grid::new("keys").striped(true).show(ui, |ui| {
                for (k, v) in [
                    ("Right-drag / Shift+Middle-drag", "Orbit (around the point under the cursor)"),
                    ("Middle-drag", "Pan"),
                    ("Wheel", "Zoom to cursor"),
                    ("Home / F6", "Home view"),
                    ("S", "Command search"),
                    ("F", "Fillet"),
                    ("Double-click profile/face", "Extrude"),
                    ("Ctrl+Z / Ctrl+Y", "Undo / redo"),
                    ("Ctrl+S / Ctrl+O / Ctrl+N", "Save / open / new"),
                    ("Ctrl+I / Ctrl+E", "Import / export"),
                    ("Sketch: L R C A P", "Line, rectangle, circle, arc, point"),
                    ("Sketch: D", "Dimension"),
                    ("Sketch: T", "Trim"),
                    ("Sketch: X", "Toggle construction"),
                    ("Sketch: Del", "Delete selection"),
                    ("Esc", "Cancel / back / finish sketch"),
                ] {
                    ui.label(RichText::new(k).strong());
                    ui.label(v);
                    ui.end_row();
                }
            });
        });
    }
}

fn params_window(app: &mut App, ctx: &egui::Context) {
    let mut open = true;
    let mut params = app.doc.parameters.clone();
    let mut remove: Option<usize> = None;
    egui::Window::new(format!("{} Parameters", ic::FUNCTION)).open(&mut open).default_width(520.0).default_pos(ctx.content_rect().center() - egui::vec2(260.0, 160.0)).show(ctx, |ui| {
        ui.label(RichText::new("Use parameter names in any dimension or feature value, e.g. `width / 2 + 3`.").small().color(theme::TEXT_DIM));
        ui.add_space(4.0);
        egui::Grid::new("params").num_columns(5).striped(true).spacing([8.0, 4.0]).show(ui, |ui| {
            ui.label(RichText::new("Name").strong());
            ui.label(RichText::new("Expression").strong());
            ui.label(RichText::new("Value").strong());
            ui.label(RichText::new("Comment").strong());
            ui.label("");
            ui.end_row();
            for (i, p) in params.iter_mut().enumerate() {
                ui.add_sized([100.0, 22.0], egui::TextEdit::singleline(&mut p.name));
                ui.add_sized([130.0, 22.0], egui::TextEdit::singleline(&mut p.expr));
                match (app.model.params.values.get(&p.name), app.model.params.errors.get(&p.name)) {
                    (_, Some(e)) => ui.label(RichText::new(ic::WARNING).color(theme::ERR)).on_hover_text(e),
                    (Some(v), _) => ui.label(fmt_num(*v)),
                    _ => ui.label("—"),
                };
                ui.add_sized([150.0, 22.0], egui::TextEdit::singleline(&mut p.comment));
                if ui.small_button(ic::TRASH).clicked() {
                    remove = Some(i);
                }
                ui.end_row();
            }
        });
        if ui.button(format!("{} Add parameter", ic::PLUS)).clicked() {
            let mut i = params.len() + 1;
            while params.iter().any(|p| p.name == format!("p{i}")) {
                i += 1;
            }
            params.push(Parameter { name: format!("p{i}"), expr: "10".into(), comment: String::new() });
        }
    });
    if let Some(i) = remove {
        params.remove(i);
    }
    if params != app.doc.parameters {
        if app.cmd.is_some() || matches!(app.mode, Mode::Sketch(_)) {
            // finish the current command/sketch first so the change is a clean undo step
            app.end_interaction();
        }
        let before = app.doc.clone();
        app.doc.parameters = params;
        app.commit(before);
    }
    app.show_params = open;
}

struct PaletteItem {
    label: &'static str,
    keys: &'static str,
    run: fn(&mut App),
}

const PALETTE_ITEMS: &[PaletteItem] = &[
    PaletteItem { label: "Sketch", keys: "new sketch create", run: |a| commands::start(a, CmdKind::NewSketch) },
    PaletteItem { label: "Extrude", keys: "pull push press", run: |a| commands::start(a, CmdKind::Extrude) },
    PaletteItem { label: "Revolve", keys: "lathe turn", run: |a| commands::start(a, CmdKind::Revolve) },
    PaletteItem { label: "Fillet", keys: "round", run: |a| commands::start(a, CmdKind::Fillet) },
    PaletteItem { label: "Chamfer", keys: "bevel", run: |a| commands::start(a, CmdKind::Chamfer) },
    PaletteItem { label: "Shell", keys: "hollow", run: |a| commands::start(a, CmdKind::Shell) },
    PaletteItem { label: "Offset plane", keys: "construction plane", run: |a| commands::start(a, CmdKind::Plane) },
    PaletteItem { label: "Combine bodies", keys: "boolean join cut", run: |a| commands::start(a, CmdKind::Combine) },
    PaletteItem { label: "Move / copy bodies", keys: "translate rotate", run: |a| commands::start(a, CmdKind::Move) },
    PaletteItem { label: "Pattern", keys: "array circular linear repeat copy", run: |a| commands::start(a, CmdKind::Pattern) },
    PaletteItem { label: "Mirror", keys: "symmetry flip", run: |a| commands::start(a, CmdKind::Mirror) },
    PaletteItem { label: "Parameters", keys: "variables", run: |a| a.show_params = true },
    PaletteItem { label: "Import file", keys: "insert step stl", run: |a| a.import_file() },
    PaletteItem { label: "Export", keys: "save step stl obj", run: |a| a.export(None) },
    PaletteItem { label: "Export for 3D printing", keys: "3mf bambu print slicer", run: crate::print::export_for_print },
    PaletteItem { label: "Save", keys: "", run: |a| { a.save(false); } },
    PaletteItem { label: "Open", keys: "load", run: |a| a.request(PendingAction::Open(None)) },
    PaletteItem { label: "New design", keys: "", run: |a| a.request(PendingAction::New) },
    PaletteItem { label: "Home view", keys: "camera", run: |a| a.home_view() },
    PaletteItem { label: "Fit all", keys: "zoom", run: |a| a.fit_all(true) },
    PaletteItem { label: "Toggle orthographic", keys: "perspective camera", run: |a| a.cam.cam.ortho = !a.cam.cam.ortho },
    PaletteItem { label: "Toggle light background", keys: "theme", run: |a| a.light_bg = !a.light_bg },
    PaletteItem { label: "New component", keys: "part", run: |a| {
        let before = a.doc.clone();
        let name = format!("Component{}", a.doc.components.len());
        let c = a.doc.add_component(&name, a.active_comp);
        a.active_comp = c;
        a.commit(before);
    } },
];

fn palette(app: &mut App, ctx: &egui::Context) {
    let Some((mut q, mut sel)) = app.palette.take() else { return };
    let ql = q.to_lowercase();
    let items: Vec<&PaletteItem> = PALETTE_ITEMS
        .iter()
        .filter(|it| ql.is_empty() || it.label.to_lowercase().contains(&ql) || it.keys.contains(&ql))
        .collect();
    let mut run: Option<fn(&mut App)> = None;
    let mut keep = true;
    egui::Window::new("command_palette").title_bar(false).resizable(false).fixed_size([380.0, 0.0]).anchor(egui::Align2::CENTER_TOP, [0.0, 90.0]).show(ctx, |ui| {
        let r = ui.add(egui::TextEdit::singleline(&mut q).hint_text(format!("{} Search commands…", ic::MAGNIFYING_GLASS)).desired_width(f32::INFINITY));
        r.request_focus();
        ui.input(|i| {
            if i.key_pressed(egui::Key::ArrowDown) {
                sel = (sel + 1).min(items.len().saturating_sub(1));
            }
            if i.key_pressed(egui::Key::ArrowUp) {
                sel = sel.saturating_sub(1);
            }
            if i.key_pressed(egui::Key::Escape) {
                keep = false;
            }
            if i.key_pressed(egui::Key::Enter) {
                if let Some(it) = items.get(sel) {
                    run = Some(it.run);
                }
                keep = false;
            }
        });
        ui.separator();
        for (i, it) in items.iter().enumerate() {
            if ui.selectable_label(i == sel, it.label).clicked() {
                run = Some(it.run);
                keep = false;
            }
        }
        if items.is_empty() {
            ui.label(RichText::new("No match").color(theme::TEXT_DIM));
        }
    });
    if keep {
        app.palette = Some((q, sel.min(items.len().saturating_sub(1))));
    }
    if let Some(f) = run {
        f(app);
    }
}

pub fn toasts(app: &mut App, ctx: &egui::Context) {
    let now = app.now();
    app.toasts.retain(|t| t.until > now);
    if app.toasts.is_empty() {
        return;
    }
    ctx.request_repaint_after(std::time::Duration::from_millis(100));
    egui::Area::new(egui::Id::new("toasts")).anchor(egui::Align2::RIGHT_BOTTOM, [-16.0, -80.0]).order(egui::Order::Tooltip).show(ctx, |ui| {
        for t in &app.toasts {
            let fade = ((t.until - now) / 0.4).clamp(0.0, 1.0) as f32;
            egui::Frame::new()
                .fill(Color32::from_rgba_unmultiplied(28, 32, 38, (235.0 * fade) as u8))
                .stroke(egui::Stroke::new(1.0, t.color.gamma_multiply(fade)))
                .corner_radius(6)
                .inner_margin(egui::Margin::symmetric(12, 8))
                .show(ui, |ui| {
                    ui.set_max_width(420.0);
                    ui.label(RichText::new(&t.text).color(Color32::from_rgba_unmultiplied(230, 235, 240, (255.0 * fade) as u8)));
                });
            ui.add_space(6.0);
        }
    });
}
