//! Colours and egui styling.

use egui::{Color32, CornerRadius, Stroke, Visuals};

pub const ACCENT: Color32 = Color32::from_rgb(0x3d, 0x9b, 0xff);
pub const ACCENT_DIM: Color32 = Color32::from_rgb(0x2a, 0x5f, 0x9e);
pub const WARN: Color32 = Color32::from_rgb(0xf0, 0xb4, 0x29);
pub const ERR: Color32 = Color32::from_rgb(0xf2, 0x55, 0x55);
pub const OK: Color32 = Color32::from_rgb(0x4c, 0xc3, 0x7a);
pub const PANEL: Color32 = Color32::from_rgb(0x1e, 0x21, 0x26);
pub const PANEL2: Color32 = Color32::from_rgb(0x25, 0x29, 0x2f);
pub const TEXT_DIM: Color32 = Color32::from_rgb(0x8a, 0x92, 0x9c);

// 3D colours (sRGB, 0..1)
pub const BODY: [f32; 4] = [0.78, 0.80, 0.83, 1.0];
pub const BODY_EDGE: [f32; 4] = [0.08, 0.09, 0.11, 1.0];
pub const MESH_BODY: [f32; 4] = [0.70, 0.74, 0.80, 1.0];
pub const HOVER: [f32; 4] = [0.33, 0.68, 1.0, 0.55];
pub const HOVER_EDGE: [f32; 4] = [0.30, 0.70, 1.0, 1.0];
pub const SELECT: [f32; 4] = [0.13, 0.55, 1.0, 0.70];
pub const SELECT_EDGE: [f32; 4] = [0.15, 0.55, 1.0, 1.0];
pub const PROFILE: [f32; 4] = [0.45, 0.62, 0.85, 0.22];
pub const PROFILE_HOVER: [f32; 4] = [0.40, 0.70, 1.0, 0.45];
pub const PROFILE_SEL: [f32; 4] = [0.20, 0.55, 1.0, 0.60];
pub const SK_FREE: [f32; 4] = [0.40, 0.70, 1.0, 1.0];
pub const SK_FIXED: [f32; 4] = [0.92, 0.94, 0.96, 1.0];
pub const SK_CONSTR: [f32; 4] = [0.95, 0.60, 0.25, 0.85];
pub const SK_SEL: [f32; 4] = [0.20, 0.95, 0.75, 1.0];
pub const SK_HOVER: [f32; 4] = [1.0, 0.85, 0.30, 1.0];
pub const SK_INACTIVE: [f32; 4] = [0.55, 0.62, 0.72, 0.9];
pub const SK_ERR: [f32; 4] = [1.0, 0.35, 0.35, 1.0];
pub const BG_TOP: [f32; 3] = [0.20, 0.23, 0.27];
pub const BG_BOTTOM: [f32; 3] = [0.09, 0.10, 0.12];
pub const BG_TOP_LIGHT: [f32; 3] = [0.86, 0.89, 0.93];
pub const BG_BOTTOM_LIGHT: [f32; 3] = [0.62, 0.66, 0.72];

pub fn c32(c: [f32; 4]) -> Color32 {
    Color32::from_rgba_unmultiplied((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8, (c[3] * 255.0) as u8)
}

pub fn apply(ctx: &egui::Context) {
    let mut v = Visuals::dark();
    v.panel_fill = PANEL;
    v.window_fill = PANEL2;
    v.extreme_bg_color = Color32::from_rgb(0x15, 0x17, 0x1b);
    v.faint_bg_color = Color32::from_rgb(0x23, 0x27, 0x2d);
    v.selection.bg_fill = ACCENT_DIM;
    v.selection.stroke = Stroke::new(1.0, ACCENT);
    v.hyperlink_color = ACCENT;
    v.window_corner_radius = CornerRadius::same(8);
    v.menu_corner_radius = CornerRadius::same(6);
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, Color32::from_rgb(0x30, 0x35, 0x3c));
    v.widgets.inactive.corner_radius = CornerRadius::same(5);
    v.widgets.hovered.corner_radius = CornerRadius::same(5);
    v.widgets.active.corner_radius = CornerRadius::same(5);
    v.widgets.inactive.weak_bg_fill = Color32::from_rgb(0x2c, 0x31, 0x38);
    v.widgets.hovered.weak_bg_fill = Color32::from_rgb(0x37, 0x3e, 0x47);
    v.window_shadow = egui::epaint::Shadow { offset: [0, 4], blur: 16, spread: 0, color: Color32::from_black_alpha(90) };
    ctx.set_visuals(v);
    ctx.global_style_mut(|s| {
        s.spacing.item_spacing = egui::vec2(6.0, 5.0);
        s.spacing.button_padding = egui::vec2(7.0, 4.0);
        s.spacing.interact_size.y = 22.0;
    });
}

pub fn fonts() -> egui::FontDefinitions {
    let mut f = egui::FontDefinitions::default();
    f.font_data.insert(
        "phosphor".into(),
        std::sync::Arc::new(egui::FontData::from_static(include_bytes!("../assets/Phosphor.ttf"))),
    );
    if let Some(p) = f.families.get_mut(&egui::FontFamily::Proportional) {
        p.insert(1.min(p.len()), "phosphor".into());
    }
    if let Some(p) = f.families.get_mut(&egui::FontFamily::Monospace) {
        p.push("phosphor".into());
    }
    f
}
