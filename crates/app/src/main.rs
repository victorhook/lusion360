//! Lusion360 — a small, fast parametric CAD modeller.

mod app;
mod camera;
mod commands;
#[cfg(target_os = "linux")]
mod dragout;
mod dragdrop;
#[allow(dead_code)]
mod icons;
mod panels;
mod pick;
mod print;
mod render;
mod sketch_mode;
mod theme;
mod viewport;

fn main() -> eframe::Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn")).init();
    app::install_panic_hook();
    let initial = std::env::args().nth(1).map(std::path::PathBuf::from);
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("Lusion360")
            .with_app_id("lusion360")
            .with_inner_size([1480.0, 920.0])
            .with_min_inner_size([900.0, 600.0])
            .with_drag_and_drop(true)
            .with_icon(egui::IconData { rgba: include_bytes!("../assets/icon128.rgba").to_vec(), width: 128, height: 128 }),
        multisampling: render::MSAA as u16,
        depth_buffer: 24,
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native("Lusion360", options, Box::new(move |cc| Ok(Box::new(app::App::new(cc, initial)))))
}
