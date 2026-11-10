//! wgpu renderer for the 3D viewport, drawn inside egui via a paint callback.

use bytemuck::{Pod, Zeroable};
use eframe::egui_wgpu::{self, wgpu};
use glam::{Mat4, Vec3};
use lusion_core::tess::BodyMesh;
use std::collections::HashMap;
use std::ops::Range;
use std::sync::Arc;
use wgpu::util::DeviceExt;

pub const MSAA: u32 = 4;
pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth24Plus;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    light: [f32; 4],
    viewport: [f32; 4],
    fwd: [f32; 4],
    bg_top: [f32; 4],
    bg_bottom: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Default)]
struct DrawU {
    color: [f32; 4],
    params: [f32; 4],
    _pad: [[f32; 4]; 14],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Debug, Default)]
pub struct LineSeg {
    pub p0: [f32; 3],
    pub p1: [f32; 3],
    pub color: [f32; 4],
    pub width: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Debug, Default)]
pub struct PointInst {
    pub p: [f32; 3],
    pub color: [f32; 4],
    pub size: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Debug, Default)]
pub struct FillVert {
    pub p: [f32; 3],
    pub color: [f32; 4],
}

/// One body to draw this frame.
pub struct BodyDraw {
    pub mesh: Arc<BodyMesh>,
    pub color: [f32; 4],
    pub edge_color: [f32; 4],
    pub show_edges: bool,
    /// (face index, color)
    pub face_hl: Vec<(u32, [f32; 4])>,
    /// (edge index, color)
    pub edge_hl: Vec<(u32, [f32; 4])>,
}

/// Everything the viewport shows in one frame.
#[derive(Default)]
pub struct Scene {
    /// viewport size in physical pixels
    pub viewport_px: [f32; 2],
    pub view_proj: Mat4,
    pub eye: Vec3,
    pub fwd: Vec3,
    /// camera distance (for depth bias in ortho)
    pub cam_dist: f32,
    pub light: Vec3,
    pub ortho: bool,
    pub bg_top: [f32; 3],
    pub bg_bottom: [f32; 3],
    pub bodies: Vec<BodyDraw>,
    /// translucent fills (profiles, plane glyphs)
    pub fills: Vec<FillVert>,
    /// depth-tested lines
    pub lines: Vec<LineSeg>,
    /// lines drawn on top of everything
    pub overlay_lines: Vec<LineSeg>,
    pub points: Vec<PointInst>,
    /// an animation is running (ask egui for another frame)
    pub wants_repaint: bool,
}

struct GpuMesh {
    vbuf: wgpu::Buffer,
    ibuf: wgpu::Buffer,
    n_idx: u32,
    /// index range per face
    face_ranges: Vec<Range<u32>>,
    edges: Option<wgpu::Buffer>,
    n_edges: u32,
    edge_ranges: Vec<Range<u32>>,
    last_used: u64,
}

struct DynBuf {
    buf: wgpu::Buffer,
    cap: u64,
    usage: wgpu::BufferUsages,
    label: &'static str,
}

impl DynBuf {
    fn new(device: &wgpu::Device, usage: wgpu::BufferUsages, label: &'static str) -> Self {
        let cap = 1 << 16;
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size: cap,
            usage: usage | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        DynBuf { buf, cap, usage, label }
    }
    fn write(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let need = data.len() as u64;
        if need > self.cap {
            let mut cap = self.cap;
            while cap < need {
                cap *= 2;
            }
            self.buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(self.label),
                size: cap,
                usage: self.usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            self.cap = cap;
        }
        queue.write_buffer(&self.buf, 0, data);
    }
}

pub struct Renderer {
    bg: wgpu::RenderPipeline,
    mesh: wgpu::RenderPipeline,
    mesh_overlay: wgpu::RenderPipeline,
    outline: wgpu::RenderPipeline,
    fill: wgpu::RenderPipeline,
    line: wgpu::RenderPipeline,
    line_top: wgpu::RenderPipeline,
    point: wgpu::RenderPipeline,
    globals: wgpu::Buffer,
    globals_bg: wgpu::BindGroup,
    draw_bgl: wgpu::BindGroupLayout,
    draws: wgpu::Buffer,
    draws_bg: wgpu::BindGroup,
    draws_cap: u64,
    meshes: HashMap<usize, GpuMesh>,
    lines: DynBuf,
    overlay: DynBuf,
    points: DynBuf,
    fills: DynBuf,
    frame: u64,
    srgb_target: bool,
    /// draw list prepared for paint()
    plan: Vec<Cmd>,
}

enum Cmd {
    Bg,
    Mesh { key: usize, draw: u32, range: Option<Range<u32>>, overlay: bool },
    Outline { key: usize, draw: u32 },
    Edges { key: usize, draw: u32, range: Range<u32> },
    Fills { draw: u32, n: u32 },
    Lines { draw: u32, n: u32 },
    Overlay { draw: u32, n: u32 },
    Points { draw: u32, n: u32 },
}

const DRAW_STRIDE: u64 = std::mem::size_of::<DrawU>() as u64; // 256

impl Renderer {
    pub fn new(rs: &egui_wgpu::RenderState) -> Self {
        let device = &rs.device;
        let target = rs.target_format;
        let srgb_target = target.is_srgb();
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("lusion shaders"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders.wgsl").into()),
        });
        let uniform_entry = |dynamic: bool| wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: dynamic,
                min_binding_size: None,
            },
            count: None,
        };
        let globals_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals"),
            entries: &[uniform_entry(false)],
        });
        let draw_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("draw"),
            entries: &[uniform_entry(true)],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("lusion layout"),
            bind_group_layouts: &[Some(&globals_bgl), Some(&draw_bgl)],
            immediate_size: 0,
        });
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("globals"),
            size: std::mem::size_of::<Globals>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let globals_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("globals"),
            layout: &globals_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() }],
        });
        let draws_cap = 256;
        let (draws, draws_bg) = Self::make_draws(device, &draw_bgl, draws_cap);

        let blend = Some(wgpu::BlendState::ALPHA_BLENDING);
        let mk = |label: &str,
                  vs: &str,
                  fs: &str,
                  buffers: &[Option<wgpu::VertexBufferLayout>],
                  topology: wgpu::PrimitiveTopology,
                  depth_write: bool,
                  compare: wgpu::CompareFunction| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers,
                },
                primitive: wgpu::PrimitiveState { topology, cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH_FORMAT,
                    depth_write_enabled: Some(depth_write),
                    depth_compare: Some(compare),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: MSAA, mask: !0, alpha_to_coverage_enabled: false },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: target, blend, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let mesh_layout = wgpu::VertexBufferLayout {
            array_stride: 24,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3],
        };
        let fill_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<FillVert>() as u64,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
        };
        let line_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<LineSeg>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x4, 3 => Float32],
        };
        let point_layout = wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<PointInst>() as u64,
            step_mode: wgpu::VertexStepMode::Instance,
            attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4, 2 => Float32],
        };
        use wgpu::CompareFunction as C;
        use wgpu::PrimitiveTopology as T;
        let bg = mk("bg", "vs_bg", "fs_bg", &[], T::TriangleList, false, C::Always);
        let mesh = mk("mesh", "vs_mesh", "fs_mesh", &[Some(mesh_layout.clone())], T::TriangleList, true, C::Less);
        let mesh_overlay =
            mk("mesh overlay", "vs_mesh", "fs_mesh", &[Some(mesh_layout.clone())], T::TriangleList, false, C::LessEqual);
        let outline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("outline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState { module: &shader, entry_point: Some("vs_outline"), compilation_options: Default::default(), buffers: &[Some(mesh_layout)] },
            primitive: wgpu::PrimitiveState { topology: T::TriangleList, cull_mode: Some(wgpu::Face::Front), ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: DEPTH_FORMAT,
                depth_write_enabled: Some(true),
                depth_compare: Some(C::Less),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState { count: MSAA, mask: !0, alpha_to_coverage_enabled: false },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_fill"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState { format: target, blend, write_mask: wgpu::ColorWrites::ALL })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let fill = mk("fill", "vs_fill", "fs_fill", &[Some(fill_layout)], T::TriangleList, false, C::LessEqual);
        let line = mk("line", "vs_line", "fs_line", &[Some(line_layout.clone())], T::TriangleList, false, C::LessEqual);
        let line_top = mk("line top", "vs_line", "fs_line", &[Some(line_layout)], T::TriangleList, false, C::Always);
        let point = mk("point", "vs_point", "fs_point", &[Some(point_layout)], T::TriangleList, false, C::Always);

        let vb = wgpu::BufferUsages::VERTEX;
        Renderer {
            bg,
            mesh,
            mesh_overlay,
            outline,
            fill,
            line,
            line_top,
            point,
            globals,
            globals_bg,
            draw_bgl,
            draws,
            draws_bg,
            draws_cap,
            meshes: HashMap::new(),
            lines: DynBuf::new(device, vb, "lines"),
            overlay: DynBuf::new(device, vb, "overlay lines"),
            points: DynBuf::new(device, vb, "points"),
            fills: DynBuf::new(device, vb, "fills"),
            frame: 0,
            srgb_target,
            plan: vec![],
        }
    }

    fn make_draws(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, cap: u64) -> (wgpu::Buffer, wgpu::BindGroup) {
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("draws"),
            size: cap * DRAW_STRIDE,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("draws"),
            layout: bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &buf,
                    offset: 0,
                    size: std::num::NonZeroU64::new(DRAW_STRIDE),
                }),
            }],
        });
        (buf, bg)
    }

    fn upload_mesh(device: &wgpu::Device, m: &BodyMesh, frame: u64) -> GpuMesh {
        let mut verts: Vec<f32> = Vec::with_capacity(m.positions.len() * 6);
        for (p, n) in m.positions.iter().zip(m.normals.iter()) {
            verts.extend_from_slice(p);
            verts.extend_from_slice(n);
        }
        let vbuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("body vb"),
            contents: if verts.is_empty() { &[0u8; 24] } else { bytemuck::cast_slice(&verts) },
            usage: wgpu::BufferUsages::VERTEX,
        });
        let ibuf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("body ib"),
            contents: if m.indices.is_empty() { &[0u8; 12] } else { bytemuck::cast_slice(&m.indices) },
            usage: wgpu::BufferUsages::INDEX,
        });
        // contiguous triangle runs per face
        let mut face_ranges: Vec<Range<u32>> = Vec::new();
        let nf = m.tri_face.iter().copied().max().map(|v| v as usize + 1).unwrap_or(0);
        face_ranges.resize(nf, 0..0);
        let mut i = 0;
        while i < m.tri_face.len() {
            let f = m.tri_face[i];
            let mut j = i;
            while j < m.tri_face.len() && m.tri_face[j] == f {
                j += 1;
            }
            face_ranges[f as usize] = (i as u32 * 3)..(j as u32 * 3);
            i = j;
        }
        let mut segs: Vec<LineSeg> = Vec::new();
        let mut edge_ranges = Vec::with_capacity(m.edges.len());
        for (i, e) in m.edges.iter().enumerate() {
            let s = segs.len() as u32;
            let smooth = m.edge_smooth.get(i).copied().unwrap_or(false);
            let (color, width) = if smooth { ([1.0, 1.0, 1.0, 0.16], 0.8) } else { ([1.0; 4], 1.0) };
            for w in e.windows(2) {
                segs.push(LineSeg { p0: w[0], p1: w[1], color, width });
            }
            edge_ranges.push(s..segs.len() as u32);
        }
        let edges = (!segs.is_empty()).then(|| {
            device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("body edges"),
                contents: bytemuck::cast_slice(&segs),
                usage: wgpu::BufferUsages::VERTEX,
            })
        });
        GpuMesh {
            vbuf,
            ibuf,
            n_idx: m.indices.len() as u32,
            face_ranges,
            edges,
            n_edges: segs.len() as u32,
            edge_ranges,
            last_used: frame,
        }
    }

    fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, scene: &Scene, vp_px: [f32; 2]) {
        self.frame += 1;
        let frame = self.frame;
        let g = Globals {
            view_proj: scene.view_proj.to_cols_array_2d(),
            eye: scene.eye.extend(1.0).to_array(),
            light: scene.light.extend(0.0).to_array(),
            viewport: [vp_px[0], vp_px[1], if self.srgb_target { 0.0 } else { 1.0 }, if scene.ortho { 1.0 } else { 0.0 }],
            fwd: scene.fwd.extend(scene.cam_dist).to_array(),
            bg_top: [scene.bg_top[0], scene.bg_top[1], scene.bg_top[2], 1.0],
            bg_bottom: [scene.bg_bottom[0], scene.bg_bottom[1], scene.bg_bottom[2], 1.0],
        };
        queue.write_buffer(&self.globals, 0, bytemuck::bytes_of(&g));

        let mut draws: Vec<DrawU> = Vec::new();
        let mut push = |color: [f32; 4], params: [f32; 4]| -> u32 {
            draws.push(DrawU { color, params, ..Default::default() });
            (draws.len() - 1) as u32
        };
        let mut plan = vec![Cmd::Bg];
        // upload meshes
        for b in &scene.bodies {
            let key = Arc::as_ptr(&b.mesh) as usize;
            let gm = self.meshes.entry(key).or_insert_with(|| Self::upload_mesh(device, &b.mesh, frame));
            gm.last_used = frame;
        }
        // opaque bodies
        for b in &scene.bodies {
            let key = Arc::as_ptr(&b.mesh) as usize;
            if b.color[3] >= 0.999 {
                if b.show_edges {
                    let o = push(b.edge_color, [1.3, 0.0, 0.0, 0.0]);
                    plan.push(Cmd::Outline { key, draw: o });
                }
                let d = push(b.color, [1.0, 0.0, 1.0, 0.0]);
                plan.push(Cmd::Mesh { key, draw: d, range: None, overlay: false });
            }
        }
        // face highlights
        for b in &scene.bodies {
            let key = Arc::as_ptr(&b.mesh) as usize;
            let gm = &self.meshes[&key];
            for (f, c) in &b.face_hl {
                if let Some(r) = gm.face_ranges.get(*f as usize) {
                    if !r.is_empty() {
                        let d = push(*c, [1.0, 0.0008, 0.35, 0.0]);
                        plan.push(Cmd::Mesh { key, draw: d, range: Some(r.clone()), overlay: true });
                    }
                }
            }
        }
        // edges
        for b in &scene.bodies {
            let key = Arc::as_ptr(&b.mesh) as usize;
            let gm = &self.meshes[&key];
            if b.show_edges && gm.n_edges > 0 {
                let d = push(b.edge_color, [1.4, 0.0015, 0.0, 0.0]);
                plan.push(Cmd::Edges { key, draw: d, range: 0..gm.n_edges });
            }
            for (e, c) in &b.edge_hl {
                if let Some(r) = gm.edge_ranges.get(*e as usize) {
                    if !r.is_empty() {
                        let d = push(*c, [3.5, 0.002, 0.0, 0.0]);
                        plan.push(Cmd::Edges { key, draw: d, range: r.clone() });
                    }
                }
            }
        }
        // translucent bodies (ghosted)
        for b in &scene.bodies {
            let key = Arc::as_ptr(&b.mesh) as usize;
            if b.color[3] < 0.999 {
                let d = push(b.color, [1.0, 0.0, 1.0, 0.0]);
                plan.push(Cmd::Mesh { key, draw: d, range: None, overlay: true });
            }
        }
        if !scene.fills.is_empty() {
            self.fills.write(device, queue, bytemuck::cast_slice(&scene.fills));
            let d = push([1.0; 4], [1.0, 0.001, 0.0, 0.0]);
            plan.push(Cmd::Fills { draw: d, n: scene.fills.len() as u32 });
        }
        if !scene.lines.is_empty() {
            self.lines.write(device, queue, bytemuck::cast_slice(&scene.lines));
            let d = push([1.0; 4], [1.0, 0.002, 0.0, 0.0]);
            plan.push(Cmd::Lines { draw: d, n: scene.lines.len() as u32 });
        }
        if !scene.overlay_lines.is_empty() {
            self.overlay.write(device, queue, bytemuck::cast_slice(&scene.overlay_lines));
            let d = push([1.0; 4], [1.0, 0.0, 0.0, 0.0]);
            plan.push(Cmd::Overlay { draw: d, n: scene.overlay_lines.len() as u32 });
        }
        if !scene.points.is_empty() {
            self.points.write(device, queue, bytemuck::cast_slice(&scene.points));
            let d = push([1.0; 4], [1.0, 0.0, 0.0, 0.0]);
            plan.push(Cmd::Points { draw: d, n: scene.points.len() as u32 });
        }
        if draws.len() as u64 > self.draws_cap {
            let mut cap = self.draws_cap;
            while cap < draws.len() as u64 {
                cap *= 2;
            }
            let (b, bg) = Self::make_draws(device, &self.draw_bgl, cap);
            self.draws = b;
            self.draws_bg = bg;
            self.draws_cap = cap;
        }
        if !draws.is_empty() {
            queue.write_buffer(&self.draws, 0, bytemuck::cast_slice(&draws));
        }
        self.plan = plan;
        // evict meshes not used for a while
        if self.meshes.len() > 32 {
            self.meshes.retain(|_, m| frame - m.last_used < 120);
        }
    }

    fn paint(&self, rp: &mut wgpu::RenderPass<'static>) {
        rp.set_bind_group(0, &self.globals_bg, &[]);
        for cmd in &self.plan {
            match cmd {
                Cmd::Bg => {
                    rp.set_pipeline(&self.bg);
                    rp.set_bind_group(1, &self.draws_bg, &[0]);
                    rp.draw(0..3, 0..1);
                }
                Cmd::Mesh { key, draw, range, overlay } => {
                    let Some(m) = self.meshes.get(key) else { continue };
                    if m.n_idx == 0 {
                        continue;
                    }
                    rp.set_pipeline(if *overlay { &self.mesh_overlay } else { &self.mesh });
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, m.vbuf.slice(..));
                    rp.set_index_buffer(m.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                    rp.draw_indexed(range.clone().unwrap_or(0..m.n_idx), 0, 0..1);
                }
                Cmd::Outline { key, draw } => {
                    let Some(m) = self.meshes.get(key) else { continue };
                    if m.n_idx == 0 {
                        continue;
                    }
                    rp.set_pipeline(&self.outline);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, m.vbuf.slice(..));
                    rp.set_index_buffer(m.ibuf.slice(..), wgpu::IndexFormat::Uint32);
                    rp.draw_indexed(0..m.n_idx, 0, 0..1);
                }
                Cmd::Edges { key, draw, range } => {
                    let Some(m) = self.meshes.get(key) else { continue };
                    let Some(eb) = &m.edges else { continue };
                    rp.set_pipeline(&self.line);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, eb.slice(..));
                    rp.draw(0..6, range.clone());
                }
                Cmd::Fills { draw, n } => {
                    rp.set_pipeline(&self.fill);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, self.fills.buf.slice(..));
                    rp.draw(0..*n, 0..1);
                }
                Cmd::Lines { draw, n } => {
                    rp.set_pipeline(&self.line);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, self.lines.buf.slice(..));
                    rp.draw(0..6, 0..*n);
                }
                Cmd::Overlay { draw, n } => {
                    rp.set_pipeline(&self.line_top);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, self.overlay.buf.slice(..));
                    rp.draw(0..6, 0..*n);
                }
                Cmd::Points { draw, n } => {
                    rp.set_pipeline(&self.point);
                    rp.set_bind_group(1, &self.draws_bg, &[(*draw as u64 * DRAW_STRIDE) as u32]);
                    rp.set_vertex_buffer(0, self.points.buf.slice(..));
                    rp.draw(0..6, 0..*n);
                }
            }
        }
    }
}

/// Paint callback carrying one frame's scene.
pub struct ViewportCallback {
    pub scene: Arc<Scene>,
}

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen: &egui_wgpu::ScreenDescriptor,
        _encoder: &mut wgpu::CommandEncoder,
        res: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let _ = screen;
        if let Some(r) = res.get_mut::<Renderer>() {
            r.prepare(device, queue, &self.scene, self.scene.viewport_px);
        }
        Vec::new()
    }

    fn paint(&self, _info: egui::PaintCallbackInfo, rp: &mut wgpu::RenderPass<'static>, res: &egui_wgpu::CallbackResources) {
        if let Some(r) = res.get::<Renderer>() {
            r.paint(rp);
        }
    }
}

