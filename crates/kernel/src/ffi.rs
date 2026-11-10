#[cxx::bridge(namespace = "lk")]
pub mod bridge {
    #[derive(Debug, Clone, Default)]
    struct MeshOut {
        positions: Vec<f32>,
        normals: Vec<f32>,
        indices: Vec<u32>,
        /// face index (MapShapes order) for every triangle
        tri_face: Vec<u32>,
        /// polyline points of all edges, flat xyz
        edge_points: Vec<f32>,
        /// `edge_points` vertex offsets; edge i = [edge_ranges[i], edge_ranges[i+1])
        edge_ranges: Vec<u32>,
        /// 1 if the edge joins two faces smoothly (tangent / nearly flat)
        edge_smooth: Vec<u8>,
    }

    #[derive(Debug, Clone, Copy, Default)]
    struct SurfaceInfo {
        /// 0 plane, 1 cylinder, 2 cone, 3 sphere, 4 torus, 5 other
        kind: u32,
        origin: [f64; 3],
        /// outward normal for planes; axis for revolved surfaces
        normal: [f64; 3],
        xdir: [f64; 3],
        radius: f64,
        area: f64,
        center: [f64; 3],
        reversed: bool,
    }

    #[derive(Debug, Clone, Copy, Default)]
    struct EdgeInfo {
        /// 0 line, 1 circle/arc, 2 other curve, 3 degenerated
        kind: u32,
        start: [f64; 3],
        end: [f64; 3],
        mid: [f64; 3],
        center: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        length: f64,
        closed: bool,
    }

    unsafe extern "C++" {
        include!("lusion-kernel/cpp/shim.h");

        type Shape;
        type Op;

        fn shape_null() -> UniquePtr<Shape>;
        fn shape_clone(s: &Shape) -> UniquePtr<Shape>;
        fn shape_deep_copy(s: &Shape) -> UniquePtr<Shape>;
        fn shape_is_null(s: &Shape) -> bool;
        fn shape_type(s: &Shape) -> u32;
        fn shape_hash(s: &Shape) -> u64;
        fn shape_is_same(a: &Shape, b: &Shape) -> bool;
        fn shape_subshapes(s: &Shape, kind: u32) -> UniquePtr<CxxVector<Shape>>;
        fn shape_vec_new() -> UniquePtr<CxxVector<Shape>>;
        fn shape_vec_push(v: Pin<&mut CxxVector<Shape>>, s: &Shape);
        fn make_compound(shapes: &CxxVector<Shape>) -> UniquePtr<Shape>;
        fn shape_bbox(s: &Shape) -> Vec<f64>;
        fn shape_volume(s: &Shape) -> Result<f64>;
        fn shape_area(s: &Shape) -> Result<f64>;
        fn shape_transform(s: &Shape, m: &[f64]) -> Result<UniquePtr<Shape>>;
        fn shape_edge_faces(s: &Shape) -> Vec<i32>;
        fn shape_is_valid(s: &Shape) -> bool;
        fn shape_fix(s: &Shape) -> Result<UniquePtr<Shape>>;

        fn face_info(f: &Shape) -> Result<SurfaceInfo>;
        fn edge_info(e: &Shape) -> Result<EdgeInfo>;
        fn shape_mesh(s: &Shape, lin_defl: f64, ang_defl: f64) -> Result<MeshOut>;
        fn edge_polyline(e: &Shape, lin_defl: f64, ang_defl: f64) -> Vec<f64>;
        fn face_loops(f: &Shape, lin_defl: f64, ang_defl: f64, sizes: &mut Vec<u32>) -> Vec<f64>;
        fn point_distance(s: &Shape, x: f64, y: f64, z: f64) -> f64;

        fn make_line_edge(ax: f64, ay: f64, az: f64, bx: f64, by: f64, bz: f64) -> Result<UniquePtr<Shape>>;
        fn make_circle_edge(cx: f64, cy: f64, cz: f64, nx: f64, ny: f64, nz: f64, r: f64) -> Result<UniquePtr<Shape>>;
        #[allow(clippy::too_many_arguments)]
        fn make_arc_edge(
            cx: f64,
            cy: f64,
            cz: f64,
            nx: f64,
            ny: f64,
            nz: f64,
            sx: f64,
            sy: f64,
            sz: f64,
            ex: f64,
            ey: f64,
            ez: f64,
        ) -> Result<UniquePtr<Shape>>;
        fn make_planar_regions(edges: &CxxVector<Shape>) -> Result<UniquePtr<Shape>>;
        fn make_plane_face(ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64, half: f64) -> Result<UniquePtr<Shape>>;

        fn make_prism(profile: &Shape, dx: f64, dy: f64, dz: f64) -> Result<UniquePtr<Op>>;
        #[allow(clippy::too_many_arguments)]
        fn make_revol(profile: &Shape, ox: f64, oy: f64, oz: f64, ax: f64, ay: f64, az: f64, angle: f64) -> Result<UniquePtr<Op>>;
        fn make_boolean(kind: u32, args: &CxxVector<Shape>, tools: &CxxVector<Shape>, simplify: bool) -> Result<UniquePtr<Op>>;
        fn make_fillet(solid: &Shape, edges: &CxxVector<Shape>, radius: f64) -> Result<UniquePtr<Op>>;
        fn make_chamfer(solid: &Shape, edges: &CxxVector<Shape>, dist: f64) -> Result<UniquePtr<Op>>;
        fn make_shell(solid: &Shape, faces: &CxxVector<Shape>, thickness: f64) -> Result<UniquePtr<Op>>;
        fn make_transform(s: &Shape, m: &[f64]) -> Result<UniquePtr<Op>>;
        fn make_mirror(s: &Shape, ox: f64, oy: f64, oz: f64, nx: f64, ny: f64, nz: f64) -> Result<UniquePtr<Op>>;
        fn op_shape(op: &Op) -> UniquePtr<Shape>;
        fn op_modified(op: &Op, s: &Shape) -> UniquePtr<CxxVector<Shape>>;
        fn op_generated(op: &Op, s: &Shape) -> UniquePtr<CxxVector<Shape>>;
        fn op_is_deleted(op: &Op, s: &Shape) -> bool;
        fn op_first(op: &Op) -> UniquePtr<Shape>;
        fn op_last(op: &Op) -> UniquePtr<Shape>;

        fn read_step(path: &str) -> Result<UniquePtr<Shape>>;
        fn write_step(s: &Shape, path: &str) -> Result<()>;
        fn read_brep_file(path: &str) -> Result<UniquePtr<Shape>>;
        fn write_brep_string(s: &Shape) -> Result<String>;
        fn read_brep_string(data: &str) -> Result<UniquePtr<Shape>>;
    }
}

// SAFETY: OCCT shapes are only ever moved between threads, never shared
// concurrently (we don't implement Sync).
unsafe impl Send for bridge::Shape {}
unsafe impl Send for bridge::Op {}
