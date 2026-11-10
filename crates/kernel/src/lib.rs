//! Safe Rust wrapper around the OpenCASCADE geometry kernel.
//!
//! The kernel is intentionally small: it exposes exactly the B-rep operations
//! the modeller needs (sketch edges, planar regions, extrude, revolve,
//! booleans, fillet/chamfer/shell, tessellation, STEP/BREP IO) together with
//! operation *history* so that the document layer can keep persistent names
//! for faces and edges across recomputes.

mod ffi;

use cxx::UniquePtr;
use ffi::bridge as sys;
use glam::{DMat4, DVec3};

pub use sys::{EdgeInfo as RawEdgeInfo, MeshOut, SurfaceInfo as RawSurfaceInfo};

#[derive(Debug, Clone, thiserror::Error)]
#[error("{0}")]
pub struct KernelError(pub String);

impl From<cxx::Exception> for KernelError {
    fn from(e: cxx::Exception) -> Self {
        KernelError(e.what().to_string())
    }
}

pub type KResult<T> = Result<T, KernelError>;

fn err<T>(msg: impl Into<String>) -> KResult<T> {
    Err(KernelError(msg.into()))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeKind {
    Compound = 0,
    CompSolid = 1,
    Solid = 2,
    Shell = 3,
    Face = 4,
    Wire = 5,
    Edge = 6,
    Vertex = 7,
    Other = 8,
    Null = 99,
}

impl ShapeKind {
    fn from_u32(v: u32) -> Self {
        match v {
            0 => Self::Compound,
            1 => Self::CompSolid,
            2 => Self::Solid,
            3 => Self::Shell,
            4 => Self::Face,
            5 => Self::Wire,
            6 => Self::Edge,
            7 => Self::Vertex,
            99 => Self::Null,
            _ => Self::Other,
        }
    }
}

/// A topological shape (solid, face, edge, compound, ...). Cheap to clone:
/// OCCT shapes are reference-counted handles.
pub struct Shape {
    inner: UniquePtr<sys::Shape>,
}

// SAFETY: see ffi.rs — handles are moved, not shared.
unsafe impl Send for Shape {}

impl Clone for Shape {
    fn clone(&self) -> Self {
        Shape { inner: sys::shape_clone(self.raw()) }
    }
}

impl std::fmt::Debug for Shape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Shape({:?}#{:x})", self.kind(), self.hash())
    }
}

impl Default for Shape {
    fn default() -> Self {
        Self::null()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceKind {
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Other,
}

#[derive(Debug, Clone, Copy)]
pub struct FaceInfo {
    pub kind: SurfaceKind,
    pub origin: DVec3,
    /// Outward normal for planes, axis for revolved surfaces.
    pub normal: DVec3,
    pub xdir: DVec3,
    pub radius: f64,
    pub area: f64,
    pub center: DVec3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CurveKind {
    Line,
    Circle,
    Other,
    Degenerated,
}

#[derive(Debug, Clone, Copy)]
pub struct EdgeInfo {
    pub kind: CurveKind,
    pub start: DVec3,
    pub end: DVec3,
    pub mid: DVec3,
    pub center: DVec3,
    pub axis: DVec3,
    pub radius: f64,
    pub length: f64,
    pub closed: bool,
}

fn v3(a: [f64; 3]) -> DVec3 {
    DVec3::from_array(a)
}

fn to_vec(v: UniquePtr<cxx::CxxVector<sys::Shape>>) -> Vec<Shape> {
    match v.as_ref() {
        None => Vec::new(),
        Some(v) => v.iter().map(|s| Shape { inner: sys::shape_clone(s) }).collect(),
    }
}

fn to_cxx<'a>(shapes: impl IntoIterator<Item = &'a Shape>) -> UniquePtr<cxx::CxxVector<sys::Shape>> {
    let mut v = sys::shape_vec_new();
    for s in shapes {
        sys::shape_vec_push(v.pin_mut(), s.raw());
    }
    v
}

fn wrap(p: UniquePtr<sys::Shape>) -> Shape {
    Shape { inner: p }
}

fn trsf12(m: &DMat4) -> [f64; 12] {
    let c = m.to_cols_array_2d(); // c[col][row]
    [
        c[0][0], c[1][0], c[2][0], c[3][0], //
        c[0][1], c[1][1], c[2][1], c[3][1], //
        c[0][2], c[1][2], c[2][2], c[3][2],
    ]
}

impl Shape {
    pub fn null() -> Self {
        wrap(sys::shape_null())
    }
    fn raw(&self) -> &sys::Shape {
        self.inner.as_ref().expect("null shape pointer")
    }
    /// Independent copy (geometry included) — safe to hand to another thread.
    pub fn deep_copy(&self) -> Shape {
        wrap(sys::shape_deep_copy(self.raw()))
    }
    pub fn is_null(&self) -> bool {
        sys::shape_is_null(self.raw())
    }
    pub fn kind(&self) -> ShapeKind {
        ShapeKind::from_u32(sys::shape_type(self.raw()))
    }
    /// Hash consistent with `same` (ignores orientation).
    pub fn hash(&self) -> u64 {
        sys::shape_hash(self.raw())
    }
    pub fn same(&self, other: &Shape) -> bool {
        sys::shape_is_same(self.raw(), other.raw())
    }
    fn sub(&self, kind: u32) -> Vec<Shape> {
        to_vec(sys::shape_subshapes(self.raw(), kind))
    }
    /// Unique faces in deterministic (MapShapes) order — the same order used by `mesh`.
    pub fn faces(&self) -> Vec<Shape> {
        self.sub(4)
    }
    pub fn edges(&self) -> Vec<Shape> {
        self.sub(6)
    }
    pub fn solids(&self) -> Vec<Shape> {
        self.sub(2)
    }
    pub fn shells(&self) -> Vec<Shape> {
        self.sub(3)
    }
    pub fn vertices(&self) -> Vec<Shape> {
        self.sub(7)
    }
    pub fn compound<'a>(shapes: impl IntoIterator<Item = &'a Shape>) -> Shape {
        let v = to_cxx(shapes);
        wrap(sys::make_compound(&v))
    }
    pub fn bbox(&self) -> Option<(DVec3, DVec3)> {
        let b = sys::shape_bbox(self.raw());
        (b.len() == 6).then(|| (DVec3::new(b[0], b[1], b[2]), DVec3::new(b[3], b[4], b[5])))
    }
    pub fn volume(&self) -> KResult<f64> {
        Ok(sys::shape_volume(self.raw())?)
    }
    pub fn area(&self) -> KResult<f64> {
        Ok(sys::shape_area(self.raw())?)
    }
    pub fn transformed(&self, m: &DMat4) -> KResult<Shape> {
        Ok(wrap(sys::shape_transform(self.raw(), &trsf12(m))?))
    }
    /// For each edge (in `edges()` order) the indices of its adjacent faces (in `faces()` order).
    pub fn edge_faces(&self) -> Vec<(i32, i32)> {
        sys::shape_edge_faces(self.raw()).chunks(2).map(|c| (c[0], c[1])).collect()
    }
    pub fn is_valid(&self) -> bool {
        sys::shape_is_valid(self.raw())
    }
    pub fn fixed(&self) -> KResult<Shape> {
        Ok(wrap(sys::shape_fix(self.raw())?))
    }
    pub fn mesh(&self, linear_deflection: f64, angular_deflection: f64) -> KResult<MeshOut> {
        Ok(sys::shape_mesh(self.raw(), linear_deflection, angular_deflection)?)
    }
    pub fn edge_polyline(&self, lin: f64, ang: f64) -> Vec<DVec3> {
        sys::edge_polyline(self.raw(), lin, ang).chunks(3).map(|c| DVec3::new(c[0], c[1], c[2])).collect()
    }
    /// Boundary loops of a face (outer first) as ordered polylines.
    pub fn face_loops(&self, lin: f64, ang: f64) -> Vec<Vec<DVec3>> {
        let mut sizes = Vec::new();
        let flat = sys::face_loops(self.raw(), lin, ang, &mut sizes);
        let mut out = Vec::with_capacity(sizes.len());
        let mut i = 0usize;
        for n in sizes {
            let n = n as usize;
            out.push((i..i + n).map(|k| DVec3::new(flat[3 * k], flat[3 * k + 1], flat[3 * k + 2])).collect());
            i += n;
        }
        out
    }
    pub fn distance_to_point(&self, p: DVec3) -> f64 {
        sys::point_distance(self.raw(), p.x, p.y, p.z)
    }
    pub fn face_info(&self) -> KResult<FaceInfo> {
        let i = sys::face_info(self.raw())?;
        Ok(FaceInfo {
            kind: match i.kind {
                0 => SurfaceKind::Plane,
                1 => SurfaceKind::Cylinder,
                2 => SurfaceKind::Cone,
                3 => SurfaceKind::Sphere,
                4 => SurfaceKind::Torus,
                _ => SurfaceKind::Other,
            },
            origin: v3(i.origin),
            normal: v3(i.normal),
            xdir: v3(i.xdir),
            radius: i.radius,
            area: i.area,
            center: v3(i.center),
        })
    }
    pub fn edge_info(&self) -> KResult<EdgeInfo> {
        let i = sys::edge_info(self.raw())?;
        Ok(EdgeInfo {
            kind: match i.kind {
                0 => CurveKind::Line,
                1 => CurveKind::Circle,
                3 => CurveKind::Degenerated,
                _ => CurveKind::Other,
            },
            start: v3(i.start),
            end: v3(i.end),
            mid: v3(i.mid),
            center: v3(i.center),
            axis: v3(i.axis),
            radius: i.radius,
            length: i.length,
            closed: i.closed,
        })
    }

    // ---- IO
    pub fn read_step(path: &str) -> KResult<Shape> {
        Ok(wrap(sys::read_step(path)?))
    }
    pub fn write_step(&self, path: &str) -> KResult<()> {
        Ok(sys::write_step(self.raw(), path)?)
    }
    pub fn read_brep_file(path: &str) -> KResult<Shape> {
        Ok(wrap(sys::read_brep_file(path)?))
    }
    pub fn to_brep_string(&self) -> KResult<String> {
        Ok(sys::write_brep_string(self.raw())?)
    }
    pub fn from_brep_string(s: &str) -> KResult<Shape> {
        Ok(wrap(sys::read_brep_string(s)?))
    }
}

/// A finished modelling operation. Keeps the OCCT algorithm alive so its
/// history can be queried.
pub struct Op {
    inner: UniquePtr<sys::Op>,
}
unsafe impl Send for Op {}

impl Op {
    fn raw(&self) -> &sys::Op {
        self.inner.as_ref().expect("null op")
    }
    pub fn shape(&self) -> Shape {
        wrap(sys::op_shape(self.raw()))
    }
    pub fn modified(&self, s: &Shape) -> Vec<Shape> {
        to_vec(sys::op_modified(self.raw(), s.raw()))
    }
    pub fn generated(&self, s: &Shape) -> Vec<Shape> {
        to_vec(sys::op_generated(self.raw(), s.raw()))
    }
    pub fn is_deleted(&self, s: &Shape) -> bool {
        sys::op_is_deleted(self.raw(), s.raw())
    }
    /// Start cap of an extrusion / revolution (may be null).
    pub fn first(&self) -> Shape {
        wrap(sys::op_first(self.raw()))
    }
    /// End cap of an extrusion / revolution (may be null).
    pub fn last(&self) -> Shape {
        wrap(sys::op_last(self.raw()))
    }
}

fn op(p: Result<UniquePtr<sys::Op>, cxx::Exception>) -> KResult<Op> {
    Ok(Op { inner: p? })
}

// ---------------------------------------------------------------- builders

pub fn line(a: DVec3, b: DVec3) -> KResult<Shape> {
    Ok(wrap(sys::make_line_edge(a.x, a.y, a.z, b.x, b.y, b.z)?))
}

pub fn circle(center: DVec3, normal: DVec3, r: f64) -> KResult<Shape> {
    Ok(wrap(sys::make_circle_edge(center.x, center.y, center.z, normal.x, normal.y, normal.z, r)?))
}

/// Arc from `start` to `end`, counter-clockwise about `normal`, centred at `center`.
pub fn arc(center: DVec3, normal: DVec3, start: DVec3, end: DVec3) -> KResult<Shape> {
    Ok(wrap(sys::make_arc_edge(
        center.x, center.y, center.z, normal.x, normal.y, normal.z, start.x, start.y, start.z, end.x, end.y, end.z,
    )?))
}

/// All closed regions bounded by a set of coplanar edges (each returned as a face).
pub fn planar_regions(edges: &[Shape]) -> KResult<Vec<Shape>> {
    let v = to_cxx(edges);
    let c = wrap(sys::make_planar_regions(&v)?);
    Ok(c.faces())
}

pub fn plane_face(origin: DVec3, normal: DVec3, half: f64) -> KResult<Shape> {
    Ok(wrap(sys::make_plane_face(origin.x, origin.y, origin.z, normal.x, normal.y, normal.z, half)?))
}

pub fn prism(profile: &Shape, dir: DVec3) -> KResult<Op> {
    op(sys::make_prism(profile.raw(), dir.x, dir.y, dir.z))
}

pub fn revolve(profile: &Shape, origin: DVec3, axis: DVec3, angle_rad: f64) -> KResult<Op> {
    if angle_rad.abs() < 1e-9 {
        return err("revolve angle must not be zero");
    }
    op(sys::make_revol(profile.raw(), origin.x, origin.y, origin.z, axis.x, axis.y, axis.z, angle_rad))
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolKind {
    Fuse = 0,
    Cut = 1,
    Common = 2,
}

pub fn boolean(kind: BoolKind, args: &[&Shape], tools: &[&Shape], simplify: bool) -> KResult<Op> {
    let a = to_cxx(args.iter().copied());
    let t = to_cxx(tools.iter().copied());
    op(sys::make_boolean(kind as u32, &a, &t, simplify))
}

pub fn fillet(solid: &Shape, edges: &[Shape], radius: f64) -> KResult<Op> {
    let e = to_cxx(edges);
    op(sys::make_fillet(solid.raw(), &e, radius))
}

pub fn chamfer(solid: &Shape, edges: &[Shape], dist: f64) -> KResult<Op> {
    let e = to_cxx(edges);
    op(sys::make_chamfer(solid.raw(), &e, dist))
}

pub fn shell(solid: &Shape, faces: &[Shape], thickness: f64) -> KResult<Op> {
    let f = to_cxx(faces);
    op(sys::make_shell(solid.raw(), &f, thickness))
}

pub fn transform(s: &Shape, m: &DMat4) -> KResult<Op> {
    op(sys::make_transform(s.raw(), &trsf12(m)))
}

/// Mirror copy across the plane through `origin` with normal `n`.
pub fn mirror(s: &Shape, origin: DVec3, n: DVec3) -> KResult<Op> {
    op(sys::make_mirror(s.raw(), origin.x, origin.y, origin.z, n.x, n.y, n.z))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rect(w: f64, h: f64) -> Vec<Shape> {
        let p = [DVec3::ZERO, DVec3::new(w, 0., 0.), DVec3::new(w, h, 0.), DVec3::new(0., h, 0.)];
        (0..4).map(|i| line(p[i], p[(i + 1) % 4]).unwrap()).collect()
    }

    #[test]
    fn regions_rect_with_hole() {
        let mut e = rect(10., 10.);
        e.push(circle(DVec3::new(5., 5., 0.), DVec3::Z, 2.).unwrap());
        let r = planar_regions(&e).unwrap();
        assert_eq!(r.len(), 2, "outer-with-hole + disk");
        let mut areas: Vec<f64> = r.iter().map(|f| f.face_info().unwrap().area).collect();
        areas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert!((areas[0] - std::f64::consts::PI * 4.).abs() < 1e-3);
        assert!((areas[1] - (100. - std::f64::consts::PI * 4.)).abs() < 1e-3);
    }

    #[test]
    fn regions_overlapping_rects() {
        let mut e = rect(10., 10.);
        for s in rect(10., 10.) {
            e.push(s.transformed(&DMat4::from_translation(DVec3::new(5., 5., 0.))).unwrap());
        }
        let r = planar_regions(&e).unwrap();
        assert_eq!(r.len(), 3);
    }

    #[test]
    fn extrude_fillet_history() {
        let r = planar_regions(&rect(20., 10.)).unwrap();
        assert_eq!(r.len(), 1);
        let face = &r[0];
        let p = prism(face, DVec3::new(0., 0., 5.)).unwrap();
        let solid = p.shape();
        assert!((solid.volume().unwrap() - 1000.).abs() < 1e-6);
        assert_eq!(solid.faces().len(), 6);
        // side faces are generated from profile edges
        let mut generated = 0;
        for e in face.edges() {
            generated += p.generated(&e).iter().filter(|s| s.kind() == ShapeKind::Face).count();
        }
        assert_eq!(generated, 4);
        assert_eq!(p.last().kind(), ShapeKind::Face);
        let last = p.last();
        let top = solid.faces().into_iter().find(|f| f.same(&last)).unwrap();
        let ti = top.face_info().unwrap();
        assert!((ti.normal - DVec3::Z).length() < 1e-9, "top normal {:?}", ti.normal);
        let first = p.first();
        let bottom = solid.faces().into_iter().find(|f| f.same(&first)).unwrap();
        assert!((bottom.face_info().unwrap().normal + DVec3::Z).length() < 1e-9);

        let edges = solid.edges();
        assert_eq!(edges.len(), 12);
        let f = fillet(&solid, &edges[..1], 1.0).unwrap();
        let fs = f.shape();
        assert_eq!(fs.faces().len(), 7);
        let mesh = fs.mesh(0.05, 0.3).unwrap();
        assert!(!mesh.indices.is_empty());
        assert_eq!(mesh.tri_face.len() * 3, mesh.indices.len());
        assert_eq!(mesh.edge_ranges.len(), fs.edges().len() + 1);
    }

    #[test]
    fn boolean_cut_and_names() {
        let a = prism(&planar_regions(&rect(10., 10.)).unwrap()[0], DVec3::Z * 10.).unwrap().shape();
        let disk = planar_regions(&[circle(DVec3::new(5., 5., 0.), DVec3::Z, 2.).unwrap()]).unwrap();
        let tool = prism(&disk[0], DVec3::Z * 10.).unwrap().shape();
        let c = boolean(BoolKind::Cut, &[&a], &[&tool], true).unwrap();
        let s = c.shape();
        assert_eq!(s.solids().len(), 1);
        assert_eq!(s.faces().len(), 7);
        // the top face of `a` is modified (gets a hole)
        let top = a.faces().into_iter().find(|f| (f.face_info().unwrap().normal - DVec3::Z).length() < 1e-9).unwrap();
        let m = c.modified(&top);
        assert_eq!(m.len(), 1);
        let edge_faces = s.edge_faces();
        // seam edges of the cylindrical hole only border one face
        assert!(edge_faces.iter().all(|(a, _)| *a >= 0));
        assert_eq!(edge_faces.iter().filter(|(_, b)| *b < 0).count(), 1);
    }

    #[test]
    fn mirror_keeps_volume() {
        let a = prism(&planar_regions(&rect(3., 4.)).unwrap()[0], DVec3::Z).unwrap().shape();
        let m = mirror(&a, DVec3::ZERO, DVec3::X).unwrap();
        let b = m.shape();
        assert!((b.volume().unwrap() - 12.).abs() < 1e-9);
        let (lo, hi) = b.bbox().unwrap();
        assert!(hi.x <= 1e-6 && lo.x < -2.9);
        assert_eq!(m.modified(&a.faces()[0]).len(), 1);
    }

    #[test]
    fn deep_copy_preserves_order() {
        let a = prism(&planar_regions(&rect(10., 6.)).unwrap()[0], DVec3::Z * 3.).unwrap().shape();
        let f = fillet(&a, &a.edges()[..3], 1.0).unwrap().shape();
        let c = f.deep_copy();
        let (fa, fc) = (f.faces(), c.faces());
        assert_eq!(fa.len(), fc.len());
        for (x, y) in fa.iter().zip(fc.iter()) {
            let (ix, iy) = (x.face_info().unwrap(), y.face_info().unwrap());
            assert!((ix.center - iy.center).length() < 1e-9);
        }
        let (ea, ec) = (f.edges(), c.edges());
        for (x, y) in ea.iter().zip(ec.iter()) {
            assert!((x.edge_info().unwrap().mid - y.edge_info().unwrap().mid).length() < 1e-9);
        }
        assert!(!fa[0].same(&fc[0]));
    }

    #[test]
    fn brep_roundtrip() {
        let a = prism(&planar_regions(&rect(3., 4.)).unwrap()[0], DVec3::Z).unwrap().shape();
        let s = a.to_brep_string().unwrap();
        let b = Shape::from_brep_string(&s).unwrap();
        assert!((b.volume().unwrap() - 12.).abs() < 1e-9);
    }
}
