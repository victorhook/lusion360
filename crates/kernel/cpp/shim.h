// Thin C++ layer over OpenCASCADE used by the Rust `lusion-kernel` crate.
// Everything that touches OCCT lives here; the Rust side only sees opaque
// `Shape` / `Op` handles plus plain-data structs.
#pragma once

#include <memory>
#include <vector>
#include "rust/cxx.h"

#include <TopoDS_Shape.hxx>
#include <BRepBuilderAPI_MakeShape.hxx>
#include <BRepTools_History.hxx>

namespace lk {

using Shape = TopoDS_Shape;

struct MeshOut;
struct SurfaceInfo;
struct EdgeInfo;

// Result of a modelling operation, keeping the algorithm alive so that the
// Rust side can query its history (Modified / Generated / IsDeleted) to
// propagate persistent face names.
class Op {
public:
  TopoDS_Shape result;
  std::shared_ptr<BRepBuilderAPI_MakeShape> mk;
  occ::handle<BRepTools_History> hist;
  TopoDS_Shape first;
  TopoDS_Shape last;
};

// ---- shape basics ----
std::unique_ptr<Shape> shape_null();
std::unique_ptr<Shape> shape_clone(const Shape& s);
bool shape_is_null(const Shape& s);
uint32_t shape_type(const Shape& s);
uint64_t shape_hash(const Shape& s);
bool shape_is_same(const Shape& a, const Shape& b);
std::unique_ptr<std::vector<Shape>> shape_subshapes(const Shape& s, uint32_t kind);
std::unique_ptr<std::vector<Shape>> shape_vec_new();
void shape_vec_push(std::vector<Shape>& v, const Shape& s);
std::unique_ptr<Shape> make_compound(const std::vector<Shape>& shapes);
rust::Vec<double> shape_bbox(const Shape& s);
double shape_volume(const Shape& s);
double shape_area(const Shape& s);
std::unique_ptr<Shape> shape_transform(const Shape& s, rust::Slice<const double> m);
rust::Vec<int32_t> shape_edge_faces(const Shape& s);
bool shape_is_valid(const Shape& s);
std::unique_ptr<Shape> shape_fix(const Shape& s);

// ---- queries ----
SurfaceInfo face_info(const Shape& f);
EdgeInfo edge_info(const Shape& e);
MeshOut shape_mesh(const Shape& s, double lin_defl, double ang_defl);
rust::Vec<double> edge_polyline(const Shape& e, double lin_defl, double ang_defl);
double point_distance(const Shape& s, double x, double y, double z);

// ---- sketch geometry ----
std::unique_ptr<Shape> make_line_edge(double ax, double ay, double az, double bx, double by, double bz);
std::unique_ptr<Shape> make_circle_edge(double cx, double cy, double cz, double nx, double ny, double nz, double r);
std::unique_ptr<Shape> make_arc_edge(double cx, double cy, double cz, double nx, double ny, double nz,
                                     double sx, double sy, double sz, double ex, double ey, double ez);
std::unique_ptr<Shape> make_planar_regions(const std::vector<Shape>& edges);
std::unique_ptr<Shape> make_plane_face(double ox, double oy, double oz, double nx, double ny, double nz, double half);

// ---- operations with history ----
std::unique_ptr<Op> make_prism(const Shape& profile, double dx, double dy, double dz);
std::unique_ptr<Op> make_revol(const Shape& profile, double ox, double oy, double oz, double ax, double ay, double az, double angle);
std::unique_ptr<Op> make_boolean(uint32_t kind, const std::vector<Shape>& args, const std::vector<Shape>& tools, bool simplify);
std::unique_ptr<Op> make_fillet(const Shape& solid, const std::vector<Shape>& edges, double radius);
std::unique_ptr<Op> make_chamfer(const Shape& solid, const std::vector<Shape>& edges, double dist);
std::unique_ptr<Op> make_shell(const Shape& solid, const std::vector<Shape>& faces, double thickness);
std::unique_ptr<Op> make_transform(const Shape& s, rust::Slice<const double> m);
std::unique_ptr<Op> make_mirror(const Shape& s, double ox, double oy, double oz, double nx, double ny, double nz);
std::unique_ptr<Shape> op_shape(const Op& op);
std::unique_ptr<std::vector<Shape>> op_modified(const Op& op, const Shape& s);
std::unique_ptr<std::vector<Shape>> op_generated(const Op& op, const Shape& s);
bool op_is_deleted(const Op& op, const Shape& s);
std::unique_ptr<Shape> op_first(const Op& op);
std::unique_ptr<Shape> op_last(const Op& op);

// ---- IO ----
std::unique_ptr<Shape> read_step(rust::Str path);
void write_step(const Shape& s, rust::Str path);
std::unique_ptr<Shape> read_brep_file(rust::Str path);
rust::String write_brep_string(const Shape& s);
std::unique_ptr<Shape> read_brep_string(rust::Str data);

}  // namespace lk
