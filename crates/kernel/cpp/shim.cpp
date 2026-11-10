#include "lusion-kernel/cpp/shim.h"
#include "lusion-kernel/src/ffi.rs.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <BOPAlgo_Builder.hxx>
#include <BOPAlgo_Tools.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepLib_ToolTriangulatedShape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <TopoDS_Wire.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <ShapeFix_Shape.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gce_MakeCirc.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

namespace lk {

namespace {

[[noreturn]] void fail(const std::string& what) { throw std::runtime_error(what); }

std::string describe(const Standard_Failure& e) {
  const char* m = e.what();
  std::string s = (m && *m) ? m : "OpenCASCADE failure";
  return s;
}

// Run `f`, converting any OCCT failure into std::runtime_error (which cxx
// maps onto a Rust `Err`).
template <typename F>
auto guarded(const char* what, F&& f) -> decltype(f()) {
  try {
    return f();
  } catch (const Standard_Failure& e) {
    fail(std::string(what) + ": " + describe(e));
  } catch (const std::runtime_error&) {
    throw;
  } catch (const std::exception& e) {
    fail(std::string(what) + ": " + e.what());
  } catch (...) {
    fail(std::string(what) + ": unknown error");
  }
}

TopAbs_ShapeEnum kind_enum(uint32_t k) {
  switch (k) {
    case 0: return TopAbs_COMPOUND;
    case 1: return TopAbs_COMPSOLID;
    case 2: return TopAbs_SOLID;
    case 3: return TopAbs_SHELL;
    case 4: return TopAbs_FACE;
    case 5: return TopAbs_WIRE;
    case 6: return TopAbs_EDGE;
    case 7: return TopAbs_VERTEX;
    default: return TopAbs_SHAPE;
  }
}

std::unique_ptr<std::vector<Shape>> list_to_vec(const TopTools_ListOfShape& l) {
  auto v = std::make_unique<std::vector<Shape>>();
  for (TopTools_ListOfShape::Iterator it(l); it.More(); it.Next()) v->push_back(it.Value());
  return v;
}

std::unique_ptr<Op> finish(std::shared_ptr<BRepBuilderAPI_MakeShape> mk, const char* what) {
  if (!mk->IsDone()) fail(std::string(what) + " failed");
  auto op = std::make_unique<Op>();
  op->result = mk->Shape();
  op->mk = mk;
  if (op->result.IsNull()) fail(std::string(what) + " produced an empty result");
  return op;
}

gp_Trsf trsf_from(rust::Slice<const double> m) {
  if (m.size() != 12) fail("transform needs 12 values (3x4 row-major)");
  gp_Trsf t;
  t.SetValues(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11]);
  return t;
}

}  // namespace

// ------------------------------------------------------------------ basics

std::unique_ptr<Shape> shape_null() { return std::make_unique<Shape>(); }
std::unique_ptr<Shape> shape_clone(const Shape& s) { return std::make_unique<Shape>(s); }
bool shape_is_null(const Shape& s) { return s.IsNull(); }

std::unique_ptr<Shape> shape_deep_copy(const Shape& s) {
  if (s.IsNull()) return std::make_unique<Shape>();
  BRepBuilderAPI_Copy cp(s, true, false);
  return std::make_unique<Shape>(cp.Shape());
}

uint32_t shape_type(const Shape& s) {
  if (s.IsNull()) return 99;
  switch (s.ShapeType()) {
    case TopAbs_COMPOUND: return 0;
    case TopAbs_COMPSOLID: return 1;
    case TopAbs_SOLID: return 2;
    case TopAbs_SHELL: return 3;
    case TopAbs_FACE: return 4;
    case TopAbs_WIRE: return 5;
    case TopAbs_EDGE: return 6;
    case TopAbs_VERTEX: return 7;
    default: return 8;
  }
}

uint64_t shape_hash(const Shape& s) { return (uint64_t)std::hash<TopoDS_Shape>{}(s); }
bool shape_is_same(const Shape& a, const Shape& b) { return a.IsSame(b); }

std::unique_ptr<std::vector<Shape>> shape_subshapes(const Shape& s, uint32_t kind) {
  auto v = std::make_unique<std::vector<Shape>>();
  if (s.IsNull()) return v;
  TopTools_IndexedMapOfShape map;
  TopExp::MapShapes(s, kind_enum(kind), map);
  v->reserve(map.Extent());
  for (int i = 1; i <= map.Extent(); ++i) v->push_back(map(i));
  return v;
}

std::unique_ptr<std::vector<Shape>> shape_vec_new() { return std::make_unique<std::vector<Shape>>(); }
void shape_vec_push(std::vector<Shape>& v, const Shape& s) { v.push_back(s); }

std::unique_ptr<Shape> make_compound(const std::vector<Shape>& shapes) {
  TopoDS_Compound c;
  BRep_Builder b;
  b.MakeCompound(c);
  for (auto& s : shapes)
    if (!s.IsNull()) b.Add(c, s);
  return std::make_unique<Shape>(c);
}

rust::Vec<double> shape_bbox(const Shape& s) {
  rust::Vec<double> out;
  Bnd_Box box;
  if (!s.IsNull()) BRepBndLib::Add(s, box);
  if (box.IsVoid()) return out;
  double x0, y0, z0, x1, y1, z1;
  box.Get(x0, y0, z0, x1, y1, z1);
  for (double v : {x0, y0, z0, x1, y1, z1}) out.push_back(v);
  return out;
}

double shape_volume(const Shape& s) {
  return guarded("volume", [&] {
    GProp_GProps p;
    BRepGProp::VolumeProperties(s, p);
    return p.Mass();
  });
}

double shape_area(const Shape& s) {
  return guarded("area", [&] {
    GProp_GProps p;
    BRepGProp::SurfaceProperties(s, p);
    return p.Mass();
  });
}

std::unique_ptr<Shape> shape_transform(const Shape& s, rust::Slice<const double> m) {
  return guarded("transform", [&] {
    gp_Trsf t = trsf_from(m);
    BRepBuilderAPI_Transform tr(s, t, true);
    return std::make_unique<Shape>(tr.Shape());
  });
}

// For every edge (in MapShapes order) the indices of up to two adjacent faces
// (in MapShapes order), -1 when absent.
rust::Vec<int32_t> shape_edge_faces(const Shape& s) {
  rust::Vec<int32_t> out;
  TopTools_IndexedMapOfShape faces, edges;
  TopExp::MapShapes(s, TopAbs_FACE, faces);
  TopExp::MapShapes(s, TopAbs_EDGE, edges);
  TopTools_IndexedDataMapOfShapeListOfShape anc;
  TopExp::MapShapesAndUniqueAncestors(s, TopAbs_EDGE, TopAbs_FACE, anc);
  for (int i = 1; i <= edges.Extent(); ++i) {
    int a = -1, b = -1;
    int idx = anc.FindIndex(edges(i));
    if (idx > 0) {
      for (TopTools_ListOfShape::Iterator it(anc(idx)); it.More(); it.Next()) {
        int fi = faces.FindIndex(it.Value()) - 1;
        if (fi < 0) continue;
        if (a < 0) a = fi;
        else if (b < 0 && fi != a) b = fi;
      }
    }
    out.push_back(a);
    out.push_back(b);
  }
  return out;
}

bool shape_is_valid(const Shape& s) {
  try {
    BRepCheck_Analyzer an(s);
    return an.IsValid();
  } catch (...) {
    return false;
  }
}

std::unique_ptr<Shape> shape_fix(const Shape& s) {
  return guarded("shape fix", [&] {
    ShapeFix_Shape fix(s);
    fix.Perform();
    return std::make_unique<Shape>(fix.Shape());
  });
}

// ------------------------------------------------------------------ queries

SurfaceInfo face_info(const Shape& shape) {
  return guarded("face info", [&] {
    if (shape.IsNull() || shape.ShapeType() != TopAbs_FACE) fail("not a face");
    const TopoDS_Face& f = TopoDS::Face(shape);
    SurfaceInfo info{};
    BRepAdaptor_Surface surf(f, true);
    bool rev = f.Orientation() == TopAbs_REVERSED;
    info.reversed = rev;
    GProp_GProps p;
    BRepGProp::SurfaceProperties(f, p);
    info.area = p.Mass();
    gp_Pnt c = p.CentreOfMass();
    info.center = {c.X(), c.Y(), c.Z()};
    auto set_ax = [&](const gp_Ax3& ax) {
      gp_Dir n = ax.Direction();
      gp_Dir x = ax.XDirection();
      gp_Pnt o = ax.Location();
      info.origin = {o.X(), o.Y(), o.Z()};
      info.normal = {n.X(), n.Y(), n.Z()};
      info.xdir = {x.X(), x.Y(), x.Z()};
    };
    switch (surf.GetType()) {
      case GeomAbs_Plane: {
        info.kind = 0;
        gp_Pln pl = surf.Plane();
        set_ax(pl.Position());
        // Outward normal: flip for reversed faces.
        if (rev) info.normal = {-info.normal[0], -info.normal[1], -info.normal[2]};
        break;
      }
      case GeomAbs_Cylinder:
        info.kind = 1;
        set_ax(surf.Cylinder().Position());
        info.radius = surf.Cylinder().Radius();
        break;
      case GeomAbs_Cone:
        info.kind = 2;
        set_ax(surf.Cone().Position());
        info.radius = surf.Cone().RefRadius();
        break;
      case GeomAbs_Sphere:
        info.kind = 3;
        set_ax(surf.Sphere().Position());
        info.radius = surf.Sphere().Radius();
        break;
      case GeomAbs_Torus:
        info.kind = 4;
        set_ax(surf.Torus().Position());
        info.radius = surf.Torus().MajorRadius();
        break;
      default:
        info.kind = 5;
        info.origin = info.center;
        break;
    }
    return info;
  });
}

EdgeInfo edge_info(const Shape& shape) {
  return guarded("edge info", [&] {
    if (shape.IsNull() || shape.ShapeType() != TopAbs_EDGE) fail("not an edge");
    const TopoDS_Edge& e = TopoDS::Edge(shape);
    EdgeInfo info{};
    if (BRep_Tool::Degenerated(e)) {
      info.kind = 3;
      return info;
    }
    BRepAdaptor_Curve c(e);
    double u0 = c.FirstParameter(), u1 = c.LastParameter();
    gp_Pnt a = c.Value(u0), b = c.Value(u1), m = c.Value(0.5 * (u0 + u1));
    info.start = {a.X(), a.Y(), a.Z()};
    info.end = {b.X(), b.Y(), b.Z()};
    info.mid = {m.X(), m.Y(), m.Z()};
    info.closed = a.Distance(b) < 1e-7;
    GProp_GProps p;
    BRepGProp::LinearProperties(e, p);
    info.length = p.Mass();
    switch (c.GetType()) {
      case GeomAbs_Line: {
        info.kind = 0;
        gp_Dir d = c.Line().Direction();
        info.axis = {d.X(), d.Y(), d.Z()};
        break;
      }
      case GeomAbs_Circle: {
        info.kind = 1;
        gp_Circ ci = c.Circle();
        info.center = {ci.Location().X(), ci.Location().Y(), ci.Location().Z()};
        gp_Dir d = ci.Axis().Direction();
        info.axis = {d.X(), d.Y(), d.Z()};
        info.radius = ci.Radius();
        break;
      }
      default:
        info.kind = 2;
        break;
    }
    return info;
  });
}

rust::Vec<double> edge_polyline(const Shape& shape, double lin, double ang) {
  rust::Vec<double> out;
  try {
    if (shape.IsNull() || shape.ShapeType() != TopAbs_EDGE) return out;
    const TopoDS_Edge& e = TopoDS::Edge(shape);
    if (BRep_Tool::Degenerated(e)) return out;
    BRepAdaptor_Curve c(e);
    GCPnts_TangentialDeflection approx(c, ang, lin);
    for (int i = 1; i <= approx.NbPoints(); ++i) {
      gp_Pnt p = approx.Value(i);
      out.push_back(p.X());
      out.push_back(p.Y());
      out.push_back(p.Z());
    }
  } catch (...) {
    out.clear();
  }
  return out;
}

// Boundary loops of a face as polylines (outer loop first), in wire order.
rust::Vec<double> face_loops(const Shape& shape, double lin, double ang, rust::Vec<uint32_t>& sizes) {
  rust::Vec<double> out;
  try {
    if (shape.IsNull() || shape.ShapeType() != TopAbs_FACE) return out;
    const TopoDS_Face& face = TopoDS::Face(shape);
    TopoDS_Wire outer = BRepTools::OuterWire(face);
    std::vector<TopoDS_Wire> wires;
    if (!outer.IsNull()) wires.push_back(outer);
    for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next())
      if (outer.IsNull() || !ex.Current().IsSame(outer)) wires.push_back(TopoDS::Wire(ex.Current()));
    for (auto& w : wires) {
      uint32_t count = 0;
      for (BRepTools_WireExplorer we(w, face); we.More(); we.Next()) {
        const TopoDS_Edge& e = we.Current();
        if (BRep_Tool::Degenerated(e)) continue;
        BRepAdaptor_Curve c(e);
        GCPnts_TangentialDeflection approx(c, ang, lin);
        std::vector<gp_Pnt> pts;
        for (int i = 1; i <= approx.NbPoints(); ++i) pts.push_back(approx.Value(i));
        if (e.Orientation() == TopAbs_REVERSED) std::reverse(pts.begin(), pts.end());
        // drop the last point: the next edge starts there
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
          out.push_back(pts[i].X());
          out.push_back(pts[i].Y());
          out.push_back(pts[i].Z());
          ++count;
        }
      }
      sizes.push_back(count);
    }
  } catch (...) {
    out.clear();
    sizes.clear();
  }
  return out;
}

MeshOut shape_mesh(const Shape& s, double lin, double ang) {
  return guarded("tessellation", [&] {
    MeshOut out;
    if (s.IsNull()) return out;
    BRepMesh_IncrementalMesh mesher(s, lin, false, ang, true);
    TopTools_IndexedMapOfShape faces, edges;
    TopExp::MapShapes(s, TopAbs_FACE, faces);
    TopExp::MapShapes(s, TopAbs_EDGE, edges);
    for (int fi = 1; fi <= faces.Extent(); ++fi) {
      const TopoDS_Face& face = TopoDS::Face(faces(fi));
      TopLoc_Location loc;
      occ::handle<Poly_Triangulation> tri = BRep_Tool::Triangulation(face, loc);
      if (tri.IsNull() || tri->NbNodes() == 0) continue;
      BRepLib_ToolTriangulatedShape::ComputeNormals(face, tri);
      // MapShapes keeps the orientation the face has inside the shape.
      bool reversed = face.Orientation() == TopAbs_REVERSED;
      const gp_Trsf& t = loc.Transformation();
      uint32_t base = (uint32_t)(out.positions.size() / 3);
      bool has_n = tri->HasNormals();
      for (int i = 1; i <= tri->NbNodes(); ++i) {
        gp_Pnt p = tri->Node(i).Transformed(t);
        out.positions.push_back((float)p.X());
        out.positions.push_back((float)p.Y());
        out.positions.push_back((float)p.Z());
        gp_Dir n(0, 0, 1);
        if (has_n) {
          n = tri->Normal(i);
          n.Transform(t);
        }
        if (reversed) n.Reverse();
        out.normals.push_back((float)n.X());
        out.normals.push_back((float)n.Y());
        out.normals.push_back((float)n.Z());
      }
      for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        if (reversed) std::swap(b, c);
        out.indices.push_back(base + a - 1);
        out.indices.push_back(base + b - 1);
        out.indices.push_back(base + c - 1);
        out.tri_face.push_back((uint32_t)(fi - 1));
      }
    }
    out.edge_ranges.push_back(0);
    for (int ei = 1; ei <= edges.Extent(); ++ei) {
      const TopoDS_Edge& e = TopoDS::Edge(edges(ei));
      if (!BRep_Tool::Degenerated(e)) {
        try {
          BRepAdaptor_Curve c(e);
          GCPnts_TangentialDeflection approx(c, ang, lin);
          for (int i = 1; i <= approx.NbPoints(); ++i) {
            gp_Pnt p = approx.Value(i);
            out.edge_points.push_back((float)p.X());
            out.edge_points.push_back((float)p.Y());
            out.edge_points.push_back((float)p.Z());
          }
        } catch (...) {
        }
      }
      out.edge_ranges.push_back((uint32_t)(out.edge_points.size() / 3));
    }
    return out;
  });
}

double point_distance(const Shape& s, double x, double y, double z) {
  try {
    BRepBuilderAPI_MakeVertex mv(gp_Pnt(x, y, z));
    BRepExtrema_DistShapeShape d(s, mv.Shape());
    if (!d.IsDone()) return 1e300;
    return d.Value();
  } catch (...) {
    return 1e300;
  }
}

// ------------------------------------------------------------------ sketch geometry

std::unique_ptr<Shape> make_line_edge(double ax, double ay, double az, double bx, double by, double bz) {
  return guarded("line", [&] {
    gp_Pnt a(ax, ay, az), b(bx, by, bz);
    if (a.Distance(b) < 1e-9) fail("zero-length line");
    BRepBuilderAPI_MakeEdge me(a, b);
    if (!me.IsDone()) fail("cannot build line");
    return std::make_unique<Shape>(me.Edge());
  });
}

std::unique_ptr<Shape> make_circle_edge(double cx, double cy, double cz, double nx, double ny, double nz, double r) {
  return guarded("circle", [&] {
    if (!(r > 1e-9)) fail("circle radius must be positive");
    gp_Circ c(gp_Ax2(gp_Pnt(cx, cy, cz), gp_Dir(nx, ny, nz)), r);
    BRepBuilderAPI_MakeEdge me(c);
    if (!me.IsDone()) fail("cannot build circle");
    return std::make_unique<Shape>(me.Edge());
  });
}

// Arc around `n` (counter-clockwise when looking against n) from s to e.
std::unique_ptr<Shape> make_arc_edge(double cx, double cy, double cz, double nx, double ny, double nz,
                                     double sx, double sy, double sz, double ex, double ey, double ez) {
  return guarded("arc", [&] {
    gp_Pnt c(cx, cy, cz), s(sx, sy, sz), e(ex, ey, ez);
    double r = c.Distance(s);
    if (r < 1e-9) fail("arc radius must be positive");
    gp_Dir n(nx, ny, nz);
    gp_Vec xs(c, s);
    gp_Ax2 ax(c, n, gp_Dir(xs));
    gp_Circ circ(ax, r);
    // angle of end point in the circle's frame
    gp_Vec ve(c, e);
    gp_Dir xd = ax.XDirection(), yd = ax.YDirection();
    double ang = std::atan2(ve.Dot(gp_Vec(yd)), ve.Dot(gp_Vec(xd)));
    if (ang <= 1e-9) ang += 2.0 * M_PI;
    BRepBuilderAPI_MakeEdge me(circ, 0.0, ang);
    if (!me.IsDone()) fail("cannot build arc");
    return std::make_unique<Shape>(me.Edge());
  });
}

// Split a set of coplanar edges into all closed planar regions
// (each region is a face; nested loops become holes plus separate inner faces).
std::unique_ptr<Shape> make_planar_regions(const std::vector<Shape>& edges) {
  return guarded("profile detection", [&] {
    TopoDS_Compound c;
    BRep_Builder b;
    b.MakeCompound(c);
    int n = 0;
    for (auto& e : edges)
      if (!e.IsNull()) {
        b.Add(c, e);
        ++n;
      }
    TopoDS_Compound empty;
    b.MakeCompound(empty);
    if (n == 0) return std::make_unique<Shape>(empty);
    TopoDS_Shape wires;
    int err = BOPAlgo_Tools::EdgesToWires(c, wires, false);
    if (err != 0 || wires.IsNull()) return std::make_unique<Shape>(empty);
    TopoDS_Shape faces;
    if (!BOPAlgo_Tools::WiresToFaces(wires, faces)) return std::make_unique<Shape>(empty);
    return std::make_unique<Shape>(faces);
  });
}

std::unique_ptr<Shape> make_plane_face(double ox, double oy, double oz, double nx, double ny, double nz, double half) {
  return guarded("plane face", [&] {
    gp_Pln pl(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz));
    BRepBuilderAPI_MakeFace mf(pl, -half, half, -half, half);
    return std::make_unique<Shape>(mf.Face());
  });
}

// ------------------------------------------------------------------ operations

std::unique_ptr<Op> make_prism(const Shape& profile, double dx, double dy, double dz) {
  return guarded("extrude", [&] {
    gp_Vec v(dx, dy, dz);
    if (v.Magnitude() < 1e-9) fail("extrude distance must not be zero");
    auto mk = std::make_shared<BRepPrimAPI_MakePrism>(profile, v, true, true);
    auto op = finish(mk, "extrude");
    op->first = mk->FirstShape();
    op->last = mk->LastShape();
    return op;
  });
}

std::unique_ptr<Op> make_revol(const Shape& profile, double ox, double oy, double oz, double ax, double ay, double az,
                               double angle) {
  return guarded("revolve", [&] {
    gp_Ax1 axis(gp_Pnt(ox, oy, oz), gp_Dir(ax, ay, az));
    std::shared_ptr<BRepPrimAPI_MakeRevol> mk;
    if (std::abs(angle) >= 2.0 * M_PI - 1e-9)
      mk = std::make_shared<BRepPrimAPI_MakeRevol>(profile, axis, true);
    else
      mk = std::make_shared<BRepPrimAPI_MakeRevol>(profile, axis, angle, true);
    auto op = finish(mk, "revolve");
    try {
      op->first = mk->FirstShape();
      op->last = mk->LastShape();
    } catch (...) {
    }
    return op;
  });
}

std::unique_ptr<Op> make_boolean(uint32_t kind, const std::vector<Shape>& args, const std::vector<Shape>& tools,
                                 bool simplify) {
  return guarded("boolean", [&] {
    TopTools_ListOfShape la, lt;
    for (auto& s : args) la.Append(s);
    for (auto& s : tools) lt.Append(s);
    std::shared_ptr<BRepAlgoAPI_BooleanOperation> mk;
    if (kind == 0) mk = std::make_shared<BRepAlgoAPI_Fuse>();
    else if (kind == 1) mk = std::make_shared<BRepAlgoAPI_Cut>();
    else mk = std::make_shared<BRepAlgoAPI_Common>();
    mk->SetArguments(la);
    mk->SetTools(lt);
    mk->SetRunParallel(true);
    mk->Build();
    if (mk->HasErrors()) {
      std::ostringstream os;
      mk->DumpErrors(os);
      fail("boolean failed: " + os.str());
    }
    if (simplify) mk->SimplifyResult(true, true);
    return finish(mk, "boolean");
  });
}

std::unique_ptr<Op> make_fillet(const Shape& solid, const std::vector<Shape>& edges, double radius) {
  return guarded("fillet", [&] {
    if (!(radius > 0)) fail("fillet radius must be positive");
    auto mk = std::make_shared<BRepFilletAPI_MakeFillet>(solid);
    for (auto& e : edges)
      if (!e.IsNull() && e.ShapeType() == TopAbs_EDGE) mk->Add(radius, TopoDS::Edge(e));
    mk->Build();
    if (!mk->IsDone()) fail("fillet failed (radius too large or unsupported geometry?)");
    return finish(mk, "fillet");
  });
}

std::unique_ptr<Op> make_chamfer(const Shape& solid, const std::vector<Shape>& edges, double dist) {
  return guarded("chamfer", [&] {
    if (!(dist > 0)) fail("chamfer distance must be positive");
    auto mk = std::make_shared<BRepFilletAPI_MakeChamfer>(solid);
    for (auto& e : edges)
      if (!e.IsNull() && e.ShapeType() == TopAbs_EDGE) mk->Add(dist, TopoDS::Edge(e));
    mk->Build();
    if (!mk->IsDone()) fail("chamfer failed (distance too large?)");
    return finish(mk, "chamfer");
  });
}

std::unique_ptr<Op> make_shell(const Shape& solid, const std::vector<Shape>& faces, double thickness) {
  return guarded("shell", [&] {
    TopTools_ListOfShape lf;
    for (auto& f : faces) lf.Append(f);
    auto mk = std::make_shared<BRepOffsetAPI_MakeThickSolid>();
    mk->MakeThickSolidByJoin(solid, lf, -thickness, 1e-3, BRepOffset_Skin, false, false, GeomAbs_Intersection);
    mk->Build();
    return finish(mk, "shell");
  });
}

std::unique_ptr<Op> make_transform(const Shape& s, rust::Slice<const double> m) {
  return guarded("move", [&] {
    gp_Trsf t = trsf_from(m);
    auto mk = std::make_shared<BRepBuilderAPI_Transform>(s, t, true);
    return finish(mk, "move");
  });
}

std::unique_ptr<Op> make_mirror(const Shape& s, double ox, double oy, double oz, double nx, double ny, double nz) {
  return guarded("mirror", [&] {
    gp_Trsf t;
    t.SetMirror(gp_Ax2(gp_Pnt(ox, oy, oz), gp_Dir(nx, ny, nz)));
    auto mk = std::make_shared<BRepBuilderAPI_Transform>(s, t, true);
    return finish(mk, "mirror");
  });
}

std::unique_ptr<Shape> op_shape(const Op& op) { return std::make_unique<Shape>(op.result); }

std::unique_ptr<std::vector<Shape>> op_modified(const Op& op, const Shape& s) {
  try {
    if (!op.hist.IsNull()) return list_to_vec(op.hist->Modified(s));
    if (op.mk) return list_to_vec(op.mk->Modified(s));
  } catch (...) {
  }
  return std::make_unique<std::vector<Shape>>();
}

std::unique_ptr<std::vector<Shape>> op_generated(const Op& op, const Shape& s) {
  try {
    if (!op.hist.IsNull()) return list_to_vec(op.hist->Generated(s));
    if (op.mk) return list_to_vec(op.mk->Generated(s));
  } catch (...) {
  }
  return std::make_unique<std::vector<Shape>>();
}

bool op_is_deleted(const Op& op, const Shape& s) {
  try {
    if (!op.hist.IsNull()) return op.hist->IsRemoved(s);
    if (op.mk) return op.mk->IsDeleted(s);
  } catch (...) {
  }
  return false;
}

std::unique_ptr<Shape> op_first(const Op& op) { return std::make_unique<Shape>(op.first); }
std::unique_ptr<Shape> op_last(const Op& op) { return std::make_unique<Shape>(op.last); }

// ------------------------------------------------------------------ IO

std::unique_ptr<Shape> read_step(rust::Str path) {
  return guarded("STEP import", [&] {
    STEPControl_Reader reader;
    std::string p(path);
    if (reader.ReadFile(p.c_str()) != IFSelect_RetDone) fail("cannot read STEP file " + p);
    reader.TransferRoots();
    TopoDS_Shape s = reader.OneShape();
    if (s.IsNull()) fail("STEP file contains no shapes");
    return std::make_unique<Shape>(s);
  });
}

void write_step(const Shape& s, rust::Str path) {
  guarded("STEP export", [&] {
    STEPControl_Writer w;
    if (w.Transfer(s, STEPControl_AsIs) != IFSelect_RetDone) fail("STEP transfer failed");
    std::string p(path);
    if (w.Write(p.c_str()) != IFSelect_RetDone) fail("cannot write " + p);
    return 0;
  });
}

std::unique_ptr<Shape> read_brep_file(rust::Str path) {
  return guarded("BREP import", [&] {
    TopoDS_Shape s;
    BRep_Builder b;
    std::string p(path);
    if (!BRepTools::Read(s, p.c_str(), b)) fail("cannot read BREP file " + p);
    return std::make_unique<Shape>(s);
  });
}

rust::String write_brep_string(const Shape& s) {
  return guarded("BREP write", [&] {
    std::ostringstream os;
    BRepTools::Write(s, os);
    return rust::String(os.str());
  });
}

std::unique_ptr<Shape> read_brep_string(rust::Str data) {
  return guarded("BREP read", [&] {
    std::istringstream is{std::string(data)};
    TopoDS_Shape s;
    BRep_Builder b;
    BRepTools::Read(s, is, b);
    if (s.IsNull()) fail("invalid BREP data");
    return std::make_unique<Shape>(s);
  });
}

}  // namespace lk
