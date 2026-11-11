//! A deliberately small, dependency-free MCP stdio server for Lusion360.
//!
//! It owns documents in memory for the duration of the process and uses the
//! same core evaluator as the desktop application.  Keeping this binary free
//! of UI and async runtime dependencies makes tool calls fast and predictable.

use anyhow::{anyhow, bail, Context, Result};
use glam::{DVec2, DVec3};
use lusion_core::doc::*;
use lusion_core::expr::Parameter;
use lusion_core::io;
use lusion_core::model::Model;
use lusion_core::sketch::{Constraint, CurveGeom, DimKind, Sketch};
use serde::Deserialize;
use serde_json::{json, Value};
use std::collections::BTreeMap;
use std::io::{self as stdio, BufRead, Write};
use std::path::Path;

struct Session {
    doc: Document,
    model: Model,
    path: Option<String>,
}

impl Session {
    fn new(doc: Document, path: Option<String>) -> Result<Self> {
        let mut s = Self {
            doc,
            model: Model::new(),
            path,
        };
        s.recompute()?;
        Ok(s)
    }
    fn recompute(&mut self) -> Result<()> {
        self.model.update(&self.doc);
        let errors: Vec<_> = self
            .model
            .status
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                s.error.as_ref().map(|e| {
                    format!(
                        "feature {} ({}): {e}",
                        self.doc.features[i].id, self.doc.features[i].name
                    )
                })
            })
            .collect();
        if errors.is_empty() {
            Ok(())
        } else {
            bail!(errors.join("; "))
        }
    }
    fn transactional<T>(
        &mut self,
        f: impl FnOnce(&mut Document, &mut Model) -> Result<T>,
    ) -> Result<T> {
        let old_doc = self.doc.clone();
        match f(&mut self.doc, &mut self.model).and_then(|v| {
            self.recompute()?;
            Ok(v)
        }) {
            Ok(v) => Ok(v),
            Err(e) => {
                self.doc = old_doc;
                // A failed feature may have touched cached downstream state. Rebuild only
                // on rollback; successful calls retain Model's incremental cache.
                self.model.invalidate();
                let _ = self.model.update(&self.doc);
                Err(e)
            }
        }
    }
}

#[derive(Default)]
struct Server {
    docs: BTreeMap<String, Session>,
    next_doc: u32,
}

#[derive(Deserialize)]
struct CreateArgs {
    #[serde(default)]
    name: String,
    #[serde(default)]
    path: Option<String>,
}
#[derive(Deserialize)]
struct OpenArgs {
    path: String,
}
#[derive(Deserialize)]
struct DocArg {
    document: String,
}
#[derive(Deserialize)]
struct SaveArgs {
    document: String,
    #[serde(default)]
    path: Option<String>,
}
#[derive(Deserialize)]
struct SetParametersArgs {
    document: String,
    values: BTreeMap<String, String>,
}
#[derive(Deserialize)]
struct Rect {
    x: f64,
    y: f64,
    width: String,
    height: String,
}
#[derive(Deserialize)]
struct Circle {
    x: f64,
    y: f64,
    diameter: String,
}
#[derive(Deserialize)]
struct SketchArgs {
    document: String,
    #[serde(default)]
    plane: Option<String>,
    #[serde(default)]
    face: Option<FaceSelect>,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    rectangles: Vec<Rect>,
    #[serde(default)]
    circles: Vec<Circle>,
}
#[derive(Deserialize)]
struct FaceSelect {
    body: u64,
    face: String,
}
#[derive(Deserialize)]
struct GeometryArgs {
    document: String,
    body: u64,
}
#[derive(Deserialize)]
struct EdgeModifyArgs {
    document: String,
    body: u64,
    edges: Vec<String>,
    size: String,
    #[serde(default)]
    name: Option<String>,
}
#[derive(Deserialize)]
struct ShellArgs {
    document: String,
    body: u64,
    faces: Vec<String>,
    thickness: String,
    #[serde(default)]
    name: Option<String>,
}
#[derive(Deserialize)]
struct ExportArgs {
    document: String,
    path: String,
    #[serde(default)]
    bodies: Vec<u64>,
}
#[derive(Deserialize)]
struct PreviewArgs {
    document: String,
    path: String,
    #[serde(default = "preview_size")]
    width: u32,
    #[serde(default = "preview_size")]
    height: u32,
}
fn preview_size() -> u32 {
    800
}
#[derive(Deserialize)]
struct ExtrudeArgs {
    document: String,
    sketch: u32,
    #[serde(default)]
    profile: Option<String>,
    distance: String,
    #[serde(default)]
    operation: Option<String>,
    #[serde(default)]
    name: Option<String>,
}

fn plane(s: &str) -> Result<PlaneRef> {
    Ok(PlaneRef::Origin(match s.to_ascii_uppercase().as_str() {
        "XY" => OriginPlane::XY,
        "XZ" => OriginPlane::XZ,
        "YZ" => OriginPlane::YZ,
        _ => bail!("plane must be XY, XZ, or YZ"),
    }))
}
fn op(s: Option<&str>) -> Result<Operation> {
    Ok(
        match s.unwrap_or("new_body").to_ascii_lowercase().as_str() {
            "new_body" | "new" => Operation::NewBody,
            "join" => Operation::Join,
            "cut" => Operation::Cut,
            "intersect" => Operation::Intersect,
            _ => bail!("operation must be new_body, join, cut, or intersect"),
        },
    )
}
fn value(expr: &str, doc: &Document) -> Result<f64> {
    lusion_core::expr::ParamValues::evaluate(&doc.parameters)
        .eval(expr)
        .map_err(|e| anyhow!(e))
}

fn face_ref(s: &Session, sel: &FaceSelect) -> Result<FaceRef> {
    let body = s
        .model
        .current(&s.doc)
        .bodies
        .get(&BodyId(sel.body))
        .ok_or_else(|| anyhow!("body {} not found", sel.body))?;
    let i = body
        .face_index(&sel.face)
        .ok_or_else(|| anyhow!("face '{}' not found; use lusion_query_geometry", sel.face))?;
    let hint = body.shape.faces()[i]
        .face_info()
        .map_err(|e| anyhow!(e.0))?
        .center
        .to_array();
    Ok(FaceRef {
        body: BodyId(sel.body),
        name: sel.face.clone(),
        hint,
    })
}

fn geometry(s: &Session, body_id: u64) -> Result<Value> {
    let b = s
        .model
        .current(&s.doc)
        .bodies
        .get(&BodyId(body_id))
        .ok_or_else(|| anyhow!("body {body_id} not found"))?;
    if b.is_mesh() {
        bail!("mesh bodies have no editable B-rep faces or edges")
    }
    let faces: Vec<Value> = b.shape.faces().iter().enumerate().filter_map(|(i, f)| f.face_info().ok().map(|x| json!({
        "name":b.face_names.get(i), "index":i, "surface":format!("{:?}",x.kind).to_lowercase(), "center_mm":x.center.to_array(), "normal":x.normal.to_array(), "area_mm2":x.area
    }))).collect();
    let edges: Vec<Value> = b.shape.edges().iter().enumerate().filter_map(|(i, e)| e.edge_info().ok().map(|x| json!({
        "name":b.edge_names.get(i), "index":i, "curve":format!("{:?}",x.kind).to_lowercase(), "midpoint_mm":x.mid.to_array(), "length_mm":x.length, "radius_mm":x.radius, "closed":x.closed
    }))).collect();
    Ok(json!({"body":body_id,"faces":faces,"edges":edges}))
}

fn svg_preview(s: &Session, path: &Path, width: u32, height: u32) -> Result<()> {
    let bodies: Vec<_> = s
        .model
        .current(&s.doc)
        .bodies
        .values()
        .filter(|b| !b.is_mesh())
        .collect();
    if bodies.is_empty() {
        bail!("no B-rep bodies to preview")
    }
    let project = |p: DVec3| DVec2::new(p.x - p.y, (p.x + p.y) * 0.5 - p.z);
    let mut pts = Vec::new();
    for b in &bodies {
        if let Some((a, z)) = b.shape.bbox() {
            for &x in &[a.x, z.x] {
                for &y in &[a.y, z.y] {
                    for &z in &[a.z, z.z] {
                        pts.push(project(DVec3::new(x, y, z)));
                    }
                }
            }
        }
    }
    let lo = pts
        .iter()
        .fold(DVec2::splat(f64::INFINITY), |a, &p| a.min(p));
    let hi = pts
        .iter()
        .fold(DVec2::splat(f64::NEG_INFINITY), |a, &p| a.max(p));
    let scale = ((width as f64 - 40.) / (hi.x - lo.x).max(1e-6))
        .min((height as f64 - 40.) / (hi.y - lo.y).max(1e-6));
    let xy = |p: DVec3| {
        let q = project(p);
        (
            (q.x - lo.x) * scale + 20.,
            height as f64 - ((q.y - lo.y) * scale + 20.),
        )
    };
    let mut paths = String::new();
    for b in bodies {
        for e in b.shape.edges() {
            let line = e.edge_polyline(0.2, 0.2);
            if line.len() < 2 {
                continue;
            }
            let (x, y) = xy(line[0]);
            paths.push_str(&format!("M{x:.2},{y:.2}"));
            for p in &line[1..] {
                let (x, y) = xy(*p);
                paths.push_str(&format!(" L{x:.2},{y:.2}"));
            }
            paths.push(' ');
        }
    }
    let svg = format!(
        r##"<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}"><rect width="100%" height="100%" fill="#16191f"/><path d="{paths}" fill="none" stroke="#d7e1ef" stroke-width="1.2" stroke-linejoin="round"/></svg>"##
    );
    std::fs::write(path, svg).with_context(|| format!("writing {}", path.display()))?;
    Ok(())
}

fn summary(s: &Session) -> Value {
    let st = s.model.current(&s.doc);
    let bodies: Vec<Value> = st.bodies.values().map(|b| {
        let (min, max) = b.shape.bbox().map(|(a,b)| (a.to_array(), b.to_array())).unwrap_or(([0.;3],[0.;3]));
        json!({"id": b.id.0, "component": b.component, "mesh": b.is_mesh(), "bbox_mm": {"min":min,"max":max}, "volume_mm3": b.shape.volume().ok()})
    }).collect();
    let features: Vec<Value> = s.doc.features.iter().enumerate().map(|(i,f)| json!({
        "id":f.id, "name":f.name, "kind":f.kind.type_name(), "suppressed":f.suppressed,
        "status": s.model.status.get(i).map(|x| json!({"error":x.error,"warnings":x.warnings,"milliseconds":x.millis}))
    })).collect();
    json!({"format": s.doc.format, "path":s.path, "parameters":s.doc.parameters, "features":features, "bodies":bodies, "update_milliseconds":s.model.last_update_ms})
}

fn schema() -> Value {
    json!({"tools":[
     {"name":"lusion_create_design","description":"Create a new parametric Lusion360 document. Returns a document handle. Save it after modelling.","inputSchema":{"type":"object","properties":{"name":{"type":"string"},"path":{"type":"string","description":"Optional .l360 save path"}}}},
     {"name":"lusion_open_design","description":"Load a native .l360 design for inspection or editing.","inputSchema":{"type":"object","required":["path"],"properties":{"path":{"type":"string"}}}},
     {"name":"lusion_inspect","description":"Return compact timeline, parameters, body bounds/volume, and evaluation status.","inputSchema":{"type":"object","required":["document"],"properties":{"document":{"type":"string"}}}},
     {"name":"lusion_set_parameters","description":"Create or update named parameters. Expressions use mm by default and may reference other parameters.","inputSchema":{"type":"object","required":["document","values"],"properties":{"document":{"type":"string"},"values":{"type":"object","additionalProperties":{"type":"string"}}}}},
     {"name":"lusion_query_geometry","description":"List stable face/edge names and geometry for a B-rep body. Use these names for sketches on faces, fillets, and chamfers.","inputSchema":{"type":"object","required":["document","body"],"properties":{"document":{"type":"string"},"body":{"type":"integer"}}}},
     {"name":"lusion_add_sketch","description":"Add a sketch on an origin plane or a queried planar face. Supply exactly one of plane (XY/XZ/YZ) or face ({body,face}). Rectangles are anchored at x,y; width/height and circle diameter are expressions.","inputSchema":{"type":"object","required":["document"],"properties":{"document":{"type":"string"},"plane":{"enum":["XY","XZ","YZ"]},"face":{"type":"object","required":["body","face"]},"name":{"type":"string"},"rectangles":{"type":"array","items":{"type":"object","required":["x","y","width","height"]}},"circles":{"type":"array","items":{"type":"object","required":["x","y","diameter"]}}}}},
     {"name":"lusion_add_extrude","description":"Extrude a solved sketch profile. profile may be largest (default) or index:N. A failed operation is rolled back.","inputSchema":{"type":"object","required":["document","sketch","distance"],"properties":{"document":{"type":"string"},"sketch":{"type":"integer"},"profile":{"type":"string"},"distance":{"type":"string"},"operation":{"enum":["new_body","join","cut","intersect"]},"name":{"type":"string"}}}},
     {"name":"lusion_add_fillet","description":"Round queried body edges by stable edge name. The operation is transactional.","inputSchema":{"type":"object","required":["document","body","edges","size"],"properties":{"document":{"type":"string"},"body":{"type":"integer"},"edges":{"type":"array","items":{"type":"string"}},"size":{"type":"string"},"name":{"type":"string"}}}},
     {"name":"lusion_add_chamfer","description":"Bevel queried body edges by stable edge name. The operation is transactional.","inputSchema":{"type":"object","required":["document","body","edges","size"],"properties":{"document":{"type":"string"},"body":{"type":"integer"},"edges":{"type":"array","items":{"type":"string"}},"size":{"type":"string"},"name":{"type":"string"}}}},
     {"name":"lusion_add_shell","description":"Hollow a B-rep body, removing named planar faces. Query geometry first for stable face names.","inputSchema":{"type":"object","required":["document","body","faces","thickness"],"properties":{"document":{"type":"string"},"body":{"type":"integer"},"faces":{"type":"array","items":{"type":"string"}},"thickness":{"type":"string"},"name":{"type":"string"}}}},
     {"name":"lusion_export","description":"Export visible or selected bodies to STEP, BREP, STL, 3MF, or OBJ based on path extension.","inputSchema":{"type":"object","required":["document","path"],"properties":{"document":{"type":"string"},"path":{"type":"string"},"bodies":{"type":"array","items":{"type":"integer"}}}}},
     {"name":"lusion_preview","description":"Write a fast deterministic isometric SVG wireframe preview of the current B-rep model.","inputSchema":{"type":"object","required":["document","path"],"properties":{"document":{"type":"string"},"path":{"type":"string"},"width":{"type":"integer"},"height":{"type":"integer"}}}},
     {"name":"lusion_save_design","description":"Atomically save a native editable .l360 document.","inputSchema":{"type":"object","required":["document"],"properties":{"document":{"type":"string"},"path":{"type":"string"}}}}
    ]})
}

fn call(server: &mut Server, name: &str, args: Value) -> Result<Value> {
    match name {
        "lusion_create_design" => {
            let a: CreateArgs = serde_json::from_value(args)?;
            let mut d = Document::default();
            if !a.name.is_empty() {
                d.components[0].name = a.name;
            }
            server.next_doc += 1;
            let id = format!("doc-{}", server.next_doc);
            server.docs.insert(id.clone(), Session::new(d, a.path)?);
            Ok(json!({"document":id}))
        }
        "lusion_open_design" => {
            let a: OpenArgs = serde_json::from_value(args)?;
            let d = io::load_native(Path::new(&a.path))?;
            server.next_doc += 1;
            let id = format!("doc-{}", server.next_doc);
            server
                .docs
                .insert(id.clone(), Session::new(d, Some(a.path))?);
            Ok(json!({"document":id}))
        }
        "lusion_inspect" => {
            let a: DocArg = serde_json::from_value(args)?;
            Ok(summary(
                server
                    .docs
                    .get(&a.document)
                    .ok_or_else(|| anyhow!("unknown document"))?,
            ))
        }
        "lusion_set_parameters" => {
            let a: SetParametersArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            s.transactional(|d, _| {
                for (name, expr) in a.values {
                    if let Some(p) = d.parameters.iter_mut().find(|p| p.name == name) {
                        p.expr = expr
                    } else {
                        d.parameters.push(Parameter {
                            name,
                            expr,
                            comment: String::new(),
                        })
                    }
                }
                Ok(())
            })?;
            Ok(summary(s))
        }
        "lusion_query_geometry" => {
            let a: GeometryArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            geometry(s, a.body)
        }
        "lusion_add_sketch" => {
            let a: SketchArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let sketch_plane = match (&a.plane, &a.face) {
                (Some(p), None) => plane(p)?,
                (None, Some(f)) => PlaneRef::Face(face_ref(s, f)?),
                _ => bail!("supply exactly one of plane or face"),
            };
            let fid = s.transactional(|d, _| {
                let mut sk = Sketch::new(sketch_plane.clone());
                for r in &a.rectangles {
                    let w = value(&r.width, d)?;
                    let h = value(&r.height, d)?;
                    let p = [
                        sk.add_point(DVec2::new(r.x, r.y)),
                        sk.add_point(DVec2::new(r.x + w, r.y)),
                        sk.add_point(DVec2::new(r.x + w, r.y + h)),
                        sk.add_point(DVec2::new(r.x, r.y + h)),
                    ];
                    sk.point_mut(p[0]).unwrap().fixed = true;
                    let l: Vec<_> = (0..4).map(|i| sk.add_line(p[i], p[(i + 1) % 4])).collect();
                    for &c in &[l[0], l[2]] {
                        sk.add_constraint(Constraint::Horizontal(c));
                    }
                    for &c in &[l[1], l[3]] {
                        sk.add_constraint(Constraint::Vertical(c));
                    }
                    sk.add_constraint(Constraint::Dimension {
                        kind: DimKind::Length(l[0]),
                        expr: r.width.clone(),
                        value: 0.,
                        offset: [0.; 2],
                    });
                    sk.add_constraint(Constraint::Dimension {
                        kind: DimKind::Length(l[1]),
                        expr: r.height.clone(),
                        value: 0.,
                        offset: [0.; 2],
                    });
                }
                for c in &a.circles {
                    let radius = value(&c.diameter, d)? / 2.;
                    let p = sk.add_point(DVec2::new(c.x, c.y));
                    sk.point_mut(p).unwrap().fixed = true;
                    let ci = sk.add_curve(CurveGeom::Circle { c: p, r: radius }, false);
                    sk.add_constraint(Constraint::Dimension {
                        kind: DimKind::Diameter(ci),
                        expr: c.diameter.clone(),
                        value: 0.,
                        offset: [0.; 2],
                    });
                }
                if sk.curves.is_empty() {
                    bail!("a sketch needs at least one rectangle or circle")
                }
                let name = a.name.clone().unwrap_or_else(|| d.next_name("Sketch"));
                Ok(d.insert_feature(name, ROOT, FeatureKind::Sketch(sk)))
            })?;
            Ok(json!({"feature":fid,"document":a.document}))
        }
        "lusion_add_extrude" => {
            let a: ExtrudeArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let fid = s.transactional(|d, m| {
                m.update(d);
                let ss = m.current(d).sketches.get(&a.sketch).ok_or_else(|| {
                    anyhow!(
                        "sketch {} is unavailable; it may be after the timeline marker",
                        a.sketch
                    )
                })?;
                let p = match a.profile.as_deref().unwrap_or("largest") {
                    "largest" => ss.profiles.iter().max_by(|x, y| x.area.total_cmp(&y.area)),
                    x if x.starts_with("index:") => x[6..]
                        .parse::<usize>()
                        .ok()
                        .and_then(|i| ss.profiles.get(i)),
                    _ => None,
                }
                .ok_or_else(|| anyhow!("profile not found"))?;
                let profile = ProfileRef::Sketch {
                    sketch: a.sketch,
                    curves: p.curves.clone(),
                    hint: p.hint.to_array(),
                };
                let name = a.name.clone().unwrap_or_else(|| d.next_name("Extrude"));
                Ok(d.insert_feature(
                    name,
                    ROOT,
                    FeatureKind::Extrude(Extrude {
                        profiles: vec![profile],
                        distance: a.distance.clone(),
                        distance2: String::new(),
                        mode: ExtentMode::OneSide,
                        start_offset: String::new(),
                        op: op(a.operation.as_deref())?,
                        targets: vec![],
                    }),
                ))
            })?;
            Ok(json!({"feature":fid,"document":a.document}))
        }
        "lusion_add_fillet" | "lusion_add_chamfer" => {
            let a: EdgeModifyArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let fillet = name == "lusion_add_fillet";
            let fid = s.transactional(|d, _| {
                let body = BodyId(a.body);
                let edges = a
                    .edges
                    .iter()
                    .map(|name| EdgeRef {
                        body,
                        name: name.clone(),
                        hint: [0.; 3],
                    })
                    .collect();
                let feature = if fillet {
                    FeatureKind::Fillet(Fillet {
                        edges,
                        radius: a.size.clone(),
                    })
                } else {
                    FeatureKind::Chamfer(Chamfer {
                        edges,
                        distance: a.size.clone(),
                    })
                };
                let base = if fillet { "Fillet" } else { "Chamfer" };
                Ok(d.insert_feature(
                    a.name.clone().unwrap_or_else(|| d.next_name(base)),
                    ROOT,
                    feature,
                ))
            })?;
            Ok(json!({"feature":fid,"document":a.document}))
        }
        "lusion_add_shell" => {
            let a: ShellArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let body = BodyId(a.body);
            let faces: Result<Vec<_>> = a
                .faces
                .iter()
                .map(|name| {
                    face_ref(
                        s,
                        &FaceSelect {
                            body: a.body,
                            face: name.clone(),
                        },
                    )
                })
                .collect();
            let faces = faces?;
            let fid = s.transactional(|d, _| {
                Ok(d.insert_feature(
                    a.name.clone().unwrap_or_else(|| d.next_name("Shell")),
                    ROOT,
                    FeatureKind::Shell(Shell {
                        body: Some(body),
                        faces,
                        thickness: a.thickness.clone(),
                    }),
                ))
            })?;
            Ok(json!({"feature":fid,"document":a.document}))
        }
        "lusion_export" => {
            let a: ExportArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let st = s.model.current(&s.doc);
            let selected: Vec<_> = if a.bodies.is_empty() {
                io::visible_bodies(&s.doc, st)
            } else {
                a.bodies
                    .iter()
                    .filter_map(|id| st.bodies.get(&BodyId(*id)))
                    .collect()
            };
            if selected.len() != a.bodies.len() && !a.bodies.is_empty() {
                bail!("one or more requested bodies do not exist")
            }
            let names: Vec<String> = selected
                .iter()
                .map(|b| {
                    s.doc
                        .body_props(b.id)
                        .name
                        .unwrap_or_else(|| format!("body{}", b.id.0))
                })
                .collect();
            io::export(Path::new(&a.path), &selected, &names)?;
            Ok(json!({"path":a.path,"bodies":selected.len()}))
        }
        "lusion_preview" => {
            let a: PreviewArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            if a.width < 64 || a.height < 64 || a.width > 4096 || a.height > 4096 {
                bail!("preview dimensions must be 64..4096")
            }
            svg_preview(s, Path::new(&a.path), a.width, a.height)?;
            Ok(json!({"path":a.path,"format":"svg","width":a.width,"height":a.height}))
        }
        "lusion_save_design" => {
            let a: SaveArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let path = a
                .path
                .or_else(|| s.path.clone())
                .ok_or_else(|| anyhow!("path is required for a document with no existing file"))?;
            io::save_native(&s.doc, Path::new(&path))?;
            s.path = Some(path.clone());
            Ok(json!({"path":path}))
        }
        _ => bail!("unknown tool '{name}'"),
    }
}

fn response(id: Value, result: Value) -> Value {
    json!({"jsonrpc":"2.0","id":id,"result":result})
}
fn main() -> Result<()> {
    let stdin = stdio::stdin();
    let mut out = stdio::stdout().lock();
    let mut server = Server::default();
    for line in stdin.lock().lines() {
        let line = line?;
        if line.trim().is_empty() {
            continue;
        }
        let req: Value = match serde_json::from_str(&line) {
            Ok(v) => v,
            Err(e) => {
                writeln!(
                    out,
                    "{}",
                    response(
                        Value::Null,
                        json!({"error":{"code":-32700,"message":e.to_string()}})
                    )
                )?;
                out.flush()?;
                continue;
            }
        };
        let Some(method) = req.get("method").and_then(Value::as_str) else {
            continue;
        };
        let id = req.get("id").cloned();
        let result = match method {
            "initialize" => Ok(
                json!({"protocolVersion":"2025-03-26","capabilities":{"tools":{}},"serverInfo":{"name":"lusion360","version":env!("CARGO_PKG_VERSION")}}),
            ),
            "tools/list" => Ok(schema()),
            "tools/call" => {
                let p = req.get("params").cloned().unwrap_or(Value::Null);
                let n = p
                    .get("name")
                    .and_then(Value::as_str)
                    .ok_or_else(|| anyhow!("tool name missing"));
                n.and_then(|n|call(&mut server,n,p.get("arguments").cloned().unwrap_or_else(||json!({})))).map(|v|json!({"content":[{"type":"text","text":serde_json::to_string(&v).unwrap()}]}))
            }
            _ => Err(anyhow!("method not found: {method}")),
        };
        if let Some(id) = id {
            let r = match result {
                Ok(v) => response(id, v),
                Err(e) => response(
                    id,
                    json!({"content":[{"type":"text","text":e.to_string()}],"isError":true}),
                ),
            };
            writeln!(out, "{}", r)?;
            out.flush()?;
        }
    }
    Ok(())
}
