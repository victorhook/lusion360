//! A deliberately small, dependency-free MCP stdio server for Lusion360.
//!
//! It owns documents in memory for the duration of the process and uses the
//! same core evaluator as the desktop application.  Keeping this binary free
//! of UI and async runtime dependencies makes tool calls fast and predictable.

use anyhow::{anyhow, bail, Result};
use glam::DVec2;
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
    plane: String,
    #[serde(default)]
    name: Option<String>,
    #[serde(default)]
    rectangles: Vec<Rect>,
    #[serde(default)]
    circles: Vec<Circle>,
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
     {"name":"lusion_add_sketch","description":"Add an XY/XZ/YZ sketch. Rectangles are anchored at x,y; width/height and circle diameter are expressions, preserving parametric editability.","inputSchema":{"type":"object","required":["document","plane"],"properties":{"document":{"type":"string"},"plane":{"enum":["XY","XZ","YZ"]},"name":{"type":"string"},"rectangles":{"type":"array","items":{"type":"object","required":["x","y","width","height"]}},"circles":{"type":"array","items":{"type":"object","required":["x","y","diameter"]}}}}},
     {"name":"lusion_add_extrude","description":"Extrude a solved sketch profile. profile may be largest (default) or index:N. A failed operation is rolled back.","inputSchema":{"type":"object","required":["document","sketch","distance"],"properties":{"document":{"type":"string"},"sketch":{"type":"integer"},"profile":{"type":"string"},"distance":{"type":"string"},"operation":{"enum":["new_body","join","cut","intersect"]},"name":{"type":"string"}}}},
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
        "lusion_add_sketch" => {
            let a: SketchArgs = serde_json::from_value(args)?;
            let s = server
                .docs
                .get_mut(&a.document)
                .ok_or_else(|| anyhow!("unknown document"))?;
            let fid = s.transactional(|d, _| {
                let mut sk = Sketch::new(plane(&a.plane)?);
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
