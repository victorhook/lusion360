//! Writes a small demo design: `cargo run -p lusion-core --example demo -- out.l360`
use glam::DVec2;
use lusion_core::doc::*;
use lusion_core::expr::Parameter;
use lusion_core::model::Model;
use lusion_core::sketch::{Constraint, CurveGeom, DimKind, Sketch};

fn rect(s: &mut Sketch, x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<u32> {
    let p = [
        s.add_point(DVec2::new(x0, y0)),
        s.add_point(DVec2::new(x1, y0)),
        s.add_point(DVec2::new(x1, y1)),
        s.add_point(DVec2::new(x0, y1)),
    ];
    let l: Vec<u32> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
    s.add_constraint(Constraint::Horizontal(l[0]));
    s.add_constraint(Constraint::Horizontal(l[2]));
    s.add_constraint(Constraint::Vertical(l[1]));
    s.add_constraint(Constraint::Vertical(l[3]));
    l
}

fn dim(kind: DimKind, e: &str) -> Constraint {
    Constraint::Dimension { kind, expr: e.into(), value: 0.0, offset: [0.0, 6.0] }
}

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| "demo.l360".into());
    let mut doc = Document::default();
    doc.components[0].name = "Bracket".into();
    doc.parameters.push(Parameter { name: "width".into(), expr: "80".into(), comment: "plate width".into() });
    doc.parameters.push(Parameter { name: "thick".into(), expr: "6".into(), comment: "plate thickness".into() });
    let mut m = Model::new();

    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let o = s.add_point(DVec2::ZERO);
    s.point_mut(o).unwrap().fixed = true;
    let l = rect(&mut s, -40.0, -25.0, 40.0, 25.0);
    s.add_constraint(dim(DimKind::Length(l[0]), "width"));
    s.add_constraint(dim(DimKind::Length(l[1]), "50"));
    for (cx, cy) in [(-30.0, -15.0), (30.0, -15.0), (-30.0, 15.0), (30.0, 15.0)] {
        let c = s.add_point(DVec2::new(cx, cy));
        s.add_curve(CurveGeom::Circle { c, r: 2.5 }, false);
    }
    let s1 = doc.insert_feature("Base sketch".into(), ROOT, FeatureKind::Sketch(s));
    m.update(&doc);
    let st = m.current(&doc);
    let outer = st.sketches[&s1].profiles.iter().max_by(|a, b| a.area.partial_cmp(&b.area).unwrap()).unwrap();
    let pr = ProfileRef::Sketch { sketch: s1, curves: outer.curves.clone(), hint: outer.hint.to_array() };
    let e1 = doc.insert_feature(
        "Plate".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![pr],
            distance: "thick".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::NewBody,
            targets: vec![],
        }),
    );
    m.update(&doc);
    let bid = *m.current(&doc).bodies.keys().next().unwrap();

    let mut s2 = Sketch::new(PlaneRef::Face(FaceRef { body: bid, name: format!("f{e1}.end"), hint: [0.0; 3] }));
    let c = s2.add_point(DVec2::new(0.0, 0.0));
    let ci = s2.add_curve(CurveGeom::Circle { c, r: 12.0 }, false);
    s2.add_constraint(dim(DimKind::Diameter(ci), "24"));
    let s2id = doc.insert_feature("Boss sketch".into(), ROOT, FeatureKind::Sketch(s2));
    m.update(&doc);
    let p = &m.current(&doc).sketches[&s2id].profiles[0];
    let pr2 = ProfileRef::Sketch { sketch: s2id, curves: p.curves.clone(), hint: p.hint.to_array() };
    doc.insert_feature(
        "Boss".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![pr2],
            distance: "15".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::Join,
            targets: vec![],
        }),
    );
    m.update(&doc);
    let body = &m.current(&doc).bodies[&bid];
    let edges: Vec<EdgeRef> = body
        .edge_names
        .iter()
        .filter(|n| n.matches(&format!("f{e1}.side")).count() == 2)
        .map(|n| EdgeRef { body: bid, name: n.clone(), hint: [0.0; 3] })
        .collect();
    doc.insert_feature("Corners".into(), ROOT, FeatureKind::Fillet(Fillet { edges, radius: "5".into() }));
    m.update(&doc);
    for (i, s) in m.status.iter().enumerate() {
        if let Some(e) = &s.error {
            eprintln!("feature {i}: {e}");
        }
    }
    // like the UI: sketches consumed by features are hidden
    for f in doc.features.iter_mut() {
        if matches!(f.kind, FeatureKind::Sketch(_)) {
            f.visible = false;
        }
    }
    lusion_core::io::save_native(&doc, std::path::Path::new(&out)).unwrap();
    println!("wrote {out}");
}
