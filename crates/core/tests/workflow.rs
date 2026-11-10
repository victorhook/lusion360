use glam::{DVec2, DVec3};
use lusion_core::doc::*;
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

fn profile_ref(model: &Model, doc: &Document, sketch: FeatureId, pick: impl Fn(&lusion_core::model::Profile) -> bool) -> ProfileRef {
    let st = model.current(doc);
    let ss = &st.sketches[&sketch];
    let p = ss.profiles.iter().find(|p| pick(p)).expect("profile");
    ProfileRef::Sketch { sketch, curves: p.curves.clone(), hint: p.hint.to_array() }
}

fn ok(model: &Model) {
    for (i, s) in model.status.iter().enumerate() {
        assert!(s.error.is_none(), "feature {i} failed: {:?}", s.error);
    }
}

#[test]
fn parametric_box_with_boss_and_fillet() {
    let mut doc = Document::default();
    doc.parameters.push(lusion_core::expr::Parameter { name: "h".into(), expr: "10".into(), comment: String::new() });
    let mut model = Model::new();

    // Sketch 1: 40x20 rectangle with dimensions
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let l = rect(&mut s, 0.0, 0.0, 40.0, 20.0);
    s.add_constraint(Constraint::Dimension { kind: DimKind::Length(l[0]), expr: "40".into(), value: 0.0, offset: [0.0; 2] });
    let s1 = doc.insert_feature("Sketch1".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    ok(&model);
    let pr = profile_ref(&model, &doc, s1, |_| true);

    // Extrude 1: h
    let e1 = doc.insert_feature(
        "Extrude1".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![pr],
            distance: "h".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::NewBody,
            targets: vec![],
        }),
    );
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    assert_eq!(st.bodies.len(), 1);
    let body = st.bodies.values().next().unwrap();
    let bid = body.id;
    assert!((body.shape.volume().unwrap() - 40.0 * 20.0 * 10.0).abs() < 1e-6);
    let top = format!("f{e1}.end");
    assert!(body.face_index(&top).is_some(), "names: {:?}", body.face_names);

    // Sketch 2 on the top face: circle
    let mut s2 = Sketch::new(PlaneRef::Face(FaceRef { body: bid, name: top.clone(), hint: [20.0, 10.0, 10.0] }));
    let c = s2.add_point(DVec2::new(20.0, 10.0));
    let ci = s2.add_curve(CurveGeom::Circle { c, r: 5.0 }, false);
    s2.add_constraint(Constraint::Dimension { kind: DimKind::Diameter(ci), expr: "8".into(), value: 0.0, offset: [0.0; 2] });
    let s2id = doc.insert_feature("Sketch2".into(), ROOT, FeatureKind::Sketch(s2));
    model.update(&doc);
    ok(&model);
    let pr2 = profile_ref(&model, &doc, s2id, |_| true);
    {
        let ss = &model.current(&doc).sketches[&s2id];
        assert!((ss.frame.origin.z - 10.0).abs() < 1e-9);
        assert_eq!(ss.profiles.len(), 1);
    }

    // Extrude 2: join a 5mm boss
    doc.insert_feature(
        "Extrude2".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![pr2],
            distance: "5".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::Join,
            targets: vec![],
        }),
    );
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    assert_eq!(st.bodies.len(), 1, "boss joined into the box");
    let body = &st.bodies[&bid];
    let v = 40.0 * 20.0 * 10.0 + std::f64::consts::PI * 16.0 * 5.0;
    assert!((body.shape.volume().unwrap() - v).abs() < 1e-3, "volume {}", body.shape.volume().unwrap());

    // Fillet a vertical edge of the box: between two side faces of Extrude1
    let edge_name = body
        .edge_names
        .iter()
        .find(|n| n.contains(&format!("f{e1}.side")) && n.matches("side").count() == 2)
        .cloned()
        .expect("vertical edge");
    doc.insert_feature(
        "Fillet1".into(),
        ROOT,
        FeatureKind::Fillet(Fillet { edges: vec![EdgeRef { body: bid, name: edge_name.clone(), hint: [0.0; 3] }], radius: "2".into() }),
    );
    model.update(&doc);
    ok(&model);
    let v_fillet = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!(v_fillet < v);

    // Change the base height through the parameter: everything downstream must follow.
    doc.parameters[0].expr = "25".into();
    model.update(&doc);
    ok(&model);
    for s in &model.status {
        assert!(s.warnings.iter().all(|w| !w.contains("lost")), "names were lost: {:?}", s.warnings);
    }
    let st = model.current(&doc);
    let body = &st.bodies[&bid];
    let bb = body.shape.bbox().unwrap();
    assert!((bb.1.z - 30.0).abs() < 1e-6, "boss sits on the taller box: {:?}", bb);
    assert!(body.shape.volume().unwrap() > v_fillet);

    // Change the sketch (width 40 → 60): the fillet edge must still resolve.
    if let FeatureKind::Sketch(s) = &mut doc.features[0].kind {
        if let Constraint::Dimension { expr, .. } = &mut s.constraints.last_mut().unwrap().c {
            *expr = "60".into();
        }
    }
    model.update(&doc);
    ok(&model);
    let bb = model.current(&doc).bodies[&bid].shape.bbox().unwrap();
    assert!((bb.1.x - bb.0.x - 60.0).abs() < 1e-6);
    for s in &model.status {
        assert!(s.warnings.iter().all(|w| !w.contains("lost")), "names were lost: {:?}", s.warnings);
    }

    // Roll back the timeline to after Extrude1
    doc.marker = 2;
    model.update(&doc);
    assert_eq!(model.current(&doc).bodies.len(), 1);
    assert!(model.current(&doc).sketches.get(&s2id).is_none());

    // Save / load roundtrip
    doc.marker = doc.features.len();
    let json = doc.to_json().unwrap();
    let doc2 = Document::from_json(&json).unwrap();
    assert_eq!(doc, doc2);
    let mut m2 = Model::new();
    m2.update(&doc2);
    ok(&m2);
    let a = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    let b = m2.current(&doc2).bodies[&bid].shape.volume().unwrap();
    assert!((a - b).abs() < 1e-9);
}

#[test]
fn cut_through_and_press_pull() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, -10.0, -10.0, 10.0, 10.0);
    let c = s.add_point(DVec2::new(0.0, 0.0));
    s.add_curve(CurveGeom::Circle { c, r: 3.0 }, false);
    let s1 = doc.insert_feature("Sketch1".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    ok(&model);
    assert_eq!(model.current(&doc).sketches[&s1].profiles.len(), 2);
    // extrude the outer region only (plate with hole)
    let outer = profile_ref(&model, &doc, s1, |p| p.curves.len() == 5);
    let e1 = doc.insert_feature(
        "Extrude1".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![outer],
            distance: "4".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::NewBody,
            targets: vec![],
        }),
    );
    model.update(&doc);
    ok(&model);
    let bid = *model.current(&doc).bodies.keys().next().unwrap();
    let vol = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!((vol - (400.0 - std::f64::consts::PI * 9.0) * 4.0).abs() < 1e-3);

    // press-pull the top face by 2mm (join)
    doc.insert_feature(
        "Extrude2".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![ProfileRef::Face(FaceRef { body: bid, name: format!("f{e1}.end"), hint: [0.0; 3] })],
            distance: "2".into(),
            distance2: String::new(),
            mode: ExtentMode::OneSide,
            start_offset: String::new(),
            op: Operation::Join,
            targets: vec![],
        }),
    );
    model.update(&doc);
    ok(&model);
    let vol2 = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!((vol2 - vol * 1.5).abs() < 1e-3, "{vol2}");

    // a symmetric cut slot through everything on the XZ plane
    let mut s3 = Sketch::new(PlaneRef::Origin(OriginPlane::XZ));
    rect(&mut s3, -2.0, 1.0, 2.0, 10.0);
    let s3id = doc.insert_feature("Sketch3".into(), ROOT, FeatureKind::Sketch(s3));
    model.update(&doc);
    ok(&model);
    let pr = profile_ref(&model, &doc, s3id, |_| true);
    doc.insert_feature(
        "Cut".into(),
        ROOT,
        FeatureKind::Extrude(Extrude {
            profiles: vec![pr],
            distance: "50".into(),
            distance2: String::new(),
            mode: ExtentMode::Symmetric,
            start_offset: String::new(),
            op: Operation::Cut,
            targets: vec![],
        }),
    );
    model.update(&doc);
    ok(&model);
    let vol3 = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!(vol3 < vol2);
    // revolve test: sketch on XZ, revolve around Z
    let mut s4 = Sketch::new(PlaneRef::Origin(OriginPlane::XZ));
    rect(&mut s4, 20.0, 0.0, 25.0, 5.0);
    let s4id = doc.insert_feature("Sketch4".into(), ROOT, FeatureKind::Sketch(s4));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, s4id, |_| true);
    doc.insert_feature(
        "Revolve".into(),
        ROOT,
        FeatureKind::Revolve(Revolve { profiles: vec![pr], axis: AxisRef::Z, angle: "360".into(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(&doc);
    ok(&model);
    assert_eq!(model.current(&doc).bodies.len(), 2);
    let ring = model.current(&doc).bodies.values().find(|b| b.id != bid).unwrap();
    let expect = std::f64::consts::PI * (25.0f64.powi(2) - 20.0f64.powi(2)) * 5.0;
    assert!((ring.shape.volume().unwrap() - expect).abs() < 1e-3);
    let _ = DVec3::ZERO;
}
