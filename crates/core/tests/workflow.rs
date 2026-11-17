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

#[test]
fn export_formats_roundtrip() {
    let doc = lusion_core::io::load_native(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/bracket.l360")));
    let doc = match doc {
        Ok(d) => d,
        Err(_) => return, // demo not generated
    };
    let mut model = Model::new();
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    let bodies: Vec<_> = st.bodies.values().collect();
    let dir = std::env::temp_dir().join(format!("l360-export-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for ext in ["step", "stl", "3mf", "obj", "brep"] {
        let p = dir.join(format!("bracket.{ext}"));
        lusion_core::io::export(&p, &bodies, &["Bracket".into()]).unwrap();
        assert!(std::fs::metadata(&p).unwrap().len() > 100, "{ext} too small");
        // re-open what we wrote
        let d2 = lusion_core::io::open_any(&p).unwrap();
        let mut m2 = Model::new();
        m2.update(&d2);
        assert!(!m2.current(&d2).bodies.is_empty(), "{ext} reimport empty");
        if ext == "step" || ext == "brep" {
            let v1 = bodies[0].shape.volume().unwrap();
            let v2: f64 = m2.current(&d2).bodies.values().map(|b| b.shape.volume().unwrap()).sum();
            assert!((v1 - v2).abs() / v1 < 1e-6, "{ext}: {v1} vs {v2}");
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

#[test]
fn pattern_and_mirror() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, 5.0, 0.0, 10.0, 5.0);
    let s1 = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, s1, |_| true);
    doc.insert_feature(
        "E".into(),
        ROOT,
        FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "2".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(&doc);
    ok(&model);
    let bid = *model.current(&doc).bodies.keys().next().unwrap();
    // 6 copies around Z, separate bodies
    doc.insert_feature(
        "P".into(),
        ROOT,
        FeatureKind::Pattern(Pattern { bodies: vec![bid], kind: PatternKind::Circular, axis: AxisRef::Z, count: "6".into(), spacing: String::new(), angle: "360".into(), axis2: None, count2: String::new(), spacing2: String::new(), join: false }),
    );
    model.update(&doc);
    ok(&model);
    assert_eq!(model.current(&doc).bodies.len(), 6);
    // mirror the original across YZ and join
    doc.insert_feature("M".into(), ROOT, FeatureKind::Mirror(Mirror { bodies: vec![bid], plane: PlaneRef::Origin(OriginPlane::YZ), join: true }));
    model.update(&doc);
    ok(&model);
    let v = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!((v - 100.0).abs() < 1e-6, "mirrored+joined volume {v}");
    // linear grid 3x2, joined
    let mut doc2 = doc.clone();
    doc2.features.truncate(2);
    doc2.marker = 2;
    doc2.insert_feature(
        "L".into(),
        ROOT,
        FeatureKind::Pattern(Pattern { bodies: vec![bid], kind: PatternKind::Linear, axis: AxisRef::X, count: "3".into(), spacing: "10".into(), angle: String::new(), axis2: Some(AxisRef::Y), count2: "2".into(), spacing2: "10".into(), join: true }),
    );
    let mut m2 = Model::new();
    m2.update(&doc2);
    ok(&m2);
    let v = m2.current(&doc2).bodies[&bid].shape.volume().unwrap();
    assert!((v - 300.0).abs() < 1e-6, "grid volume {v}");
}

#[test]
fn shell_and_chamfer() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, 0.0, 0.0, 30.0, 20.0);
    let s1 = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, s1, |_| true);
    let e1 = doc.insert_feature(
        "E".into(),
        ROOT,
        FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "10".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(&doc);
    let bid = *model.current(&doc).bodies.keys().next().unwrap();
    // chamfer all top edges
    let body = &model.current(&doc).bodies[&bid];
    let top = format!("f{e1}.end");
    let edges: Vec<EdgeRef> = body.edge_names.iter().filter(|n| n.contains(&top)).map(|n| EdgeRef { body: bid, name: n.clone(), hint: [0.0; 3] }).collect();
    assert_eq!(edges.len(), 4);
    doc.insert_feature("C".into(), ROOT, FeatureKind::Chamfer(Chamfer { edges, distance: "1".into() }));
    model.update(&doc);
    ok(&model);
    // shell, removing the bottom face
    doc.insert_feature(
        "Sh".into(),
        ROOT,
        FeatureKind::Shell(Shell { body: Some(bid), faces: vec![FaceRef { body: bid, name: format!("f{e1}.start"), hint: [0.0; 3] }], thickness: "1.5".into() }),
    );
    model.update(&doc);
    ok(&model);
    let v = model.current(&doc).bodies[&bid].shape.volume().unwrap();
    assert!(v > 0.0 && v < 30.0 * 20.0 * 10.0 * 0.5, "shelled volume {v}");
}

#[test]
fn text_emboss() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let a = s.add_point(DVec2::new(0.0, 0.0));
    s.add_text(a, "Hi8", "10");
    let s1 = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    ok(&model);
    let ss = &model.current(&doc).sketches[&s1];
    // H, i (stem + dot), 8 (outer + two holes = outer region + 2 hole regions)
    assert!(ss.profiles.len() >= 4, "profiles {}", ss.profiles.len());
    let refs: Vec<ProfileRef> = ss
        .profiles
        .iter()
        .map(|p| ProfileRef::Sketch { sketch: s1, curves: p.curves.clone(), hint: p.hint.to_array() })
        .collect();
    // extrude only the solid glyph regions: the hole regions of "8" are inside another region
    doc.insert_feature(
        "E".into(),
        ROOT,
        FeatureKind::Extrude(Extrude { profiles: refs, distance: "2".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(&doc);
    ok(&model);
    assert!(!model.current(&doc).bodies.is_empty());
}

#[test]
fn multiple_extrusions_from_one_sketch() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, 0.0, 0.0, 20.0, 20.0);
    let c = s.add_point(DVec2::new(10.0, 10.0));
    s.add_curve(CurveGeom::Circle { c, r: 4.0 }, false);
    let s1 = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let outer = profile_ref(&model, &doc, s1, |p| p.curves.len() == 5);
    let disk = profile_ref(&model, &doc, s1, |p| p.curves.len() == 1);
    let ex = |profiles, d: &str, start: &str, op| {
        FeatureKind::Extrude(Extrude { profiles, distance: d.into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: start.into(), op, targets: vec![] })
    };
    // plate 0..3, then a pin from the same sketch starting at 3 going to 13
    doc.insert_feature("Plate".into(), ROOT, ex(vec![outer], "3", "", Operation::NewBody));
    doc.insert_feature("Pin".into(), ROOT, ex(vec![disk], "10", "3", Operation::NewBody));
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    assert_eq!(st.bodies.len(), 2);
    let zs: Vec<(f64, f64)> = st.bodies.values().map(|b| { let (lo, hi) = b.shape.bbox().unwrap(); (lo.z, hi.z) }).collect();
    assert!(zs.iter().any(|(lo, hi)| (lo - 0.0).abs() < 1e-6 && (hi - 3.0).abs() < 1e-6));
    assert!(zs.iter().any(|(lo, hi)| (lo - 3.0).abs() < 1e-6 && (hi - 13.0).abs() < 1e-6));
}

/// Every edge used by exactly two triangles in opposite directions (closed, oriented, manifold).
fn manifold_errors(idx: &[u32]) -> usize {
    use std::collections::HashMap;
    let mut dir: HashMap<(u32, u32), i32> = HashMap::new();
    for t in idx.chunks_exact(3) {
        for k in 0..3 {
            *dir.entry((t[k], t[(k + 1) % 3])).or_default() += 1;
        }
    }
    let mut bad = 0;
    for (&(a, b), &n) in &dir {
        if n != 1 || dir.get(&(b, a)).copied().unwrap_or(0) != 1 {
            bad += 1;
        }
    }
    bad
}

#[test]
fn exports_are_manifold() {
    // donut: circle on XZ revolved around Z
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XZ));
    let c = s.add_point(DVec2::new(30.0, 5.0));
    s.add_curve(CurveGeom::Circle { c, r: 5.0 }, false);
    let s1 = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, s1, |_| true);
    doc.insert_feature("R".into(), ROOT, FeatureKind::Revolve(Revolve { profiles: vec![pr], axis: AxisRef::Z, angle: "360".into(), op: Operation::NewBody, targets: vec![] }));
    model.update(&doc);
    ok(&model);
    let donut: Vec<_> = model.current(&doc).bodies.values().collect();
    let bracket_doc = lusion_core::io::load_native(std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../examples/bracket.l360"))).ok();
    let mut bm = Model::new();
    let mut cases: Vec<(&str, Vec<&lusion_core::model::Body>)> = vec![("donut", donut)];
    if let Some(d) = &bracket_doc {
        bm.update(d);
        cases.push(("bracket", bm.current(d).bodies.values().collect()));
    }
    let dir = std::env::temp_dir().join(format!("l360-manifold-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    for (name, bodies) in cases {
        // OBJ: parse the file exactly like a slicer would (faces index `v` lines)
        let p = dir.join(format!("{name}.obj"));
        lusion_core::io::export(&p, &bodies, &[name.into()]).unwrap();
        let txt = std::fs::read_to_string(&p).unwrap();
        let mut idx = Vec::new();
        for l in txt.lines().filter(|l| l.starts_with("f ")) {
            for c in l.split_whitespace().skip(1) {
                idx.push(c.split('/').next().unwrap().parse::<u32>().unwrap() - 1);
            }
        }
        assert!(!idx.is_empty());
        assert_eq!(manifold_errors(&idx), 0, "{name}.obj is not manifold");
        // 3MF: inspect the model XML triangles
        let p = dir.join(format!("{name}.3mf"));
        lusion_core::io::export(&p, &bodies, &[name.into()]).unwrap();
        let meshes = lusion_meshio::load_mesh_file(&p).unwrap();
        // loader may split vertices for normals; re-weld by position before checking
        for m in meshes {
            let (_, idx) = lusion_core::io::weld(&m.positions, &m.indices);
            assert_eq!(manifold_errors(&idx), 0, "{name}.3mf is not manifold");
        }
    }
    let _ = std::fs::remove_dir_all(&dir);
}

/// Interactive (background) recompute must give exactly the synchronous result,
/// keep showing the old state while it runs, and hand over display meshes.
#[test]
fn background_recompute_matches_sync() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, 0.0, 0.0, 40.0, 20.0);
    let s1 = doc.insert_feature("Sketch1".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, s1, |_| true);
    let e1 = doc.insert_feature(
        "Extrude1".into(),
        ROOT,
        FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "10".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(&doc);
    let body = model.current(&doc).bodies.values().next().unwrap().clone();
    let edge = body.edge_names.iter().find(|n| n.contains(&format!("f{e1}.side")) && n.matches("side").count() == 2).cloned().unwrap();
    doc.insert_feature("Fillet1".into(), ROOT, FeatureKind::Fillet(Fillet { edges: vec![EdgeRef { body: body.id, name: edge.clone(), hint: [0.0; 3] }], radius: "2".into() }));
    model.update(&doc);
    ok(&model);
    let v2 = model.current(&doc).bodies[&body.id].shape.volume().unwrap();

    // drag the radius: budget 0 forces the background path
    if let FeatureKind::Fillet(f) = &mut doc.features.last_mut().unwrap().kind {
        f.radius = "5".into();
    }
    model.update_interactive(&doc, 0.0);
    assert!(model.busy(), "edit of a known feature should run in the background");
    // meanwhile the old result is still shown
    assert!((model.current(&doc).bodies[&body.id].shape.volume().unwrap() - v2).abs() < 1e-9);
    let t0 = std::time::Instant::now();
    while model.busy() {
        assert!(t0.elapsed().as_secs() < 20, "background recompute hangs");
        std::thread::sleep(std::time::Duration::from_millis(2));
        model.update_interactive(&doc, 0.0);
    }
    ok(&model);
    let v5 = model.current(&doc).bodies[&body.id].shape.volume().unwrap();
    let mut sync = Model::new();
    sync.update(&doc);
    let v5s = sync.current(&doc).bodies[&body.id].shape.volume().unwrap();
    assert!((v5 - v5s).abs() < 1e-9, "background {v5} vs sync {v5s}");
    assert!(v5 < v2);
    assert!(!model.premeshed.is_empty(), "the worker tessellates the new body");
    // untouched prefix states keep their original shapes (warm caches)
    assert!(model.state_after(2).bodies[&body.id].shape.same(&body.shape));
    // a later sync update is a no-op
    assert!(!model.update(&doc));
}

fn boxed(doc: &mut Document, model: &mut Model, w: f64, h: f64, d: f64) -> (FeatureId, BodyId) {
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s, 0.0, 0.0, w, h);
    let sid = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(doc);
    let pr = profile_ref(model, doc, sid, |_| true);
    let e = doc.insert_feature(
        "E".into(),
        ROOT,
        FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: d.to_string(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }),
    );
    model.update(doc);
    ok(model);
    let bid = *model.current(doc).bodies.keys().last().unwrap();
    (e, bid)
}

fn vol(model: &Model, doc: &Document, b: BodyId) -> f64 {
    model.current(doc).bodies[&b].shape.volume().unwrap()
}

#[test]
fn planes_and_split() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let (e, bid) = boxed(&mut doc, &mut model, 40.0, 20.0, 10.0);
    // plane at 30° through the X axis, midplane between XY and the box top, plane normal to an edge
    let p1 = doc.insert_feature("P1".into(), ROOT, FeatureKind::Plane(OffsetPlane { base: PlaneRef::Origin(OriginPlane::XY), offset: "0".into(), mode: PlaneMode::Angle { axis: AxisRef::X, angle: "30".into() } }));
    let top = FaceRef { body: bid, name: format!("f{e}.end"), hint: [20.0, 10.0, 10.0] };
    let p2 = doc.insert_feature("P2".into(), ROOT, FeatureKind::Plane(OffsetPlane { base: PlaneRef::Origin(OriginPlane::XY), offset: "0".into(), mode: PlaneMode::Midplane { other: PlaneRef::Face(top) } }));
    let body = model.current(&doc).bodies[&bid].clone();
    let long_edge = body.edge_names.iter().zip(body.shape.edges()).find(|(_, e)| e.edge_info().map(|i| (i.length - 40.0).abs() < 1e-6).unwrap_or(false)).map(|(n, _)| n.clone()).unwrap();
    let p3 = doc.insert_feature("P3".into(), ROOT, FeatureKind::Plane(OffsetPlane { base: PlaneRef::Origin(OriginPlane::XY), offset: "0".into(), mode: PlaneMode::NormalToEdge { edge: EdgeRef { body: bid, name: long_edge, hint: [0.0; 3] }, at: "25".into() } }));
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    let n1 = st.planes[&p1].n;
    assert!((n1.angle_between(glam::DVec3::Z).to_degrees() - 30.0).abs() < 1e-6, "angle plane normal {n1:?}");
    assert!((st.planes[&p2].origin.z - 5.0).abs() < 1e-9);
    assert!((st.planes[&p3].n.x.abs() - 1.0).abs() < 1e-9 && (st.planes[&p3].origin.x - 10.0).abs() < 1e-6 || (st.planes[&p3].origin.x - 30.0).abs() < 1e-6);
    // split the box with the midplane: two bodies, volumes add up
    doc.insert_feature("Split".into(), ROOT, FeatureKind::Split(Split { bodies: vec![bid], tool: PlaneRef::Plane(p2), keep: SplitKeep::Both }));
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    assert_eq!(st.bodies.len(), 2);
    for b in st.bodies.values() {
        assert!((b.shape.volume().unwrap() - 4000.0).abs() < 1e-6);
        assert!(b.face_names.iter().any(|n| n.contains(".cut")), "cut face named: {:?}", b.face_names);
    }
    // keep only the front half
    if let FeatureKind::Split(s) = &mut doc.features.last_mut().unwrap().kind {
        s.keep = SplitKeep::Front;
    }
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    assert_eq!(st.bodies.len(), 1);
    assert!(st.bodies.values().next().unwrap().shape.bbox().unwrap().0.z > 4.99);
}

#[test]
fn holes_and_threads() {
    let mut doc = Document::default();
    let mut model = Model::new();
    let (e, bid) = boxed(&mut doc, &mut model, 40.0, 20.0, 10.0);
    let top = FaceRef { body: bid, name: format!("f{e}.end"), hint: [20.0, 10.0, 10.0] };
    let v0 = vol(&model, &doc, bid);
    // face frame origin = projection of the world origin: point (x, y) in face coords = (x, y) world here
    let mk = |kind, pts: Vec<[f64; 2]>| {
        FeatureKind::Hole(Hole {
            face: top.clone(),
            points: pts,
            sketch: None,
            kind,
            diameter: "5".into(),
            depth: "6".into(),
            through: false,
            cb_diameter: "9".into(),
            cb_depth: "3".into(),
            cs_diameter: "10".into(),
            cs_angle: "90".into(),
            drill_point: false,
        })
    };
    doc.insert_feature("H1".into(), ROOT, mk(HoleKind::Simple, vec![[5.0, 5.0], [15.0, 5.0]]));
    model.update(&doc);
    ok(&model);
    let v1 = vol(&model, &doc, bid);
    let pi = std::f64::consts::PI;
    assert!((v0 - v1 - 2.0 * pi * 6.25 * 6.0).abs() < 1e-3, "two simple holes: {}", v0 - v1);
    doc.insert_feature("H2".into(), ROOT, mk(HoleKind::Counterbore, vec![[25.0, 10.0]]));
    model.update(&doc);
    ok(&model);
    let v2 = vol(&model, &doc, bid);
    let cb = pi * 20.25 * 3.0 + pi * 6.25 * 3.0;
    assert!((v1 - v2 - cb).abs() < 1e-3, "counterbore: {}", v1 - v2);
    doc.insert_feature("H3".into(), ROOT, mk(HoleKind::Countersink, vec![[35.0, 10.0]]));
    model.update(&doc);
    ok(&model);
    assert!(vol(&model, &doc, bid) < v2);

    // external M6 thread on a 6 mm boss
    let mut s = Sketch::new(PlaneRef::Face(top.clone()));
    let c = s.add_point(DVec2::new(10.0, 15.0));
    s.add_curve(CurveGeom::Circle { c, r: 3.0 }, false);
    let sid = doc.insert_feature("S2".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    ok(&model);
    let pr = profile_ref(&model, &doc, sid, |p| p.area < 50.0);
    let e2 = doc.insert_feature("Boss".into(), ROOT, FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "12".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::Join, targets: vec![] }));
    model.update(&doc);
    ok(&model);
    let body = model.current(&doc).bodies[&bid].clone();
    let side = body.face_names.iter().find(|n| n.starts_with(&format!("f{e2}.side"))).cloned().unwrap();
    let vb = vol(&model, &doc, bid);

    doc.insert_feature("Thread".into(), ROOT, FeatureKind::Thread(Thread { face: FaceRef { body: bid, name: side, hint: [0.0; 3] }, size: "6".into(), pitch: "1".into(), length: String::new(), clearance: "0".into(), left: false, flip: false }));
    let t0 = std::time::Instant::now();
    model.update(&doc);
    ok(&model);
    let vt = vol(&model, &doc, bid);
    // removed groove ≈ π·D·depth·(mean groove width / pitch) · length ≈ π·6·0.61·0.52·12
    assert!(vb - vt > 30.0 && vb - vt < 120.0, "thread removed {} mm³ (in {:?})", vb - vt, t0.elapsed());
    // printable: valid B-rep and a watertight mesh
    let sh = &model.current(&doc).bodies[&bid].shape;
    assert!(sh.is_valid(), "threaded body is valid");
    let m = lusion_core::tess::BodyMesh::from_shape(sh).unwrap();
    let (_, idx) = lusion_core::io::weld(&m.positions, &m.indices);
    assert_eq!(manifold_errors(&idx), 0, "threaded body meshes watertight");
}

#[test]
fn sweep_loft_and_joint() {
    let mut doc = Document::default();
    let mut model = Model::new();
    // sweep a circle (XZ plane) along a sketch line in XY... path: line from origin along +Y
    let mut ps = Sketch::new(PlaneRef::Origin(OriginPlane::YZ));
    let a = ps.add_point(DVec2::new(0.0, 0.0));
    let b = ps.add_point(DVec2::new(30.0, 0.0));
    let line = ps.add_line(a, b);
    let path_sk = doc.insert_feature("Path".into(), ROOT, FeatureKind::Sketch(ps));
    let mut cs = Sketch::new(PlaneRef::Origin(OriginPlane::XZ));
    let c = cs.add_point(DVec2::new(0.0, 0.0));
    cs.add_curve(CurveGeom::Circle { c, r: 2.0 }, false);
    let prof_sk = doc.insert_feature("Prof".into(), ROOT, FeatureKind::Sketch(cs));
    model.update(&doc);
    ok(&model);
    let pr = profile_ref(&model, &doc, prof_sk, |_| true);
    let sw = doc.insert_feature("Sweep".into(), ROOT, FeatureKind::Sweep(Sweep { profiles: vec![pr], path: vec![PathRef::SketchCurve { sketch: path_sk, curve: line }], op: Operation::NewBody, targets: vec![], frenet: false }));
    model.update(&doc);
    ok(&model);
    let tube = *model.current(&doc).bodies.keys().next().unwrap();
    assert!((vol(&model, &doc, tube) - std::f64::consts::PI * 4.0 * 30.0).abs() < 1e-3);
    assert!(model.current(&doc).bodies[&tube].face_names.iter().any(|n| n == &format!("f{sw}.end")));

    // loft from a 20x20 square at z=0 to a 10x10 square on a plane at z=10
    let mut s1 = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    rect(&mut s1, 50.0, 0.0, 70.0, 20.0);
    let k1 = doc.insert_feature("L1".into(), ROOT, FeatureKind::Sketch(s1));
    let pl = doc.insert_feature("PL".into(), ROOT, FeatureKind::Plane(OffsetPlane { base: PlaneRef::Origin(OriginPlane::XY), offset: "10".into(), mode: PlaneMode::Offset }));
    let mut s2 = Sketch::new(PlaneRef::Plane(pl));
    rect(&mut s2, 55.0, 5.0, 65.0, 15.0);
    let k2 = doc.insert_feature("L2".into(), ROOT, FeatureKind::Sketch(s2));
    model.update(&doc);
    ok(&model);
    let r1 = profile_ref(&model, &doc, k1, |_| true);
    let r2 = profile_ref(&model, &doc, k2, |_| true);
    let lf = doc.insert_feature("Loft".into(), 1, FeatureKind::Loft(Loft { profiles: vec![r1, r2], ruled: true, op: Operation::NewBody, targets: vec![] }));
    // the loft lives in a second component so the joint can move it
    let comp = doc.add_component("Part2", ROOT);
    doc.feature_mut(lf).unwrap().component = comp;
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    let frustum = st.bodies.values().find(|b| b.id.feature() == lf).unwrap().clone();
    assert!((frustum.shape.volume().unwrap() - 10.0 / 3.0 * (400.0 + 100.0 + 200.0)).abs() < 1e-3);

    // joint: put the loft's bottom face onto the tube's end cap
    let bottom = frustum.face_names.iter().find(|n| n.ends_with(".start")).cloned().unwrap();
    let cap = model.current(&doc).bodies[&tube].face_names.iter().find(|n| n.ends_with(".end")).cloned().unwrap();
    doc.insert_feature(
        "Joint".into(),
        ROOT,
        FeatureKind::Joint(Joint {
            moving: comp,
            a: JointRef::Face(FaceRef { body: frustum.id, name: bottom, hint: [0.0; 3] }),
            b: JointRef::Face(FaceRef { body: tube, name: cap, hint: [0.0; 3] }),
            kind: JointKind::Revolute,
            angle: "45".into(),
            offset: "0".into(),
            flip: false,
        }),
    );
    model.update(&doc);
    ok(&model);
    let st = model.current(&doc);
    let moved = &st.bodies[&frustum.id];
    let (lo, hi) = moved.shape.bbox().unwrap();
    // the tube ends at y = 30; the frustum now starts there and grows along +y
    assert!((lo.y - 30.0).abs() < 1e-6, "joined at the cap: {lo:?} {hi:?}");
    assert!((hi.y - 40.0).abs() < 1e-6);
    assert!((moved.shape.volume().unwrap() - frustum.shape.volume().unwrap()).abs() < 1e-6);
}

#[test]
fn splines_make_profiles_and_paths() {
    let mut doc = Document::default();
    let mut model = Model::new();
    // closed spline through 4 points (a rounded blob)
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let pts: Vec<u32> = [(0.0, -10.0), (10.0, 0.0), (0.0, 10.0), (-10.0, 0.0)].iter().map(|(x, y)| s.add_point(DVec2::new(*x, *y))).collect();
    s.add_spline(pts, true);
    // open spline closed by a line
    let a = s.add_point(DVec2::new(30.0, 0.0));
    let m = s.add_point(DVec2::new(40.0, 8.0));
    let b = s.add_point(DVec2::new(50.0, 0.0));
    s.add_spline(vec![a, m, b], false);
    s.add_line(b, a);
    let sid = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    ok(&model);
    let ss = model.current(&doc).sketches[&sid].clone();
    assert_eq!(ss.profiles.len(), 2, "blob + arch");
    let blob = ss.profiles.iter().find(|p| p.hint.x < 20.0).unwrap();
    // a Catmull-Rom loop through (±10, 0), (0, ±10) bulges between circle-ish and the square
    assert!(blob.area > 200.0 && blob.area < 400.0, "blob area {}", blob.area);
    let pr = profile_ref(&model, &doc, sid, |p| p.hint.x < 20.0);
    doc.insert_feature("E".into(), ROOT, FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "5".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }));
    model.update(&doc);
    ok(&model);
    let body = model.current(&doc).bodies.values().next().unwrap().clone();
    assert!((body.shape.volume().unwrap() - blob.area * 5.0).abs() < 1e-2, "volume {} vs area*5 {}", body.shape.volume().unwrap(), blob.area * 5.0);
    assert!(body.face_names.iter().any(|n| n.contains(".side(s")), "spline side face named: {:?}", body.face_names);

    // a spline as sweep path
    let mut ps = Sketch::new(PlaneRef::Origin(OriginPlane::YZ));
    let p0 = ps.add_point(DVec2::new(0.0, 20.0));
    let p1 = ps.add_point(DVec2::new(15.0, 30.0));
    let p2 = ps.add_point(DVec2::new(30.0, 25.0));
    let path = ps.add_spline(vec![p0, p1, p2], false);
    let psk = doc.insert_feature("P".into(), ROOT, FeatureKind::Sketch(ps));
    let mut cs = Sketch::new(PlaneRef::Origin(OriginPlane::XZ));
    let c = cs.add_point(DVec2::new(0.0, 20.0));
    cs.add_curve(CurveGeom::Circle { c, r: 1.5 }, false);
    let csk = doc.insert_feature("C".into(), ROOT, FeatureKind::Sketch(cs));
    model.update(&doc);
    ok(&model);
    let cpr = profile_ref(&model, &doc, csk, |_| true);
    doc.insert_feature("Sw".into(), ROOT, FeatureKind::Sweep(Sweep { profiles: vec![cpr], path: vec![PathRef::SketchCurve { sketch: psk, curve: path }], op: Operation::NewBody, targets: vec![], frenet: false }));
    model.update(&doc);
    ok(&model);
    assert_eq!(model.current(&doc).bodies.len(), 2);
}

#[test]
fn linked_projection_follows_the_model() {
    let mut doc = Document::default();
    doc.parameters.push(lusion_core::expr::Parameter { name: "w".into(), expr: "40".into(), comment: String::new() });
    let mut model = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let l = rect(&mut s, 0.0, 0.0, 40.0, 20.0);
    s.add_constraint(Constraint::Dimension { kind: DimKind::Length(l[0]), expr: "w".into(), value: 0.0, offset: [0.0; 2] });
    // pin the corner at the origin
    let p0 = match s.curve(l[0]).unwrap().geom { CurveGeom::Line { a, .. } => a, _ => unreachable!() };
    s.point_mut(p0).unwrap().fixed = true;
    let sid = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    model.update(&doc);
    let pr = profile_ref(&model, &doc, sid, |_| true);
    let e = doc.insert_feature("E".into(), ROOT, FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "10".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }));
    model.update(&doc);
    ok(&model);
    let body = model.current(&doc).bodies.values().next().unwrap().clone();
    // the top edge at x = w (vertical side face of the right edge meets the top)
    let (ei, ename) = body
        .edge_names
        .iter()
        .enumerate()
        .find(|(i, n)| n.contains(&format!("f{e}.end")) && body.shape.edges()[*i].edge_info().map(|inf| (inf.start.x - 40.0).abs() < 1e-6 && (inf.end.x - 40.0).abs() < 1e-6).unwrap_or(false))
        .map(|(i, n)| (i, n.clone()))
        .unwrap();
    let _ = ei;
    // sketch on top with a linked projection of that edge
    let top = FaceRef { body: body.id, name: format!("f{e}.end"), hint: [20.0, 10.0, 10.0] };
    let mut s2 = Sketch::new(PlaneRef::Face(top));
    let a = s2.add_point(DVec2::new(40.0, 0.0));
    let b = s2.add_point(DVec2::new(40.0, 20.0));
    s2.point_mut(a).unwrap().fixed = true;
    s2.point_mut(b).unwrap().fixed = true;
    s2.add_line(a, b);
    s2.links.push(lusion_core::sketch::SkLink { edge: EdgeRef { body: body.id, name: ename, hint: [40.0, 10.0, 10.0] }, points: vec![(a, lusion_core::sketch::LinkRole::Start), (b, lusion_core::sketch::LinkRole::End)], radius: None });
    let s2id = doc.insert_feature("S2".into(), ROOT, FeatureKind::Sketch(s2));
    model.update(&doc);
    ok(&model);
    let xs = |model: &Model, doc: &Document| {
        let ss = &model.current(doc).sketches[&s2id];
        let p = ss.sketch.pos(a);
        ss.frame.to3(p).x
    };
    assert!((xs(&model, &doc) - 40.0).abs() < 1e-6);
    // widen the box: the projected line follows
    doc.parameters[0].expr = "55".into();
    model.update(&doc);
    ok(&model);
    assert!((xs(&model, &doc) - 55.0).abs() < 1e-6, "linked point moved to {}", xs(&model, &doc));
}
