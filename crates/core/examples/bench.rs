//! Rough recompute timing on a heavier model.
use glam::DVec2;
use lusion_core::doc::*;
use lusion_core::model::Model;
use lusion_core::sketch::{Constraint, Sketch};
use std::time::Instant;

fn main() {
    let mut doc = Document::default();
    let mut m = Model::new();
    let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
    let p = [s.add_point(DVec2::new(0., 0.)), s.add_point(DVec2::new(200., 0.)), s.add_point(DVec2::new(200., 200.)), s.add_point(DVec2::new(0., 200.))];
    let l: Vec<u32> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
    s.add_constraint(Constraint::Horizontal(l[0]));
    for i in 0..10 {
        for j in 0..10 {
            let c = s.add_point(DVec2::new(10. + i as f64 * 20., 10. + j as f64 * 20.));
            s.add_curve(lusion_core::sketch::CurveGeom::Circle { c, r: 4.0 }, false);
        }
    }
    {
        let mut s2 = s.clone();
        let t = Instant::now();
        s2.solve(&Default::default(), &[], false);
        println!("  solve (no DOF): {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        let t = Instant::now();
        s2.solve(&Default::default(), &[], true);
        println!("  solve + DOF: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        let f = lusion_core::sketch::Frame::new(glam::DVec3::ZERO, glam::DVec3::Z, glam::DVec3::X);
        let edges: Vec<lusion_kernel::Shape> = s2.curves.iter().filter_map(|c| match c.geom {
            lusion_core::sketch::CurveGeom::Line { a, b } => lusion_kernel::line(f.to3(s2.pos(a)), f.to3(s2.pos(b))).ok(),
            lusion_core::sketch::CurveGeom::Circle { c, r } => lusion_kernel::circle(f.to3(s2.pos(c)), f.n, r).ok(),
            _ => None,
        }).collect();
        let t = Instant::now();
        let regions = lusion_kernel::planar_regions(&edges).unwrap();
        println!("  occt regions: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        let t = Instant::now();
        for r in &regions { let _ = r.mesh(2.0, 0.6); }
        println!("  mesh regions: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        let t = Instant::now();
        let p = lusion_core::model::compute_profiles(&s2, &f, 1, &Default::default()).unwrap();
        println!("  profiles: {:.1} ms ({})", t.elapsed().as_secs_f64() * 1e3, p.len());
    }
    let sid = doc.insert_feature("S".into(), ROOT, FeatureKind::Sketch(s));
    let t = Instant::now();
    m.update(&doc);
    println!("sketch with 100 holes: {:.1} ms ({} profiles)", t.elapsed().as_secs_f64() * 1e3, m.current(&doc).sketches[&sid].profiles.len());
    let st = m.current(&doc);
    let outer = st.sketches[&sid].profiles.iter().max_by(|a, b| a.area.partial_cmp(&b.area).unwrap()).unwrap();
    let pr = ProfileRef::Sketch { sketch: sid, curves: outer.curves.clone(), hint: outer.hint.to_array() };
    let e = doc.insert_feature("E".into(), ROOT, FeatureKind::Extrude(Extrude { profiles: vec![pr], distance: "5".into(), distance2: String::new(), mode: ExtentMode::OneSide, start_offset: String::new(), op: Operation::NewBody, targets: vec![] }));
    let t = Instant::now();
    m.update(&doc);
    println!("extrude plate with 100 holes: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
    let bid = *m.current(&doc).bodies.keys().next().unwrap();
    let body = &m.current(&doc).bodies[&bid];
    let edges: Vec<EdgeRef> = body.edge_names.iter().filter(|n| n.contains(&format!("f{e}.end"))).map(|n| EdgeRef { body: bid, name: n.clone(), hint: [0.; 3] }).collect();
    println!("fillet {} edges", edges.len());
    doc.insert_feature("F".into(), ROOT, FeatureKind::Fillet(Fillet { edges, radius: "0.8".into() }));
    let t = Instant::now();
    m.update(&doc);
    println!("fillet: {:.1} ms err={:?}", t.elapsed().as_secs_f64() * 1e3, m.status.last().unwrap().error);
    let body = m.current(&doc).bodies[&bid].clone();
    let t = Instant::now();
    let mesh = lusion_core::tess::BodyMesh::from_shape(&body.shape).unwrap();
    println!("tessellate: {:.1} ms ({} tris)", t.elapsed().as_secs_f64() * 1e3, mesh.indices.len() / 3);
    for (lin, ang) in [(0.17, 0.25), (0.28, 0.35), (0.5, 0.5)] {
        let t = Instant::now();
        let m = body.shape.mesh(lin, ang).unwrap();
        println!("  mesh lin={lin} ang={ang}: {:.1} ms ({} tris)", t.elapsed().as_secs_f64() * 1e3, m.indices.len() / 3);
    }
    // edit the extrude distance → recompute extrude + fillet
    if let FeatureKind::Extrude(x) = &mut doc.features[1].kind {
        x.distance = "8".into();
    }
    let t = Instant::now();
    m.update(&doc);
    println!("edit upstream (extrude+fillet): {:.1} ms err={:?}", t.elapsed().as_secs_f64() * 1e3, m.status.last().unwrap().error);
}

#[allow(dead_code)]
fn breakdown() {}
