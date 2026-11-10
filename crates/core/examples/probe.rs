fn main() {
    let doc = lusion_core::io::load_native(std::path::Path::new("examples/bracket.l360")).unwrap();
    let mut m = lusion_core::model::Model::new();
    m.update(&doc);
    for b in m.current(&doc).bodies.values() {
        for (i, f) in b.shape.faces().iter().enumerate() {
            let inf = f.face_info().unwrap();
            println!("{i}: {:?} n={:?} c={:?} name={}", inf.kind, inf.normal, inf.center, b.face_names[i]);
        }
    }
}
