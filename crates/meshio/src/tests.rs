use super::*;
use std::io::{Cursor, Write};

// ---------------------------------------------------------------- helpers

/// Unit-cube corners and 12 outward-facing triangles (scaled by `s`).
fn cube(s: f32) -> (Vec<[f32; 3]>, Vec<[u32; 3]>) {
    let p = (0..8)
        .map(|i| {
            [
                (i & 1) as f32 * s,
                ((i >> 1) & 1) as f32 * s,
                ((i >> 2) & 1) as f32 * s,
            ]
        })
        .collect();
    let quads = [
        [0, 2, 3, 1], // z=0 (normal -z)
        [4, 5, 7, 6], // z=1
        [0, 1, 5, 4], // y=0
        [2, 6, 7, 3], // y=1
        [0, 4, 6, 2], // x=0
        [1, 3, 7, 5], // x=1
    ];
    let mut t = Vec::new();
    for q in quads {
        t.push([q[0], q[1], q[2]]);
        t.push([q[0], q[2], q[3]]);
    }
    (p, t)
}

fn cube_mesh(s: f32) -> TriMesh {
    let (p, t) = cube(s);
    let mut m = TriMesh {
        name: "cube".into(),
        positions: p,
        indices: t.into_iter().flatten().collect(),
        ..Default::default()
    };
    m.compute_normals(30.0);
    m
}

fn ascii_stl(name: &str, s: f32) -> String {
    let (p, t) = cube(s);
    let mut o = format!("solid {name}\n");
    for tri in t {
        let n = face_normal(p[tri[0] as usize], p[tri[1] as usize], p[tri[2] as usize]);
        o += &format!(
            "  facet normal {} {} {}\n    outer loop\n",
            n[0], n[1], n[2]
        );
        for i in tri {
            let v = p[i as usize];
            o += &format!("      vertex {:e} {} {}\n", v[0], v[1], v[2]);
        }
        o += "    endloop\n  endfacet\n";
    }
    o + &format!("endsolid {name}\n")
}

fn binary_stl(header: &[u8], s: f32) -> Vec<u8> {
    let (p, t) = cube(s);
    let mut b = vec![0u8; 80];
    b[..header.len()].copy_from_slice(header);
    b.extend_from_slice(&(t.len() as u32).to_le_bytes());
    for tri in t {
        b.extend_from_slice(&[0u8; 12]);
        for i in tri {
            for c in p[i as usize] {
                b.extend_from_slice(&c.to_le_bytes());
            }
        }
        b.extend_from_slice(&[0, 0]);
    }
    b
}

fn assert_bounds(m: &TriMesh, lo: [f32; 3], hi: [f32; 3]) {
    let (a, b) = m.bounds().expect("bounds");
    for k in 0..3 {
        assert!((a[k] - lo[k]).abs() < 1e-3, "lo {a:?} != {lo:?}");
        assert!((b[k] - hi[k]).abs() < 1e-3, "hi {b:?} != {hi:?}");
    }
}

fn assert_valid(m: &TriMesh) {
    assert_eq!(m.normals.len(), m.positions.len(), "normals len");
    assert_eq!(m.indices.len() % 3, 0);
    assert!(m.indices.iter().all(|&i| (i as usize) < m.positions.len()));
    for n in &m.normals {
        let l = dot(*n, *n).sqrt();
        assert!((l - 1.0).abs() < 1e-3, "normal not unit: {n:?}");
    }
}

/// For each triangle, the vertex normals should agree with the face normal (outward cube).
fn assert_flat_cube_normals(m: &TriMesh) {
    for t in m.indices.as_chunks::<3>().0 {
        let f = face_normal(
            m.positions[t[0] as usize],
            m.positions[t[1] as usize],
            m.positions[t[2] as usize],
        );
        for &i in t {
            assert!(
                dot(f, m.normals[i as usize]) > 0.999,
                "{f:?} vs {:?}",
                m.normals[i as usize]
            );
        }
    }
}

fn scratch_dir(tag: &str) -> std::path::PathBuf {
    let d = std::env::temp_dir().join(format!("lusion_meshio_test_{}_{tag}", std::process::id()));
    std::fs::create_dir_all(&d).unwrap();
    d
}

// ---------------------------------------------------------------- normals

#[test]
fn cube_normals_are_split_at_creases() {
    let m = cube_mesh(1.0);
    assert_eq!(m.triangle_count(), 12);
    assert_eq!(m.positions.len(), 24); // 6 faces x 4 corners
    assert_valid(&m);
    assert_flat_cube_normals(&m);
}

#[test]
fn smooth_cylinder_keeps_side_vertices_shared() {
    // 64-segment cylinder side (no caps): adjacent faces differ by 5.6 deg -> fully smooth.
    let seg = 64;
    let mut m = TriMesh::default();
    for i in 0..seg {
        let a = i as f32 / seg as f32 * std::f32::consts::TAU;
        m.positions.push([a.cos(), a.sin(), 0.0]);
        m.positions.push([a.cos(), a.sin(), 1.0]);
    }
    for i in 0..seg as u32 {
        let j = (i + 1) % seg as u32;
        m.indices
            .extend_from_slice(&[2 * i, 2 * j, 2 * j + 1, 2 * i, 2 * j + 1, 2 * i + 1]);
    }
    m.compute_normals(30.0);
    assert_eq!(m.positions.len(), 2 * seg);
    assert_valid(&m);
    for (p, n) in m.positions.iter().zip(&m.normals) {
        assert!((n[0] - p[0]).abs() < 1e-3 && (n[1] - p[1]).abs() < 1e-3 && n[2].abs() < 1e-3);
    }
    // With a 1 degree crease everything is split.
    m.compute_normals(1.0);
    assert_eq!(m.positions.len(), 4 * seg);
}

#[test]
fn compute_normals_welds_triangle_soup_and_drops_bad_indices() {
    let src = cube_mesh(2.0);
    let mut soup = TriMesh::default();
    for &i in &src.indices {
        soup.indices.push(soup.positions.len() as u32);
        soup.positions.push(src.positions[i as usize]);
    }
    soup.indices.extend_from_slice(&[0, 1, 999]); // invalid
    soup.compute_normals(30.0);
    assert_eq!(soup.triangle_count(), 12);
    assert_eq!(soup.positions.len(), 24);
    assert_flat_cube_normals(&soup);
}

// ---------------------------------------------------------------- STL

#[test]
fn stl_ascii_and_binary_agree() {
    let a = load_mesh_bytes(ascii_stl("box", 10.0).as_bytes(), "stl").unwrap();
    let b = load_mesh_bytes(&binary_stl(b"some exporter", 10.0), "STL").unwrap();
    assert_eq!(a.len(), 1);
    assert_eq!(a[0].name, "box");
    for m in [&a[0], &b[0]] {
        assert_eq!(m.triangle_count(), 12);
        assert_eq!(m.positions.len(), 24);
        assert_valid(m);
        assert_flat_cube_normals(m);
        assert_bounds(m, [0.0; 3], [10.0; 3]);
    }
}

#[test]
fn stl_binary_starting_with_solid() {
    let b = binary_stl(b"solid but actually binary", 3.0);
    let m = load_mesh_bytes(&b, "stl").unwrap();
    assert_eq!(m[0].triangle_count(), 12);
    assert_bounds(&m[0], [0.0; 3], [3.0; 3]);
}

#[test]
fn stl_binary_color_header() {
    let mut h = b"COLOR=".to_vec();
    h.extend_from_slice(&[255, 0, 0, 255]);
    let m = load_mesh_bytes(&binary_stl(&h, 1.0), "stl").unwrap();
    assert_eq!(m[0].color, Some([1.0, 0.0, 0.0, 1.0]));
}

#[test]
fn stl_multiple_solids() {
    let s = ascii_stl("a", 1.0) + &ascii_stl("b", 2.0);
    let m = load_mesh_bytes(s.as_bytes(), "stl").unwrap();
    assert_eq!(m.len(), 2);
    assert_eq!(m[1].name, "b");
    assert_bounds(&m[1], [0.0; 3], [2.0; 3]);
}

#[test]
fn stl_errors() {
    assert!(load_mesh_bytes(b"", "stl").is_err());
    let mut b = binary_stl(b"x", 1.0);
    b.truncate(200);
    let e = load_mesh_bytes(&b, "stl").unwrap_err();
    assert!(format!("{e:#}").contains("truncated"), "{e:#}");
    let e = load_mesh_bytes(
        b"solid x\nfacet normal 0 0 1\nouter loop\nvertex 1 2 abc\n",
        "stl",
    )
    .unwrap_err();
    assert!(format!("{e:#}").contains("line 4"), "{e:#}");
}

#[test]
fn stl_roundtrip() {
    let d = scratch_dir("stl");
    let p = d.join("out.stl");
    let mut m2 = cube_mesh(5.0);
    for v in &mut m2.positions {
        v[0] += 20.0;
    }
    save_stl(&p, &[cube_mesh(5.0), m2]).unwrap();
    assert_eq!(std::fs::metadata(&p).unwrap().len(), 84 + 50 * 24);
    let m = load_mesh_file(&p).unwrap();
    assert_eq!(m.len(), 1);
    assert_eq!(m[0].name, "out");
    assert_eq!(m[0].triangle_count(), 24);
    assert_bounds(&m[0], [0.0; 3], [25.0, 5.0, 5.0]);
    assert_flat_cube_normals(&m[0]);
}

// ---------------------------------------------------------------- OBJ

const OBJ_QUADS: &str = "\
# cube with quads, negative indices for the second object
mtllib mats.mtl
o first
v 0 0 0
v 1 0 0
v 1 1 0
v 0 1 0
v 0 0 1
v 1 0 1
v 1 1 1
v 0 1 1
usemtl red
f 1 4 3 2
f 5 6 7 8
f 1 2 6 5
f 4 8 7 3
f 1 5 8 4
f 2 3 7 6
o second
v 5 5 5
v 6 5 5
v 5 6 5
v 5 5 6
vn 0 0 -1
f -4//1 -2//1 -3//1
f -4 -3 -1
f -4/1 -1/1 -2/1
f -3 -2 -1
";

#[test]
fn obj_objects_quads_negative_indices() {
    let m = load_mesh_bytes(OBJ_QUADS.as_bytes(), "obj").unwrap();
    assert_eq!(m.len(), 2);
    assert_eq!(m[0].name, "first");
    assert_eq!(m[0].triangle_count(), 12);
    assert_eq!(m[0].positions.len(), 24);
    assert_flat_cube_normals(&m[0]);
    assert_bounds(&m[0], [0.0; 3], [1.0; 3]);
    assert_eq!(m[0].color, None); // no mtl available from bytes
    assert_eq!(m[1].name, "second");
    assert_eq!(m[1].triangle_count(), 4);
    assert_bounds(&m[1], [5.0; 3], [6.0; 3]);
    assert_valid(&m[1]);
}

#[test]
fn obj_mtl_color_from_file_and_missing_mtl_ok() {
    let d = scratch_dir("obj");
    std::fs::write(d.join("cube.obj"), OBJ_QUADS).unwrap();
    std::fs::write(d.join("mats.mtl"), "newmtl red\nKd 1 0 0\nd 0.5\n").unwrap();
    let m = load_mesh_file(&d.join("cube.obj")).unwrap();
    assert_eq!(m[0].color, Some([1.0, 0.0, 0.0, 0.5]));
    std::fs::remove_file(d.join("mats.mtl")).unwrap();
    let m = load_mesh_file(&d.join("cube.obj")).unwrap();
    assert_eq!(m[0].color, None);
}

#[test]
fn obj_with_normals_keeps_them() {
    let s = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvn 0 0 2\nf 1//1 2//1 3//1\n";
    let m = load_mesh_bytes(s.as_bytes(), "obj").unwrap();
    assert_eq!(m[0].normals, vec![[0.0, 0.0, 1.0]; 3]);
}

#[test]
fn obj_errors() {
    assert!(load_mesh_bytes(b"v 0 0 0\n", "obj").is_err());
    let e = load_mesh_bytes(b"v 0 0\nf 1 1 1\n", "obj").unwrap_err();
    assert!(format!("{e:#}").contains("line 1"), "{e:#}");
    assert!(load_mesh_bytes(b"v 0 0 0\nf 1 2 3\n", "obj").is_err());
}

#[test]
fn obj_roundtrip() {
    let d = scratch_dir("objrt");
    let p = d.join("rt.obj");
    let mut b = cube_mesh(2.0);
    b.name = "second part".into();
    save_obj(&p, &[cube_mesh(1.0), b]).unwrap();
    let m = load_mesh_file(&p).unwrap();
    assert_eq!(m.len(), 2);
    assert_eq!(m[0].name, "cube");
    assert_eq!(m[1].name, "second_part");
    assert_eq!(m[1].triangle_count(), 12);
    assert_eq!(m[1].positions.len(), 24);
    assert_flat_cube_normals(&m[1]);
    assert_bounds(&m[1], [0.0; 3], [2.0; 3]);
}

// ---------------------------------------------------------------- PLY

#[test]
fn ply_ascii_with_extra_elements() {
    let s = "ply\r\nformat ascii 1.0\r\ncomment test\r\nelement vertex 8\r\nproperty float x\r\nproperty float y\r\nproperty float z\r\nproperty list uchar int junk\r\nelement face 6\r\nproperty uchar flags\r\nproperty list uchar int vertex_indices\r\nelement edge 1\r\nproperty int vertex1\r\nproperty int vertex2\r\nend_header\r\n\
0 0 0 0\n1 0 0 2 7 7\n1 1 0 0\n0 1 0 0\n0 0 1 0\n1 0 1 0\n1 1 1 1 9\n0 1 1 0\n\
0 4 0 3 2 1\n0 4 4 5 6 7\n0 4 0 1 5 4\n0 4 3 7 6 2\n0 4 0 4 7 3\n0 4 1 2 6 5\n0 1\n";
    let m = load_mesh_bytes(s.as_bytes(), "ply").unwrap();
    assert_eq!(m[0].triangle_count(), 12);
    assert_eq!(m[0].positions.len(), 24);
    assert_flat_cube_normals(&m[0]);
    assert_bounds(&m[0], [0.0; 3], [1.0; 3]);
}

fn binary_ply(le: bool) -> Vec<u8> {
    let (p, t) = cube(4.0);
    let fmt = if le {
        "binary_little_endian"
    } else {
        "binary_big_endian"
    };
    let mut b = format!(
        "ply\nformat {fmt} 1.0\nelement vertex 8\nproperty double x\nproperty uchar red\nproperty double y\nproperty list ushort short extra\nproperty double z\nelement face 12\nproperty list uint uint vertex_index\nproperty float quality\nend_header\n"
    )
    .into_bytes();
    fn put<const N: usize>(b: &mut Vec<u8>, le: bool, mut x: [u8; N]) {
        if !le {
            x.reverse();
        }
        b.extend_from_slice(&x);
    }
    for v in &p {
        put(&mut b, le, (v[0] as f64).to_le_bytes());
        b.push(200);
        put(&mut b, le, (v[1] as f64).to_le_bytes());
        put(&mut b, le, 2u16.to_le_bytes());
        put(&mut b, le, 7i16.to_le_bytes());
        put(&mut b, le, (-7i16).to_le_bytes());
        put(&mut b, le, (v[2] as f64).to_le_bytes());
    }
    for tri in &t {
        put(&mut b, le, 3u32.to_le_bytes());
        for &i in tri {
            put(&mut b, le, i.to_le_bytes());
        }
        put(&mut b, le, 0.5f32.to_le_bytes());
    }
    b
}

#[test]
fn ply_binary_le_and_be() {
    for le in [true, false] {
        let m = load_mesh_bytes(&binary_ply(le), "ply").unwrap();
        assert_eq!(m[0].triangle_count(), 12);
        assert_eq!(m[0].positions.len(), 24);
        assert_flat_cube_normals(&m[0]);
        assert_bounds(&m[0], [0.0; 3], [4.0; 3]);
    }
}

#[test]
fn ply_normals_and_errors() {
    let s = "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face 1\nproperty list uchar uint vertex_indices\nend_header\n0 0 0 0 0 3\n1 0 0 0 0 3\n0 1 0 0 0 3\n3 0 1 2\n";
    let m = load_mesh_bytes(s.as_bytes(), "ply").unwrap();
    assert_eq!(m[0].normals, vec![[0.0, 0.0, 1.0]; 3]);
    assert!(load_mesh_bytes(s.replace("3 0 1 2", "3 0 1 9").as_bytes(), "ply").is_err());
    let mut b = binary_ply(true);
    b.truncate(b.len() - 10);
    assert!(load_mesh_bytes(&b, "ply").is_err());
}

// ---------------------------------------------------------------- OFF

#[test]
fn off_cube() {
    let s = "OFF\n# cube\n8 6 12\n0 0 0\n2 0 0\n2 2 0\n0 2 0\n0 0 2\n2 0 2\n2 2 2\n0 2 2\n\
4 0 3 2 1\n4 4 5 6 7\n4 0 1 5 4\n4 3 7 6 2\n4 0 4 7 3\n4 1 2 6 5 255 0 0\n";
    let m = load_mesh_bytes(s.as_bytes(), "off").unwrap();
    assert_eq!(m[0].triangle_count(), 12);
    assert_flat_cube_normals(&m[0]);
    assert_bounds(&m[0], [0.0; 3], [2.0; 3]);
    assert!(load_mesh_bytes(s.replace("4 1 2 6 5", "4 1 2 6 50").as_bytes(), "off").is_err());
    assert!(load_mesh_bytes(b"OFF\n8 6 0\n0 0 0\n", "off").is_err());
}

// ---------------------------------------------------------------- 3MF

fn make_zip(files: &[(&str, &str)]) -> Vec<u8> {
    let mut zw = zip::ZipWriter::new(Cursor::new(Vec::new()));
    for (n, d) in files {
        zw.start_file(*n, zip::write::SimpleFileOptions::default())
            .unwrap();
        zw.write_all(d.as_bytes()).unwrap();
    }
    zw.finish().unwrap().into_inner()
}

fn tf_mesh_xml(s: f32) -> String {
    let (p, t) = cube(s);
    let mut x = String::from("<mesh><vertices>");
    for v in p {
        x += &format!("<vertex x=\"{}\" y=\"{}\" z=\"{}\" />", v[0], v[1], v[2]);
    }
    x += "</vertices><triangles>";
    for tri in t {
        x += &format!(
            "<triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>",
            tri[0], tri[1], tri[2]
        );
    }
    x + "</triangles></mesh>"
}

#[test]
fn threemf_inch_units_transforms_components_colors() {
    let model = format!(
        r##"<?xml version="1.0" encoding="UTF-8"?>
<model unit="inch" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">
 <resources>
  <basematerials id="9"><base name="a" displaycolor="#FF0000"/><base name="b" displaycolor="#00FF0080"/></basematerials>
  <object id="1" type="model" name="box" pid="9" pindex="1">{}</object>
  <object id="2" type="model" name="asm">
   <components><component objectid="1" transform="1 0 0 0 1 0 0 0 1 10 0 0"/></components>
  </object>
 </resources>
 <build>
  <item objectid="1" transform="1 0 0 0 1 0 0 0 1 0 0 2"/>
  <item objectid="2" transform="2 0 0 0 2 0 0 0 2 0 0 0"/>
 </build>
</model>"##,
        tf_mesh_xml(1.0)
    );
    // Root model at a non-default path, found through _rels/.rels.
    let rels = r#"<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/main.model" Id="r" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>"#;
    let z = make_zip(&[("_rels/.rels", rels), ("3D/main.model", &model)]);
    let m = load_mesh_bytes(&z, "3mf").unwrap();
    assert_eq!(m.len(), 2);
    assert_eq!(m[0].name, "box");
    assert_eq!(m[0].triangle_count(), 12);
    assert_flat_cube_normals(&m[0]);
    assert_bounds(&m[0], [0.0, 0.0, 50.8], [25.4, 25.4, 76.2]);
    // component translate 10in then scale 2 -> x in [20in, 22in]
    assert_bounds(&m[1], [508.0, 0.0, 0.0], [558.8, 50.8, 50.8]);
    let c = m[0].color.unwrap();
    assert!(c[0] < 1e-6 && (c[1] - 1.0).abs() < 1e-6 && (c[3] - 128.0 / 255.0).abs() < 1e-6);
}

#[test]
fn threemf_mirror_transform_flips_winding() {
    let model = format!(
        r#"<model unit="millimeter"><resources><object id="3">{}</object></resources>
        <build><item objectid="3" transform="-1 0 0 0 1 0 0 0 1 0 0 0"/></build></model>"#,
        tf_mesh_xml(1.0)
    );
    let z = make_zip(&[("3D/3dmodel.model", &model)]);
    let m = load_mesh_bytes(&z, "3mf").unwrap();
    assert_bounds(&m[0], [-1.0, 0.0, 0.0], [0.0, 1.0, 1.0]);
    assert_flat_cube_normals(&m[0]);
    // Outward: the normal at the x=-1 face must point to -x.
    let outward = m[0]
        .positions
        .iter()
        .zip(&m[0].normals)
        .filter(|(p, _)| p[0] == -1.0)
        .all(|(_, n)| n[0] <= 0.0);
    assert!(outward);
}

#[test]
fn threemf_errors() {
    assert!(load_mesh_bytes(b"not a zip", "3mf").is_err());
    let bad = r#"<model><resources><object id="1"><mesh><vertices><vertex x="0" y="0" z="0"/></vertices><triangles><triangle v1="0" v2="1" v3="2"/></triangles></mesh></object></resources><build><item objectid="1"/></build></model>"#;
    let e = load_mesh_bytes(&make_zip(&[("3D/3dmodel.model", bad)]), "3mf").unwrap_err();
    assert!(format!("{e:#}").contains("out of range"), "{e:#}");
    let e = load_mesh_bytes(
        &make_zip(&[("3D/3dmodel.model", "<model><resources")]),
        "3mf",
    );
    assert!(e.is_err());
}

#[test]
fn threemf_roundtrip() {
    let d = scratch_dir("3mf");
    let p = d.join("rt.3mf");
    let mut a = cube_mesh(3.0);
    a.name = "A & <b>".into();
    a.color = Some([0.2, 0.5, 1.0, 1.0]);
    let mut b = cube_mesh(1.0);
    b.name = "plain".into();
    save_3mf(&p, &[a.clone(), b]).unwrap();
    let m = load_mesh_file(&p).unwrap();
    assert_eq!(m.len(), 2);
    assert_eq!(m[0].name, "A & <b>");
    assert_eq!(m[0].triangle_count(), 12);
    assert_bounds(&m[0], [0.0; 3], [3.0; 3]);
    assert_flat_cube_normals(&m[0]);
    let c = m[0].color.unwrap();
    for k in 0..4 {
        assert!((c[k] - a.color.unwrap()[k]).abs() < 0.01, "{c:?}");
    }
    assert_eq!(m[1].color, None);
    // Package parts are present and the model is welded (8 vertices per cube).
    let mut zip = zip::ZipArchive::new(std::fs::File::open(&p).unwrap()).unwrap();
    zip.by_name("[Content_Types].xml").unwrap();
    zip.by_name("_rels/.rels").unwrap();
    let mut s = String::new();
    std::io::Read::read_to_string(&mut zip.by_name("3D/3dmodel.model").unwrap(), &mut s).unwrap();
    assert!(s.contains("unit=\"millimeter\""));
    assert_eq!(s.matches("<vertex ").count(), 16);
}

// ---------------------------------------------------------------- glTF

/// Tetrahedron (4 verts, 4 tris) as f32 positions + u16 indices.
fn tetra_buffer() -> (Vec<u8>, usize) {
    let pos: [[f32; 3]; 4] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];
    let idx: [u16; 12] = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
    let mut b = Vec::new();
    for p in pos {
        for c in p {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    let pos_len = b.len();
    for i in idx {
        b.extend_from_slice(&i.to_le_bytes());
    }
    (b, pos_len)
}

fn gltf_json(buffer_uri: Option<&str>, len: usize, pos_len: usize) -> String {
    let uri = buffer_uri
        .map(|u| format!(r#""uri":"{u}","#))
        .unwrap_or_default();
    format!(
        r#"{{
  "asset": {{"version": "2.0"}},
  "scene": 0,
  "scenes": [{{"nodes": [0]}}],
  "nodes": [
    {{"name": "root", "translation": [1, 0, 0], "children": [1]}},
    {{"name": "child", "mesh": 0, "scale": [0.01, 0.01, 0.01]}}
  ],
  "meshes": [{{"name": "tet", "primitives": [
     {{"attributes": {{"POSITION": 0}}, "indices": 1, "material": 0}},
     {{"attributes": {{"POSITION": 0}}, "mode": 1}}
  ]}}],
  "materials": [{{"name": "blue", "pbrMetallicRoughness": {{"baseColorFactor": [0, 0, 1, 1]}}}}],
  "buffers": [{{{uri}"byteLength": {len}}}],
  "bufferViews": [
    {{"buffer": 0, "byteOffset": 0, "byteLength": {pos_len}}},
    {{"buffer": 0, "byteOffset": {pos_len}, "byteLength": 24}}
  ],
  "accessors": [
    {{"bufferView": 0, "componentType": 5126, "count": 4, "type": "VEC3", "min": [0,0,0], "max": [1,1,1]}},
    {{"bufferView": 1, "componentType": 5123, "count": 12, "type": "SCALAR"}}
  ]
}}"#
    )
}

fn check_tetra(m: &[TriMesh]) {
    assert_eq!(m.len(), 1, "line primitive must be skipped");
    assert_eq!(m[0].name, "child");
    assert_eq!(m[0].triangle_count(), 4);
    assert_eq!(m[0].color, Some([0.0, 0.0, 1.0, 1.0]));
    assert_valid(&m[0]);
    // metres -> mm, scale 0.01 then translate 1 m in x.
    assert_bounds(&m[0], [1000.0, 0.0, 0.0], [1010.0, 10.0, 10.0]);
}

#[test]
fn gltf_embedded_base64() {
    use base64::Engine as _;
    let (b, pos_len) = tetra_buffer();
    let uri = format!(
        "data:application/octet-stream;base64,{}",
        base64::engine::general_purpose::STANDARD.encode(&b)
    );
    let json = gltf_json(Some(&uri), b.len(), pos_len);
    check_tetra(&load_mesh_bytes(json.as_bytes(), "gltf").unwrap());
}

#[test]
fn gltf_external_buffer_from_file() {
    let (b, pos_len) = tetra_buffer();
    let d = scratch_dir("gltf");
    std::fs::write(d.join("my data.bin"), &b).unwrap();
    let json = gltf_json(Some("my%20data.bin"), b.len(), pos_len);
    std::fs::write(d.join("t.gltf"), &json).unwrap();
    check_tetra(&load_mesh_file(&d.join("t.gltf")).unwrap());
    assert!(load_mesh_bytes(json.as_bytes(), "gltf").is_err());
}

#[test]
fn glb_in_memory() {
    let (mut bin, pos_len) = tetra_buffer();
    let mut json = gltf_json(None, bin.len(), pos_len).into_bytes();
    while !json.len().is_multiple_of(4) {
        json.push(b' ');
    }
    while bin.len() % 4 != 0 {
        bin.push(0);
    }
    let total = 12 + 8 + json.len() + 8 + bin.len();
    let mut g = Vec::new();
    g.extend_from_slice(b"glTF");
    g.extend_from_slice(&2u32.to_le_bytes());
    g.extend_from_slice(&(total as u32).to_le_bytes());
    g.extend_from_slice(&(json.len() as u32).to_le_bytes());
    g.extend_from_slice(b"JSON");
    g.extend_from_slice(&json);
    g.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    g.extend_from_slice(b"BIN\0");
    g.extend_from_slice(&bin);
    check_tetra(&load_mesh_bytes(&g, "glb").unwrap());
    // Truncated GLB must error, not panic.
    assert!(load_mesh_bytes(&g[..g.len() - 20], "glb").is_err());
}

// ---------------------------------------------------------------- misc

#[test]
fn dispatch_and_extensions() {
    assert_eq!(
        supported_import_extensions(),
        &["stl", "obj", "gltf", "glb", "3mf", "ply", "off"]
    );
    assert!(load_mesh_bytes(b"", "step").is_err());
    assert!(load_mesh_file(Path::new("/nonexistent/file.stl")).is_err());
    assert!(cube_mesh(1.0).bounds().is_some());
    assert!(TriMesh::default().bounds().is_none());
}

#[test]
fn garbage_never_panics() {
    // Deterministic pseudo-random bytes and truncations of valid files.
    let mut seed = 0x1234_5678_9abc_def0u64;
    let mut rnd = move || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        seed
    };
    let samples: Vec<(Vec<u8>, &str)> = vec![
        (ascii_stl("x", 1.0).into_bytes(), "stl"),
        (binary_stl(b"", 1.0), "stl"),
        (OBJ_QUADS.as_bytes().to_vec(), "obj"),
        (binary_ply(true), "ply"),
        (binary_ply(false), "ply"),
        (make_zip(&[("3D/3dmodel.model", &format!("<model><resources><object id=\"1\">{}</object></resources><build><item objectid=\"1\"/></build></model>", tf_mesh_xml(1.0)))]), "3mf"),
    ];
    for (data, ext) in &samples {
        for _ in 0..200 {
            let mut d = data.clone();
            let cut = (rnd() as usize) % (d.len() + 1);
            d.truncate(cut);
            for _ in 0..(rnd() % 4) {
                if !d.is_empty() {
                    let i = (rnd() as usize) % d.len();
                    d[i] = rnd() as u8;
                }
            }
            let _ = load_mesh_bytes(&d, ext);
        }
    }
    for ext in supported_import_extensions() {
        for len in [0usize, 1, 10, 84, 100, 1000] {
            let d: Vec<u8> = (0..len).map(|_| rnd() as u8).collect();
            let _ = load_mesh_bytes(&d, ext);
        }
    }
}

/// Timing check: `cargo test -p lusion-meshio --release -- --ignored --nocapture perf`
#[test]
#[ignore]
fn perf_1m_triangle_binary_stl() {
    // Tessellated height-field-like grid surface: ~1M triangles, shared vertices.
    let n = 708usize; // 2 * 707^2 ~= 1.0M triangles
    let h = |x: usize, y: usize| ((x as f32 * 0.05).sin() * (y as f32 * 0.07).cos()) * 5.0;
    let tris = 2 * (n - 1) * (n - 1);
    let mut b = vec![0u8; 80];
    b.extend_from_slice(&(tris as u32).to_le_bytes());
    fn put(b: &mut Vec<u8>, p: [f32; 3]) {
        for c in p {
            b.extend_from_slice(&c.to_le_bytes());
        }
    }
    let mut recs = Vec::new();
    for y in 0..n - 1 {
        for x in 0..n - 1 {
            let p = |x: usize, y: usize| [x as f32, y as f32, h(x, y)];
            recs.push([p(x, y), p(x + 1, y), p(x + 1, y + 1)]);
            recs.push([p(x, y), p(x + 1, y + 1), p(x, y + 1)]);
        }
    }
    for r in &recs {
        put(&mut b, [0.0; 3]);
        for v in r {
            put(&mut b, *v);
        }
        b.extend_from_slice(&[0, 0]);
    }
    let t = std::time::Instant::now();
    let m = load_mesh_bytes(&b, "stl").unwrap();
    let el = t.elapsed();
    eprintln!(
        "PERF: loaded {} triangles ({} vertices) from {:.1} MB binary STL in {:.1} ms",
        m[0].triangle_count(),
        m[0].positions.len(),
        b.len() as f64 / 1e6,
        el.as_secs_f64() * 1e3
    );
    assert_eq!(m[0].triangle_count(), tris);
}
