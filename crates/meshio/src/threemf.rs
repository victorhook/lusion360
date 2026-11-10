//! 3MF reader (core spec + basematerials/colorgroup colours + production-extension
//! component paths) and writer (core spec).

use anyhow::{anyhow, bail, Context, Result};
use quick_xml::events::{BytesStart, Event};
use std::collections::HashMap;
use std::fmt::Write as _;
use std::io::{Cursor, Read, Write};

use crate::hash::Welder;
use crate::{linear_to_srgb, srgb_to_linear, TriMesh, DEFAULT_CREASE_DEG};

/// Affine transform in 3MF row-vector convention:
/// `[m00 m01 m02 m10 m11 m12 m20 m21 m22 m30 m31 m32]`, `p' = [x y z 1] * M`.
type Affine = [f32; 12];
const IDENTITY: Affine = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0];

fn apply(m: &Affine, p: [f32; 3]) -> [f32; 3] {
    let mut r = [0f32; 3];
    for (c, v) in r.iter_mut().enumerate() {
        *v = p[0] * m[c] + p[1] * m[3 + c] + p[2] * m[6 + c] + m[9 + c];
    }
    r
}

/// `first` applied, then `then`.
fn compose(first: &Affine, then: &Affine) -> Affine {
    let mut r = [0f32; 12];
    for row in 0..4 {
        for c in 0..3 {
            let mut s = if row == 3 { then[9 + c] } else { 0.0 };
            for k in 0..3 {
                s += first[row * 3 + k] * then[k * 3 + c];
            }
            r[row * 3 + c] = s;
        }
    }
    r
}

fn det(m: &Affine) -> f32 {
    m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6])
        + m[2] * (m[3] * m[7] - m[4] * m[6])
}

fn parse_transform(s: &str) -> Result<Affine> {
    let vals: Vec<f32> = s
        .split_ascii_whitespace()
        .map(|t| t.parse::<f32>())
        .collect::<Result<_, _>>()
        .with_context(|| format!("invalid transform '{s}'"))?;
    vals.try_into()
        .map_err(|_| anyhow!("transform '{s}' must have 12 numbers"))
}

fn unit_scale(unit: &str) -> Result<f32> {
    Ok(match unit {
        "micron" => 0.001,
        "millimeter" => 1.0,
        "centimeter" => 10.0,
        "inch" => 25.4,
        "foot" => 304.8,
        "meter" => 1000.0,
        other => bail!("unknown 3MF unit '{other}'"),
    })
}

fn parse_color(s: &str) -> Option<[f32; 4]> {
    let h = s.trim().strip_prefix('#')?;
    if !(h.len() == 6 || h.len() == 8) || !h.is_ascii() {
        return None;
    }
    let byte = |i: usize| u8::from_str_radix(&h[i..i + 2], 16).ok();
    let a = if h.len() == 8 { byte(6)? } else { 255 };
    Some([
        srgb_to_linear(byte(0)? as f32 / 255.0),
        srgb_to_linear(byte(2)? as f32 / 255.0),
        srgb_to_linear(byte(4)? as f32 / 255.0),
        a as f32 / 255.0,
    ])
}

#[derive(Default)]
struct Object {
    name: String,
    positions: Vec<[f32; 3]>,
    tris: Vec<[u32; 3]>,
    components: Vec<Ref>,
    pid: Option<u32>,
    pindex: Option<u32>,
    first_tri_prop: Option<(u32, u32)>,
}

#[derive(Clone)]
struct Ref {
    objectid: u32,
    transform: Affine,
    /// Production extension: part path of the model containing the object.
    path: Option<String>,
}

#[derive(Default)]
struct Model {
    unit: f32,
    objects: HashMap<u32, Object>,
    /// basematerials / colorgroup id -> colours.
    props: HashMap<u32, Vec<Option<[f32; 4]>>>,
    items: Vec<Ref>,
}

impl Model {
    fn color_of(&self, o: &Object) -> Option<[f32; 4]> {
        let (pid, idx) = match (o.pid, o.first_tri_prop) {
            (Some(pid), _) => (pid, o.pindex.unwrap_or(0)),
            (None, Some(p)) => p,
            _ => return None,
        };
        *self.props.get(&pid)?.get(idx as usize)?
    }
}

fn attrs(e: &BytesStart) -> Result<Vec<(Vec<u8>, String)>> {
    let mut v = Vec::new();
    for a in e.attributes().with_checks(false) {
        let a = a.map_err(|e| anyhow!("malformed XML attribute: {e}"))?;
        let val = a
            .normalized_value(quick_xml::XmlVersion::Implicit1_0)
            .map_err(|e| anyhow!("malformed XML attribute value: {e}"))?;
        v.push((a.key.local_name().as_ref().to_vec(), val.into_owned()));
    }
    Ok(v)
}

#[inline]
fn num<T: std::str::FromStr>(v: &[u8]) -> Option<T> {
    std::str::from_utf8(v).ok()?.trim().parse().ok()
}

fn parse_model(xml: &[u8]) -> Result<Model> {
    let mut reader = quick_xml::Reader::from_reader(xml);
    reader.config_mut().trim_text(true);
    let mut buf = Vec::new();
    let mut model = Model {
        unit: 1.0,
        ..Default::default()
    };
    let mut cur_obj: Option<(u32, Object)> = None;
    let mut cur_props: Option<u32> = None;
    let mut in_build = false;

    loop {
        let ev = reader
            .read_event_into(&mut buf)
            .map_err(|e| anyhow!("XML error at byte {}: {e}", reader.error_position()))?;
        let (e, is_empty) = match &ev {
            Event::Start(e) => (e, false),
            Event::Empty(e) => (e, true),
            Event::End(e) => {
                match e.local_name().as_ref() {
                    b"object" => {
                        if let Some((id, o)) = cur_obj.take() {
                            model.objects.insert(id, o);
                        }
                    }
                    b"basematerials" | b"colorgroup" => cur_props = None,
                    b"build" => in_build = false,
                    _ => {}
                }
                buf.clear();
                continue;
            }
            Event::Eof => break,
            _ => {
                buf.clear();
                continue;
            }
        };
        let pos = reader.buffer_position();
        let ctx = || format!("3MF model XML near byte {pos}");
        match e.local_name().as_ref() {
            b"vertex" => {
                if let Some((_, o)) = cur_obj.as_mut() {
                    let mut p = [f32::NAN; 3];
                    for a in e.attributes().with_checks(false) {
                        let a = a.map_err(|e| anyhow!("malformed XML attribute: {e}"))?;
                        let k = match a.key.local_name().as_ref() {
                            b"x" => 0,
                            b"y" => 1,
                            b"z" => 2,
                            _ => continue,
                        };
                        p[k] =
                            num(&a.value).with_context(|| format!("{}: invalid vertex", ctx()))?;
                    }
                    if p.iter().any(|c| c.is_nan()) {
                        bail!("{}: vertex missing coordinate", ctx());
                    }
                    o.positions.push(p);
                }
            }
            b"triangle" => {
                if let Some((_, o)) = cur_obj.as_mut() {
                    let mut t = [u32::MAX; 3];
                    let (mut pid, mut p1) = (None, None);
                    for a in e.attributes().with_checks(false) {
                        let a = a.map_err(|e| anyhow!("malformed XML attribute: {e}"))?;
                        let k = match a.key.local_name().as_ref() {
                            b"v1" => 0,
                            b"v2" => 1,
                            b"v3" => 2,
                            b"pid" => {
                                pid = num::<u32>(&a.value);
                                continue;
                            }
                            b"p1" => {
                                p1 = num::<u32>(&a.value);
                                continue;
                            }
                            _ => continue,
                        };
                        t[k] = num(&a.value)
                            .with_context(|| format!("{}: invalid triangle index", ctx()))?;
                    }
                    if t.contains(&u32::MAX) {
                        bail!("{}: triangle missing vertex index", ctx());
                    }
                    if o.first_tri_prop.is_none() {
                        if let Some(pid) = pid {
                            o.first_tri_prop = Some((pid, p1.unwrap_or(0)));
                        }
                    }
                    o.tris.push(t);
                }
            }
            b"model" => {
                for (k, v) in attrs(e)? {
                    if k == b"unit" {
                        model.unit = unit_scale(v.trim()).with_context(ctx)?;
                    }
                }
            }
            b"object" => {
                let mut id = None;
                let mut o = Object::default();
                for (k, v) in attrs(e)? {
                    match k.as_slice() {
                        b"id" => id = v.trim().parse::<u32>().ok(),
                        b"name" => o.name = v,
                        b"pid" => o.pid = v.trim().parse().ok(),
                        b"pindex" => o.pindex = v.trim().parse().ok(),
                        _ => {}
                    }
                }
                let id = id.with_context(|| format!("{}: object without valid id", ctx()))?;
                if is_empty {
                    model.objects.insert(id, o);
                } else {
                    cur_obj = Some((id, o));
                }
            }
            b"component" | b"item" => {
                let mut r = Ref {
                    objectid: u32::MAX,
                    transform: IDENTITY,
                    path: None,
                };
                for (k, v) in attrs(e)? {
                    match k.as_slice() {
                        b"objectid" => r.objectid = v.trim().parse().unwrap_or(u32::MAX),
                        b"transform" => r.transform = parse_transform(&v).with_context(ctx)?,
                        b"path" => r.path = Some(v.trim().trim_start_matches('/').to_string()),
                        _ => {}
                    }
                }
                if r.objectid == u32::MAX {
                    bail!(
                        "{}: <{}> without valid objectid",
                        ctx(),
                        String::from_utf8_lossy(e.local_name().as_ref())
                    );
                }
                if e.local_name().as_ref() == b"item" {
                    if in_build {
                        model.items.push(r);
                    }
                } else if let Some((_, o)) = cur_obj.as_mut() {
                    o.components.push(r);
                }
            }
            b"build" => in_build = !is_empty,
            b"basematerials" | b"colorgroup" => {
                let id = attrs(e)?
                    .into_iter()
                    .find(|(k, _)| k == b"id")
                    .and_then(|(_, v)| v.trim().parse::<u32>().ok());
                if let Some(id) = id {
                    model.props.insert(id, Vec::new());
                    cur_props = (!is_empty).then_some(id);
                }
            }
            b"base" | b"color" => {
                if let Some(id) = cur_props {
                    let key: &[u8] = if e.local_name().as_ref() == b"base" {
                        b"displaycolor"
                    } else {
                        b"color"
                    };
                    let c = attrs(e)?
                        .into_iter()
                        .find(|(k, _)| k == key)
                        .and_then(|(_, v)| parse_color(&v));
                    if let Some(list) = model.props.get_mut(&id) {
                        list.push(c);
                    }
                }
            }
            _ => {}
        }
        buf.clear();
    }
    Ok(model)
}

struct Package {
    zip: zip::ZipArchive<Cursor<Vec<u8>>>,
    /// lowercase name -> real name
    names: HashMap<String, String>,
}

impl Package {
    fn read(&mut self, path: &str) -> Result<Vec<u8>> {
        let path = path.trim_start_matches('/');
        let real = self
            .names
            .get(&path.to_ascii_lowercase())
            .cloned()
            .with_context(|| format!("3MF package has no part '{path}'"))?;
        let mut f = self
            .zip
            .by_name(&real)
            .with_context(|| format!("3MF: cannot open '{path}'"))?;
        let mut v = Vec::with_capacity(f.size().min(1 << 30) as usize);
        f.read_to_end(&mut v)
            .with_context(|| format!("3MF: cannot decompress '{path}'"))?;
        Ok(v)
    }
}

fn root_model_path(pkg: &mut Package) -> String {
    let default = "3D/3dmodel.model".to_string();
    let Ok(rels) = pkg.read("_rels/.rels") else {
        return default;
    };
    let mut reader = quick_xml::Reader::from_reader(rels.as_slice());
    let mut buf = Vec::new();
    while let Ok(ev) = reader.read_event_into(&mut buf) {
        match ev {
            Event::Eof => break,
            Event::Start(e) | Event::Empty(e) if e.local_name().as_ref() == b"Relationship" => {
                let Ok(a) = attrs(&e) else { continue };
                let ty = a
                    .iter()
                    .find(|(k, _)| k == b"Type")
                    .map(|(_, v)| v.as_str());
                let target = a
                    .iter()
                    .find(|(k, _)| k == b"Target")
                    .map(|(_, v)| v.as_str());
                if let (Some(ty), Some(t)) = (ty, target) {
                    if ty.trim_end_matches('/').ends_with("3dmodel") {
                        return t.trim_start_matches('/').to_string();
                    }
                }
            }
            _ => {}
        }
        buf.clear();
    }
    default
}

pub(crate) fn load(bytes: &[u8]) -> Result<Vec<TriMesh>> {
    let zip = zip::ZipArchive::new(Cursor::new(bytes.to_vec()))
        .context("not a valid 3MF (zip) package")?;
    let names = zip
        .file_names()
        .map(|n| (n.to_ascii_lowercase(), n.to_string()))
        .collect();
    let mut pkg = Package { zip, names };
    let root_path = root_model_path(&mut pkg);
    let mut models: HashMap<String, Model> = HashMap::new();
    let root_xml = pkg.read(&root_path)?;
    let root = parse_model(&root_xml).with_context(|| format!("in part '{root_path}'"))?;
    let unit = root.unit;
    let mut items = root.items.clone();
    if items.is_empty() {
        // No build section: show every object that isn't used as a component.
        let used: std::collections::HashSet<u32> = root
            .objects
            .values()
            .flat_map(|o| {
                o.components
                    .iter()
                    .filter(|c| c.path.is_none())
                    .map(|c| c.objectid)
            })
            .collect();
        let mut ids: Vec<u32> = root
            .objects
            .keys()
            .copied()
            .filter(|id| !used.contains(id))
            .collect();
        ids.sort_unstable();
        items = ids
            .into_iter()
            .map(|objectid| Ref {
                objectid,
                transform: IDENTITY,
                path: None,
            })
            .collect();
    }
    models.insert(root_path.clone(), root);

    let mut out = Vec::new();
    for item in &items {
        let path = item.path.clone().unwrap_or_else(|| root_path.clone());
        emit(
            &mut pkg,
            &mut models,
            &path,
            item.objectid,
            item.transform,
            "",
            unit,
            0,
            &mut out,
        )?;
    }
    if out.is_empty() {
        bail!("3MF contains no mesh objects in its build");
    }
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn emit(
    pkg: &mut Package,
    models: &mut HashMap<String, Model>,
    path: &str,
    objectid: u32,
    transform: Affine,
    parent_name: &str,
    unit: f32,
    depth: usize,
    out: &mut Vec<TriMesh>,
) -> Result<()> {
    if depth > 64 {
        bail!("3MF component hierarchy too deep (cyclic?)");
    }
    if !models.contains_key(path) {
        let xml = pkg.read(path)?;
        let m = parse_model(&xml).with_context(|| format!("in part '{path}'"))?;
        models.insert(path.to_string(), m);
    }
    let model = &models[path];
    let obj = model
        .objects
        .get(&objectid)
        .with_context(|| format!("3MF: reference to unknown object id {objectid} in '{path}'"))?;
    let name = if obj.name.is_empty() {
        parent_name.to_string()
    } else {
        obj.name.clone()
    };
    let name = if name.is_empty() {
        format!("object{objectid}")
    } else {
        name
    };

    if !obj.tris.is_empty() {
        let n = obj.positions.len();
        if let Some(t) = obj.tris.iter().find(|t| t.iter().any(|&i| i as usize >= n)) {
            bail!("3MF object {objectid} ('{name}'): triangle {t:?} references a vertex out of range (have {n})");
        }
        let positions = obj
            .positions
            .iter()
            .map(|&p| {
                let q = apply(&transform, p);
                [q[0] * unit, q[1] * unit, q[2] * unit]
            })
            .collect();
        let flip = det(&transform) < 0.0;
        let indices = obj
            .tris
            .iter()
            .flat_map(|t| if flip { [t[0], t[2], t[1]] } else { *t })
            .collect();
        let mut mesh = TriMesh {
            name: name.clone(),
            positions,
            normals: Vec::new(),
            indices,
            color: model.color_of(obj),
        };
        mesh.compute_normals(DEFAULT_CREASE_DEG);
        out.push(mesh);
    }
    let comps = obj.components.clone();
    for c in comps {
        let cpath = c.path.clone().unwrap_or_else(|| path.to_string());
        let t = compose(&c.transform, &transform);
        emit(
            pkg,
            models,
            &cpath,
            c.objectid,
            t,
            &name,
            unit,
            depth + 1,
            out,
        )?;
    }
    Ok(())
}

fn xml_escape(s: &str) -> String {
    let mut o = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => o.push_str("&amp;"),
            '<' => o.push_str("&lt;"),
            '>' => o.push_str("&gt;"),
            '"' => o.push_str("&quot;"),
            '\'' => o.push_str("&apos;"),
            c if (c as u32) < 0x20 && !matches!(c, '\t' | '\n' | '\r') => {}
            c => o.push(c),
        }
    }
    o
}

const CONTENT_TYPES: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>
"#;

const RELS: &str = r#"<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>
"#;

pub(crate) fn write(meshes: &[TriMesh]) -> Result<Vec<u8>> {
    // Weld + clean each mesh first (3MF requires v1 != v2 != v3; printers want manifold meshes).
    type Cleaned<'a> = (&'a TriMesh, Vec<[f32; 3]>, Vec<[u32; 3]>);
    let mut cleaned: Vec<Cleaned> = Vec::new();
    for m in meshes {
        let mut w = Welder::with_capacity(m.positions.len());
        let mut tris = Vec::with_capacity(m.triangle_count());
        let np = m.positions.len();
        for t in m.indices.as_chunks::<3>().0 {
            if t.iter().any(|&i| i as usize >= np) {
                continue;
            }
            let ps = [
                m.positions[t[0] as usize],
                m.positions[t[1] as usize],
                m.positions[t[2] as usize],
            ];
            if ps.iter().flatten().any(|c| !c.is_finite()) {
                continue;
            }
            let tri = [w.index(ps[0]), w.index(ps[1]), w.index(ps[2])];
            if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                tris.push(tri);
            }
        }
        if !tris.is_empty() {
            cleaned.push((m, w.positions, tris));
        }
    }
    if cleaned.is_empty() {
        bail!("nothing to export: no valid triangles");
    }

    let est: usize = cleaned
        .iter()
        .map(|(_, p, t)| p.len() * 60 + t.len() * 50)
        .sum();
    let mut x = String::with_capacity(est + 1024);
    x.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    x.push_str("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n");
    x.push_str(" <metadata name=\"Application\">Lusion360</metadata>\n <resources>\n");

    let colored: Vec<usize> = cleaned
        .iter()
        .enumerate()
        .filter(|(_, c)| c.0.color.is_some())
        .map(|(i, _)| i)
        .collect();
    let mat_id = 1u32;
    let first_obj_id = if colored.is_empty() { 1 } else { 2 };
    if !colored.is_empty() {
        let _ = writeln!(x, "  <basematerials id=\"{mat_id}\">");
        for &i in &colored {
            let c = cleaned[i].0.color.unwrap_or([0.8, 0.8, 0.8, 1.0]);
            let b = |v: f32| (linear_to_srgb(v) * 255.0).round() as u8;
            let a = (c[3].clamp(0.0, 1.0) * 255.0).round() as u8;
            let _ = writeln!(
                x,
                "   <base name=\"{}\" displaycolor=\"#{:02X}{:02X}{:02X}{:02X}\"/>",
                xml_escape(&cleaned[i].0.name),
                b(c[0]),
                b(c[1]),
                b(c[2]),
                a
            );
        }
        x.push_str("  </basematerials>\n");
    }
    for (i, (m, pos, tris)) in cleaned.iter().enumerate() {
        let id = first_obj_id + i as u32;
        let name = if m.name.is_empty() {
            format!("mesh_{}", i + 1)
        } else {
            m.name.clone()
        };
        let _ = write!(
            x,
            "  <object id=\"{id}\" type=\"model\" name=\"{}\"",
            xml_escape(&name)
        );
        if let Some(k) = colored.iter().position(|&c| c == i) {
            let _ = write!(x, " pid=\"{mat_id}\" pindex=\"{k}\"");
        }
        x.push_str(">\n   <mesh>\n    <vertices>\n");
        for p in pos {
            let _ = writeln!(
                x,
                "     <vertex x=\"{}\" y=\"{}\" z=\"{}\"/>",
                p[0], p[1], p[2]
            );
        }
        x.push_str("    </vertices>\n    <triangles>\n");
        for t in tris {
            let _ = writeln!(
                x,
                "     <triangle v1=\"{}\" v2=\"{}\" v3=\"{}\"/>",
                t[0], t[1], t[2]
            );
        }
        x.push_str("    </triangles>\n   </mesh>\n  </object>\n");
    }
    x.push_str(" </resources>\n <build>\n");
    for i in 0..cleaned.len() {
        let _ = writeln!(x, "  <item objectid=\"{}\"/>", first_obj_id + i as u32);
    }
    x.push_str(" </build>\n</model>\n");

    let mut zw = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let opts = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Deflated)
        .large_file(x.len() > u32::MAX as usize / 2);
    for (name, data) in [
        ("[Content_Types].xml", CONTENT_TYPES.as_bytes()),
        ("_rels/.rels", RELS.as_bytes()),
        ("3D/3dmodel.model", x.as_bytes()),
    ] {
        zw.start_file(name, opts)
            .with_context(|| format!("3MF: cannot add {name}"))?;
        zw.write_all(data)?;
    }
    Ok(zw
        .finish()
        .context("3MF: cannot finalize zip")?
        .into_inner())
}
