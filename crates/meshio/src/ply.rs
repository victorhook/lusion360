//! PLY reader: ascii, binary_little_endian, binary_big_endian.

use anyhow::{bail, Context, Result};

use crate::{normalize, TriMesh, DEFAULT_CREASE_DEG};

#[derive(Clone, Copy, Debug)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn parse(s: &str) -> Option<Scalar> {
        Some(match s {
            "char" | "int8" => Scalar::I8,
            "uchar" | "uint8" => Scalar::U8,
            "short" | "int16" => Scalar::I16,
            "ushort" | "uint16" => Scalar::U16,
            "int" | "int32" => Scalar::I32,
            "uint" | "uint32" => Scalar::U32,
            "float" | "float32" => Scalar::F32,
            "double" | "float64" => Scalar::F64,
            _ => return None,
        })
    }
    fn size(self) -> usize {
        match self {
            Scalar::I8 | Scalar::U8 => 1,
            Scalar::I16 | Scalar::U16 => 2,
            Scalar::I32 | Scalar::U32 | Scalar::F32 => 4,
            Scalar::F64 => 8,
        }
    }
}

#[derive(Debug)]
enum Kind {
    Scalar(Scalar),
    List(Scalar, Scalar),
}

#[derive(Debug)]
struct Prop {
    name: String,
    kind: Kind,
}

#[derive(Debug)]
struct Element {
    name: String,
    count: usize,
    props: Vec<Prop>,
}

#[derive(Clone, Copy, PartialEq)]
enum Format {
    Ascii,
    Le,
    Be,
}

/// Source of scalar values (binary or ascii).
trait Values {
    fn read(&mut self, s: Scalar) -> Result<f64>;
    fn skip(&mut self, s: Scalar, n: usize) -> Result<()> {
        for _ in 0..n {
            self.read(s)?;
        }
        Ok(())
    }
}

struct Bin<'a> {
    b: &'a [u8],
    pos: usize,
    le: bool,
}

impl Bin<'_> {
    #[inline]
    fn take<const N: usize>(&mut self) -> Result<[u8; N]> {
        let end = self.pos + N;
        if end > self.b.len() {
            bail!("unexpected end of binary data at byte {}", self.pos);
        }
        let mut a: [u8; N] = self.b[self.pos..end].try_into().unwrap();
        if !self.le {
            a.reverse();
        }
        self.pos = end;
        Ok(a)
    }
}

impl Values for Bin<'_> {
    #[inline]
    fn read(&mut self, s: Scalar) -> Result<f64> {
        Ok(match s {
            Scalar::I8 => i8::from_le_bytes(self.take::<1>()?) as f64,
            Scalar::U8 => u8::from_le_bytes(self.take::<1>()?) as f64,
            Scalar::I16 => i16::from_le_bytes(self.take::<2>()?) as f64,
            Scalar::U16 => u16::from_le_bytes(self.take::<2>()?) as f64,
            Scalar::I32 => i32::from_le_bytes(self.take::<4>()?) as f64,
            Scalar::U32 => u32::from_le_bytes(self.take::<4>()?) as f64,
            Scalar::F32 => f32::from_le_bytes(self.take::<4>()?) as f64,
            Scalar::F64 => f64::from_le_bytes(self.take::<8>()?),
        })
    }
    fn skip(&mut self, s: Scalar, n: usize) -> Result<()> {
        let bytes = n.checked_mul(s.size()).context("list too large")?;
        if self.pos + bytes > self.b.len() {
            bail!("unexpected end of binary data at byte {}", self.pos);
        }
        self.pos += bytes;
        Ok(())
    }
}

struct Ascii<'a> {
    b: &'a [u8],
    pos: usize,
}

impl Values for Ascii<'_> {
    fn read(&mut self, _s: Scalar) -> Result<f64> {
        let b = self.b;
        let mut i = self.pos;
        while i < b.len() && b[i].is_ascii_whitespace() {
            i += 1;
        }
        let s = i;
        while i < b.len() && !b[i].is_ascii_whitespace() {
            i += 1;
        }
        self.pos = i;
        if s == i {
            bail!("unexpected end of ascii data");
        }
        let tok = std::str::from_utf8(&b[s..i]).unwrap_or("");
        tok.parse::<f64>().with_context(|| {
            let line = 1 + b[..s].iter().filter(|&&c| c == b'\n').count();
            format!("invalid number '{tok}' (body line {line})")
        })
    }
}

fn parse_header(bytes: &[u8]) -> Result<(Format, Vec<Element>, usize)> {
    if !bytes.starts_with(b"ply") {
        bail!("PLY: missing 'ply' magic");
    }
    let mut pos = 0;
    let mut format = None;
    let mut elements: Vec<Element> = Vec::new();
    let mut line_no = 0;
    loop {
        line_no += 1;
        let nl = bytes[pos..]
            .iter()
            .position(|&c| c == b'\n')
            .context("PLY: header not terminated by 'end_header'")?;
        let line = String::from_utf8_lossy(&bytes[pos..pos + nl]);
        pos += nl + 1;
        let mut t = line.split_ascii_whitespace();
        match t.next() {
            Some("end_header") => break,
            Some("format") => {
                format = Some(match t.next() {
                    Some("ascii") => Format::Ascii,
                    Some("binary_little_endian") => Format::Le,
                    Some("binary_big_endian") => Format::Be,
                    other => bail!("PLY header line {line_no}: unknown format {other:?}"),
                })
            }
            Some("element") => {
                let name = t.next().unwrap_or("").to_string();
                let count = t
                    .next()
                    .and_then(|c| c.parse().ok())
                    .with_context(|| format!("PLY header line {line_no}: invalid element count"))?;
                elements.push(Element {
                    name,
                    count,
                    props: Vec::new(),
                });
            }
            Some("property") => {
                let el = elements.last_mut().with_context(|| {
                    format!("PLY header line {line_no}: property before element")
                })?;
                let bad = || {
                    format!(
                        "PLY header line {line_no}: invalid property '{}'",
                        line.trim()
                    )
                };
                let ty = t.next().with_context(bad)?;
                let kind = if ty == "list" {
                    let c = t.next().and_then(Scalar::parse).with_context(bad)?;
                    let i = t.next().and_then(Scalar::parse).with_context(bad)?;
                    Kind::List(c, i)
                } else {
                    Kind::Scalar(Scalar::parse(ty).with_context(bad)?)
                };
                let name = t.next().with_context(bad)?.to_string();
                el.props.push(Prop { name, kind });
            }
            _ => {} // ply, comment, obj_info, blank
        }
    }
    let format = format.context("PLY: missing format line")?;
    Ok((format, elements, pos))
}

pub(crate) fn load(bytes: &[u8]) -> Result<Vec<TriMesh>> {
    let (format, elements, body) = parse_header(bytes)?;
    match format {
        Format::Ascii => read_body(
            &elements,
            &mut Ascii {
                b: bytes,
                pos: body,
            },
        ),
        Format::Le | Format::Be => read_body(
            &elements,
            &mut Bin {
                b: bytes,
                pos: body,
                le: format == Format::Le,
            },
        ),
    }
}

fn read_body(elements: &[Element], src: &mut impl Values) -> Result<Vec<TriMesh>> {
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normals: Vec<[f32; 3]> = Vec::new();
    let mut has_normals = false;
    let mut indices: Vec<u32> = Vec::new();
    let mut have_vertices = false;
    let mut poly: Vec<u32> = Vec::with_capacity(8);

    for el in elements {
        let is_vertex = el.name == "vertex" && !have_vertices;
        let is_face = el.name == "face";
        // Slot for each scalar prop: 0..3 = xyz, 3..6 = normal, usize::MAX = ignore.
        let slots: Vec<usize> = el
            .props
            .iter()
            .map(|p| match (is_vertex, p.name.as_str()) {
                (true, "x") => 0,
                (true, "y") => 1,
                (true, "z") => 2,
                (true, "nx") => 3,
                (true, "ny") => 4,
                (true, "nz") => 5,
                _ => usize::MAX,
            })
            .collect();
        let face_prop = if is_face {
            el.props.iter().position(|p| {
                matches!(p.kind, Kind::List(..))
                    && (p.name == "vertex_indices" || p.name == "vertex_index")
            })
        } else {
            None
        };
        if is_vertex {
            have_vertices = true;
            if !(0..3).all(|s| slots.contains(&s)) {
                bail!("PLY: vertex element lacks x/y/z properties");
            }
            has_normals = (3..6).all(|s| slots.contains(&s));
            let cap = el.count.min(1 << 26);
            positions.reserve(cap);
            if has_normals {
                normals.reserve(cap);
            }
        }
        if is_face {
            indices.reserve(el.count.min(1 << 26) * 3);
        }
        let count = if el.props.is_empty() { 0 } else { el.count };
        for r in 0..count {
            let mut vals = [0f64; 6];
            for (pi, p) in el.props.iter().enumerate() {
                match p.kind {
                    Kind::Scalar(s) => {
                        let v = src.read(s).with_context(|| {
                            format!("PLY: element '{}' #{r}, property '{}'", el.name, p.name)
                        })?;
                        if slots[pi] != usize::MAX {
                            vals[slots[pi]] = v;
                        }
                    }
                    Kind::List(cs, is) => {
                        let ctx = || format!("PLY: element '{}' #{r}, list '{}'", el.name, p.name);
                        let n = src.read(cs).with_context(ctx)?;
                        if !(0.0..=1e9).contains(&n) {
                            bail!("{}: invalid list length {n}", ctx());
                        }
                        let n = n as usize;
                        if Some(pi) == face_prop {
                            poly.clear();
                            for _ in 0..n {
                                let v = src.read(is).with_context(ctx)?;
                                if !(0.0..u32::MAX as f64).contains(&v) {
                                    bail!("{}: invalid vertex index {v}", ctx());
                                }
                                poly.push(v as u32);
                            }
                            crate::fan(&poly, &mut indices);
                        } else {
                            src.skip(is, n).with_context(ctx)?;
                        }
                    }
                }
            }
            if is_vertex {
                positions.push([vals[0] as f32, vals[1] as f32, vals[2] as f32]);
                if has_normals {
                    normals.push(normalize([vals[3] as f32, vals[4] as f32, vals[5] as f32]));
                }
            }
        }
    }
    if positions.is_empty() || indices.is_empty() {
        bail!("PLY contains no faces (point clouds are not supported)");
    }
    if let Some(&bad) = indices.iter().find(|&&i| i as usize >= positions.len()) {
        bail!(
            "PLY: face references vertex {bad} but only {} vertices exist",
            positions.len()
        );
    }
    let mut mesh = TriMesh {
        positions,
        indices,
        ..Default::default()
    };
    if has_normals && !normals.contains(&[0.0; 3]) {
        mesh.normals = normals;
    } else {
        mesh.compute_normals(DEFAULT_CREASE_DEG);
    }
    Ok(vec![mesh])
}
