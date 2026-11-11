//! The persistent, parametric document: components, an ordered feature
//! timeline and user parameters. Everything here is plain data (serde).

use crate::expr::Parameter;
use crate::sketch::{CId, Sketch};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;

pub type FeatureId = u32;
pub type CompId = u32;

pub const FORMAT_VERSION: u32 = 1;

/// Body identity = (creating feature, index within that feature's output).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(transparent)]
pub struct BodyId(pub u64);

impl BodyId {
    pub fn new(feature: FeatureId, index: u32) -> Self {
        BodyId(((feature as u64) << 20) | index as u64)
    }
    pub fn feature(self) -> FeatureId {
        (self.0 >> 20) as FeatureId
    }
    pub fn index(self) -> u32 {
        (self.0 & 0xFFFFF) as u32
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum OriginPlane {
    XY,
    XZ,
    YZ,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FaceRef {
    pub body: BodyId,
    pub name: String,
    /// last known face centre, used as a fallback when the name is lost
    #[serde(default)]
    pub hint: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EdgeRef {
    pub body: BodyId,
    pub name: String,
    #[serde(default)]
    pub hint: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PlaneRef {
    Origin(OriginPlane),
    Face(FaceRef),
    /// a construction plane feature
    Plane(FeatureId),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ProfileRef {
    /// A closed region of a sketch, identified by the curves bounding it
    /// (and an interior point as tie-breaker / fallback).
    Sketch { sketch: FeatureId, curves: Vec<CId>, hint: [f64; 2] },
    /// A planar face of a body ("press-pull").
    Face(FaceRef),
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AxisRef {
    X,
    Y,
    Z,
    SketchLine { sketch: FeatureId, curve: CId },
    Edge(EdgeRef),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum Operation {
    #[default]
    NewBody,
    Join,
    Cut,
    Intersect,
}

impl Operation {
    pub const ALL: [Operation; 4] = [Operation::NewBody, Operation::Join, Operation::Cut, Operation::Intersect];
    pub fn label(self) -> &'static str {
        match self {
            Operation::NewBody => "New body",
            Operation::Join => "Join",
            Operation::Cut => "Cut",
            Operation::Intersect => "Intersect",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum ExtentMode {
    #[default]
    OneSide,
    /// `distance` on both sides
    Symmetric,
    /// `distance` along the normal, `distance2` against it
    TwoSides,
    /// through the whole model (direction from the sign of `distance`)
    ThroughAll,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Extrude {
    pub profiles: Vec<ProfileRef>,
    pub distance: String,
    #[serde(default)]
    pub distance2: String,
    #[serde(default)]
    pub mode: ExtentMode,
    /// offset of the start plane along the profile normal
    #[serde(default)]
    pub start_offset: String,
    #[serde(default)]
    pub op: Operation,
    /// bodies to join/cut/intersect; empty = automatic (overlapping bodies)
    #[serde(default)]
    pub targets: Vec<BodyId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Revolve {
    pub profiles: Vec<ProfileRef>,
    pub axis: AxisRef,
    pub angle: String,
    #[serde(default)]
    pub op: Operation,
    #[serde(default)]
    pub targets: Vec<BodyId>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Fillet {
    pub edges: Vec<EdgeRef>,
    pub radius: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Chamfer {
    pub edges: Vec<EdgeRef>,
    pub distance: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Shell {
    pub body: Option<BodyId>,
    pub faces: Vec<FaceRef>,
    pub thickness: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffsetPlane {
    pub base: PlaneRef,
    pub offset: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Combine {
    pub target: Option<BodyId>,
    pub tools: Vec<BodyId>,
    pub op: Operation,
    #[serde(default)]
    pub keep_tools: bool,
}

/// Translate/rotate bodies (rigid move).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MoveBodies {
    pub bodies: Vec<BodyId>,
    pub dx: String,
    pub dy: String,
    pub dz: String,
    /// rotation angle (degrees) about `axis`
    #[serde(default)]
    pub rz: String,
    #[serde(default)]
    pub copy: bool,
    /// rotation centre; None = centre of the selected bodies
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pivot: Option<[f64; 3]>,
    /// rotation axis (X/Y/Z run through the pivot; edges / sketch lines are used as-is)
    #[serde(default = "axis_z")]
    pub axis: AxisRef,
}

fn axis_z() -> AxisRef {
    AxisRef::Z
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Default)]
pub enum PatternKind {
    #[default]
    Linear,
    Circular,
}

/// Copies of bodies along a direction or around an axis.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pattern {
    pub bodies: Vec<BodyId>,
    #[serde(default)]
    pub kind: PatternKind,
    /// direction (linear) or rotation axis (circular)
    pub axis: AxisRef,
    pub count: String,
    /// distance between copies (linear)
    #[serde(default)]
    pub spacing: String,
    /// total angle (circular), 360 = full circle
    #[serde(default)]
    pub angle: String,
    /// second direction for grids (linear only)
    #[serde(default)]
    pub axis2: Option<AxisRef>,
    #[serde(default)]
    pub count2: String,
    #[serde(default)]
    pub spacing2: String,
    /// fuse copies into the original body
    #[serde(default)]
    pub join: bool,
}

/// Mirror bodies across a plane.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Mirror {
    pub bodies: Vec<BodyId>,
    pub plane: PlaneRef,
    #[serde(default)]
    pub join: bool,
}

/// A B-rep imported from STEP/BREP, stored inside the document.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportShape {
    pub source: String,
    /// OCCT BREP text
    pub brep: Arc<String>,
}

/// A triangle mesh imported from STL/OBJ/3MF/..., stored inside the document.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ImportMesh {
    pub source: String,
    #[serde(with = "b64_f32")]
    pub positions: Arc<Vec<f32>>,
    #[serde(with = "b64_u32")]
    pub indices: Arc<Vec<u32>>,
}

// Embedded data is shared via Arc; compare pointers first (O(1) for clones).
impl PartialEq for ImportShape {
    fn eq(&self, o: &Self) -> bool {
        self.source == o.source && (Arc::ptr_eq(&self.brep, &o.brep) || self.brep == o.brep)
    }
}

impl PartialEq for ImportMesh {
    fn eq(&self, o: &Self) -> bool {
        self.source == o.source
            && (Arc::ptr_eq(&self.positions, &o.positions) || self.positions == o.positions)
            && (Arc::ptr_eq(&self.indices, &o.indices) || self.indices == o.indices)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum FeatureKind {
    Sketch(Sketch),
    Extrude(Extrude),
    Revolve(Revolve),
    Fillet(Fillet),
    Chamfer(Chamfer),
    Shell(Shell),
    Plane(OffsetPlane),
    Combine(Combine),
    Move(MoveBodies),
    Pattern(Pattern),
    Mirror(Mirror),
    ImportShape(ImportShape),
    ImportMesh(ImportMesh),
}

impl FeatureKind {
    pub fn type_name(&self) -> &'static str {
        match self {
            FeatureKind::Sketch(_) => "Sketch",
            FeatureKind::Extrude(_) => "Extrude",
            FeatureKind::Revolve(_) => "Revolve",
            FeatureKind::Fillet(_) => "Fillet",
            FeatureKind::Chamfer(_) => "Chamfer",
            FeatureKind::Shell(_) => "Shell",
            FeatureKind::Plane(_) => "Plane",
            FeatureKind::Combine(_) => "Combine",
            FeatureKind::Move(_) => "Move",
            FeatureKind::Pattern(_) => "Pattern",
            FeatureKind::Mirror(_) => "Mirror",
            FeatureKind::ImportShape(_) => "Import",
            FeatureKind::ImportMesh(_) => "Mesh",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Feature {
    pub id: FeatureId,
    pub name: String,
    pub component: CompId,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub suppressed: bool,
    /// visibility of the feature's own geometry (sketches, planes)
    #[serde(default = "yes")]
    pub visible: bool,
    pub kind: FeatureKind,
}

fn yes() -> bool {
    true
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Component {
    pub id: CompId,
    pub name: String,
    pub parent: Option<CompId>,
    #[serde(default = "yes")]
    pub visible: bool,
    #[serde(default)]
    pub color: Option<[f32; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Default)]
pub struct BodyProps {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub component: Option<CompId>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub visible: Option<bool>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub color: Option<[f32; 3]>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Document {
    pub format: u32,
    #[serde(default)]
    pub parameters: Vec<Parameter>,
    pub components: Vec<Component>,
    pub features: Vec<Feature>,
    /// Timeline marker: number of features that are rolled forward.
    pub marker: usize,
    #[serde(default)]
    pub bodies: BTreeMap<BodyId, BodyProps>,
    pub next_id: u32,
}

pub const ROOT: CompId = 0;

impl Default for Document {
    fn default() -> Self {
        Document {
            format: FORMAT_VERSION,
            parameters: vec![],
            components: vec![Component { id: ROOT, name: "Design".into(), parent: None, visible: true, color: None }],
            features: vec![],
            marker: 0,
            bodies: BTreeMap::new(),
            next_id: 1,
        }
    }
}

impl Document {
    pub fn alloc_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
    pub fn feature(&self, id: FeatureId) -> Option<&Feature> {
        self.features.iter().find(|f| f.id == id)
    }
    pub fn feature_mut(&mut self, id: FeatureId) -> Option<&mut Feature> {
        self.features.iter_mut().find(|f| f.id == id)
    }
    pub fn feature_index(&self, id: FeatureId) -> Option<usize> {
        self.features.iter().position(|f| f.id == id)
    }
    pub fn component(&self, id: CompId) -> Option<&Component> {
        self.components.iter().find(|c| c.id == id)
    }
    pub fn component_mut(&mut self, id: CompId) -> Option<&mut Component> {
        self.components.iter_mut().find(|c| c.id == id)
    }
    pub fn children(&self, id: CompId) -> impl Iterator<Item = &Component> {
        self.components.iter().filter(move |c| c.parent == Some(id))
    }
    /// Is the component (and all its ancestors) visible?
    pub fn component_visible(&self, mut id: CompId) -> bool {
        for _ in 0..1000 {
            match self.component(id) {
                Some(c) if !c.visible => return false,
                Some(c) => match c.parent {
                    Some(p) => id = p,
                    None => return true,
                },
                None => return true,
            }
        }
        true
    }
    pub fn is_ancestor(&self, anc: CompId, mut id: CompId) -> bool {
        for _ in 0..1000 {
            if id == anc {
                return true;
            }
            match self.component(id).and_then(|c| c.parent) {
                Some(p) => id = p,
                None => return false,
            }
        }
        false
    }
    pub fn add_component(&mut self, name: &str, parent: CompId) -> CompId {
        let id = self.alloc_id();
        self.components.push(Component { id, name: name.into(), parent: Some(parent), visible: true, color: None });
        id
    }
    /// Unique default name like "Sketch3".
    pub fn next_name(&self, base: &str) -> String {
        let n = self.features.iter().filter(|f| f.kind.type_name() == base).count() + 1;
        let mut i = n;
        loop {
            let cand = format!("{base}{i}");
            if !self.features.iter().any(|f| f.name == cand) {
                return cand;
            }
            i += 1;
        }
    }
    /// Insert a feature at the timeline marker and advance the marker.
    pub fn insert_feature(&mut self, name: String, component: CompId, kind: FeatureKind) -> FeatureId {
        let id = self.alloc_id();
        let at = self.marker.min(self.features.len());
        self.features.insert(at, Feature { id, name, component, suppressed: false, visible: true, kind });
        self.marker = at + 1;
        id
    }
    pub fn delete_feature(&mut self, id: FeatureId) {
        if let Some(i) = self.feature_index(id) {
            self.features.remove(i);
            if self.marker > i {
                self.marker -= 1;
            }
        }
    }
    pub fn body_props(&self, b: BodyId) -> BodyProps {
        self.bodies.get(&b).cloned().unwrap_or_default()
    }

    pub fn to_json(&self) -> anyhow::Result<String> {
        Ok(serde_json::to_string_pretty(self)?)
    }
    pub fn from_json(s: &str) -> anyhow::Result<Document> {
        let d: Document = serde_json::from_str(s)?;
        if d.format > FORMAT_VERSION {
            anyhow::bail!("file was written by a newer version (format {})", d.format);
        }
        Ok(d)
    }
}

mod b64_f32 {
    use base64::Engine;
    use serde::{Deserialize, Deserializer, Serializer};
    use std::sync::Arc;
    pub fn serialize<S: Serializer>(v: &Arc<Vec<f32>>, s: S) -> Result<S::Ok, S::Error> {
        let bytes: Vec<u8> = v.iter().flat_map(|f| f.to_le_bytes()).collect();
        s.serialize_str(&base64::engine::general_purpose::STANDARD.encode(bytes))
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Arc<Vec<f32>>, D::Error> {
        let s = String::deserialize(d)?;
        let b = base64::engine::general_purpose::STANDARD.decode(s).map_err(serde::de::Error::custom)?;
        Ok(Arc::new(b.chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()))
    }
}

mod b64_u32 {
    use base64::Engine;
    use serde::{Deserialize, Deserializer, Serializer};
    use std::sync::Arc;
    pub fn serialize<S: Serializer>(v: &Arc<Vec<u32>>, s: S) -> Result<S::Ok, S::Error> {
        let bytes: Vec<u8> = v.iter().flat_map(|f| f.to_le_bytes()).collect();
        s.serialize_str(&base64::engine::general_purpose::STANDARD.encode(bytes))
    }
    pub fn deserialize<'de, D: Deserializer<'de>>(d: D) -> Result<Arc<Vec<u32>>, D::Error> {
        let s = String::deserialize(d)?;
        let b = base64::engine::general_purpose::STANDARD.decode(s).map_err(serde::de::Error::custom)?;
        Ok(Arc::new(b.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()))
    }
}
