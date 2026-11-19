//! Display tessellation with caching (keyed by B-rep identity).

use crate::doc::ImportMesh;
use crate::model::Body;
use lusion_kernel::Shape;
use std::collections::HashMap;
use std::sync::Arc;

#[derive(Debug, Default)]
pub struct BodyMesh {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
    /// face index (B-rep face order) for each triangle
    pub tri_face: Vec<u32>,
    /// polyline per B-rep edge (edge order)
    pub edges: Vec<Vec<[f32; 3]>>,
    /// per edge: joins two faces smoothly (drawn faint)
    pub edge_smooth: Vec<bool>,
    pub bbox: ([f32; 3], [f32; 3]),
}

impl BodyMesh {
    pub fn from_shape(s: &Shape) -> anyhow::Result<BodyMesh> {
        let diag = s.bbox().map(|(a, b)| (b - a).length()).unwrap_or(1.0);
        let lin = (diag * 0.0008).clamp(0.002, 0.5);
        let m = s.mesh(lin, 0.3).map_err(|e| anyhow::anyhow!(e.0))?;
        let positions: Vec<[f32; 3]> = m.positions.chunks(3).map(|c| [c[0], c[1], c[2]]).collect();
        let normals: Vec<[f32; 3]> = m.normals.chunks(3).map(|c| [c[0], c[1], c[2]]).collect();
        let mut edges = Vec::with_capacity(m.edge_ranges.len().saturating_sub(1));
        for w in m.edge_ranges.windows(2) {
            let (a, b) = (w[0] as usize, w[1] as usize);
            edges.push((a..b).map(|i| [m.edge_points[3 * i], m.edge_points[3 * i + 1], m.edge_points[3 * i + 2]]).collect());
        }
        let bbox = bounds(&positions);
        let edge_smooth = m.edge_smooth.iter().map(|s| *s != 0).collect();
        Ok(BodyMesh { positions, normals, indices: m.indices, tri_face: m.tri_face, edges, edge_smooth, bbox })
    }

    pub fn from_import(m: &ImportMesh) -> BodyMesh {
        let mut tm = lusion_meshio::TriMesh {
            name: String::new(),
            positions: m.positions.chunks(3).map(|c| [c[0], c[1], c[2]]).collect(),
            normals: vec![],
            indices: (*m.indices).clone(),
            color: None,
        };
        tm.compute_normals(30.0);
        let bbox = bounds(&tm.positions);
        let n = tm.indices.len() / 3;
        BodyMesh {
            positions: tm.positions,
            normals: tm.normals,
            indices: tm.indices,
            tri_face: vec![0; n],
            edges: vec![],
            edge_smooth: vec![],
            bbox,
        }
    }

    pub fn from_trimesh(tm: &lusion_meshio::TriMesh) -> BodyMesh {
        let n = tm.indices.len() / 3;
        BodyMesh {
            positions: tm.positions.clone(),
            normals: tm.normals.clone(),
            indices: tm.indices.clone(),
            tri_face: vec![0; n],
            edges: vec![],
            edge_smooth: vec![],
            bbox: bounds(&tm.positions),
        }
    }
}

fn bounds(p: &[[f32; 3]]) -> ([f32; 3], [f32; 3]) {
    let mut lo = [f32::INFINITY; 3];
    let mut hi = [f32::NEG_INFINITY; 3];
    for v in p {
        for k in 0..3 {
            lo[k] = lo[k].min(v[k]);
            hi[k] = hi[k].max(v[k]);
        }
    }
    (lo, hi)
}

/// Cache of display meshes.
///
/// Small bodies are tessellated synchronously (the time it took is remembered
/// per body). Slow bodies are tessellated on a worker thread from a deep copy;
/// at most one job runs per body, and while it runs the newest finished mesh
/// of that body is shown. This keeps interactive edits (dragging an extrude)
/// smooth: the preview updates as fast as meshing allows instead of waiting
/// for a mesh of the exact current shape.
///
/// Keys are addresses (OCCT TShape pointer / import buffer), so every entry
/// holds its source alive: otherwise a freed shape's address gets reused by a
/// new shape (constantly, while dragging) and the new shape would be shown
/// with the old shape's mesh.
#[derive(Default)]
pub struct MeshCache {
    map: HashMap<u64, (Arc<BodyMesh>, u64, Keep)>,
    generation: u64,
    /// body id -> (shape key, receiver, source) of the job in flight
    inflight: HashMap<u64, (u64, std::sync::mpsc::Receiver<(Option<BodyMesh>, f64)>, Keep)>,
    /// last mesh shown per body (for stale display while recomputing)
    last_for_body: HashMap<u64, Arc<BodyMesh>>,
    /// last tessellation time per body (ms)
    cost_ms: HashMap<u64, f64>,
    failed: HashMap<u64, Keep>,
}

/// Keeps the object a cache key was derived from alive.
#[derive(Clone)]
#[allow(dead_code)]
enum Keep {
    Shape(Shape),
    Import(Arc<ImportMesh>),
}

impl Keep {
    fn of(body: &Body) -> Keep {
        match &body.mesh {
            Some(m) => Keep::Import(m.clone()),
            None => Keep::Shape(body.shape.clone()),
        }
    }
}

/// Bodies cheaper than this are meshed inline.
const SYNC_BUDGET_MS: f64 = 30.0;

impl MeshCache {
    fn key(body: &Body) -> Option<u64> {
        Some(match &body.mesh {
            Some(m) => Arc::as_ptr(&m.positions) as u64 ^ 0x9e37_79b9_7f4a_7c15,
            None => {
                if body.shape.is_null() {
                    return None;
                }
                body.shape.hash()
            }
        })
    }

    /// Is the cached mesh for this body up to date (not a stale stand-in)?
    pub fn is_current(&self, body: &Body) -> bool {
        Self::key(body).map(|k| self.map.contains_key(&k)).unwrap_or(false)
    }

    /// True while background tessellation is running.
    pub fn busy(&self) -> bool {
        !self.inflight.is_empty()
    }

    /// Collect a finished background job for `bid`, if any.
    fn poll(&mut self, bid: u64) {
        let Some((key, rx, _)) = self.inflight.get(&bid) else { return };
        let key = *key;
        match rx.try_recv() {
            Ok((Some(m), ms)) => {
                let (_, _, keep) = self.inflight.remove(&bid).unwrap();
                let m = Arc::new(m);
                self.map.insert(key, (m.clone(), self.generation, keep));
                self.last_for_body.insert(bid, m);
                self.cost_ms.insert(bid, ms);
            }
            Ok((None, _)) | Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                let (_, _, keep) = self.inflight.remove(&bid).unwrap();
                self.failed.insert(key, keep);
            }
            Err(std::sync::mpsc::TryRecvError::Empty) => {}
        }
    }

    pub fn get(&mut self, body: &Body) -> Option<Arc<BodyMesh>> {
        let key = Self::key(body)?;
        let bid = body.id.0;
        let generation = self.generation;
        self.poll(bid);
        if let Some((m, g, _)) = self.map.get_mut(&key) {
            *g = generation;
            let m = m.clone();
            self.last_for_body.insert(bid, m.clone());
            return Some(m);
        }
        let stale = self.last_for_body.get(&bid).cloned();
        if self.failed.contains_key(&key) {
            return stale;
        }
        let m = match &body.mesh {
            Some(im) => Arc::new(BodyMesh::from_import(im)),
            None => {
                let known = self.cost_ms.get(&bid).copied();
                let cheap = match known {
                    Some(ms) => ms < SYNC_BUDGET_MS,
                    None => body.shape.faces().len() <= 400 || stale.is_none(),
                };
                if !cheap {
                    // one job per body; a newer request waits for the running one
                    if !self.inflight.contains_key(&bid) {
                        let copy = body.shape.deep_copy();
                        let (tx, rx) = std::sync::mpsc::channel();
                        std::thread::spawn(move || {
                            let t = std::time::Instant::now();
                            let r = BodyMesh::from_shape(&copy).ok();
                            let _ = tx.send((r, t.elapsed().as_secs_f64() * 1000.0));
                        });
                        self.inflight.insert(bid, (key, rx, Keep::of(body)));
                    }
                    return stale;
                }
                let t = std::time::Instant::now();
                match BodyMesh::from_shape(&body.shape) {
                    Ok(m) => {
                        self.cost_ms.insert(bid, t.elapsed().as_secs_f64() * 1000.0);
                        Arc::new(m)
                    }
                    Err(e) => {
                        log::warn!("tessellation failed: {e}");
                        self.failed.insert(key, Keep::of(body));
                        return stale;
                    }
                }
            }
        };
        self.map.insert(key, (m.clone(), generation, Keep::of(body)));
        self.last_for_body.insert(bid, m.clone());
        Some(m)
    }

    /// Display mesh for an arbitrary shape (e.g. a preview tool), cached by identity.
    pub fn get_shape(&mut self, shape: &Shape) -> Option<Arc<BodyMesh>> {
        if shape.is_null() {
            return None;
        }
        let key = shape.hash() ^ 0x5bd1_e995;
        let g = self.generation;
        if let Some((m, last, _)) = self.map.get_mut(&key) {
            *last = g;
            return Some(m.clone());
        }
        let m = Arc::new(BodyMesh::from_shape(shape).ok()?);
        self.map.insert(key, (m.clone(), g, Keep::Shape(shape.clone())));
        Some(m)
    }

    /// Add a mesh computed elsewhere (e.g. by a background recompute).
    pub fn insert_premeshed(&mut self, shape: &Shape, m: BodyMesh, tool: bool) {
        if shape.is_null() {
            return;
        }
        let key = if tool { shape.hash() ^ 0x5bd1_e995 } else { shape.hash() };
        self.map.insert(key, (Arc::new(m), self.generation, Keep::Shape(shape.clone())));
    }

    /// Call once per frame/update; drops meshes unused for a while.
    pub fn tick(&mut self) {
        self.generation += 1;
        let g = self.generation;
        // entries hold their shapes: drop unused ones sooner while many pile up (dragging)
        if self.map.len() > 64 {
            self.map.retain(|_, (_, last, _)| g - *last < 120);
        }
        if self.failed.len() > 64 {
            self.failed.clear();
        }
    }
}
