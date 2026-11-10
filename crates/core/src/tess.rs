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

/// Cache of display meshes. Tessellation of B-reps runs on worker threads
/// (on a deep copy of the shape); until a mesh is ready the previous mesh of
/// the same body is shown.
#[derive(Default)]
pub struct MeshCache {
    map: HashMap<u64, (Arc<BodyMesh>, u64)>,
    generation: u64,
    pending: HashMap<u64, std::sync::mpsc::Receiver<Option<BodyMesh>>>,
    /// last mesh shown per body (for stale display while recomputing)
    last_for_body: HashMap<u64, Arc<BodyMesh>>,
    failed: std::collections::HashSet<u64>,
}

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

    /// True while background tessellation is running.
    pub fn busy(&self) -> bool {
        !self.pending.is_empty()
    }

    pub fn get(&mut self, body: &Body) -> Option<Arc<BodyMesh>> {
        let key = Self::key(body)?;
        let generation = self.generation;
        if let Some((m, g)) = self.map.get_mut(&key) {
            *g = generation;
            let m = m.clone();
            self.last_for_body.insert(body.id.0, m.clone());
            return Some(m);
        }
        // finished in the background?
        if let Some(rx) = self.pending.get(&key) {
            match rx.try_recv() {
                Ok(Some(m)) => {
                    self.pending.remove(&key);
                    let m = Arc::new(m);
                    self.map.insert(key, (m.clone(), generation));
                    self.last_for_body.insert(body.id.0, m.clone());
                    return Some(m);
                }
                Ok(None) | Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.pending.remove(&key);
                    self.failed.insert(key);
                }
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    return self.last_for_body.get(&body.id.0).cloned();
                }
            }
        }
        if self.failed.contains(&key) {
            return self.last_for_body.get(&body.id.0).cloned();
        }
        let m = match &body.mesh {
            Some(im) => Arc::new(BodyMesh::from_import(im)),
            None => {
                let faces = body.shape.faces().len();
                let stale = self.last_for_body.get(&body.id.0).cloned();
                if faces > 24 && stale.is_some() || faces > 400 {
                    // heavy: tessellate a deep copy on a worker thread
                    let copy = body.shape.deep_copy();
                    let (tx, rx) = std::sync::mpsc::channel();
                    std::thread::spawn(move || {
                        let r = BodyMesh::from_shape(&copy).ok();
                        let _ = tx.send(r);
                    });
                    self.pending.insert(key, rx);
                    return stale;
                }
                match BodyMesh::from_shape(&body.shape) {
                    Ok(m) => Arc::new(m),
                    Err(e) => {
                        log::warn!("tessellation failed: {e}");
                        self.failed.insert(key);
                        return None;
                    }
                }
            }
        };
        self.map.insert(key, (m.clone(), generation));
        self.last_for_body.insert(body.id.0, m.clone());
        Some(m)
    }

    /// Call once per frame/update; drops meshes unused for a while.
    pub fn tick(&mut self) {
        self.generation += 1;
        let g = self.generation;
        if self.map.len() > 64 {
            self.map.retain(|_, (_, last)| g - *last < 600);
        }
    }
}
