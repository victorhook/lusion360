//! 2D sketches: points, lines, circles, arcs + geometric constraints and
//! driving dimensions, with a damped least-squares (minimum-norm) solver.

use crate::doc::PlaneRef;
use crate::expr::ParamValues;
use glam::{DVec2, DVec3};
use nalgebra::{DMatrix, DVector};
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};

pub type PId = u32;
pub type CId = u32;
pub type KId = u32;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SkPoint {
    pub id: PId,
    pub x: f64,
    pub y: f64,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub fixed: bool,
}

impl SkPoint {
    pub fn pos(&self) -> DVec2 {
        DVec2::new(self.x, self.y)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum CurveGeom {
    Line { a: PId, b: PId },
    Circle { c: PId, r: f64 },
    /// Counter-clockwise from `a` to `b` around `c`.
    Arc { c: PId, a: PId, b: PId },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SkCurve {
    pub id: CId,
    pub geom: CurveGeom,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub construction: bool,
}

impl SkCurve {
    pub fn points(&self) -> Vec<PId> {
        match self.geom {
            CurveGeom::Line { a, b } => vec![a, b],
            CurveGeom::Circle { c, .. } => vec![c],
            CurveGeom::Arc { c, a, b } => vec![c, a, b],
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum DimKind {
    /// Aligned distance between two points.
    Distance(PId, PId),
    HDistance(PId, PId),
    VDistance(PId, PId),
    PointLine(PId, CId),
    Length(CId),
    Radius(CId),
    Diameter(CId),
    /// Angle between two lines, degrees.
    Angle(CId, CId),
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum Constraint {
    Coincident(PId, PId),
    PointOnCurve(PId, CId),
    Horizontal(CId),
    Vertical(CId),
    HorizontalPts(PId, PId),
    VerticalPts(PId, PId),
    Parallel(CId, CId),
    Perpendicular(CId, CId),
    Equal(CId, CId),
    Tangent(CId, CId),
    Concentric(CId, CId),
    Midpoint(PId, CId),
    Dimension {
        kind: DimKind,
        expr: String,
        /// last evaluated value (mm or degrees), for display
        #[serde(default)]
        value: f64,
        /// label offset in sketch units relative to the measured geometry
        #[serde(default)]
        offset: [f64; 2],
    },
}

impl Constraint {
    pub fn label(&self) -> &'static str {
        match self {
            Constraint::Coincident(..) => "Coincident",
            Constraint::PointOnCurve(..) => "Point on curve",
            Constraint::Horizontal(..) | Constraint::HorizontalPts(..) => "Horizontal",
            Constraint::Vertical(..) | Constraint::VerticalPts(..) => "Vertical",
            Constraint::Parallel(..) => "Parallel",
            Constraint::Perpendicular(..) => "Perpendicular",
            Constraint::Equal(..) => "Equal",
            Constraint::Tangent(..) => "Tangent",
            Constraint::Concentric(..) => "Concentric",
            Constraint::Midpoint(..) => "Midpoint",
            Constraint::Dimension { .. } => "Dimension",
        }
    }
    pub fn refs_points(&self) -> Vec<PId> {
        match *self {
            Constraint::Coincident(a, b) | Constraint::HorizontalPts(a, b) | Constraint::VerticalPts(a, b) => vec![a, b],
            Constraint::PointOnCurve(p, _) | Constraint::Midpoint(p, _) => vec![p],
            Constraint::Dimension { kind, .. } => match kind {
                DimKind::Distance(a, b) | DimKind::HDistance(a, b) | DimKind::VDistance(a, b) => vec![a, b],
                DimKind::PointLine(p, _) => vec![p],
                _ => vec![],
            },
            _ => vec![],
        }
    }
    pub fn refs_curves(&self) -> Vec<CId> {
        match *self {
            Constraint::PointOnCurve(_, c) | Constraint::Midpoint(_, c) => vec![c],
            Constraint::Horizontal(c) | Constraint::Vertical(c) => vec![c],
            Constraint::Parallel(a, b)
            | Constraint::Perpendicular(a, b)
            | Constraint::Equal(a, b)
            | Constraint::Tangent(a, b)
            | Constraint::Concentric(a, b) => vec![a, b],
            Constraint::Dimension { kind, .. } => match kind {
                DimKind::PointLine(_, c) | DimKind::Length(c) | DimKind::Radius(c) | DimKind::Diameter(c) => vec![c],
                DimKind::Angle(a, b) => vec![a, b],
                _ => vec![],
            },
            _ => vec![],
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SkConstraint {
    pub id: KId,
    pub c: Constraint,
}

/// Text placed at a sketch point (baseline-left corner).
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SkText {
    pub id: u32,
    pub anchor: PId,
    pub text: String,
    /// capital-letter height (expression, mm)
    pub height: String,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Sketch {
    pub plane: PlaneRef,
    #[serde(default)]
    pub points: Vec<SkPoint>,
    #[serde(default)]
    pub curves: Vec<SkCurve>,
    #[serde(default)]
    pub constraints: Vec<SkConstraint>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub texts: Vec<SkText>,
    #[serde(default = "one")]
    pub next_id: u32,
}

fn one() -> u32 {
    1
}

/// Orthonormal sketch frame in world space.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Frame {
    pub origin: DVec3,
    pub x: DVec3,
    pub y: DVec3,
    pub n: DVec3,
}

impl Frame {
    pub fn new(origin: DVec3, n: DVec3, xhint: DVec3) -> Frame {
        let n = n.normalize();
        let mut x = (xhint - n * xhint.dot(n)).normalize_or_zero();
        if x.length_squared() < 0.5 {
            x = n.any_orthonormal_vector();
        }
        let y = n.cross(x);
        Frame { origin, x, y, n }
    }
    pub fn to3(&self, p: DVec2) -> DVec3 {
        self.origin + self.x * p.x + self.y * p.y
    }
    pub fn to2(&self, p: DVec3) -> DVec2 {
        let d = p - self.origin;
        DVec2::new(d.dot(self.x), d.dot(self.y))
    }
    pub fn dir3(&self, d: DVec2) -> DVec3 {
        self.x * d.x + self.y * d.y
    }
    pub fn offset(&self, d: f64) -> Frame {
        Frame { origin: self.origin + self.n * d, ..*self }
    }
}

#[derive(Debug, Clone, Default)]
pub struct SolveReport {
    pub converged: bool,
    pub residual: f64,
    pub dof: usize,
    /// points whose position is fully determined
    pub fixed_points: HashSet<PId>,
    /// curves that are fully determined
    pub fixed_curves: HashSet<CId>,
    /// constraints that could not be satisfied or failed to evaluate
    pub failed: Vec<(KId, String)>,
}

impl SolveReport {
    pub fn fully_constrained(&self) -> bool {
        self.converged && self.dof == 0
    }
}

impl Sketch {
    pub fn new(plane: PlaneRef) -> Self {
        Sketch { plane, points: vec![], curves: vec![], constraints: vec![], texts: vec![], next_id: 1 }
    }
    fn alloc(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }
    pub fn point(&self, id: PId) -> Option<&SkPoint> {
        self.points.iter().find(|p| p.id == id)
    }
    pub fn point_mut(&mut self, id: PId) -> Option<&mut SkPoint> {
        self.points.iter_mut().find(|p| p.id == id)
    }
    pub fn pos(&self, id: PId) -> DVec2 {
        self.point(id).map(|p| p.pos()).unwrap_or(DVec2::ZERO)
    }
    pub fn curve(&self, id: CId) -> Option<&SkCurve> {
        self.curves.iter().find(|c| c.id == id)
    }
    pub fn add_point(&mut self, p: DVec2) -> PId {
        let id = self.alloc();
        self.points.push(SkPoint { id, x: p.x, y: p.y, fixed: false });
        id
    }
    pub fn add_curve(&mut self, geom: CurveGeom, construction: bool) -> CId {
        let id = self.alloc();
        self.curves.push(SkCurve { id, geom, construction });
        id
    }
    pub fn add_line(&mut self, a: PId, b: PId) -> CId {
        self.add_curve(CurveGeom::Line { a, b }, false)
    }
    pub fn add_text(&mut self, anchor: PId, text: &str, height: &str) -> u32 {
        let id = self.alloc();
        self.texts.push(SkText { id, anchor, text: text.into(), height: height.into() });
        id
    }
    /// Outlines of a text entity in sketch coordinates.
    pub fn text_outlines(&self, t: &SkText, params: &ParamValues) -> Vec<Vec<DVec2>> {
        let h = params.eval(&t.height).unwrap_or(5.0).abs().max(0.01);
        let o = self.pos(t.anchor);
        crate::text::outlines(&t.text, h).into_iter().map(|c| c.into_iter().map(|p| p + o).collect()).collect()
    }
    pub fn add_constraint(&mut self, c: Constraint) -> KId {
        let id = self.alloc();
        self.constraints.push(SkConstraint { id, c });
        id
    }
    pub fn line_pts(&self, c: CId) -> Option<(DVec2, DVec2)> {
        match self.curve(c)?.geom {
            CurveGeom::Line { a, b } => Some((self.pos(a), self.pos(b))),
            _ => None,
        }
    }
    pub fn radius_of(&self, c: CId) -> Option<f64> {
        match self.curve(c)?.geom {
            CurveGeom::Circle { r, .. } => Some(r),
            CurveGeom::Arc { c, a, .. } => Some((self.pos(a) - self.pos(c)).length()),
            _ => None,
        }
    }

    /// Delete curves / points / constraints and everything depending on them.
    pub fn delete(&mut self, pts: &[PId], curves: &[CId], cons: &[KId]) {
        let dead_p: HashSet<PId> = pts.iter().copied().collect();
        let mut dead_c: HashSet<CId> = curves.iter().copied().collect();
        for c in &self.curves {
            if c.points().iter().any(|p| dead_p.contains(p)) {
                dead_c.insert(c.id);
            }
        }
        // points owned by deleted curves are dropped unless still used elsewhere
        let orphans: HashSet<PId> =
            self.curves.iter().filter(|c| dead_c.contains(&c.id)).flat_map(|c| c.points()).collect();
        self.curves.retain(|c| !dead_c.contains(&c.id));
        let used: HashSet<PId> = self.curves.iter().flat_map(|c| c.points()).collect();
        self.points.retain(|p| !dead_p.contains(&p.id) && !(orphans.contains(&p.id) && !used.contains(&p.id)));
        let dead_k: HashSet<KId> = cons.iter().copied().collect();
        let live_p: HashSet<PId> = self.points.iter().map(|p| p.id).collect();
        // texts are addressed like curves (by id); their anchor must survive
        self.texts.retain(|t| !dead_c.contains(&t.id) && live_p.contains(&t.anchor));
        let live_c: HashSet<CId> = self.curves.iter().map(|c| c.id).collect();
        self.constraints.retain(|k| {
            !dead_k.contains(&k.id)
                && k.c.refs_points().iter().all(|p| live_p.contains(p))
                && k.c.refs_curves().iter().all(|c| live_c.contains(c))
        });
    }

    /// Merge point `drop` into `keep` (used when a coincident constraint is
    /// created between two free endpoints while drawing).
    pub fn merge_points(&mut self, keep: PId, drop: PId) {
        if keep == drop {
            return;
        }
        let rep = |p: &mut PId| {
            if *p == drop {
                *p = keep
            }
        };
        for c in &mut self.curves {
            match &mut c.geom {
                CurveGeom::Line { a, b } => {
                    rep(a);
                    rep(b)
                }
                CurveGeom::Circle { c, .. } => rep(c),
                CurveGeom::Arc { c, a, b } => {
                    rep(c);
                    rep(a);
                    rep(b)
                }
            }
        }
        for k in &mut self.constraints {
            match &mut k.c {
                Constraint::Coincident(a, b) | Constraint::HorizontalPts(a, b) | Constraint::VerticalPts(a, b) => {
                    rep(a);
                    rep(b)
                }
                Constraint::PointOnCurve(p, _) | Constraint::Midpoint(p, _) => rep(p),
                Constraint::Dimension { kind, .. } => match kind {
                    DimKind::Distance(a, b) | DimKind::HDistance(a, b) | DimKind::VDistance(a, b) => {
                        rep(a);
                        rep(b)
                    }
                    DimKind::PointLine(p, _) => rep(p),
                    _ => {}
                },
                _ => {}
            }
        }
        if self.point(drop).map(|p| p.fixed).unwrap_or(false) {
            if let Some(k) = self.point_mut(keep) {
                k.fixed = true;
            }
        }
        self.points.retain(|p| p.id != drop);
        // remove degenerate constraints (e.g. coincident(a,a))
        self.constraints.retain(|k| !matches!(k.c, Constraint::Coincident(a, b) if a == b));
    }

    /// Current measured value of a dimension.
    pub fn measure(&self, kind: &DimKind) -> f64 {
        match *kind {
            DimKind::Distance(a, b) => (self.pos(a) - self.pos(b)).length(),
            DimKind::HDistance(a, b) => (self.pos(b).x - self.pos(a).x).abs(),
            DimKind::VDistance(a, b) => (self.pos(b).y - self.pos(a).y).abs(),
            DimKind::PointLine(p, l) => match self.line_pts(l) {
                Some((a, b)) => dist_point_line(self.pos(p), a, b),
                None => 0.0,
            },
            DimKind::Length(l) => self.line_pts(l).map(|(a, b)| (b - a).length()).unwrap_or(0.0),
            DimKind::Radius(c) => self.radius_of(c).unwrap_or(0.0),
            DimKind::Diameter(c) => 2.0 * self.radius_of(c).unwrap_or(0.0),
            DimKind::Angle(l1, l2) => match (self.line_pts(l1), self.line_pts(l2)) {
                (Some((a1, b1)), Some((a2, b2))) => {
                    let (u, v) = ((b1 - a1).normalize_or_zero(), (b2 - a2).normalize_or_zero());
                    let ang = u.perp_dot(v).atan2(u.dot(v)).to_degrees().abs();
                    if ang > 90.0 {
                        180.0 - ang
                    } else {
                        ang
                    }
                }
                _ => 0.0,
            },
        }
    }

    /// Solve all constraints in place. `pinned` points are held fixed (used
    /// for dragging). Returns a report including degrees of freedom.
    pub fn solve(&mut self, params: &ParamValues, pinned: &[PId], analyze: bool) -> SolveReport {
        let mut report = SolveReport::default();
        // Evaluate dimension expressions.
        let mut dim_vals: HashMap<KId, f64> = HashMap::new();
        for k in &mut self.constraints {
            if let Constraint::Dimension { expr, value, kind, .. } = &mut k.c {
                match params.eval(expr) {
                    Ok(v) => {
                        let v = match kind {
                            DimKind::Angle(..) => v,
                            _ => v.abs(),
                        };
                        *value = v;
                        dim_vals.insert(k.id, v);
                    }
                    Err(e) => report.failed.push((k.id, e)),
                }
            }
        }

        let pinned: HashSet<PId> = pinned.iter().copied().collect();
        let mut x: Vec<f64> = Vec::new();
        let mut pvar: HashMap<PId, usize> = HashMap::new();
        for p in &self.points {
            if !p.fixed && !pinned.contains(&p.id) {
                pvar.insert(p.id, x.len());
                x.push(p.x);
                x.push(p.y);
            }
        }
        let mut rvar: HashMap<CId, usize> = HashMap::new();
        for c in &self.curves {
            if let CurveGeom::Circle { r, .. } = c.geom {
                rvar.insert(c.id, x.len());
                x.push(r);
            }
        }
        let fixed_pos: HashMap<PId, DVec2> = self.points.iter().map(|p| (p.id, p.pos())).collect();
        let curves: HashMap<CId, CurveGeom> = self.curves.iter().map(|c| (c.id, c.geom)).collect();

        // Pick configuration-dependent branches from the initial state.
        let mut eqs: Vec<Eq> = Vec::new();
        for k in &self.constraints {
            let failed = report.failed.iter().any(|(id, _)| *id == k.id);
            if failed {
                continue;
            }
            if let Some(e) = build_eq(self, k, &dim_vals, &curves) {
                eqs.push(e);
            }
        }
        for c in &self.curves {
            if let CurveGeom::Arc { c: cc, a, b } = c.geom {
                eqs.push(Eq { kid: 0, kind: EqKind::ArcRadius(cc, a, b) });
            }
        }

        let ctx = Ctx { pvar: &pvar, rvar: &rvar, fixed: &fixed_pos, curves: &curves };
        let n = x.len();
        let mut r = Vec::new();
        ctx.residuals(&eqs, &x, &mut r);
        let mut norm = sq(&r);
        let mut lambda = 1e-6;
        let mut iter = 0;
        while norm > 1e-20 && iter < 200 && n > 0 && !r.is_empty() {
            iter += 1;
            let j = ctx.jacobian(&eqs, &x, r.len());
            let m = r.len();
            // minimum-norm damped step: dx = -J^T (J J^T + λI)^-1 r
            let jt = j.transpose();
            let jjt = &j * &jt;
            let rv = DVector::from_column_slice(&r);
            let mut accepted = false;
            for _ in 0..12 {
                let mut a = jjt.clone();
                for i in 0..m {
                    a[(i, i)] += lambda * (1.0 + jjt[(i, i)]);
                }
                let y = match a.clone().cholesky() {
                    Some(ch) => ch.solve(&rv),
                    None => match a.lu().solve(&rv) {
                        Some(y) => y,
                        None => {
                            lambda *= 10.0;
                            continue;
                        }
                    },
                };
                let dx = &jt * y;
                let xn: Vec<f64> = x.iter().zip(dx.iter()).map(|(a, d)| a - d).collect();
                let mut rn = Vec::with_capacity(m);
                ctx.residuals(&eqs, &xn, &mut rn);
                let nn = sq(&rn);
                if nn < norm {
                    x = xn;
                    r = rn;
                    norm = nn;
                    lambda = (lambda * 0.3).max(1e-12);
                    accepted = true;
                    break;
                }
                lambda *= 8.0;
            }
            if !accepted {
                break;
            }
        }
        report.residual = norm.sqrt();
        report.converged = norm < 1e-14;
        if !report.converged {
            // name the constraints that are not satisfied
            let mut idx = 0;
            for e in &eqs {
                let cnt = e.count();
                let worst = r[idx..idx + cnt].iter().fold(0.0f64, |m, v| m.max(v.abs()));
                if worst > 1e-6 && e.kid != 0 && !report.failed.iter().any(|(k, _)| *k == e.kid) {
                    report.failed.push((e.kid, "cannot be satisfied (conflicting constraints?)".into()));
                }
                idx += cnt;
            }
        }
        // write back
        for p in &mut self.points {
            if let Some(&i) = pvar.get(&p.id) {
                p.x = x[i];
                p.y = x[i + 1];
            }
        }
        for c in &mut self.curves {
            if let CurveGeom::Circle { r, .. } = &mut c.geom {
                if let Some(&i) = rvar.get(&c.id) {
                    *r = x[i].abs();
                }
            }
        }

        if analyze {
            // degrees-of-freedom analysis via the null space of J
            let m = r.len();
            let free: Vec<bool> = if n == 0 {
                vec![]
            } else if m == 0 {
                vec![true; n]
            } else {
                let j = ctx.jacobian(&eqs, &x, m);
                let jtj = j.transpose() * &j;
                let eig = jtj.symmetric_eigen();
                let maxe = eig.eigenvalues.iter().fold(0.0f64, |a, b| a.max(b.abs())).max(1e-12);
                let mut freeness = vec![0.0f64; n];
                let mut rank = 0;
                for (k, ev) in eig.eigenvalues.iter().enumerate() {
                    if ev.abs() < 1e-9 * maxe.max(1.0) {
                        for (i, f) in freeness.iter_mut().enumerate() {
                            f.clone_from(&(*f + eig.eigenvectors[(i, k)].powi(2)));
                        }
                    } else {
                        rank += 1;
                    }
                }
                report.dof = n - rank;
                freeness.iter().map(|f| *f > 1e-8).collect()
            };
            if m == 0 {
                report.dof = n;
            }
            for p in &self.points {
                let fixed = match pvar.get(&p.id) {
                    Some(&i) => !free[i] && !free[i + 1],
                    None => true,
                };
                if fixed {
                    report.fixed_points.insert(p.id);
                }
            }
            for c in &self.curves {
                let pts_fixed = c.points().iter().all(|p| report.fixed_points.contains(p));
                let r_fixed = match rvar.get(&c.id) {
                    Some(&i) => !free[i],
                    None => true,
                };
                if pts_fixed && r_fixed {
                    report.fixed_curves.insert(c.id);
                }
            }
        }
        report
    }

    // ------------------------------------------------------------ geometry helpers

    /// Distance from `p` to curve `c` (sketch units).
    pub fn curve_distance(&self, c: &SkCurve, p: DVec2) -> f64 {
        match c.geom {
            CurveGeom::Line { a, b } => dist_point_segment(p, self.pos(a), self.pos(b)),
            CurveGeom::Circle { c, r } => ((p - self.pos(c)).length() - r).abs(),
            CurveGeom::Arc { c, a, b } => {
                let (cc, pa, pb) = (self.pos(c), self.pos(a), self.pos(b));
                if angle_in_arc(cc, pa, pb, p) {
                    ((p - cc).length() - (pa - cc).length()).abs()
                } else {
                    (p - pa).length().min((p - pb).length())
                }
            }
        }
    }

    /// Polyline approximation in sketch coordinates.
    pub fn curve_polyline(&self, c: &SkCurve, tol: f64) -> Vec<DVec2> {
        match c.geom {
            CurveGeom::Line { a, b } => vec![self.pos(a), self.pos(b)],
            CurveGeom::Circle { c, r } => {
                let n = segs(r, std::f64::consts::TAU, tol);
                let cc = self.pos(c);
                (0..=n).map(|i| cc + DVec2::from_angle(i as f64 / n as f64 * std::f64::consts::TAU) * r).collect()
            }
            CurveGeom::Arc { c, a, b } => {
                let (cc, pa, pb) = (self.pos(c), self.pos(a), self.pos(b));
                let r = (pa - cc).length();
                let a0 = (pa - cc).to_angle();
                let mut sweep = (pb - cc).to_angle() - a0;
                if sweep <= 1e-12 {
                    sweep += std::f64::consts::TAU;
                }
                let n = segs(r, sweep, tol);
                (0..=n).map(|i| cc + DVec2::from_angle(a0 + sweep * i as f64 / n as f64) * r).collect()
            }
        }
    }

    pub fn bounds(&self) -> Option<(DVec2, DVec2)> {
        let mut lo = DVec2::splat(f64::INFINITY);
        let mut hi = DVec2::splat(f64::NEG_INFINITY);
        for c in &self.curves {
            for p in self.curve_polyline(c, 0.5) {
                lo = lo.min(p);
                hi = hi.max(p);
            }
        }
        for p in &self.points {
            lo = lo.min(p.pos());
            hi = hi.max(p.pos());
        }
        for t in &self.texts {
            for c in self.text_outlines(t, &ParamValues::default()) {
                for p in c {
                    lo = lo.min(p);
                    hi = hi.max(p);
                }
            }
        }
        (lo.x <= hi.x).then_some((lo, hi))
    }
}

fn segs(r: f64, sweep: f64, tol: f64) -> usize {
    let tol = tol.max(1e-4);
    let step = if r > tol { 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos() } else { 1.0 };
    ((sweep / step.max(0.02)).ceil() as usize).clamp(8, 512)
}

pub fn dist_point_line(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let d = b - a;
    let l = d.length();
    if l < 1e-12 {
        return (p - a).length();
    }
    (d.perp_dot(p - a) / l).abs()
}

pub fn dist_point_segment(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let d = b - a;
    let l2 = d.length_squared();
    if l2 < 1e-24 {
        return (p - a).length();
    }
    let t = ((p - a).dot(d) / l2).clamp(0.0, 1.0);
    (p - (a + d * t)).length()
}

/// Is the direction of `p` from `c` within the CCW arc a→b?
pub fn angle_in_arc(c: DVec2, a: DVec2, b: DVec2, p: DVec2) -> bool {
    let a0 = (a - c).to_angle();
    let norm = |x: f64| x.rem_euclid(std::f64::consts::TAU);
    let sweep = {
        let s = norm((b - c).to_angle() - a0);
        if s < 1e-12 {
            std::f64::consts::TAU
        } else {
            s
        }
    };
    norm((p - c).to_angle() - a0) <= sweep + 1e-9
}

fn sq(r: &[f64]) -> f64 {
    r.iter().map(|v| v * v).sum()
}

// ------------------------------------------------------------------ equations

#[derive(Debug, Clone)]
struct Eq {
    kid: KId,
    kind: EqKind,
}

#[derive(Debug, Clone)]
enum EqKind {
    Coincident(PId, PId),
    OnLine(PId, PId, PId),
    OnCircle(PId, CId),
    Horizontal(PId, PId),
    Vertical(PId, PId),
    Parallel(PId, PId, PId, PId),
    Perpendicular(PId, PId, PId, PId),
    EqualLen(PId, PId, PId, PId),
    EqualRad(CId, CId),
    TangentLine(PId, PId, CId),
    TangentCircles(CId, CId, bool),
    Concentric(PId, PId),
    Midpoint(PId, PId, PId),
    Dist(PId, PId, f64),
    HDist(PId, PId, f64),
    VDist(PId, PId, f64),
    PointLineDist(PId, PId, PId, f64),
    Radius(CId, f64),
    Angle(PId, PId, PId, PId, f64),
    ArcRadius(PId, PId, PId),
}

impl Eq {
    fn count(&self) -> usize {
        match self.kind {
            EqKind::Coincident(..) | EqKind::Concentric(..) | EqKind::Midpoint(..) => 2,
            _ => 1,
        }
    }
}

fn center_of(g: &CurveGeom) -> Option<PId> {
    match *g {
        CurveGeom::Circle { c, .. } | CurveGeom::Arc { c, .. } => Some(c),
        _ => None,
    }
}

fn build_eq(sk: &Sketch, k: &SkConstraint, dims: &HashMap<KId, f64>, curves: &HashMap<CId, CurveGeom>) -> Option<Eq> {
    use CurveGeom::*;
    let line = |c: CId| match curves.get(&c)? {
        Line { a, b } => Some((*a, *b)),
        _ => None,
    };
    let kind = match k.c {
        Constraint::Coincident(a, b) => EqKind::Coincident(a, b),
        Constraint::PointOnCurve(p, c) => match curves.get(&c)? {
            Line { a, b } => EqKind::OnLine(p, *a, *b),
            _ => EqKind::OnCircle(p, c),
        },
        Constraint::Horizontal(c) => {
            let (a, b) = line(c)?;
            EqKind::Horizontal(a, b)
        }
        Constraint::Vertical(c) => {
            let (a, b) = line(c)?;
            EqKind::Vertical(a, b)
        }
        Constraint::HorizontalPts(a, b) => EqKind::Horizontal(a, b),
        Constraint::VerticalPts(a, b) => EqKind::Vertical(a, b),
        Constraint::Parallel(c1, c2) => {
            let (a, b) = line(c1)?;
            let (c, d) = line(c2)?;
            EqKind::Parallel(a, b, c, d)
        }
        Constraint::Perpendicular(c1, c2) => {
            let (a, b) = line(c1)?;
            let (c, d) = line(c2)?;
            EqKind::Perpendicular(a, b, c, d)
        }
        Constraint::Equal(c1, c2) => match (line(c1), line(c2)) {
            (Some((a, b)), Some((c, d))) => EqKind::EqualLen(a, b, c, d),
            (None, None) => EqKind::EqualRad(c1, c2),
            _ => return None,
        },
        Constraint::Tangent(c1, c2) => match (line(c1), line(c2)) {
            (Some((a, b)), None) => EqKind::TangentLine(a, b, c2),
            (None, Some((a, b))) => EqKind::TangentLine(a, b, c1),
            (None, None) => {
                // internal vs external tangency picked from current configuration
                let (g1, g2) = (curves.get(&c1)?, curves.get(&c2)?);
                let d = (sk.pos(center_of(g1)?) - sk.pos(center_of(g2)?)).length();
                let (r1, r2) = (sk.radius_of(c1)?, sk.radius_of(c2)?);
                let internal = (d - (r1 - r2).abs()).abs() < (d - (r1 + r2)).abs();
                EqKind::TangentCircles(c1, c2, internal)
            }
            _ => return None,
        },
        Constraint::Concentric(c1, c2) => {
            EqKind::Concentric(center_of(curves.get(&c1)?)?, center_of(curves.get(&c2)?)?)
        }
        Constraint::Midpoint(p, c) => {
            let (a, b) = line(c)?;
            EqKind::Midpoint(p, a, b)
        }
        Constraint::Dimension { kind, .. } => {
            let v = *dims.get(&k.id)?;
            match kind {
                DimKind::Distance(a, b) => EqKind::Dist(a, b, v),
                DimKind::HDistance(a, b) => {
                    let s = (sk.pos(b).x - sk.pos(a).x).signum();
                    EqKind::HDist(a, b, v * if s == 0.0 { 1.0 } else { s })
                }
                DimKind::VDistance(a, b) => {
                    let s = (sk.pos(b).y - sk.pos(a).y).signum();
                    EqKind::VDist(a, b, v * if s == 0.0 { 1.0 } else { s })
                }
                DimKind::PointLine(p, l) => {
                    let (a, b) = line(l)?;
                    let side = (sk.pos(b) - sk.pos(a)).perp_dot(sk.pos(p) - sk.pos(a)).signum();
                    EqKind::PointLineDist(p, a, b, v * if side == 0.0 { 1.0 } else { side })
                }
                DimKind::Length(l) => {
                    let (a, b) = line(l)?;
                    EqKind::Dist(a, b, v)
                }
                DimKind::Radius(c) => EqKind::Radius(c, v),
                DimKind::Diameter(c) => EqKind::Radius(c, v / 2.0),
                DimKind::Angle(l1, l2) => {
                    let (a, b) = line(l1)?;
                    let (c, d) = line(l2)?;
                    // choose the representation of the angle nearest the current one
                    let u = (sk.pos(b) - sk.pos(a)).normalize_or_zero();
                    let w = (sk.pos(d) - sk.pos(c)).normalize_or_zero();
                    let cur = u.perp_dot(w).atan2(u.dot(w));
                    let t = v.to_radians();
                    let cands = [t, -t, std::f64::consts::PI - t, t - std::f64::consts::PI];
                    let best = cands
                        .iter()
                        .copied()
                        .min_by(|x, y| ang_diff(*x, cur).total_cmp(&ang_diff(*y, cur)))
                        .unwrap();
                    EqKind::Angle(a, b, c, d, best)
                }
            }
        }
    };
    Some(Eq { kid: k.id, kind })
}

fn ang_diff(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d)
}

struct Ctx<'a> {
    pvar: &'a HashMap<PId, usize>,
    rvar: &'a HashMap<CId, usize>,
    fixed: &'a HashMap<PId, DVec2>,
    curves: &'a HashMap<CId, CurveGeom>,
}

impl Ctx<'_> {
    fn p(&self, x: &[f64], id: PId) -> DVec2 {
        match self.pvar.get(&id) {
            Some(&i) => DVec2::new(x[i], x[i + 1]),
            None => self.fixed.get(&id).copied().unwrap_or(DVec2::ZERO),
        }
    }
    fn center(&self, x: &[f64], c: CId) -> DVec2 {
        match self.curves.get(&c) {
            Some(CurveGeom::Circle { c, .. }) | Some(CurveGeom::Arc { c, .. }) => self.p(x, *c),
            _ => DVec2::ZERO,
        }
    }
    fn r(&self, x: &[f64], c: CId) -> f64 {
        match self.curves.get(&c) {
            Some(CurveGeom::Circle { r, .. }) => match self.rvar.get(&c) {
                Some(&i) => x[i],
                None => *r,
            },
            Some(CurveGeom::Arc { c, a, .. }) => (self.p(x, *a) - self.p(x, *c)).length(),
            _ => 0.0,
        }
    }
    fn residuals(&self, eqs: &[Eq], x: &[f64], out: &mut Vec<f64>) {
        out.clear();
        for e in eqs {
            match e.kind {
                EqKind::Coincident(a, b) | EqKind::Concentric(a, b) => {
                    let d = self.p(x, a) - self.p(x, b);
                    out.push(d.x);
                    out.push(d.y);
                }
                EqKind::OnLine(p, a, b) => {
                    let (pa, pb) = (self.p(x, a), self.p(x, b));
                    let d = pb - pa;
                    let l = d.length().max(1e-9);
                    out.push(d.perp_dot(self.p(x, p) - pa) / l);
                }
                EqKind::OnCircle(p, c) => out.push((self.p(x, p) - self.center(x, c)).length() - self.r(x, c)),
                EqKind::Horizontal(a, b) => out.push(self.p(x, a).y - self.p(x, b).y),
                EqKind::Vertical(a, b) => out.push(self.p(x, a).x - self.p(x, b).x),
                EqKind::Parallel(a, b, c, d) => {
                    let u = (self.p(x, b) - self.p(x, a)).normalize_or_zero();
                    let v = (self.p(x, d) - self.p(x, c)).normalize_or_zero();
                    out.push(u.perp_dot(v));
                }
                EqKind::Perpendicular(a, b, c, d) => {
                    let u = (self.p(x, b) - self.p(x, a)).normalize_or_zero();
                    let v = (self.p(x, d) - self.p(x, c)).normalize_or_zero();
                    out.push(u.dot(v));
                }
                EqKind::EqualLen(a, b, c, d) => {
                    out.push((self.p(x, b) - self.p(x, a)).length() - (self.p(x, d) - self.p(x, c)).length())
                }
                EqKind::EqualRad(c1, c2) => out.push(self.r(x, c1) - self.r(x, c2)),
                EqKind::TangentLine(a, b, c) => {
                    let (pa, pb) = (self.p(x, a), self.p(x, b));
                    let d = pb - pa;
                    let l = d.length().max(1e-9);
                    out.push((d.perp_dot(self.center(x, c) - pa) / l).abs() - self.r(x, c));
                }
                EqKind::TangentCircles(c1, c2, internal) => {
                    let d = (self.center(x, c1) - self.center(x, c2)).length();
                    let (r1, r2) = (self.r(x, c1), self.r(x, c2));
                    out.push(if internal { d - (r1 - r2).abs() } else { d - (r1 + r2) });
                }
                EqKind::Midpoint(p, a, b) => {
                    let m = (self.p(x, a) + self.p(x, b)) * 0.5;
                    let d = self.p(x, p) - m;
                    out.push(d.x);
                    out.push(d.y);
                }
                EqKind::Dist(a, b, v) => out.push((self.p(x, a) - self.p(x, b)).length() - v),
                EqKind::HDist(a, b, v) => out.push(self.p(x, b).x - self.p(x, a).x - v),
                EqKind::VDist(a, b, v) => out.push(self.p(x, b).y - self.p(x, a).y - v),
                EqKind::PointLineDist(p, a, b, v) => {
                    let (pa, pb) = (self.p(x, a), self.p(x, b));
                    let d = pb - pa;
                    let l = d.length().max(1e-9);
                    out.push(d.perp_dot(self.p(x, p) - pa) / l - v);
                }
                EqKind::Radius(c, v) => out.push(self.r(x, c) - v),
                EqKind::Angle(a, b, c, d, t) => {
                    let u = (self.p(x, b) - self.p(x, a)).normalize_or_zero();
                    let w = (self.p(x, d) - self.p(x, c)).normalize_or_zero();
                    // sin(θ - t) where θ is the signed angle u→w
                    out.push(u.perp_dot(w) * t.cos() - u.dot(w) * t.sin());
                }
                EqKind::ArcRadius(c, a, b) => {
                    let pc = self.p(x, c);
                    out.push((self.p(x, a) - pc).length() - (self.p(x, b) - pc).length())
                }
            }
        }
    }
    fn jacobian(&self, eqs: &[Eq], x: &[f64], m: usize) -> DMatrix<f64> {
        let n = x.len();
        let mut j = DMatrix::zeros(m, n);
        let mut xp = x.to_vec();
        let mut rp = Vec::with_capacity(m);
        let mut rm = Vec::with_capacity(m);
        for k in 0..n {
            let h = 1e-6 * (1.0 + x[k].abs());
            xp[k] = x[k] + h;
            self.residuals(eqs, &xp, &mut rp);
            xp[k] = x[k] - h;
            self.residuals(eqs, &xp, &mut rm);
            xp[k] = x[k];
            for i in 0..m {
                j[(i, k)] = (rp[i] - rm[i]) / (2.0 * h);
            }
        }
        j
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::doc::{OriginPlane, PlaneRef};

    fn rect_sketch() -> (Sketch, [PId; 4], [CId; 4]) {
        let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
        let p = [
            s.add_point(DVec2::new(0.0, 0.0)),
            s.add_point(DVec2::new(9.0, 0.5)),
            s.add_point(DVec2::new(10.0, 6.0)),
            s.add_point(DVec2::new(-0.5, 5.0)),
        ];
        let l = [s.add_line(p[0], p[1]), s.add_line(p[1], p[2]), s.add_line(p[2], p[3]), s.add_line(p[3], p[0])];
        (s, p, l)
    }

    #[test]
    fn rectangle_constraints() {
        let (mut s, p, l) = rect_sketch();
        s.add_constraint(Constraint::Horizontal(l[0]));
        s.add_constraint(Constraint::Horizontal(l[2]));
        s.add_constraint(Constraint::Vertical(l[1]));
        s.add_constraint(Constraint::Vertical(l[3]));
        let pv = ParamValues::default();
        let r = s.solve(&pv, &[], true);
        assert!(r.converged);
        assert_eq!(r.dof, 4);
        s.add_constraint(Constraint::Dimension { kind: DimKind::Length(l[0]), expr: "20".into(), value: 0.0, offset: [0.0; 2] });
        s.add_constraint(Constraint::Dimension { kind: DimKind::Length(l[1]), expr: "10".into(), value: 0.0, offset: [0.0; 2] });
        s.point_mut(p[0]).unwrap().fixed = true;
        let r = s.solve(&pv, &[], true);
        assert!(r.converged, "{:?}", r);
        assert_eq!(r.dof, 0);
        assert!(r.fixed_points.len() == 4);
        assert!(((s.pos(p[1]) - s.pos(p[0])).length() - 20.0).abs() < 1e-7);
        assert!(((s.pos(p[2]) - s.pos(p[1])).length() - 10.0).abs() < 1e-7);
        assert!((s.pos(p[2]).y - s.pos(p[3]).y).abs() < 1e-9);
    }

    #[test]
    fn circle_tangent_and_dims() {
        let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
        let a = s.add_point(DVec2::new(-10.0, 0.0));
        let b = s.add_point(DVec2::new(10.0, 0.0));
        let l = s.add_line(a, b);
        let c = s.add_point(DVec2::new(0.0, 4.0));
        let ci = s.add_curve(CurveGeom::Circle { c, r: 3.0 }, false);
        s.add_constraint(Constraint::Tangent(l, ci));
        s.add_constraint(Constraint::Dimension { kind: DimKind::Diameter(ci), expr: "2 * 5".into(), value: 0.0, offset: [0.0; 2] });
        let r = s.solve(&ParamValues::default(), &[], false);
        assert!(r.converged);
        let rr = s.radius_of(ci).unwrap();
        assert!((rr - 5.0).abs() < 1e-7);
        let (pa, pb) = s.line_pts(l).unwrap();
        assert!((dist_point_line(s.pos(c), pa, pb) - 5.0).abs() < 1e-7);
    }

    #[test]
    fn conflicting_reports_failure() {
        let (mut s, _p, l) = rect_sketch();
        s.add_constraint(Constraint::Horizontal(l[0]));
        s.add_constraint(Constraint::Vertical(l[0]));
        s.add_constraint(Constraint::Dimension { kind: DimKind::Length(l[0]), expr: "5".into(), value: 0.0, offset: [0.0; 2] });
        let r = s.solve(&ParamValues::default(), &[], false);
        assert!(!r.converged);
        assert!(!r.failed.is_empty());
    }

    #[test]
    fn drag_with_pin() {
        let (mut s, p, l) = rect_sketch();
        s.add_constraint(Constraint::Horizontal(l[0]));
        s.add_constraint(Constraint::Vertical(l[1]));
        s.point_mut(p[1]).unwrap().x = 30.0;
        s.point_mut(p[1]).unwrap().y = 3.0;
        let r = s.solve(&ParamValues::default(), &[p[1]], false);
        assert!(r.converged);
        assert!((s.pos(p[0]).y - 3.0).abs() < 1e-9);
        assert!((s.pos(p[2]).x - 30.0).abs() < 1e-9);
    }

    #[test]
    fn angle_dimension() {
        let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
        let o = s.add_point(DVec2::ZERO);
        s.point_mut(o).unwrap().fixed = true;
        let a = s.add_point(DVec2::new(10.0, 0.0));
        let b = s.add_point(DVec2::new(5.0, 6.0));
        let l1 = s.add_line(o, a);
        let l2 = s.add_line(o, b);
        s.add_constraint(Constraint::Horizontal(l1));
        s.add_constraint(Constraint::Dimension { kind: DimKind::Angle(l1, l2), expr: "30".into(), value: 0.0, offset: [0.0; 2] });
        let r = s.solve(&ParamValues::default(), &[], false);
        assert!(r.converged);
        assert!((s.measure(&DimKind::Angle(l1, l2)) - 30.0).abs() < 1e-6);
    }
}
