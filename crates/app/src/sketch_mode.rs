//! Sketch editing: drawing tools, snapping/inference, constraints,
//! dimensions, trim, dragging, and the sketch overlay.

use crate::app::{App, Mode};
use crate::camera::Ray;
use crate::icons as ic;
use crate::render::{PointInst, Scene};
use crate::theme;
use crate::viewport::{push_dashed, push_line, push_polyline, View};
use egui::{Color32, PointerButton, Pos2, RichText, Stroke};
use glam::{DVec2, Vec2, Vec3};
use lusion_core::doc::*;
use lusion_core::expr::fmt_num;
use lusion_core::sketch::*;
use std::collections::HashSet;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConKind {
    Coincident,
    HorizVert,
    Parallel,
    Perpendicular,
    Equal,
    Tangent,
    Concentric,
    Midpoint,
    Fix,
}

impl ConKind {
    pub const ALL: [ConKind; 9] = [
        ConKind::HorizVert,
        ConKind::Coincident,
        ConKind::Parallel,
        ConKind::Perpendicular,
        ConKind::Equal,
        ConKind::Tangent,
        ConKind::Concentric,
        ConKind::Midpoint,
        ConKind::Fix,
    ];
    pub fn label(self) -> &'static str {
        match self {
            ConKind::Coincident => "Coincident",
            ConKind::HorizVert => "Horizontal / Vertical",
            ConKind::Parallel => "Parallel",
            ConKind::Perpendicular => "Perpendicular",
            ConKind::Equal => "Equal",
            ConKind::Tangent => "Tangent",
            ConKind::Concentric => "Concentric",
            ConKind::Midpoint => "Midpoint",
            ConKind::Fix => "Fix / Unfix",
        }
    }
    pub fn glyph(self) -> &'static str {
        match self {
            ConKind::Coincident => ic::DOT_OUTLINE,
            ConKind::HorizVert => ic::ARROWS_LEFT_RIGHT,
            ConKind::Parallel => "//",
            ConKind::Perpendicular => ic::ANGLE,
            ConKind::Equal => ic::EQUALS,
            ConKind::Tangent => ic::CIRCLE_HALF,
            ConKind::Concentric => ic::TARGET,
            ConKind::Midpoint => ic::TRIANGLE,
            ConKind::Fix => ic::LOCK,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SkTool {
    Select,
    Line,
    Rect,
    CenterRect,
    Circle,
    Arc,
    Spline,
    Point,
    Dimension,
    Trim,
    Fillet,
    Text,
    Offset,
    Project,
    Con(ConKind),
}

impl SkTool {
    pub fn hint(self) -> &'static str {
        match self {
            SkTool::Select => "Click to select, drag to move. Double-click a dimension to edit it.",
            SkTool::Line => "Click points to draw lines. Esc / right-click / Enter ends the chain · D: dimension the last line · Ctrl: no snapping",
            SkTool::Rect => "Click two opposite corners · R: draw from the centre instead · Ctrl: no snapping",
            SkTool::CenterRect => "Click the centre, then a corner · R: draw from a corner instead · Ctrl: no snapping",
            SkTool::Circle => "Click the centre, then a point on the circle.",
            SkTool::Arc => "Click start, end, then a point the arc passes through.",
            SkTool::Spline => "Click points the curve passes through. Click the first point to close; double-click, Enter or right-click to finish.",
            SkTool::Point => "Click to place points.",
            SkTool::Dimension => "Click a line, circle, two points, point + line or two lines. Click again to place.",
            SkTool::Trim => "Click the part of a curve to remove.",
            SkTool::Fillet => "Click a corner where two lines meet to round it.",
            SkTool::Text => "Click where the text should start (baseline, left).",
            SkTool::Offset => "Click a curve (its connected chain is used), move to choose side & distance, click — or type a value.",
            SkTool::Project => "Click a face to project its outline, or an edge, onto the sketch plane.",
            SkTool::Con(_) => "Pick the entities to constrain.",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum SkEnt {
    Point(PId),
    Curve(CId),
    Con(KId),
    Text(u32),
}

#[derive(Clone, Copy, Debug)]
pub enum Snap {
    Point(PId),
    Mid(CId),
    OnCurve(CId),
    /// a point of existing model geometry projected onto the sketch plane
    Model(DVec2),
}

struct Drag {
    /// points moved rigidly with the mouse (pinned while solving)
    points: Vec<(PId, DVec2)>,
    /// circle radius drag
    radius: Option<CId>,
    /// dimension label drag
    label: Option<(KId, [f64; 2])>,
    start: DVec2,
    before: Document,
    moved: bool,
}

pub struct SketchSession {
    pub fid: FeatureId,
    pub tool: SkTool,
    pub prev_marker: usize,
    chain: Vec<(DVec2, Option<PId>)>,
    picks: Vec<SkEnt>,
    pub selection: Vec<SkEnt>,
    hover: Option<SkEnt>,
    cursor: Option<DVec2>,
    snap: Option<Snap>,
    hv_hint: Option<bool>,
    drag: Option<Drag>,
    pub report: SolveReport,
    edit_dim: Option<(KId, String, bool)>,
    /// world units per screen pixel at the sketch plane
    px: f64,
    pub look_pending: bool,
    /// camera before entering the sketch (restored on finish)
    cam_before: Option<crate::camera::Camera>,
    /// projected model points to snap to (computed lazily)
    refs: Option<Vec<DVec2>>,
    /// value typed while drawing (Fusion-style heads-up input)
    typed: Option<String>,
    /// offset tool: chosen chain (start curve)
    offset_src: Option<CId>,
    /// project tool: model entity under the cursor
    proj_hover: Option<crate::app::Pick>,
    /// text being edited: (id, text, height, newly created, original text, original height)
    text_edit: Option<(u32, String, String, bool, String, String)>,
    /// world units per pixel at the view centre (stable scale for the grid)
    grid_px: f64,
    /// alignment guides shown while snapping: (aligned-to point, cursor)
    guides: Vec<(DVec2, DVec2)>,
    /// the cursor sits on a grid point
    on_grid: bool,
    /// grid snapping enabled (mirrors the persisted setting, for drawing)
    grid_on: bool,
    /// curve made by the last drawing click (D dimensions it)
    last_curve: Option<CId>,
}

// ------------------------------------------------------------------ enter / leave

impl SketchSession {
    pub fn proj_hover(&self) -> Option<crate::app::Pick> {
        self.proj_hover
    }
}

pub fn enter(app: &mut App, fid: FeatureId) {
    let before = app.doc.clone();
    enter_with_before(app, fid, before, false);
}

pub fn enter_with_before(app: &mut App, fid: FeatureId, before: Document, created: bool) {
    if let Mode::Sketch(_) = app.mode {
        finish(app);
    }
    let Some(idx) = app.doc.feature_index(fid) else { return };
    if app.cmd.is_some() {
        crate::commands::cancel(app);
    }
    let prev_marker = if created { app.doc.marker.max(idx + 1) } else { app.doc.marker };
    if created {
        let mut b = before;
        b.marker = b.marker.min(b.features.len());
        app.commit(b);
    }
    app.doc.marker = idx + 1;
    app.selection.clear();
    app.hover = None;
    let mut s = SketchSession {
        fid,
        tool: SkTool::Line,
        prev_marker,
        chain: vec![],
        picks: vec![],
        selection: vec![],
        hover: None,
        cursor: None,
        snap: None,
        grid_px: 0.0,
        guides: vec![],
        on_grid: false,
        grid_on: true,
        last_curve: None,
        hv_hint: None,
        drag: None,
        report: SolveReport::default(),
        edit_dim: None,
        px: 0.1,
        look_pending: true,
        cam_before: Some(app.cam.cam),
        refs: None,
        typed: None,
        offset_src: None,
        proj_hover: None,
        text_edit: None,
    };
    if !created {
        s.tool = SkTool::Select;
    }
    app.mode = Mode::Sketch(Box::new(s));
    solve(app, &[], true);
    app.model.update(&app.doc);
}

pub fn finish(app: &mut App) {
    let Mode::Sketch(s) = std::mem::replace(&mut app.mode, Mode::Model) else { return };
    if let Some(c) = s.cam_before {
        // back to where we were, but keep any ortho/persp change
        let mut to = c;
        to.ortho = app.cam.cam.ortho;
        app.cam.animate_to(to);
    }
    if let Some(idx) = app.doc.feature_index(s.fid) {
        if s.prev_marker > idx {
            app.doc.marker = s.prev_marker.min(app.doc.features.len());
        }
        // an untouched new sketch is discarded (and its creation undone)
        let empty = sketch(app, s.fid).map(|sk| sk.curves.is_empty() && sk.points.len() <= 1).unwrap_or(false);
        if empty {
            if let Some(prev) = app.undo.last() {
                if prev.feature(s.fid).is_none() {
                    app.doc = app.undo.pop().unwrap();
                }
            }
            if app.doc.feature(s.fid).is_some() {
                app.doc.delete_feature(s.fid);
            }
        }
    }
    app.model.update(&app.doc);
}

pub fn look_at(app: &mut App, fid: FeatureId) {
    let st = app.display_state();
    let Some(ss) = st.sketches.get(&fid) else { return };
    let f = ss.frame;
    let r = ss.sketch.bounds().map(|(a, b)| ((b - a).length() * 0.6) as f32).filter(|r| *r > 1.0);
    let center = ss.sketch.bounds().map(|(a, b)| f.to3((a + b) * 0.5)).unwrap_or(f.origin);
    app.cam.look_at_plane(center.as_vec3(), f.x.as_vec3(), f.y.as_vec3(), f.n.as_vec3(), r.or(Some(60.0)));
    if let Some(r) = r {
        let mut to = app.cam.cam;
        to.rot = glam::Quat::from_mat3(&glam::Mat3::from_cols(f.x.as_vec3(), f.y.as_vec3(), f.n.as_vec3()));
        to.target = center.as_vec3();
        to.distance = (r / (to.fov_y * 0.5).sin()) * 1.25;
        app.cam.animate_to(to);
    }
}

fn session(app: &mut App) -> Option<&mut SketchSession> {
    match &mut app.mode {
        Mode::Sketch(s) => Some(s),
        _ => None,
    }
}

fn sketch(app: &App, fid: FeatureId) -> Option<&Sketch> {
    match app.doc.feature(fid).map(|f| &f.kind) {
        Some(FeatureKind::Sketch(s)) => Some(s),
        _ => None,
    }
}

fn sketch_mut(app: &mut App, fid: FeatureId) -> Option<&mut Sketch> {
    match app.doc.feature_mut(fid).map(|f| &mut f.kind) {
        Some(FeatureKind::Sketch(s)) => Some(s),
        _ => None,
    }
}

/// Solve the sketch in place; store the report in the session.
fn solve(app: &mut App, pinned: &[PId], analyze: bool) -> bool {
    let Some(fid) = session(app).map(|s| s.fid) else { return false };
    let params = app.model.params.clone();
    // linked projections follow the model as it is before this sketch
    let pre = app.doc.feature_index(fid).map(|i| app.model.state_after(i).clone());
    let frame = frame_of(app, fid);
    let Some(sk) = sketch_mut(app, fid) else { return false };
    if let (Some(pre), Some(frame)) = (pre, frame) {
        if !sk.links.is_empty() {
            lusion_core::model::apply_links(sk, &pre, &frame);
        }
    }
    let rep = sk.solve(&params, pinned, analyze);
    let ok = rep.converged;
    if let Some(s) = session(app) {
        if analyze || !rep.converged {
            s.report = rep;
        } else {
            s.report.converged = rep.converged;
        }
    }
    ok
}

/// Commit a sketch edit as one undo step (marker normalised to the
/// position it will have after finishing the sketch).
fn commit(app: &mut App, mut before: Document) {
    if let Some(s) = session(app) {
        before.marker = s.prev_marker.min(before.features.len());
    }
    app.commit(before);
}

fn frame_of(app: &App, fid: FeatureId) -> Option<Frame> {
    app.display_state().sketches.get(&fid).map(|s| s.frame)
}

// ------------------------------------------------------------------ geometry helpers

fn arc_from_3(a: DVec2, b: DVec2, m: DVec2) -> Option<(DVec2, bool)> {
    // circumcentre of a, m, b; returns (centre, ccw from a to b through m)
    let d = 2.0 * (a.x * (m.y - b.y) + m.x * (b.y - a.y) + b.x * (a.y - m.y));
    if d.abs() < 1e-12 {
        return None;
    }
    let ux = ((a.length_squared()) * (m.y - b.y) + (m.length_squared()) * (b.y - a.y) + (b.length_squared()) * (a.y - m.y)) / d;
    let uy = ((a.length_squared()) * (b.x - m.x) + (m.length_squared()) * (a.x - b.x) + (b.length_squared()) * (m.x - a.x)) / d;
    let c = DVec2::new(ux, uy);
    let ccw = angle_in_arc(c, a, b, m);
    Some((c, ccw))
}

/// Intersections of curve `c` with all other curves, as points.
fn intersections(sk: &Sketch, cid: CId) -> Vec<DVec2> {
    let Some(c) = sk.curve(cid) else { return vec![] };
    let mut out = Vec::new();
    for o in &sk.curves {
        if o.id == cid {
            continue;
        }
        for p in curve_curve(sk, c, o) {
            out.push(p);
        }
    }
    out
}

fn on_curve_range(sk: &Sketch, c: &SkCurve, p: DVec2) -> bool {
    match c.geom {
        CurveGeom::Line { a, b } => {
            let (pa, pb) = (sk.pos(a), sk.pos(b));
            let d = pb - pa;
            let t = (p - pa).dot(d) / d.length_squared().max(1e-24);
            (-1e-9..=1.0 + 1e-9).contains(&t)
        }
        CurveGeom::Circle { .. } => true,
        CurveGeom::Arc { c, a, b } => angle_in_arc(sk.pos(c), sk.pos(a), sk.pos(b), p),
    }
}

fn circle_of(sk: &Sketch, c: &SkCurve) -> Option<(DVec2, f64)> {
    match c.geom {
        CurveGeom::Circle { c, r } => Some((sk.pos(c), r)),
        CurveGeom::Arc { c, a, .. } => Some((sk.pos(c), (sk.pos(a) - sk.pos(c)).length())),
        _ => None,
    }
}

fn curve_curve(sk: &Sketch, c1: &SkCurve, c2: &SkCurve) -> Vec<DVec2> {
    let mut pts = Vec::new();
    match (sk.line_pts(c1.id), sk.line_pts(c2.id)) {
        (Some((a, b)), Some((c, d))) => {
            let r = b - a;
            let s = d - c;
            let den = r.perp_dot(s);
            if den.abs() > 1e-12 {
                let t = (c - a).perp_dot(s) / den;
                pts.push(a + r * t);
            }
        }
        (Some((a, b)), None) | (None, Some((a, b))) => {
            let circ = if sk.line_pts(c1.id).is_some() { c2 } else { c1 };
            if let Some((cc, r)) = circle_of(sk, circ) {
                let d = b - a;
                let f = a - cc;
                let qa = d.dot(d);
                let qb = 2.0 * f.dot(d);
                let qc = f.dot(f) - r * r;
                let disc = qb * qb - 4.0 * qa * qc;
                if disc >= 0.0 && qa > 1e-24 {
                    let sq = disc.sqrt();
                    for t in [(-qb - sq) / (2.0 * qa), (-qb + sq) / (2.0 * qa)] {
                        pts.push(a + d * t);
                    }
                }
            }
        }
        (None, None) => {
            if let (Some((p0, r0)), Some((p1, r1))) = (circle_of(sk, c1), circle_of(sk, c2)) {
                let d = (p1 - p0).length();
                if d > 1e-12 && d <= r0 + r1 + 1e-9 && d >= (r0 - r1).abs() - 1e-9 {
                    let a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
                    let h = (r0 * r0 - a * a).max(0.0).sqrt();
                    let m = p0 + (p1 - p0) * (a / d);
                    let perp = (p1 - p0).perp() / d;
                    pts.push(m + perp * h);
                    if h > 1e-12 {
                        pts.push(m - perp * h);
                    }
                }
            }
        }
    }
    pts.into_iter().filter(|p| on_curve_range(sk, c1, *p) && on_curve_range(sk, c2, *p)).collect()
}

/// Remove the segment of curve `cid` around `at`.
fn trim(sk: &mut Sketch, cid: CId, at: DVec2) -> bool {
    let Some(c) = sk.curve(cid).cloned() else { return false };
    let xs = intersections(sk, cid);
    match c.geom {
        CurveGeom::Line { a, b } => {
            let (pa, pb) = (sk.pos(a), sk.pos(b));
            let d = pb - pa;
            let l2 = d.length_squared().max(1e-24);
            let tc = (at - pa).dot(d) / l2;
            let mut ts: Vec<f64> = xs.iter().map(|p| (*p - pa).dot(d) / l2).filter(|t| *t > 1e-6 && *t < 1.0 - 1e-6).collect();
            ts.sort_by(|x, y| x.total_cmp(y));
            let lo = ts.iter().copied().filter(|t| *t < tc).last();
            let hi = ts.iter().copied().find(|t| *t > tc);
            match (lo, hi) {
                (None, None) => sk.delete(&[], &[cid], &[]),
                (None, Some(h)) => {
                    let np = sk.add_point(pa + d * h);
                    replace_line(sk, cid, np, b);
                    attach(sk, np, cid);
                }
                (Some(l), None) => {
                    let np = sk.add_point(pa + d * l);
                    replace_line(sk, cid, a, np);
                    attach(sk, np, cid);
                }
                (Some(l), Some(h)) => {
                    let p1 = sk.add_point(pa + d * l);
                    let p2 = sk.add_point(pa + d * h);
                    replace_line(sk, cid, a, p1);
                    let nl = sk.add_line(p2, b);
                    if c.construction {
                        if let Some(x) = sk.curves.iter_mut().find(|x| x.id == nl) {
                            x.construction = true;
                        }
                    }
                    attach(sk, p1, cid);
                    attach(sk, p2, nl);
                }
            }
            true
        }
        CurveGeom::Circle { c: cc, r } => {
            let pc = sk.pos(cc);
            let mut angs: Vec<f64> = xs.iter().map(|p| (*p - pc).to_angle()).collect();
            angs.sort_by(|x, y| x.total_cmp(y));
            angs.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
            if angs.len() < 2 {
                sk.delete(&[], &[cid], &[]);
                return true;
            }
            let ac = (at - pc).to_angle();
            // find the interval containing ac (cyclic)
            let n = angs.len();
            let mut k = n - 1;
            for i in 0..n {
                if angs[i] <= ac {
                    k = i;
                }
            }
            let (from, to) = (angs[k], angs[(k + 1) % n]);
            // keep the complement: arc from `to` CCW to `from`
            let pa = sk.add_point(pc + DVec2::from_angle(to) * r);
            let pb = sk.add_point(pc + DVec2::from_angle(from) * r);
            let cons: Vec<KId> = sk.constraints.iter().filter(|k| k.c.refs_curves().contains(&cid)).map(|k| k.id).collect();
            if let Some(x) = sk.curves.iter_mut().find(|x| x.id == cid) {
                x.geom = CurveGeom::Arc { c: cc, a: pa, b: pb };
            }
            // radius dims on a circle remain valid for the arc; drop others that no longer make sense
            let _ = cons;
            attach(sk, pa, cid);
            attach(sk, pb, cid);
            true
        }
        CurveGeom::Arc { c: cc, a, b } => {
            let pc = sk.pos(cc);
            let r = (sk.pos(a) - pc).length();
            let a0 = (sk.pos(a) - pc).to_angle();
            let norm = |x: f64| (x - a0).rem_euclid(std::f64::consts::TAU);
            let sweep = {
                let s = norm((sk.pos(b) - pc).to_angle());
                if s < 1e-12 {
                    std::f64::consts::TAU
                } else {
                    s
                }
            };
            let tc = norm((at - pc).to_angle());
            let mut ts: Vec<f64> = xs.iter().map(|p| norm((*p - pc).to_angle())).filter(|t| *t > 1e-6 && *t < sweep - 1e-6).collect();
            ts.sort_by(|x, y| x.total_cmp(y));
            let lo = ts.iter().copied().filter(|t| *t < tc).last();
            let hi = ts.iter().copied().find(|t| *t > tc);
            let at_ang = |t: f64| pc + DVec2::from_angle(a0 + t) * r;
            match (lo, hi) {
                (None, None) => sk.delete(&[], &[cid], &[]),
                (None, Some(h)) => {
                    let np = sk.add_point(at_ang(h));
                    set_arc(sk, cid, cc, np, b);
                    attach(sk, np, cid);
                }
                (Some(l), None) => {
                    let np = sk.add_point(at_ang(l));
                    set_arc(sk, cid, cc, a, np);
                    attach(sk, np, cid);
                }
                (Some(l), Some(h)) => {
                    let p1 = sk.add_point(at_ang(l));
                    let p2 = sk.add_point(at_ang(h));
                    set_arc(sk, cid, cc, a, p1);
                    let na = sk.add_curve(CurveGeom::Arc { c: cc, a: p2, b }, c.construction);
                    attach(sk, p1, cid);
                    attach(sk, p2, na);
                }
            }
            true
        }
    }
}

/// Round the corner at `pid` (shared by two lines) with a tangent arc.
/// Returns the id of the new radius dimension.
fn sketch_fillet(sk: &mut Sketch, pid: PId) -> Option<KId> {
    let lines: Vec<(CId, PId)> = sk
        .curves
        .iter()
        .filter_map(|c| match c.geom {
            CurveGeom::Line { a, b } if a == pid => Some((c.id, b)),
            CurveGeom::Line { a, b } if b == pid => Some((c.id, a)),
            _ => None,
        })
        .collect();
    let [(l1, o1), (l2, o2)] = lines[..] else { return None };
    let p = sk.pos(pid);
    let (a, b) = (sk.pos(o1), sk.pos(o2));
    let (u1, u2) = ((a - p).normalize_or_zero(), (b - p).normalize_or_zero());
    let theta = u1.angle_to(u2).abs();
    if !(0.05..std::f64::consts::PI - 0.05).contains(&theta) {
        return None;
    }
    let max_d = (a - p).length().min((b - p).length()) * 0.45;
    let r_guess = max_d * (theta / 2.0).tan();
    let r = {
        // a "nice" default radius
        let m = 10f64.powf(r_guess.log10().floor());
        ((r_guess / m).floor().max(1.0)) * m
    };
    let d = r / (theta / 2.0).tan();
    let t1 = p + u1 * d;
    let t2 = p + u2 * d;
    let bis = (u1 + u2).normalize_or_zero();
    let c = p + bis * (r / (theta / 2.0).sin());
    let pt1 = sk.add_point(t1);
    let pt2 = sk.add_point(t2);
    let pc = sk.add_point(c);
    // re-point the lines at the tangent points
    for (l, np) in [(l1, pt1), (l2, pt2)] {
        if let Some(x) = sk.curves.iter_mut().find(|x| x.id == l) {
            if let CurveGeom::Line { a, b } = &mut x.geom {
                if *a == pid {
                    *a = np;
                } else if *b == pid {
                    *b = np;
                }
            }
        }
    }
    // arc runs counter-clockwise from one tangent point to the other
    let ccw = (t1 - c).perp_dot(t2 - c) > 0.0;
    let (sa, sb) = if ccw { (pt1, pt2) } else { (pt2, pt1) };
    let arc = sk.add_curve(CurveGeom::Arc { c: pc, a: sa, b: sb }, false);
    sk.add_constraint(Constraint::Tangent(l1, arc));
    sk.add_constraint(Constraint::Tangent(l2, arc));
    // keep the old corner as a "virtual sharp" on both lines, so overall
    // dimensions keep measuring to the corner (like Fusion)
    sk.add_constraint(Constraint::PointOnCurve(pid, l1));
    sk.add_constraint(Constraint::PointOnCurve(pid, l2));
    for k in sk.constraints.iter_mut() {
        if let Constraint::Dimension { kind, .. } = &mut k.c {
            match *kind {
                DimKind::Length(l) if l == l1 => *kind = DimKind::Distance(o1, pid),
                DimKind::Length(l) if l == l2 => *kind = DimKind::Distance(o2, pid),
                _ => {}
            }
        }
    }
    let value = r;
    let kid = sk.add_constraint(Constraint::Dimension { kind: DimKind::Radius(arc), expr: fmt_num(r), value, offset: ((c - p).normalize_or(DVec2::X) * -r * 1.6).to_array() });
    Some(kid)
}

fn replace_line(sk: &mut Sketch, cid: CId, a: PId, b: PId) {
    let old = sk.curve(cid).map(|c| c.points()).unwrap_or_default();
    if let Some(x) = sk.curves.iter_mut().find(|x| x.id == cid) {
        x.geom = CurveGeom::Line { a, b };
    }
    drop_orphans(sk, &old);
}

fn set_arc(sk: &mut Sketch, cid: CId, c: PId, a: PId, b: PId) {
    let old = sk.curve(cid).map(|c| c.points()).unwrap_or_default();
    if let Some(x) = sk.curves.iter_mut().find(|x| x.id == cid) {
        x.geom = CurveGeom::Arc { c, a, b };
    }
    drop_orphans(sk, &old);
}

fn drop_orphans(sk: &mut Sketch, pts: &[PId]) {
    let used: HashSet<PId> = sk.curves.iter().flat_map(|c| c.points()).collect();
    let dead: Vec<PId> = pts.iter().copied().filter(|p| !used.contains(p) && !sk.point(*p).map(|q| q.fixed).unwrap_or(false)).collect();
    if !dead.is_empty() {
        sk.delete(&dead, &[], &[]);
    }
}

/// Keep a trim point on whichever other curve passes through it.
fn attach(sk: &mut Sketch, p: PId, own: CId) {
    let pos = sk.pos(p);
    let tol = 1e-6 * (1.0 + pos.length());
    let others: Vec<CId> = sk.curves.iter().filter(|c| c.id != own && sk.curve_distance(c, pos) < tol).map(|c| c.id).collect();
    for o in others {
        sk.add_constraint(Constraint::PointOnCurve(p, o));
    }
}

// ------------------------------------------------------------------ picking in 2D

fn text_hit(sk: &Sketch, p: DVec2, params: &lusion_core::expr::ParamValues) -> Option<u32> {
    sk.texts.iter().find_map(|t| {
        let h = params.eval(&t.height).unwrap_or(5.0).abs();
        let (lo, hi) = lusion_core::text::bounds(&t.text, h)?;
        let o = sk.pos(t.anchor);
        let pad = h * 0.1;
        let (lo, hi) = (lo + o - DVec2::splat(pad), hi + o + DVec2::splat(pad));
        (p.x >= lo.x && p.x <= hi.x && p.y >= lo.y && p.y <= hi.y).then_some(t.id)
    })
}

fn nearest_point(sk: &Sketch, p: DVec2, tol: f64, exclude: &[PId]) -> Option<PId> {
    sk.points
        .iter()
        .filter(|q| !exclude.contains(&q.id))
        .map(|q| (q.id, (q.pos() - p).length()))
        .filter(|(_, d)| *d < tol)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id)
}

fn nearest_curve(sk: &Sketch, p: DVec2, tol: f64) -> Option<CId> {
    sk.curves
        .iter()
        .map(|c| (c.id, sk.curve_distance(c, p)))
        .chain(sk.splines.iter().map(|sp| (sp.id, sk.spline_distance(sp, p))))
        .filter(|(_, d)| *d < tol)
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(id, _)| id)
}

fn compute_snap(sk: &Sketch, p: DVec2, px: f64, exclude: &[PId]) -> (DVec2, Option<Snap>) {
    if let Some(id) = nearest_point(sk, p, px * 10.0, exclude) {
        return (sk.pos(id), Some(Snap::Point(id)));
    }
    for c in &sk.curves {
        if let Some((a, b)) = sk.line_pts(c.id) {
            let m = (a + b) * 0.5;
            if (m - p).length() < px * 8.0 {
                return (m, Some(Snap::Mid(c.id)));
            }
        }
    }
    if let Some(cid) = nearest_curve(sk, p, px * 6.0) {
        let c = sk.curve(cid).unwrap();
        let q = match c.geom {
            CurveGeom::Line { a, b } => {
                let (pa, pb) = (sk.pos(a), sk.pos(b));
                let d = pb - pa;
                let t = ((p - pa).dot(d) / d.length_squared().max(1e-24)).clamp(0.0, 1.0);
                pa + d * t
            }
            CurveGeom::Circle { c, r } => sk.pos(c) + (p - sk.pos(c)).normalize_or_zero() * r,
            CurveGeom::Arc { c, a, .. } => sk.pos(c) + (p - sk.pos(c)).normalize_or_zero() * (sk.pos(a) - sk.pos(c)).length(),
        };
        return (q, Some(Snap::OnCurve(cid)));
    }
    (p, None)
}

/// Sketch grid spacing for a scale (world units per pixel): a 1-2-5 step at
/// least ~20 px apart, and how many steps make a major line.
pub fn grid_step(px: f64) -> (f64, i64) {
    let target = (px * 20.0).max(1e-9);
    let e = 10f64.powf(target.log10().floor());
    let m = target / e;
    let (mant, major) = if m <= 1.0 + 1e-9 {
        (1.0, 10)
    } else if m <= 2.0 {
        (2.0, 5)
    } else if m <= 5.0 {
        (5.0, 10)
    } else {
        (10.0, 10)
    };
    (mant * e, major)
}

/// Grid snapping on/off (remembered across runs).
pub fn grid_snap_on(ctx: &egui::Context) -> bool {
    ctx.data_mut(|d| *d.get_persisted_mut_or(egui::Id::new("sketch_grid_snap"), true))
}

pub fn set_grid_snap(ctx: &egui::Context, on: bool) {
    ctx.data_mut(|d| d.insert_persisted(egui::Id::new("sketch_grid_snap"), on));
}

struct Inferred {
    p: DVec2,
    guides: Vec<(DVec2, DVec2)>,
    /// horizontal (true) / vertical (false) relative to `anchor`
    hv: Option<bool>,
    on_grid: bool,
}

/// Inference for a free cursor position: horizontal/vertical from `anchor`
/// (the previous click), alignment with other points (dashed guides), then the
/// grid for whatever coordinate is still free.
fn infer(p: DVec2, px: f64, anchor: Option<DVec2>, cands: &[DVec2], grid: Option<f64>) -> Inferred {
    let tol = px * 7.0;
    let mut q = p;
    let (mut fx, mut fy) = (false, false);
    let mut hv = None;
    let mut guides = vec![];
    if let Some(a) = anchor {
        let d = p - a;
        if d.length() > px * 4.0 {
            if d.y.abs() < tol {
                q.y = a.y;
                fy = true;
                hv = Some(true);
            } else if d.x.abs() < tol {
                q.x = a.x;
                fx = true;
                hv = Some(false);
            }
        }
    }
    // closest alignment per axis (ties: the nearer point)
    let pick = |coord: fn(DVec2) -> f64| -> Option<DVec2> {
        cands
            .iter()
            .filter(|c| (coord(**c) - coord(p)).abs() < tol && (**c - p).length() > px * 3.0)
            .min_by(|a, b| {
                let da = (coord(**a) - coord(p)).abs() + (**a - p).length() * 1e-3;
                let db = (coord(**b) - coord(p)).abs() + (**b - p).length() * 1e-3;
                da.total_cmp(&db)
            })
            .copied()
    };
    let gx = if fx { None } else { pick(|v| v.x) };
    let gy = if fy { None } else { pick(|v| v.y) };
    if let Some(c) = gx {
        q.x = c.x;
        fx = true;
    }
    if let Some(c) = gy {
        q.y = c.y;
        fy = true;
    }
    let mut on_grid = false;
    if let Some(g) = grid.filter(|g| *g > 0.0) {
        if !fx {
            q.x = (q.x / g).round() * g;
        }
        if !fy {
            q.y = (q.y / g).round() * g;
        }
        on_grid = !fx || !fy;
    }
    for c in [gx, gy].into_iter().flatten() {
        guides.push((c, q));
    }
    if let (Some(a), Some(_)) = (anchor, hv) {
        guides.push((a, q));
    }
    Inferred { p: q, guides, hv, on_grid }
}

/// Points worth aligning to: sketch points, the origin, model points near the cursor.
fn align_candidates(sk: &Sketch, refs: &[DVec2], chain: &[(DVec2, Option<PId>)], p: DVec2, px: f64, exclude: &[PId]) -> Vec<DVec2> {
    let mut v: Vec<DVec2> = sk.points.iter().filter(|q| !exclude.contains(&q.id)).map(|q| q.pos()).collect();
    v.push(DVec2::ZERO);
    v.extend(chain.iter().map(|c| c.0));
    v.extend(refs.iter().filter(|r| (**r - p).length() < px * 250.0).copied());
    v
}

fn dim_anchor(sk: &Sketch, kind: &DimKind) -> (DVec2, DVec2) {
    // (anchor point, default offset direction)
    match *kind {
        DimKind::Distance(a, b) | DimKind::HDistance(a, b) | DimKind::VDistance(a, b) => {
            let (pa, pb) = (sk.pos(a), sk.pos(b));
            ((pa + pb) * 0.5, (pb - pa).perp().normalize_or(DVec2::Y))
        }
        DimKind::Length(l) => {
            let (pa, pb) = sk.line_pts(l).unwrap_or_default();
            ((pa + pb) * 0.5, (pb - pa).perp().normalize_or(DVec2::Y))
        }
        DimKind::PointLine(p, l) => {
            let (pa, pb) = sk.line_pts(l).unwrap_or_default();
            let pp = sk.pos(p);
            let d = pb - pa;
            let t = (pp - pa).dot(d) / d.length_squared().max(1e-24);
            let foot = pa + d * t;
            ((pp + foot) * 0.5, d.normalize_or(DVec2::X))
        }
        DimKind::Radius(c) | DimKind::Diameter(c) => {
            let cc = sk.curve(c).and_then(|c| match c.geom {
                CurveGeom::Circle { c, .. } | CurveGeom::Arc { c, .. } => Some(c),
                _ => None,
            });
            (cc.map(|c| sk.pos(c)).unwrap_or_default(), DVec2::new(0.7, 0.7))
        }
        DimKind::Angle(l1, l2) => {
            let (a, b) = sk.line_pts(l1).unwrap_or_default();
            let (c, d) = sk.line_pts(l2).unwrap_or_default();
            let r = b - a;
            let s = d - c;
            let den = r.perp_dot(s);
            let x = if den.abs() > 1e-12 { a + r * ((c - a).perp_dot(s) / den) } else { (a + c) * 0.5 };
            (x, (r.normalize_or_zero() + s.normalize_or_zero()).normalize_or(DVec2::Y))
        }
    }
}

/// Default label offset: outside the geometry, away from the sketch centre.
fn default_dim_offset(sk: &Sketch, kind: &DimKind, px: f64) -> DVec2 {
    let (anchor, dir) = dim_anchor(sk, kind);
    match *kind {
        DimKind::Radius(c) | DimKind::Diameter(c) => {
            let r = sk.radius_of(c).unwrap_or(0.0);
            dir.normalize_or(DVec2::X) * (r + px * 22.0)
        }
        DimKind::Angle(..) => dir * px * 40.0,
        _ => {
            let center = sk.bounds().map(|(a, b)| (a + b) * 0.5).unwrap_or(anchor);
            let d = if dir.dot(anchor - center) < 0.0 { -dir } else { dir };
            d * px * 26.0
        }
    }
}

// ------------------------------------------------------------------ input

/// Model points projected into the sketch plane (cached per session).
fn model_refs(app: &mut App, fid: FeatureId, frame: &Frame) -> Vec<DVec2> {
    if let Some(r) = session(app).and_then(|s| s.refs.clone()) {
        return r;
    }
    let st = app.pick_state().clone();
    let mut out: Vec<DVec2> = Vec::new();
    for b in st.bodies.values() {
        if !app.body_visible(&st, b.id) || b.is_mesh() {
            continue;
        }
        let edges = b.shape.edges();
        if edges.len() > 20000 {
            continue;
        }
        for e in edges {
            let Ok(i) = e.edge_info() else { continue };
            match i.kind {
                lusion_kernel::CurveKind::Line => {
                    out.push(frame.to2(i.start));
                    out.push(frame.to2(i.end));
                    out.push(frame.to2(i.mid));
                }
                lusion_kernel::CurveKind::Circle => {
                    out.push(frame.to2(i.center));
                    if !i.closed {
                        out.push(frame.to2(i.start));
                        out.push(frame.to2(i.end));
                    }
                }
                lusion_kernel::CurveKind::Other => {
                    out.push(frame.to2(i.start));
                    out.push(frame.to2(i.end));
                }
                _ => {}
            }
        }
    }
    out.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    out.dedup_by(|a, b| (*a - *b).length() < 1e-7);
    let _ = fid;
    if let Some(s) = session(app) {
        s.refs = Some(out.clone());
    }
    out
}

pub fn input(app: &mut App, ui: &egui::Ui, resp: &egui::Response, view: &View, ray: Option<Ray>) {
    let Some(fid) = session(app).map(|s| s.fid) else { return };
    let Some(idx) = app.doc.feature_index(fid) else {
        app.mode = Mode::Model;
        return;
    };
    // keep the timeline rolled to the sketch (e.g. after undo)
    if app.doc.marker != idx + 1 {
        let m = app.doc.marker;
        if let Some(s) = session(app) {
            s.prev_marker = m.max(idx + 1);
        }
        app.doc.marker = idx + 1;
        app.model.update(&app.doc);
    }
    let Some(frame) = frame_of(app, fid) else {
        return;
    };
    if session(app).map(|s| s.look_pending).unwrap_or(false) {
        session(app).unwrap().look_pending = false;
        look_at(app, fid);
    }
    keyboard(app, ui);
    if session(app).is_none() {
        // the sketch was finished from the keyboard
        return;
    }

    // cursor on the sketch plane
    let (p2, px) = match ray {
        Some(r) => match r.plane(frame.origin.as_vec3(), frame.n.as_vec3()) {
            Some(t) if t > 0.0 => {
                let px = if app.cam.cam.ortho {
                    app.cam.cam.px_size(view.size.y) as f64
                } else {
                    (2.0 * t * (app.cam.cam.fov_y * 0.5).tan() / view.size.y.max(1.0)) as f64
                };
                (Some(frame.to2(r.at(t).as_dvec3())), px)
            }
            _ => (None, 0.1),
        },
        None => (None, 0.1),
    };
    // scale at the view centre: the grid must not change with the cursor
    let grid_px = {
        let r = app.cam.ray(view.size * 0.5, view.size);
        match r.plane(frame.origin.as_vec3(), frame.n.as_vec3()) {
            Some(t) if t > 0.0 && !app.cam.cam.ortho => (2.0 * t * (app.cam.cam.fov_y * 0.5).tan() / view.size.y.max(1.0)) as f64,
            _ if app.cam.cam.ortho => app.cam.cam.px_size(view.size.y) as f64,
            _ => px,
        }
    };
    let free = ui.input(|i| i.modifiers.ctrl);
    let grid = (grid_snap_on(ui.ctx()) && !free).then(|| grid_step(grid_px).0);
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let refs = model_refs(app, fid, &frame);
    let s = session(app).unwrap();
    s.px = px;
    s.grid_px = grid_px;
    s.grid_on = grid_snap_on(ui.ctx());
    let tool = s.tool;
    let navigating = resp.dragged_by(PointerButton::Middle) || resp.dragged_by(PointerButton::Secondary);

    // hover / snapping
    let (cursor, snap) = match p2 {
        Some(p) if !navigating => {
            let exclude: Vec<PId> = s.drag.as_ref().map(|d| d.points.iter().map(|x| x.0).collect()).unwrap_or_default();
            if matches!(tool, SkTool::Select | SkTool::Dimension | SkTool::Trim | SkTool::Fillet | SkTool::Offset | SkTool::Project | SkTool::Con(_)) || free {
                (p, None)
            } else {
                let (q, sn) = compute_snap(&sk, p, px, &exclude);
                if sn.is_none() || matches!(sn, Some(Snap::OnCurve(_))) {
                    // model geometry (vertices, edge midpoints, circle centres)
                    let best = refs.iter().map(|r| (*r, (*r - p).length())).filter(|(_, d)| *d < px * 9.0).min_by(|a, b| a.1.total_cmp(&b.1));
                    match best {
                        Some((r, _)) => (r, Some(Snap::Model(r))),
                        None => (q, sn),
                    }
                } else {
                    (q, sn)
                }
            }
        }
        _ => (p2.unwrap_or_default(), None),
    };
    // inference: H/V from the previous click, alignment guides, grid (Ctrl = none)
    let mut cursor = cursor;
    s.hv_hint = None;
    s.guides.clear();
    s.on_grid = false;
    let draws = matches!(tool, SkTool::Line | SkTool::Rect | SkTool::CenterRect | SkTool::Circle | SkTool::Arc | SkTool::Spline | SkTool::Point | SkTool::Text);
    if snap.is_none() && draws && !free && p2.is_some() && !navigating {
        if tool == SkTool::Circle && s.chain.len() == 1 {
            // radius in grid steps
            let c = s.chain[0].0;
            let d = cursor - c;
            if let (Some(g), true) = (grid, d.length() > 1e-12) {
                let r = ((d.length() / g).round() * g).max(g);
                cursor = c + d.normalize() * r;
                s.on_grid = true;
            }
        } else {
            let anchor = match tool {
                SkTool::Line | SkTool::Spline | SkTool::Arc => s.chain.last().map(|c| c.0),
                _ => None,
            };
            let cands = align_candidates(&sk, &refs, &s.chain, cursor, px, &[]);
            let inf = infer(cursor, px, anchor, &cands, grid);
            cursor = inf.p;
            // only lines get an H/V constraint; elsewhere it is just alignment
            s.hv_hint = if tool == SkTool::Line { inf.hv } else { None };
            s.guides = inf.guides;
            s.on_grid = inf.on_grid;
        }
    }
    s.cursor = p2.map(|_| cursor);
    s.snap = snap;
    let mut hover = None;
    if let (Some(p), false) = (p2, navigating) {
        if matches!(tool, SkTool::Select | SkTool::Dimension | SkTool::Con(_) | SkTool::Trim | SkTool::Fillet | SkTool::Offset) {
            hover = dim_label_hit(app, fid, view).map(SkEnt::Con);
            if hover.is_none() && tool != SkTool::Trim {
                hover = nearest_point(&sk, p, px * 8.0, &[]).map(SkEnt::Point);
            }
            if hover.is_none() {
                hover = nearest_curve(&sk, p, px * 6.0).map(SkEnt::Curve);
            }
            if hover.is_none() && tool != SkTool::Trim && tool != SkTool::Fillet {
                hover = text_hit(&sk, p, &app.model.params).map(SkEnt::Text);
            }
        }
    }
    session(app).unwrap().hover = hover;
    if tool == SkTool::Project {
        let st = app.pick_state().clone();
        let filter = crate::app::Filter { faces: true, edges: true, ..Default::default() };
        let h = match (ray, navigating) {
            (Some(r), false) => crate::viewport::pick(app, &st, &r, view, filter).map(|h| h.pick),
            _ => None,
        };
        session(app).unwrap().proj_hover = h;
    }

    // ---- drags (select tool)
    if tool == SkTool::Select {
        if resp.drag_started_by(PointerButton::Primary) {
            // pick what was under the pointer when the button went down
            let press = ui.input(|i| i.pointer.press_origin()).map(|o| Vec2::new(o.x - view.rect.min.x, o.y - view.rect.min.y));
            let press2 = press.and_then(|m| {
                let r = app.cam.ray(m, view.size);
                r.plane(frame.origin.as_vec3(), frame.n.as_vec3()).filter(|t| *t > 0.0).map(|t| frame.to2(r.at(t).as_dvec3()))
            });
            let hover_at_press = press2.and_then(|pp| {
                let lbl = press.and_then(|m| dim_label_hit(app, fid, &View { mouse: Some(m), ..*view })).map(SkEnt::Con);
                lbl.or_else(|| nearest_point(&sk, pp, px * 8.0, &[]).map(SkEnt::Point))
                    .or_else(|| nearest_curve(&sk, pp, px * 6.0).map(SkEnt::Curve))
                    .or_else(|| text_hit(&sk, pp, &app.model.params).map(SkEnt::Text))
            });
            let hover = hover_at_press.or(hover);
            let p2 = press2.or(p2);
            if let (Some(h), Some(p)) = (hover, p2) {
                let before = app.doc.clone();
                let drag = match h {
                    SkEnt::Point(pid) => {
                        if sk.point(pid).map(|q| q.fixed).unwrap_or(false) {
                            None
                        } else {
                            Some(Drag { points: vec![(pid, sk.pos(pid))], radius: None, label: None, start: p, before, moved: false })
                        }
                    }
                    SkEnt::Curve(cid) => {
                        let c = sk.curve(cid).unwrap();
                        match c.geom {
                            CurveGeom::Circle { .. } => Some(Drag { points: vec![], radius: Some(cid), label: None, start: p, before, moved: false }),
                            _ => {
                                let pts: Vec<(PId, DVec2)> = c.points().iter().filter(|q| !sk.point(**q).map(|x| x.fixed).unwrap_or(false)).map(|q| (*q, sk.pos(*q))).collect();
                                Some(Drag { points: pts, radius: None, label: None, start: p, before, moved: false })
                            }
                        }
                    }
                    SkEnt::Text(tid) => sk.texts.iter().find(|t| t.id == tid).and_then(|t| {
                        let fixed = sk.point(t.anchor).map(|q| q.fixed).unwrap_or(true);
                        (!fixed).then(|| Drag { points: vec![(t.anchor, sk.pos(t.anchor))], radius: None, label: None, start: p, before, moved: false })
                    }),
                    SkEnt::Con(kid) => sk.constraints.iter().find(|k| k.id == kid).and_then(|k| match &k.c {
                        Constraint::Dimension { offset, .. } => Some(Drag { points: vec![], radius: None, label: Some((kid, *offset)), start: p, before, moved: false }),
                        _ => None,
                    }),
                };
                session(app).unwrap().drag = drag;
            }
        }
        if resp.dragged_by(PointerButton::Primary) {
            if let (Some(p), Some(drag)) = (p2, session(app).and_then(|s| s.drag.take())) {
                let mut drag = drag;
                drag.moved = true;
                let mut delta = p - drag.start;
                if let Some(s) = session(app) {
                    s.guides.clear();
                    s.on_grid = false;
                }
                if !free && drag.label.is_none() && drag.radius.is_none() {
                    // snap the grabbed point (the rest moves along)
                    if let Some(&(pid0, orig0)) = drag.points.first() {
                        let ex: Vec<PId> = drag.points.iter().map(|x| x.0).collect();
                        let target = orig0 + delta;
                        let snapped = nearest_point(&sk, target, px * 10.0, &ex).map(|q| sk.pos(q));
                        let _ = pid0;
                        let q = match snapped {
                            Some(q) => q,
                            None => {
                                let cands = align_candidates(&sk, &refs, &[], target, px, &ex);
                                let inf = infer(target, px, None, &cands, grid);
                                if let Some(s) = session(app) {
                                    s.guides = inf.guides;
                                    s.on_grid = inf.on_grid;
                                }
                                inf.p
                            }
                        };
                        delta = q - orig0;
                    }
                }
                if let Some(sk) = sketch_mut(app, fid) {
                    if let Some((kid, off)) = drag.label {
                        if let Some(k) = sk.constraints.iter_mut().find(|k| k.id == kid) {
                            if let Constraint::Dimension { offset, .. } = &mut k.c {
                                *offset = [off[0] + delta.x, off[1] + delta.y];
                            }
                        }
                    } else if let Some(cid) = drag.radius {
                        let center = sk.curve(cid).and_then(|c| match c.geom {
                            CurveGeom::Circle { c, .. } => Some(c),
                            _ => None,
                        });
                        if let Some(cc) = center {
                            let mut r = (p - sk.pos(cc)).length();
                            if let (Some(g), false) = (grid, free) {
                                r = ((r / g).round() * g).max(g);
                            }
                            if let Some(x) = sk.curves.iter_mut().find(|x| x.id == cid) {
                                if let CurveGeom::Circle { r: rr, .. } = &mut x.geom {
                                    *rr = r.max(1e-6);
                                }
                            }
                        }
                    } else {
                        for (pid, orig) in &drag.points {
                            if let Some(q) = sk.point_mut(*pid) {
                                q.x = orig.x + delta.x;
                                q.y = orig.y + delta.y;
                            }
                        }
                    }
                }
                let pinned: Vec<PId> = drag.points.iter().map(|x| x.0).collect();
                if drag.label.is_none() && !solve(app, &pinned, false) {
                    // can't follow the mouse: solve without pinning
                    solve(app, &[], false);
                }
                session(app).unwrap().drag = Some(drag);
            }
        }
        if resp.drag_stopped_by(PointerButton::Primary) {
            if let Some(drag) = session(app).and_then(|s| s.drag.take()) {
                solve(app, &[], true);
                if drag.moved {
                    commit(app, drag.before);
                }
            }
        }
    }

    // ---- clicks
    if resp.secondary_clicked() && finish_spline(app) {
        return;
    }
    if resp.secondary_clicked() {
        let s = session(app).unwrap();
        if !s.chain.is_empty() || !s.picks.is_empty() {
            s.chain.clear();
            s.picks.clear();
        } else if s.tool != SkTool::Select {
            s.tool = SkTool::Select;
        }
    }
    if resp.double_clicked_by(PointerButton::Primary) {
        if let Some(SkEnt::Con(kid)) = hover {
            start_dim_edit(app, kid);
            return;
        }
        if let Some(SkEnt::Text(tid)) = hover {
            if let Some(t) = sk.texts.iter().find(|t| t.id == tid) {
                session(app).unwrap().text_edit = Some((tid, t.text.clone(), t.height.clone(), false, t.text.clone(), t.height.clone()));
            }
            return;
        }
        if tool == SkTool::Line {
            session(app).unwrap().chain.clear();
            return;
        }
        if tool == SkTool::Spline {
            finish_spline(app);
            return;
        }
    }
    if resp.clicked_by(PointerButton::Primary) {
        let Some(p) = p2 else { return };
        let shift = ui.input(|i| i.modifiers.shift || i.modifiers.command);
        match tool {
            SkTool::Select => {
                let s = session(app).unwrap();
                match hover {
                    Some(h) => {
                        if shift {
                            if let Some(i) = s.selection.iter().position(|x| *x == h) {
                                s.selection.remove(i);
                            } else {
                                s.selection.push(h);
                            }
                        } else {
                            s.selection = vec![h];
                        }
                    }
                    None => {
                        if !shift {
                            s.selection.clear();
                        }
                    }
                }
            }
            SkTool::Line | SkTool::Rect | SkTool::CenterRect | SkTool::Circle | SkTool::Arc | SkTool::Spline | SkTool::Point => {
                draw_click(app, fid, cursor, snap);
            }
            SkTool::Dimension => dim_click(app, fid, hover, p),
            SkTool::Offset => {
                let src = session(app).and_then(|s| s.offset_src);
                match (src, hover) {
                    (Some(src), _) => create_offset(app, src, p, None),
                    (None, Some(SkEnt::Curve(c))) => session(app).unwrap().offset_src = Some(c),
                    _ => {}
                }
            }
            SkTool::Project => {
                let hit = session(app).and_then(|s| s.proj_hover);
                if let Some(h) = hit {
                    project_pick(app, h);
                }
            }
            SkTool::Text => {
                let before = app.doc.clone();
                let tid = sketch_mut(app, fid).map(|skm| {
                    let mut cons = vec![];
                    let a = point_at(skm, cursor, snap, &mut cons);
                    for c in cons {
                        skm.add_constraint(c);
                    }
                    skm.add_text(a, "Text", "5")
                });
                solve(app, &[], true);
                commit(app, before);
                if let Some(tid) = tid {
                    let s = session(app).unwrap();
                    s.text_edit = Some((tid, "Text".into(), "5".into(), true, "Text".into(), "5".into()));
                    s.tool = SkTool::Select;
                }
            }
            SkTool::Fillet => {
                if let Some(SkEnt::Point(pid)) = hover {
                    let before = app.doc.clone();
                    let kid = sketch_mut(app, fid).and_then(|sk| sketch_fillet(sk, pid));
                    match kid {
                        Some(kid) => {
                            if !solve(app, &[], true) {
                                app.doc = before;
                                solve(app, &[], true);
                                app.error("Could not fillet that corner (conflicting constraints?)");
                            } else {
                                commit(app, before);
                                start_dim_edit(app, kid);
                            }
                        }
                        None => app.error("Pick a corner shared by exactly two lines"),
                    }
                }
            }
            SkTool::Trim => {
                if let Some(SkEnt::Curve(cid)) = hover {
                    let before = app.doc.clone();
                    if let Some(sk) = sketch_mut(app, fid) {
                        trim(sk, cid, p);
                    }
                    solve(app, &[], true);
                    commit(app, before);
                }
            }
            SkTool::Con(k) => {
                if let Some(h) = hover {
                    let s = session(app).unwrap();
                    if !s.picks.contains(&h) {
                        s.picks.push(h);
                    }
                    let picks = s.picks.clone();
                    match try_constraint(app, fid, k, &picks) {
                        Ok(true) => session(app).unwrap().picks.clear(),
                        Ok(false) => {}
                        Err(e) => {
                            session(app).unwrap().picks.clear();
                            app.error(e);
                        }
                    }
                }
            }
        }
    }
}

fn keyboard(app: &mut App, ui: &egui::Ui) {
    if ui.ctx().egui_wants_keyboard_input() {
        return;
    }
    // typing a number while drawing opens the value box
    if let Some(s) = session(app) {
        let drawing = (!s.chain.is_empty() && matches!(s.tool, SkTool::Line | SkTool::Rect | SkTool::CenterRect | SkTool::Circle))
            || (s.tool == SkTool::Offset && s.offset_src.is_some());
        if drawing && s.typed.is_none() {
            let started: Option<String> = ui.input(|i| {
                i.events.iter().find_map(|e| match e {
                    egui::Event::Text(t) if t.chars().all(|c| c.is_ascii_digit() || c == '.' || c == '-') => Some(t.clone()),
                    _ => None,
                })
            });
            if let Some(t) = started {
                s.typed = Some(t);
                return;
            }
        }
    }
    use egui::Key;
    let pressed = |k: Key| ui.input(|i| i.key_pressed(k) && i.modifiers.is_none());
    let tool_keys = [
        (Key::L, SkTool::Line),
        (Key::C, SkTool::Circle),
        (Key::A, SkTool::Arc),
        (Key::B, SkTool::Spline),
        (Key::P, SkTool::Project),
        (Key::O, SkTool::Offset),
        (Key::T, SkTool::Trim),
        (Key::G, SkTool::Fillet),
    ];
    for (k, t) in tool_keys {
        if pressed(k) {
            set_tool(app, t);
        }
    }
    if pressed(Key::R) {
        // R again flips between corner and centre rectangles (also mid-draw)
        let s = session(app).unwrap();
        match s.tool {
            SkTool::Rect => s.tool = SkTool::CenterRect,
            SkTool::CenterRect => s.tool = SkTool::Rect,
            _ => set_tool(app, SkTool::Rect),
        }
    }
    if pressed(Key::D) {
        quick_dim(app);
        return;
    }
    if pressed(Key::Escape) && finish_spline(app) {
        return;
    }
    if pressed(Key::Enter) && finish_spline(app) {
        return;
    }
    if pressed(Key::Escape) {
        let s = session(app).unwrap();
        if s.edit_dim.is_some() {
            s.edit_dim = None;
        } else if !s.chain.is_empty() || !s.picks.is_empty() || s.offset_src.is_some() {
            s.chain.clear();
            s.picks.clear();
            s.offset_src = None;
        } else if s.tool != SkTool::Select {
            s.tool = SkTool::Select;
        } else if !s.selection.is_empty() {
            s.selection.clear();
        } else {
            finish(app);
            return;
        }
    }
    if pressed(Key::Enter) {
        let s = session(app).unwrap();
        s.chain.clear();
    }
    if pressed(Key::X) {
        toggle_construction(app);
    }
    if pressed(Key::Delete) || pressed(Key::Backspace) {
        delete_selection(app);
    }
}

pub fn set_tool(app: &mut App, t: SkTool) {
    finish_spline(app);
    if let SkTool::Con(k) = t {
        // apply directly to the current selection when possible
        let Some(s) = session(app) else { return };
        let fid = s.fid;
        let sel = s.selection.clone();
        if !sel.is_empty() {
            match try_constraint(app, fid, k, &sel) {
                Ok(true) => {
                    session(app).unwrap().selection.clear();
                    return;
                }
                Ok(false) => {}
                Err(e) => {
                    app.error(e);
                    return;
                }
            }
        }
    }
    if let Some(s) = session(app) {
        s.tool = t;
        s.chain.clear();
        s.picks.clear();
        s.offset_src = None;
        s.proj_hover = None;
    }
}

pub fn toggle_construction(app: &mut App) {
    let Some(s) = session(app) else { return };
    let fid = s.fid;
    let curves: Vec<CId> = s.selection.iter().filter_map(|e| if let SkEnt::Curve(c) = e { Some(*c) } else { None }).collect();
    if curves.is_empty() {
        return;
    }
    let before = app.doc.clone();
    if let Some(sk) = sketch_mut(app, fid) {
        for c in sk.curves.iter_mut().filter(|c| curves.contains(&c.id)) {
            c.construction = !c.construction;
        }
        for sp in sk.splines.iter_mut().filter(|sp| curves.contains(&sp.id)) {
            sp.construction = !sp.construction;
        }
    }
    commit(app, before);
}

pub fn delete_selection(app: &mut App) {
    let Some(s) = session(app) else { return };
    let fid = s.fid;
    let sel = std::mem::take(&mut s.selection);
    let (mut pts, mut cs, mut ks) = (vec![], vec![], vec![]);
    for e in sel {
        match e {
            SkEnt::Point(p) => pts.push(p),
            SkEnt::Curve(c) | SkEnt::Text(c) => cs.push(c),
            SkEnt::Con(k) => ks.push(k),
        }
    }
    let before = app.doc.clone();
    if let Some(sk) = sketch_mut(app, fid) {
        // never delete the fixed origin point
        pts.retain(|p| !(sk.point(*p).map(|q| q.fixed).unwrap_or(false) && sk.points.first().map(|f| f.id) == Some(*p)));
        sk.delete(&pts, &cs, &ks);
    }
    solve(app, &[], true);
    commit(app, before);
}

/// Get or create a point at `p` (merging with a snapped point).
fn point_at(sk: &mut Sketch, p: DVec2, snap: Option<Snap>, cons: &mut Vec<Constraint>) -> PId {
    match snap {
        Some(Snap::Point(id)) => id,
        Some(Snap::Mid(cid)) => {
            let id = sk.add_point(p);
            cons.push(Constraint::Midpoint(id, cid));
            id
        }
        Some(Snap::OnCurve(cid)) => {
            let id = sk.add_point(p);
            cons.push(Constraint::PointOnCurve(id, cid));
            id
        }
        Some(Snap::Model(q)) => {
            // reuse an existing point at that spot, else a fixed reference point
            if let Some(id) = sk.points.iter().find(|x| (x.pos() - q).length() < 1e-9).map(|x| x.id) {
                return id;
            }
            let id = sk.add_point(q);
            if let Some(pt) = sk.point_mut(id) {
                pt.fixed = true;
            }
            id
        }
        None => sk.add_point(p),
    }
}

fn draw_click(app: &mut App, fid: FeatureId, p: DVec2, snap: Option<Snap>) {
    let s = session(app).unwrap();
    let tool = s.tool;
    let hv = s.hv_hint;
    s.chain.push((p, match snap {
        Some(Snap::Point(id)) => Some(id),
        _ => None,
    }));
    let chain = s.chain.clone();
    let px = s.px;
    let before = app.doc.clone();
    let last_before = sketch(app, fid).and_then(|k| k.curves.last().map(|c| c.id));
    let mut done = false;
    let mut cons: Vec<Constraint> = Vec::new();
    let Some(sk) = sketch_mut(app, fid) else { return };
    match tool {
        SkTool::Point => {
            point_at(sk, p, snap, &mut cons);
            done = true;
        }
        SkTool::Line => {
            if chain.len() >= 2 {
                let (a, b) = (chain[chain.len() - 2], chain[chain.len() - 1]);
                if (a.0 - b.0).length() > px * 2.0 {
                    // start point: the end of the previous line, or a snapped point
                    let pa = match a.1 {
                        Some(id) => id,
                        None => {
                            let last_line_end = sk.curves.last().and_then(|c| match c.geom {
                                CurveGeom::Line { b, .. } if (sk.pos(b) - a.0).length() < 1e-9 => Some(b),
                                _ => None,
                            });
                            last_line_end.unwrap_or_else(|| point_at(sk, a.0, None, &mut cons))
                        }
                    };
                    let pb = point_at(sk, b.0, snap, &mut cons);
                    let l = sk.add_line(pa, pb);
                    match hv {
                        Some(true) => cons.push(Constraint::Horizontal(l)),
                        Some(false) => cons.push(Constraint::Vertical(l)),
                        None => {}
                    }
                    // closing the loop ends the chain
                    if chain.len() > 2 && matches!(snap, Some(Snap::Point(id)) if Some(id) == chain[0].1 || sk.pos(id) == chain[0].0) {
                        done = true;
                    } else if let Some(Snap::Point(_)) = snap {
                        // continue from the snapped point
                    }
                    let s_end = (b.0, Some(pb));
                    let s = match &mut app.mode {
                        Mode::Sketch(s) => s,
                        _ => return,
                    };
                    let n = s.chain.len();
                    s.chain[n - 1] = s_end;
                } else {
                    // double click on the same spot
                    let s = match &mut app.mode {
                        Mode::Sketch(s) => s,
                        _ => return,
                    };
                    s.chain.pop();
                    return;
                }
            }
        }
        SkTool::Spline => {
            // clicking the first point again closes the curve
            let n = chain.len();
            let first = chain[0];
            let closes = n >= 4 && (matches!(snap, Some(Snap::Point(id)) if Some(id) == first.1) || (chain[n - 1].0 - first.0).length() < px * 6.0);
            if closes {
                let mut ids = Vec::new();
                for (q, sid) in &chain[..n - 1] {
                    ids.push(point_at(sk, *q, snap_of(*sid), &mut cons));
                }
                ids.dedup();
                if ids.len() >= 3 {
                    sk.add_spline(ids, true);
                }
                done = true;
            } else {
                // nothing to create until the curve is finished
                let s = match &mut app.mode {
                    Mode::Sketch(s) => s,
                    _ => return,
                };
                // a double click lands twice on the same spot
                let n = s.chain.len();
                if n >= 2 && (s.chain[n - 1].0 - s.chain[n - 2].0).length() < px * 2.0 {
                    s.chain.pop();
                }
                return;
            }
        }
        SkTool::Rect | SkTool::CenterRect => {
            if chain.len() == 2 {
                let (c0, c1) = if tool == SkTool::Rect {
                    (chain[0].0, chain[1].0)
                } else {
                    let d = chain[1].0 - chain[0].0;
                    (chain[0].0 - d, chain[1].0)
                };
                if (c1.x - c0.x).abs() > 1e-9 && (c1.y - c0.y).abs() > 1e-9 {
                    let s0 = if tool == SkTool::Rect { snap_of(chain[0].1) } else { None };
                    let p0 = point_at(sk, c0, s0, &mut cons);
                    let p1 = sk.add_point(DVec2::new(c1.x, c0.y));
                    let p2 = point_at(sk, c1, snap, &mut cons);
                    let p3 = sk.add_point(DVec2::new(c0.x, c1.y));
                    let l0 = sk.add_line(p0, p1);
                    let l1 = sk.add_line(p1, p2);
                    let l2 = sk.add_line(p2, p3);
                    let l3 = sk.add_line(p3, p0);
                    cons.push(Constraint::Horizontal(l0));
                    cons.push(Constraint::Horizontal(l2));
                    cons.push(Constraint::Vertical(l1));
                    cons.push(Constraint::Vertical(l3));
                    if tool == SkTool::CenterRect {
                        let pc = point_at(sk, chain[0].0, snap_of(chain[0].1), &mut cons);
                        let diag = sk.add_curve(CurveGeom::Line { a: p0, b: p2 }, true);
                        cons.push(Constraint::Midpoint(pc, diag));
                    }
                }
                done = true;
            }
        }
        SkTool::Circle => {
            if chain.len() == 2 {
                let r = (chain[1].0 - chain[0].0).length();
                if r > 1e-9 {
                    let c = point_at(sk, chain[0].0, snap_of(chain[0].1), &mut cons);
                    sk.add_curve(CurveGeom::Circle { c, r }, false);
                }
                done = true;
            }
        }
        SkTool::Arc => {
            if chain.len() == 3 {
                if let Some((c, ccw)) = arc_from_3(chain[0].0, chain[1].0, chain[2].0) {
                    let pa = point_at(sk, chain[0].0, snap_of(chain[0].1), &mut cons);
                    let pb = point_at(sk, chain[1].0, snap_of(chain[1].1), &mut cons);
                    let pc = sk.add_point(c);
                    let (a, b) = if ccw { (pa, pb) } else { (pb, pa) };
                    sk.add_curve(CurveGeom::Arc { c: pc, a, b }, false);
                }
                done = true;
            }
        }
        _ => {}
    }
    if let Some(sk) = sketch_mut(app, fid) {
        for c in cons {
            sk.add_constraint(c);
        }
    }
    solve(app, &[], true);
    commit(app, before);
    let last_after = sketch(app, fid).and_then(|k| k.curves.last().map(|c| c.id));
    if let Some(s) = session(app) {
        if last_after != last_before {
            s.last_curve = last_after;
        }
        if done {
            s.chain.clear();
        }
    }
}

/// Turn the clicked spline points into an open spline (Enter / right-click / double-click).
fn finish_spline(app: &mut App) -> bool {
    let Some(s) = session(app) else { return false };
    if s.tool != SkTool::Spline || s.chain.len() < 2 {
        return false;
    }
    let fid = s.fid;
    let chain = std::mem::take(&mut s.chain);
    let before = app.doc.clone();
    let mut cons = Vec::new();
    let Some(sk) = sketch_mut(app, fid) else { return false };
    let mut ids = Vec::new();
    for (q, sid) in &chain {
        ids.push(point_at(sk, *q, snap_of(*sid), &mut cons));
    }
    ids.dedup();
    if ids.len() < 2 {
        return false;
    }
    sk.add_spline(ids, false);
    for c in cons {
        sk.add_constraint(c);
    }
    solve(app, &[], true);
    commit(app, before);
    true
}

/// Create geometry from a typed value while drawing.
fn apply_typed(app: &mut App, text: &str) {
    if let Some((src, cursor)) = session(app).filter(|s| s.tool == SkTool::Offset).and_then(|s| s.offset_src.zip(s.cursor)) {
        match app.model.params.eval(text) {
            Ok(d) if d.abs() > 1e-9 => create_offset(app, src, cursor, Some(d.abs())),
            Ok(_) => {}
            Err(e) => app.error(format!("Invalid value: {e}")),
        }
        return;
    }
    let Some(s) = session(app) else { return };
    let fid = s.fid;
    let tool = s.tool;
    let Some(&(start, start_id)) = s.chain.last() else { return };
    let cursor = s.cursor.unwrap_or(start + DVec2::X);
    let params = app.model.params.clone();
    let nums: Vec<String> = text.split(|c: char| c == 'x' || c == 'X' || c == ',' || c == '*').map(|t| t.trim().to_string()).filter(|t| !t.is_empty()).collect();
    let vals: Vec<f64> = match nums.iter().map(|t| params.eval(t)).collect::<Result<Vec<f64>, String>>() {
        Ok(v) if !v.is_empty() => v,
        Ok(_) => return,
        Err(e) => {
            app.error(format!("Invalid value: {e}"));
            return;
        }
    };
    let before = app.doc.clone();
    let Some(sk) = sketch_mut(app, fid) else { return };
    let mut cons = vec![];
    let dim = |kind: DimKind, expr: &str, v: f64| Constraint::Dimension { kind, expr: expr.to_string(), value: v, offset: [0.0, 0.0] };
    let mut finish_chain = false;
    let mut new_chain_end: Option<(DVec2, Option<PId>)> = None;
    match tool {
        SkTool::Line => {
            let len = vals[0].abs();
            let dir = (cursor - start).normalize_or(DVec2::X);
            let end = start + dir * len;
            let pa = match start_id {
                Some(id) => id,
                None => {
                    let last_end = sk.curves.last().and_then(|c| match c.geom {
                        CurveGeom::Line { b, .. } if (sk.pos(b) - start).length() < 1e-9 => Some(b),
                        _ => None,
                    });
                    last_end.unwrap_or_else(|| sk.add_point(start))
                }
            };
            let pb = sk.add_point(end);
            let l = sk.add_line(pa, pb);
            if dir.y.abs() < 1e-9 {
                cons.push(Constraint::Horizontal(l));
            } else if dir.x.abs() < 1e-9 {
                cons.push(Constraint::Vertical(l));
            }
            cons.push(dim(DimKind::Length(l), &nums[0], len));
            new_chain_end = Some((end, Some(pb)));
        }
        SkTool::Circle => {
            let d = vals[0].abs();
            let c = match start_id {
                Some(id) => id,
                None => sk.add_point(start),
            };
            let ci = sk.add_curve(CurveGeom::Circle { c, r: d / 2.0 }, false);
            cons.push(dim(DimKind::Diameter(ci), &nums[0], d));
            finish_chain = true;
        }
        SkTool::Rect | SkTool::CenterRect => {
            let w = vals[0].abs();
            let h = vals.get(1).copied().unwrap_or(w).abs();
            let sx = if cursor.x >= start.x { 1.0 } else { -1.0 };
            let sy = if cursor.y >= start.y { 1.0 } else { -1.0 };
            let (c0, c1) = if tool == SkTool::Rect {
                (start, start + DVec2::new(sx * w, sy * h))
            } else {
                (start - DVec2::new(w, h) * 0.5, start + DVec2::new(w, h) * 0.5)
            };
            let p0 = if tool == SkTool::Rect { start_id.unwrap_or_else(|| sk.add_point(c0)) } else { sk.add_point(c0) };
            let p1 = sk.add_point(DVec2::new(c1.x, c0.y));
            let p2 = sk.add_point(c1);
            let p3 = sk.add_point(DVec2::new(c0.x, c1.y));
            let l0 = sk.add_line(p0, p1);
            let l1 = sk.add_line(p1, p2);
            let l2 = sk.add_line(p2, p3);
            let l3 = sk.add_line(p3, p0);
            cons.push(Constraint::Horizontal(l0));
            cons.push(Constraint::Horizontal(l2));
            cons.push(Constraint::Vertical(l1));
            cons.push(Constraint::Vertical(l3));
            cons.push(dim(DimKind::Length(l0), &nums[0], w));
            cons.push(dim(DimKind::Length(l1), nums.get(1).map(|s| s.as_str()).unwrap_or(&nums[0]), h));
            if tool == SkTool::CenterRect {
                let pc = start_id.unwrap_or_else(|| sk.add_point(start));
                let diag = sk.add_curve(CurveGeom::Line { a: p0, b: p2 }, true);
                cons.push(Constraint::Midpoint(pc, diag));
            }
            finish_chain = true;
        }
        _ => return,
    }
    // place dimension labels just outside the geometry
    let px = session(app).map(|s| s.px).unwrap_or(0.1);
    if let Some(sk) = sketch_mut(app, fid) {
        for c in cons {
            let c = match c {
                Constraint::Dimension { kind, expr, value, .. } => {
                    let off = default_dim_offset(sk, &kind, px);
                    Constraint::Dimension { kind, expr, value, offset: off.to_array() }
                }
                c => c,
            };
            sk.add_constraint(c);
        }
    }
    if !solve(app, &[], true) {
        app.doc = before;
        solve(app, &[], true);
        app.error("That value conflicts with existing constraints");
        return;
    }
    commit(app, before);
    if let Some(s) = session(app) {
        if finish_chain {
            s.chain.clear();
        } else if let Some(e) = new_chain_end {
            s.chain = vec![e];
        }
    }
}

fn snap_of(id: Option<PId>) -> Option<Snap> {
    id.map(Snap::Point)
}

fn curve_kind(sk: &Sketch, c: CId) -> Option<CurveGeom> {
    sk.curve(c).map(|c| c.geom)
}

/// Try to create a constraint from the picked entities. Ok(false) = need more picks.
fn try_constraint(app: &mut App, fid: FeatureId, k: ConKind, ents: &[SkEnt]) -> Result<bool, String> {
    let Some(sk) = sketch(app, fid).cloned() else { return Ok(false) };
    let pts: Vec<PId> = ents.iter().filter_map(|e| if let SkEnt::Point(p) = e { Some(*p) } else { None }).collect();
    let cs: Vec<CId> = ents.iter().filter_map(|e| if let SkEnt::Curve(c) = e { Some(*c) } else { None }).collect();
    let is_line = |c: CId| matches!(curve_kind(&sk, c), Some(CurveGeom::Line { .. }));
    let is_round = |c: CId| matches!(curve_kind(&sk, c), Some(CurveGeom::Circle { .. } | CurveGeom::Arc { .. }));
    let mut new: Vec<Constraint> = vec![];
    match k {
        ConKind::Fix => {
            if pts.is_empty() && cs.is_empty() {
                return Ok(false);
            }
            let before = app.doc.clone();
            if let Some(skm) = sketch_mut(app, fid) {
                let mut all = pts.clone();
                for c in &cs {
                    all.extend(sk.curve(*c).map(|c| c.points()).unwrap_or_default());
                }
                let any_free = all.iter().any(|p| !skm.point(*p).map(|q| q.fixed).unwrap_or(true));
                for p in all {
                    if let Some(q) = skm.point_mut(p) {
                        q.fixed = any_free;
                    }
                }
            }
            solve(app, &[], true);
            commit(app, before);
            return Ok(true);
        }
        ConKind::HorizVert => {
            if let [c] = cs[..] {
                if pts.is_empty() && is_line(c) {
                    let (a, b) = sk.line_pts(c).unwrap();
                    let d = b - a;
                    new.push(if d.x.abs() >= d.y.abs() { Constraint::Horizontal(c) } else { Constraint::Vertical(c) });
                }
            } else if let ([a, b], true) = (&pts[..], cs.is_empty()) {
                let d = sk.pos(*b) - sk.pos(*a);
                new.push(if d.x.abs() >= d.y.abs() { Constraint::HorizontalPts(*a, *b) } else { Constraint::VerticalPts(*a, *b) });
            } else if cs.len() > 1 && cs.iter().all(|c| is_line(*c)) && pts.is_empty() {
                for c in &cs {
                    let (a, b) = sk.line_pts(*c).unwrap();
                    let d = b - a;
                    new.push(if d.x.abs() >= d.y.abs() { Constraint::Horizontal(*c) } else { Constraint::Vertical(*c) });
                }
            }
        }
        ConKind::Coincident => match (&pts[..], &cs[..]) {
            ([a, b], []) => new.push(Constraint::Coincident(*a, *b)),
            ([p], [c]) => new.push(Constraint::PointOnCurve(*p, *c)),
            _ => {}
        },
        ConKind::Parallel | ConKind::Perpendicular => {
            if let ([], [a, b]) = (&pts[..], &cs[..]) {
                if !is_line(*a) || !is_line(*b) {
                    return Err("pick two lines".into());
                }
                new.push(if k == ConKind::Parallel { Constraint::Parallel(*a, *b) } else { Constraint::Perpendicular(*a, *b) });
            }
        }
        ConKind::Equal => {
            if let ([], [a, b]) = (&pts[..], &cs[..]) {
                if is_line(*a) != is_line(*b) {
                    return Err("pick two lines or two circles/arcs".into());
                }
                new.push(Constraint::Equal(*a, *b));
            }
        }
        ConKind::Tangent => {
            if let ([], [a, b]) = (&pts[..], &cs[..]) {
                if !(is_round(*a) || is_round(*b)) {
                    return Err("tangent needs a circle or arc".into());
                }
                new.push(Constraint::Tangent(*a, *b));
            }
        }
        ConKind::Concentric => {
            if let ([], [a, b]) = (&pts[..], &cs[..]) {
                if !is_round(*a) || !is_round(*b) {
                    return Err("pick two circles/arcs".into());
                }
                new.push(Constraint::Concentric(*a, *b));
            }
        }
        ConKind::Midpoint => {
            if let ([p], [c]) = (&pts[..], &cs[..]) {
                if !is_line(*c) {
                    return Err("midpoint needs a line".into());
                }
                new.push(Constraint::Midpoint(*p, *c));
            }
        }
    }
    if new.is_empty() {
        let need = ents.len() >= 3;
        if need {
            return Err(format!("{}: unsupported selection", k.label()));
        }
        return Ok(false);
    }
    let before = app.doc.clone();
    if let Some(skm) = sketch_mut(app, fid) {
        for c in new {
            skm.add_constraint(c);
        }
    }
    if !solve(app, &[], true) {
        app.doc = before;
        solve(app, &[], true);
        return Err("That constraint conflicts with existing ones".into());
    }
    commit(app, before);
    Ok(true)
}

fn dim_click(app: &mut App, fid: FeatureId, hover: Option<SkEnt>, p: DVec2) {
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let s = session(app).unwrap();
    let mut picks = s.picks.clone();
    let place = |kind: DimKind, at: Option<DVec2>| -> (DimKind, Option<DVec2>) { (kind, at) };
    let mut result: Option<(DimKind, Option<DVec2>)> = None;
    match hover {
        Some(e @ SkEnt::Point(_)) | Some(e @ SkEnt::Curve(_)) => {
            if let SkEnt::Curve(c) = e {
                if picks.is_empty() {
                    match curve_kind(&sk, c) {
                        Some(CurveGeom::Circle { .. }) => result = Some(place(DimKind::Diameter(c), None)),
                        Some(CurveGeom::Arc { .. }) => result = Some(place(DimKind::Radius(c), None)),
                        _ => picks.push(e),
                    }
                } else {
                    picks.push(e);
                }
            } else {
                picks.push(e);
            }
        }
        Some(SkEnt::Con(kid)) => {
            start_dim_edit(app, kid);
            return;
        }
        Some(SkEnt::Text(_)) => return,
        None => {
            // click in empty space: place a pending single-line dimension
            if let [SkEnt::Curve(c)] = picks[..] {
                result = Some(place(DimKind::Length(c), Some(p)));
            } else if let [SkEnt::Point(a), SkEnt::Point(b)] = picks[..] {
                // choose H/V/aligned by where the label is placed (Fusion style)
                let (pa, pb) = (sk.pos(a), sk.pos(b));
                let lo = pa.min(pb);
                let hi = pa.max(pb);
                let kind = if p.y > hi.y || p.y < lo.y {
                    if p.x >= lo.x && p.x <= hi.x {
                        DimKind::HDistance(a, b)
                    } else {
                        DimKind::Distance(a, b)
                    }
                } else if p.x > hi.x || p.x < lo.x {
                    DimKind::VDistance(a, b)
                } else {
                    DimKind::Distance(a, b)
                };
                result = Some(place(kind, Some(p)));
            } else {
                picks.clear();
            }
        }
    }
    if result.is_none() {
        match picks[..] {
            [SkEnt::Curve(a), SkEnt::Curve(b)] => {
                let (la, lb) = (sk.line_pts(a), sk.line_pts(b));
                if let (Some((a0, a1)), Some((b0, b1))) = (la, lb) {
                    let par = (a1 - a0).normalize_or_zero().perp_dot((b1 - b0).normalize_or_zero()).abs() < 1e-6;
                    if par {
                        let pb = match curve_kind(&sk, b) {
                            Some(CurveGeom::Line { a: pa, .. }) => pa,
                            _ => return,
                        };
                        result = Some(place(DimKind::PointLine(pb, a), None));
                    } else {
                        result = Some(place(DimKind::Angle(a, b), None));
                    }
                } else {
                    picks.clear();
                }
            }
            [SkEnt::Point(pp), SkEnt::Curve(c)] | [SkEnt::Curve(c), SkEnt::Point(pp)] => {
                if sk.line_pts(c).is_some() {
                    result = Some(place(DimKind::PointLine(pp, c), None));
                } else {
                    picks.clear();
                }
            }
            [SkEnt::Point(_), SkEnt::Point(_)] => {}
            [SkEnt::Curve(_)] | [SkEnt::Point(_)] => {}
            _ => picks.clear(),
        }
    }
    let s = session(app).unwrap();
    s.picks = picks;
    let Some((kind, at)) = result else { return };
    s.picks.clear();
    add_dim(app, fid, kind, at);
}

/// The dimension D adds for a selection (no placement click needed).
fn dim_kind_for(sk: &Sketch, ents: &[SkEnt]) -> Option<DimKind> {
    Some(match ents {
        [SkEnt::Curve(c)] => match curve_kind(sk, *c)? {
            CurveGeom::Circle { .. } => DimKind::Diameter(*c),
            CurveGeom::Arc { .. } => DimKind::Radius(*c),
            CurveGeom::Line { .. } => DimKind::Length(*c),
        },
        [SkEnt::Point(a), SkEnt::Point(b)] => {
            let d = sk.pos(*b) - sk.pos(*a);
            if d.y.abs() < 1e-9 {
                DimKind::HDistance(*a, *b)
            } else if d.x.abs() < 1e-9 {
                DimKind::VDistance(*a, *b)
            } else {
                DimKind::Distance(*a, *b)
            }
        }
        [SkEnt::Point(p), SkEnt::Curve(c)] | [SkEnt::Curve(c), SkEnt::Point(p)] => {
            sk.line_pts(*c)?;
            DimKind::PointLine(*p, *c)
        }
        [SkEnt::Curve(a), SkEnt::Curve(b)] => {
            let ((a0, a1), (b0, b1)) = (sk.line_pts(*a)?, sk.line_pts(*b)?);
            let par = (a1 - a0).normalize_or_zero().perp_dot((b1 - b0).normalize_or_zero()).abs() < 1e-6;
            if par {
                match curve_kind(sk, *b)? {
                    CurveGeom::Line { a: pb, .. } => DimKind::PointLine(pb, *a),
                    _ => return None,
                }
            } else {
                DimKind::Angle(*a, *b)
            }
        }
        _ => return None,
    })
}

/// D: dimension the selection (or what is under the cursor, or the curve just
/// drawn) right away and open its value box; otherwise pick the Dimension tool.
fn quick_dim(app: &mut App) {
    let Some(s) = session(app) else { return };
    let fid = s.fid;
    let mut ents = s.selection.clone();
    if ents.is_empty() {
        ents.extend(s.hover);
    }
    let drawing = matches!(s.tool, SkTool::Line | SkTool::Rect | SkTool::CenterRect | SkTool::Circle | SkTool::Arc);
    if ents.is_empty() && drawing {
        ents.extend(s.last_curve.map(SkEnt::Curve));
    }
    if let [SkEnt::Con(kid)] = ents[..] {
        start_dim_edit(app, kid);
        return;
    }
    let Some(sk) = sketch(app, fid).cloned() else { return };
    // the curve just drawn may have been deleted / trimmed since
    ents.retain(|e| match e {
        SkEnt::Curve(c) => sk.curve(*c).is_some(),
        SkEnt::Point(p) => sk.point(*p).is_some(),
        _ => false,
    });
    if let Some(kind) = dim_kind_for(&sk, &ents) {
        if let Some(s) = session(app) {
            s.selection.clear();
            s.picks.clear();
        }
        add_dim(app, fid, kind, None);
        return;
    }
    set_tool(app, SkTool::Dimension);
}

/// Add a driving dimension at its current value and open the value box.
fn add_dim(app: &mut App, fid: FeatureId, kind: DimKind, at: Option<DVec2>) {
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let Some(s) = session(app) else { return };
    let px = s.px;
    let value = sk.measure(&kind);
    let (anchor, _) = dim_anchor(&sk, &kind);
    let offset = match at {
        Some(at) => at - anchor,
        None => default_dim_offset(&sk, &kind, px),
    };
    let before = app.doc.clone();
    let expr = fmt_num(value);
    let kid = sketch_mut(app, fid).map(|skm| {
        skm.add_constraint(Constraint::Dimension { kind, expr: expr.clone(), value, offset: offset.to_array() })
    });
    if !solve(app, &[], true) {
        app.doc = before;
        solve(app, &[], true);
        app.error("That dimension would over-constrain the sketch");
        return;
    }
    commit(app, before);
    if let Some(kid) = kid {
        start_dim_edit(app, kid);
    }
}

fn start_dim_edit(app: &mut App, kid: KId) {
    let Some(fid) = session(app).map(|s| s.fid) else { return };
    let expr = sketch(app, fid)
        .and_then(|sk| sk.constraints.iter().find(|k| k.id == kid))
        .and_then(|k| match &k.c {
            Constraint::Dimension { expr, .. } => Some(expr.clone()),
            _ => None,
        });
    if let (Some(e), Some(s)) = (expr, session(app)) {
        s.edit_dim = Some((kid, e, true));
    }
}

fn apply_dim_edit(app: &mut App, kid: KId, text: String) {
    let Some(fid) = session(app).map(|s| s.fid) else { return };
    if let Err(e) = app.model.params.eval(&text) {
        app.error(format!("Invalid value: {e}"));
        return;
    }
    let before = app.doc.clone();
    if let Some(sk) = sketch_mut(app, fid) {
        if let Some(k) = sk.constraints.iter_mut().find(|k| k.id == kid) {
            if let Constraint::Dimension { expr, .. } = &mut k.c {
                *expr = text;
            }
        }
    }
    if !solve(app, &[], true) {
        app.doc = before;
        solve(app, &[], true);
        app.error("The sketch cannot satisfy that value");
        return;
    }
    commit(app, before);
}

// ------------------------------------------------------------------ drawing

fn sk_color(s: &SketchSession, ent: SkEnt, fixed: bool, construction: bool, failed: bool) -> [f32; 4] {
    if s.hover == Some(ent) {
        theme::SK_HOVER
    } else if s.selection.contains(&ent) || s.picks.contains(&ent) {
        theme::SK_SEL
    } else if failed {
        theme::SK_ERR
    } else if construction {
        theme::SK_CONSTR
    } else if fixed {
        theme::SK_FIXED
    } else {
        theme::SK_FREE
    }
}

pub fn draw(app: &mut App, scene: &mut Scene, view: &View) {
    let Mode::Sketch(s) = &app.mode else { return };
    let fid = s.fid;
    let Some(sk) = sketch(app, fid) else { return };
    let Some(ss) = app.display_state().sketches.get(&fid).cloned() else { return };
    let f = ss.frame;
    let tol = s.px * 0.5;
    // grid on the sketch plane (the same steps the cursor snaps to)
    let d = app.cam.cam.distance.max(1e-3);
    let gpx = if s.grid_px > 0.0 { s.grid_px } else { d as f64 / 900.0 };
    let (step, major_every) = grid_step(gpx);
    let n = ((view.size.length() as f64 * 0.5 * gpx / step).ceil() as i32 + 2).clamp(10, 90);
    let t = f.to2(app.cam.cam.target.as_dvec3());
    let (cx, cy) = ((t.x / step).round() * step, (t.y / step).round() * step);
    let alpha = if s.grid_on { 1.0 } else { 0.6 };
    for i in -n..=n {
        let fade = 1.0 - (i as f32 / n as f32).powi(2);
        let ix = (cx / step).round() as i64 + i as i64;
        let a = if ix.rem_euclid(major_every) == 0 { 0.16 } else { 0.065 };
        let col = [0.65, 0.72, 0.82, a * fade * alpha];
        let x = cx + i as f64 * step;
        push_line(&mut scene.lines, f.to3(DVec2::new(x, cy - n as f64 * step)).as_vec3(), f.to3(DVec2::new(x, cy + n as f64 * step)).as_vec3(), col, 1.0);
        let iy = (cy / step).round() as i64 + i as i64;
        let a = if iy.rem_euclid(major_every) == 0 { 0.16 } else { 0.065 };
        let col = [0.65, 0.72, 0.82, a * fade * alpha];
        let y = cy + i as f64 * step;
        push_line(&mut scene.lines, f.to3(DVec2::new(cx - n as f64 * step, y)).as_vec3(), f.to3(DVec2::new(cx + n as f64 * step, y)).as_vec3(), col, 1.0);
    }
    // alignment guides
    for (from, to) in &s.guides {
        let pts = [f.to3(*from).as_vec3(), f.to3(*to).as_vec3()];
        push_dashed(&mut scene.overlay_lines, &pts, [1.0, 0.72, 0.3, 0.85], 1.2, (s.px * 5.0) as f32);
    }
    // sketch axes
    let ax = (d * 0.3) as f64;
    push_line(&mut scene.overlay_lines, f.to3(DVec2::new(-ax, 0.0)).as_vec3(), f.to3(DVec2::new(ax, 0.0)).as_vec3(), [0.9, 0.35, 0.35, 0.35], 1.0);
    push_line(&mut scene.overlay_lines, f.to3(DVec2::new(0.0, -ax)).as_vec3(), f.to3(DVec2::new(0.0, ax)).as_vec3(), [0.35, 0.85, 0.4, 0.35], 1.0);

    // profiles
    for p in &ss.profiles {
        for tri in &p.tris {
            for q in tri {
                scene.fills.push(crate::render::FillVert { p: f.to3(*q).as_vec3().to_array(), color: [0.40, 0.62, 0.95, 0.16] });
            }
        }
    }
    let failed: HashSet<KId> = s.report.failed.iter().map(|x| x.0).collect();
    let failing_curves: HashSet<CId> = sk
        .constraints
        .iter()
        .filter(|k| failed.contains(&k.id))
        .flat_map(|k| k.c.refs_curves())
        .collect();
    // curves
    for c in &sk.curves {
        let pts: Vec<Vec3> = sk.curve_polyline(c, tol).iter().map(|p| f.to3(*p).as_vec3()).collect();
        let fixed = s.report.fixed_curves.contains(&c.id);
        let col = sk_color(s, SkEnt::Curve(c.id), fixed, c.construction, failing_curves.contains(&c.id));
        let w = if s.hover == Some(SkEnt::Curve(c.id)) || s.selection.contains(&SkEnt::Curve(c.id)) { 3.0 } else { 2.0 };
        if c.construction {
            push_dashed(&mut scene.overlay_lines, &pts, col, w * 0.8, (s.px * 6.0) as f32);
        } else {
            push_polyline(&mut scene.overlay_lines, &pts, col, w);
        }
    }
    // splines
    for sp in &sk.splines {
        let pts: Vec<Vec3> = sk.spline_polyline(sp, tol).iter().map(|p| f.to3(*p).as_vec3()).collect();
        let fixed = sp.pts.iter().all(|p| s.report.fixed_points.contains(p));
        let col = sk_color(s, SkEnt::Curve(sp.id), fixed, sp.construction, false);
        let w = if s.hover == Some(SkEnt::Curve(sp.id)) || s.selection.contains(&SkEnt::Curve(sp.id)) { 3.0 } else { 2.0 };
        if sp.construction {
            push_dashed(&mut scene.overlay_lines, &pts, col, w * 0.8, (s.px * 6.0) as f32);
        } else {
            push_polyline(&mut scene.overlay_lines, &pts, col, w);
        }
    }
    // points
    for p in &sk.points {
        let fixed = s.report.fixed_points.contains(&p.id);
        let col = sk_color(s, SkEnt::Point(p.id), fixed, false, false);
        let size = if s.hover == Some(SkEnt::Point(p.id)) { 10.0 } else if p.fixed { 8.0 } else { 6.5 };
        scene.points.push(PointInst { p: f.to3(p.pos()).as_vec3().to_array(), color: col, size });
    }
    // texts
    let params = app.model.params.clone();
    for t in &sk.texts {
        let col = sk_color(s, SkEnt::Text(t.id), s.report.fixed_points.contains(&t.anchor), false, false);
        for c in sk.text_outlines(t, &params) {
            let mut pts: Vec<Vec3> = c.iter().map(|p| f.to3(*p).as_vec3()).collect();
            if let Some(first) = pts.first().copied() {
                pts.push(first);
            }
            push_polyline(&mut scene.overlay_lines, &pts, col, 1.6);
        }
    }
    // offset tool: highlight the chain under the cursor / preview the offset
    if s.tool == SkTool::Offset {
        let src = s.offset_src.or(match s.hover {
            Some(SkEnt::Curve(c)) => Some(c),
            _ => None,
        });
        if let Some(src) = src {
            let chain = offset_chain(sk, src);
            for (cid, _) in &chain.elems {
                if let Some(c) = sk.curve(*cid) {
                    let pts: Vec<Vec3> = sk.curve_polyline(c, tol).iter().map(|p| f.to3(*p).as_vec3()).collect();
                    push_polyline(&mut scene.overlay_lines, &pts, theme::SK_HOVER, 3.0);
                }
            }
            if let (Some(_), Some(cur)) = (s.offset_src, s.cursor) {
                let (side, d) = offset_side(sk, &chain, cur);
                if let Some(g) = offset_geometry(sk, &chain, side, snap_dist(d, s.px)) {
                    for poly in g.preview {
                        let pts: Vec<Vec3> = poly.iter().map(|p| f.to3(*p).as_vec3()).collect();
                        push_dashed(&mut scene.overlay_lines, &pts, [0.35, 0.75, 1.0, 0.95], 2.0, (s.px * 6.0) as f32);
                    }
                }
            }
        }
    }
    // rubber band preview
    if let Some(cur) = s.cursor {
        let accent = [0.35, 0.75, 1.0, 0.95];
        let chain = &s.chain;
        let to3 = |p: DVec2| f.to3(p).as_vec3();
        match s.tool {
            SkTool::Line => {
                if let Some((last, _)) = chain.last() {
                    push_line(&mut scene.overlay_lines, to3(*last), to3(cur), accent, 2.0);
                }
            }
            SkTool::Spline => {
                if !chain.is_empty() {
                    let mut pts: Vec<DVec2> = chain.iter().map(|c| c.0).collect();
                    pts.push(cur);
                    let poly = lusion_core::sketch::beziers_polyline(&lusion_core::sketch::catmull_rom_beziers(&pts, false), tol);
                    let p3: Vec<Vec3> = poly.iter().map(|p| to3(*p)).collect();
                    push_polyline(&mut scene.overlay_lines, &p3, accent, 2.0);
                    for (q, _) in chain {
                        scene.points.push(PointInst { p: to3(*q).to_array(), color: accent, size: 7.0 });
                    }
                }
            }
            SkTool::Rect | SkTool::CenterRect => {
                if let Some((first, _)) = chain.first() {
                    let (c0, c1) = if s.tool == SkTool::Rect { (*first, cur) } else { (*first - (cur - *first), cur) };
                    let pts = [c0, DVec2::new(c1.x, c0.y), c1, DVec2::new(c0.x, c1.y), c0];
                    let p3: Vec<Vec3> = pts.iter().map(|p| to3(*p)).collect();
                    push_polyline(&mut scene.overlay_lines, &p3, accent, 2.0);
                }
            }
            SkTool::Circle => {
                if let Some((c, _)) = chain.first() {
                    let r = (cur - *c).length();
                    let pts: Vec<Vec3> = (0..=96).map(|i| to3(*c + DVec2::from_angle(i as f64 / 96.0 * std::f64::consts::TAU) * r)).collect();
                    push_polyline(&mut scene.overlay_lines, &pts, accent, 2.0);
                }
            }
            SkTool::Arc => match chain.len() {
                1 => push_line(&mut scene.overlay_lines, to3(chain[0].0), to3(cur), accent, 1.5),
                2 => {
                    if let Some((c, ccw)) = arc_from_3(chain[0].0, chain[1].0, cur) {
                        let (a, b) = if ccw { (chain[0].0, chain[1].0) } else { (chain[1].0, chain[0].0) };
                        let r = (a - c).length();
                        let a0 = (a - c).to_angle();
                        let mut sw = (b - c).to_angle() - a0;
                        if sw <= 0.0 {
                            sw += std::f64::consts::TAU;
                        }
                        let pts: Vec<Vec3> = (0..=64).map(|i| to3(c + DVec2::from_angle(a0 + sw * i as f64 / 64.0) * r)).collect();
                        push_polyline(&mut scene.overlay_lines, &pts, accent, 2.0);
                    }
                }
                _ => {}
            },
            _ => {}
        }
        // snap marker
        if let Some(sn) = s.snap {
            let col = match sn {
                Snap::Point(_) => [1.0, 0.85, 0.3, 1.0],
                Snap::Mid(_) => [0.4, 1.0, 0.6, 1.0],
                Snap::OnCurve(_) => [0.9, 0.9, 0.95, 1.0],
                Snap::Model(_) => [1.0, 0.55, 0.85, 1.0],
            };
            scene.points.push(PointInst { p: to3(cur).to_array(), color: col, size: 11.0 });
        } else if matches!(s.tool, SkTool::Line | SkTool::Rect | SkTool::CenterRect | SkTool::Circle | SkTool::Arc | SkTool::Spline | SkTool::Point) {
            let (col, size) = if !s.guides.is_empty() {
                ([1.0, 0.72, 0.3, 1.0], 8.0)
            } else if s.on_grid {
                ([0.55, 0.8, 1.0, 1.0], 7.0)
            } else {
                ([0.9, 0.95, 1.0, 0.9], 6.0)
            };
            scene.points.push(PointInst { p: to3(cur).to_array(), color: col, size });
        }
    }
}

/// Screen-space dimension label rects (for hit testing).
fn dim_label_hit(app: &App, fid: FeatureId, view: &View) -> Option<KId> {
    let mouse = view.mouse?;
    let sk = sketch(app, fid)?;
    let f = app.display_state().sketches.get(&fid)?.frame;
    for k in &sk.constraints {
        if let Constraint::Dimension { kind, offset, .. } = &k.c {
            let (anchor, _) = dim_anchor(sk, kind);
            let lp = anchor + DVec2::from_array(*offset);
            if let Some(sp) = app.cam.project(f.to3(lp).as_vec3(), view.size) {
                if (sp - mouse).abs().cmple(Vec2::new(28.0, 10.0)).all() {
                    return Some(k.id);
                }
            }
        }
    }
    None
}

pub fn overlay(app: &mut App, ui: &mut egui::Ui, view: &View) {
    let Mode::Sketch(s) = &app.mode else { return };
    let fid = s.fid;
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let Some(f) = app.display_state().sketches.get(&fid).map(|x| x.frame) else { return };
    let painter = ui.painter_at(view.rect);
    let camsnap = app.cam.snapshot();
    let proj = |p: DVec2| -> Option<Pos2> { camsnap.project(f.to3(p).as_vec3(), view.size).map(|v| view.to_screen(v)) };
    let failed: HashSet<KId> = s.report.failed.iter().map(|x| x.0).collect();
    let dim_col = Color32::from_rgb(230, 236, 245);
    let font = egui::FontId::proportional(13.0);
    // dimensions
    for k in &sk.constraints {
        let Constraint::Dimension { kind, offset, value, expr } = &k.c else { continue };
        let hovered = s.hover == Some(SkEnt::Con(k.id));
        let selected = s.selection.contains(&SkEnt::Con(k.id));
        let col = if failed.contains(&k.id) {
            theme::ERR
        } else if hovered {
            theme::c32(theme::SK_HOVER)
        } else if selected {
            theme::c32(theme::SK_SEL)
        } else {
            dim_col
        };
        let stroke = Stroke::new(1.0, col.gamma_multiply(0.8));
        let (anchor, _) = dim_anchor(&sk, kind);
        let off = DVec2::from_array(*offset);
        let label_p = anchor + off;
        // leader geometry
        match *kind {
            DimKind::Distance(..) | DimKind::HDistance(..) | DimKind::VDistance(..) | DimKind::Length(_) => {
                let (pa, pb) = match *kind {
                    DimKind::Length(l) => sk.line_pts(l).unwrap_or_default(),
                    DimKind::Distance(a, b) | DimKind::HDistance(a, b) | DimKind::VDistance(a, b) => (sk.pos(a), sk.pos(b)),
                    _ => unreachable!(),
                };
                let (qa, qb) = match *kind {
                    DimKind::HDistance(..) => (DVec2::new(pa.x, label_p.y), DVec2::new(pb.x, label_p.y)),
                    DimKind::VDistance(..) => (DVec2::new(label_p.x, pa.y), DVec2::new(label_p.x, pb.y)),
                    _ => {
                        let dir = (pb - pa).normalize_or(DVec2::X);
                        let n = dir.perp();
                        let h = (label_p - (pa + pb) * 0.5).dot(n);
                        (pa + n * h, pb + n * h)
                    }
                };
                if let (Some(sa), Some(sb), Some(ea), Some(eb)) = (proj(pa), proj(pb), proj(qa), proj(qb)) {
                    painter.line_segment([sa, ea], stroke);
                    painter.line_segment([sb, eb], stroke);
                    painter.line_segment([ea, eb], stroke);
                    arrow(&painter, ea, eb, stroke);
                    arrow(&painter, eb, ea, stroke);
                }
            }
            DimKind::PointLine(p, l) => {
                let (la, lb) = sk.line_pts(l).unwrap_or_default();
                let pp = sk.pos(p);
                let d = lb - la;
                let foot = la + d * ((pp - la).dot(d) / d.length_squared().max(1e-24));
                if let (Some(a), Some(b)) = (proj(pp), proj(foot)) {
                    painter.line_segment([a, b], stroke);
                    arrow(&painter, a, b, stroke);
                    arrow(&painter, b, a, stroke);
                }
            }
            DimKind::Radius(c) | DimKind::Diameter(c) => {
                if let Some(curve) = sk.curve(c) {
                    let (cc, r) = match curve.geom {
                        CurveGeom::Circle { c, r } => (sk.pos(c), r),
                        CurveGeom::Arc { c, a, .. } => (sk.pos(c), (sk.pos(a) - sk.pos(c)).length()),
                        _ => continue,
                    };
                    let dir = (label_p - cc).normalize_or(DVec2::X);
                    let from = if matches!(kind, DimKind::Diameter(_)) { cc - dir * r } else { cc };
                    if let (Some(a), Some(b), Some(l)) = (proj(from), proj(cc + dir * r), proj(label_p)) {
                        painter.line_segment([a, b], stroke);
                        arrow(&painter, a, b, stroke);
                        if (l - b).length() > 2.0 {
                            painter.line_segment([b, l], stroke);
                        }
                    }
                }
            }
            DimKind::Angle(..) => {
                if let (Some(a), Some(l)) = (proj(anchor), proj(label_p)) {
                    painter.line_segment([a, l], Stroke::new(1.0, col.gamma_multiply(0.5)));
                }
            }
        }
        if let Some(lp) = proj(label_p) {
            let txt = match kind {
                DimKind::Diameter(_) => format!("Ø{}", fmt_num(*value)),
                DimKind::Radius(_) => format!("R{}", fmt_num(*value)),
                DimKind::Angle(..) => format!("{}°", fmt_num(*value)),
                _ => fmt_num(*value),
            };
            let txt = if expr.trim().parse::<f64>().is_err() { format!("ƒx {txt}") } else { txt };
            let galley = painter.layout_no_wrap(txt, font.clone(), col);
            let r = egui::Rect::from_center_size(lp, galley.size() + egui::vec2(8.0, 4.0));
            painter.rect_filled(r, 3.0, Color32::from_rgba_unmultiplied(20, 24, 30, 200));
            painter.galley(r.min + egui::vec2(4.0, 2.0), galley, col);
        }
    }
    // constraint glyphs
    let mut glyph_count: std::collections::HashMap<CId, usize> = std::collections::HashMap::new();
    for k in &sk.constraints {
        let (g, at): (&str, Option<DVec2>) = match &k.c {
            Constraint::Horizontal(c) => ("H", sk.line_pts(*c).map(|(a, b)| (a + b) * 0.5)),
            Constraint::Vertical(c) => ("V", sk.line_pts(*c).map(|(a, b)| (a + b) * 0.5)),
            Constraint::Parallel(a, _) => ("//", sk.line_pts(*a).map(|(a, b)| (a + b) * 0.5)),
            Constraint::Perpendicular(a, _) => (ic::ANGLE, sk.line_pts(*a).map(|(a, b)| (a + b) * 0.5)),
            Constraint::Equal(a, _) => (ic::EQUALS, curve_mid(&sk, *a)),
            Constraint::Tangent(a, _) => (ic::CIRCLE_HALF, curve_mid(&sk, *a)),
            Constraint::Concentric(a, _) => (ic::TARGET, curve_mid(&sk, *a)),
            Constraint::Midpoint(p, _) => (ic::TRIANGLE, Some(sk.pos(*p))),
            _ => continue,
        };
        let Some(at) = at else { continue };
        let key = k.c.refs_curves().first().copied().unwrap_or(0);
        let n = glyph_count.entry(key).or_insert(0);
        if let Some(sp) = proj(at) {
            let pos = sp + egui::vec2(10.0 + *n as f32 * 14.0, -12.0);
            *n += 1;
            let col = if failed.contains(&k.id) { theme::ERR } else { Color32::from_rgb(250, 210, 120) };
            let r = egui::Rect::from_center_size(pos, egui::vec2(13.0, 13.0));
            painter.rect_filled(r, 2.0, Color32::from_rgba_unmultiplied(30, 34, 40, 210));
            painter.text(pos, egui::Align2::CENTER_CENTER, g, egui::FontId::proportional(10.0), col);
        }
    }
    // offset distance label
    if let (SkTool::Offset, Some(src), Some(c)) = (s.tool, s.offset_src, s.cursor) {
        let chain = offset_chain(&sk, src);
        let (_, d) = offset_side(&sk, &chain, c);
        if let Some(sp) = proj(c) {
            painter.text(sp + egui::vec2(14.0, -16.0), egui::Align2::LEFT_BOTTOM, format!("{} mm", fmt_num(snap_dist(d, s.px))), egui::FontId::proportional(12.0), Color32::from_rgb(170, 210, 255));
        }
    }
    // H/V inference hint
    if let (Some(h), Some(c)) = (s.hv_hint, s.cursor) {
        if let Some(sp) = proj(c) {
            painter.text(sp + egui::vec2(12.0, 12.0), egui::Align2::LEFT_TOP, if h { "H" } else { "V" }, egui::FontId::proportional(11.0), Color32::from_rgb(250, 210, 120));
        }
    }
    // live length while drawing
    if let (Some(c), Some((last, _))) = (s.cursor, s.chain.last()) {
        if let Some(sp) = proj(c) {
            let txt = match s.tool {
                SkTool::Line => Some(format!("{} mm", fmt_num((c - *last).length()))),
                SkTool::Circle => Some(format!("Ø{} mm", fmt_num(2.0 * (c - *last).length()))),
                SkTool::Rect => Some(format!("{} × {}", fmt_num((c.x - last.x).abs()), fmt_num((c.y - last.y).abs()))),
                SkTool::CenterRect => Some(format!("{} × {}", fmt_num(2.0 * (c.x - last.x).abs()), fmt_num(2.0 * (c.y - last.y).abs()))),
                _ => None,
            };
            if let Some(t) = txt {
                painter.text(sp + egui::vec2(14.0, -16.0), egui::Align2::LEFT_BOTTOM, t, egui::FontId::proportional(12.0), Color32::from_rgb(170, 210, 255));
            }
        }
    }
    // text editor popup
    let te = match &mut app.mode {
        Mode::Sketch(s) => s.text_edit.take(),
        _ => None,
    };
    if let Some((tid, mut text, mut height, is_new, orig_text, orig_height)) = te {
        let anchor = sk.texts.iter().find(|t| t.id == tid).map(|t| sk.pos(t.anchor));
        let pos = anchor.and_then(|a| proj(a)).unwrap_or(view.rect.center()) + egui::vec2(10.0, 16.0);
        let mut keep = true;
        let mut apply = false;
        let mut cancel = false;
        egui::Area::new(egui::Id::new("text_edit")).fixed_pos(pos).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.label(RichText::new(format!("{} Text", ic::TEXT_T)).strong());
                let r = ui.add(egui::TextEdit::multiline(&mut text).desired_rows(1).desired_width(200.0));
                if is_new && !ui.memory(|m| m.has_focus(r.id)) && text == "Text" {
                    r.request_focus();
                }
                ui.horizontal(|ui| {
                    ui.label("Height");
                    ui.add(egui::TextEdit::singleline(&mut height).desired_width(60.0));
                    ui.label(RichText::new("mm").color(theme::TEXT_DIM));
                });
                ui.horizontal(|ui| {
                    if ui.button(format!("{} OK", ic::CHECK)).clicked() {
                        apply = true;
                    }
                    if ui.button("Cancel").clicked() {
                        cancel = true;
                    }
                });
                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    cancel = true;
                }
                if ui.input(|i| i.key_pressed(egui::Key::Enter) && i.modifiers.command) {
                    apply = true;
                }
            });
        });
        if apply || cancel {
            keep = false;
        }
        let fid = match &app.mode {
            Mode::Sketch(s) => s.fid,
            _ => return,
        };
        // the live preview already changed the document: undo/cancel go back to the originals
        let restore = |app: &mut App| {
            if let Some(skm) = sketch_mut(app, fid) {
                if let Some(t) = skm.texts.iter_mut().find(|t| t.id == tid) {
                    t.text = orig_text.clone();
                    t.height = orig_height.clone();
                }
            }
        };
        if apply && !text.trim().is_empty() {
            restore(app);
            let before = app.doc.clone();
            if let Some(skm) = sketch_mut(app, fid) {
                if let Some(t) = skm.texts.iter_mut().find(|t| t.id == tid) {
                    t.text = text.clone();
                    t.height = if height.trim().is_empty() { "5".into() } else { height.clone() };
                }
            }
            commit(app, before);
        } else if (cancel && is_new) || (apply && text.trim().is_empty()) {
            if let Some(skm) = sketch_mut(app, fid) {
                skm.delete(&[], &[tid], &[]);
            }
        } else if cancel {
            restore(app);
        }
        if keep {
            // live preview while typing
            let height_ok = app.model.params.eval(&height).is_ok();
            if let Some(skm) = sketch_mut(app, fid) {
                if let Some(t) = skm.texts.iter_mut().find(|t| t.id == tid) {
                    if !text.is_empty() {
                        t.text = text.clone();
                    }
                    if height_ok {
                        t.height = height.clone();
                    }
                }
            }
            if let Mode::Sketch(s) = &mut app.mode {
                s.text_edit = Some((tid, text, height, is_new, orig_text, orig_height));
            }
        }
    }
    // heads-up value box while drawing
    let typed = match &mut app.mode {
        Mode::Sketch(s) => s.typed.take().map(|t| (t, s.tool, s.cursor)),
        _ => None,
    };
    if let Some((mut text, tool, cursor)) = typed {
        let pos = cursor.and_then(|c| proj(c)).unwrap_or(view.rect.center()) + egui::vec2(18.0, 10.0);
        let mut keep = true;
        let mut apply = false;
        let hint = match tool {
            SkTool::Offset => "offset",
            SkTool::Line => "length",
            SkTool::Circle => "diameter",
            _ => "width x height",
        };
        egui::Area::new(egui::Id::new("typed_value")).fixed_pos(pos).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(RichText::new(hint).small().color(theme::TEXT_DIM));
                    let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(90.0));
                    if !r.has_focus() && !r.lost_focus() {
                        r.request_focus();
                        if let Some(mut st) = egui::TextEdit::load_state(ui.ctx(), r.id) {
                            let end = egui::text::CCursor::new(text.chars().count());
                            st.cursor.set_char_range(Some(egui::text::CCursorRange::one(end)));
                            st.store(ui.ctx(), r.id);
                        }
                    }
                    if r.lost_focus() {
                        apply = ui.input(|i| i.key_pressed(egui::Key::Enter));
                        keep = false;
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        keep = false;
                        apply = false;
                    }
                });
            });
        });
        if apply {
            apply_typed(app, &text);
        } else if keep {
            if let Mode::Sketch(s) = &mut app.mode {
                s.typed = Some(text);
            }
        }
    }
    // dimension edit box
    let edit = match &mut app.mode {
        Mode::Sketch(s) => s.edit_dim.take(),
        _ => None,
    };
    if let Some((kid, mut text, focus)) = edit {
        let pos = sk
            .constraints
            .iter()
            .find(|k| k.id == kid)
            .and_then(|k| match &k.c {
                Constraint::Dimension { kind, offset, .. } => {
                    let (a, _) = dim_anchor(&sk, kind);
                    proj(a + DVec2::from_array(*offset))
                }
                _ => None,
            })
            .unwrap_or(view.rect.center());
        let mut keep = true;
        let mut apply = false;
        egui::Area::new(egui::Id::new("dim_edit")).fixed_pos(pos - egui::vec2(60.0, 14.0)).order(egui::Order::Foreground).show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(110.0).hint_text("value or expression"));
                if focus {
                    r.request_focus();
                    // select all on first show
                    if let Some(mut st) = egui::TextEdit::load_state(ui.ctx(), r.id) {
                        let ccursor = egui::text::CCursorRange::two(egui::text::CCursor::new(0), egui::text::CCursor::new(text.chars().count()));
                        st.cursor.set_char_range(Some(ccursor));
                        st.store(ui.ctx(), r.id);
                    }
                }
                if r.lost_focus() {
                    if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        apply = true;
                    }
                    keep = false;
                }
                if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                    keep = false;
                }
                let v = app.model.params.eval(&text);
                match v {
                    Ok(v) => ui.label(RichText::new(format!("= {}", fmt_num(v))).small().color(theme::TEXT_DIM)),
                    Err(e) => ui.label(RichText::new(e).small().color(theme::ERR)),
                };
            });
        });
        if apply {
            apply_dim_edit(app, kid, text);
        } else if keep {
            if let Mode::Sketch(s) = &mut app.mode {
                s.edit_dim = Some((kid, text, false));
            }
        }
    }
}

fn curve_mid(sk: &Sketch, c: CId) -> Option<DVec2> {
    let curve = sk.curve(c)?;
    let pl = sk.curve_polyline(curve, 0.5);
    pl.get(pl.len() / 4).copied()
}

fn arrow(p: &egui::Painter, from: Pos2, to: Pos2, stroke: Stroke) {
    let d = to - from;
    let len = d.length();
    if len < 1e-3 {
        return;
    }
    let dir = d / len;
    let n = egui::vec2(-dir.y, dir.x);
    let s = 7.0;
    p.line_segment([to, to - dir * s + n * s * 0.4], stroke);
    p.line_segment([to, to - dir * s - n * s * 0.4], stroke);
}

/// Add the edges of the face the sketch sits on as fixed sketch geometry.
pub fn project_face_edges(app: &mut App) {
    let Some(fid) = session(app).map(|s| s.fid) else { return };
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let PlaneRef::Face(fr) = &sk.plane else {
        // not on a face: switch to the interactive project tool
        set_tool(app, SkTool::Project);
        return;
    };
    let st = app.pick_state().clone();
    let Some(body) = st.bodies.get(&fr.body) else { return };
    let Some(fi) = body.face_index(&fr.name) else {
        app.error("The sketch face could not be found");
        return;
    };
    let edges = body.shape.faces()[fi].edges();
    let refs = edge_refs(&st, fr.body, &edges);
    project_edges(app, &edges, &refs);
}

/// Persistent references of `edges` (sub-shapes of `body`), for linked projection.
fn edge_refs(st: &lusion_core::model::ModelState, body: lusion_core::doc::BodyId, edges: &[lusion_kernel::Shape]) -> Vec<Option<lusion_core::doc::EdgeRef>> {
    let Some(b) = st.bodies.get(&body) else { return vec![None; edges.len()] };
    let all = b.shape.edges();
    edges
        .iter()
        .map(|e| {
            let i = all.iter().position(|x| x.same(e))?;
            crate::commands::edge_ref(st, body, i as u32)
        })
        .collect()
}

/// Project a picked face (its outline) or edge.
fn project_pick(app: &mut App, pick: crate::app::Pick) {
    let st = app.pick_state().clone();
    let edges = match pick {
        crate::app::Pick::Face { body, face } => st.bodies.get(&body).and_then(|b| b.shape.faces().get(face as usize).map(|f| (body, f.edges()))),
        crate::app::Pick::Edge { body, edge } => st.bodies.get(&body).and_then(|b| b.shape.edges().get(edge as usize).cloned()).map(|e| (body, vec![e])),
        _ => None,
    };
    if let Some((body, edges)) = edges {
        let refs = edge_refs(&st, body, &edges);
        project_edges(app, &edges, &refs);
    }
}

/// Project 3D edges orthogonally onto the sketch plane as fixed geometry.
/// Edges with a reference stay linked: they follow the model when it changes.
fn project_edges(app: &mut App, edges: &[lusion_kernel::Shape], refs: &[Option<lusion_core::doc::EdgeRef>]) -> usize {
    use lusion_core::sketch::{LinkRole, SkLink};
    let Some(fid) = session(app).map(|s| s.fid) else { return 0 };
    let Some(frame) = frame_of(app, fid) else { return 0 };
    let before = app.doc.clone();
    let Some(skm) = sketch_mut(app, fid) else { return 0 };
    let fixed_pt = |skm: &mut Sketch, p: DVec2| -> PId {
        if let Some(id) = skm.points.iter().find(|q| (q.pos() - p).length() < 1e-7).map(|q| q.id) {
            return id;
        }
        let id = skm.add_point(p);
        skm.point_mut(id).unwrap().fixed = true;
        id
    };
    let mut n = 0;
    for (ei, e) in edges.iter().enumerate() {
        let Ok(info) = e.edge_info() else { continue };
        let eref = refs.get(ei).cloned().flatten();
        let mut link = |skm: &mut Sketch, points: Vec<(PId, LinkRole)>, radius: Option<KId>| {
            if let Some(er) = &eref {
                skm.links.push(SkLink { edge: er.clone(), points, radius });
            }
        };
        // circles stay circles only when they lie parallel to the sketch plane
        let flat_circle = info.kind == lusion_kernel::CurveKind::Circle && info.axis.normalize().cross(frame.n).length() < 1e-6;
        match info.kind {
            lusion_kernel::CurveKind::Line => {
                let a = fixed_pt(skm, frame.to2(info.start));
                let b = fixed_pt(skm, frame.to2(info.end));
                if a != b && (skm.pos(a) - skm.pos(b)).length() > 1e-7 {
                    skm.add_line(a, b);
                    link(skm, vec![(a, LinkRole::Start), (b, LinkRole::End)], None);
                    n += 1;
                }
            }
            lusion_kernel::CurveKind::Circle if flat_circle && info.closed => {
                let c = fixed_pt(skm, frame.to2(info.center));
                let ci = skm.add_curve(CurveGeom::Circle { c, r: info.radius }, false);
                let k = skm.add_constraint(Constraint::Dimension { kind: DimKind::Radius(ci), expr: fmt_num(info.radius), value: info.radius, offset: [info.radius * 0.7, info.radius * 0.7] });
                link(skm, vec![(c, LinkRole::Center)], Some(k));
                n += 1;
            }
            lusion_kernel::CurveKind::Circle if flat_circle => {
                let c = fixed_pt(skm, frame.to2(info.center));
                let (mut a, mut b) = (fixed_pt(skm, frame.to2(info.start)), fixed_pt(skm, frame.to2(info.end)));
                let (cc, pa, pb, pm) = (skm.pos(c), skm.pos(a), skm.pos(b), frame.to2(info.mid));
                let (mut ra, mut rb) = (LinkRole::Start, LinkRole::End);
                if !angle_in_arc(cc, pa, pb, pm) {
                    std::mem::swap(&mut a, &mut b);
                    std::mem::swap(&mut ra, &mut rb);
                }
                skm.add_curve(CurveGeom::Arc { c, a, b }, false);
                link(skm, vec![(c, LinkRole::Center), (a, ra), (b, rb)], None);
                n += 1;
            }
            lusion_kernel::CurveKind::Degenerated => {}
            _ => {
                let pts = e.edge_polyline(0.05, 0.2);
                let ids: Vec<PId> = pts.iter().map(|p| fixed_pt(skm, frame.to2(*p))).collect();
                let mut any = false;
                for w in ids.windows(2) {
                    if w[0] != w[1] {
                        skm.add_line(w[0], w[1]);
                        any = true;
                    }
                }
                n += any as usize;
            }
        }
    }
    solve(app, &[], true);
    commit(app, before);
    app.info(format!("Projected {n} edge{}", if n == 1 { "" } else { "s" }));
    n
}

// ------------------------------------------------------------------ offset

struct OffsetChain {
    /// (curve, traversed start→end in its stored direction)
    elems: Vec<(CId, bool)>,
    closed: bool,
}

fn curve_ends(sk: &Sketch, c: CId) -> Option<(PId, PId)> {
    match sk.curve(c)?.geom {
        CurveGeom::Line { a, b } | CurveGeom::Arc { a, b, .. } => Some((a, b)),
        CurveGeom::Circle { .. } => None,
    }
}

/// The connected chain of lines/arcs containing `start` (a circle is its own chain).
fn offset_chain(sk: &Sketch, start: CId) -> OffsetChain {
    let Some((sa, sb)) = curve_ends(sk, start) else {
        return OffsetChain { elems: vec![(start, true)], closed: true };
    };
    let at = |p: PId| -> Vec<CId> {
        sk.curves
            .iter()
            .filter(|c| curve_ends(sk, c.id).map(|(a, b)| a == p || b == p).unwrap_or(false))
            .map(|c| c.id)
            .collect()
    };
    let mut elems = std::collections::VecDeque::from(vec![(start, true)]);
    let mut seen: HashSet<CId> = HashSet::from([start]);
    // forward from sb
    let mut end = sb;
    loop {
        let next = at(end);
        if next.len() != 2 {
            break;
        }
        let Some(&c) = next.iter().find(|c| !seen.contains(c)) else { break };
        let (a, b) = curve_ends(sk, c).unwrap();
        let fwd = a == end;
        elems.push_back((c, fwd));
        seen.insert(c);
        end = if fwd { b } else { a };
    }
    let closed = end == sa;
    if !closed {
        let mut st = sa;
        loop {
            let prev = at(st);
            if prev.len() != 2 {
                break;
            }
            let Some(&c) = prev.iter().find(|c| !seen.contains(c)) else { break };
            let (a, b) = curve_ends(sk, c).unwrap();
            let fwd = b == st;
            elems.push_front((c, fwd));
            seen.insert(c);
            st = if fwd { a } else { b };
        }
    }
    OffsetChain { elems: elems.into(), closed }
}

/// Which side of the chain the cursor is on (+1 = left of travel) and how far.
fn offset_side(sk: &Sketch, ch: &OffsetChain, p: DVec2) -> (f64, f64) {
    let mut best = (1.0, f64::INFINITY);
    for &(cid, fwd) in &ch.elems {
        let Some(c) = sk.curve(cid) else { continue };
        let (side, d) = match c.geom {
            CurveGeom::Line { a, b } => {
                let (pa, pb) = if fwd { (sk.pos(a), sk.pos(b)) } else { (sk.pos(b), sk.pos(a)) };
                let t = (pb - pa).normalize_or_zero();
                (t.perp_dot(p - pa).signum(), dist_point_segment(p, pa, pb))
            }
            CurveGeom::Circle { c, r } => {
                let dc = (p - sk.pos(c)).length();
                // circles: "left" = inside
                (if dc < r { 1.0 } else { -1.0 }, (dc - r).abs())
            }
            CurveGeom::Arc { c, a, .. } => {
                let cc = sk.pos(c);
                let r = (sk.pos(a) - cc).length();
                let dc = (p - cc).length();
                // left of travel is towards the centre when going CCW (forward)
                let inside = dc < r;
                (if inside == fwd { 1.0 } else { -1.0 }, sk.curve_distance(sk.curve(cid).unwrap(), p))
            }
        };
        if d < best.1 {
            best = (side, d);
        }
    }
    best
}

fn snap_dist(d: f64, px: f64) -> f64 {
    let raw = (px * 3.0).max(1e-4);
    let mag = 10f64.powf(raw.log10().floor());
    let step = [1.0, 2.0, 5.0, 10.0].iter().map(|m| m * mag).find(|s| *s >= raw).unwrap_or(10.0 * mag);
    ((d / step).round() * step).max(step)
}

struct OffsetGeom {
    /// joints in sketch coordinates (open chain: n+1, closed: n)
    joints: Vec<DVec2>,
    /// per element: new radius for arcs / circles
    radii: Vec<Option<f64>>,
    preview: Vec<Vec<DVec2>>,
}

fn offset_geometry(sk: &Sketch, ch: &OffsetChain, side: f64, d: f64) -> Option<OffsetGeom> {
    let n = ch.elems.len();
    // circle
    if n == 1 {
        if let Some(CurveGeom::Circle { c, r }) = sk.curve(ch.elems[0].0).map(|c| c.geom) {
            let r2 = r - side * d;
            if r2 <= 1e-9 {
                return None;
            }
            let cc = sk.pos(c);
            let poly = (0..=96).map(|i| cc + DVec2::from_angle(i as f64 / 96.0 * std::f64::consts::TAU) * r2).collect();
            return Some(OffsetGeom { joints: vec![], radii: vec![Some(r2)], preview: vec![poly] });
        }
    }
    // offset end points of each element (in travel order) + new radius for arcs
    let mut segs: Vec<(DVec2, DVec2, Option<(DVec2, f64)>)> = Vec::with_capacity(n);
    for &(cid, fwd) in &ch.elems {
        let c = sk.curve(cid)?;
        match c.geom {
            CurveGeom::Line { a, b } => {
                let (pa, pb) = if fwd { (sk.pos(a), sk.pos(b)) } else { (sk.pos(b), sk.pos(a)) };
                let nrm = (pb - pa).normalize_or_zero().perp() * side * d;
                segs.push((pa + nrm, pb + nrm, None));
            }
            CurveGeom::Arc { c, a, b } => {
                let cc = sk.pos(c);
                let r = (sk.pos(a) - cc).length();
                let r2 = if fwd { r - side * d } else { r + side * d };
                if r2 <= 1e-9 {
                    return None;
                }
                let (pa, pb) = if fwd { (sk.pos(a), sk.pos(b)) } else { (sk.pos(b), sk.pos(a)) };
                segs.push((cc + (pa - cc).normalize_or_zero() * r2, cc + (pb - cc).normalize_or_zero() * r2, Some((cc, r2))));
            }
            CurveGeom::Circle { .. } => return None,
        }
    }
    let joint = |i: usize, j: usize| -> DVec2 {
        let (a0, a1, aa) = segs[i];
        let (b0, b1, ba) = segs[j];
        match (aa, ba) {
            (None, None) => {
                let (r, s2) = (a1 - a0, b1 - b0);
                let den = r.perp_dot(s2);
                if den.abs() < 1e-12 {
                    a1
                } else {
                    a0 + r * ((b0 - a0).perp_dot(s2) / den)
                }
            }
            (Some(_), None) => a1,
            (None, Some(_)) => b0,
            (Some(_), Some(_)) => (a1 + b0) * 0.5,
        }
    };
    let mut joints = Vec::new();
    if ch.closed {
        for i in 0..n {
            joints.push(joint((i + n - 1) % n, i));
        }
    } else {
        joints.push(segs[0].0);
        for i in 1..n {
            joints.push(joint(i - 1, i));
        }
        joints.push(segs[n - 1].1);
    }
    let mut preview = Vec::new();
    for (i, seg) in segs.iter().enumerate() {
        let (j0, j1) = (joints[i], joints[(i + 1) % joints.len()]);
        match seg.2 {
            None => preview.push(vec![j0, j1]),
            Some((cc, r2)) => {
                let a0 = (j0 - cc).to_angle();
                let mut sw = (j1 - cc).to_angle() - a0;
                // travel direction: forward arcs go CCW
                let fwd = ch.elems[i].1;
                if fwd && sw <= 0.0 {
                    sw += std::f64::consts::TAU;
                }
                if !fwd && sw >= 0.0 {
                    sw -= std::f64::consts::TAU;
                }
                preview.push((0..=48).map(|k| cc + DVec2::from_angle(a0 + sw * k as f64 / 48.0) * r2).collect());
            }
        }
    }
    let radii = segs.iter().map(|s| s.2.map(|x| x.1)).collect();
    Some(OffsetGeom { joints, radii, preview })
}

/// Create the offset of the chain containing `src`, on the cursor's side.
fn create_offset(app: &mut App, src: CId, cursor: DVec2, dist: Option<f64>) {
    let Some(fid) = session(app).map(|s| s.fid) else { return };
    let Some(sk) = sketch(app, fid).cloned() else { return };
    let px = session(app).map(|s| s.px).unwrap_or(0.1);
    let ch = offset_chain(&sk, src);
    let (side, dc) = offset_side(&sk, &ch, cursor);
    let d = dist.unwrap_or_else(|| snap_dist(dc, px));
    let Some(g) = offset_geometry(&sk, &ch, side, d) else {
        app.error("Offset too large for this geometry");
        return;
    };
    let before = app.doc.clone();
    // one parameter drives every offset distance of this chain
    let mut k = 1;
    while app.doc.parameters.iter().any(|p| p.name == format!("offset{k}")) {
        k += 1;
    }
    let pname = format!("offset{k}");
    app.doc.parameters.push(lusion_core::expr::Parameter { name: pname.clone(), expr: fmt_num(d), comment: "sketch offset".into() });
    app.model.params = lusion_core::expr::ParamValues::evaluate(&app.doc.parameters);
    let Some(skm) = sketch_mut(app, fid) else { return };
    let n = ch.elems.len();
    let construction = skm.curve(src).map(|c| c.construction).unwrap_or(false);
    if n == 1 && ch.closed && g.joints.is_empty() {
        // circle: concentric, radius = r ± offset
        if let (Some(CurveGeom::Circle { c, r }), Some(Some(r2))) = (skm.curve(src).map(|c| c.geom), g.radii.first()) {
            let ci = skm.add_curve(CurveGeom::Circle { c, r: *r2 }, construction);
            let expr = format!("2 * ({} {} {pname})", fmt_num(r), if *r2 > r { "+" } else { "-" });
            skm.add_constraint(Constraint::Dimension { kind: DimKind::Diameter(ci), expr, value: 2.0 * r2, offset: [r2 * 0.8, r2 * 0.8] });
        }
    } else {
        let jp: Vec<PId> = g.joints.iter().map(|p| skm.add_point(*p)).collect();
        let mut first_dim = true;
        let mut new_ids: Vec<Option<CId>> = vec![None; n];
        for (i, &(cid, fwd)) in ch.elems.iter().enumerate() {
            let (j0, j1) = (jp[i], jp[(i + 1) % jp.len()]);
            let Some(orig) = skm.curve(cid).map(|c| c.geom) else { continue };
            match orig {
                CurveGeom::Line { .. } => {
                    let nl = skm.add_curve(CurveGeom::Line { a: j0, b: j1 }, construction);
                    new_ids[i] = Some(nl);
                    skm.add_constraint(Constraint::Parallel(cid, nl));
                    let off = (skm.pos(j0) - skm.pos(j1)).perp().normalize_or_zero() * px * 18.0;
                    let _ = first_dim;
                    skm.add_constraint(Constraint::Dimension { kind: DimKind::PointLine(j0, cid), expr: pname.clone(), value: d, offset: off.to_array() });
                    first_dim = false;
                }
                CurveGeom::Arc { c, .. } => {
                    let (a, b) = if fwd { (j0, j1) } else { (j1, j0) };
                    let na = skm.add_curve(CurveGeom::Arc { c, a, b }, construction);
                    new_ids[i] = Some(na);
                    // radius follows from neighbouring lines when bounded by them; otherwise drive it
                    let prev_line = (i > 0 || ch.closed) && matches!(skm.curve(ch.elems[(i + n - 1) % n].0).map(|c| c.geom), Some(CurveGeom::Line { .. }));
                    let next_line = (i + 1 < n || ch.closed) && matches!(skm.curve(ch.elems[(i + 1) % n].0).map(|c| c.geom), Some(CurveGeom::Line { .. }));
                    if !(prev_line && next_line) {
                        let r = skm.radius_of(cid).unwrap_or(0.0);
                        let r2 = g.radii[i].unwrap_or(r);
                        let expr = format!("{} {} {pname}", fmt_num(r), if r2 > r { "+" } else { "-" });
                        skm.add_constraint(Constraint::Dimension { kind: DimKind::Radius(na), expr, value: r2, offset: [r2 * 0.6, r2 * 0.6] });
                    }
                }
                CurveGeom::Circle { .. } => {}
            }
        }
        // keep tangency where the original line/arc pairs were tangent
        for i in 0..n {
            let j = (i + 1) % n;
            if j == 0 && !ch.closed {
                break;
            }
            let (oi, oj) = (ch.elems[i].0, ch.elems[j].0);
            let (Some(ni), Some(nj)) = (new_ids[i], new_ids[j]) else { continue };
            let (line_o, arc_o, line_n, arc_n) = match (skm.curve(oi).map(|c| c.geom), skm.curve(oj).map(|c| c.geom)) {
                (Some(CurveGeom::Line { .. }), Some(CurveGeom::Arc { .. })) => (oi, oj, ni, nj),
                (Some(CurveGeom::Arc { .. }), Some(CurveGeom::Line { .. })) => (oj, oi, nj, ni),
                _ => continue,
            };
            let (Some((pa, pb)), Some(r)) = (skm.line_pts(line_o), skm.radius_of(arc_o)) else { continue };
            let center = match skm.curve(arc_o).map(|c| c.geom) {
                Some(CurveGeom::Arc { c, .. }) => skm.pos(c),
                _ => continue,
            };
            if (dist_point_line(center, pa, pb) - r).abs() < 1e-6 * (1.0 + r) {
                skm.add_constraint(Constraint::Tangent(line_n, arc_n));
            }
        }
    }
    if !solve(app, &[], true) {
        app.doc = before;
        app.model.params = lusion_core::expr::ParamValues::evaluate(&app.doc.parameters);
        solve(app, &[], true);
        app.error("Could not offset that chain (conflicting constraints?)");
        return;
    }
    commit(app, before);
    if let Some(s) = session(app) {
        s.offset_src = None;
    }
    app.info(format!("Offset {} mm — edit '{pname}' in Parameters to change it", fmt_num(d)));
}

/// Sketch toolbar contents (shown in the top bar while sketching).
pub fn toolbar(app: &mut App, ui: &mut egui::Ui) {
    let Mode::Sketch(s) = &app.mode else { return };
    let cur = s.tool;
    let mut tool_btn = |ui: &mut egui::Ui, t: SkTool, icon: &str, tip: &str| {
        let r = ui.add(egui::Button::selectable(cur == t, RichText::new(icon).size(18.0)).min_size(egui::vec2(32.0, 30.0))).on_hover_text(tip);
        if r.clicked() {
            set_tool(app, t);
        }
    };
    tool_btn(ui, SkTool::Select, ic::CURSOR, "Select / drag (Esc)");
    tool_btn(ui, SkTool::Line, ic::LINE_SEGMENT, "Line (L)");
    tool_btn(ui, SkTool::Rect, ic::RECTANGLE, "Rectangle from a corner (R; R again: from the centre)");
    tool_btn(ui, SkTool::CenterRect, ic::SQUARE_HALF, "Rectangle from the centre (R twice)");
    tool_btn(ui, SkTool::Circle, ic::CIRCLE, "Circle (C)");
    tool_btn(ui, SkTool::Arc, ic::CIRCLE_HALF, "3-point arc (A)");
    tool_btn(ui, SkTool::Spline, ic::WAVE_SINE, "Spline: smooth curve through points (B)");
    tool_btn(ui, SkTool::Point, ic::DOT_OUTLINE, "Point (P)");
    ui.separator();
    tool_btn(ui, SkTool::Dimension, ic::RULER, "Dimension (D). With something selected, D dimensions it right away");
    tool_btn(ui, SkTool::Trim, ic::SCISSORS, "Trim (T)");
    tool_btn(ui, SkTool::Fillet, ic::BEZIER_CURVE, "Sketch fillet: round a corner (G)");
    tool_btn(ui, SkTool::Text, ic::TEXT_T, "Text — for embossed / engraved labels");
    tool_btn(ui, SkTool::Offset, ic::SQUARE_SPLIT_HORIZONTAL, "Offset curves (O)");
    tool_btn(ui, SkTool::Project, ic::SELECTION_FOREGROUND, "Project faces / edges onto the sketch (P)");
    ui.menu_button(RichText::new(format!("{} Constrain", ic::LINK)).size(14.0), |ui| {
        for k in ConKind::ALL {
            if ui.button(format!("{}   {}", k.glyph(), k.label())).clicked() {
                set_tool(app, SkTool::Con(k));
                ui.close();
            }
        }
    });
    if ui.add(egui::Button::new(RichText::new(ic::LINE_SEGMENTS).size(18.0)).min_size(egui::vec2(32.0, 30.0))).on_hover_text("Toggle construction (X)").clicked() {
        toggle_construction(app);
    }
    if ui.add(egui::Button::new(RichText::new(ic::SELECTION_FOREGROUND).size(18.0)).min_size(egui::vec2(32.0, 30.0))).on_hover_text("Project the outline of the face this sketch is on").clicked() {
        project_face_edges(app);
    }
    ui.separator();
    let on = grid_snap_on(ui.ctx());
    if ui
        .add(egui::Button::selectable(on, RichText::new(ic::GRID_FOUR).size(18.0)).min_size(egui::vec2(32.0, 30.0)))
        .on_hover_text("Snap to the grid. Points, midpoints and horizontal/vertical alignment always snap; hold Ctrl to place freely")
        .clicked()
    {
        set_grid_snap(ui.ctx(), !on);
    }
}

#[cfg(test)]
mod offset_tests {
    use super::*;

    fn rounded_rect() -> (Sketch, CId) {
        let mut s = Sketch::new(PlaneRef::Origin(OriginPlane::XY));
        let p = [
            s.add_point(DVec2::new(0.0, 0.0)),
            s.add_point(DVec2::new(60.0, 0.0)),
            s.add_point(DVec2::new(60.0, 40.0)),
            s.add_point(DVec2::new(0.0, 40.0)),
        ];
        let l: Vec<CId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        sketch_fillet(&mut s, p[2]).unwrap();
        (s, l[0])
    }

    #[test]
    fn chain_and_inside_offset() {
        let (s, bottom) = rounded_rect();
        let ch = offset_chain(&s, bottom);
        assert!(ch.closed, "rounded rect is a closed chain");
        assert_eq!(ch.elems.len(), 5);
        let (side, d) = offset_side(&s, &ch, DVec2::new(30.0, 5.0));
        assert!((d - 5.0).abs() < 1e-9);
        let g = offset_geometry(&s, &ch, side, 5.0).expect("geometry");
        // inside offset: all joints inside the rectangle by 5
        for j in &g.joints {
            assert!(j.x >= 4.999 && j.x <= 55.001 && j.y >= 4.999 && j.y <= 35.001, "joint {j:?}");
        }
        let (side_out, _) = offset_side(&s, &ch, DVec2::new(30.0, -5.0));
        assert_eq!(side_out, -side);
        assert!(offset_geometry(&s, &ch, side_out, 5.0).is_some());
    }
}
