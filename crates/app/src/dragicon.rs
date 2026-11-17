//! Drag preview image: a small software-rendered thumbnail of the dragged
//! bodies plus a label, used both for the in-app ghost and for the X11 drag
//! icon window that follows the cursor outside the app.

use glam::{DVec2, Vec3};
use lusion_core::tess::BodyMesh;
use std::sync::Arc;

/// Premultiplied RGBA image (0..1).
#[derive(Clone)]
pub struct Img {
    pub w: usize,
    pub h: usize,
    pub px: Vec<[f32; 4]>,
}

impl Img {
    pub fn new(w: usize, h: usize) -> Img {
        Img { w, h, px: vec![[0.0; 4]; w * h] }
    }

    /// Source-over blend of a premultiplied colour.
    fn blend(&mut self, x: usize, y: usize, c: [f32; 4]) {
        if x >= self.w || y >= self.h {
            return;
        }
        let d = &mut self.px[y * self.w + x];
        let k = 1.0 - c[3];
        for i in 0..4 {
            d[i] = c[i] + d[i] * k;
        }
    }

    /// Draw `src` at (ox, oy).
    pub fn draw(&mut self, src: &Img, ox: usize, oy: usize) {
        for y in 0..src.h {
            for x in 0..src.w {
                self.blend(ox + x, oy + y, src.px[y * src.w + x]);
            }
        }
    }

    /// Packed 0xAARRGGBB (premultiplied) for X11 ARGB visuals.
    pub fn argb(&self) -> Vec<u32> {
        self.px
            .iter()
            .map(|p| {
                let q = |v: f32| (v.clamp(0.0, 1.0) * 255.0 + 0.5) as u32;
                (q(p[3]) << 24) | (q(p[0]) << 16) | (q(p[1]) << 8) | q(p[2])
            })
            .collect()
    }

    /// Straight-alpha sRGB bytes for egui.
    pub fn rgba8(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.px.len() * 4);
        for p in &self.px {
            let a = p[3];
            let un = |v: f32| if a > 1e-4 { (v / a).clamp(0.0, 1.0) } else { 0.0 };
            out.extend_from_slice(&[(un(p[0]) * 255.0) as u8, (un(p[1]) * 255.0) as u8, (un(p[2]) * 255.0) as u8, (a.clamp(0.0, 1.0) * 255.0) as u8]);
        }
        out
    }
}

/// Render the meshes seen along `fwd` (with `right`/`up` screen axes), fitted into a
/// `size`×`size` square with 3×3 supersampling.
pub fn thumbnail(meshes: &[(Arc<BodyMesh>, [f32; 3])], right: Vec3, up: Vec3, fwd: Vec3, size: usize) -> Img {
    const SS: usize = 3;
    let n = size * SS;
    // projected bounds
    let mut lo = DVec2::splat(f64::INFINITY);
    let mut hi = DVec2::splat(f64::NEG_INFINITY);
    for (m, _) in meshes {
        for p in &m.positions {
            let v = Vec3::from(*p);
            let q = DVec2::new(v.dot(right) as f64, v.dot(up) as f64);
            lo = lo.min(q);
            hi = hi.max(q);
        }
    }
    let mut out = Img::new(size, size);
    if !(lo.x <= hi.x) {
        return out;
    }
    let ext = (hi - lo).max_element().max(1e-6);
    let scale = (n as f64 * 0.88) / ext;
    let center = (lo + hi) * 0.5;
    let to_px = |v: Vec3| -> (f32, f32, f32) {
        let x = (v.dot(right) as f64 - center.x) * scale + n as f64 * 0.5;
        let y = n as f64 * 0.5 - (v.dot(up) as f64 - center.y) * scale;
        (x as f32, y as f32, v.dot(fwd))
    };
    let light = (-fwd * 0.75 + up * 0.55 - right * 0.35).normalize();
    let mut depth = vec![f32::INFINITY; n * n];
    let mut col = vec![[0.0f32; 3]; n * n];
    for (m, base) in meshes {
        let pts: Vec<(f32, f32, f32)> = m.positions.iter().map(|p| to_px(Vec3::from(*p))).collect();
        for t in m.indices.chunks_exact(3) {
            let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
            let (a, b, c) = (pts[i0], pts[i1], pts[i2]);
            let area = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
            if area.abs() < 1e-9 {
                continue;
            }
            let (n0, n1, n2) = (Vec3::from(m.normals[i0]), Vec3::from(m.normals[i1]), Vec3::from(m.normals[i2]));
            let x0 = a.0.min(b.0).min(c.0).floor().max(0.0) as usize;
            let x1 = (a.0.max(b.0).max(c.0).ceil() as usize).min(n - 1);
            let y0 = a.1.min(b.1).min(c.1).floor().max(0.0) as usize;
            let y1 = (a.1.max(b.1).max(c.1).ceil() as usize).min(n - 1);
            for y in y0..=y1 {
                let py = y as f32 + 0.5;
                for x in x0..=x1 {
                    let px = x as f32 + 0.5;
                    let w0 = ((b.0 - px) * (c.1 - py) - (b.1 - py) * (c.0 - px)) / area;
                    let w1 = ((c.0 - px) * (a.1 - py) - (c.1 - py) * (a.0 - px)) / area;
                    let w2 = 1.0 - w0 - w1;
                    if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                        continue;
                    }
                    let z = a.2 * w0 + b.2 * w1 + c.2 * w2;
                    let k = y * n + x;
                    if z >= depth[k] {
                        continue;
                    }
                    depth[k] = z;
                    let mut nn = (n0 * w0 + n1 * w1 + n2 * w2).normalize_or_zero();
                    if nn.dot(fwd) > 0.0 {
                        nn = -nn;
                    }
                    let diff = nn.dot(light).max(0.0);
                    let rim = (1.0 + nn.dot(fwd)).powi(3) * 0.12;
                    let s = 0.30 + 0.75 * diff + rim;
                    col[k] = [base[0] * s, base[1] * s, base[2] * s];
                }
            }
        }
    }
    // darken silhouettes/creases a little: depth discontinuities
    let dz = (depth.iter().filter(|d| d.is_finite()).fold(f32::NEG_INFINITY, |a, b| a.max(*b))
        - depth.iter().filter(|d| d.is_finite()).fold(f32::INFINITY, |a, b| a.min(*b)))
    .max(1e-6);
    for y in 0..size {
        for x in 0..size {
            let mut acc = [0.0f32; 4];
            for sy in 0..SS {
                for sx in 0..SS {
                    let (gx, gy) = (x * SS + sx, y * SS + sy);
                    let k = gy * n + gx;
                    if !depth[k].is_finite() {
                        continue;
                    }
                    let mut c = col[k];
                    let edge = [(1i32, 0i32), (0, 1), (-1, 0), (0, -1)].iter().any(|(ox, oy)| {
                        let (qx, qy) = (gx as i32 + ox * 2, gy as i32 + oy * 2);
                        if qx < 0 || qy < 0 || qx >= n as i32 || qy >= n as i32 {
                            return true;
                        }
                        let d = depth[qy as usize * n + qx as usize];
                        !d.is_finite() || (d - depth[k]).abs() / dz > 0.08
                    });
                    if edge {
                        c = [c[0] * 0.45, c[1] * 0.45, c[2] * 0.48];
                    }
                    acc[0] += c[0].min(1.0);
                    acc[1] += c[1].min(1.0);
                    acc[2] += c[2].min(1.0);
                    acc[3] += 1.0;
                }
            }
            let s = 1.0 / (SS * SS) as f32;
            out.px[y * size + x] = [acc[0] * s, acc[1] * s, acc[2] * s, acc[3] * s];
        }
    }
    out
}

/// Fill text (DejaVu Sans Bold) with the baseline-left at (x, y), cap height `cap` px.
pub fn text(img: &mut Img, x: f32, y: f32, s: &str, cap: f32, color: [f32; 3], max_w: f32) {
    let mut s = s.to_string();
    // ellipsize to fit
    while s.chars().count() > 3 {
        let w = lusion_core::text::bounds(&s, cap as f64).map(|(lo, hi)| hi.x - lo.x.min(0.0)).unwrap_or(0.0);
        if w <= max_w as f64 {
            break;
        }
        let mut c: Vec<char> = s.trim_end_matches('…').chars().collect();
        c.pop();
        s = c.into_iter().collect::<String>() + "…";
    }
    let contours = lusion_core::text::outlines(&s, cap as f64);
    if contours.is_empty() {
        return;
    }
    // edges in image space (y down)
    let mut edges: Vec<(DVec2, DVec2)> = vec![];
    let (mut ymin, mut ymax, mut xmax) = (f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for c in &contours {
        for i in 0..c.len() {
            let a = DVec2::new(x as f64 + c[i].x, y as f64 - c[i].y);
            let b = DVec2::new(x as f64 + c[(i + 1) % c.len()].x, y as f64 - c[(i + 1) % c.len()].y);
            ymin = ymin.min(a.y);
            ymax = ymax.max(a.y);
            xmax = xmax.max(a.x);
            edges.push((a, b));
        }
    }
    const SUB: usize = 4;
    let y0 = ymin.floor().max(0.0) as usize;
    let y1 = (ymax.ceil() as usize).min(img.h.saturating_sub(1));
    let wpx = (xmax.ceil() as usize + 1).min(img.w);
    let mut cov = vec![0.0f32; wpx];
    for py in y0..=y1 {
        cov.iter_mut().for_each(|c| *c = 0.0);
        for sy in 0..SUB {
            let yy = py as f64 + (sy as f64 + 0.5) / SUB as f64;
            // crossings with winding direction
            let mut xs: Vec<(f64, i32)> = edges
                .iter()
                .filter_map(|(a, b)| {
                    if (a.y <= yy) == (b.y <= yy) {
                        return None;
                    }
                    let t = (yy - a.y) / (b.y - a.y);
                    Some((a.x + t * (b.x - a.x), if b.y > a.y { 1 } else { -1 }))
                })
                .collect();
            xs.sort_by(|a, b| a.0.total_cmp(&b.0));
            let mut wind = 0;
            for i in 0..xs.len() {
                wind += xs[i].1;
                if wind != 0 && i + 1 < xs.len() {
                    // span [xs[i].0, xs[i+1].0): exact horizontal coverage
                    let (sa, sb) = (xs[i].0.max(0.0), xs[i + 1].0.min(wpx as f64));
                    let mut px = sa.floor() as usize;
                    while (px as f64) < sb && px < wpx {
                        let l = sa.max(px as f64);
                        let r = sb.min(px as f64 + 1.0);
                        if r > l {
                            cov[px] += ((r - l) / SUB as f64) as f32;
                        }
                        px += 1;
                    }
                }
            }
        }
        for (px, c) in cov.iter().enumerate() {
            let a = c.min(1.0);
            if a > 0.0 {
                img.blend(px, py, [color[0] * a, color[1] * a, color[2] * a, a]);
            }
        }
    }
}

/// Rounded rectangle fill (premultiplied colour), anti-aliased corners.
pub fn round_rect(img: &mut Img, x0: f32, y0: f32, x1: f32, y1: f32, r: f32, c: [f32; 4]) {
    for y in y0.floor().max(0.0) as usize..(y1.ceil() as usize).min(img.h) {
        for x in x0.floor().max(0.0) as usize..(x1.ceil() as usize).min(img.w) {
            let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
            let qx = (px - (x0 + x1) * 0.5).abs() - ((x1 - x0) * 0.5 - r);
            let qy = (py - (y0 + y1) * 0.5).abs() - ((y1 - y0) * 0.5 - r);
            let d = (qx.max(0.0).hypot(qy.max(0.0)) + qx.max(qy).min(0.0)) - r;
            let a = (0.5 - d).clamp(0.0, 1.0);
            if a > 0.0 {
                img.blend(x, y, [c[0] * a, c[1] * a, c[2] * a, c[3] * a]);
            }
        }
    }
}

pub const ICON_W: usize = 280;
pub const ICON_H: usize = 76;

/// Full drag icon: card with thumbnail, file name and a hint line.
/// `rounded = false` for servers without an ARGB visual (square opaque card).
pub fn card(thumb: Option<&Img>, name: &str, hint: &str, accent: [f32; 3], hint_color: [f32; 3], rounded: bool) -> Img {
    let mut img = Img::new(ICON_W, ICON_H);
    let (w, h) = (ICON_W as f32, ICON_H as f32);
    let r = if rounded { 10.0 } else { 0.0 };
    round_rect(&mut img, 0.0, 0.0, w, h, r, [accent[0], accent[1], accent[2], 1.0]);
    let bg = [0.105, 0.120, 0.145];
    round_rect(&mut img, 1.5, 1.5, w - 1.5, h - 1.5, (r - 1.5).max(0.0), [bg[0], bg[1], bg[2], 1.0]);
    let ts = ICON_H - 12;
    round_rect(&mut img, 6.0, 6.0, 6.0 + ts as f32, 6.0 + ts as f32, if rounded { 7.0 } else { 0.0 }, [0.16, 0.18, 0.215, 1.0]);
    if let Some(t) = thumb {
        img.draw(t, 6 + (ts.saturating_sub(t.w)) / 2, 6 + (ts.saturating_sub(t.h)) / 2);
    }
    let tx = 6.0 + ts as f32 + 10.0;
    let max_w = w - tx - 10.0;
    text(&mut img, tx, 31.0, name, 10.5, [0.95, 0.96, 0.98], max_w);
    text(&mut img, tx, 54.0, hint, 7.5, hint_color, max_w);
    img
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn card_renders_text_and_thumbnail() {
        let m = BodyMesh {
            positions: vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 10.0, 0.0]],
            normals: vec![[0.0, 0.0, 1.0]; 3],
            indices: vec![0, 1, 2],
            ..Default::default()
        };
        let t = thumbnail(&[(Arc::new(m), [0.8, 0.8, 0.8])], Vec3::X, Vec3::Y, Vec3::NEG_Z, 48);
        assert!(t.px.iter().filter(|p| p[3] > 0.5).count() > 300, "triangle should cover about half the thumbnail");
        let c = card(Some(&t), "bracket-Body1.stl", "release to drop", [0.2, 0.6, 1.0], [0.6, 0.6, 0.6], true);
        // text pixels: bright pixels right of the thumbnail
        let bright = (0..ICON_H).flat_map(|y| (80..ICON_W).map(move |x| (x, y))).filter(|(x, y)| c.px[y * ICON_W + x][0] > 0.8).count();
        assert!(bright > 100, "label text should be drawn ({bright} px)");
        // corners transparent when rounded
        assert!(c.px[0][3] < 0.1);
    }
}
