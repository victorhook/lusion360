//! Text → closed outline polylines (for embossed / engraved labels).

use glam::DVec2;
use std::sync::OnceLock;

static FONT: &[u8] = include_bytes!("../assets/DejaVuSans-Bold.ttf");

fn face() -> Option<&'static ttf_parser::Face<'static>> {
    static F: OnceLock<Option<ttf_parser::Face<'static>>> = OnceLock::new();
    F.get_or_init(|| ttf_parser::Face::parse(FONT, 0).ok()).as_ref()
}

struct Builder {
    contours: Vec<Vec<DVec2>>,
    cur: Vec<DVec2>,
    scale: f64,
    offset: DVec2,
    steps: usize,
}

impl Builder {
    fn p(&self, x: f32, y: f32) -> DVec2 {
        self.offset + DVec2::new(x as f64, y as f64) * self.scale
    }
    fn last(&self) -> DVec2 {
        *self.cur.last().unwrap_or(&self.offset)
    }
}

impl ttf_parser::OutlineBuilder for Builder {
    fn move_to(&mut self, x: f32, y: f32) {
        if self.cur.len() > 2 {
            self.contours.push(std::mem::take(&mut self.cur));
        }
        self.cur.clear();
        let p = self.p(x, y);
        self.cur.push(p);
    }
    fn line_to(&mut self, x: f32, y: f32) {
        let p = self.p(x, y);
        self.cur.push(p);
    }
    fn quad_to(&mut self, x1: f32, y1: f32, x: f32, y: f32) {
        let (p0, c, p1) = (self.last(), self.p(x1, y1), self.p(x, y));
        for i in 1..=self.steps {
            let t = i as f64 / self.steps as f64;
            let a = p0.lerp(c, t);
            let b = c.lerp(p1, t);
            self.cur.push(a.lerp(b, t));
        }
    }
    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x: f32, y: f32) {
        let (p0, c1, c2, p1) = (self.last(), self.p(x1, y1), self.p(x2, y2), self.p(x, y));
        for i in 1..=self.steps {
            let t = i as f64 / self.steps as f64;
            let u = 1.0 - t;
            self.cur.push(p0 * (u * u * u) + c1 * (3.0 * u * u * t) + c2 * (3.0 * u * t * t) + p1 * (t * t * t));
        }
    }
    fn close(&mut self) {
        if self.cur.len() > 2 {
            // drop a duplicated closing point
            if (self.cur[0] - *self.cur.last().unwrap()).length() < 1e-9 {
                self.cur.pop();
            }
            self.contours.push(std::mem::take(&mut self.cur));
        }
        self.cur.clear();
    }
}

/// Closed contours of `text` with the baseline-left corner at the origin.
/// `height` is the capital-letter height (mm).
pub fn outlines(text: &str, height: f64) -> Vec<Vec<DVec2>> {
    let Some(f) = face() else { return vec![] };
    let cap = f.capital_height().map(|c| c as f64).filter(|c| *c > 0.0).unwrap_or(f.units_per_em() as f64 * 0.72);
    let scale = height / cap;
    let line_gap = f.height() as f64 * scale;
    let mut b = Builder { contours: vec![], cur: vec![], scale, offset: DVec2::ZERO, steps: 6 };
    let mut pen = DVec2::ZERO;
    for ch in text.chars() {
        if ch == '\n' {
            pen = DVec2::new(0.0, pen.y - line_gap);
            continue;
        }
        let Some(g) = f.glyph_index(ch) else {
            pen.x += f.units_per_em() as f64 * 0.5 * scale;
            continue;
        };
        b.offset = pen;
        f.outline_glyph(g, &mut b);
        if b.cur.len() > 2 {
            let c = std::mem::take(&mut b.cur);
            b.contours.push(c);
        }
        b.cur.clear();
        pen.x += f.glyph_hor_advance(g).unwrap_or(0) as f64 * scale;
    }
    // remove consecutive duplicates
    b.contours
        .into_iter()
        .map(|mut c| {
            c.dedup_by(|a, b| (*a - *b).length() < 1e-9);
            c
        })
        .filter(|c| c.len() > 2)
        .collect()
}

/// Bounding box of the text outlines (relative to the anchor).
pub fn bounds(text: &str, height: f64) -> Option<(DVec2, DVec2)> {
    let mut lo = DVec2::splat(f64::INFINITY);
    let mut hi = DVec2::splat(f64::NEG_INFINITY);
    for c in outlines(text, height) {
        for p in c {
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    (lo.x <= hi.x).then_some((lo, hi))
}

#[cfg(test)]
mod tests {
    #[test]
    fn letters_have_holes() {
        let c = super::outlines("AB", 10.0);
        // A: outer + 1 hole, B: outer + 2 holes
        assert_eq!(c.len(), 5);
        let (lo, hi) = super::bounds("AB", 10.0).unwrap();
        assert!((hi.y - 10.0).abs() < 0.3, "cap height {}", hi.y);
        assert!(lo.x >= -0.5 && hi.x > 10.0);
    }
}
