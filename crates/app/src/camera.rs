//! Orbit camera (Z-up), perspective or orthographic, with animated transitions.

use glam::{Mat3, Mat4, Quat, Vec2, Vec3};

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    pub target: Vec3,
    pub distance: f32,
    /// camera-to-world rotation; camera looks along local -Z, up is local +Y
    pub rot: Quat,
    pub ortho: bool,
    pub fov_y: f32,
}

#[derive(Clone, Copy, Debug)]
struct Anim {
    from: Camera,
    to: Camera,
    t: f32,
}

pub struct CameraCtl {
    pub cam: Camera,
    anim: Option<Anim>,
    /// scene radius used for clip planes
    pub scene_radius: f32,
}

impl Default for Camera {
    fn default() -> Self {
        let mut c = Camera { target: Vec3::ZERO, distance: 150.0, rot: Quat::IDENTITY, ortho: false, fov_y: 30f32.to_radians() };
        c.set_view(45f32.to_radians(), 30f32.to_radians());
        c
    }
}

impl Camera {
    /// Turntable orientation: yaw about Z, pitch looking down.
    pub fn set_view(&mut self, yaw: f32, pitch: f32) {
        self.rot = Self::view_rot(yaw, pitch);
    }
    pub fn view_rot(yaw: f32, pitch: f32) -> Quat {
        Quat::from_rotation_z(yaw) * Quat::from_rotation_x(-pitch) * Quat::from_rotation_x(std::f32::consts::FRAC_PI_2)
    }
    pub fn forward(&self) -> Vec3 {
        self.rot * Vec3::NEG_Z
    }
    pub fn up(&self) -> Vec3 {
        self.rot * Vec3::Y
    }
    pub fn right(&self) -> Vec3 {
        self.rot * Vec3::X
    }
    pub fn eye(&self) -> Vec3 {
        self.target - self.forward() * self.distance
    }
    pub fn view(&self) -> Mat4 {
        look_to_rh(self.eye(), self.forward(), self.up())
    }
    fn clip(&self, radius: f32) -> (f32, f32) {
        let r = radius.max(1.0);
        if self.ortho {
            // keep the eye far back so nothing is clipped
            let far = self.distance + r * 4.0 + 1000.0;
            (-(r * 4.0 + self.distance), far)
        } else {
            let near = (self.distance * 0.02).max(0.01).min(r * 0.5);
            let far = self.distance + r * 4.0 + 100.0;
            (near, far)
        }
    }
    pub fn half_height(&self) -> f32 {
        self.distance * (self.fov_y * 0.5).tan()
    }
    pub fn proj(&self, aspect: f32, radius: f32) -> Mat4 {
        let (n, f) = self.clip(radius);
        if self.ortho {
            let h = self.half_height();
            let w = h * aspect;
            orthographic_rh(-w, w, -h, h, n, f)
        } else {
            perspective_rh(self.fov_y, aspect, n, f)
        }
    }
    pub fn view_proj(&self, aspect: f32, radius: f32) -> Mat4 {
        self.proj(aspect, radius) * self.view()
    }
    /// World units per pixel at the target depth.
    pub fn px_size(&self, viewport_h: f32) -> f32 {
        2.0 * self.half_height() / viewport_h.max(1.0)
    }
}

/// A ray in world space.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub o: Vec3,
    pub d: Vec3,
}

impl Ray {
    pub fn at(&self, t: f32) -> Vec3 {
        self.o + self.d * t
    }
    /// Intersection with plane (point, normal); returns t.
    pub fn plane(&self, p: Vec3, n: Vec3) -> Option<f32> {
        let den = self.d.dot(n);
        if den.abs() < 1e-9 {
            return None;
        }
        Some((p - self.o).dot(n) / den)
    }
}

fn lerp_cam(a: &Camera, b: &Camera, t: f32) -> Camera {
    let s = t * t * (3.0 - 2.0 * t);
    Camera {
        target: a.target.lerp(b.target, s),
        distance: a.distance + (b.distance - a.distance) * s,
        rot: a.rot.slerp(b.rot, s),
        ortho: b.ortho,
        fov_y: b.fov_y,
    }
}

impl CameraCtl {
    pub fn new() -> Self {
        CameraCtl { cam: Camera::default(), anim: None, scene_radius: 100.0 }
    }

    /// Advance animations; returns true while animating.
    pub fn tick(&mut self, dt: f32) -> bool {
        if let Some(a) = &mut self.anim {
            a.t = (a.t + dt / 0.28).min(1.0);
            self.cam = lerp_cam(&a.from, &a.to, a.t);
            if a.t >= 1.0 {
                self.anim = None;
            }
            return true;
        }
        false
    }

    pub fn animate_to(&mut self, to: Camera) {
        self.anim = Some(Anim { from: self.cam, to, t: 0.0 });
    }

    pub fn stop_anim(&mut self) {
        if let Some(a) = self.anim.take() {
            self.cam = a.to;
        }
    }

    pub fn ray(&self, px: Vec2, size: Vec2) -> Ray {
        let ndc = Vec2::new(px.x / size.x * 2.0 - 1.0, 1.0 - px.y / size.y * 2.0);
        let inv = self.cam.view_proj(size.x / size.y.max(1.0), self.scene_radius).inverse();
        let a = inv.project_point3(ndc.extend(0.0));
        let b = inv.project_point3(ndc.extend(1.0));
        Ray { o: a, d: (b - a).normalize() }
    }

    /// World → pixel coordinates (None when behind the camera).
    pub fn project(&self, p: Vec3, size: Vec2) -> Option<Vec2> {
        let vp = self.cam.view_proj(size.x / size.y.max(1.0), self.scene_radius);
        let c = vp * p.extend(1.0);
        if c.w <= 1e-6 {
            return None;
        }
        let n = c.truncate() / c.w;
        Some(Vec2::new((n.x + 1.0) * 0.5 * size.x, (1.0 - n.y) * 0.5 * size.y))
    }

    /// Orbit about `pivot` by screen delta (pixels).
    pub fn orbit(&mut self, delta: Vec2, pivot: Vec3) {
        self.stop_anim();
        let c = &mut self.cam;
        let yaw = -delta.x * 0.008;
        let pitch = -delta.y * 0.008;
        let q_yaw = Quat::from_rotation_z(yaw);
        let right = c.right();
        let q_pitch = Quat::from_axis_angle(right, pitch);
        let q = q_yaw * q_pitch;
        let new_rot = (q * c.rot).normalize();
        // prevent flipping over the pole
        let up_after = new_rot * Vec3::Y;
        let (q, new_rot) = if up_after.z < 0.0 && (c.rot * Vec3::Y).z >= 0.0 {
            (q_yaw, (q_yaw * c.rot).normalize())
        } else {
            (q, new_rot)
        };
        let eye = c.eye();
        let new_eye = pivot + q * (eye - pivot);
        c.rot = new_rot;
        c.target = new_eye + c.forward() * c.distance;
    }

    pub fn pan(&mut self, delta: Vec2, viewport_h: f32) {
        self.stop_anim();
        let c = &mut self.cam;
        let s = c.px_size(viewport_h);
        c.target += (-c.right() * delta.x + c.up() * delta.y) * s;
    }

    /// Zoom by `factor` (<1 zooms in) towards the point under the cursor.
    pub fn zoom_at(&mut self, factor: f32, px: Vec2, size: Vec2) {
        self.stop_anim();
        let ray = self.ray(px, size);
        let fwd = self.cam.forward();
        let p = ray.plane(self.cam.target, fwd).map(|t| ray.at(t)).unwrap_or(self.cam.target);
        let f = factor.clamp(0.2, 5.0);
        let c = &mut self.cam;
        let nd = (c.distance * f).clamp(0.01, 1.0e6);
        let real = nd / c.distance;
        c.target = p + (c.target - p) * real;
        c.distance = nd;
    }

    /// Fit a bounding sphere in view (animated).
    pub fn fit(&mut self, center: Vec3, radius: f32, animate: bool) {
        let mut to = self.anim.map(|a| a.to).unwrap_or(self.cam);
        to.target = center;
        to.distance = (radius.max(0.5) / (to.fov_y * 0.5).sin()) * 1.1;
        if animate {
            self.animate_to(to);
        } else {
            self.cam = to;
        }
    }

    pub fn set_orientation(&mut self, rot: Quat) {
        let mut to = self.anim.map(|a| a.to).unwrap_or(self.cam);
        to.rot = rot;
        self.animate_to(to);
    }

    /// Look straight at a plane given its axes.
    pub fn look_at_plane(&mut self, origin: Vec3, x: Vec3, y: Vec3, n: Vec3, radius: Option<f32>) {
        let rot = Quat::from_mat3(&Mat3::from_cols(x, y, n)).normalize();
        let mut to = self.cam;
        to.rot = rot;
        // keep the target on the plane
        let t = to.target;
        to.target = t - n * (t - origin).dot(n);
        if let Some(r) = radius {
            to.distance = (r.max(5.0) / (to.fov_y * 0.5).sin()) * 1.2;
        }
        self.animate_to(to);
    }
}

/// Right-handed view matrix looking along `dir`.
pub fn look_to_rh(eye: Vec3, dir: Vec3, up: Vec3) -> Mat4 {
    let f = dir.normalize();
    let s = f.cross(up).normalize();
    let u = s.cross(f);
    Mat4::from_cols(
        glam::Vec4::new(s.x, u.x, -f.x, 0.0),
        glam::Vec4::new(s.y, u.y, -f.y, 0.0),
        glam::Vec4::new(s.z, u.z, -f.z, 0.0),
        glam::Vec4::new(-eye.dot(s), -eye.dot(u), eye.dot(f), 1.0),
    )
}

/// Right-handed perspective projection with depth in [0, 1].
pub fn perspective_rh(fov_y: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
    let h = 1.0 / (0.5 * fov_y).tan();
    let w = h / aspect;
    let r = far / (near - far);
    Mat4::from_cols(
        glam::Vec4::new(w, 0.0, 0.0, 0.0),
        glam::Vec4::new(0.0, h, 0.0, 0.0),
        glam::Vec4::new(0.0, 0.0, r, -1.0),
        glam::Vec4::new(0.0, 0.0, r * near, 0.0),
    )
}

/// Right-handed orthographic projection with depth in [0, 1].
pub fn orthographic_rh(l: f32, r: f32, b: f32, t: f32, near: f32, far: f32) -> Mat4 {
    let rml = 1.0 / (r - l);
    let tmb = 1.0 / (t - b);
    let fmn = 1.0 / (near - far);
    Mat4::from_cols(
        glam::Vec4::new(2.0 * rml, 0.0, 0.0, 0.0),
        glam::Vec4::new(0.0, 2.0 * tmb, 0.0, 0.0),
        glam::Vec4::new(0.0, 0.0, fmn, 0.0),
        glam::Vec4::new(-(r + l) * rml, -(t + b) * tmb, near * fmn, 1.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projection_roundtrip() {
        let mut ctl = CameraCtl::new();
        ctl.cam.target = Vec3::new(1.0, 2.0, 3.0);
        let size = glam::Vec2::new(800.0, 600.0);
        for ortho in [false, true] {
            ctl.cam.ortho = ortho;
            let p = Vec3::new(5.0, -3.0, 7.0);
            let s = ctl.project(p, size).unwrap();
            let r = ctl.ray(s, size);
            let d = (p - r.o).cross(r.d).length();
            assert!(d < 1e-2, "ray misses point by {d}");
            // target projects to the centre
            let c = ctl.project(ctl.cam.target, size).unwrap();
            assert!((c - size * 0.5).length() < 1e-2);
        }
    }
}
