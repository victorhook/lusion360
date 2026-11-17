struct Globals {
    view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    light: vec4<f32>,
    // x,y = viewport size in px; z = 1 if output needs sRGB encoding; w = ortho
    viewport: vec4<f32>,
    fwd: vec4<f32>,
    bg_top: vec4<f32>,
    bg_bottom: vec4<f32>,
    // section plane: dot(clip.xyz, p) > clip.w is cut away
    clip: vec4<f32>,
    // x = section active
    clip_on: vec4<f32>,
};

struct Draw {
    color: vec4<f32>,
    // x = line width scale, y = depth bias, z = shading mix (0 = flat color), w = 1 if the section plane applies
    params: vec4<f32>,
};

@group(0) @binding(0) var<uniform> g: Globals;
@group(1) @binding(0) var<uniform> d: Draw;

fn to_output(c: vec3<f32>) -> vec3<f32> {
    if (g.viewport.z > 0.5) {
        // linear -> sRGB
        let lo = c * 12.92;
        let hi = 1.055 * pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.4)) - 0.055;
        return select(hi, lo, c <= vec3<f32>(0.0031308));
    }
    return c;
}

// Move a point slightly toward the camera (scale independent depth bias).
fn bias_pos(p: vec3<f32>, b: f32) -> vec3<f32> {
    if (g.viewport.w > 0.5) {
        return p - g.fwd.xyz * b * g.fwd.w;
    }
    return p + (g.eye.xyz - p) * b;
}

fn sectioned() -> bool {
    return g.clip_on.x > 0.5 && d.params.w > 0.5;
}

fn clipped(p: vec3<f32>) -> bool {
    return sectioned() && dot(g.clip.xyz, p) > g.clip.w;
}

fn srgb_to_linear(c: vec3<f32>) -> vec3<f32> {
    let lo = c / 12.92;
    let hi = pow((c + 0.055) / 1.055, vec3<f32>(2.4));
    return select(hi, lo, c <= vec3<f32>(0.04045));
}

// ---------------------------------------------------------------- background

struct BgOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) t: f32,
};

@vertex
fn vs_bg(@builtin(vertex_index) i: u32) -> BgOut {
    var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
    var o: BgOut;
    o.pos = vec4<f32>(p[i], 0.9999, 1.0);
    o.t = p[i].y * 0.5 + 0.5;
    return o;
}

@fragment
fn fs_bg(i: BgOut) -> @location(0) vec4<f32> {
    let c = mix(g.bg_bottom.rgb, g.bg_top.rgb, clamp(i.t, 0.0, 1.0));
    return vec4<f32>(to_output(srgb_to_linear(c)), 1.0);
}

// ---------------------------------------------------------------- shaded meshes

struct MeshOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) wpos: vec3<f32>,
    @location(1) n: vec3<f32>,
};

@vertex
fn vs_mesh(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>) -> MeshOut {
    var o: MeshOut;
    let c = g.view_proj * vec4<f32>(bias_pos(p, d.params.y), 1.0);
    o.pos = c;
    o.wpos = p;
    o.n = n;
    return o;
}

@fragment
fn fs_mesh(i: MeshOut, @builtin(front_facing) front: bool) -> @location(0) vec4<f32> {
    if (clipped(i.wpos)) {
        discard;
    }
    if (sectioned() && !front) {
        // looking into a cut solid: draw the cap (flat, hatched)
        let base = srgb_to_linear(d.color.rgb);
        let lit = 0.55 + 0.35 * abs(dot(normalize(g.clip.xyz), normalize(g.light.xyz)));
        let hatch = fract((i.pos.x + i.pos.y) / 9.0) < 0.5;
        let c = base * lit * select(0.78, 0.92, hatch);
        return vec4<f32>(to_output(c), d.color.a);
    }
    var n = normalize(i.n);
    var v = normalize(g.eye.xyz - i.wpos);
    if (g.viewport.w > 0.5) {
        v = -g.fwd.xyz;
    }
    if (dot(n, v) < 0.0) {
        n = -n;
    }
    let base = srgb_to_linear(d.color.rgb);
    let l = normalize(g.light.xyz);
    let key = max(dot(n, l), 0.0);
    let hemi = mix(vec3<f32>(0.32, 0.30, 0.28), vec3<f32>(0.62, 0.66, 0.72), n.z * 0.5 + 0.5);
    let h = normalize(l + v);
    let spec = pow(max(dot(n, h), 0.0), 48.0) * 0.22;
    let fres = pow(1.0 - max(dot(n, v), 0.0), 3.0) * 0.12;
    var c = base * (hemi * 0.55 + vec3<f32>(key * 0.75)) + vec3<f32>(spec + fres);
    c = mix(base, c, d.params.z);
    return vec4<f32>(to_output(c), d.color.a);
}

// ---------------------------------------------------------------- silhouette (inverted hull)

@vertex
fn vs_outline(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>) -> FillOut {
    var o: FillOut;
    let c = g.view_proj * vec4<f32>(p, 1.0);
    let cn = g.view_proj * vec4<f32>(p + normalize(n) * 0.001 * g.fwd.w, 1.0);
    var dir = cn.xy / cn.w - c.xy / c.w;
    let l = length(dir * g.viewport.xy);
    if (l > 1e-6) {
        dir = dir * g.viewport.xy / l;
    } else {
        dir = vec2<f32>(0.0, 0.0);
    }
    let off = dir * d.params.x / g.viewport.xy * 2.0;
    o.pos = vec4<f32>(c.xy + off * c.w, c.z, c.w);
    o.color = d.color;
    o.wpos = p;
    return o;
}

// ---------------------------------------------------------------- unlit triangles (fills)

struct FillOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) wpos: vec3<f32>,
};

@vertex
fn vs_fill(@location(0) p: vec3<f32>, @location(1) color: vec4<f32>) -> FillOut {
    var o: FillOut;
    o.pos = g.view_proj * vec4<f32>(bias_pos(p, d.params.y), 1.0);
    o.color = color;
    o.wpos = p;
    return o;
}

@fragment
fn fs_fill(i: FillOut) -> @location(0) vec4<f32> {
    if (clipped(i.wpos)) {
        discard;
    }
    return vec4<f32>(to_output(srgb_to_linear(i.color.rgb)), i.color.a);
}

// ---------------------------------------------------------------- thick lines

struct LineOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) dist: f32,
    @location(2) width: f32,
    @location(3) wpos: vec3<f32>,
};

@vertex
fn vs_line(
    @builtin(vertex_index) vi: u32,
    @location(0) p0: vec3<f32>,
    @location(1) p1: vec3<f32>,
    @location(2) color: vec4<f32>,
    @location(3) width: f32,
) -> LineOut {
    var c0 = g.view_proj * vec4<f32>(bias_pos(p0, d.params.y), 1.0);
    var c1 = g.view_proj * vec4<f32>(bias_pos(p1, d.params.y), 1.0);
    // clip against the near plane (w > eps)
    let eps = 1e-5;
    if (c0.w < eps && c1.w < eps) {
        var o: LineOut;
        o.pos = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        o.wpos = p0;
        return o;
    }
    if (c0.w < eps) {
        let t = (eps - c0.w) / (c1.w - c0.w);
        c0 = mix(c0, c1, t);
    } else if (c1.w < eps) {
        let t = (eps - c1.w) / (c0.w - c1.w);
        c1 = mix(c1, c0, t);
    }
    let vp = g.viewport.xy;
    let s0 = c0.xy / c0.w * 0.5 * vp;
    let s1 = c1.xy / c1.w * 0.5 * vp;
    var dir = s1 - s0;
    let len = length(dir);
    if (len < 1e-6) {
        dir = vec2<f32>(1.0, 0.0);
    } else {
        dir = dir / len;
    }
    let nrm = vec2<f32>(-dir.y, dir.x);
    let w = max(width * d.params.x, 0.5) * 0.5 + 0.5;
    // quad corners: (end, side)
    var ends = array<f32, 6>(0.0, 1.0, 1.0, 0.0, 1.0, 0.0);
    var sides = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let e = ends[vi];
    let sd = sides[vi];
    var c = c0;
    var s = s0;
    var ext = -dir;
    if (e > 0.5) {
        c = c1;
        s = s1;
        ext = dir;
    }
    let off = nrm * sd * w + ext * w * 0.5;
    let sp = (s + off) / (0.5 * vp);
    var o: LineOut;
    o.pos = vec4<f32>(sp * c.w, c.z, c.w);
    o.color = color * d.color;
    o.dist = sd * w;
    o.width = w;
    o.wpos = select(p0, p1, e > 0.5);
    return o;
}

@fragment
fn fs_line(i: LineOut) -> @location(0) vec4<f32> {
    if (clipped(i.wpos)) {
        discard;
    }
    let a = clamp(i.width - abs(i.dist) + 0.5, 0.0, 1.0);
    return vec4<f32>(to_output(srgb_to_linear(i.color.rgb)), i.color.a * a);
}

// ---------------------------------------------------------------- points

struct PointOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
    @location(2) size: f32,
};

@vertex
fn vs_point(
    @builtin(vertex_index) vi: u32,
    @location(0) p: vec3<f32>,
    @location(1) color: vec4<f32>,
    @location(2) size: f32,
) -> PointOut {
    var corners = array<vec2<f32>, 6>(
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0),
        vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, 1.0));
    let uv = corners[vi];
    let c = g.view_proj * vec4<f32>(bias_pos(p, d.params.y), 1.0);
    let r = size * 0.5 + 1.0;
    let off = uv * r / (0.5 * g.viewport.xy) * c.w;
    var o: PointOut;
    o.pos = vec4<f32>(c.xy + off, c.z, c.w);
    o.color = color;
    o.uv = uv * r;
    o.size = size * 0.5;
    return o;
}

@fragment
fn fs_point(i: PointOut) -> @location(0) vec4<f32> {
    let dist = length(i.uv);
    let a = clamp(i.size - dist + 0.5, 0.0, 1.0);
    if (a <= 0.0) {
        discard;
    }
    // dark rim for contrast
    let rim = clamp(dist - (i.size - 1.4), 0.0, 1.0);
    let col = mix(i.color.rgb, i.color.rgb * 0.35, rim);
    return vec4<f32>(to_output(srgb_to_linear(col)), i.color.a * a);
}
