//! `wrap`: closed profiles and sketch text laid on a curved face by length along the surface (docs/proposals/wrap-feature.md,
//! phase 1: a cylindrical face, outside or in a bore).
//!
//! ```json
//! {"type":"wrap","id":"F9","sketch":"F8","profiles":["t1"],"face":"F1.S.e3","op":"cut","depth":0.4,
//!  "method":"auto","alongAxis":0,"around":0,"rotation":0,"taper":0,"flip":false,"direction":null,"target":null,"tolerance":0.001}
//! ```
//!
//! * `sketch`: a sketch feature id; its closed regions (text outlines, any closed loop) are wrapped. `profiles` (optional) picks
//!   regions by the ids of their outer boundary entities, a text entity id picks all its glyphs (as `regions` of an extrude).
//!   Open curves of the sketch are ignored.
//! * `face`: one cylindrical face name (outside a boss, or the wall of a bore). `op`: `add` (raise) or `cut` (engrave), `depth` mm
//!   along the face normals (> 0).
//! * `method`: `auto` (default) and `wrap` develop the cylinder; `project` is not built yet (`wrapFaceUnsupported`).
//! * Development map. The sketch is read in the plane's in-plane directions `e_z` (the cylinder axis, projected into the plane; the
//!   sign of the axis is the one whose largest component is positive) and `e_s = e_z x normal`, about the sketch origin. A sketch
//!   plane perpendicular to the axis is `wrapPlane`. The profile lands at the point of the face nearest the plane (a plane through
//!   the axis: the side its normal points to), arc lengths on the face are the lengths of the sketch (isometric: widths are kept),
//!   and it reads correctly seen from the side the face normal points to (from inside a bore for a bore). `rotation` (degrees,
//!   counter-clockwise in the sketch) turns the selected profiles about their middle first, `flip` mirrors them along the
//!   circumference, `alongAxis` (mm) moves them along the axis (positive = towards the axis direction with the largest positive
//!   component), `around` (degrees about the axis) moves them along the reading direction (positive = towards the reader's right).
//! * Walls stand square to the face (radial); `taper` (degrees, -45..45, default 0) widens (positive) or narrows the profile as it
//!   rises from the face; 0 keeps the walls on the face normals. Edges parallel to the axis or round the axis (untapered) are exact
//!   planar walls, every other edge is a B-spline rail pair within `tolerance` (mm, default 0.001).
//! * Faces: walls `<id>.S.<tag>`, the cap that stays in the result (a cut's floor, an add's top) `<id>.C0`. Marker
//!   `markers[] = [id, {profiles, turns, deviation, method, surface}]`.
//! * Errors: `wrapNoInput`, `missingSketch`, `nothingToCut`, `missingReference`, `wrapFaceUnsupported {faceKind, hint}`, `wrapPlane`,
//!   `wrapProfileTooWide {profile, extent, circumference, hint}`, `wrapProfileOffFace {profile, at, hint}`, `wrapDepthTooLarge
//!   {depth, max, hint}`, `wrapTaperTooLarge {profile, maxTaper, hint}`, `wrapToolInvalid {detail}`, `invalidValue`, and the boolean
//!   errors (the body is left as it was). Warnings: `wrapApproximated {maxDeviation, tolerance}`, `wrapCutThrough {profile, wall}`.

use super::extrude::{self, Combine, Tool};
use super::wrap_tool::{self, DevFn, EdgeIn, Kind, LoopIn, RegionIn};
use super::Brep;
use crate::regen::{codes, FeatureError, RegenError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::Vec2;
use ck_json::Json;
use ck_topo::make::{ProfileLoop, ProfileSeg};
use std::f64::consts::{PI, TAU};

pub const WRAP_NO_INPUT: &str = "wrapNoInput";
pub const WRAP_FACE_UNSUPPORTED: &str = "wrapFaceUnsupported";
pub const WRAP_PLANE: &str = "wrapPlane";
pub const WRAP_PROFILE_TOO_WIDE: &str = "wrapProfileTooWide";
pub const WRAP_PROFILE_OFF_FACE: &str = "wrapProfileOffFace";
pub const WRAP_PROFILES_OVERLAP: &str = "wrapProfilesOverlap";
pub const WRAP_DEPTH_TOO_LARGE: &str = "wrapDepthTooLarge";
pub const WRAP_TAPER_TOO_LARGE: &str = "wrapTaperTooLarge";
pub const WRAP_APPROXIMATED: &str = "wrapApproximated";
pub const WRAP_CURVE_TOO_LONG: &str = "wrapCurveTooLong";
pub const WRAP_CUT_THROUGH: &str = "wrapCutThrough";

fn bad(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}
fn s(x: &str) -> Json {
    Json::Str(x.into())
}
fn n(x: f64) -> Json {
    Json::Num(x)
}
fn v3(p: Vec3) -> Json {
    Json::Arr(vec![n(p.x), n(p.y), n(p.z)])
}

/// The sketch to development map: sketch point -> `(X, Z)` (arc length along increasing angle, axial position).
#[derive(Clone)]
struct Dev {
    /// Sketch frame axes seen in `(e_s, e_z)`: `u = x fx.0 + y fy.0`, `w = x fx.1 + y fy.1`.
    fx: (f64, f64),
    fy: (f64, f64),
    centre: [f64; 2],
    cs: f64,
    sn: f64,
    flip: bool,
    /// +1 outside, -1 in a bore: the reading direction is `sgn` along the angle.
    sgn: f64,
    around_arc: f64,
    z_off: f64,
}

impl Dev {
    fn raw(&self, p: Vec2) -> [f64; 2] {
        [p.x * self.fx.0 + p.y * self.fy.0, p.x * self.fx.1 + p.y * self.fy.1]
    }
    fn map(&self, p: Vec2) -> [f64; 2] {
        let [u, w] = self.raw(p);
        let (du, dw) = (u - self.centre[0], w - self.centre[1]);
        let (ru, rw) = (self.centre[0] + du * self.cs - dw * self.sn, self.centre[1] + du * self.sn + dw * self.cs);
        let uu = if self.flip { 2.0 * self.centre[0] - ru } else { ru };
        [self.sgn * (uu + self.around_arc), self.z_off + rw]
    }
}

/// One edge of a loop in travel order (canonical: outer counter-clockwise, holes clockwise in `(X, Z)`).
#[derive(Clone)]
struct Path {
    seg: ProfileSeg,
    dev: Dev,
    rev: bool,
    l0: f64,
    l1: f64,
}

impl Path {
    fn at(&self, lam: f64) -> [f64; 2] {
        let mut l = self.l0 + lam * (self.l1 - self.l0);
        if self.rev {
            l = 1.0 - l;
        }
        let t = self.seg.t_start() + l * (self.seg.t_end() - self.seg.t_start());
        self.dev.map(self.seg.curve.eval(t))
    }
    /// Unit normal on the right of the direction of travel.
    fn normal(&self, lam: f64) -> [f64; 2] {
        let h = 1e-5;
        let (a, b) = (self.at((lam - h).max(0.0)), self.at((lam + h).min(1.0)));
        let (tx, tz) = (b[0] - a[0], b[1] - a[1]);
        let l = tx.hypot(tz).max(1e-300);
        [tz / l, -tx / l]
    }
}

type PLoop = Vec<Path>;

struct PRegion {
    outer: PLoop,
    holes: Vec<PLoop>,
    /// Id used in errors: the tag of the first outer edge.
    tag: String,
}

fn area(poly: &[[f64; 2]]) -> f64 {
    let mut a = 0.0;
    for i in 0..poly.len() {
        let (p, q) = (poly[i], poly[(i + 1) % poly.len()]);
        a += p[0] * q[1] - q[0] * p[1];
    }
    0.5 * a
}

fn poly_of(l: &PLoop, per: usize) -> Vec<[f64; 2]> {
    let mut out = vec![];
    for p in l {
        for k in 0..per {
            out.push(p.at(k as f64 / per as f64));
        }
    }
    out
}

/// A loop in canonical order, degenerate edges dropped, a single edge split in two.
fn loop_paths(l: &ProfileLoop, dev: &Dev, outer: bool, max_arc: f64) -> Result<PLoop, FeatureError> {
    let mut paths: PLoop = l.segs.iter().map(|sg| Path { seg: sg.clone(), dev: dev.clone(), rev: false, l0: 0.0, l1: 1.0 }).collect();
    let a = area(&poly_of(&paths, 16));
    if (a > 0.0) != outer {
        paths.reverse();
        for p in &mut paths {
            p.rev = !p.rev;
        }
    }
    paths.retain(|p| {
        let pl: Vec<[f64; 2]> = (0..=8).map(|k| p.at(k as f64 / 8.0)).collect();
        pl.windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).sum::<f64>() > 1e-9
    });
    if paths.len() == 1 {
        let p = paths.remove(0);
        paths.push(Path { l0: 0.0, l1: 0.5, ..p.clone() });
        paths.push(Path { l0: 0.5, l1: 1.0, ..p });
    }
    if paths.len() < 2 {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    // A circular arc wall (a line of constant Z) meets the host cylinder in an arc of a circle; the boolean does not close one of
    // more than about half a turn, so a long one is built as several exact walls of at most 0.4 turn each.
    let mut out: PLoop = vec![];
    for p in paths {
        let (a, b) = (p.at(0.0), p.at(1.0));
        let long_circ = matches!(p.seg.curve, Curve2::Line { .. }) && (a[1] - b[1]).abs() <= 1e-9 * (1.0 + (a[0] - b[0]).abs());
        let pieces = if long_circ { ((a[0] - b[0]).abs() / max_arc).ceil().max(1.0) as usize } else { 1 };
        for k in 0..pieces {
            let (l0, l1) = (p.l0 + (p.l1 - p.l0) * k as f64 / pieces as f64, p.l0 + (p.l1 - p.l0) * (k + 1) as f64 / pieces as f64);
            out.push(Path { l0, l1, ..p.clone() });
        }
    }
    Ok(out)
}

/// The edges of a loop for the tool builder: rails at the signed heights above the face whose profile offsets are `d_b` and `d_t`
/// (`h tan(taper)`); a corner offsets along the miter vector so neighbouring edges meet.
fn edges_of(l: &PLoop, d_b: f64, d_t: f64, exact: bool) -> Vec<EdgeIn> {
    let m = l.len();
    let mut out = vec![];
    for k in 0..m {
        let p = l[k].clone();
        let prev = &l[(k + m - 1) % m];
        let (n_in, n_out) = (prev.normal(1.0), p.normal(0.0));
        let (n1, n0) = (p.normal(1.0), n_out);
        let next_n = l[(k + 1) % m].normal(0.0);
        let miter = |a: [f64; 2], b: [f64; 2]| -> [f64; 2] {
            let d = 1.0 + a[0] * b[0] + a[1] * b[1];
            let sc = 1.0 / d.max(0.3);
            [(a[0] + b[0]) * sc, (a[1] + b[1]) * sc]
        };
        let (m0, m1) = (miter(n_in, n_out), miter(n1, next_n));
        let kind = if exact {
            match p.seg.curve {
                Curve2::Line { .. } => {
                    let (a, b) = (p.at(0.0), p.at(1.0));
                    let (dx, dz) = ((a[0] - b[0]).abs(), (a[1] - b[1]).abs());
                    if dx <= 1e-9 * (1.0 + dz) {
                        Kind::Axial
                    } else if dz <= 1e-9 * (1.0 + dx) {
                        Kind::Circ
                    } else {
                        Kind::Fit
                    }
                }
                _ => Kind::Fit,
            }
        } else {
            Kind::Fit
        };
        let tag = p.seg.tag.clone();
        let tapered = d_b != 0.0 || d_t != 0.0;
        let f: DevFn = if !tapered {
            Box::new(move |lam| {
                let q = p.at(lam);
                (q, q)
            })
        } else {
            Box::new(move |lam| {
                let q = p.at(lam);
                let nn = p.normal(lam);
                let w0 = (1.0 - lam / 0.1).max(0.0).powi(2);
                let w1 = (1.0 - (1.0 - lam) / 0.1).max(0.0).powi(2);
                let mv = [nn[0] + w0 * (m0[0] - n_out[0]) + w1 * (m1[0] - n1[0]), nn[1] + w0 * (m0[1] - n_out[1]) + w1 * (m1[1] - n1[1])];
                ([q[0] + d_b * mv[0], q[1] + d_b * mv[1]], [q[0] + d_t * mv[0], q[1] + d_t * mv[1]])
            })
        };
        let _ = n0;
        out.push(EdgeIn { tag, kind, f });
    }
    out
}

fn segs_cross(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let o = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (d1, d2, d3, d4) = (o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

/// Does any pair of non-neighbouring segments of the polygons cross (the same polygon when `same`)?
fn polys_cross(p: &[[f64; 2]], q: &[[f64; 2]], same: bool) -> bool {
    let bb = |a: [f64; 2], b: [f64; 2]| (a[0].min(b[0]), a[0].max(b[0]), a[1].min(b[1]), a[1].max(b[1]));
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        let ba = bb(a, b);
        for j in 0..q.len() {
            if same && (j <= i + 1 || (i == 0 && j == q.len() - 1)) {
                continue;
            }
            let (c, d) = (q[j], q[(j + 1) % q.len()]);
            let bc = bb(c, d);
            if ba.1 < bc.0 || bc.1 < ba.0 || ba.3 < bc.2 || bc.3 < ba.2 {
                continue;
            }
            if segs_cross(a, b, c, d) {
                return true;
            }
        }
    }
    false
}

/// The first profile the tapered rails ruin (self-crossing, a flipped loop, loops crossing each other), by its tag.
fn taper_fault(regs: &[PRegion], d_b: f64, d_t: f64) -> Option<String> {
    for r in regs {
        let mut polys: Vec<(Vec<[f64; 2]>, f64)> = vec![];
        for (i, l) in std::iter::once(&r.outer).chain(r.holes.iter()).enumerate() {
            let want = if i == 0 { 1.0 } else { -1.0 };
            for d in [d_b, d_t] {
                let es = edges_of(l, d_b, d_t, false);
                let pl: Vec<[f64; 2]> = es
                    .iter()
                    .flat_map(|e| (0..12).map(|k| {
                        let (b, t) = (e.f)(k as f64 / 12.0);
                        if d == d_b { b } else { t }
                    }).collect::<Vec<_>>())
                    .collect();
                polys.push((pl, want));
            }
        }
        for (pl, want) in &polys {
            let a = area(pl);
            if a * want <= 1e-12 || polys_cross(pl, pl, true) {
                return Some(r.tag.clone());
            }
        }
        // rails at one height: loops of a region against each other
        for rail in 0..2 {
            let at_rail: Vec<&Vec<[f64; 2]>> = polys.iter().skip(rail).step_by(2).map(|(p, _)| p).collect();
            for i in 0..at_rail.len() {
                for j in i + 1..at_rail.len() {
                    if polys_cross(at_rail[i], at_rail[j], false) {
                        return Some(r.tag.clone());
                    }
                }
            }
        }
    }
    None
}

fn point_in_poly(p: [f64; 2], poly: &[[f64; 2]]) -> bool {
    let mut inside = false;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

struct Params {
    cut: bool,
    depth: f64,
    along: f64,
    around: f64,
    rotation: f64,
    taper: f64,
    flip: bool,
    tol: f64,
}

fn parse(state: &RegenState, f: &Feature) -> Result<Params, FeatureError> {
    let num = |k: &str, def: f64| -> Result<f64, FeatureError> {
        let v = state.num(f, k, def)?;
        if !v.is_finite() {
            return Err(bad(k, "must be a finite number"));
        }
        Ok(v)
    };
    let cut = match f.raw.get_str("op") {
        None | Some("add") => false,
        Some("cut") => true,
        Some(_) => return Err(bad("op", "add (raise) or cut (engrave)")),
    };
    match f.raw.get("method") {
        None | Some(Json::Null) => {}
        Some(Json::Str(m)) if matches!(m.as_str(), "auto" | "wrap" | "project") => {}
        Some(_) => return Err(bad("method", "auto, wrap or project")),
    }
    let tol = num("tolerance", 0.001)?;
    if !(tol > 0.0) {
        return Err(bad("tolerance", "must be positive"));
    }
    let taper = num("taper", 0.0)?;
    if !(-45.0..=45.0).contains(&taper) {
        return Err(bad("taper", "between -45 and 45 degrees"));
    }
    let depth = state.size(f, "depth", 0.0)?;
    if !(depth > 0.0) {
        return Err(bad("depth", "must be positive"));
    }
    let flip = matches!(f.raw.get("flip"), Some(Json::Bool(true)));
    Ok(Params { cut, depth, along: num("alongAxis", 0.0)?, around: num("around", 0.0)?, rotation: num("rotation", 0.0)?, taper, flip, tol })
}

fn profile_selection(sk: &crate::regen::SketchResult, f: &Feature) -> Result<ck_topo::make::Profile, FeatureError> {
    match extrude::selected_profile_by(sk, f, "profiles") {
        Err(e) if e.code == codes::NO_CLOSED_PROFILE => Err(no_input("the sketch has no closed region (open curves belong to projectCurve)")),
        Err(e) if e.code == extrude::REGION_NONE => Err(no_input("profiles names no closed region of the sketch")),
        r => r,
    }
}

fn no_input(hint: &str) -> FeatureError {
    FeatureError::new(WRAP_NO_INPUT).with("hint", s(hint))
}

fn warn(state: &mut RegenState, f: &Feature, e: FeatureError) {
    state.result.warnings.push(RegenError { feature: f.id.clone(), error: e });
}

/// The reading frame of a sketch on a cylinder (shared by `wrap` and `projectCurve method:"wrap"`): the axis `a` in its canonical
/// sign, the side the material is on (`sgn` +1 outside, -1 in a bore), the point `ex` of the face nearest the plane and `ey = a x ex`
/// (angles are measured from `ex`), and the sketch axes seen in `(e_s, e_z)`.
pub(super) struct Setup {
    pub a: Vec3,
    pub sgn: f64,
    pub outside: bool,
    pub o_ax: Vec3,
    pub o_sk: Vec3,
    pub ex: Vec3,
    pub ey: Vec3,
    pub fx: (f64, f64),
    pub fy: (f64, f64),
}

pub(super) fn cyl_setup(before: &ck_topo::body::Body, fid: ck_topo::body::FaceId, cframe: Frame, sk: &crate::regen::SketchResult) -> Result<Setup, FeatureError> {
    let mut a = cframe.z.normalize();
    let big = [a.x, a.y, a.z].iter().cloned().fold(0.0f64, |m, c| if c.abs() > m.abs() { c } else { m });
    if big < 0.0 {
        a = -a;
    }
    let outside = before.outward_normal(fid, 0.0, 0.0).dot(cframe.x) > 0.0;
    let sgn = if outside { 1.0 } else { -1.0 };
    let (o_ax, nrm, o_sk) = (cframe.o, sk.frame.z.normalize(), sk.frame.o);
    let an = a.dot(nrm);
    if an.abs() > 1.0 - 1e-9 {
        return Err(FeatureError::new(WRAP_PLANE)
            .with("detail", s("the sketch plane is perpendicular to the axis of the face"))
            .with("hint", s("sketch on a plane parallel to the axis (or through it)")));
    }
    let ez = (a - nrm * an).normalize();
    let es = ez.cross(nrm);
    let nperp = (nrm - a * an).normalize();
    let dist = nrm.dot(o_ax - o_sk);
    let rho0 = if dist > 1e-9 { -nperp } else { nperp };
    let ex = rho0;
    let ey = a.cross(ex);
    let fx = (sk.frame.x.dot(es), sk.frame.x.dot(ez));
    let fy = (sk.frame.y.dot(es), sk.frame.y.dot(ez));
    Ok(Setup { a, sgn, outside, o_ax, o_sk, ex, ey, fx, fy })
}

pub fn exec_wrap(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let p = parse(state, f)?;
    // inputs
    if !matches!(f.raw.get("sketch"), Some(Json::Str(_))) {
        return Err(no_input("give the sketch feature whose closed profiles or text are wrapped"));
    }
    let sk = state.sketch(f.raw.get_str("sketch").unwrap_or("")).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?.clone();
    extrude::finite_points(&sk)?;
    let profile = profile_selection(&sk, f)?;
    let Some(before) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let face_name = match f.raw.get("face") {
        Some(Json::Str(n)) => n.clone(),
        Some(o @ Json::Obj(_)) => o.get_str("face").unwrap_or("").to_string(),
        _ => return Err(FeatureError::new(codes::MISSING_REFERENCE).with("field", s("face"))),
    };
    let (fid, _) = extrude::resolve_face(&before, &face_name, None)
        .ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE).with("field", s("face")).with("name", s(&face_name)))?;
    let face = before.face(fid);
    let unsupported = |hint: &str| FeatureError::new(WRAP_FACE_UNSUPPORTED).with("faceKind", s(ck_topo::body::wire_kind(&face.surface))).with("hint", s(hint));
    let (cframe, r) = match &face.surface {
        Surface::Cylinder { frame, r } => (*frame, *r),
        Surface::Plane { .. } => return Err(unsupported("a plane needs no wrap: use extrude")),
        Surface::Cone { .. } => return Err(unsupported("cones are not built yet: only cylinders are wrapped")),
        Surface::Sphere { .. } | Surface::Torus { .. } if f.raw.get_str("method") == Some("wrap") => {
            return Err(bad("method", "a sphere or torus cannot be developed").with("hint", s("use method project (not built yet)")))
        }
        _ => return Err(unsupported("only cylindrical faces are wrapped")),
    };
    if f.raw.get_str("method") == Some("project") {
        return Err(unsupported("method project is not built yet: only cylinders are wrapped (method wrap)"));
    }
    let Setup { a, sgn, outside, o_ax, o_sk, ex, ey, fx, fy } = cyl_setup(&before, fid, cframe, &sk)?;
    // middle of the selected outlines, in the sketch's (u, w)
    let mut dev = Dev { fx, fy, centre: [0.0, 0.0], cs: 1.0, sn: 0.0, flip: p.flip, sgn, around_arc: p.around.to_radians() * r, z_off: (o_sk - o_ax).dot(a) + p.along };
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for reg in &profile.regions {
        for sg in &reg.outer.segs {
            for k in 0..=12 {
                let t = sg.t0 + (sg.t1 - sg.t0) * k as f64 / 12.0;
                let q = dev.raw(sg.curve.eval(t));
                for i in 0..2 {
                    lo[i] = lo[i].min(q[i]);
                    hi[i] = hi[i].max(q[i]);
                }
            }
        }
    }
    dev.centre = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let ang = p.rotation.to_radians();
    dev.cs = ang.cos();
    dev.sn = ang.sin();
    // loops in canonical order
    let mut regs: Vec<PRegion> = vec![];
    for reg in &profile.regions {
        let outer = loop_paths(&reg.outer, &dev, true, 0.4 * TAU * r)?;
        let holes = reg.holes.iter().map(|h| loop_paths(h, &dev, false, 0.4 * TAU * r)).collect::<Result<Vec<_>, _>>()?;
        let tag = outer[0].seg.tag.clone();
        regs.push(PRegion { outer, holes, tag });
    }
    // heights above the face of the two rails; the tool crosses the face by `eps` on its free side
    let eps = (0.05 * p.depth).max(5e-6);
    let (t_b, t_t) = if p.cut { (-p.depth, eps) } else { (-eps, p.depth) };
    let (r_base, r_top) = (r + sgn * t_b, r + sgn * t_t);
    let r_low = r_base.min(r_top);
    if r_low < 0.05 * r || (!p.cut && !outside && p.depth >= r) {
        let max = if !p.cut && !outside { r } else { 0.95 * r };
        return Err(FeatureError::new(WRAP_DEPTH_TOO_LARGE)
            .with("depth", n(p.depth))
            .with("max", n(max))
            .with("hint", s("the tool would reach the axis of the cylinder: use a smaller depth")));
    }
    let tan = p.taper.to_radians().tan();
    let (d_b, d_t) = (t_b * tan, t_t * tan);
    // one-turn test on the outlines (and the rails when tapered)
    let circ = TAU * r;
    let ext = |polys: &[Vec<[f64; 2]>]| -> (f64, f64) {
        let xs = polys.iter().flatten().map(|q| q[0]);
        (xs.clone().fold(f64::INFINITY, f64::min), xs.fold(f64::NEG_INFINITY, f64::max))
    };
    let tapered = p.taper != 0.0;
    let mut all: Vec<Vec<[f64; 2]>> = vec![];
    let (mut widest, mut widest_tag) = (0.0f64, String::new());
    for rg in &regs {
        let mut polys = vec![poly_of(&rg.outer, 24)];
        if tapered {
            let es_ = edges_of(&rg.outer, d_b, d_t, false);
            for rail in 0..2 {
                polys.push(es_.iter().flat_map(|e| (0..12).map(|k| { let (b, t) = (e.f)(k as f64 / 12.0); if rail == 0 { b } else { t } }).collect::<Vec<_>>()).collect());
            }
        }
        let (a0, a1) = ext(&polys);
        if a1 - a0 > widest {
            widest = a1 - a0;
            widest_tag = rg.tag.clone();
        }
        if a1 - a0 >= circ * (1.0 - 1e-6) {
            return Err(too_wide(&rg.tag, a1 - a0, circ));
        }
        all.extend(polys);
    }
    let (x0, x1) = ext(&all);
    if x1 - x0 >= circ * (1.0 - 1e-6) {
        return Err(too_wide(&regs[0].tag, x1 - x0, circ));
    }
    let _ = widest_tag;
    // two profiles that overlap on the face would make a tool of overlapping lumps
    for i in 0..regs.len() {
        let pi = poly_of(&regs[i].outer, 8);
        let hi_: Vec<Vec<[f64; 2]>> = regs[i].holes.iter().map(|h| poly_of(h, 8)).collect();
        for j in i + 1..regs.len() {
            let pj = poly_of(&regs[j].outer, 8);
            let hj: Vec<Vec<[f64; 2]>> = regs[j].holes.iter().map(|h| poly_of(h, 8)).collect();
            let solid_in = |q: [f64; 2], poly: &[[f64; 2]], holes: &[Vec<[f64; 2]>]| point_in_poly(q, poly) && !holes.iter().any(|h| point_in_poly(q, h));
            if polys_cross(&pi, &pj, false) || solid_in(pj[0], &pi, &hi_) || solid_in(pi[0], &pj, &hj) {
                return Err(FeatureError::new(WRAP_PROFILES_OVERLAP).with("profiles", Json::Arr(vec![s(&regs[i].tag), s(&regs[j].tag)])));
            }
        }
    }
    let th_c = 0.5 * (x0 + x1) / r;
    let geo = wrap_tool::Geo { o: o_ax, a, ex, ey, r, th_c };
    // off the face: outline samples outside the trimmed face, and boundary points of the face inside a profile
    let mut cache = ck_ops::boolean::classify::LoopCache::default();
    for rg in &regs {
        for q in poly_of(&rg.outer, 8) {
            let p3 = geo.pt(r, q);
            if ck_ops::boolean::classify::point_in_face_cached(&before, fid, p3, 1e-6, &mut cache) == ck_ops::boolean::classify::Where::Out {
                return Err(off_face(&rg.tag, p3));
            }
        }
    }
    let to_dev = |q: Vec3| -> [f64; 2] {
        let d = q - o_ax;
        let mut th = d.dot(ey).atan2(d.dot(ex));
        while th - th_c > PI {
            th -= TAU;
        }
        while th - th_c < -PI {
            th += TAU;
        }
        [th * r, d.dot(a)]
    };
    let rims: Vec<[f64; 2]> = before.face_edges(fid).into_iter().flat_map(|e| before.sample_edge(e, 0.05)).map(to_dev).collect();
    for rg in &regs {
        let outer = poly_of(&rg.outer, 8);
        let holes: Vec<Vec<[f64; 2]>> = rg.holes.iter().map(|h| poly_of(h, 8)).collect();
        for q in &rims {
            if point_in_poly(*q, &outer) && !holes.iter().any(|h| point_in_poly(*q, h)) {
                let at = geo.pt(r, *q);
                return Err(off_face(&rg.tag, at));
            }
        }
    }
    // taper
    if tapered {
        if let Some(tag) = taper_fault(&regs, d_b, d_t) {
            let (mut ok, mut bad_t) = (0.0f64, p.taper.abs());
            for _ in 0..12 {
                let mid = 0.5 * (ok + bad_t);
                let tn = mid.to_radians().tan() * p.taper.signum();
                if taper_fault(&regs, t_b * tn, t_t * tn).is_none() {
                    ok = mid;
                } else {
                    bad_t = mid;
                }
            }
            return Err(FeatureError::new(WRAP_TAPER_TOO_LARGE)
                .with("profile", s(&tag))
                .with("maxTaper", n((ok * p.taper.signum() * 100.0).floor() / 100.0))
                .with("hint", s("reduce the taper or the depth, or enlarge the profile: the rail would cross itself or lose a counter")));
        }
    }
    // the tool
    let exact = !tapered;
    let regions: Vec<RegionIn> = regs
        .iter()
        .map(|rg| RegionIn { outer: LoopIn { edges: edges_of(&rg.outer, d_b, d_t, exact) }, holes: rg.holes.iter().map(|h| LoopIn { edges: edges_of(h, d_b, d_t, exact) }).collect() })
        .collect();
    let out = wrap_tool::build(&f.id, &geo, &regions, r_base, r_top, p.tol)?;
    if let Some(dev_max) = out.approximated {
        warn(state, f, FeatureError::new(WRAP_APPROXIMATED).with("maxDeviation", n(dev_max)).with("tolerance", n(p.tol)));
    }
    // engraving deeper than the wall: valid, a warning
    if p.cut {
        let solid = ck_ops::boolean::classify::Solid::new(&before, 1e-6);
        for rg in &regs {
            let pts = poly_of(&rg.outer, 8);
            let mut through: Option<[f64; 2]> = None;
            for q in pts.iter().step_by(3) {
                let pt = geo.pt(r - sgn * p.depth, *q);
                if solid.classify(pt, geo.dir(q[0] / r)) == ck_ops::boolean::classify::Class::Out {
                    through = Some(*q);
                    break;
                }
            }
            if let Some(q) = through {
                // wall thickness there: bisection along the radial line
                let (mut lo_, mut hi_) = (0.0, p.depth);
                for _ in 0..24 {
                    let mid = 0.5 * (lo_ + hi_);
                    if solid.classify(geo.pt(r - sgn * mid, q), geo.dir(q[0] / r)) == ck_ops::boolean::classify::Class::Out {
                        hi_ = mid;
                    } else {
                        lo_ = mid;
                    }
                }
                warn(state, f, FeatureError::new(WRAP_CUT_THROUGH).with("profile", s(&rg.tag)).with("wall", n(lo_)));
            }
        }
    }
    let tool = Tool { body: out.body, combine: if p.cut { Combine::Cut } else { Combine::Add }, recipe: None };
    let issues_before = ck_check::validate(&before, 1e-6).len();
    if let Err(e) = br.apply_feature(f, tool, Some(&face_name)) {
        br.body = Some(before);
        return Err(e);
    }
    let after = br.body.as_ref().unwrap();
    if ck_check::validate(after, 1e-6).len() > issues_before {
        br.body = Some(before);
        return Err(FeatureError::new(wrap_tool::WRAP_TOOL_INVALID).with("detail", s("the boolean produced an invalid solid")));
    }
    if after.num_lumps() != before.num_lumps() {
        br.body = Some(before);
        return Err(FeatureError::new(wrap_tool::WRAP_TOOL_INVALID).with("detail", s("the wrap split or joined bodies")));
    }
    state.result.markers.retain(|m| m.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(f.id.as_str()));
    state.result.markers.push(Json::Arr(vec![
        s(&f.id),
        Json::Obj(vec![
            ("profiles".into(), n(regs.len() as f64)),
            ("turns".into(), n(widest / circ)),
            ("deviation".into(), n((out.deviation * 1e9).round() / 1e9)),
            ("method".into(), s("wrap")),
            ("surface".into(), s("cylinder")),
        ]),
    ]));
    let _ = Frame { o: o_ax, x: ex, y: ey, z: a };
    Ok(())
}

fn too_wide(tag: &str, extent: f64, circ: f64) -> FeatureError {
    FeatureError::new(WRAP_PROFILE_TOO_WIDE)
        .with("profile", s(tag))
        .with("extent", n(extent))
        .with("circumference", n(circ))
        .with("hint", s("shorten the text, reduce its height, or split it over two wraps: a profile must span less than one turn"))
}

fn off_face(tag: &str, at: Vec3) -> FeatureError {
    FeatureError::new(WRAP_PROFILE_OFF_FACE)
        .with("profile", s(tag))
        .with("at", v3(at))
        .with("hint", s("the profile leaves the face (its edge, or a hole in it): move it with alongAxis / around or make it smaller"))
}

/// `projectCurve` with `method: "wrap"` (proposal 2.2): the non-construction curves of a sketch (or the `entities` subset) mapped
/// through the development of the one cylindrical face in `faces`: sketch arc length is arc length on the face, `theta = s / r`,
/// `z = y`, the same map as the solid `wrap` (so a curve and a wrap of the same sketch coincide). A curve may run many turns.
/// Returns one dense polyline per entity (sampled so the chords stay within a quarter of `tol` of the face) and the span in turns.
pub(super) fn curve_runs(br: &Brep, state: &RegenState, f: &Feature, tol: f64) -> Result<(Vec<Vec<Vec3>>, f64), FeatureError> {
    if matches!(f.raw.get("procedural"), Some(Json::Bool(true))) {
        return Err(bad("procedural", "a wrapped curve is not procedural"));
    }
    let num = |k: &str| -> Result<f64, FeatureError> {
        let v = state.num(f, k, 0.0)?;
        if !v.is_finite() {
            return Err(bad(k, "must be a finite number"));
        }
        Ok(v)
    };
    let (along, around, rotation) = (num("alongAxis")?, num("around")?, num("rotation")?);
    let flip = matches!(f.raw.get("flip"), Some(Json::Bool(true)));
    let Some(sname) = f.raw.get_str("sketch") else {
        return Err(no_input("method wrap takes a sketch (its open or closed curves), not a curve feature"));
    };
    let sk = state.sketch(sname).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    extrude::finite_points(sk)?;
    // the one face
    let names: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    if names.len() != 1 {
        return Err(FeatureError::new(WRAP_FACE_UNSUPPORTED)
            .with("faceKind", s("none"))
            .with("hint", s("method wrap needs faces with exactly one cylindrical face")));
    }
    let slot = br.slot_of_face(names[0]).ok_or_else(|| br.missing_face(names[0]))?;
    let body = br.slot_body(slot).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let fid = body.find_face(names[0]).ok_or_else(|| crate::refs::missing_face([body], names[0]))?;
    let face = body.face(fid);
    let (cframe, r) = match &face.surface {
        Surface::Cylinder { frame, r } => (*frame, *r),
        Surface::Plane { .. } => {
            return Err(FeatureError::new(WRAP_FACE_UNSUPPORTED).with("faceKind", s("plane")).with("hint", s("a plane needs no wrap: use projectCurve without method")))
        }
        o => {
            return Err(FeatureError::new(WRAP_FACE_UNSUPPORTED)
                .with("faceKind", s(ck_topo::body::wire_kind(o)))
                .with("hint", s("only cylindrical faces are wrapped (method wrap)")))
        }
    };
    let su = cyl_setup(body, fid, cframe, sk)?;
    // the entities
    let want: Option<Vec<String>> = f.raw.get("entities").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str().map(String::from)).collect());
    let ids: Vec<String> = match want {
        Some(w) if !w.is_empty() => {
            for id in &w {
                if sk.sketch.entity(id).is_none() {
                    return Err(FeatureError::new(codes::MISSING_REFERENCE).with("field", s("entities")).with("name", s(id)));
                }
            }
            w
        }
        _ => sk.sketch.entities.iter().filter(|e| !e.construction()).map(|e| e.id().to_string()).collect(),
    };
    // dense 2D polylines in the sketch
    let fr = &sk.frame;
    let mut polys: Vec<(String, Vec<Vec3>)> = vec![];
    for id in ids {
        if let Some(pts) = super::curve_ref::sketch_entity_samples(sk, &id) {
            let loc: Vec<Vec3> = pts.iter().map(|p| Vec3::new((*p - fr.o).dot(fr.x), (*p - fr.o).dot(fr.y), 0.0)).collect();
            let dense = super::project_curve::normalise_samples(loc);
            if dense.len() >= 2 {
                polys.push((id, dense));
            }
        }
    }
    if polys.is_empty() {
        return Err(FeatureError::new("projectCurveNoInput"));
    }
    let mut dev = Dev { fx: su.fx, fy: su.fy, centre: [0.0, 0.0], cs: 1.0, sn: 0.0, flip, sgn: su.sgn, around_arc: around.to_radians() * r, z_off: (su.o_sk - su.o_ax).dot(su.a) + along };
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (_, pl) in &polys {
        for q in pl {
            let q = dev.raw(Vec2::new(q.x, q.y));
            for i in 0..2 {
                lo[i] = lo[i].min(q[i]);
                hi[i] = hi[i].max(q[i]);
            }
        }
    }
    dev.centre = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
    let ang = rotation.to_radians();
    dev.cs = ang.cos();
    dev.sn = ang.sin();
    let geo = wrap_tool::Geo { o: su.o_ax, a: su.a, ex: su.ex, ey: su.ey, r, th_c: 0.0 };
    // chords within a quarter of the tolerance of the face: a step of s on the face is off the cylinder by s^2 / 8 r
    let max_step = (2.0 * tol * r).sqrt().max(1e-3);
    let mut cache = ck_ops::boolean::classify::LoopCache::default();
    let (mut x_lo, mut x_hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let mut runs = vec![];
    for (id, pl) in &polys {
        let mut dpts: Vec<[f64; 2]> = vec![];
        for (k, q) in pl.iter().enumerate() {
            let d = dev.map(Vec2::new(q.x, q.y));
            if k > 0 {
                let p = *dpts.last().unwrap();
                let n = (((d[0] - p[0]).abs() / max_step).ceil() as usize).clamp(1, 200);
                for j in 1..n {
                    let t = j as f64 / n as f64;
                    dpts.push([p[0] + (d[0] - p[0]) * t, p[1] + (d[1] - p[1]) * t]);
                }
            }
            dpts.push(d);
        }
        let mut out = Vec::with_capacity(dpts.len());
        for d in dpts {
            x_lo = x_lo.min(d[0]);
            x_hi = x_hi.max(d[0]);
            let p3 = geo.pt(r, d);
            if ck_ops::boolean::classify::point_in_face_cached(body, fid, p3, 1e-6, &mut cache) == ck_ops::boolean::classify::Where::Out {
                return Err(off_face(id, p3).with("entity", s(id)));
            }
            out.push(p3);
        }
        runs.push(out);
    }
    Ok((runs, (x_hi - x_lo) / (TAU * r)))
}
