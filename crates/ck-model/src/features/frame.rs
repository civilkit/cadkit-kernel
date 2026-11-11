//! `frame`: a structural frame (weldment) built from a skeleton. Every straight segment of the skeleton becomes one member, a
//! standard section (tube, bar, angle, channel, T-slot extrusion) swept along it, and the corners where members meet are trimmed:
//! mitred (both members cut on the bisecting plane) or butted (one member runs through, the other is cut against its face). A
//! member end that lands on the middle of another member is a tee: it is butted against that member's face. Each member is its
//! own body (a weldment's members are separate pieces of stock), named `<id> M<k>`, faces `<id>.M<k>.*`.
//!
//! Payload:
//! - `sketch`: the skeleton, a sketch or a `sketch3d`; every non-construction `line` is a member, in entity order. `segments`
//!   (entity ids) picks a subset. `lines`: `[[[x,y,z],[x,y,z]], ..]` extra members given in world coordinates (keys `L0`, `L1`..).
//! - `profile`: the section, `{kind, ...dims}` (numbers or expressions, mm): `rectTube {w,h,t,r}` (r = outer corner radius),
//!   `rect {w,h}` (flat or square bar), `roundTube {d,t}`, `roundBar {d}`, `angle {w,h,t}`, `channel {w,h,t}` (or `tf`, `tw` for its own flange and web thickness, `r` root radius), `ibeam {w,h,tf,tw,r}` (flanges `w` wide, depth `h`),
//!   `tslot {s, slot, lip, inner, depth, bore}`. `id` and `label` are carried through to the cut list. The section is centred on
//!   the line (its box centre); its local y axis is the skeleton sketch's normal (a sketch3d / world line: world Z, or world Y for
//!   a vertical member), turned by `rotate` (deg) about the member.
//! - `corner`: `mitre` (default) or `butt`. `members`: per-member overrides `[{segment, profile?, rotate?}]`. `joints`:
//!   `[{segments: [a, b], corner, through?}]` per corner (`through` = the member that runs on in a butt; default the first in order).
//!
//! - `hardware`: `{joint, cap, foot}` parts placed at the joints and free ends (L brackets with T-nuts and screws, gussets, end caps,
//!   levelling feet), `joints[].connector` overrides one joint; see `frame/hw.rs`. The reply's frame entry then has `hardware`.
//!
//! The regen result lists the cut list under `frames`: `[{id, members: [{segment, body, profile, label, length, angles: [a0, a1],
//! ends: [e0, e1], volume}]}]`, `length` the stock length (long point to long point), `angles` the cut angle of each end from square
//! (deg), `ends` one of `free`, `mitre`, `butt` (cut against the member that runs through a corner), `through` (runs past the corner),
//! `tee` (cut against the side of a member it ends on), `inline` (meets a member running on in line).
//! Errors: `frameInvalid {detail}` (no segment, a bad section, a corner too sharp), `booleanFailed` / invalid member solids.

use super::extrude::Combine;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::sketch::SketchEntity;
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::Vec2;
use ck_json::{Json, Obj};
use ck_topo::body::Body;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

mod hw;

pub const FRAME_INVALID: &str = "frameInvalid";
/// Endpoints closer than this are one node of the skeleton (mm).
const NODE_TOL: f64 = 1e-4;
/// Smallest angle between two members at a mitre (deg): sharper corners need a cut longer than the stock is wide.
const MIN_MITRE_DEG: f64 = 10.0;

fn invalid(detail: impl Into<String>) -> FeatureError {
    FeatureError::new(FRAME_INVALID).with("detail", Json::Str(detail.into()))
}

/// A piece of a section outline used for its support function: a point or a disc (the convex corners of a rounded section).
#[derive(Debug, Clone, Copy)]
enum Hull {
    Pt(Vec2),
    Disc(Vec2, f64),
}

/// A section: the planar regions (2D, centred on the member line) plus the pieces of its convex hull.
#[derive(Debug, Clone)]
pub struct Section {
    pub regions: Vec<ProfileRegion>,
    hull: Vec<Hull>,
}

impl Section {
    /// Furthest reach of the section in the 2D direction `v` (not normalised: the result scales with |v|).
    fn support(&self, v: Vec2) -> f64 {
        self.hull
            .iter()
            .map(|h| match *h {
                Hull::Pt(p) => p.dot(v),
                Hull::Disc(c, r) => c.dot(v) + r * v.len(),
            })
            .fold(f64::NEG_INFINITY, f64::max)
    }
    /// Radius of a circle about the line that holds the section.
    fn radius(&self) -> f64 {
        self.hull
            .iter()
            .map(|h| match *h {
                Hull::Pt(p) => p.len(),
                Hull::Disc(c, r) => c.len() + r,
            })
            .fold(0.0, f64::max)
    }
}

fn v2(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}

/// Closed polygon loop (CCW points), tags `<pre><i>`.
fn poly(pre: &str, pts: &[Vec2]) -> ProfileLoop {
    let n = pts.len();
    ProfileLoop::new((0..n).map(|i| ProfileSeg::line(&format!("{pre}{}", i + 1), pts[i], pts[(i + 1) % n])).collect())
}

/// Closed polygon loop (CCW) whose corners may be rounded: each vertex is `(point, radius)`, radius 0 = sharp. A corner that turns
/// left (convex) gets a counter-clockwise arc, one that turns right (concave: the root of a flange) a clockwise one. Errors when
/// the roundings of two neighbouring corners do not fit on the edge between them.
fn rounded_poly(pre: &str, pts: &[(Vec2, f64)]) -> Result<ProfileLoop, FeatureError> {
    let n = pts.len();
    // per corner: where its rounding starts and ends, and the arc
    struct Corner {
        s: Vec2,
        e: Vec2,
        arc: Option<ProfileSeg>,
    }
    let mut cs: Vec<Corner> = vec![];
    let mut k = 0;
    let mut tag = || {
        k += 1;
        format!("{pre}{k}")
    };
    let mut reach = vec![0.0; n];
    for i in 0..n {
        let (p, r) = pts[i];
        let (a, b) = (pts[(i + n - 1) % n].0, pts[(i + 1) % n].0);
        let (d1, d2) = (p.dist(a), p.dist(b));
        let u1 = v2((p.x - a.x) / d1, (p.y - a.y) / d1);
        let u2 = v2((b.x - p.x) / d2, (b.y - p.y) / d2);
        let cross = u1.x * u2.y - u1.y * u2.x;
        if r <= 1e-9 || cross.abs() < 1e-12 {
            cs.push(Corner { s: p, e: p, arc: None });
            continue;
        }
        let phi = cross.abs().atan2(u1.x * u2.x + u1.y * u2.y);
        let l = r * (phi / 2.0).tan();
        reach[i] = l;
        let s = v2(p.x - u1.x * l, p.y - u1.y * l);
        let e = v2(p.x + u2.x * l, p.y + u2.y * l);
        let nrm = if cross > 0.0 { v2(-u1.y, u1.x) } else { v2(u1.y, -u1.x) };
        let c = v2(s.x + nrm.x * r, s.y + nrm.y * r);
        let arc = if cross > 0.0 { ProfileSeg::arc(&tag(), c, s, e) } else { ProfileSeg::arc(&tag(), c, e, s).reversed() };
        cs.push(Corner { s, e, arc: Some(arc) });
    }
    for i in 0..n {
        if reach[i] + reach[(i + 1) % n] > pts[i].0.dist(pts[(i + 1) % n].0) + 1e-9 {
            return Err(invalid("a corner radius `r` does not fit on the edge beside it"));
        }
    }
    let mut segs = vec![];
    for i in 0..n {
        if let Some(a) = cs[i].arc.clone() {
            segs.push(a);
        }
        let (from, to) = (cs[i].e, cs[(i + 1) % n].s);
        if from.dist(to) > 1e-9 {
            segs.push(ProfileSeg::line(&tag(), from, to));
        }
    }
    Ok(ProfileLoop::new(segs))
}

/// Rectangle `w` x `h` centred on the origin with corner radius `r` (0: sharp), CCW.
fn rounded_rect(pre: &str, w: f64, h: f64, r: f64) -> (ProfileLoop, Vec<Hull>) {
    let (a, b) = (w / 2.0, h / 2.0);
    if r <= 1e-9 {
        let pts = [v2(-a, -b), v2(a, -b), v2(a, b), v2(-a, b)];
        return (poly(pre, &pts), pts.iter().map(|p| Hull::Pt(*p)).collect());
    }
    let (ia, ib) = (a - r, b - r);
    let c = [v2(ia, -ib), v2(ia, ib), v2(-ia, ib), v2(-ia, -ib)];
    let mut segs = vec![];
    let mut k = 0;
    let mut tag = || {
        k += 1;
        format!("{pre}{k}")
    };
    // bottom edge, then each corner arc CCW and the next edge
    segs.push(ProfileSeg::line(&tag(), v2(-ia, -b), v2(ia, -b)));
    segs.push(ProfileSeg::arc(&tag(), c[0], v2(ia, -b), v2(a, -ib)));
    segs.push(ProfileSeg::line(&tag(), v2(a, -ib), v2(a, ib)));
    segs.push(ProfileSeg::arc(&tag(), c[1], v2(a, ib), v2(ia, b)));
    segs.push(ProfileSeg::line(&tag(), v2(ia, b), v2(-ia, b)));
    segs.push(ProfileSeg::arc(&tag(), c[2], v2(-ia, b), v2(-a, ib)));
    segs.push(ProfileSeg::line(&tag(), v2(-a, ib), v2(-a, -ib)));
    segs.push(ProfileSeg::arc(&tag(), c[3], v2(-a, -ib), v2(-ia, -b)));
    (ProfileLoop::new(segs), c.iter().map(|c| Hull::Disc(*c, r)).collect())
}

fn hole(l: ProfileLoop) -> ProfileLoop {
    l.reversed()
}

/// Number field of a profile object (a number or an expression of the part's variables).
fn dim(state: &RegenState, p: &Json, key: &str, default: Option<f64>) -> Result<f64, FeatureError> {
    let v = match (p.get(key), default) {
        (None | Some(Json::Null), Some(d)) => d,
        (None | Some(Json::Null), None) => return Err(invalid(format!("profile needs `{key}`"))),
        (j, _) => crate::expr::eval_field(j, &state.vars, 0.0).map_err(FeatureError::new)?,
    };
    if !v.is_finite() || v < 0.0 {
        return Err(invalid(format!("profile `{key}` must be a number >= 0")));
    }
    Ok(v)
}

/// The section of a `profile` object.
pub fn section(state: &RegenState, p: &Json) -> Result<Section, FeatureError> {
    let kind = p.get_str("kind").ok_or_else(|| invalid("profile needs `kind`"))?;
    let pos = |v: f64, what: &str| if v > 1e-6 { Ok(v) } else { Err(invalid(format!("profile `{what}` must be above 0"))) };
    let s = match kind {
        "rect" | "rectTube" => {
            let (w, h) = (pos(dim(state, p, "w", None)?, "w")?, pos(dim(state, p, "h", Some(dim(state, p, "w", None)?))?, "h")?);
            let r = dim(state, p, "r", Some(0.0))?;
            if 2.0 * r > w.min(h) + 1e-9 {
                return Err(invalid("corner radius `r` is more than half the section"));
            }
            let (outer, hull) = rounded_rect("e", w, h, r);
            let mut holes = vec![];
            if kind == "rectTube" {
                let t = pos(dim(state, p, "t", None)?, "t")?;
                if 2.0 * t >= w.min(h) - 1e-6 {
                    return Err(invalid("wall `t` leaves no bore"));
                }
                holes.push(hole(rounded_rect("i", w - 2.0 * t, h - 2.0 * t, (r - t).max(0.0)).0));
            }
            Section { regions: vec![ProfileRegion { outer, holes }], hull }
        }
        "roundTube" | "roundBar" => {
            let d = pos(dim(state, p, "d", None)?, "d")?;
            let outer = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::ZERO, d / 2.0)]);
            let mut holes = vec![];
            if kind == "roundTube" {
                let t = pos(dim(state, p, "t", None)?, "t")?;
                if 2.0 * t >= d - 1e-6 {
                    return Err(invalid("wall `t` leaves no bore"));
                }
                holes.push(hole(ProfileLoop::new(vec![ProfileSeg::circle("i1", Vec2::ZERO, d / 2.0 - t)])));
            }
            Section { regions: vec![ProfileRegion { outer, holes }], hull: vec![Hull::Disc(Vec2::ZERO, d / 2.0)] }
        }
        "angle" | "channel" => {
            let w = pos(dim(state, p, "w", None)?, "w")?;
            let h = pos(dim(state, p, "h", Some(w))?, "h")?;
            let t = dim(state, p, "t", Some(0.0))?;
            // a channel may give its own flange `tf` and web `tw` thickness (default `t`) and a root radius `r`
            let (tf, tw) = if kind == "channel" { (dim(state, p, "tf", Some(t))?, dim(state, p, "tw", Some(t))?) } else { (t, t) };
            let r = if kind == "channel" { dim(state, p, "r", Some(0.0))? } else { 0.0 };
            let (tf, tw) = (pos(tf, "tf")?, pos(tw, "tw")?);
            if kind == "angle" && t <= 1e-6 {
                return Err(invalid("profile `t` must be above 0"));
            }
            if (kind == "angle" && t >= w.min(h) - 1e-6) || (kind == "channel" && (tw >= w - 1e-6 || 2.0 * tf >= h - 1e-6)) {
                return Err(invalid("wall `t` is as thick as the section"));
            }
            let (a, b) = (w / 2.0, h / 2.0);
            // angle: legs along -y (left) and -x (bottom), heel at (-a,-b); channel: web on the left, flanges top and bottom
            let outer = if kind == "angle" {
                poly("e", &[v2(-a, -b), v2(a, -b), v2(a, -b + t), v2(-a + t, -b + t), v2(-a + t, b), v2(-a, b)])
            } else {
                rounded_poly("e", &[(v2(-a, -b), 0.0), (v2(a, -b), 0.0), (v2(a, -b + tf), 0.0), (v2(-a + tw, -b + tf), r), (v2(-a + tw, b - tf), r), (v2(a, b - tf), 0.0), (v2(a, b), 0.0), (v2(-a, b), 0.0)])?
            };
            let hull = if kind == "angle" { vec![Hull::Pt(v2(-a, -b)), Hull::Pt(v2(a, -b)), Hull::Pt(v2(a, -b + t)), Hull::Pt(v2(-a + t, b)), Hull::Pt(v2(-a, b))] } else { [v2(-a, -b), v2(a, -b), v2(a, b), v2(-a, b)].iter().map(|p| Hull::Pt(*p)).collect() };
            Section { regions: vec![ProfileRegion { outer, holes: vec![] }], hull }
        }
        "ibeam" => {
            // an I or H section: flanges `w` wide and `tf` thick at the top and bottom of the depth `h`, a web `tw` thick, root radius `r`
            let w = pos(dim(state, p, "w", None)?, "w")?;
            let h = pos(dim(state, p, "h", None)?, "h")?;
            let tf = pos(dim(state, p, "tf", None)?, "tf")?;
            let tw = pos(dim(state, p, "tw", None)?, "tw")?;
            let r = dim(state, p, "r", Some(0.0))?;
            if tw >= w - 1e-6 || 2.0 * tf >= h - 1e-6 {
                return Err(invalid("flange `tf` or web `tw` is as thick as the section"));
            }
            let (a, b, c) = (w / 2.0, h / 2.0, tw / 2.0);
            let pts = [
                (v2(-a, -b), 0.0), (v2(a, -b), 0.0), (v2(a, -b + tf), 0.0), (v2(c, -b + tf), r), (v2(c, b - tf), r), (v2(a, b - tf), 0.0),
                (v2(a, b), 0.0), (v2(-a, b), 0.0), (v2(-a, b - tf), 0.0), (v2(-c, b - tf), r), (v2(-c, -b + tf), r), (v2(-a, -b + tf), 0.0),
            ];
            let hull = [v2(-a, -b), v2(a, -b), v2(a, b), v2(-a, b)].iter().map(|p| Hull::Pt(*p)).collect();
            Section { regions: vec![ProfileRegion { outer: rounded_poly("e", &pts)?, holes: vec![] }], hull }
        }
        "tslot" => {
            // a simplified T-slot extrusion: square `s`, one T slot in the middle of each side (opening `slot`, neck `lip` deep,
            // undercut `inner` wide down to `depth`), a centre bore `bore` (0: none)
            let s = pos(dim(state, p, "s", None)?, "s")?;
            let slot = pos(dim(state, p, "slot", Some(s * 0.31))?, "slot")?;
            let lip = pos(dim(state, p, "lip", Some(s * 0.09))?, "lip")?;
            let inner = pos(dim(state, p, "inner", Some(s * 0.55))?, "inner")?;
            let depth = pos(dim(state, p, "depth", Some(s * 0.3))?, "depth")?;
            let bore = dim(state, p, "bore", Some(s * 0.21))?;
            let a = s / 2.0;
            if !(slot < inner && inner < s - 2.0 * lip && lip < depth && 2.0 * depth + bore < s - 1e-6 && slot / 2.0 < 0.8 * (a - depth)) {
                return Err(invalid("T-slot dimensions do not fit the section (slot < inner < s - 2 lip, lip < depth, 2 depth + bore < s, the slot narrower than the core)"));
            }
            // one side (the bottom, y = -a, travelling +x), rotated four times; the undercut narrows to its floor (a trapezoid, as
            // the real profiles do) so that each slot stays in its own quarter of the section and the four never cross
            let (hs, hi) = (slot / 2.0, inner / 2.0);
            let hb = hi.min(0.8 * (a - depth)).max(hs);
            let side = [v2(-a, -a), v2(-hs, -a), v2(-hs, -a + lip), v2(-hi, -a + lip), v2(-hb, -a + depth), v2(hb, -a + depth), v2(hi, -a + lip), v2(hs, -a + lip), v2(hs, -a)];
            let mut pts = vec![];
            for k in 0..4 {
                let (c, sn) = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)][k];
                pts.extend(side.iter().map(|p| v2(p.x * c - p.y * sn, p.x * sn + p.y * c)));
            }
            let holes = if bore > 1e-6 { vec![hole(ProfileLoop::new(vec![ProfileSeg::circle("i1", Vec2::ZERO, bore / 2.0)]))] } else { vec![] };
            let hull = [v2(-a, -a), v2(a, -a), v2(a, a), v2(-a, a)].iter().map(|p| Hull::Pt(*p)).collect();
            Section { regions: vec![ProfileRegion { outer: poly("e", &pts), holes }], hull }
        }
        other => return Err(invalid(format!("unknown profile kind `{other}` (rectTube, rect, roundTube, roundBar, angle, channel, ibeam, tslot)"))),
    };
    Ok(s)
}

/// One straight segment of the skeleton.
#[derive(Debug, Clone)]
pub struct Seg {
    pub key: String,
    pub a: Vec3,
    pub b: Vec3,
    /// The section's "up" before `rotate`: the skeleton sketch normal, else a world axis.
    pub up: Option<Vec3>,
}

fn vec3(j: &Json) -> Option<Vec3> {
    let v = j.as_f64_vec()?;
    (v.len() == 3 && v.iter().all(|x| x.is_finite())).then(|| Vec3::new(v[0], v[1], v[2]))
}

/// The segments of the payload: the lines of `sketch` (planar or 3D, `segments` picks some), then `lines`.
fn segments(state: &RegenState, f: &Feature) -> Result<Vec<Seg>, FeatureError> {
    let pick: Option<Vec<&str>> = f.raw.get_arr("segments").map(|a| a.iter().filter_map(|s| s.as_str()).collect());
    let wanted = |id: &str| pick.as_ref().map_or(true, |p| p.contains(&id));
    let mut out = vec![];
    if let Some(id) = f.raw.get_str("sketch") {
        if let Some(sk) = state.sketch(id) {
            let fr = sk.frame;
            for e in &sk.sketch.entities {
                let SketchEntity::Line { p1, p2, .. } = e else { continue };
                if e.construction() || !wanted(e.id()) {
                    continue;
                }
                let (Some(a), Some(b)) = (sk.sketch.point_xy(p1), sk.sketch.point_xy(p2)) else { continue };
                let w = |p: [f64; 2]| fr.o + fr.x * p[0] + fr.y * p[1];
                out.push(Seg { key: e.id().to_string(), a: w(a), b: w(b), up: Some(fr.z) });
            }
        } else if let Some((_, j)) = state.result.sketches3.iter().find(|s| s.0 == id) {
            let sk = j.get("sketch").unwrap_or(j);
            let points = sk.get_arr("points").unwrap_or(&[]);
            let point = |pid: &str| -> Option<Vec3> {
                let p = points.iter().find(|p| p.get_str("id") == Some(pid))?;
                Some(Vec3::new(p.get_f64("x").unwrap_or(0.0), p.get_f64("y").unwrap_or(0.0), p.get_f64("z").unwrap_or(0.0)))
            };
            for e in sk.get_arr("entities").unwrap_or(&[]) {
                if e.get_str("type") != Some("line") || e.get("construction").and_then(|c| c.as_bool()) == Some(true) {
                    continue;
                }
                let eid = e.get_str("id").unwrap_or("");
                if !wanted(eid) {
                    continue;
                }
                let (Some(a), Some(b)) = (e.get_str("p1").and_then(point), e.get_str("p2").and_then(point)) else { continue };
                out.push(Seg { key: eid.to_string(), a, b, up: None });
            }
        } else {
            return Err(FeatureError::new(codes::MISSING_SKETCH));
        }
    }
    for (k, l) in f.raw.get_arr("lines").unwrap_or(&[]).iter().enumerate() {
        let pts = l.as_arr().ok_or_else(|| invalid("a `lines` entry is [[x,y,z],[x,y,z]]"))?;
        let (Some(a), Some(b)) = (pts.first().and_then(vec3), pts.get(1).and_then(vec3)) else { return Err(invalid("a `lines` entry is [[x,y,z],[x,y,z]]")) };
        out.push(Seg { key: format!("L{k}"), a, b, up: None });
    }
    if out.is_empty() {
        return Err(invalid("the frame has no segments: give a skeleton `sketch` with lines, or `lines`"));
    }
    for s in &out {
        if s.a.dist(s.b) < 1e-3 {
            return Err(invalid(format!("segment {} has no length", s.key)));
        }
    }
    Ok(out)
}

/// How one end of a member is finished.
#[derive(Debug, Clone, Copy)]
enum End {
    /// Square cut at axial position `t` (from the segment start, along the member).
    Square { t: f64 },
    /// Cut on a plane: material kept where (x - p).n >= 0.
    Plane { p: Vec3, n: Vec3 },
}

/// A member: segment, section frame, section, the two ends.
struct Member {
    seg: Seg,
    d: Vec3,
    len: f64,
    /// section axes (x, y) perpendicular to `d`
    sx: Vec3,
    sy: Vec3,
    sec: Section,
    prof: Json,
    ends: [End; 2],
    kinds: [&'static str; 2],
}

impl Member {
    /// Furthest reach of the section along the 3D direction `v` (its part across the member).
    fn reach(&self, v: Vec3) -> f64 {
        self.sec.support(v2(v.dot(self.sx), v.dot(self.sy)))
    }
    /// Axial range of the material at an end: (lowest, highest) t.
    fn end_range(&self, e: End) -> (f64, f64) {
        match e {
            End::Square { t } => (t, t),
            End::Plane { p, n } => {
                let dn = self.d.dot(n);
                let c = (p - self.seg.a).dot(n);
                let lo = (c - self.reach(n)) / dn;
                let hi = (c + self.reach(-n)) / dn;
                (lo.min(hi), lo.max(hi))
            }
        }
    }
    /// Stock length, long point to long point.
    fn stock(&self) -> f64 {
        self.end_range(self.ends[1]).1 - self.end_range(self.ends[0]).0
    }
    fn angle(&self, k: usize) -> f64 {
        match self.ends[k] {
            End::Square { .. } => 0.0,
            End::Plane { n, .. } => self.d.dot(n).abs().clamp(0.0, 1.0).acos().to_degrees(),
        }
    }
}

/// Unit "up" for a member along `d`: the sketch normal, else world Z (world Y for a member along Z).
fn up_for(d: Vec3, up: Option<Vec3>) -> Vec3 {
    let u = up.unwrap_or(Vec3::new(0.0, 0.0, 1.0));
    if (u - d * u.dot(d)).len() > 1e-6 {
        return u;
    }
    // along the up direction itself (a post in a sketch3d, or a line normal to the sketch)
    if up.is_some() && d.cross(Vec3::new(0.0, 0.0, 1.0)).len() > 1e-6 {
        return Vec3::new(0.0, 0.0, 1.0);
    }
    Vec3::new(0.0, 1.0, 0.0)
}

/// Half-space solid on the plane through `p` whose material is on the side `-n` (the side a member keeps is `+n`): a box `big`
/// on a side, faces `<tag>.*`.
fn half_space(tag: &str, p: Vec3, n: Vec3, big: f64) -> Result<Body, FeatureError> {
    let z = -n;
    let x = z.cross(if z.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) }).normalize();
    let y = z.cross(x);
    let fr = Frame { o: p, x, y, z };
    let (outer, _) = rounded_rect("e", 2.0 * big, 2.0 * big, 0.0);
    let prof = Profile::new(fr, vec![ProfileRegion { outer, holes: vec![] }]);
    extrude(tag, &prof, ExtrudeOpts::blind(big)).map_err(|e| invalid(format!("trim tool: {e}")))
}

fn str_list(j: Option<&Json>) -> Vec<&str> {
    j.and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|s| s.as_str()).collect()).unwrap_or_default()
}

/// `frame`.
pub fn exec_frame(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let segs = segments(state, f)?;
    let base_prof = f.raw.get("profile").filter(|p| !matches!(p, Json::Null)).ok_or_else(|| invalid("the frame needs a `profile`"))?;
    let base_rot = state.num(f, "rotate", 0.0)?;
    let corner = f.raw.get_str("corner").unwrap_or("mitre");
    if !matches!(corner, "mitre" | "miter" | "butt") {
        return Err(invalid(format!("corner `{corner}`: mitre or butt")));
    }
    let overrides = f.raw.get_arr("members").unwrap_or(&[]);
    let mut ms: Vec<Member> = vec![];
    for s in segs {
        let ov = overrides.iter().find(|o| o.get_str("segment") == Some(s.key.as_str()));
        let prof = ov.and_then(|o| o.get("profile")).filter(|p| !matches!(p, Json::Null)).unwrap_or(base_prof).clone();
        let rot = match ov.and_then(|o| o.get("rotate")) {
            Some(j) => crate::expr::eval_field(Some(j), &state.vars, 0.0).map_err(FeatureError::new)?,
            None => base_rot,
        }
        .to_radians();
        let sec = section(state, &prof)?;
        let len = s.a.dist(s.b);
        let d = (s.b - s.a) * (1.0 / len);
        let up = up_for(d, s.up);
        let y0 = (up - d * up.dot(d)).normalize();
        let x0 = y0.cross(d);
        let (c, sn) = (rot.cos(), rot.sin());
        let (sx, sy) = (x0 * c + y0 * sn, y0 * c - x0 * sn);
        ms.push(Member { seg: s, d, len, sx, sy, sec, prof, ends: [End::Square { t: 0.0 }, End::Square { t: len }], kinds: ["free"; 2] });
    }
    for i in 0..ms.len() {
        if ms[..i].iter().any(|m| m.seg.key == ms[i].seg.key) {
            return Err(invalid(format!("segment {} is listed twice", ms[i].seg.key)));
        }
    }
    let joints = f.raw.get_arr("joints").unwrap_or(&[]);
    let joint_of = |a: &str, b: &str| {
        joints.iter().find(|j| {
            let s = str_list(j.get("segments"));
            s.len() == 2 && ((s[0] == a && s[1] == b) || (s[0] == b && s[1] == a))
        })
    };
    // the end point of each member end, and the direction into the member from it
    let at = |m: &Member, k: usize| if k == 0 { m.seg.a } else { m.seg.b };
    let into = |m: &Member, k: usize| if k == 0 { m.d } else { -m.d };
    // the t of a square end `s` beyond the end point (outwards)
    let square = |m: &Member, k: usize, s: f64| End::Square { t: if k == 0 { -s } else { m.len + s } };
    let n = ms.len();
    let mut ends: Vec<[(End, &'static str); 2]> = ms.iter().map(|m| [(m.ends[0], "free"), (m.ends[1], "free")]).collect();
    for i in 0..n {
        for k in 0..2 {
            let p = at(&ms[i], k);
            let u = into(&ms[i], k);
            let others: Vec<(usize, usize)> = (0..n).filter(|&j| j != i).flat_map(|j| (0..2).map(move |kk| (j, kk))).filter(|&(j, kk)| at(&ms[j], kk).dist(p) < NODE_TOL).collect();
            // butt this end against member `t` (the through member): cut on the plane of its face
            let butt = |ms: &Vec<Member>, t: usize| -> End {
                let ut = ms[t].d;
                let w = u - ut * u.dot(ut);
                let Some(w) = w.try_normalize() else { return End::Square { t: if k == 0 { 0.0 } else { ms[i].len } } };
                let e = ms[t].reach(w);
                let q = p + w * e;
                if (w.dot(u) - 1.0).abs() < 1e-9 {
                    // square to this member: no boolean
                    let s = (q - ms[i].seg.a).dot(ms[i].d);
                    End::Square { t: s }
                } else {
                    End::Plane { p: q, n: w }
                }
            };
            if others.is_empty() {
                // a tee: the end lies inside another member's segment
                let tee = (0..n).filter(|&j| j != i).find(|&j| {
                    let m = &ms[j];
                    let s = (p - m.seg.a).dot(m.d);
                    s > NODE_TOL && s < m.len - NODE_TOL && (m.seg.a + m.d * s).dist(p) < NODE_TOL
                });
                if let Some(j) = tee {
                    if ms[j].d.cross(u).len() < 1e-6 {
                        return Err(invalid(format!("segment {} ends inside segment {} along it", ms[i].seg.key, ms[j].seg.key)));
                    }
                    ends[i][k] = (butt(&ms, j), "tee");
                }
                continue;
            }
            if others.len() == 1 {
                let (j, kk) = others[0];
                let uj = into(&ms[j], kk);
                let jo = joint_of(&ms[i].seg.key, &ms[j].seg.key);
                let c = jo.and_then(|j| j.get_str("corner")).unwrap_or(corner);
                if u.dot(uj) < -1.0 + 1e-9 {
                    ends[i][k] = (square(&ms[i], k, 0.0), "inline");
                    continue;
                }
                let theta = u.dot(uj).clamp(-1.0, 1.0).acos().to_degrees();
                if c == "butt" {
                    let through = match jo.and_then(|j| j.get_str("through")) {
                        Some(t) if t == ms[i].seg.key => i,
                        Some(t) if t == ms[j].seg.key => j,
                        Some(t) => return Err(invalid(format!("joint through `{t}` is neither of its segments"))),
                        None => i.min(j),
                    };
                    if through == i {
                        // run on past the corner until the end covers the other member: its reach across this one, plus the
                        // slant of an oblique corner
                        let w = (-u - uj * (-u).dot(uj)).normalize();
                        let sin = (1.0 - u.dot(uj).powi(2)).sqrt().max(1e-9);
                        let wt = (uj - u * uj.dot(u)).normalize();
                        let s = ms[j].reach(w) / sin + ms[i].reach(wt) * (u.dot(uj).abs() / sin);
                        ends[i][k] = (square(&ms[i], k, s), "through");
                    } else {
                        ends[i][k] = (butt(&ms, through), "butt");
                    }
                } else if c == "mitre" || c == "miter" {
                    if theta < MIN_MITRE_DEG {
                        return Err(invalid(format!("segments {} and {} meet at {theta:.1} deg, too sharp to mitre (min {MIN_MITRE_DEG})", ms[i].seg.key, ms[j].seg.key)));
                    }
                    let nrm = (u - uj).normalize();
                    ends[i][k] = (End::Plane { p, n: nrm }, "mitre");
                } else {
                    return Err(invalid(format!("joint corner `{c}`: mitre or butt")));
                }
                continue;
            }
            // three or more members at one node: the first in order runs through, the others butt against it
            let through = others.iter().map(|o| o.0).chain([i]).min().unwrap();
            if through == i {
                let s = others.iter().map(|&(j, _)| ms[j].reach(-u)).fold(0.0, f64::max);
                ends[i][k] = (square(&ms[i], k, s), "through");
            } else {
                ends[i][k] = (butt(&ms, through), "butt");
            }
        }
    }
    for (m, e) in ms.iter_mut().zip(&ends) {
        m.ends = [e[0].0, e[1].0];
        m.kinds = [e[0].1, e[1].1];
    }
    // the joints (pairs of members meeting at a node, or a member ending on another) for the hardware
    let mut jl: Vec<hw::JointAt> = vec![];
    let mut nodes3: Vec<hw::Node3> = vec![];
    for i in 0..n {
        for k in 0..2 {
            let p = at(&ms[i], k);
            let cl: Vec<(usize, usize)> = (0..n).flat_map(|j| (0..2).map(move |kk| (j, kk))).filter(|&(j, kk)| at(&ms[j], kk).dist(p) < NODE_TOL).collect();
            if cl.len() >= 2 && cl[0] == (i, k) {
                let through = cl.iter().map(|c| c.0).min().unwrap();
                let first = *cl.iter().find(|c| c.0 == through).unwrap();
                let pairs: Vec<((usize, usize), (usize, usize))> = if cl.len() == 2 { vec![(cl[0], cl[1])] } else { cl.iter().filter(|c| c.0 != through).map(|c| (first, *c)).collect() };
                // three members at right angles: a 3-way node (the through member first)
                let node3 = if cl.len() == 3 {
                    let order: Vec<(usize, usize)> = std::iter::once(first).chain(cl.iter().copied().filter(|c| c.0 != through)).collect();
                    let u: Vec<Vec3> = order.iter().map(|&(j, kk)| into(&ms[j], kk)).collect();
                    let square = (0..3).all(|a| u[a].dot(u[(a + 1) % 3]).abs() < 1e-6);
                    square.then(|| {
                        nodes3.push(hw::Node3 { m: [order[0].0, order[1].0, order[2].0], u: [u[0], u[1], u[2]], node: p });
                        nodes3.len() - 1
                    })
                } else {
                    None
                };
                for ((ja, ka), (jb, kb)) in pairs {
                    let (u1, u2) = (into(&ms[ja], ka), into(&ms[jb], kb));
                    if u1.dot(u2) < -1.0 + 1e-9 {
                        continue;
                    }
                    let corner = if ends[ja][ka].1 == "mitre" && ends[jb][kb].1 == "mitre" { "mitre" } else { "butt" };
                    jl.push(hw::JointAt { a: ja, b: jb, u1, u2, node: p, corner, room: [ms[ja].len, ms[jb].len], alt: None, node3 });
                }
            } else if cl.len() == 1 && ends[i][k].1 == "tee" {
                if let Some(j) = (0..n).filter(|&j| j != i).find(|&j| {
                    let s = (p - ms[j].seg.a).dot(ms[j].d);
                    s > NODE_TOL && s < ms[j].len - NODE_TOL && (ms[j].seg.a + ms[j].d * s).dist(p) < NODE_TOL
                }) {
                    let s = (p - ms[j].seg.a).dot(ms[j].d);
                    jl.push(hw::JointAt { a: j, b: i, u1: ms[j].d, u2: into(&ms[i], k), node: p, corner: "tee", room: [ms[j].len - s, ms[i].len], alt: Some((-ms[j].d, s)), node3: None });
                }
            }
        }
    }
    // build each member: the section extruded over the whole axial range, then the plane ends cut off
    let hw_out = hw::place(state, f, &ms, &jl, &nodes3)?;
    let mut list = vec![];
    let mut tools: Vec<Body> = vec![];
    for (idx, m) in ms.iter().enumerate() {
        let tag = format!("{}.M{}", f.id, idx + 1);
        let (lo0, _) = m.end_range(m.ends[0]);
        let (_, hi1) = m.end_range(m.ends[1]);
        let (s0, s1) = (lo0, hi1);
        if s1 - s0 < 1e-3 {
            return Err(invalid(format!("member {} is trimmed away at its corners", m.seg.key)));
        }
        let margin = 1.0 + 0.05 * m.sec.radius();
        let ext = |e: End, lo: f64, hi: f64| match e {
            End::Square { .. } => (lo, hi),
            End::Plane { .. } => (lo - margin, hi + margin),
        };
        let (a0, _) = ext(m.ends[0], s0, s0);
        let (_, a1) = ext(m.ends[1], s1, s1);
        let fr = Frame { o: m.seg.a, x: m.sx, y: m.sy, z: m.d };
        let prof = Profile::new(fr, m.sec.regions.clone()).normalized();
        let mut body = extrude(&tag, &prof, ExtrudeOpts { s0: a0, s1: a1, dir: None, draft: 0.0 }).map_err(|e| invalid(format!("member {}: {e}", m.seg.key)))?;
        let big = 4.0 * (m.len + (a1 - a0) + m.sec.radius());
        for (k, e) in m.ends.iter().enumerate() {
            let End::Plane { p, n } = *e else { continue };
            // keep (x - p).n >= 0 at the start end; at the far end the member lies on the side the plane normal points to as well
            let tool = half_space(&format!("{tag}.T{k}"), p, n, big)?;
            let out = ck_ops::boolean::boolean(&body, &tool, ck_ops::boolean::Op::Cut, super::TOL).map_err(|e| {
                FeatureError::new("booleanFailed").with("detail", Json::Str(format!("member {} end {k}: {e}", m.seg.key)))
            })?;
            body = out.body;
        }
        // holes drilled for bolted hardware
        let mut holes = 0.0;
        if let Some((_, drills, _)) = &hw_out {
            for (k, d) in drills.iter().enumerate().filter(|(_, d)| d.member == idx) {
                let tool = d.tool(&format!("{tag}.D{k}"))?;
                body = ck_ops::boolean::boolean(&body, &tool, ck_ops::boolean::Op::Cut, super::TOL)
                    .map_err(|e| FeatureError::new("booleanFailed").with("detail", Json::Str(format!("member {} hole {k}: {e}", m.seg.key))))?
                    .body;
                holes += 1.0;
            }
        }
        let issues = ck_check::validate(&body, 1e-6);
        if !issues.is_empty() || body.num_lumps() != 1 {
            let codes: Vec<String> = issues.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect();
            return Err(invalid(format!("member {} is not a valid solid ({} lumps; {})", m.seg.key, body.num_lumps(), codes.join("; "))));
        }
        let name = format!("{} M{}", f.id, idx + 1);
        for l in body.lump_ids().collect::<Vec<_>>() {
            body.lumps[l.idx()].name = Some(name.clone());
        }
        let vol = ck_topo::mass::mass_props(&body).volume;
        let r = |x: f64| (x * 1e6).round() / 1e6;
        list.push(
            Obj::new()
                .put("segment", m.seg.key.as_str())
                .put("body", name.as_str())
                .put("profile", m.prof.get_str("id").unwrap_or(m.prof.get_str("kind").unwrap_or("")))
                .put("label", m.prof.get_str("label").unwrap_or(""))
                .put("length", r(m.stock()))
                .put("angles", Json::Arr(vec![Json::Num(r(m.angle(0))), Json::Num(r(m.angle(1)))]))
                .put("ends", Json::Arr(m.kinds.iter().map(|k| Json::Str(k.to_string())).collect()))
                .put("volume", r(vol))
                .put_opt("holes", (holes > 0.0).then_some(holes))
                .build(),
        );
        tools.push(body);
    }
    for t in &tools {
        br.combine(t, Combine::Separate, false)?;
    }
    let mut entry = Obj::new().put("id", f.id.as_str()).put("members", Json::Arr(list));
    if let Some((bodies, _, report)) = hw_out {
        for b in &bodies {
            br.combine(b, Combine::Separate, false)?;
        }
        entry = entry.put("hardware", report);
    }
    state.result.put_frame(&f.id, entry.build());
    Ok(())
}
