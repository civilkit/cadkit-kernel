//! Where the copies of a linear pattern and of a path pattern go (ck-model/src/features/pattern_place.rs).
//!
//! Linear pattern (`lpattern`), per direction (`dir`, and `dir2`, `dir3` with the same fields suffixed `2` / `3`; the three
//! make a block of copies: `count × count2 × count3`):
//! * `distanceType` `"spacing"` (default): `spacing` between neighbours, as before;
//! * `"extent"`: `extent` from the seed to the last copy, spacing = extent / (count − 1);
//! * `"toObject"`: the copy furthest along the direction reaches `to`, measured from the middle of the seeds
//!   (the centre of their tools' box): a plane (`{kind:"std"}`, a plane feature `{kind:"feature"}`) or a planar
//!   face (`{kind:"face", id}`): where the line from the middle along the direction meets it; a point
//!   (`[x,y,z]`, `{kind:"point", at}`, `{kind:"vertex", faces, at}`): its distance along the direction.
//!   `toOffset` moves the end the way the copies go (negative stops short). The object gives the side, so
//!   `reverse` is not read. Copies are numbered direction 1 first: `(i, j)` is copy `j·count + i`.
//!
//! Path pattern (`curvepattern`): `path` as a sweep's (`{kind:"sketch"}` open chain or sketch3d spline,
//! `{kind:"curve"}` helix / curve feature). The path runs from its end nearest the seeds' middle (`flip`: from the
//! other end). The seed sits `start` mm along it (or `startFraction` of its length; default 0) and copy k at
//! `start + k·spacing` (`distanceType:"extent"`: spacing = extent / (count − 1)). `orientation`: `"fixed"`
//! (default) moves the copies without turning them; `"tangent"` carries them with a rotation-minimising frame:
//! exact turns about each arc's axis, the least turn at each corner between pieces, sampled small turns along a
//! helix or spline. The original format's names are read too: `reverse` for `flip`, `align: true` for
//! `orientation: "tangent"`, `equal: true` (no `distanceType`) for copies spread over the rest of the path, and a
//! bare string `path` for a sketch id.

use super::{Brep, TOL};
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef};
use ck_core::linalg::{Mat3, Vec3};
use ck_core::xform::Xform;
use ck_json::Json;
use ck_ops::sweep::PathSeg;

/// The object of a To Object direction runs along the direction: no copy can reach it.
pub const TO_OBJECT_PARALLEL: &str = "patternToObjectParallel";
/// The object (with its offset) is at the seeds' middle or behind it after the offset: no room for the copies.
pub const TO_OBJECT_AT_SEED: &str = "patternToObjectAtSeed";
/// To Object on a face that is not flat.
pub const TO_OBJECT_UNSUPPORTED: &str = "patternToObjectUnsupported";
/// A path pattern's start point is not on the path.
pub const START_OUTSIDE: &str = "patternStartOutside";
/// The last copy of a path pattern lies past the end of an open path.
pub const BEYOND_PATH: &str = "patternBeyondPath";

fn num(x: f64) -> Json {
    Json::Num(x)
}
fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}
fn invalid(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}
fn missing(field: &str) -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE).with("field", Json::Str(field.into()))
}

/// One direction of a linear pattern: unit direction and signed spacing (copies at `k·spacing·d`, k = 0..count).
pub struct Row {
    pub d: Vec3,
    pub spacing: f64,
    pub count: usize,
}

/// Field name with the direction's suffix (`""` or `"2"`).
fn key(base: &str, sfx: &str) -> String {
    format!("{base}{sfx}")
}

/// Resolve direction `sfx` (`""` = direction 1, `"2"` / `"3"` = directions 2 and 3) of an `lpattern`; `None` for an unused one.
pub fn row(br: &mut Brep, state: &RegenState, f: &Feature, sfx: &str, mid: Option<Vec3>, count_of: impl Fn(f64) -> Result<usize, FeatureError>) -> Result<Option<Row>, FeatureError> {
    let raw = &f.raw;
    let dir = raw.get(&key("dir", sfx)).and_then(GeomRef::from_json);
    if !sfx.is_empty() && dir.is_none() {
        return Ok(None);
    }
    let (_, d) = dir.as_ref().and_then(|g| crate::planes::resolve_axis(state, br, g)).ok_or_else(|| {
        if sfx.is_empty() {
            FeatureError::new(codes::MISSING_REFERENCE)
        } else {
            missing(&key("dir", sfx))
        }
    })?;
    let count = count_of(state.num(f, &key("count", sfx), if sfx.is_empty() { 0.0 } else { 1.0 })?)?;
    let reverse = matches!(raw.get(&key("reverse", sfx)), Some(Json::Bool(true)));
    let sign = if reverse { -1.0 } else { 1.0 };
    let kind = raw.get(&key("distanceType", sfx)).and_then(|v| v.as_str()).unwrap_or("spacing");
    let gaps = count.max(2) as f64 - 1.0;
    let spacing = match kind {
        "extent" => {
            let ext = state.num(f, &key("extent", sfx), 0.0)?;
            if !(ext > 0.0) || !ext.is_finite() {
                return Err(invalid(&key("extent", sfx), "the extent (seed to last copy) must be above 0"));
            }
            sign * ext / gaps
        }
        "toObject" => {
            if count < 2 {
                0.0
            } else {
                let mid = mid.ok_or_else(|| FeatureError::new(super::pattern::NO_SEEDS))?;
                let dist = to_distance(br, state, raw.get(&key("to", sfx)), &key("to", sfx), mid, d)?;
                let off = state.num(f, &key("toOffset", sfx), 0.0)?;
                let s = dist.signum();
                let reach = dist + s * off;
                if dist.abs() <= 1e-9 || reach * s <= 1e-9 {
                    return Err(FeatureError::new(TO_OBJECT_AT_SEED)
                        .with("field", Json::Str(key("to", sfx)))
                        .with("distance", num(round6(reach)))
                        .with("hint", Json::Str("the object (with its offset) must lie ahead of the middle of the patterned features along the direction".into())));
                }
                reach / gaps
            }
        }
        "spacing" => sign * state.num(f, &key("spacing", sfx), 0.0)?,
        other => return Err(invalid(&key("distanceType", sfx), &format!("unknown distance type `{other}` (spacing, extent or toObject)"))),
    };
    Ok(Some(Row { d, spacing, count }))
}

/// Signed distance along `d` from `mid` to the To Object reference `r`.
fn to_distance(br: &Brep, state: &RegenState, r: Option<&Json>, field: &str, mid: Vec3, d: Vec3) -> Result<f64, FeatureError> {
    let r = r.filter(|v| !matches!(v, Json::Null)).ok_or_else(|| missing(field))?;
    if let Some(p) = point_of(br, r) {
        return Ok((p - mid).dot(d));
    }
    let (o, n) = match GeomRef::from_json(r).ok_or_else(|| missing(field))? {
        GeomRef::Std(p) => {
            let fr = p.frame();
            (fr.o, fr.z)
        }
        GeomRef::Feature(id) => {
            let fr = state.plane(&id).ok_or_else(|| missing(field).with("feature", Json::Str(id.clone())))?;
            (fr.o, fr.z)
        }
        GeomRef::Face(id) => {
            let body = br.body.as_ref().ok_or_else(|| missing(field))?;
            let at = r.get("at").and_then(json_pt);
            let (fid, _) = super::extrude::resolve_face(body, &id, at).ok_or_else(|| missing(field).with("face", Json::Str(id.clone())))?;
            match &body.face(fid).surface {
                ck_geom::surface::Surface::Plane { frame } => (frame.o, frame.z),
                _ => {
                    return Err(FeatureError::new(TO_OBJECT_UNSUPPORTED)
                        .with("face", Json::Str(id.clone()))
                        .with("hint", Json::Str("To Object takes a flat face, a plane or a point".into())))
                }
            }
        }
        _ => return Err(missing(field)),
    };
    let c = d.dot(n);
    if c.abs() < 1e-9 {
        return Err(FeatureError::new(TO_OBJECT_PARALLEL)
            .with("field", Json::Str(field.into()))
            .with("hint", Json::Str("the object runs along the pattern direction, so no copy can reach it: pick a plane or face across the direction, or a point".into())));
    }
    Ok((o - mid).dot(n) / c)
}

fn json_pt(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    if a.len() != 3 {
        return None;
    }
    Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?))
}

/// `[x,y,z]`, `{kind:"point", at}` or `{kind:"vertex", faces, at}` (the body vertex the faces share nearest `at`).
fn point_of(br: &Brep, r: &Json) -> Option<Vec3> {
    if let Some(p) = json_pt(r) {
        return Some(p);
    }
    let at = r.get("at").and_then(json_pt);
    match r.get("kind").and_then(|k| k.as_str()) {
        Some("point") => at,
        Some("vertex") => {
            let faces: Vec<String> = r.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|v| v.as_str().map(str::to_string)).collect()).unwrap_or_default();
            let body = br.body.as_ref()?;
            super::extrude::vertex_near(body, &faces, at.unwrap_or(Vec3::ZERO)).or(at)
        }
        _ => None,
    }
}

/// Instance transforms of one or two rows (copy `j·n1 + i` for grid cell (i, j), the seed (0, 0) left out).
pub fn grid(a: &Row, b: Option<&Row>) -> Vec<Xform> {
    grid3(a, b, None)
}

/// The copies of up to three directions (the seed itself, `(0, 0, 0)`, is left out): `(i, j, k)` is copy `(k·count2 + j)·count + i`.
pub fn grid3(a: &Row, b: Option<&Row>, c: Option<&Row>) -> Vec<Xform> {
    let (n2, d2, s2) = b.map_or((1, Vec3::ZERO, 0.0), |r| (r.count.max(1), r.d, r.spacing));
    let (n3, d3, s3) = c.map_or((1, Vec3::ZERO, 0.0), |r| (r.count.max(1), r.d, r.spacing));
    let n1 = a.count.max(1);
    let mut out = Vec::with_capacity(n1 * n2 * n3);
    for k in 0..n3 {
        for j in 0..n2 {
            for i in 0..n1 {
                if i == 0 && j == 0 && k == 0 {
                    continue;
                }
                out.push(Xform::translation(a.d * (a.spacing * i as f64) + d2 * (s2 * j as f64) + d3 * (s3 * k as f64)));
            }
        }
    }
    out
}

// ---------- path pattern ----------

/// One piece of a path, walked by arc length.
struct Piece {
    seg: PathSeg,
    len: f64,
}

fn seg_len(s: &PathSeg) -> f64 {
    match s {
        PathSeg::Line { a, b } => (*b - *a).len(),
        PathSeg::Arc { r, a0, a1, .. } => r * (a1 - a0).abs(),
        PathSeg::Helix { h, .. } => h.angle.abs() * (h.r * h.r + h.k * h.k).sqrt(),
        PathSeg::Spline { sp, .. } => sp.length(),
    }
}

/// Point and unit tangent `t` mm along a piece.
fn seg_at(s: &PathSeg, len: f64, t: f64) -> (Vec3, Vec3) {
    let u = if len > 0.0 { (t / len).clamp(0.0, 1.0) } else { 0.0 };
    match s {
        PathSeg::Line { a, b } => (*a + (*b - *a) * u, (*b - *a).try_normalize().unwrap_or(Vec3::X)),
        PathSeg::Arc { frame, r, a0, a1 } => {
            let th = a0 + (a1 - a0) * u;
            let sg = if a1 >= a0 { 1.0 } else { -1.0 };
            (frame.o + (frame.x * th.cos() + frame.y * th.sin()) * *r, (frame.y * th.cos() - frame.x * th.sin()) * sg)
        }
        PathSeg::Helix { h, rev } => {
            let (th, sg) = if *rev { (h.angle * (1.0 - u), -1.0) } else { (h.angle * u, 1.0) };
            let d = h.derivs(th);
            (d[0], (d[1] * sg).try_normalize().unwrap_or(Vec3::X))
        }
        PathSeg::Spline { sp, rev } => {
            let l = sp.length();
            let (q, sg) = if *rev { (l * (1.0 - u), -1.0) } else { (l * u, 1.0) };
            (sp.eval(q), (sp.d1(q) * sg).try_normalize().unwrap_or(Vec3::X))
        }
    }
}

/// The least rotation taking unit `a` onto unit `b`.
fn least_turn(a: Vec3, b: Vec3) -> Mat3 {
    let ax = a.cross(b);
    let s = ax.len();
    let c = a.dot(b).clamp(-1.0, 1.0);
    if s < 1e-14 {
        if c > 0.0 {
            return Mat3::IDENTITY;
        }
        // half turn about any axis square to a
        let p = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        return Mat3::rotation(a.cross(p).normalize(), std::f64::consts::PI);
    }
    Mat3::rotation(ax * (1.0 / s), s.atan2(c))
}

/// A path walked from one end, with its length.
pub struct Walk {
    pieces: Vec<Piece>,
    pub length: f64,
}

impl Walk {
    /// The chain `segs` oriented to start at its end nearest `near` (ties keep the chain's own start), reversed with `flip`.
    pub fn new(segs: Vec<PathSeg>, near: Vec3, flip: bool) -> Walk {
        let (s, e) = (segs.first().map(|p| p.start()).unwrap_or(Vec3::ZERO), segs.last().map(|p| p.end()).unwrap_or(Vec3::ZERO));
        let rev = (e.dist(near) < s.dist(near) - 1e-9) != flip;
        let segs: Vec<PathSeg> = if rev { segs.iter().rev().map(|p| p.reversed()).collect() } else { segs };
        let pieces: Vec<Piece> = segs.into_iter().map(|seg| Piece { len: seg_len(&seg), seg }).collect();
        let length = pieces.iter().map(|p| p.len).sum();
        Walk { pieces, length }
    }

    /// Piece index and the distance along it for `s` mm along the path.
    fn locate(&self, s: f64) -> (usize, f64) {
        let mut acc = 0.0;
        for (i, p) in self.pieces.iter().enumerate() {
            if s <= acc + p.len || i + 1 == self.pieces.len() {
                return (i, (s - acc).clamp(0.0, p.len));
            }
            acc += p.len;
        }
        (0, 0.0)
    }

    pub fn at(&self, s: f64) -> (Vec3, Vec3) {
        let (i, t) = self.locate(s);
        let p = &self.pieces[i];
        seg_at(&p.seg, p.len, t)
    }

    /// The rotation-minimising turn carried along the path from `s0` to `s1` (`s1 >= s0`).
    fn turn(&self, s0: f64, s1: f64) -> Mat3 {
        let (i0, t0) = self.locate(s0);
        let (i1, t1) = self.locate(s1);
        let mut m = Mat3::IDENTITY;
        for i in i0..=i1 {
            let p = &self.pieces[i];
            let (a, b) = (if i == i0 { t0 } else { 0.0 }, if i == i1 { t1 } else { p.len });
            if i > i0 {
                // the corner between piece i-1 and piece i
                let (_, out) = seg_at(&self.pieces[i - 1].seg, self.pieces[i - 1].len, self.pieces[i - 1].len);
                let (_, inn) = seg_at(&p.seg, p.len, 0.0);
                m = least_turn(out, inn).mul_mat(&m);
            }
            if b <= a {
                continue;
            }
            match &p.seg {
                PathSeg::Line { .. } => {}
                PathSeg::Arc { frame, r, a0, a1 } => {
                    let sg = if a1 >= a0 { 1.0 } else { -1.0 };
                    m = Mat3::rotation(frame.z, sg * (b - a) / r).mul_mat(&m);
                }
                _ => {
                    let n = 256usize;
                    let mut prev = seg_at(&p.seg, p.len, a).1;
                    for k in 1..=n {
                        let cur = seg_at(&p.seg, p.len, a + (b - a) * k as f64 / n as f64).1;
                        m = least_turn(prev, cur).mul_mat(&m);
                        prev = cur;
                    }
                }
            }
        }
        m
    }

    /// The transform taking the path point at `s0` (with its frame, when `tangent`) to the one at `s1`.
    pub fn carry(&self, s0: f64, s1: f64, tangent: bool) -> Xform {
        let (p0, _) = self.at(s0);
        let (p1, _) = self.at(s1);
        let r = if tangent { self.turn(s0, s1) } else { Mat3::IDENTITY };
        // p ↦ p1 + r (p − p0)
        Xform::new(r, p1 - r.mul_vec(p0))
    }
}

/// The `path` of a `curvepattern`; a bare string (the original format allows one) names a sketch.
pub fn path_ref(f: &Feature) -> Option<GeomRef> {
    match f.raw.get("path")? {
        Json::Str(id) => Some(GeomRef::Other(Json::Obj(vec![("kind".into(), Json::Str("sketch".into())), ("id".into(), Json::Str(id.clone()))]))),
        j => GeomRef::from_json(j),
    }
}

/// Instance transforms of a `curvepattern` (copy k = 1..count), or the refusal.
pub fn path_instances(br: &Brep, state: &RegenState, f: &Feature, mid: Vec3, count_of: impl Fn(f64) -> Result<usize, FeatureError>) -> Result<Vec<Xform>, FeatureError> {
    let segs = super::sweep::path_segs(br, state, path_ref(f).as_ref())?;
    let on = |k: &str| matches!(f.raw.get(k), Some(Json::Bool(true)));
    // `reverse` and `align` are the original format's names of `flip` and `orientation: "tangent"`
    let flip = on("flip") || on("reverse");
    let walk = Walk::new(segs, mid, flip);
    let len = walk.length;
    if !(len > TOL) {
        return Err(FeatureError::new(super::sweep::PATH_INVALID));
    }
    let count = count_of(state.num(f, "count", 0.0)?)?;
    let start = if f.raw.get("startFraction").is_some_and(|v| !matches!(v, Json::Null)) {
        let fr = state.num(f, "startFraction", 0.0)?;
        if !(0.0..=1.0).contains(&fr) {
            return Err(FeatureError::new(START_OUTSIDE).with("startFraction", num(fr)).with("hint", Json::Str("startFraction runs from 0 (the path's start) to 1 (its end)".into())));
        }
        fr * len
    } else {
        let s = state.num(f, "start", 0.0)?;
        if s < -1e-9 || s > len + 1e-9 || !s.is_finite() {
            return Err(FeatureError::new(START_OUTSIDE).with("start", num(s)).with("length", num(round6(len))).with("hint", Json::Str(format!("the start point is measured along the path and must lie from 0 to {} mm", round6(len)))));
        }
        s.clamp(0.0, len)
    };
    if count < 2 {
        return Ok(vec![]);
    }
    let gaps = count as f64 - 1.0;
    // the original format's `equal: true` (without a distance type): the copies spread over the rest of the path
    let kind = match f.raw.get("distanceType").and_then(|v| v.as_str()) {
        Some(k) => k,
        None if on("equal") => "fill",
        None => "spacing",
    };
    let spacing = match kind {
        "fill" => (len - start) / gaps,
        "extent" => {
            let ext = state.num(f, "extent", 0.0)?;
            if !(ext > 0.0) || !ext.is_finite() {
                return Err(invalid("extent", "the extent (seed to last copy) must be above 0"));
            }
            ext / gaps
        }
        "spacing" => state.num(f, "spacing", 0.0)?,
        other => return Err(invalid("distanceType", &format!("unknown distance type `{other}` (spacing or extent)"))),
    };
    if !(spacing > 0.0) || !spacing.is_finite() {
        return Err(invalid("spacing", "the spacing along the path must be above 0"));
    }
    let last = start + spacing * gaps;
    if last > len + 1e-7 * len.max(1.0) {
        return Err(FeatureError::new(BEYOND_PATH)
            .with("lastAt", num(round6(last)))
            .with("length", num(round6(len)))
            .with("maxSpacing", num(round6((len - start) / gaps)))
            .with("hint", Json::Str(format!("the last copy lies {} mm along a path {} mm long: use a spacing of at most {} mm, fewer copies or an earlier start", round6(last), round6(len), round6((len - start) / gaps)))));
    }
    let tangent = match f.raw.get("orientation").and_then(|v| v.as_str()).unwrap_or(if on("align") { "tangent" } else { "fixed" }) {
        "fixed" => false,
        "tangent" => true,
        other => return Err(invalid("orientation", &format!("unknown orientation `{other}` (fixed or tangent)"))),
    };
    Ok((1..count).map(|k| walk.carry(start, (start + spacing * k as f64).min(len), tangent)).collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;

    #[test]
    fn a_quarter_arc_turns_a_quarter() {
        let frame = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
        let w = Walk::new(vec![PathSeg::Arc { frame, r: 10.0, a0: 0.0, a1: std::f64::consts::FRAC_PI_2 }], Vec3::new(10.0, 0.0, 0.0), false);
        let x = w.carry(0.0, w.length, true);
        assert!((x.apply_point(Vec3::new(10.0, 0.0, 0.0)) - Vec3::new(0.0, 10.0, 0.0)).len() < 1e-12);
        assert!((x.apply_vec(Vec3::Y) - Vec3::new(-1.0, 0.0, 0.0)).len() < 1e-12);
        // reversed: walked from (0, 10)
        let r = Walk::new(vec![PathSeg::Arc { frame, r: 10.0, a0: 0.0, a1: std::f64::consts::FRAC_PI_2 }], Vec3::new(0.0, 11.0, 0.0), false);
        assert!((r.at(0.0).0 - Vec3::new(0.0, 10.0, 0.0)).len() < 1e-12);
        let x = r.carry(0.0, r.length, true);
        assert!((x.apply_vec(Vec3::X) - Vec3::new(0.0, -1.0, 0.0)).len() < 1e-12);
    }

    #[test]
    fn a_helix_carries_its_tangent_onto_the_tangent() {
        let h = ck_ops::helix::Helix { o: Vec3::ZERO, a: Vec3::Z, x: Vec3::X, y: Vec3::Y, r: 10.0, k: 2.0, angle: 3.0 * std::f64::consts::PI };
        let w = Walk::new(vec![PathSeg::Helix { h, rev: false }], Vec3::new(10.0, 0.0, 0.0), false);
        let (s0, s1) = (0.0, w.length * 0.7);
        let x = w.carry(s0, s1, true);
        let ((p0, t0), (p1, t1)) = (w.at(s0), w.at(s1));
        assert!((x.apply_point(p0) - p1).len() < 1e-9);
        assert!((x.apply_vec(t0) - t1).len() < 1e-4, "{:?} vs {t1:?}", x.apply_vec(t0));
    }
}
