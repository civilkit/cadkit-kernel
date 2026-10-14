//! `thread {face, standard, size, class?, length?, hand?, start?, modeled?}`:
//! a thread on a cylindrical face (the wall of a hole, or an external shaft).
//!
//! Tables: `crate::threads` (also served by the `threadData` request).
//!
//! * `face`: a cylindrical face name (`F3.S.e2`); concave (the hole wall) is an
//!   internal thread, convex (a shaft) an external one, read from the face. A
//!   face that is not a cylinder is `threadFaceNotCylinder {face, surface}`; a
//!   missing / unresolved face is `missingReference`; a `standard` / `size` that
//!   is not in the tables is `threadSizeUnknown {standard, size, suggestions}`.
//! * `length`: a number or `"full"` (default) = the extent of the face along its
//!   axis; clamped to it. `start`: `"top"` / `"bottom"` = the end of the face with
//!   the higher / lower coordinate along the axis (the axis is oriented so its
//!   dominant world component is positive); default the open end (a hole's mouth,
//!   a shaft's free end), `"top"` when both or neither end is open.
//!   `hand`: `"right"` (default) / `"left"`.
//! * Cosmetic (`modeled` false, the default): the geometry is untouched; the
//!   regen result gets an entry in `threads` and, when the face diameter does not
//!   fit the thread, a WARNING `threadDiameterMismatch` in `warnings` (not an
//!   error: the document still regenerates).
//! * Modeled: see [`model`]; falls back to cosmetic with a `threadModelFallback`
//!   warning when the boolean fails or the model would be unreasonably large.

use super::Brep;
use crate::regen::{codes, FeatureError, RegenError, RegenState};
use crate::threads::{self, ThreadSpec};
use crate::Feature;
use ck_core::linalg::Vec3;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

pub const THREAD_SIZE_UNKNOWN: &str = "threadSizeUnknown";
pub const THREAD_FACE_NOT_CYLINDER: &str = "threadFaceNotCylinder";
pub const THREAD_DIAMETER_MISMATCH: &str = "threadDiameterMismatch";
pub const THREAD_MODEL_FALLBACK: &str = "threadModelFallback";

/// Relative tolerance of the diameter check (3 percent).
pub const DIAMETER_TOL: f64 = 0.03;

/// A resolved cylindrical face.
#[derive(Debug, Clone, Copy)]
pub struct CylFace {
    /// A point on the axis.
    pub o: Vec3,
    /// Unit axis, oriented so its dominant world component is positive.
    pub up: Vec3,
    pub radius: f64,
    /// Extent along `up` (coordinates measured from `o`).
    pub lo: f64,
    pub hi: f64,
    pub internal: bool,
    /// The ends (lo, hi) that are open (the hole mouth / the free end of a shaft).
    pub open: [bool; 2],
}

fn missing() -> FeatureError {
    FeatureError::new(codes::MISSING_REFERENCE)
}

fn orient(a: Vec3) -> Vec3 {
    let (ax, ay, az) = (a.x.abs(), a.y.abs(), a.z.abs());
    let dom = if ax >= ay && ax >= az {
        a.x
    } else if ay >= az {
        a.y
    } else {
        a.z
    };
    if dom < 0.0 {
        -a
    } else {
        a
    }
}

/// Resolve a face to its cylinder data.
pub fn cyl_face(b: &Body, name: &str) -> Result<CylFace, FeatureError> {
    let fid = b.find_face(name).ok_or_else(missing)?;
    let face = b.face(fid);
    let Surface::Cylinder { frame, r } = &face.surface else {
        return Err(FeatureError::new(THREAD_FACE_NOT_CYLINDER)
            .with("face", Json::Str(name.to_string()))
            .with("surface", Json::Str(face.surface.kind().to_string()))
            .with("hint", Json::Str(format!("{name} is a {} face; a thread goes on a cylindrical face (the wall of a hole or a shaft)", face.surface.kind()))));
    };
    let up = orient(frame.z);
    let o = frame.o;
    // extent along the axis: the face's vertices (end circles lie at constant axial coordinate)
    let mut pts: Vec<Vec3> = vec![];
    for e in b.face_edges(fid) {
        let ed = b.edge(e);
        pts.push(b.vertex(ed.v0).p);
        pts.push(b.vertex(ed.v1).p);
    }
    if pts.is_empty() {
        return Err(missing());
    }
    let s = |p: &Vec3| (*p - o).dot(up);
    let lo = pts.iter().map(s).fold(f64::INFINITY, f64::min);
    let hi = pts.iter().map(s).fold(f64::NEG_INFINITY, f64::max);
    // concave (hole) or convex (shaft): the outward normal at a point of the face against the radial direction
    let p = pts[0];
    let (u, v) = face.surface.closest_param(p);
    let n = b.outward_normal(fid, u, v);
    let rad = (p - o) - up * (p - o).dot(up);
    let internal = n.dot(rad) < 0.0;
    // open ends: the face across the end circle is a plane whose outward normal points away from the extent
    let mut open = [true, true];
    for e in b.face_edges(fid) {
        let ed = b.edge(e);
        let (s0, s1) = (s(&b.vertex(ed.v0).p), s(&b.vertex(ed.v1).p));
        if (s0 - s1).abs() > 1e-6 * (hi - lo).max(1.0) {
            continue; // a seam
        }
        let end = if (s0 - lo).abs() < 1e-6 * (hi - lo).max(1.0) { 0 } else { 1 };
        for g in b.edge_faces(e) {
            if g == fid {
                continue;
            }
            if let Surface::Plane { .. } = b.face(g).surface {
                let np = b.planar_normal(g).dot(up);
                // toward the extent = a blind bottom / a shoulder: closed
                if (end == 0 && np > 0.5) || (end == 1 && np < -0.5) {
                    open[end] = false;
                }
            }
        }
    }
    Ok(CylFace { o, up, radius: *r, lo, hi, internal, open })
}

fn fmt(x: f64) -> String {
    let s = format!("{:.3}", x);
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

/// Parsed feature fields.
struct Req {
    face: String,
    standard: String,
    size: String,
    class: Option<String>,
    length: Option<f64>,
    left: bool,
    start: Option<bool>,
    modeled: bool,
}

fn parse(state: &RegenState, f: &Feature) -> Result<Req, FeatureError> {
    let s = |k: &str| f.raw.get(k).and_then(|v| v.as_str()).map(str::to_string);
    let face = s("face").filter(|x| !x.is_empty()).ok_or_else(missing)?;
    let length = match f.raw.get("length") {
        None | Some(Json::Null) => None,
        Some(Json::Str(t)) if t.trim().eq_ignore_ascii_case("full") || t.is_empty() => None,
        Some(_) => {
            let l = state.num(f, "length", 0.0)?;
            if !(l > 0.0) || !l.is_finite() {
                return Err(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("length".into())));
            }
            Some(l)
        }
    };
    let left = match s("hand").as_deref() {
        None | Some("right") => false,
        Some("left") => true,
        Some(_) => return Err(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("hand".into()))),
    };
    let start = match s("start").as_deref() {
        None => None,
        Some("top") => Some(true),
        Some("bottom") => Some(false),
        Some(_) => return Err(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("start".into()))),
    };
    Ok(Req {
        face,
        standard: s("standard").unwrap_or_default(),
        size: s("size").unwrap_or_default(),
        class: s("class").filter(|c| !c.is_empty()),
        length,
        left,
        start,
        modeled: crate::sketch::jtruthy(f.raw.get("modeled")),
    })
}

fn size_unknown(r: &Req) -> FeatureError {
    let known = threads::standard_id(&r.standard);
    let mut e = FeatureError::new(THREAD_SIZE_UNKNOWN)
        .with("standard", Json::Str(r.standard.clone()))
        .with("size", Json::Str(r.size.clone()));
    let sugg: Vec<Json> = if known.is_some() && !r.size.is_empty() {
        threads::suggestions(Some(&r.standard), &r.size, 5).into_iter().map(Json::Str).collect()
    } else if known.is_none() {
        threads::STANDARDS.iter().map(|s| Json::Str(s.0.to_string())).collect()
    } else {
        threads::sizes(&r.standard).iter().take(5).map(|t| Json::Str(t.designation.clone())).collect()
    };
    let hint = if known.is_none() {
        format!("unknown thread standard {:?}; use one of metric, metric-fine, UNC, UNF, BSP, NPT, Tr, Acme (request threadData lists them)", r.standard)
    } else {
        format!("{:?} is not a {} size; closest: {} (request threadData for the list)", r.size, known.unwrap_or(""), sugg.iter().filter_map(|j| j.as_str()).collect::<Vec<_>>().join(", "))
    };
    e = e.with("suggestions", Json::Arr(sugg)).with("hint", Json::Str(hint));
    e
}

/// The diameter check: `Some(warning)` when the face does not fit the thread.
fn diameter_warning(t: &ThreadSpec, cf: &CylFace) -> Option<FeatureError> {
    let actual = 2.0 * cf.radius;
    let (fits, expected, what) = if cf.internal {
        let by_tap = (actual - t.tap_drill).abs() <= DIAMETER_TOL * t.tap_drill;
        let by_minor = (actual - t.minor_internal).abs() <= DIAMETER_TOL * t.minor_internal;
        (by_tap || by_minor, t.tap_drill, "tap drill")
    } else {
        ((actual - t.major).abs() <= DIAMETER_TOL * t.major, t.major, "major diameter")
    };
    if fits {
        return None;
    }
    let nearest = threads::nearest_for_diameter(t.standard, actual, cf.internal);
    let (noun, fix) = if cf.internal { ("hole", "drill the hole") } else { ("shaft", "turn the shaft") };
    let mut hint = format!("{noun} is {}, {} {what} is {}: {fix} to {}", fmt(actual), t.designation, fmt(expected), fmt(expected));
    if let Some(n) = nearest {
        if n.designation != t.designation {
            let nref = if cf.internal { n.tap_drill } else { n.major };
            hint.push_str(&format!(", or choose {} ({} {})", n.designation, if cf.internal { "tap drill" } else { "major diameter" }, fmt(nref)));
        }
    }
    if t.tapered {
        hint.push_str(" (NPT is tapered 1:16; the straight size given is the tap drill at the small end)");
    }
    Some(
        FeatureError::new(THREAD_DIAMETER_MISMATCH)
            .with("expected", Json::Num(round3(expected)))
            .with("actual", Json::Num(round3(actual)))
            .with("size", Json::Str(t.designation.clone()))
            .with("standard", Json::Str(t.standard.to_string()))
            .with("internal", Json::Bool(cf.internal))
            .with("nearest", nearest.map_or(Json::Null, |n| Json::Str(n.designation.clone())))
            .with("hint", Json::Str(hint)),
    )
}

fn round3(x: f64) -> f64 {
    (x * 1000.0).round() / 1000.0
}

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(round6(p.x)), Json::Num(round6(p.y)), Json::Num(round6(p.z))])
}
fn round6(x: f64) -> f64 {
    (x * 1e6).round() / 1e6
}

/// The geometry the feature resolved, for the modeled build and the result entry.
pub struct Placement {
    /// Where the thread starts (on the axis), and the unit direction into the face.
    pub start: Vec3,
    pub dir: Vec3,
    pub length: f64,
    /// Is the start end an open end.
    pub start_open: bool,
    /// Is the far end an open end.
    pub far_open: bool,
}

fn place(cf: &CylFace, req: &Req) -> Placement {
    let full = cf.hi - cf.lo;
    let length = req.length.map_or(full, |l| l.min(full));
    let top = match req.start {
        Some(t) => t,
        None => match cf.open {
            [false, true] => true,
            [true, false] => false,
            _ => true,
        },
    };
    let (s, dir, start_open, far_open) = if top {
        (cf.hi, -cf.up, cf.open[1], cf.open[0])
    } else {
        (cf.lo, cf.up, cf.open[0], cf.open[1])
    };
    Placement { start: cf.o + cf.up * s, dir, length, start_open, far_open }
}

/// `thread`.
pub fn exec_thread(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let req = parse(state, f)?;
    let body = br.body.as_ref().ok_or_else(missing)?;
    let cf = cyl_face(body, &req.face)?;
    let spec = threads::find(&req.standard, &req.size).ok_or_else(|| size_unknown(&req))?;
    let pl = place(&cf, &req);
    let mut warnings = vec![];
    if let Some(w) = diameter_warning(spec, &cf) {
        warnings.push(w);
    }
    let mut modeled = false;
    if req.modeled {
        match model::apply(br, state, f, spec, &cf, &pl, req.left) {
            Ok(()) => modeled = true,
            Err(why) => warnings.push(
                FeatureError::new(THREAD_MODEL_FALLBACK)
                    .with("reason", Json::Str(why.clone()))
                    .with("hint", Json::Str(format!("{} stays a cosmetic thread: {why}", spec.designation))),
            ),
        }
    }
    let minor = if cf.internal { spec.minor_internal } else { spec.minor_external };
    let class = req.class.clone().unwrap_or_else(|| (if cf.internal { spec.class_internal } else { spec.class_external }).to_string());
    let entry = Obj::new()
        .put("id", f.id.as_str())
        .put("face", req.face.as_str())
        .put("standard", spec.standard)
        .put("size", spec.designation.as_str())
        .put("class", class.as_str())
        .put("pitch", round6(spec.pitch))
        .put("major", round6(spec.major))
        .put("minor", round6(minor))
        .put("pitchDiameter", round6(spec.pitch_dia))
        .put("angle", spec.angle)
        .put("length", round6(pl.length))
        .put("hand", if req.left { "left" } else { "right" })
        .put("internal", cf.internal)
        .put("modeled", modeled)
        .put("tapered", spec.tapered)
        .put("diameter", round6(2.0 * cf.radius))
        .put("start", v3(pl.start))
        .put("dir", v3(pl.dir))
        .build();
    state.result.put_thread(&f.id, entry);
    state.result.warnings.retain(|w| w.feature != f.id);
    for w in warnings {
        state.result.warnings.push(RegenError { feature: f.id.clone(), error: w });
    }
    Ok(())
}

pub mod model {
    //! Modeled threads (the real helical groove).
    use super::*;

    pub fn apply(
        _br: &mut Brep,
        _state: &mut RegenState,
        _f: &Feature,
        spec: &ThreadSpec,
        _cf: &CylFace,
        _pl: &Placement,
        _left: bool,
    ) -> Result<(), String> {
        if spec.tapered {
            return Err(format!("{} is a tapered (1:16) thread; modeled tapered threads are not built", spec.designation));
        }
        Err("modeled threads are not built yet".to_string())
    }
}
