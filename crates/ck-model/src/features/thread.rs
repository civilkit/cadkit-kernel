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
    let reaches = length >= full - 1e-9;
    let top = match req.start {
        Some(t) => t,
        None => match cf.open {
            [false, true] => true,
            [true, false] => false,
            _ => true,
        },
    };
    let (s, dir, start_open, far_open) = if top {
        (cf.hi, -cf.up, cf.open[1], cf.open[0] && reaches)
    } else {
        (cf.lo, cf.up, cf.open[0], cf.open[1] && reaches)
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
    //! Modeled threads: a helical groove (a trapezoid swept along a screw motion about the
    //! face axis) cut from the body.
    //!
    //! Groove width at radius r (axial), flank half-angle a = angle / 2:
    //! * internal: `g(r) = fm P + 2 (r_major - r) tan a`  (narrowest at the root, the major);
    //! * external: `g(r) = (1 - fm) P - 2 (r_major - r) tan a`  (widest at the crest, the major);
    //! with `fm` the flat fraction of the profile (P/8 ISO / UN, ...). SIMPLIFICATION: straight
    //! flanks and flat crests / roots, no rounded roots; the groove reaches from the minor (external)
    //! or from just inside the bore (internal) to the major (internal) or just outside the shaft
    //! (external).
    use super::*;
    use crate::features::extrude::Combine;
    use ck_geom::{Frame, Vec2};
    use ck_ops::sweep::{sweep_motion, Motion};
    use ck_topo::body::{FaceName, Role};
    use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    /// More turns than this are not modeled (the sweep and the boolean grow with the turns).
    pub const MAX_TURNS: f64 = 60.0;
    /// Axial width of the groove never exceeds this fraction of the pitch (turns must not touch).
    const MAX_WIDTH: f64 = 0.9;

    /// Flat fraction of the profile at the major diameter.
    fn flat(spec: &ThreadSpec) -> f64 {
        match spec.standard {
            "BSP" => 0.1665,
            "Tr" => 0.366,
            "Acme" => 0.3707,
            _ => 0.125,
        }
    }

    /// The groove cross-section (radial r, axial s) as four vertices, CCW:
    /// `[(r_a, -w_a/2), (r_b, -w_b/2), (r_b, +w_b/2), (r_a, +w_a/2)]`. Returns it with the axial
    /// half-width bound.
    pub struct Groove {
        pub pts: [(f64, f64); 4],
        /// Largest axial half-width.
        pub half: f64,
        /// Volume per pitch of axial length (ring integral of the groove width).
        pub per_pitch: f64,
    }

    /// Ring integral of the groove width over `[ra, rb]`: ∫ g(r) 2π r dr (exact for the linear g).
    fn ring(g: &dyn Fn(f64) -> f64, ra: f64, rb: f64) -> f64 {
        // g is linear in r: Simpson is exact for g(r) r
        let (ga, gm, gb) = (g(ra), g(0.5 * (ra + rb)), g(rb));
        let rm = 0.5 * (ra + rb);
        std::f64::consts::TAU * (rb - ra) / 6.0 * (ga * ra + 4.0 * gm * rm + gb * rb)
    }

    pub fn groove(spec: &ThreadSpec, internal: bool, radius: f64) -> Result<Groove, String> {
        let p = spec.pitch;
        let tn = (spec.angle.to_radians() * 0.5).tan();
        let fm = flat(spec);
        let r_maj = spec.major * 0.5;
        let eps = 0.1 * p;
        if internal {
            let g = |r: f64| fm * p + 2.0 * (r_maj - r) * tn;
            if radius >= r_maj - 0.02 * p {
                return Err(format!("the bore (diameter {}) is not smaller than the {} major diameter {}; there is nothing to cut", super::fmt(2.0 * radius), spec.designation, super::fmt(spec.major)));
            }
            let r_lim = r_maj - (MAX_WIDTH - fm) * p / (2.0 * tn);
            let r_in = (radius - eps).max(r_lim);
            if r_in > radius - 0.02 * p {
                return Err(format!(
                    "the bore (diameter {}) is too small for the groove of {} to reach it (needs a diameter of at least {})",
                    super::fmt(2.0 * radius), spec.designation, super::fmt(2.0 * (r_lim + 0.02 * p))
                ));
            }
            let (wa, wb) = (g(r_in), g(r_maj));
            Ok(Groove { pts: [(r_in, -wa / 2.0), (r_maj, -wb / 2.0), (r_maj, wb / 2.0), (r_in, wa / 2.0)], half: wa / 2.0, per_pitch: ring(&g, radius, r_maj) })
        } else {
            let g = |r: f64| (1.0 - fm) * p - 2.0 * (r_maj - r) * tn;
            let r_min = spec.minor_external * 0.5;
            // keep the root groove at least 0.1 P wide and the outer end under MAX_WIDTH
            let r_root = r_min.max(r_maj - ((1.0 - fm) * p - 0.1 * p) / (2.0 * tn));
            let r_lim = r_maj + (MAX_WIDTH - (1.0 - fm)) * p / (2.0 * tn);
            let r_out = (radius + eps).min(r_lim);
            if r_out < radius + 0.02 * p {
                return Err(format!(
                    "the shaft (diameter {}) is too far over the {} major diameter {} for the groove to clear it",
                    super::fmt(2.0 * radius), spec.designation, super::fmt(spec.major)
                ));
            }
            if r_out <= r_root + 0.05 * p {
                return Err(format!("the shaft (diameter {}) is smaller than the {} root diameter; there is nothing to cut", super::fmt(2.0 * radius), spec.designation));
            }
            let (wa, wb) = (g(r_root), g(r_out));
            Ok(Groove { pts: [(r_root, -wa / 2.0), (r_out, -wb / 2.0), (r_out, wb / 2.0), (r_root, wa / 2.0)], half: wb.max(wa) / 2.0, per_pitch: ring(&g, r_root, radius.min(r_out)) })
        }
    }

    fn radial_for(d: Vec3) -> Vec3 {
        // the world axis least aligned with d, made perpendicular
        let a = if d.x.abs() <= d.y.abs() && d.x.abs() <= d.z.abs() {
            Vec3::new(1.0, 0.0, 0.0)
        } else if d.y.abs() <= d.z.abs() {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            Vec3::new(0.0, 0.0, 1.0)
        };
        (a - d * a.dot(d)).normalize()
    }

    /// Build the groove solid for a thread; faces named `<id>.T<k>`.
    pub fn tool(id: &str, spec: &ThreadSpec, cf: &CylFace, pl: &Placement, left: bool) -> Result<(Body, Groove), String> {
        let gr = groove(spec, cf.internal, cf.radius)?;
        let p = spec.pitch;
        let d = pl.dir;
        // centre line of the helix along the axis: from s0 to s1 (distance from the start end into the face)
        let margin = 0.05 * p;
        let s0 = if pl.start_open { -(gr.half + 0.5 * p) } else { gr.half + margin };
        let s1 = if pl.far_open { pl.length + gr.half + 0.5 * p } else { pl.length - gr.half - margin };
        let travel = s1 - s0;
        if !(travel > 0.5 * p) {
            return Err(format!("the thread length {} is shorter than one groove ({})", super::fmt(pl.length), super::fmt(2.0 * gr.half + 0.5 * p)));
        }
        let turns = travel / p;
        if turns > MAX_TURNS {
            return Err(format!("{} turns; modeled threads are limited to {} turns (shorten the length)", turns.round(), MAX_TURNS));
        }
        let o = pl.start + d * s0;
        let x = radial_for(d);
        let fr = Frame { o, x, y: d, z: x.cross(d) };
        let segs: Vec<ProfileSeg> = (0..4)
            .map(|i| {
                let (a, b) = (gr.pts[i], gr.pts[(i + 1) % 4]);
                ProfileSeg::line(&format!("t{i}"), Vec2::new(a.0, a.1), Vec2::new(b.0, b.1))
            })
            .collect();
        let prof = Profile::new(fr, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized();
        let angle = std::f64::consts::TAU * turns * if left { -1.0 } else { 1.0 };
        let mut body = sweep_motion(id, &prof, Motion::Screw { o, a: d, angle, rise: travel })?;
        for f in body.faces.iter_mut().filter(|f| f.alive) {
            let k = match &f.name.role {
                Role::Side(t) => t.trim_start_matches('t').parse::<u32>().unwrap_or(0),
                Role::Cap(k) => 4 + *k,
                _ => continue,
            };
            f.name = FaceName::other(id, &format!("T{k}"));
        }
        Ok((body, gr))
    }

    pub fn apply(br: &mut Brep, _state: &mut RegenState, f: &Feature, spec: &ThreadSpec, cf: &CylFace, pl: &Placement, left: bool) -> Result<(), String> {
        if spec.tapered {
            return Err(format!("{} is a tapered (1:16) thread; modeled tapered threads are not built", spec.designation));
        }
        let before = br.body.clone().ok_or("no solid")?;
        let v0 = ck_topo::mass::mass_props(&before).volume;
        let (tool, gr) = tool(&f.id, spec, cf, pl, left)?;
        let r = br.combine(&tool, Combine::Cut, false);
        let fail = |br: &mut Brep, why: String| {
            br.body = Some(before.clone());
            Err(why)
        };
        if let Err(e) = r {
            return fail(br, format!("the groove could not be cut ({})", e.describe()));
        }
        let after = br.body.as_ref().unwrap();
        let v1 = ck_topo::mass::mass_props(after).volume;
        // the removed volume must be close to the analytic groove (per pitch x turns of the threaded length)
        let want = gr.per_pitch * pl.length / spec.pitch;
        let dv = v0 - v1;
        if !(dv > 0.4 * want && dv < 1.6 * want) {
            return fail(br, format!("the cut removed {} mm3, expected about {}", super::fmt(dv), super::fmt(want)));
        }
        if ck_check::validate(after, 1e-6).len() > ck_check::validate(&before, 1e-6).len() {
            return fail(br, "the cut produced an invalid solid".to_string());
        }
        if after.num_lumps() != before.num_lumps() {
            return fail(br, "the cut split the body".to_string());
        }
        Ok(())
    }
}
