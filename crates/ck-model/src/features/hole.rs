//! `hole` (ck-model/src/features/hole.rs).
//!
//! Pinned by oracle probes (tools/asm/holecases.mjs: points on the top cap
//! of a 40×10×20 box):
//! * one tool per sketch `point` entity: a revolve of the hole section
//!   about the axis through the point, INTO the face (`−z` of the sketch);
//!   faces `S.h0` (top disc), `S.h1` (wall), `S.h2` (bottom: plane, or a
//!   118° drill-point cone with `tip`), cbore: `h0` top, `h1` cbore wall,
//!   `h2` step, `h3` hole wall, `h4` bottom; with several points every face
//!   carries the point index `~k` (`F4.S.h1~3`);
//! * diameters = the client table `So` (app.pretty.js): fits
//!   tap/close/normal/loose for `simple`, close/normal/loose for `cbore` /
//!   `csk`, anything else -> `normal`; cbore `[diameter, depth]`; unknown
//!   `size`, or no `size` at all (a non-simple hole needs one: a free
//!   `diameter` alone is `holeInvalid {detail, hint}` saying size vs
//!   diameter) -> `holeInvalid`; no points -> `holeNoPoints`; `kind:"csink"`
//!   without `cskDiameter` behaves as simple (legacy); with it, or as
//!   `countersink`, a cone `K0` is cut (head diameter, `cskAngle` included
//!   angle, default 90); `cboreDiameter` / `cboreDepth` override the table;
//! * the tools are combined into one (`boolean Unite (9 × 3 faces)` for the
//!   4th of four holes whose box meets the others'; disjoint boxes are
//!   appended) then `boolean Subtract (body × tool)`; a hole that misses ->
//!   `cutNoEffect`;
//! * section loop runs top -> axis -> bottom in the plane (sketch x, up)
//!   (pinned by the tool face order h2 before h1 in the result);
//! * radii come out of world coordinates (`1.6999999999999993` for M3
//!   normal at x = 10): the section is laid out along the sketch x axis
//!   from the sketch origin.
//! GUESS: csk (90° cone) geometry, flip, `through` length = the extrude
//! `through` rule.

use super::extrude::{through_len, Combine, Tool};
use super::{Brep, FeatureKind, HoleKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::sketch::SketchEntity;
use crate::Feature;
use ck_core::linalg::Frame;
use ck_geom::curve2::Curve2;
use ck_geom::Vec2;
use ck_topo::body::Body;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// `holeInvalid {detail, hint}`: no usable size (the hint names size vs diameter).
pub const HOLE_INVALID: &str = "holeInvalid";
pub const HOLE_NO_POINTS: &str = "holeNoPoints";
/// `holeCskDiameter {cskDiameter, holeDiameter, hint}`: countersink head diameter / angle impossible.
pub const HOLE_CSK_DIAMETER: &str = "holeCskDiameter";
/// `holeCboreInvalid {hint, ..}`: counterbore diameter / depth impossible.
pub const HOLE_CBORE_INVALID: &str = "holeCboreInvalid";

fn given(f: &Feature, key: &str) -> bool {
    f.raw.get(key).is_some_and(|v| !matches!(v, ck_json::Json::Null))
}
fn num(v: f64) -> ck_json::Json {
    ck_json::Json::Num(v)
}
fn hint(e: FeatureError, s: &str) -> FeatureError {
    e.with("hint", ck_json::Json::Str(s.to_string()))
}

/// `(tap, close, normal, loose, cbore diameter, cbore depth, csk diameter)`.
const ISO: &[(&str, [f64; 7])] = &[
    ("M2", [1.6, 2.2, 2.4, 2.6, 4.3, 2.3, 4.4]),
    ("M2.5", [2.05, 2.7, 2.9, 3.1, 5.0, 2.8, 5.5]),
    ("M3", [2.5, 3.2, 3.4, 3.6, 6.5, 3.4, 6.5]),
    ("M4", [3.3, 4.3, 4.5, 4.8, 8.0, 4.4, 8.6]),
    ("M5", [4.2, 5.3, 5.5, 5.8, 10.0, 5.4, 10.4]),
    ("M6", [5.0, 6.4, 6.6, 7.0, 11.0, 6.4, 12.4]),
    ("M8", [6.8, 8.4, 9.0, 10.0, 15.0, 8.6, 16.4]),
    ("M10", [8.5, 10.5, 11.0, 12.0, 18.0, 10.6, 20.4]),
    ("M12", [10.2, 13.0, 13.5, 14.5, 20.0, 12.6, 24.4]),
    ("M16", [14.0, 17.0, 17.5, 18.5, 26.0, 16.6, 32.4]),
    ("M20", [17.5, 21.0, 22.0, 24.0, 33.0, 20.6, 40.4]),
    ("M24", [21.0, 25.0, 26.0, 28.0, 40.0, 24.8, 48.4]),
];

/// Half angle of the drill point (118° included).
const TIP_HALF: f64 = 59.0;

fn line(a: Vec2, b: Vec2, tag: &str) -> ProfileSeg {
    let len = (b - a).len();
    ProfileSeg { tag: tag.to_string(), curve: Curve2::line_through(a, b), t0: 0.0, t1: len, rev: false }
}

/// `hole`.
pub fn exec_hole(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Hole { size, fit, kind, tip, flip, end, .. } = &f.kind else { unreachable!() };
    let sk = f.raw.get("sketch").and_then(|s| s.as_str()).and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    // GUESS (not an oracle option): a simple hole may give a plain `diameter`
    // instead of a standard `size`.
    let plain = (*kind == HoleKind::Simple && f.raw.get("diameter").is_some()).then(|| state.num(f, "diameter", 0.0)).transpose()?;
    let row = match (plain, size.as_deref().and_then(|s| ISO.iter().find(|(n, _)| *n == s)).map(|r| r.1)) {
        (Some(_), r) => r.unwrap_or([0.0; 7]),
        (None, r) => r.ok_or_else(|| {
            let kname = match kind {
                HoleKind::Simple => "simple",
                HoleKind::Cbore => "counterbore",
                HoleKind::Countersink | HoleKind::Csink => "countersink",
            };
            let e = FeatureError::new(HOLE_INVALID)
                .with("detail", ck_json::Json::Str(format!("the {kname} hole has no size for the standard table")));
            if given(f, "diameter") {
                hint(
                    e,
                    "a counterbore takes its dimensions from the size table, not from diameter: give a size (e.g. M6), or set kind to \"simple\" to drill the diameter",
                )
            } else {
                hint(e, "give a standard size (e.g. M6), or a plain diameter with kind \"simple\"")
            }
        })?,
    };
    let fit_s = f.raw.get("fit").and_then(|v| v.as_str()).unwrap_or("");
    let _ = fit;
    let d = match (plain, kind, fit_s) {
        (Some(d), ..) => d,
        (_, HoleKind::Simple, "tap") => row[0],
        (_, _, "close") => row[1],
        (_, _, "loose") => row[3],
        _ => row[2],
    };
    if d <= 0.0 {
        return Err(hint(FeatureError::new(HOLE_INVALID), "the hole diameter must be greater than zero (a standard `size`, or a positive `diameter`)"));
    }
    let pts: Vec<[f64; 2]> = sk
        .sketch
        .entities
        .iter()
        .filter_map(|e| match e {
            SketchEntity::Point { p, .. } => sk.sketch.point_xy(p),
            _ => None,
        })
        .collect();
    if pts.is_empty() {
        return Err(FeatureError::new(HOLE_NO_POINTS));
    }
    let Some(body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let fr = sk.frame;
    let down = if *flip { fr.z } else { -fr.z };
    let r = d / 2.0;
    // counterbore: explicit fields beat the table (absent: table, as before)
    let (mut cbore_d, mut cbore_h) = (row[4], row[5]);
    let mut cbore_custom = false;
    let mut csk: Option<(f64, f64)> = None; // (head radius, half angle rad)
    if *kind == HoleKind::Cbore {
        let (gd, gh) = (given(f, "cboreDiameter"), given(f, "cboreDepth"));
        if gd {
            cbore_d = state.num(f, "cboreDiameter", 0.0)?;
        }
        if gh {
            cbore_h = state.num(f, "cboreDepth", 0.0)?;
        }
        cbore_custom = gd || gh;
        if cbore_custom && (cbore_d <= d + 1e-9 || cbore_h <= 0.0) {
            return Err(hint(
                FeatureError::new(HOLE_CBORE_INVALID).with("cboreDiameter", num(cbore_d)).with("cboreDepth", num(cbore_h)).with("holeDiameter", num(d)),
                "the counterbore diameter must be larger than the hole diameter and its depth positive",
            ));
        }
    } else if *kind == HoleKind::Countersink || (*kind == HoleKind::Csink && given(f, "cskDiameter")) {
        let head = if given(f, "cskDiameter") { state.num(f, "cskDiameter", 0.0)? } else { row[6] };
        let ang = if given(f, "cskAngle") { state.num(f, "cskAngle", 90.0)? } else { 90.0 };
        if head <= d + 1e-9 {
            return Err(hint(
                FeatureError::new(HOLE_CSK_DIAMETER).with("cskDiameter", num(head)).with("holeDiameter", num(d)),
                "the countersink head diameter must be larger than the hole diameter; set cskDiameter (for a plain hole use kind simple)",
            ));
        }
        if !(ang > 0.0 && ang < 180.0) {
            return Err(hint(FeatureError::new(HOLE_CSK_DIAMETER).with("cskAngle", num(ang)), "cskAngle is the included cone angle in degrees, between 0 and 180 (typically 90 or 82)"));
        }
        csk = Some((head / 2.0, (ang / 2.0).to_radians()));
    }
    let depth = state.num(f, "depth", 0.0)?;
    let through = *end != super::End::Blind;
    let multi = pts.len() > 1;
    let mut tool: Option<Body> = None;
    for (k, p) in pts.iter().enumerate() {
        let pw = fr.o + fr.x * p[0] + fr.y * p[1];
        let len = if through { through_len(pw, &body) } else { depth };
        // section plane: through the hole axis, x = sketch x, y = down
        let sec = Frame { o: fr.o + fr.y * p[1], x: fr.x, y: -down, z: fr.x.cross(-down) };
        let u = p[0];
        let mut segs = vec![];
        let (rw, top_tag);
        if *kind == HoleKind::Cbore {
            let (cr, cd) = (cbore_d / 2.0, cbore_h);
            if cbore_custom && cd >= len - 1e-9 {
                return Err(hint(
                    FeatureError::new(HOLE_CBORE_INVALID).with("cboreDepth", num(cd)).with("holeDepth", num(len)),
                    "the counterbore must be shallower than the hole: reduce cboreDepth or increase depth",
                ));
            }
            segs.push(line(Vec2::new(u, 0.0), Vec2::new(u + cr, 0.0), "h0"));
            segs.push(line(Vec2::new(u + cr, 0.0), Vec2::new(u + cr, -(cd)), "h1"));
            segs.push(line(Vec2::new(u + cr, -(cd)), Vec2::new(u + r, -(cd)), "h2"));
            rw = cd;
            top_tag = "h3";
        } else if let Some((hr, half)) = csk {
            let cd = (hr - r) / half.tan();
            if cd >= len - 1e-9 {
                return Err(hint(
                    FeatureError::new(HOLE_CSK_DIAMETER).with("cskDiameter", num(hr * 2.0)).with("coneDepth", num(cd)).with("holeDepth", num(len)),
                    "the countersink cone is deeper than the hole: reduce cskDiameter, increase cskAngle or increase depth",
                ));
            }
            segs.push(line(Vec2::new(u, 0.0), Vec2::new(u + hr, 0.0), "h0"));
            segs.push(line(Vec2::new(u + hr, 0.0), Vec2::new(u + r, -cd), "K0"));
            rw = cd;
            top_tag = "h1";
        } else {
            segs.push(line(Vec2::new(u, 0.0), Vec2::new(u + r, 0.0), "h0"));
            rw = 0.0;
            top_tag = "h1";
        }
        let bot = if *kind == HoleKind::Cbore { "h4" } else { "h2" };
        segs.push(line(Vec2::new(u + r, -(rw)), Vec2::new(u + r, -(len)), top_tag));
        if *tip && !through {
            let h = r / TIP_HALF.to_radians().tan();
            segs.push(line(Vec2::new(u + r, -(len)), Vec2::new(u, -(len + h)), bot));
            segs.push(line(Vec2::new(u, -(len + h)), Vec2::new(u, 0.0), "axis"));
        } else {
            segs.push(line(Vec2::new(u + r, -(len)), Vec2::new(u, -(len)), bot));
            segs.push(line(Vec2::new(u, -(len)), Vec2::new(u, 0.0), "axis"));
        }
        let prof = Profile::new(sec, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized();
        let mut b = revolve(&f.id, &prof, pw, down, 0.0, std::f64::consts::TAU).map_err(|e| FeatureError::new(&e))?;
        if multi {
            for face in b.faces.iter_mut() {
                face.name.region = Some(k as u32 + 1);
            }
        }
        tool = Some(match tool {
            None => b,
            Some(t) => super::pattern::accumulate(br, t, b)?,
        });
    }
    let tool = Tool { body: tool.unwrap(), combine: Combine::Cut, recipe: None };
    let r = br.combine(&tool.body, Combine::Cut, true);
    br.tools.push((f.id.clone(), std::rc::Rc::new(tool)));
    r
}
