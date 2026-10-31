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
//!   appended; ours: a hole is united only with the tools whose own boxes
//!   meet its box, so a grid of clear holes costs no union) then
//!   `boolean Subtract (body × tool)`; a hole that misses -> `cutNoEffect`;
//! * section loop runs top -> axis -> bottom in the plane (sketch x, up)
//!   (pinned by the tool face order h2 before h1 in the result);
//! * radii come out of world coordinates (`1.6999999999999993` for M3
//!   normal at x = 10): the section is laid out along the sketch x axis
//!   from the sketch origin.
//! * (ours, not in the oracle) a plain `diameter` also serves a counterbore / countersink when no table `size`
//!   applies: with the step (`cboreDiameter` + `cboreDepth`) or cone (`cskDiameter`) given it is built as asked;
//!   with any of them missing it is drilled as a simple hole and a `holeDowngraded` WARNING says what was left out.
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
/// `holeCskDiameter {cskDiameter, holeDiameter, hint}`: countersink head diameter / angle impossible; a cone deeper
/// than the hole also carries `maxSize` (cskDiameter).
pub const HOLE_CSK_DIAMETER: &str = "holeCskDiameter";
/// `holeCboreInvalid {hint, ..}`: counterbore diameter / depth impossible; a step as deep as the hole also carries
/// `maxSize` (cboreDepth).
pub const HOLE_CBORE_INVALID: &str = "holeCboreInvalid";
/// WARNING `holeDowngraded {requested, built, missing, detail, hint}`: a counterbore / countersink with a plain
/// `diameter` but without the dimensions of its step or cone is drilled as a simple hole instead of being refused.
pub const HOLE_DOWNGRADED: &str = "holeDowngraded";

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
    // GUESS (not an oracle option): a plain `diameter` instead of a standard `size`. A simple hole always may;
    // a counterbore / countersink may when no table size applies, and then its step or cone must be given
    // explicitly (`cboreDiameter` + `cboreDepth`, `cskDiameter`), or it is drilled as a simple hole with a
    // `holeDowngraded` warning (below).
    let table_row = size.as_deref().and_then(|s| ISO.iter().find(|(n, _)| *n == s)).map(|r| r.1);
    let plain = ((*kind == HoleKind::Simple || table_row.is_none()) && f.raw.get("diameter").is_some()).then(|| state.num(f, "diameter", 0.0)).transpose()?;
    let row = match (plain, table_row) {
        (Some(_), r) => r.unwrap_or([0.0; 7]),
        (None, r) => r.ok_or_else(|| {
            let (kname, give) = match kind {
                HoleKind::Simple => ("simple", "give a standard size (e.g. M6), or a plain diameter"),
                HoleKind::Cbore => ("counterbore", "give a standard size (e.g. M6), or a diameter with cboreDiameter and cboreDepth"),
                HoleKind::Countersink | HoleKind::Csink => ("countersink", "give a standard size (e.g. M6), or a diameter with cskDiameter (and cskAngle)"),
            };
            hint(
                FeatureError::new(HOLE_INVALID).with("detail", ck_json::Json::Str(format!("the {kname} hole has no size for the standard table"))),
                give,
            )
        })?,
    };
    // what the hole is really built as: a counterbore / countersink without its explicit dimensions (and without a
    // table row to take them from) falls back to a simple hole of the given diameter
    let mut kind_eff = *kind;
    let mut downgrade: Option<FeatureError> = None; // pushed once the sketch borrow ends
    if plain.is_some() && table_row.is_none() {
        let missing: Vec<&str> = match kind {
            HoleKind::Cbore => ["cboreDiameter", "cboreDepth"].into_iter().filter(|k| !given(f, k)).collect(),
            HoleKind::Countersink | HoleKind::Csink => ["cskDiameter"].into_iter().filter(|k| !given(f, k)).collect(),
            HoleKind::Simple => vec![],
        };
        if !missing.is_empty() {
            let requested = if *kind == HoleKind::Cbore { "counterbore" } else { "countersink" };
            let w = FeatureError::new(HOLE_DOWNGRADED)
                .with("requested", ck_json::Json::Str(requested.into()))
                .with("built", ck_json::Json::Str("simple".into()))
                .with("missing", ck_json::Json::Arr(missing.iter().map(|m| ck_json::Json::Str((*m).into())).collect()))
                .with("detail", ck_json::Json::Str(format!("the {requested} has no {} to build it from, so a plain hole of the given diameter was drilled", missing.join(" or "))));
            downgrade = Some(hint(w, "give the missing dimension(s), or a standard size such as M6, to get the stepped hole"));
            kind_eff = HoleKind::Simple;
        }
    }
    let kind = &kind_eff;
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
    if let Some(w) = downgrade {
        state.result.warnings.retain(|x| x.feature != f.id);
        state.result.warnings.push(crate::regen::RegenError { feature: f.id.clone(), error: w });
    }
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
    // the shortest hole of all the points: the bound of a counterbore depth / countersink cone (`maxSize`)
    let shortest = || {
        pts.iter()
            .map(|p| if through { through_len(fr.o + fr.x * p[0] + fr.y * p[1], &body) } else { depth })
            .fold(f64::INFINITY, f64::min)
    };
    let mut parts: Vec<(Body, (ck_core::linalg::Vec3, ck_core::linalg::Vec3))> = Vec::new();
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
                let e = hint(
                    FeatureError::new(HOLE_CBORE_INVALID).with("cboreDepth", num(cd)).with("holeDepth", num(len)),
                    "the counterbore must be shallower than the hole: reduce cboreDepth or increase depth",
                );
                // the same test, solved for the depth
                let max = super::max_size::floor_nice(shortest() - 1e-6);
                return Err(if max > 0.0 { super::max_size::with_max_size(e, "cboreDepth", max, "mm") } else { e });
            }
            segs.push(line(Vec2::new(u, 0.0), Vec2::new(u + cr, 0.0), "h0"));
            segs.push(line(Vec2::new(u + cr, 0.0), Vec2::new(u + cr, -(cd)), "h1"));
            segs.push(line(Vec2::new(u + cr, -(cd)), Vec2::new(u + r, -(cd)), "h2"));
            rw = cd;
            top_tag = "h3";
        } else if let Some((hr, half)) = csk {
            let cd = (hr - r) / half.tan();
            if cd >= len - 1e-9 {
                let e = hint(
                    FeatureError::new(HOLE_CSK_DIAMETER).with("cskDiameter", num(hr * 2.0)).with("coneDepth", num(cd)).with("holeDepth", num(len)),
                    "the countersink cone is deeper than the hole: reduce cskDiameter, increase cskAngle or increase depth",
                );
                // the same test, solved for the head diameter: (head / 2 - r) / tan(half) < shortest
                let max = super::max_size::floor_nice(2.0 * (r + (shortest() - 1e-6) * half.tan()));
                return Err(if max > 2.0 * r { super::max_size::with_max_size(e, "cskDiameter", max, "mm") } else { e });
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
        // Unite the new hole only with the tools whose own boxes meet its box (repeating while the union grows
        // into more of them); tools clear of it stay separate shells. Folding every hole into one growing union
        // made each step cost the whole tool so far, and a grid of holes meets the accumulated box at once.
        let mut acc = b;
        loop {
            let (alo, ahi) = super::body_box(&acc);
            let e = super::TOL;
            let hit = parts.iter().position(|(_, (lo, hi))| !(alo.x > hi.x + e || lo.x > ahi.x + e || alo.y > hi.y + e || lo.y > ahi.y + e || alo.z > hi.z + e || lo.z > ahi.z + e));
            match hit {
                Some(i) => {
                    let (t, _) = parts.remove(i);
                    acc = super::pattern::accumulate(br, t, acc)?;
                }
                None => {
                    parts.push((acc, (alo, ahi)));
                    break;
                }
            }
        }
    }
    let mut it = parts.into_iter().map(|p| p.0);
    let mut whole = it.next().unwrap();
    for p in it {
        whole.append(&p);
    }
    let tool = Tool { body: whole, combine: Combine::Cut, recipe: None };
    let r = br.combine(&tool.body, Combine::Cut, true);
    br.tools.push((f.id.clone(), std::rc::Rc::new(tool)));
    r
}
