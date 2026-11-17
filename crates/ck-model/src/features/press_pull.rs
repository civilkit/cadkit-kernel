//! `pressPull`: continue a flat face straight out of (or into) the solid, as if the face were extruded.
//!
//! ```json
//! {"type":"pressPull","id":"F9","faces":["F2.S.e5"],"continue":"straight","distance":10,"taper":{"type":"scale","endScale":1.1}}
//! {"type":"pressPull","id":"F9","faces":["F2.S.e5"],"continue":"straight","distance":-10,"taper":{"type":"draft","angle":-3}}
//! ```
//!
//! * `faces`: flat faces of the solid by persistent name (a face split into `<id>~k` pieces is named by its base id; every piece
//!   continues). Required (`noFaces`), each must resolve (`missingReference`). Each face continues on its own, in order.
//! * `continue`: `"straight"` (default, the only value so far): the face is carried along its outward normal, unturned.
//!   `"along"` (the end follows a bend in the walls) is not built yet: `invalidValue {field: "continue"}`.
//! * `distance` (mm or `"=expr"`, not 0): positive runs out of the material and the new piece is joined, negative runs into it
//!   and the piece is cut.
//! * `taper`: `{type: "none" | "draft" | "scale"}` (default none).
//!   * `draft`: `angle` in degrees, -85..85. Each new wall leans from its edge on the face by `t * tan(angle)` at run length
//!     `t`, outward positive: the outline grows, holes shrink (the sign convention is the reverse of `extrude`'s `draft`, which
//!     tapers inward). The taper is built as `extrude` builds a draft, with the same limits and corner behaviour.
//!   * `scale`: `endScale` > 0 (1.1 = 110%). The outline, holes included, scales about the face's area centre, linearly with
//!     the run length, to `endScale` at the end (the walls are planes / cones as in `extrude` `endScale`).
//! * The moved end keeps the face's own persistent name, so sketches and features that referred to the face still find it. The
//!   new walls are named as an extrusion of the face names them: `<id>.S.<tag>`, the tag made from the name of the face across
//!   each edge (`F2.S.e1` gives `F2_S_e1`). The cap on the old face is consumed.
//! * Errors (the model is unchanged on any): `nothingToCut` (no solid), `noFaces`, `missingReference`, `invalidValue {field,
//!   detail, hint}` (distance 0 or not finite, `endScale` not above 0, `angle` out of range, unknown `taper.type`, `continue`
//!   other than straight), `pressPullUnsupported {detail}` (not a flat face, a face of a surface body), `draftTooLarge`
//!   (a draft that closes the face before the end of the run, with `maxHeight` for a round face, else `maxSize` in degrees),
//!   `moveFaceFailed {detail}` with `maxSize` (distance) where a smaller run builds: a cut through the body, a join into other
//!   material, a result that is not valid or splits the body.
//!
//! The `faceRun` query ([`face_run`]) gives what a panel needs: area centre, outward normal, reach, and whether the face is the
//! moving end of an `extrude` / `revolve` / `sweep`.

use super::move_face::{apply_prism, face_profile, failed, resolve, run_heights, unsupported};
use super::{Brep, TOL};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop};

/// How the new walls lean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Taper {
    None,
    /// Degrees, outward positive.
    Draft(f64),
    EndScale(f64),
}

/// Largest draft angle (degrees) `taper.angle` accepts, as `extrude` caps its draft.
pub const MAX_DRAFT: f64 = 85.0;

fn bad(field: &str, detail: impl Into<String>, hint: impl Into<String>) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE)
        .with("field", Json::Str(field.into()))
        .with("detail", Json::Str(detail.into()))
        .with("hint", Json::Str(hint.into()))
}

fn parse_taper(state: &RegenState, f: &Feature) -> Result<Taper, FeatureError> {
    let Some(t) = f.raw.get("taper").filter(|t| !matches!(t, Json::Null)) else { return Ok(Taper::None) };
    let num = |key: &str| crate::expr::eval_field(t.get(key), &state.vars, f64::NAN).map_err(FeatureError::new);
    match t.get_str("type").unwrap_or("none") {
        "none" => Ok(Taper::None),
        "draft" => {
            let a = num("angle")?;
            if !a.is_finite() {
                return Err(bad("taper.angle", "a draft taper needs an angle in degrees", "give taper.angle between -85 and 85"));
            }
            if a.abs() > MAX_DRAFT {
                return Err(bad("taper.angle", format!("draft angle {a} is outside -{MAX_DRAFT}..{MAX_DRAFT} degrees"), "use an angle between -85 and 85"));
            }
            Ok(if a == 0.0 { Taper::None } else { Taper::Draft(a) })
        }
        "scale" => {
            let k = num("endScale")?;
            if !k.is_finite() || k <= 0.0 {
                return Err(bad("taper.endScale", format!("endScale {k} is not a positive number"), "use an endScale above 0 (1 = no scaling, 0.5 = half size at the end)"));
            }
            if k < ck_ops::scaled_extrude::MIN_SCALE {
                return Err(bad("taper.endScale", format!("endScale {k:e} makes the end a point"), "use an endScale of at least 1e-6"));
            }
            Ok(if k == 1.0 { Taper::None } else { Taper::EndScale(k) })
        }
        other => Err(bad("taper.type", format!("unknown taper type {other}"), "use none, draft or scale")),
    }
}

/// Signed area and first moments of a closed profile loop (Green's theorem on the exact curves): `(A, A*cx, A*cy)`.
fn loop_moments(l: &ProfileLoop) -> (f64, f64, f64) {
    let (mut a, mut mx, mut my) = (0.0, 0.0, 0.0);
    for s in &l.segs {
        let (ta, tb) = (s.t_start(), s.t_end());
        let n = 16;
        for k in 0..n {
            let (p, q) = (ta + (tb - ta) * k as f64 / n as f64, ta + (tb - ta) * (k + 1) as f64 / n as f64);
            let (m, h) = ((p + q) * 0.5, (q - p) * 0.5);
            for &(x, w) in ck_topo::mass::GL.iter() {
                let d = s.curve.derivs(m + h * x, 1);
                let (pos, dt) = (d[0], d[1]);
                a += 0.5 * w * h * pos.cross(dt);
                mx += 0.5 * w * h * pos.x * pos.x * dt.y;
                my -= 0.5 * w * h * pos.y * pos.y * dt.x;
            }
        }
    }
    (a, mx, my)
}

/// The area centre of a profile in its own coordinates (holes taken out).
fn area_centre(p: &Profile) -> Option<Vec2> {
    let (mut a, mut mx, mut my) = (0.0, 0.0, 0.0);
    for r in &p.regions {
        for l in r.loops() {
            let (la, lx, ly) = loop_moments(l);
            a += la;
            mx += lx;
            my += ly;
        }
    }
    (a.abs() > 1e-12).then(|| Vec2::new(mx / a, my / a))
}

/// Farthest distance from `c` to the outer outline (sampled; exact at every vertex).
fn reach_of(p: &Profile, c: Vec2) -> f64 {
    let mut r: f64 = 0.0;
    for reg in &p.regions {
        for s in &reg.outer.segs {
            let n = if matches!(s.curve, Curve2::Line { .. }) { 1 } else { 720 };
            for k in 0..=n {
                let t = s.t0 + (s.t1 - s.t0) * k as f64 / n as f64;
                r = r.max(s.curve.eval(t).dist(c));
            }
        }
    }
    r
}

/// The tool: the face outline run `d` along its outward normal with the taper, first end on the face.
fn build_tool(profile: &Profile, n_out: Vec3, centre: Vec2, d: f64, taper: Taper, fid: &str) -> Result<Body, FeatureError> {
    let (s0, s1) = run_heights(profile.frame.z, n_out, d);
    let o = ExtrudeOpts { s0, s1, dir: None, draft: 0.0 };
    let built = match taper {
        Taper::None => extrude(fid, profile, o).map_err(|e| failed(format!("the run could not be built ({e})"))),
        Taper::EndScale(k) => ck_ops::scaled_extrude::scaled_extrude(fid, profile, o, k, centre)
            .map_err(|e| failed(format!("the scaled run could not be built ({e})"))),
        // extrude drafts inward for a positive angle, a press pull outward
        Taper::Draft(a) => return super::draft::drafted_extrude(fid, profile, ExtrudeOpts { draft: -a.to_radians(), ..o }),
    };
    built
}

fn pull_one(body: &Body, face: FaceId, d: f64, taper: Taper, fid: &str) -> Result<Body, FeatureError> {
    let shown = body.face_id_str(face);
    let Some((profile, n_out)) = face_profile(body, face) else {
        return Err(unsupported(format!("{shown} is not a flat face with closed outlines")));
    };
    let centre = area_centre(&profile).ok_or_else(|| failed(format!("{shown} has no area")))?;
    let tool = build_tool(&profile, n_out, centre, d, taper, fid)?;
    apply_prism(body, face, &tool, d, fid)
}

pub fn exec_press_pull(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let names: Vec<String> = f
        .raw
        .get("faces")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    if names.is_empty() {
        return Err(FeatureError::new("noFaces"));
    }
    match f.raw.get_str("continue").unwrap_or("straight") {
        "straight" => {}
        "along" => return Err(bad("continue", "continuing along the bend of the walls is not built yet", "use continue \"straight\"")),
        other => return Err(bad("continue", format!("unknown continue {other}"), "use \"straight\"")),
    }
    let d = state.num(f, "distance", 0.0)?;
    if !d.is_finite() {
        return Err(bad("distance", "distance is not a finite number", "give a distance in mm"));
    }
    if d == 0.0 {
        return Err(bad("distance", "a distance of 0 moves nothing", "give a positive distance to add material or a negative one to cut"));
    }
    let taper = parse_taper(state, f)?;
    let ids = resolve(br, &names).map_err(|e| {
        if e.code == "moveFaceUnsupported" {
            let mut n = FeatureError::new("pressPullUnsupported");
            n.params = e.params;
            n
        } else {
            e
        }
    })?;
    let body = br.body.as_ref().unwrap();
    let todo: Vec<String> = ids.iter().map(|&g| body.face_id_str(g)).collect();
    for (&g, n) in ids.iter().zip(&todo) {
        if !matches!(body.face(g).surface, Surface::Plane { .. }) {
            return Err(FeatureError::new("pressPullUnsupported").with(
                "detail",
                Json::Str(format!("{n} is a {} face: Press Pull continued needs a flat face (move a curved face with moveFace)", body.face(g).surface.kind())),
            ));
        }
    }
    let run = |start: &Body, d: f64| -> Result<Body, FeatureError> {
        let mut cur = start.clone();
        for n in &todo {
            let Some(g) = cur.find_face(n) else {
                return Err(failed(format!("{n} was used up by an earlier face of this pull")));
            };
            cur = pull_one(&cur, g, d, taper, &f.id)?;
        }
        Ok(cur)
    };
    let out = match run(body, d) {
        Ok(b) => b,
        Err(e) if e.code == "moveFaceFailed" => {
            let asked = d.abs();
            let max = super::max_size::largest_that_builds(asked, None, |v| run(body, v.copysign(d)).is_ok());
            return Err(match max {
                Some(m) => super::max_size::with_max_size(e, "distance", m.copysign(d), "mm"),
                None => e,
            });
        }
        Err(e) => return Err(e),
    };
    br.body = Some(out);
    Ok(())
}

// ---------------------------------------------------------------- faceRun

/// An end of a feature that moves with one of its inputs: face `<feature>.<cap>` moves with `input`.
#[derive(Debug, Clone, PartialEq)]
pub struct MovingEnd {
    pub feature: String,
    /// `extrude` | `revolve` | `sweep`.
    pub kind: String,
    /// The input that moves it: `depth`, `angle`, `path`.
    pub input: String,
    /// The cap role names (`C0` / `C1`) that move.
    pub caps: Vec<&'static str>,
}

/// The moving ends of the features of a part (what `faceRun` says about a face of one of them).
pub fn moving_ends(part: &crate::Part) -> Vec<MovingEnd> {
    use super::{End, FeatureKind, Op};
    let mut out = vec![];
    for f in &part.features {
        match &f.kind {
            // a cut runs against the sketch normal unless flipped (see extrude.rs): the end that moves is C1 for a
            // forward run, C0 for a backward one
            FeatureKind::Extrude { op, end, flip, .. } => {
                let back = *flip ^ (*op == Op::Cut);
                let caps = match end {
                    End::Blind => vec![if back { "C0" } else { "C1" }],
                    End::Mid => vec!["C0", "C1"],
                    _ => continue,
                };
                out.push(MovingEnd { feature: f.id.clone(), kind: "extrude".into(), input: "depth".into(), caps });
            }
            FeatureKind::Revolve { end, flip, .. } if matches!(end, End::Blind) => {
                out.push(MovingEnd { feature: f.id.clone(), kind: "revolve".into(), input: "angle".into(), caps: vec![if *flip { "C0" } else { "C1" }] });
            }
            FeatureKind::Sweep { surface: false, .. } => {
                out.push(MovingEnd { feature: f.id.clone(), kind: "sweep".into(), input: "path".into(), caps: vec!["C1"] });
            }
            _ => {}
        }
    }
    out
}

/// `{type:"faceRun", faces:[names]}`: per face, `{face, found, flat, centre, normal, reach, area, endOf}`. `centre` is the area
/// centre, `normal` the outward unit normal, `reach` how far the outline reaches from the centre at its farthest point (the
/// Press Pull panel's taper angle is `atan(reach * (endScale - 1) / distance)`), `endOf` `{feature, kind, input}` when the face is
/// the moving end of an extrude / revolve / sweep (edit that feature instead of adding a press pull), else null. Not flat:
/// `flat: false` and the surface kind.
pub fn face_run(body: &Body, msg: &Json, ends: &[MovingEnd]) -> Json {
    let names: Vec<String> = match msg.get("faces") {
        Some(Json::Arr(a)) => a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect(),
        _ => msg.get_str("face").map(|s| vec![s.to_string()]).unwrap_or_default(),
    };
    let items: Vec<Json> = names.iter().map(|n| face_run_one(body, n, ends)).collect();
    Obj::new().put("items", Json::Arr(items)).build()
}

fn vec_json(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

fn face_run_one(body: &Body, name: &str, ends: &[MovingEnd]) -> Json {
    let o = Obj::new().put("face", name);
    let Some(g) = body.find_face(name) else { return o.put("found", false).build() };
    let o = o.put("found", true);
    let end_of = {
        use ck_topo::body::Role;
        let nm = &body.face(g).name;
        let cap = match nm.role {
            Role::Cap(0) => Some("C0"),
            Role::Cap(1) => Some("C1"),
            _ => None,
        };
        cap.and_then(|c| ends.iter().find(|e| e.feature == nm.feature && e.caps.contains(&c)))
            .map(|e| Obj::new().put("feature", e.feature.as_str()).put("kind", e.kind.as_str()).put("input", e.input.as_str()).build())
            .unwrap_or(Json::Null)
    };
    let Some((profile, n_out)) = face_profile(body, g) else {
        return o.put("flat", false).put("surface", body.face(g).surface.kind()).put("endOf", end_of).build();
    };
    let Some(c2) = area_centre(&profile) else { return o.put("flat", true).put("endOf", end_of).build() };
    let fr = profile.frame;
    let centre = fr.o + fr.x * c2.x + fr.y * c2.y;
    let area: f64 = profile.regions.iter().flat_map(|r| r.loops()).map(|l| loop_moments(l).0).sum();
    o.put("flat", true)
        .put("centre", vec_json(centre))
        .put("normal", vec_json(n_out))
        .put("reach", reach_of(&profile, c2))
        .put("area", area.abs())
        .put("endOf", end_of)
        .build()
}
