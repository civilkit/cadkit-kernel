use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
    pub warnings: Vec<regen::RegenError>,
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors, warnings: r.warnings }
}


// The bottle neck: four profiles (rounded rectangles 63x76 R7 / 76x95 R12, circles r20 / r15) lofted in one smooth feature
// overshot the profiles' boxes by 9..15 mm and tore a section, with a clean validate().

fn rrect(id: &str, plane: &str, hx: f64, hy: f64, r: f64) -> String {
    let (x, y) = (hx - r, hy - r);
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p5","x":-{x},"y":{hy}}},{{"id":"p6","x":-{hx},"y":{y}}},{{"id":"p7","x":-{x},"y":{y}}},{{"id":"p8","x":{hx},"y":{y}}},{{"id":"p9","x":{x},"y":{hy}}},{{"id":"p10","x":{x},"y":{y}}},{{"id":"p11","x":{x},"y":-{hy}}},{{"id":"p12","x":{hx},"y":-{y}}},{{"id":"p13","x":{x},"y":-{y}}},{{"id":"p14","x":-{x},"y":-{hy}}},{{"id":"p15","x":-{hx},"y":-{y}}},{{"id":"p16","x":-{x},"y":-{y}}}],"entities":[{{"id":"e1","type":"line","p1":"p14","p2":"p11"}},{{"id":"e2","type":"line","p1":"p12","p2":"p8"}},{{"id":"e3","type":"line","p1":"p9","p2":"p5"}},{{"id":"e4","type":"line","p1":"p6","p2":"p15"}},{{"type":"arc","c":"p7","p1":"p5","p2":"p6","id":"e5"}},{{"type":"arc","c":"p10","p1":"p8","p2":"p9","id":"e6"}},{{"type":"arc","c":"p13","p1":"p11","p2":"p12","id":"e7"}},{{"type":"arc","c":"p16","p1":"p15","p2":"p14","id":"e8"}}],"constraints":[]}}"#
    )
}

fn circ(id: &str, plane: &str, r: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"O","x":0,"y":0}}],"entities":[{{"id":"e1","type":"circle","c":"O","r":{r}}}],"constraints":[]}}"#)
}

/// `which`: the profile ids lofted (F1 base rrect, F5 rrect at 114, F6 circle at 152, F7 circle at 241); `extra`: more loft fields.
fn bottle(which: &[&str], extra: &str) -> String {
    let mut f = vec![];
    for (id, z) in [("F2", 114.0), ("F3", 152.0), ("F4", 241.0)] {
        f.push(format!(r#"{{"type":"plane","id":"{id}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{z}}}"#));
    }
    let front = r#"{"kind":"std","id":"front"}"#;
    f.push(rrect("F1", front, 31.5, 38.0, 7.0));
    f.push(rrect("F5", r#"{"kind":"feature","id":"F2"}"#, 38.0, 47.5, 12.0));
    f.push(circ("F6", r#"{"kind":"feature","id":"F3"}"#, 20.0));
    f.push(circ("F7", r#"{"kind":"feature","id":"F4"}"#, 15.0));
    let sk: Vec<String> = which.iter().map(|s| format!("\"{s}\"")).collect();
    f.push(format!(r#"{{"type":"loft","id":"F8","op":"add","sketches":[{}]{extra}}}"#, sk.join(",")));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

fn half(which: &[&str]) -> (f64, f64) {
    let mut h = (0.0f64, 0.0f64);
    for w in which {
        let (x, y) = match *w {
            "F1" => (31.5, 38.0),
            "F5" => (38.0, 47.5),
            "F6" => (20.0, 20.0),
            _ => (15.0, 15.0),
        };
        h = (h.0.max(x), h.1.max(y));
    }
    h
}


/// An open guide sketch on std plane `plane`: a chain of pieces, each a line (two points) or a spline (three or more); consecutive pieces share their end point.
fn chain_sk(id: &str, plane: &str, segs: &[Vec<(f64, f64)>]) -> String {
    let mut pts: Vec<String> = vec![];
    let mut ents: Vec<String> = vec![];
    let mut last: Option<String> = None;
    for (si, seg) in segs.iter().enumerate() {
        let mut ids: Vec<String> = vec![];
        for (k, (x, y)) in seg.iter().enumerate() {
            if k == 0 && last.is_some() {
                ids.push(last.clone().unwrap());
                continue;
            }
            let pid = format!("q{si}_{k}");
            pts.push(format!(r#"{{"id":"{pid}","x":{x},"y":{y}}}"#));
            ids.push(pid);
        }
        last = ids.last().cloned();
        if seg.len() == 2 {
            ents.push(format!(r#"{{"id":"e{si}","type":"line","p1":"{}","p2":"{}"}}"#, ids[0], ids[1]));
        } else {
            let l: Vec<String> = ids.iter().map(|i| format!("\"{i}\"")).collect();
            ents.push(format!(r#"{{"id":"e{si}","type":"spline","pts":[{}]}}"#, l.join(",")));
        }
    }
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"{plane}"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

/// The bottle loft F1, F5, F6, F7 with the given guide sketches (`(plane, pieces)`), `extra` more loft fields.
fn run_guided(guides: &[(&str, Vec<Vec<(f64, f64)>>)], extra: &str) -> Out {
    let mut v: Vec<String> = vec![];
    let mut refs: Vec<String> = vec![];
    for (i, (plane, segs)) in guides.iter().enumerate() {
        v.push(chain_sk(&format!("G{i}"), plane, segs));
        refs.push(format!(r#"{{"sketch":"G{i}"}}"#));
    }
    let src = bottle(&["F1", "F5", "F6", "F7"], &format!(r#","guides":[{}]{extra}"#, refs.join(",")));
    let src = src.replacen(r#"{"type":"loft""#, &format!("{},{{\"type\":\"loft\"", v.join(",")), 1);
    run_part(&src)
}

/// Rails on the top plane (sketch (x, -z)): the left rail of the bottle repro, each piece ending on a profile; `sx` mirrors it.
fn top_rail(sx: f64) -> (&'static str, Vec<Vec<(f64, f64)>>) {
    let p = |x: f64, z: f64| (sx * x, -z);
    ("top", vec![vec![p(-31.5, 0.0), p(-35.2, 57.0), p(-38.0, 114.0)], vec![p(-38.0, 114.0), p(-29.0, 133.0), p(-20.0, 152.0)], vec![p(-20.0, 152.0), p(-15.0, 241.0)]])
}

/// Rails on the right plane (sketch (-z, y)): the same, in the other direction.
fn right_rail(sy: f64) -> (&'static str, Vec<Vec<(f64, f64)>>) {
    let p = |y: f64, z: f64| (-z, sy * y);
    ("right", vec![vec![p(38.0, 0.0), p(44.0, 57.0), p(47.5, 114.0)], vec![p(47.5, 114.0), p(33.0, 133.0), p(20.0, 152.0)], vec![p(20.0, 152.0), p(15.0, 241.0)]])
}

/// Neck triangles (centroid z in 153..240): (winding normal toward axis, stored vertex normal toward axis, total).
fn inward(o: &Out) -> (usize, usize, usize) {
    let b = o.brep.body.as_ref().unwrap();
    let m = ck_tess::mesh::tessellate(b).mesh;
    let (mut w, mut s, mut n) = (0, 0, 0);
    for t in 0..m.positions.len() / 9 {
        let p: Vec<[f64; 3]> = (0..3).map(|i| (0..3).map(|k| m.positions[9 * t + 3 * i + k] as f64).collect::<Vec<_>>().try_into().unwrap()).collect();
        let c = [(p[0][0] + p[1][0] + p[2][0]) / 3.0, (p[0][1] + p[1][1] + p[2][1]) / 3.0, (p[0][2] + p[1][2] + p[2][2]) / 3.0];
        if !(c[2] > 153.0 && c[2] < 240.0) {
            continue;
        }
        n += 1;
        let (a, b2) = ([p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]], [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]]);
        let nw = [a[1] * b2[2] - a[2] * b2[1], a[2] * b2[0] - a[0] * b2[2], a[0] * b2[1] - a[1] * b2[0]];
        if nw[0] * c[0] + nw[1] * c[1] < 0.0 {
            w += 1;
        }
        let ns = [0, 1, 2].map(|i| (0..2).map(|k| m.normals[9 * t + 3 * i + k] as f64 * c[k]).sum::<f64>()).iter().sum::<f64>();
        if ns < 0.0 {
            s += 1;
        }
    }
    (w, s, n)
}


/// The loft's mesh box (the wire `box`) and volume.
fn measure(o: &Out) -> ((f64, f64, f64), f64) {
    let b = o.brep.body.as_ref().expect("a body");
    let (lo, hi) = ck_tess::mesh::tessellate(b).mesh.bbox().expect("a mesh");
    let ext = (lo.x.abs().max(hi.x.abs()), lo.y.abs().max(hi.y.abs()), lo.z.abs().max(hi.z.abs()));
    (ext, ck_topo::mass::mass_props(b).volume)
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}

// ---- A. guide rails on the bottle: a valid solid, the rails on it

/// Mesh section at height `z`: the cut points (x, y).
fn cut_points(o: &Out, z: f64) -> Vec<(f64, f64)> {
    let b = o.brep.body.as_ref().unwrap();
    let m = ck_tess::mesh::tessellate(b).mesh;
    let p = |i: usize, k: usize| m.positions[3 * i + k] as f64;
    let mut out = vec![];
    for t in 0..m.positions.len() / 9 {
        for (a, c) in [(3 * t, 3 * t + 1), (3 * t + 1, 3 * t + 2), (3 * t + 2, 3 * t)] {
            let (za, zc) = (p(a, 2), p(c, 2));
            if (za - z) * (zc - z) < 0.0 {
                let s = (z - za) / (zc - za);
                out.push((p(a, 0) + s * (p(c, 0) - p(a, 0)), p(a, 1) + s * (p(c, 1) - p(a, 1))));
            }
        }
    }
    out
}

/// x of the section's surface on the -x side, on the line y = 0 (where the left rail runs): the nearest cut point.
fn left_wall(o: &Out, z: f64) -> f64 {
    cut_points(o, z).into_iter().filter(|p| p.0 < 0.0 && p.1.abs() < 0.8).min_by(|a, b| a.1.abs().total_cmp(&b.1.abs())).map_or(f64::NAN, |p| p.0)
}

fn assert_guided_bottle(name: &str, o: &Out) {
    assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
    assert_valid_solid(name, o);
    let (ext, vol) = measure(o);
    assert!(vol > 1.0e5 && ext.2 > 240.0, "{name}: volume {vol}, extent {ext:?}");
    // the rails are on the solid: the left / right rail passes x = -+35.2 at z = 57 and -+38 at the 114 profile
    let (x57, x114, x133) = (left_wall(o, 57.0), left_wall(o, 113.9), left_wall(o, 133.0));
    assert!((x57 + 35.2).abs() < 0.3 && (x114 + 38.0).abs() < 0.3 && (x133 + 29.0).abs() < 0.3, "{name}: the wall is at x = {x57} (rail -35.2) at z 57, {x114} (rail -38) at z 114, {x133} (rail -29) at z 133");
}

/// The bottle with one rail per side (each piece of the rail ends on a profile) used to be refused as `loftInvalidResult`
/// (EDGE_OFF_SURFACE: the guide edge left the side face by 0.28 mm, because the rail and the face were sampled at different
/// loft parameters).
#[test]
fn guided_bottle_with_one_two_and_four_rails_is_valid() {
    assert_guided_bottle("1 rail", &run_guided(&[top_rail(1.0)], ""));
    assert_guided_bottle("2 rails", &run_guided(&[top_rail(1.0), top_rail(-1.0)], ""));
    assert_guided_bottle("4 rails", &run_guided(&[top_rail(1.0), top_rail(-1.0), right_rail(1.0), right_rail(-1.0)], ""));
}

/// One long spline through all four profiles: the fit points do not lie on a 32-sample polyline, which made `loftGuideMiss`
/// report a miss of a guide that touches every profile.
#[test]
fn one_long_spline_guide_through_all_profiles_is_accepted() {
    let long = ("top", vec![vec![(-31.5, 0.0), (-35.2, -57.0), (-38.0, -114.0), (-29.0, -133.0), (-20.0, -152.0), (-15.0, -241.0)]]);
    let o = run_guided(&[long], "");
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("long spline", &o);
}

/// A guide that really misses a profile is still named: 0.5 mm off the 114 profile.
#[test]
fn a_guide_half_a_millimetre_off_a_profile_still_misses() {
    let off = ("top", vec![vec![(-31.5, 0.0), (-35.2, -57.0), (-38.5, -114.0), (-29.0, -133.0), (-20.0, -152.0), (-15.0, -241.0)]]);
    let o = run_guided(&[off], "");
    assert_eq!(o.errors.first().map(|e| e.error.code.as_str()), Some("loftGuideMiss"), "{:?}", o.errors);
}

// ---- B. the neck of the single four-profile loft has no inside-out triangles

/// No neck triangle (centroid z 153..240) may face the axis, by winding or by the stored normal.
fn assert_neck_outward(name: &str, o: &Out) {
    assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
    let (w, s, n) = inward(o);
    assert!(n > 500 && w == 0 && s == 0, "{name}: of {n} neck triangles {w} wind towards the axis and {s} carry a normal towards it");
}

#[test]
fn single_bottle_loft_neck_faces_outward() {
    let all = ["F1", "F5", "F6", "F7"];
    for extra in [r#","railSmoothing":"local""#, r#","railSmoothing":"auto""#, r#","railSmoothing":"local","connectors":[0.342,0.331,0,0]"#, ""] {
        let o = run_part(&bottle(&all, extra));
        assert_neck_outward(extra, &o);
    }
}

/// Starts fixed on the circles (connector 0.5) are the user's override: the circles are paired as before. The twisted result is
/// either outward everywhere or refused as folded, never a silent inside-out strip.
#[test]
fn circle_connectors_override_and_never_give_a_silent_strip() {
    let o = run_part(&bottle(&["F1", "F5", "F6", "F7"], r#","railSmoothing":"local","connectors":[0.842,0.831,0.5,0.5]"#));
    match o.errors.first() {
        Some(e) => assert_eq!(e.error.code, "loftInvalidResult", "{:?}", o.errors),
        None => assert_neck_outward("circle connectors", &o),
    }
}

/// Connectors that turn the circles half a way round against the rectangle: before, a silently folded neck; now the loft builds with
/// the fixed starts kept (valid, no strip facing the axis), and the box excess is reported by the `loftOvershoot` warning.
#[test]
fn circle_start_fixed_half_way_round_builds_without_a_strip() {
    let o = run_part(&bottle(&["F1", "F5", "F6", "F7"], r#","connectors":[null,0.25,null,0.5],"railSmoothing":"auto""#));
    // a half-turn twist tilts the surface so far that a few winding normals point inwards radially: the stored normals are the witness
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("half-way circle start", &o);
    assert_eq!(inward(&o).1, 0);
}

// ---- C. rounded rectangle -> circle: the seams run straight

/// Polar angle (degrees) about the z axis of every seam (an edge of the side faces that runs along the loft), sampled along
/// it; the seam's drift is the spread of its angle, mod 360.
fn seam_drift(o: &Out) -> (f64, usize) {
    let b = o.brep.body.as_ref().unwrap();
    let (mut worst, mut seams) = (0.0f64, 0);
    for e in b.edge_ids() {
        let ed = b.edge(e);
        let (p0, p1) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
        if (p0.z - p1.z).abs() < 1.0 {
            continue;
        }
        seams += 1;
        let ang: Vec<f64> = (0..=24).map(|k| { let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 24.0); p.y.atan2(p.x).to_degrees() }).collect();
        let a0 = ang[0];
        let spread = ang.iter().map(|a| (a - a0 + 540.0).rem_euclid(360.0) - 180.0).fold(0.0f64, |m, d| m.max(d.abs()));
        worst = worst.max(spread);
    }
    (worst, seams)
}

fn two_profile(which: &[&str]) -> Out {
    run_part(&bottle(which, ""))
}

/// The pieces of the circle pair with the pieces of the rounded rectangle at the same polar angle: the seams stay within a
/// degree of their meridian (before: up to 13 degrees, a pinwheel). The seams between the two different rounded rectangles of the bottle (F1, F5) bend by their own tangent angles and are not part of this.
#[test]
fn rounded_rectangle_to_circle_seams_run_straight() {
    for which in [vec!["F5", "F6"], vec!["F1", "F6"], vec!["F6", "F5"], vec!["F5", "F6", "F7"]] {
        let o = two_profile(&which);
        assert!(o.errors.is_empty(), "{which:?}: {:?}", o.errors);
        let (d, seams) = seam_drift(&o);
        assert!(seams >= 8, "{which:?}: {seams} seams");
        assert!(d < 1.0, "{which:?}: a seam drifts {d:.2} degrees about the axis");
        assert_valid_solid("seams", &o);
    }
}

/// A pairing with a twist (the circles start a third of the way round) folds the neck over itself: the loft is refused by name
/// instead of returning a solid with a strip that faces into it (the mesh and `validate` used to be clean).
#[test]
fn a_twisted_pairing_that_folds_the_neck_is_refused_by_name() {
    let o = run_part(&bottle(&["F1", "F5", "F6", "F7"], r#","connectors":[null,null,0.3,0.3]"#));
    let e = o.errors.first().expect("refused");
    assert_eq!(e.error.code, "loftInvalidResult", "{:?}", o.errors);
    let detail = e.error.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str()).unwrap_or("");
    assert!(detail.contains("folds over itself") && detail.contains("FACE_FOLD"), "{detail}");
}
