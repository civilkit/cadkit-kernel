//! Fillets and chamfers that MEET across a face: the two edges of one face are blended so that the face between them is
//! consumed to zero width (radii summing to the face width), the case OCCT reports as "a face needs to be removed".
//!
//! Prism: a w x h rectangle sketched on `top` (sketch x is world x, sketch y is world -z), extruded `l` along +y. Face
//! `F2.S.e1` (z = 0, w wide) is the "top" face; its two edges along y are `F2.S.e1|F2.S.e4` (x = 0) and
//! `F2.S.e1|F2.S.e2` (x = w). Analytic oracle for both top edges filleted with r1 + r2 <= w (each r <= w):
//! V = w h l - l (1 - pi/4) (r1^2 + r2^2); a chamfer d1, d2 removes l (d1^2 + d2^2) / 2. Every built case must be valid,
//! closed (every edge has two coedges), one solid, tessellate without a failed face, and give the same volume on a
//! repeat run.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::f64::consts::PI;

const K: f64 = 1.0 - PI / 4.0;

#[derive(Debug, Clone, PartialEq)]
struct Out {
    /// `feature: code` of every error
    errs: Vec<String>,
    /// the params of every error, as JSON text
    params: Vec<String>,
    faces: usize,
    volume: f64,
    issues: Vec<String>,
    lumps: usize,
    open_edges: usize,
    tess_failed: usize,
    /// face ids of the body
    names: Vec<String>,
    /// `feature: code` of every warning, then its params as JSON text
    warns: Vec<String>,
}

/// Sketch + extrude of the w x h rectangle (rotated by `deg` about the sketch origin), length l along +y.
fn prism(w: f64, h: f64, l: f64, deg: f64) -> String {
    let (s, c) = deg.to_radians().sin_cos();
    let p = |x: f64, y: f64| (c * x - s * y, s * x + c * y);
    let pts = [p(0.0, 0.0), p(w, 0.0), p(w, h), p(0.0, h)];
    let pj: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{l}}}"#,
        pj.join(",")
    )
}

/// World point of sketch point (x, y) of the prism rotated by `deg`, at depth y.
fn world(x: f64, y: f64, deg: f64, depth: f64) -> [f64; 3] {
    let (s, c) = deg.to_radians().sin_cos();
    [c * x - s * y, depth, -(s * x + c * y)]
}

/// Pick of the edge between faces a and b at `at` (with an own radius when given).
fn pick(a: &str, b: &str, at: [f64; 3], radius: Option<f64>) -> String {
    let r = radius.map(|r| format!(r#","radius":{r}"#)).unwrap_or_default();
    format!(r#"{{"faces":["{a}","{b}"],"at":[{},{},{}]{r}}}"#, at[0], at[1], at[2])
}

/// The two top edges (along y) of the w x h x l prism rotated by `deg`.
fn top_left(w: f64, l: f64, deg: f64, r: Option<f64>) -> String {
    let _ = w;
    pick("F2.S.e1", "F2.S.e4", world(0.0, 0.0, deg, l / 2.0), r)
}
fn top_right(w: f64, l: f64, deg: f64, r: Option<f64>) -> String {
    pick("F2.S.e1", "F2.S.e2", world(w, 0.0, deg, l / 2.0), r)
}

fn fillet(id: &str, r: f64, picks: &[String]) -> String {
    format!(r#"{{"type":"fillet","id":"{id}","radius":{r},"edges":[{}]}}"#, picks.join(","))
}
fn chamfer(id: &str, d: f64, picks: &[String]) -> String {
    format!(r#"{{"type":"chamfer","id":"{id}","chamferType":"equal","distance":{d},"edges":[{}]}}"#, picks.join(","))
}

fn run_once(base: &str, feats: &[String]) -> Out {
    let mut all = vec![base.to_string()];
    all.extend(feats.iter().cloned());
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, all.join(","));
    let part = Part::parse(&text).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    let errs = res.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect();
    let params = res
        .errors
        .iter()
        .map(|e| e.error.params.iter().map(|(k, v)| format!("{k}={}", v.to_string())).collect::<Vec<_>>().join("; "))
        .collect();
    let b = br.body.as_ref().expect("a body");
    let mesh = ck_tess::mesh::tessellate(b).mesh;
    Out {
        errs,
        params,
        faces: b.num_faces(),
        volume: ck_topo::mass::mass_props(b).volume,
        issues: ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {}", i.code, i.detail)).collect(),
        lumps: b.num_lumps(),
        open_edges: b.edge_ids().filter(|&e| b.edge_coedges(e).len() != 2).count(),
        tess_failed: mesh.failed.len(),
        names: b.face_ids().map(|f| b.face_id_str(f)).collect(),
        warns: res
            .warnings
            .iter()
            .map(|w| format!("{}: {} {}", w.feature, w.error.code, w.error.params.iter().map(|(k, v)| format!("{k}={}", v.to_string())).collect::<Vec<_>>().join("; ")))
            .collect(),
    }
}

/// Run, check the body is a valid closed single solid that meshes, has the volume `want` to 1e-6 relative and comes out
/// the same on a repeat run. Returns the outcome (face count for the caller to pin).
fn built(tag: &str, base: &str, feats: &[String], want: f64) -> Out {
    let o = run_once(base, feats);
    println!("{tag}: faces {} volume {:.9} want {want:.9} errs {:?}", o.faces, o.volume, o.errs);
    assert!(o.errs.is_empty(), "{tag}: errors {:?} {:?}", o.errs, o.params);
    assert!(o.issues.is_empty(), "{tag}: validate {:?}", o.issues);
    assert_eq!(o.lumps, 1, "{tag}: one solid");
    assert_eq!(o.open_edges, 0, "{tag}: closed");
    assert_eq!(o.tess_failed, 0, "{tag}: tessellates");
    assert!((o.volume - want).abs() <= 1e-6 * want, "{tag}: volume {} vs {want} (rel {:e})", o.volume, (o.volume - want).abs() / want);
    let again = run_once(base, feats);
    assert_eq!(again, o, "{tag}: a repeat run differs");
    assert!(!o.warns.iter().any(|w| w.contains("blendRoundsCross")) || tag.starts_with("cross"), "{tag}: {:?}", o.warns);
    o
}

fn v_fillet(w: f64, h: f64, l: f64, r1: f64, r2: f64) -> f64 {
    w * h * l - l * K * (r1 * r1 + r2 * r2)
}

/// Both top edges in one fillet feature.
fn both(w: f64, h: f64, l: f64, deg: f64, r: f64) -> Vec<String> {
    vec![fillet("F3", r, &[top_left(w, l, deg, None), top_right(w, l, deg, None)])]
}

// ---------------------------------------------------------------- one op, both edges (4 wide x 10 high x 6 long)

#[test]
fn both_edges_one_op_up_to_half_width() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    // (radius, face count): below the tie the top face stays (8 faces); at the tie it is removed and the two quarter
    // cylinders come out as one semicircular face (6)
    for (r, faces) in [(1.5, 8), (1.999, 8), (1.99999999, 6), (2.0, 6), (2.0000001, 6)] {
        let o = built(&format!("both r={r}"), &base, &both(w, h, l, 0.0, r), v_fillet(w, h, l, r, r));
        assert_eq!(o.faces, faces, "both r={r}: face count");
    }
}

#[test]
fn occt_issue_cube_r5() {
    // the 10 mm cube of the upstream report, r = 5 on two opposite edges of one face: V = 892.699082
    let base = prism(10.0, 10.0, 10.0, 0.0);
    let o = built("cube r=5", &base, &both(10.0, 10.0, 10.0, 0.0, 5.0), v_fillet(10.0, 10.0, 10.0, 5.0, 5.0));
    assert!((o.volume - 892.699082).abs() < 1e-6, "{}", o.volume);
    assert_eq!(o.faces, 6);
    let o = built("cube r=4.99", &base, &both(10.0, 10.0, 10.0, 0.0, 4.99), v_fillet(10.0, 10.0, 10.0, 4.99, 4.99));
    assert_eq!(o.faces, 8);
}

#[test]
fn scaled_tie_x10_and_div100() {
    for s in [10.0, 0.01] {
        let (w, h, l) = (4.0 * s, 10.0 * s, 6.0 * s);
        let base = prism(w, h, l, 0.0);
        let o = built(&format!("scale {s} tie"), &base, &both(w, h, l, 0.0, w / 2.0), v_fillet(w, h, l, w / 2.0, w / 2.0));
        assert_eq!(o.faces, 6, "scale {s}");
        let r = 0.375 * w;
        let o = built(&format!("scale {s} r=0.375w"), &base, &both(w, h, l, 0.0, r), v_fillet(w, h, l, r, r));
        assert_eq!(o.faces, 8, "scale {s}");
    }
}

#[test]
fn rotated_prism_tie() {
    let (w, h, l, deg) = (4.0, 10.0, 6.0, 30.0);
    let base = prism(w, h, l, deg);
    let o = built("rotated tie", &base, &both(w, h, l, deg, 2.0), v_fillet(w, h, l, 2.0, 2.0));
    assert_eq!(o.faces, 6);
}

#[test]
fn vertical_edges_of_the_end_cap_tie() {
    // the two edges of the end cap F2.C1 (y = l, w wide) that run along z: length h each
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    let a = pick("F2.C1", "F2.S.e4", [0.0, l, -h / 2.0], None);
    let b = pick("F2.C1", "F2.S.e2", [w, l, -h / 2.0], None);
    for (r, faces) in [(1.5, 8), (2.0, 6)] {
        let want = w * h * l - h * K * 2.0 * r * r;
        let o = built(&format!("cap edges r={r}"), &base, &[fillet("F3", r, &[a.clone(), b.clone()])], want);
        assert_eq!(o.faces, faces, "cap edges r={r}");
    }
}

#[test]
fn thin_plate_tie() {
    // 1 wide x 40 high x 20 long, both top edges r = 0.5
    let base = prism(1.0, 40.0, 20.0, 0.0);
    let o = built("plate r=0.5", &base, &both(1.0, 40.0, 20.0, 0.0, 0.5), v_fillet(1.0, 40.0, 20.0, 0.5, 0.5));
    assert_eq!(o.faces, 6);
}

#[test]
fn unequal_radii_summing_to_the_width_one_op() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for (r1, r2) in [(2.5, 1.5), (1.5, 2.5), (3.0, 1.0)] {
        let f = fillet("F3", r1, &[top_left(w, l, 0.0, Some(r1)), top_right(w, l, 0.0, Some(r2))]);
        let o = built(&format!("one op {r1}+{r2}"), &base, &[f], v_fillet(w, h, l, r1, r2));
        assert!(o.faces == 6 || o.faces == 7, "one op {r1}+{r2}: {} faces", o.faces);
    }
}

// ---------------------------------------------------------------- sequential

fn seq(w: f64, l: f64, deg: f64, r_left: f64, r_right: f64, left_first: bool) -> Vec<String> {
    // the first feature is F3, the second F4
    let left = |id: &str| fillet(id, r_left, &[top_left(w, l, deg, None)]);
    let right = |id: &str| fillet(id, r_right, &[top_right(w, l, deg, None)]);
    if left_first {
        vec![left("F3"), right("F4")]
    } else {
        vec![right("F3"), left("F4")]
    }
}

#[test]
fn sequential_unequal_radii_summing_to_the_width() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for (r1, r2) in [(2.5, 1.5), (1.5, 2.5), (3.0, 1.0)] {
        for left_first in [true, false] {
            let o = built(&format!("seq {r1}+{r2} left first {left_first}"), &base, &seq(w, l, 0.0, r1, r2, left_first), v_fillet(w, h, l, r1, r2));
            assert!(o.faces == 6 || o.faces == 7, "seq {r1}+{r2}: {} faces", o.faces);
        }
    }
}

/// DEFECT A: after the first r = 2 fillet the top face is exactly 2 wide, and a second r = 2 fillet on its other edge
/// consumes it to zero width. It must give the solid of the one-op fillet (volume, validity, face count within one).
#[test]
fn sequential_equal_radii_tie_matches_one_op() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    let one = built("one op r=2", &base, &both(w, h, l, 0.0, 2.0), v_fillet(w, h, l, 2.0, 2.0));
    for left_first in [true, false] {
        let o = built(&format!("seq 2+2 left first {left_first}"), &base, &seq(w, l, 0.0, 2.0, 2.0, left_first), v_fillet(w, h, l, 2.0, 2.0));
        assert!((o.volume - one.volume).abs() <= 1e-9 * one.volume, "seq vs one op volume {} vs {}", o.volume, one.volume);
        assert!(o.faces.abs_diff(one.faces) <= 1, "seq {} faces vs one op {}", o.faces, one.faces);
    }
    // the 10 mm cube of the upstream report, r = 5 then r = 5
    let base = prism(10.0, 10.0, 10.0, 0.0);
    for left_first in [true, false] {
        built(&format!("cube seq 5+5 left first {left_first}"), &base, &seq(10.0, 10.0, 0.0, 5.0, 5.0, left_first), 892.6990816987241);
    }
    // rotated and scaled
    let base = prism(40.0, 100.0, 60.0, 30.0);
    built("seq x10 rotated", &base, &seq(40.0, 60.0, 30.0, 20.0, 20.0, true), v_fillet(40.0, 100.0, 60.0, 20.0, 20.0));
}

/// DEFECT B: 3 then 2 on the 4 wide face: the second round is wider than the 1 wide face left beside it. That is a size
/// refusal (`blendOverflow` with a hint and the largest radius that builds, 1), not a missing edge.
#[test]
fn sequential_overflow_is_blend_overflow_with_max_size() {
    let (w, l) = (4.0, 6.0);
    let base = prism(w, 10.0, l, 0.0);
    for left_first in [true, false] {
        let o = run_once(&base, &seq(w, l, 0.0, if left_first { 3.0 } else { 2.0 }, if left_first { 2.0 } else { 3.0 }, left_first));
        println!("seq 3 then 2 (left first {left_first}): {:?} {:?}", o.errs, o.params);
        assert_eq!(o.errs, vec!["F4: blendOverflow".to_string()], "{:?}", o.params);
        let p = &o.params[0];
        assert!(p.contains("hint="), "{p}");
        let max = p.split("\"value\":").nth(1).and_then(|s| s.split([',', '}']).next()).and_then(|s| s.parse::<f64>().ok());
        let max = max.unwrap_or_else(|| panic!("no maxSize in {p}"));
        assert!((max - 1.0).abs() < 0.01, "maxSize {max}");
        // the first fillet's body is kept
        assert!(o.issues.is_empty(), "{:?}", o.issues);
    }
}

/// DEFECT B as the kit's selector meets it: after the r = 3 fillet the top face is 1 wide, and the pick "the top edges
/// along y near the right side, within 1" takes both of its edges: the right edge and the smooth edge where the top face
/// runs tangentially into the first fillet. That selection went to the free-form chains, where the right edge's round of
/// 2 consumed the 1 mm face and with it the tangent edge, then reported `missingEdge`. The smooth edge has nothing to
/// round at any size, so that is the refusal, the same in both pick orders and with a hint to leave it out; without it
/// the pick is the plain overflow of `sequential_overflow_is_blend_overflow_with_max_size` (maxSize 1).
#[test]
fn sequential_overflow_with_the_tangent_edge_picked_too() {
    let (w, l) = (4.0, 6.0);
    let base = prism(w, 10.0, l, 0.0);
    let right = top_right(w, l, 0.0, None);
    let tangent = pick("F2.S.e1", "F3.B0", [3.0, l / 2.0, 0.0], None);
    let first = fillet("F3", 3.0, &[top_left(w, l, 0.0, None)]);
    for (r, picks) in [(2.0, vec![right.clone(), tangent.clone()]), (2.0, vec![tangent.clone(), right.clone()]), (0.5, vec![right.clone(), tangent.clone()]), (0.5, vec![tangent.clone()])] {
        let o = run_once(&base, &[first.clone(), fillet("F4", r, &picks)]);
        println!("seq 3 then {r} with the tangent edge ({} picks): {:?} {:?}", picks.len(), o.errs, o.params);
        assert_eq!(o.errs, vec!["F4: blendUnsupported".to_string()], "{:?}", o.params);
        assert!(o.params[0].contains("tangent all along") && o.params[0].contains("hint="), "{}", o.params[0]);
        assert!(o.issues.is_empty(), "{:?}", o.issues);
    }
}

/// One op with a per-edge radius wider than the face (4.5 and 1 on the 4 wide face): a single round of 4.5 does not fit,
/// so the selection is refused as an overflow in both pick orders (with radii that each fit, see `crossing_*`).
#[test]
fn one_op_radius_wider_than_the_face_is_overflow() {
    let (w, l) = (4.0, 6.0);
    let base = prism(w, 10.0, l, 0.0);
    for left_first in [true, false] {
        let (a, b) = (top_left(w, l, 0.0, Some(4.5)), top_right(w, l, 0.0, Some(1.0)));
        let picks = if left_first { vec![a, b] } else { vec![b, a] };
        let o = run_once(&base, &[fillet("F3", 4.5, &picks)]);
        println!("one op 4.5 + 1 (left first {left_first}): {:?} {:?}", o.errs, o.params);
        assert_eq!(o.errs, vec!["F3: blendOverflow".to_string()], "{:?}", o.params);
        assert!(o.issues.is_empty(), "{:?}", o.issues);
    }
}

// ---------------------------------------------------------------- chamfers

#[test]
fn chamfers_meeting() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for (d, faces) in [(1.5, 8), (2.0, 7)] {
        let f = chamfer("F3", d, &[top_left(w, l, 0.0, None), top_right(w, l, 0.0, None)]);
        let o = built(&format!("chamfer {d}"), &base, &[f], w * h * l - l * d * d);
        assert_eq!(o.faces, faces, "chamfer {d}");
    }
    // unequal distances d1 + d2 = w, sequential in both orders
    for (d1, d2) in [(2.5, 1.5), (1.5, 2.5)] {
        let fs = vec![chamfer("F3", d1, &[top_left(w, l, 0.0, None)]), chamfer("F4", d2, &[top_right(w, l, 0.0, None)])];
        let o = built(&format!("chamfer seq {d1}+{d2}"), &base, &fs, w * h * l - l * 0.5 * (d1 * d1 + d2 * d2));
        assert_eq!(o.faces, 7, "chamfer seq {d1}+{d2}");
    }
    // equal sequential tie
    let fs = vec![chamfer("F3", 2.0, &[top_left(w, l, 0.0, None)]), chamfer("F4", 2.0, &[top_right(w, l, 0.0, None)])];
    let o = built("chamfer seq 2+2", &base, &fs, w * h * l - l * 4.0);
    assert_eq!(o.faces, 7);
}

// ---------------------------------------------------------------- single edge

#[test]
fn single_edge_up_to_full_width() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for (r, faces) in [(3.99, 7), (4.0, 6)] {
        let o = built(&format!("single r={r}"), &base, &[fillet("F3", r, &[top_left(w, l, 0.0, None)])], w * h * l - l * K * r * r);
        assert_eq!(o.faces, faces, "single r={r}");
    }
}

// ---------------------------------------------------------------- refusals by design (a change here is a decision)

fn refused(tag: &str, base: &str, feats: &[String], feature: &str) {
    let o = run_once(base, feats);
    println!("{tag}: {:?} {:?}", o.errs, o.params);
    assert_eq!(o.errs, vec![format!("{feature}: blendOverflow")], "{tag}: {:?}", o.params);
    assert!(o.params[0].contains("hint="), "{tag}: no hint in {}", o.params[0]);
    assert!(o.issues.is_empty(), "{tag}: the body before the blend is kept valid");
}

/// Removed area of the w wide top of the section when its corners are rounded with r1 (left) and r2 (right), each at most
/// w: the area above min(z1, z2) of the two rounded profiles, integrated numerically (the profile's square-root ends are
/// taken in the variable u = sqrt(x), so the midpoint rule converges fast).
fn removed_profiles(w: f64, r1: f64, r2: f64) -> f64 {
    // depth below the top of each rounded profile at x (0 where it is flat)
    let d1 = |x: f64| if x < r1 { r1 - (r1 * r1 - (r1 - x) * (r1 - x)).max(0.0).sqrt() } else { 0.0 };
    let d2 = |x: f64| if x > w - r2 { r2 - (r2 * r2 - (x - w + r2) * (x - w + r2)).max(0.0).sqrt() } else { 0.0 };
    let f = |x: f64| d1(x).max(d2(x));
    // split at the middle; on each half map x = u^2 from its sharp end
    let n = 400_000;
    let half = w / 2.0;
    let (umax, du) = (half.sqrt(), half.sqrt() / n as f64);
    (0..n)
        .map(|i| {
            let u = (i as f64 + 0.5) * du;
            let _ = umax;
            (f(u * u) + f(w - u * u)) * 2.0 * u * du
        })
        .sum()
}

#[test]
fn crossing_oracle() {
    // equal radii: the closed form; the coordinator's figures for w = 4: 2.0762 (r = 2.2), 3.7499 (r = 3)
    // (closed form: 2.076138 and 3.749792)
    for (r, want) in [(2.2, 2.0762), (3.0, 3.7499)] {
        let a = removed_crossing(4.0, r);
        assert!((a - want).abs() < 2e-4, "r={r}: {a}");
        assert!((removed_profiles(4.0, r, r) - a).abs() < 1e-9, "r={r}: {} vs {a}", removed_profiles(4.0, r, r));
    }
    // at the tie the profiles are the two quarter discs' corners
    assert!((removed_profiles(4.0, 2.5, 1.5) - K * (2.5f64.powi(2) + 1.5f64.powi(2))).abs() < 1e-9);
}

fn crossing_warning(o: &Out, feature: &str, meet: &str) {
    assert_eq!(o.warns.len(), 1, "{:?}", o.warns);
    let w = &o.warns[0];
    assert!(w.starts_with(&format!("{feature}: blendRoundsCross")) && w.contains("cross in a sharp edge") && w.contains(meet), "{w}");
}

/// The user's decision: both top edges of one face in one op with r1 + r2 > w (each at most w and within the walls) build
/// the INTERSECTION of the two rounded profiles: the top face is gone, the rounds B0 and B1 meet in a sharp crease, and the
/// feature warns `blendRoundsCross` with the radius at which they would just meet. Includes the box an upstream fork
/// reported as "accepted but invalid" (4 wide, r = 3 on both edges): valid here, with the exact volume.
#[test]
fn crossing_rounds_one_op() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for r in [2.2, 3.0, 4.0] {
        let o = built(&format!("cross r={r}"), &base, &both(w, h, l, 0.0, r), l * (w * h - removed_crossing(w, r)));
        println!("cross r={r}: {} faces {:?} {:?}", o.faces, o.names, o.warns);
        assert_eq!(o.faces, 7, "cross r={r}");
        assert!(!o.names.iter().any(|n| n.starts_with("F2.S.e1")), "the top face is gone: {:?}", o.names);
        assert!(o.names.iter().any(|n| n == "F3.B0") && o.names.iter().any(|n| n == "F3.B1"), "{:?}", o.names);
        crossing_warning(&o, "F3", "meetRadius=2");
    }
    // the thin plate, 1 wide, r = 0.6
    let base = prism(1.0, 40.0, 20.0, 0.0);
    let o = built("cross plate r=0.6", &base, &both(1.0, 40.0, 20.0, 0.0, 0.6), 20.0 * (40.0 - removed_crossing(1.0, 0.6)));
    assert_eq!(o.faces, 7);
    crossing_warning(&o, "F3", "meetRadius=0.5");
    // rotated, and scaled
    let base = prism(w, h, l, 30.0);
    built("cross rotated r=3", &base, &both(w, h, l, 30.0, 3.0), l * (w * h - removed_crossing(w, 3.0)));
    let base = prism(40.0, 100.0, 60.0, 0.0);
    built("cross x10 r=30", &base, &both(40.0, 100.0, 60.0, 0.0, 30.0), 60.0 * (4000.0 - removed_crossing(40.0, 30.0)));
}

#[test]
fn crossing_rounds_unequal_radii() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for (r1, r2) in [(3.0, 2.0), (2.0, 3.0), (4.0, 1.0)] {
        for left_first in [true, false] {
            let (a, b) = (top_left(w, l, 0.0, Some(r1)), top_right(w, l, 0.0, Some(r2)));
            let picks = if left_first { vec![a, b] } else { vec![b, a] };
            let o = built(&format!("cross {r1}+{r2} left first {left_first}"), &base, &[fillet("F3", r1, &picks)], l * (w * h - removed_profiles(w, r1, r2)));
            println!("cross {r1}+{r2}: {} faces {:?} {:?}", o.faces, o.names, o.warns);
            assert_eq!(o.faces, 7);
            crossing_warning(&o, "F3", "meetRadii=");
        }
    }
}

/// What stays refused: a round taller than its wall (4 wide, walls 3 high, r = 3.5 on both edges: each single round
/// leaves its wall), and both rounds wider than the face (r = 4.5 > w). `blendOverflow`, and the largest size that builds
/// is now the crossing one (r = w = 4) where the walls allow it.
#[test]
fn crossing_limits_stay_refused() {
    // (rounds 3.5 > the 3 high walls: each runs off its wall, is built on the wall continued and cut from the body; since
    // 3.5 + 3.5 > 4 the two cross: the kept section is below both rounded profiles, clipped by the 3 high walls)
    let base = prism(4.0, 3.0, 6.0, 0.0);
    let o = built("cross walls 3 high r=3.5", &base, &both(4.0, 3.0, 6.0, 0.0, 3.5), 6.0 * (4.0 * 3.0 - removed_profiles_clipped(4.0, 3.5, 3.5, 3.0)));
    assert!(o.warns.iter().any(|w| w.contains("blendRanOffFace")), "{:?}", o.warns);
    let base = prism(4.0, 10.0, 6.0, 0.0);
    let o = run_once(&base, &both(4.0, 10.0, 6.0, 0.0, 4.5));
    println!("both r=4.5: {:?} {:?}", o.errs, o.params);
    assert_eq!(o.errs, vec!["F3: blendOverflow".to_string()], "{:?}", o.params);
    assert!(o.warns.is_empty(), "{:?}", o.warns);
}

/// Chamfers d1 + d2 > w (out of scope of the crossing build): the equal chamfer of both edges of the 4 wide face with
/// d = 2.5 is built by the ordinary chamfer path (as at the base commit) as the intersection of the two chamfered profiles:
/// the planes cross at x = w / 2, d - w / 2 below the top, removing d w - w^2 / 4 per unit length.
#[test]
fn chamfers_wider_than_half_the_face() {
    let (w, h, l) = (4.0, 10.0, 6.0);
    let base = prism(w, h, l, 0.0);
    for d in [2.5, 3.0] {
        let o = built(&format!("chamfer both {d}"), &base, &[chamfer("F3", d, &[top_left(w, l, 0.0, None), top_right(w, l, 0.0, None)])], l * (w * h - (d * w - w * w / 4.0)));
        println!("chamfer both {d}: {} faces {:?} {:?}", o.faces, o.names, o.warns);
    }
}

#[test]
fn single_edge_wider_than_the_face_runs_off_it() {
    // (a round wider than the 4 wide top face runs off it over the far top edge: built on the top plane continued, the part
    // that lies in the body cut away (solidcore kernel case 5158); it used to be refused)
    let base = prism(4.0, 10.0, 6.0, 0.0);
    let o = built("single r=4.5", &base, &[fillet("F3", 4.5, &[top_left(4.0, 6.0, 0.0, None)])], 6.0 * (4.0 * 10.0 - removed_profiles(4.0, 4.5, 0.0)));
    assert!(o.warns.iter().any(|w| w.contains("blendRanOffFace")), "{:?}", o.warns);
}

/// [`removed_profiles`] with the rounds' depth clipped to the wall's height `h`.
fn removed_profiles_clipped(w: f64, r1: f64, r2: f64, h: f64) -> f64 {
    let d1 = |x: f64| if x < r1 { r1 - (r1 * r1 - (r1 - x) * (r1 - x)).max(0.0).sqrt() } else { 0.0 };
    let d2 = |x: f64| if x > w - r2 { r2 - (r2 * r2 - (x - w + r2) * (x - w + r2)).max(0.0).sqrt() } else { 0.0 };
    let f = |x: f64| d1(x).max(d2(x)).min(h);
    let n = 400_000;
    let du = (w / 2.0).sqrt() / n as f64;
    (0..n)
        .map(|i| {
            let u = (i as f64 + 0.5) * du;
            (f(u * u) + f(w - u * u)) * 2.0 * u * du
        })
        .sum()
}

// ---------------------------------------------------------------- crossing rounds (r1 + r2 > w)

/// Area removed from the w wide top of the section when both top corners are rounded with r, w / 2 < r <= w, and the
/// kept region is the intersection of the two rounded profiles (the arcs cross at x = w / 2, at the height
/// h - r + sqrt(w r - w^2 / 4)): 2 * integral_0^{w/2} (r - sqrt(r^2 - (r - x)^2)) dx.
fn removed_crossing(w: f64, r: f64) -> f64 {
    let f = |u: f64| 0.5 * (u * (r * r - u * u).sqrt() + r * r * (u / r).asin());
    r * w - PI * r * r / 2.0 + 2.0 * f(r - w / 2.0)
}

#[test]
fn crossing_oracle_matches_the_tie_and_a_numeric_integral() {
    // at r = w / 2 the closed form is the two quarter discs' 2 (1 - pi/4) r^2
    assert!((removed_crossing(4.0, 2.0) - 2.0 * K * 4.0).abs() < 1e-12);
    // midpoint rule for r = 3 on w = 4
    let (w, r, n) = (4.0, 3.0, 200_000);
    let dx = w / 2.0 / n as f64;
    let num: f64 = (0..n).map(|i| {
        let x = (i as f64 + 0.5) * dx;
        r - (r * r - (r - x) * (r - x)).sqrt()
    }).sum::<f64>() * dx * 2.0;
    assert!((removed_crossing(w, r) - num).abs() < 1e-8, "{} vs {num}", removed_crossing(w, r));
}

// ---------------------------------------------------------------- a thin slab that loses its side faces
//
// Slab: the 50 x 10 rectangle on `top` extruded 50 (x 0..50, y 0..50, z -10..0). Top face S.e1 (z = 0), bottom S.e3,
// sides S.e2 (x = 50), S.e4 (x = 0), C0 (y = 0), C1 (y = 50). A round of r = 5 = half the thickness on the top and the
// bottom edges of a side face meets along its middle: the side face is consumed (upstream: "a face needs to be removed").

/// The 12 edges of the slab: the 8 of the top and bottom faces, then the 4 of thickness 10 (`vertical`).
fn slab_edges() -> (Vec<String>, Vec<String>) {
    let (a, c) = (50.0, 10.0);
    let m = a / 2.0;
    let tb = vec![
        pick("F2.S.e1", "F2.S.e2", [a, m, 0.0], None),
        pick("F2.S.e1", "F2.S.e4", [0.0, m, 0.0], None),
        pick("F2.C0", "F2.S.e1", [m, 0.0, 0.0], None),
        pick("F2.C1", "F2.S.e1", [m, a, 0.0], None),
        pick("F2.S.e3", "F2.S.e2", [a, m, -c], None),
        pick("F2.S.e3", "F2.S.e4", [0.0, m, -c], None),
        pick("F2.C0", "F2.S.e3", [m, 0.0, -c], None),
        pick("F2.C1", "F2.S.e3", [m, a, -c], None),
    ];
    let v = vec![
        pick("F2.C0", "F2.S.e2", [a, 0.0, -c / 2.0], None),
        pick("F2.C0", "F2.S.e4", [0.0, 0.0, -c / 2.0], None),
        pick("F2.C1", "F2.S.e2", [a, a, -c / 2.0], None),
        pick("F2.C1", "F2.S.e4", [0.0, a, -c / 2.0], None),
    ];
    (tb, v)
}

/// Every edge of a p x q x s box rounded with r (2r <= each side): the inner box a x b x c = (p - 2r)(q - 2r)(s - 2r)
/// grown by a ball of radius r (Steiner): abc + 2(ab + bc + ca) r + pi r^2 (a + b + c) + 4/3 pi r^3.
fn v_all_rounded(p: f64, q: f64, s: f64, r: f64) -> f64 {
    let (a, b, c) = (p - 2.0 * r, q - 2.0 * r, s - 2.0 * r);
    a * b * c + 2.0 * (a * b + b * c + a * c) * r + PI * r * r * (a + b + c) + 4.0 / 3.0 * PI * r.powi(3)
}

/// Volume of the box |x| <= a, |y| <= b, |z| <= c with all 12 edges chamfered by d as the intersection of the 12
/// half-spaces (mitred corners: |x| + |y| <= a + b - d and the like), by integrating the cross-section over z.
fn v_all_chamfered_mitre(a: f64, b: f64, c: f64, d: f64) -> f64 {
    let n = 400_000;
    let dz = 2.0 * c / n as f64;
    (0..n)
        .map(|i| {
            let z = -c + (i as f64 + 0.5) * dz;
            let x = a.min(a + c - d - z.abs());
            let y = b.min(b + c - d - z.abs());
            let k = a + b - d;
            let cut = (x + y - k).max(0.0);
            (4.0 * x * y - 2.0 * cut * cut) * dz
        })
        .sum()
}

#[test]
fn slab_oracles() {
    // the coordinator's figure for the slab: 40 * 40 * 10 + (pi 25 / 2) 160 + 4/3 pi 125
    assert!((v_all_rounded(50.0, 50.0, 10.0, 5.0) - 22806.784).abs() < 1e-3, "{}", v_all_rounded(50.0, 50.0, 10.0, 5.0));
    // a cube with every edge rounded by half its side is a ball
    assert!((v_all_rounded(2.0, 2.0, 2.0, 1.0) - 4.0 / 3.0 * PI).abs() < 1e-12);
    // the mitred chamfer of the 50 x 50 x 10 slab, d = 5: 8/3 (25^3 - 20^3) - 4 (5^3 / 6)
    let want = 8.0 / 3.0 * (25f64.powi(3) - 20f64.powi(3)) - 4.0 * 125.0 / 6.0;
    assert!((v_all_chamfered_mitre(25.0, 25.0, 5.0, 5.0) - want).abs() < 1e-6, "{} vs {want}", v_all_chamfered_mitre(25.0, 25.0, 5.0, 5.0));
}

fn slab_all() -> Vec<String> {
    let (mut all, v) = slab_edges();
    all.extend(v);
    all
}

/// All 12 edges just below the tie (r = d = 4.9 on the 10 thick slab): the sides keep a 0.2 band. Built, exact volume
/// (rounds: the Steiner volume; chamfers: the mitred corners).
#[test]
fn slab_all_edges_near_the_tie() {
    let base = prism(50.0, 10.0, 50.0, 0.0);
    let o = built("slab all 12 r=4.9", &base, &[fillet("F3", 4.9, &slab_all())], v_all_rounded(50.0, 50.0, 10.0, 4.9));
    println!("slab all 12 r=4.9: {} faces", o.faces);
    let o = built("slab all 12 chamfer 4.9", &base, &[chamfer("F3", 4.9, &slab_all())], v_all_chamfered_mitre(25.0, 25.0, 5.0, 4.9));
    println!("slab all 12 chamfer 4.9: {} faces", o.faces);
}

/// All 12 edges AT the tie (r = d = 5 = half the thickness): the top and bottom rounds of each side meet along its middle,
/// the four edges of thickness 10 shrink to nothing and the side faces are consumed. Not built today: refused as an
/// overflow with a hint and the largest size that builds (it was `missingEdge` for the chamfer and the round before the
/// consumed-pick fix). The oracle of the build is `slab_all_edges_at_the_tie_built`.
#[test]
fn slab_all_edges_at_the_tie_is_refused() {
    let base = prism(50.0, 10.0, 50.0, 0.0);
    refused("slab all 12 r=5", &base, &[fillet("F3", 5.0, &slab_all())], "F3");
    refused("slab all 12 chamfer 5", &base, &[chamfer("F3", 5.0, &slab_all())], "F3");
}

#[test]
#[ignore = "all 12 slab edges at r = d = half the thickness are refused today (the thickness edges vanish at the corners); the oracle for a future build"]
fn slab_all_edges_at_the_tie_built() {
    let base = prism(50.0, 10.0, 50.0, 0.0);
    for (tag, feat, want) in [
        ("slab all 12 r=5", fillet("F3", 5.0, &slab_all()), v_all_rounded(50.0, 50.0, 10.0, 5.0)),
        ("slab all 12 chamfer 5", chamfer("F3", 5.0, &slab_all()), v_all_chamfered_mitre(25.0, 25.0, 5.0, 5.0)),
    ] {
        let o = built(tag, &base, &[feat], want);
        for side in ["F2.S.e2", "F2.S.e4", "F2.C0", "F2.C1"] {
            assert!(!o.names.iter().any(|n| n.starts_with(side)), "{tag}: side face {side} left: {:?}", o.names);
        }
    }
}

/// Only the 8 edges of the top and bottom faces (the 4 of thickness stay sharp): at each of the 8 corners two rounds meet
/// in a mitre. Removed: (1 - pi/4) r^2 along each of the 8 edges' full length, less the doubly counted corner pieces:
/// per corner the integral over the height t of h(t)^2, h = r - sqrt(r^2 - (r - t)^2), which is r^3 (5/3 - pi/2).
fn v_top_bottom_rounded(a: f64, b: f64, c: f64, r: f64) -> f64 {
    a * b * c - K * r * r * 2.0 * (2.0 * a + 2.0 * b) + 8.0 * r.powi(3) * (5.0 / 3.0 - PI / 2.0)
}

#[test]
fn slab_top_and_bottom_edges_round() {
    let base = prism(50.0, 10.0, 50.0, 0.0);
    let (tb, _) = slab_edges();
    for r in [4.99, 5.0] {
        let o = built(&format!("slab 8 edges r={r}"), &base, &[fillet("F3", r, &tb)], v_top_bottom_rounded(50.0, 50.0, 10.0, r));
        println!("slab 8 edges r={r}: {} faces {:?}", o.faces, o.names);
    }
}

/// The 10 mm cube, one edge rounded with r = 10, the full width of both its faces (upstream: fillet to the opposing
/// edge): a quarter cylinder, 1000 - 10 (1 - pi/4) 100 = 10 * 100 * pi / 4.
#[test]
fn cube_one_edge_full_width() {
    let base = prism(10.0, 10.0, 10.0, 0.0);
    let o = run_once(&base, &[fillet("F3", 10.0, &[top_left(10.0, 10.0, 0.0, None)])]);
    println!("cube one edge r=10: {:?} {:?} faces {} volume {} issues {:?}", o.errs, o.params, o.faces, o.volume, o.issues);
    if o.errs.is_empty() {
        built("cube one edge r=10", &base, &[fillet("F3", 10.0, &[top_left(10.0, 10.0, 0.0, None)])], 250.0 * PI);
    } else {
        assert_eq!(o.errs, vec!["F3: blendOverflow".to_string()], "{:?}", o.params);
        assert!(o.params[0].contains("hint="), "{}", o.params[0]);
    }
}

// ---------------------------------------------------------------- the concave pocket of the upstream report
//
// The 10 x 10 cube less the 5 x 5 corner (an L of two arms 5 wide, 10 high), as one L profile on `top` extruded 10
// (sketch x = world x, sketch y = world -z; extrusion along y). The walls e3 (sketch y = 5, x 5..10) and e4 (x = 5,
// sketch y 5..10) are 5 long, each between the concave inner corner (5, 5) and a convex corner, (10, 5) or (5, 10).
// Rounding all three with r = 2.5 consumes both walls (2.5 + 2.5 = 5). The concave round adds (1 - pi/4) r^2 per unit
// length, each convex one removes it: V = 10 (75 - (1 - pi/4) r^2).
fn l_prism() -> String {
    let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (5.0, 5.0), (5.0, 10.0), (0.0, 10.0)];
    let pj: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let ej: Vec<String> = (0..6).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % 6 + 1)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#,
        pj.join(","),
        ej.join(",")
    )
}

fn l_picks() -> Vec<String> {
    vec![
        pick("F2.S.e3", "F2.S.e4", [5.0, 5.0, -5.0], None),
        pick("F2.S.e2", "F2.S.e3", [10.0, 5.0, -5.0], None),
        pick("F2.S.e4", "F2.S.e5", [5.0, 5.0, -10.0], None),
    ]
}

#[test]
fn concave_pocket_three_edges() {
    let base = l_prism();
    for r in [2.4, 2.5] {
        let o = built(&format!("L three edges r={r}"), &base, &[fillet("F3", r, &l_picks())], 10.0 * (75.0 - K * r * r));
        println!("L three edges r={r}: {} faces {:?}", o.faces, o.names);
    }
    // r = 2.6: more than half of each wall: refused as an overflow with a hint, or a correct solid, never an invalid one
    let o = run_once(&base, &[fillet("F3", 2.6, &l_picks())]);
    println!("L three edges r=2.6: {:?} {:?} faces {} volume {} issues {:?}", o.errs, o.params, o.faces, o.volume, o.issues);
    assert!(o.issues.is_empty() && o.open_edges == 0 && o.lumps == 1, "{:?}", o.issues);
    if o.errs.is_empty() {
        panic!("r = 2.6 built ({} faces, volume {}): needs an oracle before it is accepted", o.faces, o.volume);
    }
    assert_eq!(o.errs, vec!["F3: blendOverflow".to_string()], "{:?}", o.params);
    assert!(o.params[0].contains("hint="), "{}", o.params[0]);
}

