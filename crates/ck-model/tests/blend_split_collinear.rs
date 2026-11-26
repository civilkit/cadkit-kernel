//! A fillet or chamfer on an edge run that the sketch split into collinear pieces (a mirrored sketch leaves the back
//! wall of a U as two lines meeting at the mirror axis; a split line or a vertex on the line does the same) rounds
//! the whole run, whichever piece is picked, and gives the same body as the unsplit sketch. A pick that cannot be
//! applied to its whole run is refused or warned by name, never left half done.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

/// U plate, 40 x 30 outline, arms 8 wide, cavity x 8..32, y 8..30, 10 deep. `split`: the back wall of the cavity
/// (y = 8) is two lines meeting at x = 20 (e5, e6) instead of one (e5).
fn body(split: bool) -> String {
    let mut pts: Vec<(f64, f64)> = vec![(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (32.0, 30.0), (32.0, 8.0)];
    if split {
        pts.push((20.0, 8.0));
    }
    pts.extend([(8.0, 8.0), (8.0, 30.0), (0.0, 30.0)]);
    let n = pts.len();
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, p.0, p.1)).collect();
    let es: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"right"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}"#,
        ps.join(","),
        es.join(",")
    )
}

/// Pick on the cap face (F2.C1, x = 10) and the cavity wall of line `e` at sketch point (sx, sy).
fn pick(e: &str, sx: f64, sy: f64) -> String {
    format!(r#"{{"faces":["F2.C1","F2.S.{e}"],"at":[10,{sy},{}]}}"#, -sx)
}

struct Out {
    errors: Vec<String>,
    warnings: Vec<String>,
    faces: usize,
    volume: f64,
    issues: usize,
}

fn run(split: bool, feature: &str) -> Out {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{},{feature}]}}"#, body(split));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    Out {
        errors: res.errors.iter().map(|e| e.error.code.clone()).collect(),
        warnings: res.warnings.iter().map(|e| e.error.code.clone()).collect(),
        faces: b.num_faces(),
        volume: ck_topo::mass::mass_props(b).volume,
        issues: ck_topo::check::check(b, 1e-6).len(),
    }
}

fn fillet(r: f64, edges: &[String], extra: &str) -> String {
    format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{}]{extra}}}"#, edges.join(","))
}
fn chamfer(d: f64, edges: &[String], extra: &str) -> String {
    format!(r#"{{"type":"chamfer","id":"F3","chamferType":"equal","distance":{d},"edges":[{}]{extra}}}"#, edges.join(","))
}

const BASE: f64 = 672.0 * 10.0;
/// Cavity rim: arm walls 22 long each, back wall 24 long.
const RIM: f64 = 22.0 + 24.0 + 22.0;

fn rim_picks(split_pick: &str) -> Vec<String> {
    let (back, bx) = if split_pick == "left" { ("e6", 14.0) } else { ("e5", 26.0) };
    vec![pick("e4", 32.0, 19.0), pick(back, bx, 8.0), pick("e7", 8.0, 19.0)]
}

#[test]
fn fillet_rounds_the_whole_split_back_wall() {
    let r = 1.0;
    let section = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
    // reference: the unsplit sketch, all three rim edges picked
    let one = run(false, &fillet(r, &[pick("e4", 32.0, 19.0), pick("e5", 26.0, 8.0), pick("e7", 8.0, 19.0)], ""));
    assert!(one.errors.is_empty() && one.issues == 0, "reference: {:?} {}", one.errors, one.issues);
    // the two inner corners are concave in plan: the rolling ball keeps the full section along the rim
    let want = BASE - section * RIM;
    assert!((one.volume - want).abs() < 0.3, "reference volume {} vs {want}", one.volume);
    for which in ["right", "left"] {
        // both halves of the back wall picked
        let both = vec![pick("e4", 32.0, 19.0), pick("e5", 26.0, 8.0), pick("e6", 14.0, 8.0), pick("e7", 8.0, 19.0)];
        let o = run(true, &fillet(r, &both, ""));
        assert!(o.errors.is_empty() && o.issues == 0, "both halves: {:?} {}", o.errors, o.issues);
        assert!((o.volume - one.volume).abs() < 1e-3, "both halves volume {} vs {}", o.volume, one.volume);
        // only one half of the back wall picked, with and without propagate
        for extra in ["", r#","propagate":true"#] {
            let o = run(true, &fillet(r, &rim_picks(which), extra));
            let tag = format!("{which} half, extra {extra:?}");
            assert!(o.errors.is_empty() && o.issues == 0, "{tag}: errors {:?}, check issues {}", o.errors, o.issues);
            assert!(o.warnings.is_empty(), "{tag}: warnings {:?}", o.warnings);
            assert_eq!(o.faces, one.faces, "{tag}: face count");
            assert!((o.volume - one.volume).abs() < 1e-3, "{tag}: volume {} vs whole run {}", o.volume, one.volume);
        }
    }
}

#[test]
fn chamfer_rounds_the_whole_split_back_wall() {
    let d = 1.0;
    let one = run(false, &chamfer(d, &[pick("e4", 32.0, 19.0), pick("e5", 26.0, 8.0), pick("e7", 8.0, 19.0)], ""));
    assert!(one.errors.is_empty() && one.issues == 0, "reference: {:?} {}", one.errors, one.issues);
    assert!((one.volume - (BASE - d * d / 2.0 * RIM)).abs() < 1.0, "reference volume {}", one.volume);
    for which in ["right", "left"] {
        for extra in ["", r#","propagate":true"#] {
            let o = run(true, &chamfer(d, &rim_picks(which), extra));
            let tag = format!("{which} half, extra {extra:?}");
            assert!(o.errors.is_empty() && o.issues == 0, "{tag}: errors {:?}, check issues {}", o.errors, o.issues);
            assert!(o.warnings.is_empty(), "{tag}: warnings {:?}", o.warnings);
            assert_eq!(o.faces, one.faces, "{tag}: face count");
            assert!((o.volume - one.volume).abs() < 1e-3, "{tag}: volume {} vs whole run {}", o.volume, one.volume);
        }
    }
}


// ---- thin-wall variant: an open chain of four lines (two collinear back-wall pieces meeting at the origin, as a mirrored
// sketch leaves them, and two arms), extruded as a thin wall; the wall faces are named `F2.S.<entity>.W0`.

fn thin_body(split: bool) -> String {
    let back = if split {
        r#"{"id":"e2","type":"line","p1":"O","p2":"p2"},{"id":"e5","type":"line","p1":"O","p2":"p4"}"#
    } else {
        r#"{"id":"e2","type":"line","p1":"p4","p2":"p2"}"#
    };
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0}},{{"id":"p2","x":30,"y":0}},{{"id":"p3","x":30,"y":30}},{{"id":"p4","x":-30,"y":0}},{{"id":"p5","x":-30,"y":30}}],
"entities":[{back},{{"id":"e3","type":"line","p1":"p2","p2":"p3"}},{{"id":"e4","type":"line","p1":"p4","p2":"p5"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"thin":{{"thickness":6.5,"side":"inside","ends":"flat","cornerStyle":"mitre"}}}}"#
    )
}

fn thin_pick(e: &str, x: f64, y: f64) -> String {
    format!(r#"{{"faces":["F2.C1","F2.S.{e}.W0"],"at":[{x},{y},10]}}"#)
}

struct ThinOut {
    out: Out,
    /// the back-wall points (+/-11.75, 6.5, 10) each have a blend face of F3 within 1 mm
    pieces_covered: [bool; 2],
}

fn run_thin(split: bool, feature: &str) -> ThinOut {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{},{feature}]}}"#, thin_body(split));
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    let near = |x: f64| {
        let p = ck_geom::Vec3::new(x, 6.5, 10.0);
        b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F3.B")).any(|f| {
            b.face_edges(f).iter().any(|&e| {
                let ed = b.edge(e);
                ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 < 1.0
            })
        })
    };
    ThinOut {
        pieces_covered: [near(-11.75), near(11.75)],
        out: Out {
            errors: res.errors.iter().map(|e| e.error.code.clone()).collect(),
            warnings: res.warnings.iter().map(|e| e.error.code.clone()).collect(),
            faces: b.num_faces(),
            volume: ck_topo::mass::mass_props(b).volume,
            issues: ck_topo::check::check(b, 1e-6).len(),
        },
    }
}

fn thin_cases(make: &dyn Fn(&[String], &str) -> String, kind: &str) {
    let arm_l = thin_pick("e4", -23.5, 18.25);
    let arm_r = thin_pick("e3", 23.5, 18.25);
    let right = thin_pick("e2", 11.75, 6.5);
    let left = thin_pick("e5", -11.75, 6.5);
    let whole = thin_pick("e2", 11.75, 6.5);
    let reference = run_thin(false, &make(&[arm_l.clone(), whole, arm_r.clone()], ""));
    assert!(reference.out.errors.is_empty() && reference.out.issues == 0, "{kind} reference: {:?} {}", reference.out.errors, reference.out.issues);
    assert!(reference.pieces_covered == [true, true], "{kind} reference: back wall not rounded");
    let variants: [(&str, Vec<String>); 4] = [
        ("all four", vec![arm_l.clone(), right.clone(), left.clone(), arm_r.clone()]),
        ("right piece only", vec![arm_l.clone(), right.clone(), arm_r.clone()]),
        ("left piece only", vec![arm_l.clone(), left.clone(), arm_r.clone()]),
        ("left piece first", vec![left.clone(), arm_l.clone(), arm_r.clone(), right.clone()]),
    ];
    for (name, picks) in variants {
        for extra in ["", r#","propagate":true"#] {
            let t = run_thin(true, &make(&picks, extra));
            let tag = format!("{kind} {name}, extra {extra:?}");
            assert!(t.out.errors.is_empty() && t.out.issues == 0, "{tag}: errors {:?}, check issues {}", t.out.errors, t.out.issues);
            assert!(t.out.warnings.is_empty(), "{tag}: warnings {:?}", t.out.warnings);
            assert!(t.pieces_covered == [true, true], "{tag}: a back-wall piece has no blend face {:?}", t.pieces_covered);
            assert_eq!(t.out.faces, reference.out.faces, "{tag}: face count");
            assert!((t.out.volume - reference.out.volume).abs() < 1e-3, "{tag}: volume {} vs unsplit {}", t.out.volume, reference.out.volume);
        }
    }
}

#[test]
fn thin_wall_fillet_rounds_every_back_wall_piece() {
    thin_cases(&|picks, extra| fillet(0.5, picks, extra), "fillet");
}

#[test]
fn thin_wall_chamfer_rounds_every_back_wall_piece() {
    thin_cases(&|picks, extra| chamfer(0.5, picks, extra), "chamfer");
}
