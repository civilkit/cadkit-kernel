//! `frame` hardware: brackets with T-nuts and bolts, gussets, end caps and levelling feet (features/frame/hw.rs). Checked
//! against hand-worked placement (the inner corner of a joint sits half a section from the node on each side), the volume of a
//! bracket from its dimensions, parts counted against the bill of materials the reply carries, valid solids and no overlap
//! between hardware and members.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Exec(Brep);
impl FeatureExec for Exec {
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

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
    frame: Option<ck_json::Json>,
    warnings: usize,
}

fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let wire = r.to_json();
    let frame = wire.get_arr("frames").and_then(|a| a.last()).cloned();
    let warnings = wire.get_arr("warnings").map_or(0, |w| w.len());
    Out { warnings, body: ex.0.body.clone(), errors: r.errors.iter().map(|e| format!("{}:{} {:?}", e.feature, e.error.code, e.error.params)).collect(), frame }
}

const TS30: &str = r#"{"id":"tslot-3030","label":"T-slot 30","kind":"tslot","s":30,"slot":8.2,"lip":2.2,"inner":16.5,"depth":9,"bore":6.8}"#;
const BRACKET: &str = r#"{"id":"bracket-30","label":"Bracket 30","kind":"lbracket","spec":"28 x 28 x 3","arm":25,"t":3,"w":26,"hole":6.6,"holeAt":15,
  "tnut":{"id":"tnut-m6-30","label":"T-nut M6","kind":"nut","spec":"M6","l":10},
  "bolt":{"id":"bhcs-m6x10","label":"Screw M6 x 10","kind":"bolt","spec":"M6x10","d":6,"len":10,"headD":10.5,"headH":3.3}}"#;
const CAP: &str = r#"{"id":"cap-30","label":"End cap 30","kind":"cap","t":2}"#;
const FOOT: &str = r#"{"id":"foot-30","label":"Foot","kind":"foot","base":40,"baseH":6,"studD":6.6,"studLen":20,"neck":12}"#;

fn lines(ls: &[([f64; 3], [f64; 3])]) -> String {
    ls.iter().map(|(a, b)| format!("[[{},{},{}],[{},{},{}]]", a[0], a[1], a[2], b[0], b[1], b[2])).collect::<Vec<_>>().join(",")
}

fn frame(ls: &[([f64; 3], [f64; 3])], corner: &str, extra: &str) -> Out {
    run(&format!(r#"{{"type":"frame","id":"F1","lines":[{}],"profile":{TS30},"corner":"{corner}"{extra}}}"#, lines(ls)))
}

fn rect() -> Vec<([f64; 3], [f64; 3])> {
    vec![([0., 0., 0.], [600., 0., 0.]), ([600., 0., 0.], [600., 400., 0.]), ([600., 400., 0.], [0., 400., 0.]), ([0., 400., 0.], [0., 0., 0.])]
}

fn volume(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn lumps(o: &Out) -> Vec<(String, Body)> {
    let b = o.body.as_ref().expect("a body");
    b.lump_ids().map(|l| (b.lumps[l.idx()].name.clone().unwrap_or_default(), ck_ops::interference::extract_lump(b, l))).collect()
}

/// (id, qty) of the reply's parts.
fn parts(o: &Out) -> Vec<(String, usize)> {
    let hw = o.frame.as_ref().unwrap().get("hardware").expect("a hardware entry");
    hw.get_arr("parts").unwrap().iter().map(|p| (p.get_str("id").unwrap().to_string(), p.get_f64("qty").unwrap() as usize)).collect()
}

fn qty(o: &Out, id: &str) -> usize {
    parts(o).iter().find(|p| p.0 == id).map_or(0, |p| p.1)
}

/// The bill of materials counts exactly the hardware bodies, every body is named and valid, the frame body is valid.
fn check_counts(o: &Out, members: usize) {
    let b = o.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6).iter().map(|i| (i.code, i.detail.clone())).collect::<Vec<_>>());
    let total: usize = parts(o).iter().map(|p| p.1).sum();
    let named = lumps(o).iter().filter(|l| l.0.starts_with("F1 H")).count();
    assert_eq!(total, named, "bill of materials rows vs placed bodies");
    assert_eq!(b.num_lumps(), members + total);
    let hw = o.frame.as_ref().unwrap().get("hardware").unwrap();
    for p in hw.get_arr("parts").unwrap() {
        assert_eq!(p.get_arr("bodies").unwrap().len() as f64, p.get_f64("qty").unwrap());
    }
}

/// Hardware does not overlap any member or any other hardware (bounding boxes first, then the exact intersection).
fn check_no_overlap(o: &Out) {
    let ls = lumps(o);
    let boxes: Vec<_> = ls.iter().map(|l| l.1.bbox()).collect();
    for i in 0..ls.len() {
        for j in i + 1..ls.len() {
            if !ls[i].0.contains(" H") && !ls[j].0.contains(" H") {
                continue;
            }
            let (a, b) = (boxes[i], boxes[j]);
            let eps = 1e-6;
            if a.1.x < b.0.x - eps || b.1.x < a.0.x - eps || a.1.y < b.0.y - eps || b.1.y < a.0.y - eps || a.1.z < b.0.z - eps || b.1.z < a.0.z - eps {
                continue;
            }
            match ck_ops::boolean::boolean(&ls[i].1, &ls[j].1, ck_ops::boolean::Op::Intersect, 1e-6) {
                Ok(r) => {
                    let v = if r.body.num_faces() == 0 { 0.0 } else { volume(&r.body) };
                    assert!(v < 1e-4, "{} and {} overlap by {v} mm3", ls[i].0, ls[j].0);
                }
                Err(e) => panic!("{} and {}: intersection failed ({e})", ls[i].0, ls[j].0),
            }
        }
    }
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn mitred_rectangle_gets_a_bracket_with_two_nuts_and_bolts_per_corner() {
    let o = frame(&rect(), "mitre", &format!(r#","hardware":{{"joint":{BRACKET}}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.warnings, 0, "the hardware field is a known field of a frame");
    assert_eq!((qty(&o, "bracket-30"), qty(&o, "tnut-m6-30"), qty(&o, "bhcs-m6x10")), (4, 8, 8));
    check_counts(&o, 4);
    let ls = lumps(&o);
    // the corner at the origin is the first joint: the inner corner is half a section from each axis, the arms run 25 along each
    let (_, br) = ls.iter().find(|l| l.0 == "F1 H1").unwrap();
    let (lo, hi) = br.bbox();
    assert!(near(lo.x, 15.0, 1e-6) && near(hi.x, 40.0, 1e-6) && near(lo.y, 15.0, 1e-6) && near(hi.y, 40.0, 1e-6), "{lo:?} {hi:?}");
    assert!(near(lo.z, -13.0, 1e-6) && near(hi.z, 13.0, 1e-6), "width {lo:?} {hi:?}");
    // volume: L of two 25 x 3 arms (one 3 x 3 shared), 26 wide, less two 6.6 holes through 3
    let want = 26.0 * (2.0 * 25.0 * 3.0 - 9.0) - 2.0 * std::f64::consts::PI * 3.3 * 3.3 * 3.0;
    assert!(near(volume(br), want, 1e-6 * want), "{} vs {want}", volume(br));
    // the bolt heads sit on the outer faces of the arms: the first bolt's head ends 3 mm above the face of the arm
    let (_, bolt) = ls.iter().find(|l| l.0 == "F1 H3").unwrap();
    let (blo, bhi) = bolt.bbox();
    assert!(near(blo.x, 15.0 + 15.0 - 5.25, 1e-6) && near(bhi.x, 15.0 + 15.0 + 5.25, 1e-6), "{blo:?} {bhi:?}");
    check_no_overlap(&o);
}

#[test]
fn butted_u_frame_gets_brackets_caps_and_tee_brackets() {
    // a U (two corners) with a middle rail ending on the first leg: 3 joints, 3 free ends
    let ls = [([0., 0., 0.], [600., 0., 0.]), ([600., 0., 0.], [600., 400., 0.]), ([0., 0., 0.], [0., 400., 0.]), ([300., 0., 0.], [300., 400., 0.])];
    let o = frame(&ls, "butt", &format!(r#","hardware":{{"joint":{BRACKET},"cap":{CAP}}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    // e1 runs through both corners; the tee at x = 300 is a third joint; ends at y = 400 are free: 3 caps
    assert_eq!(qty(&o, "bracket-30"), 3, "{:?}", parts(&o));
    assert_eq!(qty(&o, "cap-30"), 3);
    assert_eq!(qty(&o, "bhcs-m6x10"), 6);
    check_counts(&o, 4);
    // a cap is the 30 x 30 section outline, 2 thick, flush against the end face (the free end at y = 400 for the rail)
    let ls = lumps(&o);
    let caps: Vec<_> = ls.iter().filter(|l| { let (lo, hi) = l.1.bbox(); near(hi.x - lo.x, 30.0, 1e-6) && near(hi.y - lo.y, 2.0, 1e-6) }).collect();
    assert_eq!(caps.len(), 3);
    for c in caps {
        assert!(near(c.1.bbox().0.y, 400.0, 1e-6) && near(volume(&c.1), 30.0 * 30.0 * 2.0, 1e-6), "{:?}", c.1.bbox());
    }
    check_no_overlap(&o);
}

#[test]
fn a_joint_override_removes_or_replaces_one_connector() {
    let hw = format!(r#","hardware":{{"joint":{BRACKET}}},"joints":[{{"segments":["L0","L1"],"connector":"none"}}]"#);
    let o = frame(&rect(), "mitre", &hw);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(qty(&o, "bracket-30"), 3);
    let joints = o.frame.as_ref().unwrap().get("hardware").unwrap().get_arr("joints").unwrap().to_vec();
    assert_eq!(joints.len(), 4);
    let none: Vec<_> = joints.iter().filter(|j| matches!(j.get("connector"), Some(ck_json::Json::Null))).collect();
    assert_eq!(none.len(), 1);
    check_counts(&o, 4);
}

#[test]
fn bracket_needs_ninety_degrees_gusset_does_not() {
    let h = 500.0 * 3f64.sqrt() / 2.0;
    let tri = [([0., 0., 0.], [500., 0., 0.]), ([500., 0., 0.], [250., h, 0.]), ([250., h, 0.], [0., 0., 0.])];
    let o = frame(&tri, "mitre", &format!(r#","hardware":{{"joint":{BRACKET}}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(parts(&o).len(), 0);
    let joints = o.frame.as_ref().unwrap().get("hardware").unwrap().get_arr("joints").unwrap().to_vec();
    assert!(joints.iter().all(|j| j.get_str("reason").unwrap().contains("90")), "{joints:?}");
    let g = r#"{"id":"gusset-3","label":"Gusset 3","kind":"gusset","leg":60,"t":3}"#;
    let o = frame(&tri, "mitre", &format!(r#","hardware":{{"joint":{g}}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(qty(&o, "gusset-3"), 3);
    check_counts(&o, 3);
    // a 60 degree gusset: triangle of legs 60 and angle 60 = 60 x 60 x sin(60) / 2, 3 thick
    let ls = lumps(&o);
    let (_, g1) = ls.iter().find(|l| l.0 == "F1 H1").unwrap();
    let want = 0.5 * 60.0 * 60.0 * (60f64).to_radians().sin() * 3.0;
    assert!(near(volume(g1), want, 1e-6 * want), "{} vs {want}", volume(g1));
    check_no_overlap(&o);
}

#[test]
fn a_leg_standing_on_the_floor_gets_a_foot_and_the_top_end_a_cap() {
    // two legs 500 high joined by a rail at the top: both bottoms are free and on the floor, the rail ends are corners
    let ls = [([0., 0., 0.], [0., 500., 0.]), ([600., 0., 0.], [600., 500., 0.]), ([0., 500., 0.], [600., 500., 0.])];
    let o = frame(&ls, "butt", &format!(r#","hardware":{{"cap":{CAP},"foot":{FOOT}}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(qty(&o, "foot-30"), 2);
    assert_eq!(parts(&o).iter().find(|p| p.0 == "cap-30"), None, "tops meet the rail: no free end left to cap");
    check_counts(&o, 3);
    let ls = lumps(&o);
    let (_, f1) = ls.iter().find(|l| l.0 == "F1 H1").unwrap();
    let (lo, hi) = f1.bbox();
    // pad 40 across, stud 20 up the bore, 12 neck then a 6 pad below the end face at y = 0
    assert!(near(lo.y, -18.0, 1e-6) && near(hi.y, 20.0, 1e-6) && near(hi.x - lo.x, 40.0, 1e-6), "{lo:?} {hi:?}");
    check_no_overlap(&o);
}

#[test]
fn bad_hardware_is_frame_invalid_and_no_hardware_changes_nothing() {
    let bad = BRACKET.replace("\"len\":10", "\"len\":14");
    let o = frame(&rect(), "mitre", &format!(r#","hardware":{{"joint":{bad}}}"#));
    assert!(o.errors.iter().any(|e| e.starts_with("F1:frameInvalid") && e.contains("bolt")), "{:?}", o.errors);
    let o = frame(&rect(), "mitre", "");
    assert!(o.errors.is_empty());
    assert!(o.frame.as_ref().unwrap().get("hardware").is_none());
    assert_eq!(o.body.as_ref().unwrap().num_lumps(), 4);
}

#[test]
fn every_t_slot_series_gets_hardware_that_fits_its_slot() {
    // the three series of the kit library: 20 (slot 6.2), 30 (8.2), 40 (8.2): brackets, nuts and screws touch the stock nowhere
    let series = [
        (r#"{"id":"tslot-2020","kind":"tslot","s":20,"slot":6.2,"lip":1.8,"inner":11,"depth":6.1,"bore":4.2}"#, 20.0, 3.0, 12.0, 5.0, 8.0),
        (r#"{"id":"tslot-4040","kind":"tslot","s":40,"slot":8.2,"lip":4.3,"inner":20,"depth":12.2,"bore":7.3}"#, 35.0, 4.0, 20.0, 8.0, 12.0),
    ];
    for (prof, arm, t, hole_at, d, len) in series {
        let br = format!(
            r#"{{"id":"b","kind":"lbracket","arm":{arm},"t":{t},"w":{},"hole":{},"holeAt":{hole_at},"tnut":{{"id":"n","kind":"nut","l":8}},"bolt":{{"id":"s","kind":"bolt","d":{d},"len":{len},"headD":{},"headH":{}}}}}"#,
            arm - 2.0,
            d + 0.6,
            d * 1.75,
            d * 0.55
        );
        let o = run(&format!(r#"{{"type":"frame","id":"F1","lines":[{}],"profile":{prof},"corner":"mitre","hardware":{{"joint":{br}}}}}"#, lines(&rect())));
        assert!(o.errors.is_empty(), "{:?}", o.errors);
        assert_eq!((qty(&o, "b"), qty(&o, "n"), qty(&o, "s")), (4, 8, 8));
        check_counts(&o, 4);
        check_no_overlap(&o);
    }
}

const CORNER3: &str = r#"{"id":"corner3-30","label":"3-way corner 30","kind":"corner3","block":20,"arm":25,"t":3,"w":26,"hole":6.6,"holeAt":15,
  "tnut":{"id":"tnut-m6-30","label":"T-nut M6","kind":"nut","l":10},
  "bolt":{"id":"bhcs-m6x10","label":"Screw","kind":"bolt","d":6,"len":10,"headD":10.5,"headH":3.3}}"#;

fn corner_lines() -> Vec<([f64; 3], [f64; 3])> {
    vec![([0., 0., 0.], [300., 0., 0.]), ([0., 0., 0.], [0., 300., 0.]), ([0., 0., 0.], [0., 0., 300.])]
}

fn bbox_of(o: &Out, name: &str) -> (ck_core::linalg::Vec3, ck_core::linalg::Vec3) {
    lumps(o).iter().find(|l| l.0 == name).unwrap().1.bbox()
}

#[test]
fn a_tee_takes_a_bracket_on_each_side_when_asked() {
    let ls = [([0., 0., 0.], [600., 0., 0.]), ([300., 0., 0.], [300., 400., 0.])];
    let one = frame(&ls, "butt", &format!(r#","hardware":{{"joint":{BRACKET}}}"#));
    assert_eq!(qty(&one, "bracket-30"), 1, "{:?}", one.errors);
    let both = frame(&ls, "butt", &format!(r#","hardware":{{"joint":{BRACKET},"teeBoth":true}}"#));
    assert!(both.errors.is_empty(), "{:?}", both.errors);
    assert_eq!((qty(&both, "bracket-30"), qty(&both, "tnut-m6-30"), qty(&both, "bhcs-m6x10")), (2, 4, 4));
    check_counts(&both, 2);
    // one on each side of the rail: inner corners at x = 315 and x = 285, arms running outwards
    let xs: Vec<(f64, f64)> = ["F1 H1", "F1 H6"].iter().map(|n| { let (lo, hi) = bbox_of(&both, n); (lo.x, hi.x) }).collect();
    assert!(near(xs[0].0, 315.0, 1e-6) && near(xs[0].1, 340.0, 1e-6), "{xs:?}");
    assert!(near(xs[1].0, 260.0, 1e-6) && near(xs[1].1, 285.0, 1e-6), "{xs:?}");
    check_no_overlap(&both);
    // per joint: `both` on one tee only
    let two = [([0., 0., 0.], [600., 0., 0.]), ([200., 0., 0.], [200., 400., 0.]), ([400., 0., 0.], [400., 400., 0.])];
    let o = frame(&two, "butt", &format!(r#","hardware":{{"joint":{BRACKET}}},"joints":[{{"segments":["L0","L2"],"both":true}}]"#));
    assert_eq!(qty(&o, "bracket-30"), 3, "{:?}", o.errors);
    check_counts(&o, 3);
}

#[test]
fn a_three_way_corner_is_one_block_with_three_plates() {
    let hw = format!(r#","hardware":{{"joint":{BRACKET},"corner3":{CORNER3}}}"#);
    let o = frame(&corner_lines(), "butt", &hw);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    // the block replaces the pairwise brackets
    assert_eq!((qty(&o, "corner3-30"), qty(&o, "bracket-30"), qty(&o, "tnut-m6-30"), qty(&o, "bhcs-m6x10")), (1, 0, 3, 3));
    check_counts(&o, 3);
    let (lo, hi) = bbox_of(&o, "F1 H1");
    // cube 20 in the inner octant from (15,15,15), a plate on each member's face reaching 25 along it and 26 across, 11 past the axis
    for (l, h) in [(lo.x, hi.x), (lo.y, hi.y), (lo.z, hi.z)] {
        assert!(near(l, -11.0, 1e-6) && near(h, 40.0, 1e-6), "{lo:?} {hi:?}");
    }
    let v = volume(&lumps(&o).iter().find(|l| l.0 == "F1 H1").unwrap().1);
    let want = 8000.0 + 3.0 * (25.0 * 26.0 * 3.0 - std::f64::consts::PI * 3.3 * 3.3 * 3.0);
    assert!(near(v, want, 1e-6 * want), "{v} vs {want}");
    check_no_overlap(&o);
    // the report lists both joints of the node with the block
    let js = o.frame.as_ref().unwrap().get("hardware").unwrap().get_arr("joints").unwrap().to_vec();
    assert!(js.iter().all(|j| j.get_str("connector") == Some("corner3-30")), "{js:?}");
    // a joint with its own connector at the node falls back to pairwise brackets
    let hw2 = format!(r#","hardware":{{"joint":{BRACKET},"corner3":{CORNER3}}},"joints":[{{"segments":["L0","L1"],"connector":"none"}}]"#);
    let o2 = frame(&corner_lines(), "butt", &hw2);
    assert_eq!((qty(&o2, "corner3-30"), qty(&o2, "bracket-30")), (0, 1), "{:?}", o2.errors);
    check_counts(&o2, 3);
    // unequal sections or a node that is not square get no block (reason given)
    let slanted = [([0., 0., 0.], [300., 0., 0.]), ([0., 0., 0.], [0., 300., 0.]), ([0., 0., 0.], [100., 0., 300.])];
    let o3 = frame(&slanted, "butt", &hw);
    assert!(o3.errors.is_empty(), "{:?}", o3.errors);
    assert_eq!(qty(&o3, "corner3-30"), 0);
}

const SHS40: &str = r#"{"id":"shs-40x40x2","kind":"rectTube","w":40,"h":40,"t":2}"#;
const TUBE_BRACKET: &str = r#"{"id":"bracket-tube-40","label":"Tube bracket 40","kind":"lbracket","arm":35,"t":4,"w":36,"hole":6.6,"holeAt":20,
  "bolt":{"id":"hex-m6","label":"Hex bolt M6","kind":"bolt","spec":"M6","d":6,"headD":10,"headH":4},
  "nut":{"id":"nut-m6","label":"Hex nut M6","kind":"nut","spec":"M6","af":10,"t":5}}"#;

fn tube(ls: &[([f64; 3], [f64; 3])], corner: &str, extra: &str) -> Out {
    run(&format!(r#"{{"type":"frame","id":"F1","lines":[{}],"profile":{SHS40},"corner":"{corner}"{extra}}}"#, lines(ls)))
}

#[test]
fn tube_joints_are_drilled_and_bolted_when_asked() {
    let off = tube(&rect(), "mitre", &format!(r#","hardware":{{"joint":{TUBE_BRACKET}}}"#));
    assert!(off.errors.is_empty(), "{:?}", off.errors);
    // no drilling: the bracket plates only
    assert_eq!(parts(&off).iter().map(|p| p.0.as_str()).collect::<Vec<_>>(), ["bracket-tube-40"]);
    let o = tube(&rect(), "mitre", &format!(r#","hardware":{{"joint":{TUBE_BRACKET},"drill":true}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    // bolt long enough for plate 4 + tube 40 + nut 5 + 1, in steps of 5: 50
    assert_eq!((qty(&o, "bracket-tube-40"), qty(&o, "hex-m6x50"), qty(&o, "nut-m6")), (4, 8, 8), "{:?}", parts(&o));
    check_counts(&o, 4);
    // every member has two holes through both 2 mm walls: the volume is the section times the length less four wall cylinders... per hole 2 x 2 x pi r^2
    let a = 40.0 * 40.0 - 36.0 * 36.0;
    let hole = 2.0 * 2.0 * std::f64::consts::PI * 3.3 * 3.3;
    let ms = o.frame.as_ref().unwrap().get_arr("members").unwrap().to_vec();
    for m in &ms {
        let cl = if m.get_str("segment") == Some("L0") || m.get_str("segment") == Some("L2") { 600.0 } else { 400.0 };
        assert_eq!(m.get_f64("holes"), Some(2.0));
        let want = a * cl - 2.0 * hole;
        assert!(near(m.get_f64("volume").unwrap(), want, 1e-4 * want), "{} vs {want}", m.get_f64("volume").unwrap());
    }
    check_no_overlap(&o);
    // a round tube has no flat face for a bracket: skipped with a reason, nothing drilled
    let chs = run(&format!(r#"{{"type":"frame","id":"F1","lines":[{}],"profile":{{"id":"chs","kind":"roundTube","d":33.7,"t":2.6}},"corner":"mitre","hardware":{{"joint":{TUBE_BRACKET},"drill":true}}}}"#, lines(&rect())));
    assert!(chs.errors.is_empty(), "{:?}", chs.errors);
    assert_eq!(parts(&chs).len(), 0);
}

#[test]
fn a_three_way_block_on_tube_is_bolted_through_drilled_holes() {
    let block = r#"{"id":"corner3-tube-40","label":"3-way corner tube 40","kind":"corner3","block":25,"arm":35,"t":4,"w":36,"hole":6.6,"holeAt":20,
      "bolt":{"id":"hex-m6","label":"Hex bolt M6","kind":"bolt","spec":"M6","d":6,"headD":10,"headH":4},
      "nut":{"id":"nut-m6","label":"Hex nut M6","kind":"nut","spec":"M6","af":10,"t":5}}"#;
    let o = tube(&corner_lines(), "butt", &format!(r#","hardware":{{"corner3":{block},"drill":true}}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!((qty(&o, "corner3-tube-40"), qty(&o, "hex-m6x50"), qty(&o, "nut-m6")), (1, 3, 3), "{:?}", parts(&o));
    check_counts(&o, 3);
    check_no_overlap(&o);
}

#[test]
fn joints_name_their_hardware_bodies_and_a_library_field_is_known() {
    let o = frame(&rect(), "mitre", &format!(r#","hardware":{{"joint":{BRACKET}}},"library":{{"profiles":[]}}"#));
    assert!(o.errors.is_empty() && o.warnings == 0, "{:?} {}", o.errors, o.warnings);
    let js = o.frame.as_ref().unwrap().get("hardware").unwrap().get_arr("joints").unwrap().to_vec();
    // first joint: the bracket H1, its T-nut and screw per arm (H2..H5)
    let b: Vec<&str> = js[0].get_arr("bodies").unwrap().iter().filter_map(|x| x.as_str()).collect();
    assert_eq!(b, ["F1 H1", "F1 H2", "F1 H3", "F1 H4", "F1 H5"]);
    let all: usize = js.iter().map(|j| j.get_arr("bodies").unwrap().len()).sum();
    assert_eq!(all, 20, "every placed body belongs to exactly one joint");
}
