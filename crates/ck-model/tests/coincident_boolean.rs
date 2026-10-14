//! Booleans whose tool faces coincide exactly with target faces (plugging a hole, flush bosses).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

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

/// Run a part; (errors as "F3:code {params}", volume, sorted face ids).
fn run(src: &str) -> (Vec<String>, f64, Vec<String>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    match ex.brep.body.as_ref() {
        Some(b) => {
            let iss = ck_check::validate(b, 1e-6);
            assert!(iss.is_empty(), "{:?}", iss.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
            let mut ids: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            ids.sort();
            (errs, ck_topo::mass::mass_props(b).volume, ids)
        }
        None => (errs, 0.0, vec![]),
    }
}


const FL: &str = r#","flip":true"#;
const PL: f64 = 60.0 * 40.0;
const CIRC: f64 = PI * 36.0;

/// Plate (-30,-20)-(30,20) with circles r=6 at (+-12,0) (e5 = left, e6 = right) on the front plane.
fn plate(features: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[
          {{"id":"p1","x":-30,"y":-20}},{{"id":"p2","x":30,"y":-20}},{{"id":"p3","x":30,"y":20}},{{"id":"p4","x":-30,"y":20}},
          {{"id":"ca","x":-12,"y":0}},{{"id":"cb","x":12,"y":0}}],
         "entities":[
          {{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}},
          {{"type":"circle","id":"e5","c":"ca","r":6}},{{"type":"circle","id":"e6","c":"cb","r":6}}],"constraints":[]}},
        {features}]}}"#
    )
}
fn e(id: &str, op: &str, depth: f64, regions: &str, extra: &str) -> String {
    format!(r#"{{"type":"extrude","id":"{id}","op":"{op}","sketch":"F1","end":"blind","depth":{depth},"regions":{regions}{extra}}}"#)
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-3 * b.max(1.0), "{a} vs {b}");
}
fn go(f: &[String]) -> (Vec<String>, f64, usize) {
    let (er, v, ids) = run(&plate(&f.join(",")));
    (er, v, ids.len())
}
fn base(extra: &str) -> String {
    e("F2", "add", 10.0, r#"["e1","e5"]"#, extra)
}

#[test]
fn plug_round_hole() {
    let (er, v, n) = go(&[base(""), e("F3", "add", 10.0, r#"["e6"]"#, "")]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0);
    assert_eq!(n, 6, "box");
}
#[test]
fn plug_round_hole_flipped() {
    let fl = r#","flip":true"#;
    let (er, v, n) = go(&[base(fl), e("F3", "add", 10.0, r#"["e6"]"#, fl)]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0);
    assert_eq!(n, 6);
}
#[test]
fn plug_hole_partially() {
    let (er, v, _) = go(&[base(""), e("F3", "add", 5.0, r#"["e6"]"#, "")]);
    assert!(er.is_empty(), "{er:?}");
    close(v, (PL - CIRC) * 10.0 + CIRC * 5.0);
}
#[test]
fn plug_hole_partially_flipped() {
    let fl = r#","flip":true"#;
    let (er, v, _) = go(&[base(fl), e("F3", "add", 5.0, r#"["e6"]"#, fl)]);
    assert!(er.is_empty(), "{er:?}");
    close(v, (PL - CIRC) * 10.0 + CIRC * 5.0);
}
#[test]
fn plug_longer_than_plate() {
    let (er, v, _) = go(&[base(""), e("F3", "add", 15.0, r#"["e6"]"#, "")]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0 + CIRC * 5.0);
}
#[test]
fn plug_longer_than_plate_flipped() {
    let fl = r#","flip":true"#;
    let (er, v, _) = go(&[base(fl), e("F3", "add", 15.0, r#"["e6"]"#, fl)]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0 + CIRC * 5.0);
}
#[test]
fn cut_tool_coincident_with_hole_wall() {
    // open hole e6 already there: cutting the same disc removes nothing
    let (er, v, _) = go(&[base(""), e("F3", "cut", 10.0, r#"["e6"]"#, FL)]);
    assert!(er.iter().all(|s| s.contains("cutNoEffect")), "{er:?}");
    close(v, (PL - CIRC) * 10.0);
}
#[test]
fn cut_filled_hole_back_out() {
    let (er, v, _) = go(&[base(""), e("F3", "cut", 10.0, r#"["e5"]"#, FL)]);
    assert!(er.is_empty(), "{er:?}");
    close(v, (PL - 2.0 * CIRC) * 10.0);
    let (er, v, _) = go(&[base(""), e("F3", "cut", 4.0, r#"["e5"]"#, FL)]);
    assert!(er.is_empty(), "{er:?}");
    close(v, (PL - CIRC) * 10.0 - CIRC * 4.0);
}

/// Plate (-30,-20)-(30,20) with a square hole (-10,-10)-(10,10): outer e1..e4, square e5..e8.
fn sq_plate(features: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[
          {{"id":"p1","x":-30,"y":-20}},{{"id":"p2","x":30,"y":-20}},{{"id":"p3","x":30,"y":20}},{{"id":"p4","x":-30,"y":20}},
          {{"id":"q1","x":-10,"y":-10}},{{"id":"q2","x":10,"y":-10}},{{"id":"q3","x":10,"y":10}},{{"id":"q4","x":-10,"y":10}}],
         "entities":[
          {{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}},
          {{"type":"line","id":"e5","p1":"q1","p2":"q2"}},{{"type":"line","id":"e6","p1":"q2","p2":"q3"}},
          {{"type":"line","id":"e7","p1":"q3","p2":"q4"}},{{"type":"line","id":"e8","p1":"q4","p2":"q1"}}],"constraints":[]}},
        {features}]}}"#
    )
}
const SQ: f64 = 400.0;

fn sq(f: &[String]) -> (Vec<String>, f64, usize) {
    let (er, v, ids) = run(&sq_plate(&f.join(",")));
    (er, v, ids.len())
}

#[test]
fn plug_square_hole() {
    for fl in ["", r#","flip":true"#] {
        let (er, v, n) = sq(&[e("F2", "add", 10.0, r#"["e1"]"#, fl), e("F3", "add", 10.0, r#"["e5"]"#, fl)]);
        assert!(er.is_empty(), "{er:?}");
        close(v, PL * 10.0);
        assert_eq!(n, 6, "box");
        let (er, v, _) = sq(&[e("F2", "add", 10.0, r#"["e1"]"#, fl), e("F3", "add", 5.0, r#"["e5"]"#, fl)]);
        assert!(er.is_empty(), "{er:?}");
        close(v, (PL - SQ) * 10.0 + SQ * 5.0);
        let (er, v, _) = sq(&[e("F2", "add", 10.0, r#"["e1"]"#, fl), e("F3", "add", 15.0, r#"["e5"]"#, fl)]);
        assert!(er.is_empty(), "{er:?}");
        close(v, PL * 10.0 + SQ * 5.0);
    }
}
#[test]
fn square_plate_and_plug_in_one_extrude_is_one_lump() {
    let (er, v, n) = sq(&[e("F2", "add", 10.0, r#"["e1","e5"]"#, "")]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0);
    assert_eq!(n, 6);
}
#[test]
fn boss_flush_with_pocket_wall() {
    let (er, v, _) = sq(&[
        e("F2", "add", 10.0, r#"["e1","e5"]"#, ""),
        e("F3", "cut", 5.0, r#"["e5"]"#, FL),
        e("F4", "add", 3.0, r#"["e5"]"#, ""),
    ]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0 - SQ * 2.0);
    let (er, v, n) = sq(&[
        e("F2", "add", 10.0, r#"["e1","e5"]"#, ""),
        e("F3", "cut", 5.0, r#"["e5"]"#, FL),
        e("F4", "add", 5.0, r#"["e5"]"#, ""),
    ]);
    assert!(er.is_empty(), "{er:?}");
    close(v, PL * 10.0);
    assert_eq!(n, 6);
}
#[test]
fn two_extrusions_sharing_a_face() {
    let f = vec![e("F2", "add", 10.0, r#"["e6"]"#, ""), e("F3", "add", 10.0, r#"["e6"]"#, r#","flip":true"#)];
    let (er, v, ids) = run(&plate(&f.join(",")));
    assert!(er.is_empty(), "{er:?}");
    close(v, CIRC * 20.0);
    assert_eq!(ids.len(), 3, "{ids:?}");
}
#[test]
fn cut_tool_side_on_an_existing_wall() {
    let f = vec![e("F2", "add", 10.0, r#"["e6"]"#, ""), e("F3", "cut", 4.0, r#"["e6"]"#, FL)];
    let (er, v, ids) = run(&plate(&f.join(",")));
    assert!(er.is_empty(), "{er:?}");
    close(v, CIRC * 6.0);
    assert_eq!(ids.len(), 3, "{ids:?}");
    let f = vec![e("F2", "add", 10.0, r#"["e6"]"#, ""), e("F3", "cut", 4.0, r#"["e6"]"#, "")];
    let (er, _, _) = run(&plate(&f.join(",")));
    assert!(er.iter().all(|s| s.contains("cutNoEffect")), "{er:?}");
}
#[test]
fn plug_the_hole_of_a_ring() {
    let src = |tail: &str| {
        format!(
            r#"{{"format":"ck-part","version":1,"features":[
            {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],
             "entities":[{{"type":"circle","id":"e5","c":"c","r":8}},{{"type":"circle","id":"e6","c":"c","r":3}}],"constraints":[]}},
            {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"regions":["e5"]}}{tail}]}}"#
        )
    };
    let (er, v, _) = run(&src(r#",{"type":"extrude","id":"F3","op":"add","sketch":"F1","end":"blind","depth":10,"regions":["e6"]}"#));
    assert!(er.is_empty(), "{er:?}");
    close(v, PI * 64.0 * 10.0);
}

#[test]
fn plug_a_hole_of_an_unfused_ring_and_disc() {
    // no `regions` on F2: the nested regions sweep as two touching lumps (oracle behaviour); a later union must cope
    let src = r#"{"format":"ck-part","version":1,"features":[
        {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],
         "entities":[{"type":"circle","id":"e5","c":"c","r":8},{"type":"circle","id":"e6","c":"c","r":3}],"constraints":[]},
        {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
        {"type":"extrude","id":"F3","op":"add","sketch":"F1","end":"blind","depth":10,"regions":["e6"]}]}"#;
    let (er, v, ids) = run(src);
    assert!(er.is_empty(), "{er:?}");
    close(v, PI * 64.0 * 10.0);
    assert_eq!(ids.len(), 3, "{ids:?}");
}
