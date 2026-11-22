//! `ck_ops::interference::reply` (with the face-box prefilter) against `reply_exhaustive` (a full `boolean(Intersect)` for every
//! pair of lumps whose boxes touch): the replies must be identical on every document. Constructed cases cover touching faces,
//! nesting, overlap, a hair gap, a domed face whose box is larger than the vertex box, a bezel around a disc and a point kiss;
//! the speedboat dash (36 lumps, a lettered dial against a bezel, 43 box-touching pairs) is the slow real case.
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

fn regen_doc(text: &str) -> Body {
    let part = Part::parse(text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect::<Vec<_>>());
    ex.0.body.clone().expect("a body")
}

/// One solid as its own document: a rectangle `[x0,x1] x [y0,y1]` extruded from z = `z0` by `h` (feature ids `{id}`).
fn block(id: &str, x0: f64, y0: f64, x1: f64, y1: f64, z0: f64, h: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},{{"type":"extrude","id":"{id}","op":"add","sketch":"S{id}","end":"blind","depth":{h},"offset":{z0}}}]}}"#
    )
}

/// A disc (`r_in` = 0) or a ring (inner radius `r_in`) about (cx, cy), from z = `z0`, height `h`, optionally domed `dome` mm on its top.
fn round(id: &str, cx: f64, cy: f64, r: f64, r_in: f64, z0: f64, h: f64, dome: f64) -> String {
    let inner = if r_in > 0.0 { format!(r#",{{"type":"circle","id":"k2","c":"c","r":{r_in}}}"#) } else { String::new() };
    let d = if dome > 0.0 { format!(r#",{{"type":"dome","id":"D{id}","face":"{id}.C1","height":{dome}}}"#) } else { String::new() };
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"type":"circle","id":"k","c":"c","r":{r}}}{inner}],"constraints":[]}},{{"type":"extrude","id":"{id}","op":"add","sketch":"S{id}","end":"blind","depth":{h},"offset":{z0}}}{d}]}}"#
    )
}

fn assemble(docs: &[String]) -> Body {
    let mut it = docs.iter().map(|d| regen_doc(d));
    let mut b = it.next().unwrap();
    for o in it {
        b.append(&o);
    }
    b
}

/// Both replies as text; they must be equal. Returns the number of pairs reported.
fn same(name: &str, body: &Body) -> usize {
    let new = ck_json::to_string(&ck_ops::interference::reply(body));
    let old = ck_json::to_string(&ck_ops::interference::reply_exhaustive(body));
    assert_eq!(new, old, "{name}: prefiltered reply differs from the exhaustive one");
    ck_json::parse(&new).unwrap().get_arr("pairs").unwrap().len()
}

#[test]
fn constructed_cases_match_the_exhaustive_reply() {
    // overlapping: one pair
    assert_eq!(same("overlap", &assemble(&[block("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), block("B", 5.0, 5.0, 15.0, 15.0, 0.0, 10.0)])), 1);
    // touching faces (flush) and a hair gap: nothing reported
    assert_eq!(same("flush", &assemble(&[block("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), block("B", 10.0, 0.0, 20.0, 10.0, 0.0, 10.0)])), 0);
    assert_eq!(same("hair", &assemble(&[block("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), block("B", 10.000000001, 0.0, 20.0, 10.0, 0.0, 10.0)])), 0);
    // kissing at a point (corner to corner)
    assert_eq!(same("point", &assemble(&[block("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), block("B", 10.0, 10.0, 20.0, 20.0, 10.0, 10.0)])), 0);
    // nested: a block inside a bigger one with no contact (swallowed: one pair, volume 1000)
    assert_eq!(same("nested", &assemble(&[block("A", 0.0, 0.0, 100.0, 100.0, 0.0, 100.0), block("B", 40.0, 40.0, 50.0, 50.0, 40.0, 10.0)])), 1);
    // a disc inside a bezel ring with clearance (box nested, solids disjoint), and a pin through the ring's hole
    assert_eq!(same("bezel", &assemble(&[round("A", 0.0, 0.0, 20.0, 15.0, 0.0, 10.0, 0.0), round("B", 0.0, 0.0, 14.0, 0.0, 0.0, 10.0, 0.0)])), 0);
    assert_eq!(same("pin", &assemble(&[round("A", 0.0, 0.0, 20.0, 15.0, 0.0, 10.0, 0.0), block("B", -1.0, -1.0, 1.0, 1.0, -5.0, 30.0)])), 0);
    // a domed cylinder (box of vertices and edges stops at the rim, the dome rises 8 above it) with a block in the dome, and one beside it
    let dome = round("A", 0.0, 0.0, 10.0, 0.0, 0.0, 20.0, 8.0);
    assert_eq!(same("dome in", &assemble(&[dome.clone(), block("B", -1.0, -1.0, 1.0, 1.0, 19.0, 6.0)])), 1);
    assert_eq!(same("dome beside", &assemble(&[dome.clone(), block("B", 8.0, 8.0, 9.0, 9.0, 19.0, 6.0)])), 0);
    // two cylinders whose boxes touch at a corner but whose discs are far apart
    assert_eq!(same("cyl corner", &assemble(&[round("A", 0.0, 0.0, 10.0, 0.0, 0.0, 10.0, 0.0), round("B", 19.0, 19.0, 10.0, 0.0, 0.0, 10.0, 0.0)])), 0);
}

const DASH: &str = include_str!("fixtures/speedboat_dash.ckpart.json");

/// The speedboat dash: identical replies; with `INTERFERENCE_TIMES=1` the two timings are printed.
#[test]
fn speedboat_dash_matches_the_exhaustive_reply() {
    let body = regen_doc(DASH);
    let t = std::time::Instant::now();
    let new = ck_json::to_string(&ck_ops::interference::reply(&body));
    let t_new = t.elapsed();
    let t = std::time::Instant::now();
    let old = ck_json::to_string(&ck_ops::interference::reply_exhaustive(&body));
    let t_old = t.elapsed();
    if std::env::var("INTERFERENCE_TIMES").is_ok() {
        eprintln!("interference on {} lumps: prefiltered {:.0} ms, exhaustive {:.0} ms", body.num_lumps(), t_new.as_secs_f64() * 1e3, t_old.as_secs_f64() * 1e3);
    }
    assert_eq!(new, old);
}

use ck_ops::interference::{reply_budgeted, Cap};

fn pairs_of(j: &ck_json::Json) -> usize {
    j.get_arr("pairs").unwrap().len()
}

/// Under the cap the budgeted reply is byte-identical to the plain one (no extra field); over it, it says how many pairs it left.
#[test]
fn the_cap_truncates_and_says_so() {
    // A overlaps B, B overlaps C (A and C are apart): 2 candidate pairs, both real overlaps
    let body = assemble(&[block("A", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), block("B", 5.0, 0.0, 25.0, 10.0, 0.0, 10.0), block("C", 20.0, 0.0, 30.0, 10.0, 0.0, 10.0)]);
    let plain = ck_json::to_string(&ck_ops::interference::reply(&body));
    let roomy = ck_json::to_string(&reply_budgeted(&body, Cap { ms: 1e9, pairs: 99 }));
    assert_eq!(roomy, plain);
    assert_eq!(roomy, ck_json::to_string(&ck_ops::interference::reply_exhaustive(&body)));
    assert!(!plain.contains("truncated") && !plain.contains("uncheckedPairs"));
    let cut = reply_budgeted(&body, Cap { ms: 1e9, pairs: 1 });
    assert_eq!(cut.get_bool("truncated"), Some(true), "{}", ck_json::to_string(&cut));
    assert_eq!(cut.get_f64("uncheckedPairs"), Some(1.0));
    assert_eq!(pairs_of(&cut), 1);
    let none = reply_budgeted(&body, Cap { ms: 1e9, pairs: 0 });
    assert_eq!((none.get_f64("uncheckedPairs"), pairs_of(&none)), (Some(2.0), 0));
}

/// The dash under the default pair cap: truncated, cheap pairs first; prints the time with `INTERFERENCE_TIMES=1`.
#[test]
fn the_dash_overlap_check_stops_at_its_cap() {
    let body = regen_doc(DASH);
    let t = std::time::Instant::now();
    let r = reply_budgeted(&body, Cap { ms: 10_000.0, pairs: 12 });
    let ms = t.elapsed().as_secs_f64() * 1e3;
    if std::env::var("INTERFERENCE_TIMES").is_ok() {
        eprintln!("capped interference on the dash: {ms:.0} ms, {} pairs reported, {} unchecked", pairs_of(&r), r.get_f64("uncheckedPairs").unwrap_or(0.0));
    }
    assert_eq!(r.get_bool("truncated"), Some(true));
    assert_eq!(r.get_f64("uncheckedPairs"), Some(31.0));
    assert!(pairs_of(&r) <= 12);
}
