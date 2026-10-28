//! A `hole` feature with many positions builds its tool without quadratic cost (issue 33). The tool bodies
//! used to be folded one by one into a growing union; now only the holes whose own boxes meet are united.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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

/// A plate with an `n` x `n` grid of through holes (diameter 3, pitch `pitch`) in one `hole` feature.
fn grid(n: usize, pitch: f64) -> String {
    let side = n as f64 * pitch + 10.0;
    let mut pts = String::new();
    let mut ents = String::new();
    for i in 0..n {
        for j in 0..n {
            let k = i * n + j;
            let sep = if k == 0 { "" } else { "," };
            pts += &format!(r#"{sep}{{"id":"q{k}","x":{},"y":{}}}"#, 5.0 + i as f64 * pitch, 5.0 + j as f64 * pitch);
            ents += &format!(r#"{sep}{{"type":"point","id":"d{k}","p":"q{k}"}}"#);
        }
    }
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{side},"y":0}},{{"id":"p3","x":{side},"y":{side}}},{{"id":"p4","x":0,"y":{side}}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":10,"name":"top"}},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4","kind":"simple","diameter":3,"end":"through"}}
 ]}}"#
    )
}

struct Out {
    errors: Vec<String>,
    volume: f64,
    faces: usize,
    names: Vec<String>,
    ms: f64,
}

fn run(src: &str) -> Out {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let ms = t.elapsed().as_secs_f64() * 1000.0;
    let b = ex.0.body.as_ref().expect("body");
    let mut names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    names.sort();
    Out {
        errors: res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect(),
        volume: ck_topo::mass::mass_props(b).volume,
        faces: b.num_faces(),
        names,
        ms,
    }
}

/// FNV-1a over the sorted face names.
fn digest(names: &[String]) -> u64 {
    let mut h: u64 = 0xcbf29ce484222325;
    for n in names {
        for b in n.bytes().chain(std::iter::once(b'\n')) {
            h = (h ^ b as u64).wrapping_mul(0x100000001b3);
        }
    }
    h
}

fn summary(o: &Out) -> String {
    format!("faces={} volume={:.4} digest={:016x}", o.faces, o.volume, digest(&o.names))
}

fn check(n: usize, pitch: f64, faces: usize, volume: f64, dig: u64) {
    let o = run(&grid(n, pitch));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.faces, faces, "faces");
    assert!((o.volume - volume).abs() < 1e-3, "volume {} want {volume}", o.volume);
    assert_eq!(digest(&o.names), dig, "face names changed: {}", summary(&o));
}

// The expected values below were taken from the previous code path (every hole folded into one growing union)
// for the 10 x 10 grid, the overlapping grid and the 3 x 3 grid; the result must not change.

#[test]
fn grid_of_clear_holes_keeps_faces_names_and_volume() {
    check(10, 8.0, 106, 73931.4165, 0x3a395ed2412e8e55);
    check(3, 8.0, 15, 10923.8275, 0x34fe47d92aebeff6);
}

#[test]
fn grid_of_overlapping_holes_is_united() {
    // pitch 2 with diameter 3: every hole meets its neighbours, so the union path still runs
    check(4, 2.0, 18, 2478.1236, 0x704e384e274de963);
}

#[test]
fn thirty_by_thirty_holes_in_one_feature_is_not_quadratic() {
    // 900 points: 748 s before the fix (131 s for 20 x 20); a few seconds now
    let o = run(&grid(30, 8.0));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.faces, 906);
    assert!((o.volume - 561382.7488).abs() < 1e-2, "volume {}", o.volume);
    assert!(o.ms < 15_000.0, "30 x 30 holes took {:.0} ms", o.ms);
}
