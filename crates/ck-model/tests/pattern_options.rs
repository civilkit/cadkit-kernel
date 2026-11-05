//! Linear pattern per-direction options (Spacing / Extent / To Object, a second direction), the path pattern
//! (`curvepattern`: start point, flip, fixed or tangent orientation) and the warnings a pattern gives when copies
//! change nothing, lie apart from the body or overlap each other. Volumes and copy positions are closed form.
use ck_geom::surface::Surface;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts, RegenResult};
use ck_model::Part;
use std::f64::consts::PI;

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

/// 100 x 100 x 10 plate (z 0..10), one through hole of diameter 4 at (hx, hy), a plane `P9` at x = 80, then `tail`.
fn plate_at(hx: f64, hy: f64, tail: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[
     {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":100,"y":0}},{{"id":"p3","x":100,"y":100}},{{"id":"p4","x":0,"y":100}}],
   "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
     {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
  {{"type":"extrude","id":"E2","op":"add","sketch":"S1","end":"blind","depth":10}},
  {{"type":"plane","id":"P3","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":10,"name":"top"}},
  {{"type":"sketch","id":"S4","plane":{{"kind":"feature","id":"P3"}},"points":[{{"id":"p1","x":{hx},"y":{hy}}}],
   "entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
  {{"type":"hole","id":"H5","sketch":"S4","kind":"simple","diameter":4,"end":"through"}},
  {{"type":"plane","id":"P9","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":80}}{tail}
 ]}}"#
    )
}
fn plate(tail: &str) -> String {
    plate_at(10.0, 10.0, tail)
}

struct Run {
    res: RegenResult,
    body: ck_topo::body::Body,
}
impl Run {
    fn errors(&self) -> Vec<String> {
        self.res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn warnings(&self) -> Vec<String> {
        self.res.warnings.iter().map(|w| format!("{}:{}", w.feature, w.error.describe())).collect()
    }
    fn warning(&self, code: &str) -> Option<&regen::FeatureError> {
        self.res.warnings.iter().find(|w| w.error.code == code).map(|w| &w.error)
    }
    fn error(&self, code: &str) -> Option<&regen::FeatureError> {
        self.res.errors.iter().find(|w| w.error.code == code).map(|w| &w.error)
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(&self.body).volume
    }
    /// Centres (x, y) of the hole walls, sorted.
    fn holes(&self) -> Vec<(f64, f64)> {
        let mut v: Vec<(f64, f64)> = self
            .body
            .face_ids()
            .filter_map(|f| match &self.body.face(f).surface {
                Surface::Cylinder { frame, r } if (*r - 2.0).abs() < 1e-9 => Some((round(frame.o.x), round(frame.o.y))),
                _ => None,
            })
            .collect();
        v.sort_by(|a, b| a.partial_cmp(b).unwrap());
        v.dedup();
        v
    }
}
fn round(x: f64) -> f64 {
    let r = (x * 1e6).round() / 1e6;
    if r == 0.0 { 0.0 } else { r }
}

fn run(src: &str) -> Run {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Run { res, body: ex.0.body.clone().expect("body") }
}

const HOLE: f64 = PI * 4.0 * 10.0;
fn vol(holes: usize) -> f64 {
    100.0 * 100.0 * 10.0 - holes as f64 * HOLE
}

fn lp(extra: &str) -> String {
    format!(r#",{{"type":"lpattern","id":"L6","seeds":["H5"],"dir":{{"kind":"axis","axis":"x"}}{extra}}}"#)
}

fn xs(r: &Run) -> Vec<f64> {
    r.holes().iter().map(|h| h.0).collect()
}

// ---------- linear pattern: distance types ----------

#[test]
fn spacing_stays_the_default() {
    let r = run(&plate(&lp(r#","count":5,"spacing":15"#)));
    assert!(r.errors().is_empty() && r.warnings().is_empty(), "{:?} {:?}", r.errors(), r.warnings());
    assert_eq!(xs(&r), vec![10.0, 25.0, 40.0, 55.0, 70.0]);
    assert!((r.volume() - vol(5)).abs() < 1e-6);
}

#[test]
fn extent_spreads_the_copies_first_to_last() {
    let r = run(&plate(&lp(r#","count":5,"distanceType":"extent","extent":60"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(xs(&r), vec![10.0, 25.0, 40.0, 55.0, 70.0]);
    assert!((r.volume() - vol(5)).abs() < 1e-6);
    // a new count keeps the extent: 3 copies over the same 60 mm
    let r3 = run(&plate(&lp(r#","count":3,"distanceType":"extent","extent":60,"spacing":7"#)));
    assert_eq!(xs(&r3), vec![10.0, 40.0, 70.0]);
    // reverse still works with an extent
    let rr = run(&plate_at(90.0, 10.0, &lp(r#","count":3,"distanceType":"extent","extent":60,"reverse":true"#)));
    assert_eq!(xs(&rr), vec![30.0, 60.0, 90.0]);
}

#[test]
fn extent_must_be_positive() {
    let r = run(&plate(&lp(r#","count":3,"distanceType":"extent","extent":-5"#)));
    let e = r.error("invalidValue").unwrap_or_else(|| panic!("{:?}", r.errors()));
    assert_eq!(e.params.iter().find(|p| p.0 == "field").map(|p| p.1.as_str()), Some(Some("extent")));
}

#[test]
fn to_object_reaches_a_plane_and_follows_it() {
    // seed at x = 10, plane at x = 80: 4 copies 70 / 3 apart, the last on the plane
    let r = run(&plate(&lp(r#","count":4,"distanceType":"toObject","to":{"kind":"feature","id":"P9"}"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let x = xs(&r);
    assert_eq!(x.len(), 4);
    assert!((x[3] - 80.0).abs() < 1e-6 && (x[1] - (10.0 + 70.0 / 3.0)).abs() < 1e-6, "{x:?}");
    // the plane moves: the copies follow
    let moved = plate(&lp(r#","count":4,"distanceType":"toObject","to":{"kind":"feature","id":"P9"}"#)).replace(r#""offset":80"#, r#""offset":40"#);
    assert_eq!(xs(&run(&moved)), vec![10.0, 20.0, 30.0, 40.0]);
    // reverse is ignored: the object gives the side
    let rev = run(&plate(&lp(r#","count":4,"distanceType":"toObject","to":{"kind":"feature","id":"P9"},"reverse":true"#)));
    assert_eq!(xs(&rev), x);
}

#[test]
fn to_object_on_a_face_with_an_offset() {
    // the plate's side face at x = 100, ending 10 short: 10, 30, 50, 70, 90
    let r = run(&plate(&lp(r#","count":5,"distanceType":"toObject","to":{"kind":"face","id":"E2.S.e2"},"toOffset":-10"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(xs(&r), vec![10.0, 30.0, 50.0, 70.0, 90.0]);
    assert!((r.volume() - vol(5)).abs() < 1e-6);
}

#[test]
fn to_object_reaches_a_point_on_either_side() {
    let r = run(&plate(&lp(r#","count":3,"distanceType":"toObject","to":[50,0,0]"#)));
    assert_eq!(xs(&r), vec![10.0, 30.0, 50.0]);
    // a point behind the seed sends the copies the other way
    let b = run(&plate_at(90.0, 10.0, &lp(r#","count":3,"distanceType":"toObject","to":{"kind":"point","at":[50,70,3]}"#)));
    assert_eq!(xs(&b), vec![50.0, 70.0, 90.0]);
}

#[test]
fn to_object_refusals() {
    // a plane along the direction is never reached
    let par = run(&plate(&lp(r#","count":3,"distanceType":"toObject","to":{"kind":"std","id":"front"}"#)));
    assert!(par.error("patternToObjectParallel").is_some(), "{:?}", par.errors());
    // an object through the seed's middle leaves no room
    let at = run(&plate(&lp(r#","count":3,"distanceType":"toObject","to":[10,50,0]"#)));
    assert!(at.error("patternToObjectAtSeed").is_some(), "{:?}", at.errors());
    // an offset that takes the end back past the seed
    let back = run(&plate(&lp(r#","count":3,"distanceType":"toObject","to":[50,0,0],"toOffset":-45"#)));
    assert!(back.error("patternToObjectAtSeed").is_some(), "{:?}", back.errors());
    // no object at all, or one that does not resolve
    let none = run(&plate(&lp(r#","count":3,"distanceType":"toObject""#)));
    let e = none.error("missingReference").unwrap_or_else(|| panic!("{:?}", none.errors()));
    assert_eq!(e.params.iter().find(|p| p.0 == "field").map(|p| p.1.as_str()), Some(Some("to")));
    let lost = run(&plate(&lp(r#","count":3,"distanceType":"toObject","to":{"kind":"face","id":"E2.S.e99"}"#)));
    assert!(lost.error("missingReference").is_some(), "{:?}", lost.errors());
}

#[test]
fn a_second_direction_makes_a_grid() {
    // 3 along x every 20, 2 along y over 30: holes at x 10, 30, 50 and y 10, 40
    let r = run(&plate(&lp(r#","count":3,"spacing":20,"dir2":{"kind":"axis","axis":"y"},"count2":2,"distanceType2":"extent","extent2":30"#)));
    assert!(r.errors().is_empty() && r.warnings().is_empty(), "{:?} {:?}", r.errors(), r.warnings());
    assert_eq!(r.holes(), vec![(10.0, 10.0), (10.0, 40.0), (30.0, 10.0), (30.0, 40.0), (50.0, 10.0), (50.0, 40.0)]);
    assert!((r.volume() - vol(6)).abs() < 1e-6);
    // direction 2 to an object, reversed direction 1 ignored for it
    let t = run(&plate_at(90.0, 10.0, &lp(r#","count":2,"spacing":20,"reverse":true,"dir2":{"kind":"axis","axis":"y"},"count2":3,"distanceType2":"toObject","to2":[0,90,0]"#)));
    assert!(t.errors().is_empty(), "{:?}", t.errors());
    assert_eq!(t.holes(), vec![(70.0, 10.0), (70.0, 50.0), (70.0, 90.0), (90.0, 10.0), (90.0, 50.0), (90.0, 90.0)]);
}

#[test]
fn a_second_direction_along_the_first_is_refused() {
    let r = run(&plate(&lp(r#","count":3,"spacing":20,"dir2":{"kind":"axis","axis":"x"},"count2":2,"spacing2":5"#)));
    let e = r.error("invalidValue").unwrap_or_else(|| panic!("{:?}", r.errors()));
    assert_eq!(e.params.iter().find(|p| p.0 == "field").map(|p| p.1.as_str()), Some(Some("dir2")));
}

// ---------- warnings: missed, apart, overlapping copies ----------

#[test]
fn copies_that_miss_the_body_are_named() {
    // 12 holes every 9.5 from x = 10: copies 10 and 11 (x = 105, 114.5) are off the plate
    let r = run(&plate(&lp(r#","count":12,"spacing":9.5"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - vol(10)).abs() < 1e-6);
    let w = r.warning("patternCopiesNoEffect").unwrap_or_else(|| panic!("{:?}", r.warnings()));
    assert_eq!(ck_json::to_string(&w.params.iter().find(|p| p.0 == "copies").unwrap().1), "[10,11]");
    assert_eq!(w.params.iter().find(|p| p.0 == "of").and_then(|p| p.1.as_f64()), Some(11.0));
    // every copy off the plate (reverse)
    let all = run(&plate(&lp(r#","count":3,"spacing":15,"reverse":true"#)));
    let w = all.warning("patternCopiesNoEffect").unwrap_or_else(|| panic!("{:?}", all.warnings()));
    assert_eq!(ck_json::to_string(&w.params.iter().find(|p| p.0 == "copies").unwrap().1), "[1,2]");
}

#[test]
fn copies_that_overlap_each_other_are_reported() {
    let r = run(&plate(&lp(r#","count":3,"spacing":3"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("patternCopiesOverlap").unwrap_or_else(|| panic!("{:?}", r.warnings()));
    // the hole is 4 wide along x: a spacing of 4 or more keeps the copies apart
    assert_eq!(w.params.iter().find(|p| p.0 == "minSpacing").and_then(|p| p.1.as_f64()), Some(4.0));
    // touching copies (spacing = width) do not overlap
    let touch = run(&plate(&lp(r#","count":3,"spacing":4"#)));
    assert!(touch.warning("patternCopiesOverlap").is_none(), "{:?}", touch.warnings());
}

#[test]
fn boss_copies_off_the_body_are_apart() {
    // a 10 x 10 x 5 boss on top at x 0..10, y 40..50, copied every 60 along x: the copy at x 60..70 is on the plate, the one at
    // 120..130 stands alone
    let tail = r#",
  {"type":"sketch","id":"S7","plane":{"kind":"feature","id":"P3"},"points":[{"id":"a","x":0,"y":40},{"id":"b","x":10,"y":40},{"id":"c","x":10,"y":50},{"id":"d","x":0,"y":50}],
   "entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]},
  {"type":"extrude","id":"E8","op":"add","sketch":"S7","end":"blind","depth":5},
  {"type":"lpattern","id":"L10","seeds":["E8"],"dir":{"kind":"axis","axis":"x"},"count":3,"spacing":60}"#;
    let r = run(&plate(tail));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert!((r.volume() - (vol(1) + 3.0 * 500.0)).abs() < 1e-6, "{}", r.volume());
    let w = r.warning("patternCopiesApart").unwrap_or_else(|| panic!("{:?}", r.warnings()));
    assert_eq!(ck_json::to_string(&w.params.iter().find(|p| p.0 == "copies").unwrap().1), "[2]");
    assert!(r.warning("patternCopiesNoEffect").is_none(), "{:?}", r.warnings());
}

#[test]
fn a_circular_pattern_reports_missed_copies_too() {
    // the hole at (10, 10) turned about z: 90 and 180 degrees land off the plate (x or y negative)
    let r = run(&plate(r#",{"type":"cpattern","id":"C6","seeds":["H5"],"axis":{"kind":"axis","axis":"z"},"count":4,"angle":360}"#));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    let w = r.warning("patternCopiesNoEffect").unwrap_or_else(|| panic!("{:?}", r.warnings()));
    assert_eq!(ck_json::to_string(&w.params.iter().find(|p| p.0 == "copies").unwrap().1), "[1,2,3]");
}

// ---------- path pattern ----------

/// A sketch `S8` on the front plane holding `ents` (lines / arcs over the points `pts`), then a curvepattern of H5 along it.
fn path(pts: &str, ents: &str, extra: &str) -> String {
    format!(
        r#",
  {{"type":"sketch","id":"S8","plane":{{"kind":"std","id":"front"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},
  {{"type":"curvepattern","id":"C9","seeds":["H5"],"path":{{"kind":"sketch","id":"S8"}}{extra}}}"#
    )
}
fn line_path(extra: &str) -> String {
    path(r#"{"id":"a","x":10,"y":10},{"id":"b","x":90,"y":10}"#, r#"{"type":"line","id":"e1","p1":"a","p2":"b"}"#, extra)
}

#[test]
fn path_pattern_along_a_line() {
    let r = run(&plate(&line_path(r#","count":5,"spacing":20"#)));
    assert!(r.errors().is_empty() && r.warnings().is_empty(), "{:?} {:?}", r.errors(), r.warnings());
    assert_eq!(xs(&r), vec![10.0, 30.0, 50.0, 70.0, 90.0]);
    assert!((r.volume() - vol(5)).abs() < 1e-6);
    // extent: 3 copies over the 80 mm
    let e = run(&plate(&line_path(r#","count":3,"distanceType":"extent","extent":80"#)));
    assert_eq!(xs(&e), vec![10.0, 50.0, 90.0]);
}

#[test]
fn path_pattern_starts_at_the_end_nearest_the_seed() {
    // the line is drawn from x = 90 to x = 10; the seed sits at x = 10, so the pattern runs from there
    let p = path(r#"{"id":"a","x":90,"y":10},{"id":"b","x":10,"y":10}"#, r#"{"type":"line","id":"e1","p1":"a","p2":"b"}"#, r#","count":3,"spacing":20"#);
    assert_eq!(xs(&run(&plate(&p))), vec![10.0, 30.0, 50.0]);
}

#[test]
fn path_pattern_start_point_and_flip() {
    // seed at x = 40 on a path from x = 10: it sits 30 along the path, copies every 20 on from there
    let r = run(&plate_at(40.0, 10.0, &line_path(r#","count":3,"spacing":20,"start":30"#)));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(xs(&r), vec![40.0, 60.0, 80.0]);
    // the same as a fraction of the 80 mm path
    let f = run(&plate_at(40.0, 10.0, &line_path(r#","count":3,"spacing":20,"startFraction":0.375"#)));
    assert_eq!(xs(&f), vec![40.0, 60.0, 80.0]);
    // flipped: the path runs from x = 90, the seed sits 50 along it, copies go towards x = 10
    let fl = run(&plate_at(40.0, 10.0, &line_path(r#","count":3,"spacing":15,"start":50,"flip":true"#)));
    assert!(fl.errors().is_empty(), "{:?}", fl.errors());
    assert_eq!(xs(&fl), vec![10.0, 25.0, 40.0]);
}

#[test]
fn path_pattern_refuses_copies_past_the_end() {
    // start 20 + 4 x 20 = 100 > 80: the largest spacing that fits is (80 - 20) / 4 = 15
    let r = run(&plate(&line_path(r#","count":5,"spacing":20,"start":20"#)));
    let e = r.error("patternBeyondPath").unwrap_or_else(|| panic!("{:?}", r.errors()));
    let num = |k: &str| e.params.iter().find(|p| p.0 == k).and_then(|p| p.1.as_f64());
    assert_eq!(num("maxSpacing"), Some(15.0));
    assert_eq!(num("length"), Some(80.0));
    // a start off the path
    let s = run(&plate(&line_path(r#","count":2,"spacing":5,"start":90"#)));
    assert!(s.error("patternStartOutside").is_some(), "{:?}", s.errors());
    let sf = run(&plate(&line_path(r#","count":2,"spacing":5,"startFraction":1.5"#)));
    assert!(sf.error("patternStartOutside").is_some(), "{:?}", sf.errors());
}

#[test]
fn path_pattern_needs_a_path() {
    let r = run(&plate(r#",{"type":"curvepattern","id":"C9","seeds":["H5"],"count":3,"spacing":10}"#));
    assert!(r.error("missingPath").is_some(), "{:?}", r.errors());
    let r = run(&plate(r#",{"type":"curvepattern","id":"C9","seeds":["H5"],"path":{"kind":"sketch","id":"S77"},"count":3,"spacing":10}"#));
    assert!(r.error("missingPath").is_some(), "{:?}", r.errors());
}

/// Quarter-circle steps on an arc of radius 40 about (50, 50), from (50, 10) anticlockwise to (50, 90); the seed hole is at
/// (50, 20), 10 inside the arc.
fn arc_path(orientation: &str) -> String {
    path(
        r#"{"id":"c","x":50,"y":50},{"id":"a","x":50,"y":10},{"id":"b","x":50,"y":90}"#,
        r#"{"type":"arc","id":"e1","c":"c","p1":"a","p2":"b"}"#,
        &format!(r#","count":3,"distanceType":"extent","extent":{},"orientation":"{orientation}""#, PI * 40.0),
    )
}

#[test]
fn path_pattern_orientation_on_an_arc() {
    // fixed: the copies are moved, not turned: the arc points (90, 50) and (50, 90) plus the seed's offset (0, 10)
    let f = run(&plate_at(50.0, 20.0, &arc_path("fixed")));
    assert!(f.errors().is_empty(), "{:?}", f.errors());
    assert_eq!(f.holes(), vec![(50.0, 20.0), (50.0, 100.0), (90.0, 60.0)]);
    // tangent: the copies keep their relation to the path, turned about the arc's centre: radius 30 at 0 and 90 degrees
    let t = run(&plate_at(50.0, 20.0, &arc_path("tangent")));
    assert!(t.errors().is_empty(), "{:?}", t.errors());
    assert_eq!(t.holes(), vec![(50.0, 20.0), (50.0, 80.0), (80.0, 50.0)]);
    assert!((t.volume() - vol(3)).abs() < 1e-6);
}

#[test]
fn path_pattern_tangent_through_a_corner_of_line_and_arc() {
    // a line from (10, 10) to (50, 10), then a quarter arc about (50, 50) to (90, 50): the seed 10 above the start, at
    // (10, 20); after 40 + 20 pi along the path it is at the arc's end, turned 90 degrees: (80, 50)
    let p = path(
        r#"{"id":"a","x":10,"y":10},{"id":"b","x":50,"y":10},{"id":"c","x":50,"y":50},{"id":"d","x":90,"y":50}"#,
        r#"{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"arc","id":"e2","c":"c","p1":"b","p2":"d"}"#,
        &format!(r#","count":2,"distanceType":"extent","extent":{},"orientation":"tangent""#, 40.0 + 20.0 * PI),
    );
    let r = run(&plate_at(10.0, 20.0, &p));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.holes(), vec![(10.0, 20.0), (80.0, 50.0)]);
}

#[test]
fn path_pattern_reads_the_original_format_names() {
    // `reverse` = flip, `align` = tangent, `equal` = spread over the rest of the path, a bare string path
    let names = path(
        r#"{"id":"c","x":50,"y":50},{"id":"a","x":50,"y":10},{"id":"b","x":50,"y":90}"#,
        r#"{"type":"arc","id":"e1","c":"c","p1":"a","p2":"b"}"#,
        r#","count":3,"equal":true,"spacing":10,"align":true"#,
    )
    .replace(r#""path":{"kind":"sketch","id":"S8"}"#, r#""path":"S8""#);
    let r = run(&plate_at(50.0, 20.0, &names));
    assert!(r.errors().is_empty(), "{:?}", r.errors());
    assert_eq!(r.holes(), vec![(50.0, 20.0), (50.0, 80.0), (80.0, 50.0)]);
    let rev = run(&plate_at(40.0, 10.0, &line_path(r#","count":3,"spacing":15,"start":50,"reverse":true"#)));
    assert_eq!(xs(&rev), vec![10.0, 25.0, 40.0]);
}
