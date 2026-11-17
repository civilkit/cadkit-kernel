//! `moveFace` on one piece of a split face (Split face halves a face; one half cannot follow a move of the other).

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
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

struct Run {
    result: RegenResult,
    body: Option<ck_topo::body::Body>,
    volume: f64,
}

impl Run {
    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
    }
    fn bbox(&self) -> ([f64; 3], [f64; 3]) {
        let (a, b) = self.body.as_ref().unwrap().bbox();
        ([a.x, a.y, a.z], [b.x, b.y, b.z])
    }
    fn valid(&self) -> bool {
        ck_check::validate(self.body.as_ref().unwrap(), 1e-6).is_empty()
    }
}

fn run(features: &str) -> Run {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    Run { result, body: ex.brep.body.clone(), volume }
}

fn with(base: &str, more: &str) -> Run {
    run(&format!("{base},{more}"))
}

// ---- one half of a split face ----

/// A 70 mm disc, 10 thick (sketch F1 on the top plane, extrude F2), its top cap split in two by the plane x = 0 (splitLine F3).
const SPLIT_DISC: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":35}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{"kind":"std","id":"right"},"offset":0}"#;

fn dvol(r: &Run, base: f64) -> f64 {
    r.volume - base
}

fn names(r: &Run) -> Vec<String> {
    let b = r.body.as_ref().unwrap();
    let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    v.sort();
    v
}

/// A 40 x 20 box, 10 thick, its top split by the plane x = `x`.
fn split_box(x: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"a","x":-20,"y":-10}},{{"id":"b","x":20,"y":-10}},{{"id":"c","x":20,"y":10}},{{"id":"d","x":-20,"y":10}}],
  "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},
 {{"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{{"kind":"std","id":"right"}},"offset":{x}}}"#
    )
}

/// A spline-bounded profile (spline + closing line), 10 thick, top split by x = 0.
const SPLIT_SPLINE: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p0","x":-30,"y":0},{"id":"p1","x":-15,"y":-18},{"id":"p2","x":0,"y":-22},{"id":"p3","x":15,"y":-18},{"id":"p4","x":30,"y":0}],
  "entities":[{"id":"e1","type":"spline","pts":["p0","p1","p2","p3","p4"]},{"id":"e2","type":"line","p1":"p4","p2":"p0"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
 {"type":"splitLine","id":"F3","faces":["F2.C1"],"plane":{"kind":"std","id":"right"},"offset":0}"#;

fn mv(piece: &str, d: f64) -> String {
    format!(r#"{{"type":"moveFace","id":"F4","faces":["{piece}"],"mode":"offset","distance":{d}}}"#)
}

/// Moving one half by `d` makes a step: a wall along the split line, the other half and the rest stay, the moved half keeps its
/// name. Returns the run.
fn step(base: &str, piece: &str, d: f64, top: f64) -> Run {
    step_n(base, piece, d, top, 1)
}

/// [`step`] where the move adds `new` faces (the wall, and for a free-form rim the extension of the old wall as a face of its own).
fn step_n(base: &str, piece: &str, d: f64, top: f64, new: usize) -> Run {
    let b = run(base);
    assert!(b.errors().is_empty(), "{:?}", b.errors());
    let m = with(base, &mv(piece, d));
    assert!(m.errors().is_empty(), "{piece} {d}: {:?}", m.errors());
    assert!(m.valid(), "{piece} {d}: invalid result");
    let n = names(&m);
    assert!(n.contains(&piece.to_string()) && n.iter().any(|x| x.starts_with("F4.S.")), "{n:?}");
    assert_eq!(n.len(), names(&b).len() + new, "{new} new face(s): {n:?}");
    assert!((m.bbox().1[1] - top).abs() < 1e-6, "{:?}", m.bbox());
    // the moved face is at the new height
    let body = m.body.as_ref().unwrap();
    let f = body.find_face(piece).unwrap();
    let c = body.face_edges(f).iter().map(|&e| body.edge_mid(e).y).sum::<f64>() / body.face_edges(f).len() as f64;
    assert!((c - (10.0 + d)).abs() < 1e-6, "moved face at y={c}");
    m
}

#[test]
fn a_half_of_a_split_disc_rises_and_sinks_with_a_wall_on_the_split_line() {
    let v0 = run(SPLIT_DISC).volume;
    for piece in ["F2.C1~1", "F2.C1~2"] {
        let up = step(SPLIT_DISC, piece, 3.0, 13.0);
        let half = std::f64::consts::PI * 35.0 * 35.0 / 2.0;
        assert!((dvol(&up, v0) - 3.0 * half).abs() < 1e-3, "{}", dvol(&up, v0));
        let down = step(SPLIT_DISC, piece, -3.0, 10.0);
        assert!((dvol(&down, v0) + 3.0 * half).abs() < 1e-3, "{}", dvol(&down, v0));
        // the round wall stays one face
        assert!(names(&up).iter().filter(|n| n.starts_with("F2.S")).count() == 1, "{:?}", names(&up));
    }
}

#[test]
fn translate_mode_moves_a_half_by_the_normal_component() {
    let m = with(SPLIT_DISC, r#"{"type":"moveFace","id":"F4","faces":["F2.C1~2"],"mode":"translate","translate":[7,3,0]}"#);
    assert!(m.errors().is_empty(), "{:?}", m.errors());
    assert!((m.bbox().1[1] - 13.0).abs() < 1e-6 && m.valid());
}

#[test]
fn a_half_of_a_split_box_steps_off_centre_and_on_centre() {
    for x in [0.0, 12.0, -15.0] {
        let base = split_box(x);
        for piece in ["F2.C1~1", "F2.C1~2"] {
            step(&base, piece, 4.0, 14.0);
            step(&base, piece, -4.0, 10.0);
        }
    }
}

#[test]
fn a_half_of_a_spline_bounded_face_steps() {
    for piece in ["F2.C1~1", "F2.C1~2"] {
        // rising, the spline wall carries on in a face of its own (`F4.S.F2_S_e1`) next to the straight wall on the split line
        let up = step_n(SPLIT_SPLINE, piece, 3.0, 13.0, 2);
        assert!(names(&up).contains(&"F4.S.F2_S_e1".to_string()), "{:?}", names(&up));
        step(SPLIT_SPLINE, piece, -3.0, 10.0);
    }
}

/// The cut line reaches the boundary at a corner (a sketch line from corner to corner) and near the rim (a thin piece).
#[test]
fn a_split_that_runs_to_the_boundary_steps() {
    let diag = r#"{"type":"sketch","id":"S9","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":-20,"y":-10},{"id":"b","x":20,"y":10}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"}],"constraints":[]}"#;
    let base = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"a","x":-20,"y":-10}},{{"id":"b","x":20,"y":-10}},{{"id":"c","x":20,"y":10}},{{"id":"d","x":-20,"y":10}}],
  "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}},{diag},
 {{"type":"splitLine","id":"F3","faces":["F2.C1"],"sketch":"S9","direction":"normal"}}"#
    );
    for piece in ["F2.C1~1", "F2.C1~2"] {
        step(&base, piece, 4.0, 14.0);
        step(&base, piece, -4.0, 10.0);
    }
    // a thin sliver of the disc (plane near the rim)
    let sliver = SPLIT_DISC.replace(r#""offset":0"#, r#""offset":33"#);
    for piece in ["F2.C1~1", "F2.C1~2"] {
        step(&sliver, piece, 3.0, 13.0);
        step(&sliver, piece, -3.0, 10.0);
    }
}

#[test]
fn a_move_deeper_than_the_body_is_refused_with_a_reason_and_the_largest_distance() {
    let v0 = run(SPLIT_DISC).volume;
    for d in [-10.0, -12.0, -40.0] {
        let m = with(SPLIT_DISC, &mv("F2.C1~2", d));
        let e = m.errors();
        assert_eq!(e.len(), 1, "{d}: {e:?}");
        assert!(e[0].starts_with("F4:moveFaceFailed") && (e[0].contains("far side") || e[0].contains("not that thick")), "{}", e[0]);
        assert!(e[0].contains("maxSize"), "{}", e[0]);
        assert!(dvol(&m, v0).abs() < 1e-6 && m.valid(), "the body is untouched");
    }
}

/// Several selected faces with one flush piece are refused, with the way out; the base name and both pieces still move as one.
#[test]
fn naming_one_piece_with_other_faces_is_refused_and_the_base_name_moves_all() {
    let m = with(SPLIT_DISC, r#"{"type":"moveFace","id":"F4","faces":["F2.C1~1","F2.C0"],"mode":"offset","distance":3}"#);
    assert!(m.errors().len() == 1 && m.errors()[0].starts_with("F4:moveFaceUnsupported") && m.errors()[0].contains("one piece at a time"), "{:?}", m.errors());
    let all = with(SPLIT_DISC, r#"{"type":"moveFace","id":"F4","faces":["F2.C1"],"mode":"offset","distance":3}"#);
    assert!(all.errors().is_empty() && (all.bbox().1[1] - 13.0).abs() < 1e-6 && all.valid());
    let both = with(SPLIT_DISC, r#"{"type":"moveFace","id":"F4","faces":["F2.C1~1","F2.C1~2"],"mode":"offset","distance":3}"#);
    assert!(both.errors().is_empty() && (both.bbox().1[1] - 13.0).abs() < 1e-6);
}

/// Names survive: later features reference the moved half, the other half and the new wall.
#[test]
fn later_features_find_the_moved_half_and_the_new_wall() {
    let base = format!("{SPLIT_DISC},{}", mv("F2.C1~2", 3.0));
    let wall = "F4.S.F2_C1p1";
    // move the same half again: now an ordinary move, the wall follows
    let again = with(&base, r#"{"type":"moveFace","id":"F5","faces":["F2.C1~2"],"mode":"offset","distance":2}"#);
    assert!(again.errors().is_empty(), "{:?}", again.errors());
    assert!((again.bbox().1[1] - 15.0).abs() < 1e-6 && again.valid());
    // the other half up by less than the step: an ordinary move, valid; up to the same level (the wall would vanish) is refused with the largest distance
    let other = with(&base, r#"{"type":"moveFace","id":"F5","faces":["F2.C1~1"],"mode":"offset","distance":1}"#);
    assert!(other.errors().is_empty() && other.valid(), "{:?}", other.errors());
    let level = with(&base, r#"{"type":"moveFace","id":"F5","faces":["F2.C1~1"],"mode":"offset","distance":3}"#);
    assert!(level.errors().len() == 1 && level.errors()[0].starts_with("F5:moveFaceFailed") && level.errors()[0].contains("maxSize"), "{:?}", level.errors());
    assert!(level.valid() && (level.bbox().1[1] - 13.0).abs() < 1e-6);
    // a fillet on the edge between the moved half and the new wall
    let fil = with(&base, &format!(r#"{{"type":"fillet","id":"F5","radius":1,"edges":[{{"faces":["{wall}","F2.C1~2"],"at":[0,13,0]}}]}}"#));
    assert!(fil.errors().is_empty(), "{:?}", fil.errors());
    assert!(fil.valid());
    // sliding the wall itself
    let slide = with(&base, &format!(r#"{{"type":"moveFace","id":"F5","faces":["{wall}"],"mode":"offset","distance":2}}"#));
    assert!(slide.errors().is_empty(), "{:?}", slide.errors());
    assert!(slide.valid());
}
