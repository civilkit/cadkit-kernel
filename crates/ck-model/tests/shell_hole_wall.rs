//! Shell opened at the wall of a through hole (kernel issue 68, first slice). A 40 x 40 x 20 block with round through holes,
//! shelled inward `t` and open at one hole wall: the cavity is the block moved in by `t` (36 x 36 x 16 at t = 2) less the
//! cylinders of the holes that stay closed (radius + t), and it swallows the opened hole's cylinder. Volume =
//! block - holes - (cavity block - kept holes' (r + t) cylinders - opened hole's r cylinder), each cylinder over the cavity height.
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

/// (errors, volume, valid) of the part built from `src`.
fn build(src: &str) -> (Vec<String>, f64, bool) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.brep.body.as_ref().unwrap();
    (errors, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).is_empty())
}

/// The block with holes `(x, y, diameter)` (feature H1, H2, ...), then the shell of thickness `t` open at the wall of hole `open` (1-based).
fn part(holes: &[(f64, f64, f64)], t: f64, open: usize) -> String {
    let mut f = vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"a","x":0,"y":0},{"id":"b","x":40,"y":0},{"id":"c","x":40,"y":40},{"id":"d","x":0,"y":40}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#.to_string(),
    ];
    for (i, (x, y, d)) in holes.iter().enumerate() {
        let n = i + 1;
        f.push(format!(
            r#"{{"type":"sketch","id":"K{n}","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"a","x":{x},"y":{y}}}],"entities":[{{"type":"point","id":"e1","p":"a"}}],"constraints":[]}}"#
        ));
        f.push(format!(r#"{{"type":"hole","id":"H{n}","sketch":"K{n}","kind":"simple","diameter":{d},"end":"through"}}"#));
    }
    f.push(format!(r#"{{"type":"shell","id":"S","faces":["H{open}.S.h1"],"thickness":{t}}}"#));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

/// Closed form for the block with holes `(x, y, d)` shelled `t` and open at hole `open` (1-based).
fn expect(holes: &[(f64, f64, f64)], t: f64, open: usize) -> f64 {
    let h = 20.0 - 2.0 * t;
    let cyl = |r: f64, len: f64| PI * r * r * len;
    let mut v = 40.0 * 40.0 * 20.0;
    let mut cav = (40.0 - 2.0 * t) * (40.0 - 2.0 * t) * h;
    for (i, (_, _, d)) in holes.iter().enumerate() {
        v -= cyl(d / 2.0, 20.0);
        cav -= if i + 1 == open { cyl(d / 2.0, h) } else { cyl(d / 2.0 + t, h) };
    }
    v - cav
}

fn passes(holes: &[(f64, f64, f64)], t: f64, open: usize) {
    let (errors, v, valid) = build(&part(holes, t, open));
    assert!(errors.is_empty(), "{errors:?}");
    assert!(valid, "invalid body");
    let want = expect(holes, t, open);
    assert!((v - want).abs() < 0.05, "volume {v}, expected {want}");
}

/// The plan's case: 8 mm hole, wall 2: 11062.94.
#[test]
fn hole_wall_opened_matches_the_closed_form() {
    assert!((expect(&[(20.0, 20.0, 8.0)], 2.0, 1) - 11062.94).abs() < 0.01);
    passes(&[(20.0, 20.0, 8.0)], 2.0, 1);
}

#[test]
fn other_wall_thicknesses() {
    passes(&[(20.0, 20.0, 8.0)], 1.0, 1);
    passes(&[(20.0, 20.0, 8.0)], 3.0, 1);
}

#[test]
fn hole_off_centre() {
    passes(&[(14.0, 23.0, 8.0)], 2.0, 1);
}

/// Two holes, only the second opened: the first keeps a wall of 2 around it.
#[test]
fn two_holes_one_opened() {
    let holes = [(12.0, 20.0, 6.0), (28.0, 20.0, 8.0)];
    passes(&holes, 2.0, 2);
    passes(&holes, 2.0, 1);
}

/// A wall as thick as the part allows is refused by name (the block is 20 thick: 2 t >= 20), never an invalid body.
#[test]
fn too_thick_is_refused_by_name() {
    let (errors, _, _) = build(&part(&[(20.0, 20.0, 8.0)], 12.0, 1));
    assert_eq!(errors.len(), 1, "{errors:?}");
    assert!(errors[0].starts_with("S:") && (errors[0].contains("shellTooThick") || errors[0].contains("shellFailed") || errors[0].contains("shellUnsupported") || errors[0].contains("offsetTooLarge")), "{errors:?}");
}

/// A hole whose neighbour wall is thinner than the shell wall (kept hole near the opened one): refused or valid, never invalid.
#[test]
fn walls_that_collide_are_refused_not_invalid() {
    let holes = [(14.0, 20.0, 8.0), (25.0, 20.0, 8.0)];
    let (errors, _, valid) = build(&part(&holes, 2.0, 1));
    assert!(errors.is_empty() || errors[0].starts_with("S:"), "{errors:?}");
    assert!(valid || !errors.is_empty());
}
