//! Thin-wall shell of revolved bottle outlines (spline shoulder, torus base corner, push-up cone,
//! drafted grooves): the shapes of a PET bottle hollowed in one `shell` call.
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

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
}

pub fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

pub fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}
pub fn errs(o: &Out) -> Vec<String> {
    o.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
}
pub fn issues(o: &Out) -> Vec<String> {
    issues_at(o, 1e-6)
}
pub fn issues_at(o: &Out, tol: f64) -> Vec<String> {
    ck_check::validate(o.brep.body.as_ref().unwrap(), tol).iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect()
}

fn axis(top: bool) -> String {
    let (pl, y) = if top { ("top", -205) } else { ("front", 205) };
    format!(
        r#"{{"type":"sketch","id":"F0","plane":{{"kind":"std","id":"{pl}"}},
    "points":[{{"id":"o","x":0,"y":0}},{{"id":"p","x":0,"y":{y}}}],
    "entities":[{{"type":"line","id":"ax","p1":"o","p2":"p","construction":true}}],"constraints":[]}}"#
    )
}

/// Outline of the bottle as (r, z): which pieces are in is chosen by the flags.
pub struct Bottle {
    pub spline: bool,
    pub torus: bool,
    pub pushup: bool,
    pub height: f64,
    /// drawn on the `top` plane with y = -z, as the kit scripts do (mirrored orientation)
    pub top: bool,
    /// radial width of the flat standing ring before the corner arc (0 = the push-up cone runs into the torus)
    pub ring: f64,
    /// height of the push-up apex
    pub dome: f64,
    /// a short straight-walled cup (height `height`) instead of the bottle shoulder and neck
    pub cup: bool,
    /// radius of the flat centre of the push-up
    pub flat: f64,
    /// corner arc as the three-point arc of a script (circumcentre of the three points), not the exact tangent one
    pub arc3: bool,
}

impl Bottle {
    pub fn pet() -> Bottle {
        Bottle { spline: true, torus: true, pushup: true, height: 205.0, arc3: false, top: false, ring: 9.0, dome: 9.0, cup: false, flat: 4.0 }
    }
    /// sketch F1 + revolve F2
    pub fn features(&self) -> String {
        let h = self.height;
        let mut pts: Vec<(String, f64, f64)> = vec![];
        let mut ents: Vec<String> = vec![];
        macro_rules! add {
            ($id:expr, $x:expr, $y:expr) => {
                pts.push(($id.to_string(), $x, $y))
            };
        }
        let mut line = |ents: &mut Vec<String>, id: &str, a: &str, b: &str| ents.push(format!(r#"{{"type":"line","id":"{id}","p1":"{a}","p2":"{b}"}}"#));
        add!("top0", 0.0, h);
        add!("c0", 0.0, if self.pushup { self.dome } else { 0.0 });
        line(&mut ents, "e1", "top0", "c0");
        if self.pushup && self.ring > 0.0 {
            add!("r0", 24.0 - self.ring, 0.0);
            if self.flat > 0.0 {
                add!("f1", self.flat, self.dome);
                line(&mut ents, "e1b", "c0", "f1");
                line(&mut ents, "e2", "f1", "r0");
            } else {
                line(&mut ents, "e2", "c0", "r0");
            }
            add!("r1", 24.0, 0.0);
            line(&mut ents, "e3", "r0", "r1");
        } else if self.pushup {
            add!("r1", 24.0, 0.0);
            line(&mut ents, "e2", "c0", "r1");
        } else {
            add!("r1", 24.0, 0.0);
            line(&mut ents, "e3", "c0", "r1");
        }
        if self.torus {
            if self.arc3 { add!("cc", 23.986451612903203, 9.013548387096769) } else { add!("cc", 24.0, 9.0) }
            add!("b1", 33.0, 9.0);
            ents.push(r#"{"type":"arc","id":"e4","c":"cc","p1":"r1","p2":"b1"}"#.into());
        } else {
            add!("b1", 33.0, 0.0);
            line(&mut ents, "e4", "r1", "b1");
        }
        if self.cup {
            add!("s0", 33.0, h);
            line(&mut ents, "e5", "b1", "s0");
            line(&mut ents, "e8", "s0", "top0");
            return self.finish(pts, ents);
        }
        add!("s0", 33.0, 135.0);
        line(&mut ents, "e5", "b1", "s0");
        add!("n0", 14.6, 184.0);
        if self.spline {
            add!("s1", 32.2, 150.0);
            add!("s2", 27.0, 165.0);
            add!("s3", 19.0, 177.0);
            ents.push(r#"{"type":"spline","id":"e6","pts":["s0","s1","s2","s3","n0"]}"#.into());
        } else {
            line(&mut ents, "e6", "s0", "n0");
        }
        add!("n1", 14.6, h);
        line(&mut ents, "e7", "n0", "n1");
        line(&mut ents, "e8", "n1", "top0");
        self.finish(pts, ents)
    }
    fn finish(&self, pts: Vec<(String, f64, f64)>, mut ents: Vec<String>) -> String {
        let sg = if self.top { -1.0 } else { 1.0 };
        let p: Vec<String> = pts.iter().map(|(i, x, y)| format!(r#"{{"id":"{i}","x":{x},"y":{}}}"#, sg * y)).collect();
        if self.top {
            // mirrored: the arc runs the other way round
            for e in ents.iter_mut() {
                if e.contains(r#""id":"e4""#) && e.contains("arc") {
                    *e = e.replace(r#""p1":"r1","p2":"b1""#, r#""p1":"b1","p2":"r1""#);
                }
            }
        }
        let plane = if self.top { "top" } else { "front" };
        let ax = axis(self.top);
        format!(
            r#"{ax},{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"{plane}"}},"points":[{}],"entities":[{}],"constraints":[]}},
 {{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":{{"sketch":"F0","id":"ax"}},"angle":360}}"#,
            p.join(","),
            ents.join(",")
        )
    }
}

pub fn shell_feat(face: &str, t: f64) -> String {
    format!(r#",{{"type":"shell","id":"F3","faces":["{face}"],"thickness":{t}}}"#)
}

fn circumcentre(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> (f64, f64) {
    let d = 2.0 * (a.0 * (b.1 - c.1) + b.0 * (c.1 - a.1) + c.0 * (a.1 - b.1));
    let (a2, b2, c2) = (a.0 * a.0 + a.1 * a.1, b.0 * b.0 + b.1 * b.1, c.0 * c.0 + c.1 * c.1);
    ((a2 * (b.1 - c.1) + b2 * (c.1 - a.1) + c2 * (a.1 - b.1)) / d, (a2 * (c.0 - b.0) + b2 * (a.0 - c.0) + c2 * (b.0 - a.0)) / d)
}

/// Five lens-shaped petal outlines (two circular arcs each, as the kit's `polar` of two `arc3`s) on a plane
/// `offset` mm from `front`, cut `depth` upwards with `draft` degrees.
pub fn petals(plane_offset: f64, depth: f64, draft: f64, flip: bool, draft_out: bool) -> String {
    let (mut pts, mut ents) = (vec![], vec![]);
    for k in 0..5 {
        let r = |p: (f64, f64)| {
            let a = (72.0 * k as f64).to_radians();
            (p.0 * a.cos() - p.1 * a.sin(), p.0 * a.sin() + p.1 * a.cos())
        };
        let (pa, pb, m1, m2) = ((9.0, 0.0), (38.0, 0.0), (22.0, 7.5), (22.0, -7.5));
        let (c1, c2) = (circumcentre(pa, m1, pb), circumcentre(pb, m2, pa));
        for (n, p) in [("a", pa), ("b", pb), ("c1", c1), ("c2", c2)] {
            let q = r(p);
            pts.push(format!(r#"{{"id":"{n}{k}","x":{},"y":{}}}"#, q.0, q.1));
        }
        ents.push(format!(r#"{{"type":"arc","id":"u{k}","c":"c1{k}","p1":"b{k}","p2":"a{k}"}}"#));
        ents.push(format!(r#"{{"type":"arc","id":"l{k}","c":"c2{k}","p1":"a{k}","p2":"b{k}"}}"#));
    }
    let (plane_feat, plane_ref) = if plane_offset == 0.0 {
        (String::new(), r#"{"kind":"std","id":"front"}"#.to_string())
    } else {
        (format!(r#"{{"type":"plane","id":"PL","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{plane_offset}}},"#), r#"{"kind":"feature","id":"PL"}"#.to_string())
    };
    format!(
        r#"{plane_feat}
 {{"type":"sketch","id":"G1","plane":{plane_ref},"points":[{}],"entities":[{}],"constraints":[]}},
 {{"type":"extrude","id":"G2","op":"cut","sketch":"G1","end":"blind","depth":{depth},"draft":{draft},"draftOut":{draft_out},"flip":{flip}}}"#,
        pts.join(","),
        ents.join(",")
    )
}

pub fn cup() -> Bottle {
    Bottle { top: true, cup: true, spline: false, arc3: true, height: 45.0, ..Bottle::pet() }
}

/// Outer area of the solid built by `features`, less the open face of area `open`.
fn wall_area(features: &str, open: f64) -> f64 {
    let o = run(features);
    assert!(o.errors.is_empty(), "{:?}", errs(&o));
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).area - open
}

/// The shelled part is valid and its volume is the wall: area x thickness, a little less (the inward offset of
/// convex faces has less area than the outside).
fn assert_wall(what: &str, features: &str, shell: &str, area: f64, t: f64) {
    let o = run(&format!("{features}{shell}"));
    assert!(o.errors.is_empty(), "{what}: {:?}", errs(&o));
    assert!(issues(&o).is_empty(), "{what}: {:?}", issues(&o));
    let r = vol(&o) / (area * t);
    assert!((0.95..=1.0).contains(&r), "{what}: volume {} vs area x t {} (ratio {r:.4})", vol(&o), area * t);
}

const NECK_TOP: f64 = PI * 14.6 * 14.6;
const CUP_TOP: f64 = PI * 33.0 * 33.0;

/// The PET bottle of the brief: revolved closed outline z 0..205, r 33 body, spline shoulder to an r 14.6
/// neck, torus base corner r 9, push-up cone; the top face shelled away at 0.35 mm in one call.
#[test]
fn pet_bottle_shells_in_one_call() {
    let f = Bottle::pet().features();
    let a = wall_area(&f, NECK_TOP);
    for t in [0.35, 0.6, 1.0] {
        assert_wall(&format!("pet t={t}"), &f, &shell_feat("F2.S.e8", t), a, t);
    }
}

/// Same, drawn on the `top` plane with y = -z and the corner as a three-point arc, as the kit scripts do.
#[test]
fn pet_bottle_as_scripted() {
    let b = Bottle { top: true, arc3: true, ..Bottle::pet() };
    let f = b.features();
    assert_wall("pet scripted", &f, &shell_feat("F2.S.e8", 0.35), wall_area(&f, NECK_TOP), 0.35);
}

/// A revolved spline shoulder: the seam of the revolved face is the profile itself, and moves onto the offset
/// profile. Before, this was `offsetUnsupported "edge curve type"` with no face named.
#[test]
fn spline_shoulder_shells() {
    let b = Bottle { torus: false, pushup: false, ..Bottle::pet() };
    let f = b.features();
    let a = wall_area(&f, NECK_TOP);
    for t in [0.35, 1.0, 2.0] {
        assert_wall(&format!("spline t={t}"), &f, &shell_feat("F2.S.e8", t), a, t);
    }
}

/// The torus base corner: exact tangent arc and three-point arc, with and without a flat standing ring or a flat
/// push-up centre, both drawing orientations, with the cone shoulder.
#[test]
fn torus_corner_shells_valid() {
    for (name, b) in [
        ("tangent arc", Bottle { spline: false, ..Bottle::pet() }),
        ("3-point arc", Bottle { spline: false, arc3: true, ..Bottle::pet() }),
        ("3-point arc on top plane", Bottle { spline: false, arc3: true, top: true, ..Bottle::pet() }),
        ("no ring", Bottle { spline: false, arc3: true, ring: 0.0, ..Bottle::pet() }),
        ("flat base", Bottle { spline: false, arc3: true, pushup: false, ..Bottle::pet() }),
        ("high dome", Bottle { spline: false, arc3: true, dome: 12.0, ..Bottle::pet() }),
    ] {
        let f = b.features();
        let a = wall_area(&f, NECK_TOP);
        for t in [0.35, 0.6, 1.0] {
            assert_wall(&format!("{name} t={t}"), &f, &shell_feat("F2.S.e8", t), a, t);
        }
    }
}

/// A solid cup (r 33, torus corner r 9, push-up cone) with five lens-shaped grooves cut up from below with a
/// 10 degree draft, then shelled open at the top, at 0.35 to 1.5 mm, both draft directions and the sketch
/// plane 1 mm below the standing plane or on it.
#[test]
fn grooved_cup_shells_valid() {
    let c = cup().features();
    for (off, depth) in [(-1.0, 12.0), (0.0, 11.0)] {
        for draft_out in [false, true] {
            let f = format!("{c},{}", petals(off, depth, 10.0, true, draft_out));
            let a = wall_area(&f, CUP_TOP);
            for t in [0.35, 0.6, 1.0, 1.5] {
                assert_wall(&format!("grooves off {off} out {draft_out} t={t}"), &f, &shell_feat("F2.S.e8", t), a, t);
            }
        }
    }
}

/// A wall that does not fit the grooved cup is refused with a thickness that does.
#[test]
fn grooved_cup_too_thick_is_refused_with_a_working_wall() {
    let f = format!("{},{}", cup().features(), petals(-1.0, 12.0, 10.0, true, true));
    let o = run(&format!("{f}{}", shell_feat("F2.S.e8", 3.0)));
    assert_eq!(o.errors.len(), 1, "{:?}", errs(&o));
    let e = errs(&o).remove(0);
    assert!(e.starts_with("F3:shellTooThick"), "{e}");
    // the thickness the refusal names builds
    let t: f64 = e.split("\"maxThickness\":").nth(1).expect(&e).split(|c: char| !(c.is_ascii_digit() || c == '.')).next().unwrap().parse().unwrap();
    assert!(t > 0.5 && t < 3.0, "{e}");
    let ok = run(&format!("{f}{}", shell_feat("F2.S.e8", t)));
    assert!(ok.errors.is_empty() && issues(&ok).is_empty(), "t={t}: {:?} {:?}", errs(&ok), issues(&ok));
}

/// The groove sketch on the standing plane itself (its start face coplanar with the ring), with draft: the cut
/// is a plain boolean, not `booleanFailed`.
#[test]
fn coplanar_groove_cut_with_draft() {
    let c = cup().features();
    let base = vol(&run(&c));
    for draft in [0.0, 10.0] {
        for draft_out in [false, true] {
            let o = run(&format!("{c},{}", petals(0.0, 11.0, draft, true, draft_out)));
            assert!(o.errors.is_empty(), "draft {draft} out {draft_out}: {:?}", errs(&o));
            assert!(issues(&o).is_empty(), "{:?}", issues(&o));
            let cut = base - vol(&o);
            assert!(cut > 500.0 && cut < 20000.0, "cut volume {cut}");
        }
    }
}
