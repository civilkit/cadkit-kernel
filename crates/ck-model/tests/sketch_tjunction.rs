//! T-junctions in the region builder: an open outline whose end points lie on the inside of another
//! curve (six stadium petals whose base ends sit on a ring circle, no shared vertex) cuts that curve
//! there, so every petal, the ring minus the petals, and the rest of the sketch are separate regions
//! that extrude independently. Synthetic geometry only.

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

/// (errors, volume, sorted face ids)
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

const RING: f64 = 25.0;
const OUTER: f64 = 40.0;
const HOLE: f64 = 10.0;
const HALF: f64 = 5.0; // half petal width
const TIP: f64 = 18.0; // local x of the rounded end's centre
const DEPTH: f64 = 4.0;

#[derive(Clone, Copy, PartialEq)]
enum RingKind {
    /// one closed circle entity "ring"
    Circle,
    /// two arcs "ra", "rb" split between petals
    TwoArcs,
    /// arcs between the petal base points, which are shared vertices (the case that always worked)
    Shared,
}

/// Six petals, the ring, an outer circle "outer" and a centre hole "hole". `gap` moves each petal's
/// base ends along its axis (negative = short of the ring, positive = past it), in mm.
fn sketch(kind: RingKind, gap: f64) -> String {
    let mut pts = vec![r#"{"id":"o","x":0,"y":0}"#.to_string()];
    let mut ents = vec![
        format!(r#"{{"type":"circle","id":"outer","c":"o","r":{OUTER}}}"#),
        format!(r#"{{"type":"circle","id":"hole","c":"o","r":{HOLE}}}"#),
    ];
    let base = (RING * RING - HALF * HALF).sqrt() + gap;
    let rot = |a: f64, x: f64, y: f64| (x * a.cos() - y * a.sin(), x * a.sin() + y * a.cos());
    let mut ring_pts: Vec<(f64, String)> = vec![]; // (angle, point id) for the shared kind
    for k in 0..6 {
        let a = PI / 3.0 * k as f64;
        let mut add = |id: String, x: f64, y: f64| {
            let (px, py) = rot(a, x, y);
            pts.push(format!(r#"{{"id":"{id}","x":{px},"y":{py}}}"#));
        };
        add(format!("b{k}p"), base, HALF);
        add(format!("b{k}m"), base, -HALF);
        add(format!("t{k}p"), TIP, HALF);
        add(format!("t{k}m"), TIP, -HALF);
        add(format!("c{k}"), TIP, 0.0);
        ents.push(format!(r#"{{"type":"line","id":"s{k}a","p1":"b{k}p","p2":"t{k}p"}}"#));
        ents.push(format!(r#"{{"type":"arc","id":"s{k}e","c":"c{k}","p1":"t{k}p","p2":"t{k}m"}}"#));
        ents.push(format!(r#"{{"type":"line","id":"s{k}b","p1":"t{k}m","p2":"b{k}m"}}"#));
        let da = (HALF / RING).asin();
        ring_pts.push((a - da, format!("b{k}m")));
        ring_pts.push((a + da, format!("b{k}p")));
    }
    match kind {
        RingKind::Circle => ents.push(format!(r#"{{"type":"circle","id":"ring","c":"o","r":{RING}}}"#)),
        RingKind::TwoArcs => {
            for (i, (id, a)) in [("ra", PI / 6.0), ("rb", PI / 6.0 + PI)].into_iter().enumerate() {
                let (x0, y0) = (RING * a.cos(), RING * a.sin());
                let (x1, y1) = (-x0, -y0);
                pts.push(format!(r#"{{"id":"q{i}","x":{x0},"y":{y0}}}"#));
                pts.push(format!(r#"{{"id":"w{i}","x":{x1},"y":{y1}}}"#));
                ents.push(format!(r#"{{"type":"arc","id":"{id}","c":"o","p1":"q{i}","p2":"w{i}"}}"#));
            }
            // second arc runs w0 -> q0 (the same two points, other half)
            let last = ents.pop().unwrap().replace(r#""p1":"q1","p2":"w1""#, r#""p1":"w0","p2":"q0""#);
            ents.push(last);
        }
        RingKind::Shared => {
            ring_pts.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
            let n = ring_pts.len();
            for i in 0..n {
                ents.push(format!(r#"{{"type":"arc","id":"r{i}","c":"o","p1":"{}","p2":"{}"}}"#, ring_pts[i].1, ring_pts[(i + 1) % n].1));
            }
        }
    }
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}}"#,
        pts.join(","),
        ents.join(",")
    )
}

fn part(kind: RingKind, gap: f64, regions: Option<&str>) -> String {
    let r = regions.map(|r| format!(r#","regions":{r}"#)).unwrap_or_default();
    format!(
        r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{DEPTH}{r}}}]}}"#,
        sketch(kind, gap)
    )
}

fn vol(kind: RingKind, gap: f64, regions: Option<&str>) -> f64 {
    let (e, v, _) = run(&part(kind, gap, regions));
    assert!(e.is_empty(), "{e:?}");
    v
}
fn close(what: &str, a: f64, b: f64) {
    assert!((a - b).abs() < 2e-3 * b.max(1.0), "{what}: {a} vs {b}");
}

/// One petal: the stadium part inside the ring plus the circular segment the ring closes it with.
fn petal_area() -> f64 {
    let base = (RING * RING - HALF * HALF).sqrt();
    let theta = 2.0 * (HALF / RING).asin();
    10.0 * (base - TIP) + PI * HALF * HALF / 2.0 + RING * RING / 2.0 * (theta - theta.sin())
}
fn disc_minus_hole() -> f64 {
    PI * (RING * RING - HOLE * HOLE)
}

fn ids(v: &[&str]) -> String {
    format!("[{}]", v.iter().map(|i| format!("\"{i}\"")).collect::<Vec<_>>().join(","))
}

/// `ring_ids`: the entity ids that make up the ring (selecting them picks the ring minus the petals).
fn check(kind: RingKind, gap: f64, ring_ids: &[&str]) {
    let petal = petal_area();
    let annulus = disc_minus_hole() - 6.0 * petal;
    let band = PI * (OUTER * OUTER - RING * RING);
    let area = |regions: Option<&str>, _what: &str| vol(kind, gap, regions) / DEPTH;
    // a petal by one of its own entity ids: exactly that petal, whichever side or the rounded end
    close("petal by end arc", area(Some(&ids(&["s0e"])), ""), petal);
    close("petal by side", area(Some(&ids(&["s3a"])), ""), petal);
    close("petal by other side", area(Some(&ids(&["s5b"])), ""), petal);
    close("two petals", area(Some(&ids(&["s0e", "s2a"])), ""), 2.0 * petal);
    // the ring minus the petals, alone
    close("annulus", area(Some(&ids(ring_ids)), ""), annulus);
    // every shape: ring + all petals = the disc minus the centre hole
    let mut all: Vec<&str> = ring_ids.to_vec();
    let sides = ["s0a", "s1a", "s2a", "s3a", "s4a", "s5a"];
    all.extend(sides);
    close("all shapes", area(Some(&ids(&all)), ""), disc_minus_hole());
    // the band between the ring and the outer circle, the hole's own disc
    close("band", area(Some(&ids(&["outer"])), ""), band);
    close("hole disc", area(Some(&ids(&["hole"])), ""), PI * HOLE * HOLE);
    // no selection: the sketch as before, i.e. even-odd nesting (the band, then the hole's own disc; the ring's inside is void)
    close("whole sketch", area(None, ""), band + PI * HOLE * HOLE);
}

#[test]
fn ring_circle_petal_ends_on_it() {
    check(RingKind::Circle, 0.0, &["ring"]);
}

#[test]
fn ring_circle_petal_ends_just_short_or_past_the_ring() {
    // solver-grade noise: the ends miss the ring by far less than the sketch tolerance
    check(RingKind::Circle, -3e-5, &["ring"]);
    check(RingKind::Circle, 3e-5, &["ring"]);
}

#[test]
fn ring_as_two_arcs() {
    check(RingKind::TwoArcs, 0.0, &["ra", "rb"]);
    check(RingKind::TwoArcs, -3e-5, &["ra", "rb"]);
}

#[test]
fn petal_ends_shared_with_ring_arc_ends() {
    // the case that worked before: ring arcs end at the petal base points (shared vertices)
    let annulus_arcs: Vec<String> = (0..6).map(|k| format!("r{}", 2 * k + 1)).collect();
    let a: Vec<&str> = annulus_arcs.iter().map(String::as_str).collect();
    check(RingKind::Shared, 0.0, &a);
}

#[test]
fn single_petal_is_named_like_a_lone_region() {
    let (e, _, faces) = run(&part(RingKind::Circle, -3e-5, Some(r#"["s1b"]"#)));
    assert!(e.is_empty(), "{e:?}");
    assert!(faces.iter().all(|n| !n.contains('~')), "{faces:?}");
}

/// Outer-loop entity ids of every face of the sketch (regions, then the fills of the cut ring), sorted per face.
fn face_tags(kind: RingKind, gap: f64) -> (Vec<Vec<String>>, Vec<Vec<String>>) {
    let j = ck_json::parse(&sketch(kind, gap)).unwrap();
    let sk = ck_model::Sketch::from_json(&j);
    let p = ck_model::sketch::profile::profiles(&sk);
    let tags = |rs: &[ck_model::sketch::profile::Region]| -> Vec<Vec<String>> {
        rs.iter()
            .map(|r| {
                let mut t: Vec<String> = r.outer.iter().map(|s| s.tag.clone()).collect();
                t.sort();
                t.dedup();
                t
            })
            .collect()
    };
    (tags(&p.regions), tags(&p.fills))
}

#[test]
fn ring_is_cut_into_a_face_per_petal_plus_the_annulus() {
    for (kind, gap) in [
        (RingKind::Circle, 0.0),
        (RingKind::Circle, -3e-5),
        (RingKind::Circle, 3e-5),
        (RingKind::TwoArcs, 0.0),
        (RingKind::TwoArcs, -3e-5),
        (RingKind::Shared, 0.0),
    ] {
        let (regions, fills) = face_tags(kind, gap);
        // the regions of the sketch are as before: the band and the hole's own disc
        assert_eq!(regions, [vec!["outer".to_string()], vec!["hole".to_string()]], "gap {gap}");
        assert_eq!(fills.len(), 7, "gap {gap}: {fills:?}");
        for k in 0..6 {
            let own = [format!("s{k}a"), format!("s{k}b"), format!("s{k}e")];
            let n = fills.iter().filter(|t| own.iter().all(|o| t.contains(o))).count();
            assert_eq!(n, 2, "gap {gap} petal {k} (the petal and the ring minus petals): {fills:?}");
        }
    }
}
