//! A wheel: a hub (radius 7.5, 6 deep), two opposite quarter-sector plates from radius 7.5 to 50 (5 deep) and a rim ring
//! from radius 50 to 55 (30 deep), the three extrudes fused in that order and then filleted.
//!
//! * `wheel_plates_r5_then_r10_vertical.json`: R5 where the plate tops meet the rim, then R10 on the eight vertical
//!   concave edges (plate side with hub, plate side with rim). The R10 step is refused with `blendUnsupported`
//!   ("concave chain end is not on a planar face", and a hint that names the earlier round): the four rim-side edges run
//!   up to y = 10 where the torus of the R5 blend ends them, and no corner patch joins a concave round to another
//!   round (issue 35, see the tests at the end for what builds and what is refused).
//! * `wheel_plates_r10_hub_edge.json`: R10 on the arcs where the plate tops meet the hub, which stands 1 mm above the
//!   plates (radius larger than the face it rolls over). It builds as a pair of tori over the hub's top edge.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

/// (regen errors as text, the body, its validity issues)
fn build(src: &str) -> (Vec<String>, ck_topo::body::Body, usize) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.take().expect("a body");
    let issues = ck_topo::check::check(&b, 1e-6).len();
    (errors, b, issues)
}

/// The hub fillet no longer answers `blendOverflow`: it builds, valid, as two tori (one per plate arc).
#[test]
fn r10_on_the_plate_to_short_hub_edge_builds_valid() {
    let (errors, b, issues) = build(include_str!("fixtures/wheel_plates_r10_hub_edge.json"));
    assert!(errors.is_empty(), "{errors:?}");
    assert_eq!(issues, 0);
    let tori = b.face_ids().filter(|&f| matches!(b.face(f).surface, ck_geom::surface::Surface::Torus { .. })).count();
    assert_eq!(tori, 2, "one torus per plate arc");
}


// ---- issue 35: R10 on the vertical concave edges after an R5 at the rim

/// The base (hub, plates, rim) as the text of a part up to and including a trailing `, `.
fn prefix() -> &'static str {
    include_str!("fixtures/wheel_plates_r5_then_r10_vertical.json").split_once(r#"{"type": "fillet""#).unwrap().0
}

fn doc(fillets: &[String]) -> String {
    let base = prefix().trim_end().trim_end_matches(',');
    let mut d = base.to_string();
    for f in fillets {
        d.push_str(", ");
        d.push_str(f);
    }
    d.push_str("]}");
    d
}

fn pick(a: &str, b: &str, at: [f64; 3]) -> String {
    format!(r#"{{"faces": ["{a}", "{b}"], "at": [{}, {}, {}]}}"#, at[0], at[1], at[2])
}

fn fillet(id: &str, radius: f64, picks: &[String]) -> String {
    format!(r#"{{"type": "fillet", "id": "{id}", "radius": {radius}, "edges": [{}]}}"#, picks.join(", "))
}

/// The R5 where the plate tops meet the rim (the two arcs of the first plate pair).
fn r5(radius: f64) -> String {
    let q = 35.35533905932738;
    fillet(
        "F7",
        radius,
        &[pick("F4.C1~1", "F6.S.e2", [q, 5.0, -q]), pick("F4.C1~3", "F6.S.e2", [-q, 5.0, q])],
    )
}

/// The vertical concave edges: `(side plane, hub or rim wall, point)`; hub side four, rim side four.
fn vertical(hub: bool, rim: bool, radius: f64) -> String {
    let mut p = vec![];
    for (s, hub_at, rim_at) in [
        ("F4.S.se1", [7.5, 2.5, 0.0], [50.0, 2.5, 0.0]),
        ("F4.S.se3", [0.0, 2.5, -7.5], [0.0, 2.5, -50.0]),
        ("F4.S.te1", [-7.5, 2.5, 0.0], [-50.0, 2.5, 0.0]),
        ("F4.S.te3", [0.0, 2.5, 7.5], [0.0, 2.5, 50.0]),
    ] {
        if hub {
            p.push(pick(s, "F2.S.e1", hub_at));
        }
        if rim {
            p.push(pick(s, "F6.S.e2", rim_at));
        }
    }
    fillet("F8", radius, &p)
}

struct Built {
    errors: Vec<String>,
    body: ck_topo::body::Body,
}

fn build_doc(d: &str) -> Built {
    let part = Part::parse(d).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    Built { errors, body: ex.0.body.take().expect("a body") }
}

fn volume(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// A built body is valid in every check this test file has: the topology check, the validator, the tessellation.
fn assert_sound(b: &ck_topo::body::Body, what: &str) {
    assert_eq!(ck_topo::check::check(b, 1e-6).len(), 0, "{what}: topology check");
    let v: Vec<String> = ck_check::validate(b, 1e-6).into_iter().map(|i| format!("{} {}: {}", i.code, i.entity, i.detail)).collect();
    assert!(v.is_empty(), "{what}: validate {v:?}");
    let m = ck_tess::mesh::tessellate(b).mesh;
    assert!(m.failed.is_empty(), "{what}: faces that did not mesh: {:?}", m.failed_why);
}

/// Area of the concave-corner section of a rolling ball of radius `r` between a radial plane and the hub cylinder
/// (radius `hub`, convex): the part of the triangle (axis, contact on the plane, ball centre) that lies outside the ball and
/// outside the hub.
fn hub_section(hub: f64, r: f64) -> f64 {
    let xc = ((hub + r).powi(2) - r * r).sqrt();
    0.5 * xc * r - 0.5 * r * r * xc.atan2(r) - 0.5 * hub * hub * r.atan2(xc)
}

/// The same at the rim wall (concave cylinder, radius `wall`, the ball inside it).
fn rim_section(wall: f64, r: f64) -> f64 {
    let xc = ((wall - r).powi(2) - r * r).sqrt();
    0.5 * wall * wall * r.atan2(xc) - (0.5 * xc * r + 0.5 * r * r * (std::f64::consts::PI - xc.atan2(r)))
}

const PLATE: f64 = 5.0;

fn base_volume() -> f64 {
    let b = build_doc(&doc(&[]));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    volume(&b.body)
}

/// R10 on the eight vertical concave edges of the plain wheel (no R5): builds, and adds the section of every edge
/// (planar plane / hub and plane / rim corner areas, rolling ball) over the plate's depth.
#[test]
fn r10_on_the_vertical_edges_of_the_plain_wheel_builds() {
    let b = build_doc(&doc(&[vertical(true, true, 10.0)]));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_sound(&b.body, "r10 vertical, plain wheel");
    let want = 4.0 * PLATE * (hub_section(7.5, 10.0) + rim_section(50.0, 10.0));
    let got = volume(&b.body) - base_volume();
    assert!((got - want).abs() < 2e-3 * want, "added {got}, rolling-ball sections give {want}");
    // (the rim is 5 thick, thinner than the round: the tool must not come out through its outer wall)
    let (lo, hi) = b.body.bbox();
    assert!(hi.x.max(hi.z).max(-lo.x).max(-lo.z) < 55.0 + 1e-6, "material outside the rim: bbox {lo:?} {hi:?}");
}

/// After the R5 the four hub-side vertical edges (they end on the flat plate top) build, as before: R10, valid, and the
/// added volume is the hub-side section over the plate's depth.
#[test]
fn r10_on_the_hub_side_vertical_edges_after_r5_builds() {
    let r5_only = build_doc(&doc(&[r5(5.0)]));
    assert!(r5_only.errors.is_empty(), "{:?}", r5_only.errors);
    let b = build_doc(&doc(&[r5(5.0), vertical(true, false, 10.0)]));
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert_sound(&b.body, "r5 then r10 on the hub side");
    let want = 4.0 * PLATE * hub_section(7.5, 10.0);
    let got = volume(&b.body) - volume(&r5_only.body);
    assert!((got - want).abs() < 2e-3 * want, "added {got}, rolling-ball sections give {want}");
}

/// Issue 35: the rim-side vertical edges run up to the torus of the R5 and end on it. That is refused with its reason and a
/// hint naming the earlier round, whichever of the selections it is in, and at any radius of either round (a corner of two
/// rounds is not built); the part keeps the R5 result, valid.
#[test]
fn r10_on_the_vertical_edges_after_r5_is_refused_with_the_reason() {
    let r5_only = build_doc(&doc(&[r5(5.0)]));
    assert!(r5_only.errors.is_empty(), "{:?}", r5_only.errors);
    for (what, round5, f) in [
        ("all eight, R10", 5.0, vertical(true, true, 10.0)),
        ("rim side, R10", 5.0, vertical(false, true, 10.0)),
        ("all eight, R3", 5.0, vertical(true, true, 3.0)),
        ("all eight, R10 after R2", 2.0, vertical(true, true, 10.0)),
    ] {
        let b = build_doc(&doc(&[r5(round5), f]));
        assert_eq!(b.errors.len(), 1, "{what}: {:?}", b.errors);
        let e = &b.errors[0];
        assert!(e.starts_with("F8: blendUnsupported"), "{what}: {e}");
        assert!(e.contains("concave chain end is not on a planar face"), "{what}: {e}");
        assert!(e.contains("on the torus of an earlier round"), "{what}: {e}");
        assert!(e.contains("Leave the edges that run into the earlier round out of this selection"), "{what}: {e}");
        assert_sound(&b.body, what);
        let kept = build_doc(&doc(&[r5(round5)]));
        assert!((volume(&b.body) - volume(&kept.body)).abs() < 1e-9, "{what}: the refused fillet changed the part");
    }
    assert!(volume(&r5_only.body) > 0.0);
}

/// Issue 35, the other order: R10 on the eight vertical edges first, then the R5 on the plate-top arcs. Records what the
/// engine does today (printed with --nocapture); whatever it answers, the part must stay sound.
#[test]
fn r10_then_r5_order_is_sound_whatever_it_answers() {
    let b = build_doc(&doc(&[vertical(true, true, 10.0), r5(5.0)]));
    println!("R10 THEN R5: errors {:?} volume {}", b.errors, volume(&b.body));
    assert_sound(&b.body, "r10 then r5");
}
