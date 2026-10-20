//! A fillet / chamfer whose edge ENDS at a curved face (a bar or box edge that runs into a joined
//! cylinder, or a boss-foot arc that runs into the bar's side wall). The tool must end on that face:
//! on the cylinder for a convex edge (no notch into the boss behind it), on the wall's plane for a
//! concave arc. Expected values are analytic, integrated over the removed (or added) wedge.
//! (SolidCraft issue cases 556 and 1207.)
use ck_geom::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_ops::contain::{Containment, Loc};
use std::f64::consts::FRAC_PI_2;

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

struct M {
    vol: f64,
    area: f64,
    faces: usize,
    body: ck_topo::body::Body,
}

/// regen the part (without its last feature when `drop_last`); Err = the error codes
fn run(src: &str, drop_last: bool) -> Result<M, Vec<String>> {
    let mut part = Part::parse(src).unwrap();
    if drop_last {
        part.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if !r.errors.is_empty() {
        return Err(r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect());
    }
    let b = ex.0.body.clone().unwrap();
    assert!(ck_topo::check::check(&b, 1e-6).is_empty(), "invalid body: {:?}", ck_topo::check::check(&b, 1e-6));
    let m = ck_topo::mass::mass_props(&b);
    let faces = b.face_ids().count();
    Ok(M { vol: m.volume, area: m.area, faces, body: b })
}

/// (base, with the blend)
fn pair(src: &str) -> (M, M) {
    let a = run(src, true).unwrap_or_else(|e| panic!("base failed: {e:?}"));
    let b = run(src, false).unwrap_or_else(|e| panic!("blend failed: {e:?}"));
    (a, b)
}

fn loc(m: &M, p: [f64; 3]) -> Loc {
    Containment::new(&m.body, 1e-7).classify_point(Vec3::new(p[0], p[1], p[2]), 1e-7)
}

/// midpoint-rule integral of f over [0, r]
fn integ(r: f64, f: impl Fn(f64) -> f64) -> f64 {
    let n = 20000;
    (0..n).map(|i| f((i as f64 + 0.5) * r / n as f64)).sum::<f64>() * r / n as f64
}

/// height of the fillet wedge (outside the quarter circle of radius r) at distance d from one face
fn wedge_fillet(r: f64, d: f64) -> f64 {
    r - (r * r - (r - d) * (r - d)).max(0.0).sqrt()
}

const BOX_FILLET: &str = include_str!("fixtures/curved_ends/556-box-cyl-fillet.ckpart");
const BOX_CHAMFER: &str = include_str!("fixtures/curved_ends/556-box-cyl-chamfer.ckpart");
const BAR_STRAIGHT: &str = include_str!("fixtures/curved_ends/1207-straight-edge-into-boss-r2.ckpart");
const BAR_ARC: &str = include_str!("fixtures/curved_ends/1207-arc-edge-boss-r2.ckpart");
const BAR_ARC_ONE: &str = include_str!("fixtures/curved_ends/1207-arc-edge-boss-r2-single-pick.ckpart");

/// 40 x 40 x 20 box, joined cylinder (axis z through (40, 20), radius `cr`, z 0..40); the blend is on the
/// box edge x = 40, y 0 .. 20 - cr, at the bottom (z = 0) or the top (z = 20) of the box
fn box_cyl(chamfer: bool, size: f64, cr: f64, top: bool) -> String {
    let mut s = if chamfer { BOX_CHAMFER } else { BOX_FILLET }.replace("\"r\":12", &format!("\"r\":{cr}"));
    s = s.replace(if chamfer { "\"distance\":0.5" } else { "\"radius\":0.5" }, &format!("\"{}\":{size}", if chamfer { "distance" } else { "radius" }));
    if top {
        s = s.replace("\"E2.C0\"", "\"E2.C1\"").replace("\"at\":[40,4,0]", "\"at\":[40,2,20]");
    }
    s
}

/// removed volume of the blend on the box edge: the wedge, over the edge's length up to the cylinder
fn box_removed(chamfer: bool, size: f64, cr: f64) -> f64 {
    integ(size, |d| (if chamfer { size - d } else { wedge_fillet(size, d) }) * (20.0 - (cr * cr - d * d).sqrt()))
}

fn check_box(chamfer: bool, size: f64, cr: f64, top: bool) {
    let (a, b) = pair(&box_cyl(chamfer, size, cr, top));
    let exp = box_removed(chamfer, size, cr);
    let got = a.vol - b.vol;
    assert!((got - exp).abs() < 2e-5 * exp.max(1.0), "removed {got} expected {exp}");
    assert_eq!(b.faces, a.faces + 1, "one blend face; its end merges into the cylinder");
    let z = if top { 20.0 - 0.05 } else { 0.05 };
    // the cylinder behind the end is untouched; the wedge before it is gone
    let ye = 20.0 - cr;
    assert_eq!(loc(&b, [39.9, ye + 0.4, z]), Loc::In, "cylinder material kept");
    assert_eq!(loc(&b, [39.98, ye - 0.3, z]), Loc::Out, "wedge removed");
}

#[test]
fn box_cyl_fillet_556() {
    check_box(false, 0.5, 12.0, false);
}
#[test]
fn box_cyl_chamfer_556() {
    check_box(true, 0.5, 12.0, false);
}
#[test]
fn box_cyl_fillet_radii() {
    for r in [0.25, 1.0, 2.0, 4.0] {
        check_box(false, r, 12.0, false);
    }
}
#[test]
fn box_cyl_chamfer_sizes() {
    for r in [1.0, 2.5] {
        check_box(true, r, 12.0, false);
    }
}
#[test]
fn box_cyl_top_edge() {
    check_box(false, 1.0, 12.0, true);
    check_box(true, 1.0, 12.0, true);
}
#[test]
fn box_cyl_boss_radius_vs_blend_radius() {
    // a boss barely wider than the blend (r 1.5 on a 3 mm boss) and a large one
    check_box(false, 1.5, 3.0, false);
    check_box(false, 1.0, 6.0, false);
    check_box(true, 1.0, 3.0, false);
    check_box(false, 1.0, 19.0, false);
}

/// 100 x 30 x 10 bar (y -15..15) with a joined cylinder (axis z through the origin, radius 20, z 0..20)
fn bar(src: &str, size: f64, chamfer: bool, other_side: bool) -> String {
    let mut s = src.replace("\"radius\":2", &format!("\"radius\":{size}"));
    if chamfer {
        s = s.replace("\"type\":\"fillet\"", "\"type\":\"chamfer\",\"chamferType\":\"equal\"").replace("\"radius\"", "\"distance\"");
    }
    if other_side {
        s = s.replace("\"E2.S.e3\"", "\"E2.S.e1\"").replace("\"at\":[60,15,10]", "\"at\":[60,-15,10]");
    }
    s
}

/// removed volume of a blend on the bar's top edge y = 15 from the boss (x = sqrt(400 - y^2)) to the end x = 100
fn bar_removed(size: f64, chamfer: bool) -> f64 {
    integ(size, |d| (if chamfer { size - d } else { wedge_fillet(size, d) }) * (100.0 - (400.0 - (15.0 - d) * (15.0 - d)).sqrt()))
}

fn check_bar(size: f64, chamfer: bool, other_side: bool) {
    let (a, b) = pair(&bar(BAR_STRAIGHT, size, chamfer, other_side));
    let exp = bar_removed(size, chamfer);
    let got = a.vol - b.vol;
    assert!((got - exp).abs() < 2e-5 * exp, "removed {got} expected {exp}");
    assert_eq!(b.faces, a.faces + 1);
    let s = if other_side { -1.0 } else { 1.0 };
    // boss material next to the end stays (the old tool ran ~3 radii past the end and notched it)
    assert_eq!(loc(&b, [13.0, s * 14.99, 9.99]), Loc::In, "boss material kept");
    assert_eq!(loc(&b, [30.0, s * 14.99, 9.99]), Loc::Out, "edge rounded");
    assert_eq!(loc(&b, [30.0, s * 12.0, 8.0]), Loc::In);
}

#[test]
fn straight_edge_into_boss_1207() {
    check_bar(2.0, false, false);
}
#[test]
fn straight_edge_into_boss_radii() {
    for r in [0.5, 1.0, 3.0, 4.0] {
        check_bar(r, false, false);
    }
}
#[test]
fn straight_edge_into_boss_other_side() {
    check_bar(2.0, false, true);
    check_bar(1.0, true, true);
}
#[test]
fn straight_edge_into_boss_chamfer() {
    check_bar(2.0, true, false);
}

/// volume and area added by a concave fillet of radius r along the foot arc of the boss (radius 20) on the bar top (half width
/// 15): the swept wedge, cut by the bar's side planes
fn arc_added(r: f64) -> (f64, f64) {
    let (rr, yy) = (20.0, 15.0);
    let th = |rho: f64| 2.0 * (yy / rho).min(1.0).asin();
    let vol = integ(r, |a| (rr + a) * th(rr + a) * wedge_fillet(r, a));
    let top = integ(r, |a| (rr + a) * th(rr + a));
    let wall = rr * th(rr) * r;
    let n = 20000;
    let blend: f64 = (0..n)
        .map(|i| {
            let p = (i as f64 + 0.5) * FRAC_PI_2 / n as f64;
            let rho = rr + r - r * p.cos();
            rho * th(rho) * r * FRAC_PI_2 / n as f64
        })
        .sum();
    let end = integ(r, |a| wedge_fillet(r, a) * (rr + a) / ((rr + a) * (rr + a) - yy * yy).sqrt());
    (vol, -top - wall + blend + 2.0 * end)
}

fn check_arc(src: &str, r: f64) {
    let (a, b) = pair(&src.replace("\"radius\":2", &format!("\"radius\":{r}")));
    let (ev, ea) = arc_added(r);
    assert!((b.vol - a.vol - ev).abs() < 2e-5 * ev, "added {} expected {ev}", b.vol - a.vol);
    assert!((b.area - a.area - ea).abs() < 2e-5 * ea.abs(), "area change {} expected {ea}", b.area - a.area);
    assert_eq!(b.faces, a.faces + 2, "two blend faces (the arc is two B-rep edges); the ends merge into the bar's sides");
    // the fillet is there, and ends on the side wall (nothing added beyond y = +-15)
    assert_eq!(loc(&b, [20.0 + 0.1, 0.0, 10.1]), Loc::In);
    assert_eq!(loc(&b, [13.3, -15.2, 10.05]), Loc::Out, "nothing beyond the side wall");
    assert_eq!(loc(&b, [13.3, 15.2, 10.05]), Loc::Out, "nothing beyond the side wall");
}

#[test]
fn arc_edge_boss_two_picks_1207() {
    check_arc(BAR_ARC, 2.0);
}
#[test]
fn arc_edge_boss_single_pick_extends_over_the_seam_1207() {
    check_arc(BAR_ARC_ONE, 2.0);
    // the pick on the other half of the arc gives the same
    let other = BAR_ARC_ONE.replace("\"at\":[19.3649,5,10]", "\"at\":[19.3649,-5,10]");
    check_arc(&other, 2.0);
}
#[test]
fn arc_edge_boss_radii() {
    for r in [1.0, 3.0] {
        check_arc(BAR_ARC, r);
        check_arc(BAR_ARC_ONE, r);
    }
}


/// the error params of a refused blend
fn refusal(src: &str) -> (String, Vec<(String, String)>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let e = r.errors.first().expect("expected a refusal");
    (e.error.code.clone(), e.error.params.iter().map(|(k, v)| (k.clone(), format!("{v:?}"))).collect())
}

fn param<'a>(p: &'a [(String, String)], k: &str) -> Option<&'a str> {
    p.iter().find(|(n, _)| n == k).map(|(_, v)| v.trim_start_matches("Str(\"").trim_end_matches("\")"))
}

/// A blend too large for the face next to a curved end is refused with `blendOverflow`; the hint names the end that
/// meets the curved face / wall and the largest size that fits (and that size then works).
#[test]
fn too_large_is_refused_with_a_hint() {
    // box side face is 20 high: the largest fillet is just under 20
    let (code, p) = refusal(&box_cyl(false, 25.0, 12.0, false));
    assert_eq!(code, "blendOverflow");
    let max: f64 = param(&p, "max").unwrap().parse().unwrap();
    assert!((19.9..=20.0).contains(&max), "max {max}");
    assert!(matches!(param(&p, "curvedEnd"), Some("start" | "end")));
    pair(&box_cyl(false, max, 12.0, false));
    // the boss wall above the bar top is 10 high
    let (code, p) = refusal(&BAR_ARC.replace("\"radius\":2", "\"radius\":14"));
    assert_eq!(code, "blendOverflow");
    let max: f64 = param(&p, "max").unwrap().parse().unwrap();
    assert!((9.9..=10.0).contains(&max), "max {max}");
    assert!(param(&p, "curvedEnd").is_some());
    let (a, b) = pair(&BAR_ARC.replace("\"radius\":2", &format!("\"radius\":{max}")));
    assert!(b.vol > a.vol);
    // the straight edge into the boss: same
    let (code, p) = refusal(&bar(BAR_STRAIGHT, 14.0, false, false));
    assert_eq!(code, "blendOverflow");
    assert!(param(&p, "max").is_some() && param(&p, "curvedEnd").is_some());
}
