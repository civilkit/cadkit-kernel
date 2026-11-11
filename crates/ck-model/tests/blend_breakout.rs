//! A convex rim arc that ends on a face it meets at an oblique angle: a round pocket in a plate whose circle crosses
//! the plate's side wall (the pocket breaks out through it). The rim arc on the top face runs from one side-wall
//! crossing to the other; its rolling-ball round must end on the side wall at the oblique crossing, trimmed by that
//! wall. The section at the arc's end vertex has its top-face contact outside the plate (past the side wall), which is
//! not an overflow: the wall cuts the round there.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

/// Plate 80 x 80 x 30 (x, y in -40..40, z in 0..30); a diameter 20 pocket 10 deep from the top, centred at (0, cy);
/// optionally the rim arc on the top face rounded with radius `r`.
fn plate(cy: f64, r: Option<f64>) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let mut feats = vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-40,"y":-40},{"id":"p2","x":40,"y":-40},{"id":"p3","x":40,"y":40},{"id":"p4","x":-40,"y":40}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#.to_string(),
        r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":30}"#.to_string(),
        format!(r#"{{"type":"sketch","id":"F4","plane":{{"kind":"feature","id":"F3"}},"points":[{{"id":"c","x":0,"y":{cy}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}}"#),
        r#"{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"blind","depth":10}"#.to_string(),
    ];
    if let Some(r) = r {
        // the rim arc's point nearest the plate centre
        feats.push(format!(r#"{{"type":"fillet","id":"F6","radius":{r},"edges":[{{"faces":["F5.S.e1","F2.C1"],"at":[0,{},30]}}]}}"#, cy - 10.0));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

/// Material a round of radius `r` removes along the rim of a radius `rp` pocket whose centre is `dy` from the side
/// wall y = 40 (inside the plate): its section (the corner square less the quarter disc, in the meridian half plane at
/// radius rho in rp..rp + r) revolved over the angles where the plate is, pi + 2 asin(dy / rho) at radius rho.
fn removed(rp: f64, r: f64, dy: f64) -> f64 {
    const N: usize = 400_000;
    let mut v = 0.0;
    for i in 0..N {
        let rho = rp + r * (i as f64 + 0.5) / N as f64;
        // depth of the removed corner at rho (the ball's centre is at rho = rp + r, depth r)
        let h = r - (r * r - (rp + r - rho).powi(2)).max(0.0).sqrt();
        let ang = if dy >= rho { std::f64::consts::TAU } else { std::f64::consts::PI + 2.0 * (dy / rho).asin() };
        v += rho * ang * h;
    }
    v * r / N as f64
}

fn check(dy: f64, r: f64) {
    let cy = 40.0 - dy;
    let (bare, e0) = plate(cy, None);
    assert!(e0.is_empty(), "{e0:?}");
    let bare = bare.unwrap();
    let (b, errs) = plate(cy, Some(r));
    assert!(errs.is_empty(), "dy {dy} r {r}: {errs:?}");
    let b = b.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    assert!(issues.is_empty(), "dy {dy} r {r}: {:?}", issues.iter().map(|i| i.code).collect::<Vec<_>>());
    let want = ck_topo::mass::mass_props(&bare).volume - removed(10.0, r, dy);
    let got = ck_topo::mass::mass_props(&b).volume;
    assert!((got - want).abs() < 1e-6 * want, "dy {dy} r {r}: volume {got} vs {want} (removed {} of {})", ck_topo::mass::mass_props(&bare).volume - got, removed(10.0, r, dy));
}

#[test]
fn rim_round_ends_on_an_oblique_side_wall() {
    check(6.0, 1.0);
    check(6.0, 0.3);
    check(6.0, 2.0);
}

#[test]
fn rim_round_ends_on_a_side_wall_crossed_at_other_angles() {
    // shallow crossing (the circle just breaks out) and steep crossing (most of the circle outside)
    check(9.0, 0.5);
    check(2.0, 1.0);
    check(-4.0, 1.0);
}

#[test]
fn rim_round_breaking_out_square_and_inside() {
    // centred on the wall: the arc meets it at 90 degrees; well inside: a full ring
    check(0.0, 1.0);
    check(20.0, 1.0);
}
