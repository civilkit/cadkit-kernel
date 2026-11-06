//! A concave straight edge whose end face is an oblique plane: the foot edges of a wedge standing on a plate run into
//! the wedge's slanted end faces. The round ends on those faces (its tool is run on past the vertex and clipped by the
//! end face's plane), as SolidWorks ends the round of a rib's foot on the rib's slanted free face; a tool ending flush
//! on a plane square to the edge left material outside the slanted face.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

const R: f64 = 2.0;

fn part(fillet: bool) -> (Option<ck_topo::body::Body>, Vec<String>) {
    // plate 100 x 60 x 10 (front plane); wedge on the right plane: the triangle (y, z) = (-10, 10), (10, 10), (0, 30),
    // from x = -5 to 5 (sketch x is world -z, sketch y world y)
    let mut feats = vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-50,"y":-30},{"id":"p2","x":50,"y":-30},{"id":"p3","x":50,"y":30},{"id":"p4","x":-50,"y":30}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#,
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#,
        r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"right"},"offset":-5}"#,
        r#"{"type":"sketch","id":"F4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"p1","x":-10,"y":-10},{"id":"p2","x":-10,"y":10},{"id":"p3","x":-30,"y":0}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p1"}],"constraints":[]}"#,
        r#"{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":10}"#,
    ]
    .into_iter()
    .map(String::from)
    .collect::<Vec<_>>();
    if fillet {
        feats.push(format!(r#"{{"type":"fillet","id":"F6","radius":{R},"edges":[{{"faces":["F5.C1","F2.C1"],"at":[5,0,10]}},{{"faces":["F5.C0","F2.C1"],"at":[-5,0,10]}}]}}"#));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect())
}

#[test]
fn foot_round_ends_on_the_slanted_end_faces() {
    let (bare, e0) = part(false);
    assert!(e0.is_empty(), "{e0:?}");
    let (b, errs) = part(true);
    assert!(errs.is_empty(), "{errs:?}");
    let b = b.unwrap();
    assert!(ck_check::validate(&b, 1e-6).is_empty(), "{:?}", ck_check::validate(&b, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
    // each round's section (the corner square less the quarter disc, area r^2 (1 - pi/4)) runs along y between the
    // slanted faces y = +-(10 - (z - 10) / 2): its volume is the section's integral of 20 - (z - 10)
    let section = R * R * (1.0 - std::f64::consts::FRAC_PI_4);
    // integral of (z - 10) over the section: the square's r^3 / 2 less the quarter disc's (pi r^3 / 4 - r^3 / 3)
    let moment = R.powi(3) / 2.0 - (std::f64::consts::PI * R.powi(3) / 4.0 - R.powi(3) / 3.0);
    let want = ck_topo::mass::mass_props(&bare.unwrap()).volume + 2.0 * (20.0 * section - moment);
    let got = ck_topo::mass::mass_props(&b).volume;
    assert!((got - want).abs() < 1e-6 * want, "volume {got} vs {want}");
}
