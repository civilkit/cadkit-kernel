//! One fillet over the foot and vertical edges of two box bosses standing in line on a plate (their side walls in
//! one plane, a gap between them). Each base edge is a concave edge whose tool runs on to the corner it was
//! shortened from; the corner of the other boss on the same line is no end of it (a tool run on to there filled the
//! gap between the bosses with a strip and left the second boss's base edge `missingEdge`).
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

const R: f64 = 4.0;

fn rect(id0: usize, x0: f64, y0: f64, x1: f64, y1: f64) -> (String, String) {
    let p = |k: usize, x: f64, y: f64| format!(r#"{{"id":"p{k}","x":{x},"y":{y}}}"#);
    let pts = [p(id0, x0, y0), p(id0 + 1, x1, y0), p(id0 + 2, x1, y1), p(id0 + 3, x0, y1)].join(",");
    let ents = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, id0 + i, id0 + i, id0 + (i + 1) % 4)).collect::<Vec<_>>().join(",");
    (pts, ents)
}

/// Plate 200 x 400 x 10, bosses 80 x 150 x 50 at y 0..150 and (when `two`) 200..350; with `fillet`, radius `R` on
/// every vertical and base edge of the bosses. The body and the regen errors.
fn part(bosses: &[(f64, f64)], fillet: bool) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let (pp, pe) = rect(1, -100.0, -25.0, 100.0, 375.0);
    let mut pts = vec![];
    let mut ents = vec![];
    let mut picks = vec![];
    for (k, &(y0, y1)) in bosses.iter().enumerate() {
        let (p, e) = rect(1 + 4 * k, -40.0, y0, 40.0, y1);
        pts.push(p);
        ents.push(e);
        // sides e(1+4k).. in the order bottom (y0), right (x 40), top (y1), left (x -40)
        let s = |i: usize| format!("F5.S.e{}", 1 + 4 * k + i);
        let ym = 0.5 * (y0 + y1);
        let mids = [(0.0, y0), (40.0, ym), (0.0, y1), (-40.0, ym)];
        let corners = [(40.0, y0), (40.0, y1), (-40.0, y1), (-40.0, y0)];
        for i in 0..4 {
            picks.push(format!(r#"{{"faces":["{}","F2.C1"],"at":[{},{},10]}}"#, s(i), mids[i].0, mids[i].1));
            picks.push(format!(r#"{{"faces":["{}","{}"],"at":[{},{},35]}}"#, s(i), s((i + 1) % 4), corners[i].0, corners[i].1));
        }
    }
    let mut feats = vec![
        format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{pp}],"entities":[{pe}],"constraints":[]}}"#),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#.to_string(),
        r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10}"#.to_string(),
        format!(r#"{{"type":"sketch","id":"F4","plane":{{"kind":"feature","id":"F3"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(",")),
        r#"{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":50}"#.to_string(),
    ];
    if bosses.is_empty() {
        feats.truncate(2);
    }
    if fillet {
        feats.push(format!(r#"{{"type":"fillet","id":"F6","radius":{R},"edges":[{}]}}"#, picks.join(",")));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect())
}

fn vol(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

#[test]
fn fillet_of_two_bosses_in_line() {
    let (a, b) = ((0.0, 150.0), (200.0, 350.0));
    let (two, errs) = part(&[a, b], true);
    assert!(errs.is_empty(), "{errs:?}");
    let two = two.unwrap();
    assert!(ck_check::validate(&two, 1e-6).is_empty(), "{:?}", ck_check::validate(&two, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
    // each boss is filleted as it is on its own: the gap between them is untouched
    let one = |s: (f64, f64)| {
        let (b, e) = part(&[s], true);
        assert!(e.is_empty(), "{e:?}");
        vol(&b.unwrap())
    };
    let plate = vol(&part(&[], false).0.unwrap());
    let want = one(a) + one(b) - plate;
    assert!((vol(&two) - want).abs() < 1e-6 * want, "volume {} vs {want}", vol(&two));
    // and the fillets changed the bosses (concave base rounds add, convex vertical rounds remove)
    let bare = vol(&part(&[a, b], false).0.unwrap());
    assert!((vol(&two) - bare).abs() > 1.0, "the fillet did nothing");
}
