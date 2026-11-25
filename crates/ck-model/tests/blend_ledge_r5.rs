//! L body (slab 7 x 40 x 10 under a wall 4 x 40 x d, flush on the back and both ends), R5 on the slab's outer corner edge and on the wall's
//! outer corner edge: the wall is 4 wide, so its round runs off the back face (cliff overflow), and the round must run the full height of the wall.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::f64::consts::PI;

fn rect(id: &str, pts: [(f64, f64); 4], plane: &str) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let e: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % 4 + 1)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}

fn build(d: f64, slab_round: bool, wall_round: bool) -> (Option<ck_topo::body::Body>, Vec<String>, Vec<String>) {
    let mut f = vec![
        rect("F1", [(16.0, -20.0), (23.0, -20.0), (23.0, 20.0), (16.0, 20.0)], r#"{"kind":"std","id":"front"}"#),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#.to_string(),
        r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10}"#.to_string(),
        rect("F4", [(16.0, -20.0), (20.0, -20.0), (20.0, 20.0), (16.0, 20.0)], r#"{"kind":"feature","id":"F3"}"#),
        format!(r#"{{"type":"extrude","id":"F5","op":"add","sketch":"F4","end":"blind","depth":{d}}}"#),
    ];
    let mut edges = vec![];
    if slab_round {
        edges.push(r#"{"faces":["F2.S.e2","F2.S.e3"],"at":[23,20,5]}"#.to_string());
    }
    if wall_round {
        edges.push(format!(r#"{{"faces":["F5.S.e2","F2.S.e3"],"at":[20,20,{}]}}"#, 10.0 + d / 2.0));
    }
    if !edges.is_empty() {
        f.push(format!(r#"{{"type":"fillet","id":"F6","radius":5,"edges":[{}]}}"#, edges.join(",")));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let g = |v: &Vec<regen::RegenError>| v.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    (br.body, g(&r.errors), g(&r.warnings))
}

fn integrate(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    const N: usize = 400_000;
    (0..N).map(|i| f(a + (b - a) * (i as f64 + 0.5) / N as f64)).sum::<f64>() * (b - a) / N as f64
}

/// Err(why) unless the part builds one valid lump of volume `want` (relative 1e-6).
fn expect(name: &str, d: f64, slab: bool, wall: bool, want: f64) -> Result<(), String> {
    let (b, errs, _) = build(d, slab, wall);
    if !errs.is_empty() {
        return Err(format!("{name}: {errs:?}"));
    }
    let b = b.ok_or_else(|| format!("{name}: no body"))?;
    let issues = ck_check::validate(&b, 1e-6);
    if !issues.is_empty() {
        return Err(format!("{name}: invalid {:?}", issues.iter().map(|i| i.code).collect::<Vec<_>>()));
    }
    let v = ck_topo::mass::mass_props(&b).volume;
    if (v - want).abs() > 1e-6 * want {
        return Err(format!("{name}: volume {v} vs {want} (diff {})", v - want));
    }
    Ok(())
}

#[test]
fn both_edges_of_the_ledge_body_are_rounded_over_their_full_length() {
    // per unit height the wall's round takes the section between the corner and the arc, clipped to the wall's 4 mm back face
    let wall = integrate(1.0, 5.0, |u| 5.0 - (25.0 - u * u).sqrt());
    // the slab's round takes 25 (1 - pi / 4) per unit height over its 10 mm
    let slab = 25.0 * (1.0 - PI / 4.0) * 10.0;
    let mut fails = vec![];
    for d in [5.0, 12.0, 42.0] {
        let sharp = 2800.0 + 160.0 * d;
        for (name, s, w, want) in [
            ("wall edge alone", false, true, sharp - wall * d),
            // (the slab's tool must stop at the ledge: run on past it, it cut the wall standing on the ledge)
            ("slab edge alone", true, false, sharp - slab),
            ("both edges", true, true, sharp - slab - wall * d),
        ] {
            if let Err(e) = expect(&format!("{name} d={d}"), d, s, w, want) {
                fails.push(e);
            }
        }
    }
    assert!(fails.is_empty(), "{}", fails.join("\n"));
}

#[test]
fn the_wall_round_warns_that_it_ran_off_the_back_face() {
    let (_, e, w) = build(42.0, true, true);
    assert!(e.is_empty(), "{e:?}");
    assert!(w.iter().any(|x| x.contains("blendRanOffFace")), "{w:?}");
}
