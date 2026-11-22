//! A convex round or chamfer larger than the face or wall it runs onto (solidcore kernel cases `5158-fillet-round-just-past-a-10mm-face`,
//! `190-fillet-disc-rim-past-its-2mm-wall`, `every-hole-top-rim-r12-past-plate-bottom`, `324-fillet-over-hole-overlap`,
//! `every-chamfer-pocket-rim-5-5-past-the-floor`). The tool is the plain blend wedge on the continued support surfaces; what
//! it takes from the body is the wedge clipped to the body, so the face the blend ran past is used up (or cut back) and the
//! result is one valid lump with the analytic volume. Each volume below comes from an independent closed form or quadrature.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::f64::consts::PI;

const SQ: &str = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":W,"y":0},{"id":"p3","x":W,"y":D},{"id":"p4","x":0,"y":D}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#;

fn block(w: f64, d: f64, h: f64) -> Vec<String> {
    vec![
        SQ.replace("W", &w.to_string()).replace("D", &d.to_string()),
        format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#),
    ]
}

fn disc(r: f64, h: f64) -> Vec<String> {
    vec![
        format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#),
        format!(r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#),
    ]
}

fn through_hole(x: f64, y: f64, r: f64) -> Vec<String> {
    vec![
        format!(r#"{{"type":"sketch","id":"F4","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{x},"y":{y}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#),
        r#"{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"throughBoth"}"#.to_string(),
    ]
}

fn run(feats: Vec<String>) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

/// Err(why) unless the part builds one valid lump of volume `want` (relative 1e-6).
fn expect(name: &str, feats: Vec<String>, want: f64) -> Result<(), String> {
    let (b, errs) = run(feats);
    if !errs.is_empty() {
        return Err(format!("{name}: {errs:?}"));
    }
    let b = b.ok_or_else(|| format!("{name}: no body"))?;
    let issues = ck_check::validate(&b, 1e-6);
    if !issues.is_empty() {
        return Err(format!("{name}: invalid {:?}", issues.iter().map(|i| i.code).collect::<Vec<_>>()));
    }
    // names: every face has its own id, and none is a tool's scratch name
    let mut ids: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    ids.sort();
    if ids.windows(2).any(|w| w[0] == w[1]) || ids.iter().any(|i| i.starts_with("clip") || i.contains(".S.l")) {
        return Err(format!("{name}: face ids {ids:?}"));
    }
    let v = ck_topo::mass::mass_props(&b).volume;
    if (v - want).abs() > 1e-6 * want {
        return Err(format!("{name}: volume {v} vs {want}"));
    }
    Ok(())
}

/// Gauss-Legendre-free midpoint quadrature of f over [a, b].
fn integrate(a: f64, b: f64, f: impl Fn(f64) -> f64) -> f64 {
    const N: usize = 400_000;
    (0..N).map(|i| f(a + (b - a) * (i as f64 + 0.5) / N as f64)).sum::<f64>() * (b - a) / N as f64
}

fn cases() -> Vec<(&'static str, Vec<String>, f64)> {
    let mut out = vec![];
    // box 40 x 10 x 20, top front edge r 10.01 (past the 10 mm back face): the section taken, integral over y in [0, 10]
    let r: f64 = 10.01;
    let a = integrate(0.0, 10.0, |y| r - (r * r - (r - y) * (r - y)).max(0.0).sqrt());
    let mut f = block(40.0, 10.0, 20.0);
    f.push(format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,0,20]}}]}}"#));
    out.push(("box top-front r10.01", f, 40.0 * (200.0 - a)));
    // disc R13 x 2, rim round r 2.5 (circle centre rho 10.5, z -0.5): the part above z = 0 taken, pi int (169 - rho^2) dz
    let rr = 2.5;
    let t0 = (0.2f64).asin();
    let taken = PI * integrate(t0, PI / 2.0, |t| (169.0 - (10.5 + rr * t.cos()).powi(2)) * rr * t.cos());
    let mut f = disc(13.0, 2.0);
    f.push(r#"{"type":"fillet","id":"F3","radius":2.5,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[13,0,2]}]}"#.to_string());
    out.push(("disc rim r2.5", f, PI * 338.0 - taken));
    // plate 40 x 40 x 10 with a through hole R5 at (20, 20), the hole's top rim rounded r 12: taken = 2 pi int (5 + t) min(L(t), 10) dt
    let mut f = block(40.0, 40.0, 10.0);
    f.extend(through_hole(20.0, 20.0, 5.0));
    f.push(r#"{"type":"fillet","id":"F6","radius":12,"edges":[{"faces":["F5.S.e1","F2.C1"],"at":[25,20,10]}]}"#.to_string());
    let t_hi = 12.0 - 140.0f64.sqrt();
    let l = |t: f64| 12.0 - (144.0 - (12.0 - t) * (12.0 - t)).max(0.0).sqrt();
    let taken = 2.0 * PI * (integrate(0.0, t_hi, |_| 0.0) + integrate(0.0, t_hi, |t| (5.0 + t) * 10.0) + integrate(t_hi, 12.0, |t| (5.0 + t) * l(t)));
    out.push(("hole rim r12", f, 16000.0 - 250.0 * PI - taken));
    // block 40 x 20 x 10 with a through hole r3 at (20, 14.5) and R4 on the top long edge y = 20
    let mut f = block(40.0, 20.0, 10.0);
    f.extend(through_hole(20.0, 14.5, 3.0));
    f.push(r#"{"type":"fillet","id":"F6","radius":4,"edges":[{"faces":["F2.C1","F2.S.e3"],"at":[5,20,10]}]}"#.to_string());
    let overlap = integrate(16.0, 17.5, |y| (4.0 - (16.0 - (y - 16.0) * (y - 16.0)).sqrt()) * 2.0 * (9.0 - (y - 14.5) * (y - 14.5)).max(0.0).sqrt());
    out.push(("hole near edge r4", f, 8000.0 - 90.0 * PI - 40.0 * (16.0 - 4.0 * PI) + overlap));
    // plate 60 x 40 x 10 with the pocket x 15..45, y 10..30, 5 deep; its four rim edges chamfered 5.5 (0.5 past the floor)
    let mut f = block(60.0, 40.0, 10.0);
    f.push(r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":10}"#.to_string());
    f.push(r#"{"type":"sketch","id":"F4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"q1","x":15,"y":10},{"id":"q2","x":45,"y":10},{"id":"q3","x":45,"y":30},{"id":"q4","x":15,"y":30}],"entities":[{"type":"line","id":"e1","p1":"q1","p2":"q2"},{"type":"line","id":"e2","p1":"q2","p2":"q3"},{"type":"line","id":"e3","p1":"q3","p2":"q4"},{"type":"line","id":"e4","p1":"q4","p2":"q1"}],"constraints":[]}"#.to_string());
    f.push(r#"{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"blind","depth":5}"#.to_string());
    f.push(r#"{"type":"chamfer","id":"F6","distance":5.5,"edges":[{"faces":["F5.S.e1","F2.C1"],"at":[30,10,10]},{"faces":["F5.S.e2","F2.C1"],"at":[45,20,10]},{"faces":["F5.S.e3","F2.C1"],"at":[30,30,10]},{"faces":["F5.S.e4","F2.C1"],"at":[15,20,10]}]}"#.to_string());
    let d = 5.5;
    out.push(("pocket rim chamfer 5.5", f, 21000.0 - 50.0 * d * d - 4.0 / 3.0 * d * d * d));
    out
}

#[test]
fn convex_round_past_a_cliff_builds() {
    let fails: Vec<String> = cases().into_iter().filter_map(|(n, f, v)| expect(n, f, v).err()).collect();
    assert!(fails.is_empty(), "{}", fails.join("\n"));
}

fn run_warns(feats: Vec<String>) -> (Option<ck_topo::body::Body>, Vec<String>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let f = |v: &Vec<regen::RegenError>| v.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    (br.body, f(&r.errors), f(&r.warnings))
}

#[test]
fn a_round_that_runs_off_its_face_warns_and_is_deterministic() {
    let (_, f, _) = cases().remove(0);
    let (b1, e, w) = run_warns(f.clone());
    assert!(e.is_empty(), "{e:?}");
    assert!(w.iter().any(|x| x.starts_with("F3: blendRanOffFace") && x.contains("F2.C1")), "{w:?}");
    let (b2, _, _) = run_warns(f);
    let sig = |b: &ck_topo::body::Body| (b.face_ids().map(|x| b.face_id_str(x)).collect::<Vec<_>>(), ck_topo::mass::mass_props(b).volume);
    assert_eq!(sig(&b1.unwrap()), sig(&b2.unwrap()));
    // an ordinary round warns of nothing
    let mut f = block(40.0, 10.0, 20.0);
    f.push(r#"{"type":"fillet","id":"F3","radius":2,"edges":[{"faces":["F2.C1","F2.S.e1"],"at":[20,0,20]}]}"#.to_string());
    let (_, e, w) = run_warns(f);
    assert!(e.is_empty() && w.is_empty(), "{e:?} {w:?}");
}

#[test]
fn the_pocket_groove_wall_continues_the_pocket_wall_by_name() {
    let (_, f, _) = cases().remove(4);
    let (b, e, _) = run_warns(f);
    assert!(e.is_empty(), "{e:?}");
    let b = b.unwrap();
    let ids: Vec<String> = b.face_ids().map(|x| b.face_id_str(x)).collect();
    // (5.5 down the 5 deep wall: the wall is used up, the groove walls below the floor are all that is left of it)
    for k in 1..=4 {
        assert!(ids.contains(&format!("F5.S.e{k}")), "{ids:?}");
    }
    assert_eq!(ids.len(), 15, "{ids:?}");
}

#[test]
fn blends_that_run_off_both_faces_stay_refused() {
    // a 30 mm round or chamfer on a 40 x 20 x 10 box: both contacts are off their faces
    for (kind, key) in [("fillet", "radius"), ("chamfer", "distance")] {
        let mut f = block(40.0, 20.0, 10.0);
        f.push(format!(r#"{{"type":"{kind}","id":"F3","{key}":30,"edges":[{{"faces":["F2.C1","F2.S.e1"],"at":[20,0,10]}}]}}"#));
        let (_, e, _) = run_warns(f);
        assert!(e.len() == 1 && e[0].contains("blendOverflow"), "{kind}: {e:?}");
    }
}
