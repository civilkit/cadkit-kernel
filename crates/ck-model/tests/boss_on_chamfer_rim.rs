//! A boss (or pocket) sketched on a face from that face's own boundary, after a chamfer or fillet on the rim: the
//! profile equals the boundary of the planar top face, so the boss wall starts along the rim edges of the chamfer and
//! its floor coincides with the top face. The union must close: volume = old + area * depth, valid, one body.
//! The profile is "converted edges": splines (or lines, or a circle) through points sampled on the top face's boundary.
//! Bodies are built here from part JSON; the outline is a generic closed spline blob.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
}

fn run(feats: &[String]) -> Out {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    let errors = r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    Out { body: br.body, errors }
}

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// A pick for every edge of face `cap` (cap listed first).
fn picks(b: &Body, cap: &str) -> Vec<String> {
    let f = b.find_face(cap).unwrap();
    let mut out = vec![];
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 || !fs.contains(&f) {
            continue;
        }
        let ed = b.edge(e);
        let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let other = if fs[0] == f { fs[1] } else { fs[0] };
        out.push(format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]}}"#, b.face_id_str(f), b.face_id_str(other), p.x, p.y, p.z));
    }
    out
}

/// Sampled boundary of face `face`: one polyline per edge (ends shared by position), all in the face plane.
fn loop_samples(b: &Body, face: &str, n: usize) -> Vec<Vec<(f64, f64)>> {
    let f = b.find_face(face).unwrap();
    let mut out = vec![];
    for e in b.edge_ids() {
        if !b.edge_faces(e).contains(&f) {
            continue;
        }
        let ed = b.edge(e);
        out.push((0..=n).map(|k| { let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64); (p.x, p.y) }).collect());
    }
    out
}

fn near(a: (f64, f64), b: (f64, f64)) -> bool {
    (a.0 - b.0).abs() + (a.1 - b.1).abs() < 1e-7
}

/// The sketch JSON of the boundary as splines. `merge`: one closed spline through all samples; otherwise one spline
/// per boundary edge (the two-spline outline of a split closed spline).
fn spline_sketch(id: &str, plane: &str, loops: &[Vec<(f64, f64)>], merge: bool) -> String {
    let mut pts: Vec<(f64, f64)> = vec![];
    let mut pid = |p: (f64, f64), pts: &mut Vec<(f64, f64)>| -> usize {
        if let Some(i) = pts.iter().position(|q| near(*q, p)) { return i + 1; }
        pts.push(p);
        pts.len()
    };
    let mut ents = vec![];
    if merge {
        // chain the edges end to start, dropping the repeated joints
        let mut chain: Vec<(f64, f64)> = vec![];
        let mut rest: Vec<Vec<(f64, f64)>> = loops.to_vec();
        chain.extend(rest.remove(0));
        while !rest.is_empty() {
            let end = *chain.last().unwrap();
            let i = rest.iter().position(|r| near(r[0], end) || near(*r.last().unwrap(), end)).expect("chain");
            let mut r = rest.remove(i);
            if !near(r[0], end) { r.reverse(); }
            chain.extend(r.into_iter().skip(1));
        }
        if near(chain[0], *chain.last().unwrap()) { chain.pop(); }
        let ids: Vec<String> = chain.iter().map(|p| format!("p{}", pid(*p, &mut pts))).collect();
        ents.push(format!(r#"{{"type":"spline","id":"e1","pts":[{}],"closed":true}}"#, ids.iter().map(|s| format!("\"{s}\"")).collect::<Vec<_>>().join(",")));
    } else {
        for (k, l) in loops.iter().enumerate() {
            let closed = near(l[0], *l.last().unwrap());
            let body = if closed { &l[..l.len() - 1] } else { &l[..] };
            let ids: Vec<String> = body.iter().map(|p| format!("\"p{}\"", pid(*p, &mut pts))).collect();
            ents.push(format!(r#"{{"type":"spline","id":"e{}","pts":[{}]{}}}"#, k + 1, ids.join(","), if closed { r#","closed":true"# } else { "" }));
        }
    }
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{:.12},"y":{:.12},"fixed":true}}"#, i + 1, p.0, p.1)).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{}],"entities":[{}],"constraints":[]}}"#, ps.join(","), ents.join(","))
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Rim {
    Chamfer,
    Fillet,
}

fn blend(rim: Rim, picks: &[String]) -> String {
    match rim {
        Rim::Chamfer => format!(r#"{{"type":"chamfer","id":"F3","chamferType":"equal","distance":4,"edges":[{}]}}"#, picks.join(",")),
        Rim::Fillet => format!(r#"{{"type":"fillet","id":"F3","radius":4,"edges":[{}]}}"#, picks.join(",")),
    }
}

/// Block 14 high on a closed-spline blob outline, the top rim chamfered/filleted.
fn blob(rim: Rim) -> Vec<String> {
    let pts = [(0.0, 0.0), (0.0, 46.0), (-46.0, 46.0), (-92.0, 7.0), (-70.0, -30.0), (-31.0, -38.0), (-12.0, -20.0)];
    let ps: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{},"y":{}}}"#, i + 1, p.0, p.1)).collect();
    let ids: Vec<String> = (1..=pts.len()).map(|i| format!("\"p{i}\"")).collect();
    let mut feats = vec![
        format!(r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{{"type":"spline","id":"e1","pts":[{}],"closed":true}}],"constraints":[]}}"#, ps.join(","), ids.join(",")),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":14}"#.to_string(),
    ];
    let o = run(&feats);
    assert!(o.errors.is_empty(), "base: {:?}", o.errors);
    let p = picks(o.body.as_ref().unwrap(), "F2.C1");
    assert!(!p.is_empty());
    feats.push(blend(rim, &p));
    feats
}

fn check(b: &Body, what: &str) {
    let issues: Vec<String> = ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {}", i.code, i.detail)).collect();
    assert!(issues.is_empty(), "{what}: validate issues: {issues:?}");
}

const FACE: &str = r#"{"kind":"face","id":"F2.C1"}"#;

/// Boss/pocket of `depth` on the top face from the boundary sketch `sk`. Returns (old volume, volume of the same
/// prism as a new body = area * depth, result of the add/cut).
fn boss(feats: &[String], sk: &dyn Fn(&Body) -> String, op: &str, depth: f64) -> (f64, f64, Out) {
    let base = run(feats);
    assert!(base.errors.is_empty(), "base: {:?}", base.errors);
    let b0 = base.body.unwrap();
    let old = vol(&b0);
    let sketch = sk(&b0);
    let mut with_new = feats.to_vec();
    with_new.push(sketch.clone());
    with_new.push(format!(r#"{{"type":"extrude","id":"F5","op":"new","sketch":"F4","end":"blind","depth":{depth}}}"#));
    let nb = run(&with_new);
    assert!(nb.errors.is_empty(), "new body: {:?}", nb.errors);
    // the prism is the last body; with op new the Brep keeps one body? use the part volume delta instead
    let prism = {
        let nbody = nb.body.unwrap();
        let v = vol(&nbody);
        if v > old + 1e-6 { v - old } else { v }
    };
    let mut f = feats.to_vec();
    f.push(sketch);
    f.push(format!(r#"{{"type":"extrude","id":"F5","op":"{op}","sketch":"F4","end":"blind","depth":{depth}}}"#));
    (old, prism, run(&f))
}

fn spline_case(rim: Rim, merge: bool, op: &str, depth: f64) {
    let feats = blob(rim);
    let what = format!("{rim:?} merge={merge} {op} {depth}");
    let (old, prism, o) = boss(&feats, &|b| spline_sketch("F4", FACE, &loop_samples(b, "F2.C1", 31), merge), op, depth);
    assert!(prism > 100.0, "{what}: prism {prism}");
    assert!(o.errors.is_empty(), "{what}: {:?}", o.errors);
    let b = o.body.unwrap();
    check(&b, &what);
    let want = if op == "cut" { old - prism } else { old + prism };
    assert!((vol(&b) - want).abs() <= 1e-3 * prism, "{what}: volume {} want {want} (old {old}, prism {prism})", vol(&b));
}

#[test]
fn chamfer_two_splines_boss() {
    spline_case(Rim::Chamfer, false, "add", 10.0);
}
#[test]
fn chamfer_one_closed_spline_boss() {
    spline_case(Rim::Chamfer, true, "add", 10.0);
}
#[test]
fn chamfer_two_splines_boss_shallow() {
    spline_case(Rim::Chamfer, false, "add", 2.0);
}
#[test]
fn chamfer_two_splines_pocket() {
    spline_case(Rim::Chamfer, false, "cut", 6.0);
}
#[test]
fn fillet_two_splines_boss() {
    spline_case(Rim::Fillet, false, "add", 10.0);
}
#[test]
fn fillet_one_closed_spline_boss() {
    spline_case(Rim::Fillet, true, "add", 10.0);
}
#[test]
fn fillet_two_splines_pocket() {
    spline_case(Rim::Fillet, false, "cut", 6.0);
}

/// Rectangle block, the four top edges chamfered or filleted; boss from the inset rectangle (lines).
fn polygon_case(rim: Rim, op: &str, depth: f64) {
    let mut feats = vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-30,"y":-20},{"id":"p2","x":30,"y":-20},{"id":"p3","x":30,"y":20},{"id":"p4","x":-30,"y":20}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":14}"#.to_string(),
    ];
    let o = run(&feats);
    feats.push(blend(rim, &picks(o.body.as_ref().unwrap(), "F2.C1")));
    let sk = |_: &Body| format!(
        r#"{{"type":"sketch","id":"F4","plane":{FACE},"points":[{{"id":"p1","x":-26,"y":-16}},{{"id":"p2","x":26,"y":-16}},{{"id":"p3","x":26,"y":16}},{{"id":"p4","x":-26,"y":16}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    );
    let what = format!("polygon {rim:?} {op}");
    let (old, prism, o) = boss(&feats, &sk, op, depth);
    assert!((prism - 52.0 * 32.0 * depth).abs() < 1e-6 * prism, "{what}: prism {prism}");
    assert!(o.errors.is_empty(), "{what}: {:?}", o.errors);
    let b = o.body.unwrap();
    check(&b, &what);
    let want = if op == "cut" { old - prism } else { old + prism };
    assert!((vol(&b) - want).abs() <= 1e-6 * old, "{what}: volume {} want {want}", vol(&b));
}

#[test]
fn polygon_chamfer_boss() {
    polygon_case(Rim::Chamfer, "add", 10.0);
}
#[test]
fn polygon_fillet_boss() {
    polygon_case(Rim::Fillet, "add", 10.0);
}
#[test]
fn polygon_chamfer_pocket() {
    polygon_case(Rim::Chamfer, "cut", 6.0);
}

/// Cylinder, top rim chamfered or filleted; boss from the top face's circle.
fn disc_case(rim: Rim, op: &str, depth: f64) {
    let mut feats = vec![
        r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":20}],"constraints":[]}"#.to_string(),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":14}"#.to_string(),
    ];
    let o = run(&feats);
    feats.push(blend(rim, &picks(o.body.as_ref().unwrap(), "F2.C1")));
    let sk = |_: &Body| format!(r#"{{"type":"sketch","id":"F4","plane":{FACE},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":16}}],"constraints":[]}}"#);
    let what = format!("disc {rim:?} {op}");
    let (old, prism, o) = boss(&feats, &sk, op, depth);
    assert!(o.errors.is_empty(), "{what}: {:?}", o.errors);
    let b = o.body.unwrap();
    check(&b, &what);
    let want = if op == "cut" { old - prism } else { old + prism };
    assert!((vol(&b) - want).abs() <= 1e-6 * old, "{what}: volume {} want {want}", vol(&b));
}

#[test]
fn disc_chamfer_boss() {
    disc_case(Rim::Chamfer, "add", 10.0);
}
#[test]
fn disc_fillet_boss() {
    disc_case(Rim::Fillet, "add", 10.0);
}
