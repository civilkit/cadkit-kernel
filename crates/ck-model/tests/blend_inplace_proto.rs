//! Prototype probe for `blend/inplace.rs`: a bracket whose top outline was rounded, then its concave vertical edge
//! rounded IN PLACE (`CK_BLEND_INPLACE=1`). The reference is the same shape built the other way round: the vertical
//! edge first, then the rim fillet along the tangent outline (line, arc, line ...), which the tool + boolean engine builds.
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

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

const CASES: &str = include_str!("fixtures/blend_cases.json");

fn case_features(id: &str, with_final: bool) -> Vec<Json> {
    let cases = ck_json::parse(CASES).unwrap();
    let c = cases.get_arr("cases").unwrap().iter().find(|c| c.get_str("id") == Some(id)).unwrap();
    let mut f = cases.get("bases").unwrap().get(c.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
    if let Some(p) = c.get("pre") {
        f.push(p.clone());
    }
    if with_final {
        f.push(c.get("fillet").unwrap().clone());
    }
    f
}

fn regen_body(features: &[Json]) -> (Option<Body>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(features.to_vec())));
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let mut msgs: Vec<String> = r.errors.iter().map(|e| format!("{}:{} {}", e.feature, e.error.code, ck_json::to_string(&Json::Obj(e.error.params.clone())))).collect();
    msgs.extend(r.warnings.iter().map(|e| format!("warn {}:{} {}", e.feature, e.error.code, ck_json::to_string(&Json::Obj(e.error.params.clone())))));
    (ex.0.body.clone(), msgs)
}

pub fn dump(b: &Body) -> String {
    let mut s = String::new();
    for f in b.face_ids() {
        let fc = b.face(f);
        s += &format!("face {} {} sense={}\n", b.face_id_str(f), fc.surface.kind(), fc.sense);
        for &l in &fc.loops {
            for &c in &b.lp(l).coedges {
                let ce = b.coedge(c);
                let e = b.edge(ce.edge);
                let (p0, p1) = (b.vertex(e.v0).p, b.vertex(e.v1).p);
                s += &format!("   e{}{} {} [{:.3},{:.3},{:.3}]->[{:.3},{:.3},{:.3}] pc={}\n", ce.edge.idx(), if ce.rev { "'" } else { "" }, e.curve.kind(), p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, ce.pcurve.is_some());
            }
        }
    }
    s
}

fn fillet_all_top(b: &Body, id: &str, r: f64) -> Json {
    // every edge of the top cap F2.C1, as a pick (faces + midpoint)
    let top = b.find_face("F2.C1").unwrap();
    let mut edges = vec![];
    for e in b.face_edges(top) {
        let fs = b.edge_faces(e);
        let names: Vec<Json> = fs.iter().map(|&f| Json::Str(b.face_id_str(f))).collect();
        let m = b.edge_mid(e);
        edges.push(ck_json::parse(&format!(r#"{{"faces":{},"at":[{},{},{}]}}"#, ck_json::to_string(&Json::Arr(names)), m.x, m.y, m.z)).unwrap());
    }
    ck_json::parse(&format!(r#"{{"type":"fillet","id":"{id}","radius":{r},"edges":{}}}"#, ck_json::to_string(&Json::Arr(edges)))).unwrap()
}

/// Prints a body and returns its volume (0 for none); the issue lists are returned for assertions.
fn report(tag: &str, b: &Option<Body>, errs: &[String]) -> f64 {
    report_issues(tag, b, errs).0
}

fn report_issues(tag: &str, b: &Option<Body>, errs: &[String]) -> (f64, usize) {
    match b {
        Some(b) => {
            let v = ck_topo::mass::mass_props(b).volume;
            let issues = ck_check::validate(b, 1e-6);
            let m = ck_tess::mesh::tessellate(b).mesh;
            let mesh = ck_check::tessellation::check_mesh_why(b, &m.positions, &m.tri_face, &m.failed, &m.failed_why);
            println!(
                "{tag}: errors {errs:?} faces {} volume {v:.5} issues {:?} mesh issues {:?}",
                b.face_ids().count(),
                issues.iter().map(|i| format!("{} {}", i.code, i.detail)).take(6).collect::<Vec<_>>(),
                mesh.iter().map(|i| format!("{}: {}", i.entity, i.detail)).take(4).collect::<Vec<_>>()
            );
            (v, issues.len() + mesh.len())
        }
        None => {
            println!("{tag}: no body, errors {errs:?}");
            (0.0, 1)
        }
    }
}

/// The tests switch `CK_BLEND_INPLACE` on and off: one at a time.
static ENV: std::sync::Mutex<()> = std::sync::Mutex::new(());

#[test]
fn inplace_bracket_corner() {
    let _g = ENV.lock().unwrap_or_else(|e| e.into_inner());
    for r in [1.0, 2.0, 3.0] {
        let id = format!("l-bracket-rim-then-inner-r{r}");
        // in place
        std::env::set_var("CK_BLEND_INPLACE", "1");
        let (b, e) = regen_body(&case_features(&id, true));
        let (v_in, bad_in) = report_issues(&format!("r={r} in place"), &b, &e);
        assert!(e.is_empty() && bad_in == 0, "the in-place corner must be a valid closed body: {e:?}");
        std::env::remove_var("CK_BLEND_INPLACE");
        // the engine on the same sequence
        let (b0, e0) = regen_body(&case_features(&id, true));
        report(&format!("r={r} engine, rim first"), &b0, &e0);
        // reference: vertical edge first, then the rim chain
        let cases = ck_json::parse(CASES).unwrap();
        let c = cases.get_arr("cases").unwrap().iter().find(|c| c.get_str("id") == Some(id.as_str())).unwrap();
        let mut f = cases.get("bases").unwrap().get("l-bracket").unwrap().as_arr().unwrap().to_vec();
        f.push(c.get("fillet").unwrap().clone());
        let (b1, _) = regen_body(&f);
        let rim = fillet_all_top(&b1.unwrap(), "F95", 1.0);
        f.push(rim);
        let (b2, e2) = regen_body(&f);
        let v_ref = report(&format!("r={r} reference (vertical first)"), &b2, &e2);
        println!("r={r} in place vs reference: {:+.6}", v_in - v_ref);
        assert!((v_in - v_ref).abs() < 1e-6 * v_ref, "in place {v_in} vs reference {v_ref}");
    }
}

/// `CK_INPLACE_PART=<part file>`: a part whose last two features are a fillet of a top outline and a fillet of one concave
/// vertical edge. Prints the in-place result next to the engine's and next to the other order (corner first, then the
/// rim along the whole top outline).
#[test]
fn inplace_external_part() {
    let Ok(path) = std::env::var("CK_INPLACE_PART") else { return };
    let _g = ENV.lock().unwrap_or_else(|e| e.into_inner());
    let doc = ck_json::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut feats = doc.get_arr("features").unwrap().to_vec();
    for f in feats.iter_mut() {
        f.as_obj_mut().unwrap().retain(|(k, _)| k != "suppressed");
    }
    let n = feats.len();
    assert!(feats[n - 1].get_str("type") == Some("fillet") && feats[n - 2].get_str("type") == Some("fillet"));
    std::env::set_var("CK_BLEND_INPLACE", "1");
    let (b, e) = regen_body(&feats);
    report("in place", &b, &e);
    std::env::remove_var("CK_BLEND_INPLACE");
    let (b0, e0) = regen_body(&feats);
    report("engine, rim first", &b0, &e0);
    if let Ok(only) = std::env::var("CK_INPLACE_DUMP") {
        let (bd, _) = regen_body(&feats[..n - 1]);
        let d = dump(&bd.unwrap());
        // print the faces whose id contains one of the given substrings
        let mut show = false;
        for l in d.lines() {
            if l.starts_with("face ") {
                show = only.split(',').any(|p| l.contains(p));
            }
            if show {
                println!("{l}");
            }
        }
    }
    let (b_pre, _) = regen_body(&feats[..n - 2]);
    let b_pre = b_pre.unwrap();
    report("before both", &Some(b_pre.clone()), &[]);
    // reference: the corner first
    let mut f = feats[..n - 2].to_vec();
    f.push(feats[n - 1].clone());
    let (b1, e1) = regen_body(&f);
    let b1 = b1.unwrap();
    report("corner only", &Some(b1.clone()), &e1);
    // the rim fillet again, now along every edge of the cap the first rim fillet was picked on
    let cap = feats[n - 2].get_arr("edges").unwrap()[0].get_arr("faces").unwrap()[0].as_str().unwrap().to_string();
    let top = b1.find_face(&cap).unwrap();
    let mut edges = vec![];
    for ed in b1.face_edges(top) {
        let fs = b1.edge_faces(ed);
        let names: Vec<Json> = fs.iter().map(|&x| Json::Str(b1.face_id_str(x))).collect();
        let m = b1.edge_mid(ed);
        edges.push(ck_json::parse(&format!(r#"{{"faces":{},"at":[{},{},{}]}}"#, ck_json::to_string(&Json::Arr(names)), m.x, m.y, m.z)).unwrap());
    }
    let mut rim = feats[n - 2].clone();
    rim.as_obj_mut().unwrap().retain(|(k, _)| k != "edges");
    rim.as_obj_mut().unwrap().push(("edges".to_string(), Json::Arr(edges)));
    f.push(rim);
    let (b2, e2) = regen_body(&f);
    report("reference: corner first, then the whole rim", &b2, &e2);
}

fn with_radius(f: &Json, r: f64) -> Json {
    let mut f = f.clone();
    f.as_obj_mut().unwrap().retain(|(k, _)| k != "radius");
    f.as_obj_mut().unwrap().push(("radius".into(), Json::Num(r)));
    f
}

/// Robustness sweep of the in-place corner against the other order of the same two blends.
#[test]
fn inplace_bracket_sweep() {
    let _g = ENV.lock().unwrap_or_else(|e| e.into_inner());
    let cases = ck_json::parse(CASES).unwrap();
    let c = cases.get_arr("cases").unwrap().iter().find(|c| c.get_str("id") == Some("l-bracket-rim-then-inner-r1")).unwrap();
    let base = cases.get("bases").unwrap().get("l-bracket").unwrap().as_arr().unwrap().to_vec();
    let (mut agree, mut refused, mut differ, mut no_ref) = (0, 0, 0, 0);
    for rim in [0.25, 0.5, 1.0, 2.0] {
        for r in [0.25, 0.5, 1.0, 2.0, 3.0, 4.0, 6.0, 8.0] {
            let mut f = base.clone();
            f.push(with_radius(c.get("pre").unwrap(), rim));
            f.push(with_radius(c.get("fillet").unwrap(), r));
            std::env::set_var("CK_BLEND_INPLACE", "1");
            let (b, e) = regen_body(&f);
            std::env::remove_var("CK_BLEND_INPLACE");
            // the other order
            let mut g = base.clone();
            g.push(with_radius(c.get("fillet").unwrap(), r));
            let (b1, _) = regen_body(&g);
            let reference = b1.and_then(|b1| {
                g.push(with_radius(&fillet_all_top(&b1, "F95", rim), rim));
                let (b2, e2) = regen_body(&g);
                if e2.is_empty() { b2 } else { None }
            });
            let mine = b.filter(|_| e.is_empty());
            let tag = format!("rim {rim} corner {r}");
            match (mine, reference) {
                (Some(m), Some(rf)) => {
                    let (vm, vr) = (ck_topo::mass::mass_props(&m).volume, ck_topo::mass::mass_props(&rf).volume);
                    let ok = (vm - vr).abs() < 1e-5 * vr && ck_check::validate(&m, 1e-6).is_empty();
                    println!("{tag}: in place {vm:.5} reference {vr:.5} {}", if ok { "agree" } else { "DIFFER" });
                    if ok { agree += 1 } else { differ += 1 }
                }
                (None, Some(_)) => {
                    println!("{tag}: in place refused {e:?}");
                    refused += 1
                }
                (_, None) => {
                    println!("{tag}: no reference");
                    no_ref += 1
                }
            }
        }
    }
    println!("sweep: {agree} agree, {differ} differ, {refused} refused, {no_ref} without reference");
    assert_eq!(differ, 0);
}

/// The bracket turned by 30 degrees and moved: nothing in the construction may depend on the axes.
#[test]
fn inplace_bracket_rotated() {
    let _g = ENV.lock().unwrap_or_else(|e| e.into_inner());
    let (c, sn) = (30f64.to_radians().cos(), 30f64.to_radians().sin());
    let rot = |x: f64, y: f64| (c * x - sn * y + 7.0, sn * x + c * y - 3.0);
    let outline = [(0.0, 0.0), (40.0, 0.0), (40.0, 10.0), (10.0, 10.0), (10.0, 30.0), (0.0, 30.0)];
    let pts: Vec<String> = outline.iter().enumerate().map(|(i, &(x, y))| { let (a, b) = rot(x, y); format!(r#"{{"id":"p{}","x":{a},"y":{b}}}"#, i + 1) }).collect();
    let ents: Vec<String> = (0..6).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % 6 + 1)).collect();
    let base = format!(
        r#"[{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}}]"#,
        pts.join(","),
        ents.join(",")
    );
    let base = ck_json::parse(&base).unwrap().as_arr().unwrap().to_vec();
    let (b0, e0) = regen_body(&base);
    assert!(e0.is_empty());
    let rim = with_radius(&fillet_all_top(&b0.unwrap(), "F80", 1.0), 1.0);
    // the concave vertical edge: sketch (10, 10) -> world (x, y, -sketch y)
    let (ax, ay) = rot(10.0, 10.0);
    let mut f = base.clone();
    f.push(rim);
    let (bb, _) = regen_body(&f);
    let bb = bb.unwrap();
    let pick = bb.edge_ids().min_by(|&x, &y| {
        let d = |e| { let m = bb.edge_mid(e); ((m.x - ax).powi(2) + (m.z + ay).powi(2) + (m.y - 4.5).powi(2)).sqrt() };
        d(x).total_cmp(&d(y))
    }).unwrap();
    let fs: Vec<Json> = bb.edge_faces(pick).iter().map(|&x| Json::Str(bb.face_id_str(x))).collect();
    let m = bb.edge_mid(pick);
    let corner = ck_json::parse(&format!(r#"{{"type":"fillet","id":"F90","radius":2,"edges":[{{"faces":{},"at":[{},{},{}]}}]}}"#, ck_json::to_string(&Json::Arr(fs)), m.x, m.y, m.z)).unwrap();
    f.push(corner.clone());
    std::env::set_var("CK_BLEND_INPLACE", "1");
    let (b, e) = regen_body(&f);
    std::env::remove_var("CK_BLEND_INPLACE");
    let (v_in, bad) = report_issues("rotated in place", &b, &e);
    assert!(e.is_empty() && bad == 0, "{e:?}");
    let mut g = base.clone();
    g.push(corner);
    let (b1, e1) = regen_body(&g);
    assert!(e1.is_empty(), "{e1:?}");
    g.push(with_radius(&fillet_all_top(&b1.unwrap(), "F95", 1.0), 1.0));
    let (b2, e2) = regen_body(&g);
    let v_ref = report("rotated reference", &b2, &e2);
    assert!((v_in - v_ref).abs() < 1e-5 * v_ref, "{v_in} vs {v_ref}");
}
