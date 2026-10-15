//! A cross hole through a cylinder whose centre sits on the seam (BenchCAD
//! shaft_collar: a radial set-screw hole at the cylinder reference angle). The
//! dropped seam used to cut the hole loop into two open single-edge bounds; OCCT
//! then integrated the exported solid 15 percent too large. Every EDGE_LOOP of
//! the exported file must be a closed chain and the part must re-import clean.
use ck_json::Json;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn num(j: &Json, k: &str) -> f64 {
    j.get(k).and_then(Json::as_f64).unwrap_or(f64::NAN)
}

fn refs(s: &str) -> Vec<String> {
    s.split('#').skip(1).map(|t| t.chars().take_while(|c| c.is_ascii_digit()).collect()).collect()
}

fn open_loops(text: &str) -> usize {
    let ent = |id: &str| -> &str {
        let key = format!("\n#{id}=");
        let at = text.find(&key).unwrap() + 1;
        text[at..].split(";\n").next().unwrap()
    };
    let mut open = 0;
    for line in text.lines().filter(|l| l.contains("=EDGE_LOOP(")) {
        let mut ends = vec![];
        for oe in refs(line.split("EDGE_LOOP('',").nth(1).unwrap()) {
            let o = ent(&oe);
            let fwd = o.ends_with(".T.)");
            let ec = refs(o.split("ORIENTED_EDGE").nth(1).unwrap())[0].clone();
            let e = refs(ent(&ec).split("EDGE_CURVE('',").nth(1).unwrap());
            ends.push(if fwd { (e[0].clone(), e[1].clone()) } else { (e[1].clone(), e[0].clone()) });
        }
        let n = ends.len();
        if (0..n).any(|i| ends[i].1 != ends[(i + 1) % n].0) {
            open += 1;
        }
    }
    open
}

#[test]
fn hole_straddling_cylinder_seam_exports_closed_loops() {
    let path = format!("{}/../../tools/bench/benchcad/shaft_collar_000553.ckpart.json", env!("CARGO_MANIFEST_DIR"));
    let part = std::fs::read_to_string(path).unwrap();
    let r = call(&format!(r#"{{"type":"regen","tag":"main","part":{part}}}"#));
    assert!(!r.get("result").and_then(|r| r.get("body")).map_or(true, |b| matches!(b, Json::Null)));
    let step = call(r#"{"type":"step","tag":"main","name":"rt"}"#);
    let text = step.get("result").and_then(Json::as_str).unwrap().to_string();
    assert_eq!(open_loops(&text), 0, "open EDGE_LOOPs in the exported STEP");
    let msg = ck_json::to_string(&Json::Obj(vec![
        ("type".into(), Json::Str("importStep".into())),
        ("tag".into(), Json::Str("imp".into())),
        ("name".into(), Json::Str("rt".into())),
        ("data".into(), Json::Str(text)),
    ]));
    let imp = call(&msg);
    assert!(!imp.get("result").and_then(|r| r.get("body")).map_or(true, |b| matches!(b, Json::Null)));
    let (a, b) = (call(r#"{"type":"massProps","tag":"main"}"#), call(r#"{"type":"massProps","tag":"imp"}"#));
    let (a, b) = (a.get("result").unwrap(), b.get("result").unwrap());
    assert!((num(a, "volume") - num(b, "volume")).abs() < 1e-6 * num(a, "volume"));
    assert!((num(a, "area") - num(b, "area")).abs() < 1e-6 * num(a, "area"));
    let v = call(r#"{"type":"validate","tag":"imp"}"#);
    assert!(v.get("result").and_then(|r| r.get_arr("issues")).map_or(false, |i| i.is_empty()), "{:?}", v.get("result"));
}
