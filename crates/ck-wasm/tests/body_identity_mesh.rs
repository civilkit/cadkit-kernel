//! Body identity on the wire: `massProps {bodies:true}` entries carry `name` and `index`; the regen reply's `body.faceBody`
//! and `exportMesh`'s `faceBody` / `triBody` give the `index` of the body each face (triangle) belongs to
//! (docs/spec-agent-interface.md "Body identity").

use ck_json::Json;

fn call(msg: &Json) -> (Json, Vec<u8>) {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg.clone())]));
    let r = ck_server::handle(body.as_bytes());
    let (j, bin) = ck_server::reply::decode(&r).unwrap();
    (ck_json::parse(j).unwrap(), bin.to_vec())
}

fn obj(kv: Vec<(&str, Json)>) -> Json {
    Json::Obj(kv.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

fn cube(id: u32, x0: f64, extra: &str) -> String {
    let (a, b) = (x0, x0 + 10.0);
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{a},"y":0}},{{"id":"p2","x":{b},"y":0}},{{"id":"p3","x":{b},"y":10}},{{"id":"p4","x":{a},"y":10}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
         {{"type":"extrude","id":"F{id}","op":"newBody","sketch":"S{id}","end":"blind","depth":10{extra}}}"#
    )
}

fn regen(tag: &str, features: &str) -> Json {
    let part = ck_json::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)).unwrap();
    ck_wasm::engine::install();
    let (r, _) = call(&obj(vec![("type", "regen".into()), ("tag", tag.into()), ("part", part)]));
    r.get("result").unwrap().clone()
}

/// (name, index) of each `massProps {bodies:true}` entry.
fn bodies(tag: &str) -> Vec<(Option<String>, i64)> {
    let (r, _) = call(&obj(vec![("type", "massProps".into()), ("tag", tag.into()), ("bodies", Json::Bool(true))]));
    r.get("result").unwrap().get_arr("bodies").unwrap().iter().map(|b| (b.get_str("name").map(String::from), b.get_f64("index").unwrap() as i64)).collect()
}

fn i32s(bin: &[u8], r: &Json, k: &str) -> Vec<i32> {
    let (o, n) = (r.get(k).unwrap().get_f64("offset").unwrap() as usize, r.get(k).unwrap().get_f64("len").unwrap() as usize);
    (0..n).map(|i| i32::from_le_bytes(bin[o + 4 * i..o + 4 * i + 4].try_into().unwrap())).collect()
}

/// Three disjoint newBody cubes (A, B, C), an unnamed cube D, an intersect insert (A with an envelope cube that covers half
/// of it, named Insert, the envelope consumed) and a moveBody copy of C named Cc.
fn doc() -> String {
    [
        cube(2, 0.0, r#","body":"A""#),
        cube(4, 30.0, r#","body":"B""#),
        cube(6, 60.0, r#","body":"C""#),
        cube(8, 90.0, ""),
        cube(10, 5.0, r#","body":"Env""#),
        r#"{"type":"combine","id":"I","op":"intersect","target":{"body":"A"},"tools":[{"body":"Env"}],"body":"Insert"}"#.to_string(),
        r#"{"type":"moveBody","id":"M","bodies":[{"body":"C"}],"copy":true,"body":"Cc","translate":[0,30,0]}"#.to_string(),
    ]
    .join(",")
}

#[test]
fn names_indexes_and_face_bodies_agree() {
    let r = regen("bi1", &doc());
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    let bs = bodies("bi1");
    let names: Vec<Option<&str>> = bs.iter().map(|b| b.0.as_deref()).collect();
    assert_eq!(bs.iter().map(|b| b.1).collect::<Vec<_>>(), (0..bs.len() as i64).collect::<Vec<_>>());
    for n in ["B", "C", "Cc", "Insert"] {
        assert_eq!(names.iter().filter(|x| **x == Some(n)).count(), 1, "{names:?}");
    }
    assert!(names.contains(&None), "the unnamed cube D: {names:?}");
    assert!(!names.contains(&Some("A")) && !names.contains(&Some("Env")), "A became Insert and Env was consumed: {names:?}");
    let body = r.get("body").unwrap();
    let faces = body.get_arr("faces").unwrap();
    let fb: Vec<i64> = body.get_arr("faceBody").unwrap().iter().map(|x| x.as_f64().unwrap() as i64).collect();
    assert_eq!(fb.len(), faces.len());
    // the same index the faces' own `lump` carries
    for (f, &i) in faces.iter().zip(&fb) {
        assert_eq!(f.get_f64("lump").map(|l| l as i64), Some(i));
    }
    // each body's faces by name: B is the cube of F4, C of F6, Cc its copy (names end @M), D the cube of F8
    let ix = |n: Option<&str>| bs.iter().find(|b| b.0.as_deref() == n).unwrap().1;
    let of = |i: i64| -> Vec<String> { faces.iter().zip(&fb).filter(|(_, &b)| b == i).map(|(f, _)| f.get_str("id").unwrap().to_string()).collect() };
    assert!(of(ix(Some("B"))).iter().all(|f| f.starts_with("F4.") && !f.contains('@')), "{:?}", of(ix(Some("B"))));
    assert!(of(ix(Some("C"))).iter().all(|f| f.starts_with("F6.") && !f.contains('@')), "{:?}", of(ix(Some("C"))));
    assert!(of(ix(Some("Cc"))).iter().all(|f| f.starts_with("F6.") && f.ends_with("@M.1")), "{:?}", of(ix(Some("Cc"))));
    assert!(of(ix(None)).iter().all(|f| f.starts_with("F8.")), "{:?}", of(ix(None)));
    assert_eq!(of(ix(Some("B"))).len(), 6);
    // the insert took its faces from both the plate and the envelope: its faces are none of the others'
    assert_eq!(of(ix(Some("Insert"))).len(), 6);
    assert_eq!(fb.len(), 6 * bs.len());
}

#[test]
fn export_mesh_tri_body_follows_face_body() {
    let r = regen("bi2", &doc());
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0));
    let (m, bin) = call(&obj(vec![("type", "exportMesh".into()), ("tag", "bi2".into())]));
    let m = m.get("result").unwrap();
    let fb: Vec<i32> = m.get_arr("faceBody").unwrap().iter().map(|x| x.as_f64().unwrap() as i32).collect();
    assert_eq!(fb.len(), m.get_arr("faces").unwrap().len());
    assert_eq!(fb.as_slice(), r.get("body").unwrap().get_arr("faceBody").unwrap().iter().map(|x| x.as_f64().unwrap() as i32).collect::<Vec<_>>().as_slice());
    let tf = i32s(&bin, m, "triFace");
    let tb = i32s(&bin, m, "triBody");
    assert_eq!(tf.len(), tb.len());
    assert!(!tf.is_empty());
    for (f, b) in tf.iter().zip(&tb) {
        assert_eq!(fb[*f as usize], *b);
    }
}

#[test]
fn multi_lump_union_and_surface_bodies() {
    // a union of disjoint cubes is one body of three lumps; every face belongs to one of them
    let u = format!(
        "{},{},{},{}",
        cube(2, 0.0, r#","body":"A""#),
        cube(4, 30.0, r#","body":"B""#),
        cube(6, 60.0, r#","body":"C""#),
        r#"{"type":"combine","id":"U","op":"union","target":{"body":"A"},"tools":[{"body":"B"},{"body":"C"}],"body":"Trio","disjoint":"join"}"#
    );
    let r = regen("bi3", &u);
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    assert_eq!(bodies("bi3"), vec![(Some("Trio".into()), 0), (Some("Trio".into()), 1), (Some("Trio".into()), 2)]);
    let fb: Vec<i64> = r.get("body").unwrap().get_arr("faceBody").unwrap().iter().map(|x| x.as_f64().unwrap() as i64).collect();
    for i in 0..3 {
        assert_eq!(fb.iter().filter(|&&b| b == i).count(), 6);
    }
    // a sheet body: faces of a surface body in the document are lumps like any other
    let s = format!(
        "{},{}",
        cube(2, 0.0, r#","body":"A""#),
        r#"{"type":"sketch","id":"SS","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":40,"y":0},{"id":"p2","x":50,"y":0},{"id":"p3","x":50,"y":10},{"id":"p4","x":40,"y":10}],
          "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
         {"type":"surfaceExtrude","id":"SE","sketch":"SS","end":"blind","depth":10}"#
    );
    let r = regen("bi4", &s);
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    let bs = bodies("bi4");
    let body = r.get("body").unwrap();
    let fb: Vec<i64> = body.get_arr("faceBody").unwrap().iter().map(|x| x.as_f64().unwrap() as i64).collect();
    // every face of the main body maps to a listed body; surfaces answered separately carry their own faceBody
    assert!(fb.iter().all(|&i| (0..bs.len() as i64).contains(&i)), "{fb:?} vs {bs:?}");
    for sj in r.get_arr("surfaces").unwrap_or(&[]) {
        let b = &sj.as_arr().unwrap()[1];
        assert_eq!(b.get_arr("faceBody").unwrap().len(), b.get_arr("faces").unwrap().len());
        assert!(b.get_arr("bodyNames").is_some());
    }
}

/// `bodyNames` (index -> name, null when unnamed) has the order and indexes of `massProps` `bodies[]`.
fn names_of(j: &Json) -> Vec<Option<String>> {
    j.get_arr("bodyNames").unwrap().iter().map(|x| x.as_str().map(|s| s.to_string())).collect()
}

#[test]
fn body_names_follow_mass_props() {
    // three disjoint newBody cubes, an unnamed cube, an intersect insert and a moveBody copy (the shared doc)
    let r = regen("bn1", &doc());
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    let bs = bodies("bn1");
    let want: Vec<Option<String>> = bs.iter().map(|b| b.0.clone()).collect();
    let got = names_of(r.get("body").unwrap());
    assert_eq!(got, want);
    assert!(got.contains(&None) && got.contains(&Some("Insert".into())) && got.contains(&Some("Cc".into())));
    for n in ["B", "C"] {
        assert!(got.contains(&Some(n.into())), "{got:?}");
    }
    // faceBody indexes name a body
    for i in r.get("body").unwrap().get_arr("faceBody").unwrap() {
        assert!((i.as_f64().unwrap() as usize) < got.len());
    }
    // a joined union: three lumps, all named Trio
    let u = format!(
        "{},{},{},{}",
        cube(2, 0.0, r#","body":"A""#),
        cube(4, 30.0, r#","body":"B""#),
        cube(6, 60.0, r#","body":"C""#),
        r#"{"type":"combine","id":"U","op":"union","target":{"body":"A"},"tools":[{"body":"B"},{"body":"C"}],"body":"Trio","disjoint":"join"}"#
    );
    let r = regen("bn2", &u);
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    assert_eq!(names_of(r.get("body").unwrap()), vec![Some("Trio".to_string()); 3]);
    // three disjoint named cubes
    let t = format!("{},{},{}", cube(2, 0.0, r#","body":"P""#), cube(4, 30.0, r#","body":"Q""#), cube(6, 60.0, r#","body":"R""#));
    let r = regen("bn3", &t);
    assert_eq!(r.get_arr("errors").map(|e| e.len()), Some(0), "{:?}", r.get("errors"));
    let want: Vec<Option<String>> = bodies("bn3").iter().map(|b| b.0.clone()).collect();
    assert_eq!(names_of(r.get("body").unwrap()), want);
    assert_eq!(want.len(), 3);
}
