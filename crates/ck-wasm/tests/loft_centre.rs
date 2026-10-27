//! Loft `centreLine` through the engine (internal notes (not published) "loft centre line", crates/ck-ops/src/loft/centre.rs):
//! the loft follows a curved centre line (a moving frame carries the sections). Volumes and boxes are the
//! oracle's (tools/loft_centre_check.mjs, 1e-3), the result is a valid solid (`validate` with the mesh check),
//! survives a STEP round trip, regenerates identically with the cache on and off, and the straight / refused
//! cases behave: a straight centre line changes nothing, the others carry a code, a detail and a hint.
use ck_json::Json;

fn raw(msg: &str) -> Vec<u8> {
    ck_wasm::engine::install();
    ck_server::handle(format!(r#"{{"id":1,"msg":{msg}}}"#).as_bytes())
}

fn parse(r: &[u8]) -> Json {
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn call(msg: &str) -> Json {
    parse(&raw(msg))
}

fn num(j: &Json, k: &str) -> f64 {
    j.get(k).and_then(Json::as_f64).unwrap_or(f64::NAN)
}

// ------------------------------------------------------------------ parts

fn poly(h: f64, cx: f64) -> (String, String) {
    let c = [(cx - h, -h), (cx + h, -h), (cx + h, h), (cx - h, h)];
    let pts: Vec<String> = c.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let ents: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, (i + 1) % 4)).collect();
    (pts.join(","), ents.join(","))
}

/// A profile sketch on the plane `y`: a square (half size `h`, centre x `cx`), a circle, or a point.
enum Prof {
    Sq(f64, f64),
    Circle(f64),
    Point,
}

fn profile_feats(out: &mut Vec<String>, n: &mut usize, y: f64, p: &Prof) -> String {
    let pl = if y == 0.0 {
        r#"{"kind":"std","id":"top"}"#.to_string()
    } else {
        *n += 1;
        let id = format!("P{n}");
        out.push(format!(r#"{{"type":"plane","id":"{id}","mode":"offset","base":{{"kind":"std","id":"top"}},"offset":{y}}}"#));
        format!(r#"{{"kind":"feature","id":"{id}"}}"#)
    };
    *n += 1;
    let id = format!("S{n}");
    let (pts, ents) = match p {
        Prof::Sq(h, cx) => poly(*h, *cx),
        Prof::Circle(r) => (r#"{"id":"c","x":0,"y":0}"#.to_string(), format!(r#"{{"type":"circle","id":"e1","c":"c","r":{r}}}"#)),
        Prof::Point => (r#"{"id":"p","x":0,"y":0}"#.to_string(), r#"{"type":"point","id":"e1","p":"p"}"#.to_string()),
    };
    out.push(format!(r#"{{"type":"sketch","id":"{id}","plane":{pl},"points":[{pts}],"entities":[{ents}],"constraints":[]}}"#));
    id
}

/// Centre line sketches on the front plane (sketch x -> world x, sketch y -> world y, the loft axis).
fn arc_sketch(id: &str, bow: f64, y0: f64, y1: f64) -> String {
    let (mid, h) = (0.5 * (y0 + y1), 0.5 * (y1 - y0));
    let r = (bow * bow + h * h) / (2.0 * bow.abs());
    let cx = bow.signum() * (bow.abs() - r);
    let (p1, p2) = if bow > 0.0 { ((0.0, y0), (0.0, y1)) } else { ((0.0, y1), (0.0, y0)) };
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"g0","x":{cx},"y":{mid}}},{{"id":"g1","x":{},"y":{}}},{{"id":"g2","x":{},"y":{}}}],"entities":[{{"type":"arc","id":"e1","c":"g0","p1":"g1","p2":"g2"}}],"constraints":[]}}"#,
        p1.0, p1.1, p2.0, p2.1
    )
}

fn line_sketch(id: &str, a: (f64, f64), b: (f64, f64)) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"g1","x":{},"y":{}}},{{"id":"g2","x":{},"y":{}}}],"entities":[{{"type":"line","id":"e1","p1":"g1","p2":"g2"}}],"constraints":[]}}"#,
        a.0, a.1, b.0, b.1
    )
}

fn corner_sketch(id: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"g1","x":0,"y":0}},{{"id":"g2","x":6,"y":15}},{{"id":"g3","x":0,"y":30}}],"entities":[{{"type":"line","id":"e1","p1":"g1","p2":"g2"}},{{"type":"line","id":"e2","p1":"g2","p2":"g3"}}],"constraints":[]}}"#
    )
}

fn closed_sketch(id: &str) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":0,"y":15}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":15}}],"constraints":[]}}"#)
}

/// `profiles`: (y, profile); `centre`: the centre line sketch JSON (id "CL") or None; `extra`: more loft fields.
fn part(profiles: &[(f64, Prof)], centre: Option<String>, extra: &str) -> String {
    let (mut feats, mut n, mut ids) = (vec![], 0usize, vec![]);
    for (y, p) in profiles {
        ids.push(format!("\"{}\"", profile_feats(&mut feats, &mut n, *y, p)));
    }
    let mut cl = String::new();
    if let Some(c) = centre {
        feats.push(c);
        cl = r#","centreLine":{"sketch":"CL"}"#.to_string();
    }
    feats.push(format!(r#"{{"type":"loft","id":"L1","op":"add","sketches":[{}]{cl}{extra}}}"#, ids.join(",")));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","))
}

fn regen(part: &str, opts: &str, tag: &str) -> Vec<u8> {
    raw(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part},"opts":{opts}}}"#))
}

fn errors(r: &Json) -> Vec<(String, Json)> {
    r.get("result")
        .and_then(|r| r.get_arr("errors"))
        .map(|a| a.iter().map(|e| (e.as_arr().unwrap()[1].get_str("code").unwrap().to_string(), e.as_arr().unwrap()[1].get("params").cloned().unwrap_or(Json::Null))).collect())
        .unwrap_or_default()
}

struct Built {
    errors: Vec<(String, Json)>,
    volume: f64,
    bbox: Vec<f64>,
    faces: usize,
    edges: usize,
    issues: usize,
}

fn build(part: &str, tag: &str) -> Built {
    let r = parse(&regen(part, r#"{"solveSketches":true}"#, tag));
    let body = r.get("result").and_then(|r| r.get("body")).filter(|b| !matches!(b, Json::Null));
    let corner = |k: &str| body.and_then(|b| b.get("box")).and_then(|b| b.get_arr(k)).map(|a| a.iter().map(|x| x.as_f64().unwrap()).collect::<Vec<f64>>()).unwrap_or_default();
    let bbox: Vec<f64> = corner("min").into_iter().chain(corner("max")).collect();
    let (faces, edges) = body.map_or((0, 0), |b| (b.get_arr("faces").map_or(0, |a| a.len()), b.get_arr("edges").map_or(0, |a| a.len())));
    let m = call(&format!(r#"{{"type":"massProps","tag":"{tag}"}}"#));
    let v = call(&format!(r#"{{"type":"validate","tag":"{tag}"}}"#));
    Built {
        errors: errors(&r),
        volume: m.get("result").map_or(f64::NAN, |m| num(m, "volume")),
        bbox,
        faces,
        edges,
        issues: v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len()),
    }
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs()
}

const S2: fn() -> Vec<(f64, Prof)> = || vec![(0.0, Prof::Sq(10.0, 0.0)), (30.0, Prof::Sq(5.0, 0.0))];

// ------------------------------------------------------------------ tests

/// Oracle numbers: sq10 -> sq5 on an arc bow 5 (R 25): volume 6006.0109, x max 11.592 (bow 2: 6835.2412 / 10.165).
#[test]
fn arc_centre_line_volume_box_validity() {
    for (bow, vol, xmax) in [(5.0, 6006.0109, 11.592), (2.0, 6835.2412, 10.165), (-5.0, 6006.0109, 10.0)] {
        let b = build(&part(&S2(), Some(arc_sketch("CL", bow, 0.0, 30.0)), ""), "cl");
        assert!(b.errors.is_empty(), "bow {bow}: {:?}", b.errors);
        assert!(close(b.volume, vol, 1e-3), "bow {bow}: {} vs {vol}", b.volume);
        assert_eq!(b.issues, 0, "bow {bow}: validate (mesh check on)");
        assert_eq!((b.faces, b.edges), (6, 12), "bow {bow}: the faces and edges of the plain loft");
        // box = [minx, miny, minz, maxx, maxy, maxz]: the caps stay at y = 0 and y = 30
        if bow > 0.0 {
            assert!((b.bbox[3] - xmax).abs() < 0.03, "bow {bow}: x max {} vs {xmax}", b.bbox[3]);
        } else {
            assert!((b.bbox[0] + 11.592).abs() < 0.03, "bow {bow}: x min {}", b.bbox[0]);
        }
        assert!(b.bbox[1].abs() < 0.02 && (b.bbox[4] - 30.0).abs() < 0.02, "bow {bow}: y range {:?}", b.bbox);
    }
}

/// Three profiles, a circle section, a surface loft and a spine longer than the span.
#[test]
fn more_profiles_and_shapes() {
    let p3 = vec![(0.0, Prof::Sq(10.0, 0.0)), (30.0, Prof::Sq(8.0, 0.0)), (60.0, Prof::Sq(5.0, 0.0))];
    let b = build(&part(&p3, Some(arc_sketch("CL", 10.0, 0.0, 60.0)), ""), "cl3");
    assert!(b.errors.is_empty() && close(b.volume, 14929.5907, 1e-3) && b.issues == 0, "{:?} {} {}", b.errors, b.volume, b.issues);
    let circ = vec![(0.0, Prof::Circle(8.0)), (30.0, Prof::Circle(8.0))];
    let b = build(&part(&circ, Some(arc_sketch("CL", 5.0, 0.0, 30.0)), ""), "clc");
    assert!(b.errors.is_empty() && close(b.volume, 5175.3435, 1e-3) && b.issues == 0, "{:?} {} {}", b.errors, b.volume, b.issues);
    let b = build(&part(&S2(), Some(arc_sketch("CL", 5.0, -10.0, 40.0)), ""), "cll");
    assert!(b.errors.is_empty() && close(b.volume, 6872.471, 1e-3) && b.issues == 0, "{:?} {} {}", b.errors, b.volume, b.issues);
    // second profile off the spine: the profile stays where it is
    let off = vec![(0.0, Prof::Sq(10.0, 0.0)), (30.0, Prof::Sq(5.0, 8.0))];
    let b = build(&part(&off, Some(arc_sketch("CL", 5.0, 0.0, 30.0)), ""), "clo");
    assert!(b.errors.is_empty() && b.issues == 0, "{:?} {}", b.errors, b.issues);
}

/// A straight centre line (also slanted, also longer than the span) changes nothing: the plain loft's numbers.
#[test]
fn straight_centre_line_is_the_plain_loft() {
    let plain = build(&part(&S2(), None, ""), "plain");
    assert!(plain.errors.is_empty() && close(plain.volume, 7000.0, 1e-9));
    for (a, b) in [((0.0, 0.0), (0.0, 30.0)), ((0.0, 0.0), (8.0, 30.0)), ((0.0, -10.0), (0.0, 40.0))] {
        let s = build(&part(&S2(), Some(line_sketch("CL", a, b)), ""), "straight");
        assert!(s.errors.is_empty(), "{a:?}->{b:?}: {:?}", s.errors);
        assert_eq!((s.volume, s.faces, s.edges, s.bbox.clone()), (plain.volume, plain.faces, plain.edges, plain.bbox.clone()));
    }
}

/// STEP round trip of the bent loft: same volume, valid on import.
#[test]
fn step_round_trip() {
    let p = part(&S2(), Some(arc_sketch("CL", 5.0, 0.0, 30.0)), "");
    let b = build(&p, "rt");
    assert!(b.errors.is_empty());
    let st = call(r#"{"type":"step","tag":"rt","name":"bent"}"#);
    let text = st.get("result").and_then(Json::as_str).expect("step text").to_string();
    let msg = ck_json::to_string(&Json::Obj(vec![
        ("type".into(), Json::Str("importStep".into())),
        ("tag".into(), Json::Str("rtimp".into())),
        ("name".into(), Json::Str("bent".into())),
        ("data".into(), Json::Str(text)),
    ]));
    let imp = call(&msg);
    assert!(imp.get("result").and_then(|r| r.get("body")).map_or(false, |b| !matches!(b, Json::Null)), "import failed {:?}", imp.get("result"));
    let m = call(r#"{"type":"massProps","tag":"rtimp"}"#);
    assert!(close(num(m.get("result").unwrap(), "volume"), b.volume, 1e-3), "{} vs {}", num(m.get("result").unwrap(), "volume"), b.volume);
    let v = call(r#"{"type":"validate","tag":"rtimp"}"#);
    assert_eq!(v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len()), 0);
}

/// Cache on and off give the same reply; an edit of the centre line with a warm cache equals a cold regen.
#[test]
fn regen_cache_on_off_identical() {
    std::thread::spawn(|| {
        let a = part(&S2(), Some(arc_sketch("CL", 5.0, 0.0, 30.0)), "");
        let cold = regen(&a, r#"{"noCache":true}"#, "main");
        assert_eq!(cold, regen(&a, "{}", "main"), "cache on vs off");
        assert_eq!(cold, regen(&a, "{}", "main"), "warm full hit");
        let e = part(&S2(), Some(arc_sketch("CL", 6.0, 0.0, 30.0)), "");
        let warm = regen(&e, "{}", "main");
        let fresh = std::thread::spawn(move || regen(&e, r#"{"noCache":true}"#, "main")).join().unwrap();
        assert_eq!(warm, fresh, "edit of the centre line, warm vs cold");
        assert_ne!(warm, cold);
    })
    .join()
    .unwrap();
}

fn code_of(p: &str) -> (String, Json) {
    let r = parse(&regen(p, r#"{"solveSketches":true}"#, "main"));
    let e = errors(&r);
    assert_eq!(e.len(), 1, "{e:?}");
    e.into_iter().next().unwrap()
}

/// Error paths: every refusal has a code and a hint; none is silent.
#[test]
fn error_paths() {
    let hint = |p: &Json| p.get_str("hint").map(|s| !s.is_empty()).unwrap_or(false);
    // a corner between two entities
    let (c, p) = code_of(&part(&S2(), Some(corner_sketch("CL")), ""));
    assert_eq!(c, "loftCentreLineKink");
    assert!(hint(&p));
    // stops 15 short of the last profile (reach is the circumradius 7.07)
    let (c, p) = code_of(&part(&S2(), Some(line_sketch("CL", (0.0, 0.0), (0.0, 15.0))), ""));
    assert_eq!(c, "loftCentreLineReach");
    assert!(hint(&p) && p.get_str("detail").is_some());
    let (c, _) = code_of(&part(&S2(), Some(arc_sketch("CL", 2.0, 0.0, 15.0)), ""));
    assert_eq!(c, "loftCentreLineReach");
    // 40 wide sections on a 4 mm span bent on a radius of about 5: sections would cross
    let wide = vec![(0.0, Prof::Sq(20.0, 0.0)), (4.0, Prof::Sq(20.0, 0.0))];
    let (c, p) = code_of(&part(&wide, Some(arc_sketch("CL", 5.0, -2.0, 6.0)), ""));
    assert_eq!(c, "loftCentreLineTight");
    assert!(hint(&p) && p.get_str("detail").is_some());
    // a closed sketch is not a centre line
    let (c, p) = code_of(&part(&S2(), Some(closed_sketch("CL")), ""));
    assert_eq!(c, "loftCentreLineInvalid");
    assert!(hint(&p));
    // a missing sketch
    let missing = part(&S2(), None, r#","centreLine":{"sketch":"nope"}"#);
    assert_eq!(code_of(&missing).0, "missingSketch");
    // a point (apex) end section with a curved centre line
    let apex = vec![(0.0, Prof::Sq(10.0, 0.0)), (30.0, Prof::Point)];
    let (c, p) = code_of(&part(&apex, Some(arc_sketch("CL", 5.0, 0.0, 30.0)), ""));
    assert_eq!(c, "loftCentreLineApex");
    assert!(hint(&p));
    // ... but the straight centre line keeps the apex loft
    let r = build(&part(&apex, Some(line_sketch("CL", (0.0, 0.0), (0.0, 30.0))), ""), "apexstraight");
    assert!(r.errors.is_empty() && close(r.volume, 100.0 * 4.0 * 30.0 / 3.0, 1e-6), "{:?} {}", r.errors, r.volume);
    // guides with a curved centre line
    let mut p = part(&S2(), Some(arc_sketch("CL", 5.0, 0.0, 30.0)), "");
    p = p.replacen(
        r#""centreLine""#,
        r#""guides":[{"sketch":"G1"}],"centreLine""#,
        1,
    );
    p = p.replacen(r#"{"type":"loft""#, &format!(r#"{},{{"type":"loft""#, line_sketch("G1", (10.0, 0.0), (5.0, 30.0))), 1);
    assert_eq!(code_of(&p).0, "loftCentreLineGuides");
}
