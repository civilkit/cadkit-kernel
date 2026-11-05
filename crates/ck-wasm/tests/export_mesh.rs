//! `exportMesh`: the mesh of the current body at a quality preset and/or an absolute chord height.

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

/// The cylinder part of the query goldens (radius 10, height 10) regenerated on `tag`.
fn load(tag: &str) {
    ck_wasm::engine::install();
    let g = ck_json::parse(include_str!("../../ck-model/tests/golden/queries.json")).unwrap();
    let part = g.get("cyl").unwrap().get("part").unwrap().clone();
    call(&obj(vec![("type", "regen".into()), ("tag", tag.into()), ("part", part)]));
}

struct Out {
    chord: f64,
    angle: f64,
    raised: bool,
    tris: usize,
    positions: Vec<f32>,
    tri_face: Vec<i32>,
    faces: Vec<String>,
}

fn mesh(tag: &str, extra: Vec<(&str, Json)>) -> Result<Out, String> {
    let mut kv = vec![("type", "exportMesh".into()), ("tag", tag.into())];
    kv.extend(extra);
    let (r, bin) = call(&obj(kv));
    if r.get("ok") != Some(&Json::Bool(true)) {
        return Err(r.get_str("error").unwrap_or("?").to_string());
    }
    let res = r.get("result").unwrap();
    let buf = |k: &str| (res.get(k).unwrap().get_f64("offset").unwrap() as usize, res.get(k).unwrap().get_f64("len").unwrap() as usize);
    let (po, pl) = buf("positions");
    let (to, tl) = buf("triFace");
    Ok(Out {
        chord: res.get_f64("chord").unwrap(),
        angle: res.get_f64("angle").unwrap(),
        raised: res.get("chordRaised") == Some(&Json::Bool(true)),
        tris: res.get_f64("triangles").unwrap() as usize,
        positions: (0..pl).map(|i| f32::from_le_bytes(bin[po + 4 * i..po + 4 * i + 4].try_into().unwrap())).collect(),
        tri_face: (0..tl).map(|i| i32::from_le_bytes(bin[to + 4 * i..to + 4 * i + 4].try_into().unwrap())).collect(),
        faces: res.get_arr("faces").unwrap().iter().map(|f| f.as_str().unwrap().to_string()).collect(),
    })
}

/// The largest sag (distance of a facet from the true surface) of the side face of the radius-10 cylinder, from the longest edge of its
/// facets that lies in one ring (both ends at the same height along the axis): a chord of length c on a circle of radius r sags r - sqrt(r^2 - c^2/4).
fn side_sag(m: &Out) -> f64 {
    let side = m.faces.iter().position(|f| f == "F2.S.e7").expect("side face");
    let mut longest: f64 = 0.0;
    for t in 0..m.tris {
        if m.tri_face[t] as usize != side {
            continue;
        }
        let p = |k: usize| [m.positions[t * 9 + k * 3] as f64, m.positions[t * 9 + k * 3 + 1] as f64, m.positions[t * 9 + k * 3 + 2] as f64];
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let (pa, pb) = (p(a), p(b));
            if (pa[1] - pb[1]).abs() < 1e-6 {
                longest = longest.max(((pa[0] - pb[0]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt());
            }
        }
    }
    assert!(longest > 0.0);
    10.0 - (100.0 - longest * longest / 4.0).sqrt()
}

#[test]
fn presets_order_the_triangle_count() {
    load("em1");
    let normal = mesh("em1", vec![]).unwrap();
    let named = mesh("em1", vec![("quality", "normal".into())]).unwrap();
    let coarse = mesh("em1", vec![("quality", "coarse".into())]).unwrap();
    let fine = mesh("em1", vec![("quality", "fine".into())]).unwrap();
    assert_eq!((normal.tris, &normal.positions), (named.tris, &named.positions)); // no option is the normal preset
    assert_eq!(normal.positions.len(), normal.tris * 9);
    assert!(coarse.tris < normal.tris, "coarse {} normal {}", coarse.tris, normal.tris);
    assert!(normal.tris < fine.tris, "normal {} fine {}", normal.tris, fine.tris);
    assert!(coarse.chord > normal.chord && normal.chord > fine.chord);
    assert!(coarse.angle > normal.angle && normal.angle > fine.angle);
    // the sag stays inside the chord the reply states
    for m in [&coarse, &normal, &fine] {
        assert!(side_sag(m) <= m.chord * 1.01, "sag {} chord {}", side_sag(m), m.chord);
    }
}

#[test]
fn absolute_chord_bounds_the_sag() {
    load("em2");
    for c in [0.5, 0.05, 0.005] {
        let m = mesh("em2", vec![("chord_mm", c.into())]).unwrap();
        assert_eq!((m.chord, m.raised), (c, false));
        let s = side_sag(&m);
        assert!(s <= c * 1.01, "chord_mm {c}: sag {s}");
        // and it is used: the facets are not much finer than asked for either
        assert!(s > c * 0.02, "chord_mm {c}: sag {s} is far below the tolerance");
    }
    let a = mesh("em2", vec![("chord_mm", 0.5.into())]).unwrap();
    let b = mesh("em2", vec![("chord_mm", 0.005.into())]).unwrap();
    assert!(a.tris < b.tris);
    // the chord replaces the preset's chord but keeps its angle
    let c = mesh("em2", vec![("quality", "coarse".into()), ("chord_mm", 0.05.into())]).unwrap();
    let coarse = mesh("em2", vec![("quality", "coarse".into())]).unwrap();
    assert_eq!((c.chord, c.angle), (0.05, coarse.angle));
}

#[test]
fn refusals_and_unknown_tag() {
    load("em3");
    for (extra, want) in [
        (vec![("chord_mm", 0.0.into())], "chord_mm"),
        (vec![("chord_mm", (-1.0).into())], "chord_mm"),
        (vec![("chord_mm", "fine".into())], "chord_mm"),
        (vec![("quality", "ultra".into())], "coarse, normal, fine"),
        (vec![("quality", 3.0.into())], "quality"),
    ] {
        let e = mesh("em3", extra).err().expect("refused");
        assert!(e.contains(want), "{e}");
    }
    let tiny = mesh("em3", vec![("chord_mm", 1e-30.into())]).unwrap();
    assert!(tiny.raised && tiny.chord > 1e-30);
    // a tag the session does not hold is refused
    let (r, _) = call(&obj(vec![("type", "exportMesh".into()), ("tag", "em-none".into())]));
    assert_eq!(r.get("ok"), Some(&Json::Bool(false)));
}
