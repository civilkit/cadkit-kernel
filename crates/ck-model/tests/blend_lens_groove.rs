//! A drafted lens-shaped groove cut up into the base of a revolved cup (a flat centre, a cone pushed up into the base,
//! a flat standing ring, a corner round and a cylindrical wall). The groove's two walls are cones (circle arcs drafted
//! 10 degrees) that meet at a sharp tip under the pushed-up cone; its outer end breaks out through the corner round and
//! the wall. Each groove edge is rounded on its own: the walls against the groove's ceiling, the standing ring, the
//! wall, the corner round and the pushed-up cone, and the tip between the two walls.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

fn cup(fillet: Option<(&str, &str, [f64; 3], f64)>) -> (Option<ck_topo::body::Body>, Vec<String>) {
    // profile on the top plane (sketch x = radius, sketch y = -z), revolved about the z axis
    let pt = |id: &str, r: f64, z: f64| format!(r#"{{"id":"{id}","x":{r},"y":{}}}"#, -z);
    let pts = [pt("a", 0.0, 9.0), pt("b", 4.0, 9.0), pt("c", 15.0, 0.0), pt("d", 24.0, 0.0), pt("k", 24.0, 9.0), pt("e", 33.0, 9.0), pt("f", 33.0, 45.0), pt("g", 0.0, 45.0)].join(",");
    let profile = format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{pts}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"arc","id":"e4","c":"k","p1":"e","p2":"d"}},{{"type":"line","id":"e5","p1":"e","p2":"f"}},{{"type":"line","id":"e6","p1":"f","p2":"g"}},{{"type":"line","id":"ax","p1":"g","p2":"a"}}],"constraints":[]}}"#
    );
    // the lens: tips (9, 0) and (38, 0), half width 7.5, on the plane z = -1, cut 12 up with a 10 degree draft
    let rr = (14.5f64 * 14.5 + 7.5 * 7.5) / 15.0;
    let h = rr - 7.5;
    let lens = format!(
        r#"{{"type":"sketch","id":"F4","plane":{{"kind":"feature","id":"F3"}},"points":[{{"id":"t1","x":9,"y":0}},{{"id":"t2","x":38,"y":0}},{{"id":"cu","x":23.5,"y":{}}},{{"id":"cl","x":23.5,"y":{h}}}],"entities":[{{"type":"arc","id":"e1","c":"cu","p1":"t2","p2":"t1"}},{{"type":"arc","id":"e2","c":"cl","p1":"t1","p2":"t2"}}],"constraints":[]}}"#,
        -h
    );
    let mut feats = vec![
        profile,
        r#"{"type":"revolve","id":"F2","sketch":"F1","axis":"ax","angle":360}"#.to_string(),
        r#"{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"front"},"offset":-1}"#.to_string(),
        lens,
        r#"{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"blind","depth":12,"draft":10,"flip":true}"#.to_string(),
    ];
    if let Some((fa, fb, at, r)) = fillet {
        feats.push(format!(r#"{{"type":"fillet","id":"F6","radius":{r},"edges":[{{"faces":["{fa}","{fb}"],"at":[{},{},{}]}}]}}"#, at[0], at[1], at[2]));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

/// Rounds one edge; `Ok(volume change)` for a valid result.
fn round(fa: &str, fb: &str, at: [f64; 3], r: f64) -> Result<f64, String> {
    let (bare, e0) = cup(None);
    assert!(e0.is_empty(), "{e0:?}");
    let v0 = ck_topo::mass::mass_props(&bare.unwrap()).volume;
    let (b, errs) = cup(Some((fa, fb, at, r)));
    if !errs.is_empty() {
        return Err(errs.join("; ").chars().take(200).collect());
    }
    let b = b.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    if !issues.is_empty() {
        return Err(format!("invalid {:?}", issues.iter().map(|i| i.code).collect::<Vec<_>>()));
    }
    Ok(ck_topo::mass::mass_props(&b).volume - v0)
}

#[test]
fn cup_faces() {
    let (b, e) = cup(None);
    assert!(e.is_empty(), "{e:?}");
    let b = b.unwrap();
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    for f in b.face_ids() {
        eprintln!("{} {}", b.face_id_str(f), b.face(f).surface.kind());
    }
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        let ed = b.edge(e);
        eprintln!("edge {} {:?} {:?} {:?}", ed.curve.kind(), fs.iter().map(|&f| b.face_id_str(f)).collect::<Vec<_>>(), b.vertex(ed.v0).p, b.vertex(ed.v1).p);
    }
}

/// Each groove edge of the upper arc's wall (F5.S.e1) rounded alone, radius 0.5 and 1: the result per edge
/// (volume change, or the refusal).
#[test]
fn groove_edges_one_by_one() {
    // (which builds: the ceiling and the standing ring, each ending obliquely on curved faces; the others are listed
    // with their refusals: the wall against the outer wall, the corner round and the pushed-up cone are free-form
    // edges whose tool does not assemble at its ends, and the sharp tip ends on a cone, where a concave free-form
    // blend cannot end)
    let must: [&str; 2] = ["ceiling", "standing ring"];
    let cases: [(&str, &str, &str, [f64; 3]); 6] = [
        ("ceiling (plane x cone, concave)", "F5.S.e1", "F5.C1", [23.5, 5.38, 11.0]),
        ("standing ring (cone x plane)", "F5.S.e1", "F2.S.e3", [18.0, 6.4, 0.0]),
        ("outer wall (cone x cylinder)", "F5.S.e1", "F2.S.e5", [32.9, 2.5, 10.0]),
        ("corner round (cone x torus)", "F5.S.e1", "F2.S.e4", [28.0, 5.5, 2.0]),
        ("pushed-up cone (cone x cone)", "F5.S.e1", "F2.S.e2", [12.0, 2.5, 2.5]),
        ("tip (cone x cone, concave)", "F5.S.e1", "F5.S.e2", [11.0, 0.0, 8.0]),
    ];
    let mut rows = vec![];
    // CK_LENS_ONLY=<text>: only the cases whose description contains it (for a debug run)
    let only = std::env::var("CK_LENS_ONLY").ok();
    for (what, fa, fb, at) in cases {
        if only.as_ref().is_some_and(|o| !what.contains(o.as_str())) {
            continue;
        }
        for r in [0.5, 1.0] {
            let t = std::time::Instant::now();
            let res = round(fa, fb, at, r);
            if must.iter().any(|m| what.starts_with(m)) {
                let dv = res.clone().unwrap_or_else(|e| panic!("{what} r {r}: {e}"));
                // the round adds (ceiling, concave) or removes (ring, convex) a sliver of about its section along the
                // arc: between a quarter and twice that of a square corner (0.215 r^2) along 10 to 30 mm
                let (lo, hi) = (0.25 * 0.215 * r * r * 10.0, 2.0 * 0.215 * r * r * 30.0);
                assert!(dv.abs() > lo && dv.abs() < hi && (dv > 0.0) == what.starts_with("ceiling"), "{what} r {r}: volume change {dv}");
            }
            rows.push(format!("{what:34} r {r}: {} ({:.1} s)", match &res { Ok(dv) => format!("ok, volume change {dv:.4}"), Err(e) => e.clone() }, t.elapsed().as_secs_f64()));
        }
    }
    eprintln!("{}", rows.join("\n"));
}
