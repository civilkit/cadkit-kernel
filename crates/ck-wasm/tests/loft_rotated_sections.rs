//! Loft through identical ellipse sections on planes rotated every 10 degrees about a common axis
//! (#2847): the solid must be the revolved ring, volume = Pappus (section area times the centroid
//! path) within 1e-3 relative, `validate` clean. The orientation decision of the sections used to be
//! taken against the FIRST section, so every station past 90 degrees was reversed and the loft folded
//! (6 % of Pappus at 180) while still validating.
use ck_json::Json;
use std::f64::consts::PI;

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

/// The #2847 part: `total` degrees in 10 degree steps (sections at 0, 10, ..., `total`).
fn rotated_ellipses(total: f64, tag: &str) -> (Vec<String>, f64, usize) {
    let steps = (total / 10.0).round() as usize;
    let mut feats: Vec<String> = vec![];
    let mut ids: Vec<String> = vec![];
    for i in 0..=steps {
        let (pl, sk) = (format!("PL{i}"), format!("SK{i}"));
        if i > 0 {
            feats.push(format!(
                r#"{{"type":"plane","id":"{pl}","mode":"angle","base":{{"kind":"std","id":"top"}},"axis":{{"kind":"axis","axis":"z"}},"angle":{},"visible":false}}"#,
                10 * i
            ));
        }
        let plane = if i == 0 { r#"{"kind":"std","id":"top"}"#.to_string() } else { format!(r#"{{"kind":"feature","id":"{pl}"}}"#) };
        // section 0 draws the ellipse on `top` (centre 30 on x, the p point 5 below); the rotated
        // sections use the local coordinates that land on the same world ellipse turned by i * 10 deg
        let (cx, cy, qx, qy) = if i == 0 { (30.0, 0.0, 30.0, -5.0) } else { (0.0, 30.0, 5.0, 30.0) };
        feats.push(format!(
            r#"{{"type":"sketch","id":"{sk}","plane":{plane},"points":[{{"id":"c","x":{cx},"y":{cy}}},{{"id":"q","x":{qx},"y":{qy}}}],"entities":[{{"id":"e1","type":"ellipse","c":"c","p":"q","r":3}}],"constraints":[]}}"#
        ));
        ids.push(format!(r#""{sk}""#));
    }
    feats.push(format!(r#"{{"type":"loft","id":"LF","op":"add","sketches":[{}]}}"#, ids.join(",")));
    let part = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    let errors: Vec<String> = res.get_arr("errors").map(|a| a.iter().map(|e| ck_json::to_string(e)).collect()).unwrap_or_default();
    let m = call(&format!(r#"{{"type":"massProps","tag":"{tag}"}}"#));
    let v = call(&format!(r#"{{"type":"validate","tag":"{tag}"}}"#));
    let issues = v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len());
    (errors, m.get("result").map_or(f64::NAN, |m| num(m, "volume")), issues)
}

/// Pappus: one ellipse (area pi a b) revolved about Z at distance R from its centroid.
fn pappus(total_deg: f64) -> f64 {
    (total_deg.to_radians()) * 30.0 * (PI * 3.0 * 5.0)
}

#[test]
fn rotated_ellipse_sections_follow_pappus_90_120_180() {
    for total in [90.0, 120.0, 180.0] {
        let tag = format!("re{total}");
        let (errors, vol, issues) = rotated_ellipses(total, &tag);
        assert!(errors.is_empty(), "{total} degrees errors: {errors:?}");
        assert_eq!(issues, 0, "{total} degrees validate issues");
        let want = pappus(total);
        let rel = (vol - want).abs() / want;
        assert!(rel < 1e-3, "{total} degrees: volume {vol} vs Pappus {want} (rel {rel})");
    }
}
