//! Lofts through regular closed-spline sections (n-gon splines of different radii at equal spacing):
//! they must loft (valid solid, volume near the frustum-sum estimate), and every loft failure must carry
//! a detail and a hint.
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

/// Loft through closed n-point splines on a regular n-gon of radius `radii[i]` at `i * spacing`.
fn loft_regular(n: usize, radii: &[f64], spacing: f64, tag: &str) -> (Vec<String>, f64, usize) {
    let mut feats = vec![];
    let mut ids = vec![];
    for (i, r) in radii.iter().enumerate() {
        let (pl, sk) = (format!("P{i}"), format!("S{i}"));
        let plane = if i == 0 { r#"{"kind":"std","id":"front"}"#.to_string() } else { format!(r#"{{"kind":"feature","id":"{pl}"}}"#) };
        if i > 0 {
            feats.push(format!(r#"{{"type":"plane","id":"{pl}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{}}}"#, spacing * i as f64));
        }
        let pts: Vec<String> = (0..n).map(|k| format!(r#"{{"id":"p{k}","x":{},"y":{}}}"#, r * (2.0 * PI * k as f64 / n as f64).cos(), r * (2.0 * PI * k as f64 / n as f64).sin())).collect();
        let pid: Vec<String> = (0..n).map(|k| format!(r#""p{k}""#)).collect();
        feats.push(format!(
            r#"{{"type":"sketch","id":"{sk}","plane":{plane},"points":[{}],"entities":[{{"id":"s","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}}"#,
            pts.join(","),
            pid.join(",")
        ));
        ids.push(format!(r#""{sk}""#));
    }
    feats.push(format!(r#"{{"type":"loft","id":"L","op":"add","sketches":[{}]}}"#, ids.join(",")));
    let part = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    let errors: Vec<String> = res.get_arr("errors").map(|a| a.iter().map(|e| ck_json::to_string(e)).collect()).unwrap_or_default();
    let m = call(&format!(r#"{{"type":"massProps","tag":"{tag}"}}"#));
    let v = call(&format!(r#"{{"type":"validate","tag":"{tag}"}}"#));
    let issues = v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len());
    (errors, m.get("result").map_or(f64::NAN, |m| num(m, "volume")), issues)
}

/// Sum of frustum volumes of the circles of the polygon radii (the spline is close to the circle).
fn frustum_sum(radii: &[f64], spacing: f64) -> f64 {
    radii.windows(2).map(|w| PI * spacing / 3.0 * (w[0] * w[0] + w[0] * w[1] + w[1] * w[1])).sum()
}

#[test]
fn three_decagon_splines_loft() {
    let radii = [10.0, 14.0, 8.0];
    let (errors, vol, issues) = loft_regular(10, &radii, 30.0, "dec");
    assert!(errors.is_empty(), "{errors:?}");
    let want = frustum_sum(&radii, 30.0);
    assert!(vol > 1000.0 && vol > 0.8 * want && vol < 1.2 * want, "volume {vol} vs frustum sum {want}");
    assert_eq!(issues, 0);
}

#[test]
fn regular_splines_of_many_sizes_loft() {
    for (n, radii) in [(6usize, vec![5.0, 9.0]), (8, vec![10.0, 14.0, 8.0]), (12, vec![3.0, 11.0, 6.0, 12.0]), (16, vec![10.0, 14.0, 8.0, 9.0, 13.0])] {
        let (errors, vol, issues) = loft_regular(n, &radii, 30.0, "reg");
        assert!(errors.is_empty(), "n={n} {errors:?}");
        let want = frustum_sum(&radii, 30.0);
        assert!(vol > 0.8 * want && vol < 1.2 * want && issues == 0, "n={n} volume {vol} vs {want}, issues {issues}");
    }
}

/// Seeds of `loft_fuzz --mode regular` that left a repeated clamped end knot (an extra end span of zero length).
#[test]
fn regular_splines_with_repeated_end_knot_loft() {
    let (errors, vol, issues) = loft_regular(6, &[7.90463846269995, 5.314652932807803, 7.629190561361611], 30.0, "rep");
    assert!(errors.is_empty(), "{errors:?}");
    assert!(vol > 0.8 * frustum_sum(&[7.90463846269995, 5.314652932807803, 7.629190561361611], 30.0) && issues == 0, "{vol} {issues}");
}
