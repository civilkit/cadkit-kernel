//! A chamfer that leaves a trim far off its cone (the body is invalid, `EDGE_OFF_SURFACE`) used to be meshed with ~23,000
//! triangles on that one face (refinement never came within the tolerance and spent its whole budget), and `validate`
//! then spent about a second on the exact area of the same face. The mesh stays small and the reply stays complete.
use ck_json::Json;

fn call(msg: &str) -> (Json, usize) {
    ck_wasm::engine::install();
    let r = ck_server::handle(format!(r#"{{"id":1,"msg":{msg}}}"#).as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    (ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap(), r.len())
}

/// The same checks on a body that a LEGAL blend leaves invalid (solidcraft kernel case `1080-fillet-sloped-basin-1-mm-rim-r4-2-edges`, a
/// 100 x 60 x 30 slab with a lofted basin, two top rim edges rounded 4; the basin as our own STEP restatement, see THIRD_PARTY.md): the round's
/// tangent edge `F2.TA1~2` lies 2e-3 off its surface (`EDGE_OFF_SURFACE`), no feature error. The chamfer that used to break the cone body above
/// is refused now (below), so this is the body that keeps the guard exercised: small mesh, complete reply, still reported invalid.
#[test]
fn a_trim_far_off_its_surface_does_not_exhaust_the_refinement_budget() {
    // (the basin's four walls nudged 1e-5 off flat: STEP analytic recognition reads a flat spline wall as a plane, and the round on planar walls
    // takes the analytic path, which refuses it; with the walls left splines the round builds the invalid body this guard is about)
    let part = include_str!("fixtures/round_off_surface_basin_nudged.json");
    let (r, bytes) = call(&format!(r#"{{"type":"regen","tag":"main","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    assert!(res.get_arr("errors").is_some_and(|e| e.is_empty()), "the round builds");
    let tris = res.get("body").and_then(|b| b.get("mesh")).and_then(|m| m.get("triFace")).and_then(|t| t.get_f64("len")).expect("mesh");
    assert!(tris < 8000.0, "{tris} triangles");
    assert!(bytes < 1_500_000, "{bytes} bytes");
    let (v, _) = call(r#"{"type":"validate","tag":"main"}"#);
    assert!(v.get("result").and_then(|r| r.get_arr("issues")).is_some_and(|i| !i.is_empty()), "an invalid body is still reported invalid");
}

/// The original fixture: a chamfer of 28.2 across a disc of radius 15.5 runs past its axis. It used to build a body with its trim far off its
/// cone; it is refused now, with the wider-than-its-face sentence.
#[test]
fn a_chamfer_wider_than_the_disc_it_cuts_is_refused_not_built_invalid() {
    let part = include_str!("fixtures/chamfer_off_surface.json");
    let (r, _) = call(&format!(r#"{{"type":"regen","tag":"main","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let errs = r.get("result").and_then(|r| r.get_arr("errors")).expect("errors");
    let text = ck_json::to_string(&ck_json::Json::Arr(errs.to_vec()));
    assert!(text.contains("wider than the face beside its edge") && text.contains("F4"), "{text}");
}

/// The same basin with exactly flat walls is read with planar walls (STEP analytic recognition): the analytic round used to refuse it, with
/// the radius that builds, because its contact runs off the rim's face over the far edge. A round that runs off a face is built now (on the
/// planes continued, cut from the body): no error, a valid body, and the feature warns `blendRanOffFace`.
#[test]
fn the_flat_walled_basin_round_runs_off_its_face_and_builds_valid() {
    let part = include_str!("fixtures/round_off_surface_basin.json");
    let (r, _) = call(&format!(r#"{{"type":"regen","tag":"main","part":{part},"opts":{{"solveSketches":true}}}}"#));
    let res = r.get("result").unwrap();
    assert!(res.get_arr("errors").is_some_and(|e| e.is_empty()), "{:?}", res.get_arr("errors"));
    let (v, _) = call(r#"{"type":"validate","tag":"main"}"#);
    assert!(v.get("result").and_then(|r| r.get_arr("issues")).is_some_and(|i| i.is_empty()), "{:?}", v);
}
