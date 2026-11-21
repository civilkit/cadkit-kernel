//! The blobs of an `meshToSolid` feature (`ck_mesh::blob`) and the dry run (`ck_mesh::preview`): both layouts decode, a payload
//! written by the app's `meshPack.ts` (generated with it, see the constants) decodes to the same mesh, a damaged one is
//! `meshBadInput` and never a panic; the dry run lists the triangles a closing would add and builds nothing; a time budget
//! that has run out stops the conversion between its stages.
mod common;
use common::*;
use ck_geom::Vec3;
use ck_mesh::blob::{decode_base64, decode_bytes, encode_raw};
use ck_mesh::close::CloseOpts;
use ck_mesh::{mesh_to_solid, preview, Opts};

// cube [0,10]^3 (`common::cube`) as `encodeMesh` of packages/io/src/meshPack.ts writes it (grid step, and lossless with the
// y of vertex 0 = 0.123456789), and the first one deflated with node's `zlib.deflateRawSync`
const CKM2: &str = "434b4d3200000000080000000c0000002d431cebe2361a3f2e0000002400000000000000000000c09a0c0000bf9a0cc09a0c00c09a0c0000bf9a0cbf9a0cc09a0cc09a0c0000bf9a0cc09a0c00c09a0c0000000100080206000001080204100e061006080c04020c020a10080410040c0e0a020e0206";
const CKM2_DEFLATED: &str = "6589310a803010042747084738426ab1b4d5c2c2d622a5f8c17c46df62eb2744441bb79919b62ceb08a080017d69cf636ae601e8f8b65783addacdd79ffc3d38540238159f53c841cd8b49cceab3b7142549b800";
const CKM2_LOSSLESS: &str = "434b4d3201000000080000000c000000000000000000000060000000240000000000000000ea0000000000000000000000000000000000000000000000d60000000000000000000000000000000000000000000000fc00200000002000202000000020200020002020202020003d00410000004100414100000041410041004141414141000100080206000001080204100e061006080c04020c020a10080410040c0e0a020e0206";

fn unhex(s: &str) -> Vec<u8> {
    (0..s.len() / 2).map(|i| u8::from_str_radix(&s[2 * i..2 * i + 2], 16).unwrap()).collect()
}

fn f32s(p: &[f64]) -> Vec<f32> {
    p.iter().map(|&x| x as f32).collect()
}

fn cube_f32() -> (Vec<f32>, Vec<u32>) {
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    (f32s(&p), ix)
}

#[test]
fn raw_layout_decodes() {
    let (p, ix) = cube_f32();
    let m = decode_bytes(&encode_raw(&p, &ix)).unwrap();
    assert_eq!(m.positions.len(), 24);
    assert_eq!(m.indices, ix);
    assert_eq!(f32s(&m.positions), p);
    // base64 text, as the blobs table holds it
    let text = ck_text_b64(&encode_raw(&p, &ix));
    assert_eq!(decode_base64(&text).unwrap(), m);
}

fn ck_text_b64(b: &[u8]) -> String {
    const A: &[u8] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in b.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for k in 0..4 {
            if k <= c.len() {
                s.push(A[(n >> (18 - 6 * k) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

#[test]
fn raw_layout_deflated_decodes() {
    // a stored (uncompressed) deflate block holding the raw layout
    let (p, ix) = cube_f32();
    let raw = encode_raw(&p, &ix);
    let mut d = vec![1u8, raw.len() as u8, (raw.len() >> 8) as u8, !(raw.len() as u8), !((raw.len() >> 8) as u8)];
    d.extend(&raw);
    let m = decode_bytes(&d).unwrap();
    assert_eq!(m.indices, ix);
    assert_eq!(f32s(&m.positions), p);
}

#[test]
fn packed_mesh_of_the_app_decodes() {
    let (p, ix) = cube_f32();
    for (name, hex) in [("grid", CKM2), ("deflated", CKM2_DEFLATED)] {
        let m = decode_bytes(&unhex(hex)).unwrap_or_else(|r| panic!("{name}: {} {}", r.code, r.message));
        assert_eq!(m.indices, ix, "{name}");
        assert_eq!(f32s(&m.positions), p, "{name}");
    }
    let m = decode_bytes(&unhex(CKM2_LOSSLESS)).unwrap();
    assert_eq!(m.indices, ix);
    assert_eq!(m.positions[1] as f32, 0.123456789f32);
    assert_eq!(m.positions[0], 0.0);
    // and it converts
    let c = mesh_to_solid(&decode_bytes(&unhex(CKM2)).unwrap().positions, Some(&ix), &opts_faceted()).unwrap();
    assert_eq!(c.body.num_faces(), 6);
}

#[test]
fn damaged_blobs_are_refused_not_panics() {
    let (p, ix) = cube_f32();
    let raw = encode_raw(&p, &ix);
    let packed = unhex(CKM2);
    let mut cases: Vec<(String, Vec<u8>)> = vec![("empty".into(), vec![]), ("junk".into(), vec![7; 100]), ("cut raw".into(), raw[..raw.len() - 4].to_vec())];
    for cut in [4, 20, 36, 40, packed.len() - 1] {
        cases.push((format!("packed cut at {cut}"), packed[..cut].to_vec()));
    }
    let mut bigcount = packed.clone();
    bigcount[8..12].copy_from_slice(&0x1fff_ffffu32.to_le_bytes());
    cases.push(("vertex count beyond the data".into(), bigcount));
    let mut badindex = packed.clone();
    let n = badindex.len();
    badindex[n - 1] = 0x7e;
    cases.push(("index out of range".into(), badindex));
    let mut nostep = packed.clone();
    nostep[16..24].copy_from_slice(&0f64.to_le_bytes());
    cases.push(("no step".into(), nostep));
    for (name, b) in cases {
        match decode_bytes(&b) {
            Err(r) => assert_eq!(r.code, "meshBadInput", "{name}"),
            Ok(_) => panic!("{name}: decoded"),
        }
    }
    assert_eq!(decode_base64("not base64 !").unwrap_err().code, "meshBadInput");
}

fn cube_missing_top() -> (Vec<f64>, Vec<u32>) {
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    // quad 1 is the top (z = 10): its two triangles are indices 6..12
    let ix = ix.chunks(3).enumerate().filter(|(i, _)| !(2..4).contains(i)).flat_map(|(_, t)| t.iter().copied()).collect();
    (p, ix)
}

#[test]
fn dry_run_lists_the_fill_and_builds_nothing() {
    let (p, ix) = cube_missing_top();
    let o = Opts { analytic: false, close: Some(CloseOpts::default()), ..Default::default() };
    let rep = preview(&p, Some(&ix), &o).unwrap();
    let c = rep.closing.as_ref().expect("closing");
    assert_eq!(c.holes.len(), 1);
    assert_eq!(c.holes[0].fill.len(), 2, "a square cap is two triangles");
    let json = ck_mesh::request::preview_json("m", &rep);
    assert_eq!(json.get_bool("dryRun"), Some(true));
    let fill = json.path(&["closing", "holes"]).and_then(|h| h.at(0)).and_then(|h| h.get("fill")).expect("fill");
    assert_eq!(fill.get_arr("indices").unwrap().len(), 6);
    let pos = fill.get_arr("positions").unwrap();
    assert_eq!(pos.len(), 12, "four corners");
    for k in 0..4 {
        assert_eq!(pos[3 * k + 2].as_f64(), Some(10.0), "the cap lies on the top plane");
    }
    // nothing but the counts of the first stages: no faces, no volume
    assert!(json.get("faces").is_none() && json.get("volume").is_none());
    // and it is what the conversion adds
    let c2 = mesh_to_solid(&p, Some(&ix), &o).unwrap();
    assert_eq!(c2.report.closing.as_ref().unwrap().filled_triangles, 2);
    // the normal report does not carry the triangles
    let normal = ck_mesh::request::report_json("m", &c2.body, &c2.report);
    assert!(normal.path(&["closing", "holes"]).and_then(|h| h.at(0)).and_then(|h| h.get("fill")).is_none());
}

#[test]
fn dry_run_gives_the_refusal_the_conversion_would() {
    let (p, ix) = cube_missing_top();
    let r = preview(&p, Some(&ix), &Opts::default()).unwrap_err();
    assert_eq!(r.code, "meshNotWatertight");
    let o = Opts { close: Some(CloseOpts { max_hole_edges: 3, ..Default::default() }), ..Default::default() };
    let r = preview(&p, Some(&ix), &o).unwrap_err();
    assert_eq!(r.code, "meshHoleTooLarge");
    assert!(r.data.iter().any(|(k, _)| *k == "holes") && r.data.iter().any(|(k, _)| *k == "positions"));
    // a closed mesh previews with no holes
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    let rep = preview(&p, Some(&ix), &Opts { close: Some(CloseOpts::default()), ..Default::default() }).unwrap();
    assert!(rep.closing.unwrap().holes.is_empty());
}

#[test]
fn dry_run_option_of_the_request() {
    let msg = ck_json::parse(r#"{"type":"meshToSolid","opts":{"dryRun":true,"close":true}}"#).unwrap();
    assert!(ck_mesh::request::opts_of(&msg).dry_run);
    let msg = ck_json::parse(r#"{"type":"meshToSolid","opts":{"close":true}}"#).unwrap();
    assert!(!ck_mesh::request::opts_of(&msg).dry_run);
}

thread_local! {
    static NOW: std::cell::Cell<f64> = const { std::cell::Cell::new(0.0) };
}

fn fake_clock() -> f64 {
    NOW.with(|n| {
        n.set(n.get() + 1.0);
        n.get()
    })
}

#[test]
fn a_spent_time_budget_stops_the_conversion_between_stages() {
    use ck_core::budget::{set_clock, Scope, Which};
    let (p, ix) = cube(Vec3::new(0.0, 0.0, 0.0), 10.0);
    // no clock, no limit: converts
    assert!(mesh_to_solid(&p, Some(&ix), &Opts::default()).is_ok());
    set_clock(fake_clock);
    {
        // every poll of the clock costs a millisecond: the limit is gone after the second stage
        let _s = Scope::start(Which::Feature, Some(2.0));
        let r = mesh_to_solid(&p, Some(&ix), &Opts::default()).err().expect("stopped");
        assert_eq!(r.code, "meshBudgetExceeded");
    }
    // the scope ended with the limit: converts again
    assert!(mesh_to_solid(&p, Some(&ix), &Opts::default()).is_ok());
}
