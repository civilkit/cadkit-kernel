//! Grinding the top end of a helix-sweep coil flat: a plane cut through the wire near the helix end
//! (the centreline ends at y = 40, the wire reaches y = 40.99) used to fail with `booleanVolumeMismatch`: the
//! result was right, but the post-boolean volume check summed it with too few panels along the long fitted
//! trim curves and read it 3 percent high.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

fn coil(offset: f64) -> String {
    let circle = |id: &str, plane: &str, x: f64, r: f64| {
        format!(r#"{{"type":"sketch","id":"{id}","plane":{plane},"points":[{{"id":"p1","x":{x},"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":{r}}}],"constraints":[]}}"#)
    };
    let top = r#"{"kind":"std","id":"top"}"#;
    format!(
        r#"{{"format":"ck-part","version":1,"name":"g","features":[
{h},
{{"type":"helix","id":"X","sketch":"H","definedBy":"heightPitch","pitch":4,"height":40,"revolutions":10.0,"startAngle":0,"leftHand":false}},
{s},
{{"type":"sweep","id":"W","op":"add","sketch":"S","path":{{"kind":"curve","id":"X"}}}},
{g1},
{{"type":"extrude","id":"C1","op":"cut","sketch":"G1","end":"blind","depth":1,"flip":false}},
{{"type":"plane","id":"PT","mode":"offset","base":{top},"offset":{offset}}},
{g2},
{{"type":"extrude","id":"C2","op":"cut","sketch":"G2","end":"blind","depth":1,"flip":true}}]}}"#,
        h = circle("H", top, 0.0, 10.0),
        s = circle("S", r#"{"kind":"std","id":"front"}"#, 10.0, 1.0),
        g1 = circle("G1", top, 0.0, 12.0),
        g2 = circle("G2", r#"{"kind":"feature","id":"PT"}"#, 0.0, 12.0),
    )
}

fn run(offset: f64) -> (ck_topo::body::Body, String) {
    let part = Part::parse(&coil(offset)).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (ex.0.body.clone().expect("a body"), r.to_json().to_string())
}

/// Volume of the coil left by the two slab cuts (C1 removes y in [-1, 0], C2 y in [off, off + 1]), from an
/// independent integral over (radius, angle, height) of the screw-swept wire disc, at off = 38.0, 38.1 ... 40.9 (the wire's top is at 40.99, so
/// 41.0 and up cut nothing).
const WANT: [f64; 30] = [
    1914.11, 1914.14, 1914.31, 1914.67, 1915.25, 1916.08, 1917.19, 1918.58, 1920.27, 1922.27, 1924.58, 1927.16, 1929.93, 1932.82, 1935.78, 1938.78,
    1941.78, 1944.74, 1947.63, 1950.39, 1952.98, 1955.29, 1957.29, 1958.98, 1960.37, 1961.48, 1962.31, 1962.89, 1963.25, 1963.42,
];

#[test]
fn top_end_of_a_helix_coil_cuts_flat_at_every_offset() {
    let mut bad = vec![];
    for (i, want) in WANT.iter().enumerate() {
        let off = 38.0 + 0.1 * i as f64;
        let (b, errs) = run(off);
        if !errs.contains(r#""errors":[]"#) {
            bad.push(format!("offset {off:.1}: {}", &errs[..errs.len().min(300)]));
            continue;
        }
        if !ck_check::validate(&b, 1e-6).is_empty() {
            bad.push(format!("offset {off:.1}: invalid body"));
            continue;
        }
        let v = ck_topo::mass::mass_props(&b).volume;
        if (v - want).abs() > 0.1 {
            bad.push(format!("offset {off:.1}: volume {v:.3} vs {want}"));
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}
