//! `mirror` and `scale` of a body whose faces lie on `Surface::Offset` (as a STEP / SolidWorks import gives them).
//! The bodies are built from ordinary features and every analytic face is then rewritten, geometry unchanged, as
//! an offset of a shifted base (`Offset { base: shifted, dist: 1 }`), so the exact transform of the offset is
//! checked against the volume and validity of the plain body.
use ck_geom::surface::Surface;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

const D: f64 = 1.0;

/// The same surface as an offset of its inward shift (nested for some planes, so the collapse is exercised).
fn as_offset(s: &Surface, nest: bool) -> Option<Surface> {
    let off = |base: Surface, d: f64| Surface::Offset { base: Box::new(base), dist: d };
    Some(match s {
        Surface::Plane { frame } => {
            let mut f = frame.clone();
            f.o = f.o - f.z * D;
            if nest {
                off(off(Surface::Plane { frame: f }, 0.4 * D), 0.6 * D)
            } else {
                off(Surface::Plane { frame: f }, D)
            }
        }
        Surface::Cylinder { frame, r } => off(Surface::Cylinder { frame: frame.clone(), r: r - D }, D),
        Surface::Sphere { frame, r } => off(Surface::Sphere { frame: frame.clone(), r: r - D }, D),
        _ => return None,
    })
}

struct Exec {
    brep: Brep,
    rewrite: bool,
}
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)?;
        if self.rewrite && matches!(f.type_name.as_str(), "extrude" | "sphere") {
            if let Some(b) = self.brep.body.as_mut() {
                for (i, face) in b.faces.iter_mut().enumerate() {
                    if let Some(o) = as_offset(&face.surface, i % 2 == 0) {
                        face.surface = o;
                    }
                }
            }
        }
        Ok(())
    }
}

struct Out {
    errors: Vec<String>,
    volume: f64,
    valid: Vec<String>,
    offsets: usize,
    faces: usize,
    bbox: ([f64; 3], [f64; 3]),
}

fn run(features: &str, rewrite: bool) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::parse(&src).unwrap();
    let mut ex = Exec { brep: Brep::new(), rewrite };
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.brep.body.as_ref().expect("a body");
    let (lo, hi) = b.bbox();
    Out {
        errors: res.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect(),
        volume: ck_topo::mass::mass_props(b).volume,
        valid: ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {}: {}", i.code, i.entity, i.detail)).collect(),
        offsets: b.faces.iter().filter(|f| matches!(f.surface, Surface::Offset { .. })).count(),
        faces: b.faces.len(),
        bbox: ([lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z]),
    }
}

const BLOCK: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
  "entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}"#;
const ROD: &str = r#"
 {"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"c","x":3,"y":0}],"entities":[{"type":"circle","id":"c1","c":"c","r":5}],"constraints":[]},
 {"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#;
const SPHERE: &str = r#"{"type":"sphere","id":"F1","op":"add","center":[1,2,3],"radius":10}"#;

// (no cone: the normal offset of a cone is a cone whose v origin is shifted, so it cannot stand for the same face parameters)
const BODIES: [(&str, &str); 3] = [("block", BLOCK), ("rod", ROD), ("sphere", SPHERE)];

fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-4 * b.abs().max(1.0)
}

#[test]
fn offset_faces_validate_before_transform() {
    for (name, base) in BODIES {
        let o = run(base, true);
        assert!(o.errors.is_empty(), "{name}: {:?}", o.errors);
        assert!(o.offsets > 0, "{name}: the rewrite made offset faces");
        assert!(o.valid.is_empty(), "{name}: {:?}", o.valid);
        assert!(close(o.volume, run(base, false).volume), "{name}: same volume as the plain body");
    }
}

#[test]
fn mirror_of_offset_faces_is_exact() {
    for (name, base) in BODIES {
        let plain = run(base, false);
        for plane in [r#"{"kind":"std","id":"right"}"#, r#"{"kind":"std","id":"front"}"#, r#"{"kind":"std","id":"top"}"#] {
            let m = run(&format!(r#"{base},{{"type":"mirror","id":"M","body":true,"replace":true,"plane":{plane}}}"#), true);
            assert!(m.errors.is_empty(), "{name} {plane}: {:?}", m.errors);
            assert!(m.valid.is_empty(), "{name} {plane}: {:?}", m.valid);
            assert!(m.offsets > 0 && m.faces == plain.faces, "{name} {plane}: faces stay offset faces ({} of {})", m.offsets, m.faces);
            assert!(close(m.volume, plain.volume), "{name} {plane}: volume {} vs {}", m.volume, plain.volume);
        }
    }
}

#[test]
fn scale_of_offset_faces_is_exact() {
    for (name, base) in BODIES {
        let plain = run(base, false);
        let k = 1.5;
        let feat = format!(r#"{base},{{"type":"scale","id":"S","factor":{k},"at":[0,0,0]}}"#);
        let s = run(&feat, true);
        assert!(s.errors.is_empty(), "{name}: {:?}", s.errors);
        assert!(s.valid.is_empty(), "{name}: {:?}", s.valid);
        assert!(s.offsets > 0, "{name}: still offset faces");
        assert!(close(s.volume, plain.volume * k * k * k), "{name}: volume {} vs {}", s.volume, plain.volume * k * k * k);
        let want = run(&feat, false);
        for a in 0..3 {
            assert!((s.bbox.0[a] - want.bbox.0[a]).abs() < 1e-6 && (s.bbox.1[a] - want.bbox.1[a]).abs() < 1e-6, "{name}: bbox axis {a}");
        }
    }
}

#[test]
fn non_uniform_scale_of_an_offset_face_is_refused() {
    // an offset of a stretched base is not an offset of the stretched surface: refused, not approximated
    for (name, base) in [("block", BLOCK), ("sphere", SPHERE)] {
        let feat = format!(r#"{base},{{"type":"scale","id":"S","factor":[1,2,1],"at":[0,0,0]}}"#);
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{feat}]}}"#);
        let part = Part::parse(&src).unwrap();
        let mut ex = Exec { brep: Brep::new(), rewrite: true };
        let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
        let codes: Vec<_> = res.errors.iter().map(|e| e.error.code.clone()).collect();
        assert_eq!(codes, ["invalidValue"], "{name}");
    }
}
