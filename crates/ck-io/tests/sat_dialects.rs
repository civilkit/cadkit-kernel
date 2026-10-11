//! ACIS SAT dialect handling on the hand-written box fixture: line endings,
//! indexed records, body transforms, empty files.

use ck_io::sat::import_sat;

fn box_sat() -> String {
    let p = format!("{}/tests/fixtures/sat/box.sat", env!("CARGO_MANIFEST_DIR"));
    std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{p}: {e}"))
}

/// CR-only (old Mac) and CRLF line endings.
#[test]
fn cr_and_crlf_line_endings() {
    let t = box_sat();
    for nl in ["\r", "\r\n"] {
        let imp = import_sat(&t.replace('\n', nl)).unwrap_or_else(|e| panic!("{e}"));
        assert_eq!(imp.body.num_faces(), 6);
    }
}

/// ACIS >= 20 dialect: `-N` record prefixes and a second id integer.
#[test]
fn indexed_records_with_extra_id_integer() {
    let t = box_sat();
    let mut out = String::new();
    let mut n = 0;
    for (i, l) in t.lines().enumerate() {
        if i < 3 || l.starts_with("End-of") {
            out.push_str(l);
        } else {
            let mut tk: Vec<&str> = l.split(' ').collect();
            if !tk[0].ends_with("attrib") {
                tk.insert(3, "-1");
            }
            out.push_str(&format!("-{n} {}", tk.join(" ")));
            n += 1;
        }
        out.push('\n');
    }
    let imp = import_sat(&out).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(imp.body.num_faces(), 6);
    assert_eq!(imp.summary.bodies, 1);
}

/// Body transform: rotate 90 deg about z, uniform scale 2, translate +100 x.
#[test]
fn body_transform_rotation_scale_translation() {
    let t = box_sat();
    let n = t.lines().skip(3).filter(|l| !l.starts_with("End-of")).count();
    let t = t.replacen("body $-1 -1 $-1 $1 $-1 $-1 #", &format!("body $-1 -1 $-1 $1 $-1 ${n} #"), 1);
    let t = t.replace("End-of-ACIS-data", "transform $-1 -1 0 1 0 -1 0 0 0 0 1 100 0 0 2 rotate no_reflect no_shear #\nEnd-of-ACIS-data");
    let imp = import_sat(&t).unwrap_or_else(|e| panic!("{e}"));
    let m = ck_topo::mass::lump_moments(&imp.body);
    assert!((m[0].volume - 6000.0 * 8.0).abs() < 1e-6 * 48000.0, "{}", m[0].volume);
    let (lo, hi) = imp.body.bbox();
    for (a, b) in [(lo.x, 60.0), (hi.x, 100.0), (lo.y, 0.0), (hi.y, 20.0), (lo.z, 0.0), (hi.z, 60.0)] {
        assert!((a - b).abs() < 1e-6, "bbox {lo:?} {hi:?}");
    }
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

/// A file that yields no faces is a structured error, never an empty success.
#[test]
fn zero_faces_is_an_error() {
    let t = "700 0 1 0\n@1 x\n1 1e-06 1e-10\nbody $-1 -1 $-1 $-1 $-1 $-1 #\nEnd-of-ACIS-data\n";
    let e = import_sat(t).unwrap_err();
    assert_eq!(e.code, "sat.empty");
}
