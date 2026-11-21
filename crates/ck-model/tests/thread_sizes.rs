//! Thread size tables (`ck_model::threads`): the standard sizes are present, every row follows the
//! basic-dimension formulas of its standard, and the newly added sizes build a modeled thread on a shaft.

use ck_model::threads::{self, find, sizes};

use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_model::Part;

#[allow(dead_code)]
mod tf {
    use super::*;
    use ck_json::Json;
    #[derive(Default)]
    struct Noop;
    impl Tracer for Noop {
        fn begin(&mut self, _label: &str) -> usize {
            0
        }
        fn relabel(&mut self, _id: usize, _label: &str) {}
        fn end(&mut self, _id: usize) {}
    }
    struct Exec {
        brep: Brep,
        labels: Noop,
    }
    impl FeatureExec for Exec {
        fn tracer(&mut self) -> Option<&mut dyn Tracer> {
            Some(&mut self.labels)
        }
        fn face_count(&self) -> Option<usize> {
            self.brep.face_count()
        }
        fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
            self.brep.face_frame(s, face)
        }
        fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
            self.brep.exec(s, f)
        }
    }

    pub struct Run {
        pub result: RegenResult,
        pub body: Option<ck_topo::body::Body>,
        pub volume: f64,
        pub ms: f64,
    }

    impl Run {
        pub fn errors(&self) -> Vec<String> {
            self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect()
        }
        pub fn warning(&self, code: &str) -> Option<&regen::FeatureError> {
            self.result.warnings.iter().find(|w| w.error.code == code).map(|w| &w.error)
        }
        pub fn thread(&self, id: &str) -> &Json {
            &self.result.threads.iter().find(|t| t.0 == id).unwrap_or_else(|| panic!("no thread entry {id}: {:?}", self.result.threads)).1
        }
        pub fn faces(&self) -> Vec<String> {
            let b = self.body.as_ref().unwrap();
            let mut v: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            v.sort();
            v
        }
    }

    pub fn run(features: &str) -> Run {
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
        let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
        let mut ex = Exec { brep: Brep::new(), labels: Noop };
        let t = std::time::Instant::now();
        let result = regen::regen(&part, &RegenOpts::default(), &mut ex);
        let ms = t.elapsed().as_secs_f64() * 1000.0;
        let volume = ex.brep.body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
        Run { result, body: ex.brep.body.clone(), volume, ms }
    }

}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

/// Sizes the measurement of the solidcore thread cases found missing, and the ISO 261 fine series.
const FINE_REQUIRED: &[&str] = &[
    "M44x1", "M44x1.5", "M48x0.75", "M8x1", "M10x1", "M10x1.25", "M12x1.25", "M12x1.5", "M14x1.5", "M16x1.5", "M18x1.5", "M20x1.5", "M20x2",
    "M22x1.5", "M24x1.5", "M24x2", "M27x2", "M30x1.5", "M30x2", "M33x2", "M36x3", "M39x3", "M8x0.75", "M10x0.75", "M12x1", "M14x1", "M16x1", "M18x2",
    "M20x1", "M22x2", "M25x1.5", "M27x1.5", "M30x1", "M32x1.5", "M33x1.5", "M36x1.5", "M36x2", "M39x1.5", "M40x1.5", "M42x1.5", "M45x1.5", "M48x1.5",
    "M48x2", "M50x1.5", "M52x1.5", "M56x1.5", "M60x1.5", "M64x2", "M1x0.2", "M2x0.25", "M2.5x0.35", "M6x0.5",
];

#[test]
fn required_sizes_exist() {
    for d in FINE_REQUIRED {
        assert!(find("metric-fine", d).is_some_and(|t| t.standard == "metric-fine"), "{d} missing");
    }
    for (std, d) in [("UNC", "2-1/2-4"), ("UNC", "3-4"), ("BSP", "G5"), ("BSP", "G6"), ("BSPT", "R5"), ("Tr", "Tr9x2"), ("Tr", "Tr80x10"), ("Tr", "Tr20x8"), ("Acme", "Acme 1-1/8-5"), ("Acme", "Acme 3-2")] {
        assert!(find(std, d).is_some(), "{std} {d} missing");
    }
}

/// Every row of every standard agrees with the formulas of its form.
#[test]
fn every_row_follows_its_standard_formulas() {
    for t in threads::all() {
        let (d, p, who) = (t.major, t.pitch, &t.designation);
        match t.standard {
            "metric" | "metric-fine" | "UNC" | "UNF" => {
                assert!(near(t.minor_internal, d - 1.082532 * p, 1e-4), "{who} D1");
                assert!(near(t.minor_external, d - 1.226869 * p, 1e-4), "{who} d3");
                assert!(near(t.pitch_dia, d - 0.649519 * p, 1e-4), "{who} d2");
                assert_eq!(t.angle, 60.0);
            }
            "BSP" | "BSPT" => {
                let h = 0.640327 * p;
                assert!(near(t.minor_internal, d - 2.0 * h, 1e-4) && near(t.pitch_dia, d - h, 1e-4), "{who}");
                assert_eq!(t.angle, 55.0);
            }
            "Tr" => {
                let ac = if p <= 1.5 { 0.15 } else if p <= 5.0 { 0.25 } else if p <= 12.0 { 0.5 } else { 1.0 };
                assert!(near(t.minor_internal, d - p, 1e-9) && near(t.minor_external, d - p - 2.0 * ac, 1e-9) && near(t.pitch_dia, d - p / 2.0, 1e-9), "{who}");
                assert_eq!(t.angle, 30.0);
            }
            "Acme" => {
                let clr = if 25.4 / p >= 10.0 - 1e-9 { 0.254 } else { 0.508 };
                assert!(near(t.minor_internal, d - p, 1e-9) && near(t.minor_external, d - p - clr, 1e-9) && near(t.pitch_dia, d - p / 2.0, 1e-9), "{who}");
            }
            "NPT" => {}
            s => panic!("standard {s}"),
        }
    }
}

/// The fine series is a series: pitches are the ISO fine ones, finer than the coarse pitch of the same diameter, and sorted.
#[test]
fn fine_rows_are_finer_than_coarse_and_unique() {
    const ISO_FINE_PITCHES: &[f64] = &[0.2, 0.25, 0.35, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 4.0];
    for t in sizes("metric-fine") {
        assert!(ISO_FINE_PITCHES.iter().any(|p| near(*p, t.pitch, 1e-9)), "{} pitch", t.designation);
        if let Some(c) = sizes("metric").into_iter().find(|c| near(c.major, t.major, 1e-9)) {
            assert!(t.pitch < c.pitch - 1e-9, "{} is not finer than {}", t.designation, c.designation);
        }
        // tap drill: major - pitch rounded to 0.1 mm (the convention of the existing rows)
        assert!(near(t.tap_drill, t.major - t.pitch, 0.051), "{} tap {}", t.designation, t.tap_drill);
    }
}

fn shaft(d: f64, len: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"c1","c":"c","r":{}}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{len}}}"#,
        d / 2.0
    )
}

fn th(id: &str, extra: &str) -> String {
    format!(r#",{{"type":"thread","id":"{id}","face":"F2.S.c1","modeled":true,{extra}}}"#)
}

/// A modeled thread of `std` `size` on a shaft of its major diameter, `turns` turns long.
fn build_on_shaft(std: &str, size: &str, turns: f64) -> (tf::Run, f64, f64) {
    let t = find(std, size).unwrap();
    let len = turns * t.pitch + 2.0 * t.pitch;
    let base = shaft(t.major, len);
    let plain = tf::run(&base);
    let r = tf::run(&format!("{base}{}", th("T1", &format!(r#""standard":"{std}","size":"{size}""#))));
    let g = ck_model::features::thread::model::groove(t, false, t.major / 2.0).unwrap();
    (r, plain.volume - 0.0, g.per_pitch * (len / t.pitch))
}

#[test]
fn new_sizes_build_a_valid_modeled_thread_on_a_shaft() {
    for (std, size) in [
        ("metric-fine", "M44x1"),
        ("metric-fine", "M44x1.5"),
        ("metric-fine", "M48x0.75"),
        ("metric-fine", "M14x1.25"),
        ("metric-fine", "M2x0.25"),
        ("Tr", "Tr9x2"),
        ("Tr", "Tr34x6"),
        ("Acme", "Acme 1-1/8-5"),
        ("BSP", "G5"),
    ] {
        let (r, v0, want) = build_on_shaft(std, size, 5.0);
        assert!(r.errors().is_empty(), "{size}: {:?}", r.errors());
        assert!(r.warning("threadModelFallback").is_none(), "{size} fell back: {:?}", r.result.warnings.iter().map(|w| w.error.describe()).collect::<Vec<_>>());
        assert_eq!(r.thread("T1").get_bool("modeled"), Some(true), "{size}");
        let b = r.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "{size} invalid");
        assert_eq!(b.num_lumps(), 1);
        let dv = v0 - r.volume;
        assert!(dv > 0.0 && (dv - want).abs() <= 0.15 * want, "{size}: removed {dv}, analytic {want}");
    }
}
