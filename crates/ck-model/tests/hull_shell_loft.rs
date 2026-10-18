//! Shell and thicken of polygon-section (creased) multi-station lofts: the original-kernel gap cases.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

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

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

pub fn case_json(file: &str) -> ck_json::Json {
    let p = format!("{}/../../tools/cases/original-gaps/{file}", env!("CARGO_MANIFEST_DIR"));
    ck_json::parse(&std::fs::read_to_string(&p).unwrap()).unwrap()
}

fn case_part(id: &str) -> String {
    let j = case_json(&format!("{id}.json"));
    format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(j.get("features").expect("features")))
}

/// The original kernel's volume of case `id` (recorded by `tools/orig_diff_sweep.mjs --seed 7`).
fn orig_volume(id: &str) -> f64 {
    case_json("orig_volumes.json").get(id).and_then(|c| c.get_f64("volume")).unwrap_or_else(|| panic!("no recorded volume for {id}"))
}

fn volume(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().expect("a body")).volume
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}

fn check_against_original(id: &str) {
    let o = run_part(&case_part(id));
    assert!(o.errors.is_empty(), "{id}: {:?}", o.errors);
    assert_valid_solid(id, &o);
    let (v, want) = (volume(&o), orig_volume(id));
    if let Some((_, sheet)) = o.brep.surfs.first() {
        let a = ck_topo::mass::mass_props(sheet).area;
        eprintln!("{id}: sheet area {a:.1}");
    }
    eprintln!("{id}: volume {v:.1} original {want:.1} ({:+.4} %)", 100.0 * (v - want) / want);
    assert!((v - want).abs() <= 1e-3 * want, "{id}: volume {v} against the original's {want}");
}

/// The flared 7-station boat hull of the hand test, deck removed, 3 mm wall: a polygon-section loft whose opened
/// top is a free-form (B-spline) face. The original's volume is 20,643,366.8 mm3.
#[test]
fn seed_hull_shell_matches_original() {
    check_against_original("hull_seed_shell3");
}

/// Random 5 to 8 station hulls of `node tools/orig_diff_sweep.mjs --samples 10 --seed 7` (walls 1 to 5 mm).
#[test]
fn random_hull_shells_match_original() {
    for id in ["hull_shell_0", "hull_shell_1", "hull_shell_2", "hull_shell_3"] {
        check_against_original(id);
    }
}

#[test]
fn more_random_hull_shells_match_original() {
    for id in ["hull_shell_5", "hull_shell_6", "hull_shell_7", "hull_shell_8", "hull_shell_9"] {
        check_against_original(id);
    }
}

/// The shell of a convex body opened on one face is `area x t` less the curvature term: the wall volume must lie
/// between 90 % and 100 % of the kept area times the thickness.
#[test]
fn seed_hull_wall_is_area_times_thickness() {
    let base = {
        let src = case_part("hull_seed_shell3").replace(r#",{"type":"shell","id":"F9","faces":["F8.S.e0_2"],"thickness":3}"#, "");
        run_part(&src)
    };
    assert!(base.errors.is_empty(), "{:?}", base.errors);
    let b = base.brep.body.as_ref().unwrap();
    let total = ck_topo::mass::mass_props(b).area;
    let top = b.face_ids().find(|&f| b.face_id_str(f) == "F8.S.e0_2").unwrap();
    let top_area = ck_topo::mass::faces_moments(b, &[top], ck_geom::Vec3::ZERO).area;
    let wall = volume(&run_part(&case_part("hull_seed_shell3")));
    let at = (total - top_area) * 3.0;
    assert!(wall < at && wall > 0.9 * at, "wall {wall} against area x t {at}");
}

/// Thicken of closed-section surface lofts (polygon sections: the faces meet in creases).
#[test]
fn creased_loft_thicken_matches_original() {
    for id in ["hull_thicken_1", "hull_thicken_2", "hull_thicken_4", "hull_thicken_7", "hull_thicken_8"] {
        check_against_original(id);
    }
}

const HULL: [[f64; 4]; 7] = [[0.0, 750.0, 600.0, 0.0], [600.0, 750.0, 600.0, 0.0], [1200.0, 745.0, 595.0, 0.0], [1800.0, 700.0, 540.0, 0.0], [2400.0, 560.0, 390.0, 40.0], [3000.0, 330.0, 190.0, 110.0], [3500.0, 60.0, 20.0, 260.0]];

/// The boat hull's stations lofted as a surface from OPEN sections (`polyline`: deck, side, bottom, side, deck)
/// or closed triangles, then thickened.
fn skin_part(open: bool, t: f64, side: &str) -> String {
    let h = 450.0;
    let mut f: Vec<String> = vec![];
    let mut sk: Vec<String> = vec![];
    for (i, s) in HULL.iter().enumerate() {
        let (x, wt, wb, yb) = (s[0], s[1], s[2], s[3]);
        f.push(format!(r#"{{"type":"plane","id":"P{i}","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":{x}}}"#));
        let pts: Vec<(f64, f64)> = if open { vec![(-wt, h), (-wb, yb), (wb, yb), (wt, h)] } else { vec![(-wb, yb), (wb, yb), (wt, h)] };
        let n = pts.len();
        let ne = if open { n - 1 } else { n };
        let ps: Vec<String> = pts.iter().enumerate().map(|(j, (u, v))| format!(r#"{{"id":"p{i}_{j}","x":{u},"y":{v}}}"#)).collect();
        let es: Vec<String> = (0..ne).map(|j| format!(r#"{{"type":"line","id":"e{i}_{j}","p1":"p{i}_{j}","p2":"p{i}_{}"}}"#, (j + 1) % n)).collect();
        f.push(format!(r#"{{"type":"sketch","id":"S{i}","plane":{{"kind":"feature","id":"P{i}"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, ps.join(","), es.join(",")));
        sk.push(format!("\"S{i}\""));
    }
    f.push(format!(r#"{{"type":"loft","id":"F8","op":"add","surface":true,"sketches":[{}]}}"#, sk.join(",")));
    f.push(format!(r#"{{"type":"thicken","id":"F9","surface":"F8","thickness":{t},"side":"{side}"}}"#));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

/// The kit's open "tinnie" sheet (deck, side, bottom, side, deck sections; 7 stations) thickened to the inside:
/// the original gives 18,808,115.6 mm3 for 3 mm.
#[test]
fn open_section_loft_thickens_to_the_inside() {
    let o = run_part(&skin_part(true, 3.0, "back"));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("tinnie back", &o);
    let v = volume(&o);
    eprintln!("tinnie back: {v:.1}");
    assert!((v - 18808115.58).abs() <= 1e-3 * 18808115.58, "volume {v}");
}

/// The original refuses the outside and both-way thicken of this sheet (`thickenReach`: its offset surfaces do not
/// reach the miter of the end corner, where the bottom face is only 40 mm wide). The offset surfaces here are
/// continued as far as the corner needs, so these succeed: valid solids whose volume is the sheet area times the
/// thickness less/plus the corner terms (within 1.5 % of the inside offset's).
#[test]
fn open_section_loft_outside_and_both_thicken() {
    let back = volume(&run_part(&skin_part(true, 3.0, "back")));
    for side in ["front", "both"] {
        let o = run_part(&skin_part(true, 3.0, side));
        assert!(o.errors.is_empty(), "{side}: {:?}", o.errors);
        assert_valid_solid(side, &o);
        let v = volume(&o);
        eprintln!("tinnie {side}: {v:.1} (inside {back:.1})");
        assert!((v - back).abs() < 0.015 * back, "{side}: volume {v} against the inside offset's {back}");
    }
}

/// The closed-triangle loft (a tube whose sharpest corner is an 11 degree wedge at the stern): the inside offset
/// works; the outside offset needs the offset faces continued about 10 thicknesses past the wedge's edge, which
/// is refused with `thickenReach` ("the offset reaches past a face"), as the original refuses it.
#[test]
fn closed_triangle_loft_inside_works_outside_reaches_past_a_face() {
    let back = run_part(&skin_part(false, 3.0, "back"));
    assert!(back.errors.is_empty(), "{:?}", back.errors);
    assert_valid_solid("triangle back", &back);
    for side in ["front", "both"] {
        let o = run_part(&skin_part(false, 3.0, side));
        assert_eq!(o.errors.len(), 1, "{side}");
        assert!(format!("{:?}", o.errors[0].error).contains("thickenReach"), "{side}: {:?}", o.errors);
    }
}

fn part_from(features: Vec<String>) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))
}

/// A hull of rounded sections (a spline bottom and a straight deck line each) shelled open at the deck: the deck
/// is the loft's own straight-edge face, a free-form face.
#[test]
fn rounded_section_hull_shell_opens_the_deck() {
    let stations = [(0.0, 60.0, 30.0), (150.0, 90.0, 40.0), (300.0, 100.0, 45.0), (450.0, 80.0, 35.0), (600.0, 40.0, 20.0)];
    let mut f = vec![];
    let mut sk = vec![];
    for (i, (x, w, d)) in stations.iter().enumerate() {
        f.push(format!(r#"{{"type":"plane","id":"P{i}","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":{x}}}"#));
        let p = [(-w, 20.0), (-0.85 * w, -0.45 * d), (0.0, -d), (0.85 * w, -0.45 * d), (*w, 20.0)];
        let pts: Vec<String> = p.iter().enumerate().map(|(j, (u, v))| format!(r#"{{"id":"p{j}","x":{u},"y":{v}}}"#)).collect();
        f.push(format!(
            r#"{{"type":"sketch","id":"S{i}","plane":{{"kind":"feature","id":"P{i}"}},"points":[{}],"entities":[{{"id":"e1","type":"spline","pts":["p0","p1","p2","p3","p4"]}},{{"id":"e2","type":"line","p1":"p4","p2":"p0"}}],"constraints":[]}}"#,
            pts.join(",")
        ));
        sk.push(format!("\"S{i}\""));
    }
    f.push(format!(r#"{{"type":"loft","id":"F8","op":"add","sketches":[{}]}}"#, sk.join(",")));
    let base = run_part(&part_from(f.clone()));
    assert!(base.errors.is_empty(), "{:?}", base.errors);
    let b = base.brep.body.as_ref().unwrap();
    let deck = b.face_ids().find(|&x| b.face_id_str(x) == "F8.S.e2").expect("deck face");
    let kept_area = ck_topo::mass::mass_props(b).area - ck_topo::mass::faces_moments(b, &[deck], ck_geom::Vec3::ZERO).area;
    let t = 1.5;
    f.push(format!(r#"{{"type":"shell","id":"F9","faces":["F8.S.e2"],"thickness":{t}}}"#));
    let o = run_part(&part_from(f));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("rounded hull", &o);
    let wall = volume(&o);
    eprintln!("rounded hull wall {wall:.1} against area x t {:.1}", kept_area * t);
    assert!(wall < kept_area * t && wall > 0.9 * kept_area * t, "wall {wall} against area x t {}", kept_area * t);
}

/// A tapered loft between two circles (a truncated cone, built as a loft) with the wide cap removed: analytic wall.
#[test]
fn tapered_loft_with_the_top_removed() {
    let (r0, r1, h, t): (f64, f64, f64, f64) = (40.0, 15.0, 60.0, 2.0);
    let mut f = vec![];
    for (i, (z, r)) in [(0.0, r0), (h, r1)].iter().enumerate() {
        f.push(format!(r#"{{"type":"plane","id":"P{i}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{z}}}"#));
        f.push(format!(
            r#"{{"type":"sketch","id":"S{i}","plane":{{"kind":"feature","id":"P{i}"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#
        ));
    }
    f.push(r#"{"type":"loft","id":"F8","op":"add","sketches":["S0","S1"]}"#.to_string());
    f.push(format!(r#"{{"type":"shell","id":"F9","faces":["F8.C1"],"thickness":{t}}}"#));
    let o = run_part(&part_from(f));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_valid_solid("tapered loft", &o);
    // outer frustum less the cavity: the lateral wall offset by t inwards along the normal, the floor t above the base
    let frustum = |ra: f64, rb: f64, hh: f64| std::f64::consts::PI * hh / 3.0 * (ra * ra + ra * rb + rb * rb);
    let cosa = (h / ((r0 - r1).powi(2) + h * h).sqrt()).abs();
    let k = (r0 - r1) / h;
    let (ra, rb) = (r0 - k * t - t / cosa, r1 - t / cosa);
    let want = frustum(r0, r1, h) - frustum(ra, rb, h - t);
    let v = volume(&o);
    eprintln!("tapered loft wall {v:.1} against analytic {want:.1}");
    assert!((v - want).abs() < 0.01 * want, "wall {v} against analytic {want}");
}
