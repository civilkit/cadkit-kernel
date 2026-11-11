//! Sheet metal export: the flat pattern as DXF and the bend table as CSV / JSON (docs/sheet-metal.md, "Export").
//! The expected numbers are those of black-box runs of a reference sheet metal tool (rule: t = 2, k = 0.44, r = 2, base
//! 100 x 60, flanges as in `sheet_metal.rs`): entity counts per layer, line and arc coordinates (the reference writes 12
//! decimals), the bend table rows and the note texts. The reference names its contour layers OUTLINE and CUT, so those
//! comparisons use `LayerNames::Reference`; the default names are CUT_OUTER and CUT_INNER.
use ck_json::Json;
use ck_model::features::sheet_export::*;
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

/// The regen wire result and the `flatPattern` of the last sheet entry.
fn run(features: &[String]) -> (Json, Option<Json>, Option<ck_core::linalg::Frame>) {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    assert!(r.errors.is_empty(), "{:?}", r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect::<Vec<_>>());
    let wire = r.to_json();
    let fp = wire.get_arr("sheetMetal").and_then(|a| a.iter().rev().find(|e| e.get("flatPattern").is_some()).cloned()).and_then(|e| e.get("flatPattern").cloned());
    (wire, fp, r.sketches.first().map(|s| s.frame))
}

fn rule() -> String {
    r#"{"type":"sheetRules","id":"R1","name":"R","thickness":2,"kFactor":0.44,"bendRadius":2}"#.to_string()
}

/// A base sketch: the rectangle 100 x 60, optionally with a circular hole and/or a rounded corner (radius 8 at the
/// top right, like the reference run that filleted that corner).
fn sketch(hole: bool, round: bool) -> String {
    let (pts, ents) = if round {
        (
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":100,"y":0},{"id":"p3","x":100,"y":52},{"id":"p3b","x":92,"y":60},{"id":"p4","x":0,"y":60},{"id":"k","x":92,"y":52}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"arc","id":"e3","c":"k","p1":"p3","p2":"p3b"},{"type":"line","id":"e4","p1":"p3b","p2":"p4"},{"type":"line","id":"e5","p1":"p4","p2":"p1"}"#,
        )
    } else {
        (
            r#"{"id":"p1","x":0,"y":0},{"id":"p2","x":100,"y":0},{"id":"p3","x":100,"y":60},{"id":"p4","x":0,"y":60}"#,
            r#"{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}"#,
        )
    };
    let (hp, he) = if hole { (r#",{"id":"c","x":50,"y":30}"#, r#",{"type":"circle","id":"e9","c":"c","r":5}"#) } else { ("", "") };
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{pts}{hp}],"entities":[{ents}{he}],"constraints":[]}}"#)
}

const BASE: &str = r#"{"type":"baseFlange","id":"B1","sketch":"S1"}"#;

/// Edge picks of the 100 x 60 base: a point on the cap face (`up`) or the start face of the sheet.
fn pick(frame: &ck_core::linalg::Frame, name: &str, up: bool) -> String {
    let (x, y) = match name {
        "bottom" => (50.0, 0.0),
        "top" => (50.0, 60.0),
        "left" => (0.0, 30.0),
        "right" => (100.0, 30.0),
        _ => panic!(),
    };
    let p = frame.o + frame.x * x + frame.y * y + frame.z * if up { 2.0 } else { 0.0 };
    format!("[{},{},{}]", p.x, p.y, p.z)
}

/// Rule, base and one `edgeFlange` per entry `(fields, [(edge, up)])`.
fn plate(hole: bool, round: bool, flanges: &[(&str, &[(&str, bool)])]) -> (Json, Json) {
    let (_, _, frame) = run(&[sketch(false, false)]);
    let frame = frame.unwrap();
    let mut fs = vec![rule(), sketch(hole, round), BASE.to_string()];
    for (n, (fields, edges)) in flanges.iter().enumerate() {
        let list: Vec<String> = edges.iter().map(|(e, up)| pick(&frame, e, *up)).collect();
        fs.push(format!(r#"{{"type":"edgeFlange","id":"E{}","edges":[{}],{}}}"#, n + 1, list.join(","), fields));
    }
    let (wire, fp, _) = run(&fs);
    (wire, fp.expect("a sheet entry"))
}

// ---- a DXF reader: just enough to read the files back

#[derive(Debug, Clone)]
struct Ent {
    ty: String,
    layer: String,
    g: Vec<(i32, String)>,
}

impl Ent {
    fn f(&self, code: i32) -> f64 {
        self.g.iter().find(|(c, _)| *c == code).unwrap_or_else(|| panic!("{} has no {code}", self.ty)).1.parse().unwrap()
    }
    fn s(&self, code: i32) -> String {
        self.g.iter().find(|(c, _)| *c == code).map(|g| g.1.clone()).unwrap_or_default()
    }
}

struct Dxf {
    pairs: Vec<(i32, String)>,
    ents: Vec<Ent>,
}

fn parse(text: &str) -> Dxf {
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(lines.len() % 2, 0, "group codes and values come in pairs");
    let pairs: Vec<(i32, String)> = lines.chunks(2).map(|c| (c[0].trim().parse().expect("group code"), c[1].to_string())).collect();
    assert_eq!(pairs.last(), Some(&(0, "EOF".to_string())));
    let mut ents = vec![];
    let mut inside = false;
    let mut cur: Option<Ent> = None;
    for (c, v) in &pairs {
        if *c == 2 && v == "ENTITIES" {
            inside = true;
            continue;
        }
        if !inside {
            continue;
        }
        if *c == 0 {
            if let Some(e) = cur.take() {
                ents.push(e);
            }
            if v == "ENDSEC" {
                break;
            }
            cur = Some(Ent { ty: v.clone(), layer: String::new(), g: vec![] });
        } else if let Some(e) = cur.as_mut() {
            if *c == 8 {
                e.layer = v.clone();
            }
            e.g.push((*c, v.clone()));
        }
    }
    Dxf { pairs, ents }
}

impl Dxf {
    fn header(&self, var: &str) -> Vec<(i32, String)> {
        let i = self.pairs.iter().position(|p| p.0 == 9 && p.1 == var).unwrap_or_else(|| panic!("no {var}"));
        self.pairs[i + 1..].iter().take_while(|p| p.0 != 9 && p.0 != 0).cloned().collect()
    }
    fn layers(&self) -> Vec<(String, i32, String)> {
        let mut out = vec![];
        let mut in_layer_table = false;
        let mut cur: Option<(String, i32, String)> = None;
        for (c, v) in &self.pairs {
            if *c == 2 && v == "LAYER" {
                in_layer_table = true;
                continue;
            }
            if !in_layer_table {
                continue;
            }
            if *c == 0 {
                if let Some(l) = cur.take() {
                    out.push(l);
                }
                if v == "ENDTAB" {
                    break;
                }
                cur = Some((String::new(), 0, String::new()));
            } else if let Some(l) = cur.as_mut() {
                match *c {
                    2 => l.0 = v.clone(),
                    62 => l.1 = v.parse().unwrap(),
                    6 => l.2 = v.clone(),
                    _ => {}
                }
            }
        }
        out
    }
    fn on(&self, layer: &str, ty: &str) -> Vec<&Ent> {
        self.ents.iter().filter(|e| e.layer == layer && e.ty == ty).collect()
    }
    /// Bounds of the LINE endpoints on a layer.
    fn line_bounds(&self, layer: &str) -> [f64; 4] {
        let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for e in self.on(layer, "LINE") {
            for (x, y) in [(10, 20), (11, 21)] {
                b = [b[0].min(e.f(x)), b[1].min(e.f(y)), b[2].max(e.f(x)), b[3].max(e.f(y))];
            }
        }
        b
    }
}

fn fp_of(j: &Json) -> FlatPattern {
    flat_pattern_from_json(j).expect("flat pattern")
}
use ck_model::features::sheet::FlatPattern;

fn reference() -> DxfOptions {
    DxfOptions { names: LayerNames::Reference, ..DxfOptions::default() }
}

fn near(a: f64, b: f64, what: &str) {
    assert!((a - b).abs() < 1e-6, "{what}: {a} vs {b}");
}

fn line(e: &Ent) -> [f64; 4] {
    [e.f(10), e.f(20), e.f(11), e.f(21)]
}

fn has_line(d: &Dxf, layer: &str, want: [f64; 4]) -> bool {
    d.on(layer, "LINE").iter().any(|e| {
        let l = line(e);
        let same = (0..4).all(|k| (l[k] - want[k]).abs() < 1e-6);
        let rev = (l[0] - want[2]).abs() < 1e-6 && (l[1] - want[3]).abs() < 1e-6 && (l[2] - want[0]).abs() < 1e-6 && (l[3] - want[1]).abs() < 1e-6;
        same || rev
    })
}

const F1: &str = r#""height":20,"angle":90"#;

#[test]
fn s1_dxf_matches_reference_structure() {
    let (_, fpj) = plate(false, false, &[(F1, &[("bottom", true), ("top", true)])]);
    let out = flat_pattern_dxf(&fp_of(&fpj), &reference());
    let d = parse(&out.dxf);
    assert_eq!(d.header("$ACADVER")[0].1, "AC1015");
    assert_eq!(d.header("$INSUNITS")[0].1, "4");
    assert_eq!(d.header("$MEASUREMENT")[0].1, "1");
    // layers of the reference run: 0, BEND_DOWN, BEND_UP, CUT, OUTLINE; colours 7 5 1 4 7; bends dashed
    let layers = d.layers();
    for (name, color, lt) in [("0", 7, "Continuous"), ("OUTLINE", 7, "Continuous"), ("CUT", 4, "Continuous"), ("BEND_UP", 1, "DASHED"), ("BEND_DOWN", 5, "DASHED")] {
        assert!(layers.iter().any(|l| l.0 == name && l.1 == color && l.2 == lt), "layer {name}: {layers:?}");
    }
    assert_eq!(d.ents.len(), 6);
    assert_eq!(d.on("OUTLINE", "LINE").len(), 4);
    assert_eq!(d.on("BEND_UP", "LINE").len(), 2);
    let b = d.line_bounds("OUTLINE");
    for (a, e) in b.iter().zip([0.0, -16.523893421169, 100.0, 76.523893421169]) {
        near(*a, e, "bounds");
    }
    // the four sides (the reference starts at another corner, the sides are the same lines)
    for l in [[100.0, -16.523893421169, 100.0, 76.523893421169], [100.0, 76.523893421169, 0.0, 76.523893421169], [0.0, 76.523893421169, 0.0, -16.523893421169], [0.0, -16.523893421169, 100.0, -16.523893421169]] {
        assert!(has_line(&d, "OUTLINE", l), "{l:?}");
    }
    let bends = d.on("BEND_UP", "LINE");
    assert_eq!(line(bends[0]).map(|v| (v * 1e6).round() / 1e6), [0.0, 1.738053, 100.0, 1.738053]);
    assert_eq!(line(bends[1]).map(|v| (v * 1e6).round() / 1e6), [100.0, 58.261947, 0.0, 58.261947]);
    // EXTMIN / EXTMAX
    assert_eq!(d.header("$EXTMIN")[0].1, "0.0");
    near(d.header("$EXTMAX")[1].1.parse().unwrap(), 76.523893421169, "extmax y");
    // default layer names
    let d = parse(&flat_pattern_dxf(&fp_of(&fpj), &DxfOptions::default()).dxf);
    assert_eq!(d.on("CUT_OUTER", "LINE").len(), 4);
    for n in ["CUT_OUTER", "CUT_INNER", "BEND_UP", "BEND_DOWN", "BEND_TANGENT"] {
        assert!(d.layers().iter().any(|l| l.0 == n), "layer {n} is declared even when empty");
    }
}

#[test]
fn s4_dxf_matches_reference_structure() {
    let (_, fpj) = plate(false, false, &[(r#""height":15"#, &[("bottom", true), ("top", true), ("left", true), ("right", true)])]);
    let d = parse(&flat_pattern_dxf(&fp_of(&fpj), &reference()).dxf);
    // 28 sides (collinear pieces joined), 4 bend lines
    assert_eq!(d.ents.len(), 32);
    assert_eq!(d.on("OUTLINE", "LINE").len(), 28);
    assert_eq!(d.on("BEND_UP", "LINE").len(), 4);
    let b = d.line_bounds("OUTLINE");
    for (a, e) in b.iter().zip([-11.523893421169, -11.523893421169, 111.523893421169, 71.523893421169]) {
        near(*a, e, "bounds");
    }
    // sides of the reference file (first, a notch, last)
    assert!(has_line(&d, "OUTLINE", [96.0, 4.0, 100.523893421169, 4.0]));
    assert!(has_line(&d, "OUTLINE", [100.523893421169, 3.414213562373, 111.523893421169, 3.414213562373]));
    assert!(has_line(&d, "OUTLINE", [3.414213562373, -11.523893421169, 96.585786437627, -11.523893421169]));
    assert!(has_line(&d, "OUTLINE", [96.0, -0.523893421169, 96.0, 4.0]));
    // bend lines
    assert!(has_line(&d, "BEND_UP", [4.0, 1.738053289415, 96.0, 1.738053289415]));
    assert!(has_line(&d, "BEND_UP", [96.0, 58.261946710585, 4.0, 58.261946710585]));
    assert!(has_line(&d, "BEND_UP", [1.738053289415, 56.0, 1.738053289415, 4.0]));
    assert!(has_line(&d, "BEND_UP", [98.261946710585, 4.0, 98.261946710585, 56.0]));
}

#[test]
fn down_bends_notes_and_options() {
    // reference: bend 1 picked on the starting face is DOWN, bend 2 UP; notes `B1 DOWN 90%%d R2` 1.5 above the line
    let (_, fpj) = plate(false, false, &[(F1, &[("bottom", false), ("top", true)])]);
    let fp = fp_of(&fpj);
    let o = DxfOptions { bend_notes: true, ..reference() };
    let d = parse(&flat_pattern_dxf(&fp, &o).dxf);
    assert_eq!(d.on("BEND_DOWN", "LINE").len(), 1);
    assert_eq!(d.on("BEND_UP", "LINE").len(), 1);
    let notes = d.on("BEND_NOTES", "TEXT");
    assert_eq!(notes.len(), 2);
    assert_eq!(notes[0].s(1), "B1 DOWN 90%%d R2");
    assert_eq!(notes[1].s(1), "B2 UP 90%%d R2");
    near(notes[0].f(10), 50.0, "note x");
    near(notes[0].f(20), 1.738053289415 + 1.5, "note y");
    near(notes[0].f(40), 3.0, "note height");
    assert_eq!(notes[0].s(72), "1");
    assert!(d.layers().iter().any(|l| l.0 == "BEND_NOTES" && l.1 == 8));
    // entity order of the reference: bend line, its note, next bend line, its note
    let kinds: Vec<&str> = d.ents.iter().skip(4).map(|e| e.ty.as_str()).collect();
    assert_eq!(kinds, ["LINE", "TEXT", "LINE", "TEXT"]);

    // bend_lines off: no bend layers entities; single layer: everything on layer 0
    let d = parse(&flat_pattern_dxf(&fp, &DxfOptions { bend_lines: false, ..reference() }).dxf);
    assert_eq!(d.ents.len(), 4);
    let d = parse(&flat_pattern_dxf(&fp, &DxfOptions { layers: LayerMode::Single, ..reference() }).dxf);
    assert_eq!(d.ents.len(), 6);
    assert!(d.ents.iter().all(|e| e.layer == "0"));
}

#[test]
fn vertical_notes_are_rotated_and_set_left_of_the_line() {
    // reference (45 degree flanges, note height 2): the note of a bend running along y is rotated 90 and one note-height/2 to -x
    let (_, fpj) = plate(false, false, &[(r#""height":15,"angle":45"#, &[("left", true), ("right", false)])]);
    let o = DxfOptions { bend_notes: true, note_height: 2.0, ..reference() };
    let d = parse(&flat_pattern_dxf(&fp_of(&fpj), &o).dxf);
    let up = d.on("BEND_UP", "LINE")[0];
    let down = d.on("BEND_DOWN", "LINE")[0];
    near(up.f(10), 0.525880893708, "x of the left bend");
    near(down.f(10), 99.474119106292, "x of the right bend");
    let notes = d.on("BEND_NOTES", "TEXT");
    assert_eq!(notes[0].s(1), "B1 UP 45%%d R2");
    assert_eq!(notes[1].s(1), "B2 DOWN 45%%d R2");
    near(notes[0].f(50), 90.0, "rotation");
    near(notes[0].f(10), 0.525880893708 - 1.0, "note x");
    near(notes[0].f(20), 30.0, "note y");
    near(notes[1].f(10), 99.474119106292 - 1.0, "note x");
}

#[test]
fn holes_and_arcs_come_back_as_circles_and_arcs() {
    let (_, fpj) = plate(true, false, &[(r#""height":20"#, &[("bottom", true)])]);
    let fp = fp_of(&fpj);
    assert_eq!(fp.outline.len(), 2);
    let d = parse(&flat_pattern_dxf(&fp, &DxfOptions::default()).dxf);
    let circles = d.on("CUT_INNER", "CIRCLE");
    assert_eq!(circles.len(), 1);
    near(circles[0].f(10), 50.0, "cx");
    near(circles[0].f(20), 30.0, "cy");
    near(circles[0].f(40), 5.0, "r");
    assert_eq!(d.ents.iter().filter(|e| e.layer == "CUT_INNER").count(), 1);
    // inner contours come before the outline
    assert_eq!(d.ents[0].layer, "CUT_INNER");

    // a rounded corner (the reference: ARC about (92,52), radius 8, 0 to 90 degrees; flanges next to an arc are
    // not built yet, so this is the plate alone)
    let (_, fpj) = plate(false, true, &[]);
    let d = parse(&flat_pattern_dxf(&fp_of(&fpj), &reference()).dxf);
    let arcs = d.on("OUTLINE", "ARC");
    assert_eq!(arcs.len(), 1);
    near(arcs[0].f(10), 92.0, "cx");
    near(arcs[0].f(20), 52.0, "cy");
    near(arcs[0].f(40), 8.0, "r");
    near(arcs[0].f(50), 0.0, "start");
    near(arcs[0].f(51), 90.0, "end");
    assert_eq!(d.on("OUTLINE", "LINE").len(), 4);
    assert!(has_line(&d, "OUTLINE", [100.0, 0.0, 100.0, 52.0]));
    assert!(has_line(&d, "OUTLINE", [92.0, 60.0, 0.0, 60.0]));
    assert!(has_line(&d, "OUTLINE", [0.0, 60.0, 0.0, 0.0]));
    assert!(has_line(&d, "OUTLINE", [0.0, 0.0, 100.0, 0.0]));
    assert_eq!(d.ents.len(), 5, "no bends, no other entities");
}

#[test]
fn units_r12_and_tangent_lines() {
    let (_, fpj) = plate(false, false, &[(F1, &[("bottom", true), ("top", true)])]);
    let fp = fp_of(&fpj);
    // inches: $INSUNITS 1, $MEASUREMENT 0, coordinates scaled
    let d = parse(&flat_pattern_dxf(&fp, &DxfOptions { units: Units::In, ..reference() }).dxf);
    assert_eq!(d.header("$INSUNITS")[0].1, "1");
    assert_eq!(d.header("$MEASUREMENT")[0].1, "0");
    near(d.line_bounds("OUTLINE")[2], 100.0 / 25.4, "width in inches");
    for (u, code, f) in [(Units::Cm, "5", 0.1), (Units::M, "6", 0.001), (Units::Ft, "2", 1.0 / 304.8), (Units::Mm, "4", 1.0)] {
        let d = parse(&flat_pattern_dxf(&fp, &DxfOptions { units: u, ..reference() }).dxf);
        assert_eq!(d.header("$INSUNITS")[0].1, code);
        near(d.line_bounds("OUTLINE")[3], 76.523893421169 * f, "height");
    }
    // tangent lines: two per bend, half an allowance either side of the bend line
    let d = parse(&flat_pattern_dxf(&fp, &DxfOptions { tangent_lines: true, ..reference() }).dxf);
    let t = d.on("BEND_TANGENT", "LINE");
    assert_eq!(t.len(), 4);
    assert!(has_line(&d, "BEND_TANGENT", [0.0, 1.738053289415 - 4.523893421169 / 2.0, 100.0, 1.738053289415 - 4.523893421169 / 2.0]));
    assert!(has_line(&d, "BEND_TANGENT", [0.0, 1.738053289415 + 4.523893421169 / 2.0, 100.0, 1.738053289415 + 4.523893421169 / 2.0]));
    // R12: no handles, no subclass markers, no objects section
    let text = flat_pattern_dxf(&fp, &DxfOptions { version: DxfVersion::R12, ..reference() }).dxf;
    let d = parse(&text);
    assert_eq!(d.header("$ACADVER")[0].1, "AC1009");
    assert!(d.pairs.iter().all(|p| p.0 != 5 && p.0 != 100 && p.0 != 330), "R12 has no handles or subclass markers");
    assert!(!d.pairs.iter().any(|p| p.1 == "OBJECTS"));
    assert_eq!(d.on("OUTLINE", "LINE").len(), 4);
    assert_eq!(d.on("BEND_UP", "LINE").len(), 2);
    assert_eq!(d.layers().len(), 6);
    // R2000: every entity has a handle and an owner; handles are unique and below $HANDSEED
    let text = flat_pattern_dxf(&fp, &DxfOptions::default()).dxf;
    let d = parse(&text);
    let mut hs: Vec<u32> = d.pairs.iter().filter(|p| p.0 == 5).skip(1).map(|p| u32::from_str_radix(&p.1, 16).unwrap()).collect();
    let n = hs.len();
    hs.sort();
    hs.dedup();
    assert_eq!(hs.len(), n, "handles are unique");
    let seed = u32::from_str_radix(&d.header("$HANDSEED")[0].1, 16).unwrap();
    assert!(hs.iter().all(|h| *h < seed));
}

#[test]
fn bend_table_csv_and_json_equal_the_reference() {
    let (_, fpj) = plate(false, false, &[(F1, &[("bottom", true), ("top", true)])]);
    let fp = fp_of(&fpj);
    let csv = bend_table_csv(&fp, "Body1");
    let want = "body,rule,thickness_mm,k_factor,bend,angle_deg,direction,radius_mm,allowance_mm,length_mm,start_x_mm,start_y_mm,end_x_mm,end_y_mm,included_angle_deg\r\n\
\"Body1\",\"R\",2.000000000,0.440000000,1,90.000000000,up,2.000000000,4.523893421,100.000000000,0.000000000,1.738053289,100.000000000,1.738053289,90.000000000\r\n\
\"Body1\",\"R\",2.000000000,0.440000000,2,90.000000000,up,2.000000000,4.523893421,100.000000000,100.000000000,58.261946711,0.000000000,58.261946711,90.000000000\r\n";
    assert_eq!(csv, want);

    // JSON: the keys and numbers of the reference to 1e-6
    let j = ck_json::parse(&pretty(&bend_table_json(&fp, "Body1"))).expect("pretty JSON parses");
    let keys: Vec<&str> = j.as_obj().unwrap().iter().map(|(k, _)| k.as_str()).collect();
    assert_eq!(keys, ["bend_zones", "bends", "body", "cutout_shapes", "cutouts", "flat_size_mm", "k_factor", "outline", "perforations", "pockets", "rule", "thickness", "unfolded"]);
    let bends = j.get_arr("bends").unwrap();
    assert_eq!(bends.len(), 2);
    let bkeys: Vec<&str> = bends[0].as_obj().unwrap().iter().map(|(k, _)| k.as_str()).collect();
    assert_eq!(bkeys, ["allowance", "angle_deg", "bend", "direction", "end", "included_angle_deg", "length", "radius", "start"]);
    let b0 = &bends[0];
    near(b0.get_f64("allowance").unwrap(), 4.523893421169302, "allowance");
    near(b0.get_f64("angle_deg").unwrap(), 90.0, "angle");
    near(b0.get_f64("included_angle_deg").unwrap(), 90.0, "included");
    near(b0.get_f64("length").unwrap(), 100.0, "length");
    near(b0.get_f64("radius").unwrap(), 2.0, "radius");
    assert_eq!(b0.get_str("direction"), Some("up"));
    let (s, e) = (b0.get("start").unwrap().as_f64_vec().unwrap(), b0.get("end").unwrap().as_f64_vec().unwrap());
    near(s[1], 1.7380532894153489, "start y");
    near(e[0], 100.0, "end x");
    let b1 = &bends[1];
    near(b1.get("start").unwrap().as_f64_vec().unwrap()[1], 58.26194671058465, "bend 2 start y");
    let z = j.get_arr("bend_zones").unwrap();
    near(z[0].as_f64_vec().unwrap()[2], -0.5238934211693023, "zone ymin");
    near(z[1].as_f64_vec().unwrap()[3], 60.523893421169305, "zone ymax");
    let fs = j.get("flat_size_mm").unwrap().as_f64_vec().unwrap();
    near(fs[1], 93.04778684233861, "flat length");
    near(j.get_f64("k_factor").unwrap(), 0.44, "k");
    assert_eq!(j.get_str("rule"), Some("R"));
    assert_eq!(j.get_str("body"), Some("Body1"));
    assert_eq!(j.get_arr("outline").unwrap().len(), 1);
    assert_eq!(j.get_arr("outline").unwrap()[0].as_arr().unwrap().len(), 4);
    assert!(j.get_arr("cutouts").unwrap().is_empty() && j.get_arr("cutout_shapes").unwrap().is_empty());
    // the text layout of the reference: two-space indent, whole floats keep `.0`, `bend` is an integer
    let text = pretty(&bend_table_json(&fp, "Body1"));
    assert!(text.contains("\"angle_deg\": 90.0,"), "{text}");
    assert!(text.contains("\"bend\": 1,"), "{text}");
    assert!(text.starts_with("{\n  \"bend_zones\": [\n    [\n      0.0,"), "{text}");
}

#[test]
fn hole_shapes_in_the_bend_table() {
    let (_, fpj) = plate(true, false, &[(r#""height":20"#, &[("bottom", true)])]);
    let j = bend_table_json(&fp_of(&fpj), "Body1");
    let shapes = j.get_arr("cutout_shapes").unwrap();
    assert_eq!(shapes.len(), 1);
    let c = shapes[0].get("circle").expect("a round hole is a circle");
    near(c.get_f64("radius").unwrap(), 5.0, "radius");
    assert_eq!(c.get("center").unwrap().as_f64_vec().unwrap().len(), 2);
    assert_eq!(j.get_arr("cutouts").unwrap().len(), 1);
}

#[test]
fn wire_replies_from_the_regen_result() {
    let (wire, fpj) = plate(false, false, &[(F1, &[("bottom", true), ("top", true)])]);
    // the whole sheetMetal entry or just its flatPattern
    let entry = wire.get_arr("sheetMetal").unwrap().iter().rev().find(|e| e.get("flatPattern").is_some()).unwrap().clone();
    for fp in [fpj.clone(), entry] {
        let msg = Json::obj().with("flatPattern", fp).with("units", "mm").with("layerNames", "reference");
        let r = dxf_reply(&msg).unwrap();
        let d = parse(r.get_str("dxf").unwrap());
        assert_eq!(d.on("OUTLINE", "LINE").len(), 4);
        assert_eq!(r.get_f64("bends"), Some(2.0));
        assert_eq!(r.get("entities").unwrap().get_f64("BEND_UP"), Some(2.0));
        near(r.get("bounds").unwrap().as_f64_vec().unwrap()[3], 76.5238934211693, "bounds");
    }
    let msg = Json::obj().with("flatPattern", fpj.clone()).with("format", "csv").with("body", "Body7");
    let r = bend_table_reply(&msg).unwrap();
    assert_eq!(r.get_str("format"), Some("csv"));
    assert!(r.get_str("text").unwrap().contains("\"Body7\",\"R\",2.000000000,0.440000000,1,"));
    let r = bend_table_reply(&Json::obj().with("flatPattern", fpj.clone()).with("format", "json")).unwrap();
    assert!(ck_json::parse(r.get_str("text").unwrap()).is_ok());
    // refusals say what is wrong
    assert!(dxf_reply(&Json::obj()).unwrap_err().contains("flatPattern"));
    assert!(dxf_reply(&Json::obj().with("flatPattern", fpj.clone()).with("units", "yd")).unwrap_err().contains("units"));
    assert!(dxf_reply(&Json::obj().with("flatPattern", fpj.clone()).with("version", "R14")).unwrap_err().contains("version"));
    assert!(bend_table_reply(&Json::obj().with("flatPattern", fpj).with("format", "xml")).unwrap_err().contains("format"));
}

/// With `CK_DXF_DUMP=<dir>` the files of this suite are written there, for a check with an outside reader.
#[test]
fn dump_files_for_outside_readers() {
    let Ok(dir) = std::env::var("CK_DXF_DUMP") else { return };
    let (_, fpj) = plate(true, false, &[(r#""height":20"#, &[("bottom", true), ("top", false)])]);
    let fp = fp_of(&fpj);
    let all = DxfOptions { bend_notes: true, tangent_lines: true, ..DxfOptions::default() };
    std::fs::write(format!("{dir}/r2000.dxf"), flat_pattern_dxf(&fp, &all).dxf).unwrap();
    std::fs::write(format!("{dir}/r12.dxf"), flat_pattern_dxf(&fp, &DxfOptions { version: DxfVersion::R12, ..all.clone() }).dxf).unwrap();
    std::fs::write(format!("{dir}/inch.dxf"), flat_pattern_dxf(&fp, &DxfOptions { units: Units::In, ..all.clone() }).dxf).unwrap();
    let (_, fpj) = plate(false, true, &[]);
    std::fs::write(format!("{dir}/arc.dxf"), flat_pattern_dxf(&fp_of(&fpj), &all).dxf).unwrap();
}
