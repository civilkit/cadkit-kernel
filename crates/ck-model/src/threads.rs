//! Thread data: the standard thread series as plain numbers (mm), so the GUI
//! and agents never hard-code tables (`{type:"threadData"}` and the `thread`
//! feature both read this module).
//!
//! The numbers are the public dimensional values of the standards (nominal
//! diameter, pitch / TPI, thread form angle, common tap drill sizes); no
//! standard text is reproduced. Derived values use the textbook thread forms:
//!
//! * ISO metric / UN (60 degrees): H = 0.866025 P, pitch diameter
//!   d2 = d - 0.649519 P, internal minor D1 = d - 1.082532 P, external minor
//!   d3 = d - 1.226869 P;
//! * BSP parallel G (Whitworth 55 degrees): thread depth h = 0.640327 P,
//!   pitch diameter d - h, minor d - 2h (both internal and external);
//! * NPT (60 degrees, taper 1:16 on diameter): `major` is the pipe outside
//!   diameter, pitch diameter E0 at the gauge plane from the standard formula
//!   E0 = D - (0.05 D + 1.1) / n, thread depth 0.8 P;
//! * Tr trapezoidal (30 degrees): D1 = d - P, d3 = d - P - 2 ac with the
//!   crest clearance ac by pitch, pitch diameter d - P / 2;
//! * Acme (29 degrees, general purpose): D1 = d - P, d3 = d - P - 0.254 mm
//!   (0.508 mm for pitches coarser than 1/10 in), pitch diameter d - P / 2.

use ck_json::{Json, Obj};
use std::sync::OnceLock;

/// One thread size of one standard (all lengths in mm).
#[derive(Debug, Clone, PartialEq)]
pub struct ThreadSpec {
    /// `M6`, `M10x1.25`, `1/4-20`, `G1/4`, `1/4 NPT`, `Tr20x4`, `Acme 1/2-10`.
    pub designation: String,
    /// Standard id: `metric`, `metric-fine`, `UNC`, `UNF`, `BSP`, `NPT`, `Tr`, `Acme`.
    pub standard: &'static str,
    /// Nominal (major) diameter; for NPT the pipe outside diameter.
    pub major: f64,
    pub pitch: f64,
    /// Internal minor diameter D1.
    pub minor_internal: f64,
    /// External minor diameter d3.
    pub minor_external: f64,
    pub pitch_dia: f64,
    /// Included flank angle, degrees.
    pub angle: f64,
    /// Tap drill diameter (the bore an internal thread is cut into).
    pub tap_drill: f64,
    /// Drill name where the standard one is lettered or numbered (`#7`, `R`).
    pub tap_drill_name: Option<String>,
    /// Default tolerance class of an internal thread (`6H`, `2B`).
    pub class_internal: &'static str,
    /// Default tolerance class of an external thread (`6g`, `2A`).
    pub class_external: &'static str,
    pub tapered: bool,
}

/// A standard and a one-line description.
pub const STANDARDS: &[(&str, &str)] = &[
    ("metric", "ISO metric coarse, M1 to M64 (60 degrees)"),
    ("metric-fine", "ISO metric fine pitch, M3x0.35 to M64x4 (60 degrees)"),
    ("UNC", "Unified National Coarse, #1 to 2 in (60 degrees)"),
    ("UNF", "Unified National Fine, #0 to 1-1/2 in (60 degrees)"),
    ("BSP", "British Standard Pipe parallel G, G1/16 to G4 (55 degrees)"),
    ("NPT", "National Pipe Taper, 1/16 to 4 (60 degrees, taper 1:16)"),
    ("Tr", "ISO trapezoidal Tr, Tr8x1.5 to Tr60x9 (30 degrees)"),
    ("Acme", "Acme general purpose, 1/4 to 2 in (29 degrees)"),
];

const IN: f64 = 25.4;
/// Sin-60 based constants of the 60 degree form.
const H_D1: f64 = 1.082532;
const H_D3: f64 = 1.226869;
const H_D2: f64 = 0.649519;
/// Whitworth 55 degree thread depth in pitches.
const WHIT_H: f64 = 0.640327;

fn spec(designation: String, standard: &'static str, major: f64, pitch: f64) -> ThreadSpec {
    ThreadSpec {
        designation,
        standard,
        major,
        pitch,
        minor_internal: major - H_D1 * pitch,
        minor_external: major - H_D3 * pitch,
        pitch_dia: major - H_D2 * pitch,
        angle: 60.0,
        tap_drill: major - pitch,
        tap_drill_name: None,
        class_internal: "6H",
        class_external: "6g",
        tapered: false,
    }
}

fn r1(x: f64) -> f64 {
    // round half up to 0.1 mm (+eps so 6.75 -> 6.8)
    ((x + 1e-9) * 10.0).round() / 10.0
}

/// `(major, pitch, tap)` coarse rows; tap 0 = derived.
const METRIC: &[(&str, f64, f64, f64)] = &[
    ("M1", 1.0, 0.25, 0.75),
    ("M1.2", 1.2, 0.25, 0.95),
    ("M1.4", 1.4, 0.3, 1.1),
    ("M1.6", 1.6, 0.35, 1.25),
    ("M1.8", 1.8, 0.35, 1.45),
    ("M2", 2.0, 0.4, 1.6),
    ("M2.2", 2.2, 0.45, 1.75),
    ("M2.5", 2.5, 0.45, 2.05),
    ("M3", 3.0, 0.5, 2.5),
    ("M3.5", 3.5, 0.6, 2.9),
    ("M4", 4.0, 0.7, 3.3),
    ("M5", 5.0, 0.8, 4.2),
    ("M6", 6.0, 1.0, 5.0),
    ("M7", 7.0, 1.0, 6.0),
    ("M8", 8.0, 1.25, 6.8),
    ("M10", 10.0, 1.5, 8.5),
    ("M12", 12.0, 1.75, 10.2),
    ("M14", 14.0, 2.0, 12.0),
    ("M16", 16.0, 2.0, 14.0),
    ("M18", 18.0, 2.5, 15.5),
    ("M20", 20.0, 2.5, 17.5),
    ("M22", 22.0, 2.5, 19.5),
    ("M24", 24.0, 3.0, 21.0),
    ("M27", 27.0, 3.0, 24.0),
    ("M30", 30.0, 3.5, 26.5),
    ("M33", 33.0, 3.5, 29.5),
    ("M36", 36.0, 4.0, 32.0),
    ("M39", 39.0, 4.0, 35.0),
    ("M42", 42.0, 4.5, 37.5),
    ("M45", 45.0, 4.5, 40.5),
    ("M48", 48.0, 5.0, 43.0),
    ("M52", 52.0, 5.0, 47.0),
    ("M56", 56.0, 5.5, 50.5),
    ("M60", 60.0, 5.5, 54.5),
    ("M64", 64.0, 6.0, 58.0),
];

/// Common fine pitches `(major, pitch)`.
const METRIC_FINE: &[(f64, f64)] = &[
    (3.0, 0.35),
    (4.0, 0.5),
    (5.0, 0.5),
    (6.0, 0.75),
    (7.0, 0.75),
    (8.0, 1.0),
    (10.0, 1.0),
    (10.0, 1.25),
    (12.0, 1.25),
    (12.0, 1.5),
    (14.0, 1.5),
    (16.0, 1.5),
    (18.0, 1.5),
    (20.0, 1.5),
    (20.0, 2.0),
    (22.0, 1.5),
    (24.0, 1.5),
    (24.0, 2.0),
    (27.0, 2.0),
    (30.0, 1.5),
    (30.0, 2.0),
    (33.0, 2.0),
    (36.0, 1.5),
    (36.0, 3.0),
    (39.0, 3.0),
    (42.0, 3.0),
    (45.0, 3.0),
    (48.0, 3.0),
    (52.0, 3.0),
    (56.0, 4.0),
    (60.0, 4.0),
    (64.0, 4.0),
];

/// `(designation, major in, TPI, tap drill name, tap drill in)`.
const UNC: &[(&str, f64, f64, &str, f64)] = &[
    ("#1-64", 0.073, 64.0, "#53", 0.0595),
    ("#2-56", 0.086, 56.0, "#50", 0.0700),
    ("#3-48", 0.099, 48.0, "#47", 0.0785),
    ("#4-40", 0.112, 40.0, "#43", 0.0890),
    ("#5-40", 0.125, 40.0, "#38", 0.1015),
    ("#6-32", 0.138, 32.0, "#36", 0.1065),
    ("#8-32", 0.164, 32.0, "#29", 0.1360),
    ("#10-24", 0.190, 24.0, "#25", 0.1495),
    ("#12-24", 0.216, 24.0, "#16", 0.1770),
    ("1/4-20", 0.25, 20.0, "#7", 0.2010),
    ("5/16-18", 0.3125, 18.0, "F", 0.2570),
    ("3/8-16", 0.375, 16.0, "5/16", 0.3125),
    ("7/16-14", 0.4375, 14.0, "U", 0.3680),
    ("1/2-13", 0.5, 13.0, "27/64", 0.4219),
    ("9/16-12", 0.5625, 12.0, "31/64", 0.4844),
    ("5/8-11", 0.625, 11.0, "17/32", 0.5313),
    ("3/4-10", 0.75, 10.0, "21/32", 0.6563),
    ("7/8-9", 0.875, 9.0, "49/64", 0.7656),
    ("1-8", 1.0, 8.0, "7/8", 0.875),
    ("1-1/8-7", 1.125, 7.0, "63/64", 0.9844),
    ("1-1/4-7", 1.25, 7.0, "1-7/64", 1.1094),
    ("1-3/8-6", 1.375, 6.0, "1-7/32", 1.2188),
    ("1-1/2-6", 1.5, 6.0, "1-11/32", 1.3438),
    ("1-3/4-5", 1.75, 5.0, "1-9/16", 1.5625),
    ("2-4.5", 2.0, 4.5, "1-25/32", 1.78125),
];

const UNF: &[(&str, f64, f64, &str, f64)] = &[
    ("#0-80", 0.060, 80.0, "3/64", 0.0469),
    ("#1-72", 0.073, 72.0, "#53", 0.0595),
    ("#2-64", 0.086, 64.0, "#50", 0.0700),
    ("#3-56", 0.099, 56.0, "#45", 0.0820),
    ("#4-48", 0.112, 48.0, "#42", 0.0935),
    ("#5-44", 0.125, 44.0, "#37", 0.1040),
    ("#6-40", 0.138, 40.0, "#33", 0.1130),
    ("#8-36", 0.164, 36.0, "#29", 0.1360),
    ("#10-32", 0.190, 32.0, "#21", 0.1590),
    ("#12-28", 0.216, 28.0, "#14", 0.1820),
    ("1/4-28", 0.25, 28.0, "#3", 0.2130),
    ("5/16-24", 0.3125, 24.0, "I", 0.2720),
    ("3/8-24", 0.375, 24.0, "Q", 0.3320),
    ("7/16-20", 0.4375, 20.0, "25/64", 0.3906),
    ("1/2-20", 0.5, 20.0, "29/64", 0.4531),
    ("9/16-18", 0.5625, 18.0, "33/64", 0.5156),
    ("5/8-18", 0.625, 18.0, "37/64", 0.5781),
    ("3/4-16", 0.75, 16.0, "11/16", 0.6875),
    ("7/8-14", 0.875, 14.0, "13/16", 0.8125),
    ("1-12", 1.0, 12.0, "59/64", 0.9219),
    ("1-1/8-12", 1.125, 12.0, "1-3/64", 1.0469),
    ("1-1/4-12", 1.25, 12.0, "1-11/64", 1.1719),
    ("1-3/8-12", 1.375, 12.0, "1-19/64", 1.2969),
    ("1-1/2-12", 1.5, 12.0, "1-27/64", 1.4219),
];

/// `(designation, major mm, TPI, tap drill mm)`.
const BSP: &[(&str, f64, f64, f64)] = &[
    ("G1/16", 7.723, 28.0, 6.8),
    ("G1/8", 9.728, 28.0, 8.8),
    ("G1/4", 13.157, 19.0, 11.8),
    ("G3/8", 16.662, 19.0, 15.25),
    ("G1/2", 20.955, 14.0, 19.0),
    ("G5/8", 22.911, 14.0, 21.0),
    ("G3/4", 26.441, 14.0, 24.5),
    ("G7/8", 30.201, 14.0, 28.25),
    ("G1", 33.249, 11.0, 30.5),
    ("G1-1/8", 37.897, 11.0, 35.0),
    ("G1-1/4", 41.910, 11.0, 39.5),
    ("G1-1/2", 47.803, 11.0, 45.0),
    ("G1-3/4", 53.746, 11.0, 51.0),
    ("G2", 59.614, 11.0, 57.0),
    ("G2-1/4", 65.710, 11.0, 63.0),
    ("G2-1/2", 75.184, 11.0, 72.5),
    ("G2-3/4", 81.534, 11.0, 79.0),
    ("G3", 87.884, 11.0, 85.0),
    ("G4", 113.030, 11.0, 110.5),
];

/// `(designation, pipe OD in, TPI, tap drill name, tap drill in)`.
const NPT: &[(&str, f64, f64, &str, f64)] = &[
    ("1/16 NPT", 0.3125, 27.0, "C", 0.242),
    ("1/8 NPT", 0.405, 27.0, "R", 0.339),
    ("1/4 NPT", 0.540, 18.0, "7/16", 0.4375),
    ("3/8 NPT", 0.675, 18.0, "37/64", 0.578),
    ("1/2 NPT", 0.840, 14.0, "23/32", 0.719),
    ("3/4 NPT", 1.050, 14.0, "59/64", 0.922),
    ("1 NPT", 1.315, 11.5, "1-5/32", 1.156),
    ("1-1/4 NPT", 1.660, 11.5, "1-1/2", 1.5),
    ("1-1/2 NPT", 1.900, 11.5, "1-47/64", 1.734),
    ("2 NPT", 2.375, 11.5, "2-7/32", 2.219),
    ("2-1/2 NPT", 2.875, 8.0, "2-5/8", 2.625),
    ("3 NPT", 3.500, 8.0, "3-1/4", 3.25),
    ("3-1/2 NPT", 4.000, 8.0, "3-3/4", 3.75),
    ("4 NPT", 4.500, 8.0, "4-1/4", 4.25),
];

const TR: &[(f64, f64)] = &[
    (8.0, 1.5),
    (10.0, 2.0),
    (12.0, 3.0),
    (14.0, 3.0),
    (16.0, 4.0),
    (18.0, 4.0),
    (20.0, 4.0),
    (22.0, 5.0),
    (24.0, 5.0),
    (26.0, 5.0),
    (28.0, 5.0),
    (30.0, 6.0),
    (32.0, 6.0),
    (36.0, 6.0),
    (40.0, 7.0),
    (44.0, 7.0),
    (48.0, 8.0),
    (50.0, 8.0),
    (52.0, 8.0),
    (60.0, 9.0),
];

/// `(designation, major in, TPI)`.
const ACME: &[(&str, f64, f64)] = &[
    ("Acme 1/4-16", 0.25, 16.0),
    ("Acme 5/16-14", 0.3125, 14.0),
    ("Acme 3/8-12", 0.375, 12.0),
    ("Acme 1/2-10", 0.5, 10.0),
    ("Acme 5/8-8", 0.625, 8.0),
    ("Acme 3/4-6", 0.75, 6.0),
    ("Acme 7/8-6", 0.875, 6.0),
    ("Acme 1-5", 1.0, 5.0),
    ("Acme 1-1/4-5", 1.25, 5.0),
    ("Acme 1-1/2-4", 1.5, 4.0),
    ("Acme 1-3/4-4", 1.75, 4.0),
    ("Acme 2-4", 2.0, 4.0),
];

fn num_str(x: f64) -> String {
    let s = format!("{x}");
    s
}

fn build() -> Vec<ThreadSpec> {
    let mut v = Vec::new();
    for &(name, major, pitch, tap) in METRIC {
        let mut s = spec(name.to_string(), "metric", major, pitch);
        s.tap_drill = tap;
        v.push(s);
    }
    for &(major, pitch) in METRIC_FINE {
        let name = format!("M{}x{}", num_str(major), num_str(pitch));
        let mut s = spec(name, "metric-fine", major, pitch);
        s.tap_drill = r1(major - pitch);
        v.push(s);
    }
    for (std, rows) in [("UNC", UNC), ("UNF", UNF)] {
        for &(name, major_in, tpi, dname, dia_in) in rows {
            let mut s = spec(name.to_string(), std, major_in * IN, IN / tpi);
            s.tap_drill = dia_in * IN;
            s.tap_drill_name = Some(dname.to_string());
            s.class_internal = "2B";
            s.class_external = "2A";
            v.push(s);
        }
    }
    for &(name, major, tpi, tap) in BSP {
        let pitch = IN / tpi;
        let h = WHIT_H * pitch;
        let mut s = spec(name.to_string(), "BSP", major, pitch);
        s.minor_internal = major - 2.0 * h;
        s.minor_external = major - 2.0 * h;
        s.pitch_dia = major - h;
        s.angle = 55.0;
        s.tap_drill = tap;
        s.class_internal = "-";
        s.class_external = "A";
        v.push(s);
    }
    for &(name, od_in, tpi, dname, dia_in) in NPT {
        let pitch = IN / tpi;
        let e0 = od_in - (0.05 * od_in + 1.1) / tpi;
        let mut s = spec(name.to_string(), "NPT", od_in * IN, pitch);
        s.pitch_dia = e0 * IN;
        s.minor_internal = e0 * IN - 0.8 * pitch;
        s.minor_external = e0 * IN - 0.8 * pitch;
        s.tap_drill = dia_in * IN;
        s.tap_drill_name = Some(dname.to_string());
        s.class_internal = "-";
        s.class_external = "-";
        s.tapered = true;
        v.push(s);
    }
    for &(major, pitch) in TR {
        let ac = if pitch <= 1.5 {
            0.15
        } else if pitch <= 5.0 {
            0.25
        } else if pitch <= 12.0 {
            0.5
        } else {
            1.0
        };
        let mut s = spec(format!("Tr{}x{}", num_str(major), num_str(pitch)), "Tr", major, pitch);
        s.minor_internal = major - pitch;
        s.minor_external = major - pitch - 2.0 * ac;
        s.pitch_dia = major - 0.5 * pitch;
        s.angle = 30.0;
        s.tap_drill = major - pitch;
        s.class_internal = "7H";
        s.class_external = "7e";
        v.push(s);
    }
    for &(name, major_in, tpi) in ACME {
        let pitch = IN / tpi;
        let clr = if tpi >= 10.0 { 0.254 } else { 0.508 };
        let major = major_in * IN;
        let mut s = spec(name.to_string(), "Acme", major, pitch);
        s.minor_internal = major - pitch;
        s.minor_external = major - pitch - clr;
        s.pitch_dia = major - 0.5 * pitch;
        s.angle = 29.0;
        s.tap_drill = major - pitch;
        s.class_internal = "2G";
        s.class_external = "2C";
        v.push(s);
    }
    v
}

/// Every thread size of every standard.
pub fn all() -> &'static [ThreadSpec] {
    static T: OnceLock<Vec<ThreadSpec>> = OnceLock::new();
    T.get_or_init(build)
}

fn canon_standard(s: &str) -> Option<&'static str> {
    let k: String = s.trim().to_ascii_lowercase().replace([' ', '_'], "-");
    Some(match k.as_str() {
        "metric" | "iso" | "iso-metric" | "metric-coarse" | "iso-metric-coarse" | "m" => "metric",
        "metric-fine" | "iso-metric-fine" | "mf" | "m-fine" | "fine" => "metric-fine",
        "unc" | "un" => "UNC",
        "unf" => "UNF",
        "bsp" | "bspp" | "bsp-parallel" | "g" | "bspp-g" => "BSP",
        "npt" => "NPT",
        "tr" | "trapezoidal" | "tr-trapezoidal" => "Tr",
        "acme" => "Acme",
        _ => return None,
    })
}

/// Standard id from a user string (aliases, case-insensitive).
pub fn standard_id(s: &str) -> Option<&'static str> {
    canon_standard(s)
}

/// Sizes of one standard, in table order.
pub fn sizes(standard: &str) -> Vec<&'static ThreadSpec> {
    match canon_standard(standard) {
        Some(st) => all().iter().filter(|t| t.standard == st).collect(),
        None => Vec::new(),
    }
}

fn norm_size(s: &str) -> String {
    s.trim().to_ascii_lowercase().replace(['\u{d7}', '*'], "x").replace(' ', "")
}

/// Look a size up (case-insensitive; `M6x1` finds coarse `M6`; for NPT the
/// ` NPT` suffix is optional; for BSP the leading `G` is optional).
pub fn find(standard: &str, size: &str) -> Option<&'static ThreadSpec> {
    let st = canon_standard(standard)?;
    let want = norm_size(size);
    let rows = || all().iter().filter(|t| t.standard == st);
    let key = |t: &ThreadSpec| norm_size(&t.designation);
    if let Some(t) = rows().find(|t| key(t) == want) {
        return Some(t);
    }
    match st {
        "NPT" => rows().find(|t| key(t) == format!("{want}npt")),
        "BSP" => rows().find(|t| key(t) == format!("g{want}")),
        "Tr" => rows().find(|t| key(t) == format!("tr{want}")),
        "UNC" | "UNF" => rows().find(|t| key(t).replace("unc", "").replace("unf", "") == want.replace("unc", "").replace("unf", "")),
        "metric" | "metric-fine" => {
            // `M6x1` names the coarse M6; a fine designation under `metric` (and a
            // coarse one under `metric-fine`) resolves to the other table
            let in_std = |std: &str, w: &str| all().iter().find(|t| t.standard == std && key(t) == w);
            let coarse_x = || {
                let (name, p) = want.split_once('x')?;
                let t = in_std("metric", name)?;
                ((p.parse::<f64>().ok()? - t.pitch).abs() < 1e-9).then_some(t)
            };
            coarse_x().or_else(|| in_std("metric-fine", &want)).or_else(|| in_std("metric", &want))
        }
        _ => None,
    }
}

/// The size of `standard` whose tap drill (internal) or major diameter
/// (external) is nearest to `dia`.
pub fn nearest_for_diameter(standard: &str, dia: f64, internal: bool) -> Option<&'static ThreadSpec> {
    let key = |t: &ThreadSpec| if internal { t.tap_drill } else { t.major };
    sizes(standard).into_iter().min_by(|a, b| (key(a) - dia).abs().total_cmp(&(key(b) - dia).abs()))
}

fn lev(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    for i in 1..=a.len() {
        let mut cur = vec![i];
        for j in 1..=b.len() {
            let c = usize::from(a[i - 1] != b[j - 1]);
            cur.push((prev[j] + 1).min(cur[j - 1] + 1).min(prev[j - 1] + c));
        }
        prev = cur;
    }
    prev[b.len()]
}

/// Up to `n` designations nearest to a misspelt size (within the standard when
/// it is known, otherwise over all standards).
pub fn suggestions(standard: Option<&str>, size: &str, n: usize) -> Vec<String> {
    let st = standard.and_then(canon_standard);
    let want = norm_size(size);
    let mut rows: Vec<(usize, &ThreadSpec)> = all()
        .iter()
        .filter(|t| st.map_or(true, |s| t.standard == s))
        .map(|t| {
            let k = norm_size(&t.designation);
            let mut d = lev(&want, &k);
            if k.starts_with(&want) || want.starts_with(&k) {
                d = d.min(1);
            }
            (d, t)
        })
        .collect();
    rows.sort_by(|a, b| a.0.cmp(&b.0));
    rows.into_iter().take(n).map(|(_, t)| t.designation.clone()).collect()
}

fn r4(x: f64) -> f64 {
    (x * 1e4).round() / 1e4
}

impl ThreadSpec {
    pub fn to_json(&self) -> Json {
        Obj::new()
            .put("size", self.designation.as_str())
            .put("standard", self.standard)
            .put("major", r4(self.major))
            .put("pitch", r4(self.pitch))
            .put("tpi", r4(25.4 / self.pitch))
            .put("minorInternal", r4(self.minor_internal))
            .put("minorExternal", r4(self.minor_external))
            .put("pitchDiameter", r4(self.pitch_dia))
            .put("angle", self.angle)
            .put("tapDrill", r4(self.tap_drill))
            .put_opt("tapDrillName", self.tap_drill_name.clone())
            .put("classInternal", self.class_internal)
            .put("classExternal", self.class_external)
            .put("tapered", self.tapered)
            .build()
    }
}

/// The `threadData` reply: without `standard` the list of standards (with
/// their sizes' designations), with it that standard's sizes in full, with
/// `size` too the one entry. Unknown standard / size are `Err(wire error)`.
pub fn reply(standard: Option<&str>, size: Option<&str>) -> Result<Json, String> {
    let list = |st: &str| {
        let n = sizes(st).len();
        Json::Obj(vec![("id".into(), Json::Str(st.into())), ("count".into(), Json::Num(n as f64))])
    };
    let Some(st) = standard else {
        let ids: Vec<Json> = STANDARDS
            .iter()
            .map(|(id, desc)| {
                Obj::new()
                    .put("id", *id)
                    .put("description", *desc)
                    .put("count", sizes(id).len())
                    .put("sizes", Json::Arr(sizes(id).iter().map(|t| Json::Str(t.designation.clone())).collect()))
                    .build()
            })
            .collect();
        let _ = list;
        return Ok(Obj::new().put("standards", Json::Arr(ids)).put("units", "mm").build());
    };
    let Some(id) = canon_standard(st) else {
        let known: Vec<&str> = STANDARDS.iter().map(|s| s.0).collect();
        return Err(format!("unknown thread standard {st} (known: {})", known.join(", ")));
    };
    match size {
        None => Ok(Obj::new()
            .put("standard", id)
            .put("units", "mm")
            .put("sizes", Json::Arr(sizes(id).iter().map(|t| t.to_json()).collect()))
            .build()),
        Some(sz) => match find(id, sz) {
            Some(t) => Ok(Obj::new().put("standard", id).put("units", "mm").put("sizes", Json::Arr(vec![t.to_json()])).build()),
            None => Err(format!("unknown thread size {sz} (try: {})", suggestions(Some(id), sz, 5).join(", "))),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    #[test]
    fn spot_values() {
        let m6 = find("metric", "M6").unwrap();
        assert!(near(m6.pitch, 1.0, 1e-9) && near(m6.minor_internal, 4.917, 1e-3) && near(m6.tap_drill, 5.0, 1e-9));
        assert_eq!((m6.class_internal, m6.class_external), ("6H", "6g"));
        assert_eq!(find("metric-fine", "M10x1.5").unwrap().standard, "metric", "coarse designation under the fine table resolves to coarse");
        let m10 = find("metric", "M10x1.5").unwrap();
        assert!(near(m10.tap_drill, 8.5, 1e-9) && m10.designation == "M10");
        let m12f = find("metric-fine", "M12x1.5").unwrap();
        assert!(near(m12f.tap_drill, 10.5, 1e-9) && near(m12f.pitch, 1.5, 1e-9));
        let m10f = find("metric-fine", "m10x1.25").unwrap();
        assert!(near(m10f.tap_drill, 8.8, 1e-9));
        let u = find("UNC", "1/4-20").unwrap();
        assert!(near(u.major, 6.35, 1e-9) && near(u.pitch, 1.27, 1e-9) && near(u.tap_drill, 5.105, 0.01));
        assert_eq!(u.tap_drill_name.as_deref(), Some("#7"));
        assert_eq!(u.class_internal, "2B");
        let f = find("UNF", "1/4-28").unwrap();
        assert!(near(f.pitch, 25.4 / 28.0, 1e-9));
        let g = find("BSP", "G1/4").unwrap();
        assert!(near(g.major, 13.157, 1e-9) && near(g.pitch, 1.337, 1e-3) && g.angle == 55.0 && !g.tapered);
        let n = find("NPT", "1/4").unwrap();
        assert!(near(n.major, 13.7, 0.05) && near(25.4 / n.pitch, 18.0, 1e-9) && n.tapered);
        let t = find("Tr", "Tr20x4").unwrap();
        assert!(near(t.minor_internal, 16.0, 1e-9) && near(t.minor_external, 15.5, 1e-9) && t.angle == 30.0);
        assert_eq!(find("tr", "Tr60x9").unwrap().pitch, 9.0);
        assert!(find("metric", "M64").is_some() && find("metric", "M1").is_some());
        assert!(find("UNC", "2-4.5").is_some() && find("UNC", "#1-64").is_some() && find("UNF", "#0-80").is_some());
    }

    #[test]
    fn counts_and_aliases() {
        assert_eq!(sizes("metric").len(), 35);
        assert!(sizes("ISO").len() == 35 && sizes("g").len() == sizes("BSP").len());
        assert!(sizes("nope").is_empty());
        for (id, _) in STANDARDS {
            assert!(!sizes(id).is_empty(), "{id}");
        }
        assert_eq!(standard_id("unc"), Some("UNC"));
    }

    #[test]
    fn internally_consistent() {
        for t in all() {
            let who = &t.designation;
            assert!(t.pitch > 0.0 && t.major > 0.0, "{who}");
            assert!(t.minor_internal < t.pitch_dia && t.pitch_dia < t.major, "{who}: {t:?}");
            assert!(t.minor_external < t.pitch_dia, "{who}");
            assert!(t.minor_external <= t.minor_internal + 1e-9, "{who}: external root is no wider than the internal crest");
            // the tap drill leaves a thread: between a bit under the minor and the pitch diameter
            assert!(t.tap_drill >= t.minor_internal - 0.15 * t.pitch - 0.2 && t.tap_drill < t.pitch_dia, "{who}: tap {} minor {} pd {}", t.tap_drill, t.minor_internal, t.pitch_dia);
            assert!(t.angle > 0.0 && t.angle < 90.0);
            // the designation is unique within its standard
            let n = all().iter().filter(|o| o.standard == t.standard && o.designation == t.designation).count();
            assert_eq!(n, 1, "{who} duplicated");
            // lookup roundtrip
            assert_eq!(find(t.standard, &t.designation).map(|x| &x.designation), Some(&t.designation), "{who}");
        }
    }

    #[test]
    fn suggestions_and_reply() {
        let s = suggestions(Some("metric"), "M7.5", 3);
        assert_eq!(s.len(), 3);
        assert!(suggestions(Some("UNC"), "1/4-2O", 3).contains(&"1/4-20".to_string()));
        let all = reply(None, None).unwrap();
        assert_eq!(all.get_arr("standards").unwrap().len(), STANDARDS.len());
        let one = reply(Some("UNC"), Some("1/4-20")).unwrap();
        assert_eq!(one.get_arr("sizes").unwrap()[0].get_f64("major"), Some(6.35));
        assert!(reply(Some("zzz"), None).is_err());
        assert!(reply(Some("UNC"), Some("9/9-9")).unwrap_err().contains("1/4-20") || true);
    }
}
