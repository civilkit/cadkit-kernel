//! Sheet metal export: the flat pattern as a DXF file and the bend table as CSV or JSON (docs/sheet-metal.md, "Export").
//!
//! Pure functions of a [`FlatPattern`] (from the regen result's `sheetMetal` entries, `flatPattern`): no body, no state.
//!
//! DXF: R2000 (`AC1015`, default) or R12 (`AC1009`), written by hand, millimetres unless `units` says otherwise
//! (`$INSUNITS`, every coordinate is scaled). Layers (all declared, also when empty): `CUT_OUTER` (outer contour),
//! `CUT_INNER` (holes and cut-outs), `BEND_UP` and `BEND_DOWN` (bend centre lines, dashed), `BEND_TANGENT` (the two
//! tangent lines of every bend, that is the two ends of the bend allowance; off by default) and `BEND_NOTES` (a text
//! `B1 UP 90%%d R2` beside each bend line; off by default). The flat pattern keeps the outlines as polylines (curves are
//! sampled); the writer recovers lines, arcs and circles from them: consecutive collinear pieces become one `LINE`, a run
//! of at least [`MIN_ARC_POINTS`] equally spaced points on one circle becomes an `ARC` (or a `CIRCLE` when it closes).
//! Inner contours are written before the outer one, then the bends, each followed by its note.
//!
//! Bend table: CSV with the columns of [`CSV_HEADER`] (numbers with 9 decimals, `\r\n` line ends) or a JSON document with
//! `bends`, `bend_zones`, `outline`, `cutouts`, `cutout_shapes`, `flat_size_mm`, `rule`, `thickness`, `k_factor`, `body`.

use super::sheet::{BendRow, FlatPattern, P};
use ck_json::{Json, Obj};

/// Fewest sample points on a circle (equally spaced) that are read back as an arc or a circle.
pub const MIN_ARC_POINTS: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DxfVersion {
    /// AutoCAD R2000 (`AC1015`): handles, subclass markers, objects section.
    R2000,
    /// AutoCAD R12 (`AC1009`): the plain old format.
    R12,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    Mm,
    Cm,
    M,
    In,
    Ft,
}

impl Units {
    pub fn parse(s: &str) -> Option<Units> {
        Some(match s.trim().to_ascii_lowercase().as_str() {
            "mm" | "millimeter" | "millimetre" | "millimeters" | "millimetres" => Units::Mm,
            "cm" | "centimeter" | "centimetre" => Units::Cm,
            "m" | "meter" | "metre" => Units::M,
            "in" | "inch" | "inches" => Units::In,
            "ft" | "foot" | "feet" => Units::Ft,
            _ => return None,
        })
    }
    /// Drawing units per millimetre.
    pub fn scale(self) -> f64 {
        match self {
            Units::Mm => 1.0,
            Units::Cm => 0.1,
            Units::M => 0.001,
            Units::In => 1.0 / 25.4,
            Units::Ft => 1.0 / 304.8,
        }
    }
    /// The `$INSUNITS` code.
    pub fn code(self) -> i32 {
        match self {
            Units::In => 1,
            Units::Ft => 2,
            Units::Mm => 4,
            Units::Cm => 5,
            Units::M => 6,
        }
    }
    fn metric(self) -> bool {
        !matches!(self, Units::In | Units::Ft)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerMode {
    /// One layer per role.
    Roles,
    /// Every entity on layer `0`.
    Single,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerNames {
    /// `CUT_OUTER`, `CUT_INNER`, `BEND_UP`, `BEND_DOWN`, `BEND_TANGENT`, `BEND_NOTES`.
    CadKit,
    /// The names the reference tool uses for the contours: `OUTLINE` and `CUT` (the bend layers are the same).
    Reference,
}

#[derive(Debug, Clone, PartialEq)]
pub struct DxfOptions {
    pub version: DxfVersion,
    pub units: Units,
    /// Bend centre lines on `BEND_UP` / `BEND_DOWN` (default true).
    pub bend_lines: bool,
    /// Two tangent lines per bend on `BEND_TANGENT` (default false).
    pub tangent_lines: bool,
    /// A note beside each bend line on `BEND_NOTES` (default false).
    pub bend_notes: bool,
    /// Text height of the notes, mm.
    pub note_height: f64,
    pub layers: LayerMode,
    pub names: LayerNames,
}

impl Default for DxfOptions {
    fn default() -> DxfOptions {
        DxfOptions {
            version: DxfVersion::R2000,
            units: Units::Mm,
            bend_lines: true,
            tangent_lines: false,
            bend_notes: false,
            note_height: 3.0,
            layers: LayerMode::Roles,
            names: LayerNames::CadKit,
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Contours: lines, arcs and circles recovered from the polylines of the flat pattern
// ---------------------------------------------------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Seg {
    Line(P, P),
    /// Centre, radius, start angle (degrees), signed sweep (degrees; positive counter-clockwise), start and end points.
    Arc { c: P, r: f64, start: f64, sweep: f64, from: P, to: P },
    Circle { c: P, r: f64 },
}

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn norm(a: P) -> f64 {
    a[0].hypot(a[1])
}

/// Circumcircle of three points (`None` when they are (nearly) collinear or the circle is huge).
fn circum(a: P, b: P, c: P) -> Option<(P, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (dot(a, a), dot(b, b), dot(c, c));
    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    let r = norm(sub(a, [ux, uy]));
    (r.is_finite() && r < 1e7).then_some(([ux, uy], r))
}

fn angle(c: P, p: P) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

fn wrap_pi(mut a: f64) -> f64 {
    use std::f64::consts::{PI, TAU};
    while a > PI {
        a -= TAU;
    }
    while a <= -PI {
        a += TAU;
    }
    a
}

#[derive(Clone, Copy)]
struct ArcMark {
    c: P,
    r: f64,
    step: f64,
}

fn same_circle(a: &ArcMark, b: &ArcMark) -> bool {
    norm(sub(a.c, b.c)) <= 1e-7 * a.r.max(1.0) && (a.r - b.r).abs() <= 1e-7 * a.r.max(1.0) && (a.step - b.step).abs() <= 1e-6
}

/// Read a closed polygon back as lines, arcs and circles (see the module doc).
pub fn recover(p: &[P]) -> Vec<Seg> {
    let n = p.len();
    if n < 2 {
        return vec![];
    }
    // edge i runs from p[i] to p[i+1]; mark[i] = the circle it lies on, when it is part of a long enough run
    let mut mark: Vec<Option<ArcMark>> = vec![None; n];
    if n >= MIN_ARC_POINTS {
        for i in 0..n {
            let (a, b, c) = (p[i], p[(i + 1) % n], p[(i + 2) % n]);
            let Some((ctr, r)) = circum(a, b, c) else { continue };
            let step = wrap_pi(angle(ctr, b) - angle(ctr, a));
            if step.abs() < 1e-9 {
                continue;
            }
            let tol = 1e-8 * r.max(1.0);
            // extend: points i, i+1, ... that stay on the circle at the same angular step
            let mut pts = 3;
            while pts <= n {
                let q0 = p[(i + pts - 1) % n];
                let prev = p[(i + pts - 2) % n];
                if (norm(sub(q0, ctr)) - r).abs() > tol || (wrap_pi(angle(ctr, q0) - angle(ctr, prev)) - step).abs() > 1e-6 {
                    break;
                }
                pts += 1;
            }
            // `pts - 1` points verified from the start; the run covers edges i .. i + pts - 2
            let points = pts - 1;
            if points >= MIN_ARC_POINTS {
                let m = ArcMark { c: ctr, r, step };
                for e in 0..points - 1 {
                    mark[(i + e) % n].get_or_insert(m);
                }
            }
        }
    }
    // a loop that is one circle
    if let Some(m0) = mark[0] {
        if mark.iter().all(|m| m.map_or(false, |m| same_circle(&m, &m0))) && (m0.step.abs() * n as f64 - std::f64::consts::TAU).abs() < 1e-6 {
            return vec![Seg::Circle { c: m0.c, r: m0.r }];
        }
    }
    // start at an edge that begins a group (its predecessor is another group or a line)
    let begins = |i: usize| -> bool {
        let prev = mark[(i + n - 1) % n];
        match (prev, mark[i]) {
            (Some(a), Some(b)) => !same_circle(&a, &b),
            (None, None) => true,
            _ => true,
        }
    };
    let start = (0..n).find(|&i| begins(i)).unwrap_or(0);
    let mut segs: Vec<Seg> = vec![];
    let mut k = 0;
    while k < n {
        let i = (start + k) % n;
        match mark[i] {
            None => {
                segs.push(Seg::Line(p[i], p[(i + 1) % n]));
                k += 1;
            }
            Some(m) => {
                let mut cnt = 1;
                while k + cnt < n && mark[(start + k + cnt) % n].map_or(false, |x| same_circle(&x, &m)) {
                    cnt += 1;
                }
                let (from, to) = (p[i], p[(i + cnt) % n]);
                segs.push(Seg::Arc { c: m.c, r: m.r, start: angle(m.c, from).to_degrees(), sweep: (m.step * cnt as f64).to_degrees(), from, to });
                k += cnt;
            }
        }
    }
    merge_lines(segs)
}

/// Join consecutive collinear lines (the loop is closed: the last may join the first).
fn merge_lines(mut segs: Vec<Seg>) -> Vec<Seg> {
    let joins = |a: &Seg, b: &Seg| -> Option<Seg> {
        if let (Seg::Line(a0, a1), Seg::Line(b0, b1)) = (a, b) {
            let (u, v) = (sub(*a1, *a0), sub(*b1, *b0));
            let (lu, lv) = (norm(u), norm(v));
            if lu > 1e-12 && lv > 1e-12 && cross(u, v).abs() <= 1e-9 * lu * lv && dot(u, v) > 0.0 {
                return Some(Seg::Line(*a0, *b1));
            }
        }
        None
    };
    let mut i = 0;
    while i + 1 < segs.len() {
        match joins(&segs[i], &segs[i + 1]) {
            Some(s) => {
                segs[i] = s;
                segs.remove(i + 1);
            }
            None => i += 1,
        }
    }
    while segs.len() > 1 {
        match joins(&segs[segs.len() - 1], &segs[0]) {
            Some(s) => {
                segs[0] = s;
                segs.pop();
            }
            None => break,
        }
    }
    segs
}

// ---------------------------------------------------------------------------------------------------------------------
// DXF
// ---------------------------------------------------------------------------------------------------------------------

/// A coordinate or dimension, 12 decimals, trailing zeros trimmed (`100.0`, `-16.523893421169`).
fn num(v: f64) -> String {
    let mut s = format!("{:.12}", v);
    while s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.push('0');
    }
    if s == "-0.0" {
        s = "0.0".into();
    }
    s
}

/// A short number for a note: up to 3 decimals, no trailing zeros (`90`, `2.5`).
fn short(v: f64) -> String {
    let mut s = format!("{:.3}", v);
    while s.ends_with('0') {
        s.pop();
    }
    if s.ends_with('.') {
        s.pop();
    }
    if s == "-0" {
        s = "0".into();
    }
    s
}

struct Layer {
    name: &'static str,
    color: i32,
    ltype: &'static str,
}

struct Writer {
    v: DxfVersion,
    out: String,
    next: u32,
    owner: String,
    counts: Vec<(String, usize)>,
}

impl Writer {
    fn g(&mut self, code: i32, value: &str) {
        self.out.push_str(&format!("{:>3}\n{}\n", code, value));
    }
    fn handle(&mut self) -> String {
        let h = format!("{:X}", self.next);
        self.next += 1;
        h
    }
    fn r2000(&self) -> bool {
        self.v == DxfVersion::R2000
    }
    /// Start an entity: `0 TYPE`, handle, owner, `100 AcDbEntity`, layer, `100 <sub>`.
    fn entity(&mut self, ty: &str, layer: &str, sub: &str) {
        self.g(0, ty);
        if self.r2000() {
            let h = self.handle();
            self.g(5, &h);
            let o = self.owner.clone();
            self.g(330, &o);
            self.g(100, "AcDbEntity");
        }
        self.g(8, layer);
        if self.r2000() {
            self.g(100, sub);
        }
        match self.counts.iter_mut().find(|c| c.0 == layer) {
            Some(c) => c.1 += 1,
            None => self.counts.push((layer.into(), 1)),
        }
    }
    fn pt(&mut self, base: i32, p: P, s: f64) {
        self.g(base, &num(p[0] * s));
        self.g(base + 10, &num(p[1] * s));
        self.g(base + 20, "0.0");
    }
    fn line(&mut self, layer: &str, a: P, b: P, s: f64) {
        self.entity("LINE", layer, "AcDbLine");
        self.pt(10, a, s);
        self.pt(11, b, s);
    }
    fn circle(&mut self, layer: &str, c: P, r: f64, s: f64) {
        self.entity("CIRCLE", layer, "AcDbCircle");
        self.pt(10, c, s);
        self.g(40, &num(r * s));
    }
    fn arc(&mut self, layer: &str, c: P, r: f64, a0: f64, a1: f64, s: f64) {
        self.entity("ARC", layer, "AcDbCircle");
        self.pt(10, c, s);
        self.g(40, &num(r * s));
        if self.r2000() {
            self.g(100, "AcDbArc");
        }
        self.g(50, &num(a0));
        self.g(51, &num(a1));
    }
    fn text(&mut self, layer: &str, at: P, height: f64, rot: f64, text: &str, s: f64) {
        self.entity("TEXT", layer, "AcDbText");
        self.pt(10, at, s);
        self.g(40, &num(height * s));
        self.g(1, text);
        self.g(50, &num(rot));
        self.g(72, "1");
        self.pt(11, at, s);
        if self.r2000() {
            self.g(100, "AcDbText");
        }
        self.g(73, "0");
    }
    fn seg(&mut self, layer: &str, sg: &Seg, s: f64) {
        match sg {
            Seg::Line(a, b) => self.line(layer, *a, *b, s),
            Seg::Circle { c, r } => self.circle(layer, *c, *r, s),
            Seg::Arc { c, r, start, sweep, .. } => {
                let (a0, a1) = if *sweep >= 0.0 { (*start, *start + *sweep) } else { (*start + *sweep, *start) };
                let wrap = |a: f64| {
                    let a = ((a * 1e8).round() / 1e8).rem_euclid(360.0);
                    if a >= 360.0 - 1e-7 {
                        0.0
                    } else {
                        a
                    }
                };
                self.arc(layer, *c, *r, wrap(a0), wrap(a1), s);
            }
        }
    }
}

fn layer_set(o: &DxfOptions) -> Vec<Layer> {
    if o.layers == LayerMode::Single {
        return vec![Layer { name: "0", color: 7, ltype: "Continuous" }];
    }
    let (outer, inner) = match o.names {
        LayerNames::CadKit => ("CUT_OUTER", "CUT_INNER"),
        LayerNames::Reference => ("OUTLINE", "CUT"),
    };
    let mut v = vec![
        Layer { name: "0", color: 7, ltype: "Continuous" },
        Layer { name: outer, color: 7, ltype: "Continuous" },
        Layer { name: inner, color: 4, ltype: "Continuous" },
        Layer { name: "BEND_UP", color: 1, ltype: "DASHED" },
        Layer { name: "BEND_DOWN", color: 5, ltype: "DASHED" },
        Layer { name: "BEND_TANGENT", color: 3, ltype: "DASHED" },
    ];
    if o.bend_notes {
        v.push(Layer { name: "BEND_NOTES", color: 8, ltype: "Continuous" });
    }
    v
}

/// Bounds `[xmin, ymin, xmax, ymax]` of the outer contour (mm).
fn bounds(fp: &FlatPattern) -> [f64; 4] {
    let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    for p in fp.outline.first().into_iter().flatten() {
        b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
    }
    if b[0] > b[2] {
        b = [0.0; 4];
    }
    b
}

/// The result of a DXF export.
#[derive(Debug, Clone, PartialEq)]
pub struct DxfOut {
    pub dxf: String,
    /// Entities per layer, in order of first use.
    pub counts: Vec<(String, usize)>,
    pub bends: usize,
    /// `[xmin, ymin, xmax, ymax]` of the outer contour, in mm.
    pub bounds: [f64; 4],
}

/// The flat pattern as a DXF file.
pub fn flat_pattern_dxf(fp: &FlatPattern, o: &DxfOptions) -> DxfOut {
    let s = o.units.scale();
    let layers = layer_set(o);
    let single = o.layers == LayerMode::Single;
    let name = |role: &'static str| -> &'static str {
        if single {
            return "0";
        }
        match (role, o.names) {
            ("outer", LayerNames::CadKit) => "CUT_OUTER",
            ("outer", LayerNames::Reference) => "OUTLINE",
            ("inner", LayerNames::CadKit) => "CUT_INNER",
            ("inner", LayerNames::Reference) => "CUT",
            ("tangent", _) => "BEND_TANGENT",
            ("notes", _) => "BEND_NOTES",
            _ => "0",
        }
    };
    let bb = bounds(fp);

    let r2000 = o.version == DxfVersion::R2000;
    let mut w = Writer { v: o.version, out: String::new(), next: 0x10, owner: String::new(), counts: vec![], };
    // fixed handles first (R2000)
    let hs: Vec<String> = (0..13).map(|_| w.handle()).collect();
    let [h_vport, h_ltype, h_layer, h_style, h_view, h_ucs, h_appid, h_dim, h_brt, h_model, h_paper, h_root, h_group] =
        <[String; 13]>::try_from(hs).unwrap();
    w.owner = h_model.clone();

    // ---- entities (their handles come after the fixed ones)
    let mut inner: Vec<Seg> = vec![];
    for l in fp.outline.iter().skip(1) {
        inner.extend(recover(l));
    }
    let outer: Vec<Seg> = fp.outline.first().map(|l| recover(l)).unwrap_or_default();
    let ln_inner = name("inner");
    for sg in &inner {
        w.seg(ln_inner, sg, s);
    }
    let ln_outer = name("outer");
    for sg in &outer {
        w.seg(ln_outer, sg, s);
    }
    for b in &fp.bends {
        let lb = if single {
            "0"
        } else if b.up {
            "BEND_UP"
        } else {
            "BEND_DOWN"
        };
        if o.bend_lines {
            w.line(lb, b.start, b.end, s);
        }
        if o.tangent_lines {
            let d = sub(b.end, b.start);
            let l = norm(d);
            if l > 1e-12 {
                let nrm = [-d[1] / l * b.allowance * 0.5, d[0] / l * b.allowance * 0.5];
                for sign in [-1.0, 1.0] {
                    let off = [nrm[0] * sign, nrm[1] * sign];
                    w.line(name("tangent"), [b.start[0] + off[0], b.start[1] + off[1]], [b.end[0] + off[0], b.end[1] + off[1]], s);
                }
            }
        }
        if o.bend_notes {
            let d = sub(b.end, b.start);
            let mut th = d[1].atan2(d[0]).to_degrees().rem_euclid(180.0);
            if (th - 180.0).abs() < 1e-9 {
                th = 0.0;
            }
            let (ct, st) = (th.to_radians().cos(), th.to_radians().sin());
            let mid = [(b.start[0] + b.end[0]) * 0.5, (b.start[1] + b.end[1]) * 0.5];
            let at = [mid[0] - st * o.note_height * 0.5, mid[1] + ct * o.note_height * 0.5];
            let txt = format!("B{} {} {}%%d R{}", b.index, if b.up { "UP" } else { "DOWN" }, short(b.angle), short(b.radius * s));
            w.text(name("notes"), at, o.note_height, th, &txt, s);
        }
    }
    let entities = std::mem::take(&mut w.out);
    let counts = std::mem::take(&mut w.counts);

    // ---- assemble
    let mut f = Writer { v: o.version, out: String::new(), next: w.next, owner: h_model.clone(), counts: vec![] };
    let ext = |p: [f64; 2]| [p[0] * s, p[1] * s];
    let (emin, emax) = (ext([bb[0], bb[1]]), ext([bb[2], bb[3]]));
    f.g(0, "SECTION");
    f.g(2, "HEADER");
    f.g(9, "$ACADVER");
    f.g(1, if r2000 { "AC1015" } else { "AC1009" });
    if r2000 {
        f.g(9, "$HANDSEED");
        f.g(5, "HANDSEED_PLACEHOLDER");
    }
    f.g(9, "$INSUNITS");
    f.g(70, &o.units.code().to_string());
    f.g(9, "$MEASUREMENT");
    f.g(70, if o.units.metric() { "1" } else { "0" });
    f.g(9, "$EXTMIN");
    f.pt(10, emin, 1.0);
    f.g(9, "$EXTMAX");
    f.pt(10, emax, 1.0);
    f.g(0, "ENDSEC");
    if r2000 {
        f.g(0, "SECTION");
        f.g(2, "CLASSES");
        f.g(0, "ENDSEC");
    }
    f.g(0, "SECTION");
    f.g(2, "TABLES");
    // tables: (name, handle, records written by closure)
    let table = |f: &mut Writer, name: &str, h: &str, count: usize| {
        f.g(0, "TABLE");
        f.g(2, name);
        if f.r2000() {
            f.g(5, h);
            f.g(330, "0");
            f.g(100, "AcDbSymbolTable");
        }
        f.g(70, &count.to_string());
    };
    let record = |f: &mut Writer, ty: &str, table_h: &str, marker: &str| {
        f.g(0, ty);
        if f.r2000() {
            let h = f.handle();
            f.g(5, &h);
            f.g(330, table_h);
            f.g(100, "AcDbSymbolTableRecord");
            f.g(100, marker);
        }
    };
    if r2000 {
        table(&mut f, "VPORT", &h_vport, 1);
        record(&mut f, "VPORT", &h_vport, "AcDbViewportTableRecord");
        f.g(2, "*Active");
        f.g(70, "0");
        f.pt(10, [0.0, 0.0], 1.0);
        f.g(11, "1.0");
        f.g(21, "1.0");
        let c = [(emin[0] + emax[0]) * 0.5, (emin[1] + emax[1]) * 0.5];
        f.g(12, &num(c[0]));
        f.g(22, &num(c[1]));
        f.g(40, &num(((emax[1] - emin[1]).max(emax[0] - emin[0]) * 1.2).max(1e-3)));
        f.g(41, "1.5");
        f.g(0, "ENDTAB");
    }
    // linetypes
    let dash = 12.7 * s;
    let gap = 6.35 * s;
    table(&mut f, "LTYPE", &h_ltype, if r2000 { 4 } else { 2 });
    let ltype_names: &[&str] = if r2000 { &["ByBlock", "ByLayer", "Continuous", "DASHED"] } else { &["Continuous", "DASHED"] };
    for n in ltype_names {
        record(&mut f, "LTYPE", &h_ltype, "AcDbLinetypeTableRecord");
        f.g(2, n);
        f.g(70, "0");
        if *n == "DASHED" {
            f.g(3, "Dashed __ __ __ __ __ __ __ __ __ __");
            f.g(72, "65");
            f.g(73, "2");
            f.g(40, &num(dash + gap));
            f.g(49, &num(dash));
            f.g(74, "0");
            f.g(49, &num(-gap));
            f.g(74, "0");
        } else {
            f.g(3, if *n == "Continuous" { "Solid line" } else { "" });
            f.g(72, "65");
            f.g(73, "0");
            f.g(40, "0.0");
        }
    }
    f.g(0, "ENDTAB");
    table(&mut f, "LAYER", &h_layer, layers.len());
    for l in &layers {
        record(&mut f, "LAYER", &h_layer, "AcDbLayerTableRecord");
        f.g(2, l.name);
        f.g(70, "0");
        f.g(62, &l.color.to_string());
        f.g(6, l.ltype);
    }
    f.g(0, "ENDTAB");
    table(&mut f, "STYLE", &h_style, 1);
    record(&mut f, "STYLE", &h_style, "AcDbTextStyleTableRecord");
    f.g(2, "Standard");
    f.g(70, "0");
    f.g(40, "0.0");
    f.g(41, "1.0");
    f.g(50, "0.0");
    f.g(71, "0");
    f.g(42, "2.5");
    f.g(3, "txt");
    f.g(4, "");
    f.g(0, "ENDTAB");
    if r2000 {
        table(&mut f, "VIEW", &h_view, 0);
        f.g(0, "ENDTAB");
        table(&mut f, "UCS", &h_ucs, 0);
        f.g(0, "ENDTAB");
        table(&mut f, "APPID", &h_appid, 1);
        record(&mut f, "APPID", &h_appid, "AcDbRegAppTableRecord");
        f.g(2, "ACAD");
        f.g(70, "0");
        f.g(0, "ENDTAB");
        table(&mut f, "DIMSTYLE", &h_dim, 1);
        f.g(100, "AcDbDimStyleTable");
        f.g(0, "DIMSTYLE");
        let h = f.handle();
        f.g(105, &h);
        f.g(330, &h_dim);
        f.g(100, "AcDbSymbolTableRecord");
        f.g(100, "AcDbDimStyleTableRecord");
        f.g(2, "Standard");
        f.g(70, "0");
        f.g(0, "ENDTAB");
        table(&mut f, "BLOCK_RECORD", &h_brt, 2);
        for (hh, n) in [(&h_model, "*Model_Space"), (&h_paper, "*Paper_Space")] {
            f.g(0, "BLOCK_RECORD");
            f.g(5, hh);
            f.g(330, &h_brt);
            f.g(100, "AcDbSymbolTableRecord");
            f.g(100, "AcDbBlockTableRecord");
            f.g(2, n);
        }
        f.g(0, "ENDTAB");
    }
    f.g(0, "ENDSEC");
    if r2000 {
        f.g(0, "SECTION");
        f.g(2, "BLOCKS");
        for (rec, n, paper) in [(&h_model, "*Model_Space", false), (&h_paper, "*Paper_Space", true)] {
            for end in [false, true] {
                f.g(0, if end { "ENDBLK" } else { "BLOCK" });
                let h = f.handle();
                f.g(5, &h);
                f.g(330, rec);
                f.g(100, "AcDbEntity");
                if paper {
                    f.g(67, "1");
                }
                f.g(8, "0");
                f.g(100, if end { "AcDbBlockEnd" } else { "AcDbBlockBegin" });
                if !end {
                    f.g(2, n);
                    f.g(70, "0");
                    f.pt(10, [0.0, 0.0], 1.0);
                    f.g(3, n);
                    f.g(1, "");
                }
            }
        }
        f.g(0, "ENDSEC");
    }
    f.g(0, "SECTION");
    f.g(2, "ENTITIES");
    f.out.push_str(&entities);
    f.g(0, "ENDSEC");
    if r2000 {
        f.g(0, "SECTION");
        f.g(2, "OBJECTS");
        f.g(0, "DICTIONARY");
        f.g(5, &h_root);
        f.g(330, "0");
        f.g(100, "AcDbDictionary");
        f.g(281, "1");
        f.g(3, "ACAD_GROUP");
        f.g(350, &h_group);
        f.g(0, "DICTIONARY");
        f.g(5, &h_group);
        f.g(330, &h_root);
        f.g(100, "AcDbDictionary");
        f.g(281, "1");
        f.g(0, "ENDSEC");
    }
    f.g(0, "EOF");
    let mut dxf = f.out;
    // the entity handles were taken before the table handles were (entities first), so the seed is above all of them
    let seed = format!("{:X}", f.next + 1);
    dxf = dxf.replace("HANDSEED_PLACEHOLDER", &seed);
    DxfOut { dxf, counts, bends: fp.bends.len(), bounds: bb }
}

// ---------------------------------------------------------------------------------------------------------------------
// Bend table
// ---------------------------------------------------------------------------------------------------------------------

pub const CSV_HEADER: &str =
    "body,rule,thickness_mm,k_factor,bend,angle_deg,direction,radius_mm,allowance_mm,length_mm,start_x_mm,start_y_mm,end_x_mm,end_y_mm,included_angle_deg";

fn csv_text(s: &str) -> String {
    format!("\"{}\"", s.replace('"', "\"\""))
}

/// The bend table as CSV: [`CSV_HEADER`], one row per bend, numbers with 9 decimals, `\r\n` line ends.
pub fn bend_table_csv(fp: &FlatPattern, body: &str) -> String {
    let f = |v: f64| {
        let s = format!("{:.9}", v);
        if s == "-0.000000000" {
            "0.000000000".to_string()
        } else {
            s
        }
    };
    let mut out = String::new();
    out.push_str(CSV_HEADER);
    out.push_str("\r\n");
    for b in &fp.bends {
        let cols = [
            csv_text(body),
            csv_text(&fp.rule),
            f(fp.thickness),
            f(fp.k_factor),
            b.index.to_string(),
            f(b.angle),
            if b.up { "up" } else { "down" }.to_string(),
            f(b.radius),
            f(b.allowance),
            f(b.length),
            f(b.start[0]),
            f(b.start[1]),
            f(b.end[0]),
            f(b.end[1]),
            f(b.included_angle),
        ];
        out.push_str(&cols.join(","));
        out.push_str("\r\n");
    }
    out
}

fn pj(p: P) -> Json {
    Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])])
}

/// The cut-outs (every loop after the outer one) as exact shapes: `{circle:{center,radius}}` or `{segments:[..]}` with
/// `{line:[a,b]}` and `{arc:{center,radius,start_deg,sweep_deg}}`.
pub fn cutout_shapes(fp: &FlatPattern) -> Vec<Json> {
    fp.outline
        .iter()
        .skip(1)
        .map(|l| {
            let segs = recover(l);
            if let [Seg::Circle { c, r }] = segs.as_slice() {
                return Obj::new().put("circle", Obj::new().put("center", pj(*c)).put("radius", *r).build()).build();
            }
            let items: Vec<Json> = segs
                .iter()
                .map(|s| match s {
                    Seg::Line(a, b) => Obj::new().put("line", Json::Arr(vec![pj(*a), pj(*b)])).build(),
                    Seg::Arc { c, r, start, sweep, .. } => Obj::new()
                        .put("arc", Obj::new().put("center", pj(*c)).put("radius", *r).put("start_deg", *start).put("sweep_deg", *sweep).build())
                        .build(),
                    Seg::Circle { c, r } => Obj::new().put("circle", Obj::new().put("center", pj(*c)).put("radius", *r).build()).build(),
                })
                .collect();
            Obj::new().put("segments", Json::Arr(items)).build()
        })
        .collect()
}

/// The bend table as a JSON value (keys in alphabetical order, as the reference writes them).
pub fn bend_table_json(fp: &FlatPattern, body: &str) -> Json {
    let bends: Vec<Json> = fp
        .bends
        .iter()
        .map(|b| {
            Obj::new()
                .put("allowance", b.allowance)
                .put("angle_deg", b.angle)
                .put("bend", b.index as f64)
                .put("direction", if b.up { "up" } else { "down" })
                .put("end", pj(b.end))
                .put("included_angle_deg", b.included_angle)
                .put("length", b.length)
                .put("radius", b.radius)
                .put("start", pj(b.start))
                .build()
        })
        .collect();
    let loops = |ls: &[Vec<P>]| Json::Arr(ls.iter().map(|l| Json::Arr(l.iter().map(|p| pj(*p)).collect())).collect());
    Obj::new()
        .put("bend_zones", Json::Arr(fp.zones.iter().map(|z| Json::Arr(z.iter().map(|v| Json::Num(*v)).collect())).collect()))
        .put("bends", Json::Arr(bends))
        .put("body", body)
        .put("cutout_shapes", Json::Arr(cutout_shapes(fp)))
        .put("cutouts", loops(fp.outline.get(1..).unwrap_or(&[])))
        .put("flat_size_mm", Json::Arr(fp.size.iter().map(|v| Json::Num(*v)).collect()))
        .put("k_factor", fp.k_factor)
        .put("outline", loops(&fp.outline[..fp.outline.len().min(1)]))
        .put("perforations", Json::Arr(vec![]))
        .put("pockets", Json::Arr(vec![]))
        .put("rule", fp.rule.as_str())
        .put("thickness", fp.thickness)
        .put("unfolded", false)
        .build()
}

/// Pretty JSON text, two-space indent; floats that are whole numbers keep a `.0` (the key `bend` is an integer).
pub fn pretty(j: &Json) -> String {
    fn go(j: &Json, ind: usize, key: &str, out: &mut String) {
        let pad = |n: usize| "  ".repeat(n);
        match j {
            Json::Num(v) if v.is_finite() => {
                let s = ck_json::fmt_f64(*v);
                out.push_str(&s);
                if key != "bend" && v.fract() == 0.0 && !s.contains('.') {
                    out.push_str(".0");
                }
            }
            Json::Arr(a) if a.is_empty() => out.push_str("[]"),
            Json::Arr(a) => {
                out.push_str("[\n");
                for (i, x) in a.iter().enumerate() {
                    out.push_str(&pad(ind + 1));
                    go(x, ind + 1, key, out);
                    if i + 1 < a.len() {
                        out.push(',');
                    }
                    out.push('\n');
                }
                out.push_str(&pad(ind));
                out.push(']');
            }
            Json::Obj(m) if m.is_empty() => out.push_str("{}"),
            Json::Obj(m) => {
                out.push_str("{\n");
                for (i, (k, x)) in m.iter().enumerate() {
                    out.push_str(&pad(ind + 1));
                    ck_json::write_string(k, out);
                    out.push_str(": ");
                    go(x, ind + 1, k, out);
                    if i + 1 < m.len() {
                        out.push(',');
                    }
                    out.push('\n');
                }
                out.push_str(&pad(ind));
                out.push('}');
            }
            other => ck_json::write_to(other, out),
        }
    }
    let mut s = String::new();
    go(j, 0, "", &mut s);
    s
}

// ---------------------------------------------------------------------------------------------------------------------
// The wire: the flat pattern from the regen result, and the two requests
// ---------------------------------------------------------------------------------------------------------------------

fn p2(j: &Json) -> Option<P> {
    let a = j.as_arr()?;
    Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?])
}

/// Read a flat pattern back from the `flatPattern` object of a `sheetMetal` entry of the regen result.
pub fn flat_pattern_from_json(j: &Json) -> Result<FlatPattern, String> {
    let bad = |what: &str| format!("flatPattern: {what}");
    let num = |k: &str| j.get_f64(k).ok_or_else(|| bad(&format!("`{k}` is missing")));
    let mut outline = vec![];
    for l in j.get_arr("outline").ok_or_else(|| bad("`outline` is missing"))? {
        let pts: Option<Vec<P>> = l.as_arr().ok_or_else(|| bad("`outline` loops are arrays"))?.iter().map(p2).collect();
        outline.push(pts.ok_or_else(|| bad("an outline point is [x, y]"))?);
    }
    let mut bends = vec![];
    for b in j.get_arr("bends").unwrap_or(&[]) {
        let g = |k: &str| b.get_f64(k).ok_or_else(|| bad(&format!("a bend has no `{k}`")));
        bends.push(BendRow {
            index: g("bend")? as usize,
            up: b.get_str("direction") != Some("down"),
            angle: b.get_f64("angleDeg").or_else(|| b.get_f64("angle_deg")).ok_or_else(|| bad("a bend has no `angleDeg`"))?,
            included_angle: b.get_f64("includedAngleDeg").or_else(|| b.get_f64("included_angle_deg")).ok_or_else(|| bad("a bend has no `includedAngleDeg`"))?,
            radius: g("radius")?,
            allowance: g("allowance")?,
            length: g("length")?,
            start: b.get("start").and_then(p2).ok_or_else(|| bad("a bend has no `start`"))?,
            end: b.get("end").and_then(p2).ok_or_else(|| bad("a bend has no `end`"))?,
            feature: b.get_str("feature").unwrap_or("").to_string(),
            edge: 0,
        });
    }
    let zones: Vec<[f64; 4]> = j
        .get_arr("bendZones")
        .unwrap_or(&[])
        .iter()
        .filter_map(|z| z.as_f64_vec().and_then(|v| <[f64; 4]>::try_from(v).ok()))
        .collect();
    let size = j.get("flatSize").and_then(|s| s.as_vec3()).unwrap_or([0.0, 0.0, num("thickness")?]);
    Ok(FlatPattern {
        rule: j.get_str("rule").unwrap_or("").to_string(),
        thickness: num("thickness")?,
        k_factor: num("kFactor")?,
        size,
        outline,
        bends,
        zones,
        unfolded: super::b(j, "unfolded"),
    })
}

fn flat_of(msg: &Json, who: &str) -> Result<FlatPattern, String> {
    let fp = msg.get("flatPattern").ok_or_else(|| format!("{who}: `flatPattern` is required (the flatPattern object of a sheetMetal entry of the regen result)"))?;
    // accept a whole sheetMetal entry too
    let fp = fp.get("flatPattern").unwrap_or(fp);
    flat_pattern_from_json(fp).map_err(|e| format!("{who}: {e}"))
}

fn opt_bool(msg: &Json, k: &str, who: &str) -> Result<Option<bool>, String> {
    match msg.get(k) {
        None | Some(Json::Null) => Ok(None),
        Some(Json::Bool(b)) => Ok(Some(*b)),
        Some(_) => Err(format!("{who}: `{k}` must be true or false")),
    }
}

/// `{type:"sheetDxf", flatPattern, version?:"R2000"|"R12", units?:"mm"|"cm"|"m"|"in"|"ft", bendLines?, tangentLines?,
/// bendNotes?, noteHeight?, layers?:"roles"|"single", layerNames?:"cadkit"|"reference"}`: `{dxf, bends, bounds, entities}`.
pub fn dxf_reply(msg: &Json) -> Result<Json, String> {
    let who = "sheetDxf";
    let fp = flat_of(msg, who)?;
    let mut o = DxfOptions::default();
    if let Some(v) = msg.get_str("version") {
        o.version = match v.trim().to_ascii_uppercase().as_str() {
            "R2000" | "2000" | "AC1015" => DxfVersion::R2000,
            "R12" | "12" | "AC1009" => DxfVersion::R12,
            _ => return Err(format!("{who}: version must be R2000 or R12, got `{v}`")),
        };
    }
    if let Some(u) = msg.get_str("units") {
        o.units = Units::parse(u).ok_or_else(|| format!("{who}: units must be mm, cm, m, in or ft, got `{u}`"))?;
    }
    if let Some(b) = opt_bool(msg, "bendLines", who)? {
        o.bend_lines = b;
    }
    if let Some(b) = opt_bool(msg, "tangentLines", who)? {
        o.tangent_lines = b;
    }
    if let Some(b) = opt_bool(msg, "bendNotes", who)? {
        o.bend_notes = b;
    }
    if let Some(h) = msg.get_f64("noteHeight") {
        if !(h > 0.0) {
            return Err(format!("{who}: noteHeight must be positive"));
        }
        o.note_height = h;
    }
    if let Some(l) = msg.get_str("layers") {
        o.layers = match l {
            "roles" => LayerMode::Roles,
            "single" => LayerMode::Single,
            _ => return Err(format!("{who}: layers must be roles or single, got `{l}`")),
        };
    }
    if let Some(l) = msg.get_str("layerNames") {
        o.names = match l {
            "cadkit" => LayerNames::CadKit,
            "reference" => LayerNames::Reference,
            _ => return Err(format!("{who}: layerNames must be cadkit or reference, got `{l}`")),
        };
    }
    let out = flat_pattern_dxf(&fp, &o);
    let mut counts = Obj::new();
    for (k, v) in &out.counts {
        counts = counts.put(k, *v as f64);
    }
    Ok(Obj::new()
        .put("dxf", out.dxf)
        .put("bends", out.bends as f64)
        .put("bounds", Json::Arr(out.bounds.iter().map(|v| Json::Num(*v)).collect()))
        .put("entities", counts.build())
        .build())
}

/// `{type:"sheetBendTable", flatPattern, format?:"csv"|"json", body?}`: `{format, text, bends}`.
pub fn bend_table_reply(msg: &Json) -> Result<Json, String> {
    let who = "sheetBendTable";
    let fp = flat_of(msg, who)?;
    let body = msg.get_str("body").unwrap_or("Body1");
    let format = msg.get_str("format").unwrap_or("csv");
    let text = match format {
        "csv" => bend_table_csv(&fp, body),
        "json" => pretty(&bend_table_json(&fp, body)),
        _ => return Err(format!("{who}: format must be csv or json, got `{format}`")),
    };
    Ok(Obj::new().put("format", format).put("text", text).put("bends", fp.bends.len() as f64).build())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn circle_pts(c: P, r: f64, n: usize, a0: f64, span: f64, closed: bool) -> Vec<P> {
        let m = if closed { n } else { n + 1 };
        (0..m).map(|k| {
            let a = a0 + span * k as f64 / n as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        }).collect()
    }

    #[test]
    fn full_circle_is_one_circle() {
        let segs = recover(&circle_pts([5.0, 6.0], 2.5, 32, 0.0, std::f64::consts::TAU, true));
        assert_eq!(segs.len(), 1);
        match segs[0] {
            Seg::Circle { c, r } => {
                assert!((c[0] - 5.0).abs() < 1e-9 && (c[1] - 6.0).abs() < 1e-9 && (r - 2.5).abs() < 1e-9);
            }
            _ => panic!("circle expected"),
        }
    }

    #[test]
    fn rectangle_with_rounded_corner() {
        // (0,0) (10,0) (10,6) arc to (8,8) (0,8)
        let mut p = vec![[0.0, 0.0], [10.0, 0.0]];
        p.extend(circle_pts([8.0, 6.0], 2.0, 32, 0.0, std::f64::consts::FRAC_PI_2, false));
        p.push([0.0, 8.0]);
        let segs = recover(&p);
        assert_eq!(segs.iter().filter(|s| matches!(s, Seg::Line(..))).count(), 4);
        let arcs: Vec<_> = segs.iter().filter(|s| matches!(s, Seg::Arc { .. })).collect();
        assert_eq!(arcs.len(), 1);
        if let Seg::Arc { c, r, start, sweep, .. } = arcs[0] {
            assert!((c[0] - 8.0).abs() < 1e-9 && (r - 2.0).abs() < 1e-9 && start.abs() < 1e-9 && (sweep - 90.0).abs() < 1e-9);
        }
    }

    #[test]
    fn collinear_pieces_join() {
        let p = [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0], [10.0, 5.0], [10.0, 10.0], [0.0, 10.0]];
        let segs = recover(&p);
        assert_eq!(segs.len(), 4);
    }

    #[test]
    fn small_polygons_stay_lines() {
        // a hexagon on a circle is a polygon, not a circle
        let segs = recover(&circle_pts([0.0, 0.0], 5.0, 6, 0.0, std::f64::consts::TAU, true));
        assert_eq!(segs.len(), 6);
    }

    #[test]
    fn number_formats() {
        assert_eq!(num(100.0), "100.0");
        assert_eq!(num(-16.523893421169305), "-16.523893421169");
        assert_eq!(num(-0.0), "0.0");
        assert_eq!(short(90.0), "90");
        assert_eq!(short(2.5), "2.5");
    }
}
