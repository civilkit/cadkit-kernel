//! ck-text: lettering for sketches.
//!
//! Two OFL fonts are baked into the binary (subsets of Barlow Condensed as `gauge` and PT Serif as
//! `engrave`, see `fonts/` and THIRD_PARTY.md). A string is laid out into closed loops: straight
//! parts are lines, curved parts are quadratic B-spline runs (exactly the TrueType outline, no
//! flattening) or, with `polyline`, lines within a chord tolerance. Loops are classified into
//! regions (outer boundary CCW, holes CW) by nesting, so `O`, `8`, `B`, `%` come out as faces with
//! islands. No system fonts are read: the same request gives the same geometry on every machine.
//!
//! Layout rules (sketch units, y up):
//! * `height` is the cap height (the height of `H`); glyph scale = height / cap height of the font.
//! * advances come from `hmtx` times `spacing`; no kerning.
//! * horizontal anchor refers to the INK box of the string (left edge of the first glyph, right
//!   edge of the last); vertical anchor: `baseline` = 0, `middle` = height / 2, `top` = height.
//! * `angle` rotates counter-clockwise about the anchor point `at`. `mirror` flips x about the anchor.
//! * `arc` lays each glyph on a circle (glyph upright relative to the circle): `outside` = glyph
//!   tops point away from the centre and the text reads clockwise (a title over a dial);
//!   `inside` = tops point at the centre and the text reads counter-clockwise (a title under a dial).

mod ttf;

pub use ttf::Font;
use std::sync::OnceLock;

pub type P = [f64; 2];

pub const MAX_CHARS: usize = 500;
/// Below this cap height (sketch units, mm) the text is flagged `textTooSmall`.
pub const MIN_HEIGHT: f64 = 0.8;
/// Longest run of off-curve points kept in one B-spline edge.
const RUN_SPANS: usize = 4;
/// Corner roundings below this fraction of the cap height are made sharp.
const SHARP_FRAC: f64 = 0.03;

static GAUGE: &[u8] = include_bytes!("../fonts/gauge.ttf");
static ENGRAVE: &[u8] = include_bytes!("../fonts/engrave.ttf");

/// The font behind a name (`gauge`, `engrave`; aliases `sans`, `default`, `serif`).
pub fn font(name: &str) -> Option<&'static Font> {
    static F: OnceLock<[Option<Font>; 2]> = OnceLock::new();
    let f = F.get_or_init(|| [Font::parse(GAUGE), Font::parse(ENGRAVE)]);
    match name {
        "" | "gauge" | "sans" | "default" => f[0].as_ref(),
        "engrave" | "serif" => f[1].as_ref(),
        _ => None,
    }
}

pub const FONT_NAMES: [&str; 2] = ["gauge", "engrave"];

// ---------------------------------------------------------------------------------------------
// edges

/// One boundary piece of a loop.
#[derive(Clone, Debug, PartialEq)]
pub enum Edge {
    Line([P; 2]),
    /// Quadratic B-spline, clamped: control points `q0..q(m+1)` with `m >= 1` spans. The first and
    /// last point are on the curve, the others are off-curve (consecutive spans join at the middle
    /// of two neighbouring control points). Knots `[0,0,0,1,2,..,m-1,m,m,m]`, domain `[0, m]`.
    Quad(Vec<P>),
}

fn mid(a: P, b: P) -> P {
    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]
}

impl Edge {
    pub fn start(&self) -> P {
        match self {
            Edge::Line(l) => l[0],
            Edge::Quad(q) => q[0],
        }
    }
    pub fn end(&self) -> P {
        match self {
            Edge::Line(l) => l[1],
            Edge::Quad(q) => q[q.len() - 1],
        }
    }
    pub fn reversed(&self) -> Edge {
        match self {
            Edge::Line(l) => Edge::Line([l[1], l[0]]),
            Edge::Quad(q) => Edge::Quad(q.iter().rev().copied().collect()),
        }
    }
    /// The knot vector of a `Quad` (degree 2).
    pub fn knots(&self) -> Vec<f64> {
        let Edge::Quad(q) = self else { return vec![] };
        let m = q.len() - 2;
        let mut k = vec![0.0, 0.0, 0.0];
        for i in 1..m {
            k.push(i as f64);
        }
        k.extend([m as f64; 3]);
        k
    }
    fn map(&self, f: &impl Fn(P) -> P) -> Edge {
        match self {
            Edge::Line(l) => Edge::Line([f(l[0]), f(l[1])]),
            Edge::Quad(q) => Edge::Quad(q.iter().map(|p| f(*p)).collect()),
        }
    }
    /// Quadratic Bezier spans `(p0, c, p1)` of a `Quad`.
    fn spans(q: &[P]) -> Vec<(P, P, P)> {
        let m = q.len() - 2;
        (0..m)
            .map(|k| {
                let a = if k == 0 { q[0] } else { mid(q[k], q[k + 1]) };
                let b = if k == m - 1 { q[m + 1] } else { mid(q[k + 1], q[k + 2]) };
                (a, q[k + 1], b)
            })
            .collect()
    }
    /// Points along the edge after its start, `n` per span.
    fn flatten(&self, n: usize, out: &mut Vec<P>) {
        match self {
            Edge::Line(l) => out.push(l[1]),
            Edge::Quad(q) => {
                for (a, c, b) in Edge::spans(q) {
                    for i in 1..=n {
                        let t = i as f64 / n as f64;
                        let (u, v, w) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t);
                        out.push([u * a[0] + v * c[0] + w * b[0], u * a[1] + v * c[1] + w * b[1]]);
                    }
                }
            }
        }
    }
}

/// A loop as a polygon (`n` points per curved span, closing point not repeated).
pub fn loop_polygon(l: &[Edge], n: usize) -> Vec<P> {
    let mut pts = vec![l[0].start()];
    for e in l {
        e.flatten(n, &mut pts);
    }
    pts.pop();
    pts
}

fn signed_area(p: &[P]) -> f64 {
    let mut s = 0.0;
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        s += a[0] * b[1] - b[0] * a[1];
    }
    s * 0.5
}

fn inside(poly: &[P], q: P) -> bool {
    let mut c = false;
    let mut j = poly.len() - 1;
    for i in 0..poly.len() {
        let (a, b) = (poly[i], poly[j]);
        if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < (b[0] - a[0]) * (q[1] - a[1]) / (b[1] - a[1]) + a[0] {
            c = !c;
        }
        j = i;
    }
    c
}

/// A filled area: outer boundary counter-clockwise, holes clockwise.
#[derive(Clone, Debug)]
pub struct Region {
    pub outer: Vec<Edge>,
    pub holes: Vec<Vec<Edge>>,
    /// Index of the character (in the laid-out string, skipped characters not counted) it belongs to.
    pub glyph: usize,
}

impl Region {
    pub fn loops(&self) -> impl Iterator<Item = &Vec<Edge>> {
        std::iter::once(&self.outer).chain(self.holes.iter())
    }
    /// Filled area (outer minus holes), from fine flattening.
    pub fn area(&self) -> f64 {
        let a = |l: &Vec<Edge>| signed_area(&loop_polygon(l, 64));
        a(&self.outer) + self.holes.iter().map(a).sum::<f64>()
    }
}

// ---------------------------------------------------------------------------------------------
// request

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HAnchor {
    Left,
    Center,
    Right,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VAnchor {
    Baseline,
    Middle,
    Top,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Side {
    Outside,
    Inside,
}

#[derive(Clone, Debug)]
pub struct ArcSpec {
    pub centre: P,
    /// Radius of the baseline-anchor line (the radius the anchor point sits on).
    pub radius: f64,
    /// Direction (degrees, CCW from +x) of the middle of the text as seen from the centre.
    pub angle_centre: f64,
    pub side: Side,
}

#[derive(Clone, Debug)]
pub struct Spec {
    pub string: String,
    pub font: String,
    pub height: f64,
    pub spacing: f64,
    pub at: P,
    pub h: HAnchor,
    pub v: VAnchor,
    pub angle: f64,
    pub mirror: bool,
    pub arc: Option<ArcSpec>,
    /// Lines within a chord tolerance instead of B-spline edges.
    pub polyline: bool,
}

impl Default for Spec {
    fn default() -> Spec {
        Spec {
            string: String::new(),
            font: "gauge".into(),
            height: 5.0,
            spacing: 1.0,
            at: [0.0, 0.0],
            h: HAnchor::Left,
            v: VAnchor::Baseline,
            angle: 0.0,
            mirror: false,
            arc: None,
            polyline: false,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Warning {
    pub code: &'static str,
    pub detail: String,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    /// `textEmpty`
    Empty,
    /// `textNothingToDraw`: only spaces / unknown characters
    NothingToDraw,
    /// `textBadFont`
    UnknownFont(String),
    /// `textBadHeight`
    BadHeight,
    /// `textBadArc`
    BadArc,
    /// `textBadValue`: a position / angle / spacing that is not a finite number
    BadValue,
}

impl Error {
    pub fn code(&self) -> &'static str {
        match self {
            Error::Empty => "textEmpty",
            Error::NothingToDraw => "textNothingToDraw",
            Error::UnknownFont(_) => "textBadFont",
            Error::BadHeight => "textBadHeight",
            Error::BadArc => "textBadArc",
            Error::BadValue => "textBadValue",
        }
    }
    pub fn message(&self) -> String {
        match self {
            Error::Empty => "the text string is empty".into(),
            Error::NothingToDraw => "the text has no visible characters".into(),
            Error::UnknownFont(f) => format!("unknown font {f:?}; use one of gauge, engrave"),
            Error::BadHeight => "text height must be a positive number".into(),
            Error::BadArc => "text on an arc needs a positive radius".into(),
            Error::BadValue => "text position, angle and spacing must be finite numbers (spacing 0.2 to 5)".into(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Layout {
    pub regions: Vec<Region>,
    pub warnings: Vec<Warning>,
    /// Ink box in the final (sketch) coordinates `[xmin, ymin, xmax, ymax]`.
    pub bbox: [f64; 4],
}

impl Layout {
    pub fn area(&self) -> f64 {
        self.regions.iter().map(|r| r.area()).sum()
    }
}

#[derive(Clone, Debug)]
pub struct Metrics {
    /// Sum of the advances (x of the pen after the last character), sketch units.
    pub advance: f64,
    /// Ink box relative to the pen start on the baseline: `[xmin, ymin, xmax, ymax]`.
    pub bbox: [f64; 4],
    pub cap_height: f64,
    /// Advance of each drawn character, in order.
    pub advances: Vec<f64>,
    pub chars: usize,
    pub warnings: Vec<Warning>,
}

// ---------------------------------------------------------------------------------------------
// outlines

/// Closed loops of one glyph, font units.
fn glyph_loops(font: &Font, g: u16, polyline_tol: Option<f64>) -> Vec<Vec<Edge>> {
    let sharp = SHARP_FRAC * font.cap_height();
    let Some(contours) = font.contours(g) else { return vec![] };
    let mut out = vec![];
    for c in contours {
        if c.len() < 3 {
            continue;
        }
        if let Some(l) = contour_edges(&c, sharp) {
            let l = match polyline_tol {
                Some(tol) => polylined(&l, tol),
                None => l,
            };
            if l.len() >= 2 {
                out.push(l);
            }
        }
    }
    out
}

fn contour_edges(c: &[ttf::Pt], sharp: f64) -> Option<Vec<Edge>> {
    // start at an on-curve point (a closed run of off-curve points starts at an implied one)
    let mut pts: Vec<(P, bool)> = c.iter().map(|p| ([p.x, p.y], p.on)).collect();
    let n = pts.len();
    match pts.iter().position(|p| p.1) {
        Some(k) => pts.rotate_left(k),
        None => {
            let m = mid(pts[0].0, pts[1].0);
            pts.rotate_left(1);
            pts.insert(0, (m, true));
            pts.push((m, true));
            let _ = n;
        }
    }
    pts.push(pts[0]);
    let mut edges = vec![];
    let mut i = 0;
    while i + 1 < pts.len() {
        let start = pts[i].0;
        let mut offs = vec![];
        let mut j = i + 1;
        while j < pts.len() && !pts[j].1 {
            offs.push(pts[j].0);
            j += 1;
        }
        if j >= pts.len() {
            return None;
        }
        let end = pts[j].0;
        if offs.is_empty() {
            if start != end {
                edges.push(Edge::Line([start, end]));
            }
        } else {
            // chunk long runs; chunk ends are the implied on-curve points between off-curve ones
            let m = offs.len();
            let mut a = 0;
            while a < m {
                let b = (a + RUN_SPANS).min(m);
                let s = if a == 0 { start } else { mid(offs[a - 1], offs[a]) };
                let e = if b == m { end } else { mid(offs[b - 1], offs[b]) };
                let mut q = vec![s];
                q.extend_from_slice(&offs[a..b]);
                q.push(e);
                if q.len() == 3 && collinear(q[0], q[1], q[2]) {
                    edges.push(Edge::Line([q[0], q[2]]));
                } else {
                    edges.push(Edge::Quad(q));
                }
                a = b;
            }
        }
        i = j;
    }
    Some(merge_lines(sharpen(edges, sharp)))
}

/// A one-span curve smaller than `tol` between two lines is a corner rounding (Barlow rounds every
/// stroke corner by about 1.4 % of the cap height): such a sliver is a face too small to engrave, so
/// the corner becomes sharp (the control point is where the two tangents meet).
fn sharpen(edges: Vec<Edge>, tol: f64) -> Vec<Edge> {
    let n = edges.len();
    let mut e = edges;
    for i in 0..n {
        let (pi, ni) = ((i + n - 1) % n, (i + 1) % n);
        let Edge::Quad(q) = &e[i] else { continue };
        if q.len() != 3 || !matches!((&e[pi], &e[ni]), (Edge::Line(_), Edge::Line(_))) {
            continue;
        }
        let c = q[1];
        let far = |a: P| (a[0] - c[0]).hypot(a[1] - c[1]);
        if far(q[0]) > tol || far(q[2]) > tol {
            continue;
        }
        if let Edge::Line(l) = &mut e[pi] {
            l[1] = c;
        }
        if let Edge::Line(l) = &mut e[ni] {
            l[0] = c;
        }
        e[i] = Edge::Line([c, c]);
    }
    e.into_iter().filter(|x| !matches!(x, Edge::Line(l) if l[0] == l[1])).collect()
}

/// Consecutive collinear lines in the same direction become one line.
fn merge_lines(edges: Vec<Edge>) -> Vec<Edge> {
    let mut out: Vec<Edge> = vec![];
    for e in edges {
        if let (Some(Edge::Line(a)), Edge::Line(b)) = (out.last_mut(), &e) {
            let d = (a[1][0] - a[0][0]) * (b[1][0] - b[0][0]) + (a[1][1] - a[0][1]) * (b[1][1] - b[0][1]);
            if d > 0.0 && collinear(a[0], a[1], b[1]) {
                a[1] = b[1];
                continue;
            }
        }
        out.push(e);
    }
    if out.len() > 2 {
        let (first, last) = (out[0].clone(), out[out.len() - 1].clone());
        if let (Edge::Line(a), Edge::Line(b)) = (&last, &first) {
            let d = (a[1][0] - a[0][0]) * (b[1][0] - b[0][0]) + (a[1][1] - a[0][1]) * (b[1][1] - b[0][1]);
            if d > 0.0 && collinear(a[0], a[1], b[1]) {
                out.pop();
                out[0] = Edge::Line([a[0], b[1]]);
            }
        }
    }
    out
}

fn collinear(a: P, c: P, b: P) -> bool {
    let (ux, uy, vx, vy) = (b[0] - a[0], b[1] - a[1], c[0] - a[0], c[1] - a[1]);
    let l = ux.hypot(uy);
    l < 1e-9 || (ux * vy - uy * vx).abs() / l < 1e-6
}

fn polylined(l: &[Edge], tol: f64) -> Vec<Edge> {
    let mut out = vec![];
    for e in l {
        match e {
            Edge::Line(_) => out.push(e.clone()),
            Edge::Quad(q) => {
                let mut cur = q[0];
                for (a, c, b) in Edge::spans(q) {
                    let dev = ((a[0] - 2.0 * c[0] + b[0]).hypot(a[1] - 2.0 * c[1] + b[1])) / 4.0;
                    let n = ((dev / tol).sqrt().ceil() as usize).clamp(1, 32);
                    let mut pts = vec![];
                    Edge::Quad(vec![a, c, b]).flatten(n, &mut pts);
                    for p in pts {
                        if p != cur {
                            out.push(Edge::Line([cur, p]));
                            cur = p;
                        }
                    }
                }
            }
        }
    }
    out
}

struct Glyph {
    loops: Vec<Vec<Edge>>,
    pen: f64,
    adv: f64,
}

struct Run {
    glyphs: Vec<Glyph>,
    advances: Vec<f64>,
    total: f64,
    ink: [f64; 4],
    scale: f64,
    warnings: Vec<Warning>,
}

fn prepare(string: &str, font_name: &str, height: f64, spacing: f64, polyline: bool) -> Result<Run, Error> {
    if string.is_empty() {
        return Err(Error::Empty);
    }
    if !(height.is_finite() && height > 0.0 && height < 1e5) {
        return Err(Error::BadHeight);
    }
    if !(spacing.is_finite() && spacing > 0.0 && spacing < 20.0) {
        return Err(Error::BadValue);
    }
    let font = font(font_name).ok_or_else(|| Error::UnknownFont(font_name.to_string()))?;
    let scale = height / font.cap_height();
    let mut warnings = vec![];
    if height < MIN_HEIGHT {
        warnings.push(Warning {
            code: "textTooSmall",
            detail: format!("cap height {height} is below {MIN_HEIGHT}: strokes thinner than about 0.1 mm will not engrave or print; increase the height"),
        });
    }
    let mut chars: Vec<char> = string.chars().collect();
    if chars.len() > MAX_CHARS {
        warnings.push(Warning { code: "textTruncated", detail: format!("only the first {MAX_CHARS} of {} characters are used", chars.len()) });
        chars.truncate(MAX_CHARS);
    }
    let tol = polyline.then_some(0.005 * font.cap_height());
    let (mut pen, mut missing) = (0.0, String::new());
    let mut glyphs = vec![];
    let mut advances = vec![];
    let mut ink = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    for ch in chars {
        let Some(g) = font.glyph_index(ch) else {
            if !missing.contains(ch) {
                missing.push(ch);
            }
            continue;
        };
        let adv = font.advance(g);
        let loops = glyph_loops(font, g, tol);
        for l in &loops {
            for e in l {
                let mut pts = vec![e.start()];
                e.flatten(8, &mut pts);
                for p in pts {
                    ink[0] = ink[0].min(p[0] + pen);
                    ink[2] = ink[2].max(p[0] + pen);
                    ink[1] = ink[1].min(p[1]);
                    ink[3] = ink[3].max(p[1]);
                }
            }
        }
        glyphs.push(Glyph { loops, pen, adv });
        advances.push(adv * spacing * scale);
        pen += adv * spacing;
    }
    if !missing.is_empty() {
        warnings.push(Warning { code: "textMissingGlyph", detail: format!("no glyph for {missing:?} in font {font_name}; skipped") });
    }
    if ink[0] > ink[2] {
        return Err(Error::NothingToDraw);
    }
    Ok(Run { glyphs, advances, total: pen, ink, scale, warnings })
}

/// Advance width and ink box of a string (no placement).
pub fn metrics(string: &str, font_name: &str, height: f64, spacing: f64) -> Result<Metrics, Error> {
    if !(spacing.is_finite() && spacing > 0.0 && spacing < 20.0) {
        return Err(Error::BadValue);
    }
    let r = match prepare(string, font_name, height, spacing, false) {
        Ok(r) => r,
        Err(Error::NothingToDraw) => {
            // only blanks: still has an advance
            let font = font(font_name).ok_or_else(|| Error::UnknownFont(font_name.to_string()))?;
            let s = height / font.cap_height();
            let adv: Vec<f64> = string.chars().filter_map(|c| font.glyph_index(c)).map(|g| font.advance(g) * spacing * s).collect();
            return Ok(Metrics { advance: adv.iter().sum(), bbox: [0.0; 4], cap_height: height, chars: adv.len(), advances: adv, warnings: vec![] });
        }
        Err(e) => return Err(e),
    };
    let s = r.scale;
    Ok(Metrics {
        advance: r.total * s,
        bbox: [r.ink[0] * s, r.ink[1] * s, r.ink[2] * s, r.ink[3] * s],
        cap_height: height,
        chars: r.glyphs.len(),
        advances: r.advances,
        warnings: r.warnings,
    })
}

/// Lay a string out as regions.
pub fn layout(spec: &Spec) -> Result<Layout, Error> {
    let run = prepare(&spec.string, &spec.font, spec.height, spec.spacing, spec.polyline)?;
    let fin = |v: f64| v.is_finite() && v.abs() < 1e9;
    if !(fin(spec.at[0]) && fin(spec.at[1]) && fin(spec.angle) && spec.spacing.is_finite() && spec.spacing > 0.0 && spec.spacing < 20.0) {
        return Err(Error::BadValue);
    }
    if let Some(a) = &spec.arc {
        if !(fin(a.centre[0]) && fin(a.centre[1]) && fin(a.angle_centre)) {
            return Err(Error::BadValue);
        }
        if !(a.radius.is_finite() && a.radius > 0.0 && a.radius < 1e9) {
            return Err(Error::BadArc);
        }
    }
    let mut warnings = run.warnings.clone();
    let (mut regions, mut bbox) = place(spec, &run, 1.0)?;
    let mut over = overlapping(&regions);
    if over && spec.arc.is_some() {
        // letters on a tight circle lean towards each other (inside) or their feet do (outside): open the pitch
        for step in 1..=15 {
            let pitch = 1.0 + 0.04 * step as f64;
            (regions, bbox) = place(spec, &run, pitch)?;
            if !overlapping(&regions) {
                over = false;
                warnings.push(Warning { code: "textSpacingAdjusted", detail: format!("letters on a radius-{} arc would overlap; the pitch was widened by {:.0} %", spec.arc.as_ref().map_or(0.0, |a| a.radius), (pitch - 1.0) * 100.0) });
                break;
            }
        }
    }
    if over {
        warnings.push(Warning { code: "textOverlap", detail: "neighbouring letters overlap (spacing too tight or radius too small), which the booleans cannot merge; increase the spacing, the radius or reduce the height".into() });
    }
    Ok(Layout { regions, warnings, bbox })
}

fn overlapping(regions: &[Region]) -> bool {
    let polys: Vec<(usize, [f64; 4], Vec<Vec<P>>)> = regions
        .iter()
        .map(|r| {
            let ls: Vec<Vec<P>> = std::iter::once(&r.outer).map(|l| loop_polygon(l, 6)).collect();
            let mut b = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
            for p in ls.iter().flatten() {
                b = [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])];
            }
            (r.glyph, b, ls)
        })
        .collect();
    for i in 0..polys.len() {
        for j in i + 1..polys.len() {
            let (a, b) = (&polys[i], &polys[j]);
            if a.0 == b.0 || a.1[0] > b.1[2] || b.1[0] > a.1[2] || a.1[1] > b.1[3] || b.1[1] > a.1[3] {
                continue;
            }
            let (pa, pb) = (&a.2[0], &b.2[0]);
            if inside(pa, pb[0]) || inside(pb, pa[0]) {
                return true;
            }
            for x in 0..pa.len() {
                let (p0, p1) = (pa[x], pa[(x + 1) % pa.len()]);
                for y in 0..pb.len() {
                    let (q0, q1) = (pb[y], pb[(y + 1) % pb.len()]);
                    let o = |p: P, q: P, r: P| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
                    if o(p0, p1, q0) * o(p0, p1, q1) < 0.0 && o(q0, q1, p0) * o(q0, q1, p1) < 0.0 {
                        return true;
                    }
                }
            }
        }
    }
    false
}

/// Put the glyphs of a run in place (`pitch` widens the arc layout's letter pitch).
fn place(spec: &Spec, run: &Run, pitch: f64) -> Result<(Vec<Region>, [f64; 4]), Error> {
    let s = run.scale;
    let m = if spec.mirror { -1.0 } else { 1.0 };
    let ax = match spec.h {
        HAnchor::Left => run.ink[0],
        HAnchor::Center => (run.ink[0] + run.ink[2]) * 0.5,
        HAnchor::Right => run.ink[2],
    } * s;
    let ay = match spec.v {
        VAnchor::Baseline => 0.0,
        VAnchor::Middle => spec.height * 0.5,
        VAnchor::Top => spec.height,
    };
    let mut regions = vec![];
    let mut bbox = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
    let (ca, sa) = (spec.angle.to_radians().cos(), spec.angle.to_radians().sin());
    for (gi, g) in run.glyphs.iter().enumerate() {
        // world = o + ex * (x0 + m * s * p.x) + ey * (y0 + s * p.y)
        let (o, ex, ey, x0, y0);
        match &spec.arc {
            None => {
                o = spec.at;
                ex = [ca, sa];
                ey = [-sa, ca];
                x0 = m * (g.pen * s - ax);
                y0 = -ay;
            }
            Some(a) => {
                let gx = pitch * m * ((g.pen + g.adv * 0.5) * s - ax);
                let (rb, th, e1, e2);
                match a.side {
                    Side::Outside => {
                        rb = a.radius - ay;
                        th = a.angle_centre.to_radians() - gx / rb.max(1e-9);
                        e1 = [th.sin(), -th.cos()];
                        e2 = [th.cos(), th.sin()];
                    }
                    Side::Inside => {
                        rb = a.radius + ay;
                        th = a.angle_centre.to_radians() + gx / rb.max(1e-9);
                        e1 = [-th.sin(), th.cos()];
                        e2 = [-th.cos(), -th.sin()];
                    }
                }
                if rb <= 0.0 {
                    return Err(Error::BadArc);
                }
                o = [a.centre[0] + rb * th.cos(), a.centre[1] + rb * th.sin()];
                ex = e1;
                ey = e2;
                x0 = -m * g.adv * 0.5 * s;
                y0 = 0.0;
            }
        }
        let f = |p: P| -> P {
            let (x, y) = (x0 + m * s * p[0], y0 + s * p[1]);
            [o[0] + ex[0] * x + ey[0] * y, o[1] + ex[1] * x + ey[1] * y]
        };
        let loops: Vec<Vec<Edge>> = g.loops.iter().map(|l| l.iter().map(|e| e.map(&f)).collect()).collect();
        let rs = classify(loops, gi);
        for r in &rs {
            for l in r.loops() {
                for p in loop_polygon(l, 4) {
                    bbox[0] = bbox[0].min(p[0]);
                    bbox[1] = bbox[1].min(p[1]);
                    bbox[2] = bbox[2].max(p[0]);
                    bbox[3] = bbox[3].max(p[1]);
                }
            }
        }
        regions.extend(rs);
    }
    Ok((regions, bbox))
}

/// Group the loops of one glyph into regions by nesting; outer boundaries CCW, holes CW.
fn classify(loops: Vec<Vec<Edge>>, glyph: usize) -> Vec<Region> {
    let loops: Vec<Vec<Edge>> = loops
        .into_iter()
        .filter(|l| l.len() >= 2 && signed_area(&loop_polygon(l, 4)).abs() > 1e-12)
        .collect();
    let polys: Vec<Vec<P>> = loops.iter().map(|l| loop_polygon(l, 12)).collect();
    let n = loops.len();
    let contains = |i: usize, j: usize| i != j && inside(&polys[i], polys[j][0]);
    let depth: Vec<usize> = (0..n).map(|j| (0..n).filter(|&i| contains(i, j)).count()).collect();
    let orient = |l: &Vec<Edge>, ccw: bool| -> Vec<Edge> {
        let a = signed_area(&loop_polygon(l, 4));
        if (a > 0.0) == ccw {
            l.clone()
        } else {
            l.iter().rev().map(|e| e.reversed()).collect()
        }
    };
    let mut regions: Vec<Region> = vec![];
    let mut owner: Vec<Option<usize>> = vec![None; n];
    for j in 0..n {
        if depth[j] % 2 == 0 {
            owner[j] = Some(regions.len());
            regions.push(Region { outer: orient(&loops[j], true), holes: vec![], glyph });
        }
    }
    for j in 0..n {
        if depth[j] % 2 == 1 {
            // parent: the containing loop one level up
            if let Some(p) = (0..n).find(|&i| contains(i, j) && depth[i] + 1 == depth[j]) {
                if let Some(r) = owner[p] {
                    regions[r].holes.push(orient(&loops[j], false));
                }
            }
        }
    }
    regions
}

#[cfg(test)]
mod tests;
