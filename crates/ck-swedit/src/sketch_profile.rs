//! The SolidWorks sketch the file stores (`ck_sldprt::sketch`), as the profile of an extrude or revolve and as the sketch
//! of the rebuilt history.
//!
//! The sketch is the nearest one before the feature in the stream (the stream does not say which sketch a feature
//! uses), it must be on a plane whose frame is known, and its non-construction curves must be resolved with certain
//! centres and close into loops. [`loops`] gives the profile (used where reading the faces found none). [`build`] and
//! [`write`] give the whole sketch: points, curves (construction ones too), the relations and the dimensions of the
//! file as sketch constraints, every one of them checked against the geometry before it is kept (a constraint that the
//! drawn points do not satisfy is dropped, so the sketch the history carries is exactly the drawn one, and changing a
//! dimension moves the geometry the way the file's author meant). `profile::extrude` prefers it to a sketch drawn from
//! the faces when both give the same profile. Whatever it gives goes through the same checks as every other step (the
//! rebuilt body is compared with the file's by boolean difference), so a wrong sketch cannot be accepted.

use crate::profile::{Loop2, Seg2};
use crate::{Cx, Fr};
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, s as js, Sk};
use ck_sldprt::sketch::{DimKind, PointRole, RelTarget, RelationKind, SketchSet, SkCurve, Sketch};
use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;

thread_local! {
    static SKETCHES: RefCell<Option<Rc<SketchSet>>> = const { RefCell::new(None) };
    /// Counters for the corpus report: (sketch loops that matched the loops read from the faces, did not match, found where the faces
    /// gave none, sketch not usable where the faces gave a profile).
    pub static STATS: RefCell<[usize; 4]> = const { RefCell::new([0; 4]) };
    /// Counters of the sketches written into the history: (true sketches written, with at least one dimension, dimensions
    /// written, relations written, constraints dropped because the geometry did not satisfy them).
    pub static WRITTEN: RefCell<[usize; 5]> = const { RefCell::new([0; 5]) };
    /// Why the file's sketch could not be used where the faces gave a profile (the reason with the sketch's name left out), counted.
    pub static WHY: RefCell<std::collections::BTreeMap<String, usize>> = const { RefCell::new(std::collections::BTreeMap::new()) };
    /// The result of `build` for a feature in a frame against a profile of the faces: a read of the history is repeated (the
    /// reordered readings, the trial with the sketches drawn from the faces), and the search for a placement is not cheap.
    static BUILT: RefCell<HashMap<(i64, [u64; 8]), Result<Built, String>>> = RefCell::new(HashMap::new());
    /// Whether the history carries the file's own sketch where it gives the same profile as the faces (on by default; the
    /// corpus compares the two).
    static PREFER: Cell<bool> = const { Cell::new(true) };
}

/// The sketches of the part being rebuilt (set by `Tagged::from_import`).
pub fn set(s: Option<SketchSet>) {
    SKETCHES.with(|x| *x.borrow_mut() = s.map(Rc::new));
    BUILT.with(|b| b.borrow_mut().clear());
}

/// Switch the preference for the file's own sketch (returns the previous setting).
pub fn prefer(on: bool) -> bool {
    PREFER.with(|p| p.replace(on))
}

pub fn preferred() -> bool {
    PREFER.with(|p| p.get())
}

pub fn sketches() -> Option<Rc<SketchSet>> {
    SKETCHES.with(|x| x.borrow().clone())
}

type P2 = (f64, f64);

fn near(p: P2, q: P2) -> bool {
    (p.0 - q.0).abs() < 1e-6 && (p.1 - q.1).abs() < 1e-6
}

/// Length of the loops (lines and arcs; circles by circumference): a signature to compare two profiles by.
pub fn perimeter(loops: &[Loop2]) -> f64 {
    let mut t = 0.0;
    for l in loops {
        for (s, _) in &l.segs {
            t += match s {
                Seg2::Line { a, b } => (a.0 - b.0).hypot(a.1 - b.1),
                Seg2::Arc { a, b, c, ccw } => {
                    let r = (a.0 - c.0).hypot(a.1 - c.1);
                    let (a0, a1) = ((a.1 - c.1).atan2(a.0 - c.0), (b.1 - c.1).atan2(b.0 - c.0));
                    let mut d = if *ccw { a1 - a0 } else { a0 - a1 };
                    while d <= 1e-12 {
                        d += std::f64::consts::TAU;
                    }
                    r * d
                }
                Seg2::Circle { r, .. } => std::f64::consts::TAU * r,
                Seg2::Ellipse { c, p, r } => {
                    // Ramanujan's second approximation (an ellipse is compared by its axes as well: `dist_to`)
                    let a = (p.0 - c.0).hypot(p.1 - c.1);
                    let h = ((a - r) / (a + r)).powi(2);
                    std::f64::consts::PI * (a + r) * (1.0 + 3.0 * h / (10.0 + (4.0 - 3.0 * h).sqrt()))
                }
                _ => return f64::NAN,
            };
        }
    }
    t
}

/// Distance from `p` to the piece `s` (lines, arcs and circles; `None` for the other kinds).
fn dist_to(s: &Seg2, p: P2) -> Option<f64> {
    match s {
        Seg2::Line { a, b } => {
            let (dx, dy) = (b.0 - a.0, b.1 - a.1);
            let l2 = dx * dx + dy * dy;
            let t = if l2 < 1e-18 { 0.0 } else { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / l2).clamp(0.0, 1.0) };
            Some((p.0 - a.0 - t * dx).hypot(p.1 - a.1 - t * dy))
        }
        Seg2::Circle { c, r } => Some(((p.0 - c.0).hypot(p.1 - c.1) - r).abs()),
        Seg2::Arc { a, b, c, ccw } => {
            let r = (a.0 - c.0).hypot(a.1 - c.1);
            let ang = |q: P2| (q.1 - c.1).atan2(q.0 - c.0);
            let (a0, a1, t) = (ang(*a), ang(*b), ang(p));
            let wrap = |x: f64| x.rem_euclid(std::f64::consts::TAU);
            let (span, off) = if *ccw { (wrap(a1 - a0), wrap(t - a0)) } else { (wrap(a0 - a1), wrap(a0 - t)) };
            if off <= span + 1e-9 {
                Some(((p.0 - c.0).hypot(p.1 - c.1) - r).abs())
            } else {
                Some((p.0 - a.0).hypot(p.1 - a.1).min((p.0 - b.0).hypot(p.1 - b.1)))
            }
        }
        Seg2::Ellipse { c, p: pm, r } => {
            let a = (pm.0 - c.0).hypot(pm.1 - c.1);
            if a < 1e-12 || *r < 1e-12 {
                return None;
            }
            let (ux, uy) = ((pm.0 - c.0) / a, (pm.1 - c.1) / a);
            let (dx, dy) = (p.0 - c.0, p.1 - c.1);
            let (x, y) = (dx * ux + dy * uy, -dx * uy + dy * ux);
            let f = (x / a).powi(2) + (y / r).powi(2) - 1.0;
            let g = ((2.0 * x / (a * a)).powi(2) + (2.0 * y / (r * r)).powi(2)).sqrt().max(1e-12);
            Some(f.abs() / g)
        }
        _ => None,
    }
}

/// Points along a piece (ends, quarter points) to test the cover of one profile by another.
fn probes(s: &Seg2) -> Vec<P2> {
    match s {
        Seg2::Line { a, b } => (0..=4).map(|k| (a.0 + (b.0 - a.0) * k as f64 / 4.0, a.1 + (b.1 - a.1) * k as f64 / 4.0)).collect(),
        Seg2::Arc { a, b, c, ccw } => {
            let r = (a.0 - c.0).hypot(a.1 - c.1);
            let (a0, a1) = ((a.1 - c.1).atan2(a.0 - c.0), (b.1 - c.1).atan2(b.0 - c.0));
            let mut d = if *ccw { a1 - a0 } else { a0 - a1 };
            while d <= 1e-12 {
                d += std::f64::consts::TAU;
            }
            let sg = if *ccw { 1.0 } else { -1.0 };
            (0..=4).map(|k| a0 + sg * d * k as f64 / 4.0).map(|t| (c.0 + r * t.cos(), c.1 + r * t.sin())).collect()
        }
        Seg2::Circle { c, r } => (0..8).map(|k| k as f64 * std::f64::consts::TAU / 8.0).map(|t| (c.0 + r * t.cos(), c.1 + r * t.sin())).collect(),
        Seg2::Ellipse { c, p, r } => {
            let a = (p.0 - c.0).hypot(p.1 - c.1).max(1e-12);
            let (ux, uy) = ((p.0 - c.0) / a, (p.1 - c.1) / a);
            (0..8)
                .map(|k| k as f64 * std::f64::consts::TAU / 8.0)
                .map(|t| (c.0 + a * t.cos() * ux - r * t.sin() * uy, c.1 + a * t.cos() * uy + r * t.sin() * ux))
                .collect()
        }
        _ => vec![],
    }
}

/// A point in the middle of a piece.
pub fn mid_probe(s: &Seg2) -> P2 {
    let p = probes(s);
    p.get(p.len() / 2).copied().unwrap_or((0.0, 0.0))
}

/// True when `p` lies on the piece `s` (to a micrometre or so).
pub fn on_piece(s: &Seg2, p: P2) -> bool {
    dist_to(s, p).map_or(false, |d| d < 1e-5 * (1.0 + p.0.abs() + p.1.abs()))
}

/// True when every piece of `a` lies on a piece of `b` and the two have the same length: the same outline, whichever way
/// it is cut into pieces (the faces merge collinear sides, the sketch may not).
pub fn same_profile(a: &[Loop2], b: &[Loop2]) -> bool {
    let (pa, pb) = (perimeter(a), perimeter(b));
    if !(pa.is_finite() && pb.is_finite()) || (pa - pb).abs() > 1e-6 * (1.0 + pa.abs()) {
        return false;
    }
    let tol = 1e-5 * (1.0 + pa.abs());
    let cover = |x: &[Loop2], y: &[Loop2]| {
        x.iter().flat_map(|l| l.segs.iter()).all(|(s, _)| {
            let pr = probes(s);
            !pr.is_empty() && pr.iter().all(|&p| y.iter().flat_map(|l| l.segs.iter()).any(|(t, _)| dist_to(t, p).map_or(false, |d| d < tol)))
        })
    };
    cover(a, b) && cover(b, a)
}

/// Everything read from the sketch of a feature, in the frame of the profile.
#[derive(Clone)]
pub struct Built {
    /// The profile (non-construction curves) as closed loops; the pieces carry no faces.
    pub loops: Vec<Loop2>,
    /// For each piece of `loops` (same order), the curve of the sketch it came from.
    pub curve_of: Vec<Vec<usize>>,
    sk: Sketch,
    to_fr: Vec<Option<P2>>,
    flips: bool,
    /// The model point (mm) of the sketch's own origin: where the file's sketch plane lies (not known for a matched placement).
    origin: Option<Vec3>,
    /// The placement was found by matching the sketch to the profile of the faces (the stream gave no plane frame), not read.
    pub matched: bool,
}

impl Built {
    pub fn sketch(&self) -> &Sketch {
        &self.sk
    }
    /// The model point of the sketch's own origin (mm); its component along the profile normal is where the file's sketch plane lies.
    pub fn origin_mm(&self) -> Option<Vec3> {
        self.origin
    }
    /// The sketch point `i` in the profile frame (mm).
    pub fn point(&self, i: usize) -> Option<P2> {
        self.to_fr.get(i).copied().flatten()
    }
}

/// How the sketch's own coordinates (metres) map into the profile frame (mm): `X = (a x + b y) 1e3 + t.0`, `Y = (c x + d y) 1e3 + t.1`.
#[derive(Debug, Clone, Copy)]
struct Placement {
    m: [f64; 4],
    t: P2,
}

impl Placement {
    fn map(&self, x: f64, y: f64) -> P2 {
        ((self.m[0] * x + self.m[1] * y) * 1e3 + self.t.0, (self.m[2] * x + self.m[3] * y) * 1e3 + self.t.1)
    }
    fn flips(&self) -> bool {
        self.m[0] * self.m[3] - self.m[1] * self.m[2] < 0.0
    }
}

/// The sketch before feature `f_id`, placed in the frame `fr` (mm). The sketch's plane frame is the one the stream gives; where
/// it gives none and `face` (the profile the faces gave) is known, the placement is the one under which the sketch draws that
/// profile (see [`Built::matched`]).
pub fn build(cx: &Cx, f_id: i64, fr: &Fr, face: Option<&[Loop2]>) -> Result<Built, String> {
    let key = (
        f_id,
        [fr.o.x.to_bits(), fr.o.y.to_bits(), fr.o.z.to_bits(), fr.z.x.to_bits(), fr.z.y.to_bits(), fr.z.z.to_bits(), face.map_or(0, |l| perimeter(l).to_bits()), face.map_or(u64::MAX, |l| l.iter().map(|x| x.segs.len() as u64).sum())],
    );
    if let Some(r) = BUILT.with(|b| b.borrow().get(&key).cloned()) {
        return r;
    }
    let r = build_uncached(cx, f_id, fr, face);
    BUILT.with(|b| b.borrow_mut().insert(key, r.clone()));
    r
}

fn build_uncached(cx: &Cx, f_id: i64, fr: &Fr, face: Option<&[Loop2]>) -> Result<Built, String> {
    let _ = cx;
    let set = sketches().ok_or("no sketches read")?;
    let idx = set.features.iter().position(|r| r.id as i64 == f_id).ok_or("feature has no record in the stream")?;
    let sk = set.sketch_before(idx).ok_or("no sketch before the feature")?.clone();
    // (with the profile of the faces to check against, a sketch that holds records that are not curves of it is read: only the
    // profile has to draw what the faces show)
    if !sk.notes.is_empty() && (face.is_none() || sk.notes.iter().any(|n| !n.contains("curve-shaped records"))) {
        return Err(format!("sketch {}: {}", sk.name, sk.notes.join("; ")));
    }
    let allow_uncertain = face.is_some();
    if let Some(fd) = sk.frame() {
        // the sketch plane must be the plane of the profile frame (parallel: the sketch may lie on another level, as an extrusion
        // that starts from an offset runs from there)
        let nrm = Vec3::new(fd.x[1] * fd.y[2] - fd.x[2] * fd.y[1], fd.x[2] * fd.y[0] - fd.x[0] * fd.y[2], fd.x[0] * fd.y[1] - fd.x[1] * fd.y[0]);
        if nrm.dot(fr.z).abs() < 1.0 - 1e-6 {
            return Err(format!("sketch {}: its plane is not parallel to the profile's", sk.name));
        }
        let lift = fr.z * fr.o.dot(fr.z);
        // (onto the plane of the profile frame: the sketch plane's own offset along the normal is not the frame's concern)
        let map = |x: f64, y: f64| -> Option<P2> {
            let m = sk.to_model(x, y)?;
            let p = Vec3::new(m[0] * 1e3, m[1] * 1e3, m[2] * 1e3);
            Some(fr.xy(p - fr.z * p.dot(fr.z) + lift))
        };
        let (o, ex, ey) = (map(0.0, 0.0).ok_or("no frame")?, map(1.0, 0.0).ok_or("no frame")?, map(0.0, 1.0).ok_or("no frame")?);
        // (the unit steps are metres: the placement is in metres of the sketch to mm of the frame)
        let pl = Placement { m: [(ex.0 - o.0) / 1e3, (ey.0 - o.0) / 1e3, (ex.1 - o.1) / 1e3, (ey.1 - o.1) / 1e3], t: o };
        let m0 = sk.to_model(0.0, 0.0).map_or(Vec3::new(0.0, 0.0, 0.0), |m| Vec3::new(m[0] * 1e3, m[1] * 1e3, m[2] * 1e3));
        return place(sk, fr, pl, Some(m0), false, allow_uncertain);
    }
    let Some(face) = face else { return Err(format!("sketch {}: plane frame unknown ({:?})", sk.name, sk.plane)) };
    // no frame in the stream: the sketch's plane is the profile's (the faces say so) and its axes are along those of the frame or
    // the other way round (SolidWorks' sketch axes on a standard plane, an offset plane or a planar face are the plane's or the
    // face's own); the placement is the one of the eight under which the sketch draws the profile of the faces. Of those that do,
    // the one that keeps most of the file's relations and dimensions true is taken (the horizontal and vertical relations and
    // the dimensions along an axis tell a square from its quarter turn)
    let t0 = fr.xy(Vec3::new(0.0, 0.0, 0.0));
    let mats: [[f64; 4]; 8] = [[1.0, 0.0, 0.0, 1.0], [0.0, -1.0, 1.0, 0.0], [-1.0, 0.0, 0.0, -1.0], [0.0, 1.0, -1.0, 0.0], [1.0, 0.0, 0.0, -1.0], [-1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 1.0, 0.0], [0.0, -1.0, -1.0, 0.0]];
    // translations: the model origin's projection first, then each vertex of the faces' profile under the first sketch point
    let mut verts: Vec<P2> = vec![];
    for l in face {
        for (sg, _) in &l.segs {
            match sg {
                Seg2::Line { a, .. } | Seg2::Arc { a, .. } => verts.push(*a),
                Seg2::Circle { c, .. } | Seg2::Ellipse { c, .. } => verts.push(*c),
                _ => {}
            }
        }
    }
    verts.truncate(80);
    let pivot = sk.curves.iter().find_map(|c| match c {
        SkCurve::Line { a, .. } => Some(*a),
        SkCurve::Arc { start, .. } => Some(*start),
        SkCurve::Circle { centre: Some(k), .. } => Some(*k),
        SkCurve::Ellipse { centre, .. } => Some(*centre),
        _ => None,
    });
    let mut best: Option<(usize, Built)> = None;
    for pass in 0..2 {
        for m in &mats {
            let ts: Vec<P2> = if pass == 0 {
                vec![t0]
            } else {
                let Some(q) = pivot.and_then(|k| sk.points.get(k)).filter(|p| p.located) else { break };
                let (qx, qy) = ((m[0] * q.x + m[1] * q.y) * 1e3, (m[2] * q.x + m[3] * q.y) * 1e3);
                verts.iter().map(|v| (v.0 - qx, v.1 - qy)).collect()
            };
            for t in ts {
                let pl = Placement { m: *m, t };
                let Ok(b) = place(sk.clone(), fr, pl, None, true, allow_uncertain) else { continue };
                if !same_profile(face, &b.loops) {
                    continue;
                }
                let w = write(&b, "x", Json::Null);
                let score = w.rels + w.dims;
                if best.as_ref().map_or(true, |(s, _)| score > *s) {
                    best = Some((score, b));
                }
            }
        }
        if best.is_some() {
            break;
        }
    }
    if best.is_none() && std::env::var("SKETCH_DEBUG").is_ok() {
        // (a note for the research runs: how far the sketch is from the profile of the faces under the identity placement)
        let pl = Placement { m: mats[0], t: t0 };
        let seg_count = |l: &[Loop2]| l.iter().map(|x| x.segs.len()).sum::<usize>();
        let mine = place(sk.clone(), fr, pl, None, true, allow_uncertain).map(|b| (perimeter(&b.loops), seg_count(&b.loops), b.loops.len()));
        eprintln!("SKETCH_DEBUG {} f{}: faces perimeter {:.4} segs {} loops {}; sketch {:?} ({} curves, {} points)", sk.name, f_id, perimeter(face), seg_count(face), face.len(), mine, sk.curves.len(), sk.points.len());
    }
    best.map(|(_, b)| b).ok_or_else(|| format!("sketch {}: no frame in the stream and no placement draws the profile the faces gave", sk.name))
}

/// Places `sk` in the profile frame by `pl` and chains its non-construction curves into closed loops.
/// (`allow_uncertain`: an arc or circle whose centre the reader could not settle is placed with the centre it prefers; the profile
/// of the faces then has to be drawn by it for the placement to count.)
fn place(sk: Sketch, fr: &Fr, pl: Placement, origin: Option<Vec3>, matched: bool, allow_uncertain: bool) -> Result<Built, String> {
    let _ = fr;
    let flips = pl.flips();
    let to_fr: Vec<Option<P2>> = sk.points.iter().map(|p| if p.located { Some(pl.map(p.x, p.y)) } else { None }).collect();
    let pt = |i: usize| -> Result<P2, String> { to_fr.get(i).copied().flatten().ok_or_else(|| "point index".to_string()) };
    let mut segs: Vec<(Seg2, usize)> = vec![];
    for (ci, c) in sk.curves.iter().enumerate() {
        match c {
            SkCurve::Line { construction: true, .. } | SkCurve::Arc { construction: true, .. } | SkCurve::Circle { construction: true, .. } | SkCurve::Ellipse { construction: true, .. } => {}
            SkCurve::Line { a, b, .. } => segs.push((Seg2::Line { a: pt(*a)?, b: pt(*b)? }, ci)),
            SkCurve::Arc { start, end, centre: Some(k), centre_certain, sense, .. } if *centre_certain || allow_uncertain => {
                // the arc sense stored with the curve: 1 counter-clockwise from start to end, -1 clockwise (in the sketch)
                let ccw = match sense {
                    Some(1) => true,
                    Some(-1) => false,
                    _ if std::env::var("SKETCH_ASSUME_CCW").is_ok() => true,
                    _ => return Err(format!("sketch {}: an arc has no sense", sk.name)),
                };
                segs.push((Seg2::Arc { a: pt(*start)?, b: pt(*end)?, c: pt(*k)?, ccw: ccw != flips }, ci))
            }
            SkCurve::Circle { rim, centre: Some(k), centre_certain, .. } if *centre_certain || allow_uncertain => {
                let (c, r) = (pt(*k)?, pt(*rim)?);
                segs.push((Seg2::Circle { c, r: (c.0 - r.0).hypot(c.1 - r.1) }, ci));
            }
            SkCurve::Ellipse { centre, major, minor, .. } => segs.push((Seg2::Ellipse { c: pt(*centre)?, p: pt(*major)?, r: minor * 1e3 }, ci)),
            _ => return Err(format!("sketch {}: a curve is not exact", sk.name)),
        }
    }
    if segs.is_empty() {
        return Err("no curves".into());
    }
    // chain the pieces into closed loops
    let mut out: Vec<Loop2> = vec![];
    let mut curve_of: Vec<Vec<usize>> = vec![];
    let mut left = segs;
    while let Some((first, fi)) = left.pop() {
        if let Seg2::Circle { .. } | Seg2::Ellipse { .. } = first {
            out.push(Loop2 { segs: vec![(first, vec![])] });
            curve_of.push(vec![fi]);
            continue;
        }
        let ends = |s: &Seg2| match s {
            Seg2::Line { a, b } | Seg2::Arc { a, b, .. } => Some((*a, *b)),
            _ => None,
        };
        let start = ends(&first).ok_or("piece")?.0;
        let mut cur = vec![(first.clone(), vec![])];
        let mut idxs = vec![fi];
        let mut at = ends(&first).ok_or("piece")?.1;
        while !near(at, start) {
            let k = left.iter().position(|(s, _)| ends(s).map_or(false, |(a, b)| near(a, at) || near(b, at))).ok_or("the sketch outline does not close")?;
            let (s, si) = left.remove(k);
            let (a, _) = ends(&s).ok_or("piece")?;
            let s = if near(a, at) {
                s
            } else {
                match s {
                    Seg2::Line { a, b } => Seg2::Line { a: b, b: a },
                    Seg2::Arc { a, b, c, ccw } => Seg2::Arc { a: b, b: a, c, ccw: !ccw },
                    o => o,
                }
            };
            at = ends(&s).ok_or("piece")?.1;
            cur.push((s, vec![]));
            idxs.push(si);
        }
        out.push(Loop2 { segs: cur });
        curve_of.push(idxs);
    }
    Ok(Built { loops: out, curve_of, sk, to_fr, flips, origin, matched })
}

/// The end type word and start type word the file stores for feature `f_id` (an extrusion or revolve), and the name of the end
/// type. The numbers follow SolidWorks' public end condition enumeration; `docs/sldprt-sketch.md` says which of them the local
/// files confirm.
pub fn end_info(f_id: i64) -> Option<(u32, Option<u32>, &'static str)> {
    let set = sketches()?;
    let f = set.features.iter().find(|r| r.id as i64 == f_id)?;
    let t = f.end_type?;
    Some((t, f.from_type, end_name(t)))
}

pub fn end_name(t: u32) -> &'static str {
    match t {
        0 => "blind",
        1 => "through all",
        2 => "up to next",
        3 => "up to vertex",
        4 => "up to surface",
        5 => "offset from surface",
        6 => "mid plane",
        7 => "up to body",
        9 => "through all, both ways",
        _ => "unknown",
    }
}

/// The loops of the sketch before feature `f` in the frame `fr` (its plane frame must be in the stream).
pub fn loops(cx: &Cx, f_id: i64, fr: &Fr) -> Result<Vec<Loop2>, String> {
    build(cx, f_id, fr, None).map(|b| b.loops)
}

/// The sketch written for the history: the points and curves of the file's sketch in the frame of the profile, its relations
/// and dimensions as constraints.
pub struct Written {
    pub json: Json,
    /// Each piece of the profile loops of the [`Built`] with the id of its entity.
    pub piece_ids: Vec<(Seg2, String)>,
    /// Dimensions and relations kept; constraints dropped because the drawn geometry does not satisfy them.
    pub dims: usize,
    pub rels: usize,
    pub dropped: usize,
}

fn constraint(kind: &str, id: &str, refs: &[&str]) -> Json {
    Obj::new().put("type", kind).put("id", id).put("refs", arr(refs.iter().map(|r| js(r)).collect())).build()
}

fn with(j: Json, key: &str, v: Json) -> Json {
    match j {
        Json::Obj(mut m) => {
            m.push((key.to_string(), v));
            Json::Obj(m)
        }
        o => o,
    }
}

/// Writes the sketch as the document sketch `sid` on `plane`.
pub fn write(b: &Built, sid: &str, plane: Json) -> Written {
    let sk = &b.sk;
    let mut out = Sk::new(sid.to_string(), plane);
    // points: every located one; the circle rim points are not points of ours (a circle is a centre and a radius)
    let mut pid: HashMap<usize, String> = HashMap::new();
    let origin = sk.points.iter().position(|p| p.counters.0 == 6 && p.name.is_some() && p.located);
    for (i, p) in sk.points.iter().enumerate() {
        let Some(q) = b.to_fr[i] else { continue };
        if p.role == PointRole::Rim {
            continue;
        }
        if Some(i) == origin {
            // the file's origin reference: the fixed point `O` the sketch solver knows
            pid.insert(i, "O".to_string());
            out.points.push(Obj::new().put("id", "O").put("x", n(q.0)).put("y", n(q.1)).put("fixed", true).build());
            continue;
        }
        pid.insert(i, out.point(q.0, q.1));
    }
    // curves
    let mut eid: Vec<Option<String>> = vec![None; sk.curves.len()];
    let mut line_h: HashMap<u16, usize> = HashMap::new();
    let mut arc_h: HashMap<u16, usize> = HashMap::new();
    for (ci, c) in sk.curves.iter().enumerate() {
        let id = match c {
            SkCurve::Line { a, b: bb, construction, handle, .. } => {
                let (Some(pa), Some(pb)) = (pid.get(a), pid.get(bb)) else { continue };
                if let Some(h) = handle {
                    line_h.insert(*h, ci);
                }
                out.line(pa, pb, *construction)
            }
            SkCurve::Arc { start, end, centre: Some(k), construction, handle, sense, .. } => {
                let (Some(ps), Some(pe), Some(pk)) = (pid.get(start), pid.get(end), pid.get(k)) else { continue };
                if let Some(h) = handle {
                    arc_h.insert(*h, ci);
                }
                let ccw = (*sense == Some(1)) != b.flips;
                let id = if ccw { out.arc(pk, ps, pe) } else { out.arc(pk, pe, ps) };
                if *construction {
                    if let Some(e) = out.entities.pop() {
                        out.entities.push(with(e, "construction", Json::Bool(true)));
                    }
                }
                id
            }
            SkCurve::Circle { rim, centre: Some(k), construction, handle, .. } => {
                let (Some(pk), Some(pr), Some(pc)) = (pid.get(k), b.to_fr[*rim], b.to_fr[*k]) else { continue };
                if let Some(h) = handle {
                    arc_h.insert(*h, ci);
                }
                let id = out.circle(pk, (pc.0 - pr.0).hypot(pc.1 - pr.1));
                if *construction {
                    if let Some(e) = out.entities.pop() {
                        out.entities.push(with(e, "construction", Json::Bool(true)));
                    }
                }
                id
            }
            SkCurve::Ellipse { centre, major, minor, construction, handle, .. } => {
                let (Some(pk), Some(pm)) = (pid.get(centre), pid.get(major)) else { continue };
                if let Some(h) = handle {
                    arc_h.insert(*h, ci);
                }
                let id = format!("el{}", out.entities.len() + 1);
                let mut e = Obj::new().put("type", "ellipse").put("id", id.as_str()).put("c", pk.as_str()).put("p", pm.as_str()).put("r", n(minor * 1e3)).build();
                if *construction {
                    e = with(e, "construction", Json::Bool(true));
                }
                out.entities.push(e);
                id
            }
            _ => continue,
        };
        eid[ci] = Some(id);
    }
    let ent = |t: &RelTarget| -> Option<String> {
        match t {
            RelTarget::Line(h) => line_h.get(h).and_then(|&ci| eid[ci].clone()),
            RelTarget::Arc(h) => arc_h.get(h).and_then(|&ci| eid[ci].clone()),
            _ => None,
        }
    };
    let pnt = |t: &RelTarget| -> Option<String> {
        match t {
            RelTarget::Point(i) => pid.get(i).cloned(),
            _ => None,
        }
    };
    let is_arc = |t: &RelTarget| matches!(t, RelTarget::Arc(_));
    let mut cons: Vec<Json> = vec![];
    let mut k = 0usize;
    let mut mk = |kind: &str, refs: &[&str]| {
        k += 1;
        constraint(kind, &format!("k{k}"), refs)
    };
    // relations
    for r in &sk.relations {
        let t = &r.targets;
        let made: Option<Json> = match (r.kind, t.as_slice()) {
            (RelationKind::Horizontal, [l]) => ent(l).map(|e| mk("horizontal", &[&e])),
            (RelationKind::Vertical, [l]) => ent(l).map(|e| mk("vertical", &[&e])),
            (RelationKind::Coincident, [a, c]) => match (pnt(a), pnt(c), ent(a), ent(c)) {
                (Some(x), Some(y), _, _) => (x != y).then(|| mk("coincident", &[&x, &y])),
                (Some(x), _, _, Some(e)) | (_, Some(x), Some(e), _) => Some(mk(if is_arc(a) || is_arc(c) { "pointOnCircle" } else { "pointOnLine" }, &[&x, &e])),
                _ => None,
            },
            (RelationKind::Collinear, [a, c]) => ent(a).zip(ent(c)).map(|(x, y)| mk("collinear", &[&x, &y])),
            (RelationKind::Equal, [a, c]) => ent(a).zip(ent(c)).map(|(x, y)| mk("equal", &[&x, &y])),
            (RelationKind::Tangent, [a, c]) => ent(a).zip(ent(c)).map(|(x, y)| mk("tangent", &[&x, &y])),
            (RelationKind::Midpoint, [p, l]) => pnt(p).zip(ent(l)).map(|(x, y)| mk("midpoint", &[&x, &y])),
            (RelationKind::Symmetric, [p, q, l]) => match (pnt(p), pnt(q), ent(l)) {
                (Some(x), Some(y), Some(z)) => Some(mk("symmetric", &[&x, &y, &z])),
                _ => None,
            },
            (RelationKind::HorizontalPoints, [p, q]) => pnt(p).zip(pnt(q)).map(|(x, y)| mk("horizontal", &[&x, &y])),
            (RelationKind::VerticalPoints, [p, q]) => pnt(p).zip(pnt(q)).map(|(x, y)| mk("vertical", &[&x, &y])),
            _ => None,
        };
        if let Some(c) = made {
            cons.push(c);
        }
    }
    let n_rel = cons.len();
    // dimensions (metres and radians in the file; millimetres and degrees in the sketch)
    let mm = 1e3;
    let line_ends = |e: &str| -> Option<(String, String)> {
        let ci = eid.iter().position(|x| x.as_deref() == Some(e))?;
        if let SkCurve::Line { a, b: bb, .. } = &sk.curves[ci] {
            Some((pid.get(a)?.clone(), pid.get(bb)?.clone()))
        } else {
            None
        }
    };
    for d in &sk.dims {
        let t = d.targets.as_slice();
        let made: Option<(Json, f64)> = match (d.kind, t) {
            (DimKind::Radius, [c]) => ent(c).map(|e| (mk("radius", &[&e]), d.value * mm)),
            (DimKind::Diameter, [c]) => ent(c).map(|e| (mk("diameter", &[&e]), d.value * mm)),
            (DimKind::Distance, [a, c]) => match (pnt(a), pnt(c), ent(c)) {
                (Some(x), Some(y), _) => Some((mk("distance", &[&x, &y]), d.value * mm)),
                (Some(x), _, Some(e)) => Some((mk("pldist", &[&x, &e]), d.value * mm)),
                _ => None,
            },
            // the length of a line
            (DimKind::Distance, [l]) => ent(l).and_then(|e| line_ends(&e)).map(|(p, q)| (mk("distance", &[&p, &q]), d.value * mm)),
            (DimKind::HDist, [a, c]) => pnt(a).zip(pnt(c)).map(|(x, y)| (mk("hdist", &[&x, &y]), d.value * mm)),
            (DimKind::VDist, [a, c]) => pnt(a).zip(pnt(c)).map(|(x, y)| (mk("vdist", &[&x, &y]), d.value * mm)),
            (DimKind::Angle, [a, c]) => ent(a).zip(ent(c)).map(|(x, y)| (mk("angle", &[&x, &y]), d.value.to_degrees())),
            _ => None,
        };
        if let Some((c, v)) = made {
            cons.push(with(with(with(c, "value", n(v)), "offset", Json::Null), "name", js(&d.name)));
        }
    }
    let json = |cons: &[Json], out: &Sk| -> Json {
        Obj::new()
            .put("type", "sketch")
            .put("id", out.id.as_str())
            .put("plane", out.plane.clone())
            .put("points", arr(out.points.clone()))
            .put("entities", arr(out.entities.clone()))
            .put("constraints", arr(cons.to_vec()))
            .put("source", "sldprt")
            .build()
    };
    // every constraint must hold at the drawn geometry (and build as a solver system): the others are dropped
    let total = cons.len();
    let mut kept: Vec<Json> = vec![];
    let mut kept_dims = 0usize;
    for (i, c) in cons.into_iter().enumerate() {
        // (the file does not say which of the four angles between two lines an angle dimension is: the sign choices of the
        // solver's angle constraint are tried, the one the drawn geometry satisfies is kept)
        let variants: Vec<Json> = if c.get("type").and_then(|t| t.as_str()) == Some("angle") {
            [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)].iter().map(|&(a, b)| with(with(c.clone(), "s1", n(a)), "s2", n(b))).collect()
        } else {
            vec![c]
        };
        for v in variants {
            let mut trial = kept.clone();
            trial.push(v.clone());
            if holds(&json(&trial, &out)) {
                if i >= n_rel {
                    kept_dims += 1;
                }
                kept.push(v);
                break;
            }
        }
    }
    let piece_ids: Vec<(Seg2, String)> = b
        .loops
        .iter()
        .zip(&b.curve_of)
        .flat_map(|(l, cs)| l.segs.iter().zip(cs).filter_map(|((seg, _), &ci)| eid[ci].clone().map(|e| (seg.clone(), e))))
        .collect();
    let rels = kept.len() - kept_dims;
    let dropped = total - kept.len();
    Written { json: json(&kept, &out), piece_ids, dims: kept_dims, rels, dropped }
}

/// The line of the written sketch that lies on the axis through `p0` and `p1` (sketch frame), else a construction line added on it.
pub fn axis_entity(w: &mut Written, p0: P2, p1: P2) -> String {
    let len = (p1.0 - p0.0).hypot(p1.1 - p0.1).max(1e-9);
    let off = |q: P2| ((q.0 - p0.0) * (p1.1 - p0.1) - (q.1 - p0.1) * (p1.0 - p0.0)).abs() / len;
    let Json::Obj(top) = &mut w.json else { return String::new() };
    let get = |top: &Vec<(String, Json)>, k: &str| -> Vec<Json> { top.iter().find(|(n, _)| n == k).and_then(|(_, v)| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default() };
    let pts: HashMap<String, P2> = get(top, "points")
        .iter()
        .filter_map(|p| Some((p.get("id")?.as_str()?.to_string(), (p.get("x")?.as_f64()?, p.get("y")?.as_f64()?))))
        .collect();
    for e in get(top, "entities") {
        if e.get("type").and_then(|t| t.as_str()) != Some("line") {
            continue;
        }
        let (Some(id), Some(a), Some(b)) = (e.get("id").and_then(|v| v.as_str()), e.get("p1").and_then(|v| v.as_str()).and_then(|k| pts.get(k)), e.get("p2").and_then(|v| v.as_str()).and_then(|k| pts.get(k))) else { continue };
        if (a.0 - b.0).hypot(a.1 - b.1) > 1e-6 && off(*a) < 1e-6 * (1.0 + len) && off(*b) < 1e-6 * (1.0 + len) {
            return id.to_string();
        }
    }
    let push = |top: &mut Vec<(String, Json)>, k: &str, v: Json| {
        if let Some((_, Json::Arr(a))) = top.iter_mut().find(|(n, _)| n == k) {
            a.push(v);
        }
    };
    push(top, "points", Obj::new().put("id", "ax1").put("x", n(p0.0)).put("y", n(p0.1)).build());
    push(top, "points", Obj::new().put("id", "ax2").put("x", n(p1.0)).put("y", n(p1.1)).build());
    push(top, "entities", Obj::new().put("type", "line").put("id", "ax").put("p1", "ax1").put("p2", "ax2").put("construction", true).build());
    "ax".to_string()
}

/// True when the sketch builds as a solver system and every equation holds at the drawn points.
fn holds(j: &Json) -> bool {
    let sk = ck_model::sketch::Sketch::from_json(j);
    match ck_model::sketch::solver::build(&sk) {
        Ok(sys) => sys.eqs.iter().all(|e| e.eval(&sys.x).abs() < 1e-6),
        Err(_) => false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_sldprt::sketch::{Dimension, PlaneRef, Relation, SkPoint};

    fn pt(i: u32, x: f64, y: f64) -> SkPoint {
        SkPoint { id: i, x, y, located: true, role: PointRole::Free, name: None, counters: (0, 2), on: vec![] }
    }

    /// A 10 x 6 mm rectangle (lines with handles 0 to 3) with its relations and three dimensions, one of them wrong.
    fn rectangle(wrong: f64) -> Built {
        let pts = vec![pt(1, 0.0, 0.0), pt(2, 0.010, 0.0), pt(3, 0.010, 0.006), pt(4, 0.0, 0.006)];
        let line = |a, b, h| SkCurve::Line { a, b, construction: false, handle: Some(h), sense: Some(0) };
        let curves = vec![line(0, 1, 0), line(1, 2, 1), line(2, 3, 2), line(3, 0, 3)];
        let rel = |kind, targets: Vec<RelTarget>| Relation { kind, targets };
        let dim = |name: &str, kind, value, targets: Vec<RelTarget>| Dimension { name: name.into(), kind, value, targets, offset: 0 };
        let sk = Sketch {
            name: "Sketch1".into(),
            feature_id: 26,
            plane: PlaneRef::Plane(2),
            plane_refs: vec![],
            plane_frames: vec![],
            stored_frame: None,
            points: pts.clone(),
            curves,
            params: vec![],
            relations: vec![
                rel(RelationKind::Horizontal, vec![RelTarget::Line(0)]),
                rel(RelationKind::Vertical, vec![RelTarget::Line(1)]),
                rel(RelationKind::Horizontal, vec![RelTarget::Line(2)]),
                rel(RelationKind::Vertical, vec![RelTarget::Line(3)]),
                // a relation the drawn geometry does not satisfy: dropped
                rel(RelationKind::Equal, vec![RelTarget::Line(0), RelTarget::Line(1)]),
            ],
            raw_dims: vec![],
            dims: vec![
                dim("D1", DimKind::Distance, 0.010, vec![RelTarget::Point(0), RelTarget::Point(1)]),
                dim("D2", DimKind::VDist, 0.006, vec![RelTarget::Point(1), RelTarget::Point(2)]),
                dim("D3", DimKind::Distance, wrong, vec![RelTarget::Point(0), RelTarget::Point(3)]),
            ],
            offset: 0,
            notes: vec![],
        };
        let to_fr = pts.iter().map(|p| Some((p.x * 1e3, p.y * 1e3))).collect();
        Built { loops: vec![], curve_of: vec![], sk, to_fr, flips: false, origin: None, matched: false }
    }

    #[test]
    fn the_file_sketch_is_written_with_the_constraints_the_geometry_satisfies() {
        let w = write(&rectangle(0.006), "F1", Obj::new().put("kind", "std").put("id", "front").build());
        // four horizontal / vertical relations and three dimensions; the equal-length relation of a 10 x 6 rectangle is dropped
        assert_eq!((w.rels, w.dims, w.dropped), (4, 3, 1));
        let cons = w.json.get("constraints").and_then(|c| c.as_arr()).unwrap();
        assert!(cons.iter().any(|c| c.get("type").and_then(|t| t.as_str()) == Some("vdist") && c.get("value").and_then(|v| v.as_f64()) == Some(6.0)));
        assert!(cons.iter().any(|c| c.get("type").and_then(|t| t.as_str()) == Some("distance") && c.get("name").and_then(|v| v.as_str()) == Some("D1")));
        assert_eq!(w.json.get("source").and_then(|s| s.as_str()), Some("sldprt"));
        assert!(holds(&w.json));
        // the whole sketch solves where it is, and changing a dimension moves the geometry with it
        let mut sk = ck_model::sketch::Sketch::from_json(&w.json);
        let before: Vec<(f64, f64)> = sk.points.iter().map(|p| (p.x, p.y)).collect();
        assert!(ck_model::sketch::solver::solve_sketch(&mut sk, &Default::default()).unwrap().ok);
        let after: Vec<(f64, f64)> = sk.points.iter().map(|p| (p.x, p.y)).collect();
        for (a, b) in before.iter().zip(&after) {
            assert!((a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6);
        }
        // (the first distance is D1, the 10 mm side)
        sk.constraints.iter_mut().find(|c| c.kind == "distance").unwrap().value = Some(14.0);
        assert!(ck_model::sketch::solver::solve_sketch(&mut sk, &Default::default()).unwrap().ok);
        let x = |id: &str| sk.points.iter().find(|p| p.id == id).unwrap().x;
        assert!((x("p2") - x("p1") - 14.0).abs() < 1e-6, "{} {}", x("p1"), x("p2"));
        // a dimension whose value is wrong (D3 = 7 mm for a 6 mm side) is dropped
        let w = write(&rectangle(0.007), "F1", Obj::new().put("kind", "std").put("id", "front").build());
        assert_eq!((w.rels, w.dims, w.dropped), (4, 2, 2));
    }

    fn rect_loop(x: f64, y: f64) -> Loop2 {
        let p = [(0.0, 0.0), (x, 0.0), (x, y), (0.0, y)];
        Loop2 { segs: (0..4).map(|i| (Seg2::Line { a: p[i], b: p[(i + 1) % 4] }, vec![])).collect() }
    }

    #[test]
    fn a_profile_is_the_same_outline_however_it_is_cut_into_pieces() {
        let a = vec![rect_loop(10.0, 6.0)];
        // the long side as two pieces
        let mut b = rect_loop(10.0, 6.0);
        b.segs[0] = (Seg2::Line { a: (0.0, 0.0), b: (4.0, 0.0) }, vec![]);
        b.segs.insert(1, (Seg2::Line { a: (4.0, 0.0), b: (10.0, 0.0) }, vec![]));
        assert!(same_profile(&a, &[b]));
        assert!(!same_profile(&a, &[rect_loop(10.0, 6.5)]));
        // a circle of the same length of rim as a different outline is not the same
        assert!(!same_profile(&a, &[Loop2 { segs: vec![(Seg2::Circle { c: (0.0, 0.0), r: 5.0 }, vec![])] }]));
        assert!(same_profile(&[Loop2 { segs: vec![(Seg2::Ellipse { c: (0.0, 0.0), p: (6.0, 0.0), r: 3.0 }, vec![])] }], &[Loop2 { segs: vec![(Seg2::Ellipse { c: (0.0, 0.0), p: (6.0, 0.0), r: 3.0 }, vec![])] }]));
        assert!(!same_profile(&[Loop2 { segs: vec![(Seg2::Ellipse { c: (0.0, 0.0), p: (6.0, 0.0), r: 3.0 }, vec![])] }], &[Loop2 { segs: vec![(Seg2::Ellipse { c: (0.0, 0.0), p: (0.0, 6.0), r: 3.0 }, vec![])] }]));
    }

    #[test]
    fn an_axis_takes_the_line_of_the_sketch_that_lies_on_it_or_adds_one() {
        let mut w = write(&rectangle(0.006), "F1", Obj::new().put("kind", "std").put("id", "front").build());
        // the bottom side lies on the x axis
        let e = axis_entity(&mut w, (-5.0, 0.0), (20.0, 0.0));
        assert!(e.starts_with('e'), "{e}");
        let n_before = w.json.get("entities").and_then(|e| e.as_arr()).unwrap().len();
        // nothing lies on y = 2: a construction line is added
        let e = axis_entity(&mut w, (-5.0, 2.0), (20.0, 2.0));
        assert_eq!(e, "ax");
        assert_eq!(w.json.get("entities").and_then(|e| e.as_arr()).unwrap().len(), n_before + 1);
    }
}
