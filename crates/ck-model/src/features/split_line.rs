//! `splitLine` (Split Face): cut faces of the solid along a line so that each becomes two faces. The body stays
//! one solid with the same shape and volume; every cut adds one face.
//!
//! ```json
//! {"type":"splitLine","id":"F5","faces":["F1.S.e1"],"plane":{"kind":"std","id":"front"},"offset":5,"flip":false}
//! {"type":"splitLine","id":"F6","faces":["F1.C1"],"sketch":"F4","direction":"normal"}
//! ```
//!
//! * `faces` (required, names): the faces to split. They must be faces of the solid body and planar.
//! * Cut by a plane: `plane` (standard plane, reference plane feature or planar face, as in `split`) with `offset` (mm along
//!   the plane normal) and `flip`. The cut line of a face is its intersection with the plane.
//! * Cut by a sketch: `sketch` (id) with `direction`: `"normal"` (default, along the normal of each face), `"sketch"` (along the
//!   sketch plane normal) or `[x, y, z]`. Each straight line of the sketch (construction lines are ignored) is projected on
//!   the face along the direction and cuts it where it crosses the face.
//! * `plane` and `sketch` together: the plane wins.
//! * A cut line crosses the boundary of a face in points; each stretch between two consecutive points that lies in the
//!   face is a chord and splits the face in two. A chord must join two points of one loop of the face (a chord from the outer
//!   boundary to a hole is refused). A sketch line must reach the boundary on both ends of every chord it makes
//!   (`invalidValue` `sketch` when it ends inside the face).
//! * Edges crossed by a chord are split at the crossing, so the neighbouring faces gain a vertex (no new face).
//! * Naming: the two pieces of a split face share the old name and take the `~k` suffix the kernel gives split faces, ranked
//!   by the centre of the control box (`F1.S.e1~1`, `F1.S.e1~2`).
//! * Errors: `nothingToCut` (no body), `missingReference` (unknown face, plane), `missingSketch`, `invalidValue
//!   {field, detail}` (no `faces`, no `plane` or `sketch`, bad `direction`, `offset`, an open sketch line, a chord that does not
//!   join one loop), `cutNoEffect` (the line misses every face), `notImplemented {what}` for a face that is not planar
//!   (`splitLine.surface`) and a sketch entity that is not a line (`splitLine.curve`).

use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef};
use ck_core::linalg::Frame;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_json::Json;
use ck_ops::boolean::classify::{point_in_face_cached, Where};
use ck_ops::intersect::cs;
use ck_topo::body::{Body, EdgeId, FaceId, VertexId};

const TOL: f64 = ck_core::linalg::EPS_LINEAR;

fn invalid(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}

fn not_impl(what: &str) -> FeatureError {
    FeatureError::new(&super::extrude::todo(what))
}

/// A cut line on the plane of a face: the point `p0` and the unit direction `dir`; `limit` the stretch `(s0, s1)` of the
/// parameter along `dir` the line covers (`None`: unbounded).
struct Line {
    p0: Vec3,
    dir: Vec3,
    limit: Option<(f64, f64)>,
}

enum Cutter {
    Plane { o: Vec3, n: Vec3 },
    Segments { segs: Vec<(Vec3, Vec3)>, dir: DirMode },
}

enum DirMode {
    Normal,
    Fixed(Vec3),
}

fn plane_n(b: &Body, f: FaceId) -> Option<(Vec3, Vec3)> {
    match &b.face(f).surface {
        Surface::Plane { frame } => Some((frame.o, b.planar_normal(f))),
        _ => None,
    }
}

/// The cut line of `cutter` on the planar face `f`, or the reason there is none.
fn lines_on(b: &Body, f: FaceId, cutter: &Cutter) -> Result<Vec<Line>, FeatureError> {
    let (fo, fnrm) = plane_n(b, f).ok_or_else(|| not_impl("splitLine.surface"))?;
    match cutter {
        Cutter::Plane { o, n } => {
            let u = fnrm.cross(*n);
            if u.len() < 1e-9 {
                return Ok(vec![]); // parallel to the face: no crossing line
            }
            let (d1, d2) = (fnrm.dot(fo), n.dot(*o));
            let p0 = (n.cross(u) * d1 + u.cross(fnrm) * d2) / u.dot(u);
            Ok(vec![Line { p0, dir: u.normalize(), limit: None }])
        }
        Cutter::Segments { segs, dir } => {
            let mut out = vec![];
            for &(a, c) in segs {
                let dv = match dir {
                    DirMode::Normal => fnrm,
                    DirMode::Fixed(v) => *v,
                };
                let k = dv.dot(fnrm);
                if k.abs() < 1e-9 {
                    return Err(invalid("direction", "the projection direction lies in the face"));
                }
                let proj = |p: Vec3| p + dv * ((fo - p).dot(fnrm) / k);
                let (pa, pc) = (proj(a), proj(c));
                let len = pa.dist(pc);
                if len < 1e-9 {
                    return Err(invalid("sketch", "a sketch line projects to a point on the face"));
                }
                out.push(Line { p0: pa, dir: (pc - pa) / len, limit: Some((0.0, len)) });
            }
            Ok(out)
        }
    }
}

struct Cross {
    s: f64,
    vertex: Option<VertexId>,
    edge: EdgeId,
    t: f64,
}

/// Crossings of the boundary of `f` with the line, sorted along it.
fn crossings(b: &Body, f: FaceId, ln: &Line, fnrm: Vec3) -> Vec<Cross> {
    let n = ln.dir.cross(fnrm).normalize();
    let frame = Frame { o: ln.p0, x: ln.dir, y: n.cross(ln.dir), z: n };
    let pl = Surface::Plane { frame };
    let mut out: Vec<Cross> = vec![];
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        let r = cs::intersect(&ed.curve, (ed.t0, ed.t1), &pl, TOL);
        let mut hits: Vec<(f64, Vec3)> = r.hits.iter().map(|h| (h.t, h.p)).collect();
        for (a, c) in &r.on {
            hits.push((*a, ed.curve.eval(*a)));
            hits.push((*c, ed.curve.eval(*c)));
        }
        for (t, p) in hits {
            // a point on the plane of the line but off the line itself (the hit is on the face plane by construction)
            let vertex = if p.dist(b.vertex(ed.v0).p) < TOL {
                Some(ed.v0)
            } else if p.dist(b.vertex(ed.v1).p) < TOL {
                Some(ed.v1)
            } else {
                None
            };
            out.push(Cross { s: (p - ln.p0).dot(ln.dir), vertex, edge: e, t });
        }
    }
    out.sort_by(|a, c| a.s.total_cmp(&c.s));
    // one crossing per place; an existing vertex wins over an edge point
    let mut dedup: Vec<Cross> = vec![];
    for c in out {
        match dedup.last_mut() {
            Some(l) if (c.s - l.s).abs() < TOL => {
                if l.vertex.is_none() && c.vertex.is_some() {
                    *l = c;
                }
            }
            _ => dedup.push(c),
        }
    }
    dedup
}

fn inside(b: &Body, f: FaceId, p: Vec3) -> bool {
    let mut cache = Default::default();
    point_in_face_cached(b, f, p, TOL, &mut cache) == Where::In
}

/// Split `f` (and the pieces it makes) along every chord of `ln`. Returns the number of chords cut.
fn cut_face(b: &mut Body, f: FaceId, ln: &Line, pieces: &mut Vec<FaceId>) -> Result<usize, FeatureError> {
    let fnrm = b.planar_normal(f);
    let cr = crossings(b, f, ln, fnrm);
    // chords: consecutive crossings whose stretch lies in the face (judged on the face as it is now)
    let mut chords: Vec<(usize, usize)> = vec![];
    for i in 0..cr.len().saturating_sub(1) {
        let mid = ln.p0 + ln.dir * (0.5 * (cr[i].s + cr[i + 1].s));
        if cr[i + 1].s - cr[i].s > TOL && inside(b, f, mid) {
            chords.push((i, i + 1));
        }
    }
    if let Some((lo, hi)) = ln.limit {
        let mut kept = vec![];
        for &(i, j) in &chords {
            let (a, c) = (cr[i].s, cr[j].s);
            if c < lo + TOL || a > hi - TOL {
                continue; // the line does not reach this chord
            }
            if a < lo - TOL || c > hi + TOL {
                return Err(invalid("sketch", "a sketch line ends inside the face: a split line runs from edge to edge"));
            }
            kept.push((i, j));
        }
        chords = kept;
    }
    if chords.is_empty() {
        return Ok(0);
    }
    // materialise the vertices: edges split from the far end so the earlier parameters stay valid
    let mut vs: Vec<Option<VertexId>> = cr.iter().map(|c| c.vertex).collect();
    let mut need: Vec<usize> = chords.iter().flat_map(|&(i, j)| [i, j]).filter(|&k| vs[k].is_none()).collect();
    need.sort_unstable();
    need.dedup();
    need.sort_by(|&p, &q| cr[q].edge.idx().cmp(&cr[p].edge.idx()).then(cr[q].t.total_cmp(&cr[p].t)));
    for k in need {
        let (v, _) = ck_topo::edit::split_edge(b, cr[k].edge, cr[k].t);
        vs[k] = Some(v);
    }
    let mut n = 0;
    for (i, j) in chords {
        let (va, vb) = (vs[i].unwrap(), vs[j].unwrap());
        let mid = ln.p0 + ln.dir * (0.5 * (cr[i].s + cr[j].s));
        let mut cands = vec![f];
        cands.extend(pieces.iter().copied());
        let host = cands.into_iter().find(|&c| b.faces[c.idx()].alive && inside(b, c, mid)).ok_or_else(|| FeatureError::new(codes::INVALID_VALUE))?;
        let name = b.face(host).name.clone();
        let curve = Curve::Line { o: ln.p0, d: ln.dir };
        match ck_topo::edit::split_face(b, host, va, vb, curve, cr[i].s, cr[j].s, None, name) {
            Ok((_, nf)) => {
                pieces.push(nf);
                n += 1;
            }
            Err(_) => return Err(invalid("faces", "a split line must join two points of one boundary loop of the face; this one runs between loops")),
        }
    }
    Ok(n)
}

fn plane_cutter(br: &Brep, state: &RegenState, f: &Feature) -> Result<Cutter, FeatureError> {
    let r = f.raw.get("plane").and_then(GeomRef::from_json).ok_or_else(|| invalid("plane", "a plane is a standard plane, a reference plane or a planar face"))?;
    let fr = super::pattern::plane_frame(br, state, &r).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let mut n = fr.z.normalize();
    if crate::sketch::jtruthy(f.raw.get("flip")) {
        n = n * -1.0;
    }
    let offset = state.num(f, "offset", 0.0)?;
    if !offset.is_finite() {
        return Err(invalid("offset", "offset is a finite number"));
    }
    Ok(Cutter::Plane { o: fr.o + n * offset, n })
}

fn sketch_cutter(state: &RegenState, f: &Feature, id: &str) -> Result<Cutter, FeatureError> {
    let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let dir = match f.raw.get("direction") {
        None => DirMode::Normal,
        Some(Json::Str(s)) if s == "normal" => DirMode::Normal,
        Some(Json::Str(s)) if s == "sketch" => DirMode::Fixed(sk.frame.z.normalize()),
        Some(Json::Arr(a)) if a.len() == 3 => {
            let c: Vec<f64> = a.iter().filter_map(|x| x.as_f64()).collect();
            if c.len() != 3 || c.iter().any(|x| !x.is_finite()) {
                return Err(invalid("direction", "direction is normal, sketch or [x, y, z]"));
            }
            let v = Vec3::new(c[0], c[1], c[2]);
            DirMode::Fixed(v.try_normalize().ok_or_else(|| invalid("direction", "direction is not a zero vector"))?)
        }
        Some(_) => return Err(invalid("direction", "direction is normal, sketch or [x, y, z]")),
    };
    let mut segs = vec![];
    for e in &sk.sketch.entities {
        if e.construction() {
            continue;
        }
        let Some(pts) = super::curve_ref::sketch_entity_samples(sk, e.id()) else { continue };
        let (a, c) = (pts[0], pts[pts.len() - 1]);
        let d = (c - a).try_normalize().ok_or_else(|| not_impl("splitLine.curve"))?;
        if pts.iter().any(|p| (*p - a).cross(d).len() > 1e-6) {
            return Err(not_impl("splitLine.curve"));
        }
        segs.push((a, c));
    }
    if segs.is_empty() {
        return Err(invalid("sketch", "the sketch has no line to cut along"));
    }
    Ok(Cutter::Segments { segs, dir })
}

pub fn exec_split_line(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(mut body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let names: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    if names.is_empty() {
        return Err(invalid("faces", "name the faces to split"));
    }
    let mut targets: Vec<FaceId> = vec![];
    for n in names {
        let fid = body.find_face(n).ok_or_else(|| br.missing_face(n))?;
        if !targets.contains(&fid) {
            targets.push(fid);
        }
    }
    let cutter = if f.raw.get("plane").is_some() {
        plane_cutter(br, state, f)?
    } else if let Some(id) = f.raw.get("sketch").and_then(|s| s.as_str()) {
        sketch_cutter(state, f, id)?
    } else {
        return Err(invalid("plane", "give a plane or a sketch to cut along"));
    };
    for &t in &targets {
        if plane_n(&body, t).is_none() {
            return Err(not_impl("splitLine.surface"));
        }
    }
    let mut cuts = 0;
    for t in targets {
        let lines = lines_on(&body, t, &cutter)?;
        let mut pieces: Vec<FaceId> = vec![];
        for ln in &lines {
            // the pieces an earlier line made are cut by this one as well
            let mut hosts = vec![t];
            hosts.extend(pieces.iter().copied());
            for h in hosts {
                cuts += cut_face(&mut body, h, ln, &mut pieces)?;
            }
        }
    }
    if cuts == 0 {
        return Err(FeatureError::new(super::CUT_NO_EFFECT));
    }
    super::renumber_duplicates(&mut body);
    br.body = Some(body);
    Ok(())
}
