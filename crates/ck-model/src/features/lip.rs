//! `lip` (RECONSTRUCTION from oracle probes, `tools/brep_cases.mjs` `lip*`).
//!
//! Payload: `kind` `"lip"` (raised) / `"groove"` (sunk), `faces` (the rim
//! faces, base ids: `F24.F` matches the split pieces `F24.F~k`), `edges`
//! (`{faces:[rim, other], at}` path), `width`, `height`.
//!
//! Pinned (shelled 40x20x10 box, rim face `F2.C1`, outer edges):
//! * errors in order: no body `nothingToCut`; no / unresolved rim face
//!   `lipFace`; `width`/`height` <= 0 `invalidValue`; no / unresolved edge
//!   `missingReference`;
//! * the tool is the band of `width` between the path and its inward
//!   offset (sketch offset: left of the travel direction, rim face on the
//!   left), extruded `height` OUT of the rim face; a lip is united, a
//!   groove (same band, cut `height` INTO the rim face) is subtracted;
//! * side faces `F<n>.S.<other face id without ~k>l` (the offset side),
//!   `C1` (lip top) / `C0` (groove floor); the path side and the band ends
//!   coincide with the neighbouring faces and merge into them.
//!
//! GUESS / gaps: planar rim faces only; band ends are the perpendicular of
//! the path ends; spline edges are sampled into 16 line pieces per edge.

use super::extrude::Combine;
use super::{Brep, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::sketch::offset::{sketch_offset, OffSeg};
use crate::sketch::Sketch;
use crate::Feature;
use ck_geom::curve::Curve;
use ck_geom::{Frame, Vec2, Vec3};
use ck_json::{Json, Obj};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, FaceName, VertexId};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

pub const LIP_FACE: &str = "lipFace";

/// A recorded pick point lies on its edge within this distance.
const PICK_TOL: f64 = 1e-3;

/// One path piece in rim-plane 2D, in travel direction (rim face on the left).
#[derive(Clone)]
enum Piece {
    Line { a: Vec2, b: Vec2 },
    /// CCW arc about `c` from `a` to `b` when `ccw`, else CW travel
    Arc { c: Vec2, a: Vec2, b: Vec2, ccw: bool },
}

struct PathEdge {
    tag: String,
    pieces: Vec<Piece>,
    /// A B-spline rim edge, exact in the rim plane: curve and travel
    /// parameters `(from, to)` (`from > to` when travelled backwards)
    exact: Option<(ck_geom::nurbs::bspline::BSplineCurve<Vec2>, f64, f64)>,
    start: VertexId,
    end: VertexId,
}

/// Face id without its split suffix: `F2.S.e1~1` -> `F2.S.e1` (a side face
/// keeps `~k` inside its entity, so strip it textually).
pub(super) fn base_id(name: &str) -> String {
    let (head, inst) = match name.find('@') {
        Some(i) => (&name[..i], &name[i..]),
        None => (name, ""),
    };
    let cut = match head.rfind('~') {
        Some(i) if i + 1 < head.len() && head[i + 1..].bytes().all(|b| b.is_ascii_digit()) => &head[..i],
        _ => head,
    };
    format!("{cut}{inst}")
}

pub fn exec_lip(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Lip { kind, faces, edges, .. } = &f.kind else { unreachable!() };
    let Some(body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let rim_bases: Vec<String> = faces.iter().map(|r| base_id(r.as_str())).collect();
    let rim_faces: Vec<FaceId> = body.face_ids().filter(|&x| rim_bases.contains(&base_id(&body.face_id_str(x)))).collect();
    if faces.is_empty() || rim_faces.is_empty() {
        return Err(FeatureError::new(LIP_FACE));
    }
    let width = state.num(f, "width", 1.0)?;
    let height = state.num(f, "height", 1.0)?;
    if !(width > 0.0) || !(height > 0.0) {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let missing = || FeatureError::new(codes::MISSING_REFERENCE);
    if edges.is_empty() {
        return Err(missing());
    }
    // path edges with their rim / neighbour faces
    let mut picked: Vec<(EdgeId, FaceId, String)> = vec![];
    for e in edges {
        let (a, b) = (e.faces.first().ok_or_else(missing)?, e.faces.get(1).ok_or_else(missing)?);
        let at = e.at.map(|p| Vec3::new(p[0], p[1], p[2])).unwrap_or(Vec3::ZERO);
        // Split faces are numbered `~k` by an ordering the oracle derives from
        // its own control nets (inner / outer of a shell, blend pieces): ours can
        // differ. The pick point is on the wanted edge, so an exact-name edge that
        // misses it is replaced by the edge between the same face FAMILIES nearest
        // to it.
        let dist = |e: EdgeId| {
            let ed = body.edge(e);
            ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
        };
        let exact = ck_ops::blend::build::find_edge(&body, a, b, at).filter(|&e| dist(e) < PICK_TOL);
        let eid = match exact {
            Some(e) => e,
            None => {
                let (ba, bb) = (base_id(a), base_id(b));
                body.edge_ids()
                    .filter(|&e| {
                        let fs = body.edge_faces(e);
                        fs.len() == 2 && {
                            let (x, y) = (base_id(&body.face_id_str(fs[0])), base_id(&body.face_id_str(fs[1])));
                            (x == ba && y == bb) || (x == bb && y == ba)
                        }
                    })
                    .min_by(|&x, &y| dist(x).total_cmp(&dist(y)))
                    .filter(|&e| dist(e) < PICK_TOL)
                    .or_else(|| ck_ops::blend::build::find_edge(&body, a, b, at))
                    .ok_or_else(missing)?
            }
        };
        let fs = body.edge_faces(eid);
        let in_rim = |f: FaceId| rim_faces.contains(&f);
        let (rim, other) = match (fs.first().copied(), fs.get(1).copied()) {
            (Some(f0), Some(f1)) if in_rim(f0) => (f0, body.face_id_str(f1)),
            (Some(f0), Some(f1)) if in_rim(f1) => (f1, body.face_id_str(f0)),
            _ => return Err(missing()),
        };
        picked.push((eid, rim, base_id(&other)));
    }
    let rim = picked[0].1;
    // several rim faces are fine while they are pieces of one plane
    let same_plane = |a: FaceId, b: FaceId| {
        a == b || {
            use ck_geom::surface::Surface::Plane;
            matches!((&body.face(a).surface, &body.face(b).surface), (Plane { .. }, Plane { .. }))
                && body.planar_normal(a).dot(body.planar_normal(b)) > 1.0 - 1e-9
                && (body.vertex(body.edge(picked[0].0).v0).p - body.face(b).surface.closest_point(body.vertex(body.edge(picked[0].0).v0).p).2).len() < 1e-6
        }
    };
    if !matches!(body.face(rim).surface, ck_geom::surface::Surface::Plane { .. }) || picked.iter().any(|p| !same_plane(p.1, rim)) {
        return Err(FeatureError::new(&super::extrude::todo("lip on a curved / several rim faces")));
    }
    let n = body.planar_normal(rim);
    let o0 = body.vertex(body.edge(picked[0].0).v0).p;
    let ex = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (ex - n * ex.dot(n)).normalize();
    let frame = Frame { o: o0, x, y: n.cross(x), z: n };
    let to2 = |p: Vec3| Vec2::new((p - frame.o).dot(frame.x), (p - frame.o).dot(frame.y));

    let mut path: Vec<PathEdge> = vec![];
    for (eid, rimf, other) in &picked {
        let co: CoedgeId = body
            .edge_coedges(*eid)
            .into_iter()
            .find(|&c| body.coedge(c).alive && body.coedge_face(c) == *rimf)
            .ok_or_else(missing)?;
        let (t0, t1) = body.coedge_params(co);
        let ed = body.edge(*eid);
        let (pa, pb) = (ed.curve.eval(t0), ed.curve.eval(t1));
        let exact = match &ed.curve {
            Curve::BSpline(bs) if ed.v0 != ed.v1 => {
                let c2 = bs.map(|q: Vec3| to2(q)).segment(t0.min(t1), t0.max(t1));
                Some((c2, t0, t1))
            }
            _ => None,
        };
        let pieces = match &ed.curve {
            Curve::Line { .. } => vec![Piece::Line { a: to2(pa), b: to2(pb) }],
            Curve::Circle { frame: cf, .. } if cf.z.cross(n).len() < 1e-9 => {
                let c = to2(cf.o);
                // travel direction around the circle: sign of (travel param dir) * (cf.z . n)
                let along = if t1 >= t0 { 1.0 } else { -1.0 };
                let ccw = along * cf.z.dot(n) > 0.0;
                vec![Piece::Arc { c, a: to2(pa), b: to2(pb), ccw }]
            }
            curve => {
                let k = 16;
                (0..k)
                    .map(|i| {
                        let (u, v) = (t0 + (t1 - t0) * i as f64 / k as f64, t0 + (t1 - t0) * (i + 1) as f64 / k as f64);
                        Piece::Line { a: to2(curve.eval(u)), b: to2(curve.eval(v)) }
                    })
                    .collect()
            }
        };
        path.push(PathEdge { tag: other.clone(), pieces, exact, start: body.coedge_start(co), end: body.coedge_end(co) });
    }

    if std::env::var_os("CK_LIP_DBG").is_some() {
        for (i, pe) in path.iter().enumerate() {
            eprintln!("LIP path edge {i}: {} v {:?} -> {:?} at {:?} -> {:?}", pe.tag, pe.start, pe.end, body.vertex(pe.start).p, body.vertex(pe.end).p);
        }
    }
    // chains head to tail (travel direction); a closed loop starts anywhere
    let mut chains: Vec<Vec<usize>> = vec![];
    let mut used = vec![false; path.len()];
    for s in 0..path.len() {
        if used[s] {
            continue;
        }
        // walk back to the chain head
        let mut head = s;
        let mut guard = 0;
        while let Some(p) = (0..path.len()).find(|&j| !used[j] && j != head && path[j].end == path[head].start) {
            head = p;
            guard += 1;
            if head == s || guard > path.len() {
                // a closed loop starts at its first path edge
                head = s;
                break;
            }
        }
        let mut chain = vec![head];
        used[head] = true;
        while let Some(nx) = (0..path.len()).find(|&j| !used[j] && path[j].start == path[*chain.last().unwrap()].end) {
            used[nx] = true;
            chain.push(nx);
        }
        chains.push(chain);
    }

    // the band profile, one region per chain
    let mut regions: Vec<ProfileRegion> = vec![];
    for chain in &chains {
        // entities for the sketch offset (consecutive pieces share points)
        let mut pts: Vec<(f64, f64)> = vec![];
        let mut ents: Vec<Json> = vec![];
        let mut ent_pieces: Vec<(String, Piece)> = vec![];
        let mut pt_id = |p: Vec2, pts: &mut Vec<(f64, f64)>| -> String {
            if let Some(i) = pts.iter().position(|q| (q.0 - p.x).abs() < 1e-9 && (q.1 - p.y).abs() < 1e-9) {
                return format!("p{i}");
            }
            pts.push((p.x, p.y));
            format!("p{}", pts.len() - 1)
        };
        // consecutive edges meet at a shared vertex but their sampled ends can
        // differ by ~1e-7: snap each piece start onto the previous piece end
        // (the sketch offset only links pieces whose points coincide)
        let mut flat: Vec<(usize, Piece)> = chain.iter().flat_map(|&ei| path[ei].pieces.iter().map(move |p| (ei, p.clone()))).collect();
        let m = flat.len();
        let get_a = |p: &Piece| match p {
            Piece::Line { a, .. } | Piece::Arc { a, .. } => *a,
        };
        let (set_a, get_b): (fn(&mut Piece, Vec2), fn(&Piece) -> Vec2) = (
            |p, v| match p {
                Piece::Line { a, .. } | Piece::Arc { a, .. } => *a = v,
            },
            |p| match p {
                Piece::Line { b, .. } | Piece::Arc { b, .. } => *b,
            },
        );
        for i in 1..m + usize::from(path[*chain.last().unwrap()].end == path[chain[0]].start) {
            let (prev, cur) = (get_b(&flat[(i + m - 1) % m].1), i % m);
            let a = match &flat[cur].1 {
                Piece::Line { a, .. } | Piece::Arc { a, .. } => *a,
            };
            if (a - prev).len() < 1e-4 {
                set_a(&mut flat[cur].1, prev);
            }
        }
        for (ei, pc) in flat.iter().map(|(ei, p)| (*ei, p)) {
            {
                let k = ents.len();
                match pc {
                    Piece::Line { a, b } => {
                        let (p1, p2) = (pt_id(*a, &mut pts), pt_id(*b, &mut pts));
                        ents.push(Obj::new().put("type", "line").put("id", format!("e{k}")).put("p1", p1).put("p2", p2).build());
                    }
                    Piece::Arc { c, a, b, ccw } => {
                        let (pc_id, p1, p2) = (pt_id(*c, &mut pts), pt_id(*a, &mut pts), pt_id(*b, &mut pts));
                        let (q1, q2) = if *ccw { (p1, p2) } else { (p2, p1) };
                        ents.push(Obj::new().put("type", "arc").put("id", format!("e{k}")).put("c", pc_id).put("p1", q1).put("p2", q2).build());
                    }
                }
                ent_pieces.push((path[ei].tag.clone(), pc.clone()));
            }
        }
        let sk_json = Obj::new()
            .put(
                "points",
                Json::Arr(pts.iter().enumerate().map(|(i, p)| Obj::new().put("id", format!("p{i}")).put("x", p.0).put("y", p.1).build()).collect()),
            )
            .put("entities", Json::Arr(ents.clone()))
            .build();
        let sk = Sketch::from_json(&sk_json);
        let ids: Vec<String> = (0..ents.len()).map(|k| format!("e{k}")).collect();
        let off = sketch_offset(&sk, &ids, width).map_err(|e| FeatureError::new(e))?;
        if std::env::var_os("CK_LIP_DBG").is_some() {
            eprintln!("LIP chain of {} edges: {} pieces; offset chains {:?}", chain.len(), ent_pieces.len(), off.iter().map(|c| c.len()).collect::<Vec<_>>());
            for (k, (t, pc)) in ent_pieces.iter().enumerate() {
                match pc {
                    Piece::Arc { c, a, b, ccw } => eprintln!("  piece {k} {t} arc c {:?} r {:.4} a {:?} b {:?} ccw {ccw}", (c.x, c.y), (*a - *c).len(), (a.x, a.y), (b.x, b.y)),
                    Piece::Line { a, b } => eprintln!("  piece {k} {t} line {:?} -> {:?} len {:.4}", (a.x, a.y), (b.x, b.y), (*b - *a).len()),
                }
            }
        }
        let Some(off_chain) = off.into_iter().next() else { return Err(missing()) };
        if off_chain.len() != ent_pieces.len() {
            return Err(FeatureError::new(&super::extrude::todo("lip offset with dropped pieces")));
        }
        let closed = path[*chain.last().unwrap()].end == path[chain[0]].start;
        // original side; a B-spline rim edge stays one exact segment (the
        // tool's path-side face must share the host edge, not a chord polygon),
        // its offset side is fitted through the true offset points
        let mut orig: Vec<ProfileSeg> = vec![];
        let mut offs: Vec<ProfileSeg> = vec![];
        let mut i = 0;
        while i < flat.len() {
            let ei = flat[i].0;
            let mut j = i;
            while j < flat.len() && flat[j].0 == ei {
                j += 1;
            }
            let tag = &path[ei].tag;
            let fitted = path[ei].exact.as_ref().and_then(|(c2, ta, tb)| {
                let (first, last) = (&off_chain[i], &off_chain[j - 1]);
                let (o_start, o_end) = (off_start(first), off_end(last));
                fit_offset(c2, *ta, *tb, width, Vec2::new(o_start[0], o_start[1]), Vec2::new(o_end[0], o_end[1]))
            });
            if let Some(fit) = fitted {
                let (mut c2, ta, tb) = path[ei].exact.clone().unwrap();
                // close the profile exactly: the travel start is the snapped
                // end of the previous piece (a clamped spline's end control
                // points are its ends)
                {
                    let (sp, ep) = (get_a(&flat[i].1), get_b(&flat[j - 1].1));
                    let last = c2.ctrl.len() - 1;
                    let (fi, li) = if ta <= tb { (0, last) } else { (last, 0) };
                    c2.ctrl[fi] = sp;
                    c2.ctrl[li] = ep;
                }
                orig.push(ProfileSeg {
                    tag: format!("{tag}r"),
                    curve: ck_geom::curve2::Curve2::BSpline(c2),
                    t0: ta.min(tb),
                    t1: ta.max(tb),
                    rev: ta > tb,
                });
                let (d0, d1) = fit.domain();
                offs.push(ProfileSeg { tag: format!("{tag}l"), curve: ck_geom::curve2::Curve2::BSpline(fit), t0: d0, t1: d1, rev: false });
            } else {
                for k in i..j {
                    orig.push(seg_of_piece(&format!("{}r", ent_pieces[k].0), &ent_pieces[k].1));
                    offs.extend(segs_of_off(&format!("{}l", ent_pieces[k].0), &off_chain[k]));
                }
            }
            i = j;
        }
        if closed {
            let a_o = ProfileLoop::new(orig.clone()).signed_area();
            let (outer, hole) = if a_o >= 0.0 { (ProfileLoop::new(orig), ProfileLoop::new(offs)) } else { (ProfileLoop::new(offs), ProfileLoop::new(orig)) };
            regions.push(ProfileRegion { outer, holes: vec![hole] });
        } else {
            // pinned loop start: the offset side runs first
            // (`[e1l, end0, e1r, end1]` -> faces C1, S..l, then the merged walls)
            let (oe, fe) = (orig.last().unwrap().end(), offs.last().unwrap().end());
            let (fs, os) = (offs[0].start(), orig[0].start());
            let mut segs: Vec<ProfileSeg> = vec![ProfileSeg::line("end0", fs, os)];
            segs.extend(orig);
            segs.push(ProfileSeg::line("end1", oe, fe));
            segs.extend(offs.iter().rev().map(|s| s.reversed()));
            regions.push(ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] });
        }
    }
    // outer loops CCW, holes CW; pinned: a reversal is a plain reverse of the
    // segment order (hole `[e1l..e4l]` runs `e4l, e3l, e2l, e1l`)
    let rev = |l: &ProfileLoop| ProfileLoop::new(l.segs.iter().rev().map(|x| x.reversed()).collect());
    for r in &mut regions {
        if r.outer.signed_area() < 0.0 {
            r.outer = rev(&r.outer);
        }
        for h in &mut r.holes {
            if h.signed_area() > 0.0 {
                *h = rev(h);
            }
        }
    }
    let profile = Profile::new(frame, regions);
    let groove = kind == "groove";
    let opts = if groove { ExtrudeOpts::flipped(height) } else { ExtrudeOpts::blind(height) };
    let tool = extrude(&f.id, &profile, opts).map_err(|e| FeatureError::new(&e))?;
    // a tool built on exact spline rim curves welds to the host edges only to the accuracy of the
    // marched intersections: start at the looser tolerance the boolean ends up needing (the
    // large sample lip spent two thirds of its time in failed attempts at the tight ones)
    let tol = if path.iter().any(|p| p.exact.is_some()) { super::TOL * 100.0 } else { super::TOL };
    br.combine_at(&tool, if groove { Combine::Cut } else { Combine::Add }, false, tol)
}

fn off_start(o: &OffSeg) -> [f64; 2] {
    match o {
        OffSeg::Line { a, .. } | OffSeg::Arc { a, .. } => *a,
        OffSeg::Circle { c, r } => [c[0] + r, c[1]],
        OffSeg::Curve { pts, .. } => pts[0],
    }
}

fn off_end(o: &OffSeg) -> [f64; 2] {
    match o {
        OffSeg::Line { b, .. } | OffSeg::Arc { b, .. } => *b,
        OffSeg::Circle { c, r } => [c[0] + r, c[1]],
        OffSeg::Curve { pts, .. } => pts[pts.len() - 1],
    }
}

/// The offset of a rim B-spline travelled from `ta` to `tb` by `width` to the
/// side of the sketch offset (`o_start` / `o_end`: its trimmed ends), as a
/// cubic fitted through dense true offset points (error far below the model
/// tolerance). `None` when the fit fails or the sides disagree.
fn fit_offset(
    c2: &ck_geom::nurbs::bspline::BSplineCurve<Vec2>,
    ta: f64,
    tb: f64,
    width: f64,
    o_start: Vec2,
    o_end: Vec2,
) -> Option<ck_geom::nurbs::bspline::BSplineCurve<Vec2>> {
    let n = ((c2.length_between(ta.min(tb), ta.max(tb)) / 0.4).ceil() as usize).clamp(16, 600);
    let at = |k: usize| ta + (tb - ta) * k as f64 / n as f64;
    // side: the offset point nearest the sketch offset's start
    let nrm = |t: f64| {
        let d = c2.derivs(t, 1)[1];
        let d = if tb >= ta { d } else { -d };
        let l = d.len();
        Vec2::new(-d.y / l, d.x / l)
    };
    let (p0, n0) = (c2.eval(ta), nrm(ta));
    let side = if (p0 + n0 * width - o_start).len() <= (p0 - n0 * width - o_start).len() { 1.0 } else { -1.0 };
    let mut pts: Vec<Vec2> = (0..=n).map(|k| c2.eval(at(k)) + nrm(at(k)) * (width * side)).collect();
    // the trimmed ends of the sketch offset (corners with the neighbours)
    if (pts[0] - o_start).len() > 0.05 * width.max(1e-3) + 1e-3 || (pts[n] - o_end).len() > 0.05 * width.max(1e-3) + 1e-3 {
        // a corner: take the sketch offset's end only if it stays close
        if (pts[0] - o_start).len() > 2.0 * width || (pts[n] - o_end).len() > 2.0 * width {
            return None;
        }
    }
    pts[0] = o_start;
    pts[n] = o_end;
    pts.dedup_by(|a, b| (*a - *b).len() < 1e-9);
    ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength).ok()
}

fn seg_of_piece(tag: &str, p: &Piece) -> ProfileSeg {
    match p {
        Piece::Line { a, b } => ProfileSeg::line(tag, *a, *b),
        Piece::Arc { c, a, b, ccw: true } => ProfileSeg::arc(tag, *c, *a, *b),
        Piece::Arc { c, a, b, ccw: false } => ProfileSeg::arc(tag, *c, *b, *a).reversed(),
    }
}

fn segs_of_off(tag: &str, o: &OffSeg) -> Vec<ProfileSeg> {
    let v = |p: [f64; 2]| Vec2::new(p[0], p[1]);
    match o {
        OffSeg::Line { a, b } => vec![ProfileSeg::line(tag, v(*a), v(*b))],
        OffSeg::Arc { c, a, b, ccw, .. } => {
            if *ccw {
                vec![ProfileSeg::arc(tag, v(*c), v(*a), v(*b))]
            } else {
                vec![ProfileSeg::arc(tag, v(*c), v(*b), v(*a)).reversed()]
            }
        }
        OffSeg::Circle { c, r } => vec![ProfileSeg::circle(tag, v(*c), *r)],
        OffSeg::Curve { pts, .. } => pts.windows(2).map(|w| ProfileSeg::line(tag, v(w[0]), v(w[1]))).collect(),
    }
}

#[allow(dead_code)]
fn _unused(_: &Body) {}
