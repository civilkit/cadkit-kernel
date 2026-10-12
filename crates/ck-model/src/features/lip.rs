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
    start: VertexId,
    end: VertexId,
}

/// Face id without its split suffix: `F2.S.e1~1` -> `F2.S.e1` (a side face
/// keeps `~k` inside its entity, so strip it textually).
fn base_id(name: &str) -> String {
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
        let eid = ck_ops::blend::build::find_edge(&body, a, b, at).ok_or_else(missing)?;
        let (fa, fb) = (body.find_face(a).ok_or_else(missing)?, body.find_face(b).ok_or_else(missing)?);
        let (rim, other) = if rim_faces.contains(&fa) {
            (fa, b)
        } else if rim_faces.contains(&fb) {
            (fb, a)
        } else {
            return Err(missing());
        };
        picked.push((eid, rim, base_id(other)));
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
        path.push(PathEdge { tag: other.clone(), pieces, start: body.coedge_start(co), end: body.coedge_end(co) });
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
        // original side
        let orig: Vec<ProfileSeg> = ent_pieces.iter().map(|(t, p)| seg_of_piece(&format!("{t}r"), p)).collect();
        let offs: Vec<ProfileSeg> = off_chain
            .iter()
            .zip(&ent_pieces)
            .flat_map(|(o, (t, _))| segs_of_off(&format!("{t}l"), o))
            .collect();
        if closed {
            let a_o = ProfileLoop::new(orig).signed_area();
            let (outer, hole) = if a_o >= 0.0 {
                (ProfileLoop::new(ent_pieces.iter().map(|(t, p)| seg_of_piece(&format!("{t}r"), p)).collect()), ProfileLoop::new(offs))
            } else {
                (ProfileLoop::new(offs), ProfileLoop::new(ent_pieces.iter().map(|(t, p)| seg_of_piece(&format!("{t}r"), p)).collect()))
            };
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
    br.combine(&tool, if groove { Combine::Cut } else { Combine::Add }, false)
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
