//! Import honesty: what became of each part of an imported file. A part whose faces do not make a valid solid is not
//! kept as one: it becomes a surface body of the faces that were read, and the reasons say why (validator findings
//! in plain words, faces the reader left out). A part with no face at all is refused. Never a mesh, never a silently
//! half-built solid.

use ck_check::Issue;
use ck_topo::body::{Body, EdgeId, FaceId, LumpId, VertexId};
use std::collections::HashMap;

/// What became of a part.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// A valid closed solid: it joins the solid body.
    Solid,
    /// Faces came in but do not make a valid solid: a surface body of those faces.
    Surface,
    /// Nothing came in.
    Refused,
}

impl Outcome {
    pub fn as_str(self) -> &'static str {
        match self {
            Outcome::Solid => "solid",
            Outcome::Surface => "surface",
            Outcome::Refused => "refused",
        }
    }
}

/// Why a part is not a solid: a stable code, the faces (or edges) concerned and a plain-words message.
#[derive(Debug, Clone, PartialEq)]
pub struct Reason {
    pub code: String,
    pub faces: Vec<String>,
    pub message: String,
}

/// A part as the reader gives it: its faces in the imported body, how many the file gives it and the faces the reader
/// left out (as reasons).
#[derive(Debug, Clone)]
pub struct PartIn {
    pub label: String,
    pub faces: Vec<FaceId>,
    pub faces_total: usize,
    pub reasons: Vec<Reason>,
}

#[derive(Debug, Clone)]
pub struct Part {
    pub label: String,
    pub outcome: Outcome,
    /// Faces that came in.
    pub faces: usize,
    /// Faces the file gives the part.
    pub faces_total: usize,
    pub reasons: Vec<Reason>,
}

#[derive(Debug, Clone)]
pub struct Sorted {
    /// The valid solids (the input body unchanged when every part is a solid).
    pub solid: Body,
    /// One surface body per part that is not a solid: (index into `parts`, body).
    pub surfaces: Vec<(usize, Body)>,
    pub parts: Vec<Part>,
}

/// The reason code of a part the file gives as surfaces: informative only (see [`sort`]).
pub const SURFACE_MODEL: &str = "surfaceModel";

/// At most this many entities are named per reason (the count is in the message).
const MAX_NAMED: usize = 20;

/// Plain words for a validator finding seen `n` times.
pub fn plain(code: &str, n: usize) -> String {
    let s = |one: &str, many: &str| if n == 1 { one.to_string() } else { many.replace("{n}", &n.to_string()) };
    match code {
        "SHELL_OPEN" => s("the faces do not close up: an edge belongs to one face only", "the faces do not close up: {n} edges belong to one face only"),
        "SHELL_NONMANIFOLD" => s("an edge is shared by more than two faces", "{n} edges are shared by more than two faces"),
        "SHELL_ORIENTATION" => s("two neighbouring faces face opposite ways", "{n} pairs of neighbouring faces face opposite ways"),
        "SHELL_DISCONNECTED" => "the faces fall apart into pieces that do not touch".into(),
        "SHELL_EMPTY" => "a shell has no faces".into(),
        "VOLUME_NONPOSITIVE" => "the faces enclose no volume (open, or turned inside out)".into(),
        "EULER" => "the faces, edges and vertices do not count up to a closed solid".into(),
        "SOLID_MULTIPLE_OUTERS" => "the solid has more than one outer shell".into(),
        "EDGE_OFF_SURFACE" => s("an edge lies off its faces by more than the tolerance", "{n} edges lie off their faces by more than the tolerance"),
        "VERTEX_OFF_CURVE" => s("a vertex lies off its edge by more than the tolerance", "{n} vertices lie off their edges by more than the tolerance"),
        "EDGE_TOLERANCE_EXCEEDED" | "VERTEX_TOLERANCE_EXCEEDED" => s("an edge or vertex needs a tolerance above the limit", "{n} edges or vertices need a tolerance above the limit"),
        "EDGE_DEGENERATE" => s("an edge has no length", "{n} edges have no length"),
        "EDGE_RANGE" => s("an edge's range does not match its curve", "{n} edges' ranges do not match their curves"),
        "EDGE_UNUSED" => s("an edge belongs to no face", "{n} edges belong to no face"),
        "LOOP_OPEN" => s("a face boundary does not close", "{n} face boundaries do not close"),
        "LOOP_EMPTY" | "FACE_NO_LOOP" => s("a face has no boundary", "{n} faces have no boundary"),
        "LOOP_CROSS" | "LOOP_SELF_INTERSECT" => s("a face boundary crosses itself or another", "{n} face boundaries cross"),
        _ => format!("the validator found {n} problem(s) ({code})"),
    }
}

/// Validator findings as reasons: one per code, in plain words, naming the entities.
pub fn reasons_of(issues: &[Issue]) -> Vec<Reason> {
    let mut by: Vec<(&'static str, Vec<String>)> = vec![];
    for i in issues {
        match by.iter_mut().find(|(c, _)| *c == i.code) {
            Some((_, v)) => v.push(i.entity.clone()),
            None => by.push((i.code, vec![i.entity.clone()])),
        }
    }
    by.into_iter()
        .map(|(code, mut ents)| {
            let n = ents.len();
            ents.sort();
            ents.dedup();
            ents.truncate(MAX_NAMED);
            Reason { code: code.to_string(), faces: ents, message: plain(code, n) }
        })
        .collect()
}

/// A new body made of `faces` of `body` (only the entities they use are copied; lumps rebuilt from connectivity).
pub fn extract_faces(body: &Body, faces: &[FaceId]) -> Body {
    let mut b = Body::new();
    let l = b.add_lump();
    let sh = b.add_shell(l);
    let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
    let mut emap: HashMap<EdgeId, EdgeId> = HashMap::new();
    for &f in faces {
        let fc = body.face(f);
        if !fc.alive {
            continue;
        }
        let nf = b.add_face(sh, fc.surface.clone(), fc.sense, fc.name.clone());
        b.faces[nf.idx()].absorbed = fc.absorbed.clone();
        for &lp in &fc.loops {
            let nl = b.add_loop(nf);
            for &c in &body.lp(lp).coedges {
                let co = body.coedge(c);
                let ne = match emap.get(&co.edge) {
                    Some(&e) => e,
                    None => {
                        let e = body.edge(co.edge);
                        let mut v = |x: VertexId, b: &mut Body| {
                            *vmap.entry(x).or_insert_with(|| {
                                let nv = b.add_vertex(body.vertex(x).p);
                                b.vertices[nv.idx()].tol = body.vertex(x).tol;
                                nv
                            })
                        };
                        let (v0, v1) = (v(e.v0, &mut b), v(e.v1, &mut b));
                        let ne = b.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1);
                        b.edges[ne.idx()].tol = e.tol;
                        emap.insert(co.edge, ne);
                        ne
                    }
                };
                b.add_coedge(nl, ne, co.rev, co.pcurve.clone());
            }
        }
    }
    if b.num_faces() > 0 {
        ck_topo::region::rebuild_lumps(&mut b);
    }
    b
}

/// A copy of `body` holding only lump `keep`.
pub fn extract_lump(body: &Body, keep: LumpId) -> Body {
    extract_faces(body, &ck_topo::mass::lump_faces(body, keep))
}

/// Sort the parts of an imported `body`: each part's faces are checked on their own (strict tolerance 1e-6, edge /
/// vertex tolerance cap `cap`, as the heal pass judges); a part with no finding and no face left out is a solid, any
/// other part with faces a surface body, a part with no face refused.
pub fn sort(body: &Body, parts: &[PartIn], cap: f64) -> Sorted {
    let mut out = Sorted { solid: Body::new(), surfaces: vec![], parts: vec![] };
    let mut solids: Vec<Body> = vec![];
    for (k, p) in parts.iter().enumerate() {
        let alive: Vec<FaceId> = p.faces.iter().copied().filter(|f| f.idx() < body.faces.len() && body.face(*f).alive).collect();
        let mut reasons = p.reasons.clone();
        if alive.is_empty() {
            if reasons.is_empty() {
                reasons.push(Reason { code: "noFaces".into(), faces: vec![], message: "no face of the part could be read".into() });
            }
            out.parts.push(Part { label: p.label.clone(), outcome: Outcome::Refused, faces: 0, faces_total: p.faces_total, reasons });
            continue;
        }
        let sub = extract_faces(body, &alive);
        let issues = ck_check::validate_cap(&sub, 1e-6, cap);
        // `surfaceModel` (the file gives the part as surfaces) only explains: a closed, valid surface model is a solid
        let blocking = reasons.iter().any(|r| r.code != SURFACE_MODEL);
        let outcome = if issues.is_empty() && !blocking { Outcome::Solid } else { Outcome::Surface };
        if outcome == Outcome::Solid {
            reasons.clear();
        }
        reasons.extend(reasons_of(&issues));
        out.parts.push(Part { label: p.label.clone(), outcome, faces: alive.len(), faces_total: p.faces_total, reasons });
        match outcome {
            Outcome::Solid => solids.push(sub),
            _ => out.surfaces.push((k, sub)),
        }
    }
    let covered: usize = parts.iter().map(|p| p.faces.len()).sum();
    if out.surfaces.is_empty() && covered == body.num_faces() {
        // every part is a solid: the body as it came in (face ids unchanged)
        out.solid = body.clone();
    } else {
        for s in &solids {
            out.solid.append(s);
        }
    }
    out
}

/// A reader's left-out face as a reason.
pub fn refused_face(code: &str, face: impl Into<String>, message: &str) -> Reason {
    let face = face.into();
    Reason { code: code.to_string(), message: format!("face {face} could not be read: {message}"), faces: vec![face] }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::{Frame, Vec2, Vec3};
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    fn block(x0: f64) -> Body {
        let p = [Vec2::new(x0, 0.0), Vec2::new(x0 + 10.0, 0.0), Vec2::new(x0 + 10.0, 10.0), Vec2::new(x0, 10.0)];
        let tags = ["e1", "e2", "e3", "e4"];
        let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        extrude("F1", &Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }]), ExtrudeOpts::blind(10.0)).unwrap()
    }

    fn part(b: &Body, faces: Vec<FaceId>, label: &str) -> PartIn {
        PartIn { label: label.into(), faces_total: faces.len(), faces, reasons: vec![] }
    }

    #[test]
    fn a_valid_solid_stays_a_solid_unchanged() {
        let b = block(0.0);
        let s = sort(&b, &[part(&b, b.face_ids().collect(), "A")], ck_check::DEFAULT_TOL_CAP);
        assert_eq!(s.parts[0].outcome, Outcome::Solid);
        assert!(s.surfaces.is_empty());
        assert_eq!(s.solid.num_faces(), 6);
        assert!(s.parts[0].reasons.is_empty());
    }

    #[test]
    fn a_part_with_a_face_left_out_is_a_surface_body_with_reasons() {
        let mut b = block(0.0);
        b.append(&block(20.0));
        let all: Vec<FaceId> = b.face_ids().collect();
        let (a, mut c) = (all[..6].to_vec(), all[6..].to_vec());
        c.remove(0); // the reader could not read this face
        let mut pc = part(&b, c, "B");
        pc.faces_total = 6;
        pc.reasons.push(refused_face("unsupported", "#106", "DEGENERATE_TOROIDAL_SURFACE is not supported"));
        let s = sort(&b, &[part(&b, a, "A"), pc], ck_check::DEFAULT_TOL_CAP);
        assert_eq!(s.parts[0].outcome, Outcome::Solid);
        assert_eq!(s.parts[1].outcome, Outcome::Surface);
        assert_eq!((s.parts[1].faces, s.parts[1].faces_total), (5, 6));
        let codes: Vec<&str> = s.parts[1].reasons.iter().map(|r| r.code.as_str()).collect();
        assert_eq!(codes[0], "unsupported");
        assert!(codes.contains(&"SHELL_OPEN"), "{codes:?}");
        let open = s.parts[1].reasons.iter().find(|r| r.code == "SHELL_OPEN").unwrap();
        assert!(open.message.contains("4 edges belong to one face only"), "{}", open.message);
        assert!(!open.faces.is_empty());
        // the solid body holds the valid part only, the surface body the five faces
        assert_eq!(s.solid.num_faces(), 6);
        assert_eq!(s.surfaces.len(), 1);
        assert_eq!((s.surfaces[0].0, s.surfaces[0].1.num_faces()), (1, 5));
        assert!((ck_topo::mass::mass_props(&s.solid).volume - 1000.0).abs() < 1e-6);
    }

    #[test]
    fn a_part_with_no_face_is_refused() {
        let b = block(0.0);
        let mut p = part(&b, vec![], "C");
        p.faces_total = 3;
        p.reasons.push(refused_face("topology", "#5", "degenerate seam"));
        let s = sort(&b, &[part(&b, b.face_ids().collect(), "A"), p], ck_check::DEFAULT_TOL_CAP);
        assert_eq!(s.parts[1].outcome, Outcome::Refused);
        assert_eq!(s.parts[1].reasons[0].code, "topology");
        assert!(s.surfaces.is_empty());
    }

    #[test]
    fn extract_keeps_geometry_and_names() {
        let b = block(0.0);
        let fs: Vec<FaceId> = b.face_ids().collect();
        let e = extract_faces(&b, &fs);
        assert_eq!(e.num_faces(), 6);
        assert!(ck_check::validate(&e, 1e-6).is_empty());
        assert_eq!(b.face(fs[2]).name, e.face(e.face_ids().nth(2).unwrap()).name);
        let _ = Vec3::ZERO;
    }
}
