//! The `matchEdges` request: edges "like" a set of seed edges, for the
//! edge-picking commands ("add similar / parallel / tangent edges").
//!
//! `{part, upTo, mode, seeds:[{faces,at}], exclude:[{faces,at}]}` -> list of
//! `{faces, at}` in the oracle edge form (wire chains: sorted face pair,
//! `at` = chain mid), the union over the seeds in wire order (no seed or
//! excluded edge, no duplicates):
//!
//! * `similar` (also any unknown mode): same curve kind, length and
//!   dihedral angle (convex vs concave edges differ);
//! * `parallel`: line parallel (either direction) to a line seed / circle
//!   with a parallel axis to a circle seed;
//! * `tangent`: the G1 chain through shared end vertices, in breadth-first
//!   order (the other modes are sorted in wire order).
//!
//! GUESS: tolerances (length 1e-6 relative, parallel 1e-9, tangent 1e-3 rad)
//! and the order of tangent chains; pinned by the brep cases
//! (the reference-capture tool (not published) matchEdges`).

use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, EdgeId};

struct Chain {
    edges: Vec<EdgeId>,
    faces: Vec<String>,
    len: f64,
    mid: Vec3,
    /// (point, unit tangent pointing away from the chain) at its two ends
    ends: [(Vec3, Vec3); 2],
    /// dihedral angle inside the material (convex: the wedge angle,
    /// concave: 2pi - wedge) of a two-face edge
    dihedral: Option<f64>,
}

/// Dihedral angle of an edge between exactly two faces.
fn dihedral(body: &Body, e: EdgeId) -> Option<f64> {
    let cs = body.edge_coedges(e);
    if cs.len() != 2 {
        return None;
    }
    let ed = body.edge(e);
    let t = 0.5 * (ed.t0 + ed.t1);
    let nrm = |c| {
        let uv = body.coedge_uv(c, t, None);
        body.outward_normal(body.coedge_face(c), uv.x, uv.y)
    };
    let (n1, n2) = (nrm(cs[0]), nrm(cs[1]));
    // direction of the coedge, interior of face 1 to its left
    let (ta, tb) = body.coedge_params(cs[0]);
    let tan = ed.curve.tangent(t) * if tb >= ta { 1.0 } else { -1.0 };
    let inward1 = n1.cross(tan);
    let wedge = n1.cross(n2).len().atan2(-n1.dot(n2));
    Some(if n2.dot(inward1) < 0.0 { wedge } else { 2.0 * std::f64::consts::PI - wedge })
}

fn kind_of(c: &Curve) -> &'static str {
    c.kind()
}

fn chains(body: &Body) -> Vec<Chain> {
    let mut out = vec![];
    for ch in body.wire_chains() {
        let head = ch[0];
        if body.is_split_rail(head) {
            continue;
        }
        let mut faces: Vec<String> = body.edge_faces(head).iter().map(|&f| body.face_id_str(f)).collect();
        faces.sort();
        let len: f64 = ch.iter().map(|&e| body.edge_length(e)).sum();
        let (f, l) = (body.edge(head), body.edge(*ch.last().unwrap()));
        let t_start = f.curve.tangent(f.t0);
        let t_end = l.curve.tangent(l.t1);
        let ends = [(f.curve.eval(f.t0), -t_start), (l.curve.eval(l.t1), t_end)];
        out.push(Chain { mid: body.chain_mid(&ch), dihedral: dihedral(body, head), edges: ch, faces, len, ends });
    }
    out
}

fn same_dihedral(a: Option<f64>, b: Option<f64>) -> bool {
    match (a, b) {
        (Some(x), Some(y)) => (x - y).abs() < 1e-6,
        (None, None) => true,
        _ => false,
    }
}

fn same_pt(a: Vec3, b: Vec3) -> bool {
    (a - b).len() < 1e-7
}

/// A `{faces, at}` reference -> chain index.
fn find(cs: &[Chain], r: &Json) -> Option<usize> {
    let mut want: Vec<String> = r.get_arr("faces")?.iter().filter_map(|j| j.as_str().map(String::from)).collect();
    want.sort();
    let at = r.get("at").and_then(Json::as_vec3).map(Vec3::from_array);
    let cand = (0..cs.len()).filter(|&i| cs[i].faces == want);
    match at {
        Some(p) => cand.min_by(|&a, &b| (cs[a].mid - p).len().total_cmp(&(cs[b].mid - p).len())),
        None => cand.into_iter().next(),
    }
}

pub fn match_edges(body: &Body, mode: &str, seeds: &[Json], exclude: &[Json]) -> Json {
    let cs = chains(body);
    let seed_ix: Vec<usize> = seeds.iter().filter_map(|s| find(&cs, s)).collect();
    let excl: Vec<usize> = exclude.iter().filter_map(|s| find(&cs, s)).collect();
    let mut out: Vec<usize> = vec![];
    let add = |i: usize, out: &mut Vec<usize>| {
        if !seed_ix.contains(&i) && !excl.contains(&i) && !out.contains(&i) {
            out.push(i);
        }
    };
    for &s in &seed_ix {
        let sc = &cs[s];
        let head = &body.edge(sc.edges[0]).curve;
        match mode {
            "parallel" => {
                // lines: direction; circles: axis
                let axis = |c: &Curve| match c {
                    Curve::Line { d, .. } => Some(("line", d.normalize())),
                    Curve::Circle { frame, .. } => Some(("circle", frame.z.normalize())),
                    _ => None,
                };
                let Some((k, d)) = axis(head) else { continue };
                for (i, c) in cs.iter().enumerate() {
                    if let Some((k2, d2)) = axis(&body.edge(c.edges[0]).curve) {
                        if i != s && k == k2 && d.dot(d2).abs() >= 1.0 - 1e-9 {
                            add(i, &mut out);
                        }
                    }
                }
            }
            "tangent" => {
                let mut seen = vec![s];
                let mut queue = std::collections::VecDeque::from([s]);
                while let Some(i) = queue.pop_front() {
                    for (p, t) in cs[i].ends {
                        for (j, c) in cs.iter().enumerate() {
                            if seen.contains(&j) {
                                continue;
                            }
                            for (q, u) in c.ends {
                                if same_pt(p, q) && t.dot(u).abs() >= 1.0 - 1e-6 {
                                    seen.push(j);
                                    queue.push_back(j);
                                    add(j, &mut out);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            _ => {
                for (i, c) in cs.iter().enumerate() {
                    if i != s && kind_of(&body.edge(c.edges[0]).curve) == kind_of(head) && (c.len - sc.len).abs() <= 1e-6 * sc.len.max(1.0) && same_dihedral(c.dihedral, sc.dihedral) {
                        add(i, &mut out);
                    }
                }
            }
        }
    }
    // the result is the union in wire order (edges are grouped by lump)
    if mode != "tangent" {
        out.sort_unstable();
    }
    Json::Arr(
        out.into_iter()
            .map(|i| {
                let c = &cs[i];
                Obj::new()
                    .put("faces", Json::Arr(c.faces.iter().cloned().map(Json::Str).collect()))
                    .put("at", Json::Arr(vec![Json::Num(c.mid.x), Json::Num(c.mid.y), Json::Num(c.mid.z)]))
                    .build()
            })
            .collect(),
    )
}

/// `matchEdges` message fields -> reply.
pub fn reply(body: &Body, msg: &Json) -> Json {
    let list = |k: &str| msg.get_arr(k).map(|a| a.to_vec()).unwrap_or_default();
    match_edges(body, msg.get_str("mode").unwrap_or("similar"), &list("seeds"), &list("exclude"))
}
