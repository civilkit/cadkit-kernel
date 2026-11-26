//! Fillets and chamfers. A Fusion fillet names its edges by the file's own entity names, which a part cannot select;
//! but each edge also carries its middle point and length. The history written so far is regenerated with the kernel,
//! the edge of the body with that middle point and length is found, and the two faces that meet there (by the names
//! the kernel gave them) are what the part's `fillet` / `chamfer` feature picks, with the point. When the history so far
//! does not have such an edge (an earlier item did not map, or the body differs), the item stays a placeholder.

use ck_f3d_dump::{ChamferDetail, EdgeSet, FilletDetail, Reference};
use serde_json::{Value, json};

use crate::{Builder, CM_TO_MM, guard};

/// An edge picked by its middle point and length (mm) must be within this distance / this relative length.
const POINT_TOL: f64 = 0.05;
const LENGTH_TOL: f64 = 2e-3;

fn value(r: &Option<Reference>) -> Option<f64> {
    match r {
        Some(Reference::Parameter(p)) => p.value.filter(|v| v.is_finite()),
        _ => None,
    }
}

/// The part's pick (`{faces, at}`) of the edge of `body` that lies at `at` (mm) with length `len` (mm). The edge is found by
/// its middle point; a closed edge (a full circle: its middle is the point opposite its seam, which the kernel puts elsewhere
/// than the file does) by any point of the edge when the file's edge is closed too. `taken` holds the edges already picked:
/// the same edge twice is a wrong match, not a pick (`strict`).
pub(crate) fn pick_edge(
    body: &ck_topo::body::Body,
    at: [f64; 3],
    len: Option<f64>,
    closed: bool,
    taken: &mut std::collections::BTreeSet<usize>,
    strict: bool,
) -> Result<Value, String> {
    let mut best: Option<(f64, ck_topo::body::EdgeId)> = None;
    let p = ck_geom::Vec3::new(at[0], at[1], at[2]);
    for e in body.edge_ids() {
        let m = body.edge_mid(e);
        let mut d = ((m.x - at[0]).powi(2) + (m.y - at[1]).powi(2) + (m.z - at[2]).powi(2)).sqrt();
        if d > POINT_TOL && closed {
            let edge = body.edge(e);
            if edge.v0 == edge.v1 {
                let (_, q) = edge.curve.closest_point(p);
                d = ((q.x - at[0]).powi(2) + (q.y - at[1]).powi(2) + (q.z - at[2]).powi(2)).sqrt();
            }
        }
        if d > POINT_TOL {
            continue;
        }
        if let Some(l) = len {
            let kl = body.edge_length(e);
            if (kl - l).abs() > LENGTH_TOL * l + 0.01 {
                continue;
            }
        }
        if best.is_none_or(|(bd, _)| d < bd) {
            best = Some((d, e));
        }
    }
    let (_, e) = best.ok_or("an edge of it is not an edge of the history regenerated so far")?;
    if !taken.insert(e.idx()) && strict {
        return Err("two edges it names are one edge of the history regenerated so far".into());
    }
    let faces: Vec<String> = body.edge_faces(e).into_iter().map(|f| body.face_id_str(f)).collect();
    if faces.len() != 2 {
        return Err("an edge it names does not lie between two faces".into());
    }
    let m = body.edge_mid(e);
    Ok(json!({"faces": faces, "at": [m.x, m.y, m.z]}))
}

/// A mapped fillet or chamfer, kept for a pattern that repeats it (`pattern.rs`): the edges are the file's own (mm, with the
/// length and whether the edge is closed).
#[derive(Clone, Debug)]
pub(crate) struct BlendRec {
    pub kind: &'static str,
    pub name: String,
    pub size: f64,
    pub propagate: bool,
    pub edges: Vec<([f64; 3], Option<f64>, bool)>,
}

/// The file's edges of a reference list: middle point and length (mm) and whether the edge is closed.
fn file_edges(refs: &[Reference]) -> Result<Vec<([f64; 3], Option<f64>, bool)>, String> {
    refs.iter()
        .map(|r| {
            let Reference::Edge(fp) = r else {
                return Err("it names something other than edges".to_string());
            };
            let mid = fp.mid_point.ok_or("an edge has no known position")?;
            let at = [mid[0] * CM_TO_MM, mid[1] * CM_TO_MM, mid[2] * CM_TO_MM];
            let len = fp.length.filter(|l| *l > 0.0).map(|l| l * CM_TO_MM);
            let closed = match (fp.start_point.flatten(), fp.end_point.flatten()) {
                (Some(a), Some(c)) => (0..3).all(|i| (a[i] - c[i]).abs() < 1e-6),
                _ => false,
            };
            Ok((at, len, closed))
        })
        .collect()
}

/// The `edges` picks (`{faces, at}`) for the edge references of a set, found in the regenerated history.
fn picks(b: &mut Builder, refs: &[Reference]) -> Result<(Vec<Value>, Vec<([f64; 3], Option<f64>, bool)>), String> {
    if b.lost {
        return Err("an earlier item did not map, so the edges of the body so far are not those the file names".into());
    }
    if !b.builds {
        return Err("no earlier feature made a body".into());
    }
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = guard::regenerate_body(&part).ok_or("the history so far builds no body to find the edges in")?;
    let file = file_edges(refs)?;
    let mut taken = std::collections::BTreeSet::new();
    let mut out = Vec::new();
    for (at, len, closed) in &file {
        out.push(pick_edge(&body, *at, *len, *closed, &mut taken, false)?);
    }
    Ok((out, file))
}

/// The picks of file edges in a history that has the pattern's copies of them: `file` edges turned `k` steps of `step` radians
/// about the axis (`o`, unit `dir`, mm) for `k` in `1..count`.
pub(crate) fn pick_rotated(
    b: &Builder,
    file: &[([f64; 3], Option<f64>, bool)],
    o: [f64; 3],
    dir: [f64; 3],
    count: u32,
) -> Result<Vec<Value>, String> {
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = guard::regenerate_body(&part).ok_or("the history so far builds no body to find the edges in")?;
    let step = std::f64::consts::TAU / f64::from(count);
    let mut taken = std::collections::BTreeSet::new();
    // the edges of the first copy are in the body too (a fillet on the seed): they are not picked again
    let mut out = Vec::new();
    for k in 1..count {
        let (s, c) = (step * f64::from(k)).sin_cos();
        for (at, len, closed) in file {
            let v = [at[0] - o[0], at[1] - o[1], at[2] - o[2]];
            let dv = dir[0] * v[0] + dir[1] * v[1] + dir[2] * v[2];
            let cr = [dir[1] * v[2] - dir[2] * v[1], dir[2] * v[0] - dir[0] * v[2], dir[0] * v[1] - dir[1] * v[0]];
            let q = [
                o[0] + v[0] * c + cr[0] * s + dir[0] * dv * (1.0 - c),
                o[1] + v[1] * c + cr[1] * s + dir[1] * dv * (1.0 - c),
                o[2] + v[2] * c + cr[2] * s + dir[2] * dv * (1.0 - c),
            ];
            out.push(pick_edge(&body, q, *len, *closed, &mut taken, true)?);
        }
    }
    Ok(out)
}

fn sets(sets: &Option<Vec<EdgeSet>>) -> Result<&[EdgeSet], String> {
    match sets.as_deref() {
        Some(s) if !s.is_empty() => Ok(s),
        _ => Err("it names no edges".into()),
    }
}

pub(crate) fn fillet(b: &mut Builder, name: &str, d: &FilletDetail) -> Result<Option<String>, String> {
    if d.full_round_fillet_face_sets.as_ref().is_some_and(|v| v.as_array().is_some_and(|a| !a.is_empty())) {
        return Err("a full-round fillet is not mapped yet".into());
    }
    let sets = sets(&d.edge_sets)?;
    for set in sets {
        if set.start_radius.is_some() || set.end_radius.is_some() || set.mid_radii.as_ref().is_some_and(|m| !m.is_empty()) {
            return Err("a variable-radius fillet is not mapped yet".into());
        }
        if set.offset_one.is_some() || set.offset_two.is_some() {
            return Err("an asymmetric fillet is not mapped yet".into());
        }
        let r = value(&set.radius).filter(|r| *r > 0.0).ok_or("its radius is not a positive number")?;
        let (edges, file) = picks(b, set.edges.as_deref().unwrap_or(&[]))?;
        if edges.is_empty() {
            return Err("it names no edges".into());
        }
        let propagate = set.is_tangent_chain == Some(true) || d.is_tangent_chain == Some(true);
        let id = b.next_id();
        b.features.push(json!({
            "type": "fillet", "id": id, "name": name, "radius": r * CM_TO_MM, "edges": edges, "propagate": propagate,
        }));
        b.blends.entry(b.cur).or_default().push(BlendRec { kind: "fillet", name: name.to_owned(), size: r * CM_TO_MM, propagate, edges: file });
    }
    Ok(None)
}

pub(crate) fn chamfer(b: &mut Builder, name: &str, d: &ChamferDetail) -> Result<Option<String>, String> {
    if d.is_flipped == Some(true) {
        return Err("a flipped chamfer is not mapped yet".into());
    }
    let sets = sets(&d.edge_sets)?;
    for set in sets {
        let dist = match (&set.distance, &set.distance_one, &set.distance_two) {
            (Some(_), None, None) => value(&set.distance),
            (None, Some(_), Some(_)) => {
                let (a, c) = (value(&set.distance_one), value(&set.distance_two));
                if a.zip(c).is_some_and(|(a, c)| (a - c).abs() < 1e-9) { a } else { None }
            }
            _ => None,
        };
        if set.angle.is_some() {
            return Err("a distance-and-angle chamfer is not mapped yet".into());
        }
        let dist = dist.filter(|r| *r > 0.0).ok_or("a chamfer with two different distances is not mapped yet")?;
        let (edges, file) = picks(b, set.edges.as_deref().unwrap_or(&[]))?;
        if edges.is_empty() {
            return Err("it names no edges".into());
        }
        let propagate = set.is_tangent_chain == Some(true) || d.is_tangent_chain == Some(true);
        let id = b.next_id();
        b.features.push(json!({
            "type": "chamfer", "id": id, "name": name, "chamferType": "equal", "distance": dist * CM_TO_MM,
            "edges": edges, "propagate": propagate,
        }));
        b.blends.entry(b.cur).or_default().push(BlendRec { kind: "chamfer", name: name.to_owned(), size: dist * CM_TO_MM, propagate, edges: file });
    }
    Ok(None)
}
