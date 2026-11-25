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

/// The `edges` picks (`{faces, at}`) for the edge references of a set, found in the regenerated history.
fn picks(b: &mut Builder, refs: &[Reference]) -> Result<Vec<Value>, String> {
    if b.lost {
        return Err("an earlier item did not map, so the edges of the body so far are not those the file names".into());
    }
    if !b.builds {
        return Err("no earlier feature made a body".into());
    }
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = guard::regenerate_body(&part).ok_or("the history so far builds no body to find the edges in")?;
    let mut out = Vec::new();
    for r in refs {
        let Reference::Edge(fp) = r else {
            return Err("it names something other than edges".into());
        };
        let mid = fp.mid_point.ok_or("an edge has no known position")?;
        let at = [mid[0] * CM_TO_MM, mid[1] * CM_TO_MM, mid[2] * CM_TO_MM];
        let len = fp.length.filter(|l| *l > 0.0).map(|l| l * CM_TO_MM);
        let mut best: Option<(f64, ck_topo::body::EdgeId)> = None;
        for e in body.edge_ids() {
            let m = body.edge_mid(e);
            let d = ((m.x - at[0]).powi(2) + (m.y - at[1]).powi(2) + (m.z - at[2]).powi(2)).sqrt();
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
        let faces: Vec<String> = body.edge_faces(e).into_iter().map(|f| body.face_id_str(f)).collect();
        if faces.len() != 2 {
            return Err("an edge it names does not lie between two faces".into());
        }
        let m = body.edge_mid(e);
        out.push(json!({"faces": faces, "at": [m.x, m.y, m.z]}));
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
        let edges = picks(b, set.edges.as_deref().unwrap_or(&[]))?;
        if edges.is_empty() {
            return Err("it names no edges".into());
        }
        let id = b.next_id();
        b.features.push(json!({
            "type": "fillet", "id": id, "name": name, "radius": r * CM_TO_MM, "edges": edges,
            "propagate": set.is_tangent_chain == Some(true) || d.is_tangent_chain == Some(true),
        }));
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
        let edges = picks(b, set.edges.as_deref().unwrap_or(&[]))?;
        if edges.is_empty() {
            return Err("it names no edges".into());
        }
        let id = b.next_id();
        b.features.push(json!({
            "type": "chamfer", "id": id, "name": name, "chamferType": "equal", "distance": dist * CM_TO_MM,
            "edges": edges, "propagate": set.is_tangent_chain == Some(true) || d.is_tangent_chain == Some(true),
        }));
    }
    Ok(None)
}
