//! Shell features: the faces whose rim borders the shell's inner faces are the ones the shell removed.

use crate::{dim, Cx};
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, s};
use ck_recog::model::Kind;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::HashSet;

/// Read a shell back: thickness from the file's dimension (else the offset between an inner plane and its outer
/// parallel), removed faces from the rims.
pub fn shell(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let m = cx.m;
    let own: HashSet<FaceId> = faces.iter().copied().collect();
    // removed faces: planar faces outside the feature with an inner loop whose edges all border inner faces; a later
    // feature may have reshaped the rim (a fillet in an inner corner, a cut through the wall), so faces no earlier
    // feature made may border it too, as long as one edge borders an inner face
    let later = |h: FaceId| !cx.names.contains_key(&h) && cx.tag_of(h) != Some(f.id);
    let mut removed: Vec<FaceId> = vec![];
    for &g in &m.faces {
        if own.contains(&g) || !matches!(m.kind(g), Kind::Plane { .. }) {
            continue;
        }
        let face = m.b.face(g);
        if face.loops.len() < 2 {
            continue;
        }
        let rim = face.loops.iter().skip(1).any(|&l| {
            let cs = &m.b.lp(l).coedges;
            let across = |c: &ck_topo::body::CoedgeId| m.across(m.b.coedge(*c).edge, g);
            !cs.is_empty()
                && cs.iter().any(|c| across(c).map_or(false, |h| own.contains(&h)))
                && cs.iter().all(|c| across(c).map_or(false, |h| own.contains(&h) || later(h)))
        });
        if rim {
            removed.push(g);
        }
    }
    if removed.is_empty() {
        return Err("no removed face found (closed shell?)".into());
    }
    let thickness = match dim(cx, f, "D1") {
        Some(t) if t > 0.0 => t,
        _ => {
            // an inner plane against the nearest parallel outer plane
            let mut best: Option<f64> = None;
            for &g in faces {
                let Kind::Plane { n: ni, d: di } = m.kind(g) else { continue };
                for &h in &m.faces {
                    if own.contains(&h) {
                        continue;
                    }
                    let Kind::Plane { n: no, d: do_ } = m.kind(h) else { continue };
                    if ni.dot(*no) < -(1.0 - 1e-6) {
                        let t = (di + do_).abs();
                        if t > 1e-6 && best.map_or(true, |b| t < b) {
                            best = Some(t);
                        }
                    }
                }
            }
            best.ok_or("no thickness")?
        }
    };
    let names: Vec<Json> = removed.iter().map(|&g| cx.name_of(g).map(|x| s(&x))).collect::<Result<_, _>>()?;
    let id = cx.tree.ids.next();
    cx.tree.push(Obj::new().put("type", "shell").put("id", id.as_str()).put("faces", arr(names)).put("thickness", n(thickness)).build());
    for &g in faces {
        cx.names.insert(g, format!("{id}.S0"));
    }
    Ok(format!("thickness {thickness}; {} face(s) removed", removed.len()))
}
