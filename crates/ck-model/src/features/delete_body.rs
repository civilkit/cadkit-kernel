//! `deleteBody {mode, bodies}`: remove (`delete`) or keep only (`keep`) the
//! named bodies. RECONSTRUCTION: `bodies[]` entries are `{face: "<face id>"}`
//! (the body = the lump holding that face) or `{surface: "<feature id>"}`
//! (client `bodyRefOf`); an empty list is `noBodies`, an unresolved entry
//! `missingReference` (oracle probes `deleteBodyNone` / `deleteBodySurf`).

use super::surfaces::Slot;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_topo::body::FaceId;

pub fn exec_delete_body(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let refs = f.raw.get("bodies").and_then(|a| a.as_arr()).unwrap_or(&[]);
    if refs.is_empty() {
        return Err(FeatureError::new("noBodies"));
    }
    let keep = f.raw.get("mode").and_then(|m| m.as_str()) == Some("keep");
    let mut surf_hit: Vec<usize> = vec![];
    // solid lumps (as the face sets of their connected components) to drop
    let mut solid_faces: Vec<FaceId> = vec![];
    for r in refs {
        let missing = || FeatureError::new(codes::MISSING_REFERENCE);
        if let Some(face) = r.get_str("face") {
            match br.slot_of_face(face).ok_or_else(missing)? {
                Slot::Surf(i) => surf_hit.push(i),
                Slot::Solid => {
                    let b = br.body.as_ref().ok_or_else(missing)?;
                    let fid = b.find_face(face).ok_or_else(missing)?;
                    let comp = ck_topo::region::components(b).into_iter().find(|c| c.contains(&fid)).ok_or_else(missing)?;
                    solid_faces.extend(comp);
                }
            }
        } else if let Some(s) = r.get_str("surface") {
            surf_hit.push(br.surface_ref(s).ok_or_else(missing)?);
        } else {
            return Err(missing());
        }
    }
    // solid body: drop (delete) or keep only (keep) the hit components
    if let Some(b) = br.body.clone() {
        let all: Vec<FaceId> = b.face_ids().collect();
        let left: Vec<FaceId> = all.iter().copied().filter(|x| solid_faces.contains(x) == keep).collect();
        br.body = if left.is_empty() {
            None
        } else if left.len() == all.len() {
            Some(b)
        } else {
            Some(ck_ops::boolean::assemble::rebuild_in_order(&b, &left))
        };
    }
    if keep {
        let rest: Vec<usize> = (0..br.surfs.len()).filter(|i| !surf_hit.contains(i)).collect();
        br.drop_surfaces(rest);
    } else {
        br.drop_surfaces(surf_hit);
    }
    Ok(())
}
