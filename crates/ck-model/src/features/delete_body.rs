//! `deleteBody {mode, bodies}`: remove (`delete`) or keep only (`keep`) the
//! named bodies. RECONSTRUCTION: body references are surface feature ids
//! (or face ids of a body); an empty list is `noBodies` (oracle).

use super::surfaces::Slot;
use super::Brep;
use crate::regen::{FeatureError, RegenState};
use crate::Feature;

pub fn exec_delete_body(br: &mut Brep, _state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let refs: Vec<String> = f
        .raw
        .get("bodies")
        .and_then(|a| a.as_arr())
        .map(|a| a.iter().filter_map(|x| x.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    if refs.is_empty() {
        return Err(FeatureError::new("noBodies"));
    }
    let keep = f.raw.get("mode").and_then(|m| m.as_str()) == Some("keep");
    let mut hit: Vec<usize> = vec![];
    let mut solid = false;
    for r in &refs {
        match br.slot_of_face(r) {
            Some(Slot::Solid) => solid = true,
            Some(Slot::Surf(i)) => hit.push(i),
            None => match br.surf_index(r) {
                Some(i) => hit.push(i),
                None => return Err(FeatureError::new(crate::regen::codes::MISSING_REFERENCE)),
            },
        }
    }
    if keep {
        let all: Vec<usize> = (0..br.surfs.len()).filter(|i| !hit.contains(i)).collect();
        br.drop_surfaces(all);
        if !solid {
            br.body = None;
        }
    } else {
        br.drop_surfaces(hit);
        if solid {
            br.body = None;
        }
    }
    Ok(())
}
