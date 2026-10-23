//! `shell` feature -> ck-ops shell. Errors: invalidValue (thickness <= 0),
//! missingReference (unknown face), nothingToCut (no body), shellTooThick /
//! shellUnsupported / shellFailed / offsetTooLarge / offsetUnsupported
//! (`params.detail`).

use super::{boolean_span, Brep, FeatureKind};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::Body;

pub fn exec_shell(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Shell { faces, outward, .. } = &f.kind else { unreachable!() };
    let t = state.num(f, "thickness", 0.0)?;
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new(crate::regen::codes::NOTHING_TO_CUT)) };
    if !(t > 0.0) {
        return Err(FeatureError::new(crate::regen::codes::INVALID_VALUE));
    }
    let mut open = vec![];
    let mut single: Option<Body> = None;
    for r in faces {
        let id = body.find_face(r.as_str()).ok_or_else(|| FeatureError::new(crate::regen::codes::MISSING_REFERENCE))?;
        open.push(id);
    }
    // with several lumps (bodies), only the lump that holds the open faces is shelled: another lump, a coincident
    // copy for one, would otherwise take part in the cavity boolean
    let mut lump_of_open: Vec<_> = open.iter().map(|&g| body.face_lump(g)).collect();
    lump_of_open.sort_unstable();
    lump_of_open.dedup();
    let others: Option<Body> = if body.lump_ids().count() > 1 && lump_of_open.len() == 1 {
        let mine = ck_topo::mass::lump_faces(body, lump_of_open[0]);
        let rest: Vec<_> = body.face_ids().filter(|g| !mine.contains(g)).collect();
        let mut sub = ck_ops::boolean::assemble::rebuild_in_order(body, &mine);
        ck_topo::region::rebuild_lumps(&mut sub);
        let names: Vec<String> = open.iter().map(|&g| body.face(g).name.id()).collect();
        let mut ids = vec![];
        for n in &names {
            ids.push(sub.find_face(n).ok_or_else(|| FeatureError::new(crate::regen::codes::MISSING_REFERENCE))?);
        }
        open = ids;
        let mut others = ck_ops::boolean::assemble::rebuild_in_order(body, &rest);
        ck_topo::region::rebuild_lumps(&mut others);
        single = Some(sub);
        Some(others)
    } else {
        None
    };
    let target = single.as_ref().unwrap_or(body);
    let out = ck_ops::shell::shell(target, &open, t, *outward, super::TOL).map_err(|e| {
        let mut fe = FeatureError::new(&e.code);
        let mut detail = None;
        if let Some(d) = e.detail {
            fe = fe.with("detail", Json::Str(d.clone()));
            detail = Some(d);
        }
        if let Some(l) = &e.limit {
            fe = super::offset_limit_params(fe, l);
        }
        if e.code == "offsetUnsupported" {
            if let Some(d) = detail {
                fe = super::offset_unsupported_hint(fe, &d);
            }
        }
        fe
    })?;
    let (before, after) = (body.num_faces(), out.num_faces());
    br.spans.push(boolean_span(ck_ops::boolean::Op::Cut, before, before, after));
    match others {
        Some(rest) => {
            br.body = Some(rest);
            br.append_body(&out);
        }
        None => br.body = Some(out),
    }
    Ok(())
}
