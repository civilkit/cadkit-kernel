//! `shell` feature -> ck-ops shell. Errors: invalidValue (thickness <= 0),
//! missingReference (unknown face), nothingToCut (no body), shellTooThick /
//! shellUnsupported / shellFailed / offsetTooLarge / offsetUnsupported
//! (`params.detail`).

use super::{boolean_span, Brep, FeatureKind};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;

pub fn exec_shell(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Shell { faces, outward, .. } = &f.kind else { unreachable!() };
    let t = state.num(f, "thickness", 0.0)?;
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new(crate::regen::codes::NOTHING_TO_CUT)) };
    if !(t > 0.0) {
        return Err(FeatureError::new(crate::regen::codes::INVALID_VALUE));
    }
    let mut open = vec![];
    for r in faces {
        let id = body.find_face(r.as_str()).ok_or_else(|| FeatureError::new(crate::regen::codes::MISSING_REFERENCE))?;
        open.push(id);
    }
    let out = ck_ops::shell::shell(body, &open, t, *outward, super::TOL).map_err(|e| {
        let mut fe = FeatureError::new(&e.code);
        if let Some(d) = e.detail {
            fe = fe.with("detail", Json::Str(d));
        }
        if let Some(l) = &e.limit {
            fe = super::offset_limit_params(fe, l);
        }
        fe
    })?;
    br.spans.push(boolean_span(ck_ops::boolean::Op::Cut, body.num_faces(), body.num_faces(), out.num_faces()));
    br.body = Some(out);
    Ok(())
}
