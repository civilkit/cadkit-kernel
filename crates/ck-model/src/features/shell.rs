//! `shell` feature -> ck-ops shell. Errors: invalidValue (thickness <= 0),
//! missingReference (unknown face), nothingToCut (no body), shellTooThick /
//! shellUnsupported / shellFailed / offsetTooLarge / offsetUnsupported
//! (`params.detail`). A wall too thick for the shape also carries `maxSize` (thickness), see max_size.rs.

use super::{boolean_span, Brep, FeatureKind};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_json::Json;
use ck_topo::body::Body;

pub fn exec_shell(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Shell { faces, outward, .. } = &f.kind else { unreachable!() };
    let t = state.size(f, "thickness", 0.0)?;
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new(crate::regen::codes::NOTHING_TO_CUT)) };
    if !(t > 0.0) {
        return Err(FeatureError::new(crate::regen::codes::INVALID_VALUE));
    }
    let mut open = vec![];
    let mut single: Option<Body> = None;
    for r in faces {
        let id = body.find_face(r.as_str()).ok_or_else(|| crate::refs::missing_face([body], r.as_str()))?;
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
            ids.push(sub.find_face(n).ok_or_else(|| crate::refs::missing_face([&sub], n))?);
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
        // a wall too thick for the shape: the largest that builds, from the thickness the shell named (verified, then
        // pushed towards the refused one). Without a named thickness the shell's own search found none: say nothing.
        let named = e.limit.as_ref().map(|l| l.max_thickness).filter(|m| *m > 0.0);
        let max = match (e.code.as_str(), named) {
            ("shellTooThick" | "shellFailed" | "offsetTooLarge", Some(m)) => {
                let before = ck_check::validate(target, super::TOL).len();
                super::max_size::largest_that_builds(t, Some(m), |v| {
                    ck_ops::shell::shell_once(target, &open, v, *outward, super::TOL).is_ok_and(|b| ck_check::validate(&b, super::TOL).len() <= before)
                })
            }
            _ => None,
        };
        let mut fe = FeatureError::new(&e.code);
        let mut detail = None;
        if let Some(d) = e.detail {
            fe = fe.with("detail", Json::Str(d.clone()));
            detail = Some(d);
        }
        // a thickness the shell named that does not build a valid body is not offered (the solidcore #1528 plate: the
        // offset named 1.18 mm, which built a body with a vertex off its edge)
        let unverified = named.is_some() && max.is_none() && matches!(e.code.as_str(), "shellTooThick" | "shellFailed" | "offsetTooLarge");
        if let Some(l) = &e.limit {
            fe = super::offset_limit_params(fe, l);
            if unverified {
                fe.params.retain(|(k, _)| k != "maxThickness");
            }
        }
        if unverified {
            fe = fe.with(
                "hint",
                Json::Str("no thinner wall builds a valid body for this shape either, so the thickness in the detail does not help: shell before adding holes or cut-outs that cross curved walls, or change the shape".into()),
            );
        }
        if e.code == "offsetUnsupported" {
            if let Some(d) = detail {
                fe = super::offset_unsupported_hint(fe, &d);
            }
        }
        match max {
            Some(m) => super::max_size::with_max_size(fe, "thickness", m, "mm"),
            None => fe,
        }
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
