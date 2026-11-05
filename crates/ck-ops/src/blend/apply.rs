//! Applying a blend tool to the body by boolean, with the blend face named and the contacts hinted.

use super::BlendError;
use crate::boolean::assemble::build;
use crate::boolean::Op;
use ck_topo::body::{Body, FaceName, Role};
use super::diag::*;

/// Boolean `b op tool`, faces ordered untouched / `hint` (touched, in that
/// order) / the tool's new faces; returns the body and the hint.
pub(super) fn apply_tool(b: &Body, tool: &Body, op: Op, name: &FaceName, hint: Vec<String>, tol: f64) -> Result<(Body, Vec<String>), BlendError> {
    apply_tool_hooked(b, tool, op, name, hint, tol, None)
}

/// [`apply_tool`] with a per-face-pair SSI override (see
/// [`crate::boolean::imprint::PairHook`]).
///
/// Marched intersection curves meet their neighbours only to ~1e-6, so a
/// corner whose cut curve ends 1.5e-6 short of an existing vertex (the
/// vertical fillet at one of four symmetric chamfered corners) leaves the
/// arrangement open at `tol`. Like `boolean::boolean`, a failure is retried
/// with the welding tolerance raised 10x and 100x; a result that closes but
/// falls outside the volume bounds the operands allow (`boolean::sanity`)
/// counts as a failure too.
pub(super) fn apply_tool_hooked(
    b: &Body,
    tool: &Body,
    op: Op,
    name: &FaceName,
    hint: Vec<String>,
    tol: f64,
    hook: Option<crate::boolean::imprint::PairHook>,
) -> Result<(Body, Vec<String>), BlendError> {
    let mut first = None;
    // a tangent-contact (hooked) tool also gets a last try with a tight weld: a body with sliver faces thinner than the
    // loose weld (bands stacked a few thousandths apart) collapses them and does not assemble
    let ks: &[f64] = if hook.is_some() { &[1.0, 10.0, 100.0, 0.1] } else { &[1.0, 10.0, 100.0] };
    for &k in ks {
        match apply_tool_once(b, tool, op, name, hint.clone(), tol * k, hook) {
            Ok(r) => match crate::boolean::sanity::check_result(b, tool, op, &r.0) {
                None => return Ok(r),
                Some(m) => {
                    let msg = format!("sanity at tol {:e}: {} {} outside [{}, {}]", tol * k, m.what, m.got, m.expected_min, m.expected_max);
                    dbg_fail(&msg);
                    first.get_or_insert(BlendError::detail("blendFailed", &msg));
                }
            },
            Err(e) => {
                first.get_or_insert(e);
            }
        }
    }
    Err(first.unwrap_or_else(|| BlendError::code("blendFailed")))
}

pub(super) fn apply_tool_once(
    b: &Body,
    tool: &Body,
    op: Op,
    name: &FaceName,
    hint: Vec<String>,
    tol: f64,
    hook: Option<crate::boolean::imprint::PairHook>,
) -> Result<(Body, Vec<String>), BlendError> {
    let hook_given = hook.is_some();
    let dbg_t = super::diag::on().then(std::time::Instant::now);
    let arr = crate::boolean::imprint::imprint_hooked(b, tool, tol, hook);
    if let Some(t) = dbg_t {
        eprintln!("apply_tool: imprint {:.1}s", t.elapsed().as_secs_f64());
    }
    let dbg_t = super::diag::on().then(std::time::Instant::now);
    // a hooked (tangent contact) tool is imprinted at a loose welding tolerance; classify against it tightly
    let class_tol = if hook_given { tol.min(1e-5) } else { tol };
    let mut faces_out = crate::boolean::assemble::select_class_tol(&arr, [b, tool], op, class_tol).map_err(|e| {
        let msg = format!("select: {e:?}");
        dbg_fail(&msg);
        BlendError::detail("blendFailed", &msg)
    })?;
    if let Some(t) = dbg_t {
        eprintln!("apply_tool: select {:.1}s", t.elapsed().as_secs_f64());
    }
    for f in &mut faces_out {
        if f.group == 1 {
            let mut base = f.name.clone();
            base.region = None;
            let pos = hint.iter().position(|h| FaceName::parse(h).map(|mut x| {
                x.region = None;
                x
            }) == Some(base.clone()));
            f.key = (1, pos.unwrap_or(hint.len()), f.key.1 * 1000 + f.key.2);
        } else if f.group == 4 {
            f.key = (1, hint.len() + 1, f.key.1);
        }
        // walk items follow the face key (edge numbering = face-walk order)
        let key = f.key;
        for w in &mut f.walk {
            w.0 = key;
        }
    }
    faces_out.sort_by_key(|f| f.key);
    // the blend face may have merged into the coincident blend face of the
    // ring's previous arc (a split ring chamfered edge by edge: one cone)
    let merged_into_earlier = faces_out.iter().any(|f| (f.group == 0 || f.group == 4) && f.name.feature == name.feature && matches!(f.name.role, Role::Blend(_)));
    if !faces_out.iter().any(|f| { let mut n = f.name.clone(); n.region = None; n == *name }) && !merged_into_earlier {
        dbg_fail("no blend face in result");
        return Err(BlendError::detail("blendFailed", "no blend face in result"));
    }
    let mut body = build(&arr, &faces_out);
    super::freeform::drop_zero_edges(&mut body, 1e-4);
    let bad = ck_topo::check::has_open_or_broken(&body, tol);
    if bad {
        let detail = format!("check: {:?}", ck_topo::check::check(&body, tol));
        dbg_fail(&detail);
        if super::diag::on() {
            eprintln!("  in : {:?}", b.face_ids().map(|f| b.face_id_str(f)).collect::<Vec<_>>());
            eprintln!("  out: {:?}", body.face_ids().map(|f| body.face_id_str(f)).collect::<Vec<_>>());
            for i in ck_topo::check::check(&body, tol) {
                if let ck_topo::check::Issue::OpenEdge(e) = i {
                    let ed = body.edge(e);
                    eprintln!(
                        "  open {:?} {} t=({:.6},{:.6}) a={:?} b={:?} faces={:?}",
                        e, ed.curve.kind(), ed.t0, ed.t1, body.vertex(ed.v0).p, body.vertex(ed.v1).p,
                        body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect::<Vec<_>>()
                    );
                }
            }
        }
        return Err(BlendError::detail("blendFailed", &detail));
    }
    Ok((body, hint))
}
