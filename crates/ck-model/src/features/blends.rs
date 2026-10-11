//! Fillet (and later chamfer) features -> ck-ops blends.
//!
//! Pinned by oracle probes (bracket F11/F12):
//! * trace child span `"F11: Fillet(5.0, Circular) of 4 edges"` (`1 edge`
//!   singular; radius printed with Rust `{:?}` float formatting);
//! * edge selections `{faces:[a,b], at:[x,y,z]}` resolve to the edge between
//!   the two named faces nearest `at` (ck-ops `blend::build::find_edge`);
//! * errors `invalidValue` (radius <= 0), `missingEdge`, `blendOverflow`
//!   (`params.detail`), body unchanged.

use super::{Brep, ChamferType, FeatureKind, Span};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_geom::Vec3;
use ck_json::Json;

/// `Fillet(<r>, Circular) of <n> edge(s)`.
pub fn fillet_label(id: &str, r: f64, n: usize) -> String {
    format!("{id}: Fillet({r:?}, Circular) of {n} edge{}", if n == 1 { "" } else { "s" })
}

pub fn blend_error(e: ck_ops::blend::BlendError) -> FeatureError {
    let mut fe = FeatureError::new(&e.code);
    if let Some(d) = e.detail {
        fe = fe.with("detail", Json::Str(d));
    }
    fe
}

/// `Chamfer(<d>, <d>) of <n> edge(s)` (equal-distance chamfer).
pub fn chamfer_label(id: &str, d: f64, n: usize) -> String {
    format!("{id}: Chamfer({d:?}, {d:?}) of {n} edge{}", if n == 1 { "" } else { "s" })
}

/// `chamfer`. Oracle (box probes): `equal` (also the default) is the only
/// supported edge chamfer; `distanceDistance` / `angleDistance` / unknown
/// types fail `unsupported {what:"chamfer <type>"}` before anything else.
pub fn exec_chamfer(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Chamfer { chamfer_type, edges, .. } = &f.kind else { unreachable!() };
    let ty = f.raw.get("chamferType").and_then(|v| v.as_str()).unwrap_or("equal").to_string();
    match chamfer_type {
        ChamferType::Equal => {}
        ChamferType::Vertex => return exec_vertex_chamfer(br, state, f, edges),
        _ => return Err(FeatureError::new("unsupported").with("what", Json::Str(format!("chamfer {ty}")))),
    }
    let d = state.num(f, "distance", 0.0)?;
    if !(d > 0.0) || !d.is_finite() {
        return Err(FeatureError::new("invalidValue"));
    }
    let sel = select_edges(br, edges)?;
    br.spans.push(Span::leaf(chamfer_label(&f.id, d, sel.len())));
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    let out = ck_ops::blend::build::chamfer(body, &sel, d, &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

fn select_edges(br: &Brep, edges: &[crate::EdgeRef]) -> Result<Vec<(String, String, Vec3)>, FeatureError> {
    // GUESS: no body -> missingEdge (nothing can be selected).
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("missingEdge")) };
    let mut sel = vec![];
    for e in edges {
        let (Some(a), Some(b)) = (e.faces.first(), e.faces.get(1)) else { return Err(FeatureError::new("missingEdge")) };
        let at = match e.at {
            Some(p) => Vec3::new(p[0], p[1], p[2]),
            // GUESS: without a pick point the first edge between the faces
            None => first_edge_mid(body, a, b).ok_or_else(|| FeatureError::new("missingEdge"))?,
        };
        if ck_ops::blend::build::find_edge(body, a, b, at).is_none() {
            return Err(FeatureError::new("missingEdge"));
        }
        sel.push(forward_first(body, a, b, at));
    }
    if sel.is_empty() {
        return Err(FeatureError::new("missingEdge"));
    }
    Ok(sel)
}

/// `fillet`. Oracle (box probes): any `filletType` string (even
/// `"circular"`) fails `unsupported {what:"fillet <type>"}` first; then
/// `radius <= 0` -> `invalidValue`; then unresolved / no edges ->
/// `missingEdge`; none of these open the `Fillet(..)` span.
pub fn exec_fillet(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Fillet { edges, .. } = &f.kind else { unreachable!() };
    if let Some(t) = f.raw.get("filletType").and_then(|v| v.as_str()) {
        return Err(FeatureError::new("unsupported").with("what", Json::Str(format!("fillet {t}"))));
    }
    let r = state.num(f, "radius", 0.0)?;
    if !(r > 0.0) || !r.is_finite() {
        return Err(FeatureError::new("invalidValue"));
    }
    let sel = select_edges(br, edges)?;
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    br.spans.push(Span::leaf(fillet_label(&f.id, r, sel.len())));
    let out = ck_ops::blend::build::fillet(body, &sel, r, &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

/// Oracle (box probes, bracket F12): the blend's touched faces list the
/// edge's two faces with the one whose loop runs the edge FORWARD first,
/// whatever the selection order (`["F2.S.e8","F2.S.e9"]` -> e9, e8).
/// ck-ops keeps the selection order, so normalise it here.
fn forward_first(b: &ck_topo::body::Body, fa: &str, fb: &str, at: Vec3) -> (String, String, Vec3) {
    let keep = (fa.to_string(), fb.to_string(), at);
    let Some(e) = ck_ops::blend::build::find_edge(b, fa, fb, at) else { return keep };
    let fwd = b.edge_coedges(e).into_iter().find(|&c| !b.coedge(c).rev).map(|c| b.coedge_face(c));
    match (fwd, b.find_face(fb)) {
        (Some(f), Some(g)) if f == g => (fb.to_string(), fa.to_string(), at),
        _ => keep,
    }
}

fn first_edge_mid(b: &ck_topo::body::Body, fa: &str, fb: &str) -> Option<Vec3> {
    let (f, g) = (b.find_face(fa)?, b.find_face(fb)?);
    b.edge_ids()
        .find(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && fs.contains(&f) && fs.contains(&g)
        })
        .map(|e| b.edge_mid(e))
}

/// `chamferType: "vertex"`: the corner nearest `at` (feature `at`, else the
/// first edge's pick point) is cut by the plane through the points at
/// `distance` / `distance2` / `distance3` (defaults: `distance`) along the
/// vertex's edges in arena order. No `Chamfer(..)` span (oracle trace).
fn exec_vertex_chamfer(br: &mut Brep, state: &mut RegenState, f: &Feature, edges: &[crate::EdgeRef]) -> Result<(), FeatureError> {
    let d1 = state.num(f, "distance", 0.0)?;
    let d2 = if f.raw.get("distance2").is_some() { state.num(f, "distance2", d1)? } else { d1 };
    let d3 = if f.raw.get("distance3").is_some() { state.num(f, "distance3", d1)? } else { d1 };
    let at = f
        .raw
        .get("at")
        .and_then(|a| a.as_arr())
        .and_then(|a| (a.len() == 3).then(|| Vec3::new(a[0].as_f64().unwrap_or(0.0), a[1].as_f64().unwrap_or(0.0), a[2].as_f64().unwrap_or(0.0))))
        .or_else(|| edges.first().and_then(|e| e.at).map(|p| Vec3::new(p[0], p[1], p[2])));
    let Some(at) = at else { return Err(FeatureError::new("missingReference")) };
    if [d1, d2, d3].iter().any(|d| !(*d > 0.0) || !d.is_finite()) {
        return Err(FeatureError::new("invalidValue"));
    }
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("missingReference")) };
    let out = ck_ops::blend::vertex::chamfer_vertex(body, at, [d1, d2, d3], &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}
