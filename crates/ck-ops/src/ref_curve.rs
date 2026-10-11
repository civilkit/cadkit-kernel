//! The `refCurve` request: a model edge (or sketch curve) as an exact NURBS
//! curve, for sketch bridges / references.
//!
//! `{ref:{kind:"edge",faces:[a,b],at}|{kind:"sketchCurve",sketch,entity}}` ->
//! `{curve:{P,p,knots[,w]}, start, end, closed}` or `{"error":"missingReference"}`.
//! The edge branch lives here; the sketch branch (needs the regen's solved
//! sketches) in ck-wasm. `closed` = the edge's end vertices coincide.

use crate::measure::{resolve_ref, Ref};
use ck_core::linalg::Vec3;
use ck_geom::convert::curve_to_nurbs;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

fn arr(v: Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

pub fn missing_reference() -> Json {
    Obj::new().put("error", "missingReference").build()
}

/// `{P,p,knots[,w]}` wire form of a 3D NURBS curve.
pub fn nurbs_json(c: &BSplineCurve) -> Json {
    let mut o = Obj::new()
        .put("P", Json::Arr(c.ctrl.iter().map(|&p| arr(p)).collect()))
        .put("p", c.degree as f64)
        .put("knots", Json::Arr(c.knots.iter().map(|&k| Json::Num(k)).collect()));
    if let Some(w) = &c.weights {
        o = o.put("w", Json::Arr(w.iter().map(|&k| Json::Num(k)).collect()));
    }
    o.build()
}

/// The `{curve,start,end,closed}` reply for a NURBS curve.
pub fn curve_reply(c: &BSplineCurve, closed: bool) -> Json {
    let (s, e) = (c.ctrl[0], *c.ctrl.last().unwrap());
    Obj::new().put("curve", nurbs_json(c)).put("start", arr(s)).put("end", arr(e)).put("closed", closed).build()
}

/// Edge branch of `refCurve`.
pub fn edge_reply(body: &Body, r: &Json) -> Json {
    let Some(Ref::Edge(e)) = resolve_ref(body, r).filter(|r| matches!(r, Ref::Edge(_))) else { return missing_reference() };
    let ed = body.edge(e);
    let c = curve_to_nurbs(&ed.curve, ed.t0, ed.t1);
    curve_reply(&c, ed.v0 == ed.v1)
}
