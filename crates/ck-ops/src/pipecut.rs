//! Pipe cut (ck-ops/src/pipecut.rs): roll a ball of radius `r` along edges
//! and cut away everything of the body's faces that the ball touches. The
//! result is a SURFACE body (an open sheet): the faces keep their names, the
//! pipe surface itself is not part of it ("close the gap with Fill").
//! Codes: `pipeCutRadius {max}`, `pipeCutFailed`, `pipeCutNothingLeft`.
//!
//! Oracle probes (tools/stub_cases.mjs `pipeCut*`, docs/ck-ops-stubs.md): a
//! face loses exactly the part within distance `r` of the selected edges
//! (capsule union: cylinder / torus per edge, ball at every end); faces that
//! the ball covers completely disappear; `r` larger than an edge circle's
//! radius is `pipeCutRadius` (the pipe would fold over itself), `max` being
//! 0.999 of that radius; all faces removed is `pipeCutNothingLeft`.
//!
//! Construction (not the original's, which is a rolling-ball march): the
//! face-pair imprint of the boolean (`boolean::imprint`) splits the body's
//! faces along their intersection curves with one cutting solid per edge
//! (cylinder or torus sector with flat caps) and per distinct end vertex (a
//! ball). Those solids are only *cutters*: they need not form a valid union
//! (their pieces overlap and touch tangentially, which a boolean union would
//! choke on), because the decision is not a solid classification. After the
//! last split each fragment is kept when the distance from an interior
//! sample point to the nearest selected edge is at least `r` (the exact
//! definition of the capsule union), which also makes the operation work on
//! open sheets.

use crate::boolean::assemble::{build, select_with};
use crate::boolean::classify::Class;
use crate::boolean::imprint::{dist_in_range, imprint};
use crate::surfacing::sheet::sheet;
use ck_geom::curve::Curve;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId};
use ck_topo::make::revolve::revolve;
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

/// Feature error of a pipe cut: `{code, params}`.
#[derive(Debug, Clone, PartialEq)]
pub struct PipeCutError {
    pub code: &'static str,
    /// `pipeCutRadius`: the largest radius that does not fold the pipe.
    pub max: Option<f64>,
    pub detail: Option<String>,
}

impl PipeCutError {
    fn code(code: &'static str) -> PipeCutError {
        PipeCutError { code, max: None, detail: None }
    }
    fn detail(code: &'static str, d: impl Into<String>) -> PipeCutError {
        PipeCutError { code, max: None, detail: Some(d.into()) }
    }
}

/// One selected edge as a curve segment.
struct PipeEdge {
    curve: Curve,
    t0: f64,
    t1: f64,
}

fn any_perp(u: Vec3) -> Vec3 {
    let a = if u.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    u.cross(a).normalize()
}

fn profile_of(frame: Frame, segs: Vec<ProfileSeg>) -> Profile {
    Profile::new(frame, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized()
}

/// Cylinder solid (flat caps) of radius `r` along a line segment.
fn cylinder_tool(a: Vec3, b: Vec3, r: f64) -> Result<Body, String> {
    let len = a.dist(b);
    let u = (b - a) / len;
    let w = any_perp(u);
    let frame = Frame { o: a, x: u, y: w, z: u.cross(w) };
    let p = |x: f64, y: f64| Vec2::new(x, y);
    let prof = profile_of(
        frame,
        vec![
            ProfileSeg::line("ax", p(0.0, 0.0), p(len, 0.0)),
            ProfileSeg::line("end", p(len, 0.0), p(len, r)),
            ProfileSeg::line("cyl", p(len, r), p(0.0, r)),
            ProfileSeg::line("start", p(0.0, r), p(0.0, 0.0)),
        ],
    );
    revolve("~pipe", &prof, a, u, 0.0, TAU)
}

/// Torus sector (flat caps) of tube radius `r` around the circle (`c`,
/// `axis`, `rad`) from angle `t0` to `t1` (counter-clockwise about `axis`
/// from the circle's own x direction `x`).
fn torus_tool(c: Vec3, axis: Vec3, x: Vec3, rad: f64, t0: f64, t1: f64, r: f64) -> Result<Body, String> {
    let radial = (x * t0.cos() + axis.cross(x) * t0.sin()).normalize();
    let frame = Frame { o: c, x: radial, y: axis, z: radial.cross(axis) };
    let prof = profile_of(frame, vec![ProfileSeg::circle("tube", Vec2::new(rad, 0.0), r)]);
    revolve("~pipe", &prof, c, axis, 0.0, (t1 - t0).min(TAU))
}

/// Ball of radius `r` at `c`.
fn ball_tool(c: Vec3, r: f64) -> Result<Body, String> {
    // an axis that is neither a coordinate axis nor along a typical edge, so
    // the poles / seam do not coincide with face boundaries of the part
    let u = Vec3::new(0.3713906763541037, 0.5570860145311556, 0.7427813527082074);
    let w = any_perp(u);
    let frame = Frame { o: c, x: u, y: w, z: u.cross(w) };
    let p = |x: f64, y: f64| Vec2::new(x, y);
    let prof = profile_of(
        frame,
        vec![ProfileSeg::line("ax", p(-r, 0.0), p(r, 0.0)), ProfileSeg::arc("ball", p(0.0, 0.0), p(r, 0.0), p(-r, 0.0))],
    );
    revolve("~pipe", &prof, c, u, 0.0, TAU)
}

/// Distance from `q` to the nearest selected edge.
fn dist_to_edges(edges: &[PipeEdge], q: Vec3) -> f64 {
    edges.iter().map(|e| dist_in_range(&e.curve, (e.t0.min(e.t1), e.t0.max(e.t1)), q).1).fold(f64::INFINITY, f64::min)
}

/// Cut `r`-balls rolled along `edges` out of the faces of `body` (solid or
/// sheet); returns the remaining faces as a sheet body.
pub fn pipe_cut(body: &Body, edges: &[EdgeId], r: f64, tol: f64) -> Result<Body, PipeCutError> {
    if !(r > 0.0) || !r.is_finite() {
        return Err(PipeCutError::code("invalidValue"));
    }
    // ---- the path: edge curves, end vertices
    let mut path: Vec<PipeEdge> = vec![];
    let mut tools: Vec<Result<Body, String>> = vec![];
    let mut ends: Vec<Vec3> = vec![];
    let mut seen: Vec<EdgeId> = vec![];
    for &e in edges {
        if seen.contains(&e) {
            continue;
        }
        seen.push(e);
        let ed = body.edge(e);
        let (lo, hi) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
        let (pa, pb) = (ed.curve.eval(lo), ed.curve.eval(hi));
        match &ed.curve {
            Curve::Line { .. } => {
                if pa.dist(pb) <= tol {
                    return Err(PipeCutError::detail("pipeCutFailed", "an edge has no length"));
                }
                tools.push(cylinder_tool(pa, pb, r));
            }
            Curve::Circle { frame, r: rad } => {
                if r >= *rad {
                    // the tube would pass its own axis: the pipe folds over itself
                    return Err(PipeCutError {
                        code: "pipeCutRadius",
                        max: Some(rad * 0.999),
                        detail: Some(format!("radius {r} exceeds an edge's radius of curvature {rad:.4}")),
                    });
                }
                tools.push(torus_tool(frame.o, frame.z, frame.x, *rad, lo, hi, r));
            }
            other => {
                return Err(PipeCutError::detail("pipeCutFailed", format!("{} edges are not supported", other.kind())));
            }
        }
        let closed = pa.dist(pb) <= tol && matches!(ed.curve, Curve::Circle { .. }) && (hi - lo) >= TAU - 1e-9;
        if !closed {
            for q in [pa, pb] {
                if !ends.iter().any(|x| x.dist(q) <= tol * 10.0) {
                    ends.push(q);
                }
            }
        }
        path.push(PipeEdge { curve: ed.curve.clone(), t0: lo, t1: hi });
    }
    if path.is_empty() {
        return Err(PipeCutError::code("missingReference"));
    }
    for q in ends {
        tools.push(ball_tool(q, r));
    }
    // ---- split the faces along every cutter
    let mut cur = body.clone();
    for tool in tools {
        let tool = tool.map_err(|e| PipeCutError::detail("pipeCutFailed", format!("cutter: {e}")))?;
        let arr = imprint(&cur, &tool, tol);
        if arr.interacting_pairs == 0 {
            continue;
        }
        let faces = select_with(&arr, false, Some(1), |_side, _fi, _q, _n| (Class::Out, Some(false)))
            .map_err(|e| PipeCutError::detail("pipeCutFailed", e))?;
        cur = build(&arr, &faces);
    }
    // ---- keep the fragments farther than `r` from the path
    let ids: Vec<FaceId> = cur.face_ids().collect();
    let mut keep: Vec<FaceId> = vec![];
    for f in ids {
        let Some(q) = sample_in_face(&cur, f, tol) else {
            return Err(PipeCutError::detail("pipeCutFailed", format!("no sample point on {}", cur.face_id_str(f))));
        };
        let mut inside = 0;
        for p in &q {
            if dist_to_edges(&path, *p) < r {
                inside += 1;
            }
        }
        if inside == 0 {
            keep.push(f);
        } else if inside != q.len() {
            return Err(PipeCutError::detail("pipeCutFailed", format!("{} is cut only partly by the pipe", cur.face_id_str(f))));
        }
    }
    if keep.is_empty() {
        return Err(PipeCutError::detail("pipeCutNothingLeft", "the pipe removes every face"));
    }
    let mut out = sheet(&cur, &keep);
    rename_fragments(&mut out, body);
    Ok(out)
}

/// A few interior points of a face (for the in / out decision).
fn sample_in_face(b: &Body, f: FaceId, tol: f64) -> Option<Vec<Vec3>> {
    use crate::boolean::classify::{face_loops_uv, in_loops};
    let s = &b.face(f).surface;
    let loops = face_loops_uv(b, f);
    let (mut lo, mut hi) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for l in &loops {
        for p in l {
            lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
    }
    if !(lo.x.is_finite() && hi.x.is_finite()) {
        return None;
    }
    let mut pts = vec![];
    for (fu, fv) in [(0.37, 0.41), (0.61, 0.53), (0.23, 0.67), (0.5, 0.5), (0.8, 0.2), (0.15, 0.85), (0.7, 0.9), (0.3, 0.1)] {
        let uv = Vec2::new(lo.x + (hi.x - lo.x) * fu, lo.y + (hi.y - lo.y) * fv);
        if in_loops(s, uv, &loops) {
            pts.push(s.eval(uv.x, uv.y));
            if pts.len() == 3 {
                break;
            }
        }
    }
    let _ = tol;
    (!pts.is_empty()).then_some(pts)
}

/// The cutting passes name the pieces of a split face `<id>~k`; after the
/// fragments inside the pipe are gone a face with a single surviving piece
/// gets its original name back and several pieces are numbered from 1.
fn rename_fragments(out: &mut Body, orig: &Body) {
    use std::collections::BTreeMap;
    let ids: Vec<FaceId> = out.face_ids().collect();
    let mut groups: BTreeMap<String, Vec<FaceId>> = BTreeMap::new();
    for &f in &ids {
        let mut n = out.face(f).name.clone();
        n.region = None;
        groups.entry(n.id()).or_default().push(f);
    }
    for (base, fs) in groups {
        let orig_region = orig.face_ids().filter_map(|g| {
            let mut n = orig.face(g).name.clone();
            let reg = n.region;
            n.region = None;
            (n.id() == base).then_some(reg)
        });
        let orig_region: Vec<Option<u32>> = orig_region.collect();
        if fs.len() == 1 {
            out.faces[fs[0].idx()].name.region = if orig_region.len() == 1 { orig_region[0] } else { None };
        } else {
            for (k, f) in fs.iter().enumerate() {
                out.faces[f.idx()].name.region = Some(k as u32 + 1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use ck_topo::mass::faces_moments;

    /// 40 x 20 x 10 box of the oracle probes (`F2`).
    fn bx() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }

    fn edge_between(b: &Body, fa: &str, fb: &str, at: Vec3) -> EdgeId {
        crate::blend::build::find_edge(b, fa, fb, at).expect("edge")
    }

    fn area(b: &Body) -> f64 {
        let fs: Vec<FaceId> = b.face_ids().collect();
        faces_moments(b, &fs, Vec3::ZERO).area
    }

    fn edge_count(b: &Body) -> usize {
        b.edge_ids().count()
    }

    #[test]
    fn single_edge_matches_oracle() {
        // oracle pipeCutEdge: 6 faces / 15 edges, area 2633.7168 (mesh 2633.788)
        let b = bx();
        let e = edge_between(&b, "F2.C1", "F2.S.e1", Vec3::new(20.0, 10.0, 0.0));
        let out = pipe_cut(&b, &[e], 2.0, 1e-6).expect("pipe cut");
        assert_eq!(out.num_faces(), 6);
        assert_eq!(edge_count(&out), 15);
        // closed form: two 2-wide strips along the edge and two quarter discs
        let expect = 2800.0 - 2.0 * 40.0 * 2.0 - std::f64::consts::PI * 4.0 / 2.0;
        assert!((area(&out) - expect).abs() < 1e-9, "area {} vs {}", area(&out), expect);
        assert!((area(&out) - 2633.71681).abs() < 1e-3);
        assert!(out.find_face("F2.C1").is_some() && out.find_face("F2.C1~1").is_none(), "names restored");
    }

    fn top_edge(b: &Body, side: &str, at: Vec3) -> EdgeId {
        edge_between(b, "F2.C1", side, at)
    }

    fn run(edges: &[(&str, [f64; 3])], r: f64) -> Result<Body, PipeCutError> {
        let b = bx();
        let es: Vec<EdgeId> = edges.iter().map(|(s, p)| top_edge(&b, s, Vec3::new(p[0], p[1], p[2]))).collect();
        pipe_cut(&b, &es, r, 1e-6)
    }

    /// Validity of a pipe-cut sheet: the only topology issues are the open
    /// edges of the cut, and each of those lies exactly `r` from the path.
    fn assert_valid_sheet(body: &Body, out: &Body, path: &[EdgeId], r: f64) {
        use ck_topo::check::{check, Issue};
        let pe: Vec<PipeEdge> = path
            .iter()
            .map(|&e| {
                let ed = body.edge(e);
                PipeEdge { curve: ed.curve.clone(), t0: ed.t0.min(ed.t1), t1: ed.t0.max(ed.t1) }
            })
            .collect();
        let issues = check(out, 1e-6);
        let mut open = 0;
        for i in &issues {
            let Issue::OpenEdge(e) = i else { panic!("unexpected issue {i:?}") };
            open += 1;
            let ed = out.edge(*e);
            for k in [0.25, 0.5, 0.75] {
                let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k);
                let d = dist_to_edges(&pe, p);
                // an open edge is either a cut curve (distance r from the path) or an original edge
                // that the ball did not reach, which cannot be open on a closed input
                assert!((d - r).abs() < 1e-5, "open edge point {p:?} is {d} from the path, r = {r}");
            }
        }
        assert!(open > 0, "a cut leaves a boundary");
    }

    #[test]
    fn sheets_are_valid_and_cut_exactly_at_r() {
        let b = bx();
        let es: Vec<EdgeId> = [E1, E2, E3, E4].iter().map(|(s, p)| top_edge(&b, s, Vec3::new(p[0], p[1], p[2]))).collect();
        for (sel, r) in [(&es[..1], 2.0), (&es[..], 2.0), (&es[..2], 3.0), (&es[..1], 8.0), (&es[..1], 12.0), (&es[1..3], 5.0)] {
            let out = pipe_cut(&b, sel, r, 1e-6).unwrap_or_else(|e| panic!("r={r} n={}: {e:?}", sel.len()));
            assert_valid_sheet(&b, &out, sel, r);
        }
    }

    const E1: (&str, [f64; 3]) =("F2.S.e1", [20.0, 10.0, 0.0]);
    const E2: (&str, [f64; 3]) = ("F2.S.e2", [40.0, 10.0, -10.0]);
    const E3: (&str, [f64; 3]) = ("F2.S.e3", [20.0, 10.0, -20.0]);
    const E4: (&str, [f64; 3]) = ("F2.S.e4", [0.0, 10.0, -10.0]);

    #[test]
    fn ring_two_edges_and_big_radii_match_oracle() {
        // oracle pipeCutRing: 6 / 16, 2336 (the top face's rim band + the side strips)
        let out = run(&[E1, E2, E3, E4], 2.0).expect("ring");
        assert_eq!((out.num_faces(), edge_count(&out)), (6, 16));
        assert!((area(&out) - 2336.0).abs() < 1e-6, "{}", area(&out));
        // oracle pipeCutEdge2 (opposite edges): 6 / 18, 2467.43363
        let out = run(&[E1, E3], 2.0).expect("two");
        assert_eq!((out.num_faces(), edge_count(&out)), (6, 18));
        assert!((area(&out) - 2467.43363).abs() < 1e-3, "{}", area(&out));
        // oracle pipeCutBig r=8: 6 / 15, 2059.46904
        let out = run(&[E1], 8.0).expect("r8");
        assert_eq!((out.num_faces(), edge_count(&out)), (6, 15));
        assert!((area(&out) - 2059.46904).abs() < 1e-3, "{}", area(&out));
        // oracle pipeCut12: the ball reaches past the 10 mm face: it vanishes,
        // the bottom face and the far ends are trimmed: 5 / 12, 1446.711 (mesh)
        let out = run(&[E1], 12.0).expect("r12");
        assert_eq!((out.num_faces(), edge_count(&out)), (5, 12));
        assert!(out.find_face("F2.S.e1").is_none());
        // closed form: top 320, far end 400, bottom 800 - 40*sqrt(44), each near
        // end 200 - (quarter disc r12 - half the segment cut off by the bottom)
        let seg = 144.0 * (10.0f64 / 12.0).acos() - 10.0 * 44.0f64.sqrt();
        let end = 200.0 - (std::f64::consts::PI * 36.0 - 0.5 * seg);
        let expect = 320.0 + 400.0 + (800.0 - 40.0 * 44.0f64.sqrt()) + 2.0 * end;
        assert!((area(&out) - expect).abs() < 1e-6, "{} vs {}", area(&out), expect);
        assert!((area(&out) - 1446.711).abs() < 0.3, "oracle mesh area is a polygonal approximation");
        // oracle pipeCutHuge r=30: every face goes
        let e = run(&[E1], 30.0).unwrap_err();
        assert_eq!(e.code, "pipeCutNothingLeft");
    }

    #[test]
    fn cylinder_rim_and_radius_limit() {
        use crate::boolean::tests::cyl_body;
        let b = cyl_body("F2", 0.0, 0.0, 10.0, 0.0, 10.0);
        let e = edge_between(&b, "F2.C1", "F2.S.e1", Vec3::new(10.0, 10.0, 0.0));
        // oracle pipeCutCyl: 3 faces / 3 edges, mesh area 1016.805
        let out = pipe_cut(&b, &[e], 2.0, 1e-6).expect("rim");
        // (the oracle has no seams: 3 edges; our cylinder keeps its seam line
        // and splits the circles there)
        assert_eq!(out.num_faces(), 3);
        assert!((area(&out) - 1016.805).abs() < 1.5, "{} (oracle: mesh area)", area(&out));
        // closed form: lid disc r = 8, side 10 * 2*pi*10 minus the 2-high band
        let expect = std::f64::consts::PI * 64.0 + std::f64::consts::PI * 100.0 + 2.0 * std::f64::consts::PI * 10.0 * 8.0;
        assert!((area(&out) - expect).abs() < 1e-6, "{} vs {}", area(&out), expect);
        // oracle pipeCutCylTight: radius 11 > 10 -> pipeCutRadius, max 9.99
        let err = pipe_cut(&b, &[e], 11.0, 1e-6).unwrap_err();
        assert_eq!(err.code, "pipeCutRadius");
        assert!((err.max.unwrap() - 9.99).abs() < 1e-9);
    }
}
