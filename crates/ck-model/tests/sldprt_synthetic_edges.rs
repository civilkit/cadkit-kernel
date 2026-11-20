//! Edges the SolidWorks reader refused although the geometry is sound, as tiny synthetic parts built through the
//! public reader entry (`ck_sldprt::build::build` on a hand-made `Ir`; no file is involved):
//!  - a tolerant edge whose fin curve is an SP-curve on a rolling-ball blend: the SP-curve is in the file's parameters
//!    (the spine's own), not the built surface's, so it landed far from its vertices;
//!  - vertices a little off a curve fitted through chart points: they take the gap as their tolerance;
//!  - an intersection curve of a blend and a plane just past the end of the blend's section: the blend continues for it.

use ck_sldprt::build::build;
use ck_sldprt::ir::*;
use ck_sldprt::Limits;

type V = [f64; 3];

fn sn(id: i64, geom: IrSurface) -> IrSurfaceNode {
    IrSurfaceNode { id, geom, sense_plus: true }
}

fn cn(id: i64, geom: IrCurve) -> IrCurveNode {
    IrCurveNode { id, geom, sense_plus: true }
}

fn vx(id: i64, p: V) -> IrVertex {
    IrVertex { id, p, tol: None }
}

fn line(id: i64, o: V, d: V) -> IrCurveNode {
    cn(id, IrCurve::Line { o, d })
}

/// One sheet face on surface 0-based index `surface` bounded by `edges` (curve, fin curve, start, end, tol), traversed in order.
fn sheet(surfaces: Vec<IrSurfaceNode>, curves: Vec<IrCurveNode>, vertices: Vec<IrVertex>, edges: Vec<IrEdge>, surface: usize) -> Ir {
    let fins = (0..edges.len()).map(|edge| IrFin { edge, forward: true }).collect();
    let face = IrFace { id: 40, surface: Some(surface), same_sense: true, loops: vec![IrLoop { id: 50, fins, vertex: None }], back: false, colour: None, sw_id: None, sw_ids: vec![] };
    let body = IrBody { id: 70, kind: BodyKind::Sheet, regions: vec![IrRegion { solid: true, shells: vec![IrShell { id: 60, faces: vec![face] }] }], density: None, last_feature: None };
    Ir { schema: "test".into(), bodies: vec![body], surfaces, curves, vertices, edges, notes: vec![], import_sheets: true }
}

fn edge(id: i64, curve: Option<usize>, fin: Option<usize>, a: usize, b: usize, tol: Option<f64>) -> IrEdge {
    IrEdge { id, curve, start: Some(a), end: Some(b), fin_curve: fin, tol }
}

fn faces_left_out(ir: &Ir) -> Vec<String> {
    let built = build(&[ir], &Limits::default()).unwrap();
    built.parts.iter().flat_map(|p| p.refused.iter().map(|r| format!("{}: {}", r.0, r.1.message))).collect()
}

/// A 1 mm round between the planes z = 0 and x = 0 along y (spine y -10..10 mm), as surface 2 of the returned lists.
fn round() -> (Vec<IrSurfaceNode>, Vec<IrCurveNode>) {
    let r = 0.001;
    let surfaces = vec![
        sn(1, IrSurface::Plane { o: [0.0; 3], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] }),
        sn(2, IrSurface::Plane { o: [0.0; 3], n: [1.0, 0.0, 0.0], x: [0.0, 1.0, 0.0] }),
        sn(3, IrSurface::Blend { supports: [Some(0), Some(1)], support_plus: [true, true], spine: Some(0), spine_plus: true, range: [r, r], limits: [Some([r, -0.01, r]), Some([r, 0.01, r])] }),
    ];
    (surfaces, vec![line(4, [r, 0.0, r], [0.0, 1.0, 0.0])])
}

fn sp_section(id: i64, u: f64) -> Vec<IrCurveNode> {
    let uv = NurbsCurveIr { degree: 1, dim: 2, ctrl: vec![vec![u, 0.0], vec![u, 1.0]], weights: None, knots: vec![0.0, 0.0, 1.0, 1.0], periodic: false, closed: false };
    vec![cn(id, IrCurve::Sp { surface: Some(2), uv: Some(uv) }), cn(id + 1, IrCurve::Trimmed { basis: Some(0), p1: [0.0; 3], p2: [0.0; 3], t1: 0.0, t2: 1.0 })]
}

#[test]
fn sp_curve_sections_of_a_blend_in_the_files_parameters_are_read() {
    let (surfaces, mut curves) = round();
    let r = 0.001;
    // curves: 0 spine, 1 foot on z = 0, 2 foot on x = 0, then the two sections as (SP, trimmed) pairs
    curves.push(line(5, [r, 0.0, 0.0], [0.0, 1.0, 0.0]));
    curves.push(line(6, [0.0, 0.0, r], [0.0, 1.0, 0.0]));
    let mut sp = sp_section(7, 0.0123);
    sp[1] = cn(8, IrCurve::Trimmed { basis: Some(3), p1: [0.0; 3], p2: [0.0; 3], t1: 0.0, t2: 1.0 });
    curves.extend(sp);
    let mut sp2 = sp_section(9, 0.093);
    sp2[0] = cn(9, IrCurve::Sp { surface: Some(2), uv: match &sp2[0].geom { IrCurve::Sp { uv, .. } => uv.clone(), _ => None } });
    sp2[1] = cn(10, IrCurve::Trimmed { basis: Some(5), p1: [0.0; 3], p2: [0.0; 3], t1: 0.0, t2: 1.0 });
    curves.extend(sp2);
    let vertices = vec![vx(20, [r, -0.005, 0.0]), vx(21, [r, 0.005, 0.0]), vx(22, [0.0, 0.005, r]), vx(23, [0.0, -0.005, r])];
    let edges = vec![edge(30, Some(1), None, 0, 1, None), edge(31, None, Some(4), 1, 2, Some(3e-5)), edge(32, Some(2), None, 2, 3, None), edge(33, None, Some(6), 3, 0, Some(3e-5))];
    let left = faces_left_out(&sheet(surfaces, curves, vertices, edges, 2));
    assert!(left.is_empty(), "{left:?}");
}

#[test]
fn vertices_a_little_off_a_fitted_curve_are_read_with_a_tolerance() {
    // a square on z = 0 whose side on x = 0 is the planes' meeting line as a chart; its vertices are 0.01 mm off it
    let surfaces = vec![sn(1, IrSurface::Plane { o: [0.0; 3], n: [0.0, 0.0, 1.0], x: [1.0, 0.0, 0.0] }), sn(2, IrSurface::Plane { o: [0.0; 3], n: [1.0, 0.0, 0.0], x: [0.0, 1.0, 0.0] })];
    let pts: Vec<V> = (0..5).map(|k| [0.0, -0.005 + 0.0025 * k as f64, 0.0]).collect();
    let curves = vec![
        cn(3, IrCurve::Intersection { pts, surfaces: [Some(0), Some(1)], bounds: [None, None], chordal: 0.0 }),
        line(4, [0.0, -0.005, 0.0], [1.0, 0.0, 0.0]),
        line(5, [0.01, 0.0, 0.0], [0.0, 1.0, 0.0]),
        line(6, [0.0, 0.005, 0.0], [1.0, 0.0, 0.0]),
    ];
    let e = 1e-5;
    let vertices = vec![vx(20, [e, -0.005, 0.0]), vx(21, [0.01, -0.005, 0.0]), vx(22, [0.01, 0.005, 0.0]), vx(23, [e, 0.005, 0.0])];
    let edges = vec![edge(30, Some(1), None, 0, 1, None), edge(31, Some(2), None, 1, 2, None), edge(32, Some(3), None, 2, 3, None), edge(33, Some(0), None, 3, 0, None)];
    let left = faces_left_out(&sheet(surfaces, curves, vertices, edges, 0));
    assert!(left.is_empty(), "{left:?}");
}

#[test]
fn an_intersection_with_a_blend_just_past_its_section_is_read() {
    // the round ends at y = 10 mm; the plane y = 10.1 mm meets its continuation in a quarter arc; the face is the
    // sector of that plane between the arc and the ball's centre
    let (mut surfaces, mut curves) = round();
    let (r, y) = (0.001, 0.0101);
    surfaces.push(sn(8, IrSurface::Plane { o: [0.0, y, 0.0], n: [0.0, 1.0, 0.0], x: [1.0, 0.0, 0.0] }));
    let pts: Vec<V> = (0..4).map(|k| {
        let a = std::f64::consts::FRAC_PI_2 * k as f64 / 3.0;
        [r - r * a.sin(), y, r - r * a.cos()]
    }).collect();
    curves.push(cn(9, IrCurve::Intersection { pts, surfaces: [Some(2), Some(3)], bounds: [None, None], chordal: 0.0 }));
    curves.push(line(10, [0.0, y, r], [1.0, 0.0, 0.0]));
    curves.push(line(11, [r, y, 0.0], [0.0, 0.0, 1.0]));
    let vertices = vec![vx(20, [r, y, 0.0]), vx(21, [0.0, y, r]), vx(22, [r, y, r])];
    let edges = vec![edge(30, Some(1), None, 0, 1, None), edge(31, Some(2), None, 1, 2, None), edge(32, Some(3), None, 2, 0, None)];
    let left = faces_left_out(&sheet(surfaces, curves, vertices, edges, 3));
    assert!(left.is_empty(), "{left:?}");
}
