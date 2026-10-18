//! Fillets on the edge where a planar face (or a cylinder) meets a cone: slots cut parallel to the axis of a cone
//! frustum, a rib standing on it, notches of the fan shroud. The edge is a conic (or a space curve), so the blend runs on
//! the free-form path: ball centre solved per station, tool swept along the edge and cut / fused into the body.
//!
//! Every fixture is a kit script's document with the fillet as its last feature (`tools/cases/plane_cone_fillet`).
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use ck_topo::body::{Body, Role};
use ck_geom::Vec3;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize { 0 }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec { brep: Brep, labels: Noop }
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> { Some(&mut self.labels) }
    fn face_count(&self) -> Option<usize> { self.brep.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.brep.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> { self.brep.exec(s, f) }
}

struct Built {
    errors: Vec<String>,
    volume: f64,
    /// Validity problems that matter (a pcurve off by under 5e-3 is a fit residual of a free-form blend, not one).
    issues: Vec<String>,
    body: Body,
}

/// Regen the first `upto` features of a fixture (all when `None`).
fn build_upto(src: &str, upto: Option<usize>) -> Built {
    let mut part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    part.rollback = upto;
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let body = ex.brep.body.take().expect("a body");
    let issues = ck_topo::check::check(&body, 1e-6)
        .into_iter()
        .filter(|i| !matches!(i, ck_topo::check::Issue::PcurveOff(_, d) if *d < 5e-3))
        .map(|i| format!("{i:?}"))
        .collect();
    Built { errors, volume: ck_topo::mass::mass_props(&body).volume, issues, body }
}

fn features(src: &str) -> Vec<Json> {
    ck_json::parse(src).unwrap().get("features").unwrap().as_arr().unwrap().to_vec()
}

/// The fillet of a fixture: its radius and the (face, face, point) of the edge.
fn fillet_spec(src: &str) -> (f64, [String; 2], Vec3) {
    let f = features(src).pop().unwrap();
    assert_eq!(f.get_str("type"), Some("fillet"));
    let r = f.get("radius").unwrap().as_f64().unwrap();
    let e = &f.get("edges").unwrap().as_arr().unwrap()[0];
    let fs = e.get("faces").unwrap().as_arr().unwrap();
    let at = e.get("at").unwrap().as_arr().unwrap();
    (
        r,
        [fs[0].as_str().unwrap().to_string(), fs[1].as_str().unwrap().to_string()],
        Vec3::new(at[0].as_f64().unwrap(), at[1].as_f64().unwrap(), at[2].as_f64().unwrap()),
    )
}

/// The volume a rolling-ball fillet of radius `r` along the edge changes the body by: the sliver between the corner
/// and the arc, `r^2 (tan(phi/2) - phi/2)` per unit length for a turn of `phi` between the face normals, integrated
/// along the edge. Negative (material removed) for a convex edge, positive for a concave one.
fn expected_change(base: &Body, spec: &(f64, [String; 2], Vec3)) -> f64 {
    let (r, faces, at) = spec;
    let e = ck_ops::blend::build::find_edge(base, &faces[0], &faces[1], *at).expect("the edge of the fillet");
    let ed = base.edge(e);
    let fs = base.edge_faces(e);
    let n = 64;
    let mut prev: Option<Vec3> = None;
    let (mut total, mut sign) = (0.0, 0.0);
    for i in 0..=n {
        let t = ed.t0 + (ed.t1 - ed.t0) * i as f64 / n as f64;
        let p = ed.curve.eval(t);
        let nrm = |f| {
            let (u, v) = base.face(f).surface.closest_param(p);
            base.outward_normal(f, u, v).normalize()
        };
        let (n1, n2) = (nrm(fs[0]), nrm(fs[1]));
        let phi = n1.dot(n2).clamp(-1.0, 1.0).acos();
        let area = r * r * ((phi / 2.0).tan() - phi / 2.0);
        // convex when the second face's normal leans the way the first face's surface runs on across the edge
        let tdir = ed.curve.tangent(t).normalize();
        let co = base.edge_coedges(e).into_iter().find(|&c| base.coedge_face(c) == fs[0]).unwrap();
        let tdir = if base.coedge(co).rev { -tdir } else { tdir };
        let s = if n1.cross(tdir).dot(n2) < 0.0 { 1.0 } else { -1.0 };
        sign += s;
        if let Some(q) = prev {
            total += area * p.dist(q);
        }
        prev = Some(p);
        let _ = area;
    }
    // (the area of the end point is counted once per segment; close enough for a smooth edge)
    if sign > 0.0 { -total } else { total }
}

/// Faces of the fillet feature `F<k>` that are blends.
fn blend_faces(b: &Body, feature: &str) -> usize {
    b.face_ids().filter(|&f| b.face(f).name.feature == feature && matches!(b.face(f).name.role, Role::Blend(_))).count()
}

fn is_ok(label: &str, b: &Built) {
    assert!(b.errors.is_empty(), "{label}: regen errors {:?}", b.errors);
    assert!(b.issues.is_empty(), "{label}: body issues {:?}", b.issues);
}

/// Largest angle between the blend face's normal and its neighbour's along their shared edges with the two faces of the
/// filleted edge (the contact curves; the blend's ends meet other faces at an angle by design).
fn worst_tangent_break(b: &Body, feature: &str, faces: &[String; 2]) -> f64 {
    let mut worst = 0.0f64;
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if fs.len() != 2 {
            continue;
        }
        let (a, c) = (fs[0], fs[1]);
        let isb = |f| b.face(f).name.feature == feature && matches!(b.face(f).name.role, Role::Blend(_));
        if isb(a) == isb(c) {
            continue;
        }
        let other = if isb(a) { c } else { a };
        let id = b.face_id_str(other);
        if !faces.iter().any(|f| id.starts_with(f.as_str())) {
            continue;
        }
        let ed = b.edge(e);
        for i in 1..8 {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 8.0);
            let nrm = |f| {
                let (u, v) = b.face(f).surface.closest_param(p);
                b.outward_normal(f, u, v).normalize()
            };
            let ang = nrm(a).dot(nrm(c)).clamp(-1.0, 1.0).acos();
            worst = worst.max(ang);
        }
    }
    worst
}

type Fixture = (&'static str, &'static str);

const CONE_SLOTS: &[Fixture] = &[
    ("cone_hollow_axis_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_axis_inner_r0.5.json")),
    ("cone_hollow_axis_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_axis_inner_r2.json")),
    ("cone_hollow_axis_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_axis_outer_r0.5.json")),
    ("cone_hollow_axis_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_axis_outer_r2.json")),
    ("cone_hollow_incl_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_incl_inner_r0.5.json")),
    ("cone_hollow_incl_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_incl_inner_r2.json")),
    ("cone_hollow_incl_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_incl_outer_r0.5.json")),
    ("cone_hollow_incl_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_incl_outer_r2.json")),
    ("cone_hollow_off_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_off_inner_r0.5.json")),
    ("cone_hollow_off_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_off_inner_r2.json")),
    ("cone_hollow_off_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_off_outer_r0.5.json")),
    ("cone_hollow_off_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_hollow_off_outer_r2.json")),
    ("cone_solid_axis_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_axis_outer_r0.5.json")),
    ("cone_solid_axis_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_axis_outer_r2.json")),
    ("cone_solid_incl_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_incl_outer_r0.5.json")),
    ("cone_solid_incl_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_incl_outer_r2.json")),
    ("cone_solid_off_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_off_outer_r0.5.json")),
    ("cone_solid_off_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_off_outer_r2.json")),
];

const CYLINDER_SLOTS: &[Fixture] = &[
    ("cyl_hollow_axis_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_axis_inner_r0.5.json")),
    ("cyl_hollow_axis_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_axis_inner_r2.json")),
    ("cyl_hollow_axis_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_axis_outer_r0.5.json")),
    ("cyl_hollow_axis_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_axis_outer_r2.json")),
    ("cyl_hollow_incl_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_incl_inner_r0.5.json")),
    ("cyl_hollow_incl_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_incl_inner_r2.json")),
    ("cyl_hollow_incl_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_incl_outer_r0.5.json")),
    ("cyl_hollow_incl_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_incl_outer_r2.json")),
    ("cyl_hollow_off_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_off_inner_r0.5.json")),
    ("cyl_hollow_off_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_off_inner_r2.json")),
    ("cyl_hollow_off_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_off_outer_r0.5.json")),
    ("cyl_hollow_off_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_hollow_off_outer_r2.json")),
    ("cyl_solid_axis_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_axis_outer_r0.5.json")),
    ("cyl_solid_axis_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_axis_outer_r2.json")),
    ("cyl_solid_incl_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_incl_outer_r0.5.json")),
    ("cyl_solid_incl_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_incl_outer_r2.json")),
    ("cyl_solid_off_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_off_outer_r0.5.json")),
    ("cyl_solid_off_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cyl_solid_off_outer_r2.json")),
];

const CYLINDER_WALLED_NOTCHES: &[Fixture] = &[
    ("cylwall_hollow_b-2_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b-2_inner_r0.5.json")),
    ("cylwall_hollow_b-2_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b-2_inner_r2.json")),
    ("cylwall_hollow_b-2_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b-2_outer_r0.5.json")),
    ("cylwall_hollow_b-2_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b-2_outer_r2.json")),
    ("cylwall_hollow_b2_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b2_inner_r0.5.json")),
    ("cylwall_hollow_b2_inner_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b2_inner_r2.json")),
    ("cylwall_hollow_b2_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b2_outer_r0.5.json")),
    ("cylwall_hollow_b2_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_hollow_b2_outer_r2.json")),
    ("cylwall_solid_b-2_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_solid_b-2_outer_r0.5.json")),
    ("cylwall_solid_b-2_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_solid_b-2_outer_r2.json")),
    ("cylwall_solid_b2_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_solid_b2_outer_r0.5.json")),
    ("cylwall_solid_b2_outer_r2", include_str!("../../../tools/cases/plane_cone_fillet/cylwall_solid_b2_outer_r2.json")),
];

const RIBS: &[Fixture] = &[
    ("fin_hollow_inner_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/fin_hollow_inner_r0.5.json")),
    ("fin_hollow_inner_r1", include_str!("../../../tools/cases/plane_cone_fillet/fin_hollow_inner_r1.json")),
    ("fin_hollow_outer_r0.5", include_str!("../../../tools/cases/plane_cone_fillet/fin_hollow_outer_r0.5.json")),
    ("fin_hollow_outer_r1", include_str!("../../../tools/cases/plane_cone_fillet/fin_hollow_outer_r1.json")),
];

const FLOOR_NOTCHES: &[Fixture] = &[
    ("notch_floor_in", include_str!("../../../tools/cases/plane_cone_fillet/notch_floor_in.json")),
    ("notch_floor_out", include_str!("../../../tools/cases/plane_cone_fillet/notch_floor_out.json")),
    ("notch_vsharp_in", include_str!("../../../tools/cases/plane_cone_fillet/notch_vsharp_in.json")),
    ("notch_vsharp_out", include_str!("../../../tools/cases/plane_cone_fillet/notch_vsharp_out.json")),
];

const FAN: &[Fixture] = &[
    ("fan_inleft05", include_str!("../../../tools/cases/plane_cone_fillet/fan_inleft05.json")),
    ("fan_inleft2", include_str!("../../../tools/cases/plane_cone_fillet/fan_inleft2.json")),
    ("fan_outleft05", include_str!("../../../tools/cases/plane_cone_fillet/fan_outleft05.json")),
];

/// Fillet fixture checks: native (no regen error), valid, a blend face, volume as the rolling-ball sliver predicts,
/// tangent to its neighbours. Returns the ratio of the volume change to the prediction.
fn check_convex_or_concave(label: &str, src: &str, want_sign: f64) -> f64 {
    let n = features(src).len();
    let base = build_upto(src, Some(n - 1));
    assert!(base.errors.is_empty() && base.issues.is_empty(), "{label}: base {:?} {:?}", base.errors, base.issues);
    let got = build_upto(src, None);
    is_ok(label, &got);
    let fid = features(src).pop().unwrap().get_str("id").unwrap().to_string();
    let spec = fillet_spec(src);
    assert!(blend_faces(&got.body, &fid) >= 1, "{label}: no blend face");
    let change = got.volume - base.volume;
    assert!(change * want_sign > 0.0, "{label}: volume {} -> {} has the wrong sense", base.volume, got.volume);
    let want = expected_change(&base.body, &spec);
    let ratio = change / want;
    // (a ball rolling in a concave wall as curved as the bowl cone removes up to a quarter less than the plane formula)
    assert!((0.7..1.3).contains(&ratio), "{label}: volume change {change} vs predicted {want} (ratio {ratio})");
    let tb = worst_tangent_break(&got.body, &fid, &spec.1);
    assert!(tb < 5e-3, "{label}: blend leaves its neighbour by {tb} rad");
    ratio
}

/// A slot wall (plane) meeting a cone frustum, solid and hollow, plane off the axis / through it / inclined, outer and
/// inner cone, R0.5 and R2: all native, the convex edge loses material.
#[test]
fn planar_slot_walls_fillet_into_a_cone() {
    for (name, src) in CONE_SLOTS {
        check_convex_or_concave(name, src, -1.0);
    }
}

/// Same slots on a cylinder (a straight generator edge, the analytic builder has no section for it).
#[test]
fn planar_slot_walls_fillet_into_a_cylinder() {
    for (name, src) in CYLINDER_SLOTS {
        check_convex_or_concave(name, src, -1.0);
    }
}

/// A cylindrical notch wall meeting a cone (space-curve edge), bulging either way.
#[test]
fn cylindrical_notch_walls_fillet_into_a_cone() {
    for (name, src) in CYLINDER_WALLED_NOTCHES {
        check_convex_or_concave(name, src, -1.0);
    }
}

/// A rib standing on the cone wall, flush with its end planes (which meet the tapered wall at an angle): the
/// concave edge gains material, the fillet ends flush with the end faces.
#[test]
fn ribs_on_a_cone_gain_a_concave_fillet() {
    for (name, src) in RIBS {
        check_convex_or_concave(name, src, 1.0);
    }
}

/// The edge of a wall that runs down into the floor of the notch (a concave corner at the chain end).
#[test]
fn fillet_may_end_at_the_floor_of_a_notch() {
    for (name, src) in FLOOR_NOTCHES {
        check_convex_or_concave(name, src, -1.0);
    }
}

/// The fan shroud: a 4 degree drafted conical wall with a notch cut through by an extruded profile, its straight 4 degree
/// wall meeting the bowl cone (R0.5, R2) and the outer cone (R0.5), on the model with pillar walls and banded cuts.
#[test]
fn fan_shroud_notch_edges_fillet_into_the_cones() {
    for (name, src) in FAN {
        check_convex_or_concave(name, src, -1.0);
    }
}

/// The fan without the fillet is valid (the base the cases above are measured from).
#[test]
fn fan_shroud_base_is_valid() {
    let b = build_upto(include_str!("../../../tools/cases/plane_cone_fillet/fan_base.json"), None);
    is_ok("fan base", &b);
}

/// Changing the radius changes the result smoothly: the volume change grows about with r^2.
#[test]
fn volume_change_scales_with_the_radius_squared() {
    let a = check_convex_or_concave("r0.5", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_off_outer_r0.5.json"), -1.0);
    let b = check_convex_or_concave("r2", include_str!("../../../tools/cases/plane_cone_fillet/cone_solid_off_outer_r2.json"), -1.0);
    assert!((a - b).abs() < 0.2, "{a} vs {b}");
    let _ = (PI, Surface::plane(Vec3::ZERO, Vec3::Z));
}

#[test]
#[ignore]
fn report() {
    for set in [CONE_SLOTS, CYLINDER_SLOTS, CYLINDER_WALLED_NOTCHES, RIBS, FLOOR_NOTCHES, FAN] {
        for (name, src) in set {
            let n = features(src).len();
            let base = build_upto(src, Some(n - 1));
            let got = build_upto(src, None);
            let spec = fillet_spec(src);
            let want = expected_change(&base.body, &spec);
            let fid = features(src).pop().unwrap().get_str("id").unwrap().to_string();
            eprintln!("{name:44} base {:10.3} -> {:10.3} change {:8.3} predicted {:8.3} ratio {:5.3} tangent {:.1e} errs {:?} issues {:?}", base.volume, got.volume, got.volume - base.volume, want, (got.volume - base.volume) / want, worst_tangent_break(&got.body, &fid, &spec.1), got.errors, got.issues);
        }
    }
}
