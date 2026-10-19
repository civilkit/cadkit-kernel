//! Fillets the user found wrong by hand (`tools/cases/user_fillet/`).
//!
//! 1. R1 on the top edges of a plate whose outline is four lines joined by four tangent R5 arcs: the blend must be one
//!    continuous round-over, no planar step where a line fillet meets an arc fillet.
use ck_check::tessellation::check_mesh_why;
use ck_geom::surface::Surface;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::collections::HashMap;
use std::f64::consts::PI;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

struct Built {
    errors: Vec<String>,
    volume: f64,
    issues: Vec<String>,
    mesh_issues: Vec<String>,
    /// mesh edges (vertices welded at 1e-4) within `near` of a point that are used by one triangle or by more than two
    bad_mesh_edges: Vec<String>,
    /// faces of the last feature that are planar (a step between blend faces)
    planar_blend_faces: usize,
    /// blend faces of the last feature
    blends: usize,
}

/// Regen `src` (optionally without its last feature); `near` are the points around which the mesh must be closed.
fn build_with(src: &str, drop_last: bool, feature: &str, near: &[[f64; 3]]) -> Built {
    let mut part = Part::parse(src).unwrap();
    if drop_last {
        part.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().filter(|e| e.feature != "F3").map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    let b = ex.0.body.clone().expect("a body");
    let m = ck_tess::mesh::tessellate(&b).mesh;
    let mesh_issues = check_mesh_why(&b, &m.positions, &m.tri_face, &m.failed, &m.failed_why).into_iter().map(|i| format!("{}: {}", i.entity, i.detail)).collect();
    let issues = ck_topo::check::check(&b, 1e-6).iter().map(|i| format!("{i:?}")).collect();
    let planar_blend_faces = b.face_ids().filter(|&f| b.face(f).name.feature == feature && matches!(b.face(f).surface, Surface::Plane { .. })).count();
    let blends = b.face_ids().filter(|&f| b.face(f).name.feature == feature && matches!(b.face(f).name.role, ck_topo::body::Role::Blend(_))).count();
    Built { blends, errors, volume: ck_topo::mass::mass_props(&b).volume, issues, mesh_issues, bad_mesh_edges: bad_edges(&m.positions, near), planar_blend_faces }
}

/// Undirected mesh edges near the given points that are not shared by exactly two triangles.
fn bad_edges(pos: &[f32], near: &[[f64; 3]]) -> Vec<String> {
    let key = |i: usize| -> [i64; 3] { [0, 1, 2].map(|k| (pos[i * 3 + k] as f64 * 1e4).round() as i64) };
    let mut uses: HashMap<([i64; 3], [i64; 3]), usize> = HashMap::new();
    for t in 0..pos.len() / 9 {
        for e in 0..3 {
            let (a, b) = (key(t * 3 + e), key(t * 3 + (e + 1) % 3));
            if a == b {
                continue;
            }
            *uses.entry(if a < b { (a, b) } else { (b, a) }).or_default() += 1;
        }
    }
    let close = |k: [i64; 3]| near.iter().any(|p| (0..3).map(|i| (k[i] as f64 / 1e4 - p[i]).powi(2)).sum::<f64>().sqrt() < 3.0);
    uses.into_iter().filter(|&((a, b), n)| n != 2 && (close(a) || close(b))).map(|((a, b), n)| format!("{a:?}-{b:?} used {n}x")).collect()
}

/// The 8 tangent vertices of the plate outline (line meets R5 arc), on the top face.
fn junctions() -> Vec<[f64; 3]> {
    let mut v = vec![];
    for s in [-1.0, 1.0] {
        for t in [-22.5, 22.5] {
            v.push([s * 27.5, 5.0, t]);
            v.push([t, 5.0, s * 27.5]);
        }
    }
    v
}

/// 55 x 55 x 5 plate, R5 corners, R1 round-over of the 8 top edges. Volume: the plate minus the swept corner section
/// (area (1 - pi/4) r^2, centroid 0.2232 r inside the wall) along the 4 lines (45 each) and 4 quarter arcs of radius 5.
fn plate_r1_volume() -> f64 {
    let a = 1.0 - PI / 4.0;
    let c = (5.0 / 6.0 - PI / 4.0) / a;
    (55.0 * 55.0 - (4.0 - PI) * 25.0) * 5.0 - a * (4.0 * 45.0 + 2.0 * PI * (5.0 - c))
}

#[test]
fn rounded_plate_top_fillet_is_continuous_at_the_line_arc_junctions() {
    let b = build_with(include_str!("../../../tools/cases/user_fillet/plate_top_r1.json"), false, "F13", &junctions());
    assert!(b.errors.is_empty(), "{:?}", b.errors);
    assert!(b.issues.is_empty(), "{:?}", b.issues);
    assert!(b.mesh_issues.is_empty(), "{:?}", b.mesh_issues);
    assert!(b.bad_mesh_edges.is_empty(), "{:?}", b.bad_mesh_edges);
    assert_eq!(b.planar_blend_faces, 0, "no planar end-cap face is left between the blend faces");
    let want = plate_r1_volume();
    assert!((b.volume - want).abs() < 1e-3 * want, "{} vs {}", b.volume, want);
}

/// The user's document (holes, a thickened cone on the plate) with the same fillet F13 as its last feature: the fillet
/// removes exactly the swept section from the document without it.
#[test]
fn user_document_plate_fillet_removes_the_analytic_volume() {
    let src = include_str!("../../../tools/cases/user_fillet/fillet_issue.ckpart");
    let base = build_with(src, true, "F13", &[]);
    let b = build_with(src, false, "F13", &junctions());
    assert_eq!(b.errors, base.errors);
    assert!(b.issues.is_empty(), "{:?}", b.issues);
    assert!(b.mesh_issues.is_empty(), "{:?}", b.mesh_issues);
    assert!(b.bad_mesh_edges.is_empty(), "{:?}", b.bad_mesh_edges);
    assert_eq!(b.planar_blend_faces, 0);
    let removed = (plate_r1_volume() - (55.0 * 55.0 - (4.0 - PI) * 25.0) * 5.0).abs();
    let got = base.volume - b.volume;
    assert!((got - removed).abs() < 2e-2 * removed, "removed {got} vs {removed}");
}


// ---- the foot of the thickened cone (R12 -> R8 over 30, wall 3 outward) on the plate ----

/// Document F10 with its rim fillets F11, F12 and the plate fillet F13 (the user's document).
const DOC: &str = include_str!("../../../tools/cases/user_fillet/fillet_issue.ckpart");

/// Volume of the wedge between the wall base ring and the plate that the thicken now closes (Pappus on the triangle
/// inner foot circle, base ring outer edge, wall line at the plate; 52.7790).
const WEDGE: f64 = 52.7790;
/// Added by the concave R1, R2 fillet of the outer foot and by R2 at the inner foot (Pappus on the corner region:
/// 0.15639 at radius 15.1931, 0.62558 at 15.3597, 1.16204 at 11.4104).
const OUTER_R1: f64 = 14.9296;
const OUTER_R2: f64 = 60.3732;
const INNER_R2: f64 = 83.3104;

fn foot(src: &str) -> (Built, Built) {
    let base = build_with(DOC, false, "F99", &[]);
    let b = build_with(src, false, "F99", &[]);
    assert!(b.errors.is_empty() && base.errors == b.errors, "{:?}", b.errors);
    assert!(b.issues.is_empty(), "{:?}", b.issues);
    assert!(b.mesh_issues.is_empty(), "{:?}", b.mesh_issues);
    assert!(b.bad_mesh_edges.is_empty(), "{:?}", b.bad_mesh_edges);
    // the ring is split in arcs by the sheet's seam; the fillet still goes all the way round as one face
    assert_eq!(b.blends, 1, "one blend face all the way round");
    (base, b)
}

fn close(got: f64, want: f64) {
    assert!((got - want).abs() < 2e-3 * want.max(10.0), "{got} vs {want}");
}

/// The unfilleted document closes the wedge: the wall meets the plate in a cone / plane edge, no hovering base ring.
#[test]
fn thickened_wall_is_sunk_into_the_plate() {
    let base = build_with(DOC, false, "F99", &[]);
    // F13 plate fillet, volume of the document without the wedge: 20028.2713 (before the thicken was sunk)
    assert!((base.volume - (20028.2713 + WEDGE)).abs() < 1e-3 * WEDGE * 20.0, "{}", base.volume);
}

/// The outer foot edge: a fillet adds material (concave, tangent to the cone and the plate top) where it used to round
/// the hovering base corner over and remove 20 to 78.
#[test]
fn outer_foot_fillet_adds_material() {
    let (base, r1) = foot(include_str!("../../../tools/cases/user_fillet/outer_foot_r1_sunk.ckpart"));
    close(r1.volume - base.volume, OUTER_R1);
    let (base, r2) = foot(include_str!("../../../tools/cases/user_fillet/outer_foot_r2_sunk.ckpart"));
    close(r2.volume - base.volume, OUTER_R2);
}

/// The inner foot edge fillets the corner inside the bore (between the sheet cone and the plate top annulus): 83 mm3
/// of material in the bore, out of sight from outside.
#[test]
fn inner_foot_fillet_lies_inside_the_bore() {
    let (base, r2) = foot(include_str!("../../../tools/cases/user_fillet/inner_foot_r2_sunk.ckpart"));
    close(r2.volume - base.volume, INNER_R2);
}

/// A saved selection that names the old base ring (`F10.W0`) no longer names an edge: a structured `missingEdge`, never
/// a different blend.
#[test]
fn selection_of_the_old_base_ring_is_a_missing_edge() {
    let b = build_with(include_str!("../../../tools/cases/user_fillet/outer_foot_r2.ckpart"), false, "F99", &[]);
    assert!(b.errors.iter().any(|e| e.starts_with("F99: ") && e.contains("missingEdge")), "{:?}", b.errors);
}
