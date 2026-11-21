//! A fillet on the end rims of a solid tube built from a swept side face.
//!
//! The plain `sweep` tube (side face a torus; here a quarter arc path) now BUILDS. Its free-form ring tool is a B-spline strip
//! that closes on itself in u: the boolean could find no interior point of the tool's band (its two boundary loops are closed
//! rails, flat in uv), classified every point as outside it, and the mass integral of the band was zero. Those three read the
//! band as the rectangle between its rails now (`merge_strip_loops` in the classifier, the seam sides in `face_pieces` of the
//! mass integral), and the tool is oriented from its blend face rather than from a volume that is rounding noise.
//!
//! The `sweepSurface` + `knit` tube (side face a procedural sweep surface) builds too: the face is rounded on its certified
//! B-spline, and the closed profile of that spline (no declared period) is read through its v seam by the classifier, the
//! sampler and the mass integral (`closed_v_span`, `closed_v_width`).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

const TORUS: &str = include_str!("fixtures/torus_tube_rim_fillet.ckpart.json");
const TORUS_ONE: &str = include_str!("fixtures/torus_tube_one_rim_fillet.ckpart.json");
const SWEEP_SURFACE: &str = include_str!("fixtures/sweep_tube_rim_fillet.ckpart.json");
const SWEEP_SURFACE_ONE: &str = include_str!("fixtures/sweep_tube_one_rim_fillet.ckpart.json");
/// The tube's radius (the profile circle) and the volume of the tube before the fillet (the fixture's arc path and profile).
const RHO: f64 = 2.1990034161013345;

/// (errors as (feature, code, text), body, seconds) of a regen of `doc` with the fillet radius replaced.
fn run(doc: &str, radius: Option<f64>) -> (Vec<(String, String, String)>, Option<ck_topo::body::Body>, f64) {
    let doc = match radius {
        Some(r) => doc.replace("\"radius\": 0.2", &format!("\"radius\": {r}")),
        None => doc.to_string(),
    };
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let t = std::time::Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone(), e.error.describe())).collect();
    (errs, ex.0.body, t.elapsed().as_secs_f64())
}

fn volume(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

type Rim = (&'static str, &'static str, [f64; 3]);
const TORUS_RIMS: [Rim; 2] = [("F3.C0", "F3.S.e1", [-2.199, 0.0, 0.0]), ("F3.C1", "F3.S.e1", [30.0, 0.0, -32.199])];
const SWEEP_RIMS: [Rim; 2] = [("F3.C0", "F3.W", [-2.1990034161013345, 0.0, 0.0]), ("F3.C1", "F3.W", [21.861945984872428, 0.0, 17.209714219317448])];

/// Area of the corner section a round of radius `r` removes where the material wedge has interior angle `alpha`.
fn corner_section(alpha: f64, r: f64) -> f64 {
    r * r * ((0.5 * alpha).tan().recip() - 0.5 * (std::f64::consts::PI - alpha))
}

/// Depth below the wall (the side face) of the centroid of that corner section: the rim's circle is shorter there, so the section
/// sweeps `1 - depth / RHO` of the length it would on a straight edge (Pappus).
fn corner_centroid_depth(alpha: f64, r: f64) -> f64 {
    let (h, phi) = (0.5 * alpha, 0.5 * (std::f64::consts::PI - alpha));
    let d_c = r / h.sin();
    let (kite, d_kite) = (r * r / h.tan(), (d_c + r / h.tan() * h.cos()) / 3.0);
    let (sector, d_sector) = (phi * r * r, d_c - 2.0 * r * phi.sin() / (3.0 * phi));
    let d = (kite * d_kite - sector * d_sector) / (kite - sector);
    d * h.sin()
}

/// Volume a round of radius `r` removes along the rim between faces `cap` and `side` of `b`: the corner section for the local
/// interior angle of the material wedge (from the two outward normals), integrated along the rim edge near `p` (the angle of
/// an oblique tube changes round the rim). The section is weighted by the shorter circle its centroid sweeps (Pappus, tube radius `RHO`); the oblique rim's own curvature is not corrected.
fn removed_along_rim(b: &ck_topo::body::Body, cap: &str, side: &str, r: f64) -> f64 {
    let id = |s: &str| b.face_ids().find(|&f| b.face_id_str(f) == s).unwrap();
    let (fc, fs) = (id(cap), id(side));
    let e = b.edge_ids().find(|&e| {
        let fl = b.edge_faces(e);
        fl.contains(&fc) && fl.contains(&fs)
    }).unwrap();
    let ed = b.edge(e);
    let n = 720;
    let mut total = 0.0;
    for i in 0..n {
        let t = ed.t0 + (ed.t1 - ed.t0) * (i as f64 + 0.5) / n as f64;
        let ds = ed.curve.deriv(t).len() * (ed.t1 - ed.t0) / n as f64;
        let p = ed.curve.eval(t);
        let nrm = |f| {
            let (u, v) = b.face(f).surface.closest_param(p);
            b.outward_normal(f, u, v)
        };
        let alpha = std::f64::consts::PI - nrm(fc).dot(nrm(fs)).clamp(-1.0, 1.0).acos();
        total += corner_section(alpha, r) * (1.0 - corner_centroid_depth(alpha, r) / RHO) * ds;
    }
    total
}

/// Builds: no refusal, a valid closed single solid, one blend face per rim, the volume removed within a few percent of the
/// corner section of each rim integrated along it (the rim's wedge angle, which varies round an oblique tube).
fn builds(doc: &str, r: f64, rims: &[Rim]) {
    let fid = if doc.contains("\"id\": \"F9\"") { "F9.B" } else { "F5.B" };
    let (base_errs, base, _) = run(&doc.replace("\"type\": \"fillet\"", "\"type\": \"fillet\", \"suppressed\": true"), None);
    assert!(base_errs.is_empty(), "{base_errs:?}");
    let base = base.unwrap();
    let before = volume(&base);
    let (errs, body, secs) = run(doc, Some(r));
    assert!(errs.is_empty(), "r {r}: {errs:?}");
    let b = body.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    assert!(issues.is_empty(), "r {r}: {:?}", issues.iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect::<Vec<_>>());
    let blends = b.face_ids().filter(|&f| b.face_id_str(f).starts_with(fid)).count();
    // (a rim's ring tool is one sweep, or two where it is built as halves: one or two blend faces per rim)
    assert!(blends == rims.len() || blends == 2 * rims.len(), "r {r}: {blends} blend faces");
    assert_eq!(b.lump_ids().count(), 1, "r {r}: one solid");
    let want: f64 = rims.iter().map(|(c, s, _)| removed_along_rim(&base, c, s, r)).sum();
    let got = before - volume(&b);
    let tol = 0.07;
    assert!((got - want).abs() <= tol * want, "r {r}: removed {got}, analytic {want}");
    eprintln!("r {r}: {} rims, {secs:.2} s, removed {got:.5} vs analytic {want:.5}", rims.len());
    assert!(secs < 20.0, "r {r}: {secs:.1} s");
    // the display mesh: no face fails, quickly (the closed band of a rim round used to hang the triangulation for minutes)
    let t = std::time::Instant::now();
    let tess = ck_tess::mesh::tessellate(&b);
    assert!(tess.mesh.failed.is_empty(), "r {r}: mesh failed faces {:?} {:?}", tess.mesh.failed, tess.mesh.failed_why);
    assert!(t.elapsed().as_secs_f64() < 10.0, "r {r}: mesh took {:.1} s", t.elapsed().as_secs_f64());
}

#[test]
fn both_rims_of_a_plain_swept_tube_round() {
    for r in [0.1, 0.2, 0.5, 0.8, 1.0] {
        builds(TORUS, r, &TORUS_RIMS);
    }
}

#[test]
fn one_rim_of_a_plain_swept_tube_rounds() {
    for r in [0.2, 0.5] {
        builds(TORUS_ONE, r, &TORUS_RIMS[..1]);
    }
}

#[test]
fn rounding_a_sweep_surface_tube_twice_gives_the_same_part() {
    let a = volume(&run(SWEEP_SURFACE, Some(0.3)).1.unwrap());
    let b = volume(&run(SWEEP_SURFACE, Some(0.3)).1.unwrap());
    assert_eq!(a.to_bits(), b.to_bits());
}

#[test]
fn rounding_a_plain_swept_tube_twice_gives_the_same_part() {
    let a = volume(&run(TORUS, Some(0.3)).1.unwrap());
    let b = volume(&run(TORUS, Some(0.3)).1.unwrap());
    assert_eq!(a.to_bits(), b.to_bits());
}

#[test]
fn both_rims_of_a_sweep_surface_tube_round() {
    for r in [0.1, 0.2, 0.5, 1.0] {
        builds(SWEEP_SURFACE, r, &SWEEP_RIMS);
    }
}

#[test]
fn one_rim_of_a_sweep_surface_tube_rounds() {
    for r in [0.2, 0.5] {
        builds(SWEEP_SURFACE_ONE, r, &SWEEP_RIMS[..1]);
    }
}


/// Largest angle (degrees) between the shipped vertex normals at the two ends of a mesh edge, over the blend faces of the part
/// (the metric of `blend_mesh_quality`: what the shader interpolates over, a lump or a crease on a round).
fn blend_normal_jump(b: &ck_topo::body::Body, fid: &str) -> f64 {
    let m = ck_tess::mesh::tessellate(b).mesh;
    let ids: Vec<_> = b.face_ids().collect();
    let mut worst = 0.0f64;
    for (t, &fi) in m.tri_face.iter().enumerate() {
        if !b.face_id_str(ids[fi as usize]).starts_with(fid) {
            continue;
        }
        let nv = |k: usize| [m.normals[9 * t + 3 * k] as f64, m.normals[9 * t + 3 * k + 1] as f64, m.normals[9 * t + 3 * k + 2] as f64];
        for (x, y) in [(0, 1), (1, 2), (2, 0)] {
            let (a, c) = (nv(x), nv(y));
            let (la, lc) = ((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt(), (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt());
            if la > 0.0 && lc > 0.0 {
                worst = worst.max(((a[0] * c[0] + a[1] * c[1] + a[2] * c[2]) / (la * lc)).clamp(-1.0, 1.0).acos().to_degrees());
            }
        }
    }
    worst
}

/// `builds`, and the blend faces' mesh within the normal-jump bound of `blend_mesh_quality` (35 deg for a blend face).
fn builds_smooth(doc: &str, r: f64, rims: &[Rim]) {
    builds(doc, r, rims);
    let fid = if doc.contains("\"id\": \"F9\"") { "F9.B" } else { "F5.B" };
    let b = run(doc, Some(r)).1.unwrap();
    let jump = blend_normal_jump(&b, fid);
    eprintln!("r {r}: blend normal jump {jump:.1} deg");
    assert!(jump <= 35.0, "r {r}: blend normal jump {jump:.1} deg");
}

/// Radii the planes normal to the rim cannot march (the oblique tube's ball centre passes 0.14 mm from the rim circle's axis at
/// 1.4 and the axis is outside the centre curve from 1.5): the ring is marched along the ball centre's own curve.
#[test]
fn both_rims_of_a_sweep_surface_tube_round_at_large_radii() {
    for r in [1.4, 1.6, 1.8] {
        builds_smooth(SWEEP_SURFACE, r, &SWEEP_RIMS);
    }
}

#[test]
fn one_rim_of_a_sweep_surface_tube_rounds_at_large_radii() {
    for r in [1.4, 1.6, 1.8] {
        builds_smooth(SWEEP_SURFACE_ONE, r, &SWEEP_RIMS[..1]);
    }
}

/// The rim planes still march the radii they always did (r 1.0 is today's path), and the torus tube is unchanged.
#[test]
fn radii_the_rim_planes_march_still_build() {
    builds_smooth(SWEEP_SURFACE, 1.0, &SWEEP_RIMS);
    builds_smooth(SWEEP_SURFACE_ONE, 1.0, &SWEEP_RIMS[..1]);
    builds_smooth(TORUS, 1.0, &TORUS_RIMS);
}

#[test]
fn rounding_a_sweep_surface_tube_at_a_large_radius_twice_gives_the_same_part() {
    let a = volume(&run(SWEEP_SURFACE, Some(1.6)).1.unwrap());
    let b = volume(&run(SWEEP_SURFACE, Some(1.6)).1.unwrap());
    assert_eq!(a.to_bits(), b.to_bits());
}

/// Past what the rim allows the round is refused at once (the ring march fails), naming the limit, not after the seconds a
/// size search of failing builds used to take.
#[test]
fn a_radius_beyond_the_rim_is_refused_fast_with_the_limit() {
    for (doc, r) in [(SWEEP_SURFACE_ONE, LIMIT_PROBE), (SWEEP_SURFACE, LIMIT_PROBE)] {
        let (errs, body, secs) = run(doc, Some(r));
        eprintln!("r {r}: {secs:.2} s, {errs:?}");
        assert!(body.is_none() || !errs.is_empty(), "r {r} built");
        assert!(errs.iter().any(|e| e.2.contains("radius exceeds what the rim allows")), "r {r}: {errs:?}");
        // wall clock on a shared, loaded machine: 6 to 9 s were seen there for what takes 1 to 3.5 s idle; the bound only has to
        // catch a return of the unbounded size search (minutes), the message above proves the limit path was taken
        assert!(secs < 60.0, "r {r}: refused after {secs:.1} s");
    }
}

const LIMIT_PROBE: f64 = 2.4;
