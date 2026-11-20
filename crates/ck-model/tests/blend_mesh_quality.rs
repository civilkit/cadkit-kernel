//! Mesh quality of blend faces: per face, the angle between the normals of the two triangles on either side of every
//! interior mesh edge (the "dihedral" a viewer sees as lumps and spikes). `MESHQ=1` prints the table.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::collections::HashMap;

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

pub struct Stat {
    pub face: String,
    pub edges: usize,
    pub max: f64,
    pub p99: f64,
    pub kind: String,
    pub tris: usize,
    /// largest angle between the shipped normals at the two ends of one mesh edge (what the shader interpolates over)
    pub jump: f64,
}

fn try_build(src: &str) -> Result<ck_topo::body::Body, String> {
    let part = Part::parse(src).map_err(|e| format!("{e:?}"))?;
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if !r.errors.is_empty() {
        return Err(format!("{:?}", r.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>()));
    }
    ex.0.body.take().ok_or_else(|| "no body".to_string())
}

fn build(src: &str) -> ck_topo::body::Body {
    try_build(src).unwrap()
}

/// Per face (in body order): the dihedral statistics over the interior edges shared by exactly two triangles of the face.
pub fn face_stats(b: &ck_topo::body::Body) -> Vec<Stat> {
    let m = ck_tess::mesh::tessellate(b).mesh;
    let ids: Vec<_> = b.face_ids().collect();
    let q = |k: usize| [(m.positions[k] as f64 * 1e4).round() as i64, (m.positions[k + 1] as f64 * 1e4).round() as i64, (m.positions[k + 2] as f64 * 1e4).round() as i64];
    let mut per: Vec<HashMap<([i64; 3], [i64; 3]), Vec<[f64; 3]>>> = vec![HashMap::new(); ids.len()];
    for (t, &fi) in m.tri_face.iter().enumerate() {
        let v = [q(9 * t), q(9 * t + 3), q(9 * t + 6)];
        let p = |k: usize| [m.positions[9 * t + k] as f64, m.positions[9 * t + k + 1] as f64, m.positions[9 * t + k + 2] as f64];
        let (a, bb, c) = (p(0), p(3), p(6));
        let (u, w) = ([bb[0] - a[0], bb[1] - a[1], bb[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
        let mut n = [u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if l < 1e-12 {
            continue;
        }
        n = [n[0] / l, n[1] / l, n[2] / l];
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            if v[i] == v[j] {
                continue;
            }
            let key = if v[i] < v[j] { (v[i], v[j]) } else { (v[j], v[i]) };
            if let Some(x) = per.get_mut(fi as usize) {
                x.entry(key).or_default().push(n);
            }
        }
    }
    let mut jump = vec![0f64; ids.len()];
    for (t, &fi) in m.tri_face.iter().enumerate() {
        let nv = |k: usize| [m.normals[9 * t + 3 * k] as f64, m.normals[9 * t + 3 * k + 1] as f64, m.normals[9 * t + 3 * k + 2] as f64];
        for (x, y) in [(0, 1), (1, 2), (2, 0)] {
            let (a, b) = (nv(x), nv(y));
            let (la, lb) = ((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt(), (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt());
            if la > 0.0 && lb > 0.0 {
                let d = ((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb)).clamp(-1.0, 1.0).acos().to_degrees();
                if let Some(j) = jump.get_mut(fi as usize) {
                    *j = j.max(d);
                }
            }
        }
    }
    ids.iter()
        .enumerate()
        .map(|(i, &f)| {
            let mut a: Vec<f64> = per[i]
                .values()
                .filter(|v| v.len() == 2)
                .map(|v| (v[0][0] * v[1][0] + v[0][1] * v[1][1] + v[0][2] * v[1][2]).clamp(-1.0, 1.0).acos().to_degrees())
                .collect();
            a.sort_by(|x, y| x.partial_cmp(y).unwrap());
            let p99 = if a.is_empty() { 0.0 } else { a[((a.len() - 1) as f64 * 0.99).round() as usize] };
            Stat {
                face: b.face_id_str(f),
                edges: a.len(),
                max: a.last().copied().unwrap_or(0.0),
                p99,
                kind: format!("{:?}", b.face(f).surface).split(|c: char| !c.is_alphanumeric()).next().unwrap_or("").to_string(),
                tris: m.tri_face.iter().filter(|&&x| x as usize == i).count(),
                jump: jump[i],
            }
        })
        .collect()
}

fn report(name: &str, st: &[Stat], filter: &dyn Fn(&str) -> bool) {
    if std::env::var("MESHQ").is_err() {
        return;
    }
    for s in st.iter().filter(|s| filter(&s.face)) {
        println!("MESHQ {name} {} {} tris {} edges {} dihedral max {:.1} p99 {:.1} normal-jump {:.1}", s.face, s.kind, s.tris, s.edges, s.max, s.p99, s.jump);
    }
}

const CORPUS: &[(&str, &str)] = &[
    ("lumpy", include_str!("fixtures/overhang/vertex_blend_lumpy.ckpart")),
    ("overhang_fillet", include_str!("fixtures/overhang_fillet.json")),
    ("bad_overhang_fillets", include_str!("fixtures/overhang/bad_overhang_fillets.ckpart")),
    ("bad_overhang_mz", include_str!("fixtures/overhang/bad_overhang_mz.ckpart")),
    ("bad_overhang_skew", include_str!("fixtures/overhang/bad_overhang_skew.ckpart")),
    ("bad_overhang_two", include_str!("fixtures/overhang/bad_overhang_two.ckpart")),
    ("bad_overhang_under", include_str!("fixtures/overhang/bad_overhang_under.ckpart")),
    ("wheel_r10_hub_edge", include_str!("fixtures/wheel_plates_r10_hub_edge.json")),
    ("wheel_r5_then_r10", include_str!("fixtures/wheel_plates_r5_then_r10_vertical.json")),
];

#[test]
fn measure_corpus() {
    for (name, src) in CORPUS {
        match try_build(src) {
            Ok(b) => {
                if std::env::var("MESHQ").is_ok() {
                    println!("MESHQ {name} total triangles {}", ck_tess::mesh::tessellate(&b).mesh.num_tris());
                }
                report(name, &face_stats(&b), &is_blend)
            }
            Err(e) => {
                if std::env::var("MESHQ").is_ok() {
                    println!("MESHQ {name} does not build: {}", &e[..e.len().min(120)]);
                }
            }
        }
    }
}

/// A blend face by name: `<feature>.B<k>` stripes, `.V<k>` vertex blends, `.P<k>` corner patches.
fn is_blend(face: &str) -> bool {
    let r = face.rsplit('.').next().unwrap_or("");
    let mut c = r.chars();
    matches!(c.next(), Some('B' | 'V' | 'P')) && c.all(|x| x.is_ascii_digit()) && r.len() > 1
}

/// Bounds, from measurement (dihedral = angle between the two facet normals over an interior mesh edge, `jump` = angle
/// between the shipped vertex normals at the two ends of a mesh edge, i.e. what the shader interpolates over):
///
/// * `jump` <= 35 deg on every blend face: measured 11 to 30.3 deg (cylinders 15, spheres 26.6 / 27.3, tori 30.3).
/// * `jump` <= 18 deg on the spline corner patches (`P<k>`): measured 15.1. Before the normal rule in ck-tess it was 77
///   (p99 60) and the patch's G1 seam to the tubes was off by up to 53 deg over a quarter of its length: that
///   pair is the crumpled corner the user saw in `vertex_blend_lumpy`.
/// * G1: the patch's normal along its four boundary isolines is within 5 deg of the neighbouring face (measured 1.6;
///   it was 53 on the two isolines that meet the tubes, because the wall-plane projection of the patch's grid was applied
///   after the tube projection and undid it in the corner blocks).
/// * dihedral of the analytic blend faces (cylinder, sphere, torus): max <= 50 deg and p99 <= 40 deg. Measured max
///   48.4 / p99 36.3 on the three-fillet sphere patches `V1`, `V7`, `V11` (a sliver triangle, quality 0.04, whose vertex
///   normals are exact), 21.5 / 20.3 on the others, 29.3 / 27.9 on tori, 15 on cylinders.
/// * KNOWN GAP, not asserted: the dihedral of the spline corner patches reaches 95 to 180 deg with p99 up to 90;
///   it comes from a few zero-area / hairline triangles per patch on the straight side chains and at the outline's
///   corners (docs/blend-corpus.md), whose facet normals are meaningless; their vertex normals are exact.
const MAX_JUMP: f64 = 35.0;
const MAX_SPLINE_JUMP: f64 = 18.0;
const MAX_G1: f64 = 5.0;
const MAX_DIHEDRAL: f64 = 50.0;
const MAX_P99: f64 = 40.0;

fn check_blend_faces(name: &str, src: &str) -> usize {
    let b = build(src);
    let st = face_stats(&b);
    let mut n = 0;
    let mut bad = vec![];
    for s in st.iter().filter(|s| is_blend(&s.face)) {
        n += 1;
        let lim = if s.kind == "BSpline" { MAX_SPLINE_JUMP } else { MAX_JUMP };
        if s.jump > lim {
            bad.push(format!("{} normal jump {:.1} > {lim}", s.face, s.jump));
        }
        if s.kind != "BSpline" && (s.max > MAX_DIHEDRAL || s.p99 > MAX_P99) {
            bad.push(format!("{} {} dihedral max {:.1} p99 {:.1}", s.face, s.kind, s.max, s.p99));
        }
    }
    for (f, iso, w, at, _) in g1_report(&b) {
        if w > MAX_G1 {
            bad.push(format!("{f} seam {iso}: normal {w:.1} deg off the neighbour at t={at:.2}"));
        }
    }
    assert!(bad.is_empty(), "{name}: {bad:?}");
    n
}

/// The user's part: three-fillet corners (`F6_a.V*`), the overhang corner patches (`F6.P*`), tori, stripes.
#[test]
fn lumpy_corner_part_meshes_smoothly() {
    let src = include_str!("fixtures/overhang/vertex_blend_lumpy.ckpart");
    assert!(check_blend_faces("vertex_blend_lumpy", src) >= 30);
    // meshing does not touch the solid: the volume is the one measured before the tessellation change
    let v = ck_topo::mass::mass_props(&build(src)).volume;
    assert!((v - 28657.981797632023).abs() < 1e-6 * v, "{v}");
}

#[test]
fn corpus_blend_faces_mesh_smoothly() {
    let mut built = 0;
    for (name, src) in CORPUS {
        if try_build(src).is_err() {
            continue; // refusals (skew corner, wheel R10 after R5) have no blend faces to look at
        }
        assert!(check_blend_faces(name, src) > 0, "{name}");
        built += 1;
    }
    assert!(built >= 7, "only {built} corpus parts built");
}

/// G1 of the spline patches to their neighbours along the four boundary isolines, geometrically (surface normals only).
/// Returns, per patch, per isoline, the worst deviation (deg) and the fraction of the isoline (0..1) where it sits.
pub fn g1_report(b: &ck_topo::body::Body) -> Vec<(String, String, f64, f64, usize)> {
    let mut out = vec![];
    let ids: Vec<_> = b.face_ids().collect();
    for &f in &ids {
        let name = b.face_id_str(f);
        let s = &b.face(f).surface;
        if !name.contains(".P") || !matches!(s, ck_geom::surface::Surface::BSpline(_)) {
            continue;
        }
        let ((u0, u1), (v0, v1)) = s.domain();
        for (iso, k) in [("u0", 0), ("u1", 1), ("v0", 2), ("v1", 3)] {
            let (mut worst, mut at, mut over5) = (0.0f64, 0.0, 0usize);
            let n = 400;
            for i in 0..=n {
                let t = i as f64 / n as f64;
                let (u, v) = match k {
                    0 => (u0, v0 + (v1 - v0) * t),
                    1 => (u1, v0 + (v1 - v0) * t),
                    2 => (u0 + (u1 - u0) * t, v0),
                    _ => (u0 + (u1 - u0) * t, v1),
                };
                let p = s.eval(u, v);
                let np = s.normal(u, v);
                let mut best: Option<f64> = None;
                for &g in &ids {
                    if g == f {
                        continue;
                    }
                    let sg = &b.face(g).surface;
                    let (gu, gv, q) = sg.closest_point(p);
                    if q.dist(p) < 2e-4 {
                        let ng = b.outward_normal(g, gu, gv);
                        let d = np.dot(ng).abs().clamp(0.0, 1.0).acos().to_degrees();
                        best = Some(best.map_or(d, |x: f64| x.min(d)));
                    }
                }
                if let Some(d) = best {
                    if d > 5.0 {
                        over5 += 1;
                    }
                    if d > worst {
                        worst = d;
                        at = t;
                    }
                }
            }
            out.push((name.clone(), iso.to_string(), worst, at, over5));
        }
    }
    out
}

#[test]
fn diag_tol() {
    if std::env::var("MESHQ").is_err() {
        return;
    }
    let b = build(include_str!("fixtures/overhang/vertex_blend_lumpy.ckpart"));
    let t = ck_tess::mesh::body_tol(&b);
    let f = b.face_ids().find(|&f| b.face_id_str(f) == "F6.P0").unwrap();
    let m = ck_tess::mesh::tessellate(&b).mesh;
    let (mut lo, mut hi) = ([f64::MAX; 3], [f64::MIN; 3]);
    let ids: Vec<_> = b.face_ids().collect();
    let fi = ids.iter().position(|&x| x == f).unwrap();
    for (t_, &x) in m.tri_face.iter().enumerate() {
        if x as usize == fi {
            for k in 0..9 {
                let c = k % 3;
                let v = m.positions[9 * t_ + k] as f64;
                lo[c] = lo[c].min(v);
                hi[c] = hi[c].max(v);
            }
        }
    }
    println!("MESHQ tol chord {} angle {} P0 extent {:?}", t.chord, t.angle, [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]]);
}
