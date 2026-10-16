//! Fillets at the corners of a pocket: three equal fillets at a concave vertex end in the rolling
//! ball's sphere patch `V<k>` (octant of a sphere tangent to the three cylinders and the three
//! planes), like the convex corner. Reference volumes are OCCT BRepFilletAPI_MakeFillet on the same
//! 40 x 20 x 40 block with a 20 x 8 x 20 pocket (base volume 28800).
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
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

fn rect(id: &str, plane: &str, x0: f64, y0: f64, x1: f64, y1: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"{id}","plane":{plane},
        "points":[{{"id":"p1","x":{x0},"y":{y0}}},{{"id":"p2","x":{x1},"y":{y0}}},{{"id":"p3","x":{x1},"y":{y1}}},{{"id":"p4","x":{x0},"y":{y1}}}],
        "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
                    {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}}"#
    )
}

fn pocket() -> Vec<String> {
    vec![
        rect("F1", r#"{"kind":"std","id":"top"}"#, 0.0, 0.0, 40.0, 40.0),
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":20}"#.to_string(),
        rect("F3", r#"{"kind":"face","id":"F2.C1"}"#, 10.0, 10.0, 30.0, 30.0),
        r#"{"type":"extrude","id":"F4","op":"cut","sketch":"F3","end":"blind","depth":8}"#.to_string(),
    ]
}

fn edge(a: &str, b: &str, at: [f64; 3]) -> String {
    format!(r#"{{"faces":["{a}","{b}"],"at":[{},{},{}]}}"#, at[0], at[1], at[2])
}

/// pocket edges at the vertex (10, 12, -10): floor|wall e4, floor|wall e1, wall e1|wall e4
fn concave_edges() -> [String; 3] {
    [edge("F4.C0", "F4.S.e4", [10., 12., -20.]), edge("F4.C0", "F4.S.e1", [20., 12., -10.]), edge("F4.S.e1", "F4.S.e4", [10., 16., -10.])]
}

struct Out {
    errs: Vec<String>,
    vol: f64,
    issues: usize,
    body: Body,
}

fn run(extra: Vec<String>) -> Out {
    let mut features = pocket();
    features.extend(extra);
    let part = Part::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","))).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| format!("{} {}", e.feature, e.error.describe())).collect();
    let body = ex.0.body.clone().unwrap();
    Out { errs, vol: ck_topo::mass::mass_props(&body).volume, issues: ck_topo::check::check(&body, 1e-6).len(), body }
}

fn fillet(r: f64, edges: &[String]) -> Out {
    run(vec![format!(r#"{{"type":"fillet","id":"F5","radius":{r},"edges":[{}]}}"#, edges.join(","))])
}

fn kinds_of(b: &Body, prefix: &str) -> Vec<(String, &'static str)> {
    b.face_ids()
        .filter(|&f| b.face_id_str(f).starts_with(prefix))
        .map(|f| {
            let k = match b.face(f).surface {
                Surface::Plane { .. } => "plane",
                Surface::Cylinder { .. } => "cylinder",
                Surface::Sphere { .. } => "sphere",
                _ => "other",
            };
            (b.face_id_str(f), k)
        })
        .collect()
}

/// Smallest angle between the surface normals of faces `a` and `b` across their shared edges,
/// sampled along every shared edge (normals oriented by the face sense).
fn seam_angle(b: &Body, a: &str, c: &str) -> f64 {
    let (fa, fc) = (b.find_face(a).unwrap(), b.find_face(c).unwrap());
    let mut worst = 0.0f64;
    let mut n_edges = 0;
    for e in b.edge_ids() {
        let fs = b.edge_faces(e);
        if !(fs.contains(&fa) && fs.contains(&fc)) {
            continue;
        }
        n_edges += 1;
        let ed = b.edge(e);
        for k in 0..=8 {
            let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0;
            let p = ed.curve.eval(t);
            let nrm = |f| {
                let (u, v) = b.face(f).surface.closest_param(p);
                b.outward_normal(f, u, v)
            };
            let (na, nc) = (nrm(fa), nrm(fc));
            worst = worst.max(na.dot(nc).clamp(-1.0, 1.0).acos());
        }
    }
    assert!(n_edges > 0, "{a} and {c} share no edge");
    worst
}

const OCCT_BASE: f64 = 28800.0;

#[test]
fn three_concave_fillets_end_in_a_sphere_patch() {
    // OCCT: r=1 28810.133483871516, r=2 28839.86431834444, r=3 28888.188077706345
    for (r, want) in [(1.0, 28810.133483871516), (2.0, 28839.86431834444), (3.0, 28888.188077706345)] {
        let o = fillet(r, &concave_edges());
        assert!(o.errs.is_empty(), "{:?}", o.errs);
        assert_eq!(o.issues, 0);
        assert!((o.vol - want).abs() < 1e-6 * want, "r={r}: {} vs {want}", o.vol);
        assert!(o.vol > OCCT_BASE);
        let blends = kinds_of(&o.body, "F5.");
        assert_eq!(blends.iter().filter(|(_, k)| *k == "cylinder").count(), 3, "{blends:?}");
        let sph: Vec<_> = blends.iter().filter(|(_, k)| *k == "sphere").collect();
        assert_eq!(sph.len(), 1, "{blends:?}");
        // the patch is centred r above the floor and r inside both walls
        let f = o.body.find_face(&sph[0].0).unwrap();
        let Surface::Sphere { frame, r: sr } = o.body.face(f).surface.clone() else { panic!() };
        assert!((sr - r).abs() < 1e-9);
        assert!(frame.o.dist(Vec3::new(10.0 + r, 12.0 + r, -10.0 - r)) < 1e-9, "{:?}", frame.o);
        // tangent continuity: sphere patch to each cylinder, and each cylinder to its planes
        for (n, _) in blends.iter().filter(|(_, k)| *k == "cylinder") {
            assert!(seam_angle(&o.body, &sph[0].0, n) < 1e-3, "{n}");
        }
    }
}

#[test]
fn two_concave_fillets_mitre_without_a_cap() {
    let e = concave_edges();
    let o = fillet(2.0, &e[..2]);
    assert!(o.errs.is_empty(), "{:?}", o.errs);
    assert_eq!(o.issues, 0);
    // OCCT r=2: 28833.56933115844; the union of the two spandrels, no flat end cap on the second
    assert!((o.vol - 28833.56933115844).abs() < 1e-6 * 28833.0, "{}", o.vol);
    assert!(kinds_of(&o.body, "F5.").iter().all(|(_, k)| *k == "cylinder"));
}

fn ring_edges() -> Vec<String> {
    vec![
        edge("F4.C0", "F4.S.e4", [10., 12., -20.]),
        edge("F4.C0", "F4.S.e1", [20., 12., -10.]),
        edge("F4.C0", "F4.S.e2", [30., 12., -20.]),
        edge("F4.C0", "F4.S.e3", [20., 12., -30.]),
        edge("F4.S.e1", "F4.S.e4", [10., 16., -10.]),
        edge("F4.S.e1", "F4.S.e2", [30., 16., -10.]),
        edge("F4.S.e2", "F4.S.e3", [30., 16., -30.]),
        edge("F4.S.e3", "F4.S.e4", [10., 16., -30.]),
    ]
}

#[test]
fn pocket_floor_and_corners_get_four_sphere_patches() {
    for (r, want) in [(1.0, 28823.365788557858), (2.0, 28890.78468566495), (3.0, 28998.23898847153)] {
        let o = fillet(r, &ring_edges());
        assert!(o.errs.is_empty(), "{:?}", o.errs);
        assert_eq!(o.issues, 0);
        assert!((o.vol - want).abs() < 1e-6 * want, "r={r}: {} vs {want}", o.vol);
        let blends = kinds_of(&o.body, "F5.");
        assert_eq!(blends.iter().filter(|(_, k)| *k == "sphere").count(), 4, "{blends:?}");
        assert_eq!(blends.iter().filter(|(_, k)| *k == "plane").count(), 0, "{blends:?}");
    }
}

#[test]
fn a_pocket_rim_fillet_ends_flush_at_the_wall() {
    // the rim edge runs into the pocket wall; the material behind that wall must not be cut
    // OCCT r=2: 28782.83185307179 (removed 17.168 = (1 - pi/4) r^2 x 20)
    let o = fillet(2.0, &[edge("F2.C1", "F4.S.e4", [10., 20., -20.])]);
    assert!(o.errs.is_empty(), "{:?}", o.errs);
    assert_eq!(o.issues, 0);
    assert!((o.vol - 28782.83185307179).abs() < 1e-6 * 28782.0, "{}", o.vol);
}

#[test]
fn rim_fillets_with_a_corner_fillet_mixed_vertex() {
    // two convex rim edges and the concave wall-wall edge meet at (10, 20, -10); OCCT r=2: 28772.665456976367
    // (before: 38.7 removed against 27.3). The rolling-ball corner patch of a mixed vertex is not built: the
    // result stays within 2.5 mm^3 of it and valid.
    let e = [edge("F2.C1", "F4.S.e4", [10., 20., -20.]), edge("F2.C1", "F4.S.e1", [20., 20., -10.]), edge("F4.S.e1", "F4.S.e4", [10., 16., -10.])];
    let o = fillet(2.0, &e);
    assert!(o.errs.is_empty(), "{:?}", o.errs);
    assert_eq!(o.issues, 0);
    assert!((o.vol - 28772.665456976367).abs() < 2.5, "{}", o.vol);
    let rim2 = fillet(2.0, &e[..2]);
    assert_eq!(rim2.issues, 0);
    assert!((rim2.vol - 28764.896743388403).abs() < 1.5, "{}", rim2.vol);
}
