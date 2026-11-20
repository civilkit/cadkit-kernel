//! A fillet of "all connected" edges on two stacked blocks (block B sits on A's top face and overhangs A's
//! walls). Two 4-valent corners (V1 at x=21.5,z=0 and V2 at x=0,z=16.7, both on the y=10 plane) each have
//! four selected edges: two collinear pairs that flip convex/concave. Fixture: fixtures/overhang_fillet.json.
//! `SWEEP=1` prints a table of every subset at several radii.
use ck_check::tessellation::check_mesh_why;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;

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

const FIXTURE: &str = include_str!("fixtures/overhang_fillet.json");

/// edge picks of the fixture, in its order: V2 = 0..3, V1 = 10..13, the rest are the other 19 edges
const V1: [usize; 4] = [10, 11, 12, 13]; // A top|B wall z=0 (concave), A top|A wall x=21.5 (convex), A wall|B under (concave), B under|B wall (convex)
const V2: [usize; 4] = [3, 2, 1, 0]; // A top|B wall x=0 (concave), A top|A wall z=16.7 (convex), A wall|B under (concave), B under|B wall (convex)
/// the mixed vertex under the overhang at (21.5, 10, 16.7): A wall|B under along x (1, concave), A wall|B under along z (12, concave) and the
/// vertical A corner edge (18, convex). Edge 12 is also an edge of V1, edge 1 one of V2.
const MIXED: [usize; 3] = [1, 12, 18];

/// the picks of the fixture's fillet, as text
fn picks() -> Vec<String> {
    let i = FIXTURE.find("\"edges\"").unwrap();
    let a = i + FIXTURE[i..].find('[').unwrap();
    let mut depth = 0;
    let mut out = vec![];
    let mut start = 0;
    for (k, c) in FIXTURE[a..].char_indices() {
        match c {
            '{' => {
                if depth == 1 {
                    start = a + k;
                }
                depth += 1
            }
            '}' => {
                depth -= 1;
                if depth == 1 {
                    out.push(FIXTURE[start..a + k + 1].to_string());
                }
            }
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            _ => {}
        }
    }
    out
}

/// the part with the fillet's picks replaced by `which` (indices into the fixture's picks), radius `r`, `propagate` flag
fn part(which: &[usize], r: f64, propagate: Option<bool>) -> String {
    part_k("fillet", which, r, propagate)
}

/// `kind`: "fillet", "chamfer" (equal distance `r`), "chamfer2" (distances `r` and `2.5 r`) or "chamferA" (`r` and 40 degrees)
fn part_k(kind: &str, which: &[usize], r: f64, propagate: Option<bool>) -> String {
    let all = picks();
    let sel: Vec<&str> = which.iter().map(|&i| all[i].as_str()).collect();
    let i = FIXTURE.find("\"edges\"").unwrap();
    let a = i + FIXTURE[i..].find('[').unwrap();
    // end of the edges array: first ']' at depth 0 (picks hold nested arrays)
    let mut depth = 0;
    let mut end = a;
    for (k, c) in FIXTURE[a..].char_indices() {
        match c {
            '[' => depth += 1,
            ']' => {
                depth -= 1;
                if depth == 0 {
                    end = a + k + 1;
                    break;
                }
            }
            _ => {}
        }
    }
    let mut s = format!("{}[{}]{}", &FIXTURE[..a], sel.join(","), &FIXTURE[end..]);
    let size = match kind {
        "fillet" => format!("\"radius\": {r},"),
        "chamfer" => format!("\"chamferType\": \"equal\", \"distance\": {r},"),
        "chamfer2" => format!("\"chamferType\": \"distanceDistance\", \"distance\": {r}, \"distance2\": {},", r * 2.5),
        "chamfer2F" => format!("\"chamferType\": \"distanceDistance\", \"face\": \"F2.C1\", \"distance\": {r}, \"distance2\": {},", r * 2.5),
        _ => format!("\"chamferType\": \"angleDistance\", \"distance\": {r}, \"angle\": 40,"),
    };
    s = s.replacen("\"radius\": 1,", &size, 1);
    if kind != "fillet" {
        s = s.replacen("\"type\": \"fillet\",", "\"type\": \"chamfer\",", 1);
    }
    if let Some(p) = propagate {
        let t = if kind == "fillet" { "fillet" } else { "chamfer" };
        s = s.replacen(&format!("\"type\": \"{t}\","), &format!("\"type\": \"{t}\", \"propagate\": {p},"), 1);
    }
    s
}

pub struct Res {
    pub errs: Vec<String>,
    pub body: Option<Body>,
    pub issues: Vec<String>,
    pub vol: f64,
}

/// regen; `sharp` = without the fillet. Issues: ck_topo::check, ck_check::validate, mesh check.
fn run(src: &str, sharp: bool) -> Res {
    let mut p = Part::parse(src).unwrap();
    if sharp {
        p.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&p, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{} {} {:?}", e.feature, e.error.code, e.error.params)).collect();
    let mut issues = vec![];
    let mut vol = 0.0;
    if errs.is_empty() {
        let b = ex.0.body.as_ref().unwrap();
        vol = ck_topo::mass::mass_props(b).volume;
        for i in ck_check::validate(b, 1e-6) {
            issues.push(format!("validate: {i:?}"));
        }
        for i in ck_topo::check::check(b, 1e-6) {
            issues.push(format!("check: {i:?}"));
        }
        let m = ck_tess::mesh::tessellate(b).mesh;
        for i in check_mesh_why(b, &m.positions, &m.tri_face, &m.failed, &m.failed_why) {
            issues.push(format!("mesh {}: {}", i.entity, i.detail));
        }
    }
    Res { errs, body: ex.0.body.clone(), issues, vol }
}

/// Smoothness of the tessellation: every pair of triangles sharing an edge (positions welded) where both
/// are off the planes (blend stripes, corner patches; a blend's end against a wall is a true 90 degree edge) must meet at a small angle; returns the worst angle in degrees
/// with the two faces and the place, and writes `$DUMP.obj` when DUMP is set.
pub fn worst_fold(b: &Body, tag: &str) -> (f64, String) {
    use std::collections::HashMap;
    let m = ck_tess::mesh::tessellate(b).mesh;
    let ids: Vec<_> = b.face_ids().collect();
    let n = m.tri_face.len();
    let p = |t: usize, k: usize| [m.positions[9 * t + 3 * k] as f64, m.positions[9 * t + 3 * k + 1] as f64, m.positions[9 * t + 3 * k + 2] as f64];
    let key = |q: [f64; 3]| [(q[0] * 2e3).round() as i64, (q[1] * 2e3).round() as i64, (q[2] * 2e3).round() as i64];
    let nrm = |t: usize| {
        let (a, bb, c) = (p(t, 0), p(t, 1), p(t, 2));
        let u = [bb[0] - a[0], bb[1] - a[1], bb[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let w = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let l = (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]).sqrt();
        if l < 1e-12 { [0.0; 3] } else { [w[0] / l, w[1] / l, w[2] / l] }
    };
    let mut emap: HashMap<([i64; 3], [i64; 3]), Vec<usize>> = HashMap::new();
    for t in 0..n {
        for k in 0..3 {
            let (a, c) = (key(p(t, k)), key(p(t, (k + 1) % 3)));
            emap.entry(if a < c { (a, c) } else { (c, a) }).or_default().push(t);
        }
    }
    let planar = |t: usize| matches!(b.face(ids[m.tri_face[t] as usize]).surface, ck_geom::surface::Surface::Plane { .. });
    let mut worst = (0.0f64, String::new());
    for ((a, _), ts) in &emap {
        if ts.len() != 2 || planar(ts[0]) || planar(ts[1]) {
            continue;
        }
        let (n0, n1) = (nrm(ts[0]), nrm(ts[1]));
        let d = (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2]).clamp(-1.0, 1.0).acos().to_degrees();
        if d > worst.0 {
            worst = (d, format!("{} | {} near ({:.2},{:.2},{:.2})", b.face_id_str(ids[m.tri_face[ts[0]] as usize]), b.face_id_str(ids[m.tri_face[ts[1]] as usize]), a[0] as f64 / 2e3, a[1] as f64 / 2e3, a[2] as f64 / 2e3));
        }
    }
    if let Ok(d) = std::env::var("DUMP") {
        std::fs::create_dir_all(&d).ok();
        let mut s = String::new();
        for t in 0..n {
            for k in 0..3 {
                let q = p(t, k);
                s += &format!("v {} {} {}\n", q[0], q[1], q[2]);
            }
            s += &format!("f {} {} {} # {}\n", 3 * t + 1, 3 * t + 2, 3 * t + 3, b.face_id_str(ids[m.tri_face[t] as usize]));
        }
        std::fs::write(format!("{d}/{tag}.obj"), s).ok();
    }
    worst
}

fn label(which: &[usize]) -> String {
    which.iter().map(|i| i.to_string()).collect::<Vec<_>>().join(",")
}

/// Triangles of non-planar faces whose geometric normal disagrees with their vertex normals (folded or flipped
/// triangles of real size: slivers under 1e-3 mm^2, which the tessellator leaves along trim curves, are ignored).
pub fn flipped(b: &Body, min_area: f64) -> Vec<String> {
    let m = ck_tess::mesh::tessellate(b).mesh;
    let ids: Vec<_> = b.face_ids().collect();
    let mut out = vec![];
    for t in 0..m.tri_face.len() {
        let f = ids[m.tri_face[t] as usize];
        if matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }) {
            continue;
        }
        let q = |k: usize| ck_geom::Vec3::new(m.positions[9 * t + 3 * k] as f64, m.positions[9 * t + 3 * k + 1] as f64, m.positions[9 * t + 3 * k + 2] as f64);
        let vn = ck_geom::Vec3::new(m.normals[9 * t] as f64, m.normals[9 * t + 1] as f64, m.normals[9 * t + 2] as f64);
        let g = (q(1) - q(0)).cross(q(2) - q(0));
        if 0.5 * g.len() > min_area && g.normalize().dot(vn) < 0.2 {
            out.push(format!("{} ({:.2},{:.2},{:.2}) area {:.3}", b.face_id_str(f), q(0).x, q(0).y, q(0).z, 0.5 * g.len()));
        }
    }
    out
}

/// Dense-grid analysis of every B-spline face: the worst angle between neighbouring cell normals (degrees) and
/// the smallest cell area relative to the mean. Geometric, no tessellator.
pub fn patch_report(b: &Body) -> Vec<(String, f64, f64, f64, f64)> {
    let mut out = vec![];
    for f in b.face_ids() {
        let ck_geom::surface::Surface::BSpline(bs) = &b.face(f).surface else { continue };
        let n = 120usize;
        let g: Vec<Vec<ck_geom::Vec3>> = (0..=n).map(|i| (0..=n).map(|j| bs.eval(i as f64 / n as f64, j as f64 / n as f64)).collect()).collect();
        let cell = |i: usize, j: usize| (g[i + 1][j] - g[i][j]).cross(g[i][j + 1] - g[i][j]);
        let mean: f64 = (0..n).flat_map(|i| (0..n).map(move |j| (i, j))).map(|(i, j)| cell(i, j).len()).sum::<f64>() / (n * n) as f64;
        let (mut worst, mut minarea) = (0.0f64, f64::INFINITY);
        let mut at = (0.0, 0.0);
        for i in 0..n {
            for j in 0..n {
                let c = cell(i, j);
                minarea = minarea.min(c.len() / mean);
                for (a, bb) in [(i + 1, j), (i, j + 1)] {
                    if a < n && bb < n {
                        let d = c.normalize().dot(cell(a, bb).normalize()).clamp(-1.0, 1.0).acos().to_degrees();
                        if d > worst {
                            worst = d;
                            at = (i as f64 / n as f64, j as f64 / n as f64);
                        }
                    }
                }
            }
        }
        out.push((b.face_id_str(f), worst, minarea, at.0, at.1));
    }
    out
}

/// "nP patches, nC step faces, nF folded triangles" of a result
fn shape(b: &Body) -> (usize, usize, usize) {
    let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    let p = names.iter().filter(|n| n.starts_with("F6.P")).count();
    let c = names.iter().filter(|n| n.starts_with("F6.C")).count();
    (p, c, flipped(b, 1e-3).len())
}

#[test]
fn sweep_table() {
    if std::env::var("SWEEP").is_err() {
        return;
    }
    let v0 = run(&part(&[0], 1.0, None), true).vol;
    println!("sharp volume {v0}");
    let radii: Vec<f64> = std::env::var("SWEEP_R").ok().map(|s| s.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![1.0]);
    for &r in &radii {
        let mut sels: Vec<(String, Vec<usize>)> = vec![];
        for (name, v) in [("V1", V1), ("V2", V2)] {
            for mask in 1u32..16 {
                sels.push((name.to_string(), (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect()));
            }
        }
        sels.push(("V1+V2".into(), V1.iter().chain(V2.iter()).copied().collect()));
        sels.push(("ALL".into(), (0..picks().len()).collect()));
        for (name, which) in sels {
            for prop in [Some(false), Some(true)] {
                let o = run(&part(&which, r, prop), false);
                let sh = o.body.as_ref().filter(|_| o.errs.is_empty()).map(shape).unwrap_or_default();
                println!("r={r} {name} [{}] prop={} dvol={:+.4} P={} C={} folded={} errs={:?} issues={:?}", label(&which), prop.unwrap(), o.vol - v0, sh.0, sh.1, sh.2, o.errs, o.issues);
            }
        }
    }
}

/// `ONE=10,11,12,13 [R=1] [PROP=1] [DUMP=dir]`: one selection with the details
#[test]
fn one_selection() {
    let Ok(w) = std::env::var("ONE") else { return };
    let which: Vec<usize> = w.split(',').filter(|x| !x.is_empty()).map(|x| x.parse().unwrap()).collect();
    if which.is_empty() {
        return;
    }
    let r: f64 = std::env::var("R").ok().map(|x| x.parse().unwrap()).unwrap_or(1.0);
    let prop = std::env::var("PROP").ok().map(|_| true);
    let kind = std::env::var("KIND").unwrap_or("fillet".into());
    let v0 = run(&part(&[0], r, None), true).vol;
    let o = run(&part_k(&kind, &which, r, prop), false);
    let names: Vec<String> = o.body.as_ref().map(|b| b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6")).collect()).unwrap_or_default();
    println!("RESULT dvol={:+.4} errs={:?} issues={:?} faces={names:?}", o.vol - v0, o.errs, o.issues);
    if let Some(b) = &o.body {
        for l in patch_report(b) {
            println!("PATCH {l:?}");
        }
        println!("G1 {:?}", g1_deviation(b));
        for l in flipped(b, 1e-3) {
            println!("FLIPPED {l}");
        }
        if std::env::var("TRIS").is_ok() {
            let m = ck_tess::mesh::tessellate(b).mesh;
            let ids: Vec<_> = b.face_ids().collect();
            for t in 0..m.tri_face.len() {
                let nm = b.face_id_str(ids[m.tri_face[t] as usize]);
                if nm.starts_with("F6.P") {
                    let q = |k: usize| ck_geom::Vec3::new(m.positions[9 * t + 3 * k] as f64, m.positions[9 * t + 3 * k + 1] as f64, m.positions[9 * t + 3 * k + 2] as f64);
                    let g = (q(1) - q(0)).cross(q(2) - q(0));
                    println!("TRI {nm} area {:.4} ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2}) ({:.2},{:.2},{:.2})", 0.5 * g.len(), q(0).x, q(0).y, q(0).z, q(1).x, q(1).y, q(1).z, q(2).x, q(2).y, q(2).z);
                }
            }
        }
        let f = worst_fold(b, "one");
        println!("fold {:.1} {}", f.0, f.1);
    }
}

// ------------------------------------------------------------------ strict checks

const HX: f64 = 21.50030213344855;
const HZ: f64 = 16.727255833598424;

/// edge lengths with sign (concave +, convex -) of the fixture's picks at the two corners
fn signed_len(i: usize) -> f64 {
    match i {
        10 | 1 | 14 => HX,
        12 | 3 => HZ,
        11 | 0 => -HZ,
        13 | 2 => -HX,
        _ => 0.0,
    }
}

/// the collinear partner of each of the corner's edges (what the tangent chain adds)
fn partner(i: usize) -> usize {
    match i {
        10 => 13,
        13 => 10,
        11 => 12,
        12 => 11,
        3 => 0,
        0 => 3,
        2 => 1,
        1 => 2,
        x => x,
    }
}

fn build(which: &[usize], r: f64, prop: bool) -> (Res, f64) {
    let v0 = run(&part(&[0], r, None), true).vol;
    (run(&part(which, r, Some(prop)), false), v0)
}

/// A built result: no refusal, the topology / geometry / mesh checks clean, no folded triangle of visible size.
fn assert_sound(o: &Res, what: &str) {
    assert!(o.errs.is_empty(), "{what}: refused {:?}", o.errs);
    assert!(o.issues.is_empty(), "{what}: {:?}", o.issues);
    let b = o.body.as_ref().unwrap();
    let f = flipped(b, 0.05);
    assert!(f.is_empty(), "{what}: folded triangles {f:?}");
    for (name, worst, minarea, _, _) in patch_report(b) {
        assert!(worst < 45.0 && minarea > 0.3, "{what}: {name} is crumpled: neighbour normals (120 x 120 samples) {worst:.1} deg apart, min cell area {minarea:.2} of the mean");
    }
}

/// every blend stripe that touches a corner patch meets it tangentially (G1) along the shared edge
fn assert_g1_to_patches(b: &Body, what: &str) {
    let (dev, at) = g1_deviation(b);
    assert!(dev < 0.03, "{what}: {at} at {dev:.3} rad");
}

/// the worst angle (rad) between a blend stripe and a corner patch along the edge they share, and where
fn g1_deviation(b: &Body) -> (f64, String) {
    use ck_topo::body::Role;
    let mut worst = (0.0f64, String::new());
    let faces: Vec<_> = b.face_ids().collect();
    let patches: Vec<_> = faces.iter().copied().filter(|&f| b.face_id_str(f).starts_with("F6.P")).collect();
    for &bf in faces.iter().filter(|&&f| matches!(b.face(f).name.role, Role::Blend(_)) && b.face_id_str(f).starts_with("F6.B")) {
        for &pf in &patches {
            for e in b.face_edges(pf).into_iter().filter(|&e| b.face_edges(bf).contains(&e)) {
                let ed = b.edge(e);
                let pt = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
                let (u1, w1) = b.face(pf).surface.closest_param(pt);
                let (u2, w2) = b.face(bf).surface.closest_param(pt);
                let d = b.outward_normal(pf, u1, w1).normalize().dot(b.outward_normal(bf, u2, w2).normalize()).clamp(-1.0, 1.0);
                if d.acos() > worst.0 {
                    worst = (d.acos(), format!("{} meets {}", b.face_id_str(bf), b.face_id_str(pf)));
                }
            }
        }
    }
    worst
}

#[test]
fn the_sharp_part_is_two_blocks() {
    let v = run(&part(&[0], 1.0, None), true).vol;
    let exact = 2.0 * (2.0 * HX) * (2.0 * HZ) * 10.0;
    assert!((v - exact).abs() < 1e-6 * exact, "{v} vs {exact}");
}

/// The four edges of one overhang corner (the "two lines" of the report): the stripes wrap into one corner patch,
/// no step faces, G1 to the stripes, the volume the edge-only estimate gives (the concave pair adds what the
/// convex pair takes away), at every radius, with the tangent chain on or off.
#[test]
fn the_two_lines_of_an_overhang_corner_wrap_into_one_patch() {
    for (name, v) in [("V1", V1), ("V2", V2)] {
        for r in [0.5, 1.0, 2.0, 3.0] {
            for prop in [false, true] {
                let what = format!("{name} r={r} chain={prop}");
                let (o, v0) = build(&v, r, prop);
                assert_sound(&o, &what);
                let b = o.body.as_ref().unwrap();
                let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6.")).collect();
                assert_eq!(names.len(), 5, "{what}: {names:?}");
                assert!(names.iter().any(|n| n == "F6.P0") && !names.iter().any(|n| n.starts_with("F6.C")), "{what}: steps instead of the patch: {names:?}");
                assert_g1_to_patches(b, &what);
                let want: f64 = v.iter().map(|&i| signed_len(i)).sum::<f64>() * (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
                assert!((o.vol - v0 - want).abs() < 0.01 + 1e-3 * r * r * r, "{what}: volume change {} vs {want}", o.vol - v0);
            }
        }
    }
}

/// Every subset of the corner's four edges, chain off: builds, valid, volume within one corner term
/// (k r, the rounds of two meeting edges count the corner once) of the edge-only estimate; the whole four
/// give the patch, anything less ordinary stripes (at most one planar step where two meet).
#[test]
fn every_subset_of_the_four_edges_builds() {
    for (name, v) in [("V1", V1), ("V2", V2)] {
        for r in [0.5, 1.0, 2.0] {
            for mask in 1u32..16 {
                let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
                let what = format!("{name} {which:?} r={r}");
                let (o, v0) = build(&which, r, false);
                assert_sound(&o, &what);
                let k = (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
                let want = which.iter().map(|&i| signed_len(i)).sum::<f64>() * k;
                assert!((o.vol - v0 - want).abs() <= k * r * 1.05 + 1e-6, "{what}: volume change {} vs {want}", o.vol - v0);
                let b = o.body.as_ref().unwrap();
                let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6.")).collect();
                let patches = names.iter().filter(|n| n.starts_with("F6.P")).count();
                let steps = names.iter().filter(|n| n.starts_with("F6.C")).count();
                assert_eq!(patches, usize::from(which.len() == 4), "{what}: {names:?}");
                if which.len() == 4 {
                    assert_eq!(steps, 0, "{what}: {names:?}");
                }
            }
        }
    }
}

/// What the kit's "select the line" adds on its side (the collinear continuation) is what the kernel's own
/// tangent chain (`propagate: true`) adds: the same solid, bit for bit in volume.
#[test]
fn the_kernels_tangent_chain_is_the_collinear_partner_of_each_edge() {
    for v in [V1, V2] {
        for mask in 1u32..16 {
            let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
            let mut closed = which.clone();
            for &i in &which {
                if !closed.contains(&partner(i)) {
                    closed.push(partner(i));
                }
            }
            let (a, v0) = build(&which, 1.0, true);
            let (b, _) = build(&closed, 1.0, false);
            assert_sound(&a, &format!("chain {which:?}"));
            assert!((a.vol - b.vol).abs() < 1e-6, "{which:?}: chain {} vs closure {closed:?} {} (sharp {v0})", a.vol, b.vol);
        }
    }
}

/// "All connected": every edge of the part (the saved fillet), chain on and off, at several radii. Both overhang
/// corners get their patch (the staged mixed-vertex build used to leave planar steps and a ragged gap there),
/// the rest is as before; volume pinned and within the corner terms of the edge-only estimate.
#[test]
fn all_connected_edges_build_with_both_corner_patches() {
    let all: Vec<usize> = (0..picks().len()).collect();
    let pinned = [(0.5, -28.6144), (1.0, -113.3026), (2.0, -443.9700), (3.0, -978.1421)];
    for (r, want) in pinned {
        for prop in [false, true] {
            let what = format!("all r={r} chain={prop}");
            let (o, v0) = build(&all, r, prop);
            assert_sound(&o, &what);
            let b = o.body.as_ref().unwrap();
            let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            assert_eq!(names.iter().filter(|n| n.starts_with("F6.P")).count(), 2, "{what}: patches {names:?}");
            assert_eq!(names.iter().filter(|n| n.starts_with("F6.C")).count(), 0, "{what}: steps {names:?}");
            assert_g1_to_patches(b, &what);
            assert!((o.vol - v0 - want).abs() < 0.02 * r * r * r.max(1.0), "{what}: volume change {} vs pinned {want}", o.vol - v0);
            // edge-only estimate: convex 14 (hx + hz) + 80 long, concave 2 (hx + hz)
            let est = (2.0 * (HX + HZ) - 14.0 * (HX + HZ) - 80.0) * (1.0 - std::f64::consts::FRAC_PI_4) * r * r;
            assert!((o.vol - v0 - est).abs() < 3.0 * r * r * r, "{what}: {} vs estimate {est}", o.vol - v0);
        }
    }
}

/// both corners in one selection (8 edges)
#[test]
fn both_corners_together() {
    let which: Vec<usize> = V1.iter().chain(V2.iter()).copied().collect();
    for r in [0.5, 1.0, 2.0] {
        let (o, _) = build(&which, r, false);
        assert_sound(&o, &format!("both r={r}"));
        let b = o.body.as_ref().unwrap();
        let n = b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6.P")).count();
        assert_eq!(n, 2, "r={r}");
    }
}

// ------------------------------------------------------------------ chamfers

fn build_c(kind: &str, which: &[usize], d: f64, prop: bool) -> (Res, f64) {
    let v0 = run(&part(&[0], d, None), true).vol;
    (run(&part_k(kind, which, d, Some(prop)), false), v0)
}

/// the F6 faces of a result that are neither a chamfer strip (`B`) nor a corner patch (`P`): planar steps, end
/// faces, mitre triangles
fn loose_faces(b: &Body) -> Vec<String> {
    b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6.") && !n.starts_with("F6.B") && !n.starts_with("F6.P")).collect()
}

/// the loose faces whose every vertex lies within `reach` of `v`: a flap, ledge or open pocket at that corner
fn flaps_at(b: &Body, v: ck_geom::Vec3, reach: f64) -> Vec<String> {
    let mut out = vec![];
    for f in b.face_ids() {
        let n = b.face_id_str(f);
        if !n.starts_with("F6.") || n.starts_with("F6.B") || n.starts_with("F6.P") {
            continue;
        }
        let near = b.face_edges(f).iter().all(|&e| {
            let ed = b.edge(e);
            b.vertex(ed.v0).p.dist(v) <= reach && b.vertex(ed.v1).p.dist(v) <= reach
        });
        if near {
            out.push(n);
        }
    }
    out
}

const CORNER1: ck_geom::Vec3 = ck_geom::Vec3 { x: 21.50030213344855, y: 10.0, z: 0.0 };
const CORNER2: ck_geom::Vec3 = ck_geom::Vec3 { x: 0.0, y: 10.0, z: 16.727255833598424 };

/// The two lines of an overhang corner chamfered at equal distance: four strips and one corner patch through the
/// hexagon where the strips end, no planar step, flap or triangle at the corner (the report: V-shaped peaks, flat
/// triangular flaps, thin ledges and open pockets where the strips used to stop short), every edge shared by two
/// faces, the volume the edge-only estimate gives (the concave pair adds what the convex pair takes away).
#[test]
fn chamfered_two_lines_of_an_overhang_corner_get_one_patch() {
    for (name, v, at) in [("V1", V1, CORNER1), ("V2", V2, CORNER2)] {
        for d in [0.5, 1.0, 2.0, 3.0] {
            for prop in [false, true] {
                let what = format!("chamfer {name} d={d} chain={prop}");
                let (o, v0) = build_c("chamfer", &v, d, prop);
                assert_sound(&o, &what);
                let b = o.body.as_ref().unwrap();
                let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6.")).collect();
                assert_eq!(names.len(), 5, "{what}: {names:?}");
                assert!(names.iter().any(|n| n == "F6.P0"), "{what}: no corner patch {names:?}");
                assert!(loose_faces(b).is_empty(), "{what}: loose faces {:?}", loose_faces(b));
                assert!(flaps_at(b, at, 3.0 * d).is_empty(), "{what}: flaps");
                let want: f64 = v.iter().map(|&i| signed_len(i)).sum::<f64>() * 0.5 * d * d;
                assert!((o.vol - v0 - want).abs() < 0.01 + 1e-3 * d * d * d, "{what}: volume change {} vs {want}", o.vol - v0);
            }
        }
    }
}

/// Every subset of the corner's four edges, equal distance, chain off: builds valid. The volume is within one corner
/// term (d^3 / 2: two meeting strips end in one planar step) of the estimate; the whole four give the patch.
#[test]
fn every_subset_of_the_four_chamfered_edges_builds() {
    for (name, v) in [("V1", V1), ("V2", V2)] {
        for d in [0.5, 1.0, 2.0, 3.0] {
            for mask in 1u32..16 {
                let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
                let what = format!("chamfer {name} {which:?} d={d}");
                let (o, v0) = build_c("chamfer", &which, d, false);
                assert_sound(&o, &what);
                let want = which.iter().map(|&i| signed_len(i)).sum::<f64>() * 0.5 * d * d;
                assert!((o.vol - v0 - want).abs() <= 0.55 * d * d * d + 1e-6, "{what}: volume change {} vs {want}", o.vol - v0);
                let b = o.body.as_ref().unwrap();
                let patches = b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F6.P")).count();
                assert_eq!(patches, usize::from(which.len() == 4), "{what}");
                if which.len() == 4 {
                    assert!(loose_faces(b).is_empty(), "{what}: {:?}", loose_faces(b));
                }
            }
        }
    }
}

/// the kernel's tangent chain (`propagate`) adds the collinear partner of each edge, for chamfers too
#[test]
fn the_tangent_chain_of_a_chamfer_is_the_collinear_partner_of_each_edge() {
    for v in [V1, V2] {
        for mask in 1u32..16 {
            let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
            let mut closed = which.clone();
            for &i in &which {
                if !closed.contains(&partner(i)) {
                    closed.push(partner(i));
                }
            }
            let (a, _) = build_c("chamfer", &which, 1.0, true);
            let (b, _) = build_c("chamfer", &closed, 1.0, false);
            assert_sound(&a, &format!("chamfer chain {which:?}"));
            assert!((a.vol - b.vol).abs() < 1e-6, "{which:?}: chain {} vs closure {closed:?} {}", a.vol, b.vol);
        }
    }
}

/// "All connected" chamfers: every edge of the part, chain on and off. Both overhang corners get their patch and
/// no flap; the concave-concave-convex vertex under the overhang used to leave the solid open (an edge shared by
/// four faces: a refusal) and now closes in another round order (the corners are built first, on their own). Volume pinned and within the corner terms.
#[test]
fn all_connected_chamfers_build_with_both_corner_patches() {
    let all: Vec<usize> = (0..picks().len()).collect();
    let pinned = [(0.5, -66.0332), (1.0, -258.8994), (2.0, -993.7315), (3.0, -2141.6973)];
    for (d, want) in pinned {
        for prop in [false, true] {
            let what = format!("chamfer all d={d} chain={prop}");
            let (o, v0) = build_c("chamfer", &all, d, prop);
            assert_sound(&o, &what);
            let b = o.body.as_ref().unwrap();
            assert_eq!(b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F6.P")).count(), 2, "{what}");
            assert!(flaps_at(b, CORNER1, 3.0 * d).is_empty() && flaps_at(b, CORNER2, 3.0 * d).is_empty(), "{what}: flaps near an overhang corner");
            assert!((o.vol - v0 - want).abs() < 0.02 * d * d * d.max(1.0), "{what}: volume change {} vs pinned {want}", o.vol - v0);
            let est = (2.0 * (HX + HZ) - 14.0 * (HX + HZ) - 80.0) * 0.5 * d * d;
            assert!((o.vol - v0 - est).abs() < 12.0 * d * d * d, "{what}: {} vs estimate {est}", o.vol - v0);
        }
    }
}

#[test]
fn both_chamfered_corners_together() {
    let which: Vec<usize> = V1.iter().chain(V2.iter()).copied().collect();
    for d in [0.5, 1.0, 2.0, 3.0] {
        let (o, _) = build_c("chamfer", &which, d, false);
        assert_sound(&o, &format!("both d={d}"));
        let b = o.body.as_ref().unwrap();
        assert_eq!(b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F6.P")).count(), 2, "d={d}");
    }
}

/// Two distances: with the first distance on one named face (every edge must border it) the edges of that face
/// build; all four edges of a corner cannot name one face and are refused with the reason and a hint, so is a
/// tangent chain across edges that do not share it.
#[test]
fn chamfers_with_two_distances_build_on_one_face_and_refuse_the_four_edge_corner() {
    let (o, v0) = build_c("chamfer2F", &[V1[0], V1[1]], 1.0, false);
    assert_sound(&o, "two distances on F2.C1");
    let want = (signed_len(V1[0]) + signed_len(V1[1])) * 0.5 * 1.0 * 2.5;
    assert!((o.vol - v0 - want).abs() < 1.5, "two distances: {} vs {want}", o.vol - v0);
    for kind in ["chamfer2", "chamferA"] {
        let (o, _) = build_c(kind, &V1, 1.0, false);
        let e = o.errs.first().unwrap_or_else(|| panic!("{kind}: four edges of the corner built"));
        assert!(e.contains("mixed face order") && e.contains("hint"), "{kind}: {e}");
        // the reason (the distances would swap sides from edge to edge) and what to do (an equal-distance chamfer, or the four edges left out)
        assert!(e.contains("swap sides") && e.contains("only an equal-distance chamfer can patch"), "{kind}: {e}");
        assert!(e.contains("leave the four edges at that corner out of the selection"), "{kind}: {e}");
    }
    let (o, _) = build_c("chamfer2F", &V1, 1.0, false);
    assert!(o.errs.first().is_some_and(|e| e.contains("missingEdge")), "{:?}", o.errs);
    let (o, _) = build_c("chamfer2F", &[V1[0]], 1.0, true);
    let e = o.errs.first().unwrap_or_else(|| panic!("chain across faces built"));
    assert!(e.contains("tangent chain") && e.contains("hint"), "{e}");
}

// ------------------------------------------------------------------ corpus rows

/// `OH_EMIT=<file> cargo test -p ck-model --test overhang_fillet emit_corpus -- --nocapture`, then
/// `node tools/merge_stepped_corpus.mjs <file> overhang` (docs/blend-table.md, "Overhang cases").
#[test]
fn emit_corpus() {
    let Ok(out_path) = std::env::var("OH_EMIT") else { return };
    let base_src = part(&[0], 1.0, None);
    let base_feats = {
        let fs = base_src.find("\"features\"").unwrap();
        let fa = fs + base_src[fs..].find('[').unwrap() + 1;
        let fi = base_src.find("\"type\": \"fillet\"").unwrap();
        let cut = base_src[..fi].rfind('{').unwrap();
        base_src[fa..cut].trim().trim_end_matches(',').to_string()
    };
    let v0 = run(&base_src, true).vol;
    let mut specs: Vec<(String, Vec<usize>, f64, bool)> = vec![];
    for r in [0.5, 1.0, 2.0] {
        specs.push((format!("overhang-v1-4edges-r{r}"), V1.to_vec(), r, false));
        specs.push((format!("overhang-v2-4edges-r{r}"), V2.to_vec(), r, false));
        specs.push((format!("overhang-both-8edges-r{r}"), V1.iter().chain(V2.iter()).copied().collect(), r, false));
        specs.push((format!("overhang-all-r{r}"), (0..picks().len()).collect(), r, false));
        specs.push((format!("overhang-all-chain-r{r}"), (0..picks().len()).collect(), r, true));
    }
    for (name, v) in [("v1", V1), ("v2", V2)] {
        for mask in 1u32..15 {
            let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
            specs.push((format!("overhang-{name}-sub{}-r1", which.iter().map(|x| x.to_string()).collect::<Vec<_>>().join("_")), which, 1.0, false));
        }
        specs.push((format!("overhang-{name}-line-chain-r1"), vec![v[0]], 1.0, true));
        specs.push((format!("overhang-{name}-two-lines-chain-r1"), vec![v[0], v[1]], 1.0, true));
    }
    // the mixed vertex under the overhang (two concave edges and the convex one between them), in two pick orders
    for r in [0.5, 1.0, 2.0] {
        specs.push((format!("overhang-v3-3edges-r{r}"), MIXED.to_vec(), r, false));
        specs.push((format!("overhang-v3-3edges-rev-r{r}"), MIXED.iter().rev().copied().collect(), r, false));
    }
    // chamfers: the same selections with the equal-distance chamfer, plus two distances on one face
    let mut cspecs: Vec<(String, Vec<usize>, f64, bool, &str)> = vec![];
    for d in [0.5, 1.0, 2.0] {
        cspecs.push((format!("overhang-chamfer-v1-4edges-d{d}"), V1.to_vec(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-v2-4edges-d{d}"), V2.to_vec(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-both-8edges-d{d}"), V1.iter().chain(V2.iter()).copied().collect(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-all-d{d}"), (0..picks().len()).collect(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-all-chain-d{d}"), (0..picks().len()).collect(), d, true, "chamfer"));
    }
    for (name, v) in [("v1", V1), ("v2", V2)] {
        for mask in 1u32..15 {
            let which: Vec<usize> = (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect();
            cspecs.push((format!("overhang-chamfer-{name}-sub{}-d1", which.iter().map(|x| x.to_string()).collect::<Vec<_>>().join("_")), which, 1.0, false, "chamfer"));
        }
        cspecs.push((format!("overhang-chamfer-{name}-line-chain-d1"), vec![v[0]], 1.0, true, "chamfer"));
        cspecs.push((format!("overhang-chamfer-{name}-two-lines-chain-d1"), vec![v[0], v[1]], 1.0, true, "chamfer"));
    }
    for d in [0.5, 1.0, 2.0] {
        cspecs.push((format!("overhang-chamfer-v3-3edges-d{d}"), MIXED.to_vec(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-v3-3edges-rev-d{d}"), MIXED.iter().rev().copied().collect(), d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-v3-3edges-mid-d{d}"), vec![12, 1, 18], d, false, "chamfer"));
        cspecs.push((format!("overhang-chamfer-v3-all-rev-d{d}"), (0..picks().len()).rev().collect(), d, false, "chamfer"));
    }
    // a convex and a concave edge of the mixed vertex without the third: both pick orders
    for (name, which) in [("v3-pair-1-18", [1usize, 18]), ("v3-pair-18-1", [18, 1]), ("v3-pair-12-18", [12, 18]), ("v3-pair-18-12", [18, 12])] {
        cspecs.push((format!("overhang-chamfer-{name}-d1"), which.to_vec(), 1.0, false, "chamfer"));
    }
    cspecs.push(("overhang-chamfer-v1-two-distances-on-face-d1".into(), vec![V1[0], V1[1]], 1.0, false, "chamfer2F"));
    cspecs.push(("overhang-chamfer-v1-4edges-two-distances-d1".into(), V1.to_vec(), 1.0, false, "chamfer2"));
    cspecs.push(("overhang-chamfer-v1-4edges-angle-d1".into(), V1.to_vec(), 1.0, false, "chamferA"));
    let mut all_specs: Vec<(String, Vec<usize>, f64, bool, &str)> = specs.into_iter().map(|(a, b, c, d)| (a, b, c, d, "fillet")).collect();
    all_specs.extend(cspecs);
    let mut cases = vec![];
    let mut expect = vec![];
    for (id, which, r, prop, kind) in all_specs {
        let src = part_k(kind, &which, r, Some(prop));
        let tname = if kind == "fillet" { "fillet" } else { "chamfer" };
        let feat = object_at(&src, src[..src.find(&format!("\"type\": \"{tname}\"")).unwrap()].rfind('{').unwrap());
        let o = run(&src, false);
        let (reference, exp) = if let Some(e) = o.errs.first() {
            ("null".to_string(), format!(r#""{id}":{{"expect":"expected_fail","observed":{}}}"#, ck_json::Json::Str(format!("refused: {e}")).to_string()))
        } else if !o.issues.is_empty() {
            ("null".to_string(), format!(r#""{id}":{{"expect":"expected_fail","observed":{}}}"#, ck_json::Json::Str(format!("invalid: {}", o.issues[0])).to_string()))
        } else {
            (format!(r#"{{"ok":true,"volume":{},"base_volume":{v0},"source":"baseline"}}"#, o.vol), format!(r#""{id}":{{"expect":"pass","observed":"passes"}}"#))
        };
        println!("EMIT {id} {}", exp.split_once(':').unwrap().1);
        let fam = id.split('-').take(2).collect::<Vec<_>>().join("-");
        cases.push(format!(r#"{{"id":"{id}","family":"{fam}","base":"overhang-stack","fillet":{},"reference":{reference}}}"#, compact(&feat)));
        expect.push(exp);
    }
    let doc = format!("{{\"bases\":{{\"overhang-stack\":[{}]}},\"cases\":[{}],\"expect\":{{{}}}}}", compact(&base_feats), cases.join(","), expect.join(","));
    if let Some(d) = std::path::Path::new(&out_path).parent() {
        std::fs::create_dir_all(d).ok();
    }
    std::fs::write(&out_path, doc).unwrap();
}

/// the `{...}` object starting at byte `at`
fn object_at(s: &str, at: usize) -> String {
    let mut depth = 0;
    for (k, c) in s[at..].char_indices() {
        match c {
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return s[at..at + k + 1].to_string();
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced")
}

/// the JSON text without its layout whitespace
fn compact(s: &str) -> String {
    ck_json::parse(s).map(|j| j.to_string()).unwrap_or_else(|_| s.to_string())
}

/// `SWEEPC=1 [KIND=chamfer|chamfer2|chamferA] [SWEEP_R=0.5,1,2,3]`: the chamfer version of `sweep_table`
#[test]
fn sweep_chamfer_table() {
    if std::env::var("SWEEPC").is_err() {
        return;
    }
    let kind = std::env::var("KIND").unwrap_or("chamfer".into());
    let v0 = run(&part(&[0], 1.0, None), true).vol;
    let radii: Vec<f64> = std::env::var("SWEEP_R").ok().map(|s| s.split(',').map(|x| x.parse().unwrap()).collect()).unwrap_or(vec![1.0]);
    for &r in &radii {
        let mut sels: Vec<(String, Vec<usize>)> = vec![];
        for (name, v) in [("V1", V1), ("V2", V2)] {
            for mask in 1u32..16 {
                sels.push((name.to_string(), (0..4).filter(|i| mask >> i & 1 == 1).map(|i| v[i]).collect()));
            }
        }
        sels.push(("V1+V2".into(), V1.iter().chain(V2.iter()).copied().collect()));
        sels.push(("ALL".into(), (0..picks().len()).collect()));
        for (name, which) in sels {
            for prop in [Some(false), Some(true)] {
                let o = run(&part_k(&kind, &which, r, prop), false);
                let nm: Vec<String> = o.body.as_ref().map(|b| b.face_ids().map(|f| b.face_id_str(f)).filter(|n| n.starts_with("F6")).collect()).unwrap_or_default();
                let steps = nm.iter().filter(|n| n.starts_with("F6.C") || n.starts_with("F6.P") || n.contains(".V")).count();
                let folded = o.body.as_ref().filter(|_| o.errs.is_empty()).map(|b| flipped(b, 1e-3).len()).unwrap_or(0);
                println!("{kind} s={r} {name} [{}] prop={} dvol={:+.4} faces={} other={} folded={folded} errs={:?} issues={:?}", label(&which), prop.unwrap(), o.vol - v0, nm.len(), steps, o.errs, o.issues);
            }
        }
    }
}

/// `BIS=1 [KIND=chamfer] [R=1]`: which single pick, left out of the whole selection, lets it build?
#[test]
fn bisect_all() {
    if std::env::var("BIS").is_err() {
        return;
    }
    let kind = std::env::var("KIND").unwrap_or("chamfer".into());
    let r: f64 = std::env::var("R").ok().map(|x| x.parse().unwrap()).unwrap_or(1.0);
    let n = picks().len();
    for skip in 0..n {
        let which: Vec<usize> = (0..n).filter(|&i| i != skip).collect();
        let o = run(&part_k(&kind, &which, r, Some(false)), false);
        println!("BIS without {skip}: errs={:?} issues={:?}", o.errs.first().map(|e| e.chars().take(120).collect::<String>()), o.issues.first());
    }
    // groups: only the A block's edges, only B's
    let a: Vec<usize> = (0..n).filter(|&i| picks()[i].contains("F2.S")).collect();
    let b: Vec<usize> = (0..n).filter(|&i| !picks()[i].contains("F2.S")).collect();
    for (nm, w) in [("A-wall edges", a), ("not A-wall edges", b)] {
        let o = run(&part_k(&kind, &w, r, Some(false)), false);
        println!("BIS group {nm} {:?}: errs={:?} issues={:?}", w, o.errs.first().map(|e| e.chars().take(120).collect::<String>()), o.issues.first());
    }
}

/// `BISADD=1 [KIND=chamfer] [R=0.5]`: the two corners plus the two vertical edges that close the mixed vertices, plus one more
/// pick each: which one stops the corner patches?
#[test]
fn bisect_add() {
    if std::env::var("BISADD").is_err() {
        return;
    }
    let kind = std::env::var("KIND").unwrap_or("chamfer".into());
    let r: f64 = std::env::var("R").ok().map(|x| x.parse().unwrap()).unwrap_or(0.5);
    let base: Vec<usize> = V1.iter().chain(V2.iter()).chain([9usize, 18].iter()).copied().collect();
    let others: Vec<usize> = (0..picks().len()).filter(|i| !base.contains(i)).collect();
    for k in 1..=others.len() {
        let add = others[k - 1];
        let mut w = base.clone();
        w.extend(others[..k].iter().copied());
        let o = run(&part_k(&kind, &w, r, Some(false)), false);
        let p = o.body.as_ref().map(|b| b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F6.P")).count()).unwrap_or(0);
        println!("BISADD +{add}: patches {p} errs={:?}", o.errs.first().map(|e| e.chars().take(80).collect::<String>()));
    }
}

/// the loose faces (see `loose_faces`) with the distance of their nearest vertex to `v`
fn loose_near(b: &Body, v: ck_geom::Vec3) -> Vec<(String, f64)> {
    let mut out = vec![];
    for f in b.face_ids() {
        let n = b.face_id_str(f);
        if !n.starts_with("F6.") || n.starts_with("F6.B") || n.starts_with("F6.P") {
            continue;
        }
        let d = b.face_edges(f).iter().flat_map(|&e| [b.vertex(b.edge(e).v0).p.dist(v), b.vertex(b.edge(e).v1).p.dist(v)]).fold(f64::INFINITY, f64::min);
        out.push((n, (d * 100.0).round() / 100.0));
    }
    out
}

const MIXED_V: ck_geom::Vec3 = ck_geom::Vec3 { x: 21.50030213344855, y: 10.0, z: 16.727255833598424 };

/// `MIXPROBE=1`: every subset of the edges round the mixed vertex (21.5, 10, 16.7) and its two 4-valent neighbours, with
/// and without the other 19 edges, in picked and reversed order. Prints the selections that refuse, leave planar steps,
/// are unsound or depend on the order.
#[test]
fn mixed_probe() {
    if std::env::var("MIXPROBE").is_err() {
        return;
    }
    let core = [1usize, 12, 18, 2, 11, 10, 13, 3, 0];
    let rest: Vec<usize> = (0..picks().len()).filter(|i| !core.contains(i)).collect();
    let v0 = run(&part(&[0], 1.0, None), true).vol;
    for mask in 1u32..(1 << (core.len() + usize::from(std::env::var("MIXREST").is_ok()))) {
        let mut which: Vec<usize> = (0..core.len()).filter(|i| mask >> i & 1 == 1).map(|i| core[i]).collect();
        if mask >> core.len() & 1 == 1 {
            which.extend(rest.iter().copied());
        }
        if which.is_empty() {
            continue;
        }
        let mut rev = which.clone();
        rev.reverse();
        let kind = std::env::var("MIXKIND").unwrap_or("fillet".into());
        let a = run(&part_k(&kind, &which, 1.0, Some(false)), false);
        let b = run(&part_k(&kind, &rev, 1.0, Some(false)), false);
        let desc = |o: &Res| -> String {
            match &o.body {
                _ if !o.errs.is_empty() => format!("REFUSED {}", o.errs[0].chars().take(110).collect::<String>()),
                Some(bd) => format!("dvol={:+.4} loose={:?} folded={} issues={}", o.vol - v0, loose_near(bd, MIXED_V), flipped(bd, 1e-3).len(), o.issues.len()),
                None => "none".into(),
            }
        };
        let (da, db) = (desc(&a), desc(&b));
        let bad = |o: &Res, d: &str| d.starts_with("REFUSED") || !o.issues.is_empty() || o.body.as_ref().map_or(false, |bd| !loose_faces(bd).is_empty() || !flipped(bd, 0.05).is_empty());
        let dep = (a.vol - b.vol).abs() > 1e-6 || (a.errs.is_empty() != b.errs.is_empty());
        if bad(&a, &da) || bad(&b, &db) || dep {
            println!("MIX [{}] {}\n    fwd {da}\n    rev {db}", label(&which.iter().copied().filter(|i| core.contains(i)).collect::<Vec<_>>()) + if mask >> core.len() & 1 == 1 { " +rest" } else { "" }, if dep { "ORDER-DEPENDENT" } else { "" });
        }
    }
}

// ------------------------------------------------------------------ the mixed vertex under the overhang

/// all six pick orders of the three edges of the mixed vertex
fn orders() -> Vec<[usize; 3]> {
    let [a, b, c] = MIXED;
    vec![[a, b, c], [a, c, b], [b, a, c], [b, c, a], [c, a, b], [c, b, a]]
}

/// A chamfer of the three edges of the mixed vertex (two concave, one convex: the foot of the corner of block A under block B)
/// is the same part in every pick order: the convex strip ends on the underside of B in a straight edge, and the two concave strips and
/// that edge are one chain, mitred at its joints. Built edge by edge in the pick order it used to leave a planar step or a face of
/// the tool, and a different volume for each order (13.11, 13.61 and 14.11 at d = 1).
#[test]
fn the_mixed_vertex_chamfers_alike_in_every_pick_order() {
    for d in [0.5, 1.0, 2.0] {
        let mut first: Option<f64> = None;
        for ord in orders() {
            let what = format!("chamfer {ord:?} d={d}");
            let (o, v0) = build_c("chamfer", &ord, d, false);
            assert_sound(&o, &what);
            let b = o.body.as_ref().unwrap();
            assert!(loose_faces(b).is_empty(), "{what}: planar steps or tool faces {:?}", loose_faces(b));
            let strips = b.face_ids().filter(|&f| b.face_id_str(f).starts_with("F6.B")).count();
            assert_eq!(strips, 4, "{what}: three strips and the end strip of the convex one");
            // the edge-only estimate: wedges along the two concave edges (21.5 and 16.7 long), the vertical strip's cut (10 high)
            let est = 0.5 * d * d * (HX + HZ - 10.0);
            assert!((o.vol - v0 - est).abs() < 0.1 * d * d * d, "{what}: volume change {} vs estimate {est}", o.vol - v0);
            let want = *first.get_or_insert(o.vol);
            assert!((o.vol - want).abs() < 1e-6, "{what}: volume {} differs from the first order's {want}", o.vol);
        }
    }
}

/// the same for the fillet (built in stages since the mixed-vertex work: convex first, then the concave chain on a torus)
#[test]
fn the_mixed_vertex_fillets_alike_in_every_pick_order() {
    for r in [0.5, 1.0, 2.0] {
        let mut first: Option<f64> = None;
        for ord in orders() {
            let what = format!("fillet {ord:?} r={r}");
            let (o, _) = build(&ord, r, false);
            assert_sound(&o, &what);
            let b = o.body.as_ref().unwrap();
            assert!(loose_faces(b).is_empty(), "{what}: planar steps {:?}", loose_faces(b));
            let want = *first.get_or_insert(o.vol);
            assert!((o.vol - want).abs() < 1e-6, "{what}: volume {} differs from the first order's {want}", o.vol);
        }
    }
}

/// With the other corner edges selected too (the two overhang corners V1 and V2 share an edge with the mixed vertex) the chamfer is built
/// in the same stages and the corner patches are spliced in: reversed or not, the same part, two patches, no step near either corner or
/// the mixed vertex.
#[test]
fn the_mixed_vertex_with_the_overhang_corners_is_the_same_part_in_reverse_order() {
    let all: Vec<usize> = (0..picks().len()).collect();
    let rev: Vec<usize> = all.iter().rev().copied().collect();
    for d in [0.5, 1.0, 2.0] {
        let what = format!("chamfer all d={d}");
        let (a, _) = build_c("chamfer", &all, d, false);
        let (b, _) = build_c("chamfer", &rev, d, false);
        assert_sound(&a, &what);
        assert_sound(&b, &what);
        assert!((a.vol - b.vol).abs() < 1e-6, "{what}: {} vs reversed {}", a.vol, b.vol);
        let bd = b.body.as_ref().unwrap();
        assert!(flaps_at(bd, MIXED_V, 3.0 * d).is_empty(), "{what}: loose faces at the mixed vertex");
        // V1 and V2 each keep their corner patch
        assert_eq!(bd.face_ids().filter(|&f| bd.face_id_str(f).starts_with("F6.P")).count(), 2, "{what}");
    }
}

/// A convex and a concave edge of the mixed vertex with the third edge left sharp: the convex strip is cut first in any pick order, so the
/// two orders give one part (they gave a face of the tool in one order and a capped end in the other).
#[test]
fn a_convex_and_a_concave_edge_of_the_mixed_vertex_chamfer_alike_in_both_pick_orders() {
    for pair in [[1usize, 18], [12, 18]] {
        let (a, _) = build_c("chamfer", &pair, 1.0, false);
        let (b, _) = build_c("chamfer", &[pair[1], pair[0]], 1.0, false);
        assert_sound(&a, &format!("{pair:?}"));
        assert_sound(&b, &format!("{pair:?} reversed"));
        assert!((a.vol - b.vol).abs() < 1e-6, "{pair:?}: {} vs reversed {}", a.vol, b.vol);
        for (o, tag) in [(&a, "picked"), (&b, "reversed")] {
            let tool: Vec<String> = o.body.as_ref().unwrap().face_ids().map(|f| o.body.as_ref().unwrap().face_id_str(f)).filter(|n| n.starts_with("F6.S.")).collect();
            assert!(tool.is_empty(), "{pair:?} {tag}: faces of the tool left in the part: {tool:?}");
        }
    }
}

/// A convex and a concave edge of the mixed vertex without the third, together with a fully selected overhang corner that shares an edge with
/// it: chamfered it left a face of the tool in the part (valid and clean by every check, a flap in the model). It is refused with the vertex,
/// the reason and a hint, in either pick order, and the same selection with the third edge builds.
#[test]
fn a_partial_mixed_vertex_next_to_an_overhang_corner_is_refused_not_left_with_a_tool_face() {
    for sel in [vec![12usize, 18, 11, 10, 13], vec![13, 10, 11, 18, 12], vec![1, 12, 11, 10, 13]] {
        let (o, _) = build_c("chamfer", &sel, 1.0, false);
        assert!(o.body.is_none() || !o.errs.is_empty(), "{sel:?}: built a body with a leftover tool face");
        let e = &o.errs[0];
        assert!(e.contains("blendCorner") && e.contains("do not close into a clean corner") && e.contains("hint"), "{sel:?}: {e}");
        assert!(e.contains("(21.500, 10.000,"), "{sel:?}: the refusal names no vertex: {e}");
    }
    let mut full = vec![12usize, 18, 1, 11, 10, 13];
    full.sort_unstable();
    let (o, _) = build_c("chamfer", &full, 1.0, false);
    assert_sound(&o, "all three edges and the corner");
    assert!(loose_faces(o.body.as_ref().unwrap()).iter().all(|n| !n.starts_with("F6.S.")), "tool face left");
}
