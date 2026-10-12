//! Diagnostic (ignored): `PART=path UPTO=n cargo test -p ck-model --profile fast
//! --test helix_probe -- --ignored --nocapture` regenerates a part (up to feature n),
//! printing the elapsed time and volume.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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

#[test]
#[ignore]
fn helix_probe() {
    let path = std::env::var("PART").unwrap();
    let upto: Option<u32> = std::env::var("UPTO").ok().and_then(|s| s.parse().ok());
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts { up_to: upto, ..RegenOpts::default() }, &mut ex);
    eprintln!("regen {:.2}s errors {}", t.elapsed().as_secs_f64(), r.to_json().to_string().chars().take(300).collect::<String>());
    let b = ex.0.body.as_ref().unwrap();
    let t = Instant::now();
    let m = ck_topo::mass::mass_props(b);
    eprintln!("mass_props {:.2}s {:?} lumps {} faces {}", t.elapsed().as_secs_f64(), m, b.num_lumps(), b.face_ids().count());
    if std::env::var("FIRES").is_ok() {
        let r = ck_topo::mass::ref_point_of(b);
        for (i, f) in b.face_ids().enumerate() {
            let row: Vec<String> = [1usize, 2, 4].iter().map(|&res| format!("{:.4}", ck_topo::mass::face_integral::<1>(b, f, res, &|p, n| [(p - r).dot(n) / 3.0])[0])).collect();
            eprintln!("FIRES face {i} {} {:?}", b.face_id_str(f), row);
        }
    }
    if std::env::var("PCDEV").is_ok() {
        for (i, f) in b.face_ids().enumerate() {
            for &l in &b.face(f).loops {
                for &c in &b.lp(l).coedges {
                    let (ta, tb) = b.coedge_params(c);
                    let e = b.edge(b.coedge(c).edge);
                    let mut worst = (0.0f64, 0.0f64);
                    for k in 0..=20000 {
                        let t = ta + (tb - ta) * k as f64 / 20000.0;
                        let uv = b.coedge_uv(c, t, None);
                        let d = b.face(f).surface.eval(uv.x, uv.y).dist(e.curve.eval(t));
                        if d > worst.0 {
                            worst = (d, t);
                        }
                    }
                    if worst.0 > 1e-4 && std::env::var("PCDETAIL").is_ok() {
                        for k in 0..=12 {
                            let t = ta + (tb - ta) * [0.0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006, 0.008, 0.01, 0.015, 0.02, 0.05, 0.1][k];
                            let uv = b.coedge_uv(c, t, None);
                            let (p3, ps) = (e.curve.eval(t), b.face(f).surface.eval(uv.x, uv.y));
                            eprintln!("   t {t:.5} uv ({:.4},{:.4}) curve {:.4?} surf {:.4?} d {:.4}", uv.x, uv.y, p3, ps, p3.dist(ps));
                        }
                    }
                    if worst.0 > 2e-4 {
                        eprintln!("PCDEV face {i} {} edge {:?} {} t[{ta:.4},{tb:.4}] worst {:.5} at t={:.4} has_pc {}", b.face_id_str(f), b.coedge(c).edge, e.curve.kind(), worst.0, worst.1, b.coedge(c).pcurve.is_some());
                    }
                }
            }
        }
    }
    if std::env::var("TESS").is_ok() {
        let t = Instant::now();
        let tm = ck_tess::mesh::tessellate(b);
        let mut per = std::collections::BTreeMap::new();
        for &f in &tm.mesh.tri_face {
            *per.entry(f).or_insert(0usize) += 1;
        }
        eprintln!("tessellate {:.2}s tris {} per face {:?}", t.elapsed().as_secs_f64(), tm.mesh.tri_face.len(), per);
        if let Some(y0) = std::env::var("MESHREF").ok().and_then(|s| s.parse::<f64>().ok()) {
            // divergence-theorem volume of the display mesh about (0, y0, 0): a planar cap in y = y0 contributes nothing
            let pos = &tm.mesh.positions;
            let mut v = 0.0f64;
            for t in 0..pos.len() / 9 {
                let q = |k: usize| [pos[9 * t + 3 * k] as f64, pos[9 * t + 3 * k + 1] as f64 - y0, pos[9 * t + 3 * k + 2] as f64];
                let (a, b, c) = (q(0), q(1), q(2));
                v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
            }
            eprintln!("MESHVOL {v:.3} (positions {} f32)", pos.len());
        }
        for (i, f) in b.face_ids().enumerate() {
            eprintln!("  face {i} {} {}", b.face_id_str(f), b.face(f).surface.kind());
            if std::env::var("LOOPS").ok().and_then(|s| s.parse::<usize>().ok()) == Some(i) {
                for &l in &b.face(f).loops {
                    eprintln!("   loop with {} coedges", b.lp(l).coedges.len());
                    for &c in &b.lp(l).coedges {
                        let (ta, tb) = b.coedge_params(c);
                        let e = b.edge(b.coedge(c).edge);
                        eprintln!("     {} t[{:.4},{:.4}] {:?} -> {:?} len~{:.2}", e.curve.kind(), ta, tb, b.coedge_start(c), b.coedge_end(c), e.curve.eval(ta).dist(e.curve.eval(tb)));
                    }
                }
            }
        }
    }
}

/// Deviation of the (Y-up) sweep faces from the ideal screw tube R=56.25 r=3.52 pitch 17.24.
#[test]
#[ignore]
fn helix_dev() {
    let path = std::env::var("PART").unwrap();
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    let (big_r, rad, k) = (56.25f64, 3.52f64, 17.24 / std::f64::consts::TAU);
    let (mut worst, mut sum, mut n, mut smin, mut smax) = (0.0f64, 0.0, 0, f64::INFINITY, f64::NEG_INFINITY);
    for f in b.face_ids() {
        let ck_geom::surface::Surface::BSpline(bs) = &b.face(f).surface else { continue };
        let ((u0, u1), (v0, v1)) = bs.domain();
        for i in 0..=1200 {
            for j in 0..=48 {
                let p = bs.eval(u0 + (u1 - u0) * i as f64 / 1200.0, v0 + (v1 - v0) * j as f64 / 48.0);
                let rho = p.x.hypot(p.z);
                let alpha = (-p.z).atan2(p.x);
                let mut best = f64::INFINITY;
                for m in -2..16 {
                    let phi = alpha + std::f64::consts::TAU * m as f64;
                    let s = (rho - big_r).hypot(p.y - k * phi) - rad;
                    if s.abs() < best.abs() { best = s; }
                }
                worst = worst.max(best.abs());
                sum += best.abs();
                smin = smin.min(best);
                smax = smax.max(best);
                n += 1;
            }
        }
    }
    eprintln!("sweep faces: max |dev| {worst:.6}, mean {:.6}, signed [{smin:.6}, {smax:.6}] over {n}", sum / n as f64);
}

/// Independent volume of the (sweep-only) body clipped to `lo <= y <= hi`, by a fine triangulation of its
/// B-spline faces and flat caps (divergence theorem about (0, ref_y, 0); caps in the clip planes contribute 0).
#[test]
#[ignore]
fn clip_volume() {
    let path = std::env::var("PART").unwrap();
    let (lo, hi): (f64, f64) = (std::env::var("LO").ok().and_then(|s| s.parse().ok()).unwrap_or(-1e9), std::env::var("HI").ok().and_then(|s| s.parse().ok()).unwrap_or(1e9));
    let part = Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let _ = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let b = ex.0.body.as_ref().unwrap();
    let refy = if hi < 1e8 { hi } else if lo > -1e8 { lo } else { 0.0 };
    type P = [f64; 3];
    let clip = |tri: [P; 3]| -> Vec<[P; 3]> {
        // keep lo <= y <= hi: Sutherland-Hodgman on the two planes, fan the result
        let mut poly: Vec<P> = tri.to_vec();
        for (val, keep_above) in [(lo, true), (hi, false)] {
            let mut out: Vec<P> = vec![];
            let n = poly.len();
            for i in 0..n {
                let (a, c) = (poly[i], poly[(i + 1) % n]);
                let (da, dc) = ((a[1] - val) * if keep_above { 1.0 } else { -1.0 }, (c[1] - val) * if keep_above { 1.0 } else { -1.0 });
                if da >= 0.0 {
                    out.push(a);
                }
                if (da >= 0.0) != (dc >= 0.0) {
                    let t = da / (da - dc);
                    out.push([a[0] + (c[0] - a[0]) * t, a[1] + (c[1] - a[1]) * t, a[2] + (c[2] - a[2]) * t]);
                }
            }
            poly = out;
            if poly.is_empty() {
                break;
            }
        }
        (1..poly.len().saturating_sub(1)).map(|i| [poly[0], poly[i], poly[i + 1]]).collect()
    };
    let vol = |t: [P; 3]| -> f64 {
        let q = |k: usize| [t[k][0], t[k][1] - refy, t[k][2]];
        let (a, b, c) = (q(0), q(1), q(2));
        (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    };
    let mut total = 0.0;
    for f in b.face_ids() {
        let face = b.face(f);
        match &face.surface {
            ck_geom::surface::Surface::BSpline(bs) => {
                let ((u0, u1), (v0, v1)) = bs.domain();
                let (nu, nv) = (12000usize, 96usize);
                let mut sub = 0.0;
                for i in 0..nu {
                    for j in 0..nv {
                        let p = |a: usize, c: usize| {
                            let q = bs.eval(u0 + (u1 - u0) * a as f64 / nu as f64, v0 + (v1 - v0) * c as f64 / nv as f64);
                            [q.x, q.y, q.z]
                        };
                        let (p00, p10, p01, p11) = (p(i, j), p(i + 1, j), p(i, j + 1), p(i + 1, j + 1));
                        for t in [[p00, p10, p11], [p00, p11, p01]] {
                            for c in clip(t) {
                                sub += vol(c);
                            }
                        }
                    }
                }
                eprintln!("face {} raw {sub:.3} sense {}", b.face_id_str(f), face.sense);
                total += sub;
            }
            ck_geom::surface::Surface::Plane { frame } => {
                // flat cap: fan the sampled boundary
                let mut pts: Vec<P> = vec![];
                for &l in &face.loops {
                    for &c in &b.lp(l).coedges {
                        let (ta, tb) = b.coedge_params(c);
                        let e = b.edge(b.coedge(c).edge);
                        for k in 0..400 {
                            let q = e.curve.eval(ta + (tb - ta) * k as f64 / 400.0);
                            pts.push([q.x, q.y, q.z]);
                        }
                    }
                }
                let o = frame.o;
                let c0 = [o.x, o.y, o.z];
                let mut sub = 0.0;
                for i in 0..pts.len() {
                    for c in clip([c0, pts[i], pts[(i + 1) % pts.len()]]) {
                        sub += vol(c);
                    }
                }
                eprintln!("face {} plane raw {sub:.3} sense {}", b.face_id_str(f), face.sense);
                total += sub;
            }
            _ => {}
        }
    }
    eprintln!("CLIPVOL {total:.3} (signed, about y = {refy})");
}
