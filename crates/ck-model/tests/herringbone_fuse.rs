//! A herringbone gear is two opposite-twist extrudes of one gear profile that meet in a coplanar mid-plane face and are
//! fused. 48 teeth (module 2, 20 degrees pressure angle, helix 30, half width 5, so each half twists 2.98 degrees).
//! The profile is built here like the app's gear editor does: a tip arc per tooth and one spline down the upper flank,
//! across the root and up the next tooth's lower flank.
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

/// Sketch JSON of a helical gear's transverse outline as the app's gear editor draws it: per tooth one tip arc and one spline
/// that runs down the upper flank, through the root fillet and across the root circle, and up the next tooth's lower flank
/// (some 25 points, so a long run of cubic spans); `z` teeth of normal module `m`, helix angle `beta_deg`, root fillet `0.3 m`.
fn gear_sketch(z: usize, m: f64, pa_deg: f64, beta_deg: f64) -> String {
    use std::f64::consts::{PI, TAU};
    let beta = beta_deg.to_radians();
    let alpha_t = (pa_deg.to_radians().tan() / beta.cos()).atan();
    let mt = m / beta.cos();
    let r = mt * z as f64 / 2.0;
    let rb = r * alpha_t.cos();
    let (ra, r_in) = (r + m, r - 1.25 * m);
    let fillet = 0.3 * m;
    let c0 = (PI / 2.0 * mt) / (2.0 * r) + (alpha_t.tan() - alpha_t);
    let t_of = |rho: f64| ((rho / rb).powi(2) - 1.0).max(0.0).sqrt();
    let ang = |t: f64| c0 - (t - t.atan());
    let rho_of = |t: f64| rb * (1.0 + t * t).sqrt();
    let pos = |t: f64| (rho_of(t) * ang(t).cos(), rho_of(t) * ang(t).sin());
    let norm = |t: f64| {
        let h = 1e-5;
        let (p1, p0) = (pos(t + h), pos((t - h).max(0.0)));
        let (tx, ty) = (p1.0 - p0.0, p1.1 - p0.1);
        let l = tx.hypot(ty);
        (-ty / l, tx / l)
    };
    let (t0, t1) = (t_of(r_in), t_of(ra));
    // the fillet circle touches the root circle and the flank: bisect for its tangent point on the flank
    let f = |t: f64| {
        let (p, n) = (pos(t), norm(t));
        (p.0 + fillet * n.0).hypot(p.1 + fillet * n.1) - (r_in + fillet)
    };
    let (mut lo, mut hi) = (t0, t1.min(t0 + 2.5));
    assert!(f(lo) * f(hi) <= 0.0);
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let ts = 0.5 * (lo + hi);
    let (p, n) = (pos(ts), norm(ts));
    let (cx, cy) = (p.0 + fillet * n.0, p.1 + fillet * n.1);
    let a1 = (-cy).atan2(-cx);
    let mut da = (-n.1).atan2(-n.0) - a1;
    while da > PI {
        da -= TAU;
    }
    while da < -PI {
        da += TAU;
    }
    let mut plan: Vec<(f64, f64)> = (0..4)
        .map(|i| {
            let a = a1 + da * i as f64 / 4.0;
            let (x, y) = (cx + fillet * a.cos(), cy + fillet * a.sin());
            (x.hypot(y), y.atan2(x))
        })
        .collect();
    let n_flank = 6;
    plan.extend((0..=n_flank).map(|i| {
        let t = ts + (t1 - ts) * i as f64 / n_flank as f64;
        (rho_of(t), ang(t))
    }));
    let tau = TAU / z as f64;
    let root_arc = tau - 2.0 * plan[0].1;
    let cnt = (((root_arc * plan[0].0) / (0.45 * m)).ceil() as i64 - 1).clamp(0, 6) as usize;
    let mut pts: Vec<(f64, f64)> = vec![(0.0, 0.0)];
    let mut push = |rho: f64, a: f64, pts: &mut Vec<(f64, f64)>| -> usize {
        pts.push((rho * a.cos(), rho * a.sin()));
        pts.len()
    };
    let (mut low, mut up) = (vec![], vec![]);
    for k in 0..z {
        let phi = k as f64 * tau;
        low.push(plan.iter().map(|&(rho, a)| push(rho, phi - a, &mut pts)).collect::<Vec<_>>());
        up.push(plan.iter().map(|&(rho, a)| push(rho, phi + a, &mut pts)).collect::<Vec<_>>());
    }
    let mut ents = vec![];
    for k in 0..z {
        let nx = (k + 1) % z;
        let (tl, tu) = (*low[k].last().unwrap(), *up[k].last().unwrap());
        ents.push(format!(r#"{{"type":"arc","id":"e{}","c":"p1","p1":"p{tl}","p2":"p{tu}"}}"#, ents.len() + 1));
        let mut ids: Vec<usize> = up[k].iter().rev().copied().collect();
        for i in 1..=cnt {
            ids.push(push(plan[0].0, k as f64 * tau + plan[0].1 + root_arc * i as f64 / (cnt + 1) as f64, &mut pts));
        }
        ids.extend(low[nx].iter().copied());
        let ids: Vec<String> = ids.iter().map(|i| format!("\"p{i}\"")).collect();
        ents.push(format!(r#"{{"type":"spline","id":"e{}","pts":[{}]}}"#, ents.len() + 1, ids.join(",")));
    }
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{}","x":{:.9},"y":{:.9}}}"#, i + 1, p.0, p.1)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}}"#,
        points.join(","),
        ents.join(",")
    )
}

fn regen_src(src: &str) -> (Vec<String>, Option<Body>, f64) {
    // a clock without a limit: only makes the CK_BOOL_DBG stage timers read real time
    static T0: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
    T0.get_or_init(std::time::Instant::now);
    ck_core::budget::set_clock(|| T0.get().unwrap().elapsed().as_secs_f64() * 1e3);
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let t = std::time::Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let ms = t.elapsed().as_secs_f64() * 1e3;
    (r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect(), ex.0.body.take(), ms)
}

fn volume(b: &Body) -> f64 {
    let faces: Vec<_> = b.face_ids().collect();
    ck_topo::mass::signed_volume(b, &faces)
}

/// Half width 5, so each half twists (w/2) tan(beta) / r = 2.98 degrees.
const HALF: f64 = 5.0;

fn part_src(sketch: &str, feats: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{sketch}{feats}]}}"#)
}

#[test]
fn herringbone_halves_fuse_at_the_mid_plane() {
    let z = 48;
    let sketch = gear_sketch(z, 2.0, 20.0, 30.0);
    let r = 2.0 * z as f64 / 2.0 / 30f64.to_radians().cos();
    let tw = HALF * 30f64.to_radians().tan() / r * 180.0 / std::f64::consts::PI;

    // The profile area, measured on an untwisted extrude (planar caps, exact boundary integrals). A screw keeps area, so a
    // twisted half has the volume area * HALF, and the herringbone twice that. (The face integrals of a screw patch whose
    // profile is a spline with a couple of dozen spans used to come out 0.3 percent short, which made the fused result
    // look like it had gained material: booleanVolumeMismatch.)
    let (e, plain, _) = regen_src(&part_src(&sketch, &format!(r#",{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{HALF}}}"#)));
    assert!(e.is_empty(), "{e:?}");
    let area_depth = volume(&plain.unwrap());

    let f2 = format!(r#",{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{HALF},"twist":{tw},"twistCentre":[0,0]}}"#);
    let f3 = format!(r#",{{"type":"extrude","id":"F3","op":"add","sketch":"F1","end":"blind","depth":{HALF},"flip":true,"twist":-{tw},"twistCentre":[0,0]}}"#);
    let (e, half, ms) = regen_src(&part_src(&sketch, &f2));
    assert!(e.is_empty(), "{e:?}");
    let vh = volume(&half.unwrap());
    eprintln!("twisted half {vh:.3} vs area*depth {area_depth:.3} ({:.2e}), {ms:.0} ms", (vh - area_depth) / area_depth);
    assert!((vh - area_depth).abs() < 1e-4 * area_depth, "twisted half volume {vh} vs {area_depth}");

    let (e, body, ms) = regen_src(&part_src(&sketch, &format!("{f2}{f3}")));
    eprintln!("herringbone {z}T: {ms:.0} ms, errors {e:?}");
    assert!(e.is_empty(), "{e:?}");
    let b = body.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    assert!(issues.is_empty(), "{issues:?}");
    let (got, want) = (volume(&b), 2.0 * area_depth);
    eprintln!("fused volume {got:.3} expected {want:.3} ({:.2e})", (got - want) / want);
    assert!((got - want).abs() < 1e-4 * want, "volume {got} vs {want}");
}
