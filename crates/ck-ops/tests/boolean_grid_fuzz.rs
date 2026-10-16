//! Random add / cut sequences of integer-grid boxes: faces, edges and corners coincide all the time and the exact
//! volume is the number of unit cells. Every sequence must give a valid body of exactly that volume (or a
//! structured "nothing changed" outcome), never `booleanFailed` or a wrong volume.
use ck_geom::{Frame, Vec2, Vec3};
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::check::check;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::collections::HashSet;

fn top() -> Frame {
    Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}

fn cell_box(feature: &str, b: [i32; 6]) -> Body {
    let (x0, y0, z0, x1, y1, z1) = (b[0] as f64, b[1] as f64, b[2] as f64, b[3] as f64, b[4] as f64, b[5] as f64);
    let p = [Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude(feature, &prof, ExtrudeOpts { s0: z0, s1: z1, dir: None, draft: 0.0 }).unwrap()
}

struct Lcg(u32);
impl Lcg {
    fn next(&mut self) -> f64 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        self.0 as f64 / 4294967296.0
    }
    fn int(&mut self, lo: i32, hi: i32) -> i32 {
        lo + (self.next() * (hi - lo + 1) as f64).floor() as i32
    }
}

fn run(seed: u32, n_ops: usize, grid: i32) -> Result<(), String> {
    let mut rnd = Lcg(seed);
    let mut cells: HashSet<(i32, i32, i32)> = HashSet::new();
    let mut body: Option<Body> = None;
    let mut log = vec![];
    for k in 0..n_ops {
        let (x0, y0, z0) = (rnd.int(0, grid - 1), rnd.int(0, grid - 1), rnd.int(0, grid - 1));
        let b = [x0, y0, z0, rnd.int(x0 + 1, grid), rnd.int(y0 + 1, grid), rnd.int(z0 + 1, grid)];
        let is_cut = k > 0 && rnd.next() < 0.45;
        log.push((is_cut, b));
        for x in b[0]..b[3] {
            for y in b[1]..b[4] {
                for z in b[2]..b[5] {
                    if is_cut {
                        cells.remove(&(x, y, z));
                    } else {
                        cells.insert((x, y, z));
                    }
                }
            }
        }
        let tool = cell_box(&format!("F{}", 2 * k + 4), b);
        body = match (body.take(), is_cut) {
            (None, true) => None,
            (None, false) => Some(tool),
            (Some(cur), cut) => {
                let r = boolean(&cur, &tool, if cut { Op::Cut } else { Op::Union }, 1e-6).map_err(|e| format!("{e} at op {k}"))?;
                if r.mismatch.is_some() {
                    return Err(format!("sanity mismatch at op {k}"));
                }
                if r.no_effect {
                    Some(cur)
                } else {
                    Some(r.body)
                }
            }
        };
        if body.as_ref().is_some_and(|b| b.num_faces() == 0) {
            body = None;
        }
        if let Some(b) = &body {
            let issues = check(b, 1e-6);
            if !issues.is_empty() {
                return Err(format!("invalid after op {k} {issues:?} for {log:?}"));
            }
        }
    }
    let got = body.as_ref().map_or(0.0, |b| ck_topo::mass::mass_props(b).volume);
    if (got - cells.len() as f64).abs() > 1e-6 {
        return Err(format!("volume {got} want {} for {log:?}", cells.len()));
    }
    if let Some(b) = &body {
        let issues = check(b, 1e-6);
        if !issues.is_empty() {
            return Err(format!("invalid {issues:?} for {log:?}"));
        }
    }
    Ok(())
}

#[test]
fn grid_boxes_add_and_cut_exactly() {
    let mut bad = vec![];
    for seed in 1..=300u32 {
        for (ops, grid) in [(4, 5), (6, 4)] {
            if let Err(e) = run(seed.wrapping_mul(2654435761), ops, grid) {
                bad.push(format!("seed {seed} ops {ops} grid {grid}: {e}"));
            }
        }
    }
    assert!(bad.is_empty(), "{} of 600 failed:\n{}", bad.len(), bad.join("\n"));
}
