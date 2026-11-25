//! Research aid: for each sketch on a standard plane, how many of its points land on a body vertex (projected along the plane
//! normal) under each of the eight in-plane axis assignments.
//!
//!     cargo run --release -p ck-sldprt --example cal_frames -- <file.sldprt> [...]
use ck_sldprt::sketch::{self, PlaneRef, PointRole};
use ck_sldprt::{container, import_sldprt, Limits};

fn main() {
    let limits = Limits::default();
    for a in std::env::args().skip(1) {
        let bytes = std::fs::read(&a).unwrap();
        let Ok(c) = container::open(&bytes, &limits) else { continue };
        let Ok(set) = sketch::read(&c, &limits) else { continue };
        let Ok(imp) = import_sldprt(&bytes) else { continue };
        let b = &imp.body;
        let vs: Vec<[f64; 3]> = b.vertex_ids().map(|v| { let p = b.vertex(v).p; [p.x, p.y, p.z] }).collect();
        println!("=== {}", a.rsplit('/').nth(1).unwrap_or(""));
        for s in &set.sketches {
            let ids: Vec<u32> = match s.plane { PlaneRef::Plane(i) => vec![i], PlaneRef::Ambiguous => s.plane_refs.iter().copied().filter(|r| (2..=4).contains(r)).collect(), _ => if std::env::var("ALL").is_ok() { vec![2, 3, 4] } else { vec![] } };
            for id in ids {
            if !(2..=4).contains(&id) { continue; }
            let normal = match id { 2 => 2, 3 => 1, _ => 0 };
            let inp: Vec<usize> = (0..3).filter(|&k| k != normal).collect();
            let pts: Vec<(f64, f64)> = s.points.iter().filter(|p| p.located && p.role == PointRole::Free).map(|p| (p.x * 1e3, p.y * 1e3)).collect();
            if pts.is_empty() { continue; }
            let off = pts.iter().any(|p| p.0.abs() > 1e-6 || p.1.abs() > 1e-6);
            let mut line = format!("sketch {} plane {} pts {} offorigin {} :", s.name, id, pts.len(), off);
            for swap in [false, true] {
                for sx in [1.0, -1.0] {
                    for sy in [1.0, -1.0] {
                        let hit = pts.iter().filter(|p| {
                            let (u, v) = if swap { (sy * p.1, sx * p.0) } else { (sx * p.0, sy * p.1) };
                            vs.iter().any(|w| (w[inp[0]] - u).abs() < 1e-5 && (w[inp[1]] - v).abs() < 1e-5)
                        }).count();
                        line += &format!(" [{}{}{}:{}]", if swap { "yx" } else { "xy" }, if sx > 0.0 { "+" } else { "-" }, if sy > 0.0 { "+" } else { "-" }, hit);
                    }
                }
            }
            println!("{line}");
            }
        }
    }
}
