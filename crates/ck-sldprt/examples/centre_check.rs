//! How the centre index an arc or circle record stores compares with the centre the search chose (no files are part of the repository):
//!
//!     cargo run --release -p ck-sldprt --example centre_check -- <file.sldprt> [...]
use ck_sldprt::{container, sketch, sketch::SkCurve, Limits};

fn main() {
    let limits = Limits::default();
    let mut tot = [0usize; 8];
    let (mut sk_all, mut sk_exact, mut unc_arc, mut unc_circ) = (0usize, 0usize, 0usize, 0usize);
    for a in std::env::args().skip(1) {
        let bytes = std::fs::read(&a).unwrap();
        let Ok(c) = container::open(&bytes, &limits) else { continue };
        let Ok(set) = sketch::read(&c, &limits) else { continue };
        let mut radii: Vec<f64> = vec![];
        if let Ok(imp) = ck_sldprt::import_sldprt(&bytes) {
            for f in imp.body.face_ids() {
                match &imp.body.face(f).surface {
                    ck_geom::surface::Surface::Cylinder { r, .. } | ck_geom::surface::Surface::Sphere { r, .. } | ck_geom::surface::Surface::Cone { r, .. } => radii.push(*r),
                    ck_geom::surface::Surface::Torus { major, minor, .. } => {
                        radii.push(*major);
                        radii.push(*minor);
                    }
                    _ => {}
                }
            }
        }
        let in_body = |r: f64| radii.iter().any(|x| (x - r * 1e3).abs() < 1e-6 * (r * 1e3).max(1.0));
        let mut row = [0usize; 8]; // arcs: no stored, stored = chosen, stored differs, stored equidistant; circles: stored = chosen, differs, no stored, stored not free
        for s in &set.sketches {
            sk_all += 1;
            sk_exact += s.exact() as usize;
            if !s.exact() && std::env::var("W").is_ok() {
                let unres = s.curves.iter().filter(|c| matches!(c, SkCurve::Unresolved { .. })).count();
                let ua = s.curves.iter().filter(|c| matches!(c, SkCurve::Arc { centre_certain: false, .. })).count();
                let uc = s.curves.iter().filter(|c| matches!(c, SkCurve::Circle { centre_certain: false, .. })).count();
                println!("WHY unresolved {} uncertain arcs {} circles {} notes {:?}", unres > 0, ua > 0, uc > 0, s.notes.iter().map(|n| n.chars().take(40).collect::<String>()).collect::<Vec<_>>());
            }
            for c in &s.curves {
                match c {
                    SkCurve::Arc { centre_certain: false, construction: false, .. } => unc_arc += 1,
                    SkCurve::Circle { centre_certain: false, construction: false, .. } => unc_circ += 1,
                    _ => {}
                }
            }
            let n = s.points.len();
            let d = |i: usize, j: usize| (s.points[i].x - s.points[j].x).hypot(s.points[i].y - s.points[j].y);
            for c in &s.curves {
                match c {
                    SkCurve::Arc { start, end, centre, centre_certain, stored, .. } => {
                        match stored {
                            None => row[0] += 1,
                            Some(k) if *k >= n => row[2] += 1,
                            Some(k) => {
                                if (d(*k, *start) - d(*k, *end)).abs() <= 1e-9 * d(*start, *end).max(1e-3) { row[3] += 1 }
                                if std::env::var("B").is_ok() {
                                    let all: Vec<usize> = (0..n).filter(|&q| q != *start && q != *end && s.points[q].located && (d(q, *start) - d(q, *end)).abs() <= 1e-9 * d(*start, *end).max(1e-3)).collect();
                                    println!("  ARC {} {start}-{end} stored {k} r {:.4} in body {} | candidates {:?}", s.name, d(*k, *start) * 1e3, in_body(d(*k, *start)), all.iter().map(|&q| (q, (d(q, *start) * 1e3 * 1e4).round() / 1e4, in_body(d(q, *start)))).collect::<Vec<_>>());
                                }
                                if Some(*k) == *centre { row[1] += 1 } else {
                                    row[2] += 1;
                                    if std::env::var("V").is_ok() { println!("  {} arc {start}-{end} stored {k} chosen {centre:?} certain {centre_certain} equid {}", s.name, (d(*k, *start) - d(*k, *end)).abs() <= 1e-9 * d(*start, *end).max(1e-3)); }
                                }
                            }
                        }
                    }
                    SkCurve::Circle { rim, centre, centre_certain, stored, .. } => match stored {
                        Some(k) if *k < n && Some(*k) == *centre => row[4] += 1 + 0 * (*centre_certain as usize),
                        Some(k) if *k < n => {
                            row[5] += 1;
                            if std::env::var("V").is_ok() {
                                let hit = |r: f64| s.params.iter().any(|p| (p.value - r).abs() < 1e-9 || (p.value - 2.0 * r).abs() < 1e-9);
                                let rs = d(*k, *rim);
                                let rc = centre.map(|c| d(c, *rim));
                                println!("  {} circle {rim} stored {k} (role {:?} on {} r={:.4} dim {}) chosen {centre:?} (r={:?} dim {}) certain {centre_certain}", s.name, s.points[*k].role, s.points[*k].on.len(), rs * 1e3, hit(rs), rc.map(|r| r * 1e3), rc.map_or(false, hit));
                            }
                        }
                        Some(_) => row[7] += 1,
                        None => row[6] += 1,
                    },
                    _ => {}
                }
            }
        }
        println!("{}: arcs [no stored {}, = chosen {}, differs {}, stored equidistant {}] circles [= chosen {}, differs {}, no stored {}, out of range {}]", std::path::Path::new(&a).parent().unwrap().file_name().unwrap().to_string_lossy(), row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]);
        for i in 0..8 { tot[i] += row[i]; }
    }
    println!("total: {tot:?}");
    println!("sketches {sk_all}, exact {sk_exact}, uncertain arcs {unc_arc}, uncertain circles {unc_circ}");
}
