//! `cargo run -p ck-skp --release --example skp_time -- <file.skp>...`
//!
//! Phase timings of the importer pipeline (parse, bake, body build, tessellate), for profiling slow files.

use ck_skp::{bake, body::{build_body, BodyOptions}, parse, BakeOptions, Limits};
use std::time::Instant;

fn main() {
    for f in std::env::args().skip(1) {
        let data = std::fs::read(&f).expect("read");
        let lim = Limits::default();
        let t = Instant::now();
        let m = match parse(&data) {
            Ok(m) => m,
            Err(e) => {
                println!("{f}: parse error {e}");
                continue;
            }
        };
        let t_parse = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let opt = BakeOptions { keep_polygons: true, ..Default::default() };
        let s = match bake(&m, &lim, &opt) {
            Ok(s) => s,
            Err(e) => {
                println!("{f}: bake error {e}");
                continue;
            }
        };
        let t_bake = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let sb = match build_body(&s, &BodyOptions::default()) {
            Ok(b) => b,
            Err(e) => {
                println!("{f}: body error {e}");
                continue;
            }
        };
        let t_body = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let tess = ck_tess::mesh::tessellate(&sb.body);
        let t_tess = t.elapsed().as_secs_f64();
        let _ = tess;
        if std::env::var("DUMP").is_ok() {
            for (i, p) in s.polygons.iter().enumerate().take(6) {
                let lens: Vec<usize> = p.loops.iter().map(|l| l.len()).collect();
                let mut lo = [f64::MAX; 3]; let mut hi = [f64::MIN; 3];
                for q in &p.loops[0] { for k in 0..3 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } }
                println!("poly {i} loops {lens:?} n {:?} lo {lo:?} hi {hi:?}", p.normal);
            }
            if let Ok(i) = std::env::var("DUMP").unwrap().parse::<usize>() { let p = &s.polygons[i]; println!("poly {i}: {:?} {:?}", p.loops, p.normal); }
            let (l, h) = sb.body.bbox();
            println!("body bbox {l:?} {h:?} tol {:?}", ck_tess::mesh::body_tol(&sb.body));
        }
        println!("{f}: faces {} tris {} parse {t_parse:.3}s bake {t_bake:.3}s body {t_body:.3}s tess {t_tess:.3}s", sb.body.num_faces(), s.triangle_count());
    }
}
