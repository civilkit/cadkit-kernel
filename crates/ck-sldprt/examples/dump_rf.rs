//! Writes every `Config-N-ResolvedFeatures` stream of the given files to `<outdir>/<n>_<stream>.bin` (research aid).
//!
//!     cargo run --release -p ck-sldprt --example dump_rf -- <outdir> <file.sldprt> [...]
use ck_sldprt::{container, Limits};

fn main() {
    let limits = Limits::default();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let out = std::path::PathBuf::from(&args[0]);
    for (n, a) in args[1..].iter().enumerate() {
        let Ok(bytes) = std::fs::read(a) else { continue };
        let Ok(c) = container::open(&bytes, &limits) else { continue };
        for s in &c.streams {
            if s.name.contains("ResolvedFeatures") {
                let nm = s.name.strip_suffix("__ZLB").unwrap_or(&s.name);
                if let Some(d) = c.data(nm, &limits) {
                    let f = out.join(format!("{n:02}_{}.bin", nm.replace('/', "_")));
                    std::fs::write(&f, d).unwrap();
                    println!("{n:02} {} {}", a, f.display());
                }
            }
        }
    }
}
