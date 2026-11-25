//! The sketches and feature parameters read from the feature stream of each file (no files are part of the repository):
//!
//!     cargo run --release -p ck-sldprt --example dump_sketch -- <file.sldprt> [...]
use ck_sldprt::{container, sketch, Limits};

fn main() {
    let limits = Limits::default();
    for a in std::env::args().skip(1) {
        println!("=== {}", std::path::Path::new(&a).file_name().unwrap().to_string_lossy());
        let bytes = std::fs::read(&a).unwrap();
        let c = match container::open(&bytes, &limits) { Ok(c) => c, Err(e) => { println!("open: {e}"); continue } };
        let set = match sketch::read(&c, &limits) { Ok(s) => s, Err(e) => { println!("sketch: {e}"); continue } };
        println!("modern {} features {} sketches {}", set.modern, set.features.len(), set.sketches.len());
        for s in &set.sketches {
            println!("  sketch {} id {} plane {:?} points {} curves {} exact {} notes {:?}", s.name, s.feature_id, s.plane, s.points.len(), s.curves.len(), s.exact(), s.notes);
            if std::env::var("D").is_ok() {
                println!("    frame {:?} stored {:?} refs {:?} frames {}", s.frame().map(|f| (f.x, f.y, f.o)), s.stored_frame.is_some(), s.plane_refs, s.plane_frames.len());
                println!("    params {} dims {}", s.params.len(), s.dims.len());
                for d in &s.dims {
                    println!("    dim {} {:?} {:?} = {}", d.name, d.kind, d.targets, d.value);
                }
            }
            if std::env::var("V").is_ok() {
                for (i, p) in s.points.iter().enumerate() {
                    println!("    p{i} id {} ({:.4}, {:.4}) mm {:?} {:?} {:?}", p.id, p.x * 1e3, p.y * 1e3, p.role, p.counters, p.on);
                }
                for c in &s.curves {
                    println!("    {c:?} r={:?}", s.radius(c).map(|r| r * 1e3));
                }
                for p in &s.params {
                    println!("    param {} = {}", p.name, p.value);
                }
            }
        }
        if std::env::var("E").is_ok() {
            for f in &set.features {
                if f.end_type.is_some() || f.from_type.is_some() {
                    println!("  endspec {} id {} word {:#x} op {} end {:?} from {:?} params {:?}", f.name, f.id, f.word, f.op, f.end_type, f.from_type, f.params.iter().map(|p| (p.name.as_str(), p.value)).collect::<Vec<_>>());
                }
            }
        }
        for f in &set.features {
            if !f.params.is_empty() && set.sketch(f.id).is_none() {
                println!("  feature {} id {} params {:?}", f.name, f.id, f.params.iter().map(|p| (p.name.as_str(), p.value)).collect::<Vec<_>>());
            }
        }
    }
}
