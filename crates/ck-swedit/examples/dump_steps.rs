//! Prints the history steps of `.sldprt` files as read back (kind, name, status, note), one line per step:
//!
//!     cargo run --release -p ck-swedit --example dump_steps -- <file> [<file> ...]
use ck_swedit::{rebuild_history, Tagged};

fn main() {
    for a in std::env::args().skip(1) {
        let Ok(bytes) = std::fs::read(&a) else { continue };
        let Ok(imp) = ck_sldprt::import_sldprt(&bytes) else { continue };
        let t = Tagged::from_import(&imp);
        let out = rebuild_history(&t);
        let mp = ck_topo::mass::mass_props(&imp.body);
        println!("stored: volume {:.3} faces {}", mp.volume, imp.body.num_faces());
        if let Some(b) = &out.body {
            let mr = ck_topo::mass::mass_props(b);
            println!("rebuilt: volume {:.3} faces {}", mr.volume, b.num_faces());
            if std::env::var("DUMP_FACES").is_ok() {
                use ck_recog::model::Model;
                let (mo, mn) = (Model::new(&imp.body), Model::new(b));
                let mut free: Vec<_> = mn.faces.iter().map(|&f| (f, mn.area(f))).collect();
                for &f in &mo.faces {
                    let a = mo.area(f);
                    match free.iter().position(|&(g, ga)| (a - ga).abs() <= 1e-6 * a.abs().max(ga.abs()) && ck_swedit::fastverify::same_surface(mo.kind(f), mn.kind(g), 1e-6)) {
                        Some(i) => {
                            free.swap_remove(i);
                        }
                        None => println!("  stored unmatched {} area {a:.4} {:?}", mo.b.face_id_str(f), mo.kind(f)),
                    }
                }
                for (g, ga) in free {
                    println!("  rebuilt unmatched {} area {ga:.4} {:?}", mn.b.face_id_str(g), mn.kind(g));
                }
            }
        }
        println!("{a}: prefix {} fallback {:?}", out.prefix, out.fallback);
        for (i, c) in out.checks.iter().enumerate() {
            println!("  check {i}: {c:?}");
        }
        for (i, s) in out.steps.iter().enumerate() {
            if std::env::var("DUMP_JSON").is_ok() {
                if let Some(&(x, y)) = out.ranges.get(i) {
                    for f in &out.features[x.min(out.features.len())..y.min(out.features.len())] {
                        println!("        {}", f.to_string().chars().take(1500).collect::<String>());
                    }
                }
            }
            println!("  {i:3} {:?} {} [{}] {}", s.status, s.kind, s.name, s.note.chars().take(300).collect::<String>());
        }
    }
}
