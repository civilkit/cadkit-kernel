//! Streams, Parasolid transmits and the bodies each holds (no files are part of the repository):
//!
//!     cargo run --release -p ck-sldprt --example list_bodies -- <file.sldprt> [...]
use ck_sldprt::{container, ir, xt, Limits};

fn main() {
    let limits = Limits::default();
    for a in std::env::args().skip(1) {
        println!("=== {}", std::path::Path::new(&a).file_name().unwrap().to_string_lossy());
        let bytes = std::fs::read(&a).unwrap();
        let c = match container::open(&bytes, &limits) { Ok(c) => c, Err(e) => { println!("open: {e:?}"); continue } };
        println!("container {:?}, {} streams, damaged {:?}", c.kind, c.streams.len(), c.damaged.iter().map(|d| (&d.name, &d.why)).collect::<Vec<_>>());
        for s in &c.streams {
            println!("  stream {:>9}  {}", s.data.len(), s.name);
        }
        let m = ck_sldprt::meta::read(&c, &limits);
        for f in &m.features {
            println!("  feature {} {} {}", f.id, f.kind, f.name);
        }
        println!("stored mass: {:?}", m.mass.as_ref().map(|m| m.volume_m3 * 1e9));
        let ts = match c.transmits(&limits) { Ok(t) => t, Err(e) => { println!("transmits: {e:?}"); continue } };
        for t in &ts {
            let head: String = t.data.iter().take(120).map(|&b| if (32..127).contains(&b) { b as char } else { '.' }).collect();
            println!("  transmit {:?} in {} ({} bytes) name `{}`\n     head {head}", t.kind, t.stream, t.data.len(), t.name);
            match xt::read_xt(&t.data, &limits).and_then(|x| { let n = x.nodes.len(); ir::extract(&x, &limits).map(|i| (i, n)) }) {
                Ok((i, n)) => {
                    println!("     {n} nodes, schema {}, solid faces {}, sheet faces {}", i.schema, i.solid_faces(), i.sheet_faces());
                    if i.solid_faces() > 0 {
                        match ck_sldprt::build::build(&[&i], &limits) {
                            Ok(bt) => {
                                let f: Vec<_> = bt.body.face_ids().collect();
                                println!("     built: {} faces, volume {:.3} mm3", f.len(), ck_topo::mass::signed_volume(&bt.body, &f));
                            }
                            Err(e) => println!("     build: {e:?}"),
                        }
                    }
                    for b in &i.bodies {
                        let faces: usize = b.regions.iter().flat_map(|r| &r.shells).map(|s| s.faces.len()).sum();
                        println!("     body {} {:?} regions {} (solid {}) faces {} last_feature {:?}", b.id, b.kind, b.regions.len(), b.regions.iter().filter(|r| r.solid).count(), faces, b.last_feature);
                    }
                }
                Err(e) => println!("     read: {e:?}"),
            }
        }
    }
}
