mod bodies;
#[test]
#[ignore]
fn dump() {
    let name = std::env::var("B").unwrap();
    let b = bodies::build(&name);
    for (fi, f) in b.faces.iter().enumerate() {
        println!("face {fi} {} sense={} {:?}", f.name.id(), f.sense, f.surface);
        for l in &f.loops {
            println!("  loop {:?}", l);
            for c in &b.lp(*l).coedges {
                let ce = b.coedge(*c); let e = b.edge(ce.edge);
                println!("    ce{:?} e{:?} rev={} v{:?}->v{:?} t[{},{}] {:?} ncoedges={}", c, ce.edge, ce.rev, e.v0, e.v1, e.t0, e.t1, e.curve, e.coedges.len());
            }
        }
    }
    for (i, v) in b.vertices.iter().enumerate() { println!("v{i} {:?}", v.p); }
}
#[test]
#[ignore]
fn dump_lumps() {
    let b = bodies::build(&std::env::var("B").unwrap());
    for (i, l) in b.lumps.iter().enumerate() { println!("lump {i} {:?}", l); }
    for (i, s) in b.shells.iter().enumerate() { println!("shell {i} {:?}", s); }
    for f in &b.faces { println!("{} sense {}", f.name.id(), f.sense); }
}
/// `OUT=<dir> cargo test -p ck-io --test dump write_ours -- --ignored`
#[test]
#[ignore]
fn write_ours() {
    let dir = std::env::var("OUT").unwrap();
    for n in bodies::NAMES {
        let s = ck_io::step::write_step_signed_zeros(&bodies::build(n), n, &ck_io::step::StepOpts { unix_ms: 0.0 });
        std::fs::write(format!("{dir}/{n}.step"), s).unwrap();
    }
}
