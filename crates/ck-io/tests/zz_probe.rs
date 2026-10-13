#[test]
#[ignore]
fn probe() {
    let p = std::env::var("STEP").unwrap();
    let r = ck_io::step::read::read_step(&std::fs::read_to_string(&p).unwrap()).unwrap();
    let b = &r.body;
    for (fi, f) in b.faces.iter().enumerate() {
        println!("face {fi} {} sense={} {:?}", f.name.id(), f.sense, f.surface);
        for l in &f.loops {
            for c in &b.lp(*l).coedges {
                let ce = b.coedge(*c);
                let e = b.edge(ce.edge);
                println!("    e{:?} rev={} v{:?}->v{:?} t[{:.4},{:.4}] pc={:?}", ce.edge, ce.rev, e.v0, e.v1, e.t0, e.t1, ce.pcurve);
            }
        }
    }
    let m = ck_topo::mass::mass_props(b);
    println!("vol {} issues {:?}", m.volume, ck_check::validate(b, 1e-6));
}
