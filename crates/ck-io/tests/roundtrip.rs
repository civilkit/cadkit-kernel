//! Round trip: our own STEP export -> import. Volume / area within 1e-6,
//! face counts equal, `ck_check::validate` reports nothing.
mod bodies;
use ck_io::step::{read_step, write_step, StepOpts};
use ck_topo::body::Body;

pub fn stats(b: &Body) -> (f64, f64, usize, usize) {
    let m = ck_topo::mass::mass_props(b);
    (m.volume, m.area, b.num_faces(), b.edge_ids().filter(|&e| !b.is_seam(e)).count())
}

pub fn roundtrip(name: &str, b: &Body) -> Result<String, String> {
    let mut b0 = b.clone();
    ck_topo::region::rebuild_lumps(&mut b0);
    let text = write_step(&b0, name, &StepOpts::default());
    let imp = read_step(&text).map_err(|e| format!("{name}: import: {e}"))?;
    let (v0, a0, f0, e0) = stats(&b0);
    let (v1, a1, f1, e1) = stats(&imp.body);
    let rel = |x: f64, y: f64| (x - y).abs() / (1.0 + y.abs());
    let issues = ck_check::validate(&imp.body, 1e-6);
    let topo = ck_topo::check::check(&imp.body, 1e-6);
    let mut errs = vec![];
    if rel(v1, v0) > 1e-6 {
        errs.push(format!("volume {v1} vs {v0}"));
    }
    if rel(a1, a0) > 1e-6 {
        errs.push(format!("area {a1} vs {a0}"));
    }
    if f1 != f0 {
        errs.push(format!("faces {f1} vs {f0}"));
    }
    if e1 != e0 {
        errs.push(format!("edges {e1} vs {e0}"));
    }
    if !issues.is_empty() {
        errs.push(format!("validate: {:?}", &issues[..issues.len().min(3)]));
    }
    if !topo.is_empty() {
        errs.push(format!("check: {:?}", &topo[..topo.len().min(3)]));
    }
    if errs.is_empty() {
        Ok(format!("{name}: v={v1:.6} a={a1:.6} f={f1} e={e1}"))
    } else {
        Err(format!("{name}: {}", errs.join("; ")))
    }
}

/// Debug: `STEP=<file> cargo test -p ck-io --test roundtrip dump_file -- --ignored --nocapture`
#[test]
#[ignore]
fn dump_file() {
    let text = std::fs::read_to_string(std::env::var("STEP").unwrap()).unwrap();
    let imp = read_step(&text).unwrap();
    let b = &imp.body;
    for f in b.face_ids() {
        let fa = b.face(f);
        let area = ck_topo::mass::face_integral::<1>(b, f, 1, &|_, n| [n.len()])[0];
        let vol = ck_topo::mass::signed_volume(b, &[f]);
        let loops: Vec<usize> = fa.loops.iter().map(|&l| b.lp(l).coedges.len()).collect();
        println!("{} {} sense={} loops={:?} area={:.6} vol3={:.6}", b.face_id_str(f), fa.surface.kind(), fa.sense, loops, area, vol);
    }
    for e in b.edge_ids() {
        let cs = b.edge_coedges(e);
        if cs.len() != 2 || std::env::var("ALL").is_ok() {
            let ed = b.edge(e);
            println!(
                "edge{} {} t[{:.4},{:.4}] v{}->v{} co={:?}",
                e.idx(),
                ed.curve.kind(),
                ed.t0,
                ed.t1,
                ed.v0.idx(),
                ed.v1.idx(),
                cs.iter().map(|&c| (b.face_id_str(b.coedge_face(c)), b.coedge(c).rev)).collect::<Vec<_>>()
            );
        }
    }
    println!("{:?}", ck_check::validate(b, 1e-6));
}

#[test]
fn golden_bodies_roundtrip() {
    let mut fails = vec![];
    for n in bodies::NAMES {
        match roundtrip(n, &bodies::build(n)) {
            Ok(s) => println!("ok   {s}"),
            Err(e) => fails.push(e),
        }
    }
    assert!(fails.is_empty(), "{} failures:\n{}", fails.len(), fails.join("\n"));
}
