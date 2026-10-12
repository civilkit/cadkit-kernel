//! Diagnostic (ignored): `STEP=path cargo test -p ck-io --profile fast --test step_probe -- --ignored --nocapture`
//! prints per-face area / signed volume of an imported STEP.
#[test]
#[ignore]
fn step_probe() {
    let path = std::env::var("STEP").unwrap();
    let t0 = std::time::Instant::now();
    let r = ck_io::step::read::read_step(&std::fs::read_to_string(path).unwrap()).unwrap();
    let b = &r.body;
    eprintln!("read {:?}", t0.elapsed());
    eprintln!("solids {} faces {} warnings {:?}", r.solids, b.num_faces(), r.warnings.len());
    for f in b.face_ids() {
        let ar = ck_topo::mass::face_integral::<1>(b, f, 1, &|_, n| [n.len()])[0];
        let vol = ck_topo::mass::face_integral::<1>(b, f, 1, &|p, n| [p.dot(n) / 3.0])[0];
        eprintln!("face {} {} sense {} loops {:?} area {:.3} vol {:.3}", b.face_id_str(f), b.face(f).surface.kind(), b.face(f).sense, b.face(f).loops.iter().map(|&l| b.lp(l).coedges.len()).collect::<Vec<_>>(), ar, vol);
    }
    if let Ok(name) = std::env::var("DUMPFACE") {
        let f = b.find_face(&name).unwrap();
        eprintln!("surface periods {:?}", b.face(f).surface.periods());
        for &l in &b.face(f).loops {
            for &c in &b.lp(l).coedges {
                let (ta, tb) = b.coedge_params(c);
                match &b.coedge(c).pcurve {
                    Some(pc) => eprintln!("  edge {:?} rev {} t[{:.3},{:.3}] uv {:?} -> {:?}", b.coedge(c).edge, b.coedge(c).rev, ta, tb, pc.eval(ta), pc.eval(tb)),
                    None => eprintln!("  no pcurve"),
                }
            }
        }
    }
    let t1 = std::time::Instant::now();
    let t = ck_tess::mesh::tessellate(b);
    eprintln!("tessellate {:?}", t1.elapsed());
    let mut per = std::collections::BTreeMap::new();
    for &f in &t.mesh.tri_face {
        *per.entry(f).or_insert(0) += 1;
    }
    eprintln!("tris per face {:?}; bbox {:?}", per, t.mesh.bbox());
    for (i, m) in ck_topo::mass::lump_moments(b).iter().enumerate() {
        eprintln!("lump {i} volume {:.3} area {:.3}", m.volume, m.area);
    }
}
