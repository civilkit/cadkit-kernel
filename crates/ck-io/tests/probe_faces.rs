//! Debug aid (ignored): dump the faces / loops / pcurve end points of an imported STEP file,
//! its tessellation failures and validate issues.
//!   STEP=<file.step> [SHORT=1] [CK_TESS_DBG=1] cargo test -p ck-io --profile fast --test probe_faces -- --ignored --nocapture
#[test]
#[ignore]
fn probe() {
    let p = std::env::var("STEP").unwrap();
    let r = ck_io::step::read::read_step(&std::fs::read_to_string(&p).unwrap()).unwrap();
    let b = &r.body;
    for (fi, f) in b.faces.iter().enumerate() {
        let fid = ck_topo::body::FaceId(fi as u32);
        let mo = ck_topo::mass::faces_moments(b, &[fid], ck_geom::Vec3::ZERO);
        println!("face {fi} {} sense={} area {:.4} vol {:.4} {:?}", f.name.id(), f.sense, mo.area, mo.volume, f.surface);
        for l in &f.loops {
            for c in &b.lp(*l).coedges {
                let ce = b.coedge(*c);
                let e = b.edge(ce.edge);
                let (a, z) = if ce.rev { (e.t1, e.t0) } else { (e.t0, e.t1) };
                let uv = |t: f64| ce.pcurve.as_ref().map(|p| p.eval(t));
                let short = std::env::var("SHORT").is_ok();
                if short {
                    println!("    e{} rev={} v{}->v{} uv {:?} -> {:?}", ce.edge.0, ce.rev, e.v0.0, e.v1.0, uv(a), uv(z));
                } else {
                    println!("    e{:?} rev={} v{:?}->v{:?} t[{:.4},{:.4}] pc={:?}", ce.edge, ce.rev, e.v0, e.v1, e.t0, e.t1, ce.pcurve);
                }
            }
        }
    }
    let t = ck_tess::mesh::tessellate(b);
    println!("tris {} failed {:?}", t.mesh.tri_face.len(), t.mesh.failed);
    let m = ck_topo::mass::mass_props(b);
    println!("vol {} issues {:?}", m.volume, ck_check::validate(b, 1e-6));
}
