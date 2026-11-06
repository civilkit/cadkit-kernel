//! Why an imported body is not a valid closed solid (no files are part of the repository):
//!
//!     cargo run --release -p ck-sldprt --example diag_valid -- [-v] <file.sldprt> [...]
//!
//! Per file: faces in the file / built / refused (with the reason), approximated carriers, validator issues by code, and
//! for each free / degenerate / unused edge the faces it touches with their surface kinds; volume and area against the
//! values the file stores. `-v` also lists every face (surface kind, area).

use ck_geom::surface::Surface;
use ck_topo::body::{Body, FaceId};
use std::collections::BTreeMap;

fn kind(s: &Surface) -> String {
    match s {
        Surface::Plane { .. } => "plane".into(),
        Surface::Cylinder { r, .. } => format!("cyl r{r:.3}"),
        Surface::Cone { r, semi_angle, .. } => format!("cone r{r:.3} a{:.2}", semi_angle.to_degrees()),
        Surface::Sphere { r, .. } => format!("sphere r{r:.3}"),
        Surface::Torus { major, minor, .. } => format!("torus {major:.3}/{minor:.3}"),
        Surface::Revolution { .. } => "revolution".into(),
        Surface::Extrusion { .. } => "extrusion".into(),
        Surface::BSpline(b) => format!("bspline {}x{}", b.deg_u, b.deg_v),
        Surface::Screw { .. } => "screw".into(),
        Surface::Offset { base, dist } => format!("offset({}) {dist:.3}", kind(base)),
    }
}

fn fdesc(b: &Body, f: FaceId) -> String {
    format!("{}<{}>", b.face_id_str(f), kind(&b.face(f).surface))
}

fn main() {
    let mut verbose = false;
    for a in std::env::args().skip(1) {
        if a == "-v" {
            verbose = true;
            continue;
        }
        let name = std::path::Path::new(&a).file_name().unwrap().to_string_lossy().to_string();
        println!("=== {name}");
        let Ok(bytes) = std::fs::read(&a) else { continue };
        let t0 = std::time::Instant::now();
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                println!("  not imported: {} {}", e.code, e.message);
                continue;
            }
        };
        let secs = t0.elapsed().as_secs_f64();
        let b = &imp.body;
        let mp = ck_topo::mass::mass_props(b);
        let (sv, sa) = imp.meta.mass.map_or((0.0, 0.0), |m| (m.volume_m3 * 1e9, m.area_m2 * 1e6));
        println!("  read {secs:.2}s lumps {} faces {} | vol {:.4} stored {:.4} ({:+.3}%) | area {:.4} stored {:.4} ({:+.3}%)", b.num_lumps(), b.num_faces(), mp.volume, sv, (mp.volume / sv - 1.0) * 100.0, mp.area, sa, (mp.area / sa - 1.0) * 100.0);
        for p in &imp.parts {
            println!("  part body#{} faces {} built {} refused {}", p.body, p.faces_total, p.faces.len(), p.refused.len());
            for (f, e) in &p.refused {
                let s = p.refused_surfaces.iter().find(|r| r.0 == *f).map(|r| kind(&r.1)).unwrap_or("-".into());
                println!("    refused F{f} <{s}>: {} {}", e.code, e.message);
            }
        }
        for n in &imp.notes {
            println!("  note: {n}");
        }
        let worst = imp.approx.iter().map(|a| a.max_err_mm).fold(0.0, f64::max);
        println!("  approx {} carriers, worst {worst:.2e} mm", imp.approx.len());
        if verbose {
            for a in &imp.approx {
                println!("    approx {:.2e} {}", a.max_err_mm, a.what);
            }
        }
        let issues = ck_check::validate(b, 1e-6);
        let mut by: BTreeMap<&str, usize> = BTreeMap::new();
        for i in &issues {
            *by.entry(i.code).or_default() += 1;
        }
        println!("  issues {:?}", by);
        let mut by3: BTreeMap<&str, usize> = BTreeMap::new();
        for i in ck_check::validate(b, 1e-3) {
            *by3.entry(i.code).or_default() += 1;
        }
        println!("  issues@1e-3 {:?}", by3);
        for e in b.edge_ids() {
            let fs = b.edge_faces(e);
            let len = b.edge_length(e);
            if b.edge_coedges(e).len() != 2 || len < 1e-3 {
                let ed = b.edge(e);
                let p0 = b.vertex(ed.v0).p;
                let p1 = b.vertex(ed.v1).p;
                let d: Vec<String> = fs.iter().map(|&f| fdesc(b, f)).collect();
                println!("    edge#{} uses {} coedges {} len {len:.4} ({:.3},{:.3},{:.3})-({:.3},{:.3},{:.3}) curve {:?} faces {}", e.idx(), fs.len(), b.edge_coedges(e).len(), p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, std::mem::discriminant(&ed.curve), d.join(" "));
            }
        }
        for i in issues.iter().filter(|i| !matches!(i.code, "SHELL_OPEN" | "EDGE_DEGENERATE" | "EDGE_UNUSED")).take(12) {
            println!("    {} {} {}", i.code, i.entity, i.detail);
        }
        if verbose {
            for f in b.face_ids() {
                let m = ck_topo::mass::faces_moments(b, &[f], ck_geom::Vec3::new(0.0, 0.0, 0.0));
                let nl = b.face(f).loops.len();
                println!("    face {} area {:.4} loops {nl} sense {}", fdesc(b, f), m.area, b.face(f).sense);
            }
        }
    }
}
