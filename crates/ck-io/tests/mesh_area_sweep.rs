//! Every face of every STEP file in the repo: the area of its mesh against its exact area.
//!
//! A mesh of the wrong region (closed, so no flag marks it) shows only as an area that disagrees. Each face
//! must be within `TOL` of its exact area, unless it is listed in `KNOWN` with the reason.
use ck_io::step::read_step;

const TOL: f64 = 0.05;
/// Faces smaller than this (mm^2) are judged by chord error, not by region: a mesh of a sliver is a few triangles.
const TINY: f64 = 5.0;

/// `file:face_index` of faces known to disagree, with the reason.
const KNOWN: &[(&str, &str)] = &[];

fn files() -> Vec<std::path::PathBuf> {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let mut out = Vec::new();
    for dir in [root.join("../../tools/bench/benchcad/gt"), root.join("tests/fixtures")] {
        if let Ok(rd) = std::fs::read_dir(&dir) {
            for e in rd.flatten() {
                let p = e.path();
                if p.extension().is_some_and(|x| x == "step" || x == "stp") {
                    out.push(p);
                }
            }
        }
    }
    out.sort();
    out
}

#[test]
fn mesh_area_matches_exact_area_on_every_face() {
    let mut bad: Vec<String> = Vec::new();
    let (mut nfiles, mut nfaces) = (0, 0);
    for p in files() {
        let name = p.file_name().unwrap().to_string_lossy().to_string();
        let Ok(text) = std::fs::read_to_string(&p) else { continue };
        let Ok(r) = read_step(&text) else { continue };
        nfiles += 1;
        let t = ck_tess::mesh::tessellate(&r.body);
        for (i, id) in r.body.face_ids().enumerate() {
            nfaces += 1;
            let exact = ck_topo::mass::face_integral::<1>(&r.body, id, 1, &|_, n| [n.len()])[0].abs();
            let kind = r.body.face(id).surface.kind();
            let key = format!("{name}:{i}");
            if KNOWN.iter().any(|(k, _)| *k == key) {
                continue;
            }
            match t.faces.get(i) {
                Some(Some(fm)) => {
                    let m: f64 = fm.tris.iter().map(|t| 0.5 * (fm.pts[t[1] as usize] - fm.pts[t[0] as usize]).cross(fm.pts[t[2] as usize] - fm.pts[t[0] as usize]).len()).sum();
                    if exact > TINY && ((m - exact).abs() > TOL * exact) {
                        bad.push(format!("{key} {kind}: mesh {m:.2} exact {exact:.2} ratio {:.3} open {} unrecovered {}", m / exact, fm.open, fm.unrecovered));
                    }
                }
                _ => bad.push(format!("{key} {kind}: no mesh (exact {exact:.2})")),
            }
        }
    }
    eprintln!("MESHSWEEP files {nfiles} faces {nfaces} bad {}", bad.len());
    for b in &bad {
        eprintln!("MESHSWEEP {b}");
    }
    assert!(bad.is_empty(), "{} faces whose mesh area disagrees with the exact area", bad.len());
}
