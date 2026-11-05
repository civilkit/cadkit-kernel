//! Two NIST part files used to import as surface bodies because the reader left faces out: spline blend faces whose
//! spine is a half circle (the arc between the blend limits was taken as the complement, so the face corners were
//! off the surface) and faces on self-intersecting (apple) tori. Runs over the local corpus (`SW_CORPUS`, default
//! `corpus-local` at the workspace root); passes without it.

use std::path::PathBuf;

fn find(name: &str) -> Option<Vec<u8>> {
    let root = std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"));
    for d in [root.join(name), root.join("samples").join(name)] {
        if let Ok(rd) = std::fs::read_dir(&d) {
            for e in rd.flatten() {
                if e.path().extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
                    return std::fs::read(e.path()).ok();
                }
            }
        }
    }
    None
}

fn issues(bytes: &[u8]) -> (ck_sldprt::SldprtImport, Vec<&'static str>) {
    let imp = ck_sldprt::import_sldprt(bytes).unwrap();
    let mut body = imp.body.clone();
    ck_topo::region::rebuild_lumps(&mut body);
    ck_heal::heal_with(&mut body, &ck_heal::tolerant_only());
    let codes = ck_check::validate_cap(&body, 1e-6, ck_check::DEFAULT_TOL_CAP).iter().map(|i| i.code).collect();
    (imp, codes)
}

#[test]
fn half_circle_blend_spine_gives_a_solid() {
    let Some(b) = find("nist_ftc_07") else { return };
    let (imp, codes) = issues(&b);
    assert!(imp.parts.iter().all(|p| p.refused.is_empty()), "faces left out: {:?}", imp.parts.iter().flat_map(|p| p.refused.iter().map(|r| r.0)).collect::<Vec<_>>());
    assert_eq!(imp.body.num_faces(), 272);
    assert!(codes.is_empty(), "{codes:?}");
    let sorted = ck_model::features::import_body::sort_sldprt(imp.body, &imp.parts, "import", None);
    assert!(sorted.surfaces.is_empty() && sorted.solid.num_faces() == 272);
    let vol = ck_topo::mass::mass_props(&sorted.solid).volume;
    let file = imp.sw_mass.unwrap().volume_m3 * 1e9;
    assert!((vol - file).abs() / file < 0.005, "volume {vol} vs file {file}");
}

#[test]
fn apple_torus_faces_are_read_and_the_shells_close() {
    let Some(b) = find("nist_ctc_02") else { return };
    let (imp, codes) = issues(&b);
    assert!(imp.parts.iter().all(|p| p.refused.is_empty()));
    assert_eq!(imp.body.num_faces(), 488);
    assert!(!codes.contains(&"SHELL_OPEN"), "{codes:?}");
}
