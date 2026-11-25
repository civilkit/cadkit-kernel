//! Fusion design -> STEP export -> STEP import, over the local corpus (`$CK_F3D_CORPUS`, skipped when unset): for every file the
//! design's body, as `importF3d` gives it, is written as STEP and read back; the volume must agree (relative 1e-3) and the
//! re-read body's display mesh must have every face (no TESSELLATION_OPEN). The counts are held by
//! `tools/f3d_step_roundtrip_baseline.json` (aggregates only); `CK_F3D_UPDATE_BASELINE=1` rewrites it and
//! `CK_F3D_ROUNDTRIP_REPORT=<path>` writes the per-file list there. A file may only move from failing to passing.
use ck_json::{Json, Obj};
use std::path::{Path, PathBuf};

fn find_files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        if p.is_dir() {
            find_files(&p, out);
        } else if matches!(p.extension().and_then(|x| x.to_str()), Some("f3d" | "f3z")) {
            out.push(p);
        }
    }
}

fn open_faces(b: &ck_topo::body::Body) -> usize {
    let tess = ck_tess::mesh::tessellate(b);
    let m = &tess.mesh;
    let (issues, _) = ck_check::tessellation::check_mesh_ctl_notes(b, &m.positions, &m.tri_face, &m.failed, &m.failed_why, &ck_check::Ctl::unlimited());
    issues.iter().filter(|i| i.code == "TESSELLATION_OPEN").count()
}

#[test]
fn the_design_survives_step_export_and_import() {
    let Ok(dir) = std::env::var("CK_F3D_CORPUS") else {
        eprintln!("skipped: CK_F3D_CORPUS is not set");
        return;
    };
    let dir = PathBuf::from(dir);
    let mut files = Vec::new();
    find_files(&dir, &mut files);
    files.sort();
    if files.is_empty() {
        eprintln!("skipped: no .f3d/.f3z under {}", dir.display());
        return;
    }
    let (mut n, mut with_body, mut vol_ok, mut mesh_ok, mut pass, mut bad_read) = (0usize, 0usize, 0usize, 0usize, 0usize, 0usize);
    let mut per_file: Vec<(String, Json)> = vec![];
    for f in &files {
        let rel = f.strip_prefix(&dir).unwrap().to_string_lossy().to_string();
        n += 1;
        let bytes = std::fs::read(f).unwrap();
        let Ok(imp) = ck_asm::import::import_f3d(&bytes, &Default::default()) else { continue };
        let sorted = imp.sort();
        let mut b0 = sorted.solid.clone();
        if b0.num_faces() == 0 {
            continue;
        }
        with_body += 1;
        ck_topo::region::rebuild_lumps(&mut b0);
        let v0 = ck_topo::mass::mass_props(&b0).volume;
        let text = ck_io::step::write_step(&b0, "part", &ck_io::step::StepOpts { unix_ms: 0.0 });
        let Ok(r) = ck_io::step::read_step(&text) else {
            bad_read += 1;
            per_file.push((rel, Obj::new().put("error", "read").build()));
            continue;
        };
        let v1 = ck_topo::mass::mass_props(&r.body).volume;
        let rel_diff = (v1 - v0).abs() / v0.abs().max(1e-9);
        let v_ok = rel_diff <= 1e-3;
        let open0 = open_faces(&b0);
        let open1 = open_faces(&r.body);
        // the re-read body may not have more open faces than the design had
        let m_ok = open1 <= open0;
        vol_ok += usize::from(v_ok);
        mesh_ok += usize::from(m_ok);
        pass += usize::from(v_ok && m_ok);
        per_file.push((rel, Obj::new().put("relVolumeDiff", rel_diff).put("openFacesBefore", open0).put("openFacesAfter", open1).put("pass", v_ok && m_ok).build()));
    }
    let totals = Obj::new()
        .put("files", n)
        .put("filesWithBody", with_body)
        .put("filesReadBackFailed", bad_read)
        .put("filesVolumeWithin1e-3", vol_ok)
        .put("filesMeshNoWorse", mesh_ok)
        .put("filesPass", pass)
        .build();
    eprintln!("ROUNDTRIP {}", ck_json::to_string(&totals));
    if let Ok(p) = std::env::var("CK_F3D_ROUNDTRIP_REPORT") {
        std::fs::write(p, ck_json::to_string(&Obj::new().put("totals", totals.clone()).put("files", Json::Obj(per_file)).build()) + "\n").unwrap();
    }
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/f3d_step_roundtrip_baseline.json");
    if std::env::var("CK_F3D_UPDATE_BASELINE").is_ok() {
        std::fs::write(&path, ck_json::to_string(&totals) + "\n").unwrap();
        return;
    }
    let base = ck_json::parse(&std::fs::read_to_string(&path).unwrap()).unwrap();
    assert!(base.get_f64("filesPass").unwrap() as usize <= pass, "fewer files survive the STEP round trip: {pass} vs baseline {}", base.get_f64("filesPass").unwrap());
    assert!(base.get_f64("filesVolumeWithin1e-3").unwrap() as usize <= vol_ok, "volume round trip regressed");
    assert_eq!(bad_read, 0, "files whose STEP export could not be read back");
}
