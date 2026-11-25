//! The reader over the three real Fusion files of the corpus. The files are third-party and not
//! in the repository (docs/f3d.md says how the gate gets them).
//!
//! The corpus directory is `$CK_F3D_CORPUS`, else `crates/ck-asm/tests/corpus-local/` (an untracked,
//! gitignored copy, which `tools/remote_build.sh` syncs to the CI box with the rest of the work tree). Inside it the
//! files keep the layout of the `cadmpeg` corpus they come from (`1WireRGBWController.f3d`, `f3d/F30a.f3z`, `f3d/F30b.f3z`). A test whose file is absent says so and passes.
//!
//! Numbers recorded in `tools/f3d_corpus_baseline.json`; `CK_F3D_UPDATE_BASELINE=1` rewrites it.

use ck_asm::container::read_blobs;
use ck_asm::convert::Options;
use ck_asm::import::{ImportOptions, import_f3d};
use ck_asm::read_blob;
use ck_heal::honest::Outcome;
use ck_json::{Json, Obj};

const FILES: [&str; 3] = ["1WireRGBWController.f3d", "f3d/F30a.f3z", "f3d/F30b.f3z"];

fn corpus_file(rel: &str) -> Option<Vec<u8>> {
    let mut dirs: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(d) = std::env::var("CK_F3D_CORPUS") {
        dirs.push(d.into());
    }
    // an untracked copy beside the tests (gitignored; the CI box gets it with the synced work tree)
    dirs.push(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/corpus-local"));
    for d in dirs {
        if let Ok(b) = std::fs::read(d.join(rel)) {
            return Some(b);
        }
    }
    eprintln!("skipped: {rel} not found (set CK_F3D_CORPUS to the directory that holds it)");
    None
}

/// The upstream reader's numbers on the three files: blobs, bodies, solids, faces.
#[test]
fn the_reader_matches_the_upstream_numbers() {
    let (mut blobs, mut bodies, mut solids, mut faces, mut skipped, mut issues) = (0, 0, 0, 0, 0, 0);
    let mut any = false;
    for f in FILES {
        let Some(bytes) = corpus_file(f) else { continue };
        any = true;
        for blob in read_blobs(&bytes).unwrap() {
            blobs += 1;
            let r = read_blob(&blob.entry, &blob.data, &Options::default()).unwrap();
            assert!(r.truncated.is_none(), "{}", blob.entry);
            for b in &r.bodies {
                bodies += 1;
                solids += usize::from(b.body.is_solid());
                faces += b.body.faces.len();
                skipped += b.skipped_faces;
                issues += b.issues.len();
            }
        }
    }
    if !any {
        return;
    }
    assert_eq!(
        (blobs, bodies, solids, faces, skipped, issues),
        (6, 153, 67, 1951, 0, 0)
    );
}

fn baseline_path() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/f3d_corpus_baseline.json")
}

/// The volume of a mesh (triangle soup), mm^3.
fn mesh_volume(m: &ck_tess::mesh::Mesh) -> f64 {
    assert_eq!(m.positions.len(), m.tri_face.len() * 9, "the mesh is a triangle soup");
    let mut v = 0.0;
    for t in m.positions.chunks_exact(9) {
        let p: Vec<f64> = t.iter().map(|x| *x as f64).collect();
        let (a, b, c) = ([p[0], p[1], p[2]], [p[3], p[4], p[5]], [p[6], p[7], p[8]]);
        let cr = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
        v += (a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2]) / 6.0;
    }
    v
}

#[test]
fn every_body_builds_and_the_solids_are_valid() {
    let update = std::env::var("CK_F3D_UPDATE_BASELINE").is_ok();
    let base = std::fs::read_to_string(baseline_path()).ok().and_then(|t| ck_json::parse(&t).ok());
    let mut files_json: Vec<(String, Json)> = Vec::new();
    let mut any = false;
    for f in FILES {
        let Some(bytes) = corpus_file(f) else { continue };
        any = true;
        let imp = import_f3d(&bytes, &ImportOptions { history: true, all_bodies: true, ..Default::default() }).unwrap();

        // the reader's own verdict on each body, by (blob, record)
        let mut neutral = std::collections::HashMap::new();
        let mut faces_in_file = 0;
        for (bi, b) in read_blobs(&bytes).unwrap().iter().enumerate() {
            let r = read_blob(&b.entry, &b.data, &Options::default()).unwrap();
            for cb in r.bodies {
                faces_in_file += cb.body.faces.len();
                neutral.insert((bi, cb.record), cb.body.is_solid());
            }
        }

        // every body builds: no body is refused, no face is left out, nothing is over tolerance
        let mut built = 0;
        for b in &imp.bodies {
            assert!(b.error.is_none(), "{f} {}: {:?}", b.label, b.error);
            assert!(b.skipped.is_empty(), "{f} {}: {:?}", b.label, b.skipped);
            assert_eq!(b.faces_built, b.faces_total, "{f} {}", b.label);
            assert_eq!(b.over_tolerance, 0, "{f} {}", b.label);
            built += b.faces_built;
        }
        assert_eq!(built, faces_in_file, "{f}: faces built vs the reader's");
        assert_eq!(imp.body.num_faces(), faces_in_file, "{f}");

        // the solids are valid and have volume; a body is a solid exactly when the reader says so
        let sorted = imp.sort();
        let mut solids: Vec<(String, f64)> = Vec::new();
        let mut face_sum = 0;
        for (ib, p) in imp.bodies.iter().zip(&sorted.parts) {
            let reader_says = neutral[&(ib.blob, ib.record)];
            assert_eq!(
                p.outcome == Outcome::Solid,
                reader_says,
                "{f} {}: {:?} {:?}",
                p.label,
                p.outcome,
                p.reasons
            );
            assert_eq!(p.faces, p.faces_total, "{f} {}", p.label);
            face_sum += p.faces;
        }
        assert_eq!(face_sum, faces_in_file);
        for (k, p) in sorted.parts.iter().enumerate() {
            if p.outcome != Outcome::Solid {
                continue;
            }
            let one = ck_heal::honest::extract_faces(
                &imp.body,
                &imp.bodies[k]
                    .lumps
                    .iter()
                    .flat_map(|&l| ck_topo::mass::lump_faces(&imp.body, l))
                    .collect::<Vec<_>>(),
            );
            let vol = ck_topo::mass::mass_props(&one).volume;
            assert!(vol > 0.0, "{f} {}: volume {vol}", p.label);
            solids.push((p.label.clone(), vol));
        }
        // the solids as one body: its mesh has no failed face and the mesh's volume is the
        // faces' (an independent check of every surface, seam and orientation)
        let total: f64 = solids.iter().map(|s| s.1).sum();
        let tess = ck_tess::mesh::tessellate(&sorted.solid);
        assert!(tess.mesh.failed.is_empty(), "{f}: faces that did not tessellate: {:?}", tess.mesh.failed_why);
        let mv = mesh_volume(&tess.mesh);
        assert!((mv - total).abs() <= 2e-3 * total, "{f}: mesh volume {mv} vs {total}");

        let mut by_label = Obj::new();
        for (label, vol) in &solids {
            by_label = by_label.put(label.as_str(), *vol);
        }
        let entry = Obj::new()
            .put("blobs", imp.blobs.len() as f64)
            .put("bodies", imp.bodies.len() as f64)
            .put("solids", solids.len() as f64)
            .put("faces", faces_in_file as f64)
            .put("volumeMm3", total)
            .put("solidVolumesMm3", by_label.build())
            .build();
        eprintln!(
            "{f}: blobs {} bodies {} solids {} faces {faces_in_file} volume {total:.3} mm3 (mesh {mv:.3})",
            imp.blobs.len(),
            imp.bodies.len(),
            solids.len()
        );
        if !update {
            if let Some(b) = base.as_ref().and_then(|b| b.get("files")).and_then(|b| b.get(f)) {
                for k in ["blobs", "bodies", "solids", "faces"] {
                    assert_eq!(b.get_f64(k), entry.get_f64(k), "{f}: {k} differs from the baseline");
                }
                let bv = b.get_f64("volumeMm3").unwrap();
                assert!((bv - total).abs() <= 1e-7 * bv, "{f}: volume {total} vs baseline {bv}");
                if let (Some(Json::Obj(bs)), Json::Obj(ours)) = (b.get("solidVolumesMm3"), entry.get("solidVolumesMm3").unwrap()) {
                    assert_eq!(bs.len(), ours.len(), "{f}: solid count");
                    for ((bk, bv), (ok, ov)) in bs.iter().zip(ours.iter()) {
                        assert_eq!(bk, ok);
                        let (bv, ov) = (bv.as_f64().unwrap(), ov.as_f64().unwrap());
                        assert!((bv - ov).abs() <= 1e-7 * bv.abs().max(1.0), "{f} {bk}: {ov} vs baseline {bv}");
                    }
                }
            }
        }
        files_json.push((f.to_string(), entry));
    }
    if !any {
        return;
    }
    if update {
        let out = Obj::new()
            .put("_about", "Fusion (.f3d/.f3z) corpus numbers of crates/ck-asm/tests/corpus.rs: per file blobs, bodies, solids, faces and the volumes (mm^3) of the solids. Rewritten by CK_F3D_UPDATE_BASELINE=1. The upstream reader's numbers on the three files: 6 blobs, 153 bodies, 67 solids, 1951 faces.")
            .put("files", Json::Obj(files_json))
            .build();
        std::fs::write(baseline_path(), ck_json::to_string(&out) + "\n").unwrap();
    }
}

/// With `all_bodies`, a history body that is identical to a plain one is read once; `history` reads all.
#[test]
fn history_twins_are_read_once() {
    for (f, by_default_n, all_n) in [("1WireRGBWController.f3d", 145, 145), ("f3d/F30a.f3z", 3, 4), ("f3d/F30b.f3z", 3, 4)] {
        let Some(bytes) = corpus_file(f) else { continue };
        let by_default = import_f3d(&bytes, &ImportOptions { all_bodies: true, ..Default::default() }).unwrap();
        assert_eq!(by_default.bodies.len(), by_default_n, "{f}");
        let every = import_f3d(&bytes, &ImportOptions { history: true, all_bodies: true, ..Default::default() }).unwrap();
        assert_eq!(every.bodies.len(), all_n, "{f}");
        assert_eq!(by_default.warnings.len(), usize::from(all_n != by_default_n), "{f}: {:?}", by_default.warnings);
    }
}
