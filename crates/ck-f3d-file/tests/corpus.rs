//! The container layer over the real Fusion files of the corpus. The files are third-party and not in the repository
//! (docs/f3d.md). The directory is `$CK_F3D_CORPUS`, else `crates/ck-asm/tests/corpus-local/` (untracked, gitignored),
//! with the layout `1WireRGBWController.f3d`, `f3d/F30a.f3z`, `f3d/F30b.f3z`. A test whose file is absent says so and
//! passes. Every figure checked here is read with ck-asm's own container reader as well, so the two agree.

use ck_f3d_file::F3dFile;
use ck_f3d_file::container::EntryKind;
use ck_f3d_file::ogs::display_scene;

fn corpus_file(rel: &str) -> Option<Vec<u8>> {
    let mut dirs: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(d) = std::env::var("CK_F3D_CORPUS") {
        dirs.push(d.into());
    }
    dirs.push(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../ck-asm/tests/corpus-local"));
    for d in dirs {
        if let Ok(b) = std::fs::read(d.join(rel)) {
            return Some(b);
        }
    }
    eprintln!("skipped: {rel} not found (set CK_F3D_CORPUS to the directory that holds it)");
    None
}

#[test]
fn a_document_lists_its_segments_and_body_blobs() {
    let Some(bytes) = corpus_file("1WireRGBWController.f3d") else { return };
    let doc = F3dFile::from_bytes(bytes.clone()).expect("a zip");
    assert!(doc.segments().iter().any(|s| s.is_design()), "a design segment");
    let blobs = doc.body_blobs();
    assert!(!blobs.is_empty());
    // the same blobs as ck-asm's reader finds
    let theirs = ck_asm::container::read_blobs(&bytes).unwrap();
    assert_eq!(blobs.len(), theirs.len());
    for (a, b) in blobs.iter().zip(&theirs) {
        assert_eq!((a.guid.as_str(), a.history), (b.guid.as_str(), b.history));
        assert_eq!(doc.read_large(&a.entry).unwrap(), b.data);
    }
    // the display scene (an independent check of the bodies) reads without an issue
    if let Some(scene) = display_scene(&doc).unwrap() {
        assert!(!scene.bodies.is_empty());
    }
}

#[test]
fn a_package_opens_its_documents() {
    for rel in ["f3d/F30a.f3z", "f3d/F30b.f3z"] {
        let Some(bytes) = corpus_file(rel) else { continue };
        let pkg = F3dFile::from_bytes(bytes.clone()).expect("a zip");
        let docs = pkg.documents();
        assert!(!docs.is_empty(), "{rel}: documents");
        assert!(pkg.entries().iter().any(|e| e.kind == EntryKind::NestedDocument));
        let mut n = 0;
        for d in &docs {
            n += pkg.open_document(d).unwrap().body_blobs().len();
        }
        assert_eq!(n, ck_asm::container::read_blobs(&bytes).unwrap().len(), "{rel}: blobs of all documents");
    }
}
