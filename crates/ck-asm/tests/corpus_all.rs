//! The whole Fusion corpus through the reader and the builder, with the app's defaults.
//!
//! `$CK_F3D_CORPUS` is a directory of `.f3d` / `.f3z` files (searched recursively; they are
//! third-party and never in the repository). Unset or absent: the test says so and passes.
//!
//! What is counted, per file, on three read paths:
//!   - `reader`: the neutral bodies of every blob (what the upstream reader, MitCad's, reports): bodies, solids, faces.
//!   - `design`: `import_f3d` with the app's defaults: only the design's own bodies (top-level bodies of the history
//!     blob, or of the plain blobs of a document without one).
//!   - `all`: `import_f3d` with `all_bodies` and the history twin of a plain body read once.
//!   - `history`: `all_bodies` and `history: true` (every body of every blob; a design body that is in both a
//!     plain and a history blob is counted twice, so compare this with `reader`).
//! and for the bodies the reader says are solids but the import did not make a solid, the cause by surface or curve type.
//!
//! The totals are held by `tools/f3d_corpus_baseline.json` (key `corpus`); `CK_F3D_UPDATE_BASELINE=1` rewrites them.
//! `CK_F3D_REPORT=<path>` writes the whole per-file report there as JSON.

use ck_asm::container::read_blobs;
use ck_asm::convert::Options;
use ck_asm::import::{Import, ImportOptions, import_f3d};
use ck_asm::read_blob;
use ck_heal::honest::Outcome;
use ck_json::{Json, Obj};
use std::collections::BTreeMap;
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

#[derive(Default, Clone)]
struct Counts {
    bodies: usize,
    solids: usize,
    surfaces: usize,
    refused: usize,
    faces_built: usize,
    faces_total: usize,
    volume: f64,
    /// Of the bodies of this path, those the reader says are solids.
    reader_solids: usize,
    /// Faces of the solids that did not tessellate (design path only).
    mesh_failed: usize,
    /// Volume of the solids' mesh (design path only).
    mesh_volume: f64,
}

impl Counts {
    fn json(&self) -> Json {
        Obj::new()
            .put("bodies", self.bodies)
            .put("solids", self.solids)
            .put("surfaces", self.surfaces)
            .put("refused", self.refused)
            .put("facesBuilt", self.faces_built)
            .put("facesTotal", self.faces_total)
            .put("readerSolids", self.reader_solids)
            .put("solidVolumeMm3", self.volume)
            .put("meshFailedFaces", self.mesh_failed)
            .put("meshVolumeMm3", self.mesh_volume)
            .build()
    }
}

#[derive(Default)]
struct Cause {
    /// Reader solids this cause cost (a body with two causes is counted under both).
    solids: usize,
    /// Faces left out for it.
    faces: usize,
}

struct Run {
    counts: Counts,
    /// cause -> counts, over the bodies the reader says are solids and the import did not make solids.
    causes: BTreeMap<String, Cause>,
    /// Reader solids that did not come out as solids.
    lost_solids: usize,
    /// Bodies per (blob, record) -> outcome is solid
    panicked: Option<String>,
}

/// A cause with its numbers (mm, counts, indices) taken out, so equal causes add up.
fn generic(s: &str) -> String {
    let mut out = String::new();
    let mut in_num = false;
    for ch in s.chars() {
        let digit = ch.is_ascii_digit() || (in_num && matches!(ch, '.' | 'e' | '-' | '+'));
        if digit {
            if !in_num {
                out.push('N');
            }
            in_num = true;
        } else {
            in_num = false;
            out.push(ch);
        }
    }
    out
}

fn run(bytes: &[u8], all_bodies: bool, history: bool, neutral: &BTreeMap<(usize, usize), (bool, String)>, rel: &str) -> Run {
    let mut r = Run { counts: Counts::default(), causes: BTreeMap::new(), lost_solids: 0, panicked: None };
    let res = std::panic::catch_unwind(|| {
        // a design with no body blob is an error of the import (`f3d.empty`), not a failure of the reader
        let imp: Option<Import> = match import_f3d(bytes, &ImportOptions { history, all_bodies, ..Default::default() }) {
            Ok(i) => Some(i),
            Err(e) if e.code == "f3d.empty" => None,
            Err(e) => panic!("{e}"),
        };
        let imp = imp.unwrap_or_default();
        let sorted = imp.sort();
        (imp, sorted)
    });
    let Ok((imp, sorted)) = res else {
        r.panicked = Some("panic in import".into());
        return r;
    };
    r.counts.bodies = imp.bodies.len();
    for (ib, p) in imp.bodies.iter().zip(&sorted.parts) {
        r.counts.faces_built += ib.faces_built;
        r.counts.faces_total += ib.faces_total;
        r.counts.reader_solids += usize::from(neutral.get(&(ib.blob, ib.record)).map_or(false, |n| n.0));
        match p.outcome {
            Outcome::Solid => r.counts.solids += 1,
            Outcome::Surface => r.counts.surfaces += 1,
            Outcome::Refused => r.counts.refused += 1,
        }
        if p.outcome != Outcome::Solid && neutral.get(&(ib.blob, ib.record)).map_or(false, |n| n.0) {
            r.lost_solids += 1;
            let mut keys: BTreeMap<String, usize> = BTreeMap::new();
            for s in &ib.skipped {
                let what = if s.what.is_empty() { s.message.clone() } else { s.what.clone() };
                *keys.entry(format!("{}: {}", s.code, what)).or_default() += 1;
            }
            if let Some(e) = &ib.error {
                *keys.entry(format!("buildFailed: {e}")).or_default() += 1;
            }
            for i in &ib.issues {
                *keys.entry(format!("readerIssue: {}", i.chars().take(80).collect::<String>())).or_default() += 1;
            }
            if keys.is_empty() {
                // every face built, yet the body is not a valid closed solid: the honest sort's reasons
                for rs in &p.reasons {
                    if rs.code != ck_heal::honest::SURFACE_MODEL {
                        *keys.entry(format!("notSolid: {}", rs.code)).or_default() += 1;
                    }
                }
                if keys.is_empty() {
                    *keys.entry("notSolid: no reason given".into()).or_default() += 1;
                }
            }
            if std::env::var("CK_F3D_LOST").is_ok() && !all_bodies {
                let kinds = neutral.get(&(ib.blob, ib.record)).map(|n| n.1.clone()).unwrap_or_default();
                eprintln!("LOST {rel} | {} | {:?} {}/{} faces | {:?} | {kinds}", ib.label, p.outcome, ib.faces_built, ib.faces_total, keys);
            }
            for (k, n) in keys {
                let k = generic(&k);
                let c = r.causes.entry(k).or_default();
                c.solids += 1;
                c.faces += n;
            }
        }
    }
    let vol = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| ck_topo::mass::mass_props(&sorted.solid).volume));
    r.counts.volume = vol.unwrap_or(f64::NAN);
    if !all_bodies && sorted.solid.num_faces() > 0 {
        // the design's solids as one body: every face tessellates, and the mesh's volume is the faces'
        let t = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let tess = ck_tess::mesh::tessellate(&sorted.solid);
            let mut v = 0.0;
            for t in tess.mesh.positions.chunks_exact(9) {
                let p: Vec<f64> = t.iter().map(|x| *x as f64).collect();
                let (a, b, c) = ([p[0], p[1], p[2]], [p[3], p[4], p[5]], [p[6], p[7], p[8]]);
                let cr = [b[1] * c[2] - b[2] * c[1], b[2] * c[0] - b[0] * c[2], b[0] * c[1] - b[1] * c[0]];
                v += (a[0] * cr[0] + a[1] * cr[1] + a[2] * cr[2]) / 6.0;
            }
            if std::env::var("CK_F3D_LOST").is_ok() {
                for (f, why) in tess.mesh.failed.iter().zip(&tess.mesh.failed_why) {
                    let kind = sorted.solid.face(ck_topo::body::FaceId(*f as u32)).surface.kind();
                    eprintln!("MESHWHY {rel} | {kind} | {}", generic(why));
                }
            }
            (tess.mesh.failed.len(), v)
        }));
        match t {
            Ok((failed, v)) => {
                r.counts.mesh_failed = failed;
                r.counts.mesh_volume = v;
            }
            Err(_) => r.counts.mesh_failed = usize::MAX / 2,
        }
    }
    r
}

#[test]
fn the_whole_corpus_through_the_import() {
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
    let update = std::env::var("CK_F3D_UPDATE_BASELINE").is_ok();
    let mut per_file: Vec<(String, Json)> = Vec::new();
    // valid solids of the design path, by file (the baseline holds each as a floor)
    let mut design_by_file: Vec<(String, Json)> = Vec::new();
    let (mut reader_t, mut def_t, mut all_t, mut hist_t) = (Counts::default(), Counts::default(), Counts::default(), Counts::default());
    let mut causes_def: BTreeMap<String, Cause> = BTreeMap::new();
    let (mut failed_files, mut lost_def, mut lost_all, mut lost_hist) = (0usize, 0usize, 0usize, 0usize);
    let mut causes_hist: BTreeMap<String, Cause> = BTreeMap::new();
    for f in &files {
        let rel = f.strip_prefix(&dir).unwrap().to_string_lossy().to_string();
        let bytes = std::fs::read(f).unwrap();
        // the reader's numbers
        let mut reader = Counts::default();
        let mut neutral: BTreeMap<(usize, usize), (bool, String)> = BTreeMap::new();
        let Ok(blobs) = read_blobs(&bytes) else {
            failed_files += 1;
            per_file.push((rel, Obj::new().put("error", "container").build()));
            continue;
        };
        for (bi, b) in blobs.iter().enumerate() {
            let Ok(r) = read_blob(&b.entry, &b.data, &Options::default()) else {
                failed_files += 1;
                continue;
            };
            for cb in r.bodies {
                reader.bodies += 1;
                reader.solids += usize::from(cb.body.is_solid());
                reader.faces_total += cb.body.faces.len();
                let mut kinds: BTreeMap<&str, usize> = BTreeMap::new();
                for f in &cb.body.faces {
                    *kinds.entry(cb.body.surfaces[f.surface].kind()).or_default() += 1;
                }
                neutral.insert((bi, cb.record), (cb.body.is_solid(), format!("{kinds:?}")));
            }
        }
        let d = run(&bytes, false, false, &neutral, &rel);
        let a = run(&bytes, true, false, &neutral, &rel);
        let h = run(&bytes, true, true, &neutral, &rel);
        eprintln!(
            "{rel}: reader {}b/{}s/{}f; default {}b/{}s/{}f built {}; history {}b/{}s",
            reader.bodies, reader.solids, reader.faces_total, d.counts.bodies, d.counts.solids, d.counts.faces_total, d.counts.faces_built, h.counts.bodies, h.counts.solids
        );
        let mut o = Obj::new().put("reader", reader.json()).put("design", d.counts.json()).put("all", a.counts.json()).put("history", h.counts.json());
        o = o.put("lostSolidsDesign", d.lost_solids).put("lostSolidsAll", a.lost_solids).put("lostSolidsHistory", h.lost_solids);
        if let Some(p) = d.panicked.as_ref().or(h.panicked.as_ref()) {
            o = o.put("panic", p.as_str());
        }
        design_by_file.push((rel.clone(), Json::from(d.counts.solids)));
        per_file.push((rel, o.build()));
        for (tot, c) in [(&mut reader_t, &reader), (&mut def_t, &d.counts), (&mut all_t, &a.counts), (&mut hist_t, &h.counts)] {
            tot.bodies += c.bodies;
            tot.solids += c.solids;
            tot.surfaces += c.surfaces;
            tot.refused += c.refused;
            tot.faces_built += c.faces_built;
            tot.faces_total += c.faces_total;
            tot.reader_solids += c.reader_solids;
            tot.mesh_failed += c.mesh_failed;
            tot.mesh_volume += c.mesh_volume;
            tot.volume += if c.volume.is_finite() { c.volume } else { 0.0 };
        }
        lost_def += d.lost_solids;
        lost_all += a.lost_solids;
        lost_hist += h.lost_solids;
        for (k, c) in d.causes {
            let e = causes_def.entry(k).or_default();
            e.solids += c.solids;
            e.faces += c.faces;
        }
        for (k, c) in h.causes {
            let e = causes_hist.entry(k).or_default();
            e.solids += c.solids;
            e.faces += c.faces;
        }
    }
    let causes_json = |m: &BTreeMap<String, Cause>| {
        let mut v: Vec<(&String, &Cause)> = m.iter().collect();
        v.sort_by(|a, b| b.1.solids.cmp(&a.1.solids));
        Json::Arr(v.into_iter().map(|(k, c)| Obj::new().put("cause", k.as_str()).put("solids", c.solids).put("faces", c.faces).build()).collect())
    };
    let totals = Obj::new()
        .put("files", files.len())
        .put("failedFiles", failed_files)
        .put("reader", reader_t.json())
        .put("design", def_t.json())
        .put("all", all_t.json())
        .put("history", hist_t.json())
        .put("readerSolidsNotSolidDesign", lost_def)
        .put("readerSolidsNotSolidAll", lost_all)
        .put("readerSolidsNotSolidHistory", lost_hist)
        .build();
    eprintln!("TOTALS {}", ck_json::to_string(&totals));
    for (k, c) in &causes_def {
        eprintln!("CAUSE(design) {} solids {} faces {}: {k}", c.solids, c.faces, "");
    }
    if let Ok(p) = std::env::var("CK_F3D_REPORT") {
        let out = Obj::new()
            .put("totals", totals.clone())
            .put("causesDesign", causes_json(&causes_def))
            .put("causesHistory", causes_json(&causes_hist))
            .put("files", Json::Obj(per_file.clone()))
            .build();
        std::fs::write(p, ck_json::to_string(&out) + "\n").unwrap();
    }
    assert_eq!(failed_files, 0, "files the reader could not open");
    assert!(per_file.iter().all(|(_, j)| j.get("panic").is_none()), "an import panicked");

    // baseline: the totals, by read path
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../tools/f3d_corpus_baseline.json");
    let text = std::fs::read_to_string(&path).unwrap();
    let mut base = ck_json::parse(&text).unwrap();
    let corpus = Obj::new()
        .put("_about", "The 62-file Fusion corpus ($CK_F3D_CORPUS) through ck-asm with the app's defaults. readerSolids: solids the upstream reader finds in every blob (history blobs included). designSolids: valid solids of import_f3d with the app's defaults (the design's own bodies only: top-level bodies of the history blob, or of the plain blobs of a document without one). allSolids: valid solids of import_f3d with all_bodies (a history body identical to a plain one is read once). historySolids: valid solids of import_f3d with all_bodies and history:true (a design body in both a plain and a history blob counts twice). Rewritten by CK_F3D_UPDATE_BASELINE=1.")
        .put("files", files.len())
        .put("readerBodies", reader_t.bodies)
        .put("readerSolids", reader_t.solids)
        .put("readerFaces", reader_t.faces_total)
        .put("designBodies", def_t.bodies)
        .put("designSolids", def_t.solids)
        .put("designSurfaceBodies", def_t.surfaces)
        .put("designRefused", def_t.refused)
        .put("designFacesBuilt", def_t.faces_built)
        .put("designFacesTotal", def_t.faces_total)
        .put("designReaderSolids", def_t.reader_solids)
        .put("designMeshFailedFaces", def_t.mesh_failed)
        .put("designMeshVolumeMm3", def_t.mesh_volume)
        .put("designSolidVolumeMm3", def_t.volume)
        .put("allBodies", all_t.bodies)
        .put("allSolids", all_t.solids)
        .put("allFacesBuilt", all_t.faces_built)
        .put("allFacesTotal", all_t.faces_total)
        .put("allReaderSolids", all_t.reader_solids)
        .put("historyBodies", hist_t.bodies)
        .put("historySolids", hist_t.solids)
        .put("historyFacesBuilt", hist_t.faces_built)
        .put("historyFacesTotal", hist_t.faces_total)
        .put("designSolidsByFile", Json::Obj(design_by_file.clone()))
        .build();
    if update {
        if let Json::Obj(m) = &mut base {
            m.retain(|(k, _)| k != "corpus");
            m.push(("corpus".into(), corpus));
        }
        std::fs::write(&path, ck_json::to_string(&base) + "\n").unwrap();
        return;
    }
    let Some(b) = base.get("corpus") else {
        eprintln!("no corpus baseline recorded yet");
        return;
    };
    for k in ["files", "readerBodies", "readerSolids", "readerFaces", "designBodies", "designReaderSolids", "allBodies", "historyBodies"] {
        assert_eq!(b.get_f64(k), corpus.get_f64(k), "{k} differs from the baseline");
    }
    // the solids and faces built may only grow (a fix); a drop is a regression
    // no file loses solids of the design
    if let Some(Json::Obj(base_files)) = b.get("designSolidsByFile") {
        for (name, n) in base_files {
            let now = design_by_file.iter().find(|(k, _)| k == name).and_then(|(_, v)| v.as_f64());
            assert!(now >= n.as_f64(), "{name}: {now:?} design solids, the baseline has {n:?}");
        }
    }
    // the faces that fail to tessellate may only go down
    assert!(corpus.get_f64("designMeshFailedFaces") <= b.get_f64("designMeshFailedFaces"), "designMeshFailedFaces went up from the baseline");
    for k in ["designSolids", "allSolids", "historySolids", "designFacesBuilt", "allFacesBuilt", "historyFacesBuilt"] {
        assert!(corpus.get_f64(k) >= b.get_f64(k), "{k}: {:?} is below the baseline {:?}", corpus.get_f64(k), b.get_f64(k));
    }
}
