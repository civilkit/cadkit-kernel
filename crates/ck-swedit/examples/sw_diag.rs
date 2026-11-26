//! Per file: what the history rebuild starts from (no files are part of the repository):
//! the faces per feature kind, whether the history starts with imported bodies and why a starting body is or is not made.
//!
//!     cargo run --profile fast -p ck-swedit --example sw_diag -- <dir or file> [...]
use ck_swedit::Tagged;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

fn files(p: &Path, out: &mut Vec<PathBuf>) {
    if p.is_dir() {
        let Ok(rd) = std::fs::read_dir(p) else { return };
        let mut es: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
        es.sort();
        for q in es {
            files(&q, out);
        }
    } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
        out.push(p.to_path_buf());
    }
}

fn main() {
    let mut fs = vec![];
    for a in std::env::args().skip(1) {
        files(Path::new(&a), &mut fs);
    }
    for (i, p) in fs.iter().enumerate() {
        let bytes = std::fs::read(p).unwrap();
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                println!("#{i} not imported: {}", e.code);
                continue;
            }
        };
        let t = Tagged::from_import(&imp);
        let mut per: BTreeMap<String, usize> = BTreeMap::new();
        let mut untagged = 0;
        for tg in &t.tags {
            if tg.is_empty() {
                untagged += 1;
            }
            for (id, _) in tg {
                let k = t.features.iter().find(|f| f.id == *id).map_or("?".to_string(), |f| f.kind.clone());
                *per.entry(k).or_default() += 1;
            }
        }
        let src = ck_swedit::undo::sources(t.features, &t.tags);
        println!("#{i} faces {} solids {} sheets {} untagged {untagged} start {} sources {:?} per-kind-tags {per:?}", imp.body.num_faces(), imp.solids, imp.sheets, t.start.is_some(), src.as_ref().map(|s| s.len()));
    }
}
