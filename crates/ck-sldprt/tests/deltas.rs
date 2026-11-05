//! The `(deltas)` transmit that follows the partition: it frames to its end on every file, and it is a chain of backward
//! undo records that ends at the partition's own current mark, so the partition is the current state (docs/sldprt.md,
//! "Deltas"). The corpus part (`SW_CORPUS`, default `corpus-local` at the workspace root) passes without the corpus.

use ck_sldprt::container::{self, TransmitKind};
use ck_sldprt::xt::{self, DeltaRule};
use ck_sldprt::Limits;
use std::path::{Path, PathBuf};

struct Inv {
    sections: usize,
    changes: usize,
    nodes: usize,
}

/// Frame the deltas of every partition in `bytes` and check the chain invariants; `None` when the file has no deltas.
fn check(name: &str, bytes: &[u8]) -> Option<Inv> {
    let lim = Limits::default();
    let c = container::open(bytes, &lim).unwrap();
    let ts = c.transmits(&lim).unwrap();
    let td = ts.iter().find(|t| t.kind == TransmitKind::Deltas)?;
    let d = xt::read_deltas(&td.data, &lim).unwrap_or_else(|e| panic!("{name}: {} {}", e.code, e.message));
    assert_eq!(d.trailing_bytes, 0, "{name}: bytes after the last section");
    assert_eq!(d.maps.len(), d.pmark_sections.len() + usize::from(d.final_terminator), "{name}: map / PMARK section alternation");
    let cur = d.current_pmark.as_ref().unwrap();
    // every change is backward, the mark identifiers fall along the chain (older marks), and it ends at the initial mark
    let marks: Vec<&xt::Node> = std::iter::once(cur).chain(d.pmark_sections.iter().flat_map(|s| s.pmarks.iter())).collect();
    assert!(marks.iter().all(|n| n.i("delta_is_forward") == 0), "{name}: forward change");
    assert!(marks.windows(2).all(|w| w[0].i("identifier") > w[1].i("identifier")), "{name}: mark identifiers not falling");
    assert_eq!(marks.last().unwrap().i("preceding"), 0, "{name}: chain does not end at the initial mark");
    // the partition is at the mark the deltas call current
    let tp = ts.iter().find(|t| t.kind == TransmitKind::Partition && t.stream == td.stream).unwrap();
    let x = xt::read_xt(&tp.data, &lim).unwrap();
    let w = x.nodes.values().find(|n| n.ty == 101).expect("WORLD");
    assert_eq!(w.i("current_id"), cur.i("identifier"), "{name}: partition is not at the current mark");
    assert_eq!(w.i("highest_id"), d.partition.as_ref().unwrap().i("highest_id"), "{name}");
    Some(Inv { sections: 1 + d.maps.len() + d.pmark_sections.len(), changes: marks.len(), nodes: d.node_count() })
}

#[test]
fn fixtures_frame_and_end_at_the_partition_mark() {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut n = 0;
    for e in std::fs::read_dir(dir).unwrap() {
        let p = e.unwrap().path();
        if p.extension().map_or(false, |x| x == "sldprt") {
            let name = p.file_stem().unwrap().to_string_lossy().to_string();
            let inv = check(&name, &std::fs::read(&p).unwrap()).expect("fixture without deltas");
            assert!(inv.sections >= 5 && inv.changes >= 3, "{name}");
            n += 1;
        }
    }
    assert_eq!(n, 4);
}

fn files(dir: &Path, out: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    let mut es: Vec<PathBuf> = rd.flatten().map(|e| e.path()).collect();
    es.sort();
    for p in es {
        if p.is_dir() {
            files(&p, out);
        } else if p.extension().map_or(false, |x| x.eq_ignore_ascii_case("sldprt")) {
            out.push(p);
        }
    }
}

fn corpus_root() -> PathBuf {
    std::env::var("SW_CORPUS").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../corpus-local"))
}

#[test]
fn corpus_deltas_frame_to_the_end() {
    let mut fs = vec![];
    files(&corpus_root(), &mut fs);
    if fs.is_empty() {
        return;
    }
    let (mut with, mut nodes) = (0, 0);
    for p in &fs {
        if let Some(inv) = check(&p.display().to_string(), &std::fs::read(p).unwrap()) {
            with += 1;
            nodes += inv.nodes;
        }
    }
    // the 38-file corpus: every file carries deltas, 71 500 nodes in all (lower bounds)
    if fs.len() >= 38 {
        assert!(with >= 38, "{with}");
        assert!(nodes >= 70_000, "{nodes}");
    }
}

/// Volume of the partition body (deltas ignored, or one change rolled back) against SolidWorks' stored volume.
fn volume_error(bytes: &[u8], rule: DeltaRule) -> f64 {
    let lim = Limits::default();
    let c = container::open(bytes, &lim).unwrap();
    let ts = c.transmits(&lim).unwrap();
    let tp = ts.iter().find(|t| t.kind == TransmitKind::Partition && !t.stream.contains("Ghost")).unwrap();
    let td = ts.iter().find(|t| t.kind == TransmitKind::Deltas && t.stream == tp.stream).unwrap();
    let mut x = xt::read_xt(&tp.data, &lim).unwrap();
    xt::apply_deltas(&mut x, &xt::read_deltas(&td.data, &lim).unwrap(), rule);
    let ir = ck_sldprt::ir::extract(&x, &lim).unwrap();
    let built = ck_sldprt::build::build(&[&ir], &lim).unwrap();
    let sw = ck_sldprt::meta::read(&c, &lim).mass.unwrap().volume_m3 * 1e9;
    (ck_topo::mass::mass_props(&built.body).volume - sw) / sw
}

/// The partition is the current state. One undo step lands closer to the stored volume of the NIST parts because the last
/// feature (a cut) postdates it; applying every delta node over the partition lands far away. No rule is adopted.
#[test]
fn nist_ftc_09_rules() {
    let mut fs = vec![];
    files(&corpus_root(), &mut fs);
    let Some(p) = fs.iter().find(|p| p.display().to_string().contains("nist_ftc_09")) else { return };
    let b = std::fs::read(p).unwrap();
    let part = volume_error(&b, DeltaRule::PartitionOnly);
    assert!(part.abs() < 0.005 && part < 0.0, "partition {part}");
    let one = volume_error(&b, DeltaRule::RollBack(1));
    assert!(one.abs() < 0.0005, "one step back {one}");
    let all = volume_error(&b, DeltaRule::Overlay);
    assert!(all.abs() > 0.05, "overlay {all}");
}
