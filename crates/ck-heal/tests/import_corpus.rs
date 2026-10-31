//! Import honesty measurement over a local corpus (not committed): every STEP, SolidWorks and SketchUp file under
//! `$CK_IMPORT_CORPUS` (default `<repo>/_corpus`) plus the kernel's own STEP fixtures is imported the way the
//! `importStep` / `importSldprt` / `importSkp` requests import it, and each file is classed:
//!
//! * `refused <code>`: the reader returned an error (nothing came in);
//! * `valid`: every part is a valid solid;
//! * `surface`: some part came in as a surface body (with its reasons), as `ck_heal::honest` sorts it; Repair Part is tried
//!   on each surface body and counted;
//! * `invalid`: a part was kept in the solid body that does not pass the validator (a half-built solid: must not happen).
//!
//! One line per file on stderr (written past the test harness's capture) and a tally at the end. The test only
//! fails when a reader panics. Skipped (passes) when no corpus is present.

use std::io::Write;
use std::path::{Path, PathBuf};

fn out(s: &str) {
    let _ = std::io::stderr().write_all(s.as_bytes());
}

fn walk(d: &Path, exts: &[&str], acc: &mut Vec<PathBuf>) {
    let Ok(rd) = std::fs::read_dir(d) else { return };
    let mut v: Vec<PathBuf> = rd.filter_map(|e| e.ok().map(|e| e.path())).collect();
    v.sort();
    for p in v {
        if p.is_dir() {
            walk(&p, exts, acc);
        } else if p.extension().and_then(|e| e.to_str()).map_or(false, |e| exts.iter().any(|x| x.eq_ignore_ascii_case(e))) {
            acc.push(p);
        }
    }
}

/// Issue codes of `b` (strict tolerance, as the heal pass judges), counted, e.g. `SHELL_OPEN x3`.
fn issues(b: &ck_topo::body::Body) -> (usize, String) {
    let is = ck_check::validate(b, 1e-6);
    let mut codes: Vec<&str> = is.iter().map(|i| i.code).collect();
    codes.sort();
    let mut parts: Vec<String> = vec![];
    let mut i = 0;
    while i < codes.len() {
        let j = (i..codes.len()).find(|&j| codes[j] != codes[i]).unwrap_or(codes.len());
        parts.push(format!("{} x{}", codes[i], j - i));
        i = j;
    }
    (is.len(), parts.join(", "))
}

/// Per lump: (valid lumps, invalid lumps, issue summary of the invalid ones).
fn lumps(b: &ck_topo::body::Body) -> (usize, usize, String) {
    let ls: Vec<_> = b.lump_ids().collect();
    let (mut ok, mut bad, mut why) = (0, 0, vec![]);
    for l in ls {
        let sub = ck_heal::honest::extract_lump(b, l);
        let (n, s) = issues(&sub);
        if n == 0 {
            ok += 1;
        } else {
            bad += 1;
            why.push(s);
        }
    }
    why.sort();
    why.dedup();
    (ok, bad, why.join(" | "))
}

#[derive(Default)]
struct Tally {
    files: usize,
    refused: usize,
    /// every part a valid solid
    valid: usize,
    /// some part came in as a surface body (honest: not kept as a solid)
    surface: usize,
    /// a part kept in the solid body that does not validate (must stay 0)
    invalid: usize,
    parts: [usize; 3],
    /// surface parts Repair Part made into valid solids, of those tried
    repaired: usize,
    tried: usize,
}

fn line(t: &mut Tally, kind: &str, p: &Path, root: &Path, status: &str, detail: String, ms: u128) {
    t.files += 1;
    match status {
        "valid" => t.valid += 1,
        "surface" => t.surface += 1,
        "invalid" => t.invalid += 1,
        _ => t.refused += 1,
    }
    let rel = p.strip_prefix(root).unwrap_or(p).display().to_string();
    out(&format!("IMPORT\t{kind}\t{rel}\t{status}\t{detail}\t{ms}ms\n"));
}

/// Sorted parts: tally them, check the solid body, try Repair Part on each surface body. Returns (status, detail).
fn judge(t: &mut Tally, s: ck_heal::honest::Sorted, left: &[Vec<ck_heal::repair::LeftOut>]) -> (&'static str, String) {
    use ck_heal::honest::Outcome;
    for p in &s.parts {
        t.parts[match p.outcome {
            Outcome::Solid => 0,
            Outcome::Surface => 1,
            Outcome::Refused => 2,
        }] += 1;
    }
    let (_, bad, why_bad) = lumps(&s.solid);
    let mut codes: Vec<String> = s.parts.iter().filter(|p| p.outcome != Outcome::Solid).flat_map(|p| p.reasons.iter().map(|r| r.code.clone())).collect();
    codes.sort();
    codes.dedup();
    let (mut fixed, mut tried) = (0, 0);
    let mut rep_codes: Vec<String> = vec![];
    for (k, b) in s.surfaces {
        if b.num_faces() > 5000 {
            continue;
        }
        tried += 1;
        let mut b = b;
        match ck_heal::repair::repair(&mut b, left.get(k).map_or(&[][..], |v| &v[..]), &ck_heal::repair::RepairOpts::default()) {
            Ok(r) if r.valid => fixed += 1,
            Ok(r) => rep_codes.extend(r.remaining.iter().map(|x| x.code.clone())),
            Err(e) => rep_codes.push(e.code),
        }
    }
    rep_codes.sort();
    rep_codes.dedup();
    t.repaired += fixed;
    t.tried += tried;
    let n = |o| s.parts.iter().filter(|p| p.outcome == o).count();
    let detail = format!(
        "parts solid {} surface {} refused {} reasons [{}] repaired {fixed}/{tried} left [{}]{}",
        n(Outcome::Solid),
        n(Outcome::Surface),
        n(Outcome::Refused),
        codes.join(","),
        rep_codes.join(","),
        if bad > 0 { format!(" SOLID-INVALID {why_bad}") } else { String::new() }
    );
    let st = if bad > 0 {
        "invalid"
    } else if n(Outcome::Solid) == s.parts.len() {
        "valid"
    } else {
        "surface"
    };
    (st, detail)
}

fn step_file(t: &mut Tally, p: &Path, root: &Path) {
    let Ok(bytes) = std::fs::read(p) else { return };
    let text = String::from_utf8_lossy(&bytes);
    let t0 = std::time::Instant::now();
    match ck_io::step::read_step(&text) {
        Err(e) => line(t, "step", p, root, &format!("refused {}", e.code), e.message.chars().take(140).collect(), t0.elapsed().as_millis()),
        Ok(imp) => {
            // the healing the `importStep` request does by default, then the parts sorted as the request sorts them
            let mut ho = ck_heal::HealOpts::new(1e-4);
            ho.tolerant = true;
            let r = ck_model::features::import_step::sort_step(imp, None, Some(&ho));
            let (st, d) = judge(t, r.sorted, &r.left_out);
            line(t, "step", p, root, st, d, t0.elapsed().as_millis());
        }
    }
}

fn sldprt_file(t: &mut Tally, p: &Path, root: &Path) {
    use ck_heal::honest::{refused_face, PartIn};
    let Ok(bytes) = std::fs::read(p) else { return };
    let t0 = std::time::Instant::now();
    match ck_sldprt::import_sldprt(&bytes) {
        Err(e) => line(t, "sldprt", p, root, &format!("refused {}", e.code), e.message.chars().take(140).collect(), t0.elapsed().as_millis()),
        Ok(imp) => {
            let mut body = imp.body;
            ck_topo::region::rebuild_lumps(&mut body);
            // as the `importSldprt` request does
            ck_heal::heal_with(&mut body, &ck_heal::tolerant_only());
            let ins: Vec<PartIn> = imp
                .parts
                .iter()
                .map(|q| PartIn { label: format!("#{}", q.body), faces: q.faces.clone(), faces_total: q.faces_total, reasons: q.refused.iter().map(|(f, e)| refused_face(e.code, format!("F{f}"), &e.message)).collect() })
                .collect();
            let left: Vec<Vec<ck_heal::repair::LeftOut>> = imp.parts.iter().map(|q| q.refused_surfaces.iter().map(|(f, s, se)| ck_heal::repair::LeftOut { id: format!("F{f}"), surface: s.clone(), sense: *se }).collect()).collect();
            let s = ck_heal::honest::sort(&body, &ins, ck_check::DEFAULT_TOL_CAP);
            let (st, d) = judge(t, s, &left);
            line(t, "sldprt", p, root, st, d, t0.elapsed().as_millis());
        }
    }
}

fn skp_file(t: &mut Tally, p: &Path, root: &Path) {
    use ck_heal::honest::PartIn;
    let Ok(bytes) = std::fs::read(p) else { return };
    let t0 = std::time::Instant::now();
    let lim = ck_skp::Limits::default();
    let r = ck_skp::parse_with(&bytes, &lim)
        .and_then(|m| ck_skp::bake(&m, &lim, &ck_skp::BakeOptions { keep_polygons: true, ..Default::default() }))
        .and_then(|s| ck_skp::body::build_body(&s, &ck_skp::body::BodyOptions::default()));
    match r {
        Err(e) => line(t, "skp", p, root, &format!("refused {}", e.code), e.message.chars().take(140).collect(), t0.elapsed().as_millis()),
        Ok(sb) => {
            let body = sb.body;
            if body.num_faces() > 20_000 {
                line(t, "skp", p, root, "surface", format!("faces {} (too many to sort)", body.num_faces()), t0.elapsed().as_millis());
                return;
            }
            let ins: Vec<PartIn> = body
                .lump_ids()
                .enumerate()
                .map(|(k, l)| {
                    let faces = ck_topo::mass::lump_faces(&body, l);
                    PartIn { label: format!("piece {}", k + 1), faces_total: faces.len(), faces, reasons: vec![] }
                })
                .collect();
            let s = ck_heal::honest::sort(&body, &ins, ck_check::DEFAULT_TOL_CAP);
            let (st, d) = judge(t, s, &[]);
            line(t, "skp", p, root, st, d.chars().take(400).collect(), t0.elapsed().as_millis());
        }
    }
}

#[test]
fn import_corpus_report() {
    let repo = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let root = std::env::var("CK_IMPORT_CORPUS").map(PathBuf::from).unwrap_or_else(|_| repo.join("_corpus"));
    if !root.is_dir() {
        out(&format!("import_corpus: no corpus at {} (skipped)\n", root.display()));
        return;
    }
    let only = std::env::var("CK_IMPORT_ONLY").unwrap_or_default();
    let mut steps = vec![];
    walk(&root, &["step", "stp"], &mut steps);
    for d in ["crates/ck-io/tests/fixtures", "crates/ck-io/tests/golden", "tools/bench/benchcad/gt", "tools/bench/brepjs/gt"] {
        walk(&repo.join(d), &["step", "stp"], &mut steps);
    }
    let mut sw = vec![];
    walk(&root, &["sldprt"], &mut sw);
    let mut skp = vec![];
    walk(&root, &["skp"], &mut skp);
    let (mut ts, mut tw, mut tk) = (Tally::default(), Tally::default(), Tally::default());
    for p in steps.iter().filter(|p| p.display().to_string().contains(&only)) {
        step_file(&mut ts, p, &repo);
    }
    for p in sw.iter().filter(|p| p.display().to_string().contains(&only)) {
        sldprt_file(&mut tw, p, &repo);
    }
    for p in skp.iter().filter(|p| p.display().to_string().contains(&only)) {
        skp_file(&mut tk, p, &repo);
    }
    for (k, t) in [("step", &ts), ("sldprt", &tw), ("skp", &tk)] {
        out(&format!(
            "TALLY\t{k}\tfiles {}\tvalid {}\tsurface {}\tinvalid {}\trefused {}\tparts solid {} surface {} refused {}\trepaired {}/{}\n",
            t.files, t.valid, t.surface, t.invalid, t.refused, t.parts[0], t.parts[1], t.parts[2], t.repaired, t.tried
        ));
    }
}
