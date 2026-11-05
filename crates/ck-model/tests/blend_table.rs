//! Fillet regression table: synthetic bodies (petal outlines, box corner, bracket, pocket, cone pocket, pipe tee,
//! feet sector ...) x edge sets x radii, each recorded with the outcome of the blend engine today.
//!
//! `fixtures/blend_cases.json` holds the cases (base features + the fillet feature with resolved edge picks + the
//! volume of a reference engine where one exists; `tools/gen_blend_fixtures.mjs` writes it).
//! `fixtures/blend_expect.json` holds, per case id, `"pass"`, `"expected_fail"` or `"hard"` (+ the observed
//! outcome as a note; a `hard` entry also carries a `reason` and optionally `alsoFails`).
//!
//! A case PASSES when the fillet feature reports no error, the body validates, the volume is positive, and, with a
//! reference volume, the volume removed (or added) agrees with the reference within 0.2 %. A case marked `"sharp": true`
//! (or every case with `BLEND_SHARP=1`) also fails when a selected edge is left sharp (points along it still on the
//! result's boundary).
//!
//! The test fails when
//! - a case recorded as `pass` no longer passes (a regression), or
//! - a case recorded as `expected_fail` / `hard` passes (progress: flip it to `pass` in `blend_expect.json`), or
//! - a case has no expectation, or (unfiltered runs only) an expectation has no case.
//!
//! Environment (see docs/blend-table.md):
//! - `BLEND_CASE=a,b`: run only the cases whose id contains `a` or `b` (`=id`: exactly that id, `^p`: id starts with `p`)
//! - `BLEND_MODE=fail|pass`: run only the cases expected to fail (`expected_fail` + `hard`) / to pass
//! - `BLEND_JSON=<file>`: write the per-case results as JSON
//! - `BLEND_DEBUG_DIR=<dir>`: also run each selected case in its own process with `CK_BLEND_DEBUG=1`, its output to
//!   `<dir>/<id>.log`
//! - `BLEND_TABLE_OUT=<file>`: write the observed table as an expectation file (to regenerate it after a change)
//!
//! One compact line per case and a summary with a failure-mode cluster table are printed (`--nocapture`).
use ck_json::{Json, Obj};
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}

const CASES: &str = include_str!("fixtures/blend_cases.json");
const EXPECT: &str = include_str!("fixtures/blend_expect.json");

/// What one run of a case gave.
struct Outcome {
    /// first error of the regeneration (`code`, plus its `detail` when it has one), if any
    code: Option<String>,
    valid: bool,
    volume: f64,
    base_volume: f64,
    secs: f64,
    /// validation findings when the result does not validate (`ck_check::validate`), for the detail text
    issues: Vec<String>,
    /// selected edges still sharp in the result (`sharp` cases, or every case with `BLEND_SHARP=1`)
    sharp: Vec<String>,
    faces: usize,
}

/// The selected edges that the result leaves sharp: points along each picked edge of the base body that still lie on
/// the result's boundary (a rounded convex edge's points are outside the result, a concave one's inside it).
fn sharp_edges(base: &ck_topo::body::Body, result: &ck_topo::body::Body, fillet: &Json) -> Vec<String> {
    let mut out = vec![];
    for pick in fillet.get_arr("edges").into_iter().flatten() {
        let names: Vec<&str> = pick.get_arr("faces").into_iter().flatten().filter_map(|f| f.as_str()).collect();
        let (Some(f0), Some(f1), Some([x, y, z])) = (names.first().and_then(|n| base.find_face(n)), names.get(1).and_then(|n| base.find_face(n)), pick.get("at").and_then(|a| a.as_vec3())) else { continue };
        let at = ck_geom::Vec3::new(x, y, z);
        let Some(e) = base.face_edges(f0).into_iter().filter(|&e| base.edge_faces(e).contains(&f1)).min_by(|&a, &b| base.edge_mid(a).dist(at).total_cmp(&base.edge_mid(b).dist(at))) else { continue };
        let ed = base.edge(e);
        let hits: Vec<usize> = (1..20)
            .filter(|&k| {
                let q = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 20.0);
                ck_topo::region::classify(result, q, 1e-4) == ck_topo::region::Containment::On
            })
            .collect();
        if !hits.is_empty() {
            out.push(format!("{}/{} sharp at {}/20", names[0], names[1], hits.iter().map(|k| k.to_string()).collect::<Vec<_>>().join(",")));
        }
    }
    out
}

fn regen_doc(features: &[Json]) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(features.to_vec())));
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r
        .errors
        .iter()
        .map(|e| match e.error.params.iter().find(|(k, _)| k == "detail").and_then(|(_, v)| v.as_str()) {
            Some(d) => format!("{} ({d})", e.error.code),
            None => e.error.code.clone(),
        })
        .collect();
    (ex.0.body.clone(), errs)
}

fn run_case(bases: &Json, case: &Json) -> Outcome {
    let t = Instant::now();
    let base = bases.get(case.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
    let (b0, e0) = regen_doc(&base);
    assert!(e0.is_empty(), "{}: the base body fails: {e0:?}", case.get_str("id").unwrap());
    let base_volume = ck_topo::mass::mass_props(b0.as_ref().unwrap()).volume;
    let check_sharp = case.get("sharp").is_some_and(|v| v == &Json::Bool(true)) || std::env::var("BLEND_SHARP").is_ok_and(|v| v == "1");
    let mut feats = base;
    if let Some(pre) = case.get("pre") {
        feats.push(pre.clone());
    }
    feats.push(case.get("fillet").unwrap().clone());
    let (body, errs) = regen_doc(&feats);
    let mut o = match (body, errs.first()) {
        (Some(b), None) => {
            let issues: Vec<String> = ck_check::validate(&b, 1e-6).into_iter().map(|i| format!("{} {}: {}", i.code, i.entity, i.detail)).collect();
            let sharp = if check_sharp && case.get("pre").is_none() { sharp_edges(b0.as_ref().unwrap(), &b, case.get("fillet").unwrap()) } else { vec![] };
            Outcome {
                code: None,
                valid: issues.is_empty(),
                volume: ck_topo::mass::mass_props(&b).volume,
                base_volume,
                secs: 0.0,
                issues,
                sharp,
                faces: b.num_faces(),
            }
        }
        (_, e) => Outcome { code: Some(e.cloned().unwrap_or_else(|| "noBody".into())), valid: false, volume: 0.0, base_volume, secs: 0.0, issues: vec![], sharp: vec![], faces: 0 },
    };
    o.secs = t.elapsed().as_secs_f64();
    o
}

/// The reference volume and the base volume it is measured against.
fn reference(case: &Json, o: &Outcome) -> Option<(f64, f64)> {
    let rv = case.get("reference").and_then(|r| r.get_f64("volume"))?;
    Some((rv, case.get("reference").and_then(|r| r.get_f64("base_volume")).unwrap_or(o.base_volume)))
}

/// `None`: passes; `Some(why)`: the reason it does not.
fn judge(case: &Json, o: &Outcome) -> Option<String> {
    if let Some(c) = &o.code {
        return Some(format!("refused: {c}"));
    }
    if !o.valid {
        // keep "invalid body" as the prefix: `failure_mode` keys the cluster label off the code, and the detail
        // text carries the first findings so a case that stops refusing but still fails validation says why.
        let mut d = "invalid body".to_string();
        if !o.issues.is_empty() {
            d.push_str(": ");
            d.push_str(&o.issues.iter().take(3).cloned().collect::<Vec<_>>().join("; "));
        }
        return Some(d);
    }
    if !(o.volume > 0.0) {
        return Some("no volume".into());
    }
    if !o.sharp.is_empty() {
        return Some(format!("selected edge left sharp: {}", o.sharp.join("; ")));
    }
    if let Some((rv, v0)) = reference(case, o) {
        let change = (v0 - rv).abs();
        let diff = (o.volume - rv).abs();
        if diff > 2e-3 * change + 1e-6 * v0 {
            return Some(format!("volume {:.4} vs reference {rv:.4} (base {v0:.4})", o.volume));
        }
    }
    None
}

/// Outcome code (`ok`, an error code, `invalid`, `novolume`, `volume`) and its detail.
fn code_detail(o: &Outcome, why: &Option<String>) -> (String, String) {
    match (&o.code, why) {
        (Some(c), _) => match c.split_once(" (") {
            Some((code, d)) => (code.to_string(), d.strip_suffix(')').unwrap_or(d).to_string()),
            None => (c.clone(), String::new()),
        },
        (None, None) => ("ok".into(), String::new()),
        (None, Some(w)) if !o.valid => ("invalid".into(), w.clone()),
        (None, Some(w)) if !(o.volume > 0.0) => ("novolume".into(), w.clone()),
        (None, Some(w)) if !o.sharp.is_empty() => ("sharp".into(), w.clone()),
        (None, Some(w)) => ("volume".into(), w.clone()),
    }
}

/// The failure mode of an outcome: a short, stable label that groups cases failing the same way.
fn failure_mode(code: &str, detail: &str) -> String {
    let d = detail.to_lowercase();
    let known = [
        ("branching blend selection", "branching selection"),
        ("concave chain end", "concave chain end"),
        ("centre did not converge", "centre did not converge"),
        ("singular centre", "singular centre"),
    ];
    if let Some((_, m)) = known.iter().find(|(k, _)| d.contains(k)) {
        return m.to_string();
    }
    if let Some(i) = d.find("is used by ") {
        let n: String = d[i + 11..].chars().take_while(|c| c.is_ascii_digit()).collect();
        return format!("edge used by {n}");
    }
    match code {
        "ok" => return "ok".into(),
        "volume" => return "volume mismatch".into(),
        "invalid" => return "invalid body".into(),
        "novolume" => return "no volume".into(),
        "sharp" => return "edge left sharp".into(),
        "blendOverflow" if d.contains("spring") => return "spring overflow".into(),
        _ => {}
    }
    // first three words, without tokens naming a particular face / edge (`F90.B7:`, `#214`)
    let words: Vec<&str> = d
        .split_whitespace()
        .filter(|w| !w.contains('#') && !(w.ends_with(':') && w.contains('.')))
        .take(3)
        .map(|w| w.trim_matches(|c: char| !c.is_alphanumeric()))
        .collect();
    if words.is_empty() {
        format!("{code} (no detail)")
    } else {
        words.join(" ")
    }
}

/// `BLEND_CASE`: comma-separated terms, `=id` exact, `^p` prefix, else a substring of the id.
fn selected(id: &str, filter: &str) -> bool {
    filter.split(',').map(str::trim).filter(|t| !t.is_empty()).any(|t| {
        if let Some(x) = t.strip_prefix('=') {
            id == x
        } else if let Some(p) = t.strip_prefix('^') {
            id.starts_with(p)
        } else {
            id.contains(t)
        }
    })
}

fn short(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        s.to_string()
    } else {
        format!("{}...", s.chars().take(n - 3).collect::<String>())
    }
}

/// Runs one case again in its own process with `CK_BLEND_DEBUG=1`, its stdout and stderr to `<dir>/<id>.log`.
fn debug_dump(dir: &str, id: &str) {
    let path = std::path::Path::new(dir).join(format!("{id}.log"));
    let Ok(f) = std::fs::File::create(&path) else { return };
    let Ok(f2) = f.try_clone() else { return };
    let mut cmd = std::process::Command::new(std::env::current_exe().unwrap());
    cmd.args(["blend_regression_table", "--exact", "--nocapture", "--test-threads=1"]);
    for k in ["BLEND_MODE", "BLEND_JSON", "BLEND_DEBUG_DIR", "BLEND_TABLE_OUT"] {
        cmd.env_remove(k);
    }
    let _ = cmd.env("BLEND_CASE", format!("={id}")).env("CK_BLEND_DEBUG", "1").stdout(f).stderr(f2).status();
}

#[test]
fn blend_regression_table() {
    let cases = ck_json::parse(CASES).unwrap();
    let expect = ck_json::parse(EXPECT).unwrap();
    let bases = cases.get("bases").unwrap();
    let all = cases.get_arr("cases").unwrap();
    let env = |k: &str| std::env::var(k).ok().filter(|v| !v.is_empty());
    let (filter, mode, debug_dir) = (env("BLEND_CASE"), env("BLEND_MODE"), env("BLEND_DEBUG_DIR"));
    let want_of = |id: &str| expect.get(id).and_then(|e| e.get_str("expect")).map(str::to_string);
    let list: Vec<&Json> = all
        .iter()
        .filter(|c| {
            let id = c.get_str("id").unwrap();
            let w = want_of(id);
            filter.as_deref().map_or(true, |f| selected(id, f))
                && match mode.as_deref() {
                    Some("fail") => matches!(w.as_deref(), Some("expected_fail" | "hard")),
                    Some("pass") => w.as_deref() == Some("pass"),
                    _ => true,
                }
        })
        .collect();
    let filtered = filter.is_some() || mode.is_some();
    if let Some(d) = &debug_dir {
        // the logs of an earlier run would read as this run's
        std::fs::create_dir_all(d).unwrap();
        for e in std::fs::read_dir(d).unwrap().flatten() {
            if e.path().extension().is_some_and(|x| x == "log") {
                let _ = std::fs::remove_file(e.path());
            }
        }
    }
    let mut problems: Vec<String> = vec![];
    let (mut pass, mut xfail, mut hard, mut table) = (0, 0, 0, String::new());
    let t0 = Instant::now();
    // the cases are independent: run them on all cores, report in file order
    let results: Vec<Outcome> = {
        let next = std::sync::atomic::AtomicUsize::new(0);
        let slots: Vec<std::sync::Mutex<Option<Outcome>>> = list.iter().map(|_| std::sync::Mutex::new(None)).collect();
        let workers = std::thread::available_parallelism().map_or(4, |n| n.get()).min(16);
        std::thread::scope(|sc| {
            for _ in 0..workers {
                // the blend code recurses deeply on big selections: give each worker a big stack
                std::thread::Builder::new()
                    .stack_size(64 << 20)
                    .spawn_scoped(sc, || loop {
                        let i = next.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
                        let Some(case) = list.get(i) else { break };
                        *slots[i].lock().unwrap() = Some(run_case(bases, case));
                        if let Some(d) = &debug_dir {
                            debug_dump(d, case.get_str("id").unwrap());
                        }
                    })
                    .unwrap();
            }
        });
        slots.into_iter().map(|m| m.into_inner().unwrap().unwrap()).collect()
    };
    let mut clusters: Vec<(String, Vec<String>)> = vec![];
    let mut rows_json: Vec<Json> = vec![];
    println!("blend table: {} of {} cases selected", list.len(), all.len());
    for (case, o) in list.iter().zip(results) {
        let id = case.get_str("id").unwrap().to_string();
        let why = judge(case, &o);
        let want = want_of(&id);
        let note = why.clone().unwrap_or_else(|| "passes".to_string());
        let (code, detail) = code_detail(&o, &why);
        let ok = why.is_none();
        let status = match (want.as_deref(), ok) {
            (Some("pass"), true) => {
                pass += 1;
                "PASS"
            }
            (Some("expected_fail"), false) => {
                xfail += 1;
                "XFAIL"
            }
            (Some("hard"), false) => {
                hard += 1;
                "XFAIL"
            }
            (Some("pass"), false) => {
                problems.push(format!("REGRESSION {id}: {note}"));
                "REGRESSION"
            }
            (Some("expected_fail" | "hard"), true) => {
                problems.push(format!("PROGRESS {id} now passes: set it to \"pass\" in blend_expect.json"));
                "NEWPASS"
            }
            (None, _) => {
                problems.push(format!("{id}: no expectation recorded (observed: {note})"));
                "NOEXPECT"
            }
            (Some(w), _) => {
                problems.push(format!("{id}: unknown expectation {w:?}"));
                "BADEXPECT"
            }
        };
        let rf = reference(case, &o);
        // error of the volume change against the reference's, in % of it (positive: less volume than the reference)
        let dvol = match rf {
            Some((rv, v0)) if o.code.is_none() && (v0 - rv).abs() > 0.0 => {
                let d = ((v0 - o.volume) - (v0 - rv)) / (v0 - rv).abs() * 100.0;
                if d.abs() < 5e-4 { "0.000%".into() } else { format!("{d:+.3}%") }
            }
            _ => "-".into(),
        };
        println!("{id} {status} {code} {} dvol={dvol}  t={:.2}", if detail.is_empty() { "-".into() } else { short(&detail, 80) }, o.secs);
        if !ok {
            let m = failure_mode(&code, &detail);
            match clusters.iter_mut().find(|(k, _)| *k == m) {
                Some((_, ids)) => ids.push(id.clone()),
                None => clusters.push((m, vec![id.clone()])),
            }
        }
        rows_json.push(
            Obj::new()
                .put("id", id.as_str())
                .put("status", status)
                .put("expect", want.clone().map_or(Json::Null, Json::Str))
                .put("outcome", code.as_str())
                .put("detail", detail.as_str())
                .put("mode", if ok { "ok".to_string() } else { failure_mode(&code, &detail) })
                .put("valid", o.valid)
                .put("volume", if o.code.is_none() { Json::Num(o.volume) } else { Json::Null })
                .put("base_volume", o.base_volume)
                .put("reference", rf.map_or(Json::Null, |(rv, _)| Json::Num(rv)))
                .put("reference_base", rf.map_or(Json::Null, |(_, v0)| Json::Num(v0)))
                .put("secs", o.secs)
                .put("faces", o.faces as f64)
                .build(),
        );
        // the observed table; a hard entry that still fails keeps its status and reason
        let row = match expect.get(&id) {
            Some(e) if want.as_deref() == Some("hard") && !ok => {
                let mut e = e.clone();
                e.set("observed", note.as_str());
                e
            }
            _ => Obj::new().put("expect", if ok { "pass" } else { "expected_fail" }).put("observed", note.as_str()).build(),
        };
        table.push_str(&format!("{}{}: {}", if table.is_empty() { "{\n " } else { ",\n " }, ck_json::to_string(&Json::Str(id.clone())), ck_json::to_string(&row)));
    }
    if !filtered {
        if let Some(e) = expect.as_obj() {
            for (k, _) in e.iter() {
                if !all.iter().any(|c| c.get_str("id") == Some(k)) {
                    problems.push(format!("{k}: expectation without a case"));
                }
            }
        }
    }
    if let Some(path) = env("BLEND_TABLE_OUT") {
        std::fs::write(path, format!("{table}\n}}\n")).unwrap();
    }
    if let Some(path) = env("BLEND_JSON") {
        if let Some(d) = std::path::Path::new(&path).parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(d).unwrap();
        }
        std::fs::write(path, ck_json::to_string(&Json::Arr(rows_json)) + "\n").unwrap();
    }
    clusters.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then(a.0.cmp(&b.0)));
    if !clusters.is_empty() {
        println!("clusters (failure mode, count, first ids):");
        for (m, ids) in &clusters {
            let more = if ids.len() > 5 { format!(" +{}", ids.len() - 5) } else { String::new() };
            println!("  {m:<26} {:>3}  {}{more}", ids.len(), ids.iter().take(5).cloned().collect::<Vec<_>>().join(" "));
        }
    }
    for p in &problems {
        println!("problem: {p}");
    }
    println!(
        "blend table: {} cases, {pass} pass, {xfail} expected_fail, {hard} hard, {} problems (target {xfail}){}  t={:.1}",
        list.len(),
        problems.len(),
        if filtered { " [filtered]" } else { "" },
        t0.elapsed().as_secs_f64()
    );
    println!("blend table end");
    assert!(problems.is_empty(), "{} problems (listed above)", problems.len());
}
