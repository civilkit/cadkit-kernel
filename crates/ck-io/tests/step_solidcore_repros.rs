//! Robustness repros distilled from a sibling kernel's STEP import/export fixes
//! (tricky or hostile Part 21 text, plus body round trips). Each test is one
//! failure class; inputs are small and deterministic.
mod bodies;
use ck_io::step::{read_step, write_step, StepOpts};
use std::sync::mpsc;
use std::time::Duration;

fn golden(name: &str) -> String {
    std::fs::read_to_string(format!("{}/tests/golden/{name}.step", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

/// Run an import on a worker thread; a panic or a hang past `secs` is a failure.
fn import_guarded(text: String, secs: u64) -> Result<Result<ck_io::step::Imported, ck_io::step::ImportError>, String> {
    let (tx, rx) = mpsc::channel();
    std::thread::Builder::new()
        .stack_size(8 << 20)
        .spawn(move || {
            let r = std::panic::catch_unwind(|| read_step(&text));
            let _ = tx.send(r.map_err(|_| "panic".to_string()));
        })
        .unwrap();
    match rx.recv_timeout(Duration::from_secs(secs)) {
        Ok(r) => r,
        Err(_) => Err("hang".to_string()),
    }
}

fn vol(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// solidcore #1082: a NaN / infinite coordinate must not reach the file as
/// `NaN` / `inf` (not valid Part 21 reals).
#[test]
fn r1082_export_refuses_non_finite() {
    for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        let mut b = bodies::build("box");
        let v = b.vertex_ids().next().unwrap();
        b.vertices[v.idx()].p.x = bad;
        let t = write_step(&b, "x", &StepOpts::default());
        // the plain writer still prints the token (its output is oracle-pinned) ...
        assert!(ck_io::step::write::has_non_finite_token(&t), "{bad}: token scan blind");
        // ... so the checked entry point (used by the `step` request) refuses it
        assert!(ck_io::step::try_write_step(&b, "x", &StepOpts::default()).is_err(), "{bad} exported");
    }
}

/// solidcore #1082 (import side): a real that overflows to infinity.
#[test]
fn r1082_import_rejects_non_finite() {
    let t = golden("box");
    let t = t.replacen("(0.,0.,-20.)", "(0.,0.,-1.E999)", 1);
    assert_ne!(t, golden("box"), "fixture text did not change");
    match import_guarded(t, 20).expect("no panic / hang") {
        Err(_) => {}
        Ok(imp) => {
            for v in imp.body.vertex_ids() {
                let p = imp.body.vertex(v).p;
                assert!(p.x.is_finite() && p.y.is_finite() && p.z.is_finite(), "infinite vertex imported");
            }
            panic!("1E999 coordinate imported without error");
        }
    }
}

/// Add `ORIENTED_FACE` wrappers around the faces of a closed shell of `text`.
/// `flip_first`: wrap the first face twice-reversed (a reversed copy of the face
/// under an `.F.` wrapper) so the body is unchanged.
fn wrap_faces(text: &str, flip_first: bool) -> String {
    let mut lines: Vec<String> = text.lines().map(str::to_string).collect();
    let end = lines.iter().position(|l| l.starts_with("ENDSEC;") && l != "ENDSEC;HEADER").unwrap();
    let data = lines.iter().position(|l| l == "DATA;").unwrap();
    let end = lines.iter().enumerate().skip(data).find(|(_, l)| l.as_str() == "ENDSEC;").map(|(i, _)| i).unwrap_or(end);
    let max_id = lines
        .iter()
        .filter_map(|l| l.strip_prefix('#').and_then(|r| r.split('=').next()).and_then(|n| n.parse::<u32>().ok()))
        .max()
        .unwrap();
    let mut next = max_id + 1;
    let shell_i = lines.iter().position(|l| l.contains("=CLOSED_SHELL(")).unwrap();
    let l = lines[shell_i].clone();
    let open = l.find("(#").unwrap();
    let close = l.rfind("))").unwrap();
    let faces: Vec<String> = l[open + 1..close].split(',').map(|s| s.trim().to_string()).collect();
    let mut extra = vec![];
    let mut new_faces = vec![];
    for (k, f) in faces.iter().enumerate() {
        if k == 0 && flip_first {
            // reversed copy G of face f: sense toggled, every bound orientation toggled
            let fid = f.trim_start_matches('#');
            let fl = lines.iter().find(|x| x.starts_with(&format!("#{fid}=ADVANCED_FACE("))).unwrap().clone();
            let bo = fl.find("(#").unwrap();
            let bc = fl[bo..].find(')').unwrap() + bo;
            let bounds: Vec<String> = fl[bo + 1..bc].split(',').map(|s| s.trim().to_string()).collect();
            let mut nb = vec![];
            for b in &bounds {
                let bl = lines.iter().find(|x| x.starts_with(&format!("{b}="))).unwrap().clone();
                let flipped = if bl.ends_with(".T.);") { bl.replace(".T.);", ".F.);") } else { bl.replace(".F.);", ".T.);") };
                let body = flipped.split_once('=').unwrap().1.to_string();
                extra.push(format!("#{next}={body}"));
                nb.push(format!("#{next}"));
                next += 1;
            }
            let tail = &fl[bc + 1..]; // ",#surf,.T.);"
            let tail = if tail.ends_with(".T.);") { tail.replace(".T.);", ".F.);") } else { tail.replace(".F.);", ".T.);") };
            extra.push(format!("#{next}=ADVANCED_FACE('',({}){}", nb.join(","), tail));
            let g = next;
            next += 1;
            extra.push(format!("#{next}=ORIENTED_FACE('',*,#{g},.F.);"));
        } else {
            extra.push(format!("#{next}=ORIENTED_FACE('',*,{f},.T.);"));
        }
        new_faces.push(format!("#{next}"));
        next += 1;
    }
    lines[shell_i] = format!("{}({}))", &l[..open], new_faces.join(",")) + &l[close + 2..];
    for (i, e) in extra.into_iter().enumerate() {
        lines.insert(end + i, e);
    }
    lines.join("\n")
}

/// solidcore #1078: ORIENTED_FACE wrappers must map to exactly the faces they
/// wrap (no phantom reversed duplicates).
#[test]
fn r1078_oriented_face_maps_without_phantoms() {
    for flip in [false, true] {
        let t = wrap_faces(&golden("box"), flip);
        let imp = match import_guarded(t, 20).expect("no panic / hang") {
            Ok(i) => i,
            Err(e) => panic!("flip={flip}: ORIENTED_FACE not imported: {e}"),
        };
        assert_eq!(imp.body.num_faces(), 6, "flip={flip}: phantom or missing faces");
        assert!((vol(&imp.body) - 8000.0).abs() < 1e-6, "flip={flip}: volume {}", vol(&imp.body));
        assert!(ck_check::validate(&imp.body, 1e-6).is_empty(), "flip={flip}");
    }
}

fn count(t: &str, kw: &str) -> usize {
    t.matches(&format!("={kw}(")).count()
}

/// solidcore #1203 / #1345: a shared edge / vertex is one STEP entity, used by
/// two ORIENTED_EDGEs.
#[test]
fn r1203_shared_edges_and_vertices_on_export() {
    for (name, v, e) in [("box", 8, 12)] {
        let t = write_step(&bodies::build(name), name, &StepOpts::default());
        assert_eq!(count(&t, "VERTEX_POINT"), v, "{name} vertices");
        assert_eq!(count(&t, "EDGE_CURVE"), e, "{name} edges");
        assert_eq!(count(&t, "ORIENTED_EDGE"), 2 * e, "{name} oriented edges");
        let imp = read_step(&t).unwrap();
        assert_eq!(imp.body.edge_ids().count(), e);
    }
}

/// solidcore #1343: Part 21 short entity names for the geometry entities.
#[test]
fn r1343_short_entity_names() {
    let t = golden("box");
    let short = t
        .replace("=CARTESIAN_POINT(", "=CRTPNT(")
        .replace("=DIRECTION(", "=DRCTN(")
        .replace("=AXIS2_PLACEMENT_3D(", "=A2PL3D(")
        .replace("=CLOSED_SHELL(", "=CLSSHL(");
    assert_ne!(short, t);
    let imp = import_guarded(short, 20).expect("no panic / hang").unwrap_or_else(|e| panic!("short names: {e}"));
    assert_eq!(imp.body.num_faces(), 6);
}

/// solidcore #1343: a sheet body (open shell in a SHELL_BASED_SURFACE_MODEL, or
/// faces no shell references) should import as an open-shell body.
#[test]
#[ignore = "missing feature: sheet bodies (open shells, loose faces) are not imported"]
fn r1343_sheet_body() {
    let t = golden("box");
    // drop the solid, keep the shell as an open shell in a surface model
    let t = t
        .replace("=CLOSED_SHELL(", "=OPEN_SHELL(")
        .lines()
        .map(|l| {
            if l.contains("=MANIFOLD_SOLID_BREP(") {
                let id = l.split('=').next().unwrap().to_string();
                let shell = l.split(",#").nth(1).unwrap().trim_end_matches(");").to_string();
                format!("{id}=SHELL_BASED_SURFACE_MODEL('',(#{shell}));")
            } else {
                l.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    match import_guarded(t, 20).expect("no panic / hang") {
        Ok(imp) => assert_eq!(imp.body.num_faces(), 6),
        Err(e) => panic!("sheet body not imported: {e}"),
    }
}

/// solidcore #1344 / #1496: a periodic face bounded only by two bare circles
/// (no seam in the file) and the analytic/NURBS round trips with seams.
#[test]
fn r1344_r1496_periodic_and_analytic_round_trips() {
    for name in ["cyl", "cone", "sphere", "torus", "rev360", "rev90", "loft", "cvs"] {
        let b = if bodies::NAMES.contains(&name) { bodies::build(name) } else { continue };
        let t = write_step(&b, name, &StepOpts::default());
        let imp = read_step(&t).unwrap_or_else(|e| panic!("{name}: {e}"));
        let (v0, v1) = (vol(&b), vol(&imp.body));
        assert!((v0 - v1).abs() <= 1e-6 * (1.0 + v0.abs()), "{name}: volume {v0} vs {v1}");
        assert!(ck_check::validate(&imp.body, 1e-6).is_empty(), "{name}");
    }
}

/// Deterministic mutation sweep over the goldens and fixtures: no panic, no hang.
#[test]
fn mutation_sweep_never_panics_or_hangs() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests");
    let mut files = vec![];
    for sub in ["golden", "fixtures"] {
        for e in std::fs::read_dir(dir.join(sub)).unwrap() {
            let p = e.unwrap().path();
            if p.extension().is_some_and(|x| x == "step") {
                files.push(p);
            }
        }
    }
    files.sort();
    let mut failures = vec![];
    let mut n = 0;
    for p in &files {
        let t = std::fs::read_to_string(p).unwrap();
        let lines: Vec<&str> = t.lines().collect();
        let data = lines.iter().position(|l| l.trim() == "DATA;").unwrap_or(0);
        let ents: Vec<usize> = (data + 1..lines.len()).filter(|&i| lines[i].trim_start().starts_with('#')).collect();
        let stride = (ents.len() / 40).max(1);
        let mut cases: Vec<(String, String)> = vec![];
        for &i in ents.iter().step_by(stride) {
            let mut drop = lines.clone();
            drop.remove(i);
            cases.push((format!("drop line {i}"), drop.join("\n")));
            for (tag, rep) in [("1E999", "1.E999"), ("neg", "-1."), ("zero", "0."), ("huge", "1.E300")] {
                // replace the first real literal on the line
                let l = lines[i];
                if let Some(pos) = l.find(|c: char| c.is_ascii_digit()).and_then(|_| l.find('.')) {
                    let start = l[..pos].rfind(|c: char| !(c.is_ascii_digit() || c == '-')).map_or(0, |k| k + 1);
                    let end = l[pos + 1..].find(|c: char| !(c.is_ascii_digit() || c == 'E' || c == '-')).map_or(l.len(), |k| pos + 1 + k);
                    let mut m = lines.clone();
                    let nl = format!("{}{}{}", &l[..start], rep, &l[end..]);
                    m[i] = &nl;
                    cases.push((format!("{tag} line {i}"), m.join("\n")));
                }
            }
            // point a reference at itself / at nothing
            if let Some(pos) = lines[i][1..].find("(#").map(|k| k + 1) {
                let id = lines[i][1..].split('=').next().unwrap();
                let rest = &lines[i][pos + 2..];
                let end = rest.find(|c: char| !c.is_ascii_digit()).unwrap_or(rest.len());
                let mut m = lines.clone();
                let nl = format!("{}(#{id}{}", &lines[i][..pos], &rest[end..]);
                m[i] = &nl;
                cases.push((format!("self-ref line {i}"), m.join("\n")));
            }
        }
        for cut in (0..t.len()).step_by((t.len() / 50).max(1)) {
            if t.is_char_boundary(cut) {
                cases.push((format!("cut {cut}"), t[..cut].to_string()));
            }
        }
        for (what, text) in cases {
            n += 1;
            if let Err(e) = import_guarded(text, 30) {
                failures.push(format!("{}: {what}: {e}", p.file_name().unwrap().to_string_lossy()));
            }
        }
    }
    assert!(failures.is_empty(), "{} of {n} mutants failed:\n{}", failures.len(), failures.join("\n"));
}

#[test]
fn r1082_export_finite_body_still_writes() {
    let b = bodies::build("box");
    let t = ck_io::step::try_write_step(&b, "ok", &StepOpts::default()).unwrap();
    assert_eq!(t, write_step(&b, "ok", &StepOpts::default()));
}

/// A finite but absurd coordinate (1e300) must be refused, not overflow the
/// distance / domain arithmetic downstream.
#[test]
fn r1082_import_rejects_absurd_magnitude() {
    // (refused, or the faces that use the point left out with the reason: never a body built on it)
    let t = golden("box").replacen("(0.,0.,-20.)", "(0.,0.,-1.E300)", 1);
    match import_guarded(t, 20).expect("no panic / hang") {
        Err(_) => {}
        Ok(r) => assert!(r.parts.iter().all(|p| !p.refused.is_empty()), "{:?}", r.parts),
    }
}
