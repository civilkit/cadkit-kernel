//! Wave-2 STEP read hardening: cases taken from a sibling kernel's recent STEP
//! fixes (Part 21 reals without digits, vertex loops, short arcs, round faces
//! off their frame's quarter lines, closed shells in surface models) written
//! as small hand-edited STEP text, plus a truncation / mutation fuzz sweep.
//! Every malformed input must give a structured error, never a panic, a hang
//! or a silently wrong body.
use ck_io::step::{read_step, ImportError, Imported};
use std::sync::mpsc;
use std::time::Duration;

fn golden(name: &str) -> String {
    std::fs::read_to_string(format!("{}/tests/golden/{name}.step", env!("CARGO_MANIFEST_DIR"))).unwrap()
}

/// Import on a worker thread; Err(String) means panic or hang.
fn guarded(text: String) -> Result<Result<Imported, ImportError>, String> {
    let (tx, rx) = mpsc::channel();
    std::thread::Builder::new()
        .stack_size(8 << 20)
        .spawn(move || {
            let r = std::panic::catch_unwind(|| read_step(&text));
            let _ = tx.send(r.map_err(|_| "panic".to_string()));
        })
        .unwrap();
    rx.recv_timeout(Duration::from_secs(30)).unwrap_or_else(|_| Err("hang".to_string()))
}

/// The import says a part is not what the file claims: faces left out with their reasons, or a part that could not be read.
/// (A malformed file is either refused outright or flagged this way, never a clean solid.)
fn flagged(imp: &Imported) -> bool {
    imp.parts.iter().any(|p| !p.refused.is_empty() || p.error.is_some())
}

fn vol(b: &ck_topo::body::Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// Insert extra entity lines before the closing `ENDSEC;` of the DATA section.
fn add_entities(t: &str, extra: &[String]) -> String {
    let data = t.find("\nDATA;").unwrap();
    let end = data + t[data..].find("\nENDSEC;").unwrap();
    format!("{}\n{}{}", &t[..end], extra.join("\n"), &t[end..])
}

// ---- solidcore #1554: reals without digits -------------------------------

#[test]
fn r1554_reals_without_digits_are_refused() {
    for bad in [".", "-.", "+.", ".E5", "-.e-3", "+.E+2"] {
        let t = golden("box").replacen("(0.,0.,-20.)", &format!("(0.,{bad},-20.)"), 1);
        assert!(t.contains(&format!(",{bad},")), "fixture not edited for {bad}");
        match guarded(t).expect("no panic / hang") {
            Err(e) => assert!(!e.to_string().is_empty(), "{bad}: empty error"),
            Ok(imp) => assert!(flagged(&imp), "{bad}: accepted, {} faces, volume {}", imp.body.num_faces(), vol(&imp.body)),
        }
    }
    // a whole list with a digitless member
    let t = golden("box").replacen("(0.,0.,-20.)", "(1.,.,3.)", 1);
    assert!(guarded(t).expect("no panic / hang").map_or(true, |imp| flagged(&imp)));
}

#[test]
fn r1554_digit_forms_still_read() {
    // `1.`, `0.0`, `-0.`, `+0.0E0`, `0.E0` all hold a digit
    for ok in ["0.", "0.0", "-0.", "+0.0E0", "0.E0", "00.", "-0.0e-0"] {
        let t = golden("box").replacen("(0.,0.,-20.)", &format!("(0.,{ok},-20.)"), 1);
        let imp = guarded(t).expect("no panic / hang").unwrap_or_else(|e| panic!("{ok}: {e}"));
        assert!((vol(&imp.body) - 8000.0).abs() < 1e-6, "{ok}: volume {}", vol(&imp.body));
    }
}

// ---- solidcore #1558: vertex loops against the body's vertices ------------

/// The cylinder with an extra FACE_BOUND on its first plane, whose loop is
/// `loop_line` (a VERTEX_LOOP or anything else).
fn cyl_with_vertex_loop(loop_line: &str) -> String {
    let t = golden("cyl");
    assert!(t.contains("#32=ADVANCED_FACE('',(#31),#22,.F.);"));
    let t = t.replace("#32=ADVANCED_FACE('',(#31),#22,.F.);", "#32=ADVANCED_FACE('',(#31,#901),#22,.F.);");
    add_entities(&t, &[loop_line.to_string(), "#901=FACE_BOUND('',#900,.T.);".to_string()])
}

#[test]
fn r1558_vertex_loop_of_a_real_vertex_is_tolerated() {
    let imp = guarded(cyl_with_vertex_loop("#900=VERTEX_LOOP('',#24);")).expect("no panic / hang").expect("imports");
    assert!((vol(&imp.body) - std::f64::consts::PI * 100.0 * 10.0).abs() < 1e-6);
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

#[test]
fn r1558_vertex_loop_of_a_missing_vertex_is_refused() {
    // reference past the end of the entity table
    let r = guarded(cyl_with_vertex_loop("#900=VERTEX_LOOP('',#99999);")).expect("no panic / hang");
    assert!(r.map_or(true, |imp| flagged(&imp)), "vertex loop of a missing entity accepted");
    // a reference to something that is not a vertex
    let r = guarded(cyl_with_vertex_loop("#900=VERTEX_LOOP('',#23);")).expect("no panic / hang");
    assert!(r.map_or(true, |imp| flagged(&imp)), "vertex loop of a CARTESIAN_POINT accepted");
    // no reference at all
    let r = guarded(cyl_with_vertex_loop("#900=VERTEX_LOOP('',$);")).expect("no panic / hang");
    assert!(r.map_or(true, |imp| flagged(&imp)), "vertex loop without a vertex accepted");
}

// ---- solidcraft #8566 / #8456: short arcs, solids closing within tolerance --

/// The cylinder golden with both circle edges split in two by a second vertex
/// `arc` mm of arc length from the first (a short edge and a long one).
fn split_can(arc: f64) -> String {
    let mut t = golden("cyl");
    let th = arc / 10.0;
    let (x, z) = (5.0 + 10.0 * th.cos(), -3.0 - 10.0 * th.sin());
    let mut extra = vec![];
    let mut oe = 200usize;
    for (v0, edge, y, circle, n) in [(24, 28, 0.0, 27, 100), (37, 41, 10.0, 40, 110)] {
        extra.push(format!("#{}=CARTESIAN_POINT('',({x:.15},{y:.1},{z:.15}));", n + 1));
        extra.push(format!("#{}=VERTEX_POINT('',#{});", n + 2, n + 1));
        extra.push(format!("#{}=EDGE_CURVE('',#{v0},#{},#{circle},.T.);", n + 3, n + 2));
        extra.push(format!("#{}=EDGE_CURVE('',#{},#{v0},#{circle},.T.);", n + 4, n + 2));
        // the original edge becomes the short arc v0 -> new, the extra one the long arc
        t = t.replace(&format!("#{edge}=EDGE_CURVE('',#{v0},#{v0},#{circle},.T.);"), &format!("#{edge}=EDGE_CURVE('',#{v0},#{},#{circle},.T.);", n + 2));
        // loops: plane face (.F. on the bottom, .T. on the top) and the wall bounds
        let (plane_o, wall_o, plane_l, wall_l) = if edge == 28 { (29, 50, 30, 51) } else { (42, 53, 43, 54) };
        let plane_rev = t.contains(&format!("#{plane_o}=ORIENTED_EDGE('',*,*,#{edge},.F.);"));
        let wall_rev = t.contains(&format!("#{wall_o}=ORIENTED_EDGE('',*,*,#{edge},.F.);"));
        for (o, l, rev) in [(plane_o, plane_l, plane_rev), (wall_o, wall_l, wall_rev)] {
            let (short_s, long_s) = if rev { (".F.", ".F.") } else { (".T.", ".T.") };
            let (a, b) = (oe, oe + 1);
            oe += 2;
            extra.push(format!("#{a}=ORIENTED_EDGE('',*,*,#{edge},{short_s});"));
            extra.push(format!("#{b}=ORIENTED_EDGE('',*,*,#{},{long_s});", n + 4));
            let loop_new = if rev { format!("(#{b},#{a})") } else { format!("(#{a},#{b})" ) };
            let old_o = format!("#{o}=ORIENTED_EDGE('',*,*,#{edge},{});", if rev { ".F." } else { ".T." });
            assert!(t.contains(&old_o), "missing {old_o}");
            t = t.replace(&format!("#{l}=EDGE_LOOP('',(#{o}));"), &format!("#{l}=EDGE_LOOP('',{loop_new});"));
        }
    }
    add_entities(&t, &extra)
}

#[test]
fn split_can_baseline_imports() {
    // a quarter-sized split is just a can with extra vertices
    let imp = guarded(split_can(4.0)).expect("no panic / hang").unwrap_or_else(|e| panic!("{e}"));
    let v0 = std::f64::consts::PI * 100.0 * 10.0;
    assert!((vol(&imp.body) - v0).abs() < 1e-3 * v0, "volume {}", vol(&imp.body));
}

#[test]
fn r8566_short_arc_between_two_vertices_is_an_arc() {
    let v0 = std::f64::consts::PI * 100.0 * 10.0;
    // 0.02 mm (20 um), 2 um, and below the importer's vertex merge distance
    for arc in [0.02, 0.002, 5e-7, 5e-8] {
        match guarded(split_can(arc)).expect("no panic / hang") {
            Err(e) => assert!(!e.to_string().is_empty(), "arc {arc}: empty error"),
            Ok(imp) if flagged(&imp) => {}
            Ok(imp) => {
                let v = vol(&imp.body);
                assert!((v - v0).abs() < 1e-3 * v0, "arc {arc}: silently wrong body, volume {v} vs {v0}");
                let errs = ck_check::validate(&imp.body, 1e-6);
                // an arc under the checker's zero-length threshold (1e-6) is reported as
                // a degenerate edge by validation (flagged, not silently wrong)
                let only_degenerate = arc < 1e-6 && errs.iter().all(|i| i.code == "EDGE_DEGENERATE");
                assert!(errs.is_empty() || only_degenerate, "arc {arc}: invalid body: {errs:?}");
            }
        }
    }
}

/// solidcraft #8456 (second half): a SHELL_BASED_SURFACE_MODEL holding a closed
/// shell is a solid of its own; an open shell beside it must not spoil it.
#[test]
fn r8456_surface_model_closed_shell_is_a_solid() {
    let t = golden("box");
    let shell = t.lines().find(|l| l.contains("=CLOSED_SHELL(")).unwrap().split('=').next().unwrap().to_string();
    let t = t.lines().filter(|l| !l.contains("=MANIFOLD_SOLID_BREP(")).collect::<Vec<_>>().join("\n");
    let t = add_entities(&t, &[format!("#9000=SHELL_BASED_SURFACE_MODEL('',({shell}));")]);
    let t = t.replacen("ADVANCED_BREP_SHAPE_REPRESENTATION('box',(", "ADVANCED_BREP_SHAPE_REPRESENTATION('box',(#9000,", 1);
    match guarded(t).expect("no panic / hang") {
        Ok(imp) => assert!((vol(&imp.body) - 8000.0).abs() < 1e-6, "volume {}", vol(&imp.body)),
        Err(e) => panic!("closed shell in a surface model not imported: {e}"),
    }
}

// ---- solidcraft #8496 / #8601 / #8456: round faces off their frame's quarter lines

fn dir3(t: &str, id: u32) -> [f64; 3] {
    let l = t.lines().find(|l| l.starts_with(&format!("#{id}=DIRECTION("))).unwrap();
    let s = &l[l.find(",(").unwrap() + 2..l.rfind("))").unwrap()];
    let v: Vec<f64> = s.split(',').map(|x| x.trim().parse().unwrap()).collect();
    [v[0], v[1], v[2]]
}

fn refs(line: &str) -> Vec<u32> {
    let body = &line[line.find('(').unwrap()..];
    body.split('#').skip(1).map(|r| r.chars().take_while(char::is_ascii_digit).collect::<String>().parse().unwrap()).collect()
}

/// Turn the frame of every cylinder / cone / sphere / torus surface about its
/// own axis by `deg`: the same surface, with its angular origin (and so the
/// seam and quarter lines of any grid built on it) moved.
fn turn_frames(t: &str, deg: f64) -> String {
    let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
    let mut extra = vec![];
    let mut out = t.to_string();
    let mut next = 5000;
    for line in t.lines() {
        let is_round = ["CYLINDRICAL_SURFACE(", "CONICAL_SURFACE(", "SPHERICAL_SURFACE(", "TOROIDAL_SURFACE("].iter().any(|k| line.contains(&format!("={k}")));
        if !is_round {
            continue;
        }
        let id: u32 = line[1..line.find('=').unwrap()].parse().unwrap();
        let ax = refs(line)[0];
        let al = t.lines().find(|l| l.starts_with(&format!("#{ax}=AXIS2_PLACEMENT_3D("))).unwrap();
        let r = refs(al);
        let (o, z, x) = (r[0], dir3(t, r[1]), dir3(t, r[2]));
        let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
        let nx = [c * x[0] + s * y[0], c * x[1] + s * y[1], c * x[2] + s * y[2]];
        extra.push(format!("#{next}=DIRECTION('',({:.15},{:.15},{:.15}));", nx[0], nx[1], nx[2]));
        extra.push(format!("#{}=AXIS2_PLACEMENT_3D('',#{o},#{},#{next});", next + 1, r[1]));
        let new_line = line.replacen(&format!(",#{ax}"), &format!(",#{}", next + 1), 1);
        out = out.replace(line, &new_line);
        let _ = id;
        next += 2;
    }
    add_entities(&out, &extra)
}

#[test]
fn r8496_r8601_turned_round_frames_import_the_same_body() {
    let mut checked = 0;
    for name in ["cyl", "cone", "sphere", "torus", "rev360", "rev90", "revax360", "revax90"] {
        let t = golden(name);
        let base = guarded(t.clone()).expect("no panic / hang").unwrap_or_else(|e| panic!("{name}: {e}"));
        let v0 = vol(&base.body);
        for deg in [37.0, 75.0, 200.0, 313.0] {
            let tt = turn_frames(&t, deg);
            let imp = guarded(tt).expect("no panic / hang").unwrap_or_else(|e| panic!("{name} turned {deg}: {e}"));
            let v = vol(&imp.body);
            assert!((v - v0).abs() <= 1e-6 * (1.0 + v0.abs()), "{name} turned {deg}: volume {v} vs {v0}");
            assert_eq!(imp.body.num_faces(), base.body.num_faces(), "{name} turned {deg}: face count");
            assert!(ck_check::validate(&imp.body, 1e-6).is_empty(), "{name} turned {deg}: invalid body");
            checked += 1;
        }
    }
    assert_eq!(checked, 32);
}

// ---- fuzz: truncated and mutated STEP text --------------------------------

struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }
}

#[test]
fn fuzz_truncated_and_mutated_text_never_panics_or_hangs() {
    let seeds = ["box", "cyl", "torus", "rev90", "void"].map(golden);
    let junk = [".", "-.", "1.E999", "#", "'", "(", ")", ",", "$", "*", ";", "/*", "#0", "#99999", "1E", "--", ".E", "\u{0}", "é"];
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    let mut fails = vec![];
    let mut n = 0;
    for t in &seeds {
        // every truncation point on a coarse grid, plus the bytes just before the end
        let step = (t.len() / 120).max(1);
        let mut cuts: Vec<usize> = (0..t.len()).step_by(step).collect();
        cuts.extend(t.len().saturating_sub(40)..t.len());
        for cut in cuts {
            if t.is_char_boundary(cut) {
                n += 1;
                if let Err(e) = guarded(t[..cut].to_string()) {
                    fails.push(format!("cut {cut}: {e}"));
                }
            }
        }
        // random mutations: replace a span by junk, delete a span, duplicate a span
        for _ in 0..150 {
            let a = rng.below(t.len());
            let b = (a + 1 + rng.below(12)).min(t.len());
            if !t.is_char_boundary(a) || !t.is_char_boundary(b) {
                continue;
            }
            let m = match rng.below(3) {
                0 => format!("{}{}{}", &t[..a], junk[rng.below(junk.len())], &t[b..]),
                1 => format!("{}{}", &t[..a], &t[b..]),
                _ => format!("{}{}{}", &t[..b], &t[a..b], &t[b..]),
            };
            n += 1;
            if let Err(e) = guarded(m) {
                fails.push(format!("mutant at {a}..{b}: {e}"));
            }
        }
    }
    assert!(fails.is_empty(), "{} of {n} inputs failed:\n{}", fails.len(), fails.join("\n"));
}

/// An Ok result from a mutated file must still be a valid body (a mutation that
/// changes a coordinate may change the shape, but never to an invalid one).
#[test]
fn fuzz_accepted_mutants_are_valid_bodies() {
    let t = golden("cyl");
    let mut rng = Rng(0xD1B5_4A32_D192_ED03);
    let mut bad = vec![];
    for _ in 0..200 {
        let a = rng.below(t.len());
        if !t.is_char_boundary(a) || !t.is_char_boundary(a + 1) {
            continue;
        }
        let digit = b'0' + rng.below(10) as u8;
        if !t.as_bytes()[a].is_ascii_digit() {
            continue;
        }
        let mut b = t.clone().into_bytes();
        b[a] = digit;
        let m = String::from_utf8(b).unwrap();
        if let Ok(Ok(imp)) = guarded(m) {
            if flagged(&imp) {
                continue;
            }
            let errs = ck_check::validate(&imp.body, 1e-6);
            if !errs.is_empty() || !vol(&imp.body).is_finite() {
                bad.push(format!("digit mutation at {a}: {} validation errors", errs.len()));
            }
        }
    }
    assert!(bad.is_empty(), "{}", bad.join("\n"));
}
