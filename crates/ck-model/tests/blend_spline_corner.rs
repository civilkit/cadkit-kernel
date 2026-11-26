//! Rounds and chamfers on a profile of splines and straight lines (a D, a circle with a flat, two arcs and two flats), extruded blind
//! 20 mm: the rim loop with its sharp (mitred) or tangent joins, the vertical corner edges where a spline meets a line, and both
//! together (the three-way corner; a sphere where the sizes are equal). Built in place (`blend/spline_rim.rs`,
//! `blend/spline_vertical.rs`); every result is valid and removes the volume of an independent estimate (the rim's section times its
//! length less the turn, `spline_corner_support`), or is refused by name.
//! `SPLINE_CORNER_PROBE=1` prints a table of every variant instead (`SPLINE_CORNER_SHAPE=<name>` for one shape).
mod spline_corner_support;
use spline_corner_support::*;

fn sharp_shapes() -> Vec<Shape> {
    vec![
        d_shape("D-sharp", 60.0, 18.0, 8),
        d_shape("D-shallow", 60.0, 7.0, 8),
        circle_flat("circle-flat", 25.0, 0.6, 12),
        two_flats("two-flats", 50.0, 30.0, 6.0, 8),
    ]
}

fn tangent_shapes() -> Vec<Shape> {
    vec![d_tangent("D-tangent", 60.0, 18.0), two_flats_tangent("two-flats-tangent", 50.0, 30.0, 6.0)]
}

fn shapes() -> Vec<Shape> {
    let mut v = sharp_shapes();
    v.extend(tangent_shapes());
    v
}

fn verdict(o: &Outcome, j: &Result<(), String>) -> String {
    match j {
        Ok(()) => "ok".to_string(),
        Err(e) => format!("{} ({:.2}s)", e.chars().take(300).collect::<String>(), o.secs),
    }
}

/// A refusal names the limit (a blend error code), never an unnamed failure.
fn named(o: &Outcome) -> bool {
    !o.errors.is_empty() && o.errors.iter().all(|e| e.contains("blendUnsupported") || e.contains("blendFailed") || e.contains("blendOverflow"))
}

/// Builds and matches the estimate: panics with the reason otherwise.
fn must_build(sh: &Shape, rims: &[&str], chamfer: bool, s: f64, vert: Option<(f64, bool)>, together: bool, what: &str) {
    let o = run(sh, rims, chamfer, s, vert, together);
    let j = judge(sh, rims, chamfer, s, vert, together, &o);
    assert!(j.is_ok(), "{} {what}: {}", sh.name, verdict(&o, &j));
}

#[test]
fn probe() {
    if std::env::var_os("SPLINE_CORNER_PROBE").is_none() {
        return;
    }
    let only = std::env::var("SPLINE_CORNER_SHAPE").ok();
    for sh in shapes() {
        if only.as_deref().is_some_and(|o| o != sh.name) {
            continue;
        }
        for (rl, rims) in [("top", vec!["F2.C1"]), ("bottom", vec!["F2.C0"]), ("both", vec!["F2.C1", "F2.C0"])] {
            for chamfer in [false, true] {
                for s in [0.5, 1.0, 2.0, 5.0] {
                    let o = run(&sh, &rims, chamfer, s, None, false);
                    let j = judge(&sh, &rims, chamfer, s, None, false, &o);
                    println!("PROBE {}/rim-{rl}/{}{s}: {}", sh.name, if chamfer { "c" } else { "r" }, verdict(&o, &j));
                }
            }
        }
        for (ch, rv) in [(false, 1.0), (false, 2.0), (true, 1.0), (true, 2.0)] {
            for (rr, rim_ch) in [(0.5, false), (1.0, false), (0.5, true), (1.0, true)] {
                for together in [false, true] {
                    if together && ch != rim_ch {
                        continue;
                    }
                    let rr = if together && ch { rv } else { rr };
                    let v = Some((rv, ch));
                    let o = run(&sh, &["F2.C1"], rim_ch, rr, v, together);
                    let j = judge(&sh, &["F2.C1"], rim_ch, rr, v, together, &o);
                    println!("PROBE {}/vert-{}{rv}+rim-{}{rr}/{}: {}", sh.name, if ch { "c" } else { "r" }, if rim_ch { "c" } else { "r" }, if together { "together" } else { "staged" }, verdict(&o, &j));
                }
            }
        }
        for rv in [0.5, 1.0, 2.0, 5.0] {
            for ch in [false, true] {
                let o = run(&sh, &[], ch, rv, Some((rv, ch)), false);
                let j = judge(&sh, &[], ch, rv, Some((rv, ch)), false, &o);
                println!("PROBE {}/vert-{}{rv}: {}", sh.name, if ch { "c" } else { "r" }, verdict(&o, &j));
            }
        }
    }
}

/// Goal 1: a spline run and a line run meeting at sharp corners; round and chamfer the whole top, bottom and both rims.
#[test]
fn sharp_corner_rims_build() {
    for sh in sharp_shapes().iter().filter(|s| s.name != "D-shallow") {
        for (rl, rims) in [("top", vec!["F2.C1"]), ("bottom", vec!["F2.C0"]), ("both", vec!["F2.C1", "F2.C0"])] {
            for chamfer in [false, true] {
                for s in [0.5, 1.0, 2.0, 5.0] {
                    must_build(sh, &rims, chamfer, s, None, false, &format!("{rl} {}{s}", if chamfer { "chamfer" } else { "round" }));
                }
            }
        }
    }
}

/// An acute corner (a shallow circular segment, 26 degrees inside) builds up to the size its spline end allows, then is refused by
/// name (the offset of the end of the arc turns inside out).
#[test]
fn acute_corner_builds_then_refuses_by_name() {
    let sh = d_shape("D-shallow", 60.0, 7.0, 8);
    for rims in [vec!["F2.C1"], vec!["F2.C0"], vec!["F2.C1", "F2.C0"]] {
        for chamfer in [false, true] {
            for s in [0.5, 1.0, 2.0] {
                must_build(&sh, &rims, chamfer, s, None, false, &format!("{rims:?} {chamfer} {s}"));
            }
            let o = run(&sh, &rims, chamfer, 5.0, None, false);
            assert!(named(&o), "D-shallow 5: expected a named refusal, got {:?} / body {}", o.errors, o.body.is_some());
        }
    }
}

/// Goal 3: a spline that leaves the line tangentially: the rim is smooth all round.
#[test]
fn tangent_joins_build_like_the_smooth_ring() {
    for sh in tangent_shapes() {
        for (rl, rims) in [("top", vec!["F2.C1"]), ("bottom", vec!["F2.C0"]), ("both", vec!["F2.C1", "F2.C0"])] {
            for chamfer in [false, true] {
                for s in [0.5, 1.0, 2.0] {
                    must_build(&sh, &rims, chamfer, s, None, false, &format!("{rl} {}{s}", if chamfer { "chamfer" } else { "round" }));
                }
            }
        }
        // a bend tighter than the size: refused by name
        let o = run(&sh, &["F2.C1"], false, 5.0, None, false);
        if !o.errors.is_empty() {
            assert!(named(&o), "{}: {:?}", sh.name, o.errors);
        } else {
            assert!(judge(&sh, &["F2.C1"], false, 5.0, None, false, &o).is_ok(), "{} r5", sh.name);
        }
    }
}

/// Goal 2: the vertical corner edges alone, rounded and chamfered.
#[test]
fn vertical_edges_alone() {
    for sh in sharp_shapes() {
        for rv in [0.5, 1.0, 2.0] {
            must_build(&sh, &[], false, rv, Some((rv, false)), false, &format!("vertical round {rv}"));
            must_build(&sh, &[], true, rv, Some((rv, true)), false, &format!("vertical chamfer {rv}"));
        }
        if sh.name != "D-shallow" {
            must_build(&sh, &[], false, 5.0, Some((5.0, false)), false, "vertical round 5");
        }
    }
}

/// Goal 2: the vertical edges and the rim, in two features (vertical first) and in one: equal radii (a sphere at each corner) and a
/// larger vertical radius (a torus), rounds and chamfers.
#[test]
fn vertical_edges_and_rim() {
    for sh in sharp_shapes().iter().filter(|s| s.name != "D-shallow") {
        for together in [false, true] {
            for (rv, rr) in [(1.0, 1.0), (1.0, 0.5), (2.0, 0.5), (2.0, 1.0), (2.0, 2.0)] {
                must_build(sh, &["F2.C1"], false, rr, Some((rv, false)), together, &format!("vertical round {rv} + rim round {rr} together={together}"));
            }
            // chamfer: one distance in one feature
            for d in [0.5, 1.0, 2.0] {
                must_build(sh, &["F2.C1"], true, d, Some((d, true)), together, &format!("chamfers {d} together={together}"));
            }
        }
        // staged variants with other kinds and sizes
        for (rv, vch, rr, rch) in [(1.0, true, 0.5, false), (2.0, true, 1.0, false), (1.0, false, 0.5, true), (2.0, false, 1.0, true), (2.0, true, 1.0, true)] {
            must_build(sh, &["F2.C1", "F2.C0"], rch, rr, Some((rv, vch)), false, &format!("vertical {vch} {rv} then both rims {rch} {rr}"));
        }
    }
}

/// What does not build is refused by name: a vertical radius smaller than the rim's, the rim rounded before the vertical edge.
#[test]
fn limits_are_named() {
    let sh = d_shape("D-sharp", 60.0, 18.0, 8);
    for together in [false, true] {
        let o = run(&sh, &["F2.C1"], false, 2.0, Some((1.0, false)), together);
        assert!(named(&o), "rv 1 < rr 2 together={together}: {:?}", o.errors);
    }
    let o = run_rim_first(&sh, "F2.C1", 1.0, 1.0);
    assert!(named(&o), "rim first: {:?} body {}", o.errors, o.body.is_some());
}
