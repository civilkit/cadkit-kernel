//! Families of profiles made of splines and lines (D shapes of several depths, circles with a flat, two arcs and two flats, sharp and
//! tangent) rounded and chamfered on the top, bottom and both rims, and with the vertical corner edges. Every case either builds a
//! valid solid whose removed volume matches the independent estimate (1 %), or is refused with a named error: never a broken solid.
//! `FAMILIES=1` prints the table.
mod spline_corner_support;
use spline_corner_support::*;

fn families() -> Vec<Shape> {
    let mut v = vec![];
    for (w, h) in [(40.0, 20.0), (60.0, 18.0), (60.0, 12.0), (80.0, 30.0), (30.0, 10.0)] {
        v.push(d_shape(&format!("D{w}x{h}"), w, h, 8));
    }
    v.push(d_shape("D-shallow", 60.0, 7.0, 8));
    for (r, cut) in [(25.0, 0.6), (25.0, 1.2), (15.0, 0.9), (40.0, 0.4)] {
        v.push(circle_flat(&format!("circle{r}-cut{cut}"), r, cut, 12));
    }
    for (w, hh, bulge) in [(50.0, 30.0, 6.0), (70.0, 25.0, 8.0), (40.0, 40.0, 5.0)] {
        v.push(two_flats(&format!("flats{w}x{hh}b{bulge}"), w, hh, bulge, 8));
    }
    v.push(d_tangent("D-tangent", 60.0, 18.0));
    v.push(d_tangent("D-tangent-wide", 90.0, 25.0));
    v.push(two_flats_tangent("flats-tangent", 50.0, 30.0, 6.0));
    v.push(two_flats_tangent("flats-tangent2", 70.0, 24.0, 5.0));
    v
}

#[test]
fn spline_and_line_families_build_or_refuse_by_name() {
    let show = std::env::var_os("FAMILIES").is_some();
    let (mut total, mut built) = (0, 0);
    let mut broken: Vec<String> = vec![];
    let mut unnamed: Vec<String> = vec![];
    let mut refusals: std::collections::BTreeMap<String, usize> = Default::default();
    for sh in families() {
        let tangent = sh.name.contains("tangent");
        // (label, rims, rim chamfer, size, vertical (size, chamfer), together)
        let mut cases: Vec<(String, Vec<&str>, bool, f64, Option<(f64, bool)>, bool)> = vec![];
        for (rl, rims) in [("top", vec!["F2.C1"]), ("bottom", vec!["F2.C0"]), ("both", vec!["F2.C1", "F2.C0"])] {
            for (chamfer, s) in [(false, 0.5), (false, 1.0), (false, 2.0), (false, 5.0), (true, 0.5), (true, 1.0), (true, 2.0), (true, 5.0)] {
                cases.push((format!("rim-{rl}-{}{s}", if chamfer { "c" } else { "r" }), rims.clone(), chamfer, s, None, false));
            }
        }
        if !tangent {
            for rv in [0.5, 1.0, 2.0, 5.0] {
                cases.push((format!("vert-r{rv}"), vec![], false, rv, Some((rv, false)), false));
                cases.push((format!("vert-c{rv}"), vec![], true, rv, Some((rv, true)), false));
            }
            for (rv, rr) in [(1.0, 1.0), (1.0, 0.5), (2.0, 1.0), (2.0, 2.0), (5.0, 2.0)] {
                for together in [false, true] {
                    cases.push((format!("vert-r{rv}+rim-r{rr}-{}", if together { "one" } else { "two" }), vec!["F2.C1"], false, rr, Some((rv, false)), together));
                }
            }
            for d in [0.5, 1.0, 2.0] {
                for together in [false, true] {
                    cases.push((format!("vert-c{d}+rim-c{d}-{}", if together { "one" } else { "two" }), vec!["F2.C1"], true, d, Some((d, true)), together));
                }
            }
            cases.push(("vert-c2+rim-r1-two".into(), vec!["F2.C1", "F2.C0"], false, 1.0, Some((2.0, true)), false));
            cases.push(("vert-r2+rim-c1-two".into(), vec!["F2.C1", "F2.C0"], true, 1.0, Some((2.0, false)), false));
        }
        for (label, rims, chamfer, s, vert, together) in cases {
            total += 1;
            let tag = format!("{}/{label}", sh.name);
            let o = run(&sh, &rims, chamfer, s, vert, together);
            let j = judge(&sh, &rims, chamfer, s, vert, together, &o);
            match &j {
                Ok(()) => built += 1,
                Err(e) if e.starts_with("refused") => {
                    if !o.errors.iter().all(|x| x.contains("blendUnsupported") || x.contains("blendFailed") || x.contains("blendOverflow")) {
                        unnamed.push(format!("{tag}: {:?}", o.errors));
                    }
                    let why: String = o.errors.first().map(|x| x.split("detail").nth(1).unwrap_or(x).chars().skip(8).take(70).collect()).unwrap_or_default();
                    *refusals.entry(why).or_default() += 1;
                }
                Err(e) => broken.push(format!("{tag}: {e}")),
            }
            if show {
                println!("{tag}: {}", match &j { Ok(()) => format!("built {:.2}s", o.secs), Err(e) => e.chars().take(200).collect() });
            }
            assert!(o.secs < 30.0, "{tag}: took {:.1}s", o.secs);
        }
    }
    println!("spline and line families: {total} cases, {built} built, {} refused by name", total - built - broken.len());
    for (why, n) in &refusals {
        println!("  {n:4} x {why}");
    }
    assert!(broken.is_empty(), "broken results: {broken:#?}");
    assert!(unnamed.is_empty(), "unnamed refusals: {unnamed:#?}");
    assert!(built * 10 >= total * 7, "only {built} of {total} build");
}
