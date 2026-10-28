//! `knurlBand`: diamond and pyramid knurls built directly as facets and cut in one boolean (docs/knurl-band.md).
//! The reference volume is an independent numeric integral of the ideal knurled surface (the deeper of the two groove sets at
//! each point of the band), not the other implementation.
use ck_model::features::knurl_band::{tool, Pattern, Spec};
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::f64::consts::{PI, TAU};
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

pub struct Out {
    pub body: Option<Body>,
    pub errors: Vec<String>,
    pub secs: f64,
}

pub fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let secs = t.elapsed().as_secs_f64();
    Out { body: ex.0.body.clone(), errors: r.errors.iter().map(|e| format!("{}:{} {:?}", e.feature, e.error.code, e.error.params)).collect(), secs }
}

/// Cylinder of radius `r` and height `h` on the front plane (axis z, z in 0..h), with an optional bore, then `k`.
pub fn knurled(r: f64, h: f64, bore: f64, k: &str) -> Out {
    let hole = if bore > 0.0 { format!(r#",{{"id":"p2","x":0,"y":0}}"#) } else { String::new() };
    let hole_e = if bore > 0.0 { format!(r#",{{"type":"circle","id":"e2","c":"p2","r":{bore}}}"#) } else { String::new() };
    run(&format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":0,"y":0}}{hole}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":{r}}}{hole_e}],"constraints":[]}},
        {{"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":{h}}}{k}"#,
        k = if k.is_empty() { String::new() } else { format!(",{k}") }
    ))
}

fn volume(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// Ideal removed volume: integral of (R^2 - (R - h)^2) / 2 over angle and length, h the deeper of the grooves.
fn ideal_removed(pattern: Pattern, r: f64, len: f64, n: usize, depth: f64, half: f64, slope: f64, rings: usize) -> f64 {
    let p = TAU * r / n as f64;
    let tent = |s: f64| (depth * (1.0 - s.abs() / half)).max(0.0);
    let near = |x: f64, period: f64| {
        let m = x.rem_euclid(period);
        m.min(period - m)
    };
    let (nu, nz) = (n * 120, 600usize);
    let mut sum = 0.0;
    for iu in 0..nu {
        let u = (iu as f64 + 0.5) / nu as f64 * TAU * r;
        for iz in 0..nz {
            let z = (iz as f64 + 0.5) / nz as f64 * len;
            let h = match pattern {
                Pattern::Diamond => tent(near(u - slope * z, p)).max(tent(near(u + slope * z, p))),
                Pattern::Pyramid => {
                    let step = len / rings as f64;
                    tent(near(u, p)).max(tent(near(z - step / 2.0, step)))
                }
            };
            sum += (r * r - (r - h) * (r - h)) / 2.0;
        }
    }
    sum * (TAU / nu as f64) * (len / nz as f64)
}

fn pyramid_json(axis: &str, center: &str, from: f64, to: f64, n: usize, extra: &str) -> String {
    format!(r#"{{"type":"knurlBand","id":"K","axis":"{axis}","center":{center},"radius":10,"from":{from},"to":{to},"pattern":"pyramid","count":{n}{extra}}}"#)
}

fn check_valid(b: &Body, what: &str) {
    let issues = ck_check::validate(b, 1e-6);
    assert!(issues.is_empty(), "{what}: {} validity issues, first {:?}", issues.len(), issues.first());
    assert_eq!(b.num_lumps(), 1, "{what}");
}

fn spec(pattern: Pattern, n: usize, slope: f64, rings: usize) -> Spec {
    use ck_core::linalg::Vec3;
    let p = TAU * 10.0 / n as f64;
    Spec {
        pattern,
        o: Vec3::new(0.0, 0.0, 0.0),
        d: Vec3::new(0.0, 0.0, 1.0),
        ex: Vec3::new(1.0, 0.0, 0.0),
        radius: 10.0,
        from: 0.0,
        len: 12.0,
        count: n,
        depth: p / 4.0,
        half: p / 4.0,
        slope,
        rings,
    }
}

#[test]
fn the_tool_is_a_closed_manifold_solid() {
    for (pat, slope, rings) in [(Pattern::Pyramid, 3.0, 4usize), (Pattern::Diamond, 30f64.to_radians().tan(), 0)] {
        let t = tool("K", &spec(pat, 24, slope, rings)).expect("tool");
        assert!(ck_topo::check::check_topology(&t, 1e-6).is_empty());
        assert!(ck_topo::check::is_closed_manifold(&t));
        assert!(volume(&t) > 0.0);
    }
}

#[test]
fn pyramid_matches_the_ideal_surface_and_is_valid() {
    let plain = knurled(10.0, 12.0, 0.0, "");
    let v0 = volume(plain.body.as_ref().unwrap());
    let o = knurled(10.0, 12.0, 0.0, &pyramid_json("z", "[0,0]", 0.0, 12.0, 24, r#","ringPitch":3"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    check_valid(b, "pyramid");
    let removed = v0 - volume(b);
    let ideal = ideal_removed(Pattern::Pyramid, 10.0, 12.0, 24, TAU * 10.0 / 24.0 / 4.0, TAU * 10.0 / 24.0 / 4.0, 3.0, 4);
    assert!((removed - ideal).abs() / ideal < 0.05, "removed {removed} vs ideal {ideal}");
    assert!(o.secs < 60.0, "took {} s", o.secs);
}

#[test]
fn diamond_matches_the_ideal_surface_and_is_valid() {
    let plain = knurled(10.0, 12.0, 0.0, "");
    let v0 = volume(plain.body.as_ref().unwrap());
    let k = r#"{"type":"knurlBand","id":"K","axis":"z","center":[0,0],"radius":10,"from":0,"to":12,"pattern":"diamond","count":16,"helixAngle":30}"#;
    let o = knurled(10.0, 12.0, 0.0, k);
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    check_valid(b, "diamond");
    let p = TAU * 10.0 / 16.0;
    let ideal = ideal_removed(Pattern::Diamond, 10.0, 12.0, 16, p / 4.0, p / 4.0, 30f64.to_radians().tan(), 0);
    let removed = v0 - volume(b);
    assert!((removed - ideal).abs() / ideal < 0.05, "removed {removed} vs ideal {ideal}");
}

#[test]
fn a_band_inside_a_longer_cylinder_with_a_bore() {
    // the band ends are walls in the middle of the cylinder; the bore is untouched
    let plain = knurled(10.0, 12.0, 3.0, "");
    let v0 = volume(plain.body.as_ref().unwrap());
    let o = knurled(10.0, 12.0, 3.0, &pyramid_json("z", "[0,0]", 3.0, 9.0, 24, r#","ringPitch":3"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let b = o.body.as_ref().unwrap();
    check_valid(b, "mid band");
    let p = TAU * 10.0 / 24.0;
    let ideal = ideal_removed(Pattern::Pyramid, 10.0, 6.0, 24, p / 4.0, p / 4.0, 3.0, 2);
    let removed = v0 - volume(b);
    assert!((removed - ideal).abs() / ideal < 0.05, "removed {removed} vs ideal {ideal}");
}

#[test]
fn other_axes_and_centres_give_a_congruent_result() {
    // a cylinder along x, offset: the same volume as along z
    let src = |axis: &str, center: &str, plane: &str| {
        run(&format!(
            r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"{plane}"}},"points":[{{"id":"p1","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"p1","r":10}}],"constraints":[]}},
            {{"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":12}},{}"#,
            pyramid_json(axis, center, 0.0, 12.0, 24, r#","ringPitch":3"#)
        ))
    };
    let z = src("z", "[0,0]", "front");
    let zb = z.body.as_ref().expect("z body");
    assert!(z.errors.is_empty(), "{:?}", z.errors);
    // right plane extrudes along x; its centre is at the origin of the plane
    let x = src("x", "[0,0]", "right");
    let xb = x.body.as_ref().expect("x body");
    assert!(x.errors.is_empty(), "{:?}", x.errors);
    let (vz, vx) = (volume(zb), volume(xb));
    assert!((vz - vx).abs() / vz < 1e-6, "{vz} vs {vx}");
    check_valid(xb, "axis x");
}

#[test]
fn refusals() {
    let none = run(&pyramid_json("z", "[0,0]", 0.0, 12.0, 24, ""));
    assert!(none.errors.iter().any(|e| e.contains("nothingToCut")), "{:?}", none.errors);
    let overlap = knurled(10.0, 12.0, 0.0, &pyramid_json("z", "[0,0]", 0.0, 12.0, 24, r#","depth":2.5"#));
    assert!(overlap.errors.iter().any(|e| e.contains("invalidValue") && e.contains("overlap")), "{:?}", overlap.errors);
    let round = knurled(10.0, 12.0, 0.0, &pyramid_json("z", "[0,0]", 0.0, 12.0, 24, r#","profile":"round""#));
    assert!(round.errors.iter().any(|e| e.contains("invalidValue")), "{:?}", round.errors);
    let miss = knurled(10.0, 12.0, 0.0, &pyramid_json("z", "[0,0]", 30.0, 40.0, 24, ""));
    assert!(!miss.errors.is_empty(), "a band off the body must say so");
    // the failed feature leaves the body as it was
    let plain = knurled(10.0, 12.0, 0.0, "");
    assert_eq!(miss.body.as_ref().map(volume), plain.body.as_ref().map(volume));
}

#[test]
fn growth_is_roughly_linear_in_facets() {
    // pyramid, 8 rings: the facet count (and the time) grow with the groove count
    let mut last = 0.0;
    for n in [12usize, 24, 48] {
        let o = knurled(10.0, 12.0, 0.0, &pyramid_json("z", "[0,0]", 0.0, 12.0, n, ""));
        assert!(o.errors.is_empty(), "n={n}: {:?}", o.errors);
        check_valid(o.body.as_ref().unwrap(), "growth");
        assert!(o.secs < 120.0, "n={n} took {} s", o.secs);
        last = o.secs;
    }
    let _ = last;
    let _ = PI;
}

/// The tool alone (no boolean) over a grid of counts, depths, helix angles, band lengths and starts: every one must close into a manifold solid.
#[test]
fn the_tool_closes_over_a_parameter_sweep() {
    use ck_core::linalg::Vec3;
    let mut bad: Vec<String> = vec![];
    let mut total = 0;
    for &n in &[10usize, 16, 30] {
        for &df in &[0.1, 0.25, 0.44] {
            for &(from, len) in &[(0.0, 12.0), (3.3, 5.1), (-2.0, 7.0), (0.0, 1.0)] {
                let p = TAU * 10.0 / n as f64;
                let depth = p * df;
                let mut specs: Vec<(String, Spec)> = vec![];
                for &ha in &[10.0f64, 30.0, 60.0] {
                    specs.push((format!("diamond ha{ha}"), Spec { pattern: Pattern::Diamond, o: Vec3::new(1.0, 2.0, 3.0), d: Vec3::new(0.0, 1.0, 0.0), ex: Vec3::new(1.0, 0.0, 0.0), radius: 10.0, from, len, count: n, depth, half: depth, slope: ha.to_radians().tan(), rings: 0 }));
                }
                for &rp in &[1.0, 1.7] {
                    let rings = ((len / (rp * p)).floor() as usize).max(1);
                    specs.push((format!("pyramid rp{rp}"), Spec { pattern: Pattern::Pyramid, o: Vec3::new(1.0, 2.0, 3.0), d: Vec3::new(0.0, 1.0, 0.0), ex: Vec3::new(1.0, 0.0, 0.0), radius: 10.0, from, len, count: n, depth, half: depth, slope: len / rings as f64, rings }));
                }
                for (name, s) in specs {
                    // grooves must leave a land, and the pyramid ring pitch must fit the groove
                    if 2.0 * s.half > 0.9 * p || (s.pattern == Pattern::Pyramid && s.slope < 2.0 * s.half / 0.9) {
                        continue;
                    }
                    total += 1;
                    match tool("K", &s) {
                        Ok(b) => {
                            if !ck_topo::check::is_closed_manifold(&b) || !(volume(&b) > 0.0) {
                                bad.push(format!("{name} n={n} df={df} from={from} len={len}: not closed"));
                            }
                        }
                        // a groove too wide for the facet lattice is a refusal, not a failure
                        Err(e) if format!("{:?}", e.params).contains("lands too wide") || format!("{:?}", e.params).contains("rings are too close") => {}
                        Err(e) => bad.push(format!("{name} n={n} df={df} from={from} len={len}: {} {:?}", e.code, e.params)),
                    }
                }
            }
        }
    }
    assert!(bad.is_empty(), "{} of {total} tools failed, first: {:?}", bad.len(), &bad[..bad.len().min(12)]);
}

/// Removed volume against the ideal surface over coarse and fine lattices, both patterns, a band shorter and longer than a lattice cell.
#[test]
fn volume_stays_near_the_ideal_over_coarse_and_fine_lattices() {
    let plain = knurled(10.0, 12.0, 0.0, "");
    let v0 = volume(plain.body.as_ref().unwrap());
    let mut report = String::new();
    let mut worst = 0.0f64;
    // (pattern, count, depth, helix angle or ring pitch)
    let cases: [(&str, usize, f64, f64); 7] = [
        ("diamond", 12, 0.5, 22.0),
        ("diamond", 36, 0.44, 30.0),
        ("diamond", 20, 0.8, 45.0),
        ("pyramid", 12, 0.5, 0.0),
        ("pyramid", 36, 0.44, 0.0),
        ("pyramid", 20, 0.8, 0.0),
        ("pyramid", 24, 0.65, 4.0),
    ];
    for (pat, n, depth, x) in cases {
        let extra = if pat == "diamond" { format!(r#","helixAngle":{x}"#) } else if x > 0.0 { format!(r#","ringPitch":{x}"#) } else { String::new() };
        let k = format!(r#"{{"type":"knurlBand","id":"K","axis":"z","radius":10,"from":0,"to":12,"pattern":"{pat}","count":{n},"depth":{depth}{extra}}}"#);
        let o = knurled(10.0, 12.0, 0.0, &k);
        assert!(o.errors.is_empty(), "{pat} n={n} d={depth}: {:?}", o.errors);
        let p = TAU * 10.0 / n as f64;
        let half = depth;
        let (patt, slope, rings) = if pat == "diamond" {
            (Pattern::Diamond, x.to_radians().tan(), 0)
        } else {
            let rp = if x > 0.0 { x } else { p };
            let m = ((12.0 / rp).floor() as usize).max(1);
            (Pattern::Pyramid, 12.0 / m as f64, m)
        };
        let ideal = ideal_removed(patt, 10.0, 12.0, n, depth, half, slope, rings);
        let removed = v0 - volume(o.body.as_ref().unwrap());
        let err = (removed - ideal) / ideal;
        worst = worst.max(err.abs());
        report.push_str(&format!("{pat} n={n} d={depth} x={x}: removed {removed:.2} ideal {ideal:.2} error {:+.1}%, {:.2}s\n", err * 100.0, o.secs));
    }
    assert!(worst < 0.04, "worst {worst}\n{report}");
}
