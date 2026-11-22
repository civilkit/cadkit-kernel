//! Shell of a dog-bone plate carrying a cross boss whose cylinder is tangent to the opened top
//! face (the boss touches the top plane along a line), and a box united with a wider cylinder
//! whose cavity corners slide along the offset circles.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl regen::FeatureExec for Exec {
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

struct Out {
    body: Option<ck_topo::body::Body>,
    errors: Vec<String>,
}

fn run(features: &str) -> Out {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| format!("{} {:?}", e.error.code, e.error.params)).collect();
    Out { body: ex.0.body, errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.body.as_ref().unwrap()).volume
}

/// Dog-bone: two circles (d 40) 60 apart joined by two R40 arcs tangent to both, 30 thick
/// (z 0..30, top face F2.C1); then a cross cylinder (d 26, axis along y) through the neck,
/// centre `zc` above the base, 40 long mid-plane.
fn part(zc: f64, shell: &str) -> String {
    let s3 = 3f64.sqrt();
    let (c, ty) = (30.0 * s3, 10.0 * s3);
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},
  "points":[{{"id":"cr","x":30,"y":0}},{{"id":"cl","x":-30,"y":0}},{{"id":"ct","x":0,"y":{c}}},{{"id":"cb","x":0,"y":{nc}}},
   {{"id":"rb","x":20,"y":{nty}}},{{"id":"rt","x":20,"y":{ty}}},{{"id":"lt","x":-20,"y":{ty}}},{{"id":"lb","x":-20,"y":{nty}}}],
  "entities":[{{"type":"arc","id":"a1","c":"cr","p1":"rb","p2":"rt"}},{{"type":"arc","id":"a2","c":"ct","p1":"lt","p2":"rt"}},
   {{"type":"arc","id":"a3","c":"cl","p1":"lt","p2":"lb"}},{{"type":"arc","id":"a4","c":"cb","p1":"rb","p2":"lb"}}],"constraints":[]}},
 {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}},
 {{"type":"sketch","id":"F3","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":{nzc}}}],
  "entities":[{{"type":"circle","id":"e1","c":"c","r":13}}],"constraints":[]}},
 {{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"mid","depth":40}}{shell}"#,
        nzc = -zc, // the "top" plane's sketch y runs along -z
        nc = -c,
        nty = -ty
    )
}

fn shell(face: &str) -> String {
    format!(r#",{{"type":"shell","id":"F5","faces":["{face}"],"thickness":3,"outward":false}}"#)
}

/// Volume of the dog-bone with the cross boss, both offset inward by `d` (d = 0: the part;
/// d = 3: the cavity of a shell opened at the top, whose top stays at z = 30). Numeric
/// integration over the exact outline: section area times height, plus the boss beyond the
/// neck walls. `zc` NaN returns the outline area alone.
fn analytic(zc: f64, d: f64) -> f64 {
    let c = 30.0 * 3f64.sqrt();
    let (r, big, rb, half) = (20.0 - d, 40.0 + d, 13.0 - d, 20.0 - d);
    let xt = 30.0 * big / (big + r); // where the neck arc meets the end circles
    let half_width = |x: f64| -> f64 {
        let mut hw: f64 = 0.0;
        for cx in [30.0, -30.0] {
            if (x - cx).abs() < r {
                hw = hw.max((r * r - (x - cx) * (x - cx)).sqrt());
            }
        }
        if x.abs() <= xt {
            hw = hw.max(c - (big * big - x * x).sqrt());
        }
        hw
    };
    let n = 20000;
    let (x0, x1) = (-30.0 - r, 30.0 + r);
    let area: f64 = (0..n).map(|i| 2.0 * half_width(x0 + (x1 - x0) * (i as f64 + 0.5) / n as f64)).sum::<f64>() * (x1 - x0) / n as f64;
    if zc.is_nan() {
        return area;
    }
    let m = 800;
    let h = 2.0 * rb / m as f64;
    let mut boss = 0.0;
    for i in 0..m {
        let x = -rb + h * (i as f64 + 0.5);
        for j in 0..m {
            let z = -rb + h * (j as f64 + 0.5);
            if x * x + z * z < rb * rb && zc + z <= 30.0 + 1e-9 {
                boss += (2.0 * half - 2.0 * half_width(x)).max(0.0) * h * h;
            }
        }
    }
    (30.0 - d) * area + boss
}

/// Shell the top open, wall 3: no errors, a body no less valid than the solid, and the volume
/// of the part minus its inward offset to 1%. Returns the volume.
fn check_top_open(zc: f64) -> f64 {
    let solid = run(&part(zc, ""));
    assert!(solid.errors.is_empty(), "zc={zc}: {:?}", solid.errors);
    let t = std::time::Instant::now();
    let r = run(&part(zc, &shell("F2.C1")));
    eprintln!("zc={zc}: shell regen {:.2}s", t.elapsed().as_secs_f64());
    assert!(r.errors.is_empty(), "zc={zc}: {:?}", r.errors);
    let b = r.body.as_ref().unwrap();
    let base = ck_check::validate(solid.body.as_ref().unwrap(), 1e-6);
    let iss = ck_check::validate(b, 1e-6);
    assert!(
        iss.len() <= base.len() && iss.iter().all(|i| i.code == "EDGE_DEGENERATE"),
        "zc={zc}: solid has {} issues, shell {:?}",
        base.len(),
        iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>()
    );
    assert!(ck_topo::check::check(b, 1e-6).is_empty(), "zc={zc}: topology {:?}", ck_topo::check::check(b, 1e-6).iter().take(4).collect::<Vec<_>>());
    let v = vol(&r);
    let want = analytic(zc, 0.0) - analytic(zc, 3.0);
    assert!((v - want).abs() < 0.01 * want, "zc={zc}: shelled volume {v}, analytic {want}");
    let outer = vol(&solid);
    assert!((outer - analytic(zc, 0.0)).abs() < 0.005 * outer, "zc={zc}: solid {outer} vs {}", analytic(zc, 0.0));
    v
}

#[test]
fn outline_area() {
    // two d40 circles 60 apart joined by tangent R40 arcs: 3117.69 mm^2
    let a = analytic(f64::NAN, 0.0);
    assert!((a - 3117.69).abs() < 0.05, "{a}");
    let solid = run(&part(15.0, ""));
    let b = solid.body.as_ref().unwrap();
    let top = b.find_face("F2.C1").unwrap();
    let area = ck_topo::mass::face_integral(b, top, 4, &|_, n| [n.len()])[0];
    assert!((area - 3117.69).abs() < 0.05, "top face area {area}");
}

/// Boss centred at z=15: not tangent to the top face.
#[test]
fn shell_top_with_untangent_boss() {
    check_top_open(15.0);
}

/// Boss centred at z=17: its top line (z=30) lies in the opened top face (the CSWP 18.3 part).
/// Expected 32047.71 mm^3 (part 101498.21 minus cavity 69450.50).
#[test]
fn shell_top_with_tangent_boss() {
    let v = check_top_open(17.0);
    assert!((v - 32047.71).abs() < 0.01 * 32047.71, "{v}");
}

/// The tangent boss is fine as long as the tangent face is not the opened one.
#[test]
fn shell_bottom_with_tangent_boss() {
    let r = run(&part(17.0, &shell("F2.C0")));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(ck_check::validate(r.body.as_ref().unwrap(), 1e-6).iter().all(|i| i.code == "EDGE_DEGENERATE"));
}

/// A box (40 x 20 x 10) united with a cylinder (r 15, centred in it, poking out of both long
/// sides), shelled 2 open at the top: the cavity's cylinder/wall corners slide along the offset
/// circles, and the result is exact.
#[test]
fn box_with_wide_cylinder_shells_exactly() {
    let base = r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":40,"y":0},{"id":"p3","x":40,"y":20},{"id":"p4","x":0,"y":20}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10},
{"type":"sketch","id":"F3","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":20,"y":10}],"entities":[{"type":"circle","id":"e1","c":"c","r":15}],"constraints":[]},
{"type":"extrude","id":"F4","op":"add","sketch":"F3","end":"blind","depth":10}"#;
    let solid = run(base);
    assert!(solid.errors.is_empty(), "{:?}", solid.errors);
    let seg = |r: f64, d: f64| r * r * (d / r).acos() - d * (r * r - d * d).sqrt();
    let outer = (800.0 + 2.0 * seg(15.0, 10.0)) * 10.0;
    assert!((vol(&solid) - outer).abs() < 1e-6 * outer, "solid {} vs {outer}", vol(&solid));
    let r = run(&format!(r#"{base},{{"type":"shell","id":"F5","faces":["F2.C1"],"thickness":2,"outward":false}}"#));
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    let want = outer - (576.0 + 2.0 * seg(13.0, 8.0)) * 8.0;
    let v = vol(&r);
    assert!((v - want).abs() < 1e-6 * want, "{v} vs {want}");
}

/// The part with the top rim (the four edges of the top face, one tangent chain) rounded with radius `r` before the boss
/// is added, as in the CSWP 18.3 tutorial.
fn part_rounded(zc: f64, r: f64, shell: &str) -> String {
    let s3 = 3f64.sqrt();
    let ny = 40.0 - 30.0 * s3; // the neck arcs' nearest point to the axis: y = -11.96 / +11.96
    let fillet = format!(
        r#",{{"type":"fillet","id":"F6","radius":{r},"edges":[{{"faces":["F2.C1","F2.S.a1"],"at":[50,0,30]}},{{"faces":["F2.C1","F2.S.a2"],"at":[0,{py},30]}},{{"faces":["F2.C1","F2.S.a3"],"at":[-50,0,30]}},{{"faces":["F2.C1","F2.S.a4"],"at":[0,{ny},30]}}]}}"#,
        py = -ny
    );
    let base = part(zc, shell);
    let at = base.find(r#"{"type":"sketch","id":"F3""#).unwrap();
    let (a, b) = base.split_at(at);
    let a = a.trim_end().trim_end_matches(',');
    format!("{a}{fillet},\n {b}")
}

/// Volume the rim round of radius `r` removes from the part with the boss at `zc`: the corner between the round and
/// the square edge, swept along the outline (end arcs R20 convex over 240 degrees each, neck arcs R40 concave over 60
/// degrees each), less what the boss (added after the round) fills again at the necks. Numeric integration.
fn rim_removed(zc: f64, r: f64) -> f64 {
    let n = 120;
    let h = r / n as f64;
    let m = 720;
    let span = std::f64::consts::PI / 3.0;
    let mut v = 0.0;
    for i in 0..n {
        let s = h * (i as f64 + 0.5); // inward from the wall
        for j in 0..n {
            let d = h * (j as f64 + 0.5); // down from the top
            if (s - r).powi(2) + (d - r).powi(2) <= r * r {
                continue;
            }
            let da = h * h;
            v += 2.0 * (4.0 * std::f64::consts::PI / 3.0) * (20.0 - s) * da;
            // the necks: only what the boss does not cover
            let rad = 40.0 + s;
            let mut open = 0.0;
            for k in 0..m {
                let phi = -0.5 * span + span * (k as f64 + 0.5) / m as f64;
                let x = rad * phi.sin();
                let z = 30.0 - d;
                if x * x + (z - zc).powi(2) >= 169.0 {
                    open += span / m as f64;
                }
            }
            v += 2.0 * open * rad * da;
        }
    }
    v
}

/// Round the top rim with `r`, add the boss at `zc`, shell 3 open at the top. A rim round no larger than the wall leaves
/// the cavity of the unrounded part (its inward offset has a sharp corner there), so the wall is that of the unrounded
/// part less the rim corner the round removed.
fn check_rounded_top_open(zc: f64, r: f64) -> f64 {
    let t = std::time::Instant::now();
    let solid = run(&part_rounded(zc, r, ""));
    eprintln!("zc={zc} r={r}: solid regen {:.2}s", t.elapsed().as_secs_f64());
    assert!(solid.errors.is_empty(), "zc={zc} r={r}: {:?}", solid.errors);
    let outer = vol(&solid);
    let want_outer = analytic(zc, 0.0) - rim_removed(zc, r);
    assert!((outer - want_outer).abs() < 0.002 * outer, "zc={zc} r={r}: rounded solid {outer} vs {want_outer}");
    let t = std::time::Instant::now();
    let res = run(&part_rounded(zc, r, &shell("F2.C1")));
    eprintln!("zc={zc} r={r}: shell regen {:.2}s", t.elapsed().as_secs_f64());
    assert!(res.errors.is_empty(), "zc={zc} r={r}: {:?}", res.errors);
    let b = res.body.as_ref().unwrap();
    let base = ck_check::validate(solid.body.as_ref().unwrap(), 1e-6);
    let iss = ck_check::validate(b, 1e-6);
    assert!(
        iss.len() <= base.len() && iss.iter().all(|i| i.code == "EDGE_DEGENERATE"),
        "zc={zc} r={r}: solid has {} issues, shell {:?}",
        base.len(),
        iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>()
    );
    assert!(ck_topo::check::check(b, 1e-6).is_empty(), "zc={zc} r={r}: topology {:?}", ck_topo::check::check(b, 1e-6).iter().take(4).collect::<Vec<_>>());
    let v = vol(&res);
    let want = want_outer - analytic(zc, 3.0);
    assert!((v - want).abs() < 0.01 * want, "zc={zc} r={r}: shelled volume {v}, expected {want}");
    v
}

/// CSWP 18.3: rim round R3 (equal to the wall), boss tangent to the top, shell 3 open at the top.
#[test]
fn shell_rounded_rim_with_tangent_boss() {
    check_rounded_top_open(17.0, 3.0);
}

/// The same with the default R1.5 round (smaller than the wall).
#[test]
fn shell_small_rounded_rim_with_tangent_boss() {
    check_rounded_top_open(17.0, 1.5);
}

/// A rim round smaller than the wall with the boss clear of the top face.
#[test]
fn shell_small_rounded_rim_with_untangent_boss() {
    check_rounded_top_open(15.0, 1.5);
}

#[test]
#[ignore = "diagnostic: debug print-out with env inputs, no assertion; run by hand"]
fn zz_debug_cavity() {
    use ck_ops::boolean::{boolean, Op};
    let zc: f64 = std::env::var("ZC").ok().and_then(|s| s.parse().ok()).unwrap_or(15.0);
    let face = std::env::var("OPEN").unwrap_or("F2.C1".into());
    let rim: Option<f64> = std::env::var("RIM").ok().and_then(|s| s.parse().ok());
    let solid = run(&match rim {
        Some(r) => part_rounded(zc, r, ""),
        None => part(zc, ""),
    });
    assert!(solid.errors.is_empty(), "{:?}", solid.errors);
    let b = solid.body.as_ref().unwrap();
    for i in ck_check::validate(b, 1e-6) {
        eprintln!("solid issue {} {} {}", i.code, i.entity, i.detail);
    }
    let dump = |tag: &str, x: &ck_topo::body::Body| {
        for f in x.face_ids() {
            eprintln!("{tag} face {:?} {} {} loops {}", f, x.face_id_str(f), x.face(f).surface.kind(), x.face(f).loops.len());
        }
        for e in x.edge_ids() {
            let ed = x.edge(e);
            eprintln!(
                "{tag} edge {e:?} {} len {:.6} {:?} -> {:?} faces {:?}",
                ed.curve.kind(),
                x.edge_length(e),
                x.vertex(ed.v0).p,
                x.vertex(ed.v1).p,
                x.edge_faces(e).iter().map(|&f| x.face_id_str(f)).collect::<Vec<_>>()
            );
        }
    };
    dump("solid", b);
    let open = b.find_face(&face).unwrap();
    for (tag, d) in [("closed", -3.0), ("overshoot", 0.75)] {
        let t = std::time::Instant::now();
        let cav = ck_ops::offset::offset_body(b, &|f| if f == open { d } else { -3.0 });
        eprintln!("{tag} offset {:.2}s", t.elapsed().as_secs_f64());
        let cav = match cav {
            Ok(c) => c,
            Err(e) => {
                eprintln!("{tag} offset failed {e:?}");
                continue;
            }
        };
        for i in ck_check::validate(&cav, 1e-6) {
            eprintln!("{tag} cavity issue {} {} {}", i.code, i.entity, i.detail);
        }
        eprintln!("{tag} cavity volume {}", ck_topo::mass::mass_props(&cav).volume);
        dump(tag, &cav);
        let _skip = ck_ops::boolean::imprint::skip_pairs(b.face_ids().zip(cav.face_ids()).filter(|(a, c)| a == c).collect());
        let t = std::time::Instant::now();
        match boolean(b, &cav, Op::Cut, 1e-6) {
            Ok(o) => eprintln!("{tag} cut ok {:.2}s volume {} issues {}", t.elapsed().as_secs_f64(), ck_topo::mass::mass_props(&o.body).volume, ck_check::validate(&o.body, 1e-6).len()),
            Err(e) => eprintln!("{tag} cut failed {:.2}s: {e}", t.elapsed().as_secs_f64()),
        }
    }
}
