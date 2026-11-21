//! Fillet / chamfer on the end-face edges of an extrude with a twist (issue 67): a 40 x 40 square on the front
//! plane, extruded 40 mm with `twist`. The end faces are planes, the side faces screw surfaces.
//!
//! The rolling-ball tool's end runs on past the chain end. On a twisted face that extension is carried along the
//! face (a straight shift left the screw surface, so the tool's contact curve missed the face's boundary edge and the
//! boolean lost the neighbouring side face). A ring of end edges is split into one chain per side; the chains meet in
//! a mitre like plain edge fillets. A chamfer next to a twisted face is refused: chamfers are only built between
//! planes (or along a cylinder).
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

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

fn base(twist: f64) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"p1","x":-20,"y":-20}},{{"id":"p2","x":20,"y":-20}},{{"id":"p3","x":20,"y":20}},{{"id":"p4","x":-20,"y":20}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"S1","end":"blind","depth":40,"twist":{twist}}}]}}"#
    )
}

fn regen_src(src: &str) -> (Vec<String>, Option<ck_topo::body::Body>) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect(), ex.0.body.take())
}

/// The part with a blend of kind `kind` (radius / distance `r`) on the edges of the cap face `cap`; `only` = just that edge
/// of the cap's four.
fn blended(twist: f64, kind: &str, r: f64, cap: &str, only: Option<usize>) -> (Vec<String>, Option<ck_topo::body::Body>) {
    let (e, b) = regen_src(&base(twist));
    assert!(e.is_empty(), "{e:?}");
    let b = b.unwrap();
    let cf = b.find_face(cap).expect("cap face");
    let mut edges = vec![];
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        if fs.len() == 2 && fs.contains(&cf) {
            let other = if fs[0] == cf { fs[1] } else { fs[0] };
            let m = b.edge_mid(ed);
            edges.push(format!(r#"{{"faces":["{cap}","{}"],"at":[{},{},{}]}}"#, b.face_id_str(other), m.x, m.y, m.z));
        }
    }
    assert_eq!(edges.len(), 4, "{edges:?}");
    if let Some(k) = only {
        edges = vec![edges[k].clone()];
    }
    let param = if kind == "fillet" { "radius" } else { "distance" };
    let src = base(twist).replace("}]}", &format!(r#"}},{{"type":"{kind}","id":"F3","{param}":{r},"edges":[{}]}}]}}"#, edges.join(",")));
    regen_src(&src)
}

/// Fillet r on the top-face edges whose other face is one of `sides` (e.g. ["F2.S.e1", "F2.S.e2"]: two edges that
/// meet at a corner).
fn fillet_sides(twist: f64, r: f64, sides: &[&str]) -> (Vec<String>, Option<ck_topo::body::Body>) {
    let (_, b) = regen_src(&base(twist));
    let b = b.unwrap();
    let cf = b.find_face("F2.C1").unwrap();
    let mut edges = vec![];
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        if fs.len() == 2 && fs.contains(&cf) {
            let other = b.face_id_str(if fs[0] == cf { fs[1] } else { fs[0] });
            if sides.contains(&other.as_str()) {
                let m = b.edge_mid(ed);
                edges.push(format!(r#"{{"faces":["F2.C1","{other}"],"at":[{},{},{}]}}"#, m.x, m.y, m.z));
            }
        }
    }
    assert_eq!(edges.len(), sides.len());
    regen_src(&base(twist).replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F3","radius":{r},"edges":[{}]}}]}}"#, edges.join(","))))
}

/// Fillet r on the edges between the given face pairs.
fn fillet_pairs(twist: f64, r: f64, pairs: &[(&str, &str)]) -> (Vec<String>, Option<ck_topo::body::Body>) {
    let (_, b) = regen_src(&base(twist));
    let b = b.unwrap();
    let mut edges = vec![];
    for (fa, fb) in pairs {
        let (a, c) = (b.find_face(fa).unwrap(), b.find_face(fb).unwrap());
        let ed = b.edge_ids().find(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && fs.contains(&a) && fs.contains(&c)
        });
        let m = b.edge_mid(ed.expect("edge"));
        edges.push(format!(r#"{{"faces":["{fa}","{fb}"],"at":[{},{},{}]}}"#, m.x, m.y, m.z));
    }
    regen_src(&base(twist).replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F3","radius":{r},"edges":[{}]}}]}}"#, edges.join(","))))
}

/// The corner of two end edges and the vertical edge, filleted together (the plain extrude gets the corner's sphere patch;
/// on a twisted one the three blends meet in three mitre curves that join at one point, no patch). Shape: valid, no tool
/// face left, three blend faces and nothing else new, one mitre between each pair of blends, the three mitres
/// share one end, and on each of the three faces and the top the blends' contact lines meet end to end (no step, no
/// jog); under the bound in wall-clock time.
#[test]
fn three_edge_corner_shape() {
    let sel = [("F2.S.e1", "F2.S.e2"), ("F2.C1", "F2.S.e1"), ("F2.C1", "F2.S.e2")];
    for tw in [0.0, 5.0, 10.0, 30.0] {
        for r in [0.5, 2.0] {
            let t = std::time::Instant::now();
            let (e, b) = fillet_pairs(tw, r, &sel);
            assert!(t.elapsed().as_secs_f64() < 60.0);
            assert!(e.is_empty(), "twist {tw} r {r}: {e:?}");
            let b = b.unwrap();
            assert!(ck_topo::check::check(&b, 1e-6).is_empty(), "twist {tw} r {r}: invalid {:?}", ck_topo::check::check(&b, 1e-6));
            let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            assert!(names.iter().all(|s| !s.contains(".T")), "{names:?}");
            let blends = names.iter().filter(|s| s.starts_with("F3.B")).count();
            let patches = names.iter().filter(|s| s.starts_with("F3.V")).count();
            assert_eq!(blends, 3, "{names:?}");
            assert_eq!(names.len(), 6 + 3 + patches, "{names:?}");
            if tw == 0.0 {
                assert_eq!(patches, 1, "the plain corner has a sphere patch: {names:?}");
                continue;
            }
            assert_eq!(patches, 0);
            let is_b = |f| b.face_id_str(f).starts_with("F3.B");
            let mitres: Vec<_> = b.edge_ids().filter(|&ed| b.edge_faces(ed).len() == 2 && b.edge_faces(ed).iter().all(|&f| is_b(f))).collect();
            assert_eq!(mitres.len(), 3, "twist {tw} r {r}");
            // the three mitres have one common end
            let ends = |ed| [b.vertex(b.edge(ed).v0).p, b.vertex(b.edge(ed).v1).p];
            let common = ends(mitres[0]).into_iter().filter(|p| mitres[1..].iter().all(|&m| ends(m).iter().any(|q| q.dist(*p) < 1e-5))).count();
            assert_eq!(common, 1, "twist {tw} r {r}: the mitres do not meet at one point");
            // every other end of a mitre is where a contact line of the two blends on a face ends: it has exactly
            // two further blend edges (on a side face or the top) of its own at that point
            for &m in &mitres {
                for p in ends(m) {
                    let at = b
                        .edge_ids()
                        .filter(|&ed| b.edge_faces(ed).len() == 2 && ends(ed).iter().any(|q| q.dist(p) < 1e-5))
                        .count();
                    assert!(at == 3 || at == 4, "twist {tw} r {r}: {at} edges end at a mitre end {p:?}");
                }
            }
            // the blend contact lines on the faces are not interrupted: every end of an edge between a blend and a
            // plain face is shared with another edge that touches that face
            for ed in b.edge_ids() {
                let fs = b.edge_faces(ed);
                if fs.len() == 2 && fs.iter().filter(|&&f| is_b(f)).count() == 1 && b.edge_length(ed) < 4.0 * r {
                    // short blend/plain edges (the ends of a contact line) must reach another blend edge at both ends
                    for p in ends(ed) {
                        let n = b.edge_ids().filter(|&x| x != ed && b.edge_faces(x).len() == 2 && ends(x).iter().any(|q| q.dist(p) < 1e-5)).count();
                        assert!(n >= 2, "twist {tw} r {r}: a dangling end at {p:?}");
                    }
                }
            }
        }
    }
}

/// Selections that include the helical vertical edge (the corner where two end edges and the vertical edge meet)
/// are refused or built, but answer within seconds either way.
#[test]
fn helical_edge_selections_answer_quickly() {
    let top_and_vertical = [("F2.S.e1", "F2.S.e2"), ("F2.C1", "F2.S.e1")];
    let three = [("F2.S.e1", "F2.S.e2"), ("F2.C1", "F2.S.e1"), ("F2.C1", "F2.S.e2")];
    {
        let t = std::time::Instant::now();
        let (e, _) = fillet_pairs(10.0, 2.0, &[("F2.S.e1", "F2.S.e2")]);
        println!("TIMING vertical alone: {:.2} s {:?}", t.elapsed().as_secs_f64(), e.iter().map(|s| s.chars().take(120).collect::<String>()).collect::<Vec<_>>());
    }
    for tw in [10.0, 30.0] {
        for (name, sel) in [("vertical + one top edge", &top_and_vertical[..]), ("three edges at a corner", &three[..])] {
            let t = std::time::Instant::now();
            let (e, b) = fillet_pairs(tw, 2.0, sel);
            let dt = t.elapsed().as_secs_f64();
            println!("TIMING twist {tw} {name}: {dt:.2} s, errors {:?}", e.iter().map(|s| s.chars().take(90).collect::<String>()).collect::<Vec<_>>());
            assert!(dt < 60.0, "twist {tw} {name}: {dt:.1} s");
            if sel.len() == 3 {
                // (shape: `three_edge_corner_shape`)
                assert!(e.is_empty(), "{e:?}");
            } else {
                // two edges at a corner build: the vertical blend and the top blend meet in one mitre
                assert!(e.is_empty(), "{e:?}");
                let b = b.unwrap();
                assert!(ck_topo::check::check(&b, 1e-6).is_empty(), "invalid: {:?}", ck_topo::check::check(&b, 1e-6));
                let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
                assert_eq!(names.len(), 6 + 2, "{names:?}");
                assert!(names.iter().all(|s| !s.contains(".T")), "{names:?}");
                let v = ck_topo::mass::mass_props(&b).volume;
                let wedge = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0 * 40.0;
                assert!(v < 64000.0 - 1.5 * wedge && v > 64000.0 - 2.5 * wedge, "volume {v} for two wedges of {wedge}");
            }
        }
    }
    // a build that cannot succeed (a ring on a 90 degree twist) is refused within the bound as well, the largest-size
    // search included, under a host clock
    let t0 = std::time::Instant::now();
    ck_core::budget::set_clock(|| {
        thread_local!(static T0: std::time::Instant = std::time::Instant::now());
        T0.with(|t| t.elapsed().as_secs_f64() * 1e3)
    });
    let t = std::time::Instant::now();
    let (e, _) = blended(90.0, "fillet", 2.0, "F2.C1", None);
    let dt = t.elapsed().as_secs_f64();
    println!("TIMING refused 90 degree ring: {dt:.2} s {:?} (test {:.1} s)", e.iter().map(|s| s.chars().take(60).collect::<String>()).collect::<Vec<_>>(), t0.elapsed().as_secs_f64());
    // a guard against a runaway, not a benchmark: 6 s on a quiet box, 10 s and then 30 s were exceeded on a loaded one (several jobs at once)
    assert!(dt < 120.0, "{dt:.1} s");
}

/// Shape of the corners of a fillet of `n` end edges (a ring, or two that meet at one corner): the blend faces meet
/// pairwise along ONE mitre curve each, no tool cap or other stray face is left in the body, and the only step is the
/// short edge where the later blend dips into the side face of the earlier one. Its size comes from the dihedral of the
/// two screw faces differing by about the twist, so it is about 0 for a plain extrude and grows with the twist.
/// Returns that step (0 when there is none).
fn corner_shape(twist: f64, r: f64, sides: &[&str]) -> f64 {
    let (e, b) = fillet_sides(twist, r, sides);
    assert!(e.is_empty(), "twist {twist} r {r} {sides:?}: {e:?}");
    let b = b.unwrap();
    assert!(ck_topo::check::check(&b, 1e-6).is_empty(), "invalid: {:?}", ck_topo::check::check(&b, 1e-6));
    let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    let n = sides.len();
    assert_eq!(names.len(), 6 + n, "twist {twist} r {r} {sides:?}: faces {names:?}");
    assert!(names.iter().all(|s| !s.contains(".T")), "a tool cap or leg face is left in the body: {names:?}");
    let is_blend = |f| b.face_id_str(f).starts_with("F3.B");
    let mut mitres = 0;
    let mut step = 0.0f64;
    // the lower ends of the corners' vertical edges (side face against side face)
    let mut verticals: Vec<ck_geom::Vec3> = vec![];
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        if fs.len() == 2 && fs.iter().all(|&f| b.face_id_str(f).starts_with("F2.S")) {
            let (p, q) = (b.vertex(b.edge(ed).v0).p, b.vertex(b.edge(ed).v1).p);
            verticals.push(if p.z > q.z { p } else { q });
        }
    }
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        if fs.len() == 2 && fs.iter().all(|&f| is_blend(f)) {
            mitres += 1;
            let len = b.edge_length(ed);
            // the mitre runs from the top face to the corner's vertical edge, about the blend's own size
            assert!(len > 0.5 * r && len < 6.0 * r, "mitre length {len}");
            // its lower end is on the vertical edge: the gap between them is the step the second blend leaves
            let (p, q) = (b.vertex(b.edge(ed).v0).p, b.vertex(b.edge(ed).v1).p);
            let low = if p.z < q.z { p } else { q };
            let gap = verticals.iter().map(|v| v.dist(low)).fold(f64::INFINITY, f64::min);
            step = step.max(gap);
        }
    }
    // a ring has a mitre at each of its corners, two edges one
    assert_eq!(mitres, if n == 4 { 4 } else { 1 }, "twist {twist} r {r} {sides:?}: mitre curves");
    // the blend faces' top contact lines end exactly at the mitre's top end (no jog in the end-face outline)
    let top = b.find_face("F2.C1").unwrap();
    let mut ends: Vec<ck_geom::Vec3> = vec![];
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        if fs.len() == 2 && fs.contains(&top) {
            ends.push(b.vertex(b.edge(ed).v0).p);
            ends.push(b.vertex(b.edge(ed).v1).p);
        }
    }
    for p in &ends {
        let same = ends.iter().filter(|q| q.dist(*p) < 1e-5).count();
        assert!(same == 2, "the end-face outline has a jog at {p:?} ({same} edge ends)");
    }
    step
}

#[test]
fn corners_are_mitres_for_two_edges_and_rings() {
    let two = ["F2.S.e1", "F2.S.e2"];
    let all = ["F2.S.e1", "F2.S.e2", "F2.S.e3", "F2.S.e4"];
    // the plain extrude is the reference
    for r in [0.5, 2.0] {
        assert_eq!(corner_shape(0.0, r, &two), 0.0);
        assert_eq!(corner_shape(0.0, r, &all), 0.0);
    }
    for tw in [5.0, 10.0, 30.0] {
        for r in [0.5, 2.0] {
            for sides in [&two[..], &all[..]] {
                let step = corner_shape(tw, r, sides);
                // the step is a fraction of the twist angle times the radius (0.35 mm for 10 degrees, r 2)
                assert!(step <= 0.03 * tw * r, "twist {tw} r {r} {}: step {step}", sides.len());
            }
        }
    }
}

fn check_ok(twist: f64, r: f64, cap: &str, only: Option<usize>) {
    let (e, b) = blended(twist, "fillet", r, cap, only);
    assert!(e.is_empty(), "fillet r={r} twist={twist} on {cap}: {e:?}");
    let b = b.unwrap();
    assert!(ck_topo::check::check(&b, 1e-6).is_empty(), "invalid: {:?}", ck_topo::check::check(&b, 1e-6));
    let v = ck_topo::mass::mass_props(&b).volume;
    // twisting keeps the volume of the plain 40 x 40 x 40 box; the fillet only removes material, at most the
    // (1 - pi/4) r^2 wedge along the edges (160 mm of them for the ring, 40 for one edge)
    let n = if only.is_some() { 1.0 } else { 4.0 };
    let wedge = (1.0 - std::f64::consts::FRAC_PI_4) * r * r * 40.0 * n;
    assert!(v < 64000.0 - 0.5 * wedge && v > 64000.0 - 2.0 * wedge, "volume {v}, wedge {wedge}");
}

#[test]
fn twist_1_fillet_top_ring_r0_5() {
    check_ok(1.0, 0.5, "F2.C1", None);
}
#[test]
fn twist_1_fillet_bottom_ring_r0_5() {
    check_ok(1.0, 0.5, "F2.C0", None);
}
#[test]
fn twist_10_fillet_single_top_edge_r2() {
    check_ok(10.0, 2.0, "F2.C1", Some(0));
}

#[test]
fn twist_5_fillet_top_ring_r2() {
    check_ok(5.0, 2.0, "F2.C1", None);
}
#[test]
fn twist_30_fillet_top_ring_r2_and_r0_5() {
    check_ok(30.0, 2.0, "F2.C1", None);
    check_ok(30.0, 0.5, "F2.C1", None);
}
#[test]
fn twist_30_fillet_single_edge_r2_and_r0_5() {
    check_ok(30.0, 2.0, "F2.C1", Some(0));
    check_ok(30.0, 0.5, "F2.C1", Some(0));
}
#[test]
fn twist_30_fillet_bottom_ring_r2_and_r0_5() {
    check_ok(30.0, 2.0, "F2.C0", None);
    check_ok(30.0, 0.5, "F2.C0", None);
}
#[test]
fn twist_negative_and_45_degree_rings() {
    check_ok(-30.0, 2.0, "F2.C1", None);
    check_ok(45.0, 1.0, "F2.C1", None);
    check_ok(45.0, 2.0, "F2.C1", None);
}
#[test]
fn twist_60_and_90_single_edge_and_60_r2_ring() {
    check_ok(60.0, 1.0, "F2.C1", Some(0));
    check_ok(90.0, 1.0, "F2.C1", Some(0));
    check_ok(60.0, 2.0, "F2.C1", None);
}
#[test]
#[ignore = "open (issue 67): a ring of four end edges at a twist of 60 degrees and over: the last chains' tools meet two earlier blends and do not close (r 1 at 60 degrees; r 1 and r 2 at 90 degrees, refused by the volume check)"]
fn twist_60_and_90_rings() {
    check_ok(60.0, 1.0, "F2.C1", None);
    check_ok(90.0, 1.0, "F2.C1", None);
}
#[test]
#[ignore = "open (issue 67): chamfers are only built between planes"]
fn twist_30_chamfer_top_ring_r2() {
    let (e, _) = blended(30.0, "chamfer", 2.0, "F2.C1", None);
    assert!(e.is_empty(), "{e:?}");
}

/// The chamfer refusal names the real limit.
#[test]
fn chamfer_refusal_names_the_twisted_face() {
    let (e, _) = blended(30.0, "chamfer", 2.0, "F2.C1", None);
    assert!(e.len() == 1 && e[0].contains("blendUnsupported") && e[0].contains("twisted extrude face"), "{e:?}");
}
