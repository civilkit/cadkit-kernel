//! Stage 1: triangle mesh -> faceted solid. Round trips of our own prismatic solids (planar merging must give back the
//! original planar faces) and the refusals / repairs.
mod common;
use common::*;
use ck_geom::Vec3;
use ck_mesh::mesh_to_solid;
use std::time::Instant;

fn faceted(pos: &[f64], ix: Option<&[u32]>) -> ck_mesh::Converted {
    match mesh_to_solid(pos, ix, &opts_faceted()) {
        Ok(c) => c,
        Err(r) => panic!("refused {}: {} {:?}", r.code, r.message, r.data),
    }
}

#[test]
fn box_round_trip() {
    let b = body_of(RECT);
    let s = soup(&b, 1.0);
    let c = faceted(&s, None);
    assert_solid(&c, 1e-6);
    assert_eq!(c.body.num_faces(), 6, "{}", kinds(&c.body));
    assert_eq!(c.report.lumps, 1);
    assert!((c.report.body_volume - 24000.0).abs() < 1e-2, "{}", c.report.body_volume);
}

#[test]
fn box_with_through_hole() {
    let b = body_of(&format!(
        r#"{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":5,"y":3}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}}"#
    ));
    let s = soup(&b, 1.0);
    let c = faceted(&s, None);
    assert_solid(&c, 1e-6);
    // the 6 planes of the box come back as 6 faces (the top and bottom with a hole loop); the rest are the strips of the hole
    let holed: Vec<_> = c.body.face_ids().filter(|&f| c.body.face(f).loops.len() == 2).collect();
    assert_eq!(holed.len(), 2, "faces with a hole loop");
    let n = c.body.num_faces();
    let strips = n - 6;
    eprintln!("box with hole: {} triangles -> {n} faces ({strips} strips)", s.len() / 9);
    // the hole's two rims are sampled at different angles, so its facets are single triangles (no coplanar pairs)
    assert!(strips >= 8, "hole facets {strips}");
}

#[test]
fn l_bracket() {
    let b = body_of(&prism(&[(0.0, 0.0), (50.0, 0.0), (50.0, 8.0), (8.0, 8.0), (8.0, 40.0), (0.0, 40.0)], 25.0));
    let c = faceted(&soup(&b, 1.0), None);
    assert_solid(&c, 1e-6);
    assert_eq!(c.body.num_faces(), 8);
    let v = (50.0 * 8.0 + 8.0 * 32.0) * 25.0;
    assert!((c.report.body_volume - v).abs() < 1e-6 * v * 10.0, "{}", c.report.body_volume);
}

#[test]
fn chamfered_prism() {
    // a 40 x 30 block with its four vertical edges chamfered 4 (in the profile) and a 45 degree bevel strip
    let k = 4.0;
    let b = body_of(&prism(&[(k, 0.0), (40.0 - k, 0.0), (40.0, k), (40.0, 30.0 - k), (40.0 - k, 30.0), (k, 30.0), (0.0, 30.0 - k), (0.0, k)], 12.0));
    let c = faceted(&soup(&b, 1.0), None);
    assert_solid(&c, 1e-6);
    assert_eq!(c.body.num_faces(), 10);
    // straight edges keep no intermediate vertices: 8 + 8 vertices
    assert_eq!(c.body.vertex_ids().count(), 16);
}

#[test]
fn rotated_and_far_from_origin() {
    // the L bracket turned about an oblique axis and moved 1e4 away: same faces
    let b = body_of(&prism(&[(0.0, 0.0), (50.0, 0.0), (50.0, 8.0), (8.0, 8.0), (8.0, 40.0), (0.0, 40.0)], 25.0));
    let s = soup(&b, 1.0);
    let ax = Vec3::new(1.0, 2.0, 3.0).normalize();
    let ang: f64 = 0.7;
    let rot = |p: Vec3| p * ang.cos() + ax.cross(p) * ang.sin() + ax * ax.dot(p) * (1.0 - ang.cos()) + Vec3::new(1e4, -2e4, 5e3);
    let t: Vec<f64> = s.chunks(3).flat_map(|c| { let q = rot(Vec3::new(c[0], c[1], c[2])); [q.x, q.y, q.z] }).collect();
    let c = faceted(&t, None);
    assert_solid(&c, 1e-6);
    assert_eq!(c.body.num_faces(), 8);
}

#[test]
fn open_mesh_is_refused_with_positions() {
    let (p, mut ix) = cube(Vec3::ZERO, 10.0);
    ix.truncate(ix.len() - 3); // one triangle of the x = 10 face gone
    let r = mesh_to_solid(&p, Some(&ix), &opts_faceted()).err().expect("refused");
    assert_eq!(r.code, "meshNotWatertight");
    assert_eq!(r.get_f64("openEdges"), Some(3.0));
    assert_eq!(r.get_f64("openLoops"), Some(1.0));
    let pos = r.data.iter().find(|(k, _)| *k == "positions").unwrap().1.as_arr().unwrap().len();
    assert_eq!(pos, 3);
    assert!(!r.hint.is_empty());
}

#[test]
fn non_manifold_edge_is_refused() {
    // two cubes sharing one edge: that edge has four triangles
    let (mut p, mut ix) = cube(Vec3::ZERO, 10.0);
    let (p2, ix2) = cube(Vec3::new(10.0, 10.0, 0.0), 10.0);
    let off = (p.len() / 3) as u32;
    p.extend(p2);
    ix.extend(ix2.iter().map(|i| i + off));
    let r = mesh_to_solid(&p, Some(&ix), &opts_faceted()).err().expect("refused");
    assert_eq!(r.code, "meshNonManifold");
    assert_eq!(r.get_f64("nonManifoldEdges"), Some(1.0));
}

#[test]
fn inside_out_and_flipped_triangles_are_repaired() {
    let (p, ix) = cube(Vec3::ZERO, 10.0);
    // everything reversed
    let rev: Vec<u32> = ix.chunks(3).flat_map(|t| [t[0], t[2], t[1]]).collect();
    let c = faceted(&p, Some(&rev));
    assert_solid(&c, 1e-9);
    assert_eq!(c.report.inside_out, 1);
    assert_eq!(c.report.flipped, 0);
    assert!((c.report.body_volume - 1000.0).abs() < 1e-9);
    // three triangles reversed
    let mut mix = ix.clone();
    for t in [1usize, 4, 9] {
        mix.swap(3 * t + 1, 3 * t + 2);
    }
    let c = faceted(&p, Some(&mix));
    assert_solid(&c, 1e-9);
    assert_eq!(c.report.flipped, 3);
    assert_eq!(c.report.inside_out, 0);
    assert_eq!(c.body.num_faces(), 6);
}

#[test]
fn void_and_separate_lumps() {
    // a cube with a cubic void (the inner one given outward: it is turned inward), and a separate cube
    let (mut p, mut ix) = cube(Vec3::ZERO, 10.0);
    for (o, s) in [(Vec3::new(3.0, 3.0, 3.0), 4.0), (Vec3::new(20.0, 0.0, 0.0), 5.0)] {
        let (p2, ix2) = cube(o, s);
        let off = (p.len() / 3) as u32;
        p.extend(p2);
        ix.extend(ix2.iter().map(|i| i + off));
    }
    let c = faceted(&p, Some(&ix));
    assert_solid(&c, 1e-9);
    assert_eq!(c.report.components, 3);
    assert_eq!(c.report.voids, 1);
    assert_eq!(c.report.lumps, 2);
    assert_eq!(c.body.num_lumps(), 2);
    assert!((c.report.body_volume - (1000.0 - 64.0 + 125.0)).abs() < 1e-9, "{}", c.report.body_volume);
}

#[test]
fn soup_welding_duplicates_and_collapsed() {
    let (p, ix) = cube(Vec3::ZERO, 10.0);
    // unindexed soup with a little noise below the weld tolerance, one duplicate triangle and one collapsed triangle
    let mut s: Vec<f64> = ix.iter().flat_map(|&i| [p[3 * i as usize], p[3 * i as usize + 1], p[3 * i as usize + 2]]).collect();
    for (k, v) in s.iter_mut().enumerate() {
        *v += 1e-9 * ((k * 7919) % 13) as f64;
    }
    let first: Vec<f64> = s[0..9].to_vec();
    s.extend(first);
    s.extend([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0]);
    let c = faceted(&s, None);
    assert_solid(&c, 1e-6);
    assert_eq!(c.report.welded_vertices, 8);
    assert_eq!(c.report.duplicates, 1);
    assert_eq!(c.report.collapsed, 1);
    assert_eq!(c.body.num_faces(), 6);
}

#[test]
fn bad_input_and_limits() {
    let (p, ix) = cube(Vec3::ZERO, 10.0);
    let r = mesh_to_solid(&p, Some(&[0, 1, 99]), &opts_faceted()).err().unwrap();
    assert_eq!(r.code, "meshBadInput");
    let r = mesh_to_solid(&p[..7], None, &opts_faceted()).err().unwrap();
    assert_eq!(r.code, "meshBadInput");
    let mut o = opts_faceted();
    o.max_triangles = 10;
    let r = mesh_to_solid(&p, Some(&ix), &o).err().unwrap();
    assert_eq!(r.code, "meshTooLarge");
    assert_eq!(r.get_f64("triangles"), Some(12.0));
    assert_eq!(r.get_f64("limit"), Some(10.0));
    let mut o = opts_faceted();
    o.max_faces = 5;
    let r = mesh_to_solid(&p, Some(&ix), &o).err().unwrap();
    assert_eq!(r.code, "meshTooLarge");
    assert_eq!(r.get_f64("faces"), Some(6.0));
    // everything flat: a doubled square
    let flat = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 0.0];
    let r = mesh_to_solid(&flat, Some(&[0, 1, 2, 0, 2, 3, 0, 2, 1, 0, 3, 2]), &opts_faceted()).err().unwrap();
    assert_eq!(r.code, "meshDegenerate", "{}", r.message);
}

#[test]
fn sliver_triangles_join_a_neighbour() {
    // the top of the cube has a vertex in the middle of its edge 4-5 that the side face does not have: a zero-height
    // triangle (5, 8, 4) fills the T, as some exporters write it
    let mut p: Vec<f64> = vec![];
    let (cp, cix) = cube(Vec3::ZERO, 10.0);
    p.extend(&cp);
    p.extend([5.0, 0.0, 10.0]); // vertex 8 on the edge 4-5
    let mut ix: Vec<u32> = cix[..6].to_vec();
    ix.extend(&cix[12..]); // without the top quad (4 5 7 6)
    ix.extend([4, 8, 6, 8, 5, 7, 8, 7, 6]);
    ix.extend([5, 8, 4]);
    let c = faceted(&p, Some(&ix));
    assert_solid(&c, 1e-9);
    assert_eq!(c.report.slivers, 1);
    assert_eq!(c.body.num_faces(), 6, "{}", kinds(&c.body));
    assert_eq!(c.body.vertex_ids().count(), 8);
}

#[test]
fn bounded_runtime_on_a_fine_mesh() {
    let b = body_of(&format!(
        r#"{RECT},{{"type":"sketch","id":"F3","plane":{{"kind":"face","id":"F2.C1"}},"points":[{{"id":"k","x":5,"y":3}}],"entities":[{{"id":"pt1","type":"point","p":"k"}}],"constraints":[]}},
        {{"type":"hole","id":"F4","sketch":"F3","kind":"simple","diameter":6,"end":"through"}}"#
    ));
    let s = soup(&b, 0.01);
    let t = Instant::now();
    let c = faceted(&s, None);
    let ms = t.elapsed().as_secs_f64() * 1e3;
    eprintln!("fine box with hole: {} triangles -> {} faces in {ms:.0} ms", s.len() / 9, c.body.num_faces());
    assert_solid(&c, 1e-6);
    assert!(ms < 60_000.0, "{ms} ms");
}
