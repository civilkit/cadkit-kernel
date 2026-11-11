use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

pub struct Out {
    pub brep: Brep,
    pub errors: Vec<regen::RegenError>,
}

pub fn run_part(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.brep, errors: r.errors }
}

// A square-to-circle loft sheet, 100 mm apart, thickened (the kit report: "the offset faces do not meet at a corner").

fn part(half: f64, r: f64, gap: f64, t: f64, side: &str, end_tension: f64) -> String {
    let mut f = vec![];
    for (i, z) in [0.0, gap].iter().enumerate() {
        f.push(format!(r#"{{"type":"plane","id":"P{i}","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{z}}}"#));
    }
    f.push(format!(
        r#"{{"type":"sketch","id":"S0","plane":{{"kind":"feature","id":"P0"}},"points":[{{"id":"a","x":{h},"y":{h}}},{{"id":"b","x":-{h},"y":{h}}},{{"id":"c","x":-{h},"y":-{h}}},{{"id":"d","x":{h},"y":-{h}}}],"entities":[{{"type":"line","id":"l1","p1":"a","p2":"b"}},{{"type":"line","id":"l2","p1":"b","p2":"c"}},{{"type":"line","id":"l3","p1":"c","p2":"d"}},{{"type":"line","id":"l4","p1":"d","p2":"a"}}],"constraints":[]}}"#,
        h = half
    ));
    f.push(format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"feature","id":"P1"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}}"#
    ));
    f.push(format!(r#"{{"type":"loft","id":"F8","op":"add","surface":true,"sketches":["S0","S1"],"endNormal":{end_tension}}}"#));
    f.push(format!(r#"{{"type":"thicken","id":"F9","surface":"F8","thickness":{t},"side":"{side}"}}"#));
    format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, f.join(","))
}

#[test]
fn sweep() {
    if std::env::var("SWEEP").is_err() {
        return;
    }
    for gap in [40.0, 100.0] {
        for tension in [0.0, 2.0] {
            for (half, r) in [(50.0, 15.0), (50.0, 25.0), (25.0, 10.0), (25.0, 15.0)] {
                for t in [1.0, 2.0, 3.0, 5.0] {
                    for side in ["front", "back", "both"] {
                        let o = run_part(&part(half, r, gap, t, side, tension));
                        let e: Vec<String> = o.errors.iter().map(|e| format!("{:?}", e.error)).collect();
                        eprintln!("SWEEP gap {gap} tension {tension} sq {half} r {r} t {t} {side}: {}", if e.is_empty() { "ok".into() } else { e.join(";") });
                    }
                }
            }
        }
    }
}

#[test]
fn one() {
    let g = |k: &str, d: f64| std::env::var(k).ok().and_then(|v| v.parse().ok()).unwrap_or(d);
    let side = std::env::var("SIDE").unwrap_or("front".into());
    let o = run_part(&part(g("HALF", 50.0), g("R", 15.0), 100.0, g("T", 5.0), &side, g("TEN", 2.0)));
    eprintln!("ONE {:?}", o.errors.iter().map(|e| format!("{:?}", e.error)).collect::<Vec<_>>());
}

/// The kit report: a square and a circle 100 mm apart lofted to a sheet could not be thickened outwards or both ways
/// ("the offset faces do not meet at a corner"): the offset surfaces were continued only 3 % of a face past its edge, short
/// of the miter at the square's corners once the wall was a few millimetres. Every side builds a valid solid whose volume is
/// the sheet's area times the thickness, as the inside thicken's is (the outside is larger, as it lies on the sheet's wider side).
#[test]
fn square_to_circle_loft_thickens_on_every_side() {
    for (half, r, t, tension) in [(50.0, 15.0, 3.0, 0.0), (50.0, 15.0, 5.0, 2.0), (50.0, 10.0, 5.0, 0.0), (25.0, 10.0, 3.0, 2.0)] {
        let back = run_part(&part(half, r, 100.0, t, "back", tension));
        assert!(back.errors.is_empty(), "back {half} {r} {t}: {:?}", back.errors);
        let vb = volume(&back);
        let mut vs = vec![];
        for side in ["both", "front"] {
            let o = run_part(&part(half, r, 100.0, t, side, tension));
            assert!(o.errors.is_empty(), "{side} {half} {r} {t} {tension}: {:?}", o.errors);
            assert_valid_solid(side, &o);
            vs.push(volume(&o));
        }
        eprintln!("loft thicken {half} {r} {t}: back {vb:.1} both {:.1} front {:.1}", vs[0], vs[1]);
        // the outside wall is on the larger side of the tapering sheet, so its volume is the largest, the middle one between
        assert!(vb < vs[0] && vs[0] < vs[1] && vs[1] < 1.2 * vb, "{half} {r} {t}: volumes back {vb} both {} front {}", vs[0], vs[1]);
    }
}

fn volume(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().expect("a body")).volume
}

fn assert_valid_solid(name: &str, o: &Out) {
    let b = o.brep.body.as_ref().expect("a body");
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{name}: {iss:?}");
}

/// Every face of a thickened loft meshes to its own area (the shards on the rim walls in the kit were a mesh of the wrong region).
#[test]
fn thickened_loft_meshes_to_its_exact_area() {
    let mut bad = vec![];
    for (gap, half, r, t, side) in [(40.0, 50.0, 25.0, 3.0, "front"), (40.0, 50.0, 25.0, 3.0, "both"), (100.0, 50.0, 15.0, 3.0, "front"), (40.0, 50.0, 25.0, 3.0, "back")] {
        let o = run_part(&part(half, r, gap, t, side, 0.0));
        assert!(o.errors.is_empty(), "{:?}", o.errors);
        let b = o.brep.body.as_ref().unwrap();
        // the app meshes through the per-face cache, twice over (a rebuild reuses it): the cached mesh must be the plain one
        let _warm = ck_tess::mesh::tessellate_memo(b);
        let tm = ck_tess::mesh::tessellate_memo(b);
        let plain = ck_tess::mesh::tessellate(b);
        assert_eq!(tm.mesh.tri_face.len(), plain.mesh.tri_face.len(), "{side} gap {gap}: cached and plain meshes differ");
        let pos = &plain.mesh.positions;
        let mut mv = 0.0f64;
        for t in pos.chunks(9) {
            let (a, b2, c) = (ck_geom::Vec3::new(t[0] as f64, t[1] as f64, t[2] as f64), ck_geom::Vec3::new(t[3] as f64, t[4] as f64, t[5] as f64), ck_geom::Vec3::new(t[6] as f64, t[7] as f64, t[8] as f64));
            mv += a.dot(b2.cross(c)) / 6.0;
        }
        let ev = ck_topo::mass::mass_props(b).volume;
        eprintln!("MESH gap {gap} {side}: mesh volume {mv:.1} exact {ev:.1}");
        if (mv - ev).abs() > 0.02 * ev {
            bad.push(format!("{side} gap {gap}: mesh volume {mv} against exact {ev}"));
        }
        let zmin = b.vertex_ids().map(|v| b.vertex(v).p.z).fold(f64::INFINITY, f64::min);
        eprintln!("MESH gap {gap} {side}: lowest vertex z {zmin:.3}");
        for (i, id) in b.face_ids().enumerate() {
            let exact = ck_topo::mass::face_integral::<1>(b, id, 1, &|_, n| [n.len()])[0].abs();
            let kind = b.face(id).surface.kind();
            let name = b.face_id_str(id);
            match tm.faces.get(i) {
                Some(Some(fm)) => {
                    let m: f64 = fm.tris.iter().map(|t| 0.5 * (fm.pts[t[1] as usize] - fm.pts[t[0] as usize]).cross(fm.pts[t[2] as usize] - fm.pts[t[0] as usize]).len()).sum();
                    // shading: every vertex normal is a finite unit vector that agrees with the triangle's winding
                    let mut flipped = 0;
                    let mut flipped_area = 0.0;
                    let mut nonunit = 0;
                    for t in &fm.tris {
                        let (a, b, c) = (fm.pts[t[0] as usize], fm.pts[t[1] as usize], fm.pts[t[2] as usize]);
                        let g = (b - a).cross(c - a).try_normalize();
                        let nsum = fm.nrm[t[0] as usize] + fm.nrm[t[1] as usize] + fm.nrm[t[2] as usize];
                        if !nsum.is_finite() {
                            nonunit += 1;
                        } else if g.is_some_and(|g| g.dot(nsum) < 0.0) {
                            flipped += 1;
                            flipped_area += 0.5 * (b - a).cross(c - a).len();
                        }
                    }
                    // the slivers along a mitered edge (under 0.06 mm wide) may wind either way; a wall meshed inside out is the whole face
                    if flipped_area > 0.01 * exact || nonunit > 0 {
                        bad.push(format!("{side} gap {gap} {name} {kind}: {flipped} triangles ({flipped_area:.2} mm2) against the vertex normals, {nonunit} not unit"));
                    }
                    let mesh_v: f64 = fm.tris.iter().map(|t| fm.pts[t[0] as usize].dot(fm.pts[t[1] as usize].cross(fm.pts[t[2] as usize])) / 6.0).sum();
                    let exact_v = ck_topo::mass::face_integral::<1>(b, id, 1, &|p, n| [p.dot(n) / 3.0])[0];
                    eprintln!("MESH   {name} {kind}: face volume term mesh {mesh_v:.1} exact {exact_v:.1}");
                    if (mesh_v - exact_v).abs() > 0.02 * exact_v.abs().max(50.0) {
                        bad.push(format!("{side} gap {gap} {name} {kind}: volume term mesh {mesh_v:.1} against exact {exact_v:.1}"));
                    }
                    eprintln!("MESH   {name} {kind}: shading flipped {flipped} ({flipped_area:.2} mm2) nonunit {nonunit}");
                    eprintln!("MESH   {name} {kind}: mesh {m:.2} exact {exact:.2} tris {} open {} unrecovered {}", fm.tris.len(), fm.open, fm.unrecovered);
                    if (m - exact).abs() > 0.05 * exact.max(1.0) || fm.open || fm.unrecovered > 0 {
                        bad.push(format!("{side} gap {gap} {name} {kind}: mesh {m:.2} exact {exact:.2}"));
                    }
                }
                _ => bad.push(format!("{side} gap {gap} {name} {kind}: no mesh")),
            }
        }
    }
    assert!(bad.is_empty(), "{bad:#?}");
}
