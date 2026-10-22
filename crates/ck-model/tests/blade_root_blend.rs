//! Blade-root fillet: a lofted blade on a cylinder (blunt trailing edge).
//!
//! The root loop where a lofted blade meets the hub cylinder is a CLOSED chain of spline-intersection
//! edges. Two behaviours are pinned here (see docs/prop-probe.md, "Findings"):
//!
//! * selecting a SUBSET of the loop is an OPEN chain; it runs the chain-end logic and is refused;
//! * selecting the WHOLE loop is a clean ring (as many shared vertices as edges) and must blend as a
//!   ring. The ring path exists now (it is no longer refused up front), but the rolling-ball centre
//!   solve still diverges at the blade root's shallow dihedral, so the ring blend is `#[ignore]`d: it
//!   is the target of task 1, not a claim that it works.
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

/// A 3-section lofted blade on a cylinder (hub radius 6, blade radii 3.5 / 6.5 / 10, chord 7, 0.2 mm
/// blunt trailing edge), the small case the brief asks for.
const DOC: &str = include_str!("fixtures/blade_root_blunt.ckpart.json");

fn regen_doc(src: &str) -> ck_topo::body::Body {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    assert!(errs.is_empty(), "{errs:?}");
    ex.0.body.expect("body")
}

/// The edges where the blade (`F8`) meets the hub cylinder (`H2.S.e1`), as (blade face, hub face, midpoint)
/// picks. The criterion is a face a blend pick can name; the loop's connectivity is checked separately.
fn root_picks(b: &ck_topo::body::Body) -> Vec<(String, String, ck_geom::Vec3)> {
    let hub = b.face_ids().map(|f| b.face_id_str(f)).find(|n| n.starts_with("H2.S")).expect("hub cylinder face");
    b.edge_ids()
        .filter(|&e| {
            let fs: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
            fs.iter().any(|x| x == &hub) && fs.iter().any(|x| x.starts_with("F8"))
        })
        .map(|e| {
            let fs: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
            let blade = fs.iter().find(|x| x.starts_with("F8")).cloned().unwrap();
            let ed = b.edge(e);
            let mid = (b.vertex(ed.v0).p + b.vertex(ed.v1).p) * 0.5;
            (blade, hub.clone(), mid)
        })
        .collect()
}

#[test]
fn root_loop_is_a_closed_ring_of_shared_vertices() {
    let b = regen_doc(DOC);
    let hub = b.face_ids().map(|f| b.face_id_str(f)).find(|n| n.starts_with("H2.S")).unwrap();
    let edges: Vec<_> = b
        .edge_ids()
        .filter(|&e| {
            let fs: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
            fs.iter().any(|x| x == &hub) && fs.iter().any(|x| x.starts_with("F8"))
        })
        .collect();
    assert!(edges.len() >= 3, "expected a loop of root edges, got {}", edges.len());
    let mut ends: Vec<usize> = edges.iter().flat_map(|&e| [b.edge(e).v0.idx(), b.edge(e).v1.idx()]).collect();
    ends.sort_unstable();
    ends.dedup();
    // a ring: every edge end is shared, so there are as many distinct vertices as edges
    assert_eq!(ends.len(), edges.len(), "root loop is not a clean ring");
}

/// A subset of the loop is an OPEN chain: it runs the chain-end logic and is refused (never a trap).
#[test]
fn open_subchain_of_the_root_loop_is_refused_cleanly() {
    let b = regen_doc(DOC);
    let picks: Vec<_> = root_picks(&b).into_iter().take(3).collect();
    let e = ck_ops::blend::fillet(&b, &picks, 1.0, "F9", 1e-6).err().expect("an open sub-chain must not blend");
    eprintln!("open sub-chain: {} {:?}", e.code, e.detail);
    assert_eq!(e.code, "blendUnsupported");
}

/// TARGET of task 1: the whole loop blends as a ring. Ignored because the rolling-ball centre solve
/// still diverges at the shallow blade-root dihedral (`blendFailed "centre did not converge"`, station
/// reported on stderr with CK_BLEND_DEBUG=1); the ring is no longer refused as "closed ring".
#[test]
#[ignore = "task 1: the ring path runs but the ball-centre solve diverges at the blade root"]
fn closed_blunt_root_ring_fillets_cleanly() {
    let b = regen_doc(DOC);
    let picks = root_picks(&b);
    let r = ck_ops::blend::fillet(&b, &picks, 1.0, "F9", 1e-6).unwrap_or_else(|e| panic!("closed ring refused: {} {:?}", e.code, e.detail));
    assert!(ck_topo::check::check(&r, 1e-4).is_empty(), "{:?}", ck_topo::check::check(&r, 1e-4));
    eprintln!("closed ring blend volume {} base {}", ck_topo::mass::mass_props(&r).volume, ck_topo::mass::mass_props(&b).volume);
}
