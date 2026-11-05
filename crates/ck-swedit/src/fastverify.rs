//! Verification that avoids the boolean difference where the answer is already known.
//!
//! `ck_recog::rebuild::verify` measures the symmetric difference with our booleans. Two bodies whose surfaces agree to
//! 1e-8 but not bit for bit (an imported torus whose centre carries a few 1e-6 of noise, rebuilt from a sketch) can
//! send that boolean into near-coincident-surface work that does not finish. Two cheap cases are decided without it:
//!
//! * volume or area differ by more than 1e-3: the rebuild is wrong, and the difference volume would only say by how
//!   much;
//! * every face of the import has a face of the rebuild with the same surface (kind, parameters) and area, and
//!   volume and area agree to 1e-6: the same body (reported with no symmetric difference volume).
//!
//! Anything else goes to the boolean verification.

use ck_geom::Vec3;
use ck_recog::model::{Kind, Model};
use ck_recog::rebuild::Verify;
use ck_topo::body::{Body, FaceId};

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * (1.0 + a.abs().max(b.abs()))
}

fn same_axis(o1: Vec3, a1: Vec3, o2: Vec3, a2: Vec3, tol: f64) -> bool {
    a1.cross(a2).len() < 1e-7 && (o2 - o1).cross(a1).len() < tol
}

pub fn same_surface(x: &Kind, y: &Kind, tol: f64) -> bool {
    match (x, y) {
        (Kind::Plane { n: n1, d: d1 }, Kind::Plane { n: n2, d: d2 }) => (*n1 - *n2).len() < 1e-7 && close(*d1, *d2, tol),
        (Kind::Cyl { o: o1, a: a1, r: r1, .. }, Kind::Cyl { o: o2, a: a2, r: r2, .. }) => same_axis(*o1, *a1, *o2, *a2, tol * 10.0) && close(*r1, *r2, tol),
        (Kind::Cone { o: o1, a: a1, r0: r1, half: h1, .. }, Kind::Cone { o: o2, a: a2, r0: r2, half: h2, .. }) => {
            // the frame origin may sit anywhere on the axis: compare the apex instead
            let apex = |o: Vec3, a: Vec3, r: f64, h: f64| o + a * (-r / h.tan());
            same_axis(*o1, *a1, *o2, *a2, tol * 10.0) && close(h1.abs(), h2.abs(), 1e-7) && (apex(*o1, *a1, *r1, *h1) - apex(*o2, *a2, *r2, *h2)).len() < tol * 10.0 * (1.0 + r1.abs())
        }
        (Kind::Sphere { c: c1, r: r1, .. }, Kind::Sphere { c: c2, r: r2, .. }) => (*c1 - *c2).len() < tol * 10.0 && close(*r1, *r2, tol),
        (Kind::Torus { c: c1, a: a1, major: m1, minor: n1, .. }, Kind::Torus { c: c2, a: a2, major: m2, minor: n2, .. }) => {
            (*c1 - *c2).len() < tol * 10.0 && a1.cross(*a2).len() < 1e-7 && close(*m1, *m2, tol) && close(*n1, *n2, tol)
        }
        _ => false,
    }
}

/// The two bodies have the same faces: a bijection by surface and area.
fn same_faces(orig: &Body, new: &Body) -> bool {
    if orig.num_faces() != new.num_faces() {
        return false;
    }
    let (mo, mn) = (Model::new(orig), Model::new(new));
    let mut free: Vec<(ck_topo::body::FaceId, f64)> = mn.faces.iter().map(|&f| (f, mn.area(f))).collect();
    for &f in &mo.faces {
        let a = mo.area(f);
        let Some(i) = free.iter().position(|&(g, ga)| close(a, ga, 1e-6) && same_surface(mo.kind(f), mn.kind(g), 1e-6)) else { return false };
        free.swap_remove(i);
    }
    true
}

pub fn verify(orig: &Body, new: &Body) -> Verify {
    let (po, pn) = (ck_topo::mass::mass_props(orig), ck_topo::mass::mass_props(new));
    let (vo, vn, ao, an) = (po.volume, pn.volume, po.area, pn.area);
    let dv = (vo - vn).abs() / vo.abs().max(1e-12);
    let area_rel = (ao - an).abs() / ao.abs().max(1e-12);
    let (b0, b1) = (orig.bbox(), new.bbox());
    let bbox_dev = (b0.0 - b1.0).len().max((b0.1 - b1.1).len());
    let rel = dv.max(area_rel);
    let v = |exact: bool, close: bool| Verify { vol_orig: vo, vol_new: vn, sym_diff: None, rel, bbox_dev, area_rel, exact, close };
    if dv > 1e-3 || area_rel > 1e-3 {
        return v(false, false);
    }
    // (the bounding boxes are not compared here: they come from vertices, and a face with no edges, such as a whole
    // sphere, has none; the faces' positions are compared instead)
    if dv < 1e-6 && area_rel < 1e-6 && same_faces(orig, new) {
        return v(true, true);
    }
    // Free-form faces (a loft's lateral surface) never match by surface parameters, and the symmetric difference of
    // two such bodies sends our booleans into near-coincident B-spline work that takes a minute and then fails
    // (leaving `rel` to the volume and area). A body that cannot be exact (volume or area differ by 1e-6 or more)
    // and whose boundary is shown to lie within 1e-3 of the volume of the other's is `close` without them. The
    // boolean still decides the rest: bodies that may be exact, and bodies that are not shown to be close.
    if dv >= 1e-6 || area_rel >= 1e-6 {
        if let Some((_, gap)) = shell_deviation(orig, new, 0.05 * vo.abs() / (ao + an).max(1e-12)) {
            if gap / vo.abs().max(1e-12) < 1e-3 {
                let ok = rel < 1e-3 && dv < 1e-3 && bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len());
                return v(false, ok);
            }
        }
    }
    // Free-form faces split differently (a modelled thread's helical faces: one patch per profile side in the file,
    // pieces here where other faces cross them): the face counts differ, and the boolean difference of two bodies
    // that coincide on long free-form faces does not finish in a reasonable time. Volume and area agree to 1e-5 and
    // the faces pair up (below): close, not exact.
    if orig.num_faces() != new.num_faces() && dv < 1e-5 && area_rel < 1e-5 && same_analytic_faces(orig, new) {
        let ok = bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len());
        return v(false, ok);
    }
    // Bodies whose face counts differ and that no shortcut above could place within tolerance: the boolean
    // difference of two nearly coincident free-form boundaries (blend patches) does not finish, and a body missing
    // faces of the import is not the import. Not close (`rel` carries the volume and area error).
    if orig.num_faces() != new.num_faces() {
        return v(false, false);
    }
    ck_recog::rebuild::verify(orig, new)
}

/// Every analytic face of the import pairs with one of the rebuild with the same surface and an area within 5e-4
/// (a rebuilt helix is exact where the file's is a spline fit, so the planes it trims differ a little), and the rest
/// of the rebuild's surface (free-form faces, analytic faces nothing pairs with: a crest that is a true cylinder
/// where the file has a patch, slivers at a thread's ends) has the area of the import's free-form faces to 1e-4 of
/// the whole. The import has free-form faces.
fn same_analytic_faces(orig: &Body, new: &Body) -> bool {
    let (mo, mn) = (Model::new(orig), Model::new(new));
    let free_o: f64 = mo.faces.iter().filter(|&&f| matches!(mo.kind(f), Kind::Other(_))).map(|&f| mo.area(f)).sum();
    if free_o <= 0.0 {
        return false;
    }
    let mut left: Vec<(FaceId, f64)> = mn.faces.iter().map(|&f| (f, mn.area(f))).collect();
    let mut total = free_o;
    // (a face the rebuild splits along a seam, a torus blend in two halves, pairs with the pieces left over once
    // every face with a whole partner has taken it: same surface, areas adding up to the import's face)
    let mut split: Vec<(&FaceId, f64)> = vec![];
    for f in mo.faces.iter().filter(|&&f| !matches!(mo.kind(f), Kind::Other(_))) {
        let a = mo.area(*f);
        total += a;
        match left.iter().position(|&(g, ga)| !matches!(mn.kind(g), Kind::Other(_)) && (a - ga).abs() <= 5e-4 * a.abs() + 1e-9 && same_surface(mo.kind(*f), mn.kind(g), 1e-6)) {
            Some(i) => {
                left.swap_remove(i);
            }
            None => split.push((f, a)),
        }
    }
    for (f, a) in split {
        let pieces: Vec<usize> = (0..left.len()).filter(|&i| !matches!(mn.kind(left[i].0), Kind::Other(_)) && same_surface(mo.kind(*f), mn.kind(left[i].0), 1e-6)).collect();
        let sum: f64 = pieces.iter().map(|&i| left[i].1).sum();
        if pieces.len() < 2 || (a - sum).abs() > 5e-4 * a.abs() + 1e-9 {
            return false;
        }
        for &i in pieces.iter().rev() {
            left.swap_remove(i);
        }
    }
    let rest: f64 = left.iter().map(|l| l.1).sum();
    (rest - free_o).abs() <= 1e-4 * total
}

/// One in this many quadrature points of a face is measured by [`shell_deviation`].
const STRIDE: usize = 16;

/// How far apart the boundaries of two bodies with the same number of faces are: `(largest distance, integral of the
/// distance)` over sample points of every face of both (the trimmed face by quadrature, and points along its edges)
/// to the nearest surface of the other body. `None` when the face counts differ or a point is farther than `lim`.
///
/// The integral is the volume between the two boundaries to first order (the gap between two surfaces that close
/// is the distance along their normals), so it stands for the symmetric difference without computing it: the
/// booleans of two bodies that nearly coincide on free-form faces are slow and may fail. The distances are those to
/// the whole (untrimmed) surfaces, found by Newton from the previous point's foot, so they are upper bounds.
fn shell_deviation(a: &Body, b: &Body, lim: f64) -> Option<(f64, f64)> {
    use std::cell::{Cell, RefCell};
    if a.num_faces() != b.num_faces() || !(lim > 0.0) {
        return None;
    }
    let (mut worst, mut gap) = (0.0f64, 0.0f64);
    for (x, y) in [(a, b), (b, a)] {
        let ys: Vec<(FaceId, (Vec3, Vec3))> = y.face_ids().map(|g| (g, y.bbox_of_faces(&[g]))).collect();
        let hints: RefCell<Vec<Option<(f64, f64)>>> = RefCell::new(vec![None; ys.len()]);
        let dist = |p: Vec3| -> f64 {
            // Newton from the previous foot on each face; a point that is not close to the other boundary this way
            // (a stale foot may have run to a far local minimum) gets the exact closest point on every face
            for exact in [false, true] {
                let mut best = f64::INFINITY;
                for (i, (g, (lo, hi))) in ys.iter().enumerate() {
                    let out = Vec3::new((lo.x - p.x).max(p.x - hi.x).max(0.0), (lo.y - p.y).max(p.y - hi.y).max(0.0), (lo.z - p.z).max(p.z - hi.z).max(0.0));
                    if out.len() - lim >= best {
                        continue;
                    }
                    let h = if exact { None } else { hints.borrow()[i] };
                    let (d, uv) = y.face(*g).surface.dist_hinted_fp(p, h, None, f64::INFINITY);
                    hints.borrow_mut()[i] = Some(uv);
                    best = best.min(d);
                }
                if best <= 0.1 * lim || exact {
                    return best;
                }
            }
            unreachable!()
        };
        let (far, w) = (Cell::new(0.0f64), Cell::new(0.0f64));
        let (np, last) = (Cell::new(0usize), Cell::new(0.0f64));
        for f in x.face_ids() {
            np.set(0);
            gap += ck_topo::mass::face_integral::<1>(x, f, 1, &|p, n| {
                // (the quadrature walks the face in order: the distance is smooth, so one in STRIDE points is measured)
                np.set(np.get() + 1);
                if np.get() % STRIDE == 1 {
                    let d = dist(p);
                    last.set(d);
                    far.set(far.get().max(d));
                }
                let d = last.get();
                [d * n.len()]
            })[0];
            if far.get() > lim {
                return None;
            }
        }
        for e in x.edge_ids() {
            let ed = x.edge(e);
            for k in 0..=8 {
                w.set(w.get().max(dist(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 8.0))));
            }
        }
        worst = worst.max(far.get()).max(w.get());
        if worst > lim {
            return None;
        }
    }
    Some((worst, gap))
}
