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
use std::collections::HashMap;
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

/// Hash of a face's surface and trimming (not of its place in a body).
pub(crate) fn face_hash(b: &Body, f: FaceId) -> u64 {
    let face = b.face(f);
    let mut h = 0xcbf29ce484222325u64;
    let mut mix = |x: u64| {
        h = (h ^ x).wrapping_mul(0x100000001b3);
        h ^= h >> 29;
    };
    mix(face.surface.fingerprint());
    mix(face.sense as u64);
    for &lp in &face.loops {
        mix(0xa5a5);
        for &c in &b.lp(lp).coedges {
            let ed = b.edge(b.coedge(c).edge);
            mix(b.coedge(c).rev as u64);
            for t in [ed.t0, 0.5 * (ed.t0 + ed.t1), ed.t1] {
                let p = ed.curve.eval(t);
                mix(p.x.to_bits());
                mix(p.y.to_bits());
                mix(p.z.to_bits());
            }
        }
    }
    h
}

/// Volume and area of a body (as `ck_topo::mass::mass_props`, to the bit: the same integrals added in the same
/// order) and the area of every face, each face's integral remembered by the kernel: the same file body is measured by every
/// verification (the history is verified again for each reading order tried), and the integral over a free-form
/// face (a blend, a modelled thread) is the cost of a measurement.
pub struct Measure {
    pub volume: f64,
    pub area: f64,
    pub areas: HashMap<FaceId, f64>,
}

pub fn measure(b: &Body) -> Measure {
    let r = ck_topo::mass::ref_point_of(b);
    let (mut volume, mut area, mut first) = (0.0, 0.0, true);
    let mut areas = HashMap::new();
    for l in b.lump_ids() {
        let (mut lv, mut la) = (0.0, 0.0);
        for f in ck_topo::mass::lump_faces(b, l) {
            // (the integral is remembered by the kernel: see `ck_topo::mass::faces_moments`)
            let m = ck_topo::mass::faces_moments(b, &[f], r);
            let (v, a) = (m.volume, m.area);
            lv += v;
            la += a;
            areas.insert(f, a);
        }
        if first {
            (volume, area, first) = (lv, la, false);
        } else {
            volume += lv;
            area += la;
        }
    }
    Measure { volume, area, areas }
}

/// The two bodies have the same faces: a bijection by surface and area.
fn same_faces(orig: &Body, new: &Body, ao: &HashMap<FaceId, f64>, an: &HashMap<FaceId, f64>) -> bool {
    if orig.num_faces() != new.num_faces() {
        return false;
    }
    let (mo, mn) = (Model::new(orig), Model::new(new));
    let mut free: Vec<(ck_topo::body::FaceId, f64)> = mn.faces.iter().map(|&f| (f, an.get(&f).copied().unwrap_or_else(|| mn.area(f)))).collect();
    for &f in &mo.faces {
        let a = ao.get(&f).copied().unwrap_or_else(|| mo.area(f));
        let Some(i) = free.iter().position(|&(g, ga)| close(a, ga, 1e-6) && same_surface(mo.kind(f), mn.kind(g), 1e-6)) else {
            return false
        };
        free.swap_remove(i);
    }
    true
}

pub fn verify(orig: &Body, new: &Body) -> Verify {
    let (mo, mn) = (measure(orig), measure(new));
    let (vo, vn, ao, an) = (mo.volume, mn.volume, mo.area, mn.area);
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
    let sf = dv < 1e-6 && area_rel < 1e-6 && same_faces(orig, new, &mo.areas, &mn.areas);
    if sf {
        return v(true, true);
    }
    // the same face count, volume and area to 1e-6, yet a face or two do not pair up by area to 1e-6 (a blend's
    // contact line a few micrometres along its support): the boolean difference of two bodies that coincide on every
    // face does not finish (minutes on a part of 150 faces). The volume between the two boundaries decides instead,
    // as it does for `close` below
    if dv < 1e-6 && area_rel < 1e-6 && orig.num_faces() == new.num_faces() {
        let lim = 0.05 * vo.abs() / (ao + an).max(1e-12);
        if let Some((_, gap)) = shell_deviation(orig, new, lim) {
            if gap / vo.abs().max(1e-12) < 1e-6 && bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len()) {
                return v(true, true);
            }
        }
    }
    // Free-form faces (a loft's lateral surface) never match by surface parameters, and the symmetric difference of
    // two such bodies sends our booleans into near-coincident B-spline work that takes a minute and then fails
    // (leaving `rel` to the volume and area). A body that cannot be exact (volume or area differ by 1e-6 or more)
    // and whose boundary is shown to lie within 1e-3 of the volume of the other's is `close` without them. The
    // boolean still decides the rest: bodies that may be exact, and bodies that are not shown to be close.
    if dv >= 1e-6 || area_rel >= 1e-6 {
        let lim = 0.05 * vo.abs() / (ao + an).max(1e-12);
        let sd = shell_deviation(orig, new, lim);
        if let Some((_, gap)) = sd {
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
    // (also with equal face counts when the bodies cannot be exact anyway, volume or area differing by 1e-6 or more: the boolean difference of such bodies runs a minute or more and then fails, and a
    // failed boolean leaves exactly this verdict; the free-form faces are shown to match in area and, from the faces
    // above, the analytic faces in surface)
    if dv < 1e-5 && area_rel < 1e-5 && (orig.num_faces() != new.num_faces() || dv >= 1e-6 || area_rel >= 1e-6) && same_analytic_faces(orig, new, &mo.areas, &mn.areas) {
        let ok = bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len());
        return v(false, ok);
    }
    // Bodies whose face counts differ and that no shortcut above could place within tolerance: the boolean
    // difference of two nearly coincident free-form boundaries (blend patches) does not finish, and a body missing
    // faces of the import is not the import. Not close (`rel` carries the volume and area error).
    if orig.num_faces() != new.num_faces() {
        return v(false, false);
    }
    let r = ck_recog::rebuild::verify(orig, new);
    r
}

/// As [`verify`] for an import with `split` faces more than the rebuild can have: pieces a split line cut out of a
/// face (the split line is not rebuilt, so the rebuild has the one face where the import has the pieces; `split` is
/// the number of pieces less the number of faces they make up). The faces must otherwise pair up: a face of the
/// rebuild pairs with one face of the import of the same surface and area, or with all the import's unpaired faces on
/// its surface when their areas add up to its own. With volume and area to 1e-6 that is exact; else the boundaries
/// must lie within 1e-3 of the volume of each other (close), as for bodies with free-form faces in [`verify`].
pub fn verify_split(orig: &Body, new: &Body, split: usize) -> Verify {
    if split == 0 {
        return verify(orig, new);
    }
    let (po, pn) = (ck_topo::mass::mass_props(orig), ck_topo::mass::mass_props(new));
    let (vo, vn, ao, an) = (po.volume, pn.volume, po.area, pn.area);
    let dv = (vo - vn).abs() / vo.abs().max(1e-12);
    let area_rel = (ao - an).abs() / ao.abs().max(1e-12);
    let (b0, b1) = (orig.bbox(), new.bbox());
    let bbox_dev = (b0.0 - b1.0).len().max((b0.1 - b1.1).len());
    let rel = dv.max(area_rel);
    let v = |exact: bool, close: bool| Verify { vol_orig: vo, vol_new: vn, sym_diff: None, rel, bbox_dev, area_rel, exact, close };
    if dv > 1e-3 || area_rel > 1e-3 || orig.num_faces() != new.num_faces() + split {
        return v(false, false);
    }
    if dv < 1e-6 && area_rel < 1e-6 && same_faces_split(orig, new) {
        return v(true, true);
    }
    if let Some((_, gap)) = shell_deviation_any(orig, new, 0.05 * vo.abs() / (ao + an).max(1e-12)) {
        if gap / vo.abs().max(1e-12) < 1e-3 {
            let ok = rel < 1e-3 && dv < 1e-3 && bbox_dev < 1e-3 * (1.0 + (b0.1 - b0.0).len());
            return v(false, ok);
        }
    }
    v(false, false)
}

/// The pairing of [`verify_split`].
fn same_faces_split(orig: &Body, new: &Body) -> bool {
    let (mo, mn) = (Model::new(orig), Model::new(new));
    let mut free: Vec<(FaceId, f64)> = mo.faces.iter().map(|&f| (f, mo.area(f))).collect();
    let mut left = vec![];
    for &g in &mn.faces {
        let a = mn.area(g);
        match free.iter().position(|&(f, fa)| close(a, fa, 1e-6) && same_surface(mo.kind(f), mn.kind(g), 1e-6)) {
            Some(i) => {
                free.swap_remove(i);
            }
            None => left.push((g, a)),
        }
    }
    // (the faces of the rebuild left on one surface, together, against the import's pieces on it: several faces of
    // the rebuild can lie on one surface, each the union of some of the pieces)
    let mut left = left;
    while let Some((g, _)) = left.first().copied() {
        let mine: Vec<usize> = (0..left.len()).filter(|&j| same_surface(mn.kind(g), mn.kind(left[j].0), 1e-6)).collect();
        let a: f64 = mine.iter().map(|&j| left[j].1).sum();
        let pieces: Vec<usize> = (0..free.len()).filter(|&i| same_surface(mo.kind(free[i].0), mn.kind(g), 1e-6)).collect();
        let sum: f64 = pieces.iter().map(|&i| free[i].1).sum();
        if pieces.len() < mine.len() + 1 || !close(a, sum, 1e-6) {
            return false;
        }
        for &i in pieces.iter().rev() {
            free.swap_remove(i);
        }
        for &j in mine.iter().rev() {
            left.swap_remove(j);
        }
    }
    free.is_empty()
}

/// Every analytic face of the import pairs with one of the rebuild with the same surface and an area within 5e-4
/// (a rebuilt helix is exact where the file's is a spline fit, so the planes it trims differ a little), and the rest
/// of the rebuild's surface (free-form faces, analytic faces nothing pairs with: a crest that is a true cylinder
/// where the file has a patch, slivers at a thread's ends) has the area of the import's free-form faces to 1e-4 of
/// the whole. The import has free-form faces.
fn same_analytic_faces(orig: &Body, new: &Body, ao: &HashMap<FaceId, f64>, an: &HashMap<FaceId, f64>) -> bool {
    let (mo, mn) = (Model::new(orig), Model::new(new));
    let free_o: f64 = mo.faces.iter().filter(|&&f| matches!(mo.kind(f), Kind::Other(_))).map(|&f| ao.get(&f).copied().unwrap_or_else(|| mo.area(f))).sum();
    if free_o <= 0.0 {
        return false;
    }
    let mut left: Vec<(FaceId, f64)> = mn.faces.iter().map(|&f| (f, an.get(&f).copied().unwrap_or_else(|| mn.area(f)))).collect();
    let mut total = free_o;
    // (a face the rebuild splits along a seam, a torus blend in two halves, pairs with the pieces left over once
    // every face with a whole partner has taken it: same surface, areas adding up to the import's face)
    let mut split: Vec<(&FaceId, f64)> = vec![];
    for f in mo.faces.iter().filter(|&&f| !matches!(mo.kind(f), Kind::Other(_))) {
        let a = ao.get(f).copied().unwrap_or_else(|| mo.area(*f));
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
    if a.num_faces() != b.num_faces() {
        return None;
    }
    shell_deviation_any(a, b, lim)
}

/// Distance from `p` to a whole cylinder, sphere or torus (or plane); `None` for other surfaces.
fn quadric_dist(sf: &ck_geom::surface::Surface, p: Vec3) -> Option<f64> {
    use ck_geom::surface::Surface;
    match sf {
        Surface::Plane { frame } => Some((p - frame.o).dot(frame.z.normalize()).abs()),
        Surface::Cylinder { frame, r } => {
            let d = p - frame.o;
            let a = frame.z.normalize();
            Some(((d - a * d.dot(a)).len() - r).abs())
        }
        Surface::Sphere { frame, r } => Some(((p - frame.o).len() - r).abs()),
        Surface::Torus { frame, major, minor } => {
            let d = p - frame.o;
            let a = frame.z.normalize();
            let h = d.dot(a);
            let q = (d - a * h).len() - major;
            Some(((q * q + h * h).sqrt() - minor).abs())
        }
        _ => None,
    }
}

/// [`shell_deviation`] whatever the face counts.
fn shell_deviation_any(a: &Body, b: &Body, lim: f64) -> Option<(f64, f64)> {
    use std::cell::{Cell, RefCell};
    if !(lim > 0.0) {
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
                    // (the curved quadrics in closed form, whole surfaces as the Newton below measures, and not pruned by
                    // the face's box: the quadrature of a sphere patch can sample its surface outside the patch)
                    let sf = &y.face(*g).surface;
                    if !matches!(sf, ck_geom::surface::Surface::Plane { .. }) {
                        if let Some(d) = quadric_dist(sf, p) {
                            best = best.min(d);
                            continue;
                        }
                    }
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
