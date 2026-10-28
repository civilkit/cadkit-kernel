//! `knurlBand`: a diamond or pyramid knurl built directly as faces, then cut from the body in one boolean.
//!
//! ```json
//! {"type":"knurlBand","id":"F5","axis":"z","center":[0,0],"radius":10,"from":0,"to":12,
//!  "pattern":"diamond","count":24,"depth":0.65,"angle":90,"helixAngle":30}
//! ```
//!
//! * `axis` (`x`, `y`, `z`, default `z`), `center` (the two world coordinates of the axis line: `[x, y]` for z, `[x, z]` for y,
//!   `[y, z]` for x), `radius` (the cylinder being knurled), `from` / `to` (axial range, world coordinates).
//! * `pattern`: `diamond` (two crossed sets of V grooves turning `helixAngle` degrees from the axis, default 30) or
//!   `pyramid` (axial V grooves crossed by ring V grooves; `ringPitch`, default the groove pitch, is rounded to a whole number
//!   of rings over the band). `count` grooves round the circumference (per hand for a diamond), `depth` (default pitch / 4),
//!   `angle` included angle of the V in degrees (default 90).
//! * The knurled surface is a known piecewise linear function of the position on the band (groove depth = the deeper of the
//!   two groove sets), so its facets are generated directly: one lattice cell is a fixed set of triangles, cells are clipped to
//!   the band. The facets plus a closed polygonal envelope above the cylinder form a tool solid whose faces never meet each other,
//!   so the single cut costs a number of face/face tests linear in the facet count (the grooves are never cut one set against the
//!   other). Facets are planar triangles: the land between grooves is cylindrical in the result (the cut leaves the host cylinder
//!   face), the groove flanks are triangles through exact surface points.
//! * Faces of the tool are named `<id>.K<n>`. Only the `v` profile is built; the result is checked (the tool is a closed manifold,
//!   the cut removed volume, the result is not less valid than the body, still one lump per lump).
//! * Errors: `nothingToCut` (no body), `invalidValue {field, detail}`, `knurlBandTooLarge`, `knurlBandInvalid {detail}`,
//!   `booleanFailed` and the other boolean errors, `cutNoEffect` (the band is not on the body).

use super::extrude::Combine;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::Json;
use ck_topo::body::{Body, FaceName, VertexId};
use std::collections::HashMap;

pub const KNURL_BAND_TOO_LARGE: &str = "knurlBandTooLarge";
pub const KNURL_BAND_INVALID: &str = "knurlBandInvalid";
/// Most facets the tool may have.
pub const MAX_FACETS: usize = 60_000;

fn bad(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}
fn invalid(detail: String) -> FeatureError {
    FeatureError::new(KNURL_BAND_INVALID).with("detail", Json::Str(detail))
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Pattern {
    Diamond,
    Pyramid,
}

/// Parsed and checked request.
#[derive(Debug, Clone)]
pub struct Spec {
    pub pattern: Pattern,
    /// Point on the axis at axial coordinate 0, unit axis, unit reference direction (angle 0).
    pub o: Vec3,
    pub d: Vec3,
    pub ex: Vec3,
    pub radius: f64,
    pub from: f64,
    pub len: f64,
    pub count: usize,
    pub depth: f64,
    /// Half width of a groove at the surface (arc length).
    pub half: f64,
    /// Diamond: tan(helix angle). Pyramid: axial ring spacing.
    pub slope: f64,
    pub rings: usize,
}

fn parse(state: &RegenState, f: &Feature) -> Result<Spec, FeatureError> {
    let num = |k: &str, def: f64| -> Result<f64, FeatureError> {
        let v = state.num(f, k, def)?;
        if !v.is_finite() {
            return Err(bad(k, "must be a finite number"));
        }
        Ok(v)
    };
    let axis = f.raw.get_str("axis").unwrap_or("z");
    let (d, ex, ci) = match axis {
        "z" => (Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 0.0), (0usize, 1usize)),
        "y" => (Vec3::new(0.0, 1.0, 0.0), Vec3::new(1.0, 0.0, 0.0), (0, 2)),
        "x" => (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), (1, 2)),
        _ => return Err(bad("axis", "x, y or z")),
    };
    let mut c = [0.0f64; 2];
    if let Some(Json::Arr(a)) = f.raw.get("center") {
        if a.len() != 2 {
            return Err(bad("center", "two coordinates"));
        }
        for k in 0..2 {
            c[k] = crate::expr::eval_field(Some(&a[k]), &state.vars, 0.0).map_err(FeatureError::new)?;
        }
    }
    let mut o = [0.0f64; 3];
    o[ci.0] = c[0];
    o[ci.1] = c[1];
    let o = Vec3::new(o[0], o[1], o[2]);
    let radius = num("radius", 0.0)?;
    let (from, to) = (num("from", 0.0)?, num("to", 0.0)?);
    if !(radius > 0.0) {
        return Err(bad("radius", "must be positive"));
    }
    if !(to > from) {
        return Err(bad("to", "must be greater than from"));
    }
    let pattern = match f.raw.get_str("pattern").unwrap_or("diamond") {
        "diamond" => Pattern::Diamond,
        "pyramid" => Pattern::Pyramid,
        _ => return Err(bad("pattern", "diamond or pyramid (straight grooves are one extrude cut)")),
    };
    if let Some(p) = f.raw.get_str("profile") {
        if p != "v" {
            return Err(bad("profile", "only the v profile is built"));
        }
    }
    let circ = std::f64::consts::TAU * radius;
    let count = match (f.raw.get("count"), f.raw.get("pitch")) {
        (Some(_), _) => num("count", 0.0)?.round(),
        (None, Some(_)) => (circ / num("pitch", 0.0)?).round(),
        _ => return Err(bad("count", "give count or pitch")),
    };
    if !(count >= 6.0 && count <= 2000.0) {
        return Err(bad("count", "between 6 and 2000 grooves round the circumference"));
    }
    let count = count as usize;
    let pitch = circ / count as f64;
    let depth = num("depth", pitch / 4.0)?;
    if !(depth > 0.0) || depth >= radius {
        return Err(bad("depth", "positive and smaller than the radius"));
    }
    let angle = num("angle", 90.0)?.to_radians();
    if !(angle > 0.0 && angle < std::f64::consts::PI) {
        return Err(bad("angle", "between 0 and 180 degrees"));
    }
    let half = depth * (angle / 2.0).tan();
    let len = to - from;
    // grooves must leave a land between them (the helper's rule: 2 half <= 0.9 of the spacing)
    if 2.0 * half > 0.9 * pitch {
        let max_n = (0.9 * circ / (2.0 * half)).floor();
        return Err(bad("count", &format!("{count} grooves of width {:.3} overlap; use at most {max_n} or a smaller depth", 2.0 * half)));
    }
    let (slope, rings) = match pattern {
        Pattern::Diamond => {
            let ha = num("helixAngle", 30.0)?;
            if !(ha > 0.0 && ha < 80.0) {
                return Err(bad("helixAngle", "between 0 and 80 degrees"));
            }
            (ha.to_radians().tan(), 0)
        }
        Pattern::Pyramid => {
            let rp = num("ringPitch", pitch)?;
            if !(rp > 2.0 * half / 0.9) {
                return Err(bad("ringPitch", "too small for the groove width"));
            }
            let m = ((len / rp).floor() as usize).max(1);
            (len / m as f64, m)
        }
    };
    Ok(Spec { pattern, o, d, ex, radius, from, len, count, depth, half, slope, rings })
}

/// Welds points by position (a cell hash, as the importers do).
struct Weld {
    tol: f64,
    cells: HashMap<(i64, i64, i64), Vec<u32>>,
    pts: Vec<Vec3>,
}

impl Weld {
    fn new(tol: f64) -> Weld {
        Weld { tol, cells: HashMap::new(), pts: vec![] }
    }
    fn key(&self, p: Vec3) -> (i64, i64, i64) {
        ((p.x / self.tol).floor() as i64, (p.y / self.tol).floor() as i64, (p.z / self.tol).floor() as i64)
    }
    fn add(&mut self, p: Vec3) -> u32 {
        let k = self.key(p);
        for dx in -1..=1i64 {
            for dy in -1..=1i64 {
                for dz in -1..=1i64 {
                    if let Some(v) = self.cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
                        for &i in v {
                            if self.pts[i as usize].dist2(p) <= self.tol * self.tol {
                                return i;
                            }
                        }
                    }
                }
            }
        }
        let i = self.pts.len() as u32;
        self.pts.push(p);
        self.cells.entry(k).or_default().push(i);
        i
    }
}

/// Clip a convex planar polygon to the slab `0 <= axial <= len` in 3D (the new points lie on the polygon's edges, so a planar
/// polygon stays planar).
fn clip_slab(poly: &[Vec3], o: Vec3, d: Vec3, from: f64, len: f64) -> Vec<Vec3> {
    let mut cur: Vec<Vec3> = poly.to_vec();
    for (bound, keep_above) in [(0.0, true), (len, false)] {
        if cur.is_empty() {
            break;
        }
        let ax = |p: Vec3| (p - o).dot(d) - from - bound;
        let mut out: Vec<Vec3> = Vec::with_capacity(cur.len() + 2);
        for i in 0..cur.len() {
            let (a, b) = (cur[i], cur[(i + 1) % cur.len()]);
            let (fa, fb) = (ax(a), ax(b));
            let (ia, ib) = (if keep_above { fa >= 0.0 } else { fa <= 0.0 }, if keep_above { fb >= 0.0 } else { fb <= 0.0 });
            if ia {
                out.push(a);
            }
            if ia != ib {
                let t = fa / (fa - fb);
                out.push(a + (b - a) * t);
            }
        }
        cur = out;
    }
    cur
}

/// A polygon of the tool: rings of weld indices (outer first), plane normal.
struct Poly {
    rings: Vec<Vec<u32>>,
    n: Vec3,
}

fn newell(pts: &[Vec3]) -> Vec3 {
    let mut a = Vec3::new(0.0, 0.0, 0.0);
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        a = a + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
    }
    a
}

/// The facets of the knurled surface, clipped to the band. Per lattice cell (the groove crossing at the origin, cell edges
/// along the two groove sets): the four flanks (two of each set; each one runs from this crossing to the next one, with the
/// groove bottom as an edge) and the land between four crossings. Each is one polygon, planar for a pyramid and nearly planar
/// for a diamond (the caller splits it into triangles then).
fn lattice(s: &Spec, half: f64, kx: usize, ky: usize, place: &dyn Fn(f64, f64, f64) -> Vec3) -> Vec<(Vec<Vec3>, Option<Vec3>)> {
    let n = s.count as f64;
    let p = std::f64::consts::TAU * s.radius / n;
    let d = s.depth;
    let (q1, q2, to_uz, cells): (f64, f64, Box<dyn Fn(f64, f64) -> (f64, f64)>, Vec<(f64, f64)>) = match s.pattern {
        Pattern::Diamond => {
            let m = s.slope;
            let mut cells = vec![];
            let a_hi = (2.0 * m * s.len / p).ceil() as i64 + 2;
            for a in -2..=a_hi {
                // cells (i, j) with j - i = a and i + j = b in [0, 2 count)
                let mut b = if a.rem_euclid(2) == 0 { 0 } else { 1 };
                while b < 2 * s.count as i64 {
                    cells.push((((b - a) / 2) as f64, ((b + a) / 2) as f64));
                    b += 2;
                }
            }
            (half / p, half / p, Box::new(move |x1, x2| (p * (x1 + x2) / 2.0, p * (x2 - x1) / (2.0 * m))), cells)
        }
        Pattern::Pyramid => {
            let step = s.slope;
            let mut cells = vec![];
            for i in 0..s.count {
                for k in -1..=(s.rings as i64) {
                    cells.push((i as f64, k as f64));
                }
            }
            (half / p, half / step, Box::new(move |x1, x2| (p * x1, step * (x2 + 0.5))), cells)
        }
    };
    // removed depth at a lattice offset (a, b) from a crossing: the deeper of the two grooves
    let hf = |a: f64, b: f64| -> f64 { (d * (1.0 - a.abs() / q1)).max(d * (1.0 - b.abs() / q2)).max(0.0) };
    // The five polygons of a cell, as offsets from the crossing, each flank cut into strips so that the chords of the groove bottoms
    // stay short (kx strips along the first groove set's direction, ky along the second): strip j of a flank runs between the bottom
    // nodes j / k and (j + 1) / k and the matching nodes on the land edge, so neighbours (and the land, whose edges carry the same
    // nodes) meet without T-junctions.
    let sx = |a: f64| q1 + (1.0 - 2.0 * q1) * a;
    let ty = |a: f64| q2 + (1.0 - 2.0 * q2) * a;
    let mut shapes: Vec<Vec<(f64, f64)>> = vec![];
    for j in 0..ky {
        let (a0, a1) = (j as f64 / ky as f64, (j + 1) as f64 / ky as f64);
        shapes.push(vec![(0.0, a0), (q1, ty(a0)), (q1, ty(a1)), (0.0, a1)]);
        shapes.push(vec![(0.0, a0), (0.0, a1), (-q1, ty(a1)), (-q1, ty(a0))]);
    }
    for j in 0..kx {
        let (a0, a1) = (j as f64 / kx as f64, (j + 1) as f64 / kx as f64);
        shapes.push(vec![(a0, 0.0), (a1, 0.0), (sx(a1), q2), (sx(a0), q2)]);
        shapes.push(vec![(a0, 0.0), (sx(a0), -q2), (sx(a1), -q2), (a1, 0.0)]);
    }
    if kx == 1 && ky == 1 {
        shapes.push(vec![(q1, q2), (1.0 - q1, q2), (1.0 - q1, 1.0 - q2), (q1, 1.0 - q2)]);
    } else {
        // the land, a fan round its middle over the boundary nodes
        let mut ring: Vec<(f64, f64)> = vec![];
        for i in 0..kx {
            ring.push((sx(i as f64 / kx as f64), q2));
        }
        for j in 0..ky {
            ring.push((1.0 - q1, ty(j as f64 / ky as f64)));
        }
        for i in (1..=kx).rev() {
            ring.push((sx(i as f64 / kx as f64), 1.0 - q2));
        }
        for j in (1..=ky).rev() {
            ring.push((q1, ty(j as f64 / ky as f64)));
        }
        for k in 0..ring.len() {
            shapes.push(vec![(0.5, 0.5), ring[k], ring[(k + 1) % ring.len()]]);
        }
    }
    let (o, ax) = (s.o, s.d);
    let mut out: Vec<(Vec<Vec3>, Option<Vec3>)> = vec![];
    for &(i, j) in &cells {
        for sh in &shapes {
            let uzh: Vec<(f64, f64, f64)> = sh
                .iter()
                .map(|&(a, b)| {
                    let (u, z) = to_uz(i + a, j + b);
                    (u, z, hf(a, b))
                })
                .collect();
            let poly: Vec<Vec3> = uzh.iter().map(|&(u, z, h)| place(u, z, h)).collect();
            // the point of the ideal (planar in u, z, h) facet at the middle of a quad: the diagonal of a warped quad is chosen by it
            let k = uzh.len() as f64;
            let centre = place(uzh.iter().map(|v| v.0).sum::<f64>() / k, uzh.iter().map(|v| v.1).sum::<f64>() / k, uzh.iter().map(|v| v.2).sum::<f64>() / k);
            let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
            for v in &poly {
                let z = (*v - o).dot(ax) - s.from;
                zmin = zmin.min(z);
                zmax = zmax.max(z);
            }
            let eps = 1e-9 * (1.0 + s.len);
            if zmax <= eps || zmin >= s.len - eps {
                continue;
            }
            if zmin >= -eps && zmax <= s.len + eps {
                out.push((poly, Some(centre)));
                continue;
            }
            let c = clip_slab(&poly, o, ax, s.from, s.len);
            if c.len() >= 3 {
                out.push((c, None));
            }
        }
    }
    out
}

/// The tool solid: facets, a closed polygonal envelope above the cylinder, the two end walls.
pub fn tool(id: &str, s: &Spec) -> Result<Body, FeatureError> {
    let r = s.radius;
    let p = std::f64::consts::TAU * r / s.count as f64;
    // Groove bottoms between two crossings are chords, deeper in the middle than the ideal circle by the sagitta. The crossings are
    // raised by a part of it (2/3 for a diamond, 1/2 for a pyramid, which has straight bottoms too), which balances the straight and chord bottoms (the removed volume then matches the
    // ideal surface to about a percent).
    let node_gap = if s.pattern == Pattern::Diamond { p / 2.0 } else { p };
    let sag_of = |gap: f64| (r - s.depth) * (1.0 - (gap / (2.0 * r)).cos());
    // strips per bottom chord so that the sagitta stays within a sixth of the depth (a diamond cuts both sets along chords, a
    // pyramid only the ring set; the axial grooves are straight and need none)
    let strips = ((sag_of(node_gap) / s.depth / 0.15).sqrt().ceil() as usize).clamp(1, 8);
    let (kx, ky) = if s.pattern == Pattern::Diamond { (strips, strips) } else { (strips, 1) };
    let sag = sag_of(node_gap / strips as f64);
    // a diamond has chords only (the mean sag of a chord is 2/3 of its sagitta); a pyramid has one straight and one chord set
    let eps = sag * if s.pattern == Pattern::Diamond { 2.0 / 3.0 } else { 0.5 };
    let dd = s.depth - eps;
    // Land facets are chords too: the land is lifted above the cylinder so they never dip into the material. The flank then
    // starts higher than the cylinder, so the lattice half width grows by the same ratio and the flank crosses the cylinder
    // exactly where the ideal groove edge is.
    let delta0 = 0.01 * s.depth;
    let mut lift = delta0;
    let mut half = s.half;
    for _ in 0..6 {
        half = s.half * (1.0 + lift / dd);
        // the land is one quad, or a fan round its middle when the flanks are cut into strips (half the reach)
        let span = (p - 2.0 * half) / r / if strips == 1 { 1.0 } else { 2.0 };
        lift = (r + delta0) / (span.max(0.0) / 2.0).min(1.2).cos() - r;
    }
    half = s.half * (1.0 + lift / dd);
    if 2.0 * half > 0.95 * p {
        return Err(invalid(format!("{} grooves of this width leave lands too wide to build as facets (the lattice would need {:.3} mm of groove at a pitch of {:.3}); use more grooves, a larger depth or the boolean method", s.count, 2.0 * half, p)));
    }
    if s.pattern == Pattern::Pyramid && 2.0 * half > 0.95 * s.slope {
        return Err(invalid(format!("the rings are too close for facets of this groove width ({:.3} mm of groove at a ring pitch of {:.3}); use a larger ringPitch or the boolean method", 2.0 * half, s.slope)));
    }
    let ey = s.d.cross(s.ex);
    let rad_of = |h: f64| r + lift - h * (dd + lift) / s.depth;
    let place = |u: f64, z: f64, h: f64| -> Vec3 {
        let th = u / r;
        s.o + s.d * (s.from + z) + (s.ex * th.cos() + ey * th.sin()) * rad_of(h)
    };
    let polys = lattice(s, half, kx, ky, &place);
    if polys.len() * 2 > MAX_FACETS {
        return Err(FeatureError::new(KNURL_BAND_TOO_LARGE).with("facets", Json::Num((polys.len() * 2) as f64)).with("max", Json::Num(MAX_FACETS as f64)));
    }
    let mut weld = Weld::new(1e-7 * (1.0 + r));
    let mut faces: Vec<Poly> = vec![];
    let plane_tol = 1e-9 * (1.0 + r);
    for (poly, centre) in &polys {
        let mut ids: Vec<u32> = Vec::with_capacity(poly.len());
        for v in poly {
            let id = weld.add(*v);
            if ids.last() != Some(&id) {
                ids.push(id);
            }
        }
        while ids.len() > 1 && ids[0] == ids[ids.len() - 1] {
            ids.pop();
        }
        if ids.len() < 3 {
            continue;
        }
        let mut pts: Vec<Vec3> = ids.iter().map(|&i| weld.pts[i as usize]).collect();
        let nw = newell(&pts);
        if nw.len() < 1e-12 {
            continue;
        }
        let nn = nw.normalize();
        // one planar polygon, else a fan of triangles
        if ids.len() == 3 || pts.iter().all(|q| (*q - pts[0]).dot(nn).abs() <= plane_tol) {
            faces.push(Poly { rings: vec![ids], n: nn });
            continue;
        }
        // a warped quad: the diagonal whose middle is nearer the ideal middle of the facet
        if ids.len() == 4 {
            if let Some(c) = centre {
                let m02 = (pts[0] + pts[2]) * 0.5;
                let m13 = (pts[1] + pts[3]) * 0.5;
                if m13.dist2(*c) > m02.dist2(*c) {
                    ids.rotate_left(1);
                    pts.rotate_left(1);
                }
            }
        }
        for k in 1..ids.len() - 1 {
            let t = [ids[0], ids[k], ids[k + 1]];
            let tp = [pts[0], pts[k], pts[k + 1]];
            let nrm = (tp[1] - tp[0]).cross(tp[2] - tp[0]);
            if nrm.len() < 1e-12 {
                continue;
            }
            faces.push(Poly { rings: vec![t.to_vec()], n: nrm.normalize() });
        }
    }
    let surface_faces = faces.len();
    // boundary edges of the surface: used once
    let mut uses: HashMap<(u32, u32), i32> = HashMap::new();
    for f in &faces {
        let rg = &f.rings[0];
        for k in 0..rg.len() {
            let (a, b) = (rg[k], rg[(k + 1) % rg.len()]);
            *uses.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
        }
    }
    if let Some((e, c)) = uses.iter().find(|(_, c)| **c > 2 || **c < 1) {
        return Err(invalid(format!("facet edge {e:?} used {c} times")));
    }
    let axial = |v: u32| (weld.pts[v as usize] - s.o).dot(s.d) - s.from;
    let eps = 1e-6 * (1.0 + s.len);
    let mut next: [HashMap<u32, Vec<u32>>; 2] = [HashMap::new(), HashMap::new()];
    for f in &faces {
        let rg = &f.rings[0];
        for k in 0..rg.len() {
            let (a, b) = (rg[k], rg[(k + 1) % rg.len()]);
            let key = if a < b { (a, b) } else { (b, a) };
            if uses[&key] == 1 {
                let (za, zb) = (axial(a), axial(b));
                let side = if za.abs() < eps && zb.abs() < eps {
                    0
                } else if (za - s.len).abs() < eps && (zb - s.len).abs() < eps {
                    1
                } else {
                    return Err(invalid(format!("an open facet edge away from the band ends (axial {za:.4}, {zb:.4})")));
                };
                next[side].entry(a).or_default().push(b);
            }
        }
    }
    // envelope: a polygon round the axis, apothem above the lifted lands
    let e = (0.25 * s.depth).max(0.1);
    let apothem = r + lift + e;
    let kk = 24usize.max(s.count.min(96));
    let circ_r = apothem / (std::f64::consts::PI / kk as f64).cos();
    let mut env: [Vec<u32>; 2] = [vec![], vec![]];
    for (side, z) in [0.0, s.len].into_iter().enumerate() {
        for k in 0..kk {
            let th = std::f64::consts::TAU * k as f64 / kk as f64;
            env[side].push(weld.add(s.o + s.d * (s.from + z) + (s.ex * th.cos() + ey * th.sin()) * circ_r));
        }
    }
    for k in 0..kk {
        let k2 = (k + 1) % kk;
        let q = [env[0][k], env[0][k2], env[1][k2], env[1][k]];
        let pts: Vec<Vec3> = q.iter().map(|&i| weld.pts[i as usize]).collect();
        faces.push(Poly { rings: vec![q.to_vec()], n: newell(&pts).normalize() });
    }
    // end walls: annulus between the envelope and the boundary loop of the surface
    for side in 0..2 {
        let nm = &next[side];
        let Some(&start) = nm.keys().min() else { return Err(invalid("the band has no facets at an end".into())) };
        let mut lp = vec![start];
        let mut cur = start;
        loop {
            let outs = &nm[&cur];
            if outs.len() != 1 {
                return Err(invalid("the end boundary of the facets branches".into()));
            }
            cur = outs[0];
            if cur == start {
                break;
            }
            lp.push(cur);
            if lp.len() > nm.len() + 1 {
                return Err(invalid("the end boundary of the facets does not close".into()));
            }
        }
        if lp.len() != nm.len() {
            return Err(invalid("the end boundary of the facets is in several pieces".into()));
        }
        let n = if side == 0 { s.d * -1.0 } else { s.d };
        let area = |rg: &[u32]| -> f64 { newell(&rg.iter().map(|&i| weld.pts[i as usize]).collect::<Vec<_>>()).dot(n) };
        let mut outer = env[side].clone();
        if area(&outer) < 0.0 {
            outer.reverse();
        }
        if area(&lp) > 0.0 {
            lp.reverse();
        }
        faces.push(Poly { rings: vec![outer, lp], n });
    }
    // topology
    let mut body = Body::new();
    let lump = body.add_lump();
    let shell = body.add_shell(lump);
    let mut verts: HashMap<u32, VertexId> = HashMap::new();
    let mut edges: HashMap<(u32, u32), ck_topo::body::EdgeId> = HashMap::new();
    for (fi, f) in faces.iter().enumerate() {
        let o = weld.pts[f.rings[0][0] as usize];
        let face = body.add_face(shell, Surface::plane(o, f.n), true, FaceName::other(id, &format!("K{fi}")));
        for ring in &f.rings {
            let lp = body.add_loop(face);
            for k in 0..ring.len() {
                let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                let (lo, hi, rev) = if a < b { (a, b, false) } else { (b, a, true) };
                let eid = match edges.get(&(lo, hi)) {
                    Some(&e) => e,
                    None => {
                        let mut vid = |i: u32, body: &mut Body| -> VertexId { *verts.entry(i).or_insert_with(|| body.add_vertex(weld.pts[i as usize])) };
                        let (va, vb) = (vid(lo, &mut body), vid(hi, &mut body));
                        let (pa, pb) = (weld.pts[lo as usize], weld.pts[hi as usize]);
                        let e = body.add_edge(Curve::line_through(pa, pb), 0.0, pa.dist(pb), va, vb);
                        edges.insert((lo, hi), e);
                        e
                    }
                };
                body.add_coedge(lp, eid, rev, None);
            }
        }
    }
    let _ = surface_faces;
    let (_, vol) = ck_topo::orient::orient_body_volume(&mut body);
    if !(vol > 0.0) {
        return Err(invalid(format!("the tool has volume {vol}")));
    }
    if ck_topo::check::check_topology(&body, 1e-6).iter().any(|i| !matches!(i, ck_topo::check::Issue::VertexOffCurve(..))) {
        return Err(invalid("the facets do not close into a manifold solid".into()));
    }
    Ok(body)
}

pub fn exec_knurl_band(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(before) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let spec = parse(state, f)?;
    let tool = tool(&f.id, &spec)?;
    br.combine(&tool, Combine::Cut, true)?;
    // (no volume integral of the result: with thousands of trimmed cylinder faces it costs about three times the boolean; the
    // boolean's own sanity check has judged the volume already)
    let fail = |br: &mut Brep, detail: &str| {
        br.body = Some(before.clone());
        Err(invalid(detail.to_string()))
    };
    let after = br.body.as_ref().unwrap();
    if ck_check::validate(after, 1e-6).len() > ck_check::validate(&before, 1e-6).len() {
        return fail(br, "the cut produced an invalid solid");
    }
    if after.num_lumps() != before.num_lumps() {
        return fail(br, "the cut split the body");
    }
    Ok(())
}
