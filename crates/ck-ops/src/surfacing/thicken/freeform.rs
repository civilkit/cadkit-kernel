//! Thicken of sheets with free-form (B-spline and other non-quadric) faces.
//!
//! Both layers of the wall are faces: the sheet itself (or its offset) and its offset along the
//! natural normal, fitted by [`crate::offset::offset_face_surface`] (sampled normals, fit error
//! controlled, a vanishing area element is `offsetTooLarge` with the radius and the thickness
//! that would work). The rim edges of the offset layer are the sheet's edges moved along the
//! surface normal (adaptive samples, refined until the cubic fit follows the exact offset), and
//! each rim edge gets a ruled wall between its two images, fitted through the same samples.
//! Edges shared by two faces are offset the same way when the faces meet tangentially (the usual
//! split of one loft into spans); a kink between free-form faces is `offsetUnsupported`.

use super::{sheet_rims, Side, ThickenError};
use crate::offset::{fit_grid, fit_grid_with, offset_face_surface, OffsetLimit};
use ck_geom::curve::Curve;
use ck_geom::fit::{interpolate, Param};
use ck_geom::surface::Surface;
use ck_geom::curve2::Curve2;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role, VertexId};
use std::collections::HashMap;

const EDGE_TOL: f64 = 2e-6;

/// Map lookup that fails with a structured error instead of panicking on a key the bookkeeping lost.
fn need<'a, K: std::hash::Hash + Eq, V>(m: &'a HashMap<K, V>, k: &K, what: &str) -> Result<&'a V, ThickenError> {
    m.get(k).ok_or_else(|| ThickenError::detail("thickenFailed", format!("a {what} of the sheet has no counterpart in the offset layers; the sheet's topology is not supported by thicken")))
}

fn unsupported(detail: impl Into<String>) -> ThickenError {
    ThickenError::detail("offsetUnsupported", detail)
}

/// Unit natural normal of `f`'s surface at the point of the surface closest to `p`.
fn normal_at(sheet: &Body, f: FaceId, p: Vec3) -> Option<Vec3> {
    let s = &sheet.face(f).surface;
    let (u, v, _) = s.closest_point(p);
    s.normal(u, v).try_normalize()
}

/// Parameters and points of the edge moved by `d` along the natural normal of `f`, sampled at
/// `n` uniform parameters.
fn samples(sheet: &Body, e: EdgeId, f: FaceId, n: usize) -> Result<(Vec<f64>, Vec<Vec3>, Vec<Vec3>), ThickenError> {
    let ed = sheet.edge(e);
    let mut ts = vec![];
    let mut ps = vec![];
    let mut ns = vec![];
    for k in 0..=n {
        let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64;
        let p = ed.curve.eval(t);
        let nn = normal_at(sheet, f, p).ok_or_else(|| unsupported("degenerate surface normal on an edge"))?;
        ts.push(t);
        ps.push(p);
        ns.push(nn);
    }
    Ok((ts, ps, ns))
}

/// Edge curve moved by `d` along the normal of `f`: refined until the cubic interpolant follows
/// the exact offset at span midpoints. Returns the curve with its domain and the sample count.
fn offset_edge(sheet: &Body, e: EdgeId, f: FaceId, d: f64) -> Result<(Curve, f64, f64, usize), ThickenError> {
    let ed = sheet.edge(e);
    let mut n = 24usize;
    loop {
        let (_, ps, ns) = samples(sheet, e, f, n)?;
        let pts: Vec<Vec3> = ps.iter().zip(&ns).map(|(p, nn)| *p + *nn * d).collect();
        let c = interpolate(&pts, 3, Param::ChordLength).map_err(|m| unsupported(format!("edge fit: {m}")))?;
        let (a, b) = c.domain();
        let mut dev = 0.0f64;
        for k in 0..n {
            let t = ed.t0 + (ed.t1 - ed.t0) * (k as f64 + 0.5) / n as f64;
            let p = ed.curve.eval(t);
            let Some(nn) = normal_at(sheet, f, p) else { continue };
            let exact = p + nn * d;
            dev = dev.max(c.eval(c.closest_param(exact)).dist(exact));
        }
        if dev <= EDGE_TOL || n >= 768 {
            return Ok((Curve::BSpline(c), a, b, n));
        }
        n *= 2;
    }
}

/// Thicken a sheet that has free-form faces.
pub fn thicken_freeform(sheet: &Body, feature: &str, thickness: f64, side: Side, tol: f64) -> Result<Body, ThickenError> {
    let faces: Vec<FaceId> = sheet.face_ids().collect();
    let rims = sheet_rims(sheet, &faces);
    if rims.is_empty() {
        return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
    }
    // layer offsets along the natural normal; `Both` is half the thickness each way
    let (d_lo, d_hi, scale) = match side {
        Side::Front => (0.0, thickness, 1.0),
        Side::Back => (-thickness, 0.0, 1.0),
        Side::Both => (-0.5 * thickness, 0.5 * thickness, 2.0),
    };
    let layers = [d_lo, d_hi];

    // faces of the sheet must agree on their natural side; edges shared by two faces that are not tangent
    // are creases, offset by mitering (see `thicken_creased`)
    let mut creased = false;
    for e in sheet.edge_ids().collect::<Vec<_>>() {
        if !sheet.edge(e).alive {
            continue;
        }
        let fs = sheet.edge_faces(e);
        if fs.len() > 2 {
            return Err(unsupported("an edge with more than two faces"));
        }
        if fs.len() == 2 {
            let ed = sheet.edge(e);
            for k in 0..=4 {
                let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 4.0);
                let (Some(a), Some(b)) = (normal_at(sheet, fs[0], p), normal_at(sheet, fs[1], p)) else {
                    return Err(unsupported("degenerate surface normal on an edge"));
                };
                if a.dot(b) < -0.9995 {
                    return Err(unsupported(format!(
                        "{} and {} face opposite ways along their shared edge: the sheet's faces must share one side to be thickened",
                        sheet.face_id_str(fs[0]),
                        sheet.face_id_str(fs[1])
                    )));
                }
                if a.dot(b) < 0.9998 {
                    creased = true;
                }
            }
        }
    }

    // offset surfaces of each layer
    let mut surfaces: Vec<Vec<Surface>> = vec![vec![]; 2];
    for (li, &d) in layers.iter().enumerate() {
        for &f in &faces {
            if d == 0.0 {
                surfaces[li].push(sheet.face(f).surface.clone());
                continue;
            }
            let s = offset_face_surface(sheet, f, d, 0.0).map_err(|e| {
                let name = sheet.face_id_str(f);
                if e.code == "offsetTooLarge" {
                    let limit = e.limit.clone().map(|l| OffsetLimit {
                        // a limit found for the offset `d` of this layer, in terms of the thickness
                        max_thickness: (l.max_thickness * scale * 100.0).floor() / 100.0,
                        face: Some(name.clone()),
                        ..l
                    });
                    let detail = match &limit {
                        Some(l) => format!(
                            "{name} curves too tightly for a thickness of {thickness:.2} mm: its smallest radius of curvature is {:.2} mm, use at most {:.2} mm",
                            l.min_radius, l.max_thickness
                        ),
                        None => format!("{name} collapses under an offset of {thickness:.2} mm"),
                    };
                    ThickenError { code: "offsetTooLarge", detail, limit }
                } else {
                    ThickenError::detail(e.code, format!("{name}: {}", e.detail.unwrap_or_default()))
                }
            })?;
            surfaces[li].push(s);
        }
    }

    if creased {
        return thicken_creased(sheet, feature, &faces, &rims, &surfaces, layers, tol);
    }

    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);

    // vertices per layer and sheet vertex; edge images per layer
    let mut vmap: [HashMap<VertexId, VertexId>; 2] = [HashMap::new(), HashMap::new()];
    let mut emap: [HashMap<EdgeId, EdgeId>; 2] = [HashMap::new(), HashMap::new()];
    // sample counts of the rim edges (shared by the walls)
    let mut rim_samples: HashMap<EdgeId, usize> = HashMap::new();
    for e in sheet.edge_ids().collect::<Vec<_>>() {
        if !sheet.edge(e).alive {
            continue;
        }
        let fs = sheet.edge_faces(e);
        if fs.is_empty() {
            continue;
        }
        let ed = sheet.edge(e);
        for (li, &d) in layers.iter().enumerate() {
            let (curve, t0, t1) = if d == 0.0 {
                (ed.curve.clone(), ed.t0, ed.t1)
            } else {
                let (c, a, b, n) = offset_edge(sheet, e, fs[0], d)?;
                rim_samples.insert(e, n);
                (c, a, b)
            };
            let mut vid = |sv: VertexId, t: f64, out: &mut Body| *vmap[li].entry(sv).or_insert_with(|| out.add_vertex(curve.eval(t)));
            let v0 = vid(ed.v0, t0, &mut out);
            let v1 = vid(ed.v1, t1, &mut out);
            let ne = out.add_edge(curve.clone(), t0, t1, v0, v1);
            emap[li].insert(e, ne);
        }
    }

    // faces of both layers; the lower layer faces away from the material
    add_layer_faces(&mut out, shell, sheet, &faces, &surfaces, &emap)?;

    // connectors between the layers at the rim corners, walls along the rim edges
    let mut connectors: HashMap<VertexId, EdgeId> = HashMap::new();
    let mut w = 0u32;
    for &(e, f) in &rims {
        let ed = sheet.edge(e);
        let co = *sheet.edge_coedges(e).first().ok_or_else(|| ThickenError::code("thickenFailed"))?;
        let rev = sheet.coedge(co).rev;
        let n = *rim_samples.get(&e).unwrap_or(&48);
        // wall surface through the samples: four rows from the lower to the upper layer
        let (_, ps, ns) = samples(sheet, e, f, n)?;
        let grid: Vec<Vec<Vec3>> = ps
            .iter()
            .zip(&ns)
            .map(|(p, nn)| (0..4).map(|j| *p + *nn * (d_lo + (d_hi - d_lo) * j as f64 / 3.0)).collect())
            .collect();
        let (bs, _, _) = fit_grid(&grid).map_err(|e| unsupported(format!("wall fit: {}", e.detail.unwrap_or_default())))?;
        let wname = FaceName { feature: feature.into(), role: Role::Other(format!("W{w}")), region: None, suffix: String::new() };
        w += 1;
        let wf = out.add_face(shell, Surface::BSpline(Box::new(bs)), true, wname);
        let wl = out.add_loop(wf);
        // lower layer edge as the lower face walks it reversed, then up the connector, the upper edge, down
        let (rev_lo, rev_hi) = (!(rev != true), !(rev != false));
        let end_of = |v: (VertexId, VertexId), r: bool| if r { v.0 } else { v.1 };
        let (lo, hi) = (*need(&emap[0], &e, "edge")?, *need(&emap[1], &e, "edge")?);
        let lo_end = end_of((out.edge(lo).v0, out.edge(lo).v1), rev_lo);
        let lo_start = end_of((out.edge(lo).v0, out.edge(lo).v1), !rev_lo);
        let to_hi = |lv: VertexId, sv: VertexId, out: &mut Body, connectors: &mut HashMap<VertexId, EdgeId>| -> EdgeId {
            *connectors.entry(sv).or_insert_with(|| {
                let hv = vmap[1][&sv];
                let (a, b) = (out.vertex(lv).p, out.vertex(hv).p);
                out.add_edge(Curve::line_through(a, b), 0.0, (b - a).len(), lv, hv)
            })
        };
        let sv_of = |lv: VertexId| vmap[0].iter().find(|(_, &x)| x == lv).map(|(&k, _)| k);
        let (s_end, s_start) = (sv_of(lo_end).ok_or_else(|| ThickenError::code("thickenFailed"))?, sv_of(lo_start).ok_or_else(|| ThickenError::code("thickenFailed"))?);
        need(&vmap[1], &s_end, "vertex")?;
        need(&vmap[1], &s_start, "vertex")?;
        out.add_coedge(wl, lo, rev_lo, None);
        let c_end = to_hi(lo_end, s_end, &mut out, &mut connectors);
        out.add_coedge(wl, c_end, false, None);
        out.add_coedge(wl, hi, rev_hi, None);
        let c_start = to_hi(lo_start, s_start, &mut out, &mut connectors);
        out.add_coedge(wl, c_start, true, None);
        let _ = ed;
    }

    close_up(out, tol)
}

/// Faces of both layers: the lower layer faces away from the material.
fn add_layer_faces(out: &mut Body, shell: ck_topo::body::ShellId, sheet: &Body, faces: &[FaceId], surfaces: &[Vec<Surface>], emap: &[HashMap<EdgeId, EdgeId>; 2]) -> Result<(), ThickenError> {
    for li in 0..2 {
        let flipped = li == 0;
        for (fi, &f) in faces.iter().enumerate() {
            let face = sheet.face(f);
            let mut name: FaceName = face.name.clone();
            name.region = Some(if li == 0 { 1 } else { 2 });
            let nf = out.add_face(shell, surfaces[li][fi].clone(), face.sense != flipped, name);
            for &lid in &face.loops {
                if !sheet.lp(lid).alive {
                    continue;
                }
                let cl: Vec<_> = sheet.lp(lid).coedges.iter().copied().filter(|&c| sheet.coedge(c).alive).collect();
                let ordered: Vec<_> = if flipped { cl.iter().rev().copied().collect() } else { cl };
                let nloop = out.add_loop(nf);
                for cid in ordered {
                    let co = sheet.coedge(cid);
                    out.add_coedge(nloop, *need(&emap[li], &co.edge, "edge")?, co.rev != flipped, None);
                }
            }
        }
    }
    Ok(())
}

/// Weld, orient and check that the thickened body is closed with positive volume.
fn close_up(mut out: Body, tol: f64) -> Result<Body, ThickenError> {
    super::weld_vertices(&mut out, tol.max(1e-7));
    ck_topo::orient::orient_body(&mut out);
    for e in out.edge_ids().collect::<Vec<_>>() {
        if out.edge(e).alive && out.edge_coedges(e).len() != 2 {
            if std::env::var("CK_THICKEN_DEBUG").is_ok() {
                eprintln!("DBG open edge {} coedges {}", e.idx(), out.edge_coedges(e).len());
            }
            return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
        }
    }
    let vol = ck_topo::mass::signed_volume(&out, &out.face_ids().collect::<Vec<_>>());
    if !vol.is_finite() || vol <= 0.0 {
        return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
    }
    Ok(out)
}

// ---------------------------------------------------------------------------------------------
// Sheets with creases
//
// A crease between two free-form faces (the corners of a polygon-section loft) is offset by
// mitering: the layer's two offset surfaces are extended until they meet, and the layer's edge is
// the curve where they do. A layer vertex is where the offset surfaces of all faces at the sheet
// vertex meet and the plane normal to the crease at that vertex (the miter point). A rim edge of
// the layer is the sheet's rim moved along the face normal, clipped to the layer's end vertices,
// and each wall is the ruled surface between the sheet's rim and the layer's, so the walls run
// along the normals and the connectors join each sheet corner to its miter point. Where the offset
// surfaces do not meet (the offset reaches past a face) the thicken fails with `thickenReach`.

const REACH_MSG: &str = "the offset faces do not meet at a corner (the offset reaches past a face)";

fn plane_surface(o: Vec3, n: Vec3) -> Surface {
    Surface::Plane { frame: ck_geom::Frame::from_normal(o, n) }
}

fn edge_tangent_at(sheet: &Body, e: EdgeId, v: VertexId) -> Option<Vec3> {
    let ed = sheet.edge(e);
    let t = if ed.v0 == v { ed.t0 } else { ed.t1 };
    ed.curve.tangent(t).try_normalize()
}

/// The layer's rim edge of `e` on the offset surface `moved` of its face `f`, between the layer's end
/// vertices `q0`, `q1` (the miter corners, which sit off the plain normal offset of the sheet's rim
/// end points). Each point is the rim point moved along the face normal by `d`, shifted by the corner
/// corrections blended along the edge and carried back onto the offset surface. Where the corners are
/// plain normal offsets this is the plain normal offset of the rim; between two miter corners of a
/// straight rim it is the straight edge joining them. Refined until the cubic interpolant follows the
/// exact points.
#[allow(clippy::too_many_arguments)]
fn rim_curve(sheet: &Body, e: EdgeId, f: FaceId, moved: &Surface, d: f64, q0: Vec3, q1: Vec3) -> Result<(Curve, f64, f64), ThickenError> {
    let ed = sheet.edge(e);
    let plain = |t: f64| -> Result<Vec3, ThickenError> {
        let p = ed.curve.eval(t);
        let nn = normal_at(sheet, f, p).ok_or_else(|| unsupported("degenerate surface normal on an edge"))?;
        Ok(p + nn * d)
    };
    let (e0, e1) = (q0 - plain(ed.t0)?, q1 - plain(ed.t1)?);
    let point = |s: f64| -> Result<Vec3, ThickenError> {
        let target = plain(ed.t0 + (ed.t1 - ed.t0) * s)? + e0 * (1.0 - s) + e1 * s;
        Ok(moved.closest_point(target).2)
    };
    if q0.dist(q1) < 1e-9 {
        return Err(ThickenError::detail("thickenTooThick", "an edge vanishes"));
    }
    let mut n = 24usize;
    loop {
        let mut pts = Vec::with_capacity(n + 1);
        for k in 0..=n {
            pts.push(match k {
                0 => q0,
                _ if k == n => q1,
                _ => point(k as f64 / n as f64)?,
            });
        }
        if pts.windows(2).any(|w| w[0].dist(w[1]) < 1e-9) {
            return Err(ThickenError::detail("thickenTooThick", "an edge vanishes"));
        }
        let c = interpolate(&pts, 3, Param::ChordLength).map_err(|m| unsupported(format!("edge fit: {m}")))?;
        let (a, b) = c.domain();
        let mut dev = 0.0f64;
        for k in 0..n {
            let exact = point((k as f64 + 0.5) / n as f64)?;
            dev = dev.max(c.eval(c.closest_param(exact)).dist(exact));
        }
        if dev <= EDGE_TOL || n >= 768 {
            return Ok((Curve::BSpline(c), a, b));
        }
        n *= 2;
    }
}

/// Ruled surface between two edge curves, sampled at matching fractions of their domains (the
/// rulings at the ends are the corner connectors).
fn ruled_wall(lo: &Curve, (a0, b0): (f64, f64), hi: &Curve, (a1, b1): (f64, f64), n: usize) -> Result<Surface, ThickenError> {
    let grid: Vec<Vec<Vec3>> = (0..=n)
        .map(|k| {
            let s = k as f64 / n as f64;
            let (p, q) = (lo.eval(a0 + (b0 - a0) * s), hi.eval(a1 + (b1 - a1) * s));
            (0..4).map(|j| p + (q - p) * (j as f64 / 3.0)).collect()
        })
        .collect();
    // parameters are the sampling fractions themselves, so a wall's boundary edges and rulings are
    // iso-parameter lines and the coedges can carry exact pcurves
    let uk: Vec<f64> = (0..=n).map(|k| k as f64 / n as f64).collect();
    let vk = vec![0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0];
    let (bs, _, _) = fit_grid_with(&grid, uk, vk).map_err(|e| unsupported(format!("wall fit: {}", e.detail.unwrap_or_default())))?;
    Ok(Surface::BSpline(Box::new(bs)))
}

/// Thicken a sheet whose free-form faces meet in creases.
fn thicken_creased(sheet: &Body, feature: &str, faces: &[FaceId], rims: &[(EdgeId, FaceId)], surfaces: &[Vec<Surface>], layers: [f64; 2], tol: f64) -> Result<Body, ThickenError> {
    let moved: Vec<Vec<Option<Surface>>> = (0..2)
        .map(|li| {
            let mut v: Vec<Option<Surface>> = vec![None; sheet.faces.len()];
            for (fi, &f) in faces.iter().enumerate() {
                v[f.idx()] = Some(surfaces[li][fi].clone());
            }
            v
        })
        .collect();
    let live_edges: Vec<EdgeId> = sheet.edge_ids().filter(|&e| sheet.edge(e).alive && !sheet.edge_faces(e).is_empty()).collect();
    let rim_set: std::collections::HashSet<EdgeId> = rims.iter().map(|r| r.0).collect();
    let is_crease = |e: EdgeId| -> bool {
        let fs = sheet.edge_faces(e);
        if fs.len() != 2 {
            return false;
        }
        let ed = sheet.edge(e);
        (0..=4).any(|k| {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 4.0);
            matches!((normal_at(sheet, fs[0], p), normal_at(sheet, fs[1], p)), (Some(a), Some(b)) if a.dot(b) < 0.9998)
        })
    };
    let mut vedges: HashMap<VertexId, Vec<EdgeId>> = HashMap::new();
    let mut vfaces: HashMap<VertexId, Vec<FaceId>> = HashMap::new();
    for &e in &live_edges {
        let ed = sheet.edge(e);
        if ed.v0 == ed.v1 {
            return Err(unsupported("a closed edge of a creased sheet"));
        }
        for v in [ed.v0, ed.v1] {
            vedges.entry(v).or_default().push(e);
            for f in sheet.edge_faces(e) {
                let list = vfaces.entry(v).or_default();
                if !list.contains(&f) {
                    list.push(f);
                }
            }
        }
    }

    // layer vertices: the miter points
    let mut vpos: [HashMap<VertexId, Vec3>; 2] = [HashMap::new(), HashMap::new()];
    for (li, &d) in layers.iter().enumerate() {
        for (&v, es) in &vedges {
            let p = sheet.vertex(v).p;
            if d == 0.0 {
                vpos[li].insert(v, p);
                continue;
            }
            let fs = need(&vfaces, &v, "vertex")?;
            if fs.is_empty() {
                return Err(ThickenError::code("thickenFailed"));
            }
            let crease = es.iter().copied().find(|&e| is_crease(e));
            let mut surfs: Vec<Surface> = if crease.is_some() { fs.iter().map(|&f| moved[li][f.idx()].clone().ok_or_else(|| ThickenError::code("thickenFailed"))).collect::<Result<_, _>>()? } else { vec![moved[li][fs[0].idx()].clone().ok_or_else(|| ThickenError::code("thickenFailed"))?] };
            let nrm = normal_at(sheet, fs[0], p).ok_or_else(|| unsupported("degenerate surface normal at a corner"))?;
            let tang = crease.or_else(|| es.iter().copied().find(|e| rim_set.contains(e))).or_else(|| es.first().copied()).and_then(|e| edge_tangent_at(sheet, e, v));
            if surfs.len() < 3 {
                if let Some(t) = tang {
                    surfs.push(plane_surface(p, t));
                    if surfs.len() < 3 {
                        surfs.push(plane_surface(p, nrm.cross(t).try_normalize().ok_or_else(|| unsupported("degenerate corner"))?));
                    }
                }
            }
            let refs: Vec<&Surface> = surfs.iter().collect();
            let q = crate::offset::solve_point(p + nrm * d, &refs).ok_or_else(|| ThickenError::detail("thickenReach", REACH_MSG))?;
            let res = surfs.iter().map(|s| s.closest_point(q).2.dist(q)).fold(0.0f64, f64::max);
            if std::env::var("CK_THICKEN_DEBUG").is_ok() {
                eprintln!("corner {:?} at {:?} d={d}: miter {:?} residual {res:.2e} moved {:.3}; per surface {:?}", v, p, q, q.dist(p), surfs.iter().map(|s| (s.kind(), s.closest_point(q).2.dist(q))).collect::<Vec<_>>());
            }
            if res > 1e-6 * (1.0 + p.len()) || q.dist(p) > 50.0 * d.abs() + 1e-9 {
                return Err(ThickenError::detail("thickenReach", REACH_MSG));
            }
            vpos[li].insert(v, q);
        }
    }

    // layer edges
    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);
    let mut vmap: [HashMap<VertexId, VertexId>; 2] = [HashMap::new(), HashMap::new()];
    let mut emap: [HashMap<EdgeId, EdgeId>; 2] = [HashMap::new(), HashMap::new()];
    let mut ecurve: [HashMap<EdgeId, (Curve, f64, f64)>; 2] = [HashMap::new(), HashMap::new()];
    for &e in &live_edges {
        let ed = sheet.edge(e);
        let fs = sheet.edge_faces(e);
        for (li, &d) in layers.iter().enumerate() {
            let (p0, p1) = (*need(&vpos[li], &ed.v0, "vertex")?, *need(&vpos[li], &ed.v1, "vertex")?);
            let (curve, t0, t1) = if d == 0.0 {
                (ed.curve.clone(), ed.t0, ed.t1)
            } else if fs.len() == 2 {
                let gap = [ed.v0, ed.v1]
                    .iter()
                    .map(|v| {
                        let Some(&q) = vpos[li].get(v) else { return 0.0 };
                        vfaces.get(v).into_iter().flatten().map(|&f| moved[li][f.idx()].as_ref().map_or(0.0, |m| m.closest_point(q).2.dist(q))).fold(0.0f64, f64::max)
                    })
                    .fold(0.0f64, f64::max);
                crate::offset::offset_spline_edge(sheet, e, &moved[li], &|_f| d, p0, p1, gap).map_err(|err| match err.code {
                    "offsetDegenerate" => ThickenError::detail("thickenReach", REACH_MSG),
                    c => ThickenError::detail(c, err.detail.unwrap_or_default()),
                })?
            } else {
                rim_curve(sheet, e, fs[0], moved[li][fs[0].idx()].as_ref().ok_or_else(|| ThickenError::code("thickenFailed"))?, d, p0, p1)?
            };
            let v0 = *vmap[li].entry(ed.v0).or_insert_with(|| out.add_vertex(p0));
            let v1 = *vmap[li].entry(ed.v1).or_insert_with(|| out.add_vertex(p1));
            let ne = out.add_edge(curve.clone(), t0, t1, v0, v1);
            emap[li].insert(e, ne);
            ecurve[li].insert(e, (curve, t0, t1));
        }
    }
    add_layer_faces(&mut out, shell, sheet, faces, surfaces, &emap)?;

    // walls along the rim edges, ruled between the two layers' rim edges
    let mut connectors: HashMap<VertexId, EdgeId> = HashMap::new();
    let mut w = 0u32;
    for &(e, _f) in rims {
        let ed = sheet.edge(e);
        let co = *sheet.edge_coedges(e).first().ok_or_else(|| ThickenError::code("thickenFailed"))?;
        let rev = sheet.coedge(co).rev;
        let (lo_c, hi_c) = (need(&ecurve[0], &e, "edge")?, need(&ecurve[1], &e, "edge")?);
        let surf = ruled_wall(&lo_c.0, (lo_c.1, lo_c.2), &hi_c.0, (hi_c.1, hi_c.2), 96)?;
        let wname = FaceName { feature: feature.into(), role: Role::Other(format!("W{w}")), region: None, suffix: String::new() };
        w += 1;
        let wf = out.add_face(shell, surf, true, wname);
        let wl = out.add_loop(wf);
        let (rev_lo, rev_hi) = (!(rev != true), !(rev != false));
        let (lo, hi) = (*need(&emap[0], &e, "edge")?, *need(&emap[1], &e, "edge")?);
        // sheet vertices at the end / start of the lower layer's edge as the wall walks it
        let (s_end, s_start) = if rev_lo { (ed.v0, ed.v1) } else { (ed.v1, ed.v0) };
        let mut connector = |sv: VertexId, out: &mut Body| -> EdgeId {
            *connectors.entry(sv).or_insert_with(|| {
                let (lv, hv) = (vmap[0][&sv], vmap[1][&sv]);
                let (a, b) = (out.vertex(lv).p, out.vertex(hv).p);
                out.add_edge(Curve::line_through(a, b), 0.0, (b - a).len(), lv, hv)
            })
        };
        // exact pcurves: the wall's u is the fraction of the rim edge, v runs from the lower layer to the upper
        let frac = |c: &(Curve, f64, f64), v: f64| {
            let span = c.2 - c.1;
            Curve2::Line { o: Vec2::new(-c.1 / span, v), d: Vec2::new(1.0 / span, 0.0) }
        };
        let ruling = |ce: EdgeId, u: f64, out: &Body| {
            let ed = out.edge(ce);
            Curve2::Line { o: Vec2::new(u, -ed.t0 / (ed.t1 - ed.t0)), d: Vec2::new(0.0, 1.0 / (ed.t1 - ed.t0)) }
        };
        // the wall walks the lower edge, up the connector at its end, back along the upper edge, down the connector at its start
        let (u_end, u_start) = if rev_lo { (0.0, 1.0) } else { (1.0, 0.0) };
        for sv in [s_end, s_start] {
            need(&vmap[0], &sv, "vertex")?;
            need(&vmap[1], &sv, "vertex")?;
        }
        out.add_coedge(wl, lo, rev_lo, Some(frac(lo_c, 0.0)));
        let c_end = connector(s_end, &mut out);
        let pc = ruling(c_end, u_end, &out);
        out.add_coedge(wl, c_end, false, Some(pc));
        out.add_coedge(wl, hi, rev_hi, Some(frac(hi_c, 1.0)));
        let c_start = connector(s_start, &mut out);
        let pc = ruling(c_start, u_start, &out);
        out.add_coedge(wl, c_start, true, Some(pc));
    }
    close_up(out, tol)
}
