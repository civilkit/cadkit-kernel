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
use crate::offset::{fit_grid, offset_face_surface, OffsetLimit};
use ck_geom::curve::Curve;
use ck_geom::fit::{interpolate, Param};
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role, VertexId};
use std::collections::HashMap;

const EDGE_TOL: f64 = 2e-6;

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

    // faces of the sheet must agree on their natural side, edges shared by two faces must be tangent
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
                if a.dot(b) < 0.9998 {
                    return Err(unsupported(format!(
                        "{} and {} meet at a crease or face opposite ways: thicken offsets free-form sheets whose faces join tangentially. Thicken the faces separately or blend the crease first",
                        sheet.face_id_str(fs[0]),
                        sheet.face_id_str(fs[1])
                    )));
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
    for (li, _) in layers.iter().enumerate() {
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
                    out.add_coedge(nloop, emap[li][&co.edge], co.rev != flipped, None);
                }
            }
        }
    }

    // connectors between the layers at the rim corners, walls along the rim edges
    let mut connectors: HashMap<VertexId, EdgeId> = HashMap::new();
    let mut w = 0u32;
    for &(e, f) in &rims {
        let ed = sheet.edge(e);
        let co = sheet.edge_coedges(e)[0];
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
        let (lo, hi) = (emap[0][&e], emap[1][&e]);
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
        out.add_coedge(wl, lo, rev_lo, None);
        let c_end = to_hi(lo_end, s_end, &mut out, &mut connectors);
        out.add_coedge(wl, c_end, false, None);
        out.add_coedge(wl, hi, rev_hi, None);
        let c_start = to_hi(lo_start, s_start, &mut out, &mut connectors);
        out.add_coedge(wl, c_start, true, None);
        let _ = ed;
    }

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
