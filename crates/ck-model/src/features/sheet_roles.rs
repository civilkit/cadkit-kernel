//! Sheet metal, stage 2: stable names for the faces and edges of a sheet body (docs/sheet-metal.md, "Roles").
//!
//! A pick by point breaks when the base changes: the point may then lie on another edge. A role names a face by what it is
//! in the sheet, not where it is. The grammar:
//!
//! ```text
//! role   := face | face '|' face            (an edge is the seam of two faces)
//! face   := panel '.' pface | bend '.' bface
//! panel  := 'P' n          (panel 0 is the base plate, the root panel of a folded base, or the first leg of a contour flange;
//!                           the legs of flanges follow in the order they were made, bend by bend)
//! bend   := 'B' n          (numbered 1.. as in the bend table)
//! pface  := 'front' | 'back' | 'tip' | 'side.' name
//! bface  := 'front' | 'back' | 'side.start' | 'side.end'
//! name   := the id of the sketch curve the wall runs along (panels of the base), or 'start' | 'end' (legs)
//! ```
//!
//! `front` is the face the base flange started on (the sketch plane), `back` the cap face; they stay front and back through
//! the bends. `tip` is the free end of a leg. Roles resolve against the sheet model, not the body, so features may take them
//! in place of a point: `{"role":"P0.side.e2"}`, or `{"role":"P0.back|P0.side.e2"}` for the edge of the back face and that wall.
//! [`match_faces`] finds the body face of each role (the query result `sheetMetal[].roles`).

use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

/// Where a role's face is, for matching it to a face of the body.
#[derive(Debug, Clone, PartialEq)]
pub enum Geo {
    /// Plane through `o` with unit normal `n` (either sense); the face lies within the box `lo..hi`.
    Plane { o: Vec3, n: Vec3, lo: Vec3, hi: Vec3 },
    /// Cylinder about the line through `o` along `axis`.
    Cyl { o: Vec3, axis: Vec3, r: f64, lo: Vec3, hi: Vec3 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct RoleGeo {
    pub role: String,
    pub geo: Geo,
}

fn bbox(pts: &[Vec3]) -> (Vec3, Vec3) {
    let mut lo = pts[0];
    let mut hi = pts[0];
    for p in pts {
        lo = lo.min(*p);
        hi = hi.max(*p);
    }
    (lo, hi)
}

/// A planar role from the corners of its region (at least three, not on a line).
pub fn plane_role(role: &str, pts: &[Vec3]) -> Option<RoleGeo> {
    let mut n = Vec3::ZERO;
    'o: for i in 0..pts.len() {
        for j in i + 1..pts.len() {
            for k in j + 1..pts.len() {
                let c = (pts[j] - pts[i]).cross(pts[k] - pts[i]);
                if c.len() > 1e-9 {
                    n = c.normalize();
                    break 'o;
                }
            }
        }
    }
    if n == Vec3::ZERO {
        return None;
    }
    let (lo, hi) = bbox(pts);
    Some(RoleGeo { role: role.to_string(), geo: Geo::Plane { o: pts[0], n, lo, hi } })
}

/// A cylindrical role from a point of the axis, its direction, the radius and points of the region.
pub fn cyl_role(role: &str, o: Vec3, axis: Vec3, r: f64, pts: &[Vec3]) -> RoleGeo {
    let (lo, hi) = bbox(pts);
    RoleGeo { role: role.to_string(), geo: Geo::Cyl { o, axis: axis.normalize(), r, lo, hi } }
}

const TOL: f64 = 1e-5;

fn inside(p: Vec3, lo: Vec3, hi: Vec3) -> bool {
    (0..3).all(|k| p.get(k) >= lo.get(k) - TOL && p.get(k) <= hi.get(k) + TOL)
}

fn center(lo: Vec3, hi: Vec3) -> Vec3 {
    (lo + hi) * 0.5
}

/// The face of `body` each role names: `(role, face id)`; `None` when the body has no such face (a leg of length 0, an
/// edge consumed by a neighbouring bend). Coplanar faces of neighbouring pieces merge in the body (the wall of a plate, its
/// bends and legs is one face): every role of a piece of it names the merged face.
pub fn match_faces(body: &Body, roles: &[RoleGeo]) -> Vec<(String, Option<String>)> {
    use ck_geom::surface::Surface;
    let faces: Vec<_> = body.face_ids().collect();
    // vertices of every face: their box and centroid
    let info: Vec<(Vec3, Vec3, Vec3)> = faces
        .iter()
        .map(|&f| {
            let v = face_vertices(body, f);
            if v.is_empty() {
                return (Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
            }
            let (lo, hi) = bbox(&v);
            let c = v.iter().fold(Vec3::ZERO, |a, p| a + *p) * (1.0 / v.len() as f64);
            (lo, hi, c)
        })
        .collect();
    roles
        .iter()
        .map(|rg| {
            let mut best: Option<(f64, usize)> = None;
            for (i, &f) in faces.iter().enumerate() {
                let (flo, fhi, fc) = info[i];
                let score = match (&rg.geo, &body.face(f).surface) {
                    (Geo::Plane { o, n, lo, hi }, Surface::Plane { frame }) => {
                        let same = frame.z.dot(*n).abs() > 1.0 - 1e-9 && (frame.o - *o).dot(*n).abs() < TOL;
                        // the role's piece lies within the face (the face may be larger: merged coplanar pieces)
                        if !same || !inside(center(*lo, *hi), flo, fhi) {
                            continue;
                        }
                        (0..3).map(|k| (flo.get(k) - lo.get(k)).abs() + (fhi.get(k) - hi.get(k)).abs()).sum::<f64>()
                    }
                    (Geo::Cyl { o, axis, r, lo, hi }, Surface::Cylinder { frame, r: fr }) => {
                        let off = frame.o - *o;
                        let same = (fr - r).abs() < TOL && frame.z.dot(*axis).abs() > 1.0 - 1e-9 && off.cross(*axis).len() < TOL;
                        if !same {
                            continue;
                        }
                        // along the axis the bend face is exactly the role's extent
                        let role_ax = ext_range(*lo, *hi, *axis);
                        let (mut a, mut b) = (f64::MAX, f64::MIN);
                        for p in face_vertices(body, f) {
                            a = a.min(p.dot(*axis));
                            b = b.max(p.dot(*axis));
                        }
                        if (a - role_ax.0).abs() > 1e-4 || (b - role_ax.1).abs() > 1e-4 {
                            continue;
                        }
                        (center(*lo, *hi) - fc).len()
                    }
                    _ => continue,
                };
                if best.map_or(true, |(s, _)| score < s) {
                    best = Some((score, i));
                }
            }
            (rg.role.clone(), best.map(|(_, i)| body.face_id_str(faces[i])))
        })
        .collect()
}

fn face_vertices(body: &Body, f: ck_topo::body::FaceId) -> Vec<Vec3> {
    let mut v = vec![];
    for e in body.face_edges(f) {
        let ed = body.edge(e);
        v.push(body.vertex(ed.v0).p);
        v.push(body.vertex(ed.v1).p);
    }
    v
}

fn ext_range(lo: Vec3, hi: Vec3, axis: Vec3) -> (f64, f64) {
    let mut a = f64::MAX;
    let mut b = f64::MIN;
    for k in 0..8 {
        let p = Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
        a = a.min(p.dot(axis));
        b = b.max(p.dot(axis));
    }
    (a, b)
}

/// The seam of two roles in the body, as the midpoint of an edge both faces share.
pub fn seam(body: &Body, a: &str, b: &str) -> Option<Vec3> {
    let fa = body.find_face(a)?;
    let fb = body.find_face(b)?;
    body.face_edges(fa).into_iter().find(|&e| body.edge_faces(e).contains(&fb)).map(|e| body.edge_mid(e))
}

/// `{"faces":[{"role":..,"face":..|null}..],"edges":[{"role":"A|B","faces":[..],"mid":[x,y,z]|null}..]}`.
pub fn roles_json(body: Option<&Body>, roles: &[RoleGeo]) -> Json {
    let matched = body.map(|b| match_faces(b, roles)).unwrap_or_else(|| roles.iter().map(|r| (r.role.clone(), None)).collect());
    let faces: Vec<Json> = matched
        .iter()
        .map(|(r, f)| {
            let mut o = Obj::new().put("role", r.as_str());
            o = match f {
                Some(f) => o.put("face", f.as_str()),
                None => o.put("face", Json::Null),
            };
            o.build()
        })
        .collect();
    // edges: the seams of the faces that exist and touch
    let mut edges: Vec<Json> = vec![];
    if let Some(b) = body {
        let found: Vec<&(String, Option<String>)> = matched.iter().filter(|(_, f)| f.is_some()).collect();
        for i in 0..found.len() {
            for j in i + 1..found.len() {
                let (ra, fa) = (&found[i].0, found[i].1.as_deref().unwrap());
                let (rb, fb) = (&found[j].0, found[j].1.as_deref().unwrap());
                if fa == fb {
                    continue;
                }
                if let Some(m) = seam(b, fa, fb) {
                    edges.push(
                        Obj::new()
                            .put("role", format!("{ra}|{rb}"))
                            .put("faces", Json::Arr(vec![Json::Str(fa.into()), Json::Str(fb.into())]))
                            .put("mid", Json::Arr(vec![Json::Num(m.x), Json::Num(m.y), Json::Num(m.z)]))
                            .build(),
                    );
                }
            }
        }
    }
    Obj::new().put("faces", Json::Arr(faces)).put("edges", Json::Arr(edges)).build()
}

/// What an edge pick by role names: a wall of a panel, and which face of the panel the edge is on.
#[derive(Debug, Clone, PartialEq)]
pub struct EdgeRole {
    pub panel: usize,
    /// `Some(true)`: the edge of the back face, `Some(false)`: of the front face, `None`: the wall alone.
    pub back: Option<bool>,
    /// The wall: a sketch curve id.
    pub side: String,
}

fn panel_face(s: &str) -> Option<(usize, &str)> {
    let rest = s.trim().strip_prefix('P')?;
    let (n, face) = rest.split_once('.')?;
    Some((n.parse().ok()?, face))
}

/// Parse `P0.side.e2`, `P0.back|P0.side.e2`, `P0.front|P0.side.e2` (either order).
pub fn parse_edge_role(s: &str) -> Result<EdgeRole, String> {
    let bad = || format!("`{s}` is not a wall or an edge of a wall: P<n>.side.<curve> or P<n>.front|P<n>.side.<curve>");
    let parts: Vec<&str> = s.split('|').collect();
    if parts.len() > 2 || parts.is_empty() {
        return Err(bad());
    }
    let mut back = None;
    let mut side = None;
    let mut panel = None;
    for p in parts {
        let (n, face) = panel_face(p).ok_or_else(bad)?;
        if panel.map_or(false, |q| q != n) {
            return Err(bad());
        }
        panel = Some(n);
        match face {
            "front" => back = Some(false),
            "back" => back = Some(true),
            f if f.starts_with("side.") && f.len() > 5 => side = Some(f[5..].to_string()),
            "tip" => side = Some("tip".to_string()),
            _ => return Err(bad()),
        }
    }
    match (panel, side) {
        (Some(panel), Some(side)) => Ok(EdgeRole { panel, back, side }),
        _ => Err(bad()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn edge_roles() {
        assert_eq!(parse_edge_role("P0.side.e2").unwrap(), EdgeRole { panel: 0, back: None, side: "e2".into() });
        assert_eq!(parse_edge_role("P0.back|P0.side.e2").unwrap().back, Some(true));
        assert_eq!(parse_edge_role("P0.side.e2|P0.front").unwrap().back, Some(false));
        assert!(parse_edge_role("P0.front").is_err());
        assert!(parse_edge_role("P0.side.e2|P1.front").is_err());
        assert!(parse_edge_role("B1.front|P0.side.e2").is_err());
        assert!(parse_edge_role("side.e2").is_err());
    }
}
