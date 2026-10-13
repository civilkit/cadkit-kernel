//! Variable-radius fillets (ck-ops/src/blend/interp.rs): the radius (or the
//! chord width, `rho`, ...) changes linearly along a straight edge between two
//! planar faces.
//!
//! Codes: `invalidValue` (a radius <= 0), `blendTooLarge` (a contact line
//! leaves its face), `blendUnsupported` (curved edge or face), `missingEdge`.
//! Oracle probes (docs/ck-ops-stubs.md, `var*` in tools/stub_cases.mjs):
//! * `radii: [a, b]` (`filletType: "variable"`): `a` at the end of the edge
//!   nearer the pick point, `b` at the other; every picked edge gets the same
//!   pair (by its own pick); [1, 3] on the 40 mm box edge removes
//!   0.2146 * (1 + 3 + 9) / 3 * 40 mm3 (7962.80235): the sections are exact
//!   circles of radius `r(t)`, i.e. the surface is the cone ruled between the
//!   two end sections;
//! * the same holds for `chord` (widths), `conic` (`rhos: [a, b]` or `rho`)
//!   and `curvature` (sections `r(t) * unit curve`): 7981.40117, 7973.76509 /
//!   7971.11111, 7953.84539.
//!
//! Construction: the tool is the loft (ruled, two stations) between the corner
//! sections at the two ends of the edge, extended by a margin past both ends
//! when the blend is convex (the radius is extrapolated linearly). It is
//! applied by the boolean like the constant fillets, with the blend face
//! named `B<k>`; its pair with the two faces carries a `PairHook` (tangent
//! contact along the end rulings).

use super::build::{apply_tool_hooked, find_edge, FilletProfile};
use super::BlendError;
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::Op;
use crate::loft::{loft, LoftOpts, ProfileIn, SegIn};
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, FaceId, FaceName, Role};
use ck_topo::make::{curve2_to_3d, ProfileLoop, ProfileSeg};

/// Per-end section parameters of a variable blend.
#[derive(Debug, Clone, Copy)]
pub struct VarSpec {
    /// Radius (circular / conic / curvature) or chord width (chord) at the pick-near / far end.
    pub radii: [f64; 2],
    pub profile: FilletProfile,
    /// `conic`: `rho` at the near / far end.
    pub rhos: [f64; 2],
}

/// Variable-radius fillets of `edges` (one blend each, `B0, B1, ..` in selection order).
pub fn variable_fillet(b: &Body, edges: &[(String, String, Vec3)], v: VarSpec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    if !(v.radii[0] > 0.0 && v.radii[1] > 0.0) || !v.radii.iter().all(|r| r.is_finite()) {
        return Err(BlendError::code("invalidValue"));
    }
    if edges.is_empty() {
        return Err(BlendError::code("missingEdge"));
    }
    let mut cur = b.clone();
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        cur = blend_one(&cur, fa, fb, *at, v, feature, k as u32, tol)?;
    }
    Ok(cur)
}

fn unsupported() -> BlendError {
    BlendError::code("blendUnsupported")
}

fn blend_one(b: &Body, fa: &str, fb: &str, at: Vec3, v: VarSpec, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    let e = find_edge(b, fa, fb, at).ok_or_else(|| BlendError::code("missingEdge"))?;
    let ed = b.edge(e).clone();
    if !matches!(ed.curve, Curve::Line { .. }) {
        return Err(unsupported());
    }
    let faces: [FaceId; 2] = [b.find_face(fa).ok_or_else(|| BlendError::code("missingEdge"))?, b.find_face(fb).ok_or_else(|| BlendError::code("missingEdge"))?];
    if faces.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
        return Err(unsupported());
    }
    let (pa, pb) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
    let len = pa.dist(pb);
    let dir_e = (pb - pa) / len;
    let (near, far) = if at.dist(pa) <= at.dist(pb) { (pa, pb) } else { (pb, pa) };
    let dir = (far - near) / len;
    let (n1, n2) = (b.planar_normal(faces[0]), b.planar_normal(faces[1]));
    let c12 = n1.dot(n2);
    if (1.0 - c12).abs() < 1e-9 || (1.0 + c12).abs() < 1e-9 {
        return Err(unsupported());
    }
    let phi = c12.clamp(-1.0, 1.0).acos();
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]).ok_or_else(|| BlendError::code("missingEdge"))?;
    let tdir = if b.coedge(co).rev { -dir_e } else { dir_e };
    let (leg1, leg2) = (n1.cross(tdir).normalize(), -n2.cross(tdir).normalize());
    let convex = n1.cross(tdir).dot(n2) < 0.0;
    let s = if convex { -1.0 } else { 1.0 };
    let name = FaceName::blend(feature, k);
    // radius at distance `t` from the near end
    let slope = (v.radii[1] - v.radii[0]) / len;
    let rho_slope = (v.rhos[1] - v.rhos[0]) / len;
    let rad_at = |t: f64| v.radii[0] + slope * t;
    let rho_at = |t: f64| v.rhos[0] + rho_slope * t;
    // contact points of the section at distance t
    let contacts = |t: f64| -> Result<(Vec3, Vec3, Vec3, f64), BlendError> {
        let p = near + dir * t;
        let r = rad_at(t);
        match v.profile {
            FilletProfile::Curvature(sh) => {
                let d = super::g2::tangent_distance(r, sh, phi).ok_or_else(unsupported)?;
                Ok((p, p + leg1 * d, p + leg2 * d, r))
            }
            _ => {
                let rr = if v.profile == FilletProfile::Chord { r / (2.0 * (0.5 * phi).sin()) } else { r };
                let off = (n1 + n2) * (s * rr / (1.0 + c12));
                Ok((p, p + off - n1 * (s * rr), p + off - n2 * (s * rr), rr))
            }
        }
    };
    // the contact lines must stay on their faces (checked at both ends)
    for t in [0.0, len] {
        let (_, ta, tb, _) = contacts(t)?;
        for (f, q) in [(faces[0], ta), (faces[1], tb)] {
            if point_in_face(b, f, q, tol) == Where::Out {
                return Err(BlendError::code("blendTooLarge"));
            }
        }
    }
    // stations: the edge ends, extended past them when the blend cuts
    let rmax = v.radii[0].max(v.radii[1]);
    let base_margin = if convex { rmax.max(len * 0.05) + 1.0 } else { 0.0 };
    let limit = |r_end: f64, grows_outward: f64| -> f64 {
        // radius extrapolated by `grows_outward` per unit: keep it above a fifth of the end value
        if grows_outward < 0.0 {
            (0.8 * r_end / -grows_outward).min(base_margin)
        } else {
            base_margin
        }
    };
    let (m0, m1) = (limit(v.radii[0], -slope), limit(v.radii[1], slope));
    // a conic whose rho changes is not a ruled family (the weights would be interpolated, not rho):
    // sample it at nine stations instead of two
    let n_st = if matches!(v.profile, FilletProfile::Conic(_)) && (v.rhos[0] - v.rhos[1]).abs() > 1e-9 { 9 } else { 2 };
    let stations: Vec<f64> = (0..n_st).map(|i| -m0 + (len + m1 + m0) * i as f64 / (n_st - 1) as f64).collect();
    let mut profs: Vec<ProfileIn> = vec![];
    for &t in &stations {
        let (p, ta, tb, rr) = contacts(t.clamp(-m0, len + m1))?;
        let fr = Frame::from_normal(p, dir);
        let uv = |q: Vec3| {
            let w = fr.world_to_uv(q);
            Vec2::new(w.x, w.y)
        };
        let (cp, t1, t2) = (uv(p), uv(ta), uv(tb));
        let seg = match v.profile {
            FilletProfile::Conic(_) => super::g2::conic_segment(rho_at(t.clamp(0.0, len)), t1, cp, t2).ok_or_else(unsupported)?,
            FilletProfile::Curvature(sh) => super::g2::g2_segment(rad_at(t), sh, phi, t1, cp, t2).ok_or_else(unsupported)?,
            _ => {
                let off = (n1 + n2) * (s * rr / (1.0 + c12));
                let cc = uv(p + off);
                if (t1 - cc).cross(t2 - cc) > 0.0 {
                    ProfileSeg::arc("arc", cc, t1, t2)
                } else {
                    ProfileSeg::arc("arc", cc, t2, t1).reversed()
                }
            }
        };
        let lp = ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), seg]);
        let segs: Vec<SegIn> = lp
            .segs
            .iter()
            .map(|s| SegIn { tag: s.tag.clone(), curve: curve2_to_3d(&fr, &s.curve, Vec3::ZERO), t0: s.t0, t1: s.t1, rev: s.rev })
            .collect();
        profs.push(ProfileIn { frame: fr, closed: true, segs });
    }
    let mut tool = loft(feature, &profs, &LoftOpts::default()).map_err(|_| BlendError::code("blendFailed"))?.body;
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
    let op = if convex { Op::Cut } else { Op::Union };
    // order: untouched; edge faces (ref order), end faces (start, end); other touched faces; blend
    let ends: Vec<FaceId> = [ed.v0, ed.v1]
        .iter()
        .flat_map(|&vtx| {
            b.edge_ids()
                .filter(move |&x| b.edge(x).v0 == vtx || b.edge(x).v1 == vtx)
                .flat_map(|x| b.edge_faces(x))
                .filter(|f| !faces.contains(f))
                .collect::<Vec<_>>()
        })
        .collect();
    let mut hint: Vec<String> = faces.iter().map(|&f| b.face_id_str(f)).collect();
    for f in ends {
        let id = b.face_id_str(f);
        if !hint.contains(&id) {
            hint.push(id);
        }
    }
    let blend_f = tool.face_ids().find(|&f| tool.face(f).name == name);
    let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> { (Some(g) == blend_f && faces.contains(&f)).then(Vec::new) };
    apply_tool_hooked(b, &tool, op, &name, hint, tol, Some(&hook)).map(|x| x.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bx() -> Body {
        crate::boolean::tests::box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }

    fn pick(x: f64) -> Vec<(String, String, Vec3)> {
        vec![("F2.C1".into(), "F2.S.e1".into(), Vec3::new(x, 10.0, 0.0))]
    }

    fn vol(b: &Body) -> f64 {
        ck_topo::mass::mass_props(b).volume
    }

    fn spec(profile: FilletProfile, radii: [f64; 2]) -> VarSpec {
        VarSpec { radii, profile, rhos: [0.5, 0.5] }
    }

    /// Oracle: radii [1, 3] (circular 7962.80235, chord 7981.40117, conic rho .5 7971.11111,
    /// curvature shape .5 7953.84539); the pick end decides which end gets the first radius.
    #[test]
    fn linear_radius_volumes() {
        let b = bx();
        for x in [4.0, 36.0] {
            let out = variable_fillet(&b, &pick(x), spec(FilletProfile::Circular, [1.0, 3.0]), "F3", 1e-6).expect("var");
            assert_eq!(out.num_faces(), 7);
            assert!((vol(&out) - 7962.80235).abs() < 1e-4, "{}", vol(&out));
            assert!(ck_topo::check::check(&out, 1e-6).is_empty());
        }
        let out = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Chord, [1.0, 3.0]), "F3", 1e-6).expect("chord");
        assert!((vol(&out) - 7981.40117).abs() < 1e-4, "{}", vol(&out));
        let out = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Conic(0.5), [1.0, 3.0]), "F3", 1e-6).expect("conic");
        assert!((vol(&out) - 7971.11111).abs() < 1e-3, "{}", vol(&out));
        let out = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Curvature(0.5), [1.0, 3.0]), "F3", 1e-6).expect("curv");
        assert!((vol(&out) - 7953.84539).abs() < 0.02, "{}", vol(&out));
        // the first radius sits at the near end: the contact on the side face is 1 from the edge at x = 0
        let out = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Circular, [1.0, 3.0]), "F3", 1e-6).unwrap();
        let blend = out.face_ids().find(|&f| out.face_id_str(f) == "F3.B0").expect("blend face");
        let near0 = out.face_edges(blend).into_iter().map(|e| out.edge(e)).any(|ed| (0..=8).any(|i| {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 8.0);
            p.x.abs() < 1e-6 && (p.y - 9.0).abs() < 1e-6 && p.z.abs() < 1e-6
        }));
        assert!(near0, "contact at x=0 is 1 mm from the edge");
    }

    #[test]
    fn errors_and_big_radii() {
        let b = bx();
        let e = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Circular, [0.0, 3.0]), "F3", 1e-6).unwrap_err();
        assert_eq!(e.code, "invalidValue");
        // oracle varBig [1, 9]: 7739.61644; varTooBig [1, 12]: blendTooLarge
        let out = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Circular, [1.0, 9.0]), "F3", 1e-6).expect("big");
        assert!((vol(&out) - 7739.61644).abs() < 1e-3, "{}", vol(&out));
        let e = variable_fillet(&b, &pick(4.0), spec(FilletProfile::Circular, [1.0, 12.0]), "F3", 1e-6).unwrap_err();
        assert_eq!(e.code, "blendTooLarge");
    }
}
