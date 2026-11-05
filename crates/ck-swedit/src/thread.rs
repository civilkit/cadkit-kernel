//! Modelled threads (SolidWorks `Thread` features that made faces of their own).
//!
//! A cosmetic thread annotates faces of earlier features and has no geometry (`crate::cosmetic`). A modelled thread
//! sweeps a profile along a helix about the axis of a host cylinder (a shaft or a bore) and leaves a few free-form
//! faces (one per profile side, sampled as chains of quadratic pieces along the helix) plus a planar cap where the
//! sweep starts. Read back from them:
//!
//! * **axis**: the axis of the cylinder face next to the thread faces that the thread faces wind around (every
//!   iso-line of a thread face keeps its distance to the axis);
//! * **profile**: the thread faces at the sweep's start end form a chain of profile sides in one axial plane; its two
//!   ends lie on the host cylinder, which closes the profile (extended a little into the host material so the union
//!   or cut leaves no coincident faces);
//! * **screw**: an iso-line along the sweep gives the angle and the advance per sample: pitch, hand, turns and rise;
//! * **add or cut**: the host's convexity and whether the profile reaches out to a larger or smaller radius than the
//!   host: a shaft with a ridge, or a bore with a groove, are `add` / `cut` accordingly.
//!
//! It is emitted as the kernel's `thread` feature in its `explicit` form (a swept profile on a screw motion), which
//! the regen checks as any other step (`crate::prefix`).

use crate::Cx;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{n, vec3};
use ck_recog::model::{dist_to_line, Kind, Model};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;

/// The faces of the feature itself (the ones no earlier feature made).
pub fn own_faces(cx: &Cx, f: &SwFeature, faces: &[FaceId]) -> Vec<FaceId> {
    faces.iter().copied().filter(|&g| cx.tag_of(g) == Some(f.id)).collect()
}

/// What the faces of a modelled thread say.
#[derive(Debug, Clone)]
pub struct Thread {
    /// A point on the axis and the unit axis.
    pub o: Vec3,
    pub axis: Vec3,
    /// Unit radial direction of the start profile.
    pub x: Vec3,
    /// The closed start profile (radius, axial coordinate from `o`).
    pub profile: Vec<(f64, f64)>,
    /// Signed turn (radians about `axis`) and advance along `axis` from the start to the far end.
    pub angle: f64,
    pub rise: f64,
    pub pitch: f64,
    pub right: bool,
    /// Material is added (a ridge on a shaft, a ridge inside a bore) rather than cut.
    pub add: bool,
    pub host_r: f64,
    /// Turns at the far end to unite as a piece of their own (the thread ends against a face that trimmed it).
    pub tail: Option<f64>,
}

const NS: usize = 2000;

/// The (across0, across1, along0, along1) domain of a thread face: `across` runs over the profile side, `along` over
/// the sweep. A free-form patch (as the SolidWorks file has them) has them as (u, v), a screw patch as (v, u).
fn dom(s: &Surface) -> Option<(f64, f64, f64, f64)> {
    let ((u0, u1), (v0, v1)) = match s {
        Surface::BSpline(_) | Surface::Screw { .. } => s.domain(),
        _ => return None,
    };
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) {
        return None;
    }
    Some(if matches!(s, Surface::Screw { .. }) { (v0, v1, u0, u1) } else { (u0, u1, v0, v1) })
}

/// The point at (`across`, `along`) of a thread face.
fn pe(s: &Surface, across: f64, along: f64) -> Vec3 {
    if matches!(s, Surface::Screw { .. }) {
        s.eval(along, across)
    } else {
        s.eval(across, along)
    }
}

/// Chain the segments (end to end, either way round) into one polyline.
fn chain(segs: &[(Vec3, Vec3)], tol: f64) -> Option<Vec<Vec3>> {
    let mut used = vec![false; segs.len()];
    used[0] = true;
    let mut pts = vec![segs[0].0, segs[0].1];
    let mut left = segs.len() - 1;
    while left > 0 {
        let (head, tail) = (pts[0], *pts.last().unwrap());
        let mut hit = None;
        'f: for (i, s) in segs.iter().enumerate() {
            if used[i] {
                continue;
            }
            for (a, b) in [(s.0, s.1), (s.1, s.0)] {
                if a.dist(tail) < tol {
                    hit = Some((i, false, b));
                    break 'f;
                }
                if a.dist(head) < tol {
                    hit = Some((i, true, b));
                    break 'f;
                }
            }
        }
        let (i, front, p) = hit?;
        used[i] = true;
        left -= 1;
        if front {
            pts.insert(0, p);
        } else {
            pts.push(p);
        }
    }
    Some(pts)
}

/// Read the thread back from its own faces `own` (on the model `m`).
pub fn analyse(m: &Model, own: &[FaceId]) -> Result<Thread, String> {
    let splines: Vec<FaceId> = own.iter().copied().filter(|&g| dom(&m.b.face(g).surface).is_some()).collect();
    let caps: Vec<FaceId> = own.iter().copied().filter(|&g| matches!(m.kind(g), Kind::Plane { .. })).collect();
    if splines.is_empty() {
        return Err("the thread's faces are not free-form or screw patches".into());
    }
    if splines.len() + caps.len() != own.len() {
        return Err("a face of the thread is neither a free-form patch nor a plane".into());
    }
    // host cylinders: next to the thread faces, made by no face of the thread
    let mut hosts: Vec<(Vec3, Vec3, f64, bool)> = vec![];
    for &g in own {
        for e in m.edges(g) {
            if let Some(h) = m.across(e, g) {
                if own.contains(&h) {
                    continue;
                }
                if let Kind::Cyl { o, a, r, convex } = m.kind(h) {
                    if !hosts.iter().any(|x| (x.2 - r).abs() < 1e-9 && x.1.cross(*a).len() < 1e-9 && dist_to_line(*o, x.0, x.1) < 1e-9) {
                        hosts.push((*o, *a, *r, *convex));
                    }
                }
            }
        }
    }
    if hosts.is_empty() {
        return Err("no cylinder next to the thread faces to take the axis from".into());
    }
    let f0 = m.b.face(splines[0]);
    let (u0, _, v0, v1) = dom(&f0.surface).unwrap();
    // the host the thread winds around: constant distance to its axis along an iso-line of the first patch
    let line: Vec<Vec3> = (0..=NS).map(|i| pe(&f0.surface, u0, v0 + (v1 - v0) * i as f64 / NS as f64)).collect();
    let host = hosts
        .iter()
        .copied()
        .find(|&(o, a, _, _)| {
            let rs: Vec<f64> = line.iter().map(|&p| dist_to_line(p, o, a)).collect();
            let (lo, hi) = rs.iter().fold((f64::MAX, f64::MIN), |(l, h), &r| (l.min(r), h.max(r)));
            hi - lo < 1e-6 * (1.0 + hi)
        })
        .ok_or("the thread faces do not wind around the axis of a neighbouring cylinder")?;
    let (o, a, r_h, convex) = host;
    let tol = 1e-6 * (1.0 + r_h);
    // the profile chain at either end of the sweep
    let ends = |v: f64| -> Option<Vec<Vec3>> {
        let segs: Vec<(Vec3, Vec3)> = splines
            .iter()
            .map(|&g| {
                let s = &m.b.face(g).surface;
                let (u0, u1, _, _) = dom(s).unwrap();
                let (vlo, vhi) = (dom(s).unwrap().2, dom(s).unwrap().3);
                let vv = if (v - v0).abs() < (v - v1).abs() { vlo } else { vhi };
                (pe(s, u0, vv), pe(s, u1, vv))
            })
            .collect();
        chain(&segs, tol)
    };
    let (c0, c1) = (ends(v0), ends(v1));
    // the start end: the one whose profile meets a cap of the thread (else the first parameter end)
    let cap_pts: Vec<Vec3> = caps.iter().flat_map(|&g| m.b.face_edges(g).into_iter().flat_map(|e| [m.p0(e), m.p1(e)])).collect();
    let score = |c: &Option<Vec<Vec3>>| c.as_ref().map_or(-1i32, |c| c.iter().filter(|p| cap_pts.iter().any(|q| q.dist(**p) < 1e-5 * (1.0 + r_h))).count() as i32);
    let first = score(&c1) > score(&c0);
    let (start_v, far_v) = if first { (v1, v0) } else { (v0, v1) };
    let prof3 = if first { c1 } else { c0 }.ok_or("the thread faces do not chain into one profile at the start of the sweep")?;
    if prof3.len() < 2 {
        return Err("the thread profile has no sides".into());
    }
    // axial coordinate / radius, and the plane of the profile
    let s_of = |p: Vec3| (p - o).dot(a);
    let rad = |p: Vec3| (p - o) - a * s_of(p);
    let x = rad(prof3[0]).try_normalize().ok_or("the profile starts on the axis")?;
    let e2 = a.cross(x);
    if prof3.iter().any(|&p| (p - o).dot(e2).abs() > 1e-5 * (1.0 + r_h)) {
        return Err("the start profile is not in one axial plane".into());
    }
    let pr: Vec<(f64, f64)> = prof3.iter().map(|&p| (rad(p).len(), s_of(p))).collect();
    // the screw: the iso-line of the first patch (start end first), the angle unwrapped
    let pts: Vec<Vec3> = (0..=NS).map(|i| pe(&f0.surface, u0, start_v + (far_v - start_v) * i as f64 / NS as f64)).collect();
    let mut th = vec![0.0f64];
    let mut prev = 0.0f64;
    for p in &pts[1..] {
        let d = rad(*p);
        let ang = d.dot(e2).atan2(d.dot(x));
        let mut dlt = ang - prev;
        while dlt > std::f64::consts::PI {
            dlt -= std::f64::consts::TAU;
        }
        while dlt < -std::f64::consts::PI {
            dlt += std::f64::consts::TAU;
        }
        if dlt.abs() > 2.0 {
            return Err("the thread winds too fast to sample (more than 3 turns per sample step)".into());
        }
        th.push(th.last().unwrap() + dlt);
        prev = ang;
    }
    let ss: Vec<f64> = pts.iter().map(|&p| s_of(p)).collect();
    let (nn, mt, ms) = (th.len() as f64, th.iter().sum::<f64>() / th.len() as f64, ss.iter().sum::<f64>() / ss.len() as f64);
    let (mut cov, mut var) = (0.0, 0.0);
    for i in 0..th.len() {
        cov += (th[i] - mt) * (ss[i] - ms);
        var += (th[i] - mt) * (th[i] - mt);
    }
    let _ = nn;
    if var < 1e-12 {
        return Err("the thread does not turn".into());
    }
    let slope = cov / var;
    let (angle, rise) = (*th.last().unwrap(), ss.last().unwrap() - ss[0]);
    // a helix: the advance is linear in the angle
    let worst = th.iter().zip(&ss).map(|(t, s)| (s - ss[0] - slope * t).abs()).fold(0.0, f64::max);
    if worst > 1e-4 * (1.0 + rise.abs()) {
        return Err(format!("the thread is not a helix of constant pitch (deviation {worst:.2e})"));
    }
    // the profile reaches out to larger or smaller radii than the host
    let (r_far, _) = pr.iter().map(|p| (p.0, (p.0 - r_h).abs())).fold((r_h, 0.0), |b, c| if c.1 > b.1 { c } else { b });
    let out = r_far > r_h;
    if (r_far - r_h).abs() < 1e-6 * (1.0 + r_h) {
        return Err("the thread profile does not leave the host cylinder".into());
    }
    let pitch = std::f64::consts::TAU * slope.abs();
    // The far end: with no cap of the thread there, another face trimmed the thread (the sweep ran on into material,
    // e.g. up to a shoulder). Run the sweep half a turn further when that stays inside the body, so the tool's end
    // lies well inside the material instead of touching the face that trimmed it (a tangent contact the boolean
    // cannot close).
    let far_chain = if first { ends(v0) } else { ends(v1) };
    let (mut angle, mut rise) = (angle, rise);
    let trimmed = score(&far_chain) == 0;
    if trimmed {
        let dth = 0.5 * std::f64::consts::TAU * angle.signum();
        let ds = slope * dth;
        let th_far = angle + dth;
        let (cr, cs) = (pr.iter().map(|p| p.0).sum::<f64>() / pr.len() as f64, pr.iter().map(|p| p.1).sum::<f64>() / pr.len() as f64);
        let solid = ck_ops::boolean::classify::Solid::new(m.b, 1e-6);
        let inside = pr.iter().all(|&(r, s)| {
            let (r, s) = (cr + 0.8 * (r - cr), cs + 0.8 * (s - cs));
            let q = o + a * (s + rise + ds) + (x * th_far.cos() + e2 * th_far.sin()) * r;
            solid.classify(q, a) == ck_ops::boolean::classify::Class::In
        });
        if inside {
            angle += dth;
            rise += ds;
        }
    }
    // Close the profile on the host. Its two ends lie on the host cylinder; they are replaced by points a little
    // beyond it (into the host material, away from the ridge) on the lines of the sides that end there, so the
    // sides cross the host surface instead of ending in a rim exactly on it (a contact the boolean cannot close).
    let (ra, rb) = (pr[0].0, pr.last().unwrap().0);
    let mut profile = pr.clone();
    let near = |r: f64| (r - r_h).abs() < 1e-5 * (1.0 + r_h);
    let eps = 0.03 * pitch;
    let reach = |p: (f64, f64), q: (f64, f64)| -> Option<(f64, f64)> {
        // p on the host, q the next point of the profile: p moved along q->p to radius r_h -+ eps
        let dr = p.0 - q.0;
        let want = if out { -eps } else { eps };
        (dr.abs() > 1e-9 && dr.signum() == want.signum()).then(|| {
            let t = want / dr;
            (p.0 + (p.0 - q.0) * t, p.1 + (p.1 - q.1) * t)
        })
    };
    if near(ra) && near(rb) && pr.len() >= 3 {
        let (e0, e1) = (reach(pr[0], pr[1]), reach(pr[pr.len() - 1], pr[pr.len() - 2]));
        if let (Some(e0), Some(e1)) = (e0, e1) {
            profile.remove(0);
            profile.pop();
            profile.insert(0, e0);
            profile.push(e1);
        } else {
            let r_in = if out { r_h - eps } else { r_h + eps };
            profile.insert(0, (r_in, pr[0].1));
            profile.push((r_in, pr.last().unwrap().1));
        }
    }
    Ok(Thread { o, axis: a, x, profile, angle, rise, pitch, right: slope > 0.0, add: convex == out, host_r: r_h, tail: (trimmed && angle.abs() > 6.0 * std::f64::consts::TAU).then_some(3.0) })
}

/// Does the thread add material (`Some(true)`), cut it (`Some(false)`), or is it not readable.
pub fn adds(m: &Model, own: &[FaceId]) -> Option<bool> {
    analyse(m, own).ok().map(|t| t.add)
}

/// The kernel's `explicit` thread feature of a read-back thread.
pub fn feature(id: &str, name: &str, t: &Thread) -> Json {
    let ex = Obj::new()
        .put("o", vec3(t.o))
        .put("axis", vec3(t.axis))
        .put("x", vec3(t.x))
        .put("profile", Json::Arr(t.profile.iter().map(|p| Json::Arr(vec![n(p.0), n(p.1)])).collect()))
        .put("angle", n(t.angle))
        .put("rise", n(t.rise))
        .put("op", if t.add { "add" } else { "cut" });
    let ex = match t.tail {
        Some(k) => ex.put("tail", n(k)),
        None => ex,
    }
    .build();
    Obj::new().put("type", "thread").put("id", id).put("name", name).put("explicit", ex).build()
}

/// Read a modelled thread back as one `thread` feature.
pub fn rebuild(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let own = own_faces(cx, f, faces);
    if own.is_empty() {
        return Err("the thread has no faces of its own".into());
    }
    let t = analyse(cx.m, &own)?;
    let id = cx.tree.ids.next();
    cx.tree.push(feature(&id, &f.name, &t));
    let pitch_note = crate::dim_note(cx, f, "D7", "pitch", t.pitch);
    Ok(format!(
        "modelled {} thread: {} of a {}-sided profile, {pitch_note}, {:.3} turns, {}-handed, on a cylinder of radius {:.4}",
        if t.add { "ridge" } else { "groove" },
        if t.add { "added" } else { "cut" },
        t.profile.len(),
        t.angle.abs() / std::f64::consts::TAU,
        if t.right { "right" } else { "left" },
        t.host_r
    ))
}
