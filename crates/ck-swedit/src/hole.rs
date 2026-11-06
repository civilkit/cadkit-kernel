//! Hole wizard features: the `ck-recog` hole groups whose faces the feature made.

use crate::{dim, Cx};
use ck_json::Obj;
use ck_recog::emit::{n, Sk};
use ck_recog::holes::Hole;
use ck_recog::model::{principal, PlaneSpec};
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;
use std::collections::HashSet;

fn q(v: f64) -> i64 {
    (v * 1e5).round() as i64
}

/// Read a hole wizard feature back: one `hole` feature per group of identical holes (several points per sketch).
pub fn hole(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], holes: &[Hole]) -> Result<String, String> {
    hole_with(cx, f, faces, holes, false)
}

/// The same read from the feature's own faces alone, for holes that later features cut into (cross holes in the
/// walls, a counterbore from the other side).
pub fn hole_own(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    hole_with(cx, f, faces, &[], true)
}

fn hole_with(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], holes: &[Hole], tolerant: bool) -> Result<String, String> {
    let own: HashSet<FaceId> = faces.iter().copied().collect();
    let mut mine: Vec<&Hole> = holes.iter().filter(|h| h.faces.iter().all(|g| own.contains(g))).collect();
    // later features cut into the walls or the far end of the holes (cross holes, a counterbore from the other side):
    // read the holes off the feature's own faces alone
    let own_holes: Vec<Hole>;
    if tolerant {
        own_holes = ck_recog::holes::find_holes_in(cx.m, &mut HashSet::new(), Some(&own));
        if own_holes.iter().flat_map(|h| h.faces.iter()).collect::<HashSet<_>>().len() == own.len() {
            mine = own_holes.iter().collect();
        }
    }
    let covered: HashSet<FaceId> = mine.iter().flat_map(|h| h.faces.iter().copied()).collect();
    if mine.is_empty() {
        return Err("no hole group on the feature's faces".into());
    }
    if covered.len() != own.len() {
        return Err(format!("{} of {} faces are not part of a hole group", own.len() - covered.len(), own.len()));
    }
    // a hole open at both ends is a through-all hole only when the body has no material further along its axis, in
    // its bore: else it ended at its exit (the file's `up to next`, or a blind depth to the far face), and a
    // through-all hole would cut the material beyond the gap too
    let m = cx.m;
    let through_all = |h: &Hole| h.through && !material_beyond(m, h);
    // identical holes share one feature
    let key = |h: &Hole| (q(h.axis_in.x), q(h.axis_in.y), q(h.axis_in.z), q(h.d), through_all(h), if through_all(h) { 0 } else { q(h.depth) }, h.tip, h.cbore.map(|(d, t)| (q(d), q(t))), h.csk.map(|(d, a, _)| (q(d), q(a))));
    let mut groups: Vec<(Vec<&Hole>,)> = vec![];
    for h in &mine {
        match groups.iter_mut().find(|g| key(g.0[0]) == key(h)) {
            Some(g) => g.0.push(h),
            None => groups.push((vec![h],)),
        }
    }
    let mut notes = vec![];
    for (hv,) in &groups {
        let h0 = hv[0];
        let a_in = h0.axis_in;
        // the sketch normal: the standard plane's axis when principal (then the hole may run along +z: flip), else
        // the entry face normal (-axis), so the hole runs along -z of the sketch
        let z = match principal(a_in) {
            Some((i, _)) => PlaneSpec::for_axis(i).zs,
            None => -a_in,
        };
        // the entry planes of identical holes sit at one level
        let level_ok = hv.iter().all(|h| ((h.centre - h0.centre).dot(z)).abs() < 1e-5);
        if !level_ok {
            return Err("identical holes at different levels".into());
        }
        let fr = cx.frame(z, h0.centre);
        let flip = a_in.dot(fr.z) > 0.0;
        let sid = cx.tree.ids.next();
        let mut sk = Sk::new(sid.clone(), fr.plane.clone());
        for h in hv {
            let (x, y) = fr.xy(h.centre);
            let pid = sk.point(x, y);
            sk.point_entity(&pid);
        }
        cx.tree.push(sk.json());
        let hid = cx.tree.ids.next();
        let kind = if h0.cbore.is_some() {
            "cbore"
        } else if h0.csk.is_some() {
            "countersink"
        } else {
            "simple"
        };
        let mut o = Obj::new().put("type", "hole").put("id", hid.as_str()).put("sketch", sid.as_str()).put("kind", kind).put("diameter", n(h0.d));
        let all = through_all(h0);
        o = if all { o.put("end", "through") } else { o.put("end", "blind").put("depth", n(h0.depth)) };
        if h0.tip {
            o = o.put("tip", true);
        }
        if flip {
            o = o.put("flip", true);
        }
        if let Some((cd, ch)) = h0.cbore {
            o = o.put("cboreDiameter", n(cd)).put("cboreDepth", n(ch));
        }
        if let Some((hd, ang, _)) = h0.csk {
            o = o.put("cskDiameter", n(hd)).put("cskAngle", n(ang));
        }
        cx.tree.push(o.build());
        for h in hv {
            for &g in &h.faces {
                cx.names.insert(g, format!("{hid}.S.h1"));
            }
        }
        let listed = dim(cx, f, "Hole Dia").map(|v| if (v - h0.d).abs() < 1e-3 * (1.0 + h0.d) { " = Hole Dia.".to_string() } else { format!(" (file lists Hole Dia. = {v})") }).unwrap_or_default();
        notes.push(format!("{} x {} d={:.4}{}{}", hv.len(), kind, h0.d, if all { ", through".to_string() } else if h0.through { format!(", depth {:.4} to its exit (material further along its axis)", h0.depth) } else { format!(", depth {:.4}", h0.depth) }, listed));
    }
    Ok(notes.join("; "))
}

/// The body has material along the hole's axis outside the hole, inside its bore: past its exit or behind its
/// entry (a through-all hole cuts its whole line). Sampled on the axis and on a ring just inside the bore, from either
/// end of the hole to the body's extent.
fn material_beyond(m: &ck_recog::model::Model, h: &Hole) -> bool {
    use ck_ops::contain::{Containment, Loc};
    let b = m.b;
    let a = h.axis_in;
    let (lo, hi) = b.bbox();
    let ts: Vec<f64> = [lo.x, hi.x].iter().flat_map(|&x| [lo.y, hi.y].map(move |y| (x, y))).flat_map(|(x, y)| [lo.z, hi.z].map(move |z| ck_geom::Vec3::new(x, y, z))).map(|p| (p - h.centre).dot(a)).collect();
    let (near, far) = ts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |acc, &t| (acc.0.min(t), acc.1.max(t)));
    let u = if a.x.abs() < 0.9 { ck_geom::Vec3::X } else { ck_geom::Vec3::Y };
    let (u, v) = (a.cross(u).normalize(), a.cross(a.cross(u)).normalize());
    let r = 0.45 * h.d;
    let ring = [ck_geom::Vec3::ZERO, u * r, -u * r, v * r, -v * r];
    let fc = Containment::new(b, 1e-6);
    let n = 200;
    let hit = |s: f64| ring.iter().any(|&o| fc.classify_point(h.centre + a * s + o, 1e-6) == Loc::In);
    (far > h.depth + 1e-6 && (1..n).any(|k| hit(h.depth + (far - h.depth) * k as f64 / n as f64))) || (near < -1e-6 && (1..n).any(|k| hit(near * k as f64 / n as f64)))
}
