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
    // identical holes share one feature
    let key = |h: &Hole| (q(h.axis_in.x), q(h.axis_in.y), q(h.axis_in.z), q(h.d), h.through, if h.through { 0 } else { q(h.depth) }, h.tip, h.cbore.map(|(d, t)| (q(d), q(t))), h.csk.map(|(d, a, _)| (q(d), q(a))));
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
        o = if h0.through { o.put("end", "through") } else { o.put("end", "blind").put("depth", n(h0.depth)) };
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
        notes.push(format!("{} x {} d={:.4}{}{}", hv.len(), kind, h0.d, if h0.through { ", through".to_string() } else { format!(", depth {:.4}", h0.depth) }, listed));
    }
    Ok(notes.join("; "))
}
