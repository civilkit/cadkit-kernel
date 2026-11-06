//! A rib (`Rib`): a thin wall standing on an open line profile that runs on to the body. The file keeps two parallel
//! cap faces (the wall's two sides, `thickness` apart, the sketch plane midway) and the faces the profile's lines
//! stand for (planes square to the caps: the wall's free edge). The profile is the chain of those faces' traces in
//! the mid plane; the kernel's `rib` extends it past both ends and fills the space to the body.

use crate::{face_points, Cx};
use ck_geom::Vec3;
use ck_recog::emit::{n, Sk};
use ck_recog::model::Kind;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::FaceId;

type P2 = (f64, f64);

pub fn rebuild(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let m = cx.m;
    let planes: Vec<(FaceId, Vec3, f64)> = faces
        .iter()
        .filter_map(|&g| match m.kind(g) {
            Kind::Plane { n, d } => Some((g, *n, *d)),
            _ => None,
        })
        .collect();
    if planes.len() != faces.len() {
        return Err("rib with a curved face".into());
    }
    // the caps: an antiparallel pair of planes; the pair that spans the most area (the farthest-reaching extent)
    let mut best: Option<(f64, usize, usize)> = None;
    for i in 0..planes.len() {
        for j in i + 1..planes.len() {
            if (planes[i].1 + planes[j].1).len() > 1e-7 {
                continue;
            }
            let sep = planes[i].2 + planes[j].2;
            if sep.abs() < 1e-6 {
                continue;
            }
            let (a, b) = m.b.bbox_of_faces(&[planes[i].0, planes[j].0]);
            let ext = (b - a).len();
            if best.map_or(true, |x| ext > x.0) {
                best = Some((ext, i, j));
            }
        }
    }
    let Some((_, i, j)) = best else { return Err("rib without two parallel cap faces".into()) };
    let (nc, t) = (planes[i].1, planes[i].2 + planes[j].2);
    let thickness = t.abs();
    // the mid plane: n . x = (d_i - d_j) / 2
    let mid = (planes[i].2 - planes[j].2) / 2.0;
    let fr = cx.frame(nc, nc * mid);
    let free: Vec<usize> = (0..planes.len()).filter(|&k| k != i && k != j).collect();
    if free.is_empty() {
        return Err("rib without a free-edge face".into());
    }
    // the trace of each free face in the mid plane: a line through the extreme points along it
    let mut segs: Vec<(P2, P2, FaceId)> = vec![];
    for &k in &free {
        let (g, nk, _) = planes[k];
        if nk.dot(nc).abs() > 1e-7 {
            return Err("rib face neither cap nor free edge".into());
        }
        let n2 = (nk.dot(fr.x), nk.dot(fr.y));
        let l = n2.0.hypot(n2.1);
        let d = (-n2.1 / l, n2.0 / l);
        let pts: Vec<P2> = face_points(m, g).into_iter().map(|p| fr.xy(p)).collect();
        let o = pts[0];
        let tt = |q: &P2| (q.0 - o.0) * d.0 + (q.1 - o.1) * d.1;
        let (lo, hi) = pts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, q| (a.0.min(tt(q)), a.1.max(tt(q))));
        segs.push(((o.0 + lo * d.0, o.1 + lo * d.1), (o.0 + hi * d.0, o.1 + hi * d.1), g));
    }
    // chain them end to end
    let scale = segs.iter().map(|s| ((s.1 .0 - s.0 .0).hypot(s.1 .1 - s.0 .1))).fold(0.0, f64::max);
    let near = |a: P2, b: P2| (a.0 - b.0).hypot(a.1 - b.1) < 1e-4 * (1.0 + scale);
    let mut chain: Vec<(P2, P2, FaceId)> = vec![segs.remove(0)];
    while !segs.is_empty() {
        let (head, tail) = (chain[0].0, chain[chain.len() - 1].1);
        let Some(k) = (0..segs.len()).find(|&k| near(segs[k].0, tail) || near(segs[k].1, tail) || near(segs[k].0, head) || near(segs[k].1, head)) else {
            return Err("rib faces do not form one chain".into());
        };
        let s = segs.remove(k);
        if near(s.0, tail) {
            chain.push(s);
        } else if near(s.1, tail) {
            chain.push((s.1, s.0, s.2));
        } else if near(s.1, head) {
            chain.insert(0, s);
        } else {
            chain.insert(0, (s.1, s.0, s.2));
        }
    }
    let sid = cx.tree.ids.next();
    let mut sk = Sk::new(sid.clone(), fr.plane.clone());
    let eid = format!("F{}", cx.tree.ids.0 + 1);
    let mut pts: Vec<(P2, String)> = vec![];
    let mut named: Vec<(FaceId, String)> = vec![];
    for (a, b, g) in &chain {
        let mut pt = |sk: &mut Sk, p: P2| -> String {
            if let Some((_, id)) = pts.iter().find(|(q, _)| near(*q, p)) {
                return id.clone();
            }
            let id = sk.point(p.0, p.1);
            pts.push((p, id.clone()));
            id
        };
        let (pa, pb) = (pt(&mut sk, *a), pt(&mut sk, *b));
        let e = sk.line(&pa, &pb, false);
        named.push((*g, format!("{eid}.S.{e}r")));
    }
    cx.tree.push(sk.json());
    let eid2 = cx.tree.ids.next();
    debug_assert_eq!(eid, eid2);
    cx.tree.push(ck_json::Obj::new().put("type", "rib").put("id", eid.as_str()).put("sketch", sid.as_str()).put("thickness", n(thickness)).build());
    for (g, name) in named {
        cx.names.entry(g).or_insert(name);
    }
    // the cap on the +normal side of the sketch plane is C1
    for k in [i, j] {
        let up = planes[k].1.dot(fr.z) > 0.0;
        cx.names.entry(planes[k].0).or_insert_with(|| format!("{eid}.{}", if up { "C1" } else { "C0" }));
    }
    Ok(format!("thickness {:.4}; {} line(s) of the free edge", thickness, chain.len()) + &format!(" ({})", f.name))
}
