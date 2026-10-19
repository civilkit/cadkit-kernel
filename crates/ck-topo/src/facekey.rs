//! Exact content keys of faces for the memos of pure functions of a face's geometry (display mesh, wire box):
//! words of f64 bits, every variable-length part prefixed with its length, so equal keys mean bit-identical inputs
//! (unlike `Surface::push_key`, a B-spline net is written out, not stood in for by its hash).

use crate::body::{Body, FaceId};
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::Vec2;

pub fn key_f64s(out: &mut Vec<u64>, v: &[f64]) {
    out.push(v.len() as u64);
    out.extend(v.iter().map(|x| x.to_bits()));
}

/// Exact content of a surface.
pub fn key_surface(out: &mut Vec<u64>, s: &Surface) {
    match s {
        Surface::BSpline(b) => {
            out.extend([18, b.deg_u as u64, b.deg_v as u64, b.nu as u64, b.nv as u64]);
            key_f64s(out, &b.knots_u);
            key_f64s(out, &b.knots_v);
            out.push(b.ctrl.len() as u64);
            for c in &b.ctrl {
                out.extend([c.x.to_bits(), c.y.to_bits(), c.z.to_bits()]);
            }
            match &b.weights {
                None => out.push(0),
                Some(w) => {
                    out.push(1);
                    key_f64s(out, w);
                }
            }
        }
        Surface::Offset { base, dist } => {
            out.push(20);
            key_surface(out, base);
            out.push(dist.to_bits());
        }
        other => other.push_key(out),
    }
}

fn key_vec2(out: &mut Vec<u64>, p: Vec2) {
    out.extend([p.x.to_bits(), p.y.to_bits()]);
}

/// Exact content of an optional pcurve.
pub fn key_curve2(out: &mut Vec<u64>, c: &Option<Curve2>) {
    match c {
        None => out.push(0),
        Some(Curve2::Line { o, d }) => {
            out.push(1);
            key_vec2(out, *o);
            key_vec2(out, *d);
        }
        Some(Curve2::Circle { c, x, r, ccw }) => {
            out.push(2);
            key_vec2(out, *c);
            key_vec2(out, *x);
            out.extend([r.to_bits(), *ccw as u64]);
        }
        Some(Curve2::Ellipse { c, x, a, b, ccw }) => {
            out.push(3);
            key_vec2(out, *c);
            key_vec2(out, *x);
            out.extend([a.to_bits(), b.to_bits(), *ccw as u64]);
        }
        Some(Curve2::BSpline(b)) => {
            out.extend([4, b.degree as u64]);
            key_f64s(out, &b.knots);
            out.push(b.ctrl.len() as u64);
            for p in &b.ctrl {
                key_vec2(out, *p);
            }
            match &b.weights {
                None => out.push(0),
                Some(w) => {
                    out.push(1);
                    key_f64s(out, w);
                }
            }
        }
    }
}

/// The face's surface and, per coedge of every loop (dead ones included, flagged), direction, pcurve, edge curve and
/// range: all that `Body::coedge_uv` and `coedge_params` read for it.
pub fn push_face_trim(b: &Body, f: FaceId, out: &mut Vec<u64>) {
    let face = b.face(f);
    key_surface(out, &face.surface);
    for &l in &face.loops {
        let lp = b.lp(l);
        out.extend([u64::MAX, lp.alive as u64, lp.coedges.len() as u64]);
        for &c in &lp.coedges {
            let ce = b.coedge(c);
            let ed = b.edge(ce.edge);
            out.push(ce.rev as u64);
            key_curve2(out, &ce.pcurve);
            ed.curve.push_key(out);
            out.extend([ed.t0.to_bits(), ed.t1.to_bits()]);
        }
    }
}
