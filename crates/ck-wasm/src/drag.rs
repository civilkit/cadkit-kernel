//! The `drag` request: move one assembly component so a grabbed point
//! follows the cursor, honouring the component's mates.
//!
//! `{face, point, target}` -> `{insert, delta, place, moves:[{id,delta,place}]}`
//! (`delta` = the rigid motion about the world origin as an `{o,x,y,z}`
//! frame, `place` = the new placement) or `{error}`: `missingReference`
//! (unknown face), `dragNoComponent` (face of no component), `dragFixed`.
//! Oracle (tools/asm/dragprobe.mjs): a component without mates translates
//! by `target - point`; with mates it reaches the point of its mate
//! manifold closest to the target (nearest point on the circle for a
//! concentric mate, in-plane slide for a coincident one); the motion is the
//! minimum-norm one in the solver's scaled variables (rotation columns /
//! max(scale, 1)). Only the dragged component moves (`moves` has one
//! entry); other components keep their mates unsatisfied.
//!
//! Port of func f1363 (damped Gauss-Newton on weighted rows).

use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_model::features::mate::{comp_box, find_ref, rows, Assembly, Rec, Row, Xf};
use ck_topo::body::Body;

fn frame_json(x: &Xf) -> Json {
    ck_model::regen::frame_json(&x.to_xform().to_frame())
}

fn err(e: &str) -> Json {
    Obj::new().put("error", e).build()
}

/// Solve the dense `n x n` system `a x = b` (partial pivoting).
fn gauss(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let p = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[p][col].abs() < 1e-300 {
            return None;
        }
        a.swap(col, p);
        b.swap(col, p);
        for i in col + 1..n {
            let f = a[i][col] / a[col][col];
            for k in col..n {
                a[i][k] -= f * a[col][k];
            }
            b[i] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let s: f64 = (i + 1..n).map(|k| a[i][k] * x[k]).sum();
        x[i] = (b[i] - s) / a[i][i];
    }
    Some(x)
}

/// func f1363: damped Gauss-Newton on the stacked rows of the component's
/// mate records (weight 1e4; the angular `Dir` rows 1e4 x scale) plus three
/// unit-weight rows pulling the grabbed point `p` onto `q`. Rotation columns
/// are divided by `max(scale, 1)`; the normal matrix gets `1e-9 x max diag`
/// on every diagonal plus 0.05 on the rotation ones; rotation steps are
/// capped at 0.5 rad; done when `scale x |w| + |dt| <= 1e-9`. Returns the
/// motion about the world origin.
fn drag_solve(recs: &[Rec], c: Vec3, scale: f64, p: Vec3, q: Vec3) -> Option<Xf> {
    let sc = if scale.is_nan() || scale <= 1.0 { 1.0 } else { scale };
    let mut r = Xf::IDENTITY.r;
    let mut t = Vec3::ZERO;
    let d0 = p - c;
    for _ in 0..200 {
        let mut a = [[0.0f64; 6]; 6];
        let mut b = [0.0f64; 6];
        let mut acc = |j: [f64; 6], res: f64, w: f64| {
            let j = [j[0] / sc * w, j[1] / sc * w, j[2] / sc * w, j[3] * w, j[4] * w, j[5] * w];
            let res = res * w;
            for i in 0..6 {
                for k in 0..6 {
                    a[i][k] += j[i] * j[k];
                }
                b[i] -= j[i] * res;
            }
        };
        let mut rs: Vec<Row> = vec![];
        for rec in recs {
            rs.clear();
            rows(rec, &r, t, c, &mut rs);
            let w = if matches!(rec, Rec::Dir { .. }) { 1e4 * sc } else { 1e4 };
            for row in &rs {
                // (the weight is applied to the scaled columns and residual)
                let j = [row.j[0], row.j[1], row.j[2], row.j[3], row.j[4], row.j[5]];
                acc(j, row.r, w);
            }
        }
        // point rows: n = e_x, e_y, e_z
        let dv = Vec3::new(
            (r[0] * d0.x + r[1] * d0.y) + r[2] * d0.z,
            (r[3] * d0.x + r[4] * d0.y) + r[5] * d0.z,
            (r[6] * d0.x + r[7] * d0.y) + r[8] * d0.z,
        );
        let cc = Vec3::new(c.x + dv.x, c.y + dv.y, c.z + dv.z);
        let (qa, qb, qc) = (dv.x, dv.y, dv.z);
        acc([0.0, qc, -qb, 1.0, 0.0, 0.0], (t.x + cc.x) - q.x, 1.0);
        acc([-qc, 0.0, qa, 0.0, 1.0, 0.0], (t.y + cc.y) - q.y, 1.0);
        acc([qb, -qa, 0.0, 0.0, 0.0, 1.0], (t.z + cc.z) - q.z, 1.0);
        let m = (0..6).map(|i| a[i][i]).fold(f64::MIN_POSITIVE, f64::max);
        let eps = m * 1e-9;
        for i in 0..3 {
            a[i][i] += eps + 0.05;
            a[i + 3][i + 3] += eps;
        }
        let x = gauss(a.iter().map(|r| r.to_vec()).collect(), b.to_vec())?;
        let (mut w0, mut w1, mut w2) = (x[0] / sc, x[1] / sc, x[2] / sc);
        let mut n = ((w0 * w0 + w1 * w1) + w2 * w2).sqrt();
        if n > 0.5 {
            let k = 0.5 / n;
            w0 *= k;
            w1 *= k;
            w2 *= k;
            n = ((w2 * w2 + w0 * w0) + w1 * w1).sqrt();
        }
        if n > 0.0 {
            let rot = Xf::rotation(Vec3::new(w0 / n, w1 / n, w2 / n), n);
            r = Xf::mul_r(&rot.r, &r);
        }
        t = Vec3::new(t.x + x[3], t.y + x[4], t.z + x[5]);
        let dt = ((x[3] * x[3] + x[4] * x[4]) + x[5] * x[5]).sqrt();
        if sc * n + dt <= 1e-9 {
            break;
        }
    }
    let tx = Vec3::new(
        (c.x - (r[0] * c.x + r[1] * c.y + r[2] * c.z)) + t.x,
        (c.y - (r[3] * c.x + r[4] * c.y + r[5] * c.z)) + t.y,
        (c.z - (r[6] * c.x + r[7] * c.y + r[8] * c.z)) + t.z,
    );
    Some(Xf { r, t: [tx.x, tx.y, tx.z] })
}

/// The `drag` reply.
pub fn reply(body: &Body, asm: &Assembly, msg: &Json) -> Json {
    let v3 = |k: &str| msg.get(k).and_then(Json::as_vec3).map(Vec3::from_array);
    let (Some(face), Some(p), Some(q)) = (msg.get_str("face").and_then(|id| find_ref(body, id)), v3("point"), v3("target")) else {
        return err("missingReference");
    };
    let Some(k) = asm.components.iter().position(|c| c.owns_face(face)) else { return err("dragNoComponent") };
    let comp = &asm.components[k];
    if comp.fixed {
        return err("dragFixed");
    }
    let recs = asm.recs_for(body, k).unwrap_or_default();
    let delta = if recs.is_empty() {
        Xf::translation(q - p)
    } else {
        // (the dragged component's own box, as for a lone component)
        let (c, scale) = match comp_box(body, &comp.range) {
            Some((lo, hi)) => {
                let (ex, ey, ez) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
                (Vec3::new((hi.x + lo.x) * 0.5, (hi.y + lo.y) * 0.5, (hi.z + lo.z) * 0.5), (ez * ez + (ex * ex + ey * ey)).sqrt() * 0.5)
            }
            None => (Vec3::ZERO, 0.0),
        };
        match drag_solve(&recs, c, scale, p, q) {
            Some(x) => x,
            None => return err("mateConflict"),
        }
    };
    let place = delta.compose(&comp.place);
    // followers: components mated to the moved one are re-solved against
    // their own mates (Gauss-Seidel in insertion order, dragged one frozen)
    let mut work = body.clone();
    delta.range(&mut work, &comp.range);
    let n = asm.components.len();
    let mut cum: Vec<Option<Xf>> = vec![None; n];
    let mut moves_left = 8 * n;
    loop {
        let mut changed = false;
        for j in 0..n {
            let cj = &asm.components[j];
            if j == k || cj.fixed {
                continue;
            }
            let Ok(rj) = asm.recs_for(&work, j) else { continue };
            if rj.is_empty() {
                continue;
            }
            let (c, scale) = match comp_box(&work, &cj.range) {
                Some((lo, hi)) => {
                    let (ex, ey, ez) = (hi.x - lo.x, hi.y - lo.y, hi.z - lo.z);
                    (Vec3::new((hi.x + lo.x) * 0.5, (hi.y + lo.y) * 0.5, (hi.z + lo.z) * 0.5), (ez * ez + (ex * ex + ey * ey)).sqrt() * 0.5)
                }
                None => (Vec3::ZERO, 0.0),
            };
            let Some(sv) = ck_model::features::mate::solve(&rj, c, scale) else { return err("mateConflict") };
            if sv.iterations == 0 {
                continue;
            }
            sv.x.range(&mut work, &cj.range);
            cum[j] = Some(match cum[j] {
                Some(prev) => sv.x.compose(&prev),
                None => sv.x,
            });
            changed = true;
            moves_left = moves_left.saturating_sub(1);
            if moves_left == 0 {
                return err("mateConflict");
            }
        }
        if !changed {
            break;
        }
    }
    let mv = |g: usize, d: &Xf| {
        let c = &asm.components[g];
        Obj::new().put("id", c.id.as_str()).put("delta", frame_json(d)).put("place", frame_json(&d.compose(&c.place))).build()
    };
    let mut moves = vec![mv(k, &delta)];
    for (j, d) in cum.iter().enumerate() {
        if let Some(d) = d {
            moves.push(mv(j, d));
        }
    }
    Obj::new()
        .put("insert", comp.id.as_str())
        .put("delta", frame_json(&delta))
        .put("place", frame_json(&place))
        .put("moves", Json::Arr(moves))
        .build()
}
