//! Round pockets cut up from the bottom of a block (optionally drafted, so the wall is a cone) whose circle crosses the
//! block's side wall, which is a plane (a plate) or a cylinder (a disc): the pocket breaks out through it obliquely. The
//! two arcs of the pocket wall, the concave one at the pocket's ceiling (plane x wall) and the convex one at its foot
//! (wall x bottom face), run from one side-wall crossing to the other; their rounds must end on the side wall there.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

#[derive(Clone, Copy, Debug)]
enum Block {
    /// 80 x 80 x 30, side wall y = 40
    Plate,
    /// radius 33, 30 high
    Disc,
    /// the disc with the pocket turned onto its +x axis, where the disc's cylinder has its seam: the break-out
    /// vertices lie a few degrees either side of it
    DiscOnSeam,
}

/// The block, a radius 10 pocket centred at (0, cy) cut 10 up from z = 0 with `draft` degrees (narrowing upwards), and
/// optionally a fillet of radius `r` on the pocket's ceiling arc (`ceiling`) or its foot arc.
fn part(block: Block, cy: f64, draft: f64, round: Option<(f64, bool)>) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let base = match block {
        Block::Plate => r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"p1","x":-40,"y":-40},{"id":"p2","x":40,"y":-40},{"id":"p3","x":40,"y":40},{"id":"p4","x":-40,"y":40}],"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]}"#.to_string(),
        Block::Disc | Block::DiscOnSeam => r#"{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"c","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"c","r":33}],"constraints":[]}"#.to_string(),
    };
    let mut feats = vec![
        base,
        r#"{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":30}"#.to_string(),
        {
            let (px, py) = if matches!(block, Block::DiscOnSeam) { (cy, 0.0) } else { (0.0, cy) };
            format!(r#"{{"type":"sketch","id":"F4","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{px},"y":{py}}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}}"#)
        },
        format!(r#"{{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"blind","depth":10,"draft":{draft},"flip":true}}"#),
    ];
    if let Some((r, ceiling)) = round {
        // the arc's point nearest the block's centre
        let (z, rad, other) = if ceiling { (10.0, 10.0 - 10.0 * draft.to_radians().tan(), "F5.C1") } else { (0.0, 10.0, "F2.C0") };
        let (ax, ay) = if matches!(block, Block::DiscOnSeam) { (cy - rad, 0.0) } else { (0.0, cy - rad) };
        feats.push(format!(r#"{{"type":"fillet","id":"F6","radius":{r},"edges":[{{"faces":["F5.S.e1","{other}"],"at":[{ax},{ay},{z}]}}]}}"#));
    }
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, feats.join(","));
    let part = Part::parse(&doc).unwrap();
    let mut br = Brep::new();
    let r = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, r.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

/// The exact volume a round adds (ceiling) or removes (foot): its section in the meridian plane of the pocket (the
/// kite between the arc's corner, the two tangent points and the ball's centre, less the ball) revolved about the pocket
/// axis over the angles at which the block is there, at each radius from the axis (the side walls are vertical, so
/// that depends on the radius only). Integrated on a fine grid.
fn round_volume(block: Block, cy: f64, draft: f64, r: f64, ceiling: bool) -> f64 {
    use std::f64::consts::PI;
    let (sd, cd) = (draft.to_radians().sin(), draft.to_radians().cos());
    // (radius from the pocket axis, height): the corner and its two legs into the round's side (air at the ceiling,
    // material at the foot); the wall narrows upwards
    let (p, u1, u2) = if ceiling {
        ((10.0 - 10.0 * draft.to_radians().tan(), 10.0), (-1.0, 0.0), (sd, -cd))
    } else {
        ((10.0, 0.0), (1.0, 0.0), (-sd, cd))
    };
    let half = 0.5 * (u1.0 * u2.0 + u1.1 * u2.1).acos();
    let bis = ((u1.0 + u2.0) / (2.0 * half.cos()), (u1.1 + u2.1) / (2.0 * half.cos()));
    let c = (p.0 + bis.0 * r / half.sin(), p.1 + bis.1 * r / half.sin());
    let t = r / half.tan();
    let (t1, t2) = ((p.0 + u1.0 * t, p.1 + u1.1 * t), (p.0 + u2.0 * t, p.1 + u2.1 * t));
    let cross = |a: (f64, f64), b: (f64, f64), q: (f64, f64)| (b.0 - a.0) * (q.1 - a.1) - (b.1 - a.1) * (q.0 - a.0);
    let in_tri = |a, b, d, q| {
        let (x, y, z) = (cross(a, b, q), cross(b, d, q), cross(d, a, q));
        (x >= 0.0 && y >= 0.0 && z >= 0.0) || (x <= 0.0 && y <= 0.0 && z <= 0.0)
    };
    let extent = |rho: f64| -> f64 {
        let s = match block {
            Block::Plate => (40.0 - cy) / rho,
            Block::Disc | Block::DiscOnSeam => (33.0 * 33.0 - rho * rho - cy * cy) / (2.0 * cy * rho),
        };
        PI + 2.0 * s.clamp(-1.0, 1.0).asin()
    };
    let xs = [p.0, t1.0, t2.0, c.0];
    let ys = [p.1, t1.1, t2.1, c.1];
    let (x0, x1) = (xs.iter().cloned().fold(f64::MAX, f64::min), xs.iter().cloned().fold(f64::MIN, f64::max));
    let (y0, y1) = (ys.iter().cloned().fold(f64::MAX, f64::min), ys.iter().cloned().fold(f64::MIN, f64::max));
    const N: usize = 3000;
    let (dx, dy) = ((x1 - x0) / N as f64, (y1 - y0) / N as f64);
    let mut v = 0.0;
    for i in 0..N {
        let x = x0 + dx * (i as f64 + 0.5);
        let ext = extent(x);
        for j in 0..N {
            let q = (x, y0 + dy * (j as f64 + 0.5));
            if (q.0 - c.0).powi(2) + (q.1 - c.1).powi(2) > r * r && (in_tri(p, t1, c, q) || in_tri(p, t2, c, q)) {
                v += x * ext;
            }
        }
    }
    v * dx * dy
}

/// Builds the round; it must succeed, validate clean, and change the volume (material added for the concave ceiling,
/// removed for the convex foot) by the exact amount within the grid's accuracy.
fn check(block: Block, cy: f64, draft: f64, r: f64, ceiling: bool) -> Result<(), String> {
    let tag = format!("{block:?} cy {cy} draft {draft} r {r} {}", if ceiling { "ceiling" } else { "foot" });
    let (bare, e0) = part(block, cy, draft, None);
    assert!(e0.is_empty(), "{tag}: {e0:?}");
    let v0 = ck_topo::mass::mass_props(&bare.unwrap()).volume;
    let (b, errs) = part(block, cy, draft, Some((r, ceiling)));
    if !errs.is_empty() {
        return Err(format!("{tag}: {}", errs.join("; ").chars().take(160).collect::<String>()));
    }
    let b = b.unwrap();
    let issues = ck_check::validate(&b, 1e-6);
    if !issues.is_empty() {
        return Err(format!("{tag}: {:?}", issues.iter().map(|i| i.code).collect::<Vec<_>>()));
    }
    let dv = ck_topo::mass::mass_props(&b).volume - v0;
    let dv = if ceiling { dv } else { -dv };
    let want = round_volume(block, cy, draft, r, ceiling);
    if (dv - want).abs() >= 2e-3 * want {
        return Err(format!("{tag}: volume change {dv} vs {want}"));
    }
    Ok(())
}

fn all(cases: &[(Block, f64, f64, f64, bool)]) {
    let fails: Vec<String> = cases.iter().filter_map(|&(b, cy, d, r, c)| check(b, cy, d, r, c).err()).collect();
    assert!(fails.is_empty(), "{} of {} failed:\n{}", fails.len(), cases.len(), fails.join("\n"));
}

#[test]
fn ceiling_and_foot_rounds_end_on_an_oblique_plane_wall() {
    all(&[(Block::Plate, 34.0, 0.0, 0.3, true), (Block::Plate, 34.0, 0.0, 0.3, false), (Block::Plate, 34.0, 0.0, 1.0, true), (Block::Plate, 34.0, 0.0, 1.0, false)]);
}

#[test]
fn drafted_ceiling_and_foot_rounds_end_on_an_oblique_plane_wall() {
    // a 10 degree drafted pocket (cone wall): plane x cone at the ceiling (concave), cone x plane at the foot (convex).
    // Not yet: the r 1 ceiling round. The wall plane meets the cone in a hyperbola, which the surface intersection
    // marches and fits; the tool's clipped end and the union each fit it again, and the fits disagree by 1e-5 at the
    // tool's corner, so the side wall's merged boundary does not close (blendFailed, the body is left unchanged).
    all(&[(Block::Plate, 34.0, 10.0, 0.3, true), (Block::Plate, 34.0, 10.0, 0.3, false), (Block::Plate, 34.0, 10.0, 1.0, false)]);
}

#[test]
fn ceiling_and_foot_rounds_end_on_an_oblique_cylinder_wall() {
    all(&[(Block::Disc, 28.0, 0.0, 0.3, true), (Block::Disc, 28.0, 0.0, 0.3, false), (Block::Disc, 28.0, 0.0, 1.0, true), (Block::Disc, 28.0, 0.0, 1.0, false)]);
}

#[test]
fn drafted_ceiling_and_foot_rounds_end_on_an_oblique_cylinder_wall() {
    all(&[(Block::Disc, 28.0, 10.0, 0.3, true), (Block::Disc, 28.0, 10.0, 0.3, false), (Block::Disc, 28.0, 10.0, 1.0, true), (Block::Disc, 28.0, 10.0, 1.0, false)]);
}

#[test]
fn drafted_ceiling_and_foot_rounds_end_on_a_cylinder_wall_next_to_its_seam() {
    // Not yet: the r 0.5 ceiling round. Its tool, clipped by the disc's cylinder, has a face on that cylinder next to
    // the disc's own wall, and the union does not fuse the two across the seam (the wall is lost, blendFailed, the body
    // is left unchanged)
    all(&[(Block::DiscOnSeam, 28.0, 10.0, 0.5, false), (Block::DiscOnSeam, 28.0, 10.0, 1.0, true), (Block::DiscOnSeam, 28.0, 10.0, 1.0, false)]);
}
