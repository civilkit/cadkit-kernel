//! Hardware of a `frame`: brackets with their T-nuts and bolts, gussets, end caps and levelling feet, placed from the joints and
//! free ends the frame pass worked out (see docs/design/frame-connectors.md in the kit).
//!
//! `hardware: {joint, cap, foot}` gives the default part of each kind (absent = none); `joints[].connector` overrides one joint
//! (`"none"`, or a part). A part is an object `{id, label, kind, spec, ...dimensions}`:
//! - `lbracket {arm, t, w, hole, holeAt, tnut?: {id, label, spec, l}, bolt?: {id, label, spec, d, len, headD, headH}}`: an L angle
//!   bracket in the inner corner of a 90 deg joint, an arm on the face of each member with a hole `holeAt` from the corner. On a
//!   T-slot member each hole gets a bolt and a T-nut in the slot.
//! - `gusset {leg, t}`: a triangular plate in the corner plane (any angle), legs `leg` along both members from the inner corner.
//! - `corner3 {block, arm, t, w, hole, holeAt, tnut?, bolt?}` (in `hardware.corner3`): a 3-way corner block for a node where three members of
//!   equal flat section meet at right angles: a cube `block` in the inner octant with a plate on one face of each member (a hole each,
//!   a T-nut and screw on T-slot). It replaces the pairwise brackets at that node unless a joint there has its own `connector`.
//! - `cap {t}`: a plate over the section on a free member end.
//! - `foot {base, baseH, studD, studLen, neck}`: a round pad on a stud, on the free lower end of a vertical member that stands on
//!   the lowest level of the frame (down is -Y, or `hardware.down`); the stud goes into the member's bore when it has one.
//!
//! Every placed solid is its own body `<id> H<k>`. The regen reply's frame entry gets `hardware: {parts, joints}`: `parts` are
//! the placed parts grouped by id with their quantity (the bill of materials), `joints` lists every joint with the connector it
//! got or why it got none.

use super::*;

/// One placed solid.
struct Placed {
    id: String,
    label: String,
    kind: String,
    spec: String,
    body: Body,
}

/// A through hole to drill in a member: from `from` along `dir` (unit) for `len`, diameter `d`, starting at `o`.
pub struct Drill {
    pub member: usize,
    o: Vec3,
    dir: Vec3,
    from: f64,
    len: f64,
    d: f64,
}

impl Drill {
    pub fn tool(&self, tag: &str) -> Result<Body, FeatureError> {
        let rad = normal_to(self.dir);
        prism(tag, self.o + self.dir * self.from, rad, self.dir.cross(rad), vec![disc(self.d / 2.0)], 0.0, self.len)
    }
}

struct Cx<'a> {
    frame: &'a str,
    out: Vec<Placed>,
    drill: bool,
    drills: Vec<Drill>,
}

fn num(j: &Json, key: &str, default: Option<f64>) -> Result<f64, FeatureError> {
    match (j.get_f64(key), default) {
        (Some(v), _) if v.is_finite() => Ok(v),
        (_, Some(d)) => Ok(d),
        _ => Err(invalid(format!("hardware part `{}` needs `{key}`", j.get_str("id").unwrap_or("?")))),
    }
}

fn pos(j: &Json, key: &str, default: Option<f64>) -> Result<f64, FeatureError> {
    let v = num(j, key, default)?;
    if v > 1e-6 {
        Ok(v)
    } else {
        Err(invalid(format!("hardware part `{}`: `{key}` must be above 0", j.get_str("id").unwrap_or("?"))))
    }
}

impl Cx<'_> {
    /// Build, check and keep one solid of `part` (the part's id, label and spec go to the bill of materials).
    fn add(&mut self, part: &Json, make: impl FnOnce(&str) -> Result<Body, FeatureError>) -> Result<(), FeatureError> {
        let k = self.out.len() + 1;
        let tag = format!("{}.H{k}", self.frame);
        let mut body = make(&tag)?;
        let id = part.get_str("id").unwrap_or(part.get_str("kind").unwrap_or("part")).to_string();
        let issues = ck_check::validate(&body, 1e-6);
        if !issues.is_empty() || body.num_lumps() != 1 {
            let codes: Vec<String> = issues.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect();
            return Err(invalid(format!("hardware {id} is not a valid solid ({} lumps; {})", body.num_lumps(), codes.join("; "))));
        }
        let name = format!("{} H{k}", self.frame);
        for l in body.lump_ids().collect::<Vec<_>>() {
            body.lumps[l.idx()].name = Some(name.clone());
        }
        self.out.push(Placed {
            label: part.get_str("label").unwrap_or(&id).to_string(),
            kind: part.get_str("kind").unwrap_or("").to_string(),
            spec: part.get_str("spec").unwrap_or("").to_string(),
            id,
            body,
        });
        Ok(())
    }
}

/// Extrude `regions` on the frame (o, x, y; z = x cross y) from `s0` to `s1` along z.
fn prism(tag: &str, o: Vec3, x: Vec3, y: Vec3, regions: Vec<ProfileRegion>, s0: f64, s1: f64) -> Result<Body, FeatureError> {
    let fr = Frame { o, x, y, z: x.cross(y) };
    extrude(tag, &Profile::new(fr, regions).normalized(), ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).map_err(|e| invalid(format!("hardware {tag}: {e}")))
}

fn disc(r: f64) -> ProfileRegion {
    ProfileRegion { outer: ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::ZERO, r)]), holes: vec![] }
}

/// A body of revolution about the axis through `o` along `axis`: `pts` are (radius, height along the axis), a closed polygon
/// that starts and ends on the axis.
fn lathe(tag: &str, o: Vec3, axis: Vec3, radial: Vec3, pts: &[(f64, f64)]) -> Result<Body, FeatureError> {
    let p: Vec<Vec2> = pts.iter().map(|&(r, h)| v2(r, h)).collect();
    let fr = Frame { o, x: radial, y: axis, z: radial.cross(axis) };
    let prof = Profile::new(fr, vec![ProfileRegion { outer: poly("e", &p), holes: vec![] }]).normalized();
    ck_topo::make::revolve::revolve(tag, &prof, o, axis, 0.0, std::f64::consts::TAU).map_err(|e| invalid(format!("hardware {tag}: {e}")))
}

/// Any unit vector normal to `a`.
fn normal_to(a: Vec3) -> Vec3 {
    a.cross(if a.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) }).normalize()
}

/// The dimensions of a T-slot section that matter for fasteners.
struct Slot {
    slot: f64,
    lip: f64,
    depth: f64,
    /// width of the undercut at its top (under the lip)
    inner: f64,
    /// half width of the undercut at its floor
    hb: f64,
}

fn slot_of(state: &RegenState, prof: &Json) -> Result<Option<Slot>, FeatureError> {
    if prof.get_str("kind") != Some("tslot") {
        return Ok(None);
    }
    let s = dim(state, prof, "s", None)?;
    let slot = dim(state, prof, "slot", Some(s * 0.31))?;
    let lip = dim(state, prof, "lip", Some(s * 0.09))?;
    let inner = dim(state, prof, "inner", Some(s * 0.55))?;
    let depth = dim(state, prof, "depth", Some(s * 0.3))?;
    let hb = (inner / 2.0).min(0.8 * (s / 2.0 - depth)).max(slot / 2.0);
    Ok(Some(Slot { slot, lip, depth, inner, hb }))
}

/// Radius of the bore running through a section (0: solid or no round bore).
fn bore_radius(state: &RegenState, prof: &Json) -> Result<f64, FeatureError> {
    Ok(match prof.get_str("kind").unwrap_or("") {
        "tslot" => dim(state, prof, "bore", Some(dim(state, prof, "s", None)? * 0.21))? / 2.0,
        "rectTube" => (dim(state, prof, "w", None)?.min(dim(state, prof, "h", Some(dim(state, prof, "w", None)?))?) - 2.0 * dim(state, prof, "t", None)?) / 2.0,
        "roundTube" => dim(state, prof, "d", None)? / 2.0 - dim(state, prof, "t", None)?,
        _ => 0.0,
    })
}

/// A T-nut in the slot and the bolt through the plate into it. `o` is the outer face of the plate at the hole, `inward` the unit
/// direction into the member, `along` the slot direction, `t` the plate thickness.
fn fasten(cx: &mut Cx, slot: &Slot, tnut: &Json, bolt: &Json, o: Vec3, inward: Vec3, along: Vec3, t: f64) -> Result<(), FeatureError> {
    let (d, len, head_d, head_h) = (pos(bolt, "d", None)?, pos(bolt, "len", None)?, pos(bolt, "headD", Some(1.75 * pos(bolt, "d", None)?))?, pos(bolt, "headH", Some(0.55 * pos(bolt, "d", None)?))?);
    let nut_l = pos(tnut, "l", None)?;
    let gap = 0.3;
    let nut_top = t + slot.lip;
    let nut_h = slot.depth - slot.lip - gap;
    let id = bolt.get_str("id").unwrap_or("bolt");
    if nut_h < 1.0 || d / 2.0 >= slot.slot / 2.0 {
        return Err(invalid(format!("T-nut / bolt {id} do not fit this slot")));
    }
    if len > t + slot.depth - 0.01 || len < nut_top + 1.0 {
        return Err(invalid(format!("bolt {id} is {len} long: with the {t} plate and this slot it must reach the T-nut ({:.1}) and stop short of the slot floor ({:.1})", nut_top + 1.0, t + slot.depth)));
    }
    // the undercut narrows from `inner` under the lip to the floor: the nut is as wide as it is at the nut's lower face, less a clearance
    let w_low = 2.0 * slot.hb + (slot.inner - 2.0 * slot.hb) * (gap / (slot.depth - slot.lip));
    let across = (w_low - 0.1).min(slot.inner - 0.2);
    if across < slot.slot + 0.2 {
        return Err(invalid(format!("T-nut {id}: the slot undercut is too narrow to hold a nut")));
    }
    let y = inward.cross(along);
    let region = ProfileRegion {
        outer: rounded_rect("e", nut_l, across, 0.0).0,
        holes: vec![hole(ProfileLoop::new(vec![ProfileSeg::circle("i1", Vec2::ZERO, d / 2.0)]))],
    };
    // frame (along, y, inward): along x y = inward x along = a right-handed frame with z = inward
    cx.add(tnut, |tag| prism(tag, o, along, y, vec![region], nut_top, nut_top + nut_h))?;
    let radial = y;
    let axis = inward;
    cx.add(bolt, |tag| lathe(tag, o, axis, radial, &[(0.0, -head_h), (head_d / 2.0, -head_h), (head_d / 2.0, 0.0), (d / 2.0, 0.0), (d / 2.0, len), (0.0, len)]))
}

/// Faces a plate can lie on: T-slot, flat bar and rectangular tube sections.
fn flat(prof: &Json) -> bool {
    matches!(prof.get_str("kind"), Some("tslot" | "rect" | "rectTube"))
}

/// A hex nut with a hole of diameter `d`, thickness `nt`, across flats `af`, on the frame (o, along, y), z = inward.
fn hex_region(af: f64, d: f64) -> ProfileRegion {
    let r = af / 3f64.sqrt();
    let pts: Vec<Vec2> = (0..6).map(|k| v2(r * (k as f64 * std::f64::consts::FRAC_PI_3).cos(), r * (k as f64 * std::f64::consts::FRAC_PI_3).sin())).collect();
    ProfileRegion { outer: poly("e", &pts), holes: vec![hole(ProfileLoop::new(vec![ProfileSeg::circle("i1", Vec2::ZERO, d / 2.0)]))] }
}

/// Fasteners for one plate hole: `o` is the outer face of the plate at the hole, `inward` into the member. A T-slot member gets a T-nut
/// and screw (`tnut`, `bolt`); with drilling on, a tube member gets a through hole, a bolt run through both walls and a hex nut (`nut`).
#[allow(clippy::too_many_arguments)]
fn arm_fasteners(cx: &mut Cx, state: &RegenState, part: &Json, im: usize, m: &Member, o: Vec3, inward: Vec3, along: Vec3, t: f64, hole_d: f64) -> Result<(), FeatureError> {
    if let Some(slot) = slot_of(state, &m.prof)? {
        if let (Some(tnut), Some(bolt)) = (part.get("tnut"), part.get("bolt")) {
            fasten(cx, &slot, tnut, bolt, o, inward, along, t)?;
        }
        return Ok(());
    }
    let (Some(bolt), Some(nut), true) = (part.get("bolt"), part.get("nut"), cx.drill) else { return Ok(()) };
    let d = pos(bolt, "d", None)?;
    if d >= hole_d {
        return Err(invalid(format!("bolt {} does not go through the {hole_d} hole", bolt.get_str("id").unwrap_or("?"))));
    }
    let (af, nt) = (pos(nut, "af", None)?, pos(nut, "t", None)?);
    let wall = 2.0 * m.reach(-inward);
    let len = ((t + wall + nt + 1.0) / 5.0).ceil() * 5.0;
    let (head_d, head_h) = (pos(bolt, "headD", Some(1.6 * d))?, pos(bolt, "headH", Some(0.65 * d))?);
    let bid = bolt.get_str("id").unwrap_or("bolt");
    let sized = Obj::new()
        .put("id", format!("{bid}x{len}"))
        .put("label", format!("{} x {len}", bolt.get_str("label").unwrap_or(bid)))
        .put("kind", "bolt")
        .put("spec", format!("{} x {len}", bolt.get_str("spec").unwrap_or("")).trim().to_string())
        .build();
    let y = inward.cross(along);
    cx.add(&sized, |tag| lathe(tag, o, inward, y, &[(0.0, -head_h), (head_d / 2.0, -head_h), (head_d / 2.0, 0.0), (d / 2.0, 0.0), (d / 2.0, len), (0.0, len)]))?;
    cx.add(nut, |tag| prism(tag, o, along, y, vec![hex_region(af, d)], t + wall, t + wall + nt))?;
    cx.drills.push(Drill { member: im, o, dir: inward, from: t - 1.0, len: wall + 2.0, d: hole_d });
    Ok(())
}

/// The three members of a 3-way node: direction into each from the node.
pub struct Node3 {
    pub m: [usize; 3],
    pub u: [Vec3; 3],
    pub node: Vec3,
}

/// A box on the axes (a, b, c) (orthonormal, any handedness) from `o`: ranges along each.
fn cell(tag: &str, o: Vec3, ax: [Vec3; 3], r: [(f64, f64); 3]) -> Result<Body, FeatureError> {
    let (x, y) = (ax[0], ax[1]);
    let z = x.cross(y);
    let (c0, c1) = if z.dot(ax[2]) > 0.0 { r[2] } else { (-r[2].1, -r[2].0) };
    let (outer, _) = rounded_rect("e", r[0].1 - r[0].0, r[1].1 - r[1].0, 0.0);
    let o2 = o + x * ((r[0].0 + r[0].1) / 2.0) + y * ((r[1].0 + r[1].1) / 2.0);
    prism(tag, o2, x, y, vec![ProfileRegion { outer, holes: vec![] }], c0, c1)
}

fn bool_op(a: &Body, b: &Body, op: ck_ops::boolean::Op, what: &str) -> Result<Body, FeatureError> {
    ck_ops::boolean::boolean(a, b, op, super::super::TOL).map(|r| r.body).map_err(|e| FeatureError::new("booleanFailed").with("detail", Json::Str(format!("{what}: {e}"))))
}

/// The 3-way corner block at a node. `Ok(Some(reason))`: not possible here.
fn corner3(cx: &mut Cx, state: &RegenState, part: &Json, ms: &[Member], n: &Node3) -> Result<Option<String>, FeatureError> {
    let (c, arm, t, w) = (pos(part, "block", None)?, pos(part, "arm", None)?, pos(part, "t", None)?, pos(part, "w", None)?);
    let (hole_d, hole_at) = (pos(part, "hole", None)?, pos(part, "holeAt", None)?);
    let u = n.u;
    // flat sections turned square to the corner
    for i in 0..3 {
        let m = &ms[n.m[i]];
        let square = |v: Vec3| m.sx.dot(v).abs() > 1.0 - 1e-6 || m.sy.dot(v).abs() > 1.0 - 1e-6;
        if !flat(&m.prof) || !square(u[(i + 1) % 3]) || !square(u[(i + 2) % 3]) {
            return Ok(Some("the corner block needs flat sections square to the corner".into()));
        }
    }
    // o[k]: how far the inner octant starts along u[k]: the half section of the members that run across it (equal for both)
    let mut off = [0.0; 3];
    for k in 0..3 {
        let (a, b) = (ms[n.m[(k + 1) % 3]].reach(u[k]), ms[n.m[(k + 2) % 3]].reach(u[k]));
        if (a - b).abs() > 1e-6 {
            return Ok(Some("the members differ in size".into()));
        }
        off[k] = a;
    }
    let p3 = n.node + u[0] * off[0] + u[1] * off[1] + u[2] * off[2];
    let mut plates = vec![];
    for i in 0..3 {
        let (al, fa, wi) = (i, (i + 2) % 3, (i + 1) % 3);
        if off[al] + arm > ms[n.m[i]].len + 1e-6 {
            return Ok(Some("the members are shorter than the corner block".into()));
        }
        if w > 2.0 * off[wi] + 1e-6 || w < off[wi] + hole_d / 2.0 + 0.5 || hole_at - hole_d / 2.0 <= 0.0 || hole_at + hole_d / 2.0 >= arm {
            return Ok(Some("the plate does not fit the member face".into()));
        }
        plates.push((al, fa, wi));
    }
    let ax = |a: usize, b: usize, cc: usize| [u[a], u[b], u[cc]];
    let tag = format!("{}.H{}", cx.frame, cx.out.len() + 1);
    let mut body = cell(&format!("{tag}.B"), p3, ax(0, 1, 2), [(0.0, c), (0.0, c), (0.0, c)])?;
    for (k, &(al, fa, wi)) in plates.iter().enumerate() {
        let mut pl = cell(&format!("{tag}.P{k}"), p3, ax(al, wi, fa), [(0.0, arm), (-w, 0.0), (0.0, t)])?;
        // hole through the plate along the face normal, over the member axis
        let centre = p3 + u[al] * hole_at - u[wi] * off[wi];
        let rad = u[wi];
        let tool = prism(&format!("{tag}.D{k}"), centre - u[fa] * 1.0, rad, u[fa].cross(rad), vec![disc(hole_d / 2.0)], 0.0, t + 2.0)?;
        pl = bool_op(&pl, &tool, ck_ops::boolean::Op::Cut, "corner block hole")?;
        body = bool_op(&body, &pl, ck_ops::boolean::Op::Union, "corner block plate")?;
    }
    cx.add(part, |_| Ok(body))?;
    for &(al, fa, wi) in &plates {
        let im = n.m[al];
        let centre = p3 + u[al] * hole_at - u[wi] * off[wi];
        arm_fasteners(cx, state, part, im, &ms[im], centre + u[fa] * t, -u[fa], u[al], t, hole_d)?;
    }
    Ok(None)
}

/// The two members of a joint and where they meet: member index, unit direction into the member from the node.
struct Pair<'a> {
    ia: usize,
    ib: usize,
    a: &'a Member,
    b: &'a Member,
    u1: Vec3,
    u2: Vec3,
    node: Vec3,
    /// length of each member from the node
    room: [f64; 2],
}

/// Place the part `part` at a joint. `Ok(None)`: placed; `Ok(Some(reason))`: not possible here.
fn at_joint(cx: &mut Cx, state: &RegenState, part: &Json, p: &Pair) -> Result<Option<String>, FeatureError> {
    let kind = part.get_str("kind").unwrap_or("");
    let cos = p.u1.dot(p.u2).clamp(-1.0, 1.0);
    let sin = (1.0 - cos * cos).sqrt();
    if sin < 1e-3 {
        return Ok(Some("the members are in line".into()));
    }
    let n = p.u1.cross(p.u2).normalize();
    // orthonormal axes of the corner plane: e1 along the first member, e2 across it towards the second
    let (e1, e2) = (p.u1, n.cross(p.u1));
    let (w1, w2) = ((p.u2 - p.u1 * cos).normalize(), (p.u1 - p.u2 * cos).normalize());
    let corner = p.node + p.u1 * (p.b.reach(w2) / sin) + p.u2 * (p.a.reach(w1) / sin);
    // room along each member from the inner corner to its far end
    let fits = |l: f64| (corner - p.node).dot(p.u1) + l <= p.room[0] + 1e-6 && (corner - p.node).dot(p.u2) + l <= p.room[1] + 1e-6;
    match kind {
        "lbracket" => {
            if (cos.acos().to_degrees() - 90.0).abs() > 1.0 {
                return Ok(Some("an L bracket needs a 90 degree joint".into()));
            }
            let square = |m: &Member, w: Vec3| m.sx.dot(w).abs() > 1.0 - 1e-6 || m.sy.dot(w).abs() > 1.0 - 1e-6;
            if !square(p.a, e2) || !square(p.b, e1) {
                return Ok(Some("the section is turned: its faces are not square to the corner".into()));
            }
            let (arm, t, w) = (pos(part, "arm", None)?, pos(part, "t", None)?, pos(part, "w", None)?);
            let (hole_d, hole_at) = (pos(part, "hole", None)?, pos(part, "holeAt", None)?);
            if !fits(arm) {
                return Ok(Some("the members are shorter than the bracket".into()));
            }
            if !flat(&p.a.prof) || !flat(&p.b.prof) {
                return Ok(Some("an L bracket needs flat faces".into()));
            }
            if w > 2.0 * p.a.reach(n) + 1e-6 || w > 2.0 * p.b.reach(n) + 1e-6 {
                return Ok(Some("the bracket is wider than the member face".into()));
            }
            if hole_at - hole_d / 2.0 <= t || hole_at + hole_d / 2.0 >= arm {
                return Err(invalid(format!("bracket {}: the hole does not fit the arm", part.get_str("id").unwrap_or("?"))));
            }
            let l = poly("e", &[v2(0.0, 0.0), v2(arm, 0.0), v2(arm, t), v2(t, t), v2(t, arm), v2(0.0, arm)]);
            let (a, b) = (p.a, p.b);
            cx.add(part, |tag| {
                let mut body = prism(tag, corner, e1, e2, vec![ProfileRegion { outer: l, holes: vec![] }], -w / 2.0, w / 2.0)?;
                for (k, (c, ax)) in [(corner + e1 * hole_at, e2), (corner + e2 * hole_at, e1)].into_iter().enumerate() {
                    let radial = n;
                    let tool = prism(&format!("{tag}.D{k}"), c - ax * 1.0, radial, ax.cross(radial), vec![disc(hole_d / 2.0)], 0.0, t + 2.0)?;
                    body = ck_ops::boolean::boolean(&body, &tool, ck_ops::boolean::Op::Cut, super::super::TOL)
                        .map_err(|e| FeatureError::new("booleanFailed").with("detail", Json::Str(format!("bracket hole {k}: {e}"))))?
                        .body;
                }
                Ok(body)
            })?;
            for (im, m, c, out, along) in [(p.ia, a, corner + e1 * hole_at, e2, e1), (p.ib, b, corner + e2 * hole_at, e1, e2)] {
                arm_fasteners(cx, state, part, im, m, c + out * t, -out, along, t, hole_d)?;
            }
        }
        "gusset" => {
            let (leg, t) = (pos(part, "leg", None)?, pos(part, "t", None)?);
            if !fits(leg) {
                return Ok(Some("the members are shorter than the gusset".into()));
            }
            let q = v2(p.u2.dot(e1), p.u2.dot(e2));
            let tri = poly("e", &[v2(0.0, 0.0), v2(leg, 0.0), v2(q.x * leg, q.y * leg)]);
            cx.add(part, |tag| prism(tag, corner, e1, e2, vec![ProfileRegion { outer: tri, holes: vec![] }], -t / 2.0, t / 2.0))?;
        }
        other => return Err(invalid(format!("hardware `joint` part kind `{other}`: lbracket or gusset"))),
    }
    Ok(None)
}

fn cap_at(cx: &mut Cx, part: &Json, m: &Member, k: usize) -> Result<(), FeatureError> {
    let t = pos(part, "t", None)?;
    let outer = if m.prof.get_str("kind") == Some("tslot") {
        let s = m.sec.regions[0].outer.segs.iter().fold((f64::MAX, f64::MIN), |a, sg| (a.0.min(sg.start().x), a.1.max(sg.start().x)));
        rounded_rect("e", s.1 - s.0, s.1 - s.0, 0.0).0
    } else {
        m.sec.regions[0].outer.clone()
    };
    let (at, s0, s1) = if k == 0 { (m.seg.a, -t, 0.0) } else { (m.seg.b, 0.0, t) };
    cx.add(part, |tag| prism(tag, at, m.sx, m.sy, vec![ProfileRegion { outer, holes: vec![] }], s0, s1))
}

fn foot_at(cx: &mut Cx, state: &RegenState, part: &Json, m: &Member, k: usize) -> Result<(), FeatureError> {
    let (base, base_h, stud_d) = (pos(part, "base", None)?, pos(part, "baseH", None)?, pos(part, "studD", None)?);
    let neck = num(part, "neck", Some(0.0))?.max(0.0);
    let bore = bore_radius(state, &m.prof)?;
    let stud_in = if bore > stud_d / 2.0 + 1e-6 { pos(part, "studLen", None)?.min(m.len - 1.0) } else { 0.0 };
    let (at, out) = if k == 0 { (m.seg.a, -m.d) } else { (m.seg.b, m.d) };
    let radial = normal_to(out);
    let pts: Vec<(f64, f64)> = if stud_in > 0.0 {
        vec![(0.0, -stud_in), (stud_d / 2.0, -stud_in), (stud_d / 2.0, neck), (base / 2.0, neck), (base / 2.0, neck + base_h), (0.0, neck + base_h)]
    } else if neck > 0.0 {
        vec![(0.0, 0.0), (stud_d / 2.0, 0.0), (stud_d / 2.0, neck), (base / 2.0, neck), (base / 2.0, neck + base_h), (0.0, neck + base_h)]
    } else {
        vec![(0.0, 0.0), (base / 2.0, 0.0), (base / 2.0, base_h), (0.0, base_h)]
    };
    cx.add(part, |tag| lathe(tag, at, out, radial, &pts))
}

fn part_of<'a>(j: Option<&'a Json>) -> Option<&'a Json> {
    j.filter(|p| matches!(p, Json::Obj(_)))
}

/// One joint found by the frame pass.
pub struct JointAt {
    pub a: usize,
    pub b: usize,
    pub u1: Vec3,
    pub u2: Vec3,
    pub node: Vec3,
    pub corner: &'static str,
    pub room: [f64; 2],
    /// a tee: the other side of the through member (direction into it, length from the tee point)
    pub alt: Option<(Vec3, f64)>,
    /// index into the frame's 3-way nodes when the joint is one of the two pairs there
    pub node3: Option<usize>,
}

/// Place the hardware and build the reply's `hardware` entry. Returns the solids (named, in order) and the entry; `None` when
/// the frame asks for no hardware at all.
pub fn place(state: &RegenState, f: &Feature, ms: &[Member], joints: &[JointAt], nodes3: &[Node3]) -> Result<Option<(Vec<Body>, Vec<Drill>, Json)>, FeatureError> {
    let hw = part_of(f.raw.get("hardware"));
    let overrides = f.raw.get_arr("joints").unwrap_or(&[]);
    let any_override = overrides.iter().any(|j| j.get("connector").is_some());
    if hw.is_none() && !any_override {
        return Ok(None);
    }
    let drill = hw.and_then(|h| h.get("drill")).and_then(|d| d.as_bool()) == Some(true);
    let mut cx = Cx { frame: &f.id, out: vec![], drill, drills: vec![] };
    let default_joint = hw.and_then(|h| part_of(h.get("joint")));
    let mut jrep = vec![];
    let corner3_part = hw.and_then(|h| part_of(h.get("corner3")));
    let tee_both = hw.and_then(|h| h.get("teeBoth")).and_then(|b| b.as_bool()) == Some(true);
    let find_ov = |ka: &str, kb: &str| {
        overrides.iter().find(|o| {
            let s = str_list(o.get("segments"));
            s.len() == 2 && ((s[0] == ka && s[1] == kb) || (s[0] == kb && s[1] == ka))
        })
    };
    // 3-way nodes: the corner block, unless a joint there has its own connector; the outcome is shared by the node's two joints
    let mut node_out: Vec<Option<Option<String>>> = vec![None; nodes3.len()];
    let mut node_bodies: Vec<Vec<Json>> = vec![vec![]; nodes3.len()];
    let names = |cx: &Cx, from: usize| -> Vec<Json> { (from..cx.out.len()).map(|i| Json::Str(format!("{} H{}", cx.frame, i + 1))).collect() };
    for j in joints {
        let start = cx.out.len();
        let (ka, kb) = (ms[j.a].seg.key.as_str(), ms[j.b].seg.key.as_str());
        let ov = find_ov(ka, kb);
        let mut entry = Obj::new()
            .put("segments", Json::Arr(vec![Json::Str(ka.into()), Json::Str(kb.into())]))
            .put("node", Json::Arr(vec![Json::Num(j.node.x), Json::Num(j.node.y), Json::Num(j.node.z)]))
            .put("angle", (j.u1.dot(j.u2).clamp(-1.0, 1.0).acos().to_degrees() * 1e4).round() / 1e4)
            .put("corner", j.corner);
        if let (Some(k), Some(cp)) = (j.node3, corner3_part) {
            let n3 = &nodes3[k];
            let own = joints.iter().any(|o| o.node3 == Some(k) && find_ov(ms[o.a].seg.key.as_str(), ms[o.b].seg.key.as_str()).map_or(false, |x| x.get("connector").is_some()));
            if !own {
                if node_out[k].is_none() {
                    node_out[k] = Some(corner3(&mut cx, state, cp, ms, n3)?);
                    node_bodies[k] = names(&cx, start);
                }

                if node_out[k].as_ref().unwrap().is_none() {
                    entry = entry.put("connector", cp.get_str("id").unwrap_or("")).put("block", true).put("bodies", Json::Arr(node_bodies[k].clone()));
                    jrep.push(entry.build());
                    continue;
                }
                // the block does not fit this node: the pairwise connectors below
            }
        }
        let part = match ov.and_then(|o| o.get("connector")) {
            Some(Json::Str(s)) if s == "none" => None,
            Some(p @ Json::Obj(_)) => Some(p),
            _ => default_joint,
        };
        match part {
            None => entry = entry.put("connector", Json::Null),
            Some(part) => {
                let pair = Pair { ia: j.a, ib: j.b, a: &ms[j.a], b: &ms[j.b], u1: j.u1, u2: j.u2, node: j.node, room: j.room };
                match at_joint(&mut cx, state, part, &pair)? {
                    None => {
                        entry = entry.put("connector", part.get_str("id").unwrap_or(""));
                        let both = ov.and_then(|o| o.get("both")).and_then(|b| b.as_bool()).unwrap_or(tee_both);
                        if let (true, Some((u1b, room_b))) = (both && j.corner == "tee", j.alt) {
                            let other = Pair { u1: u1b, room: [room_b, j.room[1]], ..pair };
                            match at_joint(&mut cx, state, part, &other)? {
                                None => entry = entry.put("sides", 2.0),
                                Some(why) => entry = entry.put("sides", 1.0).put("reason", why),
                            }
                        }
                    }
                    Some(why) => entry = entry.put("connector", Json::Null).put("reason", why),
                }
            }
        }
        entry = entry.put("bodies", Json::Arr(names(&cx, start)));
        jrep.push(entry.build());
    }
    if let Some(hw) = hw {
        // down is -Y (the kit is Y up) unless `hardware.down` says otherwise
        let down = hw.get("down").and_then(vec3).and_then(|v| v.try_normalize()).unwrap_or(Vec3::new(0.0, -1.0, 0.0));
        let floor = ms.iter().flat_map(|m| [m.seg.a.dot(down), m.seg.b.dot(down)]).fold(f64::NEG_INFINITY, f64::max);
        for m in ms {
            for k in 0..2 {
                if m.kinds[k] != "free" {
                    continue;
                }
                let (at, out) = if k == 0 { (m.seg.a, -m.d) } else { (m.seg.b, m.d) };
                let on_floor = out.dot(down) > 1.0 - 1e-6 && (at.dot(down) - floor).abs() < 1e-3;
                match (part_of(hw.get("foot")), part_of(hw.get("cap"))) {
                    (Some(foot), _) if on_floor => foot_at(&mut cx, state, foot, m, k)?,
                    (_, Some(cap)) => cap_at(&mut cx, cap, m, k)?,
                    _ => {}
                }
            }
        }
    }
    // the bill of materials: placed parts grouped by id, in order of first use
    let mut order: Vec<usize> = vec![];
    for (i, p) in cx.out.iter().enumerate() {
        if !order.iter().any(|&o| cx.out[o].id == p.id) {
            order.push(i);
        }
    }
    let parts: Vec<Json> = order
        .iter()
        .map(|&o| {
            let first = &cx.out[o];
            let names: Vec<Json> = cx.out.iter().enumerate().filter(|(_, p)| p.id == first.id).map(|(i, _)| Json::Str(format!("{} H{}", f.id, i + 1))).collect();
            Obj::new()
                .put("id", first.id.as_str())
                .put("label", first.label.as_str())
                .put("kind", first.kind.as_str())
                .put("spec", first.spec.as_str())
                .put("qty", names.len() as f64)
                .put("bodies", Json::Arr(names))
                .build()
        })
        .collect();
    let bodies = cx.out.into_iter().map(|p| p.body).collect();
    Ok(Some((bodies, cx.drills, Obj::new().put("parts", Json::Arr(parts)).put("joints", Json::Arr(jrep)).build())))
}
