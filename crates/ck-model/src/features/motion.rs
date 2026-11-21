//! Motion: drive an assembly through poses (`solvePoses`) and find where a driver stops (`sweepDriver`).
//!
//! A *driver* is a mate with one value a user can change without editing the model: the angle of a hinge or `angle` mate (degrees),
//! the slide of a slot or the value of a `distance` mate (mm), the drive angle of a gear, screw or rackPinion (degrees). The value
//! replaces the mate's own for one solve and is never written back. Hinge and slot values are measured from the pose the mate was
//! created at (0 = assembled pose); the other kinds keep their meaning from the mate.
//!
//! [`Runner`] works on a copy of the regenerated body and assembly (no feature is rebuilt) and warm-starts every pose from the
//! previous one, so a sweep costs one mate solve per pose. A driver a pose does not list is not driven: in the first pose it is at its
//! assembled value, later it stays where the pose before left it (a request lists every driver it wants moved).
//!
//! Contact (the default, see [`ContactOpts`]) comes from [`super::motion_contact`]: the surfaces of two components crossing, or
//! closer than the clearance, stops the motion. [`sweep`] walks a driver from one value toward another in coarse steps and bisects
//! the first blocked step to a tolerance; the stop is a mate `limit`, a `contact`, the `solver` failing, a `branchJump` of a closed
//! loop, or the end of the `range`.

use super::mate::{comp_box, solve_all, Assembly, Xf};
use super::mate_kinds::{limit_violation, restore_places, LimitHit};
use super::motion_contact::{Contact, Kind, Scene};
use super::{Brep, MateKind as K};
use crate::regen::{self, FeatureError};
use ck_core::linalg::Vec3;
use ck_json::{Json, Obj};
use ck_topo::body::Body;

pub const NOT_DRIVABLE: &str = "notDrivable";
pub const UNKNOWN_DRIVER: &str = "unknownDriver";

/// What a mate can be driven by: `("angle", "deg")` or `("distance", "mm")`;
/// `None` for mates that cannot be driven (the geometric ones, `limits`).
pub fn driver_kind(kind: K) -> Option<(&'static str, &'static str)> {
    match kind {
        K::Hinge | K::Angle | K::Gear | K::Screw | K::RackPinion => Some(("angle", "deg")),
        K::Slot | K::Distance => Some(("distance", "mm")),
        _ => None,
    }
}

/// The drivable mates of an assembly: `[{mate, kind, unit, value}]`, `value` the
/// assembled value (0 for the kinds measured from the creation pose).
pub fn drivers_json(asm: &Assembly) -> Json {
    let mut out = vec![];
    for m in &asm.mates {
        let Some((kind, unit)) = driver_kind(m.kind) else { continue };
        let value = match m.kind {
            K::Angle => m.ext.as_ref().and_then(|e| e.angle).unwrap_or(0.0).to_degrees(),
            K::Distance => m.distance,
            _ => 0.0,
        };
        out.push(Obj::new().put("mate", m.id.as_str()).put("kind", kind).put("unit", unit).put("value", value).build());
    }
    Json::Arr(out)
}

/// Contact checking of a request: on by default (a contact is a stop), `clearance` mm, `chord` the mesh chord height (default
/// 1e-4 of the assembly's size, between 0.002 and 0.1 mm).
#[derive(Debug, Clone, Copy)]
pub struct ContactOpts {
    pub check: bool,
    pub clearance: f64,
    pub chord: Option<f64>,
}
impl Default for ContactOpts {
    fn default() -> Self {
        ContactOpts { check: true, clearance: 0.0, chord: None }
    }
}
impl ContactOpts {
    pub fn off() -> Self {
        ContactOpts { check: false, ..Self::default() }
    }
    fn from_json(msg: &Json) -> Self {
        let mut o = ContactOpts::default();
        if let Some(b) = msg.get_bool("contact") {
            o.check = b;
        }
        if let Some(c) = msg.get_f64("clearance").filter(|c| c.is_finite() && *c >= 0.0) {
            o.clearance = c;
        }
        o.chord = msg.get_f64("chord").filter(|c| c.is_finite() && *c > 0.0);
        o
    }
}

fn default_chord(body: &Body) -> f64 {
    let (lo, hi) = body.bbox();
    ((hi - lo).len() * 1e-4).clamp(0.002, 0.1)
}

/// One solved pose.
pub struct Pose {
    /// `None` when solved; else `(error code, mate that was driven last or failed)`.
    pub error: Option<(String, Option<String>)>,
    pub places: Vec<(String, Xf)>,
    /// The first thing the pose touches (a crossing before a near miss), when contact checking is on.
    pub contact: Option<ContactInfo>,
}

#[derive(Debug, Clone)]
pub struct ContactInfo {
    pub a: String,
    pub b: String,
    pub kind: Kind,
    pub distance: f64,
    pub at: [f64; 3],
}

fn contact_info(scene: &Scene, c: &Contact) -> ContactInfo {
    let name = |k: usize| if k < scene.components() { scene.id(k).to_string() } else { "(fixed scenery)".to_string() };
    ContactInfo { a: name(c.a), b: name(c.b), kind: c.kind, distance: c.distance, at: [c.at.x, c.at.y, c.at.z] }
}

impl ContactInfo {
    pub fn to_json(&self) -> Json {
        Obj::new()
            .put("a", self.a.as_str())
            .put("b", self.b.as_str())
            .put("kind", if self.kind == Kind::Cross { "contact" } else { "near" })
            .put("distance", self.distance)
            .put("at", Json::Arr(self.at.iter().map(|v| Json::Num(*v)).collect()))
            .build()
    }
}

impl Pose {
    pub fn to_json(&self) -> Json {
        let places = self
            .places
            .iter()
            .map(|(id, p)| Obj::new().put("id", id.as_str()).put("place", regen::frame_json(&p.to_xform().to_frame())).build())
            .collect();
        let mut o = Obj::new().put("ok", self.error.is_none()).put("places", Json::Arr(places));
        if let Some((code, mate)) = &self.error {
            o = o.put("error", code.as_str());
            if let Some(m) = mate {
                o = o.put("mate", m.as_str());
            }
        }
        if let Some(c) = &self.contact {
            o = o.put("contact", c.to_json());
        }
        o.build()
    }
}

/// A working copy of an assembly that is driven from pose to pose.
pub struct Runner {
    pub br: Brep,
    natural: Vec<Natural>,
    mates: Vec<(String, K, usize)>,
}

/// Natural (undriven) values of the mates, restored when a pose does not list them.
struct Natural {
    angle: Option<f64>,
    distance: f64,
}

impl Runner {
    pub fn new(body: &Body, asm: &Assembly) -> Runner {
        let mut br = Brep::new();
        br.body = Some(body.clone());
        br.asm = asm.clone();
        // a closed loop converges over many small moves
        br.asm.move_budget = 1000;
        br.asm.soft = true;
        let natural = asm.mates.iter().map(|m| Natural { angle: m.ext.as_ref().and_then(|e| e.angle), distance: m.distance }).collect();
        let mates = asm.mates.iter().map(|m| (m.id.clone(), m.kind, m.mover)).collect();
        Runner { br, natural, mates }
    }
    /// The drivable mate `id`, as an index, or `(code, id)` for an unknown or undrivable one.
    pub fn driver(&self, id: &str) -> Result<usize, (String, String)> {
        let Some(i) = self.mates.iter().position(|m| m.0 == id) else { return Err((UNKNOWN_DRIVER.into(), id.into())) };
        if driver_kind(self.mates[i].1).is_none() {
            return Err((NOT_DRIVABLE.into(), id.into()));
        }
        Ok(i)
    }
    fn set_drive(&mut self, mate: usize, value: Option<f64>) {
        let m = &mut self.br.asm.mates[mate];
        let Some(e) = m.ext.as_deref_mut() else { return };
        // a driver the pose does not list is not driven: a hinge or slot stays where it is, the others follow their mates
        match m.kind {
            K::Hinge => e.drive = value.map(f64::to_radians),
            K::Slot => e.drive = value,
            K::Gear | K::Screw | K::RackPinion => e.angle = value.map(f64::to_radians).or(self.natural[mate].angle),
            K::Angle => e.angle = value.map(f64::to_radians).or(self.natural[mate].angle),
            K::Distance => m.distance = value.unwrap_or(self.natural[mate].distance),
            _ => {}
        }
    }
    /// Solve one pose from the current state. The error is `(code, driver last set)`.
    pub fn solve(&mut self, pose: &[(String, f64)]) -> Result<(), (String, Option<String>)> {
        let mut first = None;
        let mut last = None;
        self.br.asm.frozen.clear();
        for i in 0..self.mates.len() {
            if driver_kind(self.mates[i].1).is_none() {
                continue;
            }
            let v = pose.iter().find(|(id, _)| *id == self.mates[i].0).map(|(_, v)| *v);
            self.set_drive(i, v);
            if v.is_some() {
                first.get_or_insert(self.mates[i].2);
                last = Some(self.mates[i].0.clone());
                // a hinge or slot is driven by placing its mover; the others then settle around it
                if matches!(self.mates[i].1, K::Hinge | K::Slot) {
                    self.br.asm.frozen.push(self.mates[i].2);
                }
            }
        }
        let r: Result<(), FeatureError> = solve_all(&mut self.br, first.unwrap_or(0));
        r.map_err(|e| (e.code.to_string(), last))
    }
    pub fn places(&self) -> Vec<Xf> {
        self.br.asm.components.iter().map(|c| c.place).collect()
    }
    pub fn restore(&mut self, places: &[Xf]) {
        restore_places(&mut self.br, places);
    }
    fn named(&self) -> Vec<(String, Xf)> {
        self.br.asm.components.iter().map(|c| (c.id.clone(), c.place)).collect()
    }
}

/// Solve `poses` (each `[(mate id, value)]`) on a copy of `body` / `asm`, without contact checking.
/// An unknown or undrivable mate fails the whole request (`Err((code, mate))`);
/// a pose the mates cannot satisfy is reported in that pose and the sweep goes on
/// from the last good pose.
pub fn solve_poses(body: &Body, asm: &Assembly, poses: &[Vec<(String, f64)>]) -> Result<Vec<Pose>, (String, String)> {
    solve_poses_with(body, asm, poses, &ContactOpts::off())
}

/// [`solve_poses`] with the contact options: each solved pose carries the first contact (or near miss) it has.
pub fn solve_poses_with(body: &Body, asm: &Assembly, poses: &[Vec<(String, f64)>], opts: &ContactOpts) -> Result<Vec<Pose>, (String, String)> {
    let mut run = Runner::new(body, asm);
    for p in poses {
        for (id, _) in p {
            run.driver(id)?;
        }
    }
    let scene = opts.check.then(|| Scene::build(body, asm, opts.chord.unwrap_or_else(|| default_chord(body))));
    let exempt = scene.as_ref().map(|s| s.baseline(&run.places())).unwrap_or_default();
    let mut out = Vec::with_capacity(poses.len());
    for p in poses {
        let r = run.solve(p);
        let contact = match (&scene, &r) {
            (Some(s), Ok(())) => s.contacts(&run.places(), opts.clearance, &exempt, false).first().map(|c| contact_info(s, c)),
            _ => None,
        };
        out.push(Pose { error: r.err(), places: run.named(), contact });
    }
    Ok(out)
}

// ------------------------------------------------------------------------------ sweepDriver

/// Options of [`sweep`].
#[derive(Debug, Clone)]
pub struct SweepOpts {
    pub contact: ContactOpts,
    /// Coarse steps from `from` to `to` (default 200).
    pub steps: usize,
    /// Bisection tolerance, in the driver's unit (default 0.01).
    pub tol: f64,
    /// Other drivers held at a value (the rest stay at their assembled values).
    pub hold: Vec<(String, f64)>,
    /// A component moving farther than this (mm) between two steps while its speed jumps is a `branchJump` (default: none under a
    /// quarter of the component's diagonal).
    pub branch_jump: Option<f64>,
}
impl Default for SweepOpts {
    fn default() -> Self {
        SweepOpts { contact: ContactOpts::default(), steps: 200, tol: 0.01, hold: vec![], branch_jump: None }
    }
}

/// Why a sweep ended.
#[derive(Debug, Clone)]
pub enum StopReason {
    Range,
    Limit(LimitHit),
    Contact(ContactInfo),
    Solver { code: String },
    BranchJump { component: String },
}

pub struct SweepResult {
    pub driver: String,
    pub from: f64,
    pub to: f64,
    /// The farthest value reached freely (within `tol` of the stop).
    pub reached: f64,
    pub stop: StopReason,
    /// Pairs that already touched at the start (ignored by the sweep).
    pub start_contacts: Vec<(String, String)>,
    pub poses_solved: usize,
}

/// Walk `driver` from `from` toward `to` and find where it stops.
pub fn sweep(body: &Body, asm: &Assembly, driver: &str, from: f64, to: f64, o: &SweepOpts) -> Result<SweepResult, (String, String)> {
    let mut run = Runner::new(body, asm);
    run.driver(driver)?;
    for (id, _) in &o.hold {
        run.driver(id)?;
    }
    let scene = o.contact.check.then(|| Scene::build(body, asm, o.contact.chord.unwrap_or_else(|| default_chord(body))));
    let pose_at = |v: f64| -> Vec<(String, f64)> {
        let mut p: Vec<(String, f64)> = o.hold.iter().filter(|(id, _)| id != driver).cloned().collect();
        p.push((driver.to_string(), v));
        p
    };
    let mut solved = 0usize;
    let diag: Vec<f64> = run.br.asm.components.iter().map(|c| comp_box(body, &c.range).map_or(0.0, |(lo, hi)| (hi - lo).len())).collect();
    let corners: Vec<Vec<Vec3>> = run
        .br
        .asm
        .components
        .iter()
        .map(|c| match comp_box(body, &c.range) {
            Some((lo, hi)) => (0..8).map(|k| Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z })).collect(),
            None => vec![],
        })
        .collect();
    let base_places = run.places();
    let start_blocked = |run: &mut Runner| -> Option<StopReason> {
        run.solve(&pose_at(from)).err().map(|(code, _)| StopReason::Solver { code })
    };
    solved += 1;
    if let Some(stop) = start_blocked(&mut run) {
        // classify a failing start like any other stop
        let stop = classify_failure(&mut run, &pose_at(from), stop);
        return Ok(SweepResult { driver: driver.into(), from, to, reached: from, stop, start_contacts: vec![], poses_solved: solved });
    }
    let start_places = run.places();
    let exempt = scene.as_ref().map(|s| s.baseline(&start_places)).unwrap_or_default();
    let start_contacts = exempt
        .all
        .iter()
        .map(|&(i, j)| {
            let name = |k: usize| scene.as_ref().map_or(String::new(), |s| if k < s.components() { s.id(k).to_string() } else { "(fixed scenery)".into() });
            (name(i), name(j))
        })
        .collect();
    let contact_at = |run: &Runner| -> Option<ContactInfo> {
        let s = scene.as_ref()?;
        s.contacts(&run.places(), o.contact.clearance, &exempt, false).first().map(|c| contact_info(s, c))
    };
    // contact already at the start: nothing can be reached
    if let Some(c) = contact_at(&run) {
        return Ok(SweepResult { driver: driver.into(), from, to, reached: from, stop: StopReason::Contact(c), start_contacts, poses_solved: solved });
    }
    let steps = o.steps.max(1);
    let mut free = from;
    let mut free_places = start_places;
    let mut prev_disp: Option<f64> = None;
    let mut prev_places = free_places.clone();
    for k in 1..=steps {
        let v = if k == steps { to } else { from + (to - from) * k as f64 / steps as f64 };
        let blocked = probe(&mut run, &pose_at(v), &contact_at, &mut solved);
        let step_free = match &blocked {
            Some(_) => None,
            None => {
                // closed loops can flip to another valid pose between steps
                let now = run.places();
                let jump = jump_of(&base_places, &prev_places, &now, &corners, &diag, o.branch_jump, prev_disp);
                match jump {
                    Some((comp, _)) => Some(Err(comp)),
                    None => {
                        prev_disp = Some(max_disp(&base_places, &prev_places, &now, &corners));
                        Some(Ok(now))
                    }
                }
            }
        };
        match step_free {
            Some(Ok(now)) => {
                free = v;
                free_places = now.clone();
                prev_places = now;
            }
            Some(Err(comp)) => {
                // the pose before the jump is the last free one
                run.restore(&free_places);
                return Ok(SweepResult { driver: driver.into(), from, to, reached: free, stop: StopReason::BranchJump { component: scene_name(&run, comp) }, start_contacts, poses_solved: solved });
            }
            None => {
                // bisect between the last free value and this blocked one
                let (mut lo, mut hi) = (free, v);
                let mut hi_stop = blocked.unwrap();
                while (hi - lo).abs() > o.tol {
                    let mid = (lo + hi) * 0.5;
                    run.restore(&free_places);
                    match probe(&mut run, &pose_at(mid), &contact_at, &mut solved) {
                        Some(s) => {
                            hi = mid;
                            hi_stop = s;
                        }
                        None => lo = mid,
                    }
                }
                run.restore(&free_places);
                let stop = match hi_stop {
                    Blocked::Contact(c) => StopReason::Contact(c),
                    Blocked::Solver(code) => {
                        // a limits mate that would be violated, or the solver failing for another reason
                        run.restore(&free_places);
                        classify_failure(&mut run, &pose_at(hi), StopReason::Solver { code })
                    }
                };
                return Ok(SweepResult { driver: driver.into(), from, to, reached: lo, stop, start_contacts, poses_solved: solved });
            }
        }
    }
    Ok(SweepResult { driver: driver.into(), from, to, reached: to, stop: StopReason::Range, start_contacts, poses_solved: solved })
}

enum Blocked {
    Contact(ContactInfo),
    Solver(String),
}

fn probe(run: &mut Runner, pose: &[(String, f64)], contact: &dyn Fn(&Runner) -> Option<ContactInfo>, solved: &mut usize) -> Option<Blocked> {
    *solved += 1;
    if let Err((code, _)) = run.solve(pose) {
        return Some(Blocked::Solver(code));
    }
    contact(run).map(Blocked::Contact)
}

/// A pose that failed to solve: when the same pose solves with the `limits` mates ignored and one of them is beyond its bound,
/// the stop is that limit; otherwise the solver.
fn classify_failure(run: &mut Runner, pose: &[(String, f64)], dflt: StopReason) -> StopReason {
    let saved = run.places();
    run.br.asm.skip_limits = true;
    let ok = run.solve(pose).is_ok();
    let hit = if ok { limit_violation(&run.br) } else { None };
    run.br.asm.skip_limits = false;
    run.restore(&saved);
    match hit {
        Some(h) => StopReason::Limit(h),
        None => dflt,
    }
}

fn scene_name(run: &Runner, k: usize) -> String {
    run.br.asm.components.get(k).map_or(String::new(), |c| c.id.clone())
}

fn max_disp(base: &[Xf], a: &[Xf], b: &[Xf], corners: &[Vec<Vec3>]) -> f64 {
    let mut m = 0.0f64;
    for k in 0..a.len().min(b.len()) {
        m = m.max(disp_of(&base[k], &a[k], &b[k], &corners[k]));
    }
    m
}

/// Farthest a corner of the component's box (taken at the assembled pose, `base`) moves between the placements `a` and `b`.
fn disp_of(base: &Xf, a: &Xf, b: &Xf, corners: &[Vec3]) -> f64 {
    let (at_a, at_b) = (a.compose(&super::mate_kinds::inv(base)), b.compose(&super::mate_kinds::inv(base)));
    corners.iter().map(|c| at_b.point(*c).dist(at_a.point(*c))).fold(0.0, f64::max)
}

/// `(component, displacement)` of a step that looks like a flip to another solution of a closed loop: a component moved far (a
/// quarter of its size, or `limit`) and several times farther than in the step before.
fn jump_of(base: &[Xf], a: &[Xf], b: &[Xf], corners: &[Vec<Vec3>], diag: &[f64], limit: Option<f64>, prev: Option<f64>) -> Option<(usize, f64)> {
    for k in 0..a.len().min(b.len()) {
        let d = disp_of(&base[k], &a[k], &b[k], &corners[k]);
        let big = limit.unwrap_or(0.25 * diag[k]);
        if d > big && prev.map_or(limit.is_some(), |p| d > 4.0 * p.max(1e-9)) {
            return Some((k, d));
        }
    }
    None
}

impl SweepResult {
    pub fn to_json(&self) -> Json {
        let mut stop = Obj::new().put("value", self.reached);
        stop = match &self.stop {
            StopReason::Range => stop.put("reason", "range"),
            StopReason::Limit(h) => stop.put("reason", "limit").put("mate", h.mate.as_str()).put("bound", h.bound).put("bound_value", h.value),
            StopReason::Contact(c) => {
                let near = c.kind == Kind::Near;
                stop.put("reason", "contact").put("a", c.a.as_str()).put("b", c.b.as_str()).put("near", near).put("distance", c.distance).put("at", Json::Arr(c.at.iter().map(|v| Json::Num(*v)).collect()))
            }
            StopReason::Solver { code } => stop.put("reason", "solver").put("error", code.as_str()),
            StopReason::BranchJump { component } => stop.put("reason", "branchJump").put("component", component.as_str()),
        };
        Obj::new()
            .put("driver", self.driver.as_str())
            .put("from", self.from)
            .put("to", self.to)
            .put("reached", self.reached)
            .put("stop", stop.build())
            .put("startContacts", Json::Arr(self.start_contacts.iter().map(|(a, b)| Json::Arr(vec![Json::Str(a.clone()), Json::Str(b.clone())])).collect()))
            .put("posesSolved", self.poses_solved as f64)
            .build()
    }
}

// ------------------------------------------------------------------------------ requests

fn parse_poses(msg: &Json) -> Vec<Vec<(String, f64)>> {
    let mut poses = vec![];
    for p in msg.get_arr("poses").unwrap_or(&[]) {
        let mut one = vec![];
        if let Json::Obj(kv) = p {
            for (k, v) in kv {
                if let Some(x) = v.as_f64() {
                    one.push((k.clone(), x));
                }
            }
        }
        poses.push(one);
    }
    poses
}

fn err_json(code: &str, mate: &str) -> Json {
    Obj::new().put("error", code).put("mate", mate).build()
}

/// The `solvePoses` reply: `{drivers, poses:[{ok, places, contact?, error?, mate?}]}`, or
/// `{error, mate}` for an unknown / undrivable driver. `contact` (default true), `clearance` and `chord` in the request.
pub fn reply(body: &Body, asm: &Assembly, msg: &Json) -> Json {
    match solve_poses_with(body, asm, &parse_poses(msg), &ContactOpts::from_json(msg)) {
        Ok(ps) => Obj::new().put("drivers", drivers_json(asm)).put("poses", Json::Arr(ps.iter().map(Pose::to_json).collect())).build(),
        Err((code, mate)) => err_json(&code, &mate),
    }
}

/// The `sweepDriver` reply: `{driver, from, to, reached, stop:{reason, value, ...}, startContacts, posesSolved}`.
/// Request: `driver`, `from` (default 0), `to`, `clearance`, `contact`, `steps`, `tol`, `hold` (`{mate: value}`), `chord`.
pub fn sweep_reply(body: &Body, asm: &Assembly, msg: &Json) -> Json {
    let (Some(driver), Some(to)) = (msg.get_str("driver"), msg.get_f64("to").filter(|v| v.is_finite())) else {
        return err_json("missingDriver", "");
    };
    let from = msg.get_f64("from").filter(|v| v.is_finite()).unwrap_or(0.0);
    let mut o = SweepOpts { contact: ContactOpts::from_json(msg), ..SweepOpts::default() };
    if let Some(s) = msg.get_f64("steps").filter(|s| *s >= 1.0) {
        o.steps = (s as usize).min(5000);
    }
    if let Some(t) = msg.get_f64("tol").filter(|t| *t > 0.0) {
        o.tol = t;
    }
    o.branch_jump = msg.get_f64("branchJump").filter(|v| *v > 0.0);
    if let Some(Json::Obj(kv)) = msg.get("hold") {
        for (k, v) in kv {
            if let Some(x) = v.as_f64() {
                o.hold.push((k.clone(), x));
            }
        }
    }
    match sweep(body, asm, driver, from, to, &o) {
        Ok(r) => r.to_json(),
        Err((code, mate)) => err_json(&code, &mate),
    }
}
