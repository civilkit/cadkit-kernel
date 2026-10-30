//! Independent solid validator. Check logic adapted from brepkit v2.129.15
//! (MIT OR Apache-2.0), crates/check/src/validate/{shell,wire,edge,solid}.rs,
//! re-expressed over ck-topo bodies and ck-geom evaluation.

mod edge;
pub use edge::{face_deviation, vertex_deviation};
mod shell;
mod solid;
pub mod tessellation;
mod wire;

use std::cell::{Cell, RefCell};

use ck_topo::body::{Body, EdgeId, FaceId};

/// One validation finding. `code` is a stable machine id, `entity` a face
/// id (persistent name) or `edge#N[faceA|faceB]`, `detail` human text.
#[derive(Debug, Clone, PartialEq)]
pub struct Issue {
    pub code: &'static str,
    pub entity: String,
    pub detail: String,
}

pub(crate) fn face_ent(b: &Body, f: FaceId) -> String {
    b.face_id_str(f)
}
pub(crate) fn edge_ent(b: &Body, e: EdgeId) -> String {
    let fs: Vec<String> = b.edge_faces(e).into_iter().map(|f| b.face_id_str(f)).collect();
    format!("edge#{}[{}]", e.idx(), fs.join("|"))
}

/// Default cap (mm) on an edge / vertex tolerance: a larger one is an error.
pub const DEFAULT_TOL_CAP: f64 = 0.05;

/// Largest edge or vertex tolerance in the body (0 when every entity is exact).
pub fn tolerance_used(b: &Body) -> f64 {
    let e = b.edge_ids().map(|e| b.edge(e).tol);
    let v = b.vertex_ids().map(|v| b.vertex(v).tol);
    e.chain(v).fold(0.0, f64::max)
}

/// Run every check with model tolerance `tol` (mm). Empty result = valid.
pub fn validate(b: &Body, tol: f64) -> Vec<Issue> {
    validate_cap(b, tol, DEFAULT_TOL_CAP)
}

/// Only the edge / vertex deviation checks of `validate_cap` (cheap next to the
/// loop and solid checks): used to re-judge a body after tolerances were set.
pub fn validate_edges(b: &Body, tol: f64, cap: f64) -> Vec<Issue> {
    let mut out = vec![];
    edge::check(b, tol, cap, &Ctl::unlimited(), &mut out);
    out
}

/// `validate` with an explicit cap on edge / vertex tolerances.
pub fn validate_cap(b: &Body, tol: f64, cap: f64) -> Vec<Issue> {
    validate_ctl(b, tol, cap, false, &Ctl::unlimited())
}

/// Wall-clock control of one validation: an optional time limit that the checks poll, and the record of how long each
/// check took and which ones did not (fully) run. The clock is the host clock of `ck_core::budget` (none installed, as in
/// native tests: no limit, all timings 0).
pub struct Ctl {
    limit_ms: f64,
    start: Option<f64>,
    tick: Cell<u32>,
    hit: Cell<bool>,
    stats: RefCell<Vec<(&'static str, f64)>>,
    partial: RefCell<Vec<&'static str>>,
    not_run: RefCell<Vec<&'static str>>,
}

impl Ctl {
    /// A limit of `limit_ms` (`None`, zero, negative or non-finite: unlimited).
    pub fn new(limit_ms: Option<f64>) -> Ctl {
        let limit_ms = limit_ms.filter(|m| m.is_finite() && *m > 0.0).unwrap_or(0.0);
        Ctl {
            limit_ms,
            start: ck_core::budget::mark(),
            tick: Cell::new(0),
            hit: Cell::new(false),
            stats: RefCell::new(vec![]),
            partial: RefCell::new(vec![]),
            not_run: RefCell::new(vec![]),
        }
    }
    pub fn unlimited() -> Ctl {
        Ctl::new(None)
    }
    /// The limit in ms (0: none).
    pub fn limit_ms(&self) -> f64 {
        self.limit_ms
    }
    /// Ms since the validation started (0 without a clock).
    pub fn elapsed_ms(&self) -> f64 {
        ck_core::budget::since(self.start)
    }
    /// Has the limit run out? Reads the clock.
    pub fn expired(&self) -> bool {
        if self.limit_ms <= 0.0 || self.start.is_none() {
            return false;
        }
        if !self.hit.get() && self.elapsed_ms() > self.limit_ms {
            self.hit.set(true);
        }
        self.hit.get()
    }
    /// [`Ctl::expired`] for use inside a loop: the clock is read every 32nd call.
    pub fn poll(&self) -> bool {
        if self.limit_ms <= 0.0 {
            return false;
        }
        let t = self.tick.get().wrapping_add(1);
        self.tick.set(t);
        self.hit.get() || (t % 32 == 0 && self.expired())
    }
    /// Run check `name` unless the limit has already run out (then it is listed as not run). Its time is recorded; when
    /// the limit ran out while it ran (it stopped early) it is listed as partial.
    pub fn stage<T>(&self, name: &'static str, f: impl FnOnce() -> T) -> Option<T> {
        if self.expired() {
            self.not_run.borrow_mut().push(name);
            return None;
        }
        let t = ck_core::budget::mark();
        let r = f();
        self.stats.borrow_mut().push((name, ck_core::budget::since(t)));
        if self.hit.get() {
            self.partial.borrow_mut().push(name);
        }
        Some(r)
    }
    /// Record a check that was left out on request (quick mode), so the reply names it.
    pub fn skip(&self, name: &'static str) {
        self.not_run.borrow_mut().push(name);
    }
    /// `(name, ms)` per check that ran, in order.
    pub fn stats(&self) -> Vec<(&'static str, f64)> {
        self.stats.borrow().clone()
    }
    /// Checks that stopped early because the limit ran out.
    pub fn partial(&self) -> Vec<&'static str> {
        self.partial.borrow().clone()
    }
    /// Checks that did not start (limit run out, or left out by the quick mode).
    pub fn not_run(&self) -> Vec<&'static str> {
        self.not_run.borrow().clone()
    }
    /// Did the limit run out at all?
    pub fn exceeded(&self) -> bool {
        self.hit.get()
    }
}

/// Only the planar-face loop checks (`LOOP_CROSS`, `LOOP_SELF_INTERSECT`, loop closure): a cheap test
/// that a boolean did not leave overlapping boundaries.
pub fn loop_issues(b: &Body, tol: f64) -> Vec<Issue> {
    let mut out = vec![];
    wire::check(b, tol, true, &Ctl::new(None), &mut out);
    out
}

/// `validate_cap` under `ctl`: each check is timed and skipped once the limit ran out. `quick` runs only the topology
/// and bounding checks (shell, loop closure, edge and vertex deviation); it leaves out the planar loop self-intersection
/// test (`loopCrossings`) and the solid checks (`solids`), which integrate the volume.
pub fn validate_ctl(b: &Body, tol: f64, cap: f64, quick: bool, ctl: &Ctl) -> Vec<Issue> {
    let mut out = vec![];
    ctl.stage("shell", || shell::check(b, &mut out));
    ctl.stage("loops", || wire::check(b, tol, !quick, ctl, &mut out));
    ctl.stage("edges", || edge::check(b, tol, cap, ctl, &mut out));
    if quick {
        ctl.skip("loopCrossings");
        ctl.skip("solids");
    } else {
        ctl.stage("solids", || solid::check(b, &mut out));
    }
    out
}

#[cfg(test)]
mod tests;
