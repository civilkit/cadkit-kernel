//! One running regeneration of the feature list the read-back builds, extended step by step.
//!
//! Reading a history back regenerates the document so far at many points: before a blend (to name its supports in
//! the rebuilt body), before an extrude (the material it joins or cuts), at the end (the whole body), and once more
//! with a body snapshot after every step (the prefix checks). Each of those used to start over from the first
//! feature, so the cost of a file with `n` blends grew with `n` squared. The features only ever grow at the end, so
//! the regeneration is resumed instead (`ck_model::regen::regen_from`, as the app's regen cache does): the executor
//! and the result so far stay, the features not yet run are run, and a feature list that no longer starts with the
//! features already run starts a new regeneration. The body after any feature is the one a fresh regeneration of
//! the list up to it makes (a feature sees the features before it only), which is what the checks read.

use ck_json::{Json, Obj};
use ck_model::features::{Brep, BrepSnap};
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_model::Part;
use ck_topo::body::Body;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}

/// `ck_recog::rebuild`'s executor, keeping the body after every feature it ran.
struct Exec {
    brep: Brep,
    labels: Noop,
    after: HashMap<String, Option<Body>>,
    /// The state after the last feature run (shared with the checkpoint recorder).
    last: Rc<RefCell<Option<BrepSnap>>>,
    /// The body an `importBody` feature puts in place (the read-back has no file data: `undo.rs`).
    start: Option<Rc<Body>>,
}

impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn edge_line(&mut self, s: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(ck_core::linalg::Vec3, ck_core::linalg::Vec3)> {
        self.brep.edge_line(s, faces, at)
    }
    fn edge_seg(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        self.brep.edge_seg(s, faces, at)
    }
    fn vertex_point(&mut self, s: &regen::RegenState, faces: &[String], at: Option<ck_core::linalg::Vec3>) -> Option<ck_core::linalg::Vec3> {
        self.brep.vertex_point(s, faces, at)
    }
    fn face_surf(&mut self, s: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.brep.face_surf(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        let r = match &self.start {
            // (what `importBody` with `replace` does with the file's solid, which the start body already is)
            Some(b) if f.type_name == "importBody" => {
                self.brep.body = None;
                self.brep.surfs.clear();
                // (the start body's faces are named for `F1`: named for this feature, as the file's import names them)
                let mut b = (**b).clone();
                if f.id != "F1" {
                    for face in b.faces.iter_mut().filter(|x| x.name.feature == "F1") {
                        face.name.feature = f.id.clone();
                    }
                }
                self.brep.combine(&b, ck_model::features::extrude::Combine::Separate, false)
            }
            _ => self.brep.exec(s, f),
        };
        *self.last.borrow_mut() = Some(self.brep.save(None));
        self.after.insert(f.id.clone(), self.brep.body.clone());
        r
    }
}

/// Records the state after every feature, to start again from the longest prefix a later list shares.
struct Rec {
    cps: Rc<RefCell<Vec<(RegenResult, Option<BrepSnap>)>>>,
    last: Rc<RefCell<Option<BrepSnap>>>,
}

impl regen::Checkpoint for Rec {
    fn after(&mut self, _index: usize, _ran: bool, state: &regen::RegenState, _exec: &mut dyn FeatureExec) {
        self.cps.borrow_mut().push((state.result.clone(), self.last.borrow().clone()));
    }
}

pub struct Session {
    ex: Exec,
    result: RegenResult,
    /// The features run so far.
    done: Vec<Json>,
    /// The result and executor state after each of them.
    cps: Rc<RefCell<Vec<(RegenResult, Option<BrepSnap>)>>>,
}

impl Default for Session {
    fn default() -> Self {
        Session::new()
    }
}

impl Session {
    pub fn new() -> Session {
        Session::with_start(None)
    }

    /// A session in which an `importBody` feature puts `start` in place (the file's solid as the document's
    /// `importBody` reads it: `Tagged::start`).
    pub fn with_start(start: Option<Rc<Body>>) -> Session {
        Session { ex: Exec { brep: Brep::new(), labels: Noop, after: HashMap::new(), last: Rc::default(), start }, result: RegenResult::default(), done: vec![], cps: Rc::default() }
    }

    /// Run the features of `feats` not yet run (all of them when `feats` does not start with the ones run). With
    /// `exact`, the session then stands at the end of `feats`: a list that is a proper prefix of the ones run
    /// starts over.
    fn run(&mut self, feats: &[Json], exact: bool) -> Result<(), String> {
        // the features both lists start with
        let c = self.done.iter().zip(feats).take_while(|(a, b)| a == b).count();
        if c == feats.len() && c < self.done.len() && !exact {
            return Ok(());
        }
        if c < self.done.len() {
            // start again from the state after feature `c - 1` (the later features are run again)
            let cp = if c > 0 { self.cps.borrow().get(c - 1).cloned() } else { None };
            match cp {
                Some((result, snap)) => {
                    self.cps.borrow_mut().truncate(c);
                    self.done.truncate(c);
                    self.result = result;
                    match snap {
                        Some(sn) => self.ex.brep.load(&sn),
                        None => self.ex.brep = Brep::new(),
                    }
                    *self.ex.last.borrow_mut() = snap_of(&self.ex.brep, &self.cps);
                }
                None => *self = Session::with_start(self.ex.start.clone()),
            }
        }
        if self.done.len() == feats.len() {
            return Ok(());
        }
        let part_json = Obj::new().put("format", "ck-part").put("version", 1.0).put("features", Json::Arr(feats.to_vec())).build();
        let part = Part::from_json(&part_json)?;
        let resume = (!self.done.is_empty()).then(|| regen::Resume { next: self.done.len(), result: std::mem::take(&mut self.result), axes: vec![] });
        let mut rec = Rec { cps: self.cps.clone(), last: self.ex.last.clone() };
        self.result = regen::regen_from(&part, &RegenOpts::default(), &mut self.ex, resume, Some(&mut rec));
        self.done = feats.to_vec();
        Ok(())
    }

    /// The body `feats` make, and the regen errors (`feature: code`), as `ck_recog::rebuild::rebuild(feats)`.
    pub fn rebuild(&mut self, feats: &[Json]) -> Result<(Option<Body>, Vec<String>), String> {
        self.run(feats, true)?;
        Ok((self.ex.brep.body.clone(), self.errors()))
    }

    fn errors(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect()
    }

    /// As [`crate::prefix::snapshots`]: a body snapshot after each step (`ranges[j]` = the step's features) and the
    /// regen errors per step.
    pub fn snapshots(&mut self, feats: &[Json], ranges: &[(usize, usize)]) -> Result<(Vec<Option<Body>>, Vec<Vec<String>>), String> {
        self.run(feats, false)?;
        let mut step_of = HashMap::new();
        for (j, &(a, b)) in ranges.iter().enumerate() {
            for i in a..b.min(feats.len()) {
                if let Some(id) = feats[i].get_str("id") {
                    step_of.insert(id.to_string(), j);
                }
            }
        }
        let mut snaps: Vec<Option<Body>> = vec![None; ranges.len()];
        let mut ran = vec![false; ranges.len()];
        // (the last feature of a step to run wins)
        for f in feats {
            if let (Some(id), Some(&j)) = (f.get_str("id"), f.get_str("id").and_then(|i| step_of.get(i))) {
                if let Some(b) = self.ex.after.get(id) {
                    snaps[j] = b.clone();
                    ran[j] = true;
                }
            }
        }
        let mut errs = vec![vec![]; ranges.len()];
        for e in &self.result.errors {
            if let Some(&j) = step_of.get(&e.feature) {
                errs[j].push(format!("{}: {}", e.feature, e.error.code));
            }
        }
        for j in 0..ranges.len() {
            if !ran[j] && ranges[j].0 == ranges[j].1 && j > 0 {
                snaps[j] = snaps[j - 1].clone();
            }
        }
        Ok((snaps, errs))
    }
}

/// The executor state to carry on from after a rewind: the last checkpoint's.
fn snap_of(_brep: &Brep, cps: &Rc<RefCell<Vec<(RegenResult, Option<BrepSnap>)>>>) -> Option<BrepSnap> {
    cps.borrow().last().and_then(|c| c.1.clone())
}
