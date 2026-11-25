// SPDX-License-Identifier: MIT
//! Which states of an `.smbh` blob's ASM history hold the same bodies,
//! told without converting every state's bodies whole (mitcad#675).
//!
//! The import keeps one body per distinct body of the history: a state
//! whose bodies are all the same as the state's after it is no state of
//! its own. It converted every state's bodies to compare them, most of the
//! time of reading a design with a pattern of hundreds of copies (one or
//! two operations each), whose states the replay never asks for. The same
//! answers come from the history itself:
//!
//! - **An operation that changed nothing the conversion reads.** Rolling
//!   one more state back changes only the entities of that state's
//!   bulletins. The conversion of a body reads an entity's record past its
//!   header (its attribute, history number and one more field; a transform
//!   past its attribute and history number) and its type, follows only the
//!   pointers there, and never an attribute's. So a bulletin whose two
//!   records agree in all of that, or whose entity no record but an
//!   attribute points at past the attribute field, changes no body: the
//!   copies' operations of such a pattern change only the attributes of
//!   some edges (and the edges' pointers to them).
//! - **A body none of whose entities the operation changed.** The entities
//!   a body's conversion reads are among those its pointers reach (past
//!   the attribute field) at the newer state; when none of the changed
//!   entities is among them, the two states' conversions read the same and
//!   give the same body.
//! - **Else the two conversions side by side**: a body that changed mostly
//!   differs in its first faces (the faces an operation made lead their
//!   shell's list in the files seen), so converting the first faces and
//!   edges of both tells most apart (a face may have thousands of edges:
//!   a floor round hundreds of ribs); eight times as many are converted
//!   each round until they differ or both bodies are whole, which then
//!   are compared as the converted bodies always were.
//!
//! Each answer is the one converting both states whole and comparing the
//! bodies gives.

use std::collections::{HashMap, HashSet};

use crate::asm::AsmFile;
use crate::asm::Token;
use crate::asm::history::History;
use crate::brep::Body;
use crate::convert::{BodyPrefix, ConvertedBody, Limit, Options, Stop, convert_body_prefix};
use crate::ids::{IdSet, View};

/// The faces and edges converted first when two bodies are compared side
/// by side; then eight times as many each round.
const FIRST: Limit = Limit {
    faces: 16,
    edges: 64,
};

/// Where the fields the conversion reads start in a record of this type
/// ([`crate::convert`]: past the entity header, a transform's past its
/// attribute and history number).
fn read_from(type_name: &str) -> usize {
    if type_name == "transform" { 2 } else { 3 }
}

/// The entities some record that is not an attribute points at past its
/// attribute field, in any state of the file (live records and the
/// history's copies alike): the only ones a conversion may read besides the
/// bodies it starts from. A conversion starts at a body, and reads only
/// what a record it read points at past that field, so it never comes to
/// one outside this set.
pub fn referenced(file: &AsmFile) -> IdSet {
    let mut out = IdSet::default();
    // (The history section's own records, its header and delta states,
    // take no pointer numbers: no pointer leads to them.)
    let section = file.history_section.clone().unwrap_or(0..0);
    for (i, record) in file.records.iter().enumerate() {
        if record.base_type() == "attrib" || section.contains(&i) {
            continue;
        }
        for token in file.fields(i).iter().skip(1) {
            if let Token::Ptr(p) = token
                && let Some(e) = file.record_of(*p)
            {
                out.insert(e);
            }
        }
    }
    out
}

/// Whether a conversion reads the same in two records: the same type and
/// the same fields from where it reads ([`read_from`]).
fn read_alike(file: &AsmFile, a: usize, b: usize) -> bool {
    if a == b {
        return true;
    }
    let (ra, rb) = (&file.records[a], &file.records[b]);
    if ra.type_name != rb.type_name {
        return false;
    }
    let from = read_from(&ra.type_name);
    file.fields(a).get(from..) == file.fields(b).get(from..)
}

/// The record holding an entity's data in a view (`None`: it does not
/// exist there).
fn data(view: &View, entity: usize) -> Option<usize> {
    match view.get(&entity) {
        None => Some(entity),
        Some(at) => *at,
    }
}

/// Whether the pointers of `start` reach any of `targets` in `view`
/// (following every pointer of a record past its attribute field, and
/// the entities that do not exist there not further).
fn reaches(file: &AsmFile, view: &View, start: usize, targets: &IdSet) -> bool {
    let mut seen = IdSet::default();
    let mut stack = vec![start];
    let mut pointers = Vec::new();
    while let Some(e) = stack.pop() {
        if targets.contains(&e) {
            return true;
        }
        if !seen.insert(e) {
            continue;
        }
        let Some(record) = data(view, e) else {
            continue;
        };
        // (In the order of the fields, as the conversion follows them: an
        // operation's new faces lead their shell's list.)
        pointers.clear();
        for token in file.fields(record).iter().skip(1) {
            if let Token::Ptr(p) = token
                && let Some(next) = file.record_of(*p)
                && !seen.contains(&next)
            {
                pointers.push(next);
            }
        }
        stack.extend(pointers.iter().rev());
    }
    false
}

/// A body at a state: its view (and an overlay looked up first) and its
/// `body` record.
#[derive(Clone, Copy)]
pub struct BodyAt<'v> {
    pub view: &'v View,
    pub overlay: Option<&'v View>,
    pub record: usize,
}

/// Whether two bodies convert to the same neutral body (`Body`), as
/// converting both whole and comparing them would tell: their first faces
/// and edges first, then more ([`FIRST`]).
pub fn same_body(file: &AsmFile, options: &Options, a: BodyAt, b: BodyAt) -> bool {
    let mut limit = FIRST;
    loop {
        let convert = |at: BodyAt| {
            convert_body_prefix(file, at.record, options, Some(at.view), at.overlay, limit)
        };
        if let Some(same) = compare(&convert(a), &convert(b)) {
            return same;
        }
        limit = Limit {
            faces: limit.faces.saturating_mul(8),
            edges: limit.edges.saturating_mul(8),
        };
    }
}

/// Whether two conversions give the same body; `None` while that is not
/// known yet (both stopped, alike so far).
fn compare(a: &BodyPrefix, b: &BodyPrefix) -> Option<bool> {
    let (x, y) = match (a, b) {
        (BodyPrefix::Whole(a), BodyPrefix::Whole(b)) => return Some(a.body == b.body),
        (BodyPrefix::Whole(a), BodyPrefix::Part(b, _)) => (&a.body, b),
        (BodyPrefix::Part(a, _), BodyPrefix::Whole(b)) => (a, &b.body),
        (BodyPrefix::Part(a, _), BodyPrefix::Part(b, _)) => (a, b),
    };
    if x.transform != y.transform || !lists_alike(x, y) {
        return Some(false);
    }
    // A stopped conversion has more faces (or edges) than it converted; a
    // whole body with no more than those differs from it.
    let fewer = match (a, b) {
        (BodyPrefix::Whole(w), BodyPrefix::Part(p, stop))
        | (BodyPrefix::Part(p, stop), BodyPrefix::Whole(w)) => match stop {
            Stop::Face => w.body.faces.len() <= p.faces.len(),
            Stop::Edge => w.body.edges.len() <= p.edges.len(),
        },
        _ => false,
    };
    fewer.then_some(false)
}

/// Whether the lists two conversions made agree as far as both go (each is
/// the start of its whole body's list).
fn lists_alike(a: &Body, b: &Body) -> bool {
    fn alike<T: PartialEq>(a: &[T], b: &[T]) -> bool {
        let n = a.len().min(b.len());
        a[..n] == b[..n]
    }
    alike(&a.faces, &b.faces)
        && alike(&a.vertices, &b.vertices)
        && alike(&a.edges, &b.edges)
        && alike(&a.curves, &b.curves)
        && alike(&a.surfaces, &b.surfaces)
}

/// The states of a history blob from the current one back, one at a time
/// ([`StateWalk::advance`]): which bodies each has and which of them are
/// the same as in the state after it.
pub struct StateWalk<'f> {
    file: &'f AsmFile,
    history: &'f History,
    options: Options,
    /// The top-level body records.
    records: Vec<usize>,
    /// [`referenced`].
    referenced: IdSet,
    /// The view of the current state ([`History::view`] of `step`).
    view: View,
    step: usize,
}

impl<'f> StateWalk<'f> {
    /// At the current state (no operation undone).
    pub fn new(
        file: &'f AsmFile,
        history: &'f History,
        records: &[usize],
        options: &Options,
    ) -> Self {
        StateWalk {
            file,
            history,
            options: *options,
            records: records.to_vec(),
            referenced: referenced(file),
            view: View::default(),
            step: 0,
        }
    }

    /// The operations undone so far.
    pub fn step(&self) -> usize {
        self.step
    }

    /// The view of the state ([`History::view`] of [`StateWalk::step`]).
    pub fn view(&self) -> &View {
        &self.view
    }

    /// The body records that exist at the state, in the order given.
    pub fn present(&self) -> Vec<usize> {
        present(&self.records, &self.view, None)
    }

    /// Undoes one more operation; per body record that exists both before
    /// and after, in the order given, whether its body is the same. None
    /// at the oldest state (nothing left to undo).
    pub fn advance(&mut self) -> Option<Vec<(usize, bool)>> {
        let state = self.history.states.get(self.step)?;
        // The records of the entities the operation changed, before it.
        let mut overlay = View::default();
        for &(before, after) in &state.bulletins {
            if let Some(entity) = after {
                overlay.insert(entity, before);
            }
        }
        let tops: HashSet<usize> = self.records.iter().copied().collect();
        let changed: IdSet = overlay
            .iter()
            .filter(|&(&entity, &before)| {
                let alike = match (data(&self.view, entity), before) {
                    (Some(a), Some(b)) => read_alike(self.file, a, b),
                    (None, None) => true,
                    _ => false,
                };
                !alike && (self.referenced.contains(&entity) || tops.contains(&entity))
            })
            .map(|(&entity, _)| entity)
            .collect();
        let newer = present(&self.records, &self.view, None);
        let older = present(&self.records, &self.view, Some(&overlay));
        let mut out = Vec::new();
        for &record in &older {
            if !newer.contains(&record) {
                continue;
            }
            let same = changed.is_empty()
                || !reaches(self.file, &self.view, record, &changed)
                || same_body(
                    self.file,
                    &self.options,
                    BodyAt {
                        view: &self.view,
                        overlay: None,
                        record,
                    },
                    BodyAt {
                        view: &self.view,
                        overlay: Some(&overlay),
                        record,
                    },
                );
            out.push((record, same));
        }
        for (entity, before) in overlay {
            self.view.insert(entity, before);
        }
        self.step += 1;
        Some(out)
    }
}

/// A distinct body of a history ([`body_sets`]): the operations undone to
/// its state, its `body` record, and its conversion where it has one.
#[derive(Debug)]
pub struct DistinctBody {
    pub step: usize,
    pub record: usize,
    pub body: Option<ConvertedBody>,
}

/// The distinct body sets of a history blob ([`body_sets`]).
#[derive(Debug, Default)]
pub struct BodySets {
    /// The distinct bodies.
    pub bodies: Vec<DistinctBody>,
    /// The distinct body sets (indices into `bodies`), newest first, each
    /// with the number of the operation that made it (`i64::MIN` before
    /// the first operation; of the oldest when several made it).
    pub newest_first: Vec<(i64, Vec<usize>)>,
    /// Every operation by its number, with the index into `newest_first`
    /// of the set after it.
    pub ops: Vec<(i64, usize)>,
    /// The operations that changed the bodies.
    pub changed: Vec<i64>,
    /// The set of the state `first` given to [`body_sets`].
    pub first: Option<Vec<usize>>,
}

/// The body sets of a history blob from its current state back, one per
/// state but those that hold the same bodies as the state after (the older
/// state stands for both): as converting each state's bodies whole and
/// comparing them with the state after's tells, told mostly without
/// converting them ([`StateWalk`]). `first`: the bodies of one state,
/// converted already (by the operations undone), which its distinct bodies
/// keep; no other body is converted whole.
pub fn body_sets(
    file: &AsmFile,
    history: &History,
    records: &[usize],
    options: &Options,
    first: Option<(usize, Vec<ConvertedBody>)>,
) -> BodySets {
    let mut sets = BodySets::default();
    let first_step = first.as_ref().map(|(k, _)| *k);
    let mut first = first;
    let mut walk = StateWalk::new(file, history, records, options);
    // Per body record of the state, the newest state from which on its body
    // has been the same in every state.
    let mut same_since: HashMap<usize, usize> = HashMap::new();
    for k in 0..=history.states.len() {
        // (In the state after.)
        let since_after = std::mem::take(&mut same_since);
        let same: HashMap<usize, bool> = match k {
            0 => HashMap::new(),
            _ => walk.advance().unwrap_or_default().into_iter().collect(),
        };
        let present = walk.present();
        same_since = present
            .iter()
            .map(|&r| match since_after.get(&r) {
                Some(&since) if same.get(&r) == Some(&true) => (r, since),
                _ => (r, k),
            })
            .collect();
        let mut converted: Vec<Option<ConvertedBody>> = match first.take() {
            Some((f, bodies)) if f == k => bodies.into_iter().map(Some).collect(),
            other => {
                first = other;
                Vec::new()
            }
        };
        // Whether a distinct body is this state's body of its record: the
        // same in every state between; not when it is the state after's,
        // which this state changed; else as the two conversions compare.
        let same_as = |d: &DistinctBody, record: usize| {
            if same_since.get(&record).is_some_and(|&s| s <= d.step) {
                true
            } else if since_after.get(&record).is_some_and(|&s| s <= d.step) {
                false
            } else {
                same_at(file, history, options, (d.step, d.record), (k, record))
            }
        };
        let bodies = &mut sets.bodies;
        let kept = bodies.len();
        let after: Vec<usize> = sets
            .newest_first
            .last()
            .map_or(Vec::new(), |(_, ids)| ids.clone());
        let mut set: Vec<usize> = Vec::new();
        for (i, &record) in present.iter().enumerate() {
            let same = after
                .iter()
                .copied()
                .find(|&d| bodies[d].record == record && same_as(&bodies[d], record));
            set.push(same.unwrap_or_else(|| {
                bodies.push(DistinctBody {
                    step: k,
                    record,
                    body: converted.get_mut(i).and_then(Option::take),
                });
                bodies.len() - 1
            }));
        }
        let time = history.states.get(k).map_or(i64::MIN, |s| s.id);
        // (Two distinct bodies of one record differ: the newer would have
        // been taken.)
        let same = sets.newest_first.last().is_some_and(|(_, last)| {
            last.len() == set.len()
                && last.iter().zip(&set).all(|(&a, &b)| {
                    let (x, y) = (&bodies[a], &bodies[b]);
                    a == b
                        || (x.record != y.record
                            && same_at(
                                file,
                                history,
                                options,
                                (x.step, x.record),
                                (y.step, y.record),
                            ))
                })
        });
        if same {
            // The older state made it: keep the older time.
            sets.newest_first.last_mut().expect("checked").0 = time;
            bodies.truncate(kept);
        } else {
            sets.newest_first.push((time, set));
            // The operation undone last changed the bodies.
            if let Some(s) = k.checked_sub(1).and_then(|j| history.states.get(j)) {
                sets.changed.push(s.id);
            }
        }
        if let Some(s) = history.states.get(k) {
            sets.ops.push((s.id, sets.newest_first.len() - 1));
        }
        if first_step == Some(k) {
            sets.first = sets.newest_first.last().map(|(_, ids)| ids.clone());
        }
    }
    sets
}

/// Whether two bodies of a history, each given by the operations undone
/// and its `body` record, convert to the same body ([`same_body`]).
fn same_at(
    file: &AsmFile,
    history: &History,
    options: &Options,
    a: (usize, usize),
    b: (usize, usize),
) -> bool {
    let (view_a, view_b) = (history.view(a.0), history.view(b.0));
    let at = |view, record| BodyAt {
        view,
        overlay: None,
        record,
    };
    same_body(file, options, at(&view_a, a.1), at(&view_b, b.1))
}

/// The body records that exist in a view (with an overlay looked up first).
fn present(records: &[usize], view: &View, overlay: Option<&View>) -> Vec<usize> {
    records
        .iter()
        .copied()
        .filter(|r| {
            let at = overlay.and_then(|o| o.get(r)).or_else(|| view.get(r));
            at != Some(&None)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert::{body_records, convert_body_at};
    use crate::testdata;

    fn parse(blob: &[u8]) -> (AsmFile, History) {
        let file = AsmFile::parse(blob).unwrap();
        let history = History::parse(&file).unwrap().unwrap();
        (file, history)
    }

    fn tops(file: &AsmFile) -> Vec<usize> {
        body_records(file)
            .into_iter()
            .filter(|(_, top)| *top)
            .map(|(r, _)| r)
            .collect()
    }

    /// The walk's answers are those of converting every state whole.
    fn walk_matches_whole_conversions(blob: &[u8]) -> Vec<Vec<(usize, bool)>> {
        let (file, history) = parse(blob);
        let records = tops(&file);
        let options = Options::default();
        let mut walk = StateWalk::new(&file, &history, &records, &options);
        let mut answers = Vec::new();
        for k in 0..history.states.len() {
            assert_eq!(walk.step(), k);
            assert_eq!(walk.view(), &history.view(k));
            let step = walk.advance().expect("a state to undo");
            let (newer, older) = (history.view(k), history.view(k + 1));
            for &(record, same) in &step {
                let a = convert_body_at(&file, record, &options, Some(&newer));
                let b = convert_body_at(&file, record, &options, Some(&older));
                assert_eq!(same, a.body == b.body, "state {k}, body {record}");
            }
            answers.push(step);
        }
        assert!(walk.advance().is_none());
        answers
    }

    #[test]
    fn tells_the_states_that_changed_a_body_without_converting_them_whole() {
        // State 3 moved a corner, state 2 changed nothing, state 1 made the
        // body (it does not exist before it).
        let answers = walk_matches_whole_conversions(&testdata::cube_with_history_blob());
        assert_eq!(answers.len(), 3);
        assert_eq!(answers[0].len(), 1);
        assert!(!answers[0][0].1, "the moved corner");
        assert!(answers[1][0].1, "nothing changed");
        assert!(answers[2].is_empty(), "no body before the first state");
    }

    #[test]
    fn an_operation_that_changed_only_attributes_changed_no_body() {
        let blob = testdata::named_cube_with_attribute_history_blob();
        let (file, history) = parse(&blob);
        assert_eq!(history.states.len(), 3);
        assert_eq!(tops(&file), vec![1]);
        // What the newest operation changed, as the conversion sees it:
        // nothing (the changed attribute and the new one only attributes
        // point at; the point's record differs in its attribute pointer).
        let referenced = referenced(&file);
        assert!(!referenced.contains(&86) && !referenced.contains(&93));
        assert!(referenced.contains(&60));
        assert!(read_alike(&file, 60, 99));
        assert!(!read_alike(&file, 86, 98));
        let answers = walk_matches_whole_conversions(&blob);
        assert_eq!(answers[0], vec![(1, true)], "only attributes changed");
        assert_eq!(answers[1], vec![(1, false)], "the moved corner");
        assert!(answers[2].is_empty());
    }

    /// The body sets as converting every state's bodies whole and comparing
    /// them gives them (the import's way before mitcad#675): each set's
    /// bodies (record and neutral body), its operation, the operations and
    /// the changing ones.
    #[allow(clippy::type_complexity)]
    fn body_sets_whole(
        file: &AsmFile,
        history: &History,
        records: &[usize],
        options: &Options,
    ) -> (
        Vec<(i64, Vec<(usize, Body)>)>,
        Vec<(i64, usize)>,
        Vec<i64>,
        usize,
    ) {
        let mut distinct: Vec<ConvertedBody> = Vec::new();
        let mut newest_first: Vec<(i64, Vec<usize>)> = Vec::new();
        let (mut ops, mut changed) = (Vec::new(), Vec::new());
        for k in 0..=history.states.len() {
            let view = history.view(k);
            let converted: Vec<ConvertedBody> = records
                .iter()
                .filter(|r| view.get(r) != Some(&None))
                .map(|&r| convert_body_at(file, r, options, Some(&view)))
                .collect();
            let kept = distinct.len();
            let after: Vec<usize> = newest_first
                .last()
                .map_or(Vec::new(), |(_, ids)| ids.clone());
            let mut bodies = Vec::new();
            for body in converted {
                let same = after
                    .iter()
                    .copied()
                    .find(|&d| distinct[d].record == body.record && distinct[d].body == body.body);
                bodies.push(same.unwrap_or_else(|| {
                    distinct.push(body);
                    distinct.len() - 1
                }));
            }
            let time = history.states.get(k).map_or(i64::MIN, |s| s.id);
            let same = newest_first.last().is_some_and(|(_, last)| {
                last.len() == bodies.len()
                    && last
                        .iter()
                        .zip(&bodies)
                        .all(|(&a, &b)| a == b || distinct[a].body == distinct[b].body)
            });
            if same {
                newest_first.last_mut().unwrap().0 = time;
                distinct.truncate(kept);
            } else {
                newest_first.push((time, bodies));
                if let Some(s) = k.checked_sub(1).and_then(|j| history.states.get(j)) {
                    changed.push(s.id);
                }
            }
            if let Some(s) = history.states.get(k) {
                ops.push((s.id, newest_first.len() - 1));
            }
        }
        let sets = newest_first
            .into_iter()
            .map(|(time, ids)| {
                let bodies = ids
                    .into_iter()
                    .map(|d| (distinct[d].record, distinct[d].body.clone()))
                    .collect();
                (time, bodies)
            })
            .collect();
        (sets, ops, changed, distinct.len())
    }

    /// [`body_sets`] gives the sets [`body_sets_whole`] gives, converting
    /// no body but those of the state given.
    fn body_sets_match_whole_conversions(blob: &[u8]) -> BodySets {
        let (file, history) = parse(blob);
        let records = tops(&file);
        let options = Options::default();
        // The oldest state with bodies, converted already.
        let oldest = (0..=history.states.len())
            .rev()
            .find(|&k| !present(&records, &history.view(k), None).is_empty());
        let first = oldest.map(|k| {
            let view = history.view(k);
            let bodies = present(&records, &view, None)
                .into_iter()
                .map(|r| convert_body_at(&file, r, &options, Some(&view)))
                .collect();
            (k, bodies)
        });
        let sets = body_sets(&file, &history, &records, &options, first);
        let (whole, ops, changed, count) = body_sets_whole(&file, &history, &records, &options);
        assert_eq!(sets.ops, ops);
        assert_eq!(sets.changed, changed);
        assert_eq!(sets.bodies.len(), count);
        assert_eq!(sets.newest_first.len(), whole.len());
        for ((time, ids), (whole_time, bodies)) in sets.newest_first.iter().zip(&whole) {
            assert_eq!(time, whole_time);
            assert_eq!(ids.len(), bodies.len());
            for (&d, (record, body)) in ids.iter().zip(bodies) {
                let distinct = &sets.bodies[d];
                assert_eq!(distinct.record, *record);
                let view = history.view(distinct.step);
                let converted = convert_body_at(&file, distinct.record, &options, Some(&view));
                assert_eq!(&converted.body, body);
                // Only the given state's bodies come converted, as given.
                match &distinct.body {
                    Some(given) => {
                        assert_eq!(Some(distinct.step), oldest);
                        assert_eq!(&given.body, body);
                    }
                    None => assert_ne!(Some(distinct.step), oldest),
                }
            }
        }
        // The given state's set: the one each state's operation names, the
        // oldest state's the last.
        let set_of = |k: usize| match sets.ops.get(k) {
            Some(&(_, set)) => set,
            None => sets.newest_first.len() - 1,
        };
        assert_eq!(
            sets.first,
            oldest.map(|k| sets.newest_first[set_of(k)].1.clone())
        );
        sets
    }

    #[test]
    fn body_sets_are_those_of_whole_conversions() {
        let sets = body_sets_match_whole_conversions(&testdata::cube_with_history_blob());
        // No body before the first state, the cube, the moved corner (the
        // operation that changed nothing shares its set).
        assert_eq!(sets.newest_first.len(), 3);
        let sets =
            body_sets_match_whole_conversions(&testdata::named_cube_with_attribute_history_blob());
        assert_eq!(
            sets.newest_first.len(),
            3,
            "the attributes' operation shares a set"
        );
        assert_eq!(sets.changed, vec![2, 1]);
        body_sets_match_whole_conversions(&testdata::bevelled_cube_history_blob(0.2));
    }

    #[test]
    fn bodies_compared_face_by_face_agree_with_whole_conversions() {
        let (file, history) = parse(&testdata::cube_with_history_blob());
        let record = tops(&file)[0];
        let options = Options::default();
        let views: Vec<View> = (0..3).map(|k| history.view(k)).collect();
        for a in 0..3 {
            for b in 0..3 {
                let at = |k: usize| BodyAt {
                    view: &views[k],
                    overlay: None,
                    record,
                };
                let whole = |k: usize| convert_body_at(&file, record, &options, Some(&views[k]));
                assert_eq!(
                    same_body(&file, &options, at(a), at(b)),
                    whole(a).body == whole(b).body,
                    "states {a} and {b}"
                );
            }
        }
    }

    #[test]
    fn a_conversion_stopped_after_some_faces_is_the_start_of_the_whole_body() {
        let file = AsmFile::parse(&testdata::cube_blob()).unwrap();
        let record = tops(&file)[0];
        let options = Options::default();
        let whole = convert_body_at(&file, record, &options, None);
        assert_eq!(whole.body.faces.len(), 6);
        let faces = |faces| Limit {
            faces,
            edges: usize::MAX,
        };
        for n in 0..6 {
            match convert_body_prefix(&file, record, &options, None, None, faces(n)) {
                BodyPrefix::Part(part, stop) => {
                    assert_eq!(stop, Stop::Face);
                    assert_eq!(part.faces.len(), n);
                    assert!(lists_alike(&part, &whole.body));
                    assert!(part.vertices.len() <= whole.body.vertices.len());
                    // A whole body with no more faces differs from it.
                    let stopped = BodyPrefix::Part(part, stop);
                    let fewer = convert_body_prefix(&file, record, &options, None, None, faces(6));
                    assert_eq!(compare(&stopped, &fewer), None);
                }
                BodyPrefix::Whole(_) => panic!("{n} faces: not stopped"),
            }
        }
        match convert_body_prefix(&file, record, &options, None, None, faces(6)) {
            BodyPrefix::Whole(body) => assert_eq!(body.body, whole.body),
            BodyPrefix::Part(..) => panic!("stopped with no face left"),
        }
        // Stopped before an edge: as far as the edges go.
        let edges = Limit {
            faces: usize::MAX,
            edges: 5,
        };
        match convert_body_prefix(&file, record, &options, None, None, edges) {
            BodyPrefix::Part(part, stop) => {
                assert_eq!(stop, Stop::Edge);
                assert_eq!(part.edges.len(), 5);
                assert!(lists_alike(&part, &whole.body));
            }
            BodyPrefix::Whole(_) => panic!("not stopped at the fifth edge"),
        }
    }

    #[test]
    fn records_only_attributes_point_at_are_not_read() {
        // Every entity the body reaches is pointed at by another one past
        // its attribute field.
        let file = AsmFile::parse(&testdata::cube_blob()).unwrap();
        let referenced = referenced(&file);
        let record = tops(&file)[0];
        let mut reached = IdSet::default();
        for e in 0..file.records.len() {
            let mut target = IdSet::default();
            target.insert(e);
            if e != record && reaches(&file, &View::default(), record, &target) {
                reached.insert(e);
            }
        }
        assert!(!reached.is_empty());
        assert!(reached.iter().all(|e| referenced.contains(e)));
    }
}
