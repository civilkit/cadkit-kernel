//! Session state: regenerated documents keyed by `msg.tag`, plus the hooks
//! through which the geometry layers (ck-model & co.) plug into dispatch.
//!
//! Pinned by oracle probes:
//! - the tag is `msg.tag` when it is a string, otherwise `"main"`
//!   (`"tag":1` and a missing tag both hit the `"main"` state; `""` is its
//!   own, distinct tag);
//! - requests on a tag that was never regenerated fail with
//!   `noState: nothing regenerated yet` (the frontend keys its restore logic
//!   on the `noState` prefix — app.pretty.js `startsWith("noState")`);
//! - a failed regen (`invalid part: Format`) keeps the previous state.

use crate::{reply::Buffers, trace::Trace};
use ck_json::{Json, Obj};
use std::any::Any;

pub const NO_STATE: &str = "noState: nothing regenerated yet";
pub const INVALID_PART: &str = "invalid part: Format";
pub const DEFAULT_TAG: &str = "main";

/// One regenerated document.
pub struct State {
    /// Whether the regen produced a solid (`result.body != null`).
    /// Handlers with a pinned "no body" answer use it without any geometry.
    pub has_body: bool,
    /// Engine-owned payload (B-rep, meshes, ...), opaque to the wire layer.
    pub data: Option<Box<dyn Any>>,
}

impl State {
    pub fn empty() -> State {
        State { has_body: false, data: None }
    }
}

impl std::fmt::Debug for State {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("State").field("has_body", &self.has_body).field("data", &self.data.is_some()).finish()
    }
}

#[derive(Debug, Default)]
pub struct Session {
    states: Vec<(String, State)>,
}

impl Session {
    pub fn new() -> Session {
        Session::default()
    }
    pub fn get(&self, tag: &str) -> Option<&State> {
        self.states.iter().find(|(t, _)| t == tag).map(|(_, s)| s)
    }
    pub fn get_mut(&mut self, tag: &str) -> Option<&mut State> {
        self.states.iter_mut().find(|(t, _)| t == tag).map(|(_, s)| s)
    }
    pub fn put(&mut self, tag: &str, state: State) {
        match self.states.iter_mut().find(|(t, _)| t == tag) {
            Some(slot) => slot.1 = state,
            None => self.states.push((tag.to_string(), state)),
        }
    }
}

/// Engine hook for `regen` (and `matchEdges`, which regenerates its own
/// part): gets the validated part object and must return the regen result
/// JSON plus the state to store. Spans: the hook is called inside nothing —
/// it opens "regenerate N features" / "snapshot for the view (display
/// meshes, edges)" itself (see [`regen`]'s default path for the pinned order).
pub type RegenHook = fn(part: &Json, cx: &mut Cx) -> Result<(Json, State), String>;

/// Engine hook for every state-based request whose state HAS a body:
/// `(request type, cx)`. The handler has already opened the request's span
/// and validated the pinned input errors.
pub type QueryHook = fn(ty: &str, cx: &mut Cx) -> Result<Json, String>;

/// Engine hook for `matchEdges` on a non-empty part.
pub type MatchEdgesHook = fn(part: &Json, cx: &mut Cx) -> Result<Json, String>;

/// Engine hook for `sketchOffset` on a sketch with entities.
pub type SketchOffsetHook = fn(cx: &mut Cx, distance: f64) -> Result<Json, String>;

/// Engine hook for `importStep`: `(cx, name, STEP text)` -> reply result plus
/// the state to store (None = import failed, keep the previous state).
pub type ImportHook = fn(cx: &mut Cx, name: &str, data: &str) -> Result<(Json, Option<State>), String>;

#[derive(Default, Clone, Copy)]
pub struct Hooks {
    pub import_step: Option<ImportHook>,
    pub regen: Option<RegenHook>,
    pub query: Option<QueryHook>,
    pub match_edges: Option<MatchEdgesHook>,
    pub sketch_offset: Option<SketchOffsetHook>,
}

/// Per-request context handed to every handler.
pub struct Cx<'a> {
    pub msg: &'a Json,
    pub session: &'a mut Session,
    pub bufs: &'a mut Buffers,
    pub trace: &'a mut Trace,
    pub hooks: Hooks,
}

impl<'a> Cx<'a> {
    /// `msg.tag` if it is a string, else `"main"`.
    pub fn tag(&self) -> String {
        tag_of(self.msg).to_string()
    }

    /// The state for this request's tag, or the pinned `noState` error.
    pub fn state(&self) -> Result<&State, String> {
        self.session.get(tag_of(self.msg)).ok_or_else(|| NO_STATE.to_string())
    }

    /// Run `f` inside a trace span (span closes on every path, matching the
    /// oracle where e.g. `noState` errors still report a closed span).
    pub fn span<T>(&mut self, label: impl Into<String>, f: impl FnOnce(&mut Cx<'a>) -> T) -> T {
        let id = self.trace.begin(label);
        let r = f(self);
        self.trace.end(id);
        r
    }

    /// Delegate a body-bearing request to the engine, or a clear TODO.
    pub fn query(&mut self, ty: &str) -> Result<Json, String> {
        match self.hooks.query {
            Some(h) => h(ty, self),
            None => Err(todo(ty)),
        }
    }
}

/// The error a not-yet-rebuilt path returns (never produced by the oracle).
pub fn todo(what: &str) -> String {
    format!("TODO(rebuild): {what} needs the geometry engine (no hook installed)")
}

pub fn tag_of(msg: &Json) -> &str {
    msg.get("tag").and_then(Json::as_str).unwrap_or(DEFAULT_TAG)
}

/// Validate a `.ckpart` object the way the original's part loader rejects it
/// (all variants observed so far surface as `invalid part: Format`):
/// not an object, `format != "ck-part"`, or `features` not an array.
/// `version`, `name`, `params`, ... are not checked (version 0, 1.5, -1,
/// null and "1"-with-features-missing all probed).
pub fn check_part(part: Option<&Json>) -> Result<&Json, String> {
    let p = part.filter(|p| p.is_obj()).ok_or_else(|| INVALID_PART.to_string())?;
    if p.get_str("format") != Some("ck-part") || p.get_arr("features").is_none() {
        return Err(INVALID_PART.to_string());
    }
    Ok(p)
}

/// The regen result of a part that produced nothing (pinned: empty
/// `features`). Key order matches oracle-flange.json.
pub fn empty_regen_result() -> Json {
    let e = || Json::Arr(vec![]);
    Obj::new()
        .put("errors", e())
        .put("components", e())
        .put("planes", e())
        .put("markers", e())
        .put("sketches", e())
        .put("sketches3", e())
        .put("curves", e())
        .put("body", Json::Null)
        .put("surfaces", e())
        .build()
}

pub fn regen_label(n: usize) -> String {
    format!("regenerate {n} features")
}

pub const SNAPSHOT_LABEL: &str = "snapshot for the view (display meshes, edges)";

/// `{"type":"ping"}` -> `{"pong":true}`
pub fn ping(_cx: &mut Cx) -> Result<Json, String> {
    Ok(Obj::new().put("pong", true).build())
}

/// `{"type":"regen","tag":t,"part":{...},"opts":{"upTo":n}}`
pub fn regen(cx: &mut Cx) -> Result<Json, String> {
    let part = check_part(cx.msg.get("part"))?;
    let tag = cx.tag();
    if let Some(h) = cx.hooks.regen {
        let (result, state) = h(part, cx)?;
        cx.session.put(&tag, state);
        return Ok(result);
    }
    let n = part.get_arr("features").map_or(0, |f| f.len());
    if n > 0 {
        return Err(todo("regen of a part with features"));
    }
    // Pinned (debug, empty part): two depth-0 spans, in this order.
    cx.span(regen_label(0), |_| ());
    cx.span(SNAPSHOT_LABEL, |_| ());
    cx.session.put(&tag, State::empty());
    Ok(empty_regen_result())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_json::parse;

    #[test]
    fn tags() {
        assert_eq!(tag_of(&parse(r#"{"tag":"x"}"#).unwrap()), "x");
        assert_eq!(tag_of(&parse(r#"{"tag":1}"#).unwrap()), "main");
        assert_eq!(tag_of(&parse(r#"{}"#).unwrap()), "main");
        assert_eq!(tag_of(&parse(r#"{"tag":""}"#).unwrap()), "");
    }

    #[test]
    fn part_checks() {
        let c = |s: &str| check_part(Some(&parse(s).unwrap())).is_ok();
        assert!(c(r#"{"format":"ck-part","features":[]}"#));
        assert!(c(r#"{"format":"ck-part","version":1.5,"features":[]}"#));
        assert!(!c(r#"{"format":"ck-part","version":1}"#));
        assert!(!c(r#"{"format":"ck-part","features":{}}"#));
        assert!(!c(r#"{"format":"ck-part","features":null}"#));
        assert!(!c(r#"{"version":1,"features":[]}"#));
        assert!(!c(r#""x""#));
        assert!(check_part(None).is_err());
    }
}
