//! ck-server: the request envelope, dispatch table and reply framing.
//!
//! One call = one message: `handle(body) -> reply bytes`. Everything here is
//! pinned by probing the original kernel.wasm (tools/q.mjs, tools/diff.mjs).
//!
//! Envelope rules (oracle):
//! 1. `start = ck_now()` (always sampled first).
//! 2. Body must parse as JSON, else `bad request: the body must be JSON {id, msg}`
//!    with `"ms":0` (no second clock sample).
//! 3. `msg` (first key wins) must be an object — top-level `null`, `[]`, `1`,
//!    `{}`, `{"msg":[]}` all give `bad request: msg must be an object`, `"ms":0`.
//!    `id` and every other envelope key (`session`, `debug`, ...) are ignored.
//! 4. `msg.debug === true` (boolean only) enables the trace: origin sampled
//!    now; the reply gains `"buffers":{"count","bytes"}` (ok replies only)
//!    and `"trace":[...]` after `"ms"`.
//! 5. `msg.type` must be a non-empty string, else `message without a type`.
//! 6. Unknown types: `unknown request <type>` (type echoed verbatim).
//! 7. `ms = elapsed(start, ck_now())`; reply
//!    `{"ok":true,"result":R,"ms":M}` / `{"ok":false,"error":E,"ms":M}`.
//!
//! The request-type list is exhaustive for the oracle: every identifier in
//! kernel.strings and app.pretty.js (107k candidates, all substrings) was
//! sent as a type; exactly these 17 are not "unknown request".

pub mod clock;
pub mod comb;
pub mod curvature;
pub mod drawing;
pub mod geometry;
pub mod model;
pub mod query;
pub mod reply;
pub mod session;
pub mod trace;
pub mod view;

pub use reply::{BufType, Buffers};
pub use session::{Cx, Hooks, Session, State};
pub use trace::Trace;

use ck_json::{Json, Obj};

pub const BAD_BODY: &str = "bad request: the body must be JSON {id, msg}";
pub const BAD_MSG: &str = "bad request: msg must be an object";
pub const NO_TYPE: &str = "message without a type";
pub const UNKNOWN: &str = "unknown request ";

pub type Handler = fn(&mut Cx) -> Result<Json, String>;

/// Every request type the original kernel answers, and its handler.
pub const REQUEST_TYPES: &[(&str, Handler)] = &[
    ("ping", session::ping),
    ("regen", session::regen),
    ("matchEdges", model::match_edges),
    ("drag", model::drag),
    ("convert", model::convert),
    ("sketchOffset", model::sketch_offset),
    ("refCurve", model::ref_curve),
    ("measure", geometry::measure),
    ("massProps", geometry::mass_props),
    ("interference", geometry::interference),
    ("step", geometry::step),
    // Extension (not in the original kernel): independent solid validator.
    ("validate", geometry::validate),
    // Extension: shape healing (ck-heal) of the current body.
    ("heal", geometry::heal),
    ("importStep", geometry::import_step),
    // Extension: ACIS SAT import -> a body in the tag's state.
    ("importSat", geometry::import_sat),
    ("inspect", query::inspect),
    ("continuity", query::continuity),
    ("curvatureAt", curvature::curvature_at),
    ("curvatureMap", curvature::curvature_map),
    ("comb", comb::comb),
    ("drawing", drawing::drawing),
];

pub fn handler_for(ty: &str) -> Option<Handler> {
    REQUEST_TYPES.iter().find(|(t, _)| *t == ty).map(|(_, h)| *h)
}

/// The kernel: session state + engine hooks.
#[derive(Default)]
pub struct Server {
    pub session: Session,
    pub hooks: Hooks,
}

impl Server {
    pub fn new() -> Server {
        Server::default()
    }

    /// Run one raw request body; returns the framed reply bytes.
    pub fn handle(&mut self, body: &[u8]) -> Vec<u8> {
        let start = clock::now();
        let Ok(env) = ck_json::parse_bytes(body) else {
            return error_reply(BAD_BODY, 0.0, None);
        };
        let msg = match env.get("msg") {
            Some(m @ Json::Obj(_)) => m,
            _ => return error_reply(BAD_MSG, 0.0, None),
        };
        let mut trace = if msg.get_bool("debug") == Some(true) { Trace::on() } else { Trace::off() };
        let mut bufs = Buffers::new();
        let result = match msg.get_str("type").filter(|t| !t.is_empty()) {
            None => Err(NO_TYPE.to_string()),
            Some(ty) => match handler_for(ty) {
                None => Err(format!("{UNKNOWN}{ty}")),
                Some(h) => {
                    let mut cx =
                        Cx { msg, session: &mut self.session, bufs: &mut bufs, trace: &mut trace, hooks: self.hooks };
                    h(&mut cx)
                }
            },
        };
        let ms = clock::elapsed_ms(start, clock::now());
        let tr = trace.enabled().then_some(&trace);
        match result {
            Ok(r) => ok_reply(r, ms, &bufs, tr),
            Err(e) => error_reply(&e, ms, tr),
        }
    }
}

fn ok_reply(result: Json, ms: f64, bufs: &Buffers, trace: Option<&Trace>) -> Vec<u8> {
    let mut o = Obj::new().put("ok", true).put("result", result).put("ms", ms);
    if let Some(t) = trace {
        o = o
            .put("buffers", Obj::new().put("count", bufs.count()).put("bytes", bufs.bytes()))
            .put("trace", t.to_json());
    }
    reply::encode(&ck_json::to_string(&o.build()), bufs.data())
}

fn error_reply(e: &str, ms: f64, trace: Option<&Trace>) -> Vec<u8> {
    let mut o = Obj::new().put("ok", false).put("error", e).put("ms", ms);
    if let Some(t) = trace {
        o = o.put("trace", t.to_json());
    }
    reply::encode(&ck_json::to_string(&o.build()), &[])
}

thread_local! {
    static SERVER: std::cell::RefCell<Server> = std::cell::RefCell::new(Server::new());
}

/// Run one request against the process-global server (what ck-wasm calls).
pub fn handle(body: &[u8]) -> Vec<u8> {
    SERVER.with(|s| s.borrow_mut().handle(body))
}

/// Install engine hooks on the global server.
pub fn set_hooks(h: Hooks) {
    SERVER.with(|s| s.borrow_mut().hooks = h);
}

#[cfg(test)]
mod tests;
