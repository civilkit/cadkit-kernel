//! Debug trace: `msg.debug === true` adds `"trace":[...]` to the reply.
//!
//! Entry shape (oracle, debug regen of bracket.ckpart):
//! `{"at":<ms since trace origin>,"ms":<span duration>,"depth":<nesting>,
//!   "label":"...","level":"span"}` — entries in span START order.
//!
//! Clock protocol (oracle with `ck_now` returning 1,4,9,16,...): the request
//! start is sample 1, the trace origin is sample 2 (taken right after `debug`
//! is read), each span samples once at begin and once at end, and the reply
//! end is the last sample. When debug is off, spans never touch the clock.
//!
//! NOTE: spans opened deep inside geometry code (boolean, fillet, tessellate)
//! belong in ck-core's trace module once that exists; this collector covers
//! the server-level spans and is the sink the reply writer serialises.

use crate::clock;
use ck_json::{Json, Obj};

#[derive(Debug, Clone, PartialEq)]
pub struct Span {
    pub at: f64,
    pub ms: f64,
    pub depth: usize,
    pub label: String,
}

#[derive(Debug, Default)]
pub struct Trace {
    enabled: bool,
    origin: f64,
    depth: usize,
    spans: Vec<Span>,
    /// begin samples of the open spans (index into `spans`, start time)
    open: Vec<(usize, f64)>,
}

/// Handle for an open span; pass back to [`Trace::end`].
#[derive(Debug)]
#[must_use]
pub struct SpanId(Option<usize>);

impl Trace {
    /// Disabled trace (no clock calls, no output).
    pub fn off() -> Trace {
        Trace::default()
    }

    /// Enabled trace; samples the clock once for the origin.
    pub fn on() -> Trace {
        Trace { enabled: true, origin: clock::now(), ..Trace::default() }
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn begin(&mut self, label: impl Into<String>) -> SpanId {
        if !self.enabled {
            return SpanId(None);
        }
        let t = clock::now();
        let idx = self.spans.len();
        self.spans.push(Span { at: clock::elapsed_ms(self.origin, t), ms: 0.0, depth: self.depth, label: label.into() });
        self.open.push((idx, t));
        self.depth += 1;
        SpanId(Some(idx))
    }

    pub fn end(&mut self, id: SpanId) {
        let Some(idx) = id.0 else { return };
        let t = clock::now();
        if let Some(pos) = self.open.iter().rposition(|(i, _)| *i == idx) {
            let (_, start) = self.open.remove(pos);
            self.spans[idx].ms = clock::elapsed_ms(start, t);
            self.depth = self.depth.saturating_sub(1);
        }
    }

    /// Rename an open span (e.g. "F2 extrude" -> "F2 extrude → 8 faces").
    pub fn relabel(&mut self, id: &SpanId, label: impl Into<String>) {
        if let Some(idx) = id.0 {
            self.spans[idx].label = label.into();
        }
    }

    pub fn spans(&self) -> &[Span] {
        &self.spans
    }

    pub fn to_json(&self) -> Json {
        Json::Arr(
            self.spans
                .iter()
                .map(|s| {
                    Obj::new()
                        .put("at", s.at)
                        .put("ms", s.ms)
                        .put("depth", s.depth)
                        .put("label", s.label.as_str())
                        .put("level", "span")
                        .build()
                })
                .collect(),
        )
    }
}
