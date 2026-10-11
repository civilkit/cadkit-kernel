//! Diagnostic regen trace (ck-core/src/trace.rs).
//!
//! When a request carries `"debug": true` the reply gets a top-level
//! `"trace": [...]` array (app.pretty.js `post()` adds `debug:!0`; the
//! kernel-log panel renders `trace[]` entries by `level`/`depth`/`at`/`ms`).
//! Without `debug` the key is absent and no clock reads happen.
//!
//! Entry shape, pinned by the oracle (q.mjs with a stepping clock):
//! `{"at":1.25,"ms":558.75,"depth":0,"label":"regenerate 11 features","level":"span"}`
//! `{"at":2.5,"ms":null,"depth":1,"label":"features 1–11 reused from the cache","level":"info"}`
//! - key order `at, ms, depth, label, level` (kernel.strings: the packed
//!   literal run `atmsdepthlabellevel`)
//! - levels `span` | `info` | `warn` (data section literals; the frontend
//!   counts `warn`s)
//! - `at` = clock at the event − trace start, `ms` = end − start for spans,
//!   `null` for info/warn; raw f64 differences, no rounding
//! - spans are recorded in pre-order (slot reserved when the span opens,
//!   `ms` filled on close); `depth` = number of open spans at that point;
//!   children of a span have depth+1
//! - clock reads (oracle with a clock stepping 1.25 per call): trace start
//!   = 1 read, span open = 1, span close = 1, info/warn = 1. The trace start
//!   read is separate from the request's own `ms` stopwatch (`ping` with
//!   debug reports ms 2.5 = 3 reads; without debug 1.25 = 2 reads).
//!
//! The clock is the host import `ck_now()` in the wasm build; ck-core has
//! no access to it, so it is injected with [`set_clock`] (default: a
//! constant 0.0, matching tools/q.mjs).
//!
//! The original keeps this in a global (kernel.strings: "cannot recursively
//! acquire mutex", "invalid Once state" nearby). Here the global is a
//! thread-local, plus a plain [`Trace`] value for explicit use.

use std::cell::{Cell, RefCell};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Level {
    Span,
    Info,
    Warn,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Span => "span",
            Level::Info => "info",
            Level::Warn => "warn",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    pub at: f64,
    /// Duration for spans (None while open / for info & warn → `null`).
    pub ms: Option<f64>,
    pub depth: u32,
    pub label: String,
    pub level: Level,
}

/// Default clock: constant 0 (as the probe harness supplies).
fn zero_clock() -> f64 {
    0.0
}

thread_local! {
    static CLOCK: Cell<fn() -> f64> = const { Cell::new(zero_clock as fn() -> f64) };
    static CURRENT: RefCell<Option<Trace>> = const { RefCell::new(None) };
}

/// Install the millisecond clock (`ck_now` in the wasm build).
pub fn set_clock(f: fn() -> f64) {
    CLOCK.with(|c| c.set(f));
}

/// Read the installed clock.
pub fn now() -> f64 {
    CLOCK.with(|c| (c.get())())
}

/// Simple stopwatch on the installed clock (request-level `"ms"`).
#[derive(Debug, Clone, Copy)]
pub struct Stopwatch {
    pub start: f64,
}

impl Stopwatch {
    pub fn start() -> Stopwatch {
        Stopwatch { start: now() }
    }
    pub fn ms(&self) -> f64 {
        now() - self.start
    }
}

/// A trace being recorded.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    pub t0: f64,
    pub events: Vec<Event>,
    depth: u32,
    /// stack of open span slots
    open: Vec<usize>,
}

impl Trace {
    /// Start a trace (one clock read).
    pub fn new() -> Trace {
        Trace { t0: now(), events: Vec::new(), depth: 0, open: Vec::new() }
    }

    /// Open a span; returns its slot. One clock read.
    pub fn begin(&mut self, label: impl Into<String>) -> usize {
        let at = now() - self.t0;
        let i = self.events.len();
        self.events.push(Event { at, ms: None, depth: self.depth, label: label.into(), level: Level::Span });
        self.depth += 1;
        self.open.push(i);
        i
    }

    /// Close span `slot` (and any spans opened inside it still open). One
    /// clock read.
    pub fn end(&mut self, slot: usize) {
        let t = now() - self.t0;
        while let Some(i) = self.open.pop() {
            let e = &mut self.events[i];
            e.ms = Some(t - e.at);
            self.depth = self.depth.saturating_sub(1);
            if i == slot {
                break;
            }
        }
    }

    /// Replace a span's label (labels like "F2 revolve → 6 faces" are only
    /// known after the work is done).
    pub fn set_label(&mut self, slot: usize, label: impl Into<String>) {
        if let Some(e) = self.events.get_mut(slot) {
            e.label = label.into();
        }
    }

    fn point(&mut self, level: Level, label: impl Into<String>) {
        let at = now() - self.t0;
        self.events.push(Event { at, ms: None, depth: self.depth, label: label.into(), level });
    }

    pub fn info(&mut self, label: impl Into<String>) {
        self.point(Level::Info, label)
    }

    pub fn warn(&mut self, label: impl Into<String>) {
        self.point(Level::Warn, label)
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }

    pub fn warn_count(&self) -> usize {
        self.events.iter().filter(|e| e.level == Level::Warn).count()
    }

    /// The `[...]` array value (without the `"trace":` key).
    pub fn to_json(&self) -> String {
        let mut s = String::from("[");
        for (k, e) in self.events.iter().enumerate() {
            if k > 0 {
                s.push(',');
            }
            s.push_str("{\"at\":");
            s.push_str(&fmt_num(e.at));
            s.push_str(",\"ms\":");
            match e.ms {
                Some(m) => s.push_str(&fmt_num(m)),
                None => s.push_str("null"),
            }
            s.push_str(",\"depth\":");
            s.push_str(&e.depth.to_string());
            s.push_str(",\"label\":");
            push_json_str(&mut s, &e.label);
            s.push_str(",\"level\":\"");
            s.push_str(e.level.as_str());
            s.push_str("\"}");
        }
        s.push(']');
        s
    }
}

impl Default for Trace {
    fn default() -> Self {
        Trace::new()
    }
}

/// f64 as the kernel prints it: Rust `Display` (shortest round-trip, no
/// exponent, integers without `.0`). GUESS: non-finite → `null`.
pub fn fmt_num(v: f64) -> String {
    if v.is_finite() {
        format!("{}", v)
    } else {
        "null".to_string()
    }
}

/// JSON string literal, kernel escaping (oracle: an echoed request type):
/// `\"` `\\` `\n` `\r` `\t`; other C0 controls as `\u00xx` (lowercase hex,
/// so `\b` → `\u0008`, `\f` → `\u000c`); `/`, DEL and non-ASCII raw.
pub fn push_json_str(s: &mut String, v: &str) {
    s.push('"');
    for c in v.chars() {
        match c {
            '"' => s.push_str("\\\""),
            '\\' => s.push_str("\\\\"),
            '\n' => s.push_str("\\n"),
            '\r' => s.push_str("\\r"),
            '\t' => s.push_str("\\t"),
            c if (c as u32) < 0x20 => s.push_str(&format!("\\u{:04x}", c as u32)),
            c => s.push(c),
        }
    }
    s.push('"');
}

// ---- thread-local global trace ---------------------------------------

/// Begin a request: when `debug`, start a global trace (one clock read);
/// otherwise make sure none is active (all calls below become no-ops with
/// no clock reads).
pub fn start(debug: bool) {
    CURRENT.with(|c| *c.borrow_mut() = if debug { Some(Trace::new()) } else { None });
}

/// End the request and take the recorded trace (None when not debugging).
pub fn take() -> Option<Trace> {
    CURRENT.with(|c| c.borrow_mut().take())
}

pub fn enabled() -> bool {
    CURRENT.with(|c| c.borrow().is_some())
}

fn with<R>(f: impl FnOnce(&mut Trace) -> R) -> Option<R> {
    CURRENT.with(|c| c.borrow_mut().as_mut().map(f))
}

/// RAII span on the global trace: opened by [`span`], closed on drop.
#[must_use = "a span closes when dropped"]
pub struct Span {
    slot: Option<usize>,
}

impl Span {
    /// Update the label (no-op when tracing is off).
    pub fn label(&self, label: impl Into<String>) {
        if let Some(i) = self.slot {
            let l = label.into();
            with(|t| t.set_label(i, l));
        }
    }

    /// Close now (same as dropping).
    pub fn end(mut self) {
        self.close();
    }

    fn close(&mut self) {
        if let Some(i) = self.slot.take() {
            with(|t| t.end(i));
        }
    }
}

impl Drop for Span {
    fn drop(&mut self) {
        self.close();
    }
}

/// Open a span on the global trace.
pub fn span(label: impl Into<String>) -> Span {
    let l = label.into();
    Span { slot: with(|t| t.begin(l)) }
}

/// Lazily-labelled span: the closure only runs when tracing is on.
pub fn span_with(f: impl FnOnce() -> String) -> Span {
    if enabled() {
        span(f())
    } else {
        Span { slot: None }
    }
}

pub fn info(label: impl Into<String>) {
    let l = label.into();
    with(|t| t.info(l));
}

pub fn warn(label: impl Into<String>) {
    let l = label.into();
    with(|t| t.warn(l));
}

#[cfg(test)]
mod tests {
    use super::*;

    thread_local! { static T: Cell<f64> = const { Cell::new(0.0) }; }
    /// Same stepping clock as the oracle probe (+1.25 per read).
    fn step() -> f64 {
        T.with(|t| {
            t.set(t.get() + 1.25);
            t.get()
        })
    }

    #[test]
    fn reproduces_oracle_cached_regen_trace() {
        // Oracle (bracket.ckpart regen twice, 2nd with debug, clock +1.25/read):
        // {"ms":10,"trace":[...]} — the trace part is reproduced byte-exact.
        set_clock(step);
        T.with(|t| t.set(0.0));
        let req = Stopwatch::start();
        start(true);
        {
            let _s = span("regenerate 11 features");
            info("features 1\u{2013}11 reused from the cache");
        }
        {
            let _s = span("snapshot for the view (display meshes, edges)");
            info("display mesh of 34 faces reused");
        }
        let t = take().unwrap();
        assert_eq!(
            t.to_json(),
            "[{\"at\":1.25,\"ms\":2.5,\"depth\":0,\"label\":\"regenerate 11 features\",\"level\":\"span\"},\
{\"at\":2.5,\"ms\":null,\"depth\":1,\"label\":\"features 1\u{2013}11 reused from the cache\",\"level\":\"info\"},\
{\"at\":5,\"ms\":2.5,\"depth\":0,\"label\":\"snapshot for the view (display meshes, edges)\",\"level\":\"span\"},\
{\"at\":6.25,\"ms\":null,\"depth\":1,\"label\":\"display mesh of 34 faces reused\",\"level\":\"info\"}]"
        );
        // the request stopwatch is independent of the trace's start read
        assert_eq!(req.start, 1.25);
        set_clock(zero_clock);
    }

    #[test]
    fn nesting_relabel_and_disabled() {
        set_clock(step);
        T.with(|t| t.set(0.0));
        start(true);
        {
            let outer = span("F4 extrude");
            {
                let _b = span("boolean");
                let _l = span("locate");
                warn("careful");
            }
            outer.label("F4 extrude \u{2192} 8 faces");
        }
        let t = take().unwrap();
        let d: Vec<u32> = t.events.iter().map(|e| e.depth).collect();
        assert_eq!(d, vec![0, 1, 2, 3]);
        assert_eq!(t.events[0].label, "F4 extrude \u{2192} 8 faces");
        assert_eq!(t.warn_count(), 1);
        assert!(t.events.iter().filter(|e| e.level == Level::Span).all(|e| e.ms.is_some()));
        // children close before parents: parent ms covers child
        assert!(t.events[0].ms.unwrap() > t.events[1].ms.unwrap());

        // disabled: no events and NO clock reads
        start(false);
        let before = T.with(|t| t.get());
        {
            let s = span("x");
            s.label("y");
            info("z");
            let _w = span_with(|| panic!("label closure must not run"));
        }
        assert!(take().is_none());
        assert_eq!(T.with(|t| t.get()), before);
        set_clock(zero_clock);
    }

    #[test]
    fn escaping_and_numbers() {
        let mut s = String::new();
        push_json_str(&mut s, "a\"b\\\n\t\u{1}\u{7f}\u{e9}/c\r\u{8}\u{c}\u{1b}");
        // oracle: "unknown request a\"b\\\n\t\u0001é/c" and "\r\u0008\u000c\u001f\u001b"
        assert_eq!(s, "\"a\\\"b\\\\\\n\\t\\u0001\u{7f}\u{e9}/c\\r\\u0008\\u000c\\u001b\"");
        assert_eq!(fmt_num(10.0), "10");
        assert_eq!(fmt_num(558.75), "558.75");
        assert_eq!(fmt_num(1.3187749835348084e-15), "0.0000000000000013187749835348084");
        assert_eq!(fmt_num(f64::NAN), "null");
    }
}
