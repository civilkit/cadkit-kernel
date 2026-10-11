//! Regen program execution: feature ordering, upTo support, rollback,
//! caching ("reused from the cache"), error collection.
//!
//! Input: a `crate::Part` (parsed .ckpart) + options (`{upTo}` for partial
//! regen, `tag` for cache identity: "main" | "asm" | "bridge" | "plain" | "base" | "sketch").
//! Output: `RegenResult` (errors + components + planes/markers/sketches/… + body).
//! Shape is pinned by oracle goldens (`oracle-flange.json`).

/// Regen options (`msg.opts`).
#[derive(Debug, Clone, Default)]
pub struct RegenOpts {
    /// Regenerate only up to this feature index.
    pub up_to: Option<u32>,
}

/// Cache tags seen in app.pretty.js: main, asm, bridge, plain, base, sketch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegenTag {
    Main,
    Asm,
    Bridge,
    Plain,
    Base,
    Sketch,
}

/// One geometry error, wire form: `{"feature":"F4","msg":"..."}`
#[derive(Debug, Clone, Default)]
pub struct RegenError {
    pub feature: Option<String>,
    pub msg: String,
}
