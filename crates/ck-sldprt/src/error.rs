//! Structured errors: the reader never panics on malformed input.

use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub struct SldprtError {
    /// Stable machine code: `container.*`, `inflate.*`, `limit.*`, `xt.*`, `topology.*`, `geometry.*`, `unsupported.*`.
    pub code: &'static str,
    pub message: String,
}

impl SldprtError {
    pub fn new(code: &'static str, message: impl Into<String>) -> SldprtError {
        SldprtError { code, message: message.into() }
    }
}

impl fmt::Display for SldprtError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.code, self.message)
    }
}

impl std::error::Error for SldprtError {}

pub type R<T> = Result<T, SldprtError>;

/// Safety limits for untrusted input (all configurable).
#[derive(Debug, Clone)]
pub struct Limits {
    /// Largest accepted file.
    pub max_file: usize,
    /// Largest single inflated stream.
    pub max_stream: usize,
    /// Largest total of all inflated streams.
    pub max_total: usize,
    /// Largest inflated/deflated ratio accepted for one stream (decompression-bomb guard), checked once the output exceeds 1 MiB.
    pub max_ratio: usize,
    /// Most streams read from one file.
    pub max_streams: usize,
    /// Most XT nodes.
    pub max_nodes: usize,
    /// Most faces imported.
    pub max_faces: usize,
}

impl Default for Limits {
    fn default() -> Limits {
        Limits { max_file: 256 << 20, max_stream: 256 << 20, max_total: 768 << 20, max_ratio: 2000, max_streams: 4096, max_nodes: 4_000_000, max_faces: 50_000 }
    }
}
