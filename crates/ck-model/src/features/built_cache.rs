//! Saved results of the expensive import features (`importBody`), so that a later open of the document skips the
//! rebuild. See `docs/regen-cache.md` ("Saved bodies") and the book page of `importBody`.
//!
//! A saved result is what the import builds from its file: the solid body, the surface bodies and what became of each
//! part ([`import_body::Sorted`]), written with `ck_topo::codec` and closed with a SHA-256 of everything before it. The
//! document keeps it in its top-level `built` table, `{ "<key>": "<base64 of the bytes>" }`.
//!
//! It is a cache and never an input: the feature reads the entry only after the key, the checksum and the body's own
//! structure all check out, and in every other case (no entry, a different kernel, damage) it builds from the file as it
//! always did. The result is the same either way; only the time differs. The key is a SHA-256 over what the result is a
//! function of: the file (the blob's own key, itself a SHA-256 of its text), the feature id (faces and surface bodies are
//! named after it), the kind of import, the kernel version and [`BUILT_REV`] (bump it whenever an import would build a
//! different body from the same file without the kernel version changing) and the codec version.
//!
//! The results built or read in this process are remembered (a few, newest last) so that the `builtExport` request can
//! hand them to the app, which stores them in the document or beside it.

use super::import_body::Sorted;
use ck_heal::honest::{Outcome, Part, Reason};
use ck_json::Json;
use ck_topo::codec::{self, R, W};
use std::cell::RefCell;
use std::rc::Rc;

/// Bump when an import builds another body from the same file at the same kernel version.
pub const BUILT_REV: u32 = 1;
const MAGIC: &[u8; 4] = b"CKBS";
/// Results kept for export, newest last.
const KEEP: usize = 4;

/// The key a result of `kind` (`"importBody"`) for feature `feature` and file `blob_key` is saved under.
pub fn key(kind: &str, blob_key: &str, feature: &str) -> String {
    let ver = format!("{kind}|{blob_key}|{feature}|{}|{BUILT_REV}|{}", env!("CARGO_PKG_VERSION"), codec::VERSION);
    ck_text::sha256_hex(&ck_text::sha256(&[ver.as_bytes()]))
}

fn w_part(w: &mut W, p: &Part) {
    w.str(&p.label);
    w.u8(match p.outcome {
        Outcome::Solid => 0,
        Outcome::Surface => 1,
        Outcome::Refused => 2,
    });
    w.len(p.faces);
    w.len(p.faces_total);
    w.len(p.reasons.len());
    for r in &p.reasons {
        w.str(&r.code);
        w.len(r.faces.len());
        for f in &r.faces {
            w.str(f);
        }
        w.str(&r.message);
    }
}

fn r_part(r: &mut R) -> Result<Part, String> {
    let label = r.str()?;
    let outcome = match r.u8()? {
        0 => Outcome::Solid,
        1 => Outcome::Surface,
        2 => Outcome::Refused,
        t => return Err(format!("bad outcome {t}")),
    };
    let (faces, faces_total) = (r.u32()? as usize, r.u32()? as usize);
    let n = r.count(12)?;
    let mut reasons = Vec::with_capacity(n);
    for _ in 0..n {
        let code = r.str()?;
        let nf = r.count(4)?;
        let faces = (0..nf).map(|_| r.str()).collect::<Result<Vec<_>, _>>()?;
        reasons.push(Reason { code, faces, message: r.str()? });
    }
    Ok(Part { label, outcome, faces, faces_total, reasons })
}

/// The saved form of `s`: bytes closed with their SHA-256.
pub fn encode(s: &Sorted) -> Vec<u8> {
    let mut w = W::new();
    w.buf.extend_from_slice(MAGIC);
    let body = codec::encode(&s.solid);
    w.len(body.len());
    w.buf.extend_from_slice(&body);
    w.len(s.surfaces.len());
    for (id, b) in &s.surfaces {
        w.str(id);
        let bytes = codec::encode(b);
        w.len(bytes.len());
        w.buf.extend_from_slice(&bytes);
    }
    w.len(s.parts.len());
    for p in &s.parts {
        w_part(&mut w, p);
    }
    let sum = ck_text::sha256(&[&w.buf]);
    w.buf.extend_from_slice(&sum);
    w.buf
}

/// The result `bytes` hold, or why they are not one (checksum, structure, anything).
pub fn decode(bytes: &[u8]) -> Result<Sorted, String> {
    if bytes.len() < MAGIC.len() + 32 {
        return Err("too short".into());
    }
    let (data, sum) = bytes.split_at(bytes.len() - 32);
    if ck_text::sha256(&[data]) != sum {
        return Err("checksum does not match".into());
    }
    if &data[..4] != MAGIC {
        return Err("not a saved import".into());
    }
    let mut r = R::new(&data[4..]);
    let mut sub = |r: &mut R| -> Result<ck_topo::body::Body, String> {
        let n = r.count(1)?;
        codec::decode(r.bytes(n)?)
    };
    let solid = sub(&mut r)?;
    let ns = r.count(8)?;
    let mut surfaces = Vec::with_capacity(ns);
    for _ in 0..ns {
        let id = r.str()?;
        surfaces.push((id, sub(&mut r)?));
    }
    let np = r.count(10)?;
    let parts = (0..np).map(|_| r_part(&mut r)).collect::<Result<Vec<_>, _>>()?;
    if !r.at_end() {
        return Err("trailing bytes".into());
    }
    Ok(Sorted { solid, surfaces, parts })
}

/// What happened to the saved result of a feature in this process.
#[derive(Clone, Debug, PartialEq)]
pub enum Source {
    /// Built from the file.
    Built,
    /// Read from the document's `built` table.
    Saved,
}

#[derive(Clone)]
pub struct Entry {
    pub feature: String,
    pub key: String,
    pub source: Source,
    /// Why a saved entry under the key was not used (it was rebuilt), if one was present.
    pub rejected: Option<String>,
    pub bytes: Rc<Vec<u8>>,
}

thread_local! {
    static KEPT: RefCell<Vec<Entry>> = const { RefCell::new(Vec::new()) };
}

fn remember(e: Entry) {
    KEPT.with(|k| {
        let mut k = k.borrow_mut();
        k.retain(|x| x.key != e.key);
        k.push(e);
        let n = k.len();
        if n > KEEP {
            k.drain(..n - KEEP);
        }
    });
}

/// The results built or read in this process, oldest first.
pub fn kept() -> Vec<Entry> {
    KEPT.with(|k| k.borrow().clone())
}

pub fn forget_all() {
    KEPT.with(|k| k.borrow_mut().clear());
}

/// The saved result under `key` in the document's `built` table, if there is one and it holds up. `Err` is the reason a
/// present entry was not used.
pub fn lookup(part_raw: &Json, key: &str) -> Option<Result<(Sorted, Vec<u8>), String>> {
    let text = part_raw.get("built")?.get(key)?.as_str()?;
    Some(ck_text::base64_decode(text).and_then(|b| decode(&b).map(|s| (s, b))))
}

/// Run the import of `feature` through the cache: the saved result when the document has a good one under the key,
/// else `build` (and the result is remembered for export). `kind` and `blob_key` make the key.
pub fn through(part_raw: &Json, kind: &str, blob_key: &str, feature: &str, build: impl FnOnce() -> Result<Sorted, crate::regen::FeatureError>) -> Result<Sorted, crate::regen::FeatureError> {
    let key = key(kind, blob_key, feature);
    let mut rejected = None;
    match lookup(part_raw, &key) {
        Some(Ok((s, bytes))) => {
            remember(Entry { feature: feature.into(), key, source: Source::Saved, rejected: None, bytes: Rc::new(bytes) });
            return Ok(s);
        }
        Some(Err(why)) => rejected = Some(why),
        None => {}
    }
    let s = build()?;
    remember(Entry { feature: feature.into(), key, source: Source::Built, rejected, bytes: Rc::new(encode(&s)) });
    Ok(s)
}
