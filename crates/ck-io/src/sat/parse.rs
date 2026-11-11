//! ACIS SAT text tokenizer and record table.
//!
//! Adapted from the author's own earlier ACIS 7.00 SAT text parser
//! (dependency-free); the tokenizer was rewritten to
//! split records on the `#` terminator (records may span lines, e.g. spline
//! blocks), to accept optional `-N` record-index prefixes and to keep the
//! header's unit scale.

use super::SatError;

/// One SAT record: `t[0]` is the entity type (index prefix stripped).
#[derive(Debug, Clone)]
pub struct Rec {
    /// 0-based ordinal among the records after the header (what `$N` means).
    pub idx: usize,
    pub line: usize,
    pub t: Vec<String>,
}

impl Rec {
    pub fn ty(&self) -> &str {
        &self.t[0]
    }
}

pub struct Table {
    pub recs: Vec<Rec>,
    /// Millimetres per model unit (header line 3, first number).
    pub units: f64,
    pub version: u32,
    /// Inline subtype definitions (`{ exactcur ... }`, `{ exppc ... }`, ...)
    /// in file order, as (record ordinal, token index of the `{`). ACIS
    /// `{ ref N }` points at entry N.
    pub defs: Vec<(usize, usize)>,
}

fn is_index_prefix(s: &str) -> bool {
    s.len() > 1 && s.starts_with('-') && s[1..].bytes().all(|b| b.is_ascii_digit())
}

/// Entity type canonicalisation: tolerant variants map onto their base type.
fn canon(ty: &str) -> String {
    match ty {
        "tedge-edge" => "edge".into(),
        "tvertex-vertex" => "vertex".into(),
        "tcoedge-coedge" => "coedge".into(),
        other => other.to_string(),
    }
}

pub fn parse_table(text: &str) -> Result<Table, SatError> {
    // old-Mac exports use CR-only line endings, Windows ones CRLF
    let norm;
    let text = if text.contains('\r') {
        norm = text.replace("\r\n", "\n").replace('\r', "\n");
        norm.as_str()
    } else {
        text
    };
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let mut lines = text.split_inclusive('\n');
    let l1 = lines.next().ok_or_else(|| SatError::parse(1, "empty file"))?;
    let l2 = lines.next();
    let l3 = lines.next();
    let h1: Vec<&str> = l1.split_whitespace().collect();
    let version: u32 = h1.first().and_then(|s| s.parse().ok()).ok_or_else(|| SatError::parse(1, "bad SAT header (version)"))?;
    let units = l3
        .and_then(|l| l.split_whitespace().next())
        .and_then(|s| s.parse::<f64>().ok())
        .filter(|u| u.is_finite() && *u > 0.0)
        .ok_or_else(|| SatError::parse(3, "bad SAT header (units)"))?;
    let consumed: usize = l1.len() + l2.map_or(0, |l| l.len()) + l3.map_or(0, |l| l.len());
    let body = &text[consumed..];
    let bytes = body.as_bytes();
    let mut line = 4usize;
    let mut i = 0usize;
    let mut cur: Vec<String> = vec![];
    let mut cur_line = line;
    let mut recs: Vec<Rec> = vec![];
    let finish = |cur: &mut Vec<String>, recs: &mut Vec<Rec>, ln: usize| {
        if cur.is_empty() {
            return;
        }
        let mut t = std::mem::take(cur);
        if is_index_prefix(&t[0]) && t.len() > 1 {
            t.remove(0);
        }
        t[0] = canon(&t[0]);
        // ACIS >= ~20 writes a second integer after the entity id
        // (`body $att -1 -1 $hist ...`); drop it so field offsets match 7.0.
        // Attributes and transforms have their own layouts.
        if !t[0].ends_with("attrib") && t[0] != "transform" && t.len() > 3 && !t[3].starts_with('$') && t[3].parse::<i64>().is_ok() {
            t.remove(3);
        }
        let idx = recs.len();
        recs.push(Rec { idx, line: ln, t });
    };
    'outer: while i < bytes.len() {
        let c = bytes[i];
        if c == b'\n' {
            line += 1;
            i += 1;
            continue;
        }
        if c.is_ascii_whitespace() {
            i += 1;
            continue;
        }
        if cur.is_empty() {
            cur_line = line;
        }
        match c {
            b'#' => {
                finish(&mut cur, &mut recs, cur_line);
                i += 1;
            }
            b'{' | b'}' => {
                cur.push((c as char).to_string());
                i += 1;
            }
            b'@' => {
                let mut j = i + 1;
                while j < bytes.len() && bytes[j].is_ascii_digit() {
                    j += 1;
                }
                let n: usize = body[i + 1..j].parse().unwrap_or(0);
                j += 1; // the single separating space
                let end = (j + n).min(bytes.len());
                let s = String::from_utf8_lossy(&bytes[j.min(end)..end]).to_string();
                line += s.bytes().filter(|&b| b == b'\n').count();
                cur.push(s);
                i = end;
            }
            _ => {
                let mut j = i;
                while j < bytes.len() && !bytes[j].is_ascii_whitespace() && bytes[j] != b'{' && bytes[j] != b'}' {
                    j += 1;
                }
                let tok = &body[i..j];
                if tok.starts_with("End-of-ACIS") {
                    break 'outer;
                }
                cur.push(tok.to_string());
                i = j;
            }
        }
    }
    finish(&mut cur, &mut recs, cur_line);
    let mut defs = vec![];
    for r in &recs {
        for (k, tok) in r.t.iter().enumerate() {
            if tok == "{" && r.t.get(k + 1).map(|s| s.as_str()) != Some("ref") {
                defs.push((r.idx, k));
            }
        }
    }
    Ok(Table { recs, units, version, defs })
}
