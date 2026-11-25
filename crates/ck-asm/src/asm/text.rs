// SPDX-License-Identifier: MIT
//! The text form of ASM files, as drawings (`.dwg` up to R2010) keep the
//! B-rep record of their solids: the same records as the binary form, one
//! per line or more, written as words and numbers and ended by `#`.
//!
//! [`to_binary`] writes the text as the binary form, which [`AsmFile`]
//! parses as any other; the geometry reader takes what the text writes
//! differently ([`super::geom::Cursor`]): positions and vectors as three
//! numbers, enumerations as words (`open`, `periodic`, ...). The text's
//! logical words become booleans here: `reversed`, `double`, `in`,
//! `rotate`, `reflect`, `shear`, `reversed_v` and `T` are true, their
//! counterparts false; an interval end is `I` (infinite, false) or `F`
//! followed by its value (finite, true).
//!
//! - Strings are `@<length> <bytes>`; pointers `$<index>`; subtype objects
//!   `{ ... }` as in the binary form.
//! - The header: the save version and three counts; the product, the
//!   version and the date as strings (`<length> <bytes>`, with or without
//!   `@`); units, `resabs` and `resnor`.
//! - A record may start with its sequence number (`-12 face ...`).
//! - Files without an `asmheader` record (older versions) get one, so that
//!   the records number as in the binary form (pointers move up by one).
//! - Files of versions before 700 lack the history integer and the pointer
//!   after an entity's attribute pointer; they are added (`-1 $-1`, only
//!   `-1` for `transform`).

use super::file::{AsmError, AsmFile};

/// The value of an enumeration written as a word: closures and
/// singularities of B-spline data.
pub fn enumeration(word: &str) -> Option<i64> {
    Some(match word {
        "open" | "none" => 0,
        "closed" | "start" | "low" => 1,
        "periodic" | "end" | "high" => 2,
        "both" | "full" => 3,
        _ => return None,
    })
}

/// The logical words and their values.
fn logical(word: &str) -> Option<bool> {
    Some(match word {
        "reversed" | "double" | "in" | "rotate" | "reflect" | "shear" | "reversed_v" | "T"
        | "F" => true,
        "forward" | "single" | "out" | "no_rotate" | "no_reflect" | "no_shear" | "forward_v"
        | "I" => false,
        _ => return None,
    })
}

#[derive(Clone, Debug, PartialEq)]
enum Tok {
    Int(i64),
    Double(f64),
    Str(Vec<u8>),
    Ptr(i64),
    Word(String),
    Open,
    Close,
    End,
}

struct Lexer<'a> {
    data: &'a [u8],
    pos: usize,
}

fn error(pos: usize, message: impl Into<String>) -> AsmError {
    AsmError::Token(super::token::TokenError {
        offset: pos,
        message: message.into(),
    })
}

impl Lexer<'_> {
    fn skip_space(&mut self) {
        while self.pos < self.data.len() && self.data[self.pos].is_ascii_whitespace() {
            self.pos += 1;
        }
    }

    fn word(&mut self) -> &[u8] {
        let start = self.pos;
        while self.pos < self.data.len()
            && !self.data[self.pos].is_ascii_whitespace()
            && self.data[self.pos] != b'#'
        {
            self.pos += 1;
        }
        &self.data[start..self.pos]
    }

    /// `<length> <bytes>` after an optional `@`.
    fn string(&mut self) -> Result<Vec<u8>, AsmError> {
        self.skip_space();
        if self.data.get(self.pos) == Some(&b'@') {
            self.pos += 1;
        }
        let at = self.pos;
        let n: usize = std::str::from_utf8(self.word())
            .ok()
            .and_then(|w| w.parse().ok())
            .ok_or_else(|| error(at, "a string without its length"))?;
        // One space separates the length from the bytes.
        if self.data.get(self.pos) == Some(&b' ') {
            self.pos += 1;
        }
        let bytes = self
            .data
            .get(self.pos..self.pos + n)
            .ok_or_else(|| error(self.pos, "a string runs past the end"))?
            .to_vec();
        self.pos += n;
        Ok(bytes)
    }

    fn next(&mut self) -> Result<Option<Tok>, AsmError> {
        self.skip_space();
        let Some(&c) = self.data.get(self.pos) else {
            return Ok(None);
        };
        match c {
            b'#' => {
                self.pos += 1;
                return Ok(Some(Tok::End));
            }
            b'@' => return Ok(Some(Tok::Str(self.string()?))),
            _ => {}
        }
        let at = self.pos;
        let w = self.word();
        let text = std::str::from_utf8(w).map_err(|_| error(at, "a word that is not text"))?;
        Ok(Some(match text {
            "{" => Tok::Open,
            "}" => Tok::Close,
            _ if text.starts_with('$') => Tok::Ptr(
                text[1..]
                    .parse()
                    .map_err(|_| error(at, format!("a pointer {text:?}")))?,
            ),
            _ => number(text).unwrap_or_else(|| Tok::Word(text.to_owned())),
        }))
    }
}

fn number(text: &str) -> Option<Tok> {
    let first = text.chars().next()?;
    if !(first.is_ascii_digit() || first == '-' || first == '+' || first == '.') {
        return None;
    }
    if !text.contains(['.', 'e', 'E'])
        && let Ok(v) = text.parse::<i64>()
    {
        return Some(Tok::Int(v));
    }
    text.parse::<f64>().ok().map(Tok::Double)
}

/// One record: its type name and its fields.
struct Rec {
    name: String,
    fields: Vec<Tok>,
}

/// The binary form of a text ASM file.
pub fn to_binary(text: &[u8]) -> Result<Vec<u8>, AsmError> {
    let mut lx = Lexer { data: text, pos: 0 };
    let mut ints = [0i64; 4];
    for (i, v) in ints.iter_mut().enumerate() {
        match lx.next()? {
            Some(Tok::Int(n)) => *v = n,
            _ => {
                return Err(AsmError::Header(format!(
                    "expected the header's number {i}"
                )));
            }
        }
    }
    let version = ints[0];
    let mut strings = Vec::new();
    for _ in 0..3 {
        strings.push(String::from_utf8_lossy(&lx.string()?).into_owned());
    }
    let mut doubles = Vec::new();
    for _ in 0..3 {
        match lx.next()? {
            Some(Tok::Int(n)) => doubles.push(n as f64),
            Some(Tok::Double(d)) => doubles.push(d),
            _ => return Err(AsmError::Header("expected the header's units".into())),
        }
    }
    // The records.
    let mut records: Vec<Rec> = Vec::new();
    let mut complete = false;
    while let Some(first) = lx.next()? {
        let name = match first {
            // A sequence number before the record (`-0`, `-1`, ...).
            Tok::Int(_) => match lx.next()? {
                Some(Tok::Word(w)) => w,
                _ => return Err(error(lx.pos, "a sequence number without a record")),
            },
            Tok::Word(w) => w,
            other => return Err(error(lx.pos, format!("a record starts with {other:?}"))),
        };
        if name.starts_with("End-of-") && name.ends_with("-data") {
            complete = true;
            break;
        }
        if name.starts_with("Begin-of-") || name.starts_with("End-of-") {
            // History sections: not taken (drawings keep none).
            return Err(error(
                lx.pos,
                format!("{name}: history data in the text form"),
            ));
        }
        let mut fields = Vec::new();
        loop {
            match lx.next()? {
                Some(Tok::End) => break,
                Some(t) => fields.push(t),
                None => return Err(error(lx.pos, format!("the record {name} has no end"))),
            }
        }
        records.push(Rec { name, fields });
    }
    // Older versions end with their last record, without the marker
    // (drawings of R2000 keep version 400 so).
    if !complete && version >= 700 {
        return Err(error(lx.pos, "no end marker"));
    }
    // Older versions: the history integer and the pointer after the
    // attribute pointer, and the fields of the topology the later layout
    // added.
    if version < 700 {
        for r in &mut records {
            if r.fields.is_empty() || !matches!(r.fields[0], Tok::Ptr(_)) {
                continue;
            }
            let mut add = vec![Tok::Int(-1)];
            if r.name != "transform" {
                add.push(Tok::Ptr(-1));
            }
            r.fields.splice(1..1, add);
            older_topology(r);
        }
    }
    let mut entity_count = ints[2];
    if records.first().map(|r| r.name.as_str()) != Some("asmheader") {
        for r in &mut records {
            for f in &mut r.fields {
                if let Tok::Ptr(p) = f
                    && *p >= 0
                {
                    *p += 1;
                }
            }
        }
        records.insert(
            0,
            Rec {
                name: "asmheader".into(),
                fields: vec![
                    Tok::Ptr(-1),
                    Tok::Int(-1),
                    Tok::Str(strings[1].clone().into_bytes()),
                ],
            },
        );
        entity_count += 1;
    }

    let mut w = super::writer::Writer {
        data: b"ASM BinaryFile4".to_vec(),
    };
    for v in [version, 0, entity_count, ints[3]] {
        w.data.extend_from_slice(&(v as i32).to_le_bytes());
    }
    for s in &strings {
        string(&mut w, s.as_bytes());
    }
    for d in doubles {
        w.dbl(d);
    }
    for r in &records {
        w.record(&r.name);
        for f in &r.fields {
            match f {
                Tok::Int(v) => {
                    match i32::try_from(*v) {
                        Ok(v) => w.int(v),
                        Err(_) => w.dbl(*v as f64),
                    };
                }
                Tok::Double(v) => {
                    w.dbl(*v);
                }
                Tok::Str(s) => string(&mut w, s),
                Tok::Ptr(p) => {
                    w.ptr(*p as i32);
                }
                Tok::Word(word) => match logical(word) {
                    Some(b) => {
                        w.bool(b);
                    }
                    None => {
                        w.ident(word);
                    }
                },
                Tok::Open => {
                    w.sub_start();
                }
                Tok::Close => {
                    w.sub_end();
                }
                Tok::End => {}
            }
        }
        w.end();
    }
    Ok(w.finish())
}

/// The topology of older versions (400) in the later layout: an edge has
/// no parameters (start vertex, end vertex, coedge, curve, sense), so its
/// parameters are written as NaN and the converter takes them from where
/// its vertices lie on its curve; a vertex has no end integer (edge,
/// point) and a coedge no integer before its pcurve: both are 0.
fn older_topology(r: &mut Rec) {
    let ptrs = |f: &[Tok]| f.iter().all(|t| matches!(t, Tok::Ptr(_)));
    let f = &mut r.fields;
    match r.name.as_str() {
        // attribute, -1, $-1, start, end, coedge, curve, sense
        "edge" if f.len() == 8 && ptrs(&f[3..7]) => {
            f.insert(5, Tok::Double(f64::NAN));
            f.insert(4, Tok::Double(f64::NAN));
        }
        // attribute, -1, $-1, edge, point
        "vertex" if f.len() == 5 && ptrs(&f[3..5]) => f.insert(4, Tok::Int(0)),
        // attribute, -1, $-1, next, previous, partner, edge, sense, loop,
        // pcurve
        "coedge" if f.len() == 10 && ptrs(&f[3..7]) && ptrs(&f[8..10]) => f.insert(9, Tok::Int(0)),
        _ => {}
    }
}

/// A string token: one-byte length up to 255 bytes, else four.
fn string(w: &mut super::writer::Writer, s: &[u8]) {
    if s.len() <= 255 {
        w.data.push(0x07);
        w.data.push(s.len() as u8);
    } else {
        w.data.push(0x09);
        w.data.extend_from_slice(&(s.len() as u32).to_le_bytes());
    }
    w.data.extend_from_slice(s);
}

/// Parses a text ASM file.
pub fn parse(text: &[u8]) -> Result<AsmFile, AsmError> {
    AsmFile::parse(&to_binary(text)?)
}

/// Writes the text form of a binary ASM file (the drawings' test files):
/// positions and vectors as three numbers, booleans as `T` and `I` (both
/// read back as written whatever the field), enumerations as words.
pub fn from_binary(file: &AsmFile) -> String {
    use super::token::Token;
    let h = &file.header;
    let mut out = format!(
        "{} {} {} {}\n@{} {} @{} {} @{} {}\n{} {} {}\n",
        h.version,
        h.record_count,
        h.entity_count,
        h.flags,
        h.product.len(),
        h.product,
        h.asm_version.len(),
        h.asm_version,
        h.date.len(),
        h.date,
        fmt(h.units),
        fmt(h.resabs),
        fmt(h.resnor)
    );
    for (i, r) in file.records.iter().enumerate() {
        out.push_str(&format!("-{i} {}", r.type_name));
        for t in file.fields(i) {
            out.push(' ');
            match t {
                Token::Int(v) => out.push_str(&v.to_string()),
                Token::Double(v) => out.push_str(&fmt(*v)),
                Token::Str(s) | Token::Literal(s) => out.push_str(&format!("@{} {s}", s.len())),
                Token::True => out.push('T'),
                Token::False => out.push('I'),
                Token::Ptr(p) => out.push_str(&format!("${p}")),
                Token::Ident(s) => out.push_str(s),
                Token::IdentPrefix(s) => out.push_str(&format!("{s}-")),
                Token::SubStart => out.push('{'),
                Token::SubEnd => out.push('}'),
                Token::End => {}
                Token::Pos(p) | Token::Vector(p) => {
                    out.push_str(&format!("{} {} {}", fmt(p[0]), fmt(p[1]), fmt(p[2])))
                }
                Token::Enum(v) => out.push_str(match v {
                    0 => "open",
                    1 => "closed",
                    2 => "periodic",
                    _ => "both",
                }),
                Token::Pair(p) => out.push_str(&format!("{} {}", fmt(p[0]), fmt(p[1]))),
            }
        }
        out.push_str(" #\n");
    }
    out.push_str("End-of-ASM-data\n");
    out
}

/// A double as the text form writes it (always with a point or exponent,
/// so that it reads back as a double).
fn fmt(v: f64) -> String {
    let s = format!("{v:?}");
    if s.contains(['.', 'e', 'E', 'N', 'i']) {
        s
    } else {
        format!("{s}.0")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::convert;

    #[test]
    fn the_text_form_converts_as_the_binary_one() {
        for blob in [
            crate::testdata::cube_blob(),
            crate::testdata::cylinder_blob(),
            crate::testdata::cone_blob(),
        ] {
            let binary = AsmFile::parse(&blob).unwrap();
            let text = from_binary(&binary);
            let file = parse(text.as_bytes()).unwrap_or_else(|e| panic!("{e}\n{text}"));
            assert!(file.complete);
            assert_eq!(file.records.len(), binary.records.len());
            let options = convert::Options::default();
            let a = convert::convert_file(&binary, &options);
            let b = convert::convert_file(&file, &options);
            assert_eq!(a.len(), b.len());
            for (x, y) in a.iter().zip(&b) {
                assert_eq!(x.body.faces.len(), y.body.faces.len(), "{text}");
                assert_eq!(x.issues, y.issues);
                assert_eq!(x.skipped_faces, y.skipped_faces);
                assert_eq!(x.body.vertices.len(), y.body.vertices.len());
                for (p, q) in x.body.vertices.iter().zip(&y.body.vertices) {
                    assert_eq!(p.point, q.point);
                }
            }
        }
    }

    /// A cylinder of radius 1 along x from 0 to 2 in the layout of version
    /// 400 as drawings of R2000 keep it: no end marker, edges without
    /// parameters, vertices without their end integer, coedges without the
    /// integer before the pcurve.
    const CYLINDER_400: &str = "400 26 1 0\n\
        14 Mitcad testing 7 ASM 4.0 4 date\n\
        1 9.9999999999999995e-07 1e-10\n\
        body $-1 $1 $-1 $2 #\n\
        lump $-1 $-1 $3 $0 #\n\
        transform $-1 1 0 0 0 1 0 0 0 1 0 0 0 1 no_rotate no_reflect no_shear #\n\
        shell $-1 $-1 $-1 $4 $-1 $1 #\n\
        face $-1 $5 $6 $3 $-1 $7 forward single #\n\
        face $-1 $8 $9 $3 $-1 $10 reversed single #\n\
        loop $-1 $11 $12 $4 #\n\
        cone-surface $-1 2 0 0 -1 0 0 0 0 -1 1 I I 0 1 1 forward I I I I #\n\
        face $-1 $-1 $13 $3 $-1 $14 forward single #\n\
        loop $-1 $-1 $15 $5 #\n\
        plane-surface $-1 2 0 0 -1 0 0 0 0 1 forward_v I I I I #\n\
        loop $-1 $-1 $16 $4 #\n\
        coedge $-1 $12 $12 $17 $18 forward $6 $-1 #\n\
        loop $-1 $-1 $17 $8 #\n\
        plane-surface $-1 0 0 0 -1 0 0 0 0 1 forward_v I I I I #\n\
        coedge $-1 $15 $15 $16 $19 reversed $9 $-1 #\n\
        coedge $-1 $16 $16 $15 $19 forward $11 $-1 #\n\
        coedge $-1 $17 $17 $12 $18 reversed $13 $-1 #\n\
        edge $-1 $20 $20 $12 $21 forward #\n\
        edge $-1 $22 $22 $16 $23 forward #\n\
        vertex $-1 $18 $24 #\n\
        ellipse-curve $-1 0 0 0 1 0 0 0 0 -1 1 I I #\n\
        vertex $-1 $19 $25 #\n\
        ellipse-curve $-1 2 0 0 -1 0 0 0 0 -1 1 I I #\n\
        point $-1 0 1 0 #\n\
        point $-1 2 1 0 #\n";

    #[test]
    fn version_400_converts_with_edge_parameters_from_the_vertices() {
        let file = parse(CYLINDER_400.as_bytes()).unwrap();
        let bodies = convert::convert_file(
            &file,
            &convert::Options {
                scale: 1.0,
                ..Default::default()
            },
        );
        assert_eq!(bodies.len(), 1);
        let b = &bodies[0];
        assert!(b.issues.is_empty(), "{:?}", b.issues);
        assert_eq!(b.body.faces.len(), 3);
        // Two full circles, each from its vertex once around.
        assert_eq!(b.body.edges.len(), 2);
        for e in &b.body.edges {
            let length = e.t[1] - e.t[0];
            assert!((length - std::f64::consts::TAU).abs() < 1e-9, "{:?}", e.t);
            let start = b.body.curves[e.curve].eval(e.t[0]);
            let v = b.body.vertices[e.vertices[0]].point;
            assert!(crate::brep::dist(start, v) < 1e-9, "{start:?} {v:?}");
        }
    }

    #[test]
    fn version_208_vertices_have_no_end_integer() {
        // The layout of ASM 208 (drawings of R2004): the history integer
        // and pointer, edge parameters, but vertices and coedges without
        // their integers.
        let text = CYLINDER_400
            .replacen("400 26 1 0", "20800 26 1 0", 1)
            .replace(
                "edge $-1 $20 $20 $12 $21 forward",
                "edge $-1 -1 $-1 $20 0 $20 6.283185307179586 $12 $21 forward @7 unknown",
            )
            .replace(
                "edge $-1 $22 $22 $16 $23 forward",
                "edge $-1 -1 $-1 $22 0 $22 6.283185307179586 $16 $23 forward @7 unknown",
            );
        // Every other record gets the history integer and pointer.
        let mut lines: Vec<String> = Vec::new();
        for (i, line) in text.lines().enumerate() {
            if i < 3 || line.starts_with("edge") {
                lines.push(line.to_owned());
            } else if let Some((name, rest)) = line.split_once(" $-1 ") {
                let head = if name == "transform" {
                    " $-1 -1 "
                } else {
                    " $-1 -1 $-1 "
                };
                lines.push(format!("{name}{head}{rest}"));
            }
        }
        lines.push("End-of-ASM-data".into());
        let file = parse(lines.join("\n").as_bytes()).unwrap();
        let bodies = convert::convert_file(
            &file,
            &convert::Options {
                scale: 1.0,
                ..Default::default()
            },
        );
        assert!(bodies[0].issues.is_empty(), "{:?}", bodies[0].issues);
        assert_eq!(bodies[0].body.faces.len(), 3);
    }

    #[test]
    fn older_text_gets_the_header_record_and_fields() {
        let text = b"400 0 1 0\n14 A test product 7 ASM 4.0 1 x\n10 9.9999999999999995e-07 1e-10\n\
                     body $-1 $1 $-1 $-1 #\nlump $-1 $-1 $-1 $0 #\nEnd-of-ASM-data\n";
        let file = parse(text).unwrap();
        assert_eq!(file.records[0].type_name, "asmheader");
        assert_eq!(file.records[1].type_name, "body");
        assert_eq!(file.records.len(), 3);
        // body: attribute, history, pointer, then the lump (record 2).
        assert_eq!(file.record_text(1, 200), "1 body $-1 -1 $-1 $2 $-1 $-1");
        assert_eq!(file.header.entity_count, 2);
        assert_eq!(file.header.units, 10.0);
    }

    #[test]
    fn words() {
        assert_eq!(enumeration("periodic"), Some(2));
        assert_eq!(logical("reversed"), Some(true));
        assert_eq!(logical("I"), Some(false));
        assert_eq!(number("-1"), Some(Tok::Int(-1)));
        assert_eq!(number("1e-10"), Some(Tok::Double(1e-10)));
        assert_eq!(number("open"), None);
    }
}
