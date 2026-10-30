//! ISO 10303-21 (STEP Part 21) exchange-structure parser.
//!
//! Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/io/src/step/reader.rs
//! (licence texts: crates/ck-io/third_party/brepkit/). brepkit splits the DATA
//! section on `;` and hunts attribute substrings; this version keeps its
//! entity model (`#id = TYPE(attrs)`, complex entities `(A(..)B(..))`,
//! B-spline knot-multiplicity expansion, rational weight grids) but parses
//! attributes into a real value tree so nesting, strings holding `;`/`(`,
//! comments and typed parameters (`LENGTH_MEASURE(5.)`) are handled exactly.

use std::collections::HashMap;
use std::fmt;

/// One attribute value.
#[derive(Debug, Clone, PartialEq)]
pub enum Val {
    /// `$` (unset)
    Null,
    /// `*` (derived)
    Star,
    Int(i64),
    Real(f64),
    Str(String),
    /// `.NAME.` (also `.T.` / `.F.`)
    Enum(String),
    Ref(u32),
    List(Vec<Val>),
    /// `NAME(args)` used as a value (`LENGTH_MEASURE(5.)`)
    Typed(String, Vec<Val>),
}

impl Val {
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Val::Real(x) => Some(*x),
            Val::Int(i) => Some(*i as f64),
            Val::Typed(_, a) if a.len() == 1 => a[0].as_f64(),
            _ => None,
        }
    }
    pub fn as_usize(&self) -> Option<usize> {
        match self {
            Val::Int(i) if *i >= 0 => Some(*i as usize),
            // below 2^53 the float is an exact integer; above it a saturating cast
            // would hand out usize::MAX as a "count"
            Val::Real(x) if *x >= 0.0 && x.fract() == 0.0 && *x < 9.007_199_254_740_992e15 => Some(*x as usize),
            _ => None,
        }
    }
    pub fn as_ref(&self) -> Option<u32> {
        match self {
            Val::Ref(r) => Some(*r),
            _ => None,
        }
    }
    pub fn as_list(&self) -> Option<&[Val]> {
        match self {
            Val::List(l) => Some(l),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Val::Str(s) => Some(s),
            _ => None,
        }
    }
    /// `.T.` / `.F.` (also `.TRUE.` / `.FALSE.`); anything else None.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Val::Enum(e) => match e.as_str() {
                "T" | "TRUE" => Some(true),
                "F" | "FALSE" => Some(false),
                _ => None,
            },
            _ => None,
        }
    }
}

/// One keyword + argument list of an entity instance.
#[derive(Debug, Clone, PartialEq)]
pub struct Part {
    pub name: String,
    pub args: Vec<Val>,
}

/// An entity instance: one part for a simple entity, several for a complex
/// (`#9=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));`).
#[derive(Debug, Clone, PartialEq)]
pub struct Ent {
    pub parts: Vec<Part>,
}

impl Ent {
    pub fn part(&self, name: &str) -> Option<&Part> {
        self.parts.iter().find(|p| p.name == name)
    }
    pub fn has(&self, name: &str) -> bool {
        self.part(name).is_some()
    }
    /// Keyword of a simple entity (first part otherwise).
    pub fn kind(&self) -> &str {
        self.parts.first().map_or("", |p| p.name.as_str())
    }
}

/// Structured import failure; `entity` is the STEP `#id` it concerns.
#[derive(Debug, Clone, PartialEq)]
pub struct ImportError {
    pub code: &'static str,
    pub entity: Option<u32>,
    pub message: String,
}

impl ImportError {
    pub fn new(code: &'static str, entity: Option<u32>, message: impl Into<String>) -> Self {
        ImportError { code, entity, message: message.into() }
    }
}

impl fmt::Display for ImportError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.entity {
            Some(e) => write!(f, "{} (#{e}): {}", self.code, self.message),
            None => write!(f, "{}: {}", self.code, self.message),
        }
    }
}

/// A parsed file: the DATA section instances.
#[derive(Debug, Default)]
pub struct StepFile {
    pub ents: HashMap<u32, Ent>,
}

/// Deepest nesting of lists / typed parameters accepted in one attribute. Real
/// files nest a handful of levels; the parser recurses once per level, so an
/// unbounded depth is a stack overflow (a crash) on hostile input.
pub const MAX_NESTING: usize = 64;

struct Lexer<'a> {
    b: &'a [u8],
    i: usize,
    /// current list / typed-parameter nesting
    depth: usize,
}

type PResult<T> = Result<T, ImportError>;

fn perr(l: &Lexer, msg: &str) -> ImportError {
    let line = 1 + l.b[..l.i.min(l.b.len())].iter().filter(|&&c| c == b'\n').count();
    ImportError::new("parse", None, format!("{msg} at line {line}"))
}

impl<'a> Lexer<'a> {
    fn ws(&mut self) {
        loop {
            while self.i < self.b.len() && self.b[self.i].is_ascii_whitespace() {
                self.i += 1;
            }
            if self.b.get(self.i..).is_some_and(|s| s.starts_with(b"/*")) {
                match find(&self.b[self.i + 2..], b"*/") {
                    Some(k) => self.i += k + 4,
                    None => self.i = self.b.len(),
                }
            } else {
                break;
            }
        }
    }
    fn peek(&mut self) -> Option<u8> {
        self.ws();
        self.b.get(self.i).copied()
    }
    fn eat(&mut self, c: u8) -> bool {
        if self.peek() == Some(c) {
            self.i += 1;
            true
        } else {
            false
        }
    }
    fn ident(&mut self) -> Option<String> {
        self.ws();
        let s = self.i;
        while self.i < self.b.len() && (self.b[self.i].is_ascii_alphanumeric() || self.b[self.i] == b'_') {
            self.i += 1;
        }
        (self.i > s).then(|| String::from_utf8_lossy(&self.b[s..self.i]).to_ascii_uppercase())
    }

    fn string(&mut self) -> PResult<String> {
        // at the opening quote
        self.i += 1;
        let mut raw = Vec::new();
        loop {
            match self.b.get(self.i) {
                None => return Err(perr(self, "unterminated string")),
                Some(b'\'') => {
                    if self.b.get(self.i + 1) == Some(&b'\'') {
                        raw.push(b'\'');
                        self.i += 2;
                    } else {
                        self.i += 1;
                        break;
                    }
                }
                Some(&c) => {
                    raw.push(c);
                    self.i += 1;
                }
            }
        }
        Ok(decode_string(&String::from_utf8_lossy(&raw)))
    }

    fn number(&mut self) -> PResult<Val> {
        let s = self.i;
        while self.i < self.b.len() && matches!(self.b[self.i], b'0'..=b'9' | b'+' | b'-' | b'.' | b'e' | b'E') {
            self.i += 1;
        }
        let t = std::str::from_utf8(&self.b[s..self.i]).unwrap_or("");
        if !t.contains(['.', 'e', 'E']) {
            if let Ok(i) = t.parse::<i64>() {
                return Ok(Val::Int(i));
            }
        }
        // STEP reals may end in '.' ("1.") or start with it (".5"); Rust's
        // parser takes both.
        match t.parse::<f64>() {
            // "1E999" parses to infinity; a non-finite coordinate poisons every
            // downstream computation, so it is refused at the source
            Ok(x) if x.is_finite() => Ok(Val::Real(x)),
            Ok(_) => Err(perr(self, &format!("real {t:?} is out of range"))),
            Err(_) => Err(perr(self, &format!("bad number {t:?}"))),
        }
    }

    fn value(&mut self) -> PResult<Val> {
        if self.depth >= MAX_NESTING {
            return Err(perr(self, "parameters nested too deeply"));
        }
        self.depth += 1;
        let r = self.value_inner();
        self.depth -= 1;
        r
    }

    fn value_inner(&mut self) -> PResult<Val> {
        match self.peek() {
            None => Err(perr(self, "unexpected end of data")),
            Some(b'#') => {
                self.i += 1;
                let s = self.i;
                while self.i < self.b.len() && self.b[self.i].is_ascii_digit() {
                    self.i += 1;
                }
                std::str::from_utf8(&self.b[s..self.i])
                    .ok()
                    .and_then(|t| t.parse::<u32>().ok())
                    .map(Val::Ref)
                    .ok_or_else(|| perr(self, "bad entity reference"))
            }
            Some(b'$') => {
                self.i += 1;
                Ok(Val::Null)
            }
            Some(b'*') => {
                self.i += 1;
                Ok(Val::Star)
            }
            Some(b'\'') => self.string().map(Val::Str),
            Some(b'(') => {
                self.i += 1;
                Ok(Val::List(self.args_until_close()?))
            }
            Some(b'.') if self.b.get(self.i + 1).is_some_and(|c| c.is_ascii_alphabetic()) => {
                self.i += 1;
                let s = self.i;
                while self.i < self.b.len() && self.b[self.i] != b'.' {
                    self.i += 1;
                }
                let e = String::from_utf8_lossy(&self.b[s..self.i]).to_ascii_uppercase();
                if self.i >= self.b.len() {
                    return Err(perr(self, "unterminated enumeration"));
                }
                self.i += 1;
                Ok(Val::Enum(e))
            }
            Some(c) if c.is_ascii_digit() || matches!(c, b'+' | b'-' | b'.') => self.number(),
            Some(c) if c.is_ascii_alphabetic() || c == b'_' => {
                let name = self.ident().unwrap_or_default();
                if self.eat(b'(') {
                    Ok(Val::Typed(canonical_name(name), self.args_until_close()?))
                } else {
                    Err(perr(self, &format!("unexpected word {name}")))
                }
            }
            Some(c) => Err(perr(self, &format!("unexpected character {:?}", c as char))),
        }
    }

    /// Values up to and including the closing paren (opening already eaten).
    fn args_until_close(&mut self) -> PResult<Vec<Val>> {
        let mut out = Vec::new();
        if self.eat(b')') {
            return Ok(out);
        }
        loop {
            out.push(self.value()?);
            if self.eat(b',') {
                continue;
            }
            if self.eat(b')') {
                return Ok(out);
            }
            return Err(perr(self, "expected ',' or ')'"));
        }
    }

    fn part(&mut self) -> PResult<Part> {
        let name = canonical_name(self.ident().ok_or_else(|| perr(self, "expected entity keyword"))?);
        if !self.eat(b'(') {
            return Err(perr(self, "expected '(' after keyword"));
        }
        Ok(Part { name, args: self.args_until_close()? })
    }
}

/// Part 21 allows an entity's short name wherever its long name is valid. Only
/// the geometry / topology entities the reader consumes are mapped.
fn canonical_name(n: String) -> String {
    const SHORT: [(&str, &str); 11] = [
        ("FCSRF", "FACE_SURFACE"),
        ("FCOTBN", "FACE_OUTER_BOUND"),
        ("PLYLP", "POLY_LOOP"),
        ("CRTPNT", "CARTESIAN_POINT"),
        ("DRCTN", "DIRECTION"),
        ("A2PL3D", "AXIS2_PLACEMENT_3D"),
        ("CLSSHL", "CLOSED_SHELL"),
        ("BSSWK", "B_SPLINE_SURFACE_WITH_KNOTS"),
        ("BSCWK", "B_SPLINE_CURVE_WITH_KNOTS"),
        ("TRMCRV", "TRIMMED_CURVE"),
        ("CMCRSG", "COMPOSITE_CURVE"),
    ];
    match SHORT.iter().find(|(s, _)| *s == n) {
        Some((_, long)) => (*long).to_string(),
        None => n,
    }
}

fn find(h: &[u8], n: &[u8]) -> Option<usize> {
    h.windows(n.len()).position(|w| w == n)
}

/// `\X2\0041\X0\` (UTF-16 hex), `\X\41` (ISO 8859-1) and `\\` escapes.
fn decode_string(s: &str) -> String {
    if !s.contains('\\') {
        return s.to_string();
    }
    let mut out = String::new();
    let c: Vec<char> = s.chars().collect();
    let mut i = 0;
    while i < c.len() {
        if c[i] == '\\' {
            let rest: String = c[i..].iter().take(4).collect();
            if rest.starts_with("\\\\") {
                out.push('\\');
                i += 2;
                continue;
            }
            if rest.starts_with("\\X2\\") {
                let mut j = i + 4;
                let mut units = Vec::new();
                while j + 4 <= c.len() && c[j] != '\\' {
                    let h: String = c[j..j + 4].iter().collect();
                    if let Ok(u) = u16::from_str_radix(&h, 16) {
                        units.push(u);
                    }
                    j += 4;
                }
                out.push_str(&String::from_utf16_lossy(&units));
                i = if c[j..].iter().take(4).collect::<String>() == "\\X0\\" { j + 4 } else { j };
                continue;
            }
            if rest.starts_with("\\X\\") && i + 5 <= c.len() {
                let h: String = c[i + 3..i + 5].iter().collect();
                if let Ok(b) = u8::from_str_radix(&h, 16) {
                    out.push(b as char);
                    i += 5;
                    continue;
                }
            }
        }
        out.push(c[i]);
        i += 1;
    }
    out
}

/// Parse the first DATA section of a Part 21 file.
pub fn parse(input: &str) -> Result<StepFile, ImportError> {
    let b = input.as_bytes();
    if !input.trim_start().starts_with("ISO-10303-21") {
        return Err(ImportError::new("parse", None, "not a STEP file (missing ISO-10303-21 header)"));
    }
    // locate `DATA;` outside strings/comments by lexing the header sections
    let mut l = Lexer { b, i: 0, depth: 0 };
    let mut found = false;
    while l.i < b.len() {
        l.ws();
        if l.i >= b.len() {
            break;
        }
        match b[l.i] {
            b'\'' => {
                l.string()?;
            }
            c if c.is_ascii_alphabetic() => {
                let w = l.ident().unwrap_or_default();
                if w == "DATA" {
                    l.ws();
                    if b.get(l.i) == Some(&b';') {
                        l.i += 1;
                        found = true;
                        break;
                    }
                }
            }
            _ => l.i += 1,
        }
    }
    if !found {
        return Err(ImportError::new("parse", None, "no DATA section found"));
    }
    let mut f = StepFile::default();
    loop {
        match l.peek() {
            None => return Err(ImportError::new("parse", None, "no ENDSEC after DATA")),
            Some(b'#') => {
                let id = match l.value()? {
                    Val::Ref(r) => r,
                    _ => unreachable!(),
                };
                if !l.eat(b'=') {
                    return Err(perr(&l, "expected '=' after entity id"));
                }
                let ent = if l.eat(b'(') {
                    let mut parts = Vec::new();
                    while l.peek() != Some(b')') {
                        parts.push(l.part()?);
                    }
                    l.i += 1;
                    Ent { parts }
                } else {
                    Ent { parts: vec![l.part()?] }
                };
                if !l.eat(b';') {
                    return Err(perr(&l, "expected ';' after entity"));
                }
                f.ents.insert(id, ent);
            }
            Some(_) => {
                let w = l.ident().ok_or_else(|| perr(&l, "unexpected character in DATA"))?;
                if w == "ENDSEC" {
                    return Ok(f);
                }
                return Err(perr(&l, &format!("unexpected {w} in DATA")));
            }
        }
    }
}

// ---- B-spline attribute helpers (knot expansion as brepkit `expand_knots`) ----

/// Most knots any one B-spline in a file may expand to. A knot vector is
/// `control points + degree + 1` long, so this also bounds the control net.
pub const MAX_KNOTS: usize = 1 << 22;

/// Expand `(mults)`, `(knot values)` into a flat knot vector.
pub fn expand_knots(mults: &[Val], vals: &[Val]) -> Option<Vec<f64>> {
    expand_knots_max(mults, vals, MAX_KNOTS)
}

/// [`expand_knots`] that gives up (None) rather than build more than `max`
/// knots: the multiplicities are file input and must not size an allocation.
pub fn expand_knots_max(mults: &[Val], vals: &[Val], max: usize) -> Option<Vec<f64>> {
    if mults.len() != vals.len() {
        return None;
    }
    let mut total = 0usize;
    for m in mults {
        total = total.checked_add(m.as_usize()?)?;
        if total > max {
            return None;
        }
    }
    let mut k = Vec::with_capacity(total);
    for (m, v) in mults.iter().zip(vals) {
        let x = v.as_f64()?;
        for _ in 0..m.as_usize()? {
            k.push(x);
        }
    }
    Some(k)
}

/// Default knot vector of a form-less B-spline (`QUASI_UNIFORM`, `UNIFORM`,
/// `BEZIER` entities carry no explicit knots): `n` control points, `deg`.
pub fn default_knots(kind: &str, n: usize, deg: usize) -> Vec<f64> {
    match kind {
        "UNIFORM_CURVE" | "UNIFORM_SURFACE" => (0..n + deg + 1).map(|i| i as f64).collect(),
        // BEZIER (n = deg+1) is the 2-knot case of the clamped uniform vector
        _ => {
            let interior = n.saturating_sub(deg + 1);
            let mut k = vec![0.0; deg + 1];
            for i in 1..=interior {
                k.push(i as f64);
            }
            k.extend(std::iter::repeat((interior + 1) as f64).take(deg + 1));
            k
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_simple_and_complex() {
        let f = parse(
            "ISO-10303-21;\nHEADER;FILE_NAME('a;b(',$);ENDSEC;\nDATA;\n/* c */ #1=CARTESIAN_POINT('p',(1.,-2.5E-1,3));\n#2=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));\n#3=X(LENGTH_MEASURE(5.),.T.,#1,$);\nENDSEC;\nEND-ISO-10303-21;",
        )
        .unwrap();
        assert_eq!(f.ents[&1].parts[0].args[1], Val::List(vec![Val::Real(1.0), Val::Real(-0.25), Val::Int(3)]));
        assert_eq!(f.ents[&2].parts.len(), 3);
        assert_eq!(f.ents[&2].part("SI_UNIT").unwrap().args[0], Val::Enum("MILLI".into()));
        assert_eq!(f.ents[&3].parts[0].args[0].as_f64(), Some(5.0));
    }

    #[test]
    fn errors_are_structured() {
        assert_eq!(parse("").unwrap_err().code, "parse");
        assert!(parse("ISO-10303-21;HEADER;ENDSEC;").is_err());
        assert!(parse("ISO-10303-21;DATA;#1=A(1,;ENDSEC;").is_err());
    }

    #[test]
    fn knots() {
        let m = vec![Val::Int(3), Val::Int(1)];
        let v = vec![Val::Real(0.0), Val::Real(1.0)];
        assert_eq!(expand_knots(&m, &v).unwrap(), vec![0.0, 0.0, 0.0, 1.0]);
        assert_eq!(default_knots("QUASI_UNIFORM_CURVE", 5, 3), vec![0., 0., 0., 0., 1., 2., 2., 2., 2.]);
    }
}
