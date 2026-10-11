//! Zero-dependency JSON DOM, parser and writer.
//!
//! The original kernel has no serde — this is the single JSON layer shared by
//! every crate (wire format, .ckpart files, config). Float output MUST be
//! shortest round-trip (Rust `{}`/ryu style) to match the oracle goldens
//! byte-for-byte.
//!
//! NOTE: port/merge of sectionprops-rs `src/json.rs` (proven, zero-dep,
//! insertion-ordered Map, NaN/Infinity tolerant). Numbers parse through f64.

/// Insertion-ordered string map (JSON objects preserve order).
pub type Map = Vec<(String, Json)>;

#[derive(Debug, Clone, PartialEq)]
pub enum Json {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Map),
}

impl Json {
    pub fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(m) => m.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Json::Num(n) => Some(*n),
            _ => None,
        }
    }
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Json::Str(s) => Some(s),
            _ => None,
        }
    }
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Json::Bool(b) => Some(*b),
            _ => None,
        }
    }
    pub fn as_arr(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }
}

pub fn parse(input: &str) -> Result<Json, ParseError> {
    let bytes = input.as_bytes();
    let mut p = Parser { b: bytes, i: 0 };
    p.skip_ws();
    let v = p.value()?;
    p.skip_ws();
    if p.i != bytes.len() {
        return Err(p.err("trailing data"));
    }
    Ok(v)
}

#[derive(Debug)]
pub struct ParseError {
    pub pos: usize,
    pub msg: String,
}

struct Parser<'a> {
    b: &'a [u8],
    i: usize,
}

impl<'a> Parser<'a> {
    fn err(&self, msg: &str) -> ParseError {
        ParseError { pos: self.i, msg: msg.to_string() }
    }
    fn skip_ws(&mut self) {
        while self.i < self.b.len() && matches!(self.b[self.i], b' ' | b'\t' | b'\n' | b'\r') {
            self.i += 1;
        }
    }
    fn peek(&self) -> Option<u8> {
        self.b.get(self.i).copied()
    }
    fn expect(&mut self, c: u8) -> Result<(), ParseError> {
        if self.peek() == Some(c) {
            self.i += 1;
            Ok(())
        } else {
            Err(self.err(&format!("expected '{}'", c as char)))
        }
    }
    fn value(&mut self) -> Result<Json, ParseError> {
        match self.peek() {
            Some(b'{') => self.object(),
            Some(b'[') => self.array(),
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => self.lit("true", Json::Bool(true)),
            Some(b'f') => self.lit("false", Json::Bool(false)),
            Some(b'n') => self.lit("null", Json::Null),
            Some(_) => self.number(),
            None => Err(self.err("unexpected end")),
        }
    }
    fn lit(&mut self, s: &str, v: Json) -> Result<Json, ParseError> {
        if self.b[self.i..].starts_with(s.as_bytes()) {
            self.i += s.len();
            Ok(v)
        } else {
            Err(self.err("bad literal"))
        }
    }
    fn object(&mut self) -> Result<Json, ParseError> {
        self.expect(b'{')?;
        let mut m = Map::new();
        self.skip_ws();
        if self.peek() == Some(b'}') {
            self.i += 1;
            return Ok(Json::Obj(m));
        }
        loop {
            self.skip_ws();
            let k = self.string()?;
            self.skip_ws();
            self.expect(b':')?;
            self.skip_ws();
            let v = self.value()?;
            m.push((k, v));
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b'}') => {
                    self.i += 1;
                    return Ok(Json::Obj(m));
                }
                _ => return Err(self.err("expected ',' or '}'")),
            }
        }
    }
    fn array(&mut self) -> Result<Json, ParseError> {
        self.expect(b'[')?;
        let mut a = Vec::new();
        self.skip_ws();
        if self.peek() == Some(b']') {
            self.i += 1;
            return Ok(Json::Arr(a));
        }
        loop {
            self.skip_ws();
            a.push(self.value()?);
            self.skip_ws();
            match self.peek() {
                Some(b',') => self.i += 1,
                Some(b']') => {
                    self.i += 1;
                    return Ok(Json::Arr(a));
                }
                _ => return Err(self.err("expected ',' or ']'")),
            }
        }
    }
    fn string(&mut self) -> Result<String, ParseError> {
        self.expect(b'"')?;
        let mut s = String::new();
        loop {
            match self.peek() {
                None => return Err(self.err("unterminated string")),
                Some(b'"') => {
                    self.i += 1;
                    return Ok(s);
                }
                Some(b'\\') => {
                    self.i += 1;
                    let e = self.peek().ok_or_else(|| self.err("bad escape"))?;
                    self.i += 1;
                    match e {
                        b'"' => s.push('"'),
                        b'\\' => s.push('\\'),
                        b'/' => s.push('/'),
                        b'b' => s.push('\u{8}'),
                        b'f' => s.push('\u{c}'),
                        b'n' => s.push('\n'),
                        b'r' => s.push('\r'),
                        b't' => s.push('\t'),
                        b'u' => {
                            let cp = self.hex4()?;
                            // surrogate pair
                            if (0xD800..=0xDBFF).contains(&cp) {
                                if self.peek() == Some(b'\\') {
                                    self.i += 2; // \u
                                    let lo = self.hex4()?;
                                    let c = 0x10000 + ((cp - 0xD800) << 10) + (lo - 0xDC00);
                                    s.push(char::from_u32(c).unwrap_or('\u{fffd}'));
                                } else {
                                    s.push('\u{fffd}');
                                }
                            } else {
                                s.push(char::from_u32(cp).unwrap_or('\u{fffd}'));
                            }
                        }
                        _ => return Err(self.err("bad escape")),
                    }
                }
                Some(_) => {
                    // consume one UTF-8 sequence
                    let start = self.i;
                    self.i += 1;
                    while self.i < self.b.len() && (self.b[self.i] & 0xC0) == 0x80 {
                        self.i += 1;
                    }
                    s.push_str(std::str::from_utf8(&self.b[start..self.i]).unwrap_or("\u{fffd}"));
                }
            }
        }
    }
    fn hex4(&mut self) -> Result<u32, ParseError> {
        let bytes = self.b.get(self.i..self.i + 4).ok_or_else(|| self.err("bad \\u"))?;
        let s = std::str::from_utf8(bytes).map_err(|_| self.err("bad \\u"))?;
        let v = u32::from_str_radix(s, 16).map_err(|_| self.err("bad \\u"))?;
        self.i += 4;
        Ok(v)
    }
    fn number(&mut self) -> Result<Json, ParseError> {
        let start = self.i;
        if self.peek() == Some(b'-') {
            self.i += 1;
        }
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() || c == b'.' || c == b'e' || c == b'E' || c == b'+' || c == b'-' {
                self.i += 1;
            } else {
                break;
            }
        }
        let s = std::str::from_utf8(&self.b[start..self.i]).map_err(|_| self.err("bad number"))?;
        s.parse::<f64>().map(Json::Num).map_err(|_| self.err("bad number"))
    }
}

/// Shortest round-trip float formatting (matches Rust `{}` for f64 and the
/// oracle goldens). Integers print without a fraction.
pub fn fmt_f64(v: f64) -> String {
    if v == v.trunc() && v.abs() < 1e15 {
        format!("{}", v as i64)
    } else {
        format!("{}", v)
    }
}

pub fn to_string(v: &Json) -> String {
    let mut s = String::new();
    write_json(v, &mut s);
    s
}

fn write_json(v: &Json, out: &mut String) {
    match v {
        Json::Null => out.push_str("null"),
        Json::Bool(true) => out.push_str("true"),
        Json::Bool(false) => out.push_str("false"),
        Json::Num(n) => {
            if n.is_finite() {
                out.push_str(&fmt_f64(*n))
            } else {
                out.push_str("null")
            }
        }
        Json::Str(s) => write_str(s, out),
        Json::Arr(a) => {
            out.push('[');
            for (i, x) in a.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_json(x, out);
            }
            out.push(']');
        }
        Json::Obj(m) => {
            out.push('{');
            for (i, (k, x)) in m.iter().enumerate() {
                if i > 0 {
                    out.push(',');
                }
                write_str(k, out);
                out.push(':');
                write_json(x, out);
            }
            out.push('}');
        }
    }
}

fn write_str(s: &str, out: &mut String) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn roundtrip() {
        let v = parse(r#"{"a":[1,2.5,-3],"b":"x\ny","c":null,"d":true}"#).unwrap();
        assert_eq!(v.get("b").unwrap().as_str(), Some("x\ny"));
        let s = to_string(&v);
        assert_eq!(parse(&s).unwrap(), v);
        assert!(s.contains("2.5"));
        assert!(s.contains("-3"));
    }
    #[test]
    fn float_format() {
        assert_eq!(fmt_f64(0.5), "0.5");
        assert_eq!(fmt_f64(20.0), "20");
        assert_eq!(fmt_f64(-1e-16), "-0.0000000000000001");
    }
}
