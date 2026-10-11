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
    /// Object entries (insertion order, duplicates kept).
    pub fn as_obj(&self) -> Option<&Map> {
        match self {
            Json::Obj(m) => Some(m),
            _ => None,
        }
    }
    pub fn as_obj_mut(&mut self) -> Option<&mut Map> {
        match self {
            Json::Obj(m) => Some(m),
            _ => None,
        }
    }
    pub fn as_arr_mut(&mut self) -> Option<&mut Vec<Json>> {
        match self {
            Json::Arr(a) => Some(a),
            _ => None,
        }
    }
    /// Mutable lookup of the FIRST entry with `key`.
    pub fn get_mut(&mut self, key: &str) -> Option<&mut Json> {
        match self {
            Json::Obj(m) => m.iter_mut().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }
    /// Number as i64 when it is integral and in range.
    pub fn as_i64(&self) -> Option<i64> {
        let n = self.as_f64()?;
        (n.is_finite() && n == n.trunc() && n.abs() < 9.2e18).then(|| n as i64)
    }
    /// Number as usize when it is a non-negative integer.
    pub fn as_usize(&self) -> Option<usize> {
        let n = self.as_i64()?;
        (n >= 0).then(|| n as usize)
    }
    /// Array element.
    pub fn at(&self, i: usize) -> Option<&Json> {
        self.as_arr().and_then(|a| a.get(i))
    }
    /// `[x, y, z]` numeric triple.
    pub fn as_vec3(&self) -> Option<[f64; 3]> {
        let a = self.as_arr()?;
        if a.len() != 3 {
            return None;
        }
        Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
    }
    /// Array of numbers.
    pub fn as_f64_vec(&self) -> Option<Vec<f64>> {
        self.as_arr()?.iter().map(|v| v.as_f64()).collect()
    }
    /// Lookup by a path of object keys: `v.path(&["msg", "type"])`.
    pub fn path(&self, keys: &[&str]) -> Option<&Json> {
        keys.iter().try_fold(self, |v, k| v.get(k))
    }
    /// `get(key).and_then(as_str)`
    pub fn get_str(&self, key: &str) -> Option<&str> {
        self.get(key).and_then(Json::as_str)
    }
    /// `get(key).and_then(as_f64)`
    pub fn get_f64(&self, key: &str) -> Option<f64> {
        self.get(key).and_then(Json::as_f64)
    }
    /// `get(key).and_then(as_bool)`
    pub fn get_bool(&self, key: &str) -> Option<bool> {
        self.get(key).and_then(Json::as_bool)
    }
    /// `get(key).and_then(as_arr)`
    pub fn get_arr(&self, key: &str) -> Option<&[Json]> {
        self.get(key).and_then(Json::as_arr)
    }
    pub fn is_null(&self) -> bool {
        matches!(self, Json::Null)
    }
    pub fn is_obj(&self) -> bool {
        matches!(self, Json::Obj(_))
    }
    /// Empty object `{}`.
    pub fn obj() -> Json {
        Json::Obj(Map::new())
    }
    /// Append `key: value` to an object (no-op on non-objects); chainable.
    pub fn with(mut self, key: &str, value: impl Into<Json>) -> Json {
        if let Json::Obj(m) = &mut self {
            m.push((key.to_string(), value.into()));
        }
        self
    }
    /// Append `key: value` to an object in place (no dedup — like the
    /// original writer, which streams keys in code order).
    pub fn push(&mut self, key: &str, value: impl Into<Json>) {
        if let Json::Obj(m) = self {
            m.push((key.to_string(), value.into()));
        }
    }
    /// Replace the first `key` or append it.
    pub fn set(&mut self, key: &str, value: impl Into<Json>) {
        if let Json::Obj(m) = self {
            let v = value.into();
            match m.iter_mut().find(|(k, _)| k == key) {
                Some(slot) => slot.1 = v,
                None => m.push((key.to_string(), v)),
            }
        }
    }
}

/// Ordered object builder: `Obj::new().put("a", 1).put("b", "x").build()`.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Obj(pub Map);

impl Obj {
    pub fn new() -> Obj {
        Obj(Map::new())
    }
    pub fn put(mut self, key: &str, value: impl Into<Json>) -> Obj {
        self.0.push((key.to_string(), value.into()));
        self
    }
    /// Put only when `value` is `Some`.
    pub fn put_opt<T: Into<Json>>(self, key: &str, value: Option<T>) -> Obj {
        match value {
            Some(v) => self.put(key, v),
            None => self,
        }
    }
    pub fn build(self) -> Json {
        Json::Obj(self.0)
    }
}

impl From<Obj> for Json {
    fn from(o: Obj) -> Json {
        Json::Obj(o.0)
    }
}
impl From<f64> for Json {
    fn from(v: f64) -> Json {
        Json::Num(v)
    }
}
impl From<f32> for Json {
    fn from(v: f32) -> Json {
        Json::Num(v as f64)
    }
}
macro_rules! from_int {
    ($($t:ty),*) => {$(
        impl From<$t> for Json {
            fn from(v: $t) -> Json { Json::Num(v as f64) }
        }
    )*};
}
from_int!(i8, i16, i32, i64, isize, u8, u16, u32, u64, usize);
impl From<bool> for Json {
    fn from(v: bool) -> Json {
        Json::Bool(v)
    }
}
impl From<&str> for Json {
    fn from(v: &str) -> Json {
        Json::Str(v.to_string())
    }
}
impl From<String> for Json {
    fn from(v: String) -> Json {
        Json::Str(v)
    }
}
impl From<&String> for Json {
    fn from(v: &String) -> Json {
        Json::Str(v.clone())
    }
}
impl<T: Into<Json>> From<Vec<T>> for Json {
    fn from(v: Vec<T>) -> Json {
        Json::Arr(v.into_iter().map(Into::into).collect())
    }
}
impl<T: Into<Json> + Clone, const N: usize> From<[T; N]> for Json {
    fn from(v: [T; N]) -> Json {
        Json::Arr(v.into_iter().map(Into::into).collect())
    }
}
impl<T: Into<Json>> From<Option<T>> for Json {
    fn from(v: Option<T>) -> Json {
        v.map_or(Json::Null, Into::into)
    }
}

impl std::fmt::Display for Json {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&to_string(self))
    }
}

/// Maximum container nesting the original parser accepts.
///
/// Pinned by oracle bisection: a body whose deepest array/object sits at
/// nesting level 512 parses; 513 is rejected ("bad request: the body must be
/// JSON {id, msg}"). Measured both for top-level `[[..]]` and for arrays /
/// objects nested inside `{"msg":{"a":...}}`.
pub const MAX_DEPTH: usize = 512;

/// Parse a JSON document (strict RFC 8259 grammar with the original kernel's
/// quirks, all pinned by oracle probes):
///
/// - whitespace is only ` \t\n\r` (form feed and a UTF-8 BOM are rejected);
/// - numbers: `-? digit+ ( . digit* )? ( [eE] [+-]? digit+ )?` — leading zeros
///   (`01`) and a bare trailing dot (`1.`, `-01.e5`) are ACCEPTED; `.5`, `+1`,
///   `-`, `1e`, `1e+`, `0x1`, `NaN`, `Infinity` are rejected; overflow
///   (`1e999`) parses to infinity;
/// - strings: raw control characters (tab, U+0001...) are accepted unescaped;
///   unknown escapes (`\x`) and bad `\u` hex are rejected; unpaired
///   surrogates decode to U+FFFD and the following character is kept
///   (`"\ud800A"` -> `"\u{fffd}A"`);
/// - trailing commas, comments, single quotes and trailing data are rejected;
/// - duplicate keys are kept in order (lookups via [`Json::get`] see the first).
pub fn parse(input: &str) -> Result<Json, ParseError> {
    parse_bytes(input.as_bytes())
}

/// Parse raw bytes. Invalid UTF-8 inside strings decodes lossily to U+FFFD
/// (oracle: a type `"p\xffg"` is echoed as `"p\u{fffd}g"`).
pub fn parse_bytes(bytes: &[u8]) -> Result<Json, ParseError> {
    let mut p = Parser { b: bytes, i: 0, depth: 0 };
    p.skip_ws();
    let v = p.value()?;
    p.skip_ws();
    if p.i != bytes.len() {
        return Err(p.err("trailing data"));
    }
    Ok(v)
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub pos: usize,
    pub msg: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} at byte {}", self.msg, self.pos)
    }
}

impl std::error::Error for ParseError {}

struct Parser<'a> {
    b: &'a [u8],
    i: usize,
    depth: usize,
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
            Some(b'{') => self.nested(Self::object),
            Some(b'[') => self.nested(Self::array),
            Some(b'"') => Ok(Json::Str(self.string()?)),
            Some(b't') => self.lit("true", Json::Bool(true)),
            Some(b'f') => self.lit("false", Json::Bool(false)),
            Some(b'n') => self.lit("null", Json::Null),
            Some(b'-' | b'0'..=b'9') => self.number(),
            Some(_) => Err(self.err("unexpected character")),
            None => Err(self.err("unexpected end")),
        }
    }
    fn nested(&mut self, f: fn(&mut Self) -> Result<Json, ParseError>) -> Result<Json, ParseError> {
        self.depth += 1;
        if self.depth > MAX_DEPTH {
            return Err(self.err("too deeply nested"));
        }
        let r = f(self);
        self.depth -= 1;
        r
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
        let mut raw: Vec<u8> = Vec::new();
        loop {
            match self.peek() {
                None => return Err(self.err("unterminated string")),
                Some(b'"') => {
                    self.i += 1;
                    return Ok(String::from_utf8_lossy(&raw).into_owned());
                }
                Some(b'\\') => {
                    self.i += 1;
                    let e = self.peek().ok_or_else(|| self.err("bad escape"))?;
                    self.i += 1;
                    let c = match e {
                        b'"' => '"',
                        b'\\' => '\\',
                        b'/' => '/',
                        b'b' => '\u{8}',
                        b'f' => '\u{c}',
                        b'n' => '\n',
                        b'r' => '\r',
                        b't' => '\t',
                        b'u' => {
                            let cp = self.hex4()?;
                            if (0xD800..=0xDBFF).contains(&cp) {
                                // Oracle quirk: a high surrogate followed by
                                // ANY `\uXXXX` combines with it by masking
                                // (`\ud800\u0041` -> U+10041, `\ud800\ud800`
                                // -> U+10000); bad hex there is a parse
                                // error. Anything else leaves it unpaired
                                // (U+FFFD) and is parsed normally after.
                                if self.b.get(self.i..self.i + 2) == Some(b"\\u") {
                                    self.i += 2;
                                    let lo = self.hex4()?;
                                    let c = 0x10000 + ((cp & 0x3FF) << 10) + (lo & 0x3FF);
                                    char::from_u32(c).unwrap_or('\u{fffd}')
                                } else {
                                    '\u{fffd}'
                                }
                            } else {
                                char::from_u32(cp).unwrap_or('\u{fffd}')
                            }
                        }
                        _ => return Err(self.err("bad escape")),
                    };
                    let mut buf = [0u8; 4];
                    raw.extend_from_slice(c.encode_utf8(&mut buf).as_bytes());
                }
                Some(c) => {
                    raw.push(c);
                    self.i += 1;
                }
            }
        }
    }
    fn hex4(&mut self) -> Result<u32, ParseError> {
        let bytes = self.b.get(self.i..self.i + 4).ok_or_else(|| self.err("bad \\u"))?;
        if !bytes.iter().all(|c| c.is_ascii_hexdigit()) {
            return Err(self.err("bad \\u"));
        }
        let s = std::str::from_utf8(bytes).map_err(|_| self.err("bad \\u"))?;
        let v = u32::from_str_radix(s, 16).map_err(|_| self.err("bad \\u"))?;
        self.i += 4;
        Ok(v)
    }
    fn digits(&mut self) -> usize {
        let s = self.i;
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.i += 1;
        }
        self.i - s
    }
    fn number(&mut self) -> Result<Json, ParseError> {
        let start = self.i;
        if self.peek() == Some(b'-') {
            self.i += 1;
        }
        if self.digits() == 0 {
            return Err(self.err("bad number"));
        }
        if self.peek() == Some(b'.') {
            self.i += 1;
            self.digits(); // oracle accepts `1.` (zero fraction digits)
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.i += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.i += 1;
            }
            if self.digits() == 0 {
                return Err(self.err("bad number"));
            }
        }
        let s = std::str::from_utf8(&self.b[start..self.i]).map_err(|_| self.err("bad number"))?;
        s.parse::<f64>().map(Json::Num).map_err(|_| self.err("bad number"))
    }
}

/// Float formatting exactly as the original writer (pinned by echoing numbers
/// through sketch points in a regen reply):
///
/// - Rust `{}` Display for f64: shortest round-trip digits, NEVER exponent
///   form (`1e-17` -> `0.00000000000000001`, `1e21` ->
///   `1000000000000000000000`, `1e300` -> 301 digits), integral values
///   without `.0`;
/// - negative zero prints as `0` (oracle: `-0`, `-0.0` -> `0`);
/// - non-finite values are written as `null` by [`to_string`].
pub fn fmt_f64(v: f64) -> String {
    if v == 0.0 {
        "0".to_string()
    } else {
        format!("{}", v)
    }
}

pub fn to_string(v: &Json) -> String {
    let mut s = String::new();
    write_json(v, &mut s);
    s
}

/// Append the serialization of `v` to `out` (compact, no whitespace).
pub fn write_to(v: &Json, out: &mut String) {
    write_json(v, out)
}

/// Append `s` as a quoted JSON string with the original escaping rules.
pub fn write_string(s: &str, out: &mut String) {
    write_str(s, out)
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

/// String escaping, pinned by echoing through "unknown request <type>":
/// only `"` `\\` `\n` `\r` `\t` get short escapes; every other C0 control
/// is `\u00XX` with lowercase hex (`\b` -> `\u0008`, `\f` -> `\u000c`);
/// `/`, DEL (U+007F), U+2028 and all non-ASCII are written raw.
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

    /// Every pair echoed by the oracle through sketch-point x (raw reply).
    #[test]
    fn float_format_oracle() {
        let cases: &[(f64, &str)] = &[
            (0.0, "0"),
            (-0.0, "0"),
            (1.0, "1"),
            (-1.0, "-1"),
            (-0.5, "-0.5"),
            (0.1, "0.1"),
            (3.0000000000000004, "3.0000000000000004"),
            (1e15, "1000000000000000"),
            (1e16, "10000000000000000"),
            (9007199254740993.0, "9007199254740992"),
            (1e21, "1000000000000000000000"),
            (-1e21, "-1000000000000000000000"),
            (1e-5, "0.00001"),
            (1e-7, "0.0000001"),
            (1e-17, "0.00000000000000001"),
            (-1e-17, "-0.00000000000000001"),
            (2.5e-8, "0.000000025"),
            (12345678901234567890.0, "12345678901234567000"),
            (0.30000000000000004, "0.30000000000000004"),
            (4.35, "4.35"),
        ];
        for &(v, want) in cases {
            assert_eq!(fmt_f64(v), want, "{v:e}");
        }
        assert_eq!(fmt_f64(1.5e-300).len(), 303);
        assert!(fmt_f64(5e-324).ends_with("0005"));
        assert_eq!(fmt_f64(1.7976931348623157e308).len(), 309);
        // non-finite -> null (oracle: "extra":1e999 echoes as null)
        assert_eq!(to_string(&Json::Num(f64::INFINITY)), "null");
        assert_eq!(to_string(&Json::Num(f64::NAN)), "null");
    }

    #[test]
    fn parse_strictness_oracle() {
        let ok = |s: &str| parse(s).is_ok();
        // accepted by the oracle
        for s in ["01", "1.", "-01.e5", "1E5", "1e999", " {} ", "{\"a\":1}\n\t\r", "\"\t\u{1}\""] {
            assert!(ok(s), "{s:?} should parse");
        }
        assert_eq!(parse("1e999").unwrap(), Json::Num(f64::INFINITY));
        // rejected by the oracle
        for s in [
            "", " ", "x", ".5", "+1", "-", "1e", "1e+", "0x1", "NaN", "Infinity", "-Infinity", "tru",
            "[1,]", "{\"a\":1,}", "{}/*c*/", "{'a':1}", "{}{}", "{}x", "\"\\x\"", "\"\\u00zz\"",
            "\u{feff}{}", "\u{c}{}",
        ] {
            assert!(!ok(s), "{s:?} should fail");
        }
    }

    #[test]
    fn depth_limit_oracle() {
        let nest = |n: usize| format!("{}{}", "[".repeat(n), "]".repeat(n));
        assert!(parse(&nest(512)).is_ok());
        assert!(parse(&nest(513)).is_err());
    }

    #[test]
    fn surrogates_oracle() {
        let p = |s: &str| parse(s).unwrap().as_str().unwrap().to_string();
        assert_eq!(p(r#""a\ud800\u0041b""#), "a\u{10041}b");
        assert_eq!(p(r#""a\ud800\ud800b""#), "a\u{10000}b");
        assert_eq!(p(r#""a\udbff\uffffb""#), "a\u{10ffff}b");
        assert_eq!(p(r#""a\ud800\b""#), "a\u{fffd}\u{8}");
        assert!(parse(r#""\ud800\uzzzz""#).is_err());
        assert_eq!(p(r#""a\ud800xb""#), "a\u{fffd}xb");
        assert_eq!(p(r#""a\udc00\ud800b""#), "a\u{fffd}\u{fffd}b");
        assert_eq!(p(r#""a\uD83D\uDE00""#), "a\u{1F600}");
        assert_eq!(p(r#""a\ud800\n""#), "a\u{fffd}\n");
        assert_eq!(parse_bytes(b"\"p\xffg\"").unwrap().as_str(), Some("p\u{fffd}g"));
    }

    #[test]
    fn escape_oracle() {
        // oracle: "unknown request bogus\u0008\u000c\n\r\t/\u{7f}\u{2028}\u001f😀"
        let s = "\u{8}\u{c}\n\r\t/\u{7f}\u{2028}\u{1f}\u{1F600}\"\\\u{0}";
        assert_eq!(
            to_string(&Json::Str(s.into())),
            "\"\\u0008\\u000c\\n\\r\\t/\u{7f}\u{2028}\\u001f\u{1F600}\\\"\\\\\\u0000\""
        );
    }

    #[test]
    fn duplicate_keys_first_wins() {
        // oracle: {"msg":{ping},"msg":{bogus}} answers pong; duplicates are
        // echoed back in order.
        let v = parse(r#"{"q":5,"id":"p","q":1}"#).unwrap();
        assert_eq!(v.get("q").unwrap().as_f64(), Some(5.0));
        assert_eq!(to_string(&v), r#"{"q":5,"id":"p","q":1}"#);
    }

    #[test]
    fn builder() {
        let v = Obj::new().put("ok", true).put("n", 2u32).put_opt("x", None::<f64>).put("a", vec![1.5, 2.0]).build();
        assert_eq!(to_string(&v), r#"{"ok":true,"n":2,"a":[1.5,2]}"#);
        let mut w = Json::obj().with("a", "s");
        w.set("a", Json::Null);
        w.push("b", [1, 2, 3]);
        assert_eq!(w.to_string(), r#"{"a":null,"b":[1,2,3]}"#);
        assert_eq!(w.get("b").unwrap().as_vec3(), Some([1.0, 2.0, 3.0]));
        assert_eq!(parse(r#"{"m":{"t":"x"}}"#).unwrap().path(&["m", "t"]).and_then(Json::as_str), Some("x"));
        assert_eq!(Json::Num(3.0).as_usize(), Some(3));
        assert_eq!(Json::Num(-3.5).as_i64(), None);
    }
}
