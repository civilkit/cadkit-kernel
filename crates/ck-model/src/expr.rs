//! Expression evaluator for dimension values and numeric feature fields.
//!
//! Grammar (identical to the client's `al()` in app.pretty.js ~l.389, which
//! the kernel mirrors — every oracle probe below agrees with it):
//!
//! ```text
//! expr   := term (('+'|'-') term)*
//! term   := power (('*'|'/') power)*
//! power  := unary ('^' power)?            // right associative
//! unary  := '(' expr ')' | '-' power | '+' power
//!         | ident                         // variable (exact case), `pi`,
//!         | func '(' expr ')'             //   or sqrt/sin/cos/tan/abs (any case)
//!         | number unit?
//! number := /(\d+\.?\d*|\.\d+)(e[+-]?\d+)?/i
//! unit   := mm | cm | m | in | deg | °     // case-insensitive, scale to mm / deg
//! ```
//!
//! A leading `=` is stripped, `,` is a decimal separator, trig is in degrees,
//! and a non-finite result is an error (`=1e400` -> badExpression).
//!
//! Wire behaviour pinned with the oracle (extrude `depth` field):
//! `"=w*2"` (w=30) -> 60, `"=2^3^2"` -> 512, `"=-2^2"` -> -4, `"=cos(60)*10"`
//! -> 5.000000000000001, `"=1,5"` -> 1.5, `"=3 m"` -> 3000, `"=W"` / `"=abc"`
//! / `"=7)"` / `"=sqrt 4"` -> error `badExpression`; non-`=` strings are plain
//! numbers: `" 7 "` -> 7, `"2*3"` / `"abc"` -> 0 (no error).
//!
//! Global variables (`part.variables = [{name, expr}]`) resolve like the
//! client's `zs()`: invalid/reserved/duplicate names -> `badName`, a failing
//! expression -> `badExpression`, a reference cycle -> `cyclicVariables`
//! (all three strings are adjacent in kernel.strings).

use ck_json::Json;

/// Error code strings (kernel.strings: "badExpression", "badNamecyclicVariables").
pub const BAD_EXPRESSION: &str = "badExpression";
pub const BAD_NAME: &str = "badName";
pub const CYCLIC_VARIABLES: &str = "cyclicVariables";

/// Resolved variable values, insertion ordered.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Vars {
    pub values: Vec<(String, f64)>,
}

impl Vars {
    pub fn new() -> Self {
        Vars::default()
    }
    pub fn get(&self, name: &str) -> Option<f64> {
        self.values.iter().find(|(n, _)| n == name).map(|(_, v)| *v)
    }
    pub fn has(&self, name: &str) -> bool {
        self.values.iter().any(|(n, _)| n == name)
    }
    pub fn set(&mut self, name: &str, v: f64) {
        if let Some(e) = self.values.iter_mut().find(|(n, _)| n == name) {
            e.1 = v;
        } else {
            self.values.push((name.to_string(), v));
        }
    }
}

/// A global variable definition (`{"name":"D1","expr":"10"}`).
#[derive(Debug, Clone, PartialEq)]
pub struct Variable {
    pub name: String,
    pub expr: String,
}

/// Result of resolving `part.variables`.
#[derive(Debug, Clone, Default)]
pub struct VarResolution {
    pub values: Vars,
    /// (variable name, error code)
    pub errors: Vec<(String, &'static str)>,
}

const FUNCS: [&str; 5] = ["sqrt", "sin", "cos", "tan", "abs"];
const UNITS: [(&str, f64); 6] = [("mm", 1.0), ("cm", 10.0), ("m", 1e3), ("in", 25.4), ("deg", 1.0), ("\u{b0}", 1.0)];

fn call(f: &str, x: f64) -> f64 {
    let rad = x * std::f64::consts::PI / 180.0;
    match f {
        "sqrt" => x.sqrt(),
        "sin" => rad.sin(),
        "cos" => rad.cos(),
        "tan" => rad.tan(),
        _ => x.abs(),
    }
}

struct P<'a> {
    t: Vec<char>,
    n: usize,
    vars: Option<&'a Vars>,
}

type R = Result<f64, ()>;

fn is_ident_start(c: char) -> bool {
    c.is_ascii_alphabetic() || c == '_'
}
fn is_ident(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

impl<'a> P<'a> {
    fn peek(&self) -> Option<char> {
        self.t.get(self.n).copied()
    }
    fn ws(&mut self) {
        while self.n < self.t.len() && self.t[self.n].is_whitespace() {
            self.n += 1;
        }
    }
    /// `/^(\d+\.?\d*|\.\d+)(e[+-]?\d+)?/i` + optional unit.
    fn number(&mut self) -> Option<f64> {
        self.ws();
        let t = &self.t;
        let s = self.n;
        let mut i = s;
        let digit = |i: usize| i < t.len() && t[i].is_ascii_digit();
        if digit(i) {
            while digit(i) {
                i += 1;
            }
            if i < t.len() && t[i] == '.' {
                i += 1;
                while digit(i) {
                    i += 1;
                }
            }
        } else if i < t.len() && t[i] == '.' && digit(i + 1) {
            i += 1;
            while digit(i) {
                i += 1;
            }
        } else {
            return None;
        }
        // exponent
        if i < t.len() && (t[i] == 'e' || t[i] == 'E') {
            let mut j = i + 1;
            if j < t.len() && (t[j] == '+' || t[j] == '-') {
                j += 1;
            }
            if digit(j) {
                while digit(j) {
                    j += 1;
                }
                i = j;
            }
        }
        let lit: String = t[s..i].iter().collect();
        self.n = i;
        let mut v: f64 = parse_float_prefix(&lit);
        self.ws();
        // unit: (mm|cm|m|in|deg|°)(?![a-z])  — alternation order matters.
        for (u, k) in UNITS {
            let uc: Vec<char> = u.chars().collect();
            let end = self.n + uc.len();
            if end <= self.t.len()
                && self.t[self.n..end].iter().zip(uc.iter()).all(|(a, b)| a.to_ascii_lowercase() == *b)
                && !self.t.get(end).map_or(false, |c| c.is_ascii_alphabetic())
            {
                v *= k;
                self.n = end;
                break;
            }
        }
        Some(v)
    }
    fn unary(&mut self) -> R {
        self.ws();
        match self.peek() {
            Some('(') => {
                self.n += 1;
                let v = self.expr()?;
                self.ws();
                if self.peek() != Some(')') {
                    return Err(());
                }
                self.n += 1;
                return Ok(v);
            }
            Some('-') => {
                self.n += 1;
                return Ok(-self.power()?);
            }
            Some('+') => {
                self.n += 1;
                return self.power();
            }
            _ => {}
        }
        if self.peek().map_or(false, is_ident_start) {
            let s = self.n;
            while self.peek().map_or(false, is_ident) {
                self.n += 1;
            }
            let id: String = self.t[s..self.n].iter().collect();
            if let Some(v) = self.vars.and_then(|m| m.get(&id)) {
                return Ok(v);
            }
            let low = id.to_ascii_lowercase();
            if low == "pi" {
                return Ok(std::f64::consts::PI);
            }
            let f = FUNCS.iter().find(|f| **f == low).ok_or(())?;
            self.ws();
            if self.peek() != Some('(') {
                return Err(());
            }
            let arg = self.unary()?;
            return Ok(call(f, arg));
        }
        self.number().ok_or(())
    }
    fn power(&mut self) -> R {
        let m = self.unary()?;
        self.ws();
        if self.peek() == Some('^') {
            self.n += 1;
            let e = self.power()?;
            return Ok(m.powf(e));
        }
        Ok(m)
    }
    fn term(&mut self) -> R {
        let mut m = self.power()?;
        loop {
            self.ws();
            match self.peek() {
                Some('*') => {
                    self.n += 1;
                    m *= self.power()?;
                }
                Some('/') => {
                    self.n += 1;
                    m /= self.power()?;
                }
                _ => return Ok(m),
            }
        }
    }
    fn expr(&mut self) -> R {
        let mut m = self.term()?;
        loop {
            self.ws();
            match self.peek() {
                Some('+') => {
                    self.n += 1;
                    m += self.term()?;
                }
                Some('-') => {
                    self.n += 1;
                    m -= self.term()?;
                }
                _ => return Ok(m),
            }
        }
    }
}

/// JS `parseFloat` on a literal already matched by the number regex.
fn parse_float_prefix(s: &str) -> f64 {
    s.parse::<f64>().unwrap_or_else(|_| {
        // "5." style literals: Rust accepts them; "1e" can't reach here.
        s.trim_end_matches('.').parse().unwrap_or(0.0)
    })
}

/// Evaluate an expression string. `Err(())` on syntax error, unknown
/// identifier or a non-finite result (client `al()` throws "syntax").
pub fn eval(src: &str, vars: Option<&Vars>) -> Result<f64, ()> {
    let s = src.trim();
    let s = s.strip_prefix('=').unwrap_or(s).replace(',', ".");
    let mut p = P { t: s.chars().collect(), n: 0, vars };
    let v = p.expr()?;
    p.ws();
    if p.n < p.t.len() || !v.is_finite() {
        return Err(());
    }
    Ok(v)
}

/// Identifiers mentioned in an expression (`ll()`/regex `[a-z_][a-z0-9_]*`gi).
pub fn identifiers(src: &str) -> Vec<String> {
    let c: Vec<char> = src.chars().collect();
    let mut out = Vec::new();
    let mut i = 0;
    while i < c.len() {
        if is_ident_start(c[i]) {
            let s = i;
            while i < c.len() && is_ident(c[i]) {
                i += 1;
            }
            out.push(c[s..i].iter().collect());
        } else {
            i += 1;
        }
    }
    out
}

fn reserved(name_lower: &str) -> bool {
    name_lower == "pi" || FUNCS.contains(&name_lower) || UNITS.iter().any(|(u, _)| *u == name_lower)
}

fn valid_name(n: &str) -> bool {
    let mut it = n.chars();
    matches!(it.next(), Some(c) if is_ident_start(c)) && it.all(is_ident)
}

/// Resolve the global variable table (client `zs()`).
pub fn resolve_variables(vars: &[Variable]) -> VarResolution {
    let mut res = VarResolution::default();
    let mut pending: Vec<&Variable> = Vec::new();
    for v in vars {
        if !valid_name(&v.name) || reserved(&v.name.to_ascii_lowercase()) || pending.iter().any(|p| p.name == v.name) {
            res.errors.push((v.name.clone(), BAD_NAME));
        } else {
            pending.push(v);
        }
    }
    let mut progress = true;
    while progress && !pending.is_empty() {
        progress = false;
        let mut o = 0;
        while o < pending.len() {
            let a = pending[o];
            let blocked = identifiers(&a.expr).iter().any(|h| pending.iter().any(|d| &d.name == h));
            if blocked {
                o += 1;
                continue;
            }
            match eval(&a.expr, Some(&res.values)) {
                Ok(c) => res.values.set(&a.name, c),
                Err(()) => res.errors.push((a.name.clone(), BAD_EXPRESSION)),
            }
            pending.remove(o);
            progress = true;
        }
    }
    for p in pending {
        res.errors.push((p.name.clone(), CYCLIC_VARIABLES));
    }
    res
}

/// Parse `part.variables` (`[{name, expr}]`; expr may be a JSON number).
pub fn parse_variables(j: Option<&Json>) -> Vec<Variable> {
    let mut out = Vec::new();
    if let Some(arr) = j.and_then(|j| j.as_arr()) {
        for v in arr {
            let name = v.get("name").and_then(|n| n.as_str()).unwrap_or("").to_string();
            let expr = match v.get("expr") {
                Some(Json::Str(s)) => s.clone(),
                Some(Json::Num(n)) => ck_json::fmt_f64(*n),
                _ => String::new(),
            };
            out.push(Variable { name, expr });
        }
    }
    out
}

/// Evaluate a numeric feature field (`depth`, `radius`, `offset`, ...).
///
/// JSON number -> itself; string starting with `=` -> expression (error
/// `badExpression`); other string -> trimmed float or 0; anything else
/// (missing, bool, null) -> `default`. Pinned by the extrude-depth probes in
/// the module doc.
pub fn eval_field(j: Option<&Json>, vars: &Vars, default: f64) -> Result<f64, &'static str> {
    match j {
        Some(Json::Num(n)) => Ok(*n),
        Some(Json::Str(s)) => {
            let t = s.trim_start();
            if t.starts_with('=') {
                eval(t, Some(vars)).map_err(|_| BAD_EXPRESSION)
            } else {
                // GUESS: plain-number strings that fail to parse fall back to 0
                // (oracle: "abc" / "2*3" / "1in" give a zero-depth extrude).
                Ok(s.trim().parse::<f64>().ok().filter(|v| v.is_finite()).unwrap_or(0.0))
            }
        }
        _ => Ok(default),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(s: &str) -> Option<f64> {
        eval(s, None).ok()
    }

    #[test]
    fn grammar() {
        assert_eq!(ev("20"), Some(20.0));
        assert_eq!(ev("=2^3^2"), Some(512.0));
        assert_eq!(ev("=-2^2"), Some(-4.0));
        assert_eq!(ev("--3"), Some(3.0));
        assert_eq!(ev("abs(-4)"), Some(4.0));
        assert_eq!(ev("SQRT(4)"), Some(2.0));
        assert_eq!(ev("PI"), Some(std::f64::consts::PI));
        assert_eq!(ev("sqrt(16)+pi"), Some(4.0 + std::f64::consts::PI));
        assert_eq!(ev("cos(60)*10"), Some(5.000000000000001)); // oracle
        assert_eq!(ev("1cm"), Some(10.0));
        assert_eq!(ev("1in"), Some(25.4));
        assert_eq!(ev("3 m"), Some(3000.0));
        assert_eq!(ev("3 mm"), Some(3.0));
        assert_eq!(ev("1,5"), Some(1.5));
        assert_eq!(ev("1e1"), Some(10.0));
        assert_eq!(ev(".5"), Some(0.5));
        assert_eq!(ev("5."), Some(5.0));
        assert_eq!(ev("(1+2)*3"), Some(9.0));
        assert_eq!(ev("10/4"), Some(2.5));
        for bad in ["7)", "(7", "7 7", "sqrt 4", "1e400", "abc", "", "3 mx", "foo(2)"] {
            assert_eq!(ev(bad), None, "{bad}");
        }
    }

    #[test]
    fn variables_case_sensitive() {
        let mut v = Vars::new();
        v.set("w", 30.0);
        assert_eq!(eval("=w*2", Some(&v)), Ok(60.0));
        assert_eq!(eval("w/3", Some(&v)), Ok(10.0));
        assert!(eval("W", Some(&v)).is_err());
    }

    #[test]
    fn fields() {
        let mut v = Vars::new();
        v.set("w", 30.0);
        let f = |j: Json| eval_field(Some(&j), &v, 0.0);
        assert_eq!(f(Json::Num(10.0)), Ok(10.0));
        assert_eq!(f(Json::Str("=10".into())), Ok(10.0));
        assert_eq!(f(Json::Str("= w * 2 ".into())), Ok(60.0));
        assert_eq!(f(Json::Str(" 7 ".into())), Ok(7.0));
        assert_eq!(f(Json::Str("2*3".into())), Ok(0.0));
        assert_eq!(f(Json::Str("=abc".into())), Err(BAD_EXPRESSION));
        assert_eq!(f(Json::Bool(true)), Ok(0.0));
        assert_eq!(eval_field(None, &v, 5.0), Ok(5.0));
    }

    #[test]
    fn resolve() {
        let vs = |l: &[(&str, &str)]| -> Vec<Variable> {
            l.iter().map(|(n, e)| Variable { name: n.to_string(), expr: e.to_string() }).collect()
        };
        let r = resolve_variables(&vs(&[("b", "a*2"), ("a", "10"), ("c", "c+1"), ("pi", "3"), ("1x", "2"), ("d", "q")]));
        assert_eq!(r.values.get("a"), Some(10.0));
        assert_eq!(r.values.get("b"), Some(20.0));
        assert!(r.errors.contains(&("c".to_string(), CYCLIC_VARIABLES)));
        assert!(r.errors.contains(&("pi".to_string(), BAD_NAME)));
        assert!(r.errors.contains(&("1x".to_string(), BAD_NAME)));
        assert!(r.errors.contains(&("d".to_string(), BAD_EXPRESSION)));
    }
}
