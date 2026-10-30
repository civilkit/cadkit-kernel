//! solidcore #1132: deeply nested parameters must be refused, not overflow the
//! stack. Separate test binary so a crash cannot hide other results.
use ck_io::step::read_step;

fn wrap(open: &str, close: &str, n: usize, inner: &str) -> String {
    let mut s = String::from("ISO-10303-21;\nHEADER;ENDSEC;\nDATA;\n#1=X(");
    for _ in 0..n {
        s.push_str(open);
    }
    s.push_str(inner);
    for _ in 0..n {
        s.push_str(close);
    }
    s.push_str(");\nENDSEC;\nEND-ISO-10303-21;\n");
    s
}

fn run(text: String) -> Result<(), String> {
    // a small stack so an unbounded recursion fails fast rather than slowly
    std::thread::Builder::new()
        .stack_size(4 << 20)
        .spawn(move || {
            let _ = read_step(&text);
        })
        .unwrap()
        .join()
        .map_err(|_| "panic".to_string())
}

#[test]
fn typed_parameter_nesting_is_bounded() {
    run(wrap("T(", ")", 100_000, "1")).unwrap();
}

#[test]
fn list_nesting_is_bounded() {
    run(wrap("(", ")", 100_000, "1")).unwrap();
}

#[test]
fn nesting_within_the_limit_still_parses() {
    let f = ck_io::step::parse::parse(&wrap("(", ")", 20, "1")).unwrap();
    assert_eq!(f.ents.len(), 1);
    let f = ck_io::step::parse::parse(&wrap("T(", ")", 20, "1")).unwrap();
    assert_eq!(f.ents.len(), 1);
}
