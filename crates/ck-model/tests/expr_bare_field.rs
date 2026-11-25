//! A numeric field given as a string without `=`: a plain number, else an expression; a comma is refused.
use ck_model::regen::{regen, NoGeometry, RegenOpts};
use ck_model::Part;

fn offset_of(v: &str) -> (Option<f64>, Vec<String>) {
    let src = format!(
        r#"{{"format":"ck-part","version":1,"name":"t","variables":[{{"name":"w","expr":"30"}}],"features":[{{"type":"plane","id":"P","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{v}}}]}}"#
    );
    let r = regen(&Part::parse(&src).unwrap(), &RegenOpts::default(), &mut NoGeometry);
    (r.planes.iter().find(|(i, _)| i == "P").map(|(_, f)| f.o.z), r.errors.iter().map(|e| e.error.code.clone()).collect())
}

#[test]
fn bare_strings() {
    assert_eq!(offset_of(r#""20+20""#), (Some(40.0), vec![]));
    assert_eq!(offset_of(r#""=20+20""#), (Some(40.0), vec![]));
    assert_eq!(offset_of(r#"" 7 ""#), (Some(7.0), vec![]));
    assert_eq!(offset_of(r#""w/2""#), (Some(15.0), vec![]));
    assert_eq!(offset_of(r#""1in""#), (Some(25.4), vec![]));
    assert_eq!(offset_of(r#""abc""#), (Some(0.0), vec![]));
}

#[test]
fn comma_is_refused() {
    let (z, errs) = offset_of(r#""1,5""#);
    assert_eq!(z, None);
    assert_eq!(errs, vec!["badExpression"]);
}
