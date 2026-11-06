//! A history that starts with the file's solid (`imported.rs`): the part document names the file once, as the blob of
//! the starting `importBody`, wherever that feature stands in the list.

use ck_swedit::{imported, Outcome};

#[test]
fn start_feature_gets_the_file_as_its_blob() {
    let o = Outcome {
        features: vec![imported::start_feature("F1", "Imported1")],
        steps: vec![],
        body: None,
        verify: None,
        errors: vec![],
        fallback: None,
        notes: vec![],
        prefix: 0,
        remainder: None,
        checks: vec![],
        ranges: vec![],
    };
    let none = o.part_json(None);
    assert_eq!(none.get("blobs").map(|b| b.to_string()), Some("{}".into()));
    let with = o.part_json(Some(b"file bytes"));
    let f = &with.get("features").unwrap().as_arr().unwrap()[0];
    let key = f.get_str("blob").unwrap().to_string();
    assert!(!key.is_empty());
    assert!(with.get("blobs").unwrap().get(&key).is_some());
    assert_eq!(f.get_str("type"), Some("importBody"));
}

#[test]
fn imported_kinds() {
    assert!(imported::is_imported("Imported") && imported::is_imported("Multi_CAD_Imported"));
    assert!(!imported::is_imported("Combine") && !imported::is_imported("Extrusion"));
}
