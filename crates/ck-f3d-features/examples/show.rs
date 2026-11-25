//! Local debugging aid: maps one file and prints the part, the guard's check and the regeneration errors.
use ck_f3d_features::{guard, timeline_to_features};
fn main() {
    let path = std::env::args().nth(1).expect("a file");
    let bytes = std::fs::read(&path).expect("readable");
    let designs = ck_f3d_design::decode_bytes(bytes.clone(), &path).expect("decodes");
    let references = guard::measure_documents(&bytes);
    let only: Option<usize> = std::env::var("DESIGN").ok().and_then(|v| v.parse().ok());
    for (di, d) in designs.into_iter().enumerate() {
        if only.is_some_and(|o| o != di) {
            continue;
        }
        let reference = references.get(di).and_then(|r| r.as_ref().ok()).copied();
        println!("== design {di} label {}", d.label);
        let Some(dump) = d.dump else { continue };
        let import = timeline_to_features(&dump);
        for it in &import.items {
            println!("item {} {} {} {:?} {:?}", it.index, it.name, it.object_type, it.status, it.note);
        }
        if std::env::var("SHOW_PART").is_ok() {
            println!("{}", serde_json::to_string(&import.part()).unwrap_or_default());
        }
        println!("{:?}", guard::verify(&import, reference.as_ref()));
    }
}
