//! Regen an assembly part natively and print the mate solver trace.
//! usage: cargo run --release -p ck-model --example asm_debug -- part.json
use ck_model::features::Brep;
use ck_model::regen::{regen, RegenOpts};
fn main() {
    let path = std::env::args().nth(1).expect("part.json");
    let part = ck_model::Part::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    let mut ex = Brep::new();
    let r = regen(&part, &RegenOpts::default(), &mut ex);
    for l in ck_model::features::mate::DEBUG.with(|d| d.borrow().clone()) {
        println!("{l}");
    }
    println!("errors {:?}", r.errors.iter().map(|e| (&e.feature, &e.error.code)).collect::<Vec<_>>());
    if std::env::var("FACES").is_ok() {
        let b = ex.body.as_ref().unwrap();
        for f in b.face_ids() {
            println!("{} {:?} sense {}", b.face_id_str(f), b.face(f).surface, b.face(f).sense);
        }
    }
    for c in ex.components_json() {
        println!("{}", ck_json::to_string(&c));
    }
}
// (debug) print plane frames of every face: set FACES=1
#[allow(dead_code)]
fn _unused() {}
