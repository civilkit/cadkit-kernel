//! Probe harness: solve sketch cases from a JSON file ({name, sketch}), print
//! solved points to full precision for oracle comparison.
use ck_json::Json;
use ck_model::sketch::solver::{solve_sketch, SolveOpts};
use ck_model::Sketch;

fn main() {
    let path = std::env::args().nth(1).unwrap();
    let src = std::fs::read_to_string(&path).unwrap();
    let cases = ck_json::parse(&src).unwrap();
    for case in cases.as_arr().unwrap() {
        let name = case.get("name").unwrap().as_str().unwrap();
        let mut sk = Sketch::from_json(case.get("sketch").unwrap());
        for c in sk.constraints.iter_mut() {
            if let Some(e) = &c.expr {
                c.value = Some(ck_model::expr::eval(e, None).unwrap());
            }
        }
        let pts: Vec<String> = match solve_sketch(&mut sk, &SolveOpts::default()) {
            Ok(r) if r.ok => sk.points.iter().map(|p| format!("[{:?},{:?}]", p.x, p.y)).collect(),
            _ => vec!["UNSOLVED".into()],
        };
        println!("{name}: {}", pts.join(" "));
    }
}
