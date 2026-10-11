//! Evaluate the built equation system at given points (oracle check).
use ck_json::Json;
use ck_model::sketch::solver::build;
use ck_model::Sketch;

fn main() {
    // args: cases.json name x1,y1 x2,y2 ...
    let args: Vec<String> = std::env::args().collect();
    let cases = ck_json::parse(&std::fs::read_to_string(&args[1]).unwrap()).unwrap();
    let name = &args[2];
    let pts: Vec<[f64; 2]> = args[3..]
        .iter()
        .map(|p| {
            let t: Vec<f64> = p.split(',').map(|v| v.parse().unwrap()).collect();
            [t[0], t[1]]
        })
        .collect();
    for case in cases.as_arr().unwrap() {
        if case.get("name").unwrap().as_str().unwrap() != name.as_str() {
            continue;
        }
        let mut sk = Sketch::from_json(case.get("sketch").unwrap());
        for c in sk.constraints.iter_mut() {
            if let Some(e) = &c.expr {
                c.value = Some(ck_model::expr::eval(e, None).unwrap());
            }
        }
        for (i, p) in sk.points.iter_mut().enumerate() {
            if !p.fixed {
                if let Some(q) = pts.get(i) {
                    p.x = q[0];
                    p.y = q[1];
                }
            }
        }
        let sys = build(&sk).unwrap();
        println!("{name}: {} unknowns, {} eqs", sys.x.len(), sys.eqs.len());
        for (i, e) in sys.eqs.iter().enumerate() {
            println!("  eq{i} [{}] = {:e}", e.cid, e.eval(&sys.x));
        }
    }
}
