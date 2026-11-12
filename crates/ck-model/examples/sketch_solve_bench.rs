//! Sketch solver timing on a dense drag (solidcraft #8631 asked the same of theirs): an N x N grid of cells drawn
//! slightly off-square (every segment horizontal or vertical, the cell widths of the bottom row equal, a circle
//! in every cell tangent to its left and bottom sides), its far corner dragged in `steps` pin moves, each a full
//! solve from the last result as the GUI does. Prints the time per solve and a hash of every solved coordinate's
//! bits, so two builds can be compared for identical results.
//!   cargo run --profile fast -p ck-model --example sketch_solve_bench -- [N=8] [steps=12] [repeats=3]
use ck_model::sketch::solver::{build, solve, SolveOpts};
use ck_model::Sketch;
use std::time::Instant;

pub fn grid(n: usize, drag: (f64, f64)) -> Sketch {
    let id = |i: usize, j: usize| format!("p{i}_{j}");
    let mut pts = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            // off-square by a deterministic wobble so the first solve has work to do
            let (x, y) = (10.0 * j as f64 + 0.3 * ((i * 7 + j * 3) % 5) as f64, 10.0 * i as f64 + 0.2 * ((i * 3 + j * 11) % 7) as f64);
            let (x, y) = if i == n && j == n { (x + drag.0, y + drag.1) } else { (x, y) };
            let fixed = if i == 0 && j == 0 { r#","fixed":true"# } else { "" };
            pts.push(format!(r#"{{"id":"{}","x":{x},"y":{y}{fixed}}}"#, id(i, j)));
        }
    }
    let (mut ents, mut cons) = (Vec::new(), Vec::new());
    for i in 0..=n {
        for j in 0..=n {
            if j < n {
                ents.push(format!(r#"{{"type":"line","id":"h{i}_{j}","p1":"{}","p2":"{}"}}"#, id(i, j), id(i, j + 1)));
                cons.push(format!(r#"{{"type":"horizontal","id":"kh{i}_{j}","refs":["h{i}_{j}"]}}"#));
            }
            if i < n {
                ents.push(format!(r#"{{"type":"line","id":"v{i}_{j}","p1":"{}","p2":"{}"}}"#, id(i, j), id(i + 1, j)));
                cons.push(format!(r#"{{"type":"vertical","id":"kv{i}_{j}","refs":["v{i}_{j}"]}}"#));
            }
            if i < n && j < n {
                pts.push(format!(r#"{{"id":"c{i}_{j}","x":{},"y":{}}}"#, 10.0 * j as f64 + 4.0, 10.0 * i as f64 + 4.0));
                ents.push(format!(r#"{{"type":"circle","id":"o{i}_{j}","c":"c{i}_{j}","r":3}}"#));
                cons.push(format!(r#"{{"type":"tangent","id":"tl{i}_{j}","refs":["v{i}_{j}","o{i}_{j}"]}}"#));
                cons.push(format!(r#"{{"type":"tangent","id":"tb{i}_{j}","refs":["h{i}_{j}","o{i}_{j}"]}}"#));
            }
        }
    }
    for j in 1..n {
        cons.push(format!(r#"{{"type":"equal","id":"ke{j}","refs":["h0_{}","h0_{j}"]}}"#, j - 1));
    }
    cons.push(format!(r#"{{"type":"fix","id":"drag","refs":["{}"],"value":[{},{}]}}"#, id(n, n), 10.0 * n as f64 + drag.0, 10.0 * n as f64 + drag.1));
    let src = format!(r#"{{"type":"sketch","id":"F1","points":[{}],"entities":[{}],"constraints":[{}]}}"#, pts.join(","), ents.join(","), cons.join(","));
    Sketch::from_json(&ck_json::parse(&src).unwrap())
}

fn main() {
    let a: Vec<usize> = std::env::args().skip(1).map(|s| s.parse().unwrap()).collect();
    let (n, steps, repeats) = (*a.first().unwrap_or(&8), *a.get(1).unwrap_or(&12), *a.get(2).unwrap_or(&3));
    let mut best = f64::INFINITY;
    let mut hash = 0xcbf29ce484222325u64;
    let (mut eqs, mut unknowns, mut iters, mut solved) = (0, 0, 0, 0);
    for rep in 0..repeats {
        let mut prev: Option<Sketch> = None;
        let t0 = Instant::now();
        for s in 0..steps {
            let d = (1.5 * (s + 1) as f64, -1.0 * (s + 1) as f64);
            let mut sk = grid(n, d);
            // start each step from the previous solved coordinates (except the dragged corner), as a drag does
            if let Some(p) = &prev {
                for (q, o) in sk.points.iter_mut().zip(&p.points) {
                    if q.id != format!("p{n}_{n}") {
                        (q.x, q.y) = (o.x, o.y);
                    }
                }
            }
            let sys = build(&sk).unwrap();
            let r = solve(&sys, &SolveOpts::default());
            if rep == 0 {
                (eqs, unknowns) = (sys.eqs.len(), sys.x.len());
                iters += r.iterations;
                solved += r.ok as usize;
                for v in &r.x {
                    for b in v.to_bits().to_le_bytes() {
                        hash = (hash ^ b as u64).wrapping_mul(0x100000001b3);
                    }
                }
            }
            if r.ok {
                ck_model::sketch::solver::write_back(&mut sk, &sys, &r.x);
            }
            prev = Some(sk);
        }
        best = best.min(t0.elapsed().as_secs_f64());
    }
    println!(
        "grid {n}x{n}: {unknowns} unknowns, {eqs} equations, {steps} drag solves ({solved} solved, {iters} iterations): best of {repeats} {:.1} ms total, {:.2} ms per solve; result hash {hash:016x}",
        best * 1e3,
        best * 1e3 / steps as f64
    );
}
