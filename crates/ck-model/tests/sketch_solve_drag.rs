//! The dense sketch drag of `examples/sketch_solve_bench.rs` as a test: an N x N grid of off-square cells with a tangent circle in
//! each, its far corner dragged in pin steps, each a full solve from the last result. Every solve must converge, and (loose bound,
//! a loaded box must not trip it) take under a second at N = 8 and under 10 s at N = 16. The per-solve times are printed.
use ck_model::sketch::solver::{build, solve, write_back, SolveOpts};
use std::time::Instant;

#[allow(dead_code)]
#[path = "../examples/sketch_solve_bench.rs"]
mod bench;

fn drag(n: usize, steps: usize) -> (usize, usize, f64, f64) {
    let mut prev: Option<ck_model::Sketch> = None;
    let (mut worst, mut total) = (0.0f64, 0.0f64);
    let (mut eqs, mut unknowns) = (0, 0);
    for s in 0..steps {
        let mut sk = bench::grid(n, (1.5 * (s + 1) as f64, -1.0 * (s + 1) as f64));
        if let Some(p) = &prev {
            for (q, o) in sk.points.iter_mut().zip(&p.points) {
                if q.id != format!("p{n}_{n}") {
                    (q.x, q.y) = (o.x, o.y);
                }
            }
        }
        let t = Instant::now();
        let sys = build(&sk).unwrap();
        let r = solve(&sys, &SolveOpts::default());
        let dt = t.elapsed().as_secs_f64();
        assert!(r.ok, "step {s} of the {n} x {n} drag did not converge");
        (eqs, unknowns) = (sys.eqs.len(), sys.x.len());
        worst = worst.max(dt);
        total += dt;
        write_back(&mut sk, &sys, &r.x);
        prev = Some(sk);
    }
    (unknowns, eqs, total / steps as f64, worst)
}

#[test]
fn dense_drag_8_by_8_solves_fast() {
    let (u, e, mean, worst) = drag(8, 12);
    eprintln!("drag 8x8: {u} unknowns, {e} equations: mean {:.2} ms, worst {:.2} ms per solve", mean * 1e3, worst * 1e3);
    assert!(worst < 1.0, "worst solve {worst} s");
}

#[test]
fn dense_drag_16_by_16_solves() {
    let (u, e, mean, worst) = drag(16, 12);
    eprintln!("drag 16x16: {u} unknowns, {e} equations: mean {:.2} ms, worst {:.2} ms per solve", mean * 1e3, worst * 1e3);
    assert!(worst < 10.0, "worst solve {worst} s");
}
