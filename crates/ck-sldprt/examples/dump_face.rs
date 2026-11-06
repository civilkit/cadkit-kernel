//! Print the XT topology and carriers of chosen faces (no files are part of the repository):
//!
//!     cargo run --release -p ck-sldprt --example dump_face -- <file.sldprt> <xt face id> [...]
//!
//! Per face: surface (recursively), loops with each fin's edge, curve (recursively) and vertex positions (mm).

use ck_sldprt::ir::*;
use ck_sldprt::{container, xt, Limits};

fn mm(p: &V3) -> String {
    format!("({:.4},{:.4},{:.4})", p[0] * 1e3, p[1] * 1e3, p[2] * 1e3)
}

fn surf(ir: &Ir, si: usize, d: usize) -> String {
    let n = &ir.surfaces[si];
    let s = match &n.geom {
        IrSurface::Plane { o, n, .. } => format!("plane o{} n{:?}", mm(o), n),
        IrSurface::Cylinder { o, axis, r, .. } => format!("cyl o{} ax{:?} r{:.4}", mm(o), axis, r * 1e3),
        IrSurface::Cone { o, axis, r, sin, .. } => format!("cone o{} ax{:?} r{:.4} sin{sin:.4}", mm(o), axis, r * 1e3),
        IrSurface::Sphere { c, r, .. } => format!("sphere c{} r{:.4}", mm(c), r * 1e3),
        IrSurface::Torus { c, axis, major, minor, .. } => format!("torus c{} ax{:?} {:.4}/{:.4}", mm(c), axis, major * 1e3, minor * 1e3),
        IrSurface::Nurbs(b) => format!("bsurf {}x{} {}x{} per {}{}", b.du, b.dv, b.nu, b.nv, b.u_periodic, b.v_periodic),
        IrSurface::Swept { section, dir } => format!("swept dir{dir:?} sec[{}]", section.map_or("-".into(), |c| curve(ir, c, d + 1))),
        IrSurface::Spun { profile, base, axis } => format!("spun base{} ax{axis:?} prof[{}]", mm(base), profile.map_or("-".into(), |c| curve(ir, c, d + 1))),
        IrSurface::Offset { base, dist } => format!("offset {:.4} of [{}]", dist * 1e3, base.map_or("-".into(), |b| surf(ir, b, d + 1))),
        IrSurface::Blend { supports, support_plus, spine, spine_plus, range, limits } => format!(
            "blend r{:?} plus{support_plus:?} spine_plus {spine_plus} lim {:?}\n{pad}  s0[{}]\n{pad}  s1[{}]\n{pad}  spine[{}]",
            [range[0] * 1e3, range[1] * 1e3],
            limits.map(|l| l.map(|p| mm(&p))),
            supports[0].map_or("-".into(), |b| surf(ir, b, d + 1)),
            supports[1].map_or("-".into(), |b| surf(ir, b, d + 1)),
            spine.map_or("-".into(), |c| curve(ir, c, d + 1)),
            pad = " ".repeat(2 * d + 4)
        ),
        IrSurface::Unsupported(m) => format!("unsupported {m}"),
    };
    format!("#{} sense+{} {s}", n.id, n.sense_plus)
}

fn curve(ir: &Ir, ci: usize, d: usize) -> String {
    if d > 6 {
        return "...".into();
    }
    let n = &ir.curves[ci];
    let s = match &n.geom {
        IrCurve::Line { o, d } => format!("line o{} d{d:?}", mm(o)),
        IrCurve::Circle { c, n, r, .. } => format!("circle c{} n{n:?} r{:.4}", mm(c), r * 1e3),
        IrCurve::Ellipse { c, a, b, .. } => format!("ellipse c{} {:.4}/{:.4}", mm(c), a * 1e3, b * 1e3),
        IrCurve::BCurve(b) => format!("bcurve deg{} n{} dim{} rat{} per{} [{:.4},{:.4}]", b.degree, b.ctrl.len(), b.dim, b.weights.is_some(), b.periodic, b.knots[0], b.knots[b.knots.len() - 1]),
        IrCurve::Trimmed { basis, p1, p2, t1, t2 } => format!("trimmed {} {} t {t1:.6}..{t2:.6} of [{}]", mm(p1), mm(p2), basis.map_or("-".into(), |c| curve(ir, c, d + 1))),
        IrCurve::Intersection { pts, surfaces, bounds, .. } => format!(
            "intersection {} pts {}..{}\n{pad}  A[{}]\n{pad}  B[{}]\n{pad}  bounds {:?}",
            pts.len(),
            mm(&pts[0]),
            mm(&pts[pts.len() - 1]),
            surfaces[0].map_or("-".into(), |s| surf(ir, s, d + 1)),
            surfaces[1].map_or("-".into(), |s| surf(ir, s, d + 1)),
            bounds.map(|b| b.map(|(s, k)| (ir.surfaces[s].id, k))),
            pad = " ".repeat(2 * d + 4)
        ),
        IrCurve::Sp { surface, uv } => format!("sp on [{}] uv {}", surface.map_or("-".into(), |s| surf(ir, s, d + 1)), uv.as_ref().map_or("-".into(), |b| format!("deg{} n{} [{:.4},{:.4}]", b.degree, b.ctrl.len(), b.knots[0], b.knots[b.knots.len() - 1]))),
        IrCurve::Unsupported(m) => format!("unsupported {m}"),
    };
    format!("#{} sense+{} {s}", n.id, n.sense_plus)
}

/// Raw fields of the node with XT node id `id` and of the nodes it points at, `depth` levels down.
fn raw(x: &xt::XtFile, n: &xt::Node, depth: usize, ind: usize) {
    let short = |v: &xt::Val| {
        let s = format!("{v:?}");
        if s.len() > 140 { format!("{}...", &s[..140]) } else { s }
    };
    println!("{:ind$}[{}] {} {}", "", n.index, n.name(), n.dump().iter().filter(|f| !matches!(f.0, "attributes_features" | "owner" | "next" | "previous")).map(|f| format!("{}={}", f.0, short(f.1))).collect::<Vec<_>>().join(" "), ind = ind);
    if depth == 0 {
        return;
    }
    for (name, v) in n.dump() {
        if matches!(name, "attributes_features" | "owner" | "next" | "previous" | "node_id") {
            continue;
        }
        let ptrs: Vec<i64> = match v {
            xt::Val::I(i) => vec![*i],
            xt::Val::AI(a) if a.len() <= 4 => a.clone(),
            _ => vec![],
        };
        for p in ptrs {
            if let Some(m) = x.get(p).filter(|m| m.index != n.index && (m.ty >= 29 || m.ty == 18 || m.ty == 17)) {
                print!("{:ind$}{name}: ", "", ind = ind + 2);
                raw(x, m, depth - 1, ind + 2);
            }
        }
    }
}

fn main() {
    let a: Vec<String> = std::env::args().skip(1).collect();
    if a.len() >= 2 && a[1] == "--list" {
        let bytes = std::fs::read(&a[0]).unwrap();
        let lim = Limits::default();
        let c = container::open(&bytes, &lim).unwrap();
        for t in c.transmits(&lim).unwrap() {
            let faces = xt::read_xt(&t.data, &lim).ok().and_then(|x| extract(&x, &lim).ok()).map_or(0, |ir| ir.solid_faces());
            println!("{} {:?} {} bytes, {faces} solid faces", t.stream, t.kind, t.data.len());
        }
        return;
    }
    if a.len() >= 3 && a[1] == "--node" {
        let bytes = std::fs::read(&a[0]).unwrap();
        let lim = Limits::default();
        let c = container::open(&bytes, &lim).unwrap();
        let id: i64 = a[2].parse().unwrap();
        let depth: usize = a.get(3).and_then(|d| d.parse().ok()).unwrap_or(2);
        for t in c.transmits(&lim).unwrap() {
            let Ok(x) = xt::read_xt(&t.data, &lim) else { continue };
            for n in x.nodes.values().filter(|n| n.i("node_id") == id) {
                println!("[{}]", t.stream);
                raw(&x, n, depth, 0);
            }
        }
        return;
    }
    let bytes = std::fs::read(&a[0]).unwrap();
    let ids: Vec<i64> = a[1..].iter().map(|s| s.trim_start_matches('F').parse().unwrap()).collect();
    let lim = Limits::default();
    let c = container::open(&bytes, &lim).unwrap();
    for t in c.transmits(&lim).unwrap() {
        let Ok(x) = xt::read_xt(&t.data, &lim) else { continue };
        let Ok(ir) = extract(&x, &lim) else { continue };
        for f in ir.bodies.iter().flat_map(|b| &b.regions).flat_map(|r| &r.shells).flat_map(|s| &s.faces) {
            if !ids.contains(&f.id) {
                continue;
            }
            println!("[{}] face #{} same_sense {} back {} surf {}", t.stream, f.id, f.same_sense, f.back, f.surface.map_or("-".into(), |s| surf(&ir, s, 0)));
            for l in &f.loops {
                println!("  loop #{} fins {}", l.id, l.fins.len());
                for fin in &l.fins {
                    let e = &ir.edges[fin.edge];
                    let v = |o: Option<usize>| o.map_or("-".into(), |v| format!("{}{}", mm(&ir.vertices[v].p), ir.vertices[v].tol.map_or(String::new(), |t| format!("~{:.1e}", t * 1e3))));
                    println!("    edge #{} fwd {} {} -> {}", e.id, fin.forward, v(e.start), v(e.end));
                    println!("      {}", e.curve.or(e.fin_curve).map_or("-".into(), |c| curve(&ir, c, 3)));
                }
            }
        }
    }
}
