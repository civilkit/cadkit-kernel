//! STEP texts written by hand with the analytic geometry stored as B-splines (what exporters do): a box whose edges
//! are degree-1 splines and whose faces are bilinear B-spline surfaces, and a cylinder whose circles are rational
//! quadratic splines and whose caps are flat B-spline surfaces. `splines = false` writes the same parts with LINE,
//! CIRCLE and PLANE.
#![allow(dead_code)]
use std::fmt::Write;

struct W {
    s: String,
    n: u32,
}

impl W {
    fn new() -> W {
        let mut w = W { s: String::new(), n: 0 };
        w.s.push_str("ISO-10303-21;\nHEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));ENDSEC;\nDATA;\n");
        w
    }
    fn e(&mut self, body: &str) -> u32 {
        self.n += 1;
        writeln!(self.s, "#{}={};", self.n, body).unwrap();
        self.n
    }
    fn pt(&mut self, p: [f64; 3]) -> u32 {
        self.e(&format!("CARTESIAN_POINT('',({:?},{:?},{:?}))", p[0], p[1], p[2]))
    }
    fn dir(&mut self, d: [f64; 3]) -> u32 {
        self.e(&format!("DIRECTION('',({:?},{:?},{:?}))", d[0], d[1], d[2]))
    }
    fn axis(&mut self, o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> u32 {
        let (o, z, x) = (self.pt(o), self.dir(z), self.dir(x));
        self.e(&format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }
    fn finish(mut self, solid: u32) -> String {
        let _ = solid;
        self.s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
        self.s
    }
    fn lin_spline(&mut self, a: [f64; 3], b: [f64; 3]) -> u32 {
        let (pa, pb) = (self.pt(a), self.pt(b));
        self.e(&format!("B_SPLINE_CURVE_WITH_KNOTS('',1,(#{pa},#{pb}),.UNSPECIFIED.,.F.,.F.,(2,2),(0.,1.),.UNSPECIFIED.)"))
    }
    fn lin(&mut self, a: [f64; 3], b: [f64; 3]) -> u32 {
        let d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let l = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        let (p, v) = (self.pt(a), self.dir([d[0] / l, d[1] / l, d[2] / l]));
        let vec = self.e(&format!("VECTOR('',#{v},{l:?})"));
        self.e(&format!("LINE('',#{p},#{vec})"))
    }
    /// Bilinear patch p00, p10 (u), p01 (v), p11 as a B-spline surface, or the plane through it.
    fn flat(&mut self, p00: [f64; 3], p10: [f64; 3], p01: [f64; 3], p11: [f64; 3], spline: bool) -> u32 {
        if spline {
            let g: Vec<u32> = [p00, p01, p10, p11].iter().map(|&p| self.pt(p)).collect();
            self.e(&format!(
                "B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#{},#{}),(#{},#{})),.UNSPECIFIED.,.F.,.F.,.F.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)",
                g[0], g[1], g[2], g[3]
            ))
        } else {
            let u = [p10[0] - p00[0], p10[1] - p00[1], p10[2] - p00[2]];
            let v = [p01[0] - p00[0], p01[1] - p00[1], p01[2] - p00[2]];
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            let a = self.axis(p00, n, u);
            self.e(&format!("PLANE('',#{a})"))
        }
    }
    /// Full circle of radius `r` about the z axis at height `z` as a nine-point rational quadratic, or a CIRCLE.
    fn circle(&mut self, r: f64, z: f64, spline: bool) -> u32 {
        if !spline {
            let a = self.axis([0.0, 0.0, z], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
            return self.e(&format!("CIRCLE('',#{a},{r:?})"));
        }
        let pts = [[r, 0.0], [r, r], [0.0, r], [-r, r], [-r, 0.0], [-r, -r], [0.0, -r], [r, -r], [r, 0.0]];
        let ids: Vec<String> = pts.iter().map(|p| format!("#{}", self.pt([p[0], p[1], z]))).collect();
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let ws = [1.0, w, 1.0, w, 1.0, w, 1.0, w, 1.0].map(|x| format!("{x:?}")).join(",");
        let cps = ids.join(",");
        self.e(&format!(
            "(BOUNDED_CURVE()B_SPLINE_CURVE(2,({cps}),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,2,2,2,3),(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({ws}))REPRESENTATION_ITEM(''))"
        ))
    }
}

/// A box `dx` x `dy` x `dz` with a corner at the origin.
pub fn box_step(dx: f64, dy: f64, dz: f64, splines: bool) -> String {
    let mut w = W::new();
    let c = [
        [0.0, 0.0, 0.0],
        [dx, 0.0, 0.0],
        [dx, dy, 0.0],
        [0.0, dy, 0.0],
        [0.0, 0.0, dz],
        [dx, 0.0, dz],
        [dx, dy, dz],
        [0.0, dy, dz],
    ];
    let v: Vec<u32> = c.iter().map(|&p| { let p = w.pt(p); w.e(&format!("VERTEX_POINT('',#{p})")) }).collect();
    // faces as corner cycles, counter-clockwise seen from outside
    let faces: [[usize; 4]; 6] = [[0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
    let mut edges = std::collections::HashMap::new();
    let mut fids = vec![];
    for f in faces {
        let mut oes = vec![];
        for k in 0..4 {
            let (a, b) = (f[k], f[(k + 1) % 4]);
            let (lo, hi) = (a.min(b), a.max(b));
            let id = *edges.entry((lo, hi)).or_insert_with(|| {
                let cu = if splines { w.lin_spline(c[lo], c[hi]) } else { w.lin(c[lo], c[hi]) };
                w.e(&format!("EDGE_CURVE('',#{},#{},#{cu},.T.)", v[lo], v[hi]))
            });
            oes.push(w.e(&format!("ORIENTED_EDGE('',*,*,#{id},{})", if a < b { ".T." } else { ".F." })));
        }
        let list: Vec<String> = oes.iter().map(|o| format!("#{o}")).collect();
        let lp = w.e(&format!("EDGE_LOOP('',({}))", list.join(",")));
        let fb = w.e(&format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
        // u along the first edge, v along the last (reversed): normal = (p1 - p0) x (p3 - p0), outward
        let s = w.flat(c[f[0]], c[f[1]], c[f[3]], c[f[2]], splines);
        fids.push(w.e(&format!("ADVANCED_FACE('',(#{fb}),#{s},.T.)")));
    }
    let list: Vec<String> = fids.iter().map(|o| format!("#{o}")).collect();
    let sh = w.e(&format!("CLOSED_SHELL('',({}))", list.join(",")));
    let so = w.e(&format!("MANIFOLD_SOLID_BREP('',#{sh})"));
    w.finish(so)
}

/// A cylinder of radius `r` and height `h` on the xy plane, axis z.
pub fn cylinder_step(r: f64, h: f64, splines: bool) -> String {
    let mut w = W::new();
    let mut circ = |w: &mut W, z: f64| {
        let p = w.pt([r, 0.0, z]);
        let v = w.e(&format!("VERTEX_POINT('',#{p})"));
        let c = w.circle(r, z, splines);
        w.e(&format!("EDGE_CURVE('',#{v},#{v},#{c},.T.)"))
    };
    let (eb, et) = (circ(&mut w, 0.0), circ(&mut w, h));
    let a = r * 1.5;
    let mut face = |w: &mut W, e: u32, fwd: bool, s: u32| {
        let oe = w.e(&format!("ORIENTED_EDGE('',*,*,#{e},{})", if fwd { ".T." } else { ".F." }));
        let lp = w.e(&format!("EDGE_LOOP('',(#{oe}))"));
        let fb = w.e(&format!("FACE_OUTER_BOUND('',#{lp},.T.)"));
        w.e(&format!("ADVANCED_FACE('',(#{fb}),#{s},.T.)"))
    };
    let sb = w.flat([-a, -a, 0.0], [-a, a, 0.0], [a, -a, 0.0], [a, a, 0.0], splines);
    let st = w.flat([-a, -a, h], [a, -a, h], [-a, a, h], [a, a, h], splines);
    let fb = face(&mut w, eb, false, sb);
    let ft = face(&mut w, et, true, st);
    let ax = w.axis([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let cy = w.e(&format!("CYLINDRICAL_SURFACE('',#{ax},{r:?})"));
    let o1 = w.e(&format!("ORIENTED_EDGE('',*,*,#{eb},.T.)"));
    let o2 = w.e(&format!("ORIENTED_EDGE('',*,*,#{et},.F.)"));
    let l1 = w.e(&format!("EDGE_LOOP('',(#{o1}))"));
    let l2 = w.e(&format!("EDGE_LOOP('',(#{o2}))"));
    let b1 = w.e(&format!("FACE_OUTER_BOUND('',#{l1},.T.)"));
    let b2 = w.e(&format!("FACE_BOUND('',#{l2},.T.)"));
    let fs = w.e(&format!("ADVANCED_FACE('',(#{b1},#{b2}),#{cy},.T.)"));
    let sh = w.e(&format!("CLOSED_SHELL('',(#{fb},#{ft},#{fs}))"));
    let so = w.e(&format!("MANIFOLD_SOLID_BREP('',#{sh})"));
    w.finish(so)
}
