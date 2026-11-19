//! Procedural curves (and surfaces) have no STEP entity: the writer degrades
//! each to its closed form or B-spline approximation, never panics, and writes
//! finite numbers only.
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Vec2, Vec3};
use ck_io::step::StepWriter;

fn cyl() -> Surface {
    Surface::Cylinder { frame: frame_from(Vec3::new(0.5, -0.2, 0.1), Vec3::new(0.1, 0.2, 1.0), Vec3::X), r: 3.0 }
}

pub fn procedural_curves() -> Vec<Curve> {
    let line = Curve::line_through(Vec3::new(-2., 0., 0.), Vec3::ZERO);
    let arc = Curve::circle_x(Vec3::new(0., 1., 0.), Vec3::Z, Vec3::new(0., -1., 0.), 1.0);
    let comp = Curve::composite(vec![(line.clone(), 0.0, 2.0), (arc.clone(), 0.0, 1.5)], 1e-12).unwrap();
    let sphere = Surface::Sphere { frame: frame_from(Vec3::ZERO, Vec3::Z, Vec3::X), r: 5.0 };
    vec![
        comp.clone(),
        Curve::offset(comp, 0.25, Vec3::Z).unwrap(),
        Curve::offset(line.clone(), 0.5, Vec3::Z).unwrap(),
        Curve::offset(arc.clone(), 0.5, Vec3::Z).unwrap(),
        Curve::offset_on(Curve::iso(cyl(), true, 0.5), -0.1, cyl()).unwrap(),
        Curve::projected(arc.clone(), sphere.clone(), None).unwrap(),
        Curve::projected(Curve::line_through(Vec3::new(3.5, 0., 0.), Vec3::new(3.6, 1.5, 2.0)), cyl(), Some(Vec3::X)).unwrap(),
        Curve::on_surface(sphere.clone(), Curve2::line_through(Vec2::new(0.2, -0.5), Vec2::new(2.0, 0.8))),
        Curve::iso(sphere, false, 0.4),
        Curve::iso(cyl(), true, 1.1),
    ]
}

fn finite_text(t: &str) {
    assert!(!t.is_empty());
    assert!(!ck_io::step::write::has_non_finite_token(t), "non-finite token in\n{t}");
}

#[test]
fn procedural_curves_export_without_panic() {
    for c in procedural_curves() {
        let mut w = StepWriter::new();
        w.curve(&c);
        let t = w.text();
        finite_text(t);
        assert!(t.contains("B_SPLINE_CURVE") || t.contains("LINE(") || t.contains("CIRCLE("), "{}: {t}", c.kind());
        // as the profile of an extrusion and a revolution too
        for s in [
            Surface::Extrusion { profile: Box::new(c.clone()), dir: Vec3::new(0., 0., 2.) },
            Surface::Revolution { profile: Box::new(c.clone()), axis_o: Vec3::new(20., 0., 0.), axis: Vec3::Y },
        ] {
            let mut w = StepWriter::new();
            w.surface(&s);
            finite_text(w.text());
        }
    }
}

#[test]
fn procedural_surfaces_export_as_bsplines() {
    use ck_geom::procsurf::{RollingBall, SweepLaw, SweepSurface};
    let arc = Curve::circle_x(Vec3::new(6., 0., 0.), Vec3::Y, Vec3::X, 1.0);
    let path = Curve::circle_x(Vec3::ZERO, Vec3::Z, Vec3::X, 6.0);
    let pl = |n: Vec3| Surface::plane(Vec3::ZERO, n);
    let surfaces = vec![
        Surface::Sweep(Box::new(SweepSurface::new(arc.clone(), path, SweepLaw::Up(Vec3::Z), (0.0, 2.0), (0.0, 3.0)).unwrap())),
        Surface::Sweep(Box::new(SweepSurface::new(arc, Curve::line(Vec3::ZERO, Vec3::Z), SweepLaw::Translate, (0.0, 2.0), (0.0, 3.0)).unwrap())),
        Surface::Blend(Box::new(RollingBall::new(pl(Vec3::Z), pl(Vec3::X), 1.0, 1.0, (1.5, 2.0), Curve::line(Vec3::ZERO, Vec3::Y), (-1.0, 2.0)).unwrap())),
    ];
    for s in surfaces {
        let mut w = StepWriter::new();
        w.surface(&s);
        let t = w.text();
        finite_text(t);
        assert!(t.contains("B_SPLINE_SURFACE"), "{t}");
    }
}
