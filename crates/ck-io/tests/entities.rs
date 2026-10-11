//! Entity-level format checks against lines of ORIGINAL-kernel STEP replies
//! (parts not constructible with ck-topo's make::*: ellipse/spline
//! extrudes, spline revolve, circle->square loft). Each case replays the
//! geometry the oracle wrote and compares the emitted text verbatim.
use ck_geom::curve::Curve;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_io::step::StepWriter;

fn v(x: f64, y: f64, z: f64) -> Vec3 {
    Vec3::new(x, y, z)
}
fn world(w: &mut StepWriter) {
    w.seed_dir(Vec3::Z, 16);
    w.seed_dir(Vec3::X, 17);
}

#[test]
fn rational_loft_surface_and_curve() {
    // loft of r10 circle (top plane) to a 10x10 square 20 above: #19..#25, #30..#33
    let ctrl = vec![v(10., 0., 0.), v(5., 20., 5.), v(10., 0., -9.999999999999998), v(5., 20., 0.),
        v(6.123233995736766E-16, 0., -10.), v(5., 20., -5.)];
    let s = BSplineSurface {
        deg_u: 2, deg_v: 1, knots_u: vec![0., 0., 0., 1., 1., 1.], knots_v: vec![0., 0., 1., 1.], nu: 3, nv: 2, ctrl,
        weights: Some(vec![1., 1., std::f64::consts::FRAC_1_SQRT_2, 1., 1., 1.]),
    };
    let mut w = StepWriter::starting_at(19);
    world(&mut w);
    w.surface(&Surface::BSpline(Box::new(s)));
    assert_eq!(w.text(), "#19=CARTESIAN_POINT('',(10.,0.,0.));
#20=CARTESIAN_POINT('',(5.,20.,5.));
#21=CARTESIAN_POINT('',(10.,0.,-9.999999999999998));
#22=CARTESIAN_POINT('',(5.,20.,0.));
#23=CARTESIAN_POINT('',(6.123233995736766E-16,0.,-10.));
#24=CARTESIAN_POINT('',(5.,20.,-5.));
#25=(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,1,((#19,#20),(#21,#22),(#23,#24)),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(((1.,1.),(0.7071067811865476,1.),(1.,1.)))REPRESENTATION_ITEM('')SURFACE());
");
    let c = BSplineCurve {
        degree: 2, knots: vec![0., 0., 0., 1., 1., 1.],
        ctrl: vec![v(10., 0., 0.), v(10., 0., -9.999999999999998), v(6.123233995736766E-16, 0., -10.)],
        weights: Some(vec![1., std::f64::consts::FRAC_1_SQRT_2, 1.]),
    };
    let mut w = StepWriter::starting_at(30);
    w.curve(&Curve::BSpline(c));
    assert_eq!(w.text(), "#30=CARTESIAN_POINT('',(10.,0.,0.));
#31=CARTESIAN_POINT('',(10.,0.,-9.999999999999998));
#32=CARTESIAN_POINT('',(6.123233995736766E-16,0.,-10.));
#33=(BOUNDED_CURVE()B_SPLINE_CURVE(2,(#30,#31,#32),.UNSPECIFIED.,.F.,.F.)B_SPLINE_CURVE_WITH_KNOTS((3,3),(0.,1.),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE((1.,0.7071067811865476,1.))REPRESENTATION_ITEM(''));
");
}

#[test]
fn ellipse_extrusion() {
    // ellipse c=(0,0) p=(10,0) r=4 extruded 5 on the front plane: #45..#49
    let fr = Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
    let mut w = StepWriter::starting_at(45);
    world(&mut w);
    w.surface(&Surface::Extrusion { profile: Box::new(Curve::Ellipse { frame: fr, a: 10., b: 4. }), dir: v(0., 0., 5.) });
    assert_eq!(w.text(), "#45=CARTESIAN_POINT('',(0.,0.,0.));
#46=AXIS2_PLACEMENT_3D('',#45,#16,#17);
#47=ELLIPSE('',#46,10.,4.);
#48=VECTOR('',#16,1.);
#49=SURFACE_OF_LINEAR_EXTRUSION('',#47,#48);
");
}

#[test]
fn spline_revolution() {
    // spline (10,0)(14,5)(11,10)(15,15) + line, revolved 90° about Y: #73..#84
    let ctrl = vec![v(10., 0., 0.), v(12.168702415880148, 1.6104781708137568, 0.), v(14.3374048317603, 3.2209563416275135, 0.),
        v(14., 5., 0.), v(13.84637254642185, 5.810035664321152, 0.), v(13.173186273210925, 6.655017832160578, 0.),
        v(12.500000000000002, 7.500000000000003, 0.)];
    let k = vec![0., 0., 0., 0., 6.4031242374328485, 6.4031242374328485, 6.4031242374328485, 9.3186001848555, 9.3186001848555,
        9.3186001848555, 9.3186001848555];
    let c = Curve::BSpline(BSplineCurve { degree: 3, knots: k, ctrl, weights: None });
    let mut w = StepWriter::starting_at(73);
    world(&mut w);
    w.surface(&Surface::Revolution { profile: Box::new(c), axis_o: Vec3::ZERO, axis: Vec3::Y });
    assert_eq!(w.text(), "#73=CARTESIAN_POINT('',(10.,0.,0.));
#74=CARTESIAN_POINT('',(12.168702415880148,1.6104781708137568,0.));
#75=CARTESIAN_POINT('',(14.3374048317603,3.2209563416275135,0.));
#76=CARTESIAN_POINT('',(14.,5.,0.));
#77=CARTESIAN_POINT('',(13.84637254642185,5.810035664321152,0.));
#78=CARTESIAN_POINT('',(13.173186273210925,6.655017832160578,0.));
#79=CARTESIAN_POINT('',(12.500000000000002,7.500000000000003,0.));
#80=B_SPLINE_CURVE_WITH_KNOTS('',3,(#73,#74,#75,#76,#77,#78,#79),.UNSPECIFIED.,.F.,.F.,(4,3,4),(0.,6.4031242374328485,9.3186001848555),.UNSPECIFIED.);
#81=CARTESIAN_POINT('',(0.,0.,0.));
#82=DIRECTION('',(0.,1.,0.));
#83=AXIS1_PLACEMENT('',#81,#82);
#84=SURFACE_OF_REVOLUTION('',#80,#83);
");
}

#[test]
fn closed_spline_profile_flag() {
    // closed-spline extrude: periodic profile written closed_curve=.T. (spl #107)
    let p = [v(0., 0., 0.), v(1., 2., 0.), v(3., 1., 0.), v(0., 0., 0.)];
    let c = Curve::BSpline(BSplineCurve { degree: 3, knots: vec![0., 0., 0., 0., 1., 1., 1., 1.], ctrl: p.to_vec(), weights: None });
    let mut w = StepWriter::starting_at(1);
    w.curve(&c);
    assert!(w.text().ends_with("#5=B_SPLINE_CURVE_WITH_KNOTS('',3,(#1,#2,#3,#4),.UNSPECIFIED.,.T.,.F.,(4,4),(0.,1.),.UNSPECIFIED.);\n"));
}
