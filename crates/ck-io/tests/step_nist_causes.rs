//! Regression tests for causes found importing the SolidWorks-exported NIST PMI reference files (hand-written
//! minimal STEP bodies, one per cause):
//! - the plane-angle unit is the one named by the geometric representation context (a DEGREE conversion unit
//!   defined against a RADIAN SI unit that has a lower entity id),
//! - a cone bounded by one circle (the apex is the pole) must not be taken for a pole jump (zero-area face),
//! - a torus with negative radii,
//! - a hemisphere bounded by one great circle through the poles,
//! - an intersection edge written as a spline that is a few 1e-3 off its two faces.
use ck_io::step::read::read_step;

const HEAD: &str = "ISO-10303-21;
HEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('t','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC;
DATA;
";

const UNITS_DEG: &str = "#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#2=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#3=PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.0174532925199433),#2);
#4=(CONVERSION_BASED_UNIT('DEGREE',#3)NAMED_UNIT(*)PLANE_ANGLE_UNIT());
#5=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#4))REPRESENTATION_CONTEXT('c','3'));
";

fn import(body: &str) -> (f64, f64, usize, Vec<String>) {
    let text = format!("{HEAD}{body}ENDSEC;\nEND-ISO-10303-21;\n");
    let r = read_step(&text).unwrap_or_else(|e| panic!("import failed: {e}"));
    let m = ck_topo::mass::mass_props(&r.body);
    let issues = ck_check::validate(&r.body, 1e-6).iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect();
    (m.volume, m.area, r.body.num_faces(), issues)
}

fn near(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs()
}

/// Cone (semi angle 45 degrees, base radius 10, height 10) closed by a disk. The cone face has one boundary, the
/// base circle; the apex is the pole.
#[test]
fn cone_in_degrees_bounded_by_one_circle() {
    let body = format!(
        "{UNITS_DEG}#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(10.,0.,0.));
#20=DIRECTION('',(0.,0.,1.));#21=DIRECTION('',(0.,0.,-1.));#22=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#22);#31=AXIS2_PLACEMENT_3D('',#10,#21,#22);
#40=CIRCLE('',#30,10.);#41=VERTEX_POINT('',#11);#42=EDGE_CURVE('',#41,#41,#40,.T.);
#43=CONICAL_SURFACE('',#31,10.,45.);#44=PLANE('',#31);
#50=ORIENTED_EDGE('',*,*,#42,.T.);#51=ORIENTED_EDGE('',*,*,#42,.F.);
#52=EDGE_LOOP('',(#50));#53=EDGE_LOOP('',(#51));
#54=FACE_OUTER_BOUND('',#52,.T.);#55=FACE_OUTER_BOUND('',#53,.T.);
#56=ADVANCED_FACE('',(#54),#43,.T.);#57=ADVANCED_FACE('',(#55),#44,.T.);
#60=CLOSED_SHELL('',(#56,#57));#61=MANIFOLD_SOLID_BREP('',#60);
"
    );
    let (vol, area, faces, issues) = import(&body);
    let (r, h) = (10.0f64, 10.0f64);
    assert_eq!(faces, 2);
    assert!(near(vol, std::f64::consts::PI * r * r * h / 3.0, 1e-6), "volume {vol}");
    let slant = (r * r + h * h).sqrt();
    assert!(near(area, std::f64::consts::PI * r * (r + slant), 1e-6), "area {area}");
    assert!(issues.is_empty(), "{issues:?}");
}

/// Whole torus (one face bounded by a vertex, the OCCT primitive form) with a negative major radius.
#[test]
fn torus_with_negative_major_radius() {
    for (major, minor) in [("10.", "3."), ("-10.", "3."), ("10.", "-3."), ("-10.", "-3.")] {
        let body = format!(
            "#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));#11=CARTESIAN_POINT('',(13.,0.,0.));
#20=DIRECTION('',(0.,0.,1.));#22=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#22);
#41=VERTEX_POINT('',#11);#43=TOROIDAL_SURFACE('',#30,{major},{minor});
#52=VERTEX_LOOP('',#41);#54=FACE_BOUND('',#52,.T.);
#56=ADVANCED_FACE('',(#54),#43,.T.);
#60=CLOSED_SHELL('',(#56));#61=MANIFOLD_SOLID_BREP('',#60);
"
        );
        let (vol, area, faces, issues) = import(&body);
        let pi2 = std::f64::consts::PI * std::f64::consts::PI;
        assert_eq!(faces, 1);
        assert!(near(vol, 2.0 * pi2 * 10.0 * 9.0, 1e-6), "{major} {minor}: volume {vol}");
        assert!(near(area, 4.0 * pi2 * 10.0 * 3.0, 1e-6), "{major} {minor}: area {area}");
        assert!(issues.is_empty(), "{major} {minor}: {issues:?}");
    }
}

/// Hemisphere (radius 5, y >= 0) closed by a disk; the dome's only boundary is a great circle through both
/// poles of the sphere's own frame.
#[test]
fn hemisphere_bounded_by_a_meridian_circle() {
    let body = "#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));#11=CARTESIAN_POINT('',(5.,0.,0.));
#20=DIRECTION('',(0.,1.,0.));#21=DIRECTION('',(0.,-1.,0.));#22=DIRECTION('',(1.,0.,0.));#23=DIRECTION('',(0.,0.,1.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#22);#31=AXIS2_PLACEMENT_3D('',#10,#21,#22);#32=AXIS2_PLACEMENT_3D('',#10,#23,#22);
#40=CIRCLE('',#30,5.);#41=VERTEX_POINT('',#11);#42=EDGE_CURVE('',#41,#41,#40,.T.);
#43=SPHERICAL_SURFACE('',#32,5.);#44=PLANE('',#31);
#50=ORIENTED_EDGE('',*,*,#42,.T.);#51=ORIENTED_EDGE('',*,*,#42,.F.);
#52=EDGE_LOOP('',(#50));#53=EDGE_LOOP('',(#51));
#54=FACE_OUTER_BOUND('',#52,.T.);#55=FACE_OUTER_BOUND('',#53,.T.);
#56=ADVANCED_FACE('',(#54),#43,.T.);#57=ADVANCED_FACE('',(#55),#44,.T.);
#60=CLOSED_SHELL('',(#56,#57));#61=MANIFOLD_SOLID_BREP('',#60);
";
    let (vol, area, _faces, issues) = import(body);
    let pi = std::f64::consts::PI;
    assert!(near(vol, 2.0 / 3.0 * pi * 125.0, 1e-6), "volume {vol}");
    assert!(near(area, 3.0 * pi * 25.0, 1e-6), "area {area}");
    assert!(issues.is_empty(), "{issues:?}");
}

/// Half-cylinder block (radius 10, y >= 0, z 0..5) whose two arc edges are written as cubic B-splines that miss
/// the cylinder by ~1e-3 (the classic 4-arc approximation of a circle).
#[test]
fn spline_intersection_edges_a_little_off_their_faces() {
    let k = 5.5;
    let arc = |z: f64, id: u32| {
        format!(
            "#{p0}=CARTESIAN_POINT('',(10.,0.,{z}));#{p1}=CARTESIAN_POINT('',(10.,{k},{z}));#{p2}=CARTESIAN_POINT('',({k},10.,{z}));#{p3}=CARTESIAN_POINT('',(0.,10.,{z}));
#{p4}=CARTESIAN_POINT('',(-{k},10.,{z}));#{p5}=CARTESIAN_POINT('',(-10.,{k},{z}));#{p6}=CARTESIAN_POINT('',(-10.,0.,{z}));
#{c}=B_SPLINE_CURVE_WITH_KNOTS('',3,(#{p0},#{p1},#{p2},#{p3},#{p4},#{p5},#{p6}),.UNSPECIFIED.,.F.,.F.,(4,3,4),(0.,1.,2.),.UNSPECIFIED.);
",
            p0 = id,
            p1 = id + 1,
            p2 = id + 2,
            p3 = id + 3,
            p4 = id + 4,
            p5 = id + 5,
            p6 = id + 6,
            c = id + 7,
        )
    };
    let body = format!(
        "#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
{}{}#100=CARTESIAN_POINT('',(0.,0.,0.));#101=CARTESIAN_POINT('',(0.,0.,5.));
#102=CARTESIAN_POINT('',(0.,0.,0.));
#110=DIRECTION('',(0.,0.,1.));#111=DIRECTION('',(0.,0.,-1.));#112=DIRECTION('',(1.,0.,0.));#113=DIRECTION('',(0.,-1.,0.));
#114=DIRECTION('',(0.,1.,0.));#115=DIRECTION('',(1.,0.,0.));
#120=VECTOR('',#112,1.);#121=VECTOR('',#110,1.);
#122=CARTESIAN_POINT('',(-10.,0.,0.));#123=CARTESIAN_POINT('',(-10.,0.,5.));#124=CARTESIAN_POINT('',(10.,0.,0.));
#125=CARTESIAN_POINT('',(10.,0.,5.));
#130=LINE('',#122,#120);#131=LINE('',#123,#120);#132=LINE('',#122,#121);#133=LINE('',#124,#121);
#140=VERTEX_POINT('',#124);#141=VERTEX_POINT('',#122);#142=VERTEX_POINT('',#125);#143=VERTEX_POINT('',#123);
#150=EDGE_CURVE('',#140,#141,#27,.T.);
#151=EDGE_CURVE('',#142,#143,#47,.T.);
#152=EDGE_CURVE('',#141,#140,#130,.T.);
#153=EDGE_CURVE('',#143,#142,#131,.T.);
#154=EDGE_CURVE('',#141,#143,#132,.T.);
#155=EDGE_CURVE('',#140,#142,#133,.T.);
#160=AXIS2_PLACEMENT_3D('',#100,#110,#112);#161=AXIS2_PLACEMENT_3D('',#100,#111,#112);
#162=AXIS2_PLACEMENT_3D('',#101,#110,#112);
#163=AXIS2_PLACEMENT_3D('',#100,#113,#112);
#170=PLANE('',#161);#171=PLANE('',#162);#172=PLANE('',#163);#173=CYLINDRICAL_SURFACE('',#160,10.);
#180=ORIENTED_EDGE('',*,*,#150,.F.);#181=ORIENTED_EDGE('',*,*,#152,.F.);
#182=ORIENTED_EDGE('',*,*,#151,.T.);#183=ORIENTED_EDGE('',*,*,#153,.T.);
#184=ORIENTED_EDGE('',*,*,#152,.T.);#185=ORIENTED_EDGE('',*,*,#155,.T.);#186=ORIENTED_EDGE('',*,*,#153,.F.);#187=ORIENTED_EDGE('',*,*,#154,.F.);
#188=ORIENTED_EDGE('',*,*,#150,.T.);#189=ORIENTED_EDGE('',*,*,#154,.T.);#190=ORIENTED_EDGE('',*,*,#151,.F.);#191=ORIENTED_EDGE('',*,*,#155,.F.);
#200=EDGE_LOOP('',(#180,#181));
#201=EDGE_LOOP('',(#182,#183));
#202=EDGE_LOOP('',(#184,#185,#186,#187));
#203=EDGE_LOOP('',(#188,#189,#190,#191));
#210=FACE_OUTER_BOUND('',#200,.T.);#211=FACE_OUTER_BOUND('',#201,.T.);#212=FACE_OUTER_BOUND('',#202,.T.);#213=FACE_OUTER_BOUND('',#203,.T.);
#220=ADVANCED_FACE('',(#210),#170,.T.);#221=ADVANCED_FACE('',(#211),#171,.T.);#222=ADVANCED_FACE('',(#212),#172,.T.);#223=ADVANCED_FACE('',(#213),#173,.T.);
#230=CLOSED_SHELL('',(#220,#221,#222,#223));#231=MANIFOLD_SOLID_BREP('',#230);
",
        arc(0.0, 20),
        arc(5.0, 40),
    );
    let (vol, _area, faces, issues) = import(&body);
    assert_eq!(faces, 4);
    // the splines bulge by ~1e-3, so the volume is the half cylinder's to 1e-3
    assert!(near(vol, 0.5 * std::f64::consts::PI * 100.0 * 5.0, 1e-3), "volume {vol}");
    assert!(issues.is_empty(), "{issues:?}");
}

/// NIST FTC-11 (public domain, 7.6 KB, SolidWorks export): OCCT volume 5129.38613642268, area 5474.478682976481.
#[test]
fn nist_ftc_11_matches_occt() {
    let text = std::fs::read_to_string(format!("{}/tests/fixtures/nist_ftc_11.step", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let r = read_step(&text).unwrap();
    let m = ck_topo::mass::mass_props(&r.body);
    assert!(near(m.volume, 5129.38613642268, 1e-6), "volume {}", m.volume);
    assert!(near(m.area, 5474.478682976481, 1e-6), "area {}", m.area);
    assert_eq!(r.body.num_faces(), 6);
    assert!(ck_check::validate(&r.body, 1e-6).is_empty());
}
