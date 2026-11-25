//! A dome that is one rational B-spline patch (a revolved quarter circle: closed in v, a collapsed row at the pole of u),
//! bounded by the single circle at its rim, as other CAD systems write a revolved spline. The file has no seam and no pole
//! edge: reading it back must give the whole dome (here a hemisphere), not a sliver. Also what our own export writes for
//! such a face, so the body survives export -> import. The same for a ring of such a surface (a spherical zone as one patch,
//! two rim circles, no pole).
use ck_io::step::{read_step, write_step, StepOpts};

fn dome_step(sense_dome: bool) -> String {
    let r = 10.0f64;
    let s2 = std::f64::consts::FRAC_1_SQRT_2;
    let mut t = String::new();
    t.push_str("ISO-10303-21;\nHEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));ENDSEC;\nDATA;\n");
    let mut n = 100;
    let mut pts = |t: &mut String, p: [f64; 3]| {
        n += 1;
        t.push_str(&format!("#{n}=CARTESIAN_POINT('',({:?},{:?},{:?}));\n", p[0], p[1], p[2]));
        n
    };
    // profile (radius, height, weight): equator, corner, pole
    let prof = [(r, 0.0, 1.0), (r, r, s2), (0.0, r, 1.0)];
    let ring = [(1.0, 0.0, 1.0), (1.0, 1.0, s2), (0.0, 1.0, 1.0), (-1.0, 1.0, s2), (-1.0, 0.0, 1.0), (-1.0, -1.0, s2), (0.0, -1.0, 1.0), (1.0, -1.0, s2), (1.0, 0.0, 1.0)];
    let mut rows = vec![];
    let mut wrows = vec![];
    for &(rho, z, wu) in &prof {
        let mut ids = vec![];
        let mut ws = vec![];
        for &(cx, cy, wv) in &ring {
            ids.push(format!("#{}", pts(&mut t, [rho * cx, rho * cy, z])));
            ws.push(format!("{:?}", wu * wv));
        }
        rows.push(format!("({})", ids.join(",")));
        wrows.push(format!("({})", ws.join(",")));
    }
    let hp = std::f64::consts::FRAC_PI_2;
    t.push_str(&format!(
        "#1=(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,2,({}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(3,2,2,2,3),(0.,1.),(0.,{:?},{:?},{:?},{:?}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({}))REPRESENTATION_ITEM('')SURFACE());\n",
        rows.join(","),
        hp,
        2.0 * hp,
        3.0 * hp,
        4.0 * hp,
        wrows.join(",")
    ));
    t.push_str("#2=CARTESIAN_POINT('',(0.,0.,0.));\n#3=DIRECTION('',(0.,0.,1.));\n#4=DIRECTION('',(1.,0.,0.));\n#5=AXIS2_PLACEMENT_3D('',#2,#3,#4);\n#6=CIRCLE('',#5,10.);\n");
    t.push_str("#7=CARTESIAN_POINT('',(10.,0.,0.));\n#8=VERTEX_POINT('',#7);\n#9=EDGE_CURVE('',#8,#8,#6,.T.);\n");
    t.push_str("#10=ORIENTED_EDGE('',*,*,#9,.T.);\n#11=ORIENTED_EDGE('',*,*,#9,.F.);\n#12=EDGE_LOOP('',(#10));\n#13=EDGE_LOOP('',(#11));\n#14=FACE_OUTER_BOUND('',#12,.T.);\n#15=FACE_OUTER_BOUND('',#13,.T.);\n");
    t.push_str(&format!("#16=ADVANCED_FACE('',(#14),#1,{});\n", if sense_dome { ".T." } else { ".F." }));
    t.push_str("#17=DIRECTION('',(0.,0.,-1.));\n#18=AXIS2_PLACEMENT_3D('',#2,#17,#4);\n#19=PLANE('',#18);\n#20=ADVANCED_FACE('',(#15),#19,.T.);\n");
    t.push_str("#21=CLOSED_SHELL('',(#16,#20));\n#22=MANIFOLD_SOLID_BREP('',#21);\nENDSEC;\nEND-ISO-10303-21;\n");
    t
}

fn hemisphere_volume() -> f64 {
    2.0 / 3.0 * std::f64::consts::PI * 1000.0
}

#[test]
fn a_revolved_spline_dome_with_one_rim_circle_reads_whole() {
    // the dome's surface normal (du x dv) points inward, so the outward face is same_sense .F.
    let r = read_step(&dome_step(false)).unwrap();
    assert!(r.parts[0].refused.is_empty(), "{:?}", r.parts[0].refused);
    assert_eq!(r.body.num_faces(), 2);
    let m = ck_topo::mass::mass_props(&r.body);
    assert!((m.volume - hemisphere_volume()).abs() < 5e-3 * hemisphere_volume(), "volume {} vs {}", m.volume, hemisphere_volume());
    let issues = ck_check::validate(&r.body, 1e-6);
    assert!(issues.is_empty(), "{:?}", &issues[..issues.len().min(3)]);
    // and our own export of what was read reads back the same
    let text = write_step(&r.body, "dome", &StepOpts::default());
    let r2 = read_step(&text).unwrap();
    let m2 = ck_topo::mass::mass_props(&r2.body);
    assert!((m2.volume - m.volume).abs() < 1e-6 * m.volume, "round trip volume {} vs {}", m2.volume, m.volume);
    assert_eq!(r2.body.num_faces(), 2);
    let issues = ck_check::validate(&r2.body, 1e-6);
    assert!(issues.is_empty(), "{:?}", &issues[..issues.len().min(3)]);
}

/// A spherical zone (radius 10, latitudes -30..30 degrees) as one rational patch revolved from a quadratic arc, closed by two
/// flat discs. Volume `pi h / 6 (3 a^2 + 3 a^2 + h^2)` with a = 10 cos 30, h = 10.
fn barrel_step() -> String {
    let (r, c30, s30) = (10.0f64, 30f64.to_radians().cos(), 30f64.to_radians().sin());
    let s2 = std::f64::consts::FRAC_1_SQRT_2;
    let mut t = String::new();
    t.push_str("ISO-10303-21;\nHEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));ENDSEC;\nDATA;\n");
    let mut n = 100;
    let prof = [(r * c30, -r * s30, 1.0), (r / c30, 0.0, c30), (r * c30, r * s30, 1.0)];
    let ring = [(1.0, 0.0, 1.0), (1.0, 1.0, s2), (0.0, 1.0, 1.0), (-1.0, 1.0, s2), (-1.0, 0.0, 1.0), (-1.0, -1.0, s2), (0.0, -1.0, 1.0), (1.0, -1.0, s2), (1.0, 0.0, 1.0)];
    let (mut rows, mut wrows) = (vec![], vec![]);
    for &(rho, z, wu) in &prof {
        let (mut ids, mut ws) = (vec![], vec![]);
        for &(cx, cy, wv) in &ring {
            n += 1;
            t.push_str(&format!("#{n}=CARTESIAN_POINT('',({:?},{:?},{:?}));\n", rho * cx, rho * cy, z));
            ids.push(format!("#{n}"));
            ws.push(format!("{:?}", wu * wv));
        }
        rows.push(format!("({})", ids.join(",")));
        wrows.push(format!("({})", ws.join(",")));
    }
    let hp = std::f64::consts::FRAC_PI_2;
    t.push_str(&format!(
        "#1=(BOUNDED_SURFACE()B_SPLINE_SURFACE(2,2,({}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS((3,3),(3,2,2,2,3),(0.,1.),(0.,{:?},{:?},{:?},{:?}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({}))REPRESENTATION_ITEM('')SURFACE());\n",
        rows.join(","),
        hp,
        2.0 * hp,
        3.0 * hp,
        4.0 * hp,
        wrows.join(",")
    ));
    let (a, h) = (r * c30, r * s30);
    t.push_str(&format!("#2=CARTESIAN_POINT('',(0.,0.,{h:?}));\n#3=DIRECTION('',(0.,0.,1.));\n#4=DIRECTION('',(1.,0.,0.));\n#5=AXIS2_PLACEMENT_3D('',#2,#3,#4);\n#6=CIRCLE('',#5,{a:?});\n"));
    t.push_str(&format!("#7=CARTESIAN_POINT('',({a:?},0.,{h:?}));\n#8=VERTEX_POINT('',#7);\n#9=EDGE_CURVE('',#8,#8,#6,.T.);\n"));
    t.push_str(&format!("#32=CARTESIAN_POINT('',(0.,0.,{:?}));\n#35=AXIS2_PLACEMENT_3D('',#32,#3,#4);\n#36=CIRCLE('',#35,{a:?});\n", -h));
    t.push_str(&format!("#37=CARTESIAN_POINT('',({a:?},0.,{:?}));\n#38=VERTEX_POINT('',#37);\n#39=EDGE_CURVE('',#38,#38,#36,.T.);\n", -h));
    // barrel: top rim clockwise, bottom rim counter-clockwise (the outward normal is the inverse of du x dv: sense .F.)
    t.push_str("#10=ORIENTED_EDGE('',*,*,#9,.F.);\n#11=ORIENTED_EDGE('',*,*,#39,.T.);\n#12=EDGE_LOOP('',(#10));\n#13=EDGE_LOOP('',(#11));\n#14=FACE_OUTER_BOUND('',#12,.T.);\n#15=FACE_BOUND('',#13,.T.);\n");
    t.push_str("#16=ADVANCED_FACE('',(#14,#15),#1,.F.);\n");
    // top disc: normal +z, rim counter-clockwise; bottom disc: normal -z, rim clockwise seen from above
    t.push_str("#40=AXIS2_PLACEMENT_3D('',#2,#3,#4);\n#41=PLANE('',#40);\n#42=ORIENTED_EDGE('',*,*,#9,.T.);\n#43=EDGE_LOOP('',(#42));\n#44=FACE_OUTER_BOUND('',#43,.T.);\n#45=ADVANCED_FACE('',(#44),#41,.T.);\n");
    t.push_str("#50=DIRECTION('',(0.,0.,-1.));\n#51=AXIS2_PLACEMENT_3D('',#32,#50,#4);\n#52=PLANE('',#51);\n#53=ORIENTED_EDGE('',*,*,#39,.F.);\n#54=EDGE_LOOP('',(#53));\n#55=FACE_OUTER_BOUND('',#54,.T.);\n#56=ADVANCED_FACE('',(#55),#52,.T.);\n");
    t.push_str("#60=CLOSED_SHELL('',(#16,#45,#56));\n#61=MANIFOLD_SOLID_BREP('',#60);\nENDSEC;\nEND-ISO-10303-21;\n");
    t
}

#[test]
fn a_revolved_spline_ring_with_two_rim_circles_reads_whole() {
    let want = std::f64::consts::PI * 10.0 / 6.0 * (6.0 * 75.0 + 100.0);
    let r = read_step(&barrel_step()).unwrap();
    assert!(r.parts[0].refused.is_empty(), "{:?}", r.parts[0].refused);
    assert_eq!(r.body.num_faces(), 3);
    let m = ck_topo::mass::mass_props(&r.body);
    assert!((m.volume - want).abs() < 5e-3 * want, "volume {} vs {want}", m.volume);
    let issues = ck_check::validate(&r.body, 1e-6);
    assert!(issues.is_empty(), "{:?}", &issues[..issues.len().min(3)]);
    let text = write_step(&r.body, "barrel", &StepOpts::default());
    let r2 = read_step(&text).unwrap();
    let m2 = ck_topo::mass::mass_props(&r2.body);
    assert!((m2.volume - m.volume).abs() < 1e-6 * m.volume, "round trip volume {} vs {}", m2.volume, m.volume);
    assert_eq!(r2.body.num_faces(), 3);
}
