//! Hardening tests for `ck_io::step::read` and `ck_io::step::recog`.
//! Each test feeds crafted malformed STEP text and asserts no panic and no hang.
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_io::step::{read_step, ImportError, Imported};
use std::sync::mpsc;
use std::time::Duration;

fn guarded(text: String) -> Result<Result<Imported, ImportError>, String> {
    let (tx, rx) = mpsc::channel();
    std::thread::Builder::new()
        .stack_size(8 << 20)
        .spawn(move || {
            let r = std::panic::catch_unwind(|| read_step(&text));
            let _ = tx.send(r.map_err(|_| "panic".to_string()));
        })
        .unwrap();
    rx.recv_timeout(Duration::from_secs(10)).unwrap_or_else(|_| Err("hang".to_string()))
}

#[test]
fn sphere_huge_radius_vertex_loop_does_not_panic() {
    let step = "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(1.,0.,0.));
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#21);
#40=VERTEX_POINT('',#11);
#41=SPHERICAL_SURFACE('',#30,1.E20);
#50=VERTEX_LOOP('',#40);
#51=FACE_BOUND('',#50,.T.);
#52=ADVANCED_FACE('',(#51),#41,.T.);
#60=CLOSED_SHELL('',(#52));
#61=MANIFOLD_SOLID_BREP('',#60);
ENDSEC;
END-ISO-10303-21;";
    let res = guarded(step.to_string()).expect("must not panic or hang");
    if let Ok(imp) = res {
        for v in imp.body.vertex_ids() {
            let p = imp.body.vertex(v).p;
            assert!(p.is_finite(), "non-finite vertex: {p:?}");
        }
    }
}

#[test]
fn cone_tiny_semi_angle_apex_does_not_panic() {
    let step = "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(1.E11,0.,0.));
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(0.,0.,-1.));
#22=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#22);
#31=AXIS2_PLACEMENT_3D('',#10,#21,#22);
#40=CIRCLE('',#30,1.E11);
#41=VERTEX_POINT('',#11);
#42=EDGE_CURVE('',#41,#41,#40,.T.);
#43=CONICAL_SURFACE('',#31,1.E11,1.E-11);
#50=ORIENTED_EDGE('',*,*,#42,.T.);
#52=EDGE_LOOP('',(#50));
#54=FACE_OUTER_BOUND('',#52,.T.);
#56=ADVANCED_FACE('',(#54),#43,.T.);
#60=CLOSED_SHELL('',(#56));
#61=MANIFOLD_SOLID_BREP('',#60);
ENDSEC;
END-ISO-10303-21;";
    let res = guarded(step.to_string()).expect("must not panic or hang");
    if let Ok(imp) = res {
        for v in imp.body.vertex_ids() {
            let p = imp.body.vertex(v).p;
            assert!(p.is_finite(), "non-finite vertex: {p:?}");
        }
    }
}

#[test]
fn huge_conversion_unit_overflow_does_not_poison_radii() {
    for rad in ["0.", "1."] {
        let step = format!(
            "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT($,.METRE.));
#2=LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E307),#1);
#3=(CONVERSION_BASED_UNIT('HUGE',#2)LENGTH_UNIT()NAMED_UNIT(*));
#4=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNIT_ASSIGNED_CONTEXT((#3))REPRESENTATION_CONTEXT('c','3'));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(1.,0.,0.));
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#21);
#40=CIRCLE('',#30,{rad});
#41=VERTEX_POINT('',#11);
#42=EDGE_CURVE('',#41,#41,#40,.T.);
#43=CYLINDRICAL_SURFACE('',#30,{rad});
#50=ORIENTED_EDGE('',*,*,#42,.T.);
#52=EDGE_LOOP('',(#50));
#54=FACE_OUTER_BOUND('',#52,.T.);
#56=ADVANCED_FACE('',(#54),#43,.T.);
#60=CLOSED_SHELL('',(#56));
#61=MANIFOLD_SOLID_BREP('',#60);
ENDSEC;
END-ISO-10303-21;"
        );
        let res = guarded(step).expect("must not panic or hang");
        if let Ok(imp) = res {
            assert!(imp.length_scale.is_finite() && imp.length_scale > 0.0);
            for f in &imp.body.faces {
                if let Surface::Cylinder { r, .. } = f.surface {
                    assert!(r.is_finite() && r > 0.0, "non-finite cylinder radius: {r}");
                }
            }
            for e in &imp.body.edges {
                if let Curve::Circle { r, .. } = e.curve {
                    assert!(r.is_finite() && r > 0.0, "non-finite circle radius: {r}");
                }
            }
        }
    }
}

#[test]
fn hyperbola_huge_semi_axes_does_not_overflow() {
    for sa in ["1.E200", "1.E308"] {
        let step = format!(
            "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(1.,0.,0.));
#12=CARTESIAN_POINT('',(2.,1.,0.));
#20=DIRECTION('',(0.,0.,1.));
#21=DIRECTION('',(1.,0.,0.));
#30=AXIS2_PLACEMENT_3D('',#10,#20,#21);
#40=HYPERBOLA('',#30,{sa},{sa});
#41=VERTEX_POINT('',#11);
#42=VERTEX_POINT('',#12);
#43=EDGE_CURVE('',#41,#42,#40,.T.);
#44=PLANE('',#30);
#50=ORIENTED_EDGE('',*,*,#43,.T.);
#52=EDGE_LOOP('',(#50));
#54=FACE_OUTER_BOUND('',#52,.T.);
#56=ADVANCED_FACE('',(#54),#44,.T.);
#60=CLOSED_SHELL('',(#56));
#61=MANIFOLD_SOLID_BREP('',#60);
ENDSEC;
END-ISO-10303-21;"
        );
        let res = guarded(step).expect("must not panic or hang");
        if let Ok(imp) = res {
            for e in &imp.body.edges {
                if let Curve::Conic { frame, p, e: ecc } = e.curve {
                    assert!(frame.o.is_finite() && p.is_finite() && ecc.is_finite(), "poisoned hyperbola: {frame:?} {p} {ecc}");
                }
            }
        }
    }
}

#[test]
fn closed_degree_zero_bspline_seam_does_not_panic() {
    let step = "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(10.,0.,0.));
#12=CARTESIAN_POINT('',(10.,10.,0.));
#13=CARTESIAN_POINT('',(0.,0.,0.));
#14=B_SPLINE_CURVE_WITH_KNOTS('',0,(#10,#11,#12,#13),.UNSPECIFIED.,.F.,.F.,(1,1,1,1,1),(0.,0.25,0.5,0.75,1.),.UNSPECIFIED.);
#20=VERTEX_POINT('',#12);
#21=VERTEX_POINT('',#11);
#22=EDGE_CURVE('',#20,#21,#14,.T.);
#30=DIRECTION('',(0.,0.,1.));
#31=DIRECTION('',(1.,0.,0.));
#32=AXIS2_PLACEMENT_3D('',#10,#30,#31);
#33=PLANE('',#32);
#40=ORIENTED_EDGE('',*,*,#22,.T.);
#41=EDGE_LOOP('',(#40));
#42=FACE_OUTER_BOUND('',#41,.T.);
#43=ADVANCED_FACE('',(#42),#33,.T.);
#50=CLOSED_SHELL('',(#43));
#51=MANIFOLD_SOLID_BREP('',#50);
ENDSEC;
END-ISO-10303-21;";
    let _ = guarded(step.to_string()).expect("must not panic or hang");
}

#[test]
fn empty_complex_entity_does_not_panic() {
    let step = "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=();
#2=MANIFOLD_SOLID_BREP('s',#1);
#3=CLOSED_SHELL('',(#1));
#4=MANIFOLD_SOLID_BREP('s2',#3);
#5=SHELL_BASED_SURFACE_MODEL('m',(#1));
#6=BREP_WITH_VOIDS('v',#3,(#1));
ENDSEC;
END-ISO-10303-21;";
    let res = guarded(step.to_string()).expect("must not panic or hang");
    assert!(res.is_err());
}

#[test]
fn periodic_face_tiny_extrusion_period_does_not_panic() {
    let step = "ISO-10303-21;
HEADER;ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(10.,0.,0.));
#12=CARTESIAN_POINT('',(0.,10.,0.));
#13=CARTESIAN_POINT('',(0.,0.,0.));
#14=B_SPLINE_CURVE_WITH_KNOTS('',1,(#10,#11,#12,#13),.UNSPECIFIED.,.F.,.F.,(2,1,1,2),(0.,0.3E-20,0.7E-20,1.E-20),.UNSPECIFIED.);
#15=DIRECTION('',(0.,0.,1.));
#16=VECTOR('',#15,1.);
#17=SURFACE_OF_LINEAR_EXTRUSION('',#14,#16);
#20=CARTESIAN_POINT('',(0.,0.,5.));
#21=CARTESIAN_POINT('',(10.,0.,5.));
#22=LINE('',#20,#16);
#23=DIRECTION('',(1.,0.,0.));
#24=VECTOR('',#23,1.);
#25=LINE('',#20,#24);
#30=VERTEX_POINT('',#20);
#31=VERTEX_POINT('',#21);
#32=EDGE_CURVE('',#30,#31,#25,.T.);
#40=ORIENTED_EDGE('',*,*,#32,.T.);
#41=EDGE_LOOP('',(#40));
#42=FACE_OUTER_BOUND('',#41,.T.);
#43=ADVANCED_FACE('',(#42),#17,.T.);
#50=CLOSED_SHELL('',(#43));
#51=MANIFOLD_SOLID_BREP('',#50);
ENDSEC;
END-ISO-10303-21;";
    let _ = guarded(step.to_string()).expect("must not panic or hang");
}


