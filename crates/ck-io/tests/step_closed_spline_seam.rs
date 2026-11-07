//! An edge on a closed B-spline carrier whose arc runs across the carrier's seam (first vertex later on the curve
//! than the second) reads back as a face instead of being refused for "zero extent". The sample is a closed cubic
//! with its seam at the corner (1,1); the two edges are the arcs u 0.75 -> 0.25 (across the seam) and 0.25 -> 0.75.
use ck_io::step::read_step;

const STEP: &str = "ISO-10303-21;
HEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));ENDSEC;
DATA;
#1=CARTESIAN_POINT('',(0.,0.,0.));
#2=DIRECTION('',(0.,0.,1.));
#3=DIRECTION('',(1.,0.,0.));
#4=AXIS2_PLACEMENT_3D('',#1,#2,#3);
#5=PLANE('',#4);
#10=CARTESIAN_POINT('',(1.,1.,0.));
#11=CARTESIAN_POINT('',(-1.,1.,0.));
#12=CARTESIAN_POINT('',(-1.,-1.,0.));
#13=CARTESIAN_POINT('',(1.,-1.,0.));
#14=B_SPLINE_CURVE_WITH_KNOTS('',3,(#10,#11,#12,#13,#10),.UNSPECIFIED.,.F.,.F.,(4,1,4),(0.,0.5,1.),.UNSPECIFIED.);
#20=CARTESIAN_POINT('',(0.4375,-0.6875,0.));
#21=CARTESIAN_POINT('',(-0.6875,0.4375,0.));
#22=VERTEX_POINT('',#20);
#23=VERTEX_POINT('',#21);
#24=EDGE_CURVE('',#22,#23,#14,.T.);
#25=EDGE_CURVE('',#23,#22,#14,.T.);
#26=ORIENTED_EDGE('',*,*,#24,.T.);
#27=ORIENTED_EDGE('',*,*,#25,.T.);
#28=EDGE_LOOP('',(#26,#27));
#29=FACE_OUTER_BOUND('',#28,.T.);
#30=ADVANCED_FACE('',(#29),#5,.T.);
#31=CLOSED_SHELL('',(#30));
#32=MANIFOLD_SOLID_BREP('',#31);
ENDSEC;
END-ISO-10303-21;
";

#[test]
fn arc_across_the_seam_of_a_closed_spline_reads() {
    let r = read_step(STEP).unwrap();
    let p = &r.parts[0];
    assert!(p.refused.is_empty(), "{:?}", p.refused);
    assert_eq!(p.faces.len(), 1);
    // the face is the whole curve's interior: the cubic's enclosed area
    let t = ck_tess::mesh::tessellate(&r.body);
    assert!(t.faces.iter().all(|f| f.is_some()));
}
