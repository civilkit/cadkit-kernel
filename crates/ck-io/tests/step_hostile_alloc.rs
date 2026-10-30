//! solidcore #1063: sizes read from the file must not drive unchecked
//! allocation. Separate binary: an allocation failure aborts the process.
use ck_io::step::read_step;

fn curve_file(kind_args: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;ENDSEC;\nDATA;\n#1=CARTESIAN_POINT('',(0.,0.,0.));\n#2=CARTESIAN_POINT('',(1.,0.,0.));\n\
         #3=VERTEX_POINT('',#1);\n#4=VERTEX_POINT('',#2);\n{kind_args}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

fn run(text: String) -> Result<(), String> {
    std::thread::spawn(move || {
        let _ = read_step(&text);
    })
    .join()
    .map_err(|_| "panic".to_string())
}

/// Reads like a solid so the curve is reached: edge on the curve.
fn edge_on(curve: &str) -> String {
    curve_file(&format!("{curve}\n#6=EDGE_CURVE('',#3,#4,#5,.T.);\n#7=ORIENTED_EDGE('',*,*,#6,.T.);\n#8=EDGE_LOOP('',(#7));\n#9=FACE_OUTER_BOUND('',#8,.T.);\n#10=PLANE('',#11);\n#11=AXIS2_PLACEMENT_3D('',#1,$,$);\n#12=ADVANCED_FACE('',(#9),#10,.T.);\n#13=CLOSED_SHELL('',(#12));\n#14=MANIFOLD_SOLID_BREP('',#13);"))
}

#[test]
fn huge_degree_with_default_knots_is_refused() {
    for deg in ["1.E18", "1.E30", "4611686018427387904"] {
        let t = edge_on(&format!("#5=QUASI_UNIFORM_CURVE('',{deg},(#1,#2),.UNSPECIFIED.,.F.,.F.);"));
        run(t).unwrap();
        let t = edge_on(&format!("#5=UNIFORM_CURVE('',{deg},(#1,#2),.UNSPECIFIED.,.F.,.F.);"));
        run(t).unwrap();
    }
}


#[test]
fn huge_knot_multiplicity_is_refused() {
    // 1.E12 knots would be 8 TB: must be a structured error before allocating
    let t = edge_on("#5=B_SPLINE_CURVE_WITH_KNOTS('',1,(#1,#2),.UNSPECIFIED.,.F.,.F.,(1000000000000,1000000000000),(0.,1.),.UNSPECIFIED.);");
    let e = read_step(&t).unwrap_err();
    assert!(e.message.contains("knot"), "{e}");
}

/// A face whose surface (#10) is `surf`; the edge is an ordinary line.
fn face_on(surf: &str) -> String {
    edge_on("#30=DIRECTION('',(1.,0.,0.));\n#31=VECTOR('',#30,1.);\n#5=LINE('',#1,#31);")
        .replace("#10=PLANE('',#11);", surf)
}

#[test]
fn huge_surface_degree_and_multiplicity_are_refused() {
    let degree = "#10=B_SPLINE_SURFACE_WITH_KNOTS('',1000000000000,1,((#1,#2),(#1,#2)),.UNSPECIFIED.,.F.,.F.,.F.,.UNSPECIFIED.,(2,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);";
    let e = read_step(&face_on(degree)).unwrap_err();
    assert!(e.message.contains("degree"), "{e}");
    let mult = "#10=B_SPLINE_SURFACE_WITH_KNOTS('',1,1,((#1,#2),(#1,#2)),.UNSPECIFIED.,.F.,.F.,.F.,.UNSPECIFIED.,(1000000000000,2),(2,2),(0.,1.),(0.,1.),.UNSPECIFIED.);";
    let e = read_step(&face_on(mult)).unwrap_err();
    assert!(e.message.contains("knot"), "{e}");
}
