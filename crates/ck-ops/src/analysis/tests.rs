use super::*;
use crate::boolean::tests::{box_body, cyl_body};

fn j(s: &str) -> Json {
    ck_json::parse(s).unwrap()
}
fn two(a: Body, b: Body) -> Body {
    let mut x = a;
    x.append(&b);
    x
}
fn num(r: &Json, k: &str) -> f64 {
    r.get_f64(k).unwrap_or_else(|| panic!("no {k} in {r:?}"))
}

#[test]
fn boxes_five_apart() {
    let b = two(box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), box_body("F3", 15.0, 0.0, 25.0, 10.0, 0.0, 10.0));
    let r = distance_reply(&b, None, &j(r#"{"a":"lump:0","b":"lump:1"}"#)).unwrap();
    assert!((num(&r, "distance") - 5.0).abs() < 1e-9, "{r:?}");
    assert_eq!(r.get("intersects"), Some(&Json::Bool(false)));
    let d = r.get("direction").unwrap().as_vec3().unwrap();
    assert!((d[0].abs() - 1.0).abs() < 1e-9);
    // diagonal offset: edge to edge in the plane, 3-4-5
    let b = two(box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), box_body("F3", 13.0, 14.0, 20.0, 20.0, 0.0, 10.0));
    let r = distance_reply(&b, None, &j(r#"{"a":"lump:0","b":"lump:1"}"#)).unwrap();
    assert!((num(&r, "distance") - 5.0).abs() < 1e-9, "{r:?}");
    // touching faces: intersects
    let b = two(box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), box_body("F3", 10.0, 0.0, 20.0, 10.0, 0.0, 10.0));
    let r = distance_reply(&b, None, &j(r#"{"a":"lump:0","b":"lump:1"}"#)).unwrap();
    assert_eq!(num(&r, "distance"), 0.0);
    assert_eq!(r.get("intersects"), Some(&Json::Bool(true)));
    assert_eq!(r.get("intersection").unwrap().get_str("kind"), Some("area"));
}

#[test]
fn point_and_cylinders() {
    let b = two(cyl_body("F2", 0.0, 0.0, 5.0, 0.0, 10.0), cyl_body("F3", 20.0, 0.0, 5.0, 0.0, 10.0));
    let r = distance_reply(&b, None, &j(r#"{"a":"lump:0","b":"lump:1"}"#)).unwrap();
    assert!((num(&r, "distance") - 10.0).abs() < 1e-9, "{r:?}");
    let r = distance_reply(&b, None, &j(r#"{"a":[0,5,20],"b":"lump:0"}"#)).unwrap();
    assert!((num(&r, "distance") - 15.0).abs() < 1e-9, "{r:?}");
    let r = max_distance_reply(&b, None, &j(r#"{"opts":{"samples":2000}}"#)).unwrap();
    assert!(num(&r, "maxDistance") < 1e-9, "{r:?}");
}

#[test]
fn max_distance_between_bodies() {
    let a = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
    let c = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 12.0);
    let r = max_distance_reply(&a, Some(&c), &j(r#"{"opts":{"samples":3000,"both":true}}"#)).unwrap();
    // surface Hausdorff distance: the top of the short box is 2 below the top of the tall one
    assert!((num(&r, "hausdorff") - 2.0).abs() < 1e-6, "{r:?}");
}

#[test]
fn plate_thickness() {
    let plate = box_body("F2", 0.0, 0.0, 50.0, 30.0, 0.0, 2.0);
    let r = thickness_reply(&plate, &j(r#"{}"#)).unwrap();
    assert!((num(&r, "minThickness") - 2.0).abs() < 1e-6, "{r:?}");
    let r = thickness_reply(&plate, &j(r#"{"opts":{"method":"sphere"}}"#)).unwrap();
    assert!((num(&r, "minThickness") - 2.0).abs() < 1e-5, "{r:?}");
    let r = thickness_at_reply(&plate, &j(r#"{"point":[25,2,-15]}"#)).unwrap();
    assert!((num(r.get("result").unwrap(), "thickness") - 2.0).abs() < 1e-6, "{r:?}");
}

#[test]
fn box_dihedrals_and_closest_point() {
    let b = box_body("F2", 0.0, 0.0, 10.0, 20.0, 0.0, 5.0);
    let r = angles_reply(&b, &j(r#"{"edges":"all"}"#)).unwrap();
    let es = r.get("edges").unwrap().as_arr().unwrap();
    assert_eq!(es.len(), 12);
    for e in es {
        assert!((num(e, "angle") - 90.0).abs() < 1e-6, "{e:?}");
        assert_eq!(e.get_str("type"), Some("convex"));
    }
    let r = closest_point_reply(&b, &j(r#"{"points":[[-3,2.5,-1],[5,7,-10],[5,2.5,-10]]}"#)).unwrap();
    let rs = r.get("results").unwrap().as_arr().unwrap();
    assert_eq!(rs.len(), 3);
    let d0 = rs[0].get("nearest").unwrap();
    assert!((num(d0, "distance") - 3.0).abs() < 1e-9, "{d0:?}");
    let r = closest_point_reply(&b, &j(r#"{"points":[[-3,-3,3]],"on":"edge"}"#)).unwrap();
    assert!(r.get("results").unwrap().as_arr().unwrap()[0].get("entity").is_some());
    let r = angles_reply(&b, &j(r#"{"draft":{"pull":[0,1,0]}}"#)).unwrap();
    let ds = r.get("draft").unwrap().as_arr().unwrap();
    assert!(ds.iter().any(|d| d.get_str("class") == Some("vertical")));
    assert!(ds.iter().any(|d| (num(d, "draft") - 90.0).abs() < 1e-6));
}

#[test]
fn clearance_and_symmetry() {
    let b = two(box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0), box_body("F3", 13.0, 0.0, 23.0, 10.0, 0.0, 10.0));
    let r = clearance_reply(&b, &j(r#"{"min":4}"#)).unwrap();
    assert!(num(&r, "count") >= 1.0, "{r:?}");
    let p0 = &r.get("pairs").unwrap().as_arr().unwrap()[0];
    assert!((num(p0, "distance") - 3.0).abs() < 1e-9);
    assert_eq!(p0.get_str("kind"), Some("gap"));
    let r = clearance_reply(&b, &j(r#"{"min":2}"#)).unwrap();
    assert_eq!(num(&r, "count"), 0.0);
    let plate = box_body("F2", 0.0, 0.0, 50.0, 30.0, 0.0, 2.0);
    let r = clearance_reply(&plate, &j(r#"{"min":3}"#)).unwrap();
    let p = r.get("pairs").unwrap().as_arr().unwrap();
    assert_eq!(p.len(), 1, "{r:?}");
    assert_eq!(p[0].get_str("kind"), Some("wall"));
    assert!((num(&p[0], "distance") - 2.0).abs() < 1e-9);
    let r = symmetry_reply(&box_body("F2", 0.0, 0.0, 10.0, 20.0, 0.0, 5.0), &j(r#"{}"#)).unwrap();
    assert!(r.get("symmetric").unwrap().as_arr().unwrap().len() >= 3, "{r:?}");
    let b = two(cyl_body("F2", 0.0, 0.0, 5.0, 0.0, 10.0), cyl_body("F3", 20.0, 0.0, 3.0, 0.0, 10.0));
    let r = symmetry_reply(&b, &j(r#"{}"#)).unwrap();
    // equal heights, both axes in the plane z = 0: the y and z mirrors, not the x mirror
    assert_eq!(r.get("symmetric").unwrap().as_arr().unwrap().len(), 2, "{r:?}");
}
