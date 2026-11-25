//! Circular and rectangular patterns and mirrors of features, and the mirror of the whole body. A part's pattern and
//! mirror take feature ids as seeds (the seed's tool body is repeated), so a Fusion pattern maps only when every input
//! is an extrude or revolve that mapped (by the timeline index the file gives), and its axis, direction or plane is
//! given exactly by the dump: the circular pattern's axis is the decoder's `_f3d_axis` (origin and direction), the
//! rectangular pattern's directions are `directionOne` / `directionTwo`, a mirror's plane is the plane's own geometry.
//! Anything the part's pattern cannot say (a partial turn, a symmetric or suppressed pattern, patterns of bodies or faces
//! by the file's names, a component placed away from the origin) stays a placeholder.

use ck_f3d_dump::{CircularPatternDetail, Detail, RectangularPatternDetail, Reference};
use serde_json::{Map, Value, json};

use crate::{Builder, CM_TO_MM, Frame};

fn value(r: &Option<Reference>) -> Option<f64> {
    match r {
        Some(Reference::Parameter(p)) => p.value.filter(|v| v.is_finite()),
        _ => None,
    }
}

fn count(r: &Option<Reference>, what: &str) -> Result<u32, String> {
    let v = value(r).ok_or_else(|| format!("its {what} is not a number"))?;
    if v.fract().abs() > 1e-9 || v < 1.0 || v > 10000.0 {
        return Err(format!("its {what} is not a whole number from 1"));
    }
    Ok(v as u32)
}

fn here(b: &Builder) -> Result<(), String> {
    if b.place.is_some() {
        return Err("it is in a component placed away from the origin".into());
    }
    if b.multi {
        return Err("the design has bodies in several components, and a part's pattern acts on the one body".into());
    }
    if !b.builds {
        return Err("no earlier feature made a body".into());
    }
    Ok(())
}

/// The part feature ids of the inputs: each a mapped extrude or revolve, by the timeline index.
fn seeds(b: &Builder, inputs: &[Reference]) -> Result<Vec<String>, String> {
    if inputs.is_empty() {
        return Err("it has no input".into());
    }
    let mut out = Vec::new();
    for r in inputs {
        let Reference::Feature(f) = r else {
            return Err("it patterns something other than features (bodies and faces are named by the file's own names)".into());
        };
        let Some(Some(i)) = f.timeline_index else {
            return Err("an input feature is not given by a timeline index".into());
        };
        match b.seeds.get(&i) {
            Some(id) => out.push(id.clone()),
            None => {
                return Err(format!(
                    "an input feature ({}) is not an extrude or revolve that mapped, and only those are repeated",
                    f.object_type.as_deref().unwrap_or("?")
                ));
            }
        }
    }
    Ok(out)
}

fn compute(opt: &Option<String>) -> Result<(), String> {
    match opt.as_deref() {
        None | Some("IdenticalPatternCompute" | "AdjustPatternCompute") => Ok(()),
        Some(o) => Err(format!("its compute option {o} is not mapped yet")),
    }
}

fn vec3(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    let g = |i: usize| a.get(i).and_then(Value::as_f64).filter(|x| x.is_finite());
    if a.len() != 3 {
        return None;
    }
    Some([g(0)?, g(1)?, g(2)?])
}

pub(crate) fn circular(b: &mut Builder, name: &str, d: &CircularPatternDetail) -> Result<Option<String>, String> {
    if d.pattern_entity_type.as_deref().is_some_and(|t| t != "FeaturesPatternType") {
        return Err("a circular pattern of bodies or faces is not mapped yet".into());
    }
    here(b)?;
    let seeds = seeds(b, d.input_entities.as_deref().unwrap_or(&[]))?;
    compute(&d.pattern_compute_option)?;
    let n = count(&d.quantity, "quantity")?;
    if n < 2 {
        return Err("its quantity is 1".into());
    }
    let angle = value(&d.total_angle).ok_or("its angle is not a number")?;
    if (angle.abs() - std::f64::consts::TAU).abs() > 1e-9 {
        return Err("a circular pattern over a partial turn is not mapped yet".into());
    }
    if d.is_symmetric == Some(true) {
        return Err("a symmetric circular pattern is not mapped yet".into());
    }
    if d.suppressed_elements_ids.as_ref().is_some_and(|s| !s.is_empty()) {
        return Err("a circular pattern with suppressed instances is not mapped yet".into());
    }
    let ax = d.other.get("_f3d_axis").and_then(Value::as_object).ok_or("the file gives no axis for it")?;
    let o = ax.get("origin").and_then(vec3).ok_or("its axis has no origin")?;
    let dir = ax.get("direction").and_then(vec3).ok_or("its axis has no direction")?;
    let id = b.next_id();
    b.features.push(json!({
        "type": "cpattern", "id": id, "name": name, "seeds": seeds,
        "axis": {"kind": "axis", "point": o.map(|v| v * CM_TO_MM), "dir": dir},
        "count": n, "angle": 360.0, "equal": true,
    }));
    Ok(None)
}

pub(crate) fn rectangular(b: &mut Builder, name: &str, d: &RectangularPatternDetail) -> Result<Option<String>, String> {
    if d.pattern_entity_type.as_deref().is_some_and(|t| t != "FeaturesPatternType") {
        return Err("a rectangular pattern of bodies or faces is not mapped yet".into());
    }
    here(b)?;
    let seeds = seeds(b, d.input_entities.as_deref().unwrap_or(&[]))?;
    compute(&d.pattern_compute_option)?;
    if d.is_symmetric_in_direction_one == Some(true) || d.is_symmetric_in_direction_two == Some(true) {
        return Err("a symmetric rectangular pattern is not mapped yet".into());
    }
    if d.suppressed_elements_ids.as_ref().is_some_and(|s| !s.is_empty()) {
        return Err("a rectangular pattern with suppressed instances is not mapped yet".into());
    }
    let kind = match d.pattern_distance_type.as_deref() {
        Some("ExtentPatternDistanceType") => "extent",
        Some("SpacingPatternDistanceType") => "spacing",
        _ => return Err("its distance type is not known".into()),
    };
    let (q1, q2) = (count(&d.quantity_one, "quantity")?, count(&d.quantity_two, "second quantity").unwrap_or(1));
    if q1 < 2 {
        return Err("its first quantity is 1".into());
    }
    let mut item = json!({"type": "lpattern", "name": name, "seeds": seeds});
    let rows = [(d.direction_one, &d.distance_one, q1, ""), (d.direction_two, &d.distance_two, q2, "2")];
    for (dir, dist, q, sfx) in rows {
        if q < 2 {
            continue;
        }
        let dir = dir.ok_or("it has no direction")?;
        let dist = value(dist).filter(|v| *v > 0.0).ok_or("its distance is not a positive number")? * CM_TO_MM;
        item[format!("dir{sfx}")] = json!({"kind": "vector", "dir": dir});
        item[format!("count{sfx}")] = json!(q);
        item[format!("distanceType{sfx}")] = json!(kind);
        item[format!("{kind}{sfx}")] = json!(dist);
    }
    item["id"] = json!(b.next_id());
    b.features.push(item);
    Ok(None)
}

/// A mirror (`MirrorFeature` has no typed detail: the map is read as is).
pub(crate) fn mirror(b: &mut Builder, name: &str, m: &Map<String, Value>) -> Result<Option<String>, String> {
    let plane = m.get("mirrorPlane").and_then(Value::as_object).ok_or("it has no mirror plane")?;
    let g = plane.get("geometry").and_then(Value::as_object).filter(|g| g.get("type").and_then(Value::as_str) == Some("Plane"));
    let g = g.ok_or("its mirror plane is not given by a plane's geometry")?;
    let o = g.get("origin").and_then(vec3).ok_or("its plane has no origin")?.map(|v| v * CM_TO_MM);
    let u = g.get("uDirection").and_then(vec3).ok_or("its plane has no direction")?;
    let v = g.get("vDirection").and_then(vec3).ok_or("its plane has no direction")?;
    let frame = Frame::new(o, u, v).ok_or("its plane is not a plane")?;
    let inputs: Vec<Reference> = m
        .get("inputEntities")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(|x| x.as_object().map(|o| Reference::from_map(o.clone()))).collect())
        .unwrap_or_default();
    let is_features = match m.get("patternEntityType").and_then(Value::as_str) {
        Some("FeaturesPatternType") => true,
        Some("BodiesPatternType") => false,
        _ => return Err("a mirror of faces or of something not known is not mapped yet".into()),
    };
    if b.place.is_some() {
        return Err("it is in a component placed away from the origin".into());
    }
    if is_features {
        here(b)?;
        let seeds = seeds(b, &inputs)?;
        let plane = b.plane_for(name, &frame);
        let id = b.next_id();
        b.features.push(json!({"type": "mirror", "id": id, "name": name, "seeds": seeds, "plane": plane, "body": false}));
        return Ok(None);
    }
    // the mirror of the one body the design has, joined to it (the part's `body` mirror reflects the whole body)
    if b.multi || b.lost || !b.builds || b.new_bodies > 0 {
        return Err("a mirror of bodies is mapped only for a design with one body and nothing unmapped before it".into());
    }
    if inputs.len() != 1 || !matches!(inputs[0], Reference::Body(_)) {
        return Err("a mirror of bodies is mapped only for the design's one body".into());
    }
    if m.get("isCombine").and_then(Value::as_bool) != Some(true) {
        return Err("a mirror of a body that stays a body of its own is not mapped yet".into());
    }
    let plane = b.plane_for(name, &frame);
    let id = b.next_id();
    b.features.push(json!({"type": "mirror", "id": id, "name": name, "seeds": [], "plane": plane, "body": true}));
    Ok(None)
}

/// The detail of a mirror item, when it is one.
pub(crate) fn mirror_map(d: &Detail) -> Option<&Map<String, Value>> {
    match d {
        Detail::Other(m) => Some(m),
        _ => None,
    }
}

/// A Move of the design's one body (`MoveFeature`, a free move or a translation, given as its 4 x 4 matrix, cm): the
/// part's `moveBody` with no body named moves the whole body, rotation first and then the translation, as the matrix
/// does. A move of several bodies, or in a design with several, is named by the file's own body names and stays a placeholder.
pub(crate) fn move_one_body(b: &mut Builder, name: &str, m: &Map<String, Value>) -> Result<Option<String>, String> {
    if b.named {
        return crate::bodies::move_named(b, name, m);
    }
    if b.place.is_some() {
        return Err("it is in a component placed away from the origin".into());
    }
    if b.multi || b.lost || !b.builds || b.new_bodies > 0 {
        return Err("a move is mapped only for a design with one body and nothing unmapped before it".into());
    }
    let inputs = m.get("inputEntities").and_then(Value::as_array).map_or(&[][..], Vec::as_slice);
    if inputs.len() != 1 || inputs[0].get("kind").and_then(Value::as_str) != Some("body") {
        return Err("a move of something other than the design's one body is not mapped yet".into());
    }
    let rows = m.get("transform").and_then(Value::as_array).filter(|r| r.len() == 4).ok_or("it has no transform")?;
    let mut mat = [[0.0f64; 4]; 4];
    for (i, row) in rows.iter().enumerate() {
        let row = row.as_array().filter(|r| r.len() == 4).ok_or("its transform is not a 4 x 4 matrix")?;
        for (j, v) in row.iter().enumerate() {
            mat[i][j] = v.as_f64().filter(|v| v.is_finite()).ok_or("its transform is not numbers")?;
        }
    }
    let Some(p) = crate::Placement::from_mat(&mat)? else {
        return Ok(Some("it moves nothing".into()));
    };
    let id = b.next_id();
    let mut f = json!({"type": "moveBody", "id": id, "name": name, "translate": p.t});
    if let Some((axis, angle)) = p.axis_angle() {
        f["rotate"] = json!({"axis": axis, "origin": [0.0, 0.0, 0.0], "angle": angle});
    }
    b.features.push(f);
    Ok(None)
}
