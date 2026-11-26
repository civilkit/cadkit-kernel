//! Sketches: a Fusion sketch (points and curves in sketch space, cm, and the sketch's model frame) as a ck-part `sketch`
//! on a standard plane or on a `plane` feature given by its frame. Points, lines, arcs and circles map; constraints
//! and dimensions are not carried (the geometry is already at its solved positions). The port of the idea follows
//! Dutchman's `dutchman-ckpart` sketch writer (Apache-2.0): a plane is a standard one when the frame is, else a frame
//! plane added first, an arc runs counter-clockwise from its first point to its second.

use std::collections::{BTreeMap, BTreeSet};

use ck_asm::vec3::dot;
use ck_f3d_dump::{SketchCurve, SketchDetail};
use serde_json::{Value, json};

use crate::{Builder, CM_TO_MM, Frame};

/// What later features need of a sketch that was mapped.
#[derive(Clone, Debug)]
pub(crate) struct SketchInfo {
    /// The ck feature id (`F3`).
    pub id: String,
    /// The plane reference the sketch is on.
    pub plane: Value,
    pub frame: Frame,
    /// `z_axis` of the Fusion sketch (its normal) relative to the ck frame's normal: 1 or -1.
    pub normal_sign: f64,
    /// Curve ids (Fusion `c<i>`) that are not construction.
    pub curves: BTreeSet<String>,
    /// Every line: id -> (point id, point id).
    pub lines: BTreeMap<String, (String, String)>,
    /// Every sketch point, in sketch space (mm).
    pub points: BTreeMap<String, [f64; 2]>,
    /// The outline (entity ids) of every region the kernel finds in the sketch.
    pub regions: Vec<BTreeSet<String>>,
    /// The entity ids of every hole loop of those regions (an id of one picks the hole's filled disc).
    pub holes: Vec<BTreeSet<String>>,
    /// A point well inside each of `regions` (same order), when exactly one region of the kernel's has it inside:
    /// the part's `regionAt` pick of a region no entity id picks alone.
    pub region_points: Vec<Option<[f64; 2]>>,
}

/// Maps a sketch; the error says why it does not (nothing is added to the builder then).
pub(crate) fn map(b: &mut Builder, name: &str, d: &SketchDetail) -> Result<SketchInfo, String> {
    if d.is_3d == Some(true) {
        return Err("a 3D sketch is not mapped yet".into());
    }
    let frame = b.world(frame_of(d)?);
    let mut points = Vec::new();
    let mut positions = BTreeMap::new();
    let mut seen = BTreeSet::new();
    let known = geometry_points(d);
    for p in d.points.as_deref().unwrap_or(&[]) {
        let id = p.id.clone().ok_or("a sketch point has no id")?;
        let mut at = p.xyz.ok_or("a sketch point has no position")?;
        if !sane(at) {
            // the decoder did not read this point; a curve that ends or is centred on it may know where it is
            at = known.get(&id).copied().filter(|a| sane(*a)).ok_or("a sketch point has no usable position")?;
        }
        if !seen.insert(id.clone()) {
            return Err(format!("the point name {id} is used twice"));
        }
        let (x, y) = (at[0] * CM_TO_MM, at[1] * CM_TO_MM);
        let mut item = json!({"id": id, "x": x, "y": y});
        if p.is_fixed == Some(true) {
            item["fixed"] = json!(true);
        }
        points.push(item);
        positions.insert(id, [x, y]);
    }
    let mut entities = Vec::new();
    let mut curve_ids = BTreeSet::new();
    let mut lines = BTreeMap::new();
    let mut taken: BTreeSet<String> = seen.clone();
    for c in d.curves.as_deref().unwrap_or(&[]) {
        let id = c.id.clone().ok_or("a sketch curve has no id")?;
        if !taken.insert(id.clone()) {
            return Err(format!("the curve name {id} is used twice"));
        }
        let kind = c.curve_type.as_deref().unwrap_or("");
        let construction = c.is_construction == Some(true) || c.is_center_line == Some(true);
        let mut item = match kind {
            "SketchLine" => {
                let (a, e) = (end(c, &c.start_sketch_point, &positions)?, end(c, &c.end_sketch_point, &positions)?);
                lines.insert(id.clone(), (a.clone(), e.clone()));
                json!({"type": "line", "id": id, "p1": a, "p2": e})
            }
            "SketchArc" => {
                let centre = end(c, &c.center_sketch_point, &positions)?;
                let (mut a, mut e) = (end(c, &c.start_sketch_point, &positions)?, end(c, &c.end_sketch_point, &positions)?);
                // an arc runs counter-clockwise about its own normal; CadKit's about the sketch's +z
                let normal_z = c.geometry.as_ref().and_then(|g| g.normal).map_or(1.0, |n| n[2]);
                if normal_z < 0.0 {
                    std::mem::swap(&mut a, &mut e);
                }
                json!({"type": "arc", "id": id, "c": centre, "p1": a, "p2": e})
            }
            "SketchCircle" => {
                let centre = end(c, &c.center_sketch_point, &positions)?;
                let r = c.radius.filter(|r| *r > 0.0 && r.is_finite()).ok_or("a circle has no radius")?;
                json!({"type": "circle", "id": id, "c": centre, "r": r * CM_TO_MM})
            }
            "SketchFittedSpline" => {
                let pts: Vec<String> = c
                    .fit_points
                    .clone()
                    .unwrap_or_default()
                    .into_iter()
                    .collect();
                if pts.len() < 2 {
                    return Err("a fitted spline has fewer than two fit points".into());
                }
                for p in &pts {
                    if !positions.contains_key(p) {
                        return Err(format!("a spline names the point {p}, which the sketch does not have"));
                    }
                }
                let mut j = json!({"type": "spline", "id": id, "pts": pts});
                if c.is_closed == Some(true) {
                    j["closed"] = json!(true);
                }
                j
            }
            "SketchFixedSpline" | "SketchControlPointSpline" => {
                // a B-spline given by its control points, degree and knots (the part's `cvspline`), in sketch space
                let nurbs = c.geometry.as_ref().and_then(|g| g.nurbs.as_ref()).and_then(Value::as_object);
                let nurbs = nurbs.ok_or_else(|| format!("a {} has no control points in the file", kind.trim_start_matches("Sketch")))?;
                let degree = nurbs.get("degree").and_then(Value::as_u64).filter(|d| (1..=9).contains(d)).ok_or("a spline has no usable degree")?;
                if nurbs.get("isPeriodic").and_then(Value::as_bool) == Some(true) {
                    return Err("a periodic spline is not mapped yet".into());
                }
                // a rational spline carries one weight per control point (the part's cvspline `weights`)
                let rational = nurbs.get("isRational").and_then(Value::as_bool) == Some(true);
                let weights: Option<Vec<f64>> = if rational {
                    let w: Option<Vec<f64>> = nurbs
                        .get("weights")
                        .and_then(Value::as_array)
                        .map(|a| a.iter().filter_map(Value::as_f64).collect::<Vec<_>>())
                        .filter(|w| w.len() == nurbs.get("controlPoints").and_then(Value::as_array).map_or(usize::MAX, Vec::len));
                    match w {
                        Some(w) if w.iter().all(|x| *x > 0.0 && x.is_finite()) => Some(w),
                        _ => return Err("a rational spline has no usable weights".into()),
                    }
                } else {
                    None
                };
                let cps: Vec<[f64; 3]> = nurbs
                    .get("controlPoints")
                    .and_then(Value::as_array)
                    .map(|a| {
                        a.iter()
                            .map(|p| {
                                let p = p.as_array()?;
                                let g = |i: usize| p.get(i).and_then(Value::as_f64).filter(|v| v.is_finite());
                                Some([g(0)?, g(1)?, g(2).unwrap_or(0.0)])
                            })
                            .collect::<Option<Vec<_>>>()
                    })
                    .unwrap_or(None)
                    .ok_or("a spline's control points are not numbers")?;
                if cps.len() <= degree as usize || cps.iter().any(|p| p[2].abs() > 1e-4 || p[0].abs() > 1e5 || p[1].abs() > 1e5) {
                    return Err("a spline's control points are not in the sketch plane".into());
                }
                let knots: Vec<f64> = nurbs
                    .get("knots")
                    .and_then(Value::as_array)
                    .map(|a| a.iter().filter_map(Value::as_f64).collect())
                    .unwrap_or_default();
                if knots.len() != cps.len() + degree as usize + 1 || knots.windows(2).any(|w| w[1] < w[0]) || knots[0] >= knots[knots.len() - 1] {
                    return Err("a spline's knots do not fit its control points".into());
                }
                let mut ids = Vec::with_capacity(cps.len());
                for (k, p) in cps.iter().enumerate() {
                    let pid = format!("{id}_cv{k}");
                    if !taken.insert(pid.clone()) {
                        return Err(format!("the point name {pid} is used twice"));
                    }
                    points.push(json!({"id": pid, "x": p[0] * CM_TO_MM, "y": p[1] * CM_TO_MM}));
                    ids.push(pid);
                }
                let mut j = json!({"type": "cvspline", "id": id, "pts": ids, "degree": degree, "knots": knots});
                if let Some(w) = weights {
                    j["weights"] = json!(w);
                }
                j
            }
            "SketchEllipse" => {
                let centre = end(c, &c.center_sketch_point, &positions)?;
                let at = positions[&centre];
                let (a, r) = (
                    c.major_axis_radius.filter(|v| *v > 0.0 && v.is_finite()),
                    c.minor_axis_radius.filter(|v| *v > 0.0 && v.is_finite()),
                );
                let (a, r) = a.zip(r).ok_or("an ellipse has no radii")?;
                let dir = c.major_axis.or_else(|| c.geometry.as_ref().and_then(|g| g.major_axis)).ok_or("an ellipse has no major axis")?;
                let n = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
                if n < 1e-12 || !n.is_finite() {
                    return Err("an ellipse has no major axis".into());
                }
                // the end of the major axis as a sketch point of its own
                let pid = format!("{id}_major");
                if !taken.insert(pid.clone()) {
                    return Err(format!("the point name {pid} is used twice"));
                }
                let end_pt = [at[0] + dir[0] / n * a * CM_TO_MM, at[1] + dir[1] / n * a * CM_TO_MM];
                points.push(json!({"id": pid, "x": end_pt[0], "y": end_pt[1]}));
                positions.insert(pid.clone(), end_pt);
                json!({"type": "ellipse", "id": id, "c": centre, "p": pid, "r": r * CM_TO_MM})
            }
            other => {
                return Err(format!(
                    "its {} curves are not mapped yet",
                    if other.is_empty() { "unnamed" } else { other.trim_start_matches("Sketch") }
                ));
            }
        };
        if construction {
            item["construction"] = json!(true);
        } else {
            curve_ids.insert(id);
        }
        entities.push(item);
    }

    let plane = b.plane_for(name, &frame);
    let normal_sign = {
        let n = frame.normal();
        let z = d.model_frame.as_ref().and_then(|m| m.z_axis).unwrap_or(n);
        if dot(n, z) < 0.0 { -1.0 } else { 1.0 }
    };
    let id = b.next_id();
    let mut feature = json!({
        "type": "sketch", "id": id, "name": name, "plane": plane.clone(),
        "points": points, "entities": entities, "constraints": [],
    });
    if d.is_visible == Some(false) {
        feature["visible"] = json!(false);
    }
    // Fusion cuts two lines that run along one another at each other's ends (a notch drawn on an edge of a plate splits the
    // edge); the part's region finder leaves collinear overlaps alone unless told to
    if overlapping_lines(&lines, &positions, &curve_ids) {
        feature["splitOverlaps"] = json!(true);
    }
    b.features.push(feature);
    let (regions, holes, region_points) = region_outlines(&b.features[b.features.len() - 1]);
    Ok(SketchInfo { id, plane, frame, normal_sign, curves: curve_ids, lines, points: positions, regions, holes, region_points })
}

/// Whether two (non-construction) lines of a sketch lie along one another over a stretch of positive length.
fn overlapping_lines(lines: &BTreeMap<String, (String, String)>, at: &BTreeMap<String, [f64; 2]>, solid: &BTreeSet<String>) -> bool {
    let segs: Vec<([f64; 2], [f64; 2])> = lines
        .iter()
        .filter(|(id, _)| solid.contains(*id))
        .filter_map(|(_, (a, c))| Some((*at.get(a)?, *at.get(c)?)))
        .collect();
    for (i, (p, q)) in segs.iter().enumerate() {
        let d = [q[0] - p[0], q[1] - p[1]];
        let len = d[0].hypot(d[1]);
        if len < 1e-9 {
            continue;
        }
        for (r, s) in &segs[i + 1..] {
            let e = [s[0] - r[0], s[1] - r[1]];
            let lb = e[0].hypot(e[1]);
            if lb < 1e-9 || (d[0] * e[1] - d[1] * e[0]).abs() > 1e-9 * len * lb {
                continue;
            }
            // the other line is on this one's line, and the two share more than a point
            if ((r[0] - p[0]) * d[1] - (r[1] - p[1]) * d[0]).abs() / len > 5e-7 {
                continue;
            }
            let t = |x: [f64; 2]| ((x[0] - p[0]) * d[0] + (x[1] - p[1]) * d[1]) / (len * len);
            let (t0, t1) = (t(*r), t(*s));
            let tol = 1e-6 / len;
            if t0.max(t1) > tol && t0.min(t1) < 1.0 - tol {
                return true;
            }
        }
    }
    false
}

/// The id of a curve's end point, which must be a point of the sketch.
fn end(c: &SketchCurve, key: &Option<Option<String>>, points: &BTreeMap<String, [f64; 2]>) -> Result<String, String> {
    let id = key.clone().flatten().ok_or_else(|| format!("curve {} has no end point", c.id.as_deref().unwrap_or("?")))?;
    if points.contains_key(&id) {
        Ok(id)
    } else {
        Err(format!("a curve names the point {id}, which the sketch does not have"))
    }
}

/// The sketch's frame in model space (mm): the model frame, else the origin and axes.
fn frame_of(d: &SketchDetail) -> Result<Frame, String> {
    let m = d.model_frame.as_ref();
    let origin = m.and_then(|m| m.origin).or(d.origin).ok_or("it has no frame")?;
    let x = m.and_then(|m| m.x_axis).or(d.x_direction).ok_or("it has no frame")?;
    let y = m.and_then(|m| m.y_axis).or(d.y_direction).ok_or("it has no frame")?;
    Frame::new(origin.map(|v| v * CM_TO_MM), x, y).ok_or_else(|| "its frame has a zero axis".to_string())
}

/// Whether a point is a plausible one (cm): finite and within a kilometre of the origin.
fn sane(at: [f64; 3]) -> bool {
    at.iter().all(|v| v.is_finite() && v.abs() < 1e5)
}

/// Points the curves' own geometry places: a line's ends, an arc's or circle's centre (cm).
fn geometry_points(d: &SketchDetail) -> BTreeMap<String, [f64; 3]> {
    let mut found = BTreeMap::new();
    for c in d.curves.as_deref().unwrap_or(&[]) {
        let Some(g) = c.geometry.as_ref() else { continue };
        let mut put = |id: &Option<Option<String>>, at: Option<[f64; 3]>| {
            if let (Some(Some(id)), Some(at)) = (id, at) {
                found.entry(id.clone()).or_insert(at);
            }
        };
        match c.curve_type.as_deref() {
            Some("SketchLine") => {
                put(&c.start_sketch_point, g.start_point);
                put(&c.end_sketch_point, g.end_point);
            }
            Some("SketchArc" | "SketchCircle") => put(&c.center_sketch_point, g.center),
            _ => {}
        }
    }
    found
}

/// The outline of each region the kernel finds in a sketch feature (entity ids of its outer loop), regions and fills,
/// and the entity ids of each hole loop.
#[allow(clippy::type_complexity)]
fn region_outlines(feature: &Value) -> (Vec<BTreeSet<String>>, Vec<BTreeSet<String>>, Vec<Option<[f64; 2]>>) {
    let Ok(j) = ck_json::parse(&feature.to_string()) else { return (Vec::new(), Vec::new(), Vec::new()) };
    let sk = ck_model::sketch::Sketch::from_json(&j);
    let p = ck_model::sketch::profile::profiles(&sk);
    let outers = p.regions.iter().chain(p.fills.iter()).map(|r| r.outer.iter().map(|s| s.tag.clone()).collect()).collect();
    let holes = p.regions.iter().chain(p.fills.iter()).flat_map(|r| r.holes.iter()).map(|h| h.iter().map(|s| s.tag.clone()).collect()).collect();
    let all: Vec<&ck_model::sketch::profile::Region> = p.regions.iter().chain(p.fills.iter()).collect();
    let points = all
        .iter()
        .map(|r| r.interior_point().filter(|pt| all.iter().filter(|o| o.contains(*pt)).count() == 1))
        .collect();
    (outers, holes, points)
}
