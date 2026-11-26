//! Extrudes, revolves and construction planes. The extent and operation mapping follows Dutchman's `dutchman-ckpart`
//! `solid.rs` (Apache-2.0): a cut runs against its sketch normal in a part unless flipped, a symmetric extrude is `mid`
//! with the full length as `depth`, a through-all cut is `through` (`throughBoth` both ways), and a first `newBody` is
//! an `add`.

use std::collections::BTreeSet;

use ck_asm::vec3::{dot, unit};
use ck_f3d_dump::Vec3;
use ck_f3d_dump::{ConstructionPlaneDetail, Definition, ExtrudeDetail, Reference, RevolveDetail};
use serde_json::{Map, Value, json};

use crate::sketch::SketchInfo;
use crate::{Builder, CM_TO_MM, Frame};

/// Angles closer to a full turn than this (radians) are one.
const FULL_TURN: f64 = 1e-6;

/// The value of a parameter reference (internal units: cm, rad).
fn value(r: &Option<Reference>) -> Option<f64> {
    match r {
        Some(Reference::Parameter(p)) => p.value.filter(|v| v.is_finite()),
        _ => None,
    }
}

fn kind_is(d: &Option<Definition>, name: &str) -> bool {
    d.as_ref().and_then(|d| d.definition_type.as_deref()) == Some(name)
}

/// What a feature does to the body: the part's `op`, the name of the body it makes (a new body) and the body it acts
/// on (`target`, when the design has bodies in several components and the component has exactly one).
struct Op {
    op: &'static str,
    body: Option<String>,
    target: Option<Value>,
}

/// `Join`, `Cut`, `NewBody` of an operation name; `None` for the others.
fn operation(b: &Builder, op: Option<&str>, participants: &[Reference]) -> Result<Op, String> {
    let plain = |op| Op { op, body: None, target: None };
    if b.named && !b.multi {
        return named_operation(b, op, participants);
    }
    if !b.multi {
        return match op {
            Some("JoinFeatureOperation") | None => Ok(plain("add")),
            Some("CutFeatureOperation") if !b.builds => Err("it cuts, but no earlier feature made a body".into()),
            Some("CutFeatureOperation") => Ok(plain("cut")),
            Some("NewBodyFeatureOperation") if !b.builds => Ok(plain("add")),
            Some("NewBodyFeatureOperation") => Ok(plain("newBody")),
            Some("IntersectFeatureOperation") => Err("an intersecting feature is not mapped yet".into()),
            Some(other) => Err(format!("the operation {other} is not mapped yet")),
        };
    }
    // bodies of several components: each component's first body is a body of its own, named after it
    let have = b.comp_bodies.get(&b.comp).copied().unwrap_or(0);
    let own = format!("{} body", b.comp);
    let new = |n: usize| Op { op: "newBody", body: Some(if n == 0 { own.clone() } else { format!("{own} {}", n + 1) }), target: None };
    // an add or cut acts on its own component's bodies only (every one when it has several: a join with no target would
    // fuse with a body of a component next to it)
    // (a body a combine consumed is no longer one)
    let alive: Vec<String> = (0..have)
        .map(|k| if k == 0 { own.clone() } else { format!("{own} {}", k + 1) })
        .filter(|n| !b.consumed.contains(n))
        .collect();
    let target = match alive.len() {
        0 => None,
        1 => Some(json!({"body": alive[0]})),
        _ => Some(Value::Array(alive.iter().map(|n| json!({"body": n})).collect())),
    };
    match op {
        Some("JoinFeatureOperation") | None if have == 0 => Ok(new(0)),
        Some("JoinFeatureOperation") | None if alive.is_empty() => Ok(new(have)),
        Some("JoinFeatureOperation") | None => Ok(Op { op: "add", body: None, target }),
        Some("CutFeatureOperation") if alive.is_empty() => Err("it cuts, but no earlier feature of its component made a body".into()),
        Some("CutFeatureOperation") => Ok(Op { op: "cut", body: None, target }),
        Some("NewBodyFeatureOperation") => Ok(new(have)),
        Some("IntersectFeatureOperation") => Err("an intersecting feature is not mapped yet".into()),
        Some(other) => Err(format!("the operation {other} is not mapped yet")),
    }
}

/// The operation in a design whose bodies are named (`Builder::named`): a body a feature makes is `B<item>`; a join or cut
/// acts on the participant bodies the file lists when the history names every one of them, else on whatever it touches.
fn named_operation(b: &Builder, op: Option<&str>, participants: &[Reference]) -> Result<Op, String> {
    let own = format!("B{}", b.cur);
    let new = || Op { op: "newBody", body: Some(own.clone()), target: None };
    let target = crate::bodies::participants(b, participants);
    match op {
        Some("JoinFeatureOperation") | None if !b.builds => Ok(new()),
        Some("NewBodyFeatureOperation") => Ok(new()),
        Some("JoinFeatureOperation") | None => Ok(Op { op: "add", body: None, target }),
        Some("CutFeatureOperation") if !b.builds => Err("it cuts, but no earlier feature made a body".into()),
        Some("CutFeatureOperation") => Ok(Op { op: "cut", body: None, target }),
        Some("IntersectFeatureOperation") => Err("an intersecting feature is not mapped yet".into()),
        Some(other) => Err(format!("the operation {other} is not mapped yet")),
    }
}

/// Writes `op`'s fields on a feature and counts the body it makes.
fn finish(b: &mut Builder, item: &mut Value, op: &Op) {
    item["op"] = json!(op.op);
    if let Some(n) = &op.body {
        item["body"] = json!(n);
    }
    if let Some(t) = &op.target {
        item["target"] = t.clone();
    }
    if op.op == "newBody" {
        b.new_bodies += 1;
        // a named design: the body is the one the file's (item, 0) names (in bodies of several components the part's name
        // is the component's, and a component's later bodies have their own)
        if b.named && let Some(n) = &op.body {
            b.bodies.insert((b.cur, 0), (n.clone(), b.comp.clone()));
        }
    }
    if b.multi && op.op == "newBody" {
        *b.comp_bodies.entry(b.comp.clone()).or_default() += 1;
    }
}

/// The profile references of a feature: a list, or the one reference some dumps write in place of a list.
fn profile_refs(typed: &Option<Vec<Reference>>, other: &Map<String, Value>) -> Vec<Reference> {
    if let Some(list) = typed {
        return list.clone();
    }
    match other.get("profile") {
        Some(Value::Object(m)) => vec![Reference::from_map(m.clone())],
        Some(Value::Array(a)) => a
            .iter()
            .filter_map(|v| v.as_object().map(|m| Reference::from_map(m.clone())))
            .collect(),
        _ => Vec::new(),
    }
}

/// How the regions are picked: entity ids (`regions`) and points inside the regions no id picks alone (`regionAt`);
/// both empty is every region of the sketch.
#[derive(Default)]
pub(crate) struct Picks {
    ids: Vec<String>,
    at: Vec<[f64; 2]>,
}

impl Picks {
    fn write(&self, item: &mut Value) {
        if !self.ids.is_empty() {
            item["regions"] = json!(self.ids);
        }
        if !self.at.is_empty() {
            item["regionAt"] = json!(self.at);
        }
    }
}

/// The sketch the profiles belong to, and the regions to pick (empty: every region of the sketch).
fn selection<'a>(
    b: &'a Builder,
    refs: &[Reference],
    other: &Map<String, Value>,
) -> Result<(&'a SketchInfo, Picks), String> {
    if refs.is_empty() {
        return Err("it has no profile".into());
    }
    let mut found: BTreeSet<i64> = BTreeSet::new();
    for r in refs {
        let Reference::Profile(p) = r else {
            return Err("its profile is not a sketch profile (a face, say), which is not mapped yet".into());
        };
        let index = p
            .sketch_timeline_index
            .flatten()
            .or_else(|| p.sketch.clone().flatten().and_then(|n| b.sketch_by_name.get(&n).copied()));
        let Some(index) = index else {
            return Err("it names no sketch".into());
        };
        found.insert(index);
    }
    if found.len() != 1 {
        return Err("its profiles are in more than one sketch, which is not mapped yet".into());
    }
    let index = *found.iter().next().ok_or("it names no sketch")?;
    let sketch = b.sketches.get(&index).ok_or("its sketch is not in the part")?;
    let regions = regions(other.get("_f3d_profile_loops"), sketch)?;
    Ok((sketch, regions))
}

/// The ids for `regions` that pick the profiles the file names: one curve of each profile's outer loop, a curve no
/// other chosen outline uses (a part selects every region whose outline carries the id). With no loops known the file
/// said only "this sketch": every region is taken.
fn regions(loops: Option<&Value>, sketch: &SketchInfo) -> Result<Picks, String> {
    let Some(profiles) = loops.and_then(Value::as_array) else {
        return Ok(Picks::default());
    };
    if profiles.is_empty() {
        return Ok(Picks::default());
    }
    // per profile: the ids of its outer loop's curves. A profile the file builds of several regions (a chord across a
    // region makes two) lists them as `children` beside the merged `loops`: each child is a region of its own.
    let outer_of = |loops: &[Value]| -> Result<Vec<String>, String> {
        let mut outer: Vec<String> = Vec::new();
        let mut known = false;
        for l in loops {
            if l.get("outer").and_then(Value::as_bool) != Some(true) {
                continue;
            }
            known = true;
            for c in l.get("curves").and_then(Value::as_array).into_iter().flatten() {
                if let Some(id) = c.get("id").and_then(Value::as_str) {
                    outer.push(id.to_owned());
                }
            }
        }
        if !known || outer.is_empty() {
            return Err("it names a profile whose outline is not known".into());
        }
        Ok(outer)
    };
    let mut outlines: Vec<Vec<String>> = Vec::new();
    for p in profiles {
        let children = p.get("children").and_then(Value::as_array).filter(|c| c.len() > 1);
        match children {
            Some(children) => {
                for child in children {
                    let loops = child.as_array().ok_or("it names a profile whose outline is not known")?;
                    outlines.push(outer_of(loops)?);
                }
            }
            None => outlines.push(outer_of(p.get("loops").and_then(Value::as_array).map_or(&[][..], |v| &v[..]))?),
        }
    }
    // Each chosen profile is the kernel region with the same outline; a curve only that region's outline carries picks
    // exactly it (a part selects every region whose outline carries the id)
    let mut ids = Vec::new();
    let mut at: Vec<[f64; 2]> = Vec::new();
    let mut picked: BTreeSet<usize> = BTreeSet::new();
    let mut disc = false;
    for outline in &outlines {
        let set: BTreeSet<&String> = outline.iter().collect();
        // the region with the same outline, else the one that shares the most curves with it (the file's loops
        // and the kernel's regions can cut a curve differently)
        let score = |r: &BTreeSet<String>| {
            let common = r.iter().filter(|c| set.contains(c)).count() as f64;
            common / (r.len() + set.len()) as f64 * 2.0
        };
        let scored: Vec<(usize, f64)> = sketch.regions.iter().enumerate().map(|(i, r)| (i, score(r))).filter(|(_, sc)| *sc >= 0.5).collect();
        let best = scored.iter().copied().max_by(|a, b| a.1.total_cmp(&b.1));
        // regions that are all outlined by the same curves (the lens and the crescents of two circles) cannot be told
        // apart by an outline of ids: the file's profile may be any of them
        if let Some((_, top)) = best
            && scored.iter().filter(|(_, sc)| *sc >= top - 1e-9).count() > 1
        {
            let tied: Vec<usize> = scored.iter().filter(|(_, sc)| *sc >= top - 1e-9).map(|(i, _)| *i).collect();
            // the file picks as many profiles with this outline as the kernel has regions with it: every one of them
            // is picked (each by a point inside it), whichever is which
            let same = outlines.iter().filter(|o| o.iter().collect::<BTreeSet<_>>() == set).count();
            let points: Option<Vec<[f64; 2]>> = tied.iter().map(|i| sketch.region_points.get(*i).copied().flatten()).collect();
            match points {
                Some(points) if same == tied.len() => {
                    for p in points {
                        if !at.contains(&p) {
                            at.push(p);
                        }
                    }
                    picked.extend(tied);
                    continue;
                }
                _ => return Err("a profile's outline is the outline of several regions of the sketch, which curve ids cannot tell apart".into()),
            }
        }
        let Some((region, _)) = best else {
            // not a region of the kernel's: perhaps the disc inside a hole loop (an island), which an id of the hole
            // picks and which no id of any region's outer loop must also pick
            let hole = sketch
                .holes
                .iter()
                .map(|h| (h, score(h)))
                .filter(|(_, sc)| *sc >= 0.5)
                .max_by(|a, b| a.1.total_cmp(&b.1));
            let id = hole.and_then(|(h, _)| {
                h.iter().find(|id| sketch.curves.contains(*id) && !sketch.regions.iter().any(|r| r.contains(*id)))
            });
            match id {
                Some(id) => {
                    disc = true;
                    if !ids.contains(id) {
                        ids.push(id.clone());
                    }
                    continue;
                }
                None => {
                    if std::env::var("CK_F3D_DEBUG").is_ok() {
                        eprintln!("profile outline {outline:?} vs kernel regions {:?} holes {:?}", sketch.regions, sketch.holes);
                    }
                    return Err("a profile's outline is not a region the kernel finds in the sketch".into());
                }
            }
        };
        if std::env::var("CK_F3D_DEBUG").is_ok() {
            eprintln!("profile outline {outline:?} -> region {:?} (of {} regions)", sketch.regions[region], sketch.regions.len());
        }
        picked.insert(region);
        let unique = sketch.regions[region]
            .iter()
            .find(|id| sketch.curves.contains(*id) && sketch.regions.iter().filter(|r| r.contains(*id)).count() == 1);
        match unique {
            Some(id) => {
                if !ids.contains(id) {
                    ids.push(id.clone());
                }
            }
            // every curve of its outline is another region's too: a point inside it picks it alone
            None => match sketch.region_points.get(region).copied().flatten() {
                Some(p) => {
                    if !at.contains(&p) {
                        at.push(p);
                    }
                }
                None => return Err("a profile's outline shares all its curves with other regions, and no point is inside it alone".into()),
            },
        }
    }
    // every region of the sketch chosen: the whole sketch
    let plain = sketch.regions.len();
    if !disc && picked.len() == plain && plain > 0 {
        return Ok(Picks::default());
    }
    Ok(Picks { ids, at })
}

pub(crate) fn extrude(b: &mut Builder, name: &str, d: &ExtrudeDetail) -> Result<Option<String>, String> {
    if d.is_solid == Some(false) {
        return Err("a surface extrude is not mapped yet".into());
    }
    if d.is_thin_extrude == Some(true) {
        return Err("a thin extrude is not mapped yet".into());
    }
    let refs = profile_refs(&d.profile, &d.other);
    // The letters of a sketch text are profiles whose outlines are the font's, which the file does not give (the part's sketch
    // text has the kernel's own font): such an extrude would pick "every region" of a sketch that has none of its own (or
    // other regions than the letters).
    if d.other.get("_f3d_text_profiles").is_some() || d.other.get("_f3d_missing_texts").is_some_and(|m| m.as_array().is_some_and(|a| !a.is_empty())) {
        return Err("an extrude of the letters of a sketch text is not mapped yet (the font's outlines are not in the file)".into());
    }
    if !refs.is_empty() && refs.iter().all(|r| matches!(r, Reference::Face(_))) {
        return face_extrude(b, name, d, &refs);
    }
    let (sketch, regions) = selection(b, &refs, &d.other)?;
    let sketch = sketch.clone();
    let op = operation(b, d.operation.as_deref(), d.participant_bodies.as_deref().unwrap_or(&[]))?;
    let cut = op.op == "cut";

    // A taper of the one blind side is the part's `draft` (degrees, inward; `draftOut` when Fusion's angle is negative).
    if value(&d.taper_angle_two).is_some_and(|a| a.abs() > 1e-9) {
        return Err("a tapered extrude (side two) is not mapped yet".into());
    }
    let taper = value(&d.taper_angle_one).filter(|a| a.abs() > 1e-9).map(f64::to_degrees);
    if let Some(s) = &d.start_extent
        && s.definition_type.as_deref().is_some_and(|t| t != "ProfilePlaneStartDefinition")
    {
        return Err("an extrude that starts away from its sketch plane is not mapped yet".into());
    }

    let extent_type = d.extent_type.as_deref().unwrap_or("");
    let two_sides = d.has_two_extents == Some(true) || extent_type.contains("TwoSides");
    let symmetric = extent_type.contains("Symmetric")
        || kind_is(&d.extent_one, "SymmetricExtentDefinition")
        || d.symmetric_extent.is_some();
    // (end, depth in mm, backwards along the Fusion normal)
    let mut to_face: Option<Value> = None;
    let (end, depth, back): (&str, Option<f64>, bool) = if symmetric {
        let def = d.symmetric_extent.as_ref().or(d.extent_one.as_ref()).ok_or("it has no extent")?;
        let plain = def.definition_type.as_deref() == Some("DistanceExtentDefinition");
        if !plain && def.definition_type.as_deref().is_some_and(|t| t != "SymmetricExtentDefinition") {
            return Err("a symmetric extrude with this extent is not mapped yet".into());
        }
        let dist = value(&def.distance).filter(|v| *v > 0.0).ok_or("its length is not a positive number")?;
        // a distance given with a symmetric extent type is the length to each side (Fusion's default measurement)
        let full = !plain && def.is_full_length != Some(false);
        ("mid", Some(if full { dist } else { 2.0 * dist } * CM_TO_MM), false)
    } else if two_sides {
        let (one, two) = (d.extent_one.as_ref(), d.extent_two.as_ref());
        match (one.and_then(|e| e.definition_type.as_deref()), two.and_then(|e| e.definition_type.as_deref())) {
            (Some("ThroughAllExtentDefinition"), Some("ThroughAllExtentDefinition")) if cut => ("throughBoth", None, false),
            (Some("DistanceExtentDefinition"), Some("DistanceExtentDefinition")) => {
                let (a, c) = (value(&one.and_then(|e| e.distance.clone())), value(&two.and_then(|e| e.distance.clone())));
                match (a, c) {
                    (Some(a), Some(c)) if a > 0.0 && c > 0.0 && (a - c).abs() <= 1e-9 * a => {
                        ("mid", Some((a + c) * CM_TO_MM), false)
                    }
                    _ => return Err("an extrude with a different length each way is not mapped yet".into()),
                }
            }
            _ => return Err("an extrude with these two extents is not mapped yet".into()),
        }
    } else {
        let def = d.extent_one.as_ref().ok_or("it has no extent")?;
        match def.definition_type.as_deref() {
            Some("DistanceExtentDefinition") => {
                let v = value(&def.distance).filter(|v| *v != 0.0).ok_or("its length is not a number")?;
                ("blind", Some(v.abs() * CM_TO_MM), v < 0.0)
            }
            Some("ToEntityExtentDefinition") => {
                if def.is_chained == Some(true) {
                    return Err("an extrude to a chain of faces is not mapped yet".into());
                }
                let entity = def.entity.clone().ok_or("it names no entity to extrude to")?;
                if !matches!(entity, Reference::Face(_)) {
                    return Err("an extrude to something other than a face is not mapped yet".into());
                }
                let (name, at) = find_faces(b, std::slice::from_ref(&entity))?.remove(0);
                // toward the face: along the part's sketch normal or against it (`back` is against the Fusion normal)
                let n = sketch.frame.normal();
                let toward = dot([at[0] - sketch.frame.origin[0], at[1] - sketch.frame.origin[1], at[2] - sketch.frame.origin[2]], n);
                if toward.abs() < 1e-9 {
                    return Err("its end face is in the sketch plane".into());
                }
                to_face = Some(json!({"kind": "face", "id": name, "at": at}));
                ("upToFace", None, (toward < 0.0) != (sketch.normal_sign < 0.0))
            }
            Some("ThroughAllExtentDefinition") if cut => {
                // The decoder's `isPositiveDirection` is the flag byte. Its schema says 0 is along the sketch normal, but a
                // through-all cut from a face sketch (normal out of the body) with the flag 0 cuts into the body in the
                // corpus (the regenerated body matches the imported one only that way), so 0 is against the normal.
                ("through", None, def.is_positive_direction != Some(true))
            }
            Some("ThroughAllExtentDefinition") => {
                return Err("an extrude through all that adds material has no end in a part".into());
            }
            Some(other) => {
                return Err(format!("an extrude with the extent {} is not mapped yet", other.trim_end_matches("ExtentDefinition")));
            }
            None => return Err("its extent is not known".into()),
        }
    };
    // The taper is measured with the extrusion: a positive angle narrows along it, so on a negative length (the extrude runs
    // against the sketch normal) the angle's sign turns. The corpus settles it: a keycap pin extruded -3.8 mm with a -0.5 degree
    // taper has the edges of a body that narrows away from its sketch (their ends, 3.8 mm apart, are 0.033 mm closer in).
    let taper = if end == "blind" && back { taper.map(|a| -a) } else { taper };
    // `back` is against the Fusion normal; the part's frame may have the opposite normal. A cut runs against the
    // sketch normal unless flipped, an add along it.
    let against = back != (sketch.normal_sign < 0.0);
    let flip = end != "mid" && end != "throughBoth" && against != cut;

    if taper.is_some() && end != "blind" {
        return Err("a tapered extrude that is not one blind side is not mapped yet".into());
    }
    let id = b.next_id();
    let mut item = json!({"type": "extrude", "id": id, "name": name, "sketch": sketch.id, "end": end});
    finish(b, &mut item, &op);
    if let Some(depth) = depth {
        item["depth"] = json!(depth);
    }
    if let Some(a) = taper {
        item["draft"] = json!(a.abs());
        if a < 0.0 {
            item["draftOut"] = json!(true);
        }
    }
    if flip {
        item["flip"] = json!(true);
    }
    if let Some(r) = to_face {
        item["endRef"] = r;
    }
    regions.write(&mut item);
    b.features.push(item);
    b.builds = true;
    Ok(taper.map(|_| "taper sign: a positive angle narrows along the extrusion (settled by the edges of a corpus design's tapered pins)".to_owned()))
}

pub(crate) fn revolve(b: &mut Builder, name: &str, d: &RevolveDetail) -> Result<Option<String>, String> {
    if d.is_solid == Some(false) {
        return Err("a surface revolve is not mapped yet".into());
    }
    let refs = profile_refs(&d.profile, &d.other);
    let (sketch, regions) = selection(b, &refs, &d.other)?;
    let sketch = sketch.clone();
    let op = operation(b, d.operation.as_deref(), d.participant_bodies.as_deref().unwrap_or(&[]))?;

    // the angle: a full turn, or evenly either side of the sketch plane
    let def = d.extent_definition.as_ref().ok_or("it has no extent")?;
    let kind = def.definition_type.as_deref().unwrap_or("");
    let (degrees, mid) = match kind {
        "AngleExtentDefinition" => {
            let a = value(&def.angle).ok_or("its angle is not a number")?.abs();
            if (std::f64::consts::TAU - a).abs() <= FULL_TURN {
                (360.0, false)
            } else if def.is_symmetric == Some(true) {
                (a.to_degrees(), true)
            } else {
                return Err("a revolve of a partial turn in one direction is not mapped yet".into());
            }
        }
        "TwoSidesAngleExtentDefinition" => {
            let (a, c) = (value(&def.angle_one), value(&def.angle_two));
            match (a, c) {
                (Some(a), Some(c)) if a > 0.0 && (a - c).abs() <= 1e-9 => ((a + c).to_degrees(), true),
                _ => return Err("a revolve with a different angle each way is not mapped yet".into()),
            }
        }
        _ => return Err("a revolve with this extent is not mapped yet".into()),
    };

    // the axis: a line of the profile's sketch (or of an earlier one), or a world axis through the origin lying in
    // the sketch plane, written as a construction line in a sketch of its own
    let axis = match d.axis.as_ref() {
        Some(Reference::SketchEntity(e)) => {
            let id = e.id.clone().flatten().ok_or("its axis names no line")?;
            let index = e
                .sketch_timeline_index
                .flatten()
                .or_else(|| e.sketch.clone().flatten().and_then(|n| b.sketch_by_name.get(&n).copied()))
                .ok_or("its axis names no sketch")?;
            let owner = b.sketches.get(&index).ok_or("its axis is in a sketch that is not in the part")?;
            if !owner.lines.contains_key(&id) {
                return Err("its axis is not a line of its sketch in the part".into());
            }
            if owner.id == sketch.id { json!(id) } else { json!({"sketch": owner.id, "id": id}) }
        }
        Some(Reference::ConstructionAxis(c)) => {
            let which = c.origin.clone().flatten().ok_or("its axis is a construction axis that is not an origin axis")?;
            let dir = match which.as_str() {
                "X" => [1.0, 0.0, 0.0],
                "Y" => [0.0, 1.0, 0.0],
                "Z" => [0.0, 0.0, 1.0],
                _ => return Err("its axis is a construction axis that is not an origin axis".into()),
            };
            origin_axis(b, name, &sketch, dir)?
        }
        _ => return Err("its axis is not a sketch line or an origin axis".into()),
    };

    let id = b.next_id();
    let mut item = json!({
        "type": "revolve", "id": id, "name": name, "sketch": sketch.id, "axis": axis,
        "angle": degrees, "end": if mid { "mid" } else { "blind" },
    });
    finish(b, &mut item, &op);
    regions.write(&mut item);
    b.features.push(item);
    b.builds = true;
    Ok(None)
}

/// A construction line along a world axis through the origin, in a sketch of its own on the profile's plane.
fn origin_axis(b: &mut Builder, name: &str, sketch: &SketchInfo, dir: [f64; 3]) -> Result<Value, String> {
    let frame: &Frame = &sketch.frame;
    let normal = frame.normal();
    // the axis of the component's own origin, which sits at its placement
    let (dir, o) = match &b.place {
        Some(p) => (p.dir(dir), p.t),
        None => (dir, [0.0; 3]),
    };
    let d = unit(dir).ok_or("its axis has no direction")?;
    if dot(d, normal).abs() > 1e-9 {
        return Err("its axis does not lie in its sketch plane".into());
    }
    let off = dot([o[0] - frame.origin[0], o[1] - frame.origin[1], o[2] - frame.origin[2]], normal);
    if off.abs() > 1e-6 {
        return Err("its axis is not in its sketch plane".into());
    }
    let local = |p: [f64; 3]| -> [f64; 2] {
        let q = [p[0] + o[0] - frame.origin[0], p[1] + o[1] - frame.origin[1], p[2] + o[2] - frame.origin[2]];
        [dot(q, frame.x), dot(q, frame.y)]
    };
    // long enough to reach past the profile whichever way it lies along the line
    let reach = sketch.points.values().fold(1.0_f64, |m, p| m.max(p[0].abs()).max(p[1].abs())) * 4.0
        + dot(frame.origin, frame.origin).sqrt()
        + dot(o, o).sqrt();
    let tail = local([-d[0] * reach, -d[1] * reach, -d[2] * reach]);
    let head = local([d[0] * reach, d[1] * reach, d[2] * reach]);
    let id = b.next_id();
    b.features.push(json!({
        "type": "sketch", "id": id, "name": format!("{name} axis"), "plane": sketch.plane, "visible": false,
        "points": [{"id": "a1", "x": tail[0], "y": tail[1]}, {"id": "a2", "x": head[0], "y": head[1]}],
        "entities": [{"type": "line", "id": "ax", "p1": "a1", "p2": "a2", "construction": true}],
        "constraints": [],
    }));
    Ok(json!({"sketch": id, "id": "ax"}))
}

pub(crate) fn construction_plane(b: &mut Builder, name: &str, d: &ConstructionPlaneDetail) -> Result<(), String> {
    let g = d.geometry.as_ref().ok_or("its plane is not known")?;
    let (o, u, v) = (g.origin.ok_or("its plane has no origin")?, g.u_direction.ok_or("its plane has no x axis")?, g.v_direction.ok_or("its plane has no y axis")?);
    let frame = Frame::new(o.map(|c| c * CM_TO_MM), u, v).ok_or("its plane has a zero axis")?;
    let frame = b.world(frame);
    b.plane_for(name, &frame);
    Ok(())
}

/// The faces of the history regenerated so far that the references name: planar faces through the reference's point on
/// the face, with its normal (the part's names, which a part can select).
fn find_faces(b: &mut Builder, refs: &[Reference]) -> Result<Vec<(String, Vec3)>, String> {
    if b.lost {
        return Err("an earlier item did not map, so the faces of the body so far are not those the file names".into());
    }
    if !b.builds {
        return Err("no earlier feature made a body".into());
    }
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = crate::guard::regenerate_body(&part).ok_or("the history so far builds no body to find the face in")?;
    let mut out = Vec::new();
    for r in refs {
        let Reference::Face(fp) = r else { return Err("its profile is not a face".into()) };
        let p = fp.point_on_face.ok_or("its face has no known position")?;
        let n = fp.normal_at_point.and_then(unit).ok_or("its face has no known normal")?;
        let p = b.world_point([p[0] * CM_TO_MM, p[1] * CM_TO_MM, p[2] * CM_TO_MM]);
        let n = b.world_dir(n);
        let pt = ck_geom::Vec3::new(p[0], p[1], p[2]);
        let mut hit = None;
        for f in body.face_ids() {
            let ck_geom::surface::Surface::Plane { frame } = &body.face(f).surface else { continue };
            let off = (pt - frame.o).dot(frame.z).abs();
            let fnorm = body.planar_normal(f);
            if off > 0.02 || (fnorm.x * n[0] + fnorm.y * n[1] + fnorm.z * n[2]) < 0.999 {
                continue;
            }
            if ck_ops::boolean::classify::point_in_face(&body, f, pt, 0.02) != ck_ops::boolean::classify::Where::Out {
                if hit.is_some() {
                    return Err("its face is not told from another by its point and normal".into());
                }
                hit = Some(f);
            }
        }
        let f = hit.ok_or("its face is not a planar face of the history regenerated so far")?;
        if std::env::var("CK_F3D_DEBUG").is_ok() {
            let fnorm = body.planar_normal(f);
            eprintln!("face pick: face {} outward normal {:?} file normal {:?} at {:?}; faces of body: {}", body.face_id_str(f), fnorm, n, p, body.face_ids().count());
            for g in body.face_ids() {
                if let ck_geom::surface::Surface::Plane { .. } = &body.face(g).surface {
                    let gn = body.planar_normal(g);
                    eprintln!("  plane face {} normal {:?}", body.face_id_str(g), gn);
                }
            }
        }
        out.push((body.face_id_str(f), p));
    }
    Ok(out)
}

/// An extrude of the face of an earlier body: a press-pull of that face (out of the body joins, into it cuts).
fn face_extrude(b: &mut Builder, name: &str, d: &ExtrudeDetail, refs: &[Reference]) -> Result<Option<String>, String> {
    let join = matches!(d.operation.as_deref(), Some("JoinFeatureOperation") | None);
    let cut = d.operation.as_deref() == Some("CutFeatureOperation");
    if !join && !cut {
        return Err("an extrude of a face that is not a join or a cut is not mapped yet".into());
    }
    for def in [value(&d.taper_angle_one), value(&d.taper_angle_two)] {
        if def.is_some_and(|a| a.abs() > 1e-9) {
            return Err("a tapered extrude of a face is not mapped yet".into());
        }
    }
    if let Some(s) = &d.start_extent
        && s.definition_type.as_deref().is_some_and(|t| t != "ProfilePlaneStartDefinition")
    {
        return Err("an extrude that starts away from its profile is not mapped yet".into());
    }
    let two = d.has_two_extents == Some(true) || d.extent_type.as_deref().is_some_and(|t| t.contains("TwoSides") || t.contains("Symmetric"));
    if two || d.symmetric_extent.is_some() {
        return Err("an extrude of a face with two sides is not mapped yet".into());
    }
    let def = d.extent_one.as_ref().ok_or("it has no extent")?;
    if def.definition_type.as_deref() != Some("DistanceExtentDefinition") {
        return Err("an extrude of a face with this extent is not mapped yet".into());
    }
    let v = value(&def.distance).filter(|v| *v != 0.0).ok_or("its length is not a number")?;
    // a positive distance runs along the face's outward normal, a negative one against it (into the body: a cut)
    let along = v > 0.0;
    let signed = if along { v.abs() } else { -v.abs() } * CM_TO_MM;
    if join && signed < 0.0 || cut && signed > 0.0 {
        return Err(format!("an extrude of a face that {} {} is not mapped yet", if join { "joins" } else { "cuts" }, if v > 0.0 { "positive" } else { "negative" }));
    }
    let found = find_faces(b, refs)?;
    let faces: Vec<String> = found.iter().map(|(n, _)| n.clone()).collect();
    let id = b.next_id();
    if cut && let Some(thin) = sliver_behind(b, refs, signed) {
        // The file's body has this cut applied, and it removes a sliver of material thinner than the tolerance the
        // body is kept to (the face lies a hair inside the opposite wall); the kernel cannot build a face that
        // close to another and refuses it (`moveFaceFailed`). The press-pull is kept, suppressed, so the history
        // shows it, and the guard still compares the volume and box with the file's body.
        b.features.push(json!({"type": "pressPull", "id": id, "name": name, "suppressed": true, "faces": faces, "continue": "straight", "distance": signed}));
        b.builds = true;
        return Ok(Some(format!("a press-pull of the face, kept suppressed: it removes only a sliver of material {thin:.1e} mm thick or less, which the kernel cannot build")));
    }
    b.features.push(json!({"type": "pressPull", "id": id, "name": name, "faces": faces, "continue": "straight", "distance": signed}));
    b.builds = true;
    Ok(Some("a press-pull of the face".into()))
}

/// How deep a cut of the face is: `Some(depth)` when the body has no material 1e-3 mm behind the face (the face's own
/// point, shifted against its outward normal), so the cut removes at most that much. Needs the face's point and
/// normal in the regenerated history, as `find_faces` does.
fn sliver_behind(b: &mut Builder, refs: &[Reference], signed: f64) -> Option<f64> {
    const PROBE: f64 = 1e-3;
    if signed.abs() <= PROBE {
        return None;
    }
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = crate::guard::regenerate_body(&part)?;
    let inside = ck_ops::contain::Containment::new(&body, 1e-6);
    for r in refs {
        let Reference::Face(fp) = r else { return None };
        let p = fp.point_on_face?;
        let n = fp.normal_at_point.and_then(unit)?;
        let p = b.world_point([p[0] * CM_TO_MM, p[1] * CM_TO_MM, p[2] * CM_TO_MM]);
        let n = b.world_dir(n);
        // the face's outward normal is the file's (`find_faces` matched them), a cut runs against it
        let q = ck_geom::Vec3::new(p[0] - n[0] * PROBE, p[1] - n[1] * PROBE, p[2] - n[2] * PROBE);
        if inside.classify_point(q, 1e-6) != ck_ops::contain::Loc::Out {
            return None;
        }
    }
    Some(PROBE)
}
