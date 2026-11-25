//! Features that act on bodies the file names: `Combine`, `CopyPasteBodies` and `Move` of bodies.
//!
//! The file names a body by the timeline item that made it (`_f3d.producer`, the decoder's reading of the body record)
//! and its index among that item's bodies (`_f3d.body_index`). In a design that has such a feature every body a mapped
//! extrude or revolve makes (and every copy a paste makes) is a named body of the part, `B<item>` (`B<item>.<n>` for the
//! later bodies of a paste), and the table `Builder::bodies` takes (item, index) to that name. A feature selects the body
//! its file reference names through the table; a reference to a body the history did not make (an item that is not mapped,
//! one that makes bodies in a way the part cannot name, a body consumed by an earlier combine) is not guessed.
//!
//! The history so far is regenerated before a feature is written, and every body it names must be exactly one lump of the
//! result: an extrude with several separate profiles makes several lumps under one name, and then (item, index) does
//! not select one of them.
//!
//! The Combine operation codes (join, cut, intersect), the target-before-tools order and the consumed tools follow the
//! public cadmpeg project's Fusion notes (docs/formats/f3d.md, CC BY 4.0, knowledge only, see THIRD_PARTY.md); they were
//! re-derived on the corpus by the guard, not copied.

use ck_f3d_dump::Reference;
use serde_json::{Map, Value, json};

use crate::{Builder, Placement};

/// The (item, index) a body reference names, as the decoder gives it.
fn key_of(r: &Reference) -> Option<(i64, i64)> {
    let Reference::Body(fp) = r else { return None };
    let f = fp.f3d.as_ref()?;
    Some((f.producer?, f.body_index?))
}

fn refs(v: Option<&Value>) -> Vec<Reference> {
    match v {
        Some(Value::Array(a)) => a.iter().filter_map(|x| x.as_object().map(|o| Reference::from_map(o.clone()))).collect(),
        Some(Value::Object(o)) => vec![Reference::from_map(o.clone())],
        _ => Vec::new(),
    }
}

/// Whether a move's inputs are bodies that name their producer (the design then names its bodies).
pub(crate) fn move_names_producers(m: &Map<String, Value>) -> bool {
    let inputs = refs(m.get("inputEntities"));
    !inputs.is_empty() && inputs.iter().all(|r| key_of(r).is_some())
}

/// The part's name of the body a reference names, or why there is none.
fn lookup(b: &Builder, r: &Reference, what: &str) -> Result<String, String> {
    let Reference::Body(fp) = r else {
        return Err(format!("its {what} is not a body"));
    };
    let found = fp.f3d.as_ref().and_then(|f| f.found.as_deref()).unwrap_or("");
    if found.contains("suppressed") {
        return Err(format!("its {what} is suppressed in the file"));
    }
    let Some((p, i)) = key_of(r) else {
        return Err(format!("the file does not say which item made its {what}"));
    };
    match b.bodies.get(&(p, i)) {
        Some((n, comp)) if *comp == b.comp => Ok(n.clone()),
        Some(_) => Err(format!("its {what} is in another component than the item")),
        None => {
            let kind = b.kinds.get(&p).map_or("an item", String::as_str);
            Err(format!("its {what} is made by a {kind} item, and the history so far has no such body (that item is not mapped, makes no named body, or the body was consumed)"))
        }
    }
}

/// The participant bodies of an extrude or revolve as part body selections, when the file lists some and the history names
/// every one of them (`None`: it acts on whatever it touches, as before).
pub(crate) fn participants(b: &Builder, refs: &[Reference]) -> Option<Value> {
    if refs.is_empty() {
        return None;
    }
    let names: Option<Vec<Value>> = refs
        .iter()
        .map(|r| key_of(r).and_then(|k| b.bodies.get(&k)).filter(|(_, c)| *c == b.comp).map(|(n, _)| json!({"body": n})))
        .collect();
    names.map(Value::Array)
}

/// Every name must be exactly one lump of the history regenerated so far.
fn check_one_lump_each(b: &Builder, names: &[&str]) -> Result<(), String> {
    let part = json!({"format": "ck-part", "version": 1, "features": b.features});
    let body = crate::guard::regenerate_body(&part).ok_or("the history so far builds no body to select from")?;
    for n in names {
        let c = body.lump_ids().filter(|l| body.lump(*l).name.as_deref() == Some(*n)).count();
        if c != 1 {
            return Err(format!(
                "the body {n} is {c} lumps in the history regenerated so far, and a selection by the file's names needs exactly one"
            ));
        }
    }
    Ok(())
}

/// `CombineFeature`: join, cut or intersect of the target body with the tool bodies, all selected by the file's names.
pub(crate) fn combine(b: &mut Builder, name: &str, m: &Map<String, Value>) -> Result<Option<String>, String> {
    if !b.named {
        return Err("the design does not name its bodies".into());
    }
    if m.get("isNewComponent").and_then(Value::as_bool) == Some(true) {
        return Err("a combine that puts its result in a new component is not mapped yet".into());
    }
    let op = match m.get("operation").and_then(Value::as_str) {
        Some("JoinFeatureOperation") => "union",
        Some("CutFeatureOperation") => "cut",
        Some("IntersectFeatureOperation") => "intersect",
        Some(o) => return Err(format!("the combine operation {o} is not mapped yet")),
        None => return Err("its operation is not known".into()),
    };
    let target = refs(m.get("targetBody"));
    if target.len() != 1 {
        return Err("it names no single target body".into());
    }
    let tools = refs(m.get("toolBodies"));
    if tools.is_empty() {
        return Err("it names no tool body".into());
    }
    let tn = lookup(b, &target[0], "target body")?;
    let mut tool_names = Vec::new();
    for t in &tools {
        let n = lookup(b, t, "tool body")?;
        if n == tn || tool_names.contains(&n) {
            return Err("its target and tools are not distinct bodies".into());
        }
        tool_names.push(n);
    }
    let all: Vec<&str> = std::iter::once(tn.as_str()).chain(tool_names.iter().map(String::as_str)).collect();
    check_one_lump_each(b, &all)?;
    let keep = m.get("isKeepToolBodies").and_then(Value::as_bool) == Some(true);
    let id = b.next_id();
    b.features.push(json!({
        "type": "combine", "id": id, "name": name, "op": op, "target": {"body": tn},
        "tools": tool_names.iter().map(|n| json!({"body": n})).collect::<Vec<_>>(), "keepTools": keep,
    }));
    b.builds = true;
    if !keep {
        // the tools are consumed: the file names no body by them from here on
        for (t, n) in tools.iter().zip(&tool_names) {
            if let Some(k) = key_of(t) {
                b.bodies.remove(&k);
            }
            b.consumed.insert(n.clone());
        }
    }
    Ok(None)
}

/// `CopyPasteBodies`: a copy of each source body in place (a body of its own), named by the paste's item.
pub(crate) fn copy_paste(b: &mut Builder, name: &str, m: &Map<String, Value>) -> Result<Option<String>, String> {
    if !b.named {
        return Err("the design does not name its bodies".into());
    }
    if b.place.is_some() {
        return Err("it is in a component placed away from the origin".into());
    }
    let sources = refs(m.get("sourceBody"));
    if sources.is_empty() {
        return Err("it names no source body".into());
    }
    let mut names = Vec::new();
    for s in &sources {
        names.push(lookup(b, s, "source body")?);
    }
    let all: Vec<&str> = names.iter().map(String::as_str).collect();
    check_one_lump_each(b, &all)?;
    for (i, src) in names.iter().enumerate() {
        let new = if i == 0 { format!("B{}", b.cur) } else { format!("B{}.{i}", b.cur) };
        let id = b.next_id();
        b.features.push(json!({"type": "moveBody", "id": id, "name": name, "bodies": [{"body": src}], "copy": true, "body": new}));
        b.bodies.insert((b.cur, i as i64), (new, b.comp.clone()));
    }
    b.builds = true;
    Ok(None)
}

/// A `MoveFeature` of bodies by the file's names: the part's `moveBody` on those bodies, the rotation of the matrix and
/// then its translation.
pub(crate) fn move_named(b: &mut Builder, name: &str, m: &Map<String, Value>) -> Result<Option<String>, String> {
    if b.place.is_some() {
        return Err("it is in a component placed away from the origin".into());
    }
    let inputs = refs(m.get("inputEntities"));
    if inputs.is_empty() {
        return Err("it moves no body".into());
    }
    let mut names = Vec::new();
    for r in &inputs {
        names.push(lookup(b, r, "moved body")?);
    }
    let rows = m.get("transform").and_then(Value::as_array).filter(|r| r.len() == 4).ok_or("it has no transform")?;
    let mut mat = [[0.0f64; 4]; 4];
    for (i, row) in rows.iter().enumerate() {
        let row = row.as_array().filter(|r| r.len() == 4).ok_or("its transform is not a 4 x 4 matrix")?;
        for (j, v) in row.iter().enumerate() {
            mat[i][j] = v.as_f64().filter(|v| v.is_finite()).ok_or("its transform is not numbers")?;
        }
    }
    let Some(p) = Placement::from_mat(&mat)? else {
        return Ok(Some("it moves nothing".into()));
    };
    let all: Vec<&str> = names.iter().map(String::as_str).collect();
    check_one_lump_each(b, &all)?;
    let id = b.next_id();
    let mut f = json!({
        "type": "moveBody", "id": id, "name": name, "bodies": names.iter().map(|n| json!({"body": n})).collect::<Vec<_>>(),
        "translate": p.t,
    });
    if let Some((axis, angle)) = p.axis_angle() {
        f["rotate"] = json!({"axis": axis, "origin": [0.0, 0.0, 0.0], "angle": angle});
    }
    b.features.push(f);
    Ok(None)
}
