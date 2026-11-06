//! A part that starts with imported bodies (`Imported`, `Multi_CAD_Imported`, and the `Combine` / body moves that
//! join them) and goes on with features of its own (cuts and bosses on the imported body).
//!
//! The file holds only the final body, not the imported one, so the starting body is made from the final body by
//! undoing the later steps, last first: the material a cut removed is put back (the cut's tool, as an add) and the
//! material a boss added is taken away (the boss's tool, as a cut). For a cut `T` after a body `B`, `B' = B - T` and
//! `(B' + T) - T = B'`; for a boss, `B' = B + T` and `(B' - T) + T = B'`: whatever the imported body was inside the
//! tool, the steps run again on the undone body give the final body back. So the document is an `importBody` of the
//! file's solid (`F1`), the undo features (the later steps' sketches and tools with the operation turned round), then
//! the later steps themselves, editable, read back as for any part and verified against the file as a whole (or kept
//! as an editable prefix by the usual checks).
//!
//! The steps are read back in passes: the first on the file's solid itself, which tells which leading steps are
//! booleans whose tools can be undone; each later pass starts from the solid with those undone, so the steps see the
//! material they were made on, and is kept when it reads further. Only extrudes and revolves that add or cut are
//! undone; the first step that is not one ends the undo (the steps after it are read on the body as it is then).

use crate::{read_v, Outcome, Status, Tagged};
use ck_json::{Json, Obj};
use ck_recog::model::Model;
use ck_sldprt::meta::SwFeature;
use ck_topo::body::{Body, FaceId};
use std::collections::{HashMap, HashSet};
use std::rc::Rc;

/// A feature that brings bodies into the part, or moves and joins them, without faces of its own to read back.
fn is_source(f: &SwFeature) -> bool {
    crate::imported::is_imported(&f.kind) || f.kind == "Combine" || f.kind.starts_with("Body-Move")
}

/// The source features the starting body stands for (ids of those with faces on the final body): the first run of
/// sources with faces (features with no faces between them do not end it; at the start of the history, or after
/// steps read back as usual), when at least one feature after it has faces (else the imported body is the whole
/// history, `imported.rs`).
pub fn sources(features: &[SwFeature], tags: &[Vec<(i64, i64)>]) -> Option<HashSet<i64>> {
    let with_faces: HashSet<i64> = tags.iter().flatten().map(|t| t.0).collect();
    let mut src = HashSet::new();
    for f in features.iter().filter(|f| crate::classify(f) != crate::Class::Skip && with_faces.contains(&f.id)) {
        if is_source(f) {
            src.insert(f.id);
        } else if !src.is_empty() {
            return Some(src);
        }
    }
    None
}

/// The file's solid as our `importBody` feature `F1` puts it in the document, for a history that starts with
/// imported bodies; `None` when it does not, or the file's solid is not one valid solid.
pub fn start_body(imp: &ck_sldprt::SldprtImport, tags: &[Vec<(i64, i64)>]) -> Option<Rc<Body>> {
    sources(&imp.meta.features, tags)?;
    if imp.sheets > 0 || imp.body.num_faces() == 0 {
        return None;
    }
    let s = ck_model::features::import_body::sort_sldprt(imp.body.clone(), &imp.parts, "F1", Some("F1"));
    if !s.surfaces.is_empty() || s.solid.num_faces() != imp.body.num_faces() {
        return None;
    }
    Some(Rc::new(s.solid))
}

/// The starting body: the `importBody` of the file's solid (`S0`) and the undo features (`S1`, ...), given ids of
/// the document where the first source step puts them.
pub struct Start {
    pub feats: Vec<Json>,
    /// Names of the file's faces that are faces of the starting body (`S0.I<k>F<n>`): every face no step after the
    /// sources made or changed.
    pub names: HashMap<FaceId, String>,
    pub src: HashSet<i64>,
    /// The note of the first source step.
    pub note: String,
    /// Regenerate and verify the whole history after the read (a first reading only tells which steps to undo).
    pub verify: bool,
}

/// Names of the faces of the starting body: the start body is the file's solid face for face (the same faces in the
/// same order, renamed `F1.I<k>F<n>`), so a face of the file that no step after the sources made or changed is named
/// as the start body's face at its index (as `S0.I<k>F<n>`), where the two lie on the same surface.
fn names(t: &Tagged, start: &Body, src: &HashSet<i64>) -> HashMap<FaceId, String> {
    let mut out = HashMap::new();
    if start.faces.len() != t.body.faces.len() {
        return out;
    }
    let pos: HashMap<i64, usize> = t.features.iter().enumerate().map(|(i, f)| (f.id, i)).collect();
    let Some(last) = t.features.iter().rposition(|f| src.contains(&f.id)) else { return out };
    let (m, ms) = (Model::new(t.body), Model::new(start));
    let live: HashSet<FaceId> = ms.faces.iter().copied().collect();
    for &f in &m.faces {
        if !t.tags.get(f.idx()).is_some_and(|v| !v.is_empty() && v.iter().all(|x| pos.get(&x.0).is_some_and(|&p| p <= last))) {
            continue;
        }
        if live.contains(&f) && crate::blend::same_surface(m.kind(f), ms.kind(f)) {
            let n = start.face_id_str(f);
            out.insert(f, n.strip_prefix("F1.").map_or(n.clone(), |r| format!("S0.{r}")));
        }
    }
    out
}

/// The start features and names with the document ids they get (`S0`, `S1`, ... to fresh ids from `ids`).
pub fn place(st: &Start, ids: &mut ck_recog::emit::Ids) -> (Vec<Json>, HashMap<FaceId, String>) {
    let mut map = HashMap::new();
    for f in &st.feats {
        if let Some(id) = f.get_str("id") {
            map.insert(id.to_string(), ids.next());
        }
    }
    let feats = st.feats.iter().map(|f| retag(f, &map)).collect();
    let names = st.names.iter().map(|(&f, n)| (f, match retag(&Json::Str(n.clone()), &map) { Json::Str(s) => s, _ => n.clone() })).collect();
    (feats, names)
}

fn retag(j: &Json, map: &HashMap<String, String>) -> Json {
    match j {
        Json::Str(s) => {
            if let Some(n) = map.get(s.as_str()) {
                return Json::Str(n.clone());
            }
            if let Some((head, rest)) = s.split_once('.') {
                if let Some(n) = map.get(head) {
                    return Json::Str(format!("{n}.{rest}"));
                }
            }
            j.clone()
        }
        Json::Arr(a) => Json::Arr(a.iter().map(|x| retag(x, map)).collect()),
        Json::Obj(kv) => Json::Obj(kv.iter().map(|(k, v)| (k.clone(), retag(v, map))).collect()),
        _ => j.clone(),
    }
}

/// The feature with its material operation turned round (`add` <-> `cut`); `None` for anything else. An extrude cut
/// runs against the sketch normal unless flipped, an add along it: the flip turns too, so the tool stays in place.
fn inverted(f: &Json) -> Option<Json> {
    let ty = f.get_str("type")?;
    let op = f.get_str("op")?;
    let new_op = match op {
        "cut" => "add",
        "add" => "cut",
        _ => return None,
    };
    let Json::Obj(kv) = f else { return None };
    let mut kv = kv.clone();
    for (k, v) in kv.iter_mut() {
        if k == "op" {
            *v = Json::Str(new_op.into());
        }
    }
    match ty {
        "revolve" => {}
        "extrude" => {
            // (the end conditions that depend on other faces of the body are not undone)
            if !matches!(f.get_str("end"), None | Some("blind" | "through" | "mid" | "throughBoth")) || f.get("target").is_some() {
                return None;
            }
            let flip = matches!(f.get("flip"), Some(Json::Bool(true)));
            kv.retain(|(k, _)| k != "flip");
            if !flip {
                kv.push(("flip".into(), Json::Bool(true)));
            }
        }
        _ => return None,
    }
    Some(Json::Obj(kv))
}

/// The undo features of a read.
struct Block {
    /// `S1`, ...: the sketches and planes of the undone steps, then their tools turned round, last step first.
    feats: Vec<Json>,
    /// Steps read after the sources that the block undoes (the first ones).
    steps: usize,
    /// The SolidWorks step of each tool feature of `feats` (by id).
    of: HashMap<String, i64>,
}

/// The undo features of the steps right after the source steps of a read whose tools can be undone, up to `limit`
/// steps.
fn undo_block(o: &Outcome, src: &HashSet<i64>, limit: usize) -> Block {
    let mut keep: Vec<(usize, usize, i64, String)> = vec![];
    let empty = Block { feats: vec![], steps: 0, of: HashMap::new() };
    let Some(i0) = o.steps.iter().position(|s| src.contains(&s.sw_id)) else { return empty };
    let Some(&(a0, base)) = o.ranges.get(i0) else { return empty };
    let mut map = HashMap::new();
    if let Some(id) = o.features.get(a0).and_then(|f| f.get_str("id")) {
        map.insert(id.to_string(), "S0".to_string());
    }
    for (s, &(a, b)) in o.steps.iter().zip(&o.ranges).skip(i0 + 1) {
        if src.contains(&s.sw_id) {
            continue;
        }
        if keep.len() >= limit {
            break;
        }
        match s.status {
            Status::Skipped if a == b => continue,
            Status::Rebuilt if a >= base && b <= o.features.len() && a < b => {
                let solid: Vec<&Json> = o.features[a..b].iter().filter(|f| !matches!(f.get_str("type"), Some("plane" | "sketch"))).collect();
                if solid.is_empty() || solid.iter().any(|f| inverted(f).is_none()) {
                    break;
                }
                keep.push((a, b, s.sw_id, s.name.clone()));
            }
            _ => break,
        }
    }
    let mut next = 0;
    for &(a, b, ..) in &keep {
        for f in &o.features[a..b] {
            if let Some(id) = f.get_str("id") {
                next += 1;
                map.insert(id.to_string(), format!("S{next}"));
            }
        }
    }
    let mut out: Vec<Json> = vec![];
    for &(a, b, ..) in &keep {
        out.extend(o.features[a..b].iter().filter(|f| matches!(f.get_str("type"), Some("plane" | "sketch"))).map(|f| retag(f, &map)));
    }
    let mut of = HashMap::new();
    for (a, b, id, name) in keep.iter().rev() {
        for f in o.features[*a..*b].iter().rev().filter(|f| !matches!(f.get_str("type"), Some("plane" | "sketch"))) {
            let mut g = retag(&inverted(f).expect("checked above"), &map);
            if let Json::Obj(kv) = &mut g {
                kv.retain(|(k, _)| k != "name");
                kv.push(("name".into(), Json::Str(format!("Undo {name}"))));
            }
            if let Some(gid) = g.get_str("id") {
                of.insert(gid.to_string(), *id);
            }
            out.push(g);
        }
    }
    Block { feats: out, steps: keep.len(), of }
}

/// [`undo_block`] cut short where an undo does not regenerate on the file's solid (a boolean of a tool that
/// coincides with faces of the body): the block ends before that step.
fn checked_block(t: &Tagged, o: &Outcome, src: &HashSet<i64>, f0: &Json) -> Block {
    let mut limit = usize::MAX;
    loop {
        let b = undo_block(o, src, limit);
        if b.feats.is_empty() {
            return b;
        }
        let mut feats = vec![f0.clone()];
        feats.extend(b.feats.iter().cloned());
        let errs = crate::session::Session::with_start(t.start.clone()).rebuild(&feats).map(|r| r.1).unwrap_or_else(|e| vec![format!("S0: {e}")]);
        // (the undo of the step nearest the sources that fails: the block keeps the steps before it)
        let order: Vec<i64> = o.steps.iter().map(|s| s.sw_id).collect();
        let Some(k) = errs.iter().filter_map(|e| e.split(':').next().and_then(|id| b.of.get(id.trim()))).filter_map(|id| order.iter().position(|x| x == id)).min() else {
            return b;
        };
        let i0 = o.steps.iter().position(|s| src.contains(&s.sw_id)).unwrap_or(0);
        limit = o.steps[i0 + 1..k].iter().zip(&o.ranges[i0 + 1..k]).filter(|(s, r)| s.status == Status::Rebuilt && !src.contains(&s.sw_id) && r.0 < r.1).count();
    }
}

/// The steps after the sources read back into features of their own, in order.
fn own_steps(o: &Outcome, src: &HashSet<i64>) -> Vec<usize> {
    let Some(i0) = o.steps.iter().position(|s| src.contains(&s.sw_id)) else { return vec![] };
    (i0 + 1..o.steps.len().min(o.ranges.len())).filter(|&i| o.steps[i].status == Status::Rebuilt && !src.contains(&o.steps[i].sw_id) && o.ranges[i].0 < o.ranges[i].1).collect()
}

/// How many faces fewer than the file the rebuild `new` may have, for a part that starts with imported bodies: the
/// booleans of the later steps merge neighbouring faces of one surface that the file keeps apart (an imported
/// cylinder in two halves), so up to the number of such merges (per group of neighbouring faces on one surface, its
/// size less one) the faces only have to pair up by surface and area (`fastverify::verify_split`). `None` when the
/// part does not start with imported bodies or the rebuild has no fewer faces.
pub fn merges(t: &Tagged, new: &Body) -> Option<usize> {
    t.start.as_ref()?;
    let d = t.body.num_faces().checked_sub(new.num_faces()).filter(|&d| d > 0)?;
    let m = Model::new(t.body);
    let mut seen: HashSet<FaceId> = HashSet::new();
    let mut bound = 0;
    for &f in &m.faces {
        if !seen.insert(f) {
            continue;
        }
        let mut todo = vec![f];
        while let Some(g) = todo.pop() {
            for e in m.edges(g) {
                let Some(h) = m.across(e, g) else { continue };
                if !seen.contains(&h) && crate::blend::same_surface(m.kind(g), m.kind(h)) {
                    seen.insert(h);
                    bound += 1;
                    todo.push(h);
                }
            }
        }
    }
    (d <= bound).then_some(d)
}

/// The history of a part that starts with imported bodies (see the module docs); `None` when it does not, or no
/// read of it is editable.
pub fn rebuild(t: &Tagged) -> Option<Outcome> {
    let start = t.start.as_ref()?;
    let src = sources(t.features, &t.tags)?;
    let names = names(t, start, &src);
    let f1 = Obj::new().put("type", "importBody").put("id", "S0").put("name", "Imported body").put("format", "sldprt").put("blob", "").put("replace", true).build();
    let mut undo: Vec<Json> = vec![];
    let mut undone = 0usize;
    let mut last: Option<Outcome> = None;
    for _ in 0..4 {
        let mut feats = vec![f1.clone()];
        feats.extend(undo.iter().cloned());
        let note = if undone == 0 {
            "the file's solid as the starting body (importBody)".to_string()
        } else {
            format!("the file's solid as the starting body (importBody), with the material of the next {undone} step(s) undone ({} undo feature(s))", undo.len())
        };
        // (the first reading, on the file's solid itself, only tells which steps to undo)
        let st = Start { feats, names: names.clone(), src: src.clone(), note, verify: undone > 0 };
        let o = read_v(t, 0, Some(&st));
        let own = own_steps(&o, &src);
        if own.is_empty() {
            break;
        }
        // (a history that verifies is taken only when every step of its own after the sources is undone in the
        // starting body: those steps then make the material they stand for, where on the file's solid itself a boss
        // only adds what is there)
        if (o.exact() || o.close()) && own.len() == undone {
            return Some(o);
        }
        let blk = checked_block(t, &o, &src, &f1);
        if blk.steps == 0 && undone == 0 {
            // nothing to undo: no step of the file's own can be kept on the starting body
            return None;
        }
        last = Some(o);
        if blk.steps <= undone {
            break;
        }
        undo = blk.feats;
        undone = blk.steps;
    }
    // (the read on the most material undone) the steps whose material is undone that pass their checks, then the
    // file's remainder: a step after them is read on material it did not make
    let mut o = last?;
    let own = own_steps(&o, &src);
    if let Some(&i) = own.get(undone) {
        if o.steps.iter().take(i).all(|s| s.status != Status::Unsupported) {
            o.steps[i].status = Status::Unsupported;
            o.steps[i].note = format!("its material is not undone in the starting body ({})", o.steps[i].note);
            o.fallback = Some(format!("{} ({}): its material is not undone in the starting body", o.steps[i].name, o.steps[i].kind));
            o.features.truncate(o.ranges[i].0);
            o.ranges.truncate(i);
        }
    }
    crate::keep_prefix(t, &mut o, &std::cell::RefCell::new(crate::session::Session::with_start(t.start.clone())));
    // (a prefix of the starting body alone keeps nothing editable of the file's own)
    let own = o.steps.iter().zip(&o.ranges).take(o.prefix).any(|(s, r)| s.status == Status::Rebuilt && !src.contains(&s.sw_id) && r.0 < r.1);
    (o.editable() && own).then_some(o)
}

/// Steps kept as editable features: every rebuilt step of a history that rebuilds to the end, else of its prefix.
pub fn kept(o: &Outcome) -> usize {
    let n = if o.exact() || o.close() { o.steps.len() } else { o.prefix };
    o.steps[..n.min(o.steps.len())].iter().filter(|s| s.status == Status::Rebuilt).count()
}
