//! `sldasmTree`: the component tree of a SolidWorks assembly (`.SLDASM`) as JSON, and optionally as a part document of
//! `insertPart` features (ck-sldprt `asm`). Read only. See book/src/io/sldprt.md.

use ck_json::{Json, Obj};
use ck_server::session::Cx;
use ck_sldprt::asm::AsmComponent;
use ck_sldprt::mates::{self, MateOperand};

fn num_arr(v: &[f64]) -> Json {
    Json::Arr(v.iter().map(|&x| Json::Num(if x == 0.0 { 0.0 } else { x })).collect())
}

fn component_json(c: &AsmComponent) -> Json {
    let mut o = Obj::new()
        .put("name", c.name.as_str())
        .put("model", c.model.as_str())
        .put("part", c.part.as_deref().map_or(Json::Null, Json::from))
        .put("kind", c.kind.map_or(Json::Null, Json::from))
        .put("configuration", c.configuration.as_str())
        .put("transform", num_arr(&c.transform))
        .put("suppressed", c.suppressed)
        .put("hidden", c.hidden)
        .put("virtual", c.virtual_component)
        .put("excludeFromBom", c.exclude_from_bom)
        .put("flexible", c.flexible);
    o = o.put("bbox", c.bbox.map_or(Json::Null, |b| num_arr(&b)));
    o.put("children", Json::Arr(c.children.iter().map(component_json).collect())).build()
}

/// 4x4 row-major column-vector matrices: `a * b`.
fn mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut m = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            m[r * 4 + c] = (0..4).map(|k| a[r * 4 + k] * b[k * 4 + c]).sum();
        }
    }
    m
}

struct Ctx<'a> {
    /// Parts the caller supplied, by referenced file name (case-insensitive): documents inserted in place of the empty stand-in.
    parts: &'a [(String, Json)],
    features: Vec<Json>,
    unresolved: Vec<String>,
    skipped: usize,
}

fn flatten(cx: &mut Ctx, comps: &[AsmComponent], parent: &[f64; 16], path: &str) {
    for c in comps {
        if c.suppressed {
            cx.skipped += 1;
            continue;
        }
        let path_name = if path.is_empty() { c.name.clone() } else { format!("{path}/{}", c.name) };
        let w = mul(parent, &c.transform);
        if !c.children.is_empty() {
            flatten(cx, &c.children, &w, &path_name);
            continue;
        }
        let id = format!("F{}", cx.features.len() + 1);
        let file = c.part.clone().unwrap_or_else(|| c.model.clone());
        let given = cx.parts.iter().find(|(k, _)| k.eq_ignore_ascii_case(&file)).map(|p| p.1.clone());
        let resolved = given.is_some();
        // an unsupplied part is an empty `ck-part` document: it regenerates to nothing (no component, no error) and is marked
        let part = given.unwrap_or_else(|| Obj::new().put("format", "ck-part").put("version", 1).put("name", c.model.as_str()).put("features", Json::Arr(vec![])).build());
        let col = |k: usize| num_arr(&[w[k], w[4 + k], w[8 + k]]);
        let place = Obj::new().put("o", num_arr(&[w[3], w[7], w[11]])).put("x", col(0)).put("y", col(1)).put("z", col(2)).build();
        if !resolved {
            cx.unresolved.push(file.clone());
        }
        cx.features.push(
            Obj::new()
                .put("type", "insertPart")
                .put("id", id.as_str())
                .put("name", path_name.as_str())
                .put("part", part)
                .put("source", file.as_str())
                .put("place", place)
                .put("fixed", false)
                .put("hidden", c.hidden)
                .put("unresolved", !resolved)
                .put("configuration", c.configuration.as_str())
                .build(),
        );
    }
}

/// Most mates listed in the reply (the counts cover all).
const MAX_MATES_REPLY: usize = 5000;

fn operand_json(o: &MateOperand) -> Json {
    Obj::new().put("component", o.component.as_deref().map_or(Json::Null, Json::from)).put("ref", o.reference.as_deref().map_or(Json::Null, Json::from)).put("kind", o.kind).build()
}

/// Adds `mates`, `matesStream`, `mateSummary` and `mateNotes` for the configuration read.
fn mates_json(o: Obj, c: &ck_sldprt::container::Container, tree: &ck_sldprt::AsmTree, limits: &ck_sldprt::Limits) -> Obj {
    let lists = mates::read_container(c, limits);
    let id = tree.configurations.iter().find(|x| x.name == tree.configuration).and_then(|x| x.id).or(if tree.configurations.is_empty() { Some(0) } else { None });
    let mut notes: Vec<String> = vec![];
    let Some(list) = mates::list_for(&lists, id) else {
        if !lists.is_empty() {
            notes.push("the selected configuration has no mate list stream (its mates are not stored separately)".to_string());
        }
        return o
            .put("mates", Json::Arr(vec![]))
            .put("matesStream", Json::Null)
            .put("mateSummary", Obj::new().put("total", 0).build())
            .put("mateNotes", Json::Arr(notes.into_iter().map(Json::Str).collect()));
    };
    let checks = mates::check_mates(tree, list);
    let (mut by_type, mut holds, mut violated, mut unchecked, mut suppressed) = (std::collections::BTreeMap::<&str, usize>::new(), 0, 0, 0, 0);
    let mut arr = vec![];
    for (m, ck) in list.mates.iter().zip(checks.iter()) {
        *by_type.entry(m.kind).or_default() += 1;
        match ck.status {
            "holds" => holds += 1,
            "violated" => violated += 1,
            _ => unchecked += 1,
        }
        suppressed += m.suppressed as usize;
        if arr.len() < MAX_MATES_REPLY {
            let side = |k: usize| m.operands.get(k).map_or(Json::Null, operand_json);
            let check = Obj::new().put("status", ck.status).put("reason", ck.reason.map_or(Json::Null, Json::from)).put("residual", ck.residual.map_or(Json::Null, Json::Num)).build();
            arr.push(
                Obj::new()
                    .put("type", m.kind)
                    .put("name", m.name.as_str())
                    .put("id", m.id)
                    .put("class", m.class.as_deref().map_or(Json::Null, Json::from))
                    .put("a", side(0))
                    .put("b", side(1))
                    .put("more", Json::Arr(m.operands.iter().skip(2).map(operand_json).collect()))
                    .put("value", Json::Null)
                    .put("alignment", Json::Null)
                    .put("suppressed", m.suppressed)
                    .put("check", check)
                    .build(),
            );
        }
    }
    if list.mates.len() > MAX_MATES_REPLY {
        notes.push(format!("{} mates listed of {}", MAX_MATES_REPLY, list.mates.len()));
    }
    notes.extend(list.notes.iter().cloned());
    notes.push("`value` (distance, angle) and `alignment` are not decoded; faces are named by the part feature that made them, so only mates between principal planes are checked".to_string());
    let summary = Obj::new()
        .put("total", list.mates.len())
        .put("suppressed", suppressed)
        .put("folders", list.folders)
        .put("byType", Json::Obj(by_type.into_iter().map(|(k, v)| (k.to_string(), Json::from(v))).collect()))
        .put("holds", holds)
        .put("violated", violated)
        .put("unchecked", unchecked)
        .build();
    o.put("mates", Json::Arr(arr)).put("matesStream", format!("Config-{}-MatesList", list.config_id).as_str()).put("mateSummary", summary).put("mateNotes", Json::Arr(notes.into_iter().map(Json::Str).collect()))
}

pub fn sldasm_tree_hook(cx: &mut Cx) -> Result<Json, String> {
    let data = cx.msg.get_str("data").unwrap_or("").to_string();
    let bytes = crate::engine::base64_decode(&data, "sldasmTree")?;
    let wanted = cx.msg.get_str("configuration").map(|s| s.to_string());
    let limits = ck_sldprt::Limits::default();
    let fail = |e: ck_sldprt::SldprtError| Ok(Obj::new().put("ok", false).put("code", e.code).put("reason", e.message.as_str()).build());
    let container = match ck_sldprt::container::open(&bytes, &limits) {
        Ok(c) => c,
        Err(e) => return fail(e),
    };
    let tree = match ck_sldprt::asm::read_container(&container, wanted.as_deref(), &limits) {
        Ok(t) => t,
        Err(e) => return fail(e),
    };
    let mut o = Obj::new()
        .put("ok", true)
        .put("name", tree.name.as_str())
        .put("configuration", tree.configuration.as_str())
        .put("configurations", Json::Arr(tree.configurations.iter().map(|c| Obj::new().put("name", c.name.as_str()).put("active", c.active).put("id", c.id.map_or(Json::Null, |v| Json::from(v as usize))).build()).collect()))
        .put("count", tree.count)
        .put("components", Json::Arr(tree.components.iter().map(component_json).collect()))
        .put("notes", Json::Arr(tree.notes.iter().map(|n| Json::Str(n.clone())).collect()));
    let want_mates = cx.msg.get_bool("mates") != Some(false);
    if want_mates {
        o = mates_json(o, &container, &tree, &limits);
    }
    if cx.msg.get_bool("asPart") == Some(true) {
        let supplied: Vec<(String, Json)> = match cx.msg.get("parts") {
            Some(Json::Obj(kv)) => kv.clone(),
            _ => vec![],
        };
        let identity = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let mut c = Ctx { parts: &supplied, features: vec![], unresolved: vec![], skipped: 0 };
        flatten(&mut c, &tree.components, &identity, "");
        let mut un = c.unresolved.clone();
        un.sort();
        un.dedup();
        let part = Obj::new().put("format", "ck-part").put("version", 1).put("name", tree.name.as_str()).put("features", Json::Arr(c.features)).build();
        o = o
            .put("part", part)
            .put("unresolved", Json::Arr(un.into_iter().map(Json::Str).collect()))
            .put("suppressedSkipped", c.skipped);
        if want_mates {
            o = o.put(
                "mateMapping",
                Obj::new()
                    .put("mapped", 0)
                    .put("reason", "the kernel `mate` feature joins faces of the inserted parts by their persistent names; a SolidWorks mate names its faces by the part feature that made them (and a principal plane has no face): the references are not resolved to kernel faces, so no mate feature is written (see `mates` for the decoded mates and their checks)")
                    .build(),
            );
        }
    }
    Ok(o.build())
}
