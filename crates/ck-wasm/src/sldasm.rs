//! `sldasmTree`: the component tree of a SolidWorks assembly (`.SLDASM`) as JSON, and optionally as a part document of
//! `insertPart` features (ck-sldprt `asm`). Read only. See book/src/io/sldprt.md.

use ck_json::{Json, Obj};
use ck_server::session::Cx;
use ck_sldprt::asm::AsmComponent;

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

pub fn sldasm_tree_hook(cx: &mut Cx) -> Result<Json, String> {
    let data = cx.msg.get_str("data").unwrap_or("").to_string();
    let bytes = crate::engine::base64_decode(&data, "sldasmTree")?;
    let wanted = cx.msg.get_str("configuration").map(|s| s.to_string());
    let tree = match ck_sldprt::asm::read_asm_with(&bytes, wanted.as_deref(), &ck_sldprt::Limits::default()) {
        Ok(t) => t,
        Err(e) => return Ok(Obj::new().put("ok", false).put("code", e.code).put("reason", e.message.as_str()).build()),
    };
    let mut o = Obj::new()
        .put("ok", true)
        .put("name", tree.name.as_str())
        .put("configuration", tree.configuration.as_str())
        .put("configurations", Json::Arr(tree.configurations.iter().map(|c| Obj::new().put("name", c.name.as_str()).put("active", c.active).build()).collect()))
        .put("count", tree.count)
        .put("components", Json::Arr(tree.components.iter().map(component_json).collect()))
        .put("notes", Json::Arr(tree.notes.iter().map(|n| Json::Str(n.clone())).collect()));
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
    }
    Ok(o.build())
}
