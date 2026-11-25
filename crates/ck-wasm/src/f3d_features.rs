//! `f3dFeatures`: the timeline of a Fusion 360 design as a ck-part feature list, with the check against the imported body
//! (docs/f3d-lazy.md). The mapping lives here, in the main kernel, because it regenerates the history built so far to find the
//! edges (fillets) and faces (face-profile extrudes, region picking) it names, which needs the kernel's regen; the lazily loaded
//! decoder wasm only decodes (`f3dDump`) and does not link it.

use ck_f3d_dump::Dump;
use ck_f3d_features::{guard, timeline_to_features, Measure, Status, Verdict};
use ck_json::{Json, Obj};
use ck_server::session::Cx;

fn measure(m: &Measure) -> Json {
    let a = |v: [f64; 3]| Json::Arr(v.iter().map(|x| Json::Num(*x)).collect());
    Obj::new().put("volume", m.volume).put("min", a(m.min)).put("max", a(m.max)).put("solids", m.solids as f64).build()
}

fn num(v: Option<f64>) -> Json {
    v.map_or(Json::Null, Json::Num)
}

fn read_measure(j: &Json) -> Option<Measure> {
    let v3 = |k: &str| -> Option<[f64; 3]> {
        let a = j.get(k)?.as_arr()?;
        Some([a.first()?.as_f64()?, a.get(1)?.as_f64()?, a.get(2)?.as_f64()?])
    };
    Some(Measure { volume: j.get("volume")?.as_f64()?, min: v3("min")?, max: v3("max")?, solids: j.get("solids")?.as_f64()? as usize })
}

/// `{type:"f3dFeatures", designs:[{label, dump, reference?, referenceError?}]}` (the `designs` of the decoder's `f3dDump`;
/// `dump` may be the JSON object or its text) -> `{designs:[{label, part, mapped, placeholders, items[], notes[], check}]}`;
/// a design whose `dump` is null is left out. `check` is the regenerate-and-compare verdict against `reference`. Read only.
pub fn f3d_features_hook(cx: &mut Cx) -> Result<Json, String> {
    let Some(designs) = cx.msg.get("designs").and_then(|d| d.as_arr()) else { return Err("f3dFeatures: designs (the decoder's f3dDump designs) is required".to_string()) };
    let mut out = vec![];
    for d in designs {
        let label = d.get_str("label").unwrap_or("").to_string();
        let text = match d.get("dump") {
            None | Some(Json::Null) => continue,
            Some(Json::Str(s)) => s.clone(),
            Some(j) => ck_json::to_string(j),
        };
        let dump = Dump::from_json(&text).map_err(|e| format!("f3dFeatures: the dump of {label:?} does not read: {e}"))?;
        let reference = d.get("reference").and_then(read_measure);
        let imp = timeline_to_features(&dump);
        let items = imp
            .items
            .iter()
            .map(|i| {
                Obj::new()
                    .put("index", i.index as f64)
                    .put("name", i.name.as_str())
                    .put("type", i.object_type.as_str())
                    .put("status", if i.status == Status::Mapped { "mapped" } else { "placeholder" })
                    .put("features", Json::Arr(i.features.iter().map(|f| Json::Str(f.clone())).collect()))
                    .put_opt("note", i.note.as_deref())
                    .build()
            })
            .collect();
        let notes = imp.notes.iter().map(|n| Obj::new().put("item", n.item.map_or(Json::Null, |i| Json::Num(i as f64))).put("text", n.text.as_str()).build()).collect();
        let part = ck_json::parse(&serde_json::to_string(&imp.part()).map_err(|e| e.to_string())?).map_err(|e| format!("{e:?}"))?;
        let check = guard::verify(&imp, reference.as_ref());
        let check_json = Obj::new()
            .put("verdict", if check.verdict == Verdict::Verified { "verified" } else { "unverified" })
            .put("reference", check.reference.as_ref().map_or(Json::Null, measure))
            .put("history", check.history.as_ref().map_or(Json::Null, measure))
            .put("volumeDelta", num(check.volume_delta))
            .put("boxDelta", num(check.box_delta))
            .put("reason", check.reason.as_deref().map_or(Json::Null, |r| Json::Str(r.into())))
            .put("errors", Json::Arr(check.errors.iter().map(|e| Json::Str(e.clone())).collect()))
            .build();
        out.push(
            Obj::new()
                .put("label", label.as_str())
                .put("part", part)
                .put("mapped", imp.mapped() as f64)
                .put("placeholders", imp.placeholders() as f64)
                .put("items", Json::Arr(items))
                .put("notes", Json::Arr(notes))
                .put("check", check_json)
                .build(),
        );
    }
    Ok(Obj::new().put("designs", Json::Arr(out)).build())
}
