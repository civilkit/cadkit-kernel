//! `meshDecimate`: a triangle mesh in, the decimated mesh out (ck-mesh quadric decimation within a deviation bound).
//! Stateless: no tag is read or written. Reply: the report (`ck_mesh::decimate::DecimateReport::json`) plus `positions`
//! (f32 buffer, 3 per vertex) and `indices` (u32 buffer, 3 per triangle); a refusal answers `errors` as `meshToSolid` does.

use ck_json::{Json, Obj};
use ck_server::session::Cx;

pub fn reply(cx: &mut Cx) -> Result<Json, String> {
    match ck_mesh::decimate::run(cx.msg) {
        Err(r) => {
            let err = Json::Arr(vec![Json::Str("import".into()), ck_mesh::request::refusal_json(&r)]);
            Ok(Obj::new().put("errors", Json::Arr(vec![err])).build())
        }
        Ok(d) => {
            let pos: Vec<f32> = d.pts.iter().flat_map(|p| [p.x as f32, p.y as f32, p.z as f32]).collect();
            let idx: Vec<u32> = d.tris.iter().flatten().copied().collect();
            let mut o = d.report.json();
            if let Json::Obj(kv) = &mut o {
                kv.push(("errors".into(), Json::Arr(vec![])));
                kv.push(("positions".into(), cx.bufs.f32(&pos)));
                kv.push(("indices".into(), cx.bufs.u32(&idx)));
            }
            Ok(o)
        }
    }
}
