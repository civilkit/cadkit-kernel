//! Wire request `meshToSolid` (docs/agent-queries.md): the message in, the report or the refusal out. The engine stores the
//! body in the tag and answers a regen-shaped reply (like `importSkp`); this module only knows the JSON.

use crate::{mesh_to_solid, Converted, Opts, Refusal, Report};
use ck_json::{Json, Obj};
use ck_topo::body::Body;

fn b64(s: &str, what: &str) -> Result<Vec<u8>, Refusal> {
    let s = s.find("base64,").map_or(s, |i| &s[i + 7..]);
    let mut out = Vec::with_capacity(s.len() / 4 * 3);
    let (mut acc, mut nb) = (0u32, 0u32);
    for c in s.bytes() {
        let v = match c {
            b'A'..=b'Z' => c - b'A',
            b'a'..=b'z' => c - b'a' + 26,
            b'0'..=b'9' => c - b'0' + 52,
            b'+' | b'-' => 62,
            b'/' | b'_' => 63,
            b'=' | b'\n' | b'\r' | b' ' | b'\t' => continue,
            _ => return Err(Refusal::new("meshBadInput", format!("{what}: not base64"), "send the typed array's bytes as base64")),
        } as u32;
        acc = (acc << 6) | v;
        nb += 6;
        if nb >= 8 {
            nb -= 8;
            out.push((acc >> nb) as u8);
            acc &= (1 << nb) - 1;
        }
    }
    Ok(out)
}

fn nums(a: &[Json], what: &str) -> Result<Vec<f64>, Refusal> {
    a.iter().map(|v| v.as_f64().ok_or_else(|| Refusal::new("meshBadInput", format!("{what}: every entry must be a number"), "send a flat array of numbers"))).collect()
}

/// Positions and indices of the message: `positions` (numbers) | `positionsF32` | `positionsF64` (base64, little endian);
/// `indices` (numbers) | `indicesU32` (base64); no indices = a triangle soup.
pub fn mesh_of(msg: &Json) -> Result<(Vec<f64>, Option<Vec<u32>>), Refusal> {
    let pos = if let Some(a) = msg.get_arr("positions") {
        nums(a, "positions")?
    } else if let Some(s) = msg.get_str("positionsF32") {
        let b = b64(s, "positionsF32")?;
        if b.len() % 4 != 0 {
            return Err(Refusal::new("meshBadInput", "positionsF32: byte count is not a multiple of 4", "send a Float32Array's bytes"));
        }
        b.chunks_exact(4).map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]) as f64).collect()
    } else if let Some(s) = msg.get_str("positionsF64") {
        let b = b64(s, "positionsF64")?;
        if b.len() % 8 != 0 {
            return Err(Refusal::new("meshBadInput", "positionsF64: byte count is not a multiple of 8", "send a Float64Array's bytes"));
        }
        b.chunks_exact(8).map(|c| f64::from_le_bytes(c.try_into().unwrap())).collect()
    } else {
        return Err(Refusal::new("meshBadInput", "positions are required", "send positions (numbers) or positionsF32 / positionsF64 (base64)"));
    };
    let idx = if let Some(a) = msg.get_arr("indices") {
        let v = nums(a, "indices")?;
        if let Some(bad) = v.iter().find(|x| !(**x >= 0.0 && x.fract() == 0.0 && **x < u32::MAX as f64)) {
            return Err(Refusal::new("meshBadInput", format!("indices: {bad} is not a vertex index"), "indices are whole numbers from 0"));
        }
        Some(v.into_iter().map(|x| x as u32).collect())
    } else if let Some(s) = msg.get_str("indicesU32") {
        let b = b64(s, "indicesU32")?;
        if b.len() % 4 != 0 {
            return Err(Refusal::new("meshBadInput", "indicesU32: byte count is not a multiple of 4", "send a Uint32Array's bytes"));
        }
        Some(b.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect())
    } else {
        None
    };
    Ok((pos, idx))
}

/// `opts` of the message over the defaults.
pub fn opts_of(msg: &Json) -> Opts {
    let mut o = Opts::default();
    let Some(m) = msg.get("opts") else { return o };
    let pos = |k: &str| m.get_f64(k).filter(|v| *v > 0.0 && v.is_finite());
    o.weld_tol = pos("weldTol");
    o.plane_tol = pos("planeTol");
    if let Some(v) = pos("angleTolDeg") {
        o.angle_tol_deg = v.min(10.0);
    }
    if let Some(v) = pos("maxTriangles") {
        o.max_triangles = v as usize;
    }
    if let Some(v) = pos("maxFaces") {
        o.max_faces = v as usize;
    }
    if let Some(v) = m.get_bool("analytic") {
        o.analytic = v;
    }
    if let Some(v) = m.get_bool("deviation") {
        o.deviation = v;
    }
    match m.get("close") {
        Some(ck_json::Json::Bool(true)) => o.close = Some(Default::default()),
        Some(c @ ck_json::Json::Obj(_)) => {
            let mut co = crate::close::CloseOpts::default();
            co.tol = c.get_f64("tolerance").filter(|v| *v > 0.0 && v.is_finite());
            if let Some(v) = c.get_bool("fill") {
                co.fill = v;
            }
            if let Some(v) = c.get_f64("maxHoleEdges").filter(|v| *v >= 3.0) {
                co.max_hole_edges = v as usize;
            }
            if let Some(v) = c.get_f64("maxHoleArea").filter(|v| *v > 0.0 && v.is_finite()) {
                co.max_hole_area = v;
            }
            if let Some(v) = c.get_f64("planarTol").filter(|v| *v > 0.0 && v.is_finite()) {
                co.planar_tol = v;
            }
            o.close = Some(co);
        }
        _ => {}
    }
    o.fit.tol = pos("tol");
    o.fit.fit_tol = pos("fitTol");
    o
}

fn v3(v: ck_geom::Vec3) -> Json {
    Json::Arr(vec![Json::Num(v.x), Json::Num(v.y), Json::Num(v.z)])
}

/// Counts of the body's faces by surface kind.
pub fn face_types(b: &Body) -> Json {
    let mut o = Obj::new();
    let mut counts: Vec<(&'static str, usize)> = vec![];
    for f in b.face_ids() {
        let k = ck_topo::body::wire_kind(&b.face(f).surface);
        match counts.iter_mut().find(|(n, _)| *n == k) {
            Some(c) => c.1 += 1,
            None => counts.push((k, 1)),
        }
    }
    counts.sort();
    for (k, n) in counts {
        o = o.put(k, n as f64);
    }
    o.build()
}

/// The report as JSON (`import` of the reply).
pub fn report_json(name: &str, b: &Body, r: &Report) -> Json {
    let mut o = Obj::new()
        .put("format", "mesh")
        .put("name", name)
        .put("triangles", r.input_triangles as f64)
        .put("vertices", r.input_vertices as f64)
        .put("weldedVertices", r.welded_vertices as f64)
        .put("diagonal", r.diagonal)
        .put("weldTol", r.weld_tol)
        .put("planeTol", r.plane_tol)
        .put("angleTolDeg", r.angle_tol_deg)
        .put("collapsed", r.collapsed as f64)
        .put("duplicates", r.duplicates as f64)
        .put("slivers", r.slivers as f64)
        .put("flipped", r.flipped as f64)
        .put("insideOut", r.inside_out as f64)
        .put("components", r.components as f64)
        .put("lumps", r.lumps as f64)
        .put("voids", r.voids as f64)
        .put("planarFaces", r.planar_faces as f64)
        .put("faces", b.num_faces() as f64)
        .put("faceTypes", face_types(b))
        .put("volume", Obj::new().put("mesh", r.mesh_volume).put("body", r.body_volume).put("relative", r.volume_rel_err()).build())
        .put("issues", r.issues as f64)
        .put("healFixes", r.heal_fixes as f64);
    if let Some(c) = &r.closing {
        o = o.put("closing", c.json());
    }
    if let Some(a) = &r.analytic {
        o = o.put("analytic", crate::analytic::report_json(a));
    }
    if let Some(d) = &r.deviation {
        o = o.put(
            "deviation",
            Obj::new()
                .put("max", d.max)
                .put("rms", d.rms)
                .put("meshToBody", d.max_mesh_to_body)
                .put("bodyToMesh", d.max_body_to_mesh)
                .put("samples", d.samples as f64)
                .put("chord", d.chord)
                .put("at", v3(d.at))
                .build(),
        );
    }
    o.build()
}

/// The refusal as `{code, params:{message, hint, ..data}}` (the shape of an import error).
pub fn refusal_json(r: &Refusal) -> Json {
    let mut p = Obj::new().put("message", r.message.as_str()).put("hint", r.hint.as_str());
    for (k, v) in &r.data {
        p = p.put(k, v.clone());
    }
    Obj::new().put("code", r.code).put("params", p.build()).build()
}

/// Run the request: the converted body and its report, or the refusal.
pub fn run(msg: &Json) -> Result<Converted, Refusal> {
    let (pos, idx) = mesh_of(msg)?;
    let o = opts_of(msg);
    mesh_to_solid(&pos, idx.as_deref(), &o)
}
