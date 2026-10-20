//! Simple mesh exports of a baked [`Scene`]: Wavefront OBJ (+ MTL) and binary glTF (GLB). Optional conveniences; the
//! kernel import path does not use them.
//!
//! Part of ck-skp, a Rust reimplementation of the SketchUp reader of OpenSKP (MIT, (c) Ahsan Mehmood), see NOTICE.
//! Like OpenSKP's exporters, glTF is Y-up in metres; OBJ keeps millimetres and SketchUp's Z-up frame.

use crate::scene::Scene;
use std::fmt::Write;

/// `(obj, mtl)` text. Materials are `m<index>`; one `usemtl` run per primitive.
pub fn to_obj(s: &Scene) -> (String, String) {
    let mut obj = String::from("# ck-skp export, millimetres, Z up\nmtllib scene.mtl\n");
    let mut mtl = String::new();
    for (i, m) in s.materials.iter().enumerate() {
        let _ = writeln!(mtl, "newmtl m{i}\nKd {:.4} {:.4} {:.4}\nd {:.4}", m.rgb[0] as f32 / 255.0, m.rgb[1] as f32 / 255.0, m.rgb[2] as f32 / 255.0, m.opacity);
    }
    let mut base = 1usize;
    for p in &s.primitives {
        let _ = writeln!(obj, "usemtl m{}", p.material);
        for v in &p.positions {
            let _ = writeln!(obj, "v {} {} {}", v[0], v[1], v[2]);
        }
        for n in &p.normals {
            let _ = writeln!(obj, "vn {} {} {}", n[0], n[1], n[2]);
        }
        for t in &p.uvs {
            let _ = writeln!(obj, "vt {} {}", t[0], t[1]);
        }
        for t in p.indices.chunks_exact(3) {
            let (a, b, c) = (base + t[0] as usize, base + t[1] as usize, base + t[2] as usize);
            let _ = writeln!(obj, "f {a}/{a}/{a} {b}/{b}/{b} {c}/{c}/{c}");
        }
        base += p.positions.len();
    }
    (obj, mtl)
}

fn pad4(v: &mut Vec<u8>, byte: u8) {
    while v.len() % 4 != 0 {
        v.push(byte);
    }
}

/// GLB (binary glTF 2.0). Textures are embedded when `textures` is set.
pub fn to_glb(s: &Scene, textures: bool) -> Vec<u8> {
    let mut bin: Vec<u8> = vec![];
    let mut views = String::new();
    let mut accs = String::new();
    let mut prims = String::new();
    let mut nview = 0usize;
    let mut nacc = 0usize;
    let mut view = |bin: &mut Vec<u8>, bytes: &[u8], target: Option<u32>| -> usize {
        pad4(bin, 0);
        let off = bin.len();
        bin.extend_from_slice(bytes);
        if !views.is_empty() {
            views.push(',');
        }
        let _ = write!(views, "{{\"buffer\":0,\"byteOffset\":{off},\"byteLength\":{}", bytes.len());
        if let Some(t) = target {
            let _ = write!(views, ",\"target\":{t}");
        }
        views.push('}');
        nview += 1;
        nview - 1
    };
    let mut meshes = String::new();
    let mut nmesh = 0usize;
    for p in &s.primitives {
        if p.indices.is_empty() {
            continue;
        }
        // Z-up mm -> Y-up m
        let mut pos = Vec::with_capacity(p.positions.len() * 12);
        let (mut lo, mut hi) = ([f32::INFINITY; 3], [f32::NEG_INFINITY; 3]);
        for v in &p.positions {
            let q = [(v[0] * 1e-3) as f32, (v[2] * 1e-3) as f32, (-v[1] * 1e-3) as f32];
            for k in 0..3 {
                lo[k] = lo[k].min(q[k]);
                hi[k] = hi[k].max(q[k]);
                pos.extend_from_slice(&q[k].to_le_bytes());
            }
        }
        let mut nor = Vec::with_capacity(p.normals.len() * 12);
        for n in &p.normals {
            for q in [n[0], n[2], -n[1]] {
                nor.extend_from_slice(&q.to_le_bytes());
            }
        }
        let mut uv = Vec::with_capacity(p.uvs.len() * 8);
        for t in &p.uvs {
            uv.extend_from_slice(&t[0].to_le_bytes());
            uv.extend_from_slice(&(-t[1]).to_le_bytes());
        }
        let mut idx = Vec::with_capacity(p.indices.len() * 4);
        for i in &p.indices {
            idx.extend_from_slice(&i.to_le_bytes());
        }
        let (vp, vn, vt, vi) = (view(&mut bin, &pos, Some(34962)), view(&mut bin, &nor, Some(34962)), view(&mut bin, &uv, Some(34962)), view(&mut bin, &idx, Some(34963)));
        let mut acc = |view: usize, ct: u32, count: usize, ty: &str, mm: Option<([f32; 3], [f32; 3])>| -> usize {
            if !accs.is_empty() {
                accs.push(',');
            }
            let _ = write!(accs, "{{\"bufferView\":{view},\"componentType\":{ct},\"count\":{count},\"type\":\"{ty}\"");
            if let Some((a, b)) = mm {
                let _ = write!(accs, ",\"min\":[{},{},{}],\"max\":[{},{},{}]", a[0], a[1], a[2], b[0], b[1], b[2]);
            }
            accs.push('}');
            nacc += 1;
            nacc - 1
        };
        let (ap, an, at, ai) = (acc(vp, 5126, p.positions.len(), "VEC3", Some((lo, hi))), acc(vn, 5126, p.normals.len(), "VEC3", None), acc(vt, 5126, p.uvs.len(), "VEC2", None), acc(vi, 5125, p.indices.len(), "SCALAR", None));
        if !prims.is_empty() {
            prims.push(',');
        }
        let _ = write!(prims, "{{\"attributes\":{{\"POSITION\":{ap},\"NORMAL\":{an},\"TEXCOORD_0\":{at}}},\"indices\":{ai},\"material\":{}}}", p.material);
        if !meshes.is_empty() {
            meshes.push(',');
        }
        let _ = write!(meshes, "{{\"primitives\":[{{\"attributes\":{{\"POSITION\":{ap},\"NORMAL\":{an},\"TEXCOORD_0\":{at}}},\"indices\":{ai},\"material\":{}}}]}}", p.material);
        nmesh += 1;
    }
    // images
    let mut images = String::new();
    let mut gtex = String::new();
    if textures {
        for (i, t) in s.textures.iter().enumerate() {
            let v = view(&mut bin, &t.data, None);
            if i > 0 {
                images.push(',');
                gtex.push(',');
            }
            let _ = write!(images, "{{\"bufferView\":{v},\"mimeType\":\"{}\"}}", t.mime);
            let _ = write!(gtex, "{{\"source\":{i},\"sampler\":0}}");
        }
    }
    let mut mats = String::new();
    for (i, m) in s.materials.iter().enumerate() {
        if i > 0 {
            mats.push(',');
        }
        let _ = write!(mats, "{{\"pbrMetallicRoughness\":{{\"baseColorFactor\":[{},{},{},{}],\"metallicFactor\":0,\"roughnessFactor\":0.8", m.rgb[0] as f32 / 255.0, m.rgb[1] as f32 / 255.0, m.rgb[2] as f32 / 255.0, m.opacity);
        if let (true, Some(t)) = (textures, m.texture) {
            let _ = write!(mats, ",\"baseColorTexture\":{{\"index\":{t}}}");
        }
        mats.push('}');
        if m.double_sided {
            mats.push_str(",\"doubleSided\":true");
        }
        if m.opacity < 1.0 {
            mats.push_str(",\"alphaMode\":\"BLEND\"");
        } else if textures && m.texture.is_some() {
            mats.push_str(",\"alphaMode\":\"MASK\"");
        }
        mats.push('}');
    }
    pad4(&mut bin, 0);
    let nodes: Vec<String> = (0..nmesh).map(|i| format!("{{\"mesh\":{i}}}")).collect();
    let scene_nodes: Vec<String> = (0..nmesh).map(|i| i.to_string()).collect();
    let mut json = format!("{{\"asset\":{{\"version\":\"2.0\",\"generator\":\"ck-skp\"}},\"scene\":0,\"scenes\":[{{\"nodes\":[{}]}}],\"nodes\":[{}],\"meshes\":[{}],\"materials\":[{}],\"accessors\":[{}],\"bufferViews\":[{}],\"buffers\":[{{\"byteLength\":{}}}]", scene_nodes.join(","), nodes.join(","), meshes, mats, accs, views, bin.len());
    if textures && !s.textures.is_empty() {
        let _ = write!(json, ",\"images\":[{images}],\"textures\":[{gtex}],\"samplers\":[{{\"magFilter\":9729,\"minFilter\":9987,\"wrapS\":10497,\"wrapT\":10497}}]");
    }
    json.push('}');
    let _ = &prims;
    let mut jb = json.into_bytes();
    pad4(&mut jb, b' ');
    let total = 12 + 8 + jb.len() + 8 + bin.len();
    let mut out = Vec::with_capacity(total);
    out.extend_from_slice(b"glTF");
    out.extend_from_slice(&2u32.to_le_bytes());
    out.extend_from_slice(&(total as u32).to_le_bytes());
    out.extend_from_slice(&(jb.len() as u32).to_le_bytes());
    out.extend_from_slice(b"JSON");
    out.extend_from_slice(&jb);
    out.extend_from_slice(&(bin.len() as u32).to_le_bytes());
    out.extend_from_slice(b"BIN\0");
    out.extend_from_slice(&bin);
    out
}
