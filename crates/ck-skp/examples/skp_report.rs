//! `cargo run -p ck-skp --release --example skp_report -- <file-or-dir>...`
//!
//! One JSON line per `.skp` file: container, version, parse result, entity totals, baked bounding box (mm), triangle count
//! and area. Used for the parity check against OpenSKP (see tests/openskp_parity.rs and the NOTICE).

use ck_skp::{bake, parse, BakeOptions, Limits};
use std::path::PathBuf;

fn collect(p: &PathBuf, out: &mut Vec<PathBuf>) {
    if p.is_dir() {
        let mut v: Vec<_> = std::fs::read_dir(p).map(|d| d.filter_map(|e| e.ok().map(|e| e.path())).collect()).unwrap_or_default();
        v.sort();
        for c in v {
            collect(&c, out);
        }
    } else if p.extension().map_or(false, |e| e.eq_ignore_ascii_case("skp")) {
        out.push(p.clone());
    }
}

fn esc(s: &str) -> String {
    s.chars().flat_map(|c| if c == '"' || c == '\\' { vec!['\\', c] } else if (c as u32) < 32 { vec![' '] } else { vec![c] }).collect()
}

fn main() {
    let mut files = vec![];
    let mut hidden = true;
    for a in std::env::args().skip(1) {
        if a == "--no-hidden" {
            hidden = false;
            continue;
        }
        collect(&PathBuf::from(a), &mut files);
    }
    for f in files {
        let data = match std::fs::read(&f) {
            Ok(d) => d,
            Err(e) => {
                println!("{{\"file\":\"{}\",\"ok\":false,\"error\":\"read: {}\"}}", esc(&f.display().to_string()), esc(&e.to_string()));
                continue;
            }
        };
        let lim = Limits::default();
        let t0 = std::time::Instant::now();
        match parse(&data) {
            Err(e) => println!("{{\"file\":\"{}\",\"ok\":false,\"code\":\"{}\",\"error\":\"{}\",\"bytes\":{}}}", esc(&f.display().to_string()), e.code, esc(&e.to_string()), data.len()),
            Ok(m) => {
                let (v, e, fc, i) = m.totals();
                let opt = BakeOptions { include_hidden: hidden, openskp_compat: hidden, ..Default::default() };
                let (bake_s, bb) = match bake(&m, &lim, &opt) {
                    Ok(s) => {
                        let bb = s.bbox().map(|(l, h)| format!("[{:.3},{:.3},{:.3},{:.3},{:.3},{:.3}]", l[0], l[1], l[2], h[0], h[1], h[2])).unwrap_or("null".into());
                        (format!("\"prims\":{},\"tris\":{},\"area_mm2\":{:.3},\"materials\":{},\"textures\":{}", s.primitives.len(), s.triangle_count(), s.area(), s.materials.len(), s.textures.len()), bb)
                    }
                    Err(e) => (format!("\"bake_error\":\"{}\"", esc(&e.to_string())), "null".into()),
                };
                println!(
                    "{{\"file\":\"{}\",\"ok\":true,\"container\":\"{:?}\",\"version\":\"{}\",\"defs\":{},\"vertices\":{},\"edges\":{},\"faces\":{},\"instances\":{},\"materials\":{},\"layers\":{},\"warnings\":{},\"warning_text\":\"{}\",\"ms\":{},{},\"bbox_mm\":{}}}",
                    esc(&f.display().to_string()),
                    m.container,
                    esc(&m.version),
                    m.definitions.len(),
                    v,
                    e,
                    fc,
                    i,
                    m.materials.len(),
                    m.layers.len(),
                    m.warnings.len(),
                    esc(&m.warnings.iter().map(|w| format!("{}: {}", w.code, w.message)).collect::<Vec<_>>().join(" | ")),
                    t0.elapsed().as_millis(),
                    bake_s,
                    bb
                );
            }
        }
    }
}
