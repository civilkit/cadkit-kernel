//! Benchmark (ignored): interactive edit latency on the fan shroud with the regen cache warm.
//! `cargo test --profile fast -p ck-wasm --test edit_latency -- --ignored --nocapture`
//! Prints wall time, instructions retired (macOS), reply size and cache statistics per edit.
use ck_json::Json;
use std::time::Instant;

#[cfg(target_os = "macos")]
fn instructions() -> f64 {
    extern "C" {
        fn proc_pid_rusage(pid: i32, flavor: i32, buffer: *mut u64) -> i32;
    }
    let mut buf = [0u64; 64];
    let r = unsafe { proc_pid_rusage(std::process::id() as i32, 4, buf.as_mut_ptr()) };
    if r == 0 { buf[2 + 29] as f64 * 1e-9 } else { f64::NAN }
}
#[cfg(not(target_os = "macos"))]
fn instructions() -> f64 {
    f64::NAN
}

fn parse(r: &[u8]) -> Json {
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

/// Hash of the reply without the cache statistics (they legitimately change with the cache layout).
fn digest(r: &[u8]) -> u64 {
    use std::hash::{Hash, Hasher};
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    let j = &r[4..4 + jl];
    let cut = j.windows(13).position(|w| w == b"\"cacheStats\":").unwrap_or(jl);
    let mut h = std::collections::hash_map::DefaultHasher::new();
    j[..cut].hash(&mut h);
    r[4 + jl..].iter().skip_while(|b| **b == b' ').for_each(|b| b.hash(&mut h));
    h.finish()
}

fn regen(part: &str, tag: &str) -> (Vec<u8>, f64, f64) {
    let (t, i0) = (Instant::now(), instructions());
    let r = ck_server::handle(format!(r#"{{"id":1,"msg":{{"type":"regen","tag":"{tag}","part":{part},"opts":{{"cacheStats":true}}}}}}"#).as_bytes());
    (r, t.elapsed().as_secs_f64(), instructions() - i0)
}

#[test]
#[ignore]
fn fan_edit_latency() {
    std::thread::Builder::new().stack_size(256 << 20).spawn(|| {
        ck_wasm::engine::install();
        let path = std::env::var("CK_PART").unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"), "/../../tools/cases/perf/fan_shroud.ckpart").into());
        let base = std::fs::read_to_string(path).unwrap();
        let (r, t, i) = regen(&base, "main");
        if let Ok(d) = std::env::var("CK_DUMP") {
            std::fs::write(d, &r).unwrap();
        }
        println!("cold            {t:6.2}s {i:7.2} G  reply {} KB  digest {:016x}", r.len() / 1024, digest(&r));
        // CK_LOOP=n: n edits of the last feature's radius (distinct values), for a sampling profiler
        if let Some(n) = std::env::var("CK_LOOP").ok().and_then(|v| v.parse::<usize>().ok()) {
            for k in 0..n {
                let at = base.find("\"id\": \"F49\"").unwrap();
                let rel = base[at..].find("\"radius\": 0.5").unwrap();
                let mut e = base.clone();
                e.replace_range(at + rel..at + rel + 13, &format!("\"radius\": 0.{}", 51 + k));
                let (_, t, i) = regen(&e, "main");
                println!("loop {k} {t:6.2}s {i:7.2} G");
            }
            return;
        }
        let edits: [(&str, &str, &str); 3] = [
            ("last (F49)", r#""radius": 0.5"#, r#""radius": 0.6"#),
            ("F48", r#""radius": 2"#, r#""radius": 1.9"#),
            ("F42", r#""angle": 360"#, r#""angle": 359"#),
        ];
        for (name, a, b) in edits {
            assert!(base.contains(a), "{name}");
            // replace the first occurrence after the feature id
            let id = if name.starts_with("last") { "\"F49\"" } else if name == "F48" { "\"F48\"" } else { "\"F42\"" };
            let at = base.find(&format!("\"id\": {id}")).unwrap();
            let rel = base[at..].find(a).unwrap();
            let mut e = base.clone();
            e.replace_range(at + rel..at + rel + a.len(), b);
            let (r, t, i) = regen(&e, "main");
            let st = parse(&r).get("result").unwrap().get("cacheStats").unwrap().clone();
            println!("edit {name:10} {t:6.2}s {i:7.2} G  reply {} KB  digest {:016x} reused {} ran {} tessLookups {} hits {}", r.len() / 1024, digest(&r),
                st.get_f64("reusedFeatures").unwrap(), st.get_f64("ranFeatures").unwrap(), st.get_f64("tessFaceLookups").unwrap(), st.get_f64("tessFaceHits").unwrap());
            let (_, t2, i2) = regen(&base, "main");
            println!("  back to base  {t2:6.2}s {i2:7.2} G");
        }
    }).unwrap().join().unwrap();
}
