//! What becomes of each part of a part file (no files are part of the repository): its outcome, faces read / given,
//! and the reasons it is not a solid, as `code: message` with numbers blanked, then a histogram of those.
//!
//!     cargo run --release -p ck-model --example sldprt_parts -- <file.sldprt>
//!
//! With `RAW=1` every reason's own message is printed too.
use std::collections::BTreeMap;

fn main() {
    let a = std::env::args().nth(1).expect("file");
    let bytes = std::fs::read(a).unwrap();
    let imp = ck_sldprt::import_sldprt(&bytes).unwrap();
    let sorted = ck_model::features::import_body::sort_sldprt(imp.body, &imp.parts, "import", None);
    let mut outcomes: BTreeMap<&str, usize> = BTreeMap::new();
    let mut hist: BTreeMap<String, usize> = BTreeMap::new();
    let mut per_part: BTreeMap<String, usize> = BTreeMap::new();
    for p in &sorted.parts {
        *outcomes.entry(p.outcome.as_str()).or_default() += 1;
        println!("part {} {} faces {}/{} reasons {}", p.label.split(' ').next().unwrap_or(""), p.outcome.as_str(), p.faces, p.faces_total, p.reasons.len());
        let mut seen = std::collections::BTreeSet::new();
        for r in &p.reasons {
            if std::env::var("RAW").is_ok() {
                println!("  raw {}: {}", r.code, r.message);
            }
            let msg: String = r.message.split(':').skip(1).collect::<Vec<_>>().join(":");
            let mut norm = String::new();
            let mut prev_digit = false;
            for c in msg.chars() {
                let d = c.is_ascii_digit() || (prev_digit && (c == '.' || c == 'e' || c == '-' || c == '+'));
                if d {
                    if !prev_digit {
                        norm.push('N');
                    }
                } else {
                    norm.push(c);
                }
                prev_digit = d;
            }
            let key = format!("{} |{}", r.code, norm.trim());
            *hist.entry(key.clone()).or_default() += 1;
            seen.insert(key);
        }
        for k in seen {
            *per_part.entry(k).or_default() += 1;
        }
    }
    println!("outcomes {outcomes:?}");
    println!("--- reasons (count of faces / count of parts)");
    for (k, n) in &hist {
        println!("{n:5} {:3}  {k}", per_part[k]);
    }
}
