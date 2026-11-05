//! Experiment harness for the `(deltas)` transmit (see docs/sldprt.md, "Deltas"). Not used by the importer.
//!
//! `deltas_rules inventory FILE...` prints, per file, the section / change / node counts of the deltas transmit and
//! whether its current mark equals the partition's WORLD `current_id`.
//! `deltas_rules score RULE FILE...` rebuilds the partition body with the deltas applied under RULE (`none`, `overlay`,
//! `rollback:N`) and prints its faces, closed-shell issues and volume / area against SolidWorks' stored mass properties.

use ck_sldprt::container::{self, TransmitKind};
use ck_sldprt::error::Limits;
use ck_sldprt::xt::{self, DeltaRule};

fn label(p: &str) -> String {
    p.rsplit('/').nth(1).unwrap_or("?").to_string()
}

fn inventory(p: &str, l: &Limits) {
    let name = label(p);
    let b = std::fs::read(p).unwrap();
    let Ok(c) = container::open(&b, l) else { return println!("{name}: not opened") };
    let ts = c.transmits(l).unwrap_or_default();
    for td in ts.iter().filter(|t| t.kind == TransmitKind::Deltas) {
        match xt::read_deltas(&td.data, l) {
            Err(e) => println!("{name}: ERR {} {}", e.code, e.message),
            Ok(d) => {
                let cur = d.current_pmark.as_ref().unwrap();
                let part = ts.iter().find(|t| t.kind == TransmitKind::Partition && t.stream == td.stream).and_then(|t| xt::read_xt(&t.data, l).ok());
                let w = part.as_ref().and_then(|x| x.nodes.values().find(|n| n.ty == 101));
                let wc = w.map_or(-1, |n| n.i("current_id"));
                let backward = d.pmark_sections.iter().flat_map(|s| s.pmarks.iter()).chain(std::iter::once(cur)).all(|n| n.i("delta_is_forward") == 0);
                println!(
                    "{name}: sections={} changes={} nodes={} map_entries={} empty={} to_terminator={} current_id_match={} backward_only={}",
                    1 + d.maps.len() + d.pmark_sections.len(),
                    d.pmark_sections.len() + 1,
                    d.node_count(),
                    d.maps.iter().map(|m| m.map.len()).sum::<usize>(),
                    d.node_count() == 0,
                    d.trailing_bytes == 0,
                    wc == cur.i("identifier"),
                    backward
                );
            }
        }
    }
}

fn score(rule: DeltaRule, p: &str, l: &Limits) {
    let name = label(p);
    let b = std::fs::read(p).unwrap();
    let r = std::panic::catch_unwind(|| -> Result<String, String> {
        let c = container::open(&b, l).map_err(|e| e.code.to_string())?;
        let ts = c.transmits(l).map_err(|e| e.code.to_string())?;
        let tp = ts.iter().filter(|t| t.kind == TransmitKind::Partition && !t.stream.contains("Ghost")).max_by_key(|t| t.data.len()).ok_or("no partition")?;
        let mut x = xt::read_xt(&tp.data, l).map_err(|e| e.code.to_string())?;
        if let Some(td) = ts.iter().find(|t| t.kind == TransmitKind::Deltas && t.stream == tp.stream) {
            let d = xt::read_deltas(&td.data, l).map_err(|e| e.code.to_string())?;
            xt::apply_deltas(&mut x, &d, rule);
        }
        let ir = ck_sldprt::ir::extract(&x, l).map_err(|e| e.code.to_string())?;
        let built = ck_sldprt::build::build(&[&ir], l).map_err(|e| e.code.to_string())?;
        let meta = ck_sldprt::meta::read(&c, l);
        let mp = ck_topo::mass::mass_props(&built.body);
        let mut codes: Vec<&str> = ck_check::validate_cap(&built.body, 1e-6, ck_check::DEFAULT_TOL_CAP).iter().map(|i| i.code).collect();
        let n = codes.len();
        codes.sort();
        codes.dedup();
        let (sv, sa) = meta.mass.as_ref().map_or((f64::NAN, f64::NAN), |m| (m.volume_m3 * 1e9, m.area_m2 * 1e6));
        Ok(format!("faces={:4} vol={:14.3} sw={:14.3} relV={:+.5} relA={:+.5} issues={n} {codes:?}", built.body.num_faces(), mp.volume, sv, (mp.volume - sv) / sv, (mp.area - sa) / sa))
    });
    match r {
        Ok(Ok(s)) => println!("{name:32} {s}"),
        Ok(Err(e)) => println!("{name:32} ERR {e}"),
        Err(_) => println!("{name:32} PANIC"),
    }
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    let l = Limits::default();
    match a.get(1).map(String::as_str) {
        Some("inventory") => a[2..].iter().for_each(|p| inventory(p, &l)),
        Some("score") => {
            let rule = match a[2].as_str() {
                "none" => DeltaRule::PartitionOnly,
                "overlay" => DeltaRule::Overlay,
                s => DeltaRule::RollBack(s.strip_prefix("rollback:").and_then(|n| n.parse().ok()).expect("rule: none | overlay | rollback:N")),
            };
            a[3..].iter().for_each(|p| score(rule, p, &l));
        }
        _ => eprintln!("usage: deltas_rules inventory FILE... | score none|overlay|rollback:N FILE..."),
    }
}
