//! Times a full open (`import_sldprt` + `rebuild_history` + `mass_props`) of part files: `time_open FILE...` or
//! `time_open -l LISTFILE`; prints one line per file (faces, result, steps rebuilt, volume, seconds per stage).
//! Contains no corpus data.

use ck_swedit::{classify, rebuild_history, Class, Status, Tagged};
use std::time::Instant;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let list: Vec<String> = if args.len() == 2 && args[0] == "-l" { std::fs::read_to_string(&args[1]).unwrap().lines().map(String::from).collect() } else { args };
    for p in list {
        let bytes = std::fs::read(&p).unwrap();
        let name = p.rsplit('/').nth(1).unwrap_or("?").to_string();
        let file = p.rsplit('/').next().unwrap_or("?").to_string();
        let t0 = Instant::now();
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                println!("{name}/{file}\tERR {}\t{:.2}s", e.code, t0.elapsed().as_secs_f64());
                continue;
            }
        };
        let t_imp = t0.elapsed().as_secs_f64();
        let t = Tagged::from_import(&imp);
        let t1 = Instant::now();
        let out = rebuild_history(&t);
        let t_reb = t1.elapsed().as_secs_f64();
        let t2 = Instant::now();
        let vol = ck_topo::mass::mass_props(&imp.body).volume;
        let t_mass = t2.elapsed().as_secs_f64();
        let nfeat = imp.meta.features.iter().filter(|f| !matches!(classify(f), Class::Skip)).count();
        let rebuilt = out.steps.iter().filter(|s| s.status == Status::Rebuilt).count();
        let res = if out.exact() { "exact" } else if out.close() { "close" } else if out.prefix > 0 { "partial" } else { "plain" };
        let v = out.verify.as_ref().map(|v| format!("{:.3e}", v.rel)).unwrap_or("-".into());
        println!("{name}/{file}\tfaces={}\t{res}\t{rebuilt}/{nfeat}\tkept={}\trel={v}\tvol={vol:.9e}\timport={t_imp:.2}s\trebuild={t_reb:.2}s\tmass={t_mass:.2}s", imp.body.num_faces(), out.prefix);
    }
}
