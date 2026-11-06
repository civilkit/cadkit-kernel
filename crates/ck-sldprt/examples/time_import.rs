//! Times `import_sldprt` and `mass_props` per file: `time_import FILE...` or `time_import -l LISTFILE` prints one line per file (faces, issue count,
//! volume in mm3 against the stored one, seconds). Contains no corpus data.



/// User CPU seconds of this process (steady under machine load, unlike wall time).
fn cpu() -> f64 {
    extern "C" {
        fn getrusage(who: i32, usage: *mut [i64; 18]) -> i32;
    }
    let mut r = [0i64; 18];
    unsafe { getrusage(0, &mut r) };
    r[0] as f64 + r[1] as f64 * 1e-6
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let list: Vec<String> = if args.len() == 2 && args[0] == "-l" { std::fs::read_to_string(&args[1]).unwrap().lines().map(String::from).collect() } else { args };
    for p in list {
        let bytes = std::fs::read(&p).unwrap();
        let name = p.rsplit('/').nth(1).unwrap_or("?").to_string();
        let c0 = cpu();
        let imp = match ck_sldprt::import_sldprt(&bytes) {
            Ok(i) => i,
            Err(e) => {
                println!("{name}\tERR {}\t{:.2}s", e.code, cpu() - c0);
                continue;
            }
        };
        let t_imp = cpu() - c0;
        let issues = ck_topo::check::check(&imp.body, 1e-4).len();
        let c1 = cpu();
        let mp = ck_topo::mass::mass_props(&imp.body);
        let t_mass = cpu() - c1;
        let sw = imp.sw_mass.as_ref().map_or(f64::NAN, |m| m.volume_m3 * 1e9);
        println!("{name}\tfaces={}\tissues={issues}\tvol={:.12e}\tsw={sw:.6e}\tarea={:.12e}\tcom={:.9e},{:.9e},{:.9e}\tprincipal={:.9e},{:.9e},{:.9e}\tcpu_import={t_imp:.2}s\tcpu_mass={t_mass:.2}s", imp.body.num_faces(), mp.volume, mp.area, mp.com.x, mp.com.y, mp.com.z, mp.principal[0], mp.principal[1], mp.principal[2]);
    }
}
