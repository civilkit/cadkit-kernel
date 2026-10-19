//! `ck-native`: local kernel service. See docs/native-service.md.
use ck_native::{config_dir, load_or_create_token, serve, Config};
use std::net::TcpListener;
use std::path::PathBuf;

const USAGE: &str = "ck-native [--port N] [--config-dir DIR] [--allow-origin URL]... [--site-origin URL]
          [--max-body-mib N] [--max-reply-mib N] [--max-sessions N] [--idle-secs N]
          [--stack-mib N] [--quiet] [--print-token]
Binds 127.0.0.1 only. The bearer token lives in <config-dir>/token (mode 600).";

fn main() {
    let mut cfg = Config::default();
    let mut dir: Option<PathBuf> = None;
    let mut port: u16 = 7461;
    let mut print_token = false;
    let mut a = std::env::args().skip(1);
    let num = |a: &mut dyn Iterator<Item = String>, f: &str| -> u64 {
        a.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| die(&format!("{f} needs a number")))
    };
    while let Some(f) = a.next() {
        match f.as_str() {
            "--port" => port = num(&mut a, &f) as u16,
            "--config-dir" => dir = Some(a.next().unwrap_or_else(|| die("--config-dir needs a path")).into()),
            "--allow-origin" | "--site-origin" => cfg.origins.push(a.next().unwrap_or_else(|| die(&format!("{f} needs a URL")))),
            "--max-body-mib" => cfg.max_body = (num(&mut a, &f) as usize) << 20,
            "--max-reply-mib" => cfg.max_reply = (num(&mut a, &f) as usize) << 20,
            "--max-sessions" => cfg.max_sessions = num(&mut a, &f) as usize,
            "--idle-secs" => cfg.idle = std::time::Duration::from_secs(num(&mut a, &f)),
            "--stack-mib" => cfg.stack = (num(&mut a, &f) as usize) << 20,
            "--quiet" => cfg.quiet = true,
            "--print-token" => print_token = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                return;
            }
            _ => die(&format!("unknown flag {f}\n{USAGE}")),
        }
    }
    let dir = dir.unwrap_or_else(|| config_dir().unwrap_or_else(|e| die(&e)));
    let (token, path) = load_or_create_token(&dir).unwrap_or_else(|e| die(&e));
    if print_token {
        println!("{token}");
        return;
    }
    cfg.token = token;
    let l = TcpListener::bind(("127.0.0.1", port)).unwrap_or_else(|e| die(&format!("bind 127.0.0.1:{port}: {e}")));
    let port = l.local_addr().unwrap().port();
    eprintln!("ck-native {} listening on http://127.0.0.1:{port}", env!("CARGO_PKG_VERSION"));
    eprintln!("token file: {}", path.display());
    eprintln!("allowed origins: {}", cfg.all_origins(port).join(", "));
    serve(l, cfg);
}

fn die(m: &str) -> ! {
    eprintln!("ck-native: {m}");
    std::process::exit(2)
}
