//! `ck-native`: local kernel service. See docs/native-service.md.
use ck_native::{config_dir, list_clients, load_or_create_token, pair_code, revoke, serve, Config};
use std::net::TcpListener;
use std::path::PathBuf;
use std::time::Duration;

const USAGE: &str = "ck-native [--port N] [--config-dir DIR] [--allow-origin URL]... [--site-origin URL]
          [--max-body-mib N] [--max-reply-mib N] [--max-sessions N] [--idle-secs N]
          [--stack-mib N] [--quiet] [--print-token] [--pair]
ck-native clients              list paired clients
ck-native revoke <id>          revoke one paired client
Binds 127.0.0.1 only. The bearer token lives in <config-dir>/token (mode 600).
--pair prints a one-time code (valid 2 minutes, single use) for POST /pair.";

fn main() {
    let mut cfg = Config::default();
    let mut dir: Option<PathBuf> = None;
    let mut port: u16 = 7461;
    let mut print_token = false;
    let mut pair = false;
    let mut list = false;
    let mut revoke_id: Option<String> = None;
    let mut a = std::env::args().skip(1);
    let num = |a: &mut dyn Iterator<Item = String>, f: &str| -> u64 {
        a.next().and_then(|v| v.parse().ok()).unwrap_or_else(|| die(&format!("{f} needs a number")))
    };
    while let Some(f) = a.next() {
        match f.as_str() {
            "--port" => port = u16::try_from(num(&mut a, &f)).unwrap_or_else(|_| die("--port must be 0-65535")),
            "--config-dir" => dir = Some(a.next().unwrap_or_else(|| die("--config-dir needs a path")).into()),
            "--allow-origin" | "--site-origin" => cfg.origins.push(a.next().unwrap_or_else(|| die(&format!("{f} needs a URL")))),
            "--max-body-mib" => cfg.max_body = (num(&mut a, &f) as usize) << 20,
            "--max-reply-mib" => cfg.max_reply = (num(&mut a, &f) as usize) << 20,
            "--max-sessions" => cfg.max_sessions = num(&mut a, &f) as usize,
            "--idle-secs" => cfg.idle = std::time::Duration::from_secs(num(&mut a, &f)),
            "--stack-mib" => cfg.stack = (num(&mut a, &f) as usize) << 20,
            "--quiet" => cfg.quiet = true,
            "--print-token" => print_token = true,
            "--pair" => pair = true,
            "clients" => list = true,
            "revoke" => revoke_id = Some(a.next().unwrap_or_else(|| die("revoke needs a client id"))),
            "-h" | "--help" => {
                println!("{USAGE}");
                return;
            }
            _ => die(&format!("unknown flag {f}\n{USAGE}")),
        }
    }
    let dir = dir.unwrap_or_else(|| config_dir().unwrap_or_else(|e| die(&e)));
    // The three subcommands touch only the config dir; they do not need the shared token.
    if pair {
        match pair_code(&dir, Duration::from_secs(120)) {
            Ok(code) => {
                println!("pairing code: {code}");
                println!("valid for 2 minutes, single use");
                return;
            }
            Err(e) => die(&e),
        }
    }
    if list {
        let rs = list_clients(&dir);
        if rs.is_empty() {
            println!("no paired clients");
            return;
        }
        for r in rs {
            println!("{}  {}  {}  created={}", r.id, r.label, r.origin, r.created);
        }
        return;
    }
    if let Some(id) = revoke_id {
        match revoke(&dir, &id) {
            Ok(true) => {
                println!("revoked {id}");
                return;
            }
            Ok(false) => die(&format!("unknown client {id}")),
            Err(e) => die(&e),
        }
    }
    let (token, path) = load_or_create_token(&dir).unwrap_or_else(|e| die(&e));
    if print_token {
        println!("{token}");
        return;
    }
    cfg.token = token;
    cfg.dir = Some(dir);
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
