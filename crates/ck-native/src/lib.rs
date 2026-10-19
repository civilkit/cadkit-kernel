//! Local HTTP service around the kernel protocol (`ck_server::handle`).
//!
//! One worker thread per session, each owning its own kernel state (the kernel keeps its state in
//! thread-locals, exactly like one wasm instance per Worker). A small hand-written HTTP/1.1
//! layer over `std::net`; no async runtime. See docs/native-service.md.

use std::collections::HashMap;
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::mpsc::{channel, Sender};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::{Duration, Instant};

/// Version of the HTTP shape + message framing the app handshakes against.
pub const PROTOCOL_VERSION: u32 = 1;
const DEFAULT_ORIGIN: &str = "http://localhost:5173";
const MAX_CONNECTIONS: usize = 64;
const MAX_HEADER_BYTES: u64 = 32 * 1024;

#[derive(Clone)]
pub struct Config {
    pub token: String,
    /// Extra allowed Origins (besides http://localhost:5173).
    pub origins: Vec<String>,
    pub max_body: usize,
    pub max_reply: usize,
    pub max_sessions: usize,
    pub idle: Duration,
    pub stack: usize,
    pub quiet: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            token: String::new(),
            origins: vec![],
            max_body: 64 << 20,
            max_reply: 1 << 30,
            max_sessions: 8,
            idle: Duration::from_secs(600),
            stack: 256 << 20,
            quiet: false,
        }
    }
}

impl Config {
    pub fn all_origins(&self, _port: u16) -> Vec<String> {
        let mut v = vec![DEFAULT_ORIGIN.to_string()];
        for o in &self.origins {
            let o = o.trim_end_matches('/').to_string();
            if !v.contains(&o) {
                v.push(o);
            }
        }
        v
    }
}

// ---------------------------------------------------------------- token / config dir

pub fn config_dir() -> Result<PathBuf, String> {
    if let Ok(d) = std::env::var("CK_NATIVE_CONFIG_DIR") {
        return Ok(d.into());
    }
    if let Ok(d) = std::env::var("XDG_CONFIG_HOME") {
        return Ok(Path::new(&d).join("ck-native"));
    }
    let h = std::env::var("HOME").map_err(|_| "no HOME; pass --config-dir".to_string())?;
    Ok(Path::new(&h).join(".config").join("ck-native"))
}

fn random_hex(bytes: usize) -> Result<String, String> {
    let mut b = vec![0u8; bytes];
    std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut b))
        .map_err(|e| format!("random: {e}"))?;
    Ok(b.iter().map(|x| format!("{x:02x}")).collect())
}

/// Read `<dir>/token`, creating it (mode 600) with a random 256-bit token on first start.
pub fn load_or_create_token(dir: &Path) -> Result<(String, PathBuf), String> {
    let path = dir.join("token");
    if let Ok(t) = std::fs::read_to_string(&path) {
        let t = t.trim().to_string();
        if t.len() >= 32 {
            return Ok((t, path));
        }
    }
    std::fs::create_dir_all(dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    let t = random_hex(32)?;
    let mut o = std::fs::OpenOptions::new();
    o.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        o.mode(0o600);
    }
    let mut f = o.open(&path).map_err(|e| format!("write {}: {e}", path.display()))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = f.set_permissions(std::fs::Permissions::from_mode(0o600));
    }
    f.write_all(t.as_bytes()).map_err(|e| format!("write {}: {e}", path.display()))?;
    Ok((t, path))
}

fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    let mut d = (a.len() ^ b.len()) as u8;
    for i in 0..a.len().max(b.len()) {
        d |= a.get(i).copied().unwrap_or(0) ^ b.get(i).copied().unwrap_or(0);
    }
    d == 0
}

// ---------------------------------------------------------------- native clock

fn start() -> &'static Instant {
    static S: OnceLock<Instant> = OnceLock::new();
    S.get_or_init(Instant::now)
}
fn native_now() -> f64 {
    start().elapsed().as_secs_f64() * 1000.0
}
fn native_unix_ms() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64() * 1000.0)
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------- sessions

struct Job {
    body: Vec<u8>,
    reply: Sender<Result<Vec<u8>, String>>,
}

struct Session {
    tx: Mutex<Sender<Job>>,
    last: Mutex<Instant>,
    busy: AtomicUsize,
}

struct Shared {
    cfg: Config,
    port: u16,
    sessions: Mutex<HashMap<String, Arc<Session>>>,
    conns: AtomicUsize,
}

fn spawn_worker(stack: usize) -> Result<Sender<Job>, String> {
    let (tx, rx) = channel::<Job>();
    std::thread::Builder::new()
        .name("ck-session".into())
        .stack_size(stack)
        .spawn(move || {
            // Per-thread kernel setup: the clock, the budget clocks and the hooks are thread-locals.
            ck_server::clock::set_now(native_now);
            ck_server::clock::set_unix_ms(native_unix_ms);
            ck_wasm::engine::install();
            while let Ok(job) = rx.recv() {
                let r = catch_unwind(AssertUnwindSafe(|| ck_server::handle(&job.body)));
                match r {
                    Ok(v) => {
                        let _ = job.reply.send(Ok(v));
                    }
                    Err(_) => {
                        // Kernel state may be inconsistent after a panic: end the session.
                        let _ = job.reply.send(Err("kernel panic".into()));
                        return;
                    }
                }
            }
        })
        .map_err(|e| format!("spawn: {e}"))?;
    Ok(tx)
}

// ---------------------------------------------------------------- HTTP

struct Req {
    method: String,
    path: String,
    headers: Vec<(String, String)>,
}

impl Req {
    fn h(&self, k: &str) -> Option<&str> {
        self.headers.iter().find(|(n, _)| n == k).map(|(_, v)| v.as_str())
    }
}

enum ReadErr {
    Closed,
    Bad(&'static str),
}

fn read_req(r: &mut BufReader<TcpStream>) -> Result<Req, ReadErr> {
    let mut budget = MAX_HEADER_BYTES;
    let mut line = String::new();
    let mut next = |r: &mut BufReader<TcpStream>, line: &mut String, budget: &mut u64| -> Result<usize, ReadErr> {
        line.clear();
        let mut t = r.by_ref().take(*budget);
        let n = t.read_line(line).map_err(|_| ReadErr::Closed)?;
        *budget -= n as u64;
        if n > 0 && !line.ends_with('\n') {
            return Err(ReadErr::Bad("header too large"));
        }
        Ok(n)
    };
    // tolerate a stray blank line before the request line
    loop {
        if next(r, &mut line, &mut budget)? == 0 {
            return Err(ReadErr::Closed);
        }
        if !line.trim().is_empty() {
            break;
        }
    }
    let mut it = line.trim_end().split(' ');
    let (m, p, v) = (it.next().unwrap_or(""), it.next().unwrap_or(""), it.next().unwrap_or(""));
    if m.is_empty() || p.is_empty() || !v.starts_with("HTTP/1.") || it.next().is_some() {
        return Err(ReadErr::Bad("bad request line"));
    }
    let mut req = Req { method: m.to_string(), path: p.to_string(), headers: vec![] };
    loop {
        if next(r, &mut line, &mut budget)? == 0 {
            return Err(ReadErr::Closed);
        }
        let l = line.trim_end_matches(['\r', '\n']);
        if l.is_empty() {
            return Ok(req);
        }
        let (k, v) = l.split_once(':').ok_or(ReadErr::Bad("bad header"))?;
        req.headers.push((k.trim().to_ascii_lowercase(), v.trim().to_string()));
        if req.headers.len() > 100 {
            return Err(ReadErr::Bad("too many headers"));
        }
    }
}

struct Resp {
    status: u16,
    body: Vec<u8>,
    ctype: &'static str,
    cors: Option<String>,
    preflight: bool,
    close: bool,
}

fn reason(s: u16) -> &'static str {
    match s {
        200 => "OK",
        204 => "No Content",
        400 => "Bad Request",
        401 => "Unauthorized",
        403 => "Forbidden",
        404 => "Not Found",
        405 => "Method Not Allowed",
        410 => "Gone",
        411 => "Length Required",
        413 => "Payload Too Large",
        429 => "Too Many Requests",
        431 => "Request Header Fields Too Large",
        500 => "Internal Server Error",
        501 => "Not Implemented",
        _ => "Error",
    }
}

fn err(status: u16, msg: &str) -> Resp {
    let b = format!("{{\"error\":\"{}\"}}", msg.replace('"', "'"));
    Resp { status, body: b.into_bytes(), ctype: "application/json", cors: None, preflight: false, close: false }
}

fn write_resp(w: &mut TcpStream, r: &Resp, keep: bool) -> std::io::Result<()> {
    let mut h = format!(
        "HTTP/1.1 {} {}\r\nContent-Length: {}\r\nConnection: {}\r\nCache-Control: no-store\r\nX-Content-Type-Options: nosniff\r\n",
        r.status,
        reason(r.status),
        r.body.len(),
        if keep { "keep-alive" } else { "close" }
    );
    if r.status != 204 {
        h += &format!("Content-Type: {}\r\n", r.ctype);
    }
    if let Some(o) = &r.cors {
        h += &format!("Access-Control-Allow-Origin: {o}\r\nVary: Origin\r\n");
        if r.preflight {
            h += "Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\nAccess-Control-Allow-Headers: authorization, content-type\r\nAccess-Control-Allow-Private-Network: true\r\nAccess-Control-Max-Age: 600\r\n";
        }
    }
    h += "\r\n";
    w.write_all(h.as_bytes())?;
    w.write_all(&r.body)?;
    w.flush()
}

fn host_ok(host: Option<&str>, port: u16) -> bool {
    match host {
        Some(h) => {
            let h = h.to_ascii_lowercase();
            h == format!("127.0.0.1:{port}") || h == format!("localhost:{port}")
        }
        None => false,
    }
}

fn handle_conn(sh: Arc<Shared>, stream: TcpStream) {
    let _ = stream.set_read_timeout(Some(Duration::from_secs(300)));
    let _ = stream.set_nodelay(true);
    let mut w = match stream.try_clone() {
        Ok(w) => w,
        Err(_) => return,
    };
    let mut r = BufReader::new(stream);
    loop {
        let req = match read_req(&mut r) {
            Ok(q) => q,
            Err(ReadErr::Closed) => return,
            Err(ReadErr::Bad(m)) => {
                let mut e = err(if m.contains("large") { 431 } else { 400 }, m);
                e.close = true;
                let _ = write_resp(&mut w, &e, false);
                return;
            }
        };
        let t0 = Instant::now();
        let mut resp = route(&sh, &req, &mut r);
        let keep = !resp.close && req.h("connection").map(|c| !c.eq_ignore_ascii_case("close")).unwrap_or(true);
        if !sh.cfg.quiet {
            // method, route shape, status, size, time: never the body or the token
            let p = req.path.split('?').next().unwrap_or("");
            eprintln!("{} {} -> {} ({} B, {:.0} ms)", req.method, p, resp.status, resp.body.len(), t0.elapsed().as_secs_f64() * 1000.0);
        }
        if resp.body.len() > sh.cfg.max_reply {
            resp = err(413, "reply exceeds the reply size limit");
        }
        if write_resp(&mut w, &resp, keep).is_err() || !keep {
            return;
        }
    }
}

fn route(sh: &Arc<Shared>, req: &Req, r: &mut BufReader<TcpStream>) -> Resp {
    let cfg = &sh.cfg;
    let deny = |s, m| {
        let mut e = err(s, m);
        e.close = true;
        e
    };
    if !host_ok(req.h("host"), sh.port) {
        return deny(403, "bad Host");
    }
    let origin = req.h("origin");
    let allowed = cfg.all_origins(sh.port);
    let origin_ok = match origin {
        None => true,
        Some(o) => allowed.iter().any(|a| a.eq_ignore_ascii_case(o)),
    };
    if !origin_ok {
        return deny(403, "Origin not allowed");
    }
    let cors = origin.map(|o| o.to_string());
    if req.method == "OPTIONS" {
        // Preflight: only for allowed origins, no token (browsers never send one here).
        if origin.is_none() {
            return deny(403, "preflight needs an Origin");
        }
        return Resp { status: 204, body: vec![], ctype: "", cors, preflight: true, close: false };
    }
    let tok = req.h("authorization").and_then(|a| a.strip_prefix("Bearer ")).unwrap_or("");
    if cfg.token.is_empty() || !ct_eq(tok.trim().as_bytes(), cfg.token.as_bytes()) {
        let mut e = deny(401, "missing or wrong token");
        e.cors = cors;
        return e;
    }
    let mut resp = dispatch(sh, req, r);
    resp.cors = cors;
    resp
}

fn read_body(sh: &Shared, req: &Req, r: &mut BufReader<TcpStream>) -> Result<Vec<u8>, Resp> {
    if req.h("transfer-encoding").is_some() {
        let mut e = err(501, "chunked request bodies are not supported");
        e.close = true;
        return Err(e);
    }
    let n: usize = match req.h("content-length").map(|v| v.parse::<usize>()) {
        Some(Ok(n)) => n,
        Some(Err(_)) => {
            let mut e = err(400, "bad Content-Length");
            e.close = true;
            return Err(e);
        }
        None => {
            let mut e = err(411, "Content-Length required");
            e.close = true;
            return Err(e);
        }
    };
    if n > sh.cfg.max_body {
        let mut e = err(413, "request body exceeds the size limit");
        e.close = true;
        return Err(e);
    }
    let mut b = vec![0u8; n];
    if r.read_exact(&mut b).is_err() {
        let mut e = err(400, "short body");
        e.close = true;
        return Err(e);
    }
    Ok(b)
}

fn json(status: u16, s: String) -> Resp {
    Resp { status, body: s.into_bytes(), ctype: "application/json", cors: None, preflight: false, close: false }
}

fn dispatch(sh: &Arc<Shared>, req: &Req, r: &mut BufReader<TcpStream>) -> Resp {
    let path = req.path.split('?').next().unwrap_or("");
    let parts: Vec<&str> = path.trim_matches('/').split('/').collect();
    match (req.method.as_str(), parts.as_slice()) {
        ("GET", ["version"]) => {
            let cores = std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1);
            json(
                200,
                format!(
                    "{{\"kernel\":\"{}\",\"protocol\":{},\"git\":\"{}\",\"build\":\"{}\",\"cores\":{},\"sessions\":{},\"maxSessions\":{}}}",
                    env!("CARGO_PKG_VERSION"),
                    PROTOCOL_VERSION,
                    env!("CK_NATIVE_GIT"),
                    ck_wasm::COMMIT,
                    cores,
                    sh.sessions.lock().unwrap().len(),
                    sh.cfg.max_sessions
                ),
            )
        }
        ("POST", ["session"]) => {
            // drain any (ignored) body so keep-alive stays in sync
            if let Err(e) = read_body(sh, req, r) {
                return e;
            }
            let mut map = sh.sessions.lock().unwrap();
            if map.len() >= sh.cfg.max_sessions {
                return err(429, "maximum number of sessions reached");
            }
            let id = match random_hex(8) {
                Ok(i) => i,
                Err(e) => return err(500, &e),
            };
            let tx = match spawn_worker(sh.cfg.stack) {
                Ok(t) => t,
                Err(e) => return err(500, &e),
            };
            map.insert(id.clone(), Arc::new(Session { tx: Mutex::new(tx), last: Mutex::new(Instant::now()), busy: AtomicUsize::new(0) }));
            json(200, format!("{{\"session\":\"{id}\"}}"))
        }
        ("POST", ["session", id, "rpc"]) => {
            let body = match read_body(sh, req, r) {
                Ok(b) => b,
                Err(e) => return e,
            };
            let s = {
                let map = sh.sessions.lock().unwrap();
                match map.get(*id) {
                    Some(s) => {
                        s.busy.fetch_add(1, Ordering::SeqCst);
                        s.clone()
                    }
                    None => return err(404, "unknown session"),
                }
            };
            let (rtx, rrx) = channel();
            let sent = s.tx.lock().unwrap().send(Job { body, reply: rtx }).is_ok();
            let out = if sent { rrx.recv().map_err(|_| "worker gone".to_string()).and_then(|x| x) } else { Err("worker gone".into()) };
            *s.last.lock().unwrap() = Instant::now();
            s.busy.fetch_sub(1, Ordering::SeqCst);
            match out {
                Ok(bytes) => Resp { status: 200, body: bytes, ctype: "application/octet-stream", cors: None, preflight: false, close: false },
                Err(m) => {
                    sh.sessions.lock().unwrap().remove(*id);
                    err(500, &format!("{m}; the session was closed"))
                }
            }
        }
        ("DELETE", ["session", id]) => {
            if sh.sessions.lock().unwrap().remove(*id).is_some() {
                json(200, "{\"closed\":true}".into())
            } else {
                err(404, "unknown session")
            }
        }
        (_, ["version"]) | (_, ["session"]) | (_, ["session", _]) | (_, ["session", _, "rpc"]) => err(405, "method not allowed"),
        _ => err(404, "not found"),
    }
}

/// Serve on `listener` until the process ends. Spawns the idle reaper and one thread per connection.
pub fn serve(listener: TcpListener, cfg: Config) {
    let _ = start();
    // A kernel panic must not dump document text: log the location only.
    std::panic::set_hook(Box::new(|info| {
        let loc = info.location().map(|l| format!("{}:{}", l.file(), l.line())).unwrap_or_default();
        eprintln!("kernel panic at {loc}");
    }));
    let port = listener.local_addr().map(|a| a.port()).unwrap_or(0);
    let sh = Arc::new(Shared { cfg, port, sessions: Mutex::new(HashMap::new()), conns: AtomicUsize::new(0) });
    {
        let sh = sh.clone();
        std::thread::spawn(move || loop {
            std::thread::sleep((sh.cfg.idle / 4).clamp(Duration::from_millis(50), Duration::from_secs(5)));
            let mut map = sh.sessions.lock().unwrap();
            map.retain(|_, s| s.busy.load(Ordering::SeqCst) > 0 || s.last.lock().unwrap().elapsed() < sh.cfg.idle);
        });
    }
    for c in listener.incoming() {
        let Ok(c) = c else { continue };
        if sh.conns.fetch_add(1, Ordering::SeqCst) >= MAX_CONNECTIONS {
            sh.conns.fetch_sub(1, Ordering::SeqCst);
            continue; // dropped
        }
        let sh2 = sh.clone();
        let spawned = std::thread::Builder::new().stack_size(1 << 20).spawn(move || {
            handle_conn(sh2.clone(), c);
            sh2.conns.fetch_sub(1, Ordering::SeqCst);
        });
        if spawned.is_err() {
            sh.conns.fetch_sub(1, Ordering::SeqCst);
        }
    }
}
