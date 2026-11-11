//! Protocol tests over a real loopback socket.
use ck_native::{serve, Config};
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::Duration;

const TOKEN: &str = "0123456789abcdef0123456789abcdef0123456789abcdef";
const PART: &str = include_str!("../../../tools/cases/bracket.ckpart");

fn start(f: impl FnOnce(&mut Config)) -> u16 {
    let mut cfg = Config { token: TOKEN.into(), quiet: true, stack: 64 << 20, ..Config::default() };
    f(&mut cfg);
    let l = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = l.local_addr().unwrap().port();
    std::thread::spawn(move || serve(l, cfg));
    port
}

struct R {
    status: u16,
    headers: Vec<(String, String)>,
    body: Vec<u8>,
}
impl R {
    fn h(&self, k: &str) -> Option<&str> {
        self.headers.iter().find(|(n, _)| n.eq_ignore_ascii_case(k)).map(|(_, v)| v.as_str())
    }
}

fn read_resp(s: &mut TcpStream) -> Option<R> {
    let mut buf = Vec::new();
    let mut b = [0u8; 65536];
    let end = loop {
        if let Some(i) = buf.windows(4).position(|w| w == b"\r\n\r\n") {
            break i;
        }
        let n = s.read(&mut b).ok()?;
        if n == 0 {
            return None;
        }
        buf.extend_from_slice(&b[..n]);
    };
    let head = String::from_utf8_lossy(&buf[..end]).to_string();
    let mut lines = head.split("\r\n");
    let status: u16 = lines.next()?.split(' ').nth(1)?.parse().ok()?;
    let headers: Vec<(String, String)> =
        lines.filter_map(|l| l.split_once(':')).map(|(k, v)| (k.trim().to_string(), v.trim().to_string())).collect();
    let len: usize = headers.iter().find(|(k, _)| k.eq_ignore_ascii_case("content-length")).and_then(|(_, v)| v.parse().ok()).unwrap_or(0);
    let mut body = buf[end + 4..].to_vec();
    while body.len() < len {
        let n = s.read(&mut b).ok()?;
        if n == 0 {
            return None;
        }
        body.extend_from_slice(&b[..n]);
    }
    Some(R { status, headers, body })
}

fn send(s: &mut TcpStream, port: u16, method: &str, path: &str, extra: &[(&str, &str)], body: &[u8]) -> Option<R> {
    let mut h = format!("{method} {path} HTTP/1.1\r\n");
    if !extra.iter().any(|(k, _)| *k == "Host") {
        h += &format!("Host: 127.0.0.1:{port}\r\n");
    }
    for (k, v) in extra {
        h += &format!("{k}: {v}\r\n");
    }
    if method == "POST" && !extra.iter().any(|(k, _)| *k == "Content-Length") {
        h += &format!("Content-Length: {}\r\n", body.len());
    }
    h += "\r\n";
    s.write_all(h.as_bytes()).ok()?;
    s.write_all(body).ok()?;
    read_resp(s)
}

fn req(port: u16, method: &str, path: &str, extra: &[(&str, &str)], body: &[u8]) -> R {
    let mut s = TcpStream::connect(("127.0.0.1", port)).unwrap();
    s.set_read_timeout(Some(Duration::from_secs(120))).unwrap();
    send(&mut s, port, method, path, extra, body).expect("response")
}

fn auth() -> (&'static str, String) {
    ("Authorization", format!("Bearer {TOKEN}"))
}

fn open(port: u16) -> String {
    let a = auth();
    let r = req(port, "POST", "/session", &[(a.0, &a.1)], b"");
    assert_eq!(r.status, 200, "{}", String::from_utf8_lossy(&r.body));
    let t = String::from_utf8(r.body).unwrap();
    t.split('"').nth(3).unwrap().to_string()
}

fn rpc(port: u16, id: &str, body: &[u8]) -> R {
    let a = auth();
    req(port, "POST", &format!("/session/{id}/rpc"), &[(a.0, &a.1)], body)
}

/// Frame = u32 json length, json, padding to 8, binary. Normalise `ms` / `at` in the json.
fn norm(f: &[u8]) -> (String, Vec<u8>) {
    let jl = u32::from_le_bytes(f[..4].try_into().unwrap()) as usize;
    let j = String::from_utf8_lossy(&f[4..4 + jl]).to_string();
    let mut out = String::new();
    let mut rest = j.as_str();
    while let Some(i) = rest.find("\"ms\":") {
        out += &rest[..i + 5];
        let t = &rest[i + 5..];
        let e = t.find(|c: char| !(c.is_ascii_digit() || ".e+-".contains(c))).unwrap_or(t.len());
        out += "0";
        rest = &t[e..];
    }
    out += rest;
    (out, f[((4 + jl + 7) & !7).min(f.len())..].to_vec())
}

fn regen_msg() -> Vec<u8> {
    format!("{{\"id\":1,\"msg\":{{\"type\":\"regen\",\"tag\":\"main\",\"part\":{PART}}}}}").into_bytes()
}

fn direct(body: &[u8]) -> Vec<u8> {
    // fresh thread = fresh kernel state, set up the way a session worker does
    let b = body.to_vec();
    std::thread::Builder::new()
        .stack_size(64 << 20)
        .spawn(move || {
            ck_server::clock::set_now(|| 0.0);
            ck_wasm::engine::install();
            ck_server::handle(&b)
        })
        .unwrap()
        .join()
        .unwrap()
}

#[test]
fn regen_matches_direct_handle() {
    let port = start(|_| {});
    let id = open(port);
    let msg = regen_msg();
    let r = rpc(port, &id, &msg);
    assert_eq!(r.status, 200);
    assert_eq!(norm(&r.body), norm(&direct(&msg)));
    assert!(norm(&r.body).0.contains("\"ok\":true"));
    // a follow-up query sees the regen'd state of the session
    let q = rpc(port, &id, br#"{"id":2,"msg":{"type":"massProps","tag":"main"}}"#);
    assert!(norm(&q.body).0.contains("volume"), "{}", norm(&q.body).0);
}

#[test]
fn sessions_do_not_share_state() {
    let port = start(|_| {});
    let (a, b) = (open(port), open(port));
    let ts: Vec<_> = [a.clone(), b.clone()]
        .into_iter()
        .map(|id| std::thread::spawn(move || {
            assert_eq!(rpc(port, &id, &regen_msg()).status, 200);
        }))
        .collect();
    for t in ts { t.join().unwrap(); }
    // both regen'd in parallel; now a third, untouched session has no body
    let c = open(port);
    let q = br#"{"id":2,"msg":{"type":"massProps","tag":"main"}}"#;
    let (ra, rc) = (rpc(port, &a, q), rpc(port, &c, q));
    assert!(norm(&ra.body).0.contains("volume"));
    assert_ne!(norm(&ra.body), norm(&rc.body), "fresh session must not see the other's body");
    // deleting one leaves the other alive
    let au = auth();
    assert_eq!(req(port, "DELETE", &format!("/session/{a}"), &[(au.0, &au.1)], b"").status, 200);
    assert_eq!(rpc(port, &a, q).status, 404);
    assert!(norm(&rpc(port, &b, q).body).0.contains("volume"));
}

#[test]
fn auth_origin_host_matrix() {
    let port = start(|c| c.origins.push("https://site.example.com".into()));
    let a = auth();
    let good = [(a.0, a.1.as_str())];
    // missing / wrong token
    assert_eq!(req(port, "GET", "/version", &[], b"").status, 401);
    assert_eq!(req(port, "GET", "/version", &[("Authorization", "Bearer nope")], b"").status, 401);
    assert_eq!(req(port, "GET", "/version", &[("Authorization", TOKEN)], b"").status, 401);
    // ok without Origin (non-browser client), and with allowed origins
    let v = req(port, "GET", "/version", &good, b"");
    assert_eq!(v.status, 200);
    let vs = String::from_utf8(v.body).unwrap();
    for k in ["kernel", "protocol", "git", "build", "cores"] {
        assert!(vs.contains(&format!("\"{k}\"")), "{vs}");
    }
    assert!(!vs.contains(TOKEN));
    for o in ["http://localhost:5173", "https://site.example.com"] {
        let r = req(port, "GET", "/version", &[good[0], ("Origin", o)], b"");
        assert_eq!(r.status, 200);
        assert_eq!(r.h("Access-Control-Allow-Origin"), Some(o));
    }
    // bad Origin even with a valid token
    for o in ["http://evil.example.com", "null", "http://localhost:5174", "https://site.example.com.evil.com"] {
        assert_eq!(req(port, "GET", "/version", &[good[0], ("Origin", o)], b"").status, 403, "{o}");
    }
    // bad / missing Host
    for h in ["evil.example.com", "127.0.0.1", "127.0.0.1:1", "localhost", "0.0.0.0:80"] {
        assert_eq!(req(port, "GET", "/version", &[good[0], ("Host", h)], b"").status, 403, "{h}");
    }
    assert_eq!(req(port, "GET", "/version", &[good[0], ("Host", &format!("localhost:{port}"))], b"").status, 200);
    // preflight: allowed (no token), denied origin, bad host, no origin
    let pf = |o: &str, host: &str| {
        req(port, "OPTIONS", "/session/x/rpc", &[("Origin", o), ("Host", host), ("Access-Control-Request-Method", "POST"), ("Access-Control-Request-Private-Network", "true")], b"")
    };
    let h = format!("127.0.0.1:{port}");
    let r = pf("https://site.example.com", &h);
    assert_eq!(r.status, 204);
    assert_eq!(r.h("Access-Control-Allow-Private-Network"), Some("true"));
    assert_eq!(r.h("Access-Control-Allow-Origin"), Some("https://site.example.com"));
    assert!(r.h("Access-Control-Allow-Headers").unwrap().contains("authorization"));
    assert_eq!(pf("http://evil.example.com", &h).status, 403);
    assert!(pf("http://evil.example.com", &h).h("Access-Control-Allow-Origin").is_none());
    assert_eq!(pf("https://site.example.com", "evil.example.com").status, 403);
    assert_eq!(req(port, "OPTIONS", "/version", &[], b"").status, 403);
}

#[test]
fn limits_and_unknown_session() {
    let port = start(|c| {
        c.max_body = 1024;
        c.max_sessions = 2;
    });
    let a = auth();
    // oversized body: rejected from the header, before the body is read
    let mut s = TcpStream::connect(("127.0.0.1", port)).unwrap();
    s.set_read_timeout(Some(Duration::from_secs(20))).unwrap();
    let r = send(&mut s, port, "POST", "/session/abc/rpc", &[(a.0, &a.1), ("Content-Length", "2000")], b"").unwrap();
    assert_eq!(r.status, 413);
    let big = vec![b' '; 2000];
    let id = open(port);
    assert_eq!(rpc(port, &id, &big).status, 413);
    assert_eq!(rpc(port, &id, br#"{"id":1,"msg":{"type":"ping"}}"#).status, 200);
    // unknown session
    assert_eq!(rpc(port, "deadbeef", b"{}").status, 404);
    // no body-less POST without Content-Length, chunked refused
    assert_eq!(req(port, "POST", &format!("/session/{id}/rpc"), &[(a.0, &a.1), ("Transfer-Encoding", "chunked")], b"").status, 501);
    // max sessions
    let _second = open(port);
    let r = req(port, "POST", "/session", &[(a.0, &a.1)], b"");
    assert_eq!(r.status, 429);
    // deleting frees a slot
    assert_eq!(req(port, "DELETE", &format!("/session/{id}"), &[(a.0, &a.1)], b"").status, 200);
    open(port);
    // unknown routes
    assert_eq!(req(port, "GET", "/nope", &[(a.0, &a.1)], b"").status, 404);
}

#[test]
fn keep_alive_serves_many_requests_on_one_connection() {
    let port = start(|_| {});
    let a = auth();
    let mut s = TcpStream::connect(("127.0.0.1", port)).unwrap();
    s.set_read_timeout(Some(Duration::from_secs(20))).unwrap();
    let r = send(&mut s, port, "POST", "/session", &[(a.0, &a.1)], b"").unwrap();
    let id = String::from_utf8(r.body).unwrap().split('"').nth(3).unwrap().to_string();
    for _ in 0..3 {
        let r = send(&mut s, port, "POST", &format!("/session/{id}/rpc"), &[(a.0, &a.1)], br#"{"id":1,"msg":{"type":"ping"}}"#).unwrap();
        assert_eq!(r.status, 200);
        assert!(norm(&r.body).0.contains("pong"));
    }
}

#[test]
fn idle_sessions_expire() {
    let port = start(|c| c.idle = Duration::from_millis(400));
    let id = open(port);
    assert_eq!(rpc(port, &id, br#"{"id":1,"msg":{"type":"ping"}}"#).status, 200);
    std::thread::sleep(Duration::from_millis(1500));
    assert_eq!(rpc(port, &id, br#"{"id":1,"msg":{"type":"ping"}}"#).status, 404);
}

// ---------------------------------------------------------------- v1.1: ping and pairing

fn tmpdir() -> std::path::PathBuf {
    use std::sync::atomic::{AtomicUsize, Ordering};
    static N: AtomicUsize = AtomicUsize::new(0);
    let d = std::env::temp_dir().join(format!("ck-native-test-{}-{}", std::process::id(), N.fetch_add(1, Ordering::SeqCst)));
    let _ = std::fs::remove_dir_all(&d);
    d
}

fn json_field(reply: &str, key: &str) -> String {
    let pat = format!("\"{key}\":\"");
    let i = reply.find(&pat).unwrap_or_else(|| panic!("no {key} in {reply}")) + pat.len();
    let j = reply[i..].find('"').unwrap_or_else(|| panic!("unterminated {key}")) + i;
    reply[i..j].to_string()
}

fn pair_req(port: u16, code: &str, origin: &str, label: &str) -> R {
    let body = format!("{{\"code\":\"{code}\",\"origin\":\"{origin}\",\"label\":\"{label}\"}}");
    req(port, "POST", "/pair", &[("Origin", origin)], body.as_bytes())
}

fn paired(dir: &std::path::Path, port: u16, origin: &str, label: &str) -> (String, String) {
    let code = ck_native::pair_code(dir, Duration::from_secs(120)).expect("pair code");
    let r = pair_req(port, &code, origin, label);
    assert_eq!(r.status, 200, "{}", String::from_utf8_lossy(&r.body));
    let reply = String::from_utf8(r.body).unwrap();
    (json_field(&reply, "token"), json_field(&reply, "id"))
}

/// GET /ping: no token, the two advertised fields only, CORS for allowed origins only.
#[test]
fn ping_needs_no_token_and_leaks_nothing() {
    let port = start(|c| c.origins.push("https://site.example.com".into()));
    let want = format!("{{\"service\":\"ck-native\",\"protocol\":{}}}", ck_native::PROTOCOL_VERSION);
    // no Origin (non-browser discovery)
    let r = req(port, "GET", "/ping", &[], b"");
    assert_eq!(r.status, 200);
    assert_eq!(r.body, want.as_bytes(), "ping must carry only service and protocol");
    assert!(r.h("Access-Control-Allow-Origin").is_none());
    // allowed origins get CORS, same body
    for o in ["http://localhost:5173", "https://site.example.com"] {
        let r = req(port, "GET", "/ping", &[("Origin", o)], b"");
        assert_eq!(r.status, 200, "{o}");
        assert_eq!(r.body, want.as_bytes(), "{o}");
        assert_eq!(r.h("Access-Control-Allow-Origin"), Some(o), "{o}");
    }
    // Origin: null and unlisted origins: 403, no CORS headers, no body leak
    for o in ["null", "http://evil.example.com", "http://localhost:5174"] {
        let r = req(port, "GET", "/ping", &[("Origin", o)], b"");
        assert_eq!(r.status, 403, "{o}");
        assert!(r.h("Access-Control-Allow-Origin").is_none(), "{o}");
        assert!(!String::from_utf8_lossy(&r.body).contains("protocol"), "{o}");
    }
    // /ping opened nothing else: /version still needs the token
    assert_eq!(req(port, "GET", "/version", &[], b"").status, 401);
    // a preflight for /ping needs no token either
    let r = req(port, "OPTIONS", "/ping", &[("Origin", "http://localhost:5173"), ("Access-Control-Request-Method", "GET")], b"");
    assert_eq!(r.status, 204);
    assert_eq!(r.h("Access-Control-Allow-Origin"), Some("http://localhost:5173"));
    // POST /ping is a method error for a token holder, not a new route
    let a = auth();
    assert_eq!(req(port, "POST", "/ping", &[(a.0, a.1.as_str())], b"").status, 405);
}

/// Wrong, expired and single-use (reused) pairing codes.
#[test]
fn pairing_codes_wrong_expired_and_reused() {
    let dir = tmpdir();
    let port = start(|c| {
        c.dir = Some(dir.clone());
        c.origins.push("https://site.example.com".into());
    });
    // no code written yet
    let r = pair_req(port, "01234567", "http://localhost:5173", "kit");
    assert_eq!(r.status, 401, "{}", String::from_utf8_lossy(&r.body));
    // wrong code: refused, the real code survives
    let code = ck_native::pair_code(&dir, Duration::from_secs(120)).unwrap();
    let r = pair_req(port, "00000000", "http://localhost:5173", "kit");
    assert_eq!(r.status, 401);
    assert!(String::from_utf8_lossy(&r.body).contains("wrong"), "{}", String::from_utf8_lossy(&r.body));
    // right code: a token bound to origin and label
    let r = pair_req(port, &code, "http://localhost:5173", "kit dev");
    assert_eq!(r.status, 200, "{}", String::from_utf8_lossy(&r.body));
    let reply = String::from_utf8(r.body.clone()).unwrap();
    assert_eq!(json_field(&reply, "origin"), "http://localhost:5173");
    assert_eq!(json_field(&reply, "label"), "kit dev");
    assert_eq!(json_field(&reply, "token").len(), 64);
    assert_eq!(json_field(&reply, "id").len(), 8);
    let token = json_field(&reply, "token");
    // reused: the code is consumed
    let r = pair_req(port, &code, "http://localhost:5173", "kit");
    assert_eq!(r.status, 401, "{}", String::from_utf8_lossy(&r.body));
    assert!(String::from_utf8_lossy(&r.body).contains("no pairing code"));
    // expired: ttl already elapsed when the service reads it
    let code2 = ck_native::pair_code(&dir, Duration::ZERO).unwrap();
    let r = pair_req(port, &code2, "http://localhost:5173", "kit");
    assert_eq!(r.status, 401);
    assert!(String::from_utf8_lossy(&r.body).contains("expired"), "{}", String::from_utf8_lossy(&r.body));
    // a page must claim its own Origin: header and body disagree
    let code3 = ck_native::pair_code(&dir, Duration::from_secs(120)).unwrap();
    let body = format!("{{\"code\":\"{code3}\",\"origin\":\"https://site.example.com\",\"label\":\"kit\"}}");
    let r = req(port, "POST", "/pair", &[("Origin", "http://localhost:5173")], body.as_bytes());
    assert_eq!(r.status, 403, "{}", String::from_utf8_lossy(&r.body));
    // a non-browser client still cannot pair an origin that is not on the allow-list
    let body = format!("{{\"code\":\"{code3}\",\"origin\":\"http://evil.example.com\",\"label\":\"kit\"}}");
    let r = req(port, "POST", "/pair", &[], body.as_bytes());
    assert_eq!(r.status, 403);
    let _ = std::fs::remove_dir_all(&dir);
    let _ = token;
}

/// A revoked client's token stops working on the running service (file reload), the id is gone.
#[test]
fn revoked_client_token_is_refused() {
    let dir = tmpdir();
    let port = start(|c| c.dir = Some(dir.clone()));
    let (token, id) = paired(&dir, port, "http://localhost:5173", "kit");
    let a = ("Authorization", format!("Bearer {token}"));
    assert_eq!(req(port, "GET", "/version", &[(a.0, a.1.as_str())], b"").status, 200);
    assert!(ck_native::revoke(&dir, &id).unwrap(), "the client existed");
    assert!(!ck_native::revoke(&dir, &id).unwrap(), "revoked twice is unknown");
    assert_eq!(req(port, "GET", "/version", &[(a.0, a.1.as_str())], b"").status, 401, "revoked token must die live");
    assert!(ck_native::list_clients(&dir).is_empty());
    // the shared token keeps working
    let s = auth();
    assert_eq!(req(port, "GET", "/version", &[(s.0, s.1.as_str())], b"").status, 200);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Past the connection limit the service answers 503 instead of dropping the connection.
#[test]
fn connections_over_the_limit_get_a_503() {
    let port = start(|_| {});
    let idle: Vec<TcpStream> = (0..64).map(|_| TcpStream::connect(("127.0.0.1", port)).unwrap()).collect();
    std::thread::sleep(Duration::from_millis(300));
    let mut extra = TcpStream::connect(("127.0.0.1", port)).unwrap();
    extra.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let mut text = String::new();
    let _ = extra.read_to_string(&mut text);
    assert!(text.starts_with("HTTP/1.1 503"), "{text:?}");
    drop(idle);
}

/// A guessed code gets a few tries; then the pending code is cancelled, even for the right code.
#[test]
fn pairing_code_is_cancelled_after_repeated_wrong_guesses() {
    let dir = tmpdir();
    let port = start(|c| {
        c.dir = Some(dir.clone());
        c.origins.push("https://site.example.com".into());
    });
    let code = ck_native::pair_code(&dir, Duration::from_secs(120)).unwrap();
    for _ in 0..5 {
        let r = pair_req(port, "00000000", "http://localhost:5173", "kit");
        assert_eq!(r.status, 401);
    }
    let r = pair_req(port, &code, "http://localhost:5173", "kit");
    assert_eq!(r.status, 401, "{}", String::from_utf8_lossy(&r.body));
    assert!(String::from_utf8_lossy(&r.body).contains("no pairing code"));
}

/// A paired token only works from the origin it was paired with.
#[test]
fn paired_token_is_bound_to_its_origin() {
    let dir = tmpdir();
    let port = start(|c| {
        c.dir = Some(dir.clone());
        c.origins.push("https://site.example.com".into());
    });
    let (token, _id) = paired(&dir, port, "http://localhost:5173", "kit");
    let b = format!("Bearer {token}");
    let a = ("Authorization", b.as_str());
    // its own origin: ok, with CORS
    let r = req(port, "GET", "/version", &[a, ("Origin", "http://localhost:5173")], b"");
    assert_eq!(r.status, 200);
    assert_eq!(r.h("Access-Control-Allow-Origin"), Some("http://localhost:5173"));
    // a different origin, even an allowed one: 403 without CORS headers
    let r = req(port, "GET", "/version", &[a, ("Origin", "https://site.example.com")], b"");
    assert_eq!(r.status, 403, "{}", String::from_utf8_lossy(&r.body));
    assert!(r.h("Access-Control-Allow-Origin").is_none());
    // no Origin (non-browser): allowed, like the shared token
    assert_eq!(req(port, "GET", "/version", &[a], b"").status, 200);
    // the shared token is not origin-bound
    let s = auth();
    let r = req(port, "GET", "/version", &[(s.0, s.1.as_str()), ("Origin", "https://site.example.com")], b"");
    assert_eq!(r.status, 200);
    let _ = std::fs::remove_dir_all(&dir);
}

/// Eight simultaneous attempts on one code: exactly one wins, one record is written.
#[test]
fn concurrent_pairing_attempts_take_the_code_once() {
    let dir = tmpdir();
    let port = start(|c| c.dir = Some(dir.clone()));
    let code = ck_native::pair_code(&dir, Duration::from_secs(120)).unwrap();
    let ts: Vec<_> = (0..8)
        .map(|i| {
            let code = code.clone();
            let origin = "http://localhost:5173".to_string();
            std::thread::spawn(move || pair_req(port, &code, &origin, &format!("c{i}")).status)
        })
        .collect();
    let mut ok = 0;
    let mut refused = 0;
    for t in ts {
        match t.join().unwrap() {
            200 => ok += 1,
            401 => refused += 1,
            s => panic!("unexpected status {s}"),
        }
    }
    assert_eq!(ok, 1, "exactly one pairing wins");
    assert_eq!(refused, 7);
    assert_eq!(ck_native::list_clients(&dir).len(), 1, "one record, one token");
    let _ = std::fs::remove_dir_all(&dir);
}

/// Without a config dir there is no pairing store: the body is drained,501, nothing written.
#[test]
fn pairing_without_a_config_dir_is_not_implemented() {
    let port = start(|_| {});
    let r = pair_req(port, "01234567", "http://localhost:5173", "kit");
    assert_eq!(r.status, 501);
}

/// End-to-end through the real binary: --pair, POST /pair, clients, revoke, modes, and the
/// request log never carries a token or a code.
#[test]
fn binary_paired_flow_logs_no_secret() {
    use std::io::BufRead;
    use std::process::{Command, Stdio};
    let dir = tmpdir();
    let bin = env!("CARGO_BIN_EXE_ck-native");
    let mut child = Command::new(bin)
        .args(["--config-dir", dir.to_str().unwrap(), "--port", "0"])
        .stdout(Stdio::null())
        .stderr(Stdio::piped())
        .spawn()
        .expect("spawn ck-native");
    let mut err = std::io::BufReader::new(child.stderr.take().unwrap());
    let mut log = String::new();
    let port = loop {
        let mut line = String::new();
        assert!(err.read_line(&mut line).unwrap() > 0, "the service died before listening");
        log.push_str(&line);
        if let Some(i) = line.rfind("http://127.0.0.1:") {
            let p: u16 = line[i + "http://127.0.0.1:".len()..].trim().parse().expect("port");
            break p;
        }
    };
    // ck-native --pair prints a one-time code to stdout
    let out = Command::new(bin).args(["--config-dir", dir.to_str().unwrap(), "--pair"]).output().expect("run --pair");
    assert!(out.status.success());
    let printed = String::from_utf8(out.stdout).unwrap();
    let code = printed
        .lines()
        .find_map(|l| l.strip_prefix("pairing code: "))
        .unwrap_or_else(|| panic!("no code printed: {printed}"))
        .to_string();
    assert!(code.len() >= 8, "short code: {code}");
    // pair over the wire, then use the token
    let r = pair_req(port, &code, "http://localhost:5173", "log-test");
    assert_eq!(r.status, 200, "{}", String::from_utf8_lossy(&r.body));
    let reply = String::from_utf8(r.body).unwrap();
    let token = json_field(&reply, "token");
    let id = json_field(&reply, "id");
    let a = ("Authorization", format!("Bearer {token}"));
    assert_eq!(req(port, "GET", "/version", &[(a.0, a.1.as_str())], b"").status, 200);
    // the CLI lists and revokes the client while the service runs
    let run = |args: &[&str]| {
        let mut c = vec!["--config-dir", dir.to_str().unwrap()];
        c.extend_from_slice(args);
        let out = Command::new(bin).args(&c).output().expect("cli");
        String::from_utf8(out.stdout).unwrap()
    };
    let listed = run(&["clients"]);
    assert!(listed.contains(&id) && listed.contains("log-test"), "{listed}");
    let revoked = run(&["revoke", &id]);
    assert!(revoked.contains(&format!("revoked {id}")), "{revoked}");
    assert_eq!(req(port, "GET", "/version", &[(a.0, a.1.as_str())], b"").status, 401, "revoke is live");
    assert!(run(&["clients"]).contains("no paired clients"));
    // /ping still answers without a token
    let r = req(port, "GET", "/ping", &[], b"");
    assert_eq!(r.status, 200);
    assert_eq!(String::from_utf8(r.body).unwrap(), format!("{{\"service\":\"ck-native\",\"protocol\":{}}}", ck_native::PROTOCOL_VERSION));
    // stop the service, collect the rest of its request log
    let _ = child.kill();
    let _ = child.wait();
    let mut line = String::new();
    while err.read_line(&mut line).unwrap() > 0 {
        log.push_str(&line);
        line.clear();
    }
    let shared = std::fs::read_to_string(dir.join("token")).unwrap();
    let shared = shared.trim().to_string();
    // the log carries routes, never credentials
    assert!(log.contains("/pair"), "{log}");
    assert!(log.contains("/version"), "{log}");
    assert!(log.contains("/ping"), "{log}");
    assert!(!log.contains(&token), "paired token leaked into the log");
    assert!(!log.contains(&shared), "shared token leaked into the log");
    assert!(!log.contains(&code), "pairing code leaked into the log");
    // config dir and files:700 /600, the consumed code file is gone
    #[cfg(unix)]
    {
        use std::os::unix::fs::MetadataExt;
        assert_eq!(std::fs::metadata(&dir).unwrap().mode() & 0o777, 0o700, "config dir mode");
        assert_eq!(std::fs::metadata(dir.join("token")).unwrap().mode() & 0o777, 0o600, "token mode");
        assert_eq!(std::fs::metadata(dir.join("clients.json")).unwrap().mode() & 0o777, 0o600, "clients mode");
    }
    assert!(!dir.join("pairing.json").exists(), "the code file is single use");
    // tokens are stored hashed: the file holds no token material
    let clients = std::fs::read_to_string(dir.join("clients.json")).unwrap();
    assert!(!clients.contains(&token) && !clients.contains(&shared));
    let _ = std::fs::remove_dir_all(&dir);
}
