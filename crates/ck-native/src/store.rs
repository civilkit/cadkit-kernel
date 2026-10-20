//! Pairing records and one-time codes in the config dir (mode 700, files mode 600).
//!
//! Two files next to the shared `token`:
//! - `clients.json` — paired clients: id, **hashed** token, bound origin, label, creation time.
//! - `pairing.json` — the one-time code a `ck-native --pair` wrote: code + expiry (unix seconds).
//!
//! The service keeps `clients.json` in a mtime-stamped cache (reloaded on change, so a
//! `ck-native revoke` takes effect on the running service), the CLI reads and writes the
//! files directly. Tokens are never stored; only their SHA-256.

use std::io::{Read, Write};
use std::path::Path;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ck_json::{Json, Map};

/// How long a `--pair` code lives (spec: two minutes, single use).
pub const PAIR_TTL: Duration = Duration::from_secs(120);

#[derive(Debug, Clone)]
pub struct Record {
    pub id: String,
    pub hash: String,
    pub origin: String,
    pub label: String,
    pub created: u64,
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

/// SHA-256 of the token, hex (the stored form; the token itself is never written).
pub fn sha256_hex(s: &str) -> String {
    let d = ck_text::sha256(&[s.as_bytes()]);
    d.iter().map(|b| format!("{b:02x}")).collect()
}

fn random_hex(bytes: usize) -> Result<String, String> {
    let mut b = vec![0u8; bytes];
    std::fs::File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut b))
        .map_err(|e| format!("random: {e}"))?;
    Ok(b.iter().map(|x| format!("{x:02x}")).collect())
}

/// Create the config dir if needed (mode 700 on Unix).
pub fn ensure_dir(dir: &Path) -> Result<(), String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("create {}: {e}", dir.display()))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(dir, std::fs::Permissions::from_mode(0o700));
    }
    Ok(())
}

/// Write a small file with mode 600 (Unix): temp file in the same dir, then rename.
fn write600(path: &Path, bytes: &[u8]) -> Result<(), String> {
    let tmp = path.with_extension("tmp");
    let mut o = std::fs::OpenOptions::new();
    o.write(true).create(true).truncate(true);
    #[cfg(unix)]
    {
        use std::os::unix::fs::OpenOptionsExt;
        o.mode(0o600);
    }
    let mut f = o.open(&tmp).map_err(|e| format!("write {}: {e}", tmp.display()))?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = f.set_permissions(std::fs::Permissions::from_mode(0o600));
    }
    f.write_all(bytes).map_err(|e| format!("write {}: {e}", tmp.display()))?;
    drop(f);
    std::fs::rename(&tmp, path).map_err(|e| format!("rename {}: {e}", path.display()))
}

fn stamp(path: &Path) -> Option<(u64, u64)> {
    let m = std::fs::metadata(path).ok()?;
    let s = m.modified().ok()?.duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0);
    Some((s, m.len()))
}

// ---------------------------------------------------------------- clients

/// All paired clients (missing or unreadable file = none).
pub fn list_clients(dir: &Path) -> Vec<Record> {
    let Ok(text) = std::fs::read_to_string(dir.join("clients.json")) else { return vec![] };
    let Ok(v) = ck_json::parse(&text) else { return vec![] };
    let Some(arr) = v.get("clients").and_then(|c| c.as_arr()) else { return vec![] };
    arr.iter()
        .filter_map(|c| {
            Some(Record {
                id: c.get("id")?.as_str()?.to_string(),
                hash: c.get("hash")?.as_str()?.to_string(),
                origin: c.get("origin")?.as_str()?.to_string(),
                label: c.get("label")?.as_str()?.to_string(),
                created: c.get("created")?.as_f64()? as u64,
            })
        })
        .collect()
}

fn write_clients(dir: &Path, records: &[Record]) -> Result<(), String> {
    let clients: Vec<Json> = records
        .iter()
        .map(|r| {
            let mut m: Map = vec![];
            m.push(("id".into(), Json::Str(r.id.clone())));
            m.push(("hash".into(), Json::Str(r.hash.clone())));
            m.push(("origin".into(), Json::Str(r.origin.clone())));
            m.push(("label".into(), Json::Str(r.label.clone())));
            m.push(("created".into(), Json::Num(r.created as f64)));
            Json::Obj(m)
        })
        .collect();
    let mut root: Map = vec![];
    root.push(("clients".into(), Json::Arr(clients)));
    write600(&dir.join("clients.json"), ck_json::to_string(&Json::Obj(root)).as_bytes())
}

/// Mint a one-time pairing code: writes `pairing.json` (mode 600) with `ttl`, returns the code.
/// The service consumes the file on the first successful `POST /pair`.
pub fn pair_code(dir: &Path, ttl: Duration) -> Result<String, String> {
    ensure_dir(dir)?;
    let code = random_hex(4)?;
    let mut m: Map = vec![];
    m.push(("code".into(), Json::Str(code.clone())));
    m.push(("exp".into(), Json::Num((now() + ttl.as_secs()) as f64)));
    write600(&dir.join("pairing.json"), ck_json::to_string(&Json::Obj(m)).as_bytes())?;
    Ok(code)
}

struct Pending {
    code: String,
    exp: u64,
}

fn read_pending(dir: &Path) -> Option<Pending> {
    let text = std::fs::read_to_string(dir.join("pairing.json")).ok()?;
    let v = ck_json::parse(&text).ok()?;
    Some(Pending { code: v.get("code")?.as_str()?.to_string(), exp: v.get("exp")?.as_f64()? as u64 })
}

/// Remove the consumed code. Single use: whoever removes it first wins.
fn consume_pending(dir: &Path) {
    let _ = std::fs::remove_file(dir.join("pairing.json"));
}

/// Remove one client by id; `false` if unknown. A running service notices on the next request.
pub fn revoke(dir: &Path, id: &str) -> Result<bool, String> {
    let mut records = list_clients(dir);
    let before = records.len();
    records.retain(|r| r.id != id);
    if records.len() == before {
        return Ok(false);
    }
    write_clients(dir, &records)?;
    Ok(true)
}

/// Append a freshly paired client (read-modify-write; the file is small).
fn append_client(dir: &Path, rec: &Record) -> Result<(), String> {
    let mut records = list_clients(dir);
    records.push(rec.clone());
    write_clients(dir, &records)
}

// ---------------------------------------------------------------- service-side cache

/// `clients.json` kept in memory, reloaded when the file's stamp changes.
pub struct Cache {
    stamp: Option<(u64, u64)>,
    loaded: bool,
    records: Vec<Record>,
}

impl Cache {
    pub fn new() -> Self {
        Cache { stamp: None, loaded: false, records: vec![] }
    }

    /// The current records; re-reads the file when it changed (missing = empty).
    pub fn records(&mut self, dir: &Path) -> &[Record] {
        let s = stamp(&dir.join("clients.json"));
        if !self.loaded || s != self.stamp {
            self.stamp = s;
            self.loaded = true;
            self.records = if s.is_some() { list_clients(dir) } else { vec![] };
        }
        &self.records
    }

    /// The file was just written by this process: re-read next time regardless of the stamp.
    pub fn invalidate(&mut self) {
        self.loaded = false;
    }
}

/// The pending code, checked under the caller's lock. `Err` explains why it cannot be used.
pub fn take_code(dir: &Path, code: &str) -> Result<(), &'static str> {
    let Some(p) = read_pending(dir) else { return Err("no pairing code") };
    if now() >= p.exp {
        return Err("pairing code expired");
    }
    if !crate::ct_eq(p.code.as_bytes(), code.as_bytes()) {
        return Err("wrong pairing code");
    }
    consume_pending(dir);
    Ok(())
}

/// Issue a token for `origin`/`label`; the token is returned, only its hash is stored.
pub fn admit(dir: &Path, origin: &str, label: &str) -> Result<(String, Record), String> {
    let token = random_hex(32)?;
    let rec = Record {
        id: random_hex(4)?,
        hash: sha256_hex(&token),
        origin: origin.to_string(),
        label: label.to_string(),
        created: now(),
    };
    append_client(dir, &rec)?;
    Ok((token, rec))
}
