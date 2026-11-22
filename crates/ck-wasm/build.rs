//! Build identity for `ck_commit()`: `CK_COMMIT` from the environment, else the git revision of the
//! source tree, else "unknown".
use std::process::Command;

fn git(args: &[&str]) -> Option<String> {
    let o = Command::new("git").args(args).output().ok().filter(|o| o.status.success())?;
    Some(String::from_utf8_lossy(&o.stdout).trim().to_string()).filter(|s| !s.is_empty())
}

fn main() {
    println!("cargo:rerun-if-env-changed=CK_COMMIT");
    let rev = std::env::var("CK_COMMIT")
        .ok()
        .filter(|s| !s.trim().is_empty())
        .or_else(|| git(&["rev-parse", "HEAD"]))
        .unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=CK_WASM_COMMIT={}", rev.trim());
    // Rebuild when the checked-out commit moves (also inside a linked worktree, where `.git` is a file).
    if let Some(p) = git(&["rev-parse", "--git-path", "HEAD"]) {
        println!("cargo:rerun-if-changed={p}");
    }
}
