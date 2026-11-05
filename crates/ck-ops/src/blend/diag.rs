//! Debug output and error helpers of the blend modules.
//!
//! Debug output is behind `CK_BLEND_DEBUG` (level 1), `CK_BLEND_DEBUG2` and `CK_BLEND_DEBUG3` (more verbose
//! traces); nothing is printed when they are unset and nothing here changes a result. The error constructors
//! give a refusal its `detail` (and optional `hint`) and trace it.

use super::BlendError;

/// Debug level 1 is on (`CK_BLEND_DEBUG` set).
pub(super) fn on() -> bool {
    std::env::var_os("CK_BLEND_DEBUG").is_some()
}

/// Debug level `n` (2 or 3) is on (`CK_BLEND_DEBUG2` / `CK_BLEND_DEBUG3` set).
pub(super) fn on_level(n: u8) -> bool {
    match n {
        1 => on(),
        2 => std::env::var_os("CK_BLEND_DEBUG2").is_some(),
        _ => std::env::var_os("CK_BLEND_DEBUG3").is_some(),
    }
}

/// One trace line (level 1), stamped with the time since the first one on native builds.
pub(super) fn dbg(m: &str) {
    if on() {
        #[cfg(not(target_arch = "wasm32"))]
        {
            thread_local!(static T0: std::time::Instant = std::time::Instant::now());
            eprintln!("freeform [{:.1}s]: {m}", T0.with(|t| t.elapsed().as_secs_f64()));
        }
        #[cfg(target_arch = "wasm32")]
        eprintln!("freeform: {m}");
    }
}

thread_local! {
    static LAST_FAIL: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

/// The reason the last blend step gave up (see [`dbg_fail`]); empty when none did.
pub(super) fn last_fail() -> String {
    LAST_FAIL.with(|c| c.borrow().clone())
}

/// Records why a blend step gave up; `CK_BLEND_DEBUG=1` prints it (native runs only).
pub(super) fn dbg_fail(m: &str) {
    LAST_FAIL.with(|c| *c.borrow_mut() = m.chars().take(400).collect());
    if on() {
        eprintln!("blend fail: {m}");
    }
}

/// A blend refusal: the message is the `detail` of the structured error (`blendUnsupported` / `blendFailed` carry
/// `params.detail`), so a rejected edit says which chain / face pair / end condition failed.
pub(super) fn unsupported(m: &str) -> BlendError {
    dbg(m);
    BlendError::detail("blendUnsupported", m)
}

pub(super) fn failed(m: &str) -> BlendError {
    dbg(m);
    BlendError::detail("blendFailed", m)
}

/// As [`failed`], with a `hint` the wizard shows the user for what to change.
pub(super) fn failed_hint(m: &str, hint: &str) -> BlendError {
    dbg(m);
    BlendError { code: "blendFailed".into(), detail: Some(m.into()), params: vec![("hint".into(), hint.into())] }
}

/// As [`unsupported`], with a `hint`.
pub(super) fn unsupported_hint(m: &str, hint: &str) -> BlendError {
    dbg(m);
    BlendError { code: "blendUnsupported".into(), detail: Some(m.into()), params: vec![("hint".into(), hint.into())] }
}
