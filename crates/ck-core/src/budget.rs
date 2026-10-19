//! Work budgets: a wall-clock deadline that long loops poll so a slow request ends with a structured error
//! instead of running for minutes.
//!
//! Two nested limits, both off until a clock is installed (`set_clock`; ck-wasm wires the host clock, native
//! tests install none, so they are never cut short):
//! * the regen limit (`opts.regenTimeMs`, default none): the whole document;
//! * the feature limit (`opts.featureTimeMs`, default [`DEFAULT_FEATURE_MS`]): one feature.
//!
//! A loop calls [`poll`] once per iteration (cheap: the clock is read every [`STRIDE`] calls) and, when it
//! returns `true`, stops and returns its own `...BudgetExceeded` error. Once a limit has passed it stays passed
//! until its scope ends, so the callers up the stack all see it and unwind.
use std::cell::Cell;

/// Default per-feature budget (ms) when the host has a clock.
pub const DEFAULT_FEATURE_MS: f64 = 30_000.0;
/// The clock is read on every `STRIDE`-th poll.
const STRIDE: u32 = 16;

type Clock = fn() -> f64;

thread_local! {
    static CLOCK: Cell<Option<Clock>> = const { Cell::new(None) };
    /// (deadline ms, start ms, limit ms) of the regen and of the feature scope.
    static REGEN: Cell<Option<(f64, f64, f64)>> = const { Cell::new(None) };
    static FEATURE: Cell<Option<(f64, f64, f64)>> = const { Cell::new(None) };
    /// Latched expiry of the regen and of the feature scope.
    static EXP_REGEN: Cell<bool> = const { Cell::new(false) };
    static EXP_FEATURE: Cell<bool> = const { Cell::new(false) };
    static TICK: Cell<u32> = const { Cell::new(0) };
}

/// Which limit ran out.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Which {
    Regen,
    Feature,
}

/// Install the monotonic clock (ms).
pub fn set_clock(f: Clock) {
    CLOCK.with(|c| c.set(Some(f)));
}

fn now() -> Option<f64> {
    CLOCK.with(|c| c.get()).map(|f| f())
}

fn cell(which: Which) -> (&'static std::thread::LocalKey<Cell<Option<(f64, f64, f64)>>>, &'static std::thread::LocalKey<Cell<bool>>) {
    match which {
        Which::Regen => (&REGEN, &EXP_REGEN),
        Which::Feature => (&FEATURE, &EXP_FEATURE),
    }
}

/// A scope restores the previous limit when dropped.
pub struct Scope {
    which: Which,
    prev: Option<(f64, f64, f64)>,
    prev_exp: bool,
}

impl Scope {
    /// Start a limit of `ms` (none, zero, negative or non-finite: no limit). Without a clock there is no limit.
    pub fn start(which: Which, ms: Option<f64>) -> Scope {
        let (limit, exp) = cell(which);
        let prev = limit.with(|c| c.get());
        let prev_exp = exp.with(|e| e.replace(false));
        let new = match (now(), ms) {
            (Some(t), Some(ms)) if ms.is_finite() && ms > 0.0 => Some((t + ms, t, ms)),
            _ => None,
        };
        limit.with(|c| c.set(new));
        Scope { which, prev, prev_exp }
    }
}

impl Drop for Scope {
    fn drop(&mut self) {
        let (limit, exp) = cell(self.which);
        limit.with(|c| c.set(self.prev));
        exp.with(|e| e.set(self.prev_exp));
    }
}

fn latched() -> bool {
    EXP_REGEN.with(|e| e.get()) || EXP_FEATURE.with(|e| e.get())
}

/// Has a limit run out? Reads the clock every [`STRIDE`]-th call (and always when a limit already ran out).
pub fn poll() -> bool {
    if latched() {
        return true;
    }
    if REGEN.with(|c| c.get()).is_none() && FEATURE.with(|c| c.get()).is_none() {
        return false;
    }
    let t = TICK.with(|t| {
        let v = t.get().wrapping_add(1);
        t.set(v);
        v
    });
    if t % STRIDE != 0 {
        return false;
    }
    check_now()
}

/// Like [`poll`] but always reads the clock (for loops whose iterations are already expensive).
pub fn expired() -> bool {
    if latched() {
        return true;
    }
    check_now()
}

fn check_now() -> bool {
    let Some(t) = now() else { return false };
    for which in [Which::Regen, Which::Feature] {
        let (limit, exp) = cell(which);
        if let Some((deadline, _, _)) = limit.with(|c| c.get()) {
            if t > deadline {
                exp.with(|e| e.set(true));
            }
        }
    }
    latched()
}

/// The host clock reading in ms, if a clock is installed (for measuring a stretch of work with [`since`]).
pub fn mark() -> Option<f64> {
    now()
}

/// Ms elapsed since a [`mark`] (0 without a clock).
pub fn since(mark: Option<f64>) -> f64 {
    match (mark, now()) {
        (Some(a), Some(b)) => (b - a).max(0.0),
        _ => 0.0,
    }
}

/// Which limit ran out (the regen's wins), with its limit and the time spent since its start, in ms:
/// `(which, limit, elapsed)`.
pub fn report() -> Option<(Which, f64, f64)> {
    let which = if EXP_REGEN.with(|e| e.get()) {
        Which::Regen
    } else if EXP_FEATURE.with(|e| e.get()) {
        Which::Feature
    } else {
        return None;
    };
    let (_, start, limit) = cell(which).0.with(|c| c.get())?;
    Some((which, limit, (now().unwrap_or(start) - start).max(0.0)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;

    thread_local! { static T: Cell<f64> = const { Cell::new(0.0) }; }
    fn fake() -> f64 {
        T.with(|t| t.get())
    }

    #[test]
    fn feature_limit_expires_and_clears() {
        set_clock(fake);
        T.with(|t| t.set(0.0));
        assert!(!expired());
        {
            let _s = Scope::start(Which::Feature, Some(100.0));
            assert!(!expired());
            T.with(|t| t.set(150.0));
            assert!(expired());
            assert!(poll());
            let (w, limit, el) = report().unwrap();
            assert_eq!((w, limit, el), (Which::Feature, 100.0, 150.0));
        }
        assert!(!expired());
        assert!(!poll());
        CLOCK.with(|c| c.set(None));
    }

    #[test]
    fn regen_limit_survives_feature_scopes() {
        set_clock(fake);
        T.with(|t| t.set(0.0));
        let _r = Scope::start(Which::Regen, Some(1000.0));
        {
            let _f = Scope::start(Which::Feature, Some(100.0));
            T.with(|t| t.set(1500.0));
            assert!(expired());
            assert_eq!(report().unwrap().0, Which::Regen);
        }
        assert!(expired());
        CLOCK.with(|c| c.set(None));
    }

    #[test]
    fn no_clock_no_limit() {
        CLOCK.with(|c| c.set(None));
        let _s = Scope::start(Which::Feature, Some(1.0));
        assert!(!expired());
        assert!(!poll());
    }
}
