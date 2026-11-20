#!/usr/bin/env bash
# Build the kernel wasm and run its checks on the CI box instead of this (busy) machine. See docs/remote-build.md.
#   tools/remote_build.sh [--worktree DIR] [--checks gate,audit,large sample,loft,apex,fillet,section,spline,thin,suite,lint,text,engrave,projcurve,dfm,propeller,crashfuzz,wasmparity,all|none]
#                         [--cargo-test CRATE|"-p a -p b"|all] [--out DIR] [--no-sync-back] [--no-wasm] [--wasm-features LIST] [--jobs N] [--reset]
#                         [--test NAME] [--env K=V ...] [--fetch REL] [--musl-libm]
#   --test NAME: with --cargo-test, run only that integration test, with --nocapture; --env K=V: set for the cargo
#   tests (`{RUN}` in V is the box's run directory); --fetch REL: copy {RUN}/REL back to --out/REL after the run
#   --quick: agent iteration, NOT a release verification: wasm build, ck-ops + ck-topo tests, only the ck-model integration
#   tests listed in tools/ci/quick_tests.txt, and --checks (default gate, started beside the tests). --full = today's behaviour (the default).
#   Cargo runs write per-test-binary times to test_times.txt beside remote_results.txt (fetched back with --out).
#   --job-timeout SECS (default 10800): the whole job on the box (lock wait included) is killed after that and recorded as FAIL job-timeout.
#   --test-timeout SECS (default 2400): each cargo test binary is killed after that (cargo runner `timeout`). One job per worktree name:
#   a second submission while one waits or runs is refused (exit 75) with the run to fetch instead.
#   --slots N (default 3, REMOTE_BUILD_SLOTS): the box runs N cargo jobs at once, each with cores/N build jobs unless --jobs is given.
#   --no-impact: with --quick, do not add the crates that depend on the ones this branch changed (vs main).
#   --musl-libm: cargo test binaries use the wasm's libm (an LD_PRELOAD shim, tools/libm_shim; docs/wasm-parity.md)
#   tools/remote_build.sh --clean [--worktree DIR]     remove this worktree's directories on the box
#   tools/remote_build.sh --prune                      remove worktree directories on the box unused for 3 days
# Host: ssh alias from $REMOTE_BUILD_HOST, else the host part of this repo's `ci` git remote (as tools/ci_push.sh).
# Everything on the box lives under ~/ci/remote-build; nothing outside it is written or deleted.
set -uo pipefail
WT=""; CHECKS="gate"; CTEST=""; OUT=""; SYNC_BACK=1; MODE=run; RESET=0; NOWASM=0; WASMF=""; JOBS="${REMOTE_BUILD_JOBS:-}"; SLOTS="${REMOTE_BUILD_SLOTS:-3}"; IMPACT=1
TESTNAME=""; FETCH=""; ENVS=(); QUICK=0; CHECKS_SET=0; JOBT=10800; BINT=2400
usage() { sed -n 2,21p "$0" >&2; exit 64; }
while [ $# -gt 0 ]; do
  case "$1" in
    --worktree) WT="$2"; shift 2 ;;
    --checks) CHECKS="$2"; CHECKS_SET=1; shift 2 ;;
    --quick) QUICK=1; shift ;;
    --full) QUICK=0; shift ;;
    --job-timeout) JOBT="$2"; shift 2 ;;
    --test-timeout) BINT="$2"; shift 2 ;;
    --cargo-test) CTEST="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --no-sync-back) SYNC_BACK=0; shift ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --slots) SLOTS="$2"; shift 2 ;;
    --no-impact) IMPACT=0; shift ;;
    --reset) RESET=1; shift ;;
    --test) TESTNAME="$2"; shift 2 ;;
    --env) ENVS+=("$2"); shift 2 ;;
    --fetch) FETCH="$2"; shift 2 ;;
    --no-wasm) NOWASM=1; shift ;;
    --wasm-features) WASMF="$2"; shift 2 ;;
    --musl-libm) ENVS+=("CK_MUSL_LIBM=1"); shift ;;
    --clean) MODE=clean; shift ;;
    --prune) MODE=prune; shift ;;
    -h|--help) usage ;;
    *) echo "unknown arg: $1" >&2; usage ;;
  esac
done
if [ "$QUICK" = 1 ]; then
  [ -z "$CTEST" ] && [ -z "$TESTNAME" ] || { echo "--quick fixes the cargo tests: drop --cargo-test/--test (or use --full)" >&2; exit 64; }
  [ -s "$(dirname "$0")/ci/quick_tests.txt" ] || { echo "tools/ci/quick_tests.txt missing" >&2; exit 64; }
  CTEST="-p ck-ops -p ck-topo -p ck-model"
fi
case "$JOBS" in *[!0-9]*) echo "--jobs must be a number" >&2; exit 64 ;; esac
case "$SLOTS" in ''|*[!0-9]*|0) echo "--slots must be a number >= 1" >&2; exit 64 ;; esac
case "$JOBT" in ''|*[!0-9]*) echo "--job-timeout must be seconds" >&2; exit 64 ;; esac
case "$BINT" in ''|*[!0-9]*) echo "--test-timeout must be seconds" >&2; exit 64 ;; esac
case "$TESTNAME" in *[!A-Za-z0-9_-]*) echo "bad --test" >&2; exit 64 ;; esac
case "$FETCH" in /*|*..*|*[!A-Za-z0-9_./-]*) echo "bad --fetch" >&2; exit 64 ;; esac
for e in ${ENVS[@]+"${ENVS[@]}"}; do case "$e" in [A-Za-z_]*=*) ;; *) echo "bad --env $e" >&2; exit 64 ;; esac; done
WT=$(cd "${WT:-$(dirname "$0")/..}" && pwd -P) || { echo "bad --worktree" >&2; exit 64; }
[ -f "$WT/Cargo.toml" ] && [ -d "$WT/crates/ck-wasm" ] || { echo "not a kernel worktree: $WT" >&2; exit 64; }
OUT="${OUT:-$WT/target/wasm32-unknown-unknown/release}"
H=${REMOTE_BUILD_HOST:-}
if [ -z "$H" ]; then
  U=$(git -C "$WT" remote get-url ci 2>/dev/null) || { echo "set REMOTE_BUILD_HOST or add the 'ci' git remote (docs/ci.md)" >&2; exit 64; }
  H=${U%%:*}
fi
NAME="$(basename "$WT" | tr -c 'A-Za-z0-9._\n-' '_')-$(printf '%s' "$WT" | cksum | cut -d' ' -f1)"
SSHO=(-o BatchMode=yes -o ConnectTimeout=20 -o ServerAliveInterval=30)
RB='$HOME/ci/remote-build'     # expanded on the box only
QUIET='post-quantum|store now|upgraded|openssh.com/pq'
rsh() { ssh "${SSHO[@]}" "$H" "$@" 2> >(grep -Ev "$QUIET" >&2); }

if [ "$MODE" = prune ]; then
  rsh "set -u; RB=$RB; [ -d \"\$RB/wt\" ] || exit 0; cd \"\$RB/wt\" || exit 1
    for d in */; do d=\${d%/}; [ -n \"\$d\" ] || continue; s=\"\$RB/runs/\$d/.last-used\"
      if [ -z \"\$(find \"\$s\" -mtime -3 2>/dev/null)\" ]; then echo \"prune \$d\"; rm -rf -- \"\$RB/wt/\$d\" \"\$RB/target/\$d\" \"\$RB/runs/\$d\"; fi; done"
  exit $?
fi
if [ "$MODE" = clean ]; then
  rsh "set -u; RB=$RB; N=$(printf %q "$NAME"); case \"\$N\" in ''|*/*|.*) exit 2;; esac
    rm -rf -- \"\$RB/wt/\$N\" \"\$RB/target/\$N\" \"\$RB/runs/\$N\"; echo \"removed \$N\""
  exit $?
fi

# one job per worktree name: a job that waits for the box lock or runs holds runs/<name>/job.lock on the box for its whole life
# (a dropped ssh does not end it), so a second submission is refused here, before anything is synced over the running source.
BUSY=$(rsh "RB=$RB; mkdir -p \"\$RB/runs/$NAME\"; exec 8>\"\$RB/runs/$NAME/job.lock\"; flock -n 8 && exit 0; echo busy; cat \"\$RB/runs/$NAME/job.info\" 2>/dev/null; exit 75")
if [ $? = 75 ]; then
  echo "refused: a job for $NAME is already waiting or running on the box; not queueing a second one." >&2
  printf '%s\n' "$BUSY" | sed 's/^/  /' >&2
  echo "  results so far: ssh $H 'cat ci/remote-build/runs/$NAME/remote_results.txt'" >&2
  echo "  when it ends:   rsync $H:ci/remote-build/runs/$NAME/remote_results.txt $H:ci/remote-build/runs/$NAME/test_times.txt <dir>/" >&2
  exit 75
fi
T0=$(date +%s)
echo "remote_build: $NAME checks=$CHECKS cargo-test=${CTEST:-none} jobs=${JOBS:-auto} slots=$SLOTS"
[ "$QUICK" = 1 ] && echo "QUICK: not a release verification"
# 1. sync the source (never target/, .git, node_modules, .claude/). Files deleted locally are not deleted
#    on the box (--reset wipes the box's source copy first; the target dir is kept).
if [ "$RESET" = 1 ]; then rsh "rm -rf -- $RB/wt/$NAME" || exit 3; fi
rsh "mkdir -p $RB/wt/$NAME $RB/target $RB/runs/$NAME" || { echo "ssh failed" >&2; exit 3; }
rsync -a -e "ssh ${SSHO[*]}" \
  --exclude='/target' --exclude='/target-*' --exclude='.git' --exclude='node_modules' --exclude='.claude' \
  --exclude='/packages/dist' --exclude='*.log' --exclude='.DS_Store' \
  "$WT"/ "$H:ci/remote-build/wt/$NAME/" 2> >(grep -Ev "$QUIET" >&2) || { echo "rsync failed" >&2; exit 3; }
echo "synced in $(( $(date +%s) - T0 ))s"

# 2. build + checks on the box (script on stdin, every argument quoted)
read -r -d '' REMOTE <<'RSCRIPT'
set -u
NAME=$1; CHECKS=$2; CTEST=$3; JOBS=$4; NOWASM=${5:-0}; WASMF=${6:-}; REV=${7:-}; TESTNAME=${8:-}; QUICK=${9:-0}; BINT=${10:-2400}; SLOTS=${11:-3}; IMPCR=${12:-}
shift 12 2>/dev/null || shift $#
RB="$HOME/ci/remote-build"
case "$NAME" in ''|*/*|.*) echo "bad name"; exit 2;; esac
SRC="$RB/wt/$NAME"; RUN="$RB/runs/$NAME"; export CARGO_TARGET_DIR="$RB/target/$NAME"
export RUSTUP_HOME="$HOME/ci/rustup" CARGO_HOME="$HOME/ci/cargo" PATH="$HOME/ci/cargo/bin:$PATH"
export PYTHONUTF8=1
cd "$SRC" || exit 2
mkdir -p "$RUN"; exec 8>"$RUN/job.lock"
flock -n 8 || { echo "BUSY: another job holds $NAME"; exit 75; }   # released when this job (and anything it started) ends
printf 'started %s, pid %s, checks %s, cargo-test %s, quick %s\n' "$(date -u +%FT%TZ)" "$$" "$CHECKS" "$CTEST" "$QUICK" > "$RUN/job.info"
touch "$RUN/.last-used"; rm -f "$RUN/gate_summary.txt"; R="$RUN/remote_results.txt"; : > "$R"; TT="$RUN/test_times.txt"; : > "$TT"; TTRAW="$RUN/test_times.raw"; : > "$TTRAW"
if [ "$QUICK" = 1 ]; then echo "QUICK: not a release verification" | tee -a "$R"; fi
touch "$RUN/.parity-start"   # a native table older than this run is not compared (wasmparity)
TENV=(); for e in "$@"; do TENV+=("${e//\{RUN\}/$RUN}"); done   # --env pairs for the cargo tests
free=$(df -BG --output=avail "$RB" | tail -1 | tr -dc '0-9')
echo "box: cores=$(nproc) load=$(cut -d' ' -f1-3 /proc/loadavg) free=${free}G"
if [ "${free:-0}" -lt 15 ]; then echo "FAIL: less than 15G free on the box"; echo "FAIL disk 0" >> "$R"; exit 1; fi
# the gate and large sample check read the oracle at ../../kernel.wasm (read only: the gate writes its artifacts to $RUN/gate-art, CK_GATE_ART);
# shared by every run, so it is copied under a temp name and moved into place (a concurrent run never sees half a file)
# no comparison build on the box: the gate then runs standalone (tools/gate.mjs), without the checks that need one
[ -s "$RB/wt/kernel.wasm" ] || { cp "$HOME/kernel.wasm" "$RB/wt/.kernel.wasm.$$" 2>/dev/null && mv -f "$RB/wt/.kernel.wasm.$$" "$RB/wt/kernel.wasm"; } || echo "note: no comparison kernel.wasm; the gate runs standalone"
W="$CARGO_TARGET_DIR/wasm32-unknown-unknown/release/ck_wasm.wasm"
FAILS=0
rec() { # rec name rc secs
  if [ "$2" = 0 ]; then echo "ok $1 $3" >> "$R"; echo "== ok   $1 (${3}s)"; else echo "FAIL $1 $3 rc=$2" >> "$R"; echo "== FAIL $1 (${3}s, rc=$2)"; FAILS=$((FAILS+1)); fi
}
step() { local n=$1; shift; local t=$SECONDS rc; echo "== $n"; "$@"; rc=$?; rec "$n" $rc $((SECONDS-t)); return 0; }
if [ ! -d "$CARGO_TARGET_DIR" ] && [ -d "$HOME/ci/cache/kernel-build-target" ]; then   # seed deps (plain copy: hardlinks would be rewritten in place)
  echo "seeding target dir"; cp -a "$HOME/ci/cache/kernel-build-target" "$CARGO_TARGET_DIR"
fi
mkdir -p "$CARGO_TARGET_DIR"
BINRUN="timeout -k 10 $BINT"   # every native test binary runs under this (cargo runner): a hung one fails after --test-timeout, the rest go on
# N cargo job slots on the whole box (--slots, default 3): the first is the old single build.lock itself, so a job started by an
# older copy of this script (one flock on build.lock) still excludes every new job from that slot. A job takes the first free
# slot, polling every few seconds, and holds it for its build and cargo tests. Each run has its own target dir.
# housekeeping fallback: if the daily cron has not pruned idle worktree folders for 6 hours, do it now in the background
PRUNE="$HOME/ci/bin/prune_remote_build.sh"
[ -x "$PRUNE" ] && [ -z "$(find "$HOME/ci/prune.log" -mmin -360 2>/dev/null)" ] && (setsid nohup "$PRUNE" > /dev/null 2>&1 < /dev/null &)
echo "waiting for a build slot (of $SLOTS)"
SLOT=""
while [ -z "$SLOT" ]; do
  for i in $(seq 1 "$SLOTS"); do
    if [ "$i" = 1 ]; then LF="$RB/build.lock"; else LF="$RB/build.lock.$i"; fi
    exec 9>"$LF"
    if flock -n 9; then SLOT=$i; break; fi
    exec 9>&-
  done
  [ -n "$SLOT" ] || sleep 3
done
if [ -z "$JOBS" ]; then JOBS=$(( ($(nproc) + SLOTS - 1) / SLOTS )); [ "$JOBS" -ge 1 ] || JOBS=1; fi   # cores per slot, rounded up (16 cores, 3 slots: 6)
export CARGO_BUILD_JOBS="$JOBS"
export RUST_TEST_THREADS="${RUST_TEST_THREADS:-$JOBS}"   # test threads per slot, not per machine
echo "build slot $SLOT of $SLOTS taken, cargo jobs $JOBS"
[ "$NOWASM" = 1 ] || step build-wasm env CK_BUILD_REV="$REV" nice -n 5 cargo build -q -p ck-wasm ${WASMF:+--features "$WASMF"} --target wasm32-unknown-unknown --release
[ -s "$W" ] && echo "wasm bytes $(wc -c < "$W")"
# --musl-libm: the cargo test binaries run with the host libm's transcendental functions replaced by the `libm` crate's
# (the musl port the wasm32 build uses), through an LD_PRELOAD shim set as the cargo runner, so native float results
# match the wasm's (docs/wasm-parity.md). Only the test binaries are preloaded, never cargo or rustc.
case " ${TENV[*]-} " in *" CK_MUSL_LIBM=1 "*)
  SHIM="$CARGO_TARGET_DIR/libm_shim/release/liblibm_shim.so"
  ( cd tools/libm_shim && CARGO_TARGET_DIR="$CARGO_TARGET_DIR/libm_shim" cargo build -q --release --offline ) && [ -s "$SHIM" ] \
    && TENV+=("CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER=$BINRUN env LD_PRELOAD=$SHIM") && echo "musl libm shim: $SHIM" \
    || { echo "FAIL: the libm shim did not build"; rec musl-libm 1 0; } ;;
esac

# RustSec advisories for the lockfile (cargo-audit, installed under the box's CARGO_HOME): a known vulnerability fails the gate, warnings (unmaintained or
# yanked crates) are printed and do not. A missing tool is a failure, never a silent skip: install it with
#   CARGO_HOME=~/ci/cargo RUSTUP_HOME=~/ci/rustup cargo install cargo-audit --locked
audit_deps() {
  if ! command -v cargo-audit >/dev/null 2>&1; then echo "FAIL: cargo-audit is not installed on the box (see tools/remote_build.sh, audit_deps)"; return 1; fi
  ( flock 7; nice -n 5 cargo audit ) 7>"$RB/audit.lock"   # one at a time: cargo-audit updates the shared advisory db
}
# crash fuzz at the settings of its baseline. A regression it reports is re-run on its own (one worker, the same cpu-time timeout) and
# counts only if it fails again: a case that passes alone was load (case_4001 failed a whole full run once and passed every other time).
crashfuzz_check() {
  local out="$RUN/crash.out" rc=0 rc2=0 ids flaky="" i
  node tools/crash_fuzz.mjs --wasm "$W" --cases 5000 --seed 7 --budget-ms 400 --timeout 6000 --workers 8 --no-shrink --check --json "$RUN/crash.json" > "$out" 2>&1 || rc=$?
  cat "$out"
  [ "$rc" = 0 ] && return 0
  ids=$(sed -n 's/^  \([^ :]*\): [a-z-]* -> .*/\1/p' "$out" | paste -sd, -)
  [ -n "$ids" ] || return "$rc"
  echo "crashfuzz: re-running $ids one at a time"
  node tools/crash_fuzz.mjs --wasm "$W" --cases 5000 --seed 7 --budget-ms 400 --timeout 6000 --workers 1 --no-shrink --check --only "$ids" --json "$RUN/crash_rerun.json" > "$RUN/crash_rerun.out" 2>&1 || rc2=$?
  cat "$RUN/crash_rerun.out"
  for i in $(printf '%s' "$ids" | tr ',' ' '); do grep -q "^  $i: " "$RUN/crash_rerun.out" || flaky="$flaky $i"; done
  [ -n "$flaky" ] && echo "crashfuzz: flaky, passed on their own (not counted):$flaky"
  [ "$rc2" = 0 ] && echo "crashfuzz: no regression after the re-run"
  return "$rc2"
}
CH=$(printf '%s' "$CHECKS" | tr ',' ' ')
case " $CH " in *" all "*) CH="gate large sample loft apex fillet section spline thin suite lint text engrave projcurve dfm";; esac
case " $CH " in *" none "*) CH="";; esac
# the gate in the background (node, no build lock needed, so --quick starts it before the cargo tests)
export CK_GATE_WORKERS="${CK_GATE_WORKERS:-4}"   # gate.mjs worker threads (1 = the old sequential run)
start_gate() {
  ( t=$SECONDS; rc=0; mkdir -p "$RUN/gate-art"; export CK_GATE_ART="$RUN/gate-art"; nice -n 5 node tools/gate.mjs --fast --quiet --wasm "$W" > "$RUN/gate.out" 2>&1 || rc=$?; printf '\n== blend corpus\n' >> "$RUN/gate.out"; nice -n 5 node tools/blend_corpus.mjs --wasm "$W" >> "$RUN/gate.out" 2>&1 || rc=$?; printf '\n== cargo audit\n' >> "$RUN/gate.out"; audit_deps >> "$RUN/gate.out" 2>&1 || rc=$?; echo $rc > "$RUN/gate.rc"; echo $((SECONDS-t)) > "$RUN/gate.secs" ) 9>&- &   # 9>&-: the gate must not hold the build lock
  GATE_PID=$!
}
GATE_PID=""
# tools/ci/quick_tests.txt as "crate name" pairs: a bare name is a ck-model test, `ck-io:name` (or `io:name`) one of another crate
quick_pairs() { sed -e 's/#.*//' tools/ci/quick_tests.txt | tr -s ' \t' '\n\n' | grep . | awk '{ if (index($0,":")) { split($0,a,":"); c=a[1]; sub(/^ck-/,"",c); print "ck-" c, a[2] } else print "ck-model", $0 }'; }
if [ "$QUICK" = 1 ]; then   # quick: every name in tools/ci/quick_tests.txt must exist; the gate runs beside the tests
  quick_pairs | while read -r qc qt; do
    [ -f "crates/$qc/tests/$qt.rs" ] || { echo "FAIL: quick_tests.txt names $qc $qt, no crates/$qc/tests/$qt.rs"; echo "FAIL quick-list-$qt 0 rc=1" >> "$R"; }
  done
  grep -q '^FAIL quick-list-' "$R" && FAILS=$((FAILS+1))
  case " $CH " in *" gate "*) [ -s "$W" ] && start_gate ;; esac
fi
cargo_test_args() { # lib, bins and integration tests except ffi_diff (needs the dev machine's oracle library)
  local c=$1 t a="-p $1 --profile fast --lib --bins"
  if [ -n "$TESTNAME" ]; then echo "-p $1 --profile fast --test $TESTNAME -- --nocapture"; return; fi
  if [ "$QUICK" = 1 ]; then   # a crate with names in tools/ci/quick_tests.txt runs only those test files; one without runs all of its tests
    local qn; qn=$(quick_pairs | awk -v c="$c" '$1==c {print $2}')
    if [ -n "$qn" ]; then for t in $qn; do [ -f "crates/$c/tests/$t.rs" ] && a="$a --test $t"; done; echo "$a"; return; fi
  fi
  for t in $(ls "crates/$c/tests/"*.rs 2>/dev/null | sed -E 's|.*/([^/]+)\.rs$|\1|'); do [ "$t" = ffi_diff ] || a="$a --test $t"; done
  echo "$a"
}
# cargo test with per-binary times: the output passes through unchanged; each `Running <binary>` line is paired with the
# `finished in Ns` of its `test result:` line and appended to test_times.raw. Does not change the exit status or the order of runs.
timed_cargo_test() { # timed_cargo_test crate cargo-args...
  local c=$1; shift
  ( set -o pipefail; "$@" 2>&1 | CK_CRATE="$c" CK_RAW="$TTRAW" perl -e '
      $|=1; open(R,">>",$ENV{CK_RAW}); select((select(R),$|=1)[0]); my $cur="";
      while(<STDIN>){ print;
        if(/^\s*Running\s+(?:unittests\s+)?(\S+)/){ ($cur=$1)=~s{^tests/}{}; $cur=~s{\.rs$}{}; $cur=~s{^src/}{}; }
        elsif(/^\s*Doc-tests\s+(\S+)/){ $cur="doc-tests"; }
        elsif(/^test result:.*finished in ([0-9.]+)s/ && $cur ne ""){ print R "$1 $ENV{CK_CRATE} $cur\n"; $cur=""; } }' )
}
case " ${TENV[*]-} " in *CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER=*) ;; *) TENV+=("CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_RUNNER=$BINRUN") ;; esac
if [ -n "$CTEST" ]; then
  if [ "$CTEST" = all ]; then CRATES=$(ls crates | grep '^ck-' | sed 's/^ck-//'); else CRATES=$(printf '%s\n' $CTEST | grep -v '^-p$' | sed 's/^ck-//'); fi
  if [ "$QUICK" = 1 ] && [ -n "$IMPCR" ]; then   # --quick: add the crates that depend (directly or not) on the ones this branch changed
    AFF=$(cargo metadata --no-deps --format-version 1 --offline 2>/dev/null | node -e '
      const m=JSON.parse(require("fs").readFileSync(0,"utf8")); const ws=m.packages.map(p=>p.name); const want=process.argv[1].trim();
      const set=new Set(want==="ALL"?ws:want.split(/\s+/).filter(x=>ws.includes(x)));
      for (let grew=true; grew;) { grew=false; for (const p of m.packages) if (!set.has(p.name) && p.dependencies.some(d=>set.has(d.name))) { set.add(p.name); grew=true; } }
      console.log([...set].filter(n=>n!=="ck-wasm").sort().join(" "));' "$IMPCR")
    echo "quick impact: changed [$IMPCR] -> tests of: ${AFF:-none}"
    for a in $AFF; do a=${a#ck-}; case " $CRATES " in *" $a "*) ;; *) CRATES="$CRATES $a" ;; esac; done
  fi
  for c in $CRATES; do
    [ -d "crates/ck-$c" ] || { echo "no crate ck-$c"; rec "cargo-test-$c" 1 0; continue; }
    step "cargo-test-$c" timed_cargo_test "ck-$c" env ${TENV[@]+"${TENV[@]}"} nice -n 5 cargo test --no-fail-fast $(cargo_test_args "ck-$c")
  done
  sort -rn "$TTRAW" > "$TT"; echo "per-binary test times (seconds crate test), slowest first: $TT"; head -15 "$TT"
fi
flock -u 9; exec 9>&-    # checks are plain node: release the build lock

# The gate's real numbers, kept in runs/<name>/gate_summary.txt and as `gate-metric <name> <got>/<total>` lines in remote_results.txt, so they
# survive a dropped ssh (the streamed log is not needed) and "ok gate" is never the only evidence: wire, valid.brep, vol.brep, mass.brep and
# naming.stable must be present, not negative (-1 = the metric did not run), and not below tools/scoreboard.json; a missing/negative one
# or a drop fails the gate step. The blend corpus summary line is kept too.
gate_summary() { # gate_summary gate.out
  node -e '
    const fs=require("fs"); const [out,sbf]=process.argv.slice(1);
    const txt=fs.readFileSync(out,"utf8"); const [gt,rest=""]=txt.split("\n== blend corpus");
    const sb=fs.existsSync(sbf)?JSON.parse(fs.readFileSync(sbf,"utf8")):{};
    const got={}; for (const l of gt.split("\n")) { const m=/^(\S+)\s+(-?\d+)\/(-?\d+)/.exec(l); if (m) got[m[1]]=[+m[2],+m[3]]; }
    const lines=[]; let bad=0;
    for (const k of ["wire","valid.brep","vol.brep","mass.brep","naming.stable"]) {
      const g=got[k]; if (!g) { lines.push(`gate-metric ${k} missing`); console.log(`gate metric ${k}: not in the gate output`); bad++; continue; }
      const b=sb[k]; lines.push(`gate-metric ${k} ${g[0]}/${g[1]}`);
      if (g[0]<0||g[1]<0) { console.log(`gate metric ${k}: ${g[0]}/${g[1]} (negative: the metric did not run)`); bad++; }
      else if (b && g[0]<b[0]) { console.log(`gate metric ${k}: ${g[0]}/${g[1]} below the scoreboard ${b[0]}/${b[1]}`); bad++; }
    }
    for (const k of Object.keys(got)) if (got[k][0]<0||got[k][1]<0) { if (!lines.some(l=>l.startsWith(`gate-metric ${k} `))) lines.push(`gate-metric ${k} ${got[k][0]}/${got[k][1]}`); console.log(`gate metric ${k}: negative`); bad++; }
    const bm=/(\d+)\/(\d+) ok, (\d+) failed, (\d+) known/.exec(rest);
    if (bm) lines.push(`gate-metric blend-corpus ${bm[1]}/${bm[2]}`); else { lines.push("gate-metric blend-corpus missing"); console.log("gate metric blend-corpus: no summary line"); bad++; }
    fs.writeFileSync(process.argv[3], lines.join("\n")+"\n"); console.log(lines.join("\n"));
    process.exit(bad?1:0);' "$1" tools/scoreboard.json "$RUN/gate_summary.txt"
}
SUITE="region_select thin_extrude thread end_reference profile coincident_boolean hole_dims move spline_shell spline_boolean mate"
pids=""
if [ -s "$W" ]; then
  for c in $CH; do
    case "$c" in
      # the gate also runs the blend corpus (documented shortfalls carry `known` and never fail it)
      gate) if [ -z "$GATE_PID" ]; then start_gate; fi; pids="$pids $GATE_PID" ;;
      audit) step cargo-audit audit_deps ;;
      large sample) step large sample node tools/check_large sample.mjs --wasm "$W" ;;
      loft) step loft node tools/loft_point_check.mjs --wasm "$W" ;;
      apex) step apex node tools/loft_apex_check.mjs --wasm "$W" ;;
      fillet) step fillet node tools/concave_fillet_check.mjs --wasm "$W" ;;
      section) step section node tools/section_check.mjs --wasm "$W" ;;
      spline) step spline_shell node tools/spline_shell_check.mjs --wasm "$W"; step spline_boolean node tools/spline_boolean_check.mjs --wasm "$W" ;;
      thin) step thin_extrude node tools/thin_extrude_check.mjs --wasm "$W" ;;
      lint) step lint-protocol node tools/lint_protocol.mjs --wasm "$W" --quiet ;;
      text) step text-corpus node tools/text_corpus.mjs --wasm "$W" --quiet ;;
      engrave) step engrave-bench node tools/engrave_bench.mjs --wasm "$W" --quiet --check --machine box ;;
      projcurve) step project-curve-corpus node tools/project_curve_corpus.mjs --wasm "$W" --quiet ;;
      dfm) step dfm node tools/dfm_check.mjs --wasm "$W" --quiet ;;
      # the fillet table through the wasm, compared with the native table's BLEND_JSON when this run wrote one (tools/wasm_parity.sh)
      wasmparity) NJ="$RUN/blend/table.json"; [ -s "$NJ" ] && [ "$NJ" -nt "$RUN/.parity-start" ] || NJ=""
        step wasmparity env ${TENV[@]+"${TENV[@]}"} node tools/wasm_parity.mjs --wasm "$W" ${NJ:+--native "$NJ"} --json "$RUN/parity/parity.json" ;;
      # validate time against face count on generated parts (informational: prints a table; not part of `all`)
      validatescale) step validatescale node tools/validate_scale_check.mjs --wasm "$W" ;;
      # the crash/hang fuzz at the settings of its baseline (tools/crash_fuzz_baseline.json); lists every case that is not ok
      crashfuzz) step crashfuzz crashfuzz_check
        node -e 'const j=JSON.parse(require("fs").readFileSync(process.argv[1],"utf8")); const c={}; for (const r of j.results) c[r.cls]=(c[r.cls]||0)+1; console.log("crashfuzz classes", JSON.stringify(c)); for (const r of j.results) if (r.cls==="hang"||r.cls==="trap") console.log("  not ok:", r.id, r.cls, r.detail||"");' "$RUN/crash.json" ;;
      propeller) step propeller node tools/propeller_check.mjs --wasm "$W" ;;
      suite) for s in $SUITE; do step "check-$s" node "tools/${s}_check.mjs" --wasm "$W"; done ;;
      *) echo "unknown check $c"; rec "check-$c" 64 0 ;;
    esac
  done
  [ -n "$pids" ] && wait $pids
  if [ -f "$RUN/gate.rc" ]; then
    echo "== gate output"; cat "$RUN/gate.out"
    grc=$(cat "$RUN/gate.rc"); gate_summary "$RUN/gate.out" || { echo "gate metrics: FAILED (see gate-metric lines)"; [ "$grc" = 0 ] && grc=1; }
    rec gate "$grc" "$(cat "$RUN/gate.secs")"; cat "$RUN/gate_summary.txt" >> "$R"
    rm -f "$RUN/gate.rc" "$RUN/gate.secs" "$RUN/gate.out"
  fi
elif [ -n "$CH" ]; then
  echo "no wasm built: checks skipped"; [ "$FAILS" = 0 ] && FAILS=1
fi
echo "box load after: $(cut -d' ' -f1-3 /proc/loadavg)"
echo "REMOTE_FAILS=$FAILS"
[ -f "$RUN/gate_summary.txt" ] && echo "REMOTE_FAILS=$FAILS" >> "$RUN/gate_summary.txt"
[ "$FAILS" = 0 ]
RSCRIPT
# --quick: the crates this branch changes against main (committed, uncommitted and untracked), as `ck-x ck-y`, or ALL when the
# workspace manifest or lockfile changed; the box adds their reverse dependents to the quick test set (cargo metadata there).
IMPCR=""
if [ "$QUICK" = 1 ] && [ "$IMPACT" = 1 ]; then
  MB=$(git -C "$WT" merge-base HEAD main 2>/dev/null || true)
  if [ -n "$MB" ]; then
    CHG=$( { git -C "$WT" diff --name-only "$MB" 2>/dev/null; git -C "$WT" ls-files -o --exclude-standard 2>/dev/null; } | sort -u )
    case "
$CHG
" in *"
Cargo.toml
"*|*"
Cargo.lock
"*) IMPCR=ALL ;; *) IMPCR=$(printf '%s\n' "$CHG" | sed -n 's|^crates/\([^/]*\)/.*|\1|p' | sort -u | tr '\n' ' ') ;; esac
    echo "quick impact: changed crates vs main: ${IMPCR:-none}"
  fi
fi
# the commit the wasm is built from, reported by `ping` as `build` (`-dirty`: uncommitted changes in the tracked files)
REV=$(git -C "$WT" rev-parse --short=12 HEAD 2>/dev/null || true)
[ -n "$REV" ] && [ -n "$(git -C "$WT" status --porcelain --untracked-files=no 2>/dev/null)" ] && REV="$REV-dirty"
QA=""; for a in "$NAME" "$CHECKS" "$CTEST" "$JOBS" "$NOWASM" "$WASMF" "$REV" "$TESTNAME" "$QUICK" "$BINT" "$SLOTS" "$IMPCR" ${ENVS[@]+"${ENVS[@]}"}; do QA="$QA $(printf %q "$a")"; done
# The job runs DETACHED on the box (setsid nohup), writing runs/NAME/job.log and, when done, runs/NAME/job.rc. This side only
# follows the log and reconnects when the ssh link drops, so a dropped connection no longer kills a long run (or reports a
# failure that did not happen).
printf '%s\n' "$REMOTE" | rsh "cat > $RB/runs/$NAME/remote.sh" || { echo "ssh failed" >&2; exit 3; }
printf '%s\n' 'RB="$HOME/ci/remote-build"' "cd \"\$RB/runs/$NAME\"" "timeout -k 30 $JOBT bash remote.sh $QA" 'echo $? > job.rc' | rsh "cat > $RB/runs/$NAME/job.sh" || { echo "ssh failed" >&2; exit 3; }
rsh "cd $RB/runs/$NAME && rm -f job.rc job.log && (setsid nohup bash job.sh > job.log 2>&1 < /dev/null &)" || { echo "could not start the job on the box" >&2; exit 3; }
FOLLOW=$(mktemp); SZ=0; FAILS_IN_ROW=0; RC=255
while :; do
  DONE=0
  if rsh "test -f $RB/runs/$NAME/job.rc" 2>/dev/null; then DONE=1; fi
  if rsh "tail -c +$((SZ+1)) $RB/runs/$NAME/job.log" >> "$FOLLOW" 2>/dev/null; then
    FAILS_IN_ROW=0
    tail -c +$((SZ+1)) "$FOLLOW"; SZ=$(wc -c < "$FOLLOW" | tr -d ' ')
    if [ "$DONE" = 1 ]; then RC=$(rsh "cat $RB/runs/$NAME/job.rc" 2>/dev/null | tr -dc '0-9'); RC=${RC:-255}; break; fi
  else
    FAILS_IN_ROW=$((FAILS_IN_ROW+1)); echo "(ssh link dropped, reconnecting: $FAILS_IN_ROW)" >&2
    [ "$FAILS_IN_ROW" -gt 120 ] && { echo "giving up following the box job; it may still be running (runs/$NAME/job.log)" >&2; break; }
  fi
  sleep 5
done
rm -f "$FOLLOW"
if [ "$RC" = 124 ]; then   # --job-timeout: the box job was killed
  echo "FAIL: job timeout (${JOBT}s), killed on the box" >&2
  rsh "echo 'FAIL job-timeout $JOBT rc=124' >> $RB/runs/$NAME/remote_results.txt" || true
fi

# 3. copy back
if [ "$SYNC_BACK" = 1 ]; then
  mkdir -p "$OUT"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci/remote-build/target/$NAME/wasm32-unknown-unknown/release/ck_wasm.wasm" "$OUT/ck_wasm.wasm" 2>/dev/null \
    && echo "wasm -> $OUT/ck_wasm.wasm" || echo "(no wasm to copy back)"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci/remote-build/runs/$NAME/remote_results.txt" "$OUT/remote_results.txt" 2>/dev/null
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci/remote-build/runs/$NAME/test_times.txt" "$OUT/test_times.txt" 2>/dev/null && [ -s "$OUT/test_times.txt" ] && echo "test times -> $OUT/test_times.txt"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci/remote-build/runs/$NAME/gate_summary.txt" "$OUT/gate_summary.txt" 2>/dev/null && echo "gate summary -> $OUT/gate_summary.txt"
  [ -f "$OUT/remote_results.txt" ] && { echo "--- $OUT/remote_results.txt"; cat "$OUT/remote_results.txt"; }
fi
if [ -n "$FETCH" ]; then
  mkdir -p "$OUT/$FETCH"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci/remote-build/runs/$NAME/$FETCH/" "$OUT/$FETCH/" 2>/dev/null && echo "fetched -> $OUT/$FETCH" || echo "(nothing fetched)"
fi
echo "total $(( $(date +%s) - T0 ))s, exit $RC"
exit "$RC"
