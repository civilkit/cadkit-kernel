#!/usr/bin/env bash
# Build the kernel wasm and run its checks on the CI box instead of this (busy) machine. See docs/remote-build.md.
#   tools/remote_build.sh [--worktree DIR] [--checks gate,large sample,loft,apex,fillet,section,spline,thin,suite,lint,text,projcurve,dfm,all|none]
#                         [--cargo-test CRATE|"-p a -p b"|all] [--out DIR] [--no-sync-back] [--no-wasm] [--wasm-features LIST] [--jobs N] [--reset]
#   tools/remote_build.sh --clean [--worktree DIR]     remove this worktree's directories on the box
#   tools/remote_build.sh --prune                      remove worktree directories on the box unused for 3 days
# Host: ssh alias from $REMOTE_BUILD_HOST, else the host part of this repo's `ci` git remote (as tools/ci_push.sh).
# Everything on the box lives under ~/ci-cad/remote-build; nothing outside it is written or deleted.
set -uo pipefail
WT=""; CHECKS="gate"; CTEST=""; OUT=""; SYNC_BACK=1; MODE=run; RESET=0; NOWASM=0; WASMF=""; JOBS="${REMOTE_BUILD_JOBS:-12}"
usage() { sed -n 2,7p "$0" >&2; exit 64; }
while [ $# -gt 0 ]; do
  case "$1" in
    --worktree) WT="$2"; shift 2 ;;
    --checks) CHECKS="$2"; shift 2 ;;
    --cargo-test) CTEST="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --no-sync-back) SYNC_BACK=0; shift ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --reset) RESET=1; shift ;;
    --no-wasm) NOWASM=1; shift ;;
    --wasm-features) WASMF="$2"; shift 2 ;;
    --clean) MODE=clean; shift ;;
    --prune) MODE=prune; shift ;;
    -h|--help) usage ;;
    *) echo "unknown arg: $1" >&2; usage ;;
  esac
done
case "$JOBS" in ''|*[!0-9]*) echo "--jobs must be a number" >&2; exit 64 ;; esac
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
RB='$HOME/ci-cad/remote-build'     # expanded on the box only
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

T0=$(date +%s)
echo "remote_build: $NAME checks=$CHECKS cargo-test=${CTEST:-none} jobs=$JOBS"
# 1. sync the source (never target/, .git, node_modules, .claude/). Files deleted locally are not deleted
#    on the box (--reset wipes the box's source copy first; the target dir is kept).
if [ "$RESET" = 1 ]; then rsh "rm -rf -- $RB/wt/$NAME" || exit 3; fi
rsh "mkdir -p $RB/wt/$NAME $RB/target $RB/runs/$NAME" || { echo "ssh failed" >&2; exit 3; }
rsync -a -e "ssh ${SSHO[*]}" \
  --exclude='/target' --exclude='/target-*' --exclude='.git' --exclude='node_modules' --exclude='.claude' \
  --exclude='/packages/dist' --exclude='*.log' --exclude='.DS_Store' \
  "$WT"/ "$H:ci-cad/remote-build/wt/$NAME/" 2> >(grep -Ev "$QUIET" >&2) || { echo "rsync failed" >&2; exit 3; }
echo "synced in $(( $(date +%s) - T0 ))s"

# 2. build + checks on the box (script on stdin, every argument quoted)
read -r -d '' REMOTE <<'RSCRIPT'
set -u
NAME=$1; CHECKS=$2; CTEST=$3; JOBS=$4; NOWASM=${5:-0}; WASMF=${6:-}; REV=${7:-}
RB="$HOME/ci-cad/remote-build"
case "$NAME" in ''|*/*|.*) echo "bad name"; exit 2;; esac
SRC="$RB/wt/$NAME"; RUN="$RB/runs/$NAME"; export CARGO_TARGET_DIR="$RB/target/$NAME"
export RUSTUP_HOME="$HOME/ci-cad/rustup" CARGO_HOME="$HOME/ci-cad/cargo" PATH="$HOME/ci-cad/cargo/bin:$PATH" CARGO_BUILD_JOBS="$JOBS"
export PYTHONUTF8=1
cd "$SRC" || exit 2
touch "$RUN/.last-used"; R="$RUN/remote_results.txt"; : > "$R"
free=$(df -BG --output=avail "$RB" | tail -1 | tr -dc '0-9')
echo "box: cores=$(nproc) load=$(cut -d' ' -f1-3 /proc/loadavg) free=${free}G"
if [ "${free:-0}" -lt 15 ]; then echo "FAIL: less than 15G free on the box"; echo "FAIL disk 0" >> "$R"; exit 1; fi
# the gate and large sample check read the oracle at ../../kernel.wasm (and the gate writes next to it): private copy inside the root
[ -s "$RB/wt/kernel.wasm" ] || cp "$HOME/kernel.wasm" "$RB/wt/kernel.wasm" || { echo "FAIL: oracle kernel.wasm missing"; exit 1; }
W="$CARGO_TARGET_DIR/wasm32-unknown-unknown/release/ck_wasm.wasm"
FAILS=0
rec() { # rec name rc secs
  if [ "$2" = 0 ]; then echo "ok $1 $3" >> "$R"; echo "== ok   $1 (${3}s)"; else echo "FAIL $1 $3 rc=$2" >> "$R"; echo "== FAIL $1 (${3}s, rc=$2)"; FAILS=$((FAILS+1)); fi
}
step() { local n=$1; shift; local t=$SECONDS rc; echo "== $n"; "$@"; rc=$?; rec "$n" $rc $((SECONDS-t)); return 0; }
if [ ! -d "$CARGO_TARGET_DIR" ] && [ -d "$HOME/ci-cad/cache/kernel-build-target" ]; then   # seed deps (plain copy: hardlinks would be rewritten in place)
  echo "seeding target dir"; cp -a "$HOME/ci-cad/cache/kernel-build-target" "$CARGO_TARGET_DIR"
fi
mkdir -p "$CARGO_TARGET_DIR"
# one cargo job set at a time on the whole box (this also serialises each target dir)
exec 9>"$RB/build.lock"
echo "waiting for the build lock"; flock 9; echo "build lock taken"
[ "$NOWASM" = 1 ] || step build-wasm env CK_BUILD_REV="$REV" nice -n 5 cargo build -q -p ck-wasm ${WASMF:+--features "$WASMF"} --target wasm32-unknown-unknown --release
[ -s "$W" ] && echo "wasm bytes $(wc -c < "$W")"

cargo_test_args() { # lib, bins and integration tests except ffi_diff (needs the dev machine's oracle library)
  local c=$1 t a="-p $1 --profile fast --lib --bins"
  for t in $(ls "crates/$c/tests/"*.rs 2>/dev/null | sed -E 's|.*/([^/]+)\.rs$|\1|'); do [ "$t" = ffi_diff ] || a="$a --test $t"; done
  echo "$a"
}
if [ -n "$CTEST" ]; then
  if [ "$CTEST" = all ]; then CRATES=$(ls crates | sed 's/^ck-//'); else CRATES=$(printf '%s\n' $CTEST | grep -v '^-p$' | sed 's/^ck-//'); fi
  for c in $CRATES; do
    [ -d "crates/ck-$c" ] || { echo "no crate ck-$c"; rec "cargo-test-$c" 1 0; continue; }
    step "cargo-test-$c" nice -n 5 cargo test --no-fail-fast $(cargo_test_args "ck-$c")
  done
fi
flock -u 9; exec 9>&-    # checks are plain node: release the build lock

CH=$(printf '%s' "$CHECKS" | tr ',' ' ')
case " $CH " in *" all "*) CH="gate large sample loft apex fillet section spline thin suite lint text projcurve dfm";; esac
case " $CH " in *" none "*) CH="";; esac
SUITE="region_select thin_extrude thread end_reference profile coincident_boolean hole_dims move spline_shell spline_boolean mate"
pids=""
if [ -s "$W" ]; then
  for c in $CH; do
    case "$c" in
      gate) ( t=$SECONDS; nice -n 5 node tools/gate.mjs --fast --quiet --wasm "$W" > "$RUN/gate.out" 2>&1; echo $? > "$RUN/gate.rc"; echo $((SECONDS-t)) > "$RUN/gate.secs" ) & pids="$pids $!" ;;
      large sample) step large sample node tools/check_large sample.mjs --wasm "$W" ;;
      loft) step loft node tools/loft_point_check.mjs --wasm "$W" ;;
      apex) step apex node tools/loft_apex_check.mjs --wasm "$W" ;;
      fillet) step fillet node tools/concave_fillet_check.mjs --wasm "$W" ;;
      section) step section node tools/section_check.mjs --wasm "$W" ;;
      spline) step spline_shell node tools/spline_shell_check.mjs --wasm "$W"; step spline_boolean node tools/spline_boolean_check.mjs --wasm "$W" ;;
      thin) step thin_extrude node tools/thin_extrude_check.mjs --wasm "$W" ;;
      lint) step lint-protocol node tools/lint_protocol.mjs --wasm "$W" --quiet ;;
      text) step text-corpus node tools/text_corpus.mjs --wasm "$W" --quiet ;;
      projcurve) step project-curve-corpus node tools/project_curve_corpus.mjs --wasm "$W" --quiet ;;
      dfm) step dfm node tools/dfm_check.mjs --wasm "$W" --quiet ;;
      suite) for s in $SUITE; do step "check-$s" node "tools/${s}_check.mjs" --wasm "$W"; done ;;
      *) echo "unknown check $c"; rec "check-$c" 64 0 ;;
    esac
  done
  [ -n "$pids" ] && wait $pids
  if [ -f "$RUN/gate.rc" ]; then
    echo "== gate output"; cat "$RUN/gate.out"
    rec gate "$(cat "$RUN/gate.rc")" "$(cat "$RUN/gate.secs")"; rm -f "$RUN/gate.rc" "$RUN/gate.secs" "$RUN/gate.out"
  fi
else
  echo "no wasm built: checks skipped"; [ "$FAILS" = 0 ] && FAILS=1
fi
echo "box load after: $(cut -d' ' -f1-3 /proc/loadavg)"
echo "REMOTE_FAILS=$FAILS"
[ "$FAILS" = 0 ]
RSCRIPT
# the commit the wasm is built from, reported by `ping` as `build` (`-dirty`: uncommitted changes in the tracked files)
REV=$(git -C "$WT" rev-parse --short=12 HEAD 2>/dev/null || true)
[ -n "$REV" ] && [ -n "$(git -C "$WT" status --porcelain --untracked-files=no 2>/dev/null)" ] && REV="$REV-dirty"
QA=""; for a in "$NAME" "$CHECKS" "$CTEST" "$JOBS" "$NOWASM" "$WASMF" "$REV"; do QA="$QA $(printf %q "$a")"; done
printf '%s\n' "$REMOTE" | rsh "bash -s -- $QA"
RC=$?
[ "$RC" = 255 ] && echo "ssh connection failed or dropped" >&2

# 3. copy back
if [ "$SYNC_BACK" = 1 ]; then
  mkdir -p "$OUT"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci-cad/remote-build/target/$NAME/wasm32-unknown-unknown/release/ck_wasm.wasm" "$OUT/ck_wasm.wasm" 2>/dev/null \
    && echo "wasm -> $OUT/ck_wasm.wasm" || echo "(no wasm to copy back)"
  rsync -aq -e "ssh ${SSHO[*]}" "$H:ci-cad/remote-build/runs/$NAME/remote_results.txt" "$OUT/remote_results.txt" 2>/dev/null
  [ -f "$OUT/remote_results.txt" ] && { echo "--- $OUT/remote_results.txt"; cat "$OUT/remote_results.txt"; }
fi
echo "total $(( $(date +%s) - T0 ))s, exit $RC"
exit "$RC"
