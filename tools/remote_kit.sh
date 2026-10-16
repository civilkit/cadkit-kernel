#!/usr/bin/env bash
# Run the KIT (GUI) vitest suite on the CI box. See docs/remote-build.md.
#   tools/remote_kit.sh [--kit DIR] [--files PATTERN ...] [--wasm PATH] [--kernel DIR] [--tsc] [--out DIR] [--clean]
#     --kit DIR      the kit worktree (default /path/to/work/kit)
#     --files PAT    vitest file filter(s), e.g. apps/cad/src/controller (repeatable; default: all of apps packages)
#     --wasm PATH    kernel wasm to test against (uploaded; e.g. the one tools/remote_build.sh copied back).
#                    Default: the kernel-main artifact the box CI keeps.
#     --kernel DIR   a kernel worktree whose packages/ the kit imports (synced and built on the box);
#                    default: the box CI's checkout of kernel main (read only).
#     --tsc          also run `tsc --noEmit`
#     --clean        remove this kit worktree's directories on the box
# Same host rules as tools/remote_build.sh. Everything lives under ~/ci-cad/remote-build.
set -uo pipefail
KIT=/path/to/work/kit; FILES=(); WASM=""; KERNEL=""; TSC=0; OUT=""; CLEAN=0
while [ $# -gt 0 ]; do
  case "$1" in
    --kit) KIT="$2"; shift 2 ;;
    --files) FILES+=("$2"); shift 2 ;;
    --wasm) WASM="$2"; shift 2 ;;
    --kernel) KERNEL="$2"; shift 2 ;;
    --tsc) TSC=1; shift ;;
    --out) OUT="$2"; shift 2 ;;
    --clean) CLEAN=1; shift ;;
    *) sed -n 2,12p "$0" >&2; exit 64 ;;
  esac
done
KIT=$(cd "$KIT" && pwd -P) || exit 64
[ -f "$KIT/package.json" ] && [ -d "$KIT/apps" ] || { echo "not a kit worktree: $KIT" >&2; exit 64; }
H=${REMOTE_BUILD_HOST:-}
if [ -z "$H" ]; then
  U=$(git -C "$KIT" remote get-url ci 2>/dev/null) || { echo "set REMOTE_BUILD_HOST or add the 'ci' git remote" >&2; exit 64; }
  H=${U%%:*}
fi
NAME="kit-$(basename "$KIT" | tr -c 'A-Za-z0-9._\n-' '_')-$(printf '%s' "$KIT" | cksum | cut -d' ' -f1)"
SSHO=(-o BatchMode=yes -o ConnectTimeout=20 -o ServerAliveInterval=30)
RB='$HOME/ci-cad/remote-build'
QUIET='post-quantum|store now|upgraded|openssh.com/pq'
rsh() { ssh "${SSHO[@]}" "$H" "$@" 2> >(grep -Ev "$QUIET" >&2); }
if [ "$CLEAN" = 1 ]; then rsh "N=$(printf %q "$NAME"); case \"\$N\" in ''|*/*|.*) exit 2;; esac; rm -rf -- $RB/wt/\"\$N\" $RB/runs/\"\$N\"; echo removed \"\$N\""; exit $?; fi
RS=(rsync -a -e "ssh ${SSHO[*]}")
T0=$(date +%s)
rsh "mkdir -p $RB/wt/$NAME $RB/runs/$NAME $RB/kernel-src" || exit 3
"${RS[@]}" --exclude=node_modules --exclude=.git --exclude=.claude --exclude=dist --exclude='*.log' "$KIT"/ "$H:ci-cad/remote-build/wt/$NAME/" 2> >(grep -Ev "$QUIET" >&2) || { echo "rsync failed" >&2; exit 3; }
KSYNC=0
if [ -n "$KERNEL" ]; then
  KERNEL=$(cd "$KERNEL" && pwd -P) || exit 64
  rsh "mkdir -p $RB/kernel-src/$NAME" || exit 3
  "${RS[@]}" --exclude=/target --exclude='/target-*' --exclude=.git --exclude=node_modules --exclude=.claude --exclude=/crates --exclude=/packages/dist \
    "$KERNEL"/packages "$KERNEL"/docs "$KERNEL"/tools "$H:ci-cad/remote-build/kernel-src/$NAME/" 2> >(grep -Ev "$QUIET" >&2) || exit 3
  KSYNC=1
fi
WREM=""
if [ -n "$WASM" ]; then
  [ -s "$WASM" ] || { echo "no such wasm: $WASM" >&2; exit 64; }
  "${RS[@]}" "$WASM" "$H:ci-cad/remote-build/runs/$NAME/kernel.wasm" 2> >(grep -Ev "$QUIET" >&2) || exit 3
  WREM=1
fi
echo "synced in $(( $(date +%s) - T0 ))s"

read -r -d '' REMOTE <<'RSCRIPT'
set -u
NAME=$1; KSYNC=$2; WREM=$3; TSC=$4; shift 4
RB="$HOME/ci-cad/remote-build"
case "$NAME" in ''|*/*|.*) echo "bad name"; exit 2;; esac
SRC="$RB/wt/$NAME"; RUN="$RB/runs/$NAME"; R="$RUN/remote_results.txt"; : > "$R"
export PATH="$HOME/ci-cad/cargo/bin:$PATH" CI=1
free=$(df -BG --output=avail "$RB" | tail -1 | tr -dc '0-9'); [ "${free:-0}" -ge 15 ] || { echo "FAIL: less than 15G free"; exit 1; }
echo "box: load=$(cut -d' ' -f1-3 /proc/loadavg) free=${free}G"
cd "$SRC" || exit 2
FAILS=0
rec() { if [ "$2" = 0 ]; then echo "ok $1 $3" >> "$R"; echo "== ok   $1 (${3}s)"; else echo "FAIL $1 $3 rc=$2" >> "$R"; echo "== FAIL $1 (${3}s, rc=$2)"; FAILS=$((FAILS+1)); fi; }
step() { local n=$1; shift; local t=$SECONDS rc; echo "== $n"; "$@"; rc=$?; rec "$n" $rc $((SECONDS-t)); return 0; }
# node_modules: installed once per lockfile hash (the box lockfile may be out of sync: fall back to npm install)
h=$(sha256sum package-lock.json | cut -c1-16)
if [ ! -d node_modules ] || [ "$(cat node_modules/.rb-lock-hash 2>/dev/null)" != "$h" ]; then
  step npm-install bash -c 'npm ci --no-audit --no-fund --silent >/dev/null 2>&1 || npm install --no-audit --no-fund --silent'
  echo "$h" > node_modules/.rb-lock-hash
fi
# kernel sources the kit imports (packages/): own synced copy, else the box CI's checkout of kernel main (read only)
if [ "$KSYNC" = 1 ]; then
  K="$RB/kernel-src/$NAME"
  if [ ! -d "$K/packages/node_modules" ]; then step kernel-packages bash -c "cd '$K/packages' && npm ci --no-audit --no-fund --silent && npx tsc -p tsconfig.build.json"; fi
  [ -d "$K/packages/dist" ] || step kernel-packages-build bash -c "cd '$K/packages' && npx tsc -p tsconfig.build.json"
else
  K="$HOME/ci-cad/work/kernel-src"; echo "kernel packages: box CI checkout of kernel main (read only)"
fi
DEV=/path/to/work/rebuild
grep -rIl "$DEV" tsconfig.json vite.config.ts packages apps --include='*.ts' --include='*.tsx' --include='*.json' 2>/dev/null | grep -v node_modules | while read -r f; do sed -i "s|$DEV|$K|g" "$f"; done
if [ -n "$WREM" ]; then WASMP="$RUN/kernel.wasm"; else WASMP="$HOME/ci-cad/artifacts/kernel-main.wasm"; fi
[ -s "$WASMP" ] || { echo "no kernel wasm for CK_WASM"; exit 1; }
echo "CK_WASM: $(if [ -n "$WREM" ]; then echo uploaded; else echo box CI kernel-main artifact; fi)"
[ "$TSC" = 1 ] && step tsc nice -n 5 npx tsc --noEmit -p tsconfig.json
step vitest env CK_REBUILD="$K" CK_WASM="$WASMP" nice -n 5 npx vitest run "$@" --exclude '**/node_modules/**' --reporter=default --poolOptions.forks.maxForks=8 --poolOptions.forks.minForks=1
echo "box load after: $(cut -d' ' -f1-3 /proc/loadavg)"
[ "$FAILS" = 0 ]
RSCRIPT
if [ "${#FILES[@]}" = 0 ]; then FILES=(apps packages); fi
QA=""; for a in "$NAME" "$KSYNC" "$WREM" "$TSC" "${FILES[@]}"; do QA="$QA $(printf %q "$a")"; done
printf '%s\n' "$REMOTE" | rsh "bash -s -- $QA"
RC=$?
if [ -n "$OUT" ]; then mkdir -p "$OUT"; rsync -aq -e "ssh ${SSHO[*]}" "$H:ci-cad/remote-build/runs/$NAME/remote_results.txt" "$OUT/remote_kit_results.txt" 2>/dev/null; fi
echo "total $(( $(date +%s) - T0 ))s, exit $RC"
exit "$RC"
