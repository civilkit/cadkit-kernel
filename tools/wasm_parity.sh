#!/usr/bin/env bash
# Wasm parity of the fillet regression table on the CI box: builds the wasm, runs the native table
# (crates/ck-model/tests/blend_table.rs, BLEND_JSON) and then every case through the wasm (tools/wasm_parity.mjs), and
# prints only the differing cases, traps and failed faces (one line each) plus the counts. See docs/wasm-parity.md.
#   tools/wasm_parity.sh [--case X[,Y]] [--wasm-only] [--musl-libm]
#   --case: the table's filter (=id, ^prefix, substring; comma = or); --wasm-only: skip the native table (no comparison);
#   --musl-libm: the native table runs with the wasm's libm (tools/libm_shim), leaving only non-libm differences.
#   Per-case JSON -> target/wasm-parity/parity/parity.json; the full remote log -> target/wasm-parity/last.log.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd -P)
OUT="$ROOT/target/wasm-parity"
ARGS=(--checks wasmparity --no-sync-back --out "$OUT" --fetch parity --env "BLEND_JSON={RUN}/blend/table.json")
NATIVE=1
while [ $# -gt 0 ]; do
  case "$1" in --case=*) set -- "${1%%=*}" "${1#*=}" "${@:2}" ;; esac
  case "$1" in
    --case) ARGS+=(--env "BLEND_CASE=$2"); shift 2 ;;
    --wasm-only) NATIVE=0; shift ;;
    --musl-libm) ARGS+=(--musl-libm); shift ;;
    -h|--help) sed -n 2,8p "$0" >&2; exit 64 ;;
    *) echo "unknown arg: $1" >&2; sed -n 2,8p "$0" >&2; exit 64 ;;
  esac
done
[ "$NATIVE" = 1 ] && ARGS+=(--cargo-test model --test blend_table)
mkdir -p "$OUT"
T0=$(date +%s)
"$ROOT/tools/remote_build.sh" --worktree "$ROOT" "${ARGS[@]}" > "$OUT/last.log" 2>&1
RC=$?
if grep -q '^wasm parity end' "$OUT/last.log"; then
  sed -n '/^wasm parity:$/,/^wasm parity end/p' "$OUT/last.log" | grep -v '^wasm parity\(:\| end\)$'
  grep -q '^blend table end' "$OUT/last.log" || [ "$NATIVE" = 0 ] || echo "wasm_parity.sh: the native table did not finish; nothing was compared"
else
  echo "wasm_parity.sh: no parity output; the end of the log:"; tail -n 30 "$OUT/last.log"
fi
[ -f "$OUT/parity/parity.json" ] && echo "json: $OUT/parity/parity.json"
echo "wasm_parity.sh: ${RC/#0/ok} in $(( $(date +%s) - T0 ))s" >&2
exit "$RC"
