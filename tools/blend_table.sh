#!/usr/bin/env bash
# Run the fillet regression table (crates/ck-model/tests/blend_table.rs) on the CI box and print only its compact
# per-case lines and the summary. See docs/blend-table.md.
#   tools/blend_table.sh [--case X[,Y]] [--mode fail|pass] [--json] [--debug]
#   --json: per-case results -> target/blend-table/blend/table.json; --debug: CK_BLEND_DEBUG output per case ->
#   target/blend-table/blend/debug/<id>.log. The full remote log is kept in target/blend-table/last.log.
set -uo pipefail
ROOT=$(cd "$(dirname "$0")/.." && pwd -P)
OUT="$ROOT/target/blend-table"
ARGS=(--checks none --no-wasm --no-sync-back --cargo-test model --test blend_table --out "$OUT")
FETCH=0; DEBUG=0
while [ $# -gt 0 ]; do
  case "$1" in --case=*|--mode=*) set -- "${1%%=*}" "${1#*=}" "${@:2}" ;; esac
  case "$1" in
    --case) ARGS+=(--env "BLEND_CASE=$2"); shift 2 ;;
    --mode) ARGS+=(--env "BLEND_MODE=$2"); shift 2 ;;
    --json) ARGS+=(--env "BLEND_JSON={RUN}/blend/table.json"); FETCH=1; shift ;;
    --debug) ARGS+=(--env "BLEND_DEBUG_DIR={RUN}/blend/debug"); FETCH=1; DEBUG=1; shift ;;
    -h|--help) sed -n 2,6p "$0" >&2; exit 64 ;;
    *) echo "unknown arg: $1" >&2; sed -n 2,6p "$0" >&2; exit 64 ;;
  esac
done
[ "$FETCH" = 1 ] && ARGS+=(--fetch blend)
mkdir -p "$OUT"
[ "$DEBUG" = 1 ] && rm -f "$OUT"/blend/debug/*.log 2>/dev/null
T0=$(date +%s)
"$ROOT/tools/remote_build.sh" --worktree "$ROOT" "${ARGS[@]}" > "$OUT/last.log" 2>&1
RC=$?
if grep -q '^blend table end' "$OUT/last.log"; then
  sed -n '/^blend table: /,/^blend table end/p' "$OUT/last.log" | grep -v '^blend table end'
else
  echo "blend_table.sh: no table output; the end of the log:"; tail -n 30 "$OUT/last.log"
fi
[ "$FETCH" = 1 ] && [ -d "$OUT/blend" ] && echo "files: $OUT/blend"
echo "blend_table.sh: ${RC/#0/ok} in $(( $(date +%s) - T0 ))s" >&2
exit "$RC"
