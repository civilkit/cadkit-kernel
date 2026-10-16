#!/usr/bin/env bash
# One command to make the whole stack current: TypeScript packages, kernel wasm and the copies the
# frontend uses.
#
# usage: tools/sync_stack.sh [--wasm] [--kit <kit dir>]
#   (default)  compile packages/, report whether the wasm is older than the Rust sources, copy the wasm
#              into the kit (and kit-film when present)
#   --wasm     also rebuild the kernel wasm on the CI box (tools/remote_build.sh --checks large sample) first
set -euo pipefail
cd "$(dirname "$0")/.."
ROOT="$PWD"
KIT="$ROOT/../kit"
REBUILD_WASM=0
while [ $# -gt 0 ]; do
  case "$1" in
    --wasm) REBUILD_WASM=1 ;;
    --kit) KIT="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

WASM="$ROOT/target/wasm32-unknown-unknown/release/ck_wasm.wasm"

if [ "$REBUILD_WASM" = 1 ]; then
  tools/remote_build.sh --checks large sample
fi

echo "== packages: tsc"
npm --prefix packages run build >/dev/null

# Is the wasm older than the last commit (or edit) touching the Rust sources?
if [ ! -f "$WASM" ]; then
  echo "== wasm missing: run tools/sync_stack.sh --wasm" >&2
  exit 1
fi
NEWEST_RS=$(find crates -name '*.rs' -o -name Cargo.toml | xargs stat -f %m 2>/dev/null | sort -n | tail -1)
WASM_TIME=$(stat -f %m "$WASM")
if [ "$WASM_TIME" -lt "$NEWEST_RS" ]; then
  echo "== WARNING: wasm is older than the Rust sources; run tools/sync_stack.sh --wasm" >&2
  STALE=1
else
  STALE=0
fi

for dest in "$KIT/apps/cad/public/kernel" "$ROOT/../kit-film/apps/cad/public/kernel"; do
  if [ -d "$dest" ]; then
    cp "$WASM" "$dest/ck_wasm.wasm"
    echo "== copied wasm -> $dest"
  fi
done

echo "== stack: kernel $(git rev-parse --short HEAD), wasm $(date -r "$WASM" '+%F %T'), $(wc -c < "$WASM") bytes"
exit $STALE
