#!/usr/bin/env bash
# Release build of the separate, lazily loaded Fusion 360 decoder wasm (crates/ck-wasm-f3d, docs/f3d-lazy.md), then
# binaryen's wasm-opt like tools/build_wasm.sh (optional; never changes floating point results).
#
# usage: tools/build_wasm_f3d.sh [out.wasm]      default out: target/wasm32-unknown-unknown/release/ck_f3d.opt.wasm
# The file is served to the app as `ck_f3d.wasm`. F3D_FEATURES=0 builds it without the timeline (`f3dDump`) request.
set -euo pipefail
cd "$(dirname "$0")/.."
FLAGS=()
[ "${F3D_FEATURES:-1}" = "0" ] && FLAGS=(--no-default-features)
cargo build --locked -p ck-wasm-f3d --features abi ${FLAGS[@]+"${FLAGS[@]}"} --target wasm32-unknown-unknown --release
IN="${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/ck_wasm_f3d.wasm"
OUT="${1:-${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/ck_f3d.opt.wasm}"
OPT="${WASM_OPT:-$(command -v wasm-opt || true)}"
if [ -z "$OPT" ] || [ ! -x "$OPT" ]; then
  echo "build_wasm_f3d: wasm-opt not found (set WASM_OPT or put it on PATH); using the plain cargo output" >&2
  cp "$IN" "$OUT"
else
  "$OPT" -O3 --enable-bulk-memory --enable-sign-ext --enable-nontrapping-float-to-int \
    --enable-mutable-globals --enable-multivalue --enable-reference-types "$IN" -o "$OUT"
fi
echo "build_wasm_f3d: $(wc -c < "$OUT") bytes -> $OUT"
