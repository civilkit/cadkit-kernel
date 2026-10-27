#!/usr/bin/env bash
# Release wasm build, optionally followed by binaryen's wasm-opt (a size and speed pass over the
# finished module; it never changes floating point results: the volume of a large part stays
# 57733.46939995121, see internal notes (not published) "Round 5").
#
# usage: tools/build_wasm.sh [out.wasm]      default out: target/wasm32-unknown-unknown/release/ck_wasm.opt.wasm
#
# wasm-opt is optional: taken from $WASM_OPT, else from PATH. Without it the plain cargo output is
# copied to the output path (a warning is printed) so the rest of the pipeline works unchanged.
# One way to get it without touching the system (binaryen is Apache-2.0):
#   npm install --prefix <scratch dir> binaryen && WASM_OPT=<scratch dir>/node_modules/.bin/wasm-opt tools/build_wasm.sh
set -euo pipefail
cd "$(dirname "$0")/.."
cargo build -p ck-wasm --target wasm32-unknown-unknown --release
IN="${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/ck_wasm.wasm"
OUT="${1:-${CARGO_TARGET_DIR:-target}/wasm32-unknown-unknown/release/ck_wasm.opt.wasm}"
OPT="${WASM_OPT:-$(command -v wasm-opt || true)}"
if [ -z "$OPT" ] || [ ! -x "$OPT" ]; then
  echo "build_wasm: wasm-opt not found (set WASM_OPT or put it on PATH); using the plain cargo output" >&2
  cp "$IN" "$OUT"
else
  # (only the features rustc emits for this target; --all-features would also enable ones that
  # change the binary encoding, which node 22 rejects)
  "$OPT" -O3 --enable-bulk-memory --enable-sign-ext --enable-nontrapping-float-to-int \
    --enable-mutable-globals --enable-multivalue --enable-reference-types "$IN" -o "$OUT"
fi
echo "build_wasm: $(wc -c < "$OUT") bytes -> $OUT"
