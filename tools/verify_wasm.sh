#!/bin/sh
# One-command verifier: tools/verify_wasm.sh <wasm> [--quick]
here=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec node "$here/verify_wasm.mjs" "$@"
