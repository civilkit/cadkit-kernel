#!/usr/bin/env bash
# Time ours (+ the original kernel) on one of the manifests under the box's ~/timing.lock, copy the json back.
#   BENCH_HOST=<ssh host> tools/bench/box_kernels.sh <wasm> <out.json> [--set benchcad|brepjs] [--only id,id] [--no-orig]
# Refreshes ~/bench-env/wasm/ours.wasm from <wasm>, waits for the lock (polling), always releases it.
# The OCCT numbers are not re-measured here (reuse the table of the last full `three_way.mjs --remote` run).
set -euo pipefail
: "${BENCH_HOST:?set BENCH_HOST to the ssh host of the timing box}"
WASM=$1; OUT=$2; shift 2
SET=benchcad; ONLY=""; ORIG="--orig wasm/orig.wasm"
while [ $# -gt 0 ]; do
  case $1 in
    --set) SET=$2; shift 2;;
    --only) ONLY="--only $2"; shift 2;;
    --no-orig) ORIG="--orig wasm/ours.wasm"; shift;;
    *) echo "unknown arg $1"; exit 2;;
  esac
done
SSH="ssh -o BatchMode=yes $BENCH_HOST"
rsync -aqL -e 'ssh -o BatchMode=yes' "$WASM" "$BENCH_HOST:bench-env/wasm/ours.wasm"
rsync -aqL -e 'ssh -o BatchMode=yes' "$(dirname "$0")/three_way.mjs" "$BENCH_HOST:bench-env/tools/bench/"
until $SSH 'mkdir ~/timing.lock' 2>/dev/null; do echo "timing lock busy, waiting 30 s" >&2; sleep 30; done
trap '$SSH "rmdir ~/timing.lock"' EXIT
$SSH "cd ~/bench-env && echo load before: \$(cut -d' ' -f1-3 /proc/loadavg) && CK_SET=$SET node tools/bench/three_way.mjs run --ours wasm/ours.wasm $ORIG --runs 5 $ONLY --timeout 300 --out k_$SET.json" >&2
$SSH "cat ~/bench-env/k_$SET.json" > "$OUT"
