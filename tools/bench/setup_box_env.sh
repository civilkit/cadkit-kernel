#!/usr/bin/env bash
# Idempotent setup of the remote timing environment (one directory, ~/bench-env, nothing system-wide).
#   BENCH_HOST=<ssh host> tools/bench/setup_box_env.sh [--no-wasm]
# Creates on the box: ~/bench-env/{venv,tools,tools/bench,wasm,cq}
#   venv    python venv with pinned cadquery (OCCT through the OCP bindings) -> ~/bench-env/requirements.lock
#   tools   kernel.mjs + the bench scripts and the BenchCAD translations (tools/bench/benchcad/*.ckpart.json)
#   wasm    ours.wasm (release build of this checkout) and orig.wasm (the read-only oracle kernel.wasm)
#   cq      CadQuery programs of the BenchCAD parts (fetched here from the dataset, cc-by-4.0, then copied)
# Needs on the box: python3 >= 3.10 (with venv), node >= 18, rsync, network for pip. Re-running only refreshes the
# copied files and installs what is missing. Build first: cargo build -p ck-wasm --target wasm32-unknown-unknown --release
set -euo pipefail
cd "$(dirname "$0")/../.."
: "${BENCH_HOST:?set BENCH_HOST to the ssh host of the timing box}"
CADQUERY_PIN=${CADQUERY_PIN:-2.8.0}   # the version the dataset ground truth was produced with
ORIG=${ORIG_WASM:-../kernel.wasm}
SSH="ssh -o BatchMode=yes $BENCH_HOST"
RS="rsync -aqL -e 'ssh -o BatchMode=yes'"

$SSH 'bash -s' <<EOF
set -e
command -v node >/dev/null || { echo "node missing"; exit 1; }
command -v rsync >/dev/null || { echo "rsync missing"; exit 1; }
python3 -c 'import sys; assert sys.version_info >= (3, 10), sys.version' || { echo "python3 >= 3.10 needed"; exit 1; }
node -e 'if (+process.versions.node.split(".")[0] < 18) { console.error("node >= 18 needed"); process.exit(1); }'
mkdir -p ~/bench-env/tools/bench ~/bench-env/wasm ~/bench-env/cq
[ -x ~/bench-env/venv/bin/python ] || python3 -m venv ~/bench-env/venv
if ! ~/bench-env/venv/bin/python -c "import cadquery, sys; sys.exit(cadquery.__version__ != '$CADQUERY_PIN')" 2>/dev/null; then
  ~/bench-env/venv/bin/pip install -q --upgrade pip
  ~/bench-env/venv/bin/pip install -q "cadquery==$CADQUERY_PIN"
fi
~/bench-env/venv/bin/pip freeze > ~/bench-env/requirements.lock
echo "python \$(python3 -V), node \$(node -v)"
grep -i -E '^(cadquery|cadquery-ocp|cadquery-ocp-novtk|vtk|numpy)==' ~/bench-env/requirements.lock
EOF

eval "$RS tools/kernel.mjs $BENCH_HOST:bench-env/tools/"
eval "$RS tools/bench/three_way.mjs tools/bench/occt_times.py tools/bench/occt_shim.py $BENCH_HOST:bench-env/tools/bench/"
eval "$RS --exclude=gt tools/bench/benchcad/ $BENCH_HOST:bench-env/tools/bench/benchcad/"
if [ "${1:-}" != "--no-wasm" ]; then
  eval "$RS target/wasm32-unknown-unknown/release/ck_wasm.wasm $BENCH_HOST:bench-env/wasm/ours.wasm"
  eval "$RS $ORIG $BENCH_HOST:bench-env/wasm/orig.wasm"
fi
if [ "$(ls "${CQ_DIR:-/nonexistent}" 2>/dev/null | wc -l)" -gt 0 ]; then
  eval "$RS $CQ_DIR/ $BENCH_HOST:bench-env/cq/"
fi
echo "bench-env ready on the box: $($SSH 'ls ~/bench-env/cq | wc -l') CadQuery programs"

# optional: the brepjs examples (OCCT in wasm) for the kit's brepjs-* samples.
#   BREPJS_EX_DIR=<dir with ex/<id>.mjs, see tools/bench/brepjs_ref/README.txt> tools/bench/setup_box_env.sh --no-wasm
if [ -n "${BREPJS_EX_DIR:-}" ]; then
  $SSH 'mkdir -p ~/bench-env/brepjs && cd ~/bench-env/brepjs && { [ -d node_modules/brepjs ] || npm install --silent --ignore-scripts brepjs@20.5.0 occt-wasm@5.6.0 >/dev/null 2>&1; }; ls node_modules | tr "\n" " "'
  eval "$RS $BREPJS_EX_DIR/ex/ $BENCH_HOST:bench-env/brepjs/ex/"
  eval "$RS tools/bench/brepjs_times.mjs $BENCH_HOST:bench-env/tools/bench/"
  eval "$RS --exclude=gt tools/bench/brepjs/ $BENCH_HOST:bench-env/tools/bench/brepjs/"
fi
