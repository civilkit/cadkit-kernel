#!/bin/sh
# Land a worktree branch onto main: merge main into the branch (inside its
# worktree), build, run tests + the gate there, then fast-forward main.
# usage: tools/land.sh <worktree-path>     (refuses on any test/gate failure)
set -e
WT=$(cd "$1" && pwd)
MAIN=$(git -C "$WT" worktree list | head -1 | cut -d' ' -f1)
BR=$(git -C "$WT" rev-parse --abbrev-ref HEAD)
[ -z "$(git -C "$WT" status --porcelain --untracked-files=no)" ] || { echo "land: $WT has uncommitted changes"; exit 1; }
cd "$WT"
git merge -q --no-edit main
cargo test -q --workspace --profile fast 2>&1 | grep -E "^test result|FAILED|panicked" | grep -v " 0 failed" | grep . && { echo "land: tests failed"; exit 1; } || true
cargo build -q -p ck-wasm --target wasm32-unknown-unknown --release
node tools/gate.mjs --fast --quiet | tail -1 | grep -q "gate ok" || { node tools/gate.mjs --fast --quiet | grep REGRESSION; echo "land: gate failed"; exit 1; }
git -C "$MAIN" merge --ff-only "$BR"
echo "landed $BR -> main @ $(git -C "$MAIN" rev-parse --short HEAD)"
