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
if ! git merge -q --no-edit main; then
  # auto-resolve the shared index files; anything else is a real conflict
  for F in $(git diff --name-only --diff-filter=U); do
    case "$F" in
      tools/funcmap/rust_map.json)
        git show :2:"$F" > "$F.ours"; git show :3:"$F" > "$F.theirs"
        python3 -I "$MAIN/tools/funcmap/jsonunion.py" x "$F.ours" "$F.theirs" && mv "$F.ours" "$F"; rm -f "$F.theirs"; git add "$F" ;;
      tools/funcmap/layouts.md|*README.md)
        git show :2:"$F" > "$F.a"; git show :3:"$F" > "$F.b"; (git show :1:"$F" 2>/dev/null || :) > "$F.o"
        git merge-file --union "$F.a" "$F.o" "$F.b"; mv "$F.a" "$F"; rm -f "$F.o" "$F.b"; git add "$F" ;;
      tools/scoreboard.json)
        git checkout --theirs "$F" && git add "$F" ;;   # baseline follows main
      *) echo "land: real conflict in $F — resolve in $WT and rerun"; exit 1 ;;
    esac
  done
  git commit -q --no-edit
fi
DEL=$(git diff --name-only --diff-filter=D main HEAD -- '*.test.*' 'crates/*/tests/*')
[ -z "$DEL" ] || [ -n "$ALLOW_TEST_DELETE" ] || { echo "land: branch deletes test files (set ALLOW_TEST_DELETE=1 if intended):"; echo "$DEL"; exit 1; }
cargo test -q --workspace --profile fast 2>&1 | grep -E "^test result|FAILED|panicked" | grep -v " 0 failed" | grep . && { echo "land: tests failed"; exit 1; } || true
cargo build -q -p ck-wasm --target wasm32-unknown-unknown --release
npm --prefix packages ci --silent 2>&1 | tail -2
npm --prefix packages test --silent 2>&1 | tail -8 | grep -q "# fail 0" || { echo "land: packages failed"; exit 1; }
node tools/region_select_check.mjs >/dev/null || { echo "land: region select check failed"; exit 1; }
node tools/thread_check.mjs >/dev/null || { echo "land: thread check failed"; exit 1; }
node tools/coincident_boolean_check.mjs >/dev/null || { echo "land: coincident boolean check failed"; exit 1; }
node tools/lint_protocol.mjs --quiet || { echo "land: protocol lint failed"; exit 1; }
node tools/gate.mjs --fast --quiet | tail -1 | grep -q "gate ok" || { node tools/gate.mjs --fast --quiet | grep REGRESSION; echo "land: gate failed"; exit 1; }
git -C "$MAIN" merge --ff-only "$BR"
echo "landed $BR -> main @ $(git -C "$MAIN" rev-parse --short HEAD)"
