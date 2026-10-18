#!/bin/sh
# Fetch the OCCT test scripts used by tools/occt_corpus.mjs.
#
#   tools/occt_fetch.sh [DIR]        default: /tmp/occt
#
# Sparse, blob-filtered clone: tests/{boolean,blend,draft,chamfer,offset,sewing,
# heal,bugs} (~60 MB). The corpus runner reads these scripts; the translated
# geometry and edge/face picks are what live in this repo. The separate OCCT
# test-data download (CSF_TestDataPath) is not public and is not fetched.
set -eu

DIR=${1:-/tmp/occt}
REPO=https://github.com/Open-Cascade-SAS/OCCT.git

if [ -d "$DIR/tests/boolean" ]; then
  echo "occt_fetch: $DIR already has the tests"
  exit 0
fi

mkdir -p "$DIR"
if [ ! -d "$DIR/.git" ]; then
  git -C "$DIR" init -q
  git -C "$DIR" remote add origin "$REPO"
fi
mkdir -p "$DIR/.git/info"
git -C "$DIR" config core.sparseCheckout true
printf 'tests/boolean/\ntests/blend/\ntests/draft/\ntests/chamfer/\ntests/offset/\ntests/sewing/\ntests/heal/\ntests/bugs/\n' > "$DIR/.git/info/sparse-checkout"
git -C "$DIR" fetch --depth 1 --filter=blob:none origin master
git -C "$DIR" checkout -q FETCH_HEAD
echo "occt_fetch: $(find "$DIR/tests" -type f | wc -l | tr -d ' ') scripts in $DIR/tests"
