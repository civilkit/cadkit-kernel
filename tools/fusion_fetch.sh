#!/bin/sh
# Fetch one fixed 200-file Fusion 360 Gallery STEP sample.
#
#   tools/fusion_fetch.sh 1234 [DEST]
#   tools/fusion_fetch.sh 5678 [DEST]
#
# NON-COMMERCIAL RESEARCH ONLY. The dataset licence forbids redistribution;
# this script keeps the files in ~/.cache and never writes them into git.
# It downloads the 483 MB extended-STEP zip, extracts only the ids listed in
# tools/fusion_sample_<set>.json, and deletes the zip.
set -eu

SET=${1:-1234}
DEST=${2:-$HOME/.cache/b-rep-corpora/fusion360/sample$([ "$SET" = 5678 ] && echo 2 || echo)}
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
MANIFEST="$HERE/fusion_sample_$SET.json"
CACHE=${XDG_CACHE_HOME:-$HOME/.cache}/b-rep-corpora/fusion360
ZIP="$CACHE/s2.0.1_extended_step.zip"
URL=https://fusion-360-gallery-dataset.s3.us-west-2.amazonaws.com/segmentation/s2.0.1/s2.0.1_extended_step.zip

if [ ! -f "$MANIFEST" ]; then echo "fusion_fetch: no $MANIFEST" >&2; exit 2; fi
mkdir -p "$CACHE" "$DEST"
NEED=$(python3 -c "import json;print(len(json.load(open('$MANIFEST'))['ids']))")
HAVE=$(find "$DEST" -maxdepth 1 -type f -name '*.stp' | wc -l | tr -d ' ')
if [ "$HAVE" -ge "$NEED" ]; then echo "fusion_fetch: $DEST already has $HAVE files"; exit 0; fi

echo "fusion_fetch: downloading 483 MB extended-STEP archive"
curl -sSL --max-time 900 -o "$ZIP" "$URL"
python3 - "$ZIP" "$MANIFEST" "$DEST" <<'PY'
import json, os, sys, zipfile
zip_path, manifest_path, dest = sys.argv[1:4]
ids = set(json.load(open(manifest_path))["ids"])
z = zipfile.ZipFile(zip_path)
by_name = {n.rsplit("/", 1)[-1]: n for n in z.namelist() if n.endswith(".stp")}
missing = ids - set(by_name)
if missing:
    raise SystemExit(f"missing {len(missing)} ids in archive, e.g. {sorted(missing)[:3]}")
for name in sorted(ids):
    with open(os.path.join(dest, name), "wb") as fh:
        fh.write(z.read(by_name[name]))
z.close()
print(f"extracted {len(ids)} files to {dest}")
PY
rm -f "$ZIP"
