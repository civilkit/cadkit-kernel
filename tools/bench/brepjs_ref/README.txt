Reference-run scripts for tools/bench/brepjs (scratch layout used for the figures in manifest.json):
- a clone of github.com/andymai/brepjs in /private/tmp/claude-501/brepjs-src (untrusted data; extract.mjs only reads it as text)
- a separate dir /private/tmp/claude-501/brepjs-run with `npm install --ignore-scripts brepjs@20.5.0 occt-wasm@5.6.0 sucrase`
Copy these four files into the second dir and run: node extract.mjs && node runall.mjs
(writes out/<id>.json with volume, area, bbox, centre of mass and out/<id>.step; overlap.mjs <id> compares the sum of
an example's solids with their union). Paths are hard-coded to that layout.
