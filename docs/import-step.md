# importStep: a STEP body held by the part

## Problem

A part could only be built from sketches and operations. A file converted from another CAD system often starts from a solid that exists only as a STEP body, followed by sketches and operations that act on it. Without a feature that holds such a body, the importer had to drop it and everything after it had nothing to act on.

## Decision

A feature `importStep` carries STEP text and yields its solids as bodies when the part is regenerated.

- Where the text lives: a top-level `blobs` table in the part, `{ "<sha-256 hex>": "<STEP text>" }`, and the feature names a key (`blob`). Inline `step` text is also accepted for small files. A table was chosen over inline text because the regen cache key hashes the JSON of each feature on every request; a 50 MB file inline would be hashed every time. With a key the feature adds 64 characters to the hash.
- Cache soundness: the table is not part of any cache key, only the key the feature names is. So the kernel checks, each time the feature runs, that the key is the SHA-256 of the text. A cached state therefore always belongs to the data its key names. Hashing only happens when the feature runs, not on a cache hit.
- Text, not base64: STEP is text, and JSON stores it directly. The file keeps its size apart from escapes. Parts that hold large blobs should be stored compressed by whatever container the application uses.
- Reader: the existing `ck-io` STEP reader (the one behind the `importStep` protocol message). `ck-model` now depends on `ck-io`; there is no cycle. Units are converted by the reader.
- Naming: faces are `<feature id>.I<solid>F<n>` from file order. The reader numbers them deterministically, and the names are rebuilt from the file on every run, so they survive a save and a load. Edges have no names of their own and are picked by their faces like everywhere else.
- Several solids: each solid is a lump; `body` names them `<body>`, `<body> 2`, ... The solids are added with the separate-bodies rule, so a feature in the middle of a history does not unite them with existing bodies.
- Errors: `importStepInvalid` with `detail` (and `reason`, `entity` when the reader gives them). Truncated or hostile text returns this error; the tests cut a real file at many places.
- Healing is a choice of the feature, `heal: true`; without it the body is the reader's result after the lumps are rebuilt, as before. The protocol message heals by default; a feature that changed geometry silently would be surprising inside a history, so the part says so. A converter that restates a part's geometry as STEP can write some faces turned the other way: read as it is, the solid is open or has a wrong volume (a cube with one face turned reads at two thirds of its volume), healed it is the solid of the source. `heal: true` runs the message's repair (`ck-heal`, all fixers, tolerant edges) after reading and before the lumps are rebuilt again; the flag is part of the feature, so it is part of the regen cache key. Leave it off for the STEP of the program that made the part.

## Parts that are not solids, and Repair Part (2026-10-31)

An import never keeps a mesh or a half-built solid.

- The STEP and SolidWorks readers leave out a face they cannot build (with its reason: code, face id, message, and the surface it lay on when that could be read) instead of failing the whole file; the file is refused only when nothing could be read. STEP surface models (`SHELL_BASED_SURFACE_MODEL`) are read as parts too.
- Each part is then checked on its own (`ck_heal::honest::sort`, validator at 1e-6): a valid solid joins the solid body; any other part with faces becomes a surface body of exactly those faces; a part with none is refused. Every part carries its reasons in plain words.
- `importStep` feature: surface bodies are `<id>.P<k>` (k = the part's place in the file) and the feature carries the warning `importStepParts {detail, solids, surfaces, refused, parts}` (the "needs repair" mark). The `importStep`, `importSldprt` and `importSkp` messages reply `import.parts` the same way (STEP / SolidWorks: the valid solids as `body`, the others in `surfaces`; SketchUp keeps every piece in the body and only reports which pieces are solids). `step` on an import tag writes its surface bodies too.
- `repairPart {target, gap = 0.2, patchHoles = true}` (modelled on SolidCraft's Repair Part, PR #8676): rebuilds the left-out faces on their own surface, sews free edges within `gap` (at most 1 mm), fills flat holes, and reports per surface body what it did (`repairReport`). A part that comes out valid joins the solid body. Curved holes are reported, not filled (no curved patch yet). Refusals: `repairTarget`, `repairNothing`, `repairGap`.
- SolidWorks solids get the tolerance-only pass (`ck_heal::tolerant_only`) before they are judged, as a healed STEP import does: edges that follow their faces only to the file's accuracy get a tolerance, nothing else changes.

## Not done

- Importing only some solids of a file, and moving or rotating the body at import: use `moveBody` after the feature.
- Removing unused blobs: left to editors.
