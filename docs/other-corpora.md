# Other kernels' corpora and reference kernels

Research only. No kernel changes, no translators built, no data committed.

## 1. NVIDIA USD BRep

* Official repo: `https://github.com/NVIDIA-Omniverse/usd-brep`
  (org `NVIDIA-Omniverse`, created 2026-10-08).
* Licence: **Apache-2.0**, verified in the repo's `LICENSE` (copyright
  2020-2026 NVIDIA CORPORATION & AFFILIATES).
* Size: about **20 MB** (GitHub `size` 19,738 KB). Git LFS is used for some
  binary fixtures; a plain clone still gets the code and test scripts.
* What it is: NURBS B-rep kernel (SMLib/NLib), C++ and Python APIs, OpenUSD
  import/export, booleans, fillets/chamfers, offset, stitching, intersectors,
  mass properties. It exposes `_omni_solid` and the supported `usd_brep`
  package.

### Tests shipped

| location | format | size / groups |
|---|---|---|
| `tests/prog_test/` | C++ custom harness | 13 source files, **18 suites**: topology, booleans, offset, fillets, local-ops, sweeps, primitives, tessellation, unit, trimmed-surfaces, brep-import, ssi-analytic, ssi-advanced, cci-advanced, section-advanced, silhouette-advanced, stitch |
| `tests/SM_API_test/` | C++ custom harness | 18 source files (`TestSmBrep`, `TestSmFillets`, `TestSmIntersectors`, `TestSmHeal`, `TestSmImportExport`, ...) |
| `tests/usd_test/` | C++ CLI + `.usda` files | regression / `--assert-valid` import checks |
| `tests/python/`, `source/SmPyLib/tests/` | pytest/unittest | 7 files: round-trips, package API, heal, schema registration, examples |
| `TestFiles/` | data | **593 files**: 200 `.smb`, 134 `.usda`, 74 `.smp`, 50 `.dat`, 30 `.brep`, 20 `.fdf`, 18 `.stl`, ... |
| `TestFilesSmb/` | data | 5 `.smb` primitives |

Expected values: the tests assert **topology counts** (faces/edges/vertices),
**validity** (`AssertValid`), operation success, and sometimes volume
equality/deltas (e.g. `TestSmBrep.cpp` checks a poly-boolean volume is
unchanged to 1e-9) or positive/finite face area. They are **not** a corpus of
analytic volume/area expectations for each modeling operation. The `.smb`
and `.usda` fixtures are golden shapes, not numeric goldens.

### Python bindings here

* PyPI `usd-brep` 1.0.0 ships CPython 3.12 wheels for **Linux x86_64,
  Linux aarch64 and Windows x86_64 only**. There is **no macOS wheel**, and
  the GitHub release assets are also Linux/Windows only.
* So it cannot be installed here with `pip`. Running it on this Mac needs a
  source build (Apple Silicon only; Packman pulls OpenUSD, pybind11 and
  premake; Python-only build is the default). That is a large C++ build and
  was **not** attempted; ask before spending the time.
* On a Linux CI box with Python 3.12: `pip install usd-brep` is enough
  (the wheel bundles its OpenUSD runtime). A 5-line smoke test would be:

  ```python
  import usd_brep
  box = usd_brep.primitives.create_box((-0.5,-0.5,-0.5),1,1,1)
  cyl = usd_brep.primitives.create_cylinder((0,0,0),0.25,1)
  fused = usd_brep.booleans.union(box, cyl)
  print(usd_brep.queries.volume(fused))   # exact call name to confirm from the API docs
  ```

  (The package is real; confirm the query helper name against
  `docs/python.rst` before running.)

### Could it be a second reference kernel?

Yes in principle: replay the same translated recipes (primitives,
transforms, booleans, fillets) and compare `volume`/`area` with OCCT and ours.
Cost is the blocker on macOS (source build). On Linux CI the wheel makes it
cheap. The 20-case cross-check (10 cases where OCCT and we disagree) is
deferred until either the CI box is approved or a macOS build is approved.


> **Licence gate:** the Fusion 360 Gallery data is **non-commercial research
> only**, must not be committed, redistributed, or used as a release/CI gate
> for a commercial product. It currently lives only in
> `~/.cache/b-rep-corpora/fusion360/` and is **not** in git. If the kernel may
> be sold, drop this sample and use synthetic, NIST (public domain) or OCCT
> self-generated geometry instead.

## 2. Public B-rep corpora

| corpus | licence / terms | size | format | sample |
|---|---|---|---|---|
| **ABC** (NYU) | Copyright owned by the creators; use subject to Onshape Terms of Use 1.g.ii; research terms | 1M models; STEP chunks are **0.8-1.6 GB per 10,000 models**; 100 chunks/format | STEP, Parasolid, STL, OBJ, features, meta | No per-file index; a 200-file sample means one chunk or scraping the hosted individual files. **Not fetched.** |
| **Fusion 360 Gallery** (Autodesk) | Dataset licence: **non-commercial research only**; no redistribution of the full set; portions only with attribution and the same restrictions | Reconstruction 2.0 GB; segmentation 3.1 GB; **extended STEP 483 MB (42,912 STEP files)** | `.f3d`, `.step/.stp`, `.json`, `.seg` | Sampled from the 483 MB extended STEP zip; see result below. Files kept in `~/.cache/b-rep-corpora/fusion360/sample`, not in git. |
| **DeepCAD** | No repository licence file; data parsed from Onshape public documents via ABC, so Onshape/ABC terms apply | `data.tar` **208 MB**; ~? models | JSON construction sequences (`cad_json`, `cad_vec`), not STEP; `export2step.py` can emit STEP via OpenCASCADE | Good for sketch-extrude sequence translation, not a direct STEP import corpus. |
| **NIST CAD models** | US Government work / public domain (we already use FTC-11 etc.) | small | STEP | Already in `tools/step_nist_reference.json` / NIST tests. |
| **OCCT bug attachments** | GitHub issue attachments; GitHub ToS, no separate dataset licence; not a curated corpus | per-file | STEP/IGES/BREP | Useful one-off repros; not a random corpus. |

### Fusion 360 sample result

The extended STEP zip was downloaded (483 MB, 1m46s), 200 `.stp` files were
sampled with a fixed seed, the zip was deleted, and the 200 files (22 MB)
remain in `~/.cache/b-rep-corpora/fusion360/sample`. Through our kernel's
`importStep` + `validate`:

| class | count |
|---|---|
| ok | **185** |
| invalid | **15** |
| import-failed | **0** |

Invalid codes (some files carry two): `TESSELLATION_OPEN` 12,
`EDGE_OFF_SURFACE` 6, `EULER` 1, `LOOP_SELF_INTERSECT` 1. This is an import /
display-mesh robustness signal, separate from the modeling corpus. The tool
is `tools/import_sample_check.mjs`.

### Import regression check (opt-in, research-only)

`tools/import_sample_check.mjs` reads a manifest of fixed ids and classifies
each file as `ok` / `invalid` / `import-failed` / `missing`. `--check`
compares against a baseline; `--update-baseline` rewrites it. Baselines:
`tools/import_baseline_1234.json`, `tools/import_baseline_5678.json`
(185 ok / 15 invalid each). The sample ids are in
`tools/fusion_sample_1234.json` / `_5678.json`; `tools/fusion_fetch.sh`
recreates only those files in `~/.cache`. Failure triage is in
`docs/mimo-brief-import.md`.

## 3. Other kernels

| project | licence | test corpus? |
|---|---|---|
| **truck** (`ricosjp/truck`, Rust) | Apache-2.0 | 111 test files, mostly inline unit/integration shapes; no external B-rep corpus. |
| **Manifold** (`elalish/manifold`) | Apache-2.0 | 154 test files, unit tests and small meshes; mesh boolean, not B-rep. |
| **Fornjot** (`hannobraun/fornjot`) | NOASSERTION (check repo) | 5 test files, early unit tests; no corpus. |
| **CGAL** (`CGAL/cgal`) | NOASSERTION (LGPL/GPL components) | 5,348 test files and 274 `Data/` files plus per-package `test/*/data`; geometry test data (OFF/PLY/etc.), not a B-rep solid corpus. |

## Recommendation, in order

1. **Fusion 360 extended-STEP sample (done):** cheapest real-world import
   robustness signal. 15/200 invalid is actionable, especially
   `TESSELLATION_OPEN` and `EDGE_OFF_SURFACE`. Keep it as a separate
   `import` corpus, not the modeling corpus. Terms: non-commercial research;
   never commit the files.
2. **USD BRep as a second reference on the Linux CI box:** `pip install
   usd-brep` + replay selected recipes. Low setup cost there; no macOS path
   without a source build. This answers “which kernel is right where OCCT and
   we disagree” for booleans/fillets.
3. **DeepCAD 200 sketch-extrude sequences:** 208 MB, JSON; translate to our
   wire and compare volume with OCCT. Medium effort; no STEP import needed.
4. **ABC:** defer. The per-chunk size (0.8-1.6 GB) and lack of a per-file
   index make a 200-file sample expensive and awkward.
5. **NIST / OCCT bug attachments:** keep as targeted repros, not a
   systematic corpus.
6. **truck / Manifold / Fornjot / CGAL:** not useful as external B-rep
   corpora; their tests are inline/unit-level.

No translators were started. `tools/import_sample_check.mjs` is the only
tool added; the sample data lives outside git.
