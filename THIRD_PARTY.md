# Third-party code

Vendored or adapted code and its licence. Only permissively licensed sources.

| Path | Source | Licence |
|---|---|---|
| crates/ck-check/src/{shell,wire,edge,solid}.rs, LICENSE-MIT, LICENSE-APACHE | brepkit v2.129.15, crates/check/src/validate/ (check logic adapted onto ck-topo; no brepkit code linked) | MIT OR Apache-2.0 |
| crates/ck-heal/src/{vertex,small,wire,sew,orient}.rs, LICENSE-MIT, LICENSE-APACHE | brepkit v2.129.15, crates/heal/src/{fix/wire,fix/face,fix/shell,fix/solid,fix/small_face,fix/split_vertex,fix/wireframe,analysis/wire_order,upgrade/shell_sewing}.rs (fix catalogue and ordering adapted onto ck-topo / ck-geom; detection via ck-check; no brepkit code linked) | MIT OR Apache-2.0 |
| crates/ck-io/src/step/{parse,read}.rs, crates/ck-io/third_party/brepkit/LICENSE-MIT, LICENSE-APACHE | brepkit v2.129.15, crates/io/src/step/reader.rs (STEP reader: entity walk, surface/curve entity set, knot-multiplicity and rational-weight decoding, adapted onto ck-topo / ck-geom; no brepkit code linked) | MIT OR Apache-2.0 |
| crates/ck-io/src/sat/{mod,parse,build}.rs | the author's own `sat_import.rs` in SGRebuild (`fem-core`, dependency-free ACIS 7.00 text parser; reused freely). Record parser/tokenizer seed; this module builds the real B-rep (ck-topo / ck-geom) | same author (SGRebuild) |
| crates/ck-tess/src/cdt.rs | the author's own zero-dep mesher in sectionprops-rs (`src/mesher/cdt.rs` + `predicates.rs`; Lawson flips, Sloan segment recovery, Shewchuk-style adaptive predicates) | MIT (sectionprops-rs) |
| tools/bench/brepjs/{*.ckpart.json,manifest.json,gt/*.step}, tools/bench/gen4.mjs | brepjs playground examples, category "mechanical" (https://github.com/andymai/brepjs, apps/playground/src/lib/examples/mechanical.ts). Used as test parts and references only: each example was translated by hand into our feature tree (parameters paraphrased, no source text copied); reference volume / area / bbox / centre of mass / STEP come from running the unmodified examples with brepjs 20.5.0. Attribution: the brepjs project (Andy Aragon) | Apache-2.0 |
| crates/ck-io/tests/fixtures/nist_ftc_11.step (7.6 KB), tools/step_nist_reference.json (OCCT-computed volumes / areas of the NIST PMI reference STEP files; the other files are not committed) | NIST MBE PMI Validation and Conformance Testing Project, FTC-11 geometry-only STEP (nist.gov, "NIST PMI STEP files"). Used as a test input and reference only | US Government work, public domain |
| tools/build_wasm.sh (optional step, not vendored or linked) | binaryen `wasm-opt` (https://github.com/WebAssembly/binaryen), run over the finished wasm when present; installed per developer (for example `npm install binaryen` into a scratch directory), not part of the repository | Apache-2.0 |
| crates/ck-sldprt/src/{xt,container,ir,build}.rs, crates/ck-sldprt/{NOTICE,NOTICE-sldprt2step,LICENSE-APACHE} | BlinkingSun/sldprt2step (port of Jozo132/open-sld-to-step): XT neutral-binary reader structure and SCH_13006 base field tables (`xt/reader.py`, `xt/schema.py`), GUID-wrapped transmit blobs and OLE2 stream walk (`container/*.py`), XT node to surface / curve / topology conventions (`step/convert.py`, `step/mapper.py`); rewritten in Rust onto ck-topo / ck-geom | Apache-2.0 |
| crates/ck-sldprt/src/container.rs (format facts), crates/ck-sldprt/LICENSE-MIT-sldprt-format-research | blussyya/sldprt-format-research: modern block container layout, CRC and version detection notes (no code copied) | MIT |
| crates/ck-sldprt/src/container.rs (format facts) | cadmpeg/cadmpeg `docs/formats/sldprt.md`: block frame and cache-cell description (facts only, no code copied; attribution as CC-BY-4.0 requires) | CC-BY-4.0 (docs), Apache-2.0 (code) |
| crates/ck-sldprt/tests/fixtures/*.sldprt (4 files, 260 KB), SOURCES.txt | blussyya/sldprt-research-dump, folder "test files new": SW2022_C00 cube, SW2022_C04 cube with hole, SW2022_C13 cone, SW2011_C00 cube (OLE2); copied unchanged | MIT |
| crates/ck-text/fonts/gauge.ttf, crates/ck-text/fonts/OFL-BarlowCondensed.txt | Barlow Condensed Medium, Copyright 2017 The Barlow Project Authors (the jpt/barlow project); subset to Basic Latin + Latin-1 + a few punctuation marks with fontTools (unhinted, layout tables and name table dropped), renamed `gauge.ttf`. Baked into the wasm with `include_bytes!` (docs/text.md) | SIL Open Font License 1.1 |
| crates/ck-text/fonts/engrave.ttf, crates/ck-text/fonts/OFL-PTSerif.txt | PT Serif Regular, Copyright 2010 ParaType Ltd. (reserved font names "PT Sans", "PT Serif", "ParaType"); same subset, renamed `engrave.ttf` so the reserved name is not used by the modified font. Baked into the wasm | SIL Open Font License 1.1 |
| crates/ck-text/src/ttf.rs | written from the OpenType / TrueType and Adobe CFF / Type 2 charstring specifications (head, maxp, hhea, hmtx, loca, glyf, cmap, name, OS/2, CFF); no third-party code, no crate dependency | n/a |
| crates/ck-text/src/sha256.rs | SHA-256 per FIPS 180-4 (moved from the regen cache; the author's own) | n/a |
| crates/ck-text/tests/fixtures/fx-*.ttf, fx-sans-source-otf.otf (11 files, 3 to 19 KB each), the OFL-*.txt / LICENSE-Apache-2.0.txt next to them | Subsets (the letters and digits of the tests, `pyftsubset`, unhinted, name table dropped) of the curated fonts listed below plus Inter (variable-font test); test inputs only, not compiled into the wasm. The name tables are dropped so no reserved font name is carried by a modified font | SIL Open Font License 1.1 (all but the two Apache 2.0 ones, see the table) |

## Curated fonts (hosted by the kit, not in this repository)

Fonts the kit may host and hand to `loadFont` (docs/text.md, "Loading fonts"). The kernel repository holds none of
these files; only the tiny test subsets above. Files were taken from the Google Fonts repository
(github.com/google/fonts, commit 47d54ca) except Source Sans 3 (github.com/adobe-fonts/source-sans, branch `release`,
`OTF/SourceSans3-Regular.otf`, commit 87b37a2). Each licence text must travel with the hosted file. Only SIL OFL 1.1 and
Apache 2.0 fonts are listed; no AGPL / GPL fonts.

| Role | Font (file) | Copyright | Licence | Upstream | sha256 of the file checked |
|---|---|---|---|---|---|
| sans | Barlow Regular (`ofl/barlow/Barlow-Regular.ttf`) | Copyright 2017 The Barlow Project Authors | SIL OFL 1.1 | https://github.com/jpt/barlow | 95aa02c7c43096e0dd44d787ba6216864a67157e402adab59b35572e0c1577ea |
| sans, humanist (OpenType / CFF) | Source Sans 3 Regular (`OTF/SourceSans3-Regular.otf`) | (c) 2023 Adobe, Reserved Font Name "Source" | SIL OFL 1.1 | https://github.com/adobe-fonts/source-sans | 08df266400933d3178d081a45f94a08814c3e55b4b7dd2e0ff69cb1329f13ab6 |
| condensed | Barlow Condensed Medium (`ofl/barlowcondensed/BarlowCondensed-Medium.ttf`; the built-in `gauge` is a subset of it) | Copyright 2017 The Barlow Project Authors | SIL OFL 1.1 | https://github.com/jpt/barlow | 262bd143292ce479ee0cd09a42b47ab173fca8e9c6eb5ed0b5c8a845bc371d17 |
| condensed capitals, display | Bebas Neue Regular (`ofl/bebasneue/BebasNeue-Regular.ttf`) | Copyright 2019 The Bebas Neue Project Authors | SIL OFL 1.1 | https://github.com/dharmatype/Bebas-Neue | 08e4623805102d819f58601e46e345648846075e363b2ceb23313c2d1c83ec73 |
| serif | PT Serif Regular (`ofl/ptserif/PT_Serif-Web-Regular.ttf`; the built-in `engrave` is a subset of it) | Copyright (c) 2010 ParaType Ltd., Reserved Font Names "PT Sans", "PT Serif", "ParaType" | SIL OFL 1.1 | https://www.paratype.com/fonts/pt/pt-serif | a4951fade06ff8f09b7673aa81ffb65a8cd409e24d3289a6dc670bc4dda2557a |
| slab / typewriter | Special Elite Regular (`apache/specialelite/SpecialElite-Regular.ttf`) | Copyright (c) 2010 Brian J. Bonislawsky DBA Astigmatic (AOETI) | Apache 2.0 | https://fonts.google.com/specimen/Special+Elite | a776fcb4ceb8bdf03e2967688ebdad42680de5b91a7e62c17e718ae212d14bc4 |
| mono | IBM Plex Mono Regular (`ofl/ibmplexmono/IBMPlexMono-Regular.ttf`) | Copyright 2017 IBM Corp., Reserved Font Name "Plex" | SIL OFL 1.1 | https://github.com/IBM/plex | 6a3412f058c7d8dfd9170c41e85ade48e5156ecb89356110ca57a0a27734af46 |
| script / nameplate | Great Vibes Regular (`ofl/greatvibes/GreatVibes-Regular.ttf`) | Copyright 2010 The Great Vibes Pro Project Authors | SIL OFL 1.1 | https://github.com/googlefonts/great-vibes | 8d509802186f1b51572531ecf313e8098f9a5bfdfaca93f0c9b34467f9982d15 |
| script (brush) | Satisfy Regular (`apache/satisfy/Satisfy-Regular.ttf`) | Copyright (c) 2011 Font Diner, Inc. | Apache 2.0 | https://fonts.google.com/specimen/Satisfy | 55bb141a0a23a32278d5fa7fd2e853824faa173b94774660ebeddb106be07c80 |
| bold display | Anton Regular (`ofl/anton/Anton-Regular.ttf`) | Copyright 2020 The Anton Project Authors | SIL OFL 1.1 | https://github.com/googlefonts/AntonFont | a4ba3a92350ebb031da0cb47630ac49eb265082ca1bc0450442f4a83ab947cab |

Also read without problems in the exploratory run (whole files, not part of the list): Cousine, Fira Sans, Lato, Tinos,
Pacifico, Inter (variable), Source Code Pro (variable). Their licences are checked per font before any of them is hosted.
