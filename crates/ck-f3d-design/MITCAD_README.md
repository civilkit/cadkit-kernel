# mitcad-f3d-design: the design segment of `.f3d` files

`core/f3d-design` builds the design segment of an `.f3d` document (the
object database with the timeline, parameters, sketches, features,
components and occurrences) into the dump IR of
[SCHEMA.md](../import/SCHEMA.md) (`mitcad-dump`) from the streams that
[`mitcad-f3d-decode`](../f3d-decode/README.md) decodes, and finds the
feature inputs it names in the ASM history of the body blobs
(`mitcad-asm`, `mitcad-f3d-names`):

| Module | Contents |
|---|---|
| `build` | The decoded data shaped as the dump IR. |
| `inputs` | Feature inputs named as the bodies name their entities, found in the bodies' history. |
| `appearance` | Appearances and materials of bodies and faces (mitcad#228). |
| `derive_sources` | Where derives' placements without a matrix are (mitcad#324). |
| `package` | What an `.f3z` package says about its designs (mitcad#229). |
| `stream`, `classes`, `decode`, `sketch`, `recipe`, `curve`, `learn`, `writer` | Re-exported from `mitcad-f3d-decode` (mitcad#779) under their old paths. |

The crate root has the entry points (`decode_path`, `decode_document`,
`decode_streams`, `resolve_inputs`, ...). Its tests (`src/tests.rs`) run
synthetic design streams through the decoding and the building. The
corpus comparisons with the reference decoder are `mitcad-f3d`'s tests
(`core/f3d/tests/`). Until mitcad#776 this was `mitcad_f3d::design`; that
path still works (`mitcad-f3d` re-exports it).
