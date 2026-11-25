# mitcad-dump: the design dump IR

`core/dump` holds the design dump IR, the JSON of
[SCHEMA.md](../import/SCHEMA.md), as Rust types (`Dump`, `TimelineItem`,
the feature details, the occurrence tree, appearances): the `.f3d` reader's
stream decoder (`core/f3d`) and the `.ipt` reader (`core/ipt`) write it,
the import (`core/import`) replays it. Records are lenient (`record.rs`):
every key is optional and unmodelled keys are kept, so a dump survives a
read-write round trip. `state` interprets an item's result number and state
bytes (mitcad#180).

It was `mitcad_f3d::design::ir` until mitcad#776; that path still works
(`mitcad-f3d` re-exports this crate as `design::ir`).
