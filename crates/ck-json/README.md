# ck-json

Zero-dependency JSON: DOM (insertion-ordered objects), parser, writer.
The original kernel has NO serde — this is the only JSON layer.

## Interface

- `parse(&str) -> Result<Json, ParseError>`
- `to_string(&Json) -> String`
- `Json` enum: Null/Bool/Num(f64)/Str/Arr/Obj(insertion-ordered Vec pairs)
- Objects preserve key order (matches original output ordering)
- Float output is shortest round-trip; integral floats print without `.0`
  (matches oracle: `"x":20`, `"y":-0.0000000000000001`)

## Tests

- roundtrip + float format tests included; extend against oracle goldens.
- A proven bigger implementation lives in
  civil_engineering/sectionprops-rs/src/json.rs (same hand-rolled style,
  NaN/Infinity tolerant) — port features from there as needed.
