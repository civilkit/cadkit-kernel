# The Fusion 360 decoder as a separate, lazily loaded wasm

Decision: the `.f3d` decoder is not in the main kernel wasm (`ck_wasm.wasm`). It is `ck_f3d.wasm` (crate `crates/ck-wasm-f3d`),
fetched the first time a `.f3d` / `.f3z` / `.smb` is opened. The reply of `importF3d` (docs/f3d.md) does not change.

## Sizes (release, wasm32, Mac build, `cargo build --release`; raw / gzip -9 / brotli -q 9, bytes)

| Module | raw | gzip | brotli |
|---|---|---|---|
| main kernel before (decoder linked: `ck-asm`) | 13,673,940 | 4,509,413 | 3,561,870 |
| main kernel after (`ck_wasm.wasm`) | 13,256,039 | 4,363,034 | 3,448,430 |
| `ck_f3d.wasm`, build before this change (`decodeF3d` + the mapper, no regen) | 4,113,857 | 1,362,454 | 1,098,338 |
| the same after `tools/build_wasm_f3d.sh` (wasm-opt -O3) | 3,337,943 | 1,277,244 | 1,054,059 |
| `ck_f3d.wasm` with `F3D_FEATURES=0` (`decodeF3d` only) | 1,362,936 | 477,902 | 410,300 |
| `ck_f3d.wasm` with the mapper's guard (kernel regen linked in; no longer a build option) | 12,043,964 | 4,030,077 | 3,192,678 |

The main wasm only drops 0.42 MB (3 %) because on this branch it linked only `ck-asm` (the ASM reader and the zip / Zstandard
code); the design crates (`ck-f3d-design` and the others, the +2 MB or so) were never in it and now cannot get in. The decoder
wasm is 1.1 MB over the wire (brotli) with the timeline request, 0.4 MB without. Both modules share the kernel's geometry
crates, so the decoder carries its own copy of what it uses (ck-topo, ck-heal, part of ck-model); that is the price of being
separate. The numbers above are from before the mapper moved (next section): the decoder is now the `decodeF3d` build plus the
design dump (`f3dDump`, the design decoder without the mapper), between the 1.4 MB and 4.1 MB rows, and the main kernel carries
the mapper.

## Why the timeline mapping is in the main kernel

The mapper (`ck-f3d-features`, about 2,100 lines) finds the edges of a fillet, the face a face-profile extrude / press-pull
starts on and the region a point names by regenerating the history written so far and looking in the real body
(`guard::regenerate_body`). That needs the kernel's whole regen (a 12 MB wasm when linked into the decoder), so it cannot be in
the lazy decoder. Rather than leave fillets (43 items of the corpus), face-profile press-pulls and region picking as
placeholders in the decoder, the mapper runs in the main kernel wasm, which has the regen already; the decoder only decodes:

1. `ck_f3d.wasm` `f3dDump` reads the design and answers each design's dump IR as JSON plus the measure (volume, box, lumps) of
   the body the import builds for it;
2. the main kernel's `f3dFeatures {designs}` maps each dump to the ck-part feature list and regenerate-and-compares it with
   that measure (`check.verdict`).

Cost in the main wasm: the mapper (2.1k lines) and the dump IR types with their serde code (2.4k lines), no decoder (the design
decoder stack of 73k lines, the ASM reader and the zip / Zstandard code stay in `ck_f3d.wasm`; the mapper uses only `ck-asm`'s
vector helpers). The decoder stack was +2.75 MB raw (4.11 MB vs 1.36 MB with `F3D_FEATURES=0`), so by lines the mapper and IR are
about 0.2 MB, and with serde's generated code, 0.2 to 0.4 MB raw (about 3 %). This is an estimate: no release wasm was built for this change;
measure `ck_wasm.wasm` with `tools/build_wasm.sh` on the next release build. The mapping quality is the one of the corpus
baseline (`tools/f3d_features_baseline.json`: items mapped 3539, verified 17), since the mapper is unchanged and the dump
arrives by a lossless JSON trip (`a_dump_sent_as_json_maps_the_same` in the synthetic tests).

## Why decoded bodies cross as bytes

The decoder builds `ck-topo` bodies; the session state (the tag's body) lives in the main kernel. The bodies cross in the
lossless binary form of `ck_topo::codec` (`CKBD`, bit-exact, checked on decode: damaged input is an error, never a panic),
as base64 inside the JSON the kernel's ABI already takes. `importF3d` with `decoded` then builds the same reply as a decode in
place (`crates/ck-wasm/tests/lazy_f3d.rs` checks the two replies are identical, byte for byte of JSON).

## The decoder wasm: `ck_f3d.wasm`

Same ABI as the kernel, so `WasmHost` (kernel-client) hosts it unchanged:

- exports: `memory`, `ck_alloc(n) -> ptr`, `ck_request(ptr, len) -> u32`, `ck_reply() -> ptr`, `ck_commit() -> ptr`,
  `ck_commit_len() -> u32` (`"ck <revision>"`, the same build id rule as the kernel);
- imports (module `env`): `ck_panic(ptr, len)`, `ck_now() -> f64`, `ck_unix_ms() -> f64`;
- requests: the kernel's envelope `{"id":n,"msg":{...}}`, replies framed like the kernel's (`{"ok":true,"result":...,"ms":0}`).
  It keeps no state between requests. A trap (panic = abort) re-instantiates in `WasmHost`.

| Request | Reply `result` |
|---|---|
| `{type:"decodeF3d", name?, data:<base64 of the .f3d / .f3z / .smb / .smbh>, opts?:{history?, allBodies?}}` | `{decoded:{solid:<b64>, surfaces:[[id,<b64>],...], info:{...}}}`, or `{errors:[["import",{code,params:{message}}]]}` (`f3d.container`, `f3d.asm`, `f3d.empty`: the file cannot be read) |
| `{type:"f3dDump", name?, data}` | `{designs:[{label, dump, reference, referenceError}]}`: `dump` is the design dump IR (`ck-f3d-dump`, JSON; `null` without a timeline), `reference` the `{volume, min, max, solids}` of the body the import builds for that design (`null` with `referenceError` when none) |
| `{type:"ping"}` | `{pong:true, f3d:true, features:bool}` (`features`: `f3dDump` is available) |

`f3dFeatures` is NOT a decoder request any more: see "The main kernel" below.

`info` is exactly the `import` object of docs/f3d.md (format, name, container, unitScale, blobs, bodies, solids, surfaces,
refused, faces, facesTotal, parts, skippedFaces, report).

## The main kernel: `importF3d`

```
{type:"importF3d", tag, name?, decoded:{solid, surfaces, info}}          -> the reply of docs/f3d.md (body, surfaces, warnings, import)
{type:"importF3d", tag, name?, data:<base64>, opts?}                     -> decoded in place only by a build with cargo feature `f3d`
```

```
{type:"f3dFeatures", designs:[{label, dump, reference?, referenceError?}]}   -> {designs:[{label, part, mapped, placeholders, items[], notes[], check}]}
```

`designs` is the decoder's `f3dDump` reply as it is (a design with `dump: null` is left out). `part` is a `ck-part` document
(`{format,version,features,seq}`), `items[]` one entry per timeline item (`index, name, type, status: mapped|placeholder,
features[], note?`), `check` the guard's `{verdict: verified|unverified, reference, history, volumeDelta, boxDelta, reason, errors}`:
the history is the design's only when `verified`. Read only, no tag. The same request works in a native service and in tests.

The main wasm is built without `f3d`: `data` answers `errors:[["import",{code:"f3d.decoderMissing",...}]]` and leaves the tag
alone. The native service (`ck-native`) is built with `f3d` and still decodes `data` itself. `ck-server` accepts a request with
`data` or `decoded` (neither: the usual "is required" error).

## The loader contract: `packages/kernel-client/src/f3d-decoder.ts`

```ts
configureF3dDecoder(source)       // once at startup; nothing is fetched. source: URL string | URL | bytes | WebAssembly.Module | () => Promise<those | Response>
loadF3dDecoder(source?)           // first call fetches + compiles (a failure is not cached), later calls return the same decoder
decoder.decode(bytes, {name, history, allBodies}) // -> {decoded} | {errors}   (synchronous, runs in the calling thread)
decoder.dump(bytes, {name})       // -> {designs:[{label, dump, reference, referenceError}]} | {errors}   (the f3dDump request)
new F3dDecoderMissingError(...)   // code 'f3d.decoderMissing': no source configured, fetch failed / non-2xx, bad module
```

`@ck/session` gets it wired already: `Session.importF3d(base64, name, opts)` (and `importF3dLazy(kernel, bytes, opts)` in
`queries.ts`) load the decoder, decode, and send `importF3d {decoded}` to the kernel; the reply is the same `ImportReport`.
Without a decoder it first tries the kernel with `data` (works on a native service) and then throws `F3dDecoderMissingError`
(the app shows its message). `Session.f3dFeatures(base64, name)` (and `f3dFeatures(kernel, bytes, {name})` in `queries.ts`) runs `dump` on the decoder and sends
the designs to the kernel's `f3dFeatures`; it returns the reply above, or throws `F3dDecoderMissingError` without a decoder. So the kit's `importF3dBytes` needs only a `configureF3dDecoder(<url of ck_f3d.wasm>)` at startup.
`decode` runs on the calling thread: a big file blocks it for the decode time. The kit can host `ck_f3d.wasm` inside its kernel
worker instead by implementing the same `F3dDecoder` interface over `WasmHost` there (the message shapes above are all it needs).

The timeline history is not trusted on its own: `f3dFeatures` regenerates each `part` in the kernel and compares volume and box with
the import's `reference` (volume within 2e-3 relative, box sides within 0.01 mm + 1e-4 of the diagonal); the app shows the history
as the design's only when `check.verdict` is `verified`. The reference is measured in the decoder from the file, the same body the
import hands over as `decoded.solid`.

## Build and files

- Kernel: `tools/build_wasm.sh` (unchanged) -> `ck_wasm.wasm`.
- Decoder (the wasm ABI exports need cargo feature `abi`, which the script enables; without it ck-wasm-f3d is a plain library, so native builds of ck-wasm with `f3d` do not export a second `ck_*` ABI): `tools/build_wasm_f3d.sh [out]` -> `target/wasm32-unknown-unknown/release/ck_f3d.opt.wasm`, served as `ck_f3d.wasm`
  (`F3D_FEATURES=0` for the bodies-only build, without `f3dDump`). The cargo output is `ck_wasm_f3d.wasm`.
- `ck_f3d.wasm` must come from the same kernel commit as `ck_wasm.wasm` (the body codec has a version, `CKBD` v1; a mismatch is
  a decode error, not a wrong body). Compare the last token of `decoder.commit` with the kernel's `ck_commit`.
- Test: `node --test` of `packages/kernel-client/src/f3d-decoder.test.ts` (needs both wasms; `CK_F3D_WASM` overrides the decoder
  path), and `cargo test -p ck-wasm --features f3d --test lazy_f3d`.
