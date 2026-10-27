# Build the kernel

This chapter shows how to build the kernel as a WebAssembly module, how to build the native HTTP service that serves the same protocol, and which Cargo profiles and features exist.

## Requirements

- A Rust toolchain with the `wasm32-unknown-unknown` target (`rustup target add wasm32-unknown-unknown`). The repository does not pin a toolchain version; it uses the 2021 edition.
- Node.js, to run the tools, the TypeScript packages and the examples in this book. The examples in this book were run with Node 22.
- Optional: `wasm-opt` from binaryen, for a size and speed pass over the finished module.

The workspace is the `crates/*` directory. It has no third-party Rust dependencies: `Cargo.lock` lists only the workspace crates (the JSON parser and writer is the workspace crate `ck-json`), so a build needs no network access.

## Build the wasm module

```sh
cargo build -p ck-wasm --target wasm32-unknown-unknown --release
```

The module is written to

```text
target/wasm32-unknown-unknown/release/ck_wasm.wasm
```

It is about 8 MB. The module exports `memory` and five functions and imports three functions from the module `env`; [Call it from Node](node.md) lists them and gives a complete host.

### Profiles

The workspace `Cargo.toml` defines three profiles that matter here.

| Profile | Used for | Settings |
|---|---|---|
| `release` | the shipped wasm | `opt-level = 3`, `lto = true`, `codegen-units = 1`, `panic = "abort"` |
| `fast` | iteration and the test runs | inherits `release`, but `lto = false`, `codegen-units = 16`, incremental builds, `panic = "unwind"` |
| `service` | the native service | inherits `release`, but `panic = "unwind"` |

`codegen-units = 1` gives the same results as more units (the optimiser does not reassociate floating point) and a smaller module. A `fast` build is much quicker to produce; its output is at `target/wasm32-unknown-unknown/fast/ck_wasm.wasm`:

```sh
cargo build -p ck-wasm --target wasm32-unknown-unknown --profile fast
```

With `panic = "abort"` a kernel panic ends the call with a wasm trap. The host reports the panic text through the `ck_panic` import first and then has to create a fresh instance, because the old one is unusable ([Call it from Node](node.md) shows this).

### Features

| Feature | Crate | Effect |
|---|---|---|
| `skp` | `ck-wasm` | Links the SketchUp reader (`ck-skp`) and enables the `importSkp` request. Off by default: it adds about 320 KB (4.6 %) to the module. Without it, `importSkp` answers an error that begins `TODO(rebuild)`. |
| `sanity-shadow` | `ck-wasm`, `ck-ops` | Runs the exact tier of the boolean sanity check even where the cheap tier already passed, and asserts that both agree. A checking mode for development, not for the shipped module. |

```sh
cargo build -p ck-wasm --target wasm32-unknown-unknown --release --features skp
```

[SketchUp](../io/skp.md) describes the import.

### Build identity

`ck_commit()` returns a fixed build string compiled into the module. The source revision is reported differently: when the environment variable `CK_BUILD_REV` is set at compile time, the `ping` reply carries it as `result.build`.

```sh
CK_BUILD_REV=$(git rev-parse --short=12 HEAD) \
  cargo build -p ck-wasm --target wasm32-unknown-unknown --release
```

Without the variable the `ping` result is just `{"pong":true}`. 

### Optional size pass

`tools/build_wasm.sh` runs the release build and then `wasm-opt -O3`:

```sh
tools/build_wasm.sh                 # writes target/wasm32-unknown-unknown/release/ck_wasm.opt.wasm
tools/build_wasm.sh out/kernel.wasm # choose the output path
```

`wasm-opt` is taken from `$WASM_OPT`, else from `PATH`. If it is not found, the script prints a warning and copies the plain `cargo` output to the output path, so later steps keep working. The pass enables only the wasm features that `rustc` emits for this target (bulk memory, sign extension, non-trapping float-to-int, mutable globals, multivalue, reference types). The repository documents that the pass does not change floating point results. There is no separate `strip` step: the workspace does not set `strip`, and `wasm-opt` removes unused code.

## Build the native service

`ck-native` is an HTTP service on `127.0.0.1` that runs the same dispatcher (`ck_server::handle`) as the wasm module, one worker thread per session.

```sh
cargo build --profile service -p ck-native
target/service/ck-native --port 7461
```

Use the `service` profile. The `release` profile aborts on a panic, which would end every session; the service profile unwinds, so the service can answer `500` and close only the session that panicked. `cargo build -p ck-native` (debug) also works but is slow.

`ck-native` links `ck-wasm` without features, so a native service built this way has no `importSkp`. The service takes no third-party dependencies; its flags, token handling, pairing and HTTP shape are described in [The native service](../protocol/native-service.md), and the transport is summarised in [Call it from Node](node.md).

Numbers from the native build can differ from the wasm build in the last digits of some floating point values, because the native build uses the system maths library and the wasm build uses the one compiled into the module. The repository's comparison script reports the 280 protocol cases it runs as identical once `ms` and `at` are normalised, with the binary mesh buffers compared separately, and the two large documents it regenerates as equivalent to a relative difference of at most 1e-6.

## Tests and the regression run

```sh
cargo test --workspace            # unit and integration tests; runs from a bare checkout
```

The corpus and check scripts in `tools/` (`tools/*_check.mjs`, `tools/*_corpus.mjs`, `tools/*_sweep.mjs`) take `--wasm <path>` to choose the module and run it under Node. Some need extra sample inputs; each script's header says what it reads. `tools/gate.mjs` is different: it compares the built module with a second kernel module that is placed next to the checkout as `kernel.wasm` and is not distributed with the repository, so it cannot be run from a bare checkout. The book's examples need only the built module and Node. [Testing](../quality/testing.md) describes what the test and regression runs cover.

Environment variables named `CK_*` select inputs for the tools; `CK_WASM` is the path of the module that the TypeScript packages and tests load.

## See also

- [Call it from Node](node.md)
- [A first part](first-part.md)
- [Testing](../quality/testing.md)
- [Crate map](../reference/crates.md)
