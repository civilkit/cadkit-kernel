# ck-wasm

The wasm ABI boundary — cdylib exporting exactly 5 functions.
No wasm-bindgen (the original used std + dlmalloc + panic=abort).

## ABI (matches kernelworker.js + oracle.mjs)

```text
ck_alloc(n: usize) -> ptr        // room for the request body
ck_request(ptr, len) -> u32      // run one message; reply length
ck_reply() -> ptr                // reply start
ck_commit() -> ptr               // build string ptr
ck_commit_len() -> u32           // build string len ("class-a-surfacing da3c6f49...")
imports: ck_panic(ptr, len)      // panic message capture (host stores it)
         ck_now() -> f64         // performance.now
         ck_unix_ms() -> f64     // Date.now
```

## Reply layout

```
[u32 LE json_len][json bytes][pad to 8-byte alignment][buffers...]
```

Buffers are referenced in JSON as {"$buf":i,"type":"f32|f64|u32|i32|u8",
"len":elements,"offset":bytes-from-aligned-base}. See root README.

## Host-side reference

kernelworker.js (exact working client) and rebuild/oracle.mjs (harness).
On panic: ck_panic receives ptr/len; host resets by re-instantiating the
module ("the kernel ran out of stack" handling).
