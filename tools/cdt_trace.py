#!/usr/bin/env python3
"""Instrument the ORIGINAL kernel.wat to log the CDT operations of ck-tess.

usage: python3 tools/cdt_trace.py [out.wasm]      (default: /tmp-like scratch path given as argv[1])
Then:  node tools/runlog.mjs out.wasm part.json   (prints 'id a b c d' per call)

Log ids (a b c d = the first four params as numbers):
  1 f3305 insert_in_tri(cdt, tri, vid)         2 f3295 insert_on_edge(cdt, tri, k, vid)
  3 f3309 flip(out, cdt, tri, k)               4 f3303 locate(out, cdt, x, y)
  5 f3278 find_edge(out, cdt, u, v)            6 f3307 insert_segment(cdt, a, b)
  7 f3306 mark_segment(cdt, a, b)              9 f3325 face mesher entry
 10 f3299 classify(out? , cdt, ...)
"""
import re, subprocess, sys

import os
SRC = os.environ.get('KERNEL_WAT', '../kernel.wat')
out = sys.argv[1] if len(sys.argv) > 1 else 'k_cdt.wasm'
FUNCS = {1: 3305, 2: 3295, 3: 3309, 4: 3303, 5: 3278, 6: 3307, 7: 3306, 9: 3325}

src = open(SRC).read()
imp = '  (import "env" "ck_unix_ms" (func $env.ck_unix_ms (type $t20)))\n'
assert imp in src
src = src.replace(imp, imp + '  (import "env" "ck_log" (func $env.ck_log (param i32 f64 f64 f64 f64)))\n', 1)

for lid, fn in FUNCS.items():
    m = re.search(r'  \(func \$f%d \(type \$t\d+\)((?: \(param \$p\d+ (?:i32|f64|i64)\))*)\n' % fn, src)
    assert m, fn
    params = re.findall(r'\(param \$(p\d+) (i32|f64|i64)\)', m.group(1))
    pos = m.end()
    # skip local declarations
    while True:
        nl = src.index('\n', pos)
        line = src[pos:nl]
        if line.strip().startswith('(local'):
            pos = nl + 1
        else:
            break
    ins = '    i32.const %d\n' % lid
    for k in range(4):
        if k < len(params):
            name, ty = params[k]
            ins += '    local.get $%s\n' % name
            if ty == 'i32':
                ins += '    f64.convert_i32_u\n'
            elif ty == 'i64':
                ins += '    f64.convert_i64_u\n'
        else:
            ins += '    f64.const 0\n'
    ins += '    call $env.ck_log\n'
    src = src[:pos] + ins + src[pos:]

# id 20: f3325 entry, the face info struct (param 3): uv bounds doubles at +0x20..+0x38
m = re.search(r'  \(func \$f3325 \(type \$t\d+\)((?: \(param \$p\d+ (?:i32|f64|i64)\))*)\n', src)
pos = m.end()
while src[pos:src.index('\n', pos)].strip().startswith('(local'):
    pos = src.index('\n', pos) + 1
# (the id 9 call inserted above comes first; place the bounds log right after it)
assert src[pos:].startswith('    i32.const 9\n')
pos = src.index('    call $env.ck_log\n', pos) + len('    call $env.ck_log\n')
ins = '    i32.const 20\n' + ''.join('    local.get $p3\n    f64.load offset=%d\n' % o for o in (32, 40, 48, 56)) + '    call $env.ck_log\n'
src = src[:pos] + ins + src[pos:]

wat = out + '.wat'
open(wat, 'w').write(src)
subprocess.check_call(['wat2wasm', '--no-check', wat, '-o', out])
os.remove(wat)
print('wrote', out)
