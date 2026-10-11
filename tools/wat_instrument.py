import sys
src=open('../kernel.wat').read()
imp='  (import "env" "ck_unix_ms" (func $env.ck_unix_ms (type $t20)))\n'
assert imp in src
src=src.replace(imp, imp+'  (import "env" "ck_log" (func $env.ck_log (param i32 f64 f64 f64 f64)))\n',1)
# f3283
h='  (func $f3283 (type $t144) (param $p0 i32) (param $p1 f64) (param $p2 f64) (param $p3 i32) (param $p4 i32) (param $p5 f64) (param $p6 i32) (param $p7 i32)\n    (local $l8 i32) (local $l9 f64)\n'
assert h in src
src=src.replace(h, h+'    i32.const 1\n    local.get $p1\n    local.get $p2\n    local.get $p3\n    local.get $p4\n    call $f3906\n    local.get $p5\n    call $env.ck_log\n',1)
i=src.index('  (func $f3575 (type $t152)')
j=src.index('      block $B3\n', i)
src=src[:j]+'      i32.const 2\n      local.get $p1\n      local.get $p2\n      local.get $l7\n      local.get $p3\n      call $env.ck_log\n'+src[j:]
open('k_inst.wat','w').write(src)
