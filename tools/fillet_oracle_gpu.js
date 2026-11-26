// Optional WebGPU path of the fillet oracle viewer (inlined into the viewer by fillet_oracle_build_viewer.mjs). No dependencies.
// The CPU code (fillet_oracle_core.mjs) is the reference; this reproduces rasterise() (mask + exact distance to the outline) and
// distToEroded() (seed mask, two squared EDTs with nearest-seed tracking, edge-point refinement) in f32 WGSL.
// Buffers: segs (ro), Dg, ins, fA, fB, scratch, arg = 7 storage buffers (the default limit is 8 per stage).
const GPU = { state: 'untested', msg: 'not checked yet', info: null, adapterName: '', dev: null, lost: false, gridCounter: 0, pipe: null, bgl: null, res: null, disabledBy: '' };

const gpuWGSL = () => {
  const edt = (name, SRC, DST, LL, MM, ARGOFF) => `
@compute @workgroup_size(64)
fn ${name}(@builtin(global_invocation_id) g: vec3<u32>) {
  let L: u32 = ${LL}; let M: u32 = ${MM}; let x = g.x; if (x >= L) { return; }
  let zo: u32 = u.zoff;
  var k: i32 = -1;
  for (var q: u32 = 0u; q < M; q = q + 1u) {
    let fq = ${SRC}[q * L + x];
    if (fq >= 1e19) { continue; }
    if (k < 0) { k = 0; scratch[x] = q; scratch[zo + x] = bitcast<u32>(-BIG); scratch[zo + L + x] = bitcast<u32>(BIG); continue; }
    var s: f32 = 0.0;
    loop {
      let vq = scratch[u32(k) * L + x];
      let fp = ${SRC}[vq * L + x];
      let a = f32(q); let b = f32(vq);
      s = ((fq + a * a) - (fp + b * b)) / (2.0 * (a - b));
      if (s <= bitcast<f32>(scratch[zo + u32(k) * L + x])) { k = k - 1; if (k < 0) { break; } } else { break; }
    }
    k = k + 1; scratch[u32(k) * L + x] = q; scratch[zo + u32(k) * L + x] = bitcast<u32>(s); scratch[zo + u32(k + 1) * L + x] = bitcast<u32>(BIG);
  }
  if (k < 0) {
    for (var q: u32 = 0u; q < M; q = q + 1u) { ${DST}[x * M + q] = 1e30; arg[${ARGOFF} + x * M + q] = 0xFFFFFFFFu; }
    return;
  }
  var kk: u32 = 0u;
  for (var q: u32 = 0u; q < M; q = q + 1u) {
    let fq = f32(q);
    loop { if (bitcast<f32>(scratch[zo + (kk + 1u) * L + x]) < fq) { kk = kk + 1u; } else { break; } }
    let vq = scratch[kk * L + x]; let d = fq - f32(vq);
    ${DST}[x * M + q] = d * d + ${SRC}[vq * L + x];
    arg[${ARGOFF} + x * M + q] = vq;
  }
}`;
  return `
struct U { nx: u32, ny: u32, nseg: u32, zoff: u32, cell: f32, r: f32, a: f32, b: f32 };
@group(0) @binding(0) var<uniform> u: U;
@group(0) @binding(1) var<storage, read> segs: array<vec4<f32>>;
@group(0) @binding(2) var<storage, read_write> Dg: array<f32>;
@group(0) @binding(3) var<storage, read_write> ins: array<u32>;
@group(0) @binding(4) var<storage, read_write> fA: array<f32>;
@group(0) @binding(5) var<storage, read_write> fB: array<f32>;
@group(0) @binding(6) var<storage, read_write> scratch: array<u32>;
@group(0) @binding(7) var<storage, read_write> arg: array<u32>;
const BIG: f32 = 1e30;

@compute @workgroup_size(64)
fn dmask(@builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>, @builtin(local_invocation_index) li: u32) {
  let i = (wid.y * nw.x + wid.x) * 64u + li; if (i >= u.nx * u.ny) { return; }
  let px = (f32(i % u.nx) + 0.5) * u.cell; let py = (f32(i / u.nx) + 0.5) * u.cell;
  var best: f32 = 1e30; var cnt: u32 = 0u;
  for (var s: u32 = 0u; s < u.nseg; s = s + 1u) {
    let sg = segs[s];
    if ((sg.y <= py) != (sg.w <= py)) { let xi = sg.x + (py - sg.y) / (sg.w - sg.y) * (sg.z - sg.x); if (xi > px) { cnt = cnt + 1u; } }
    let dx = sg.z - sg.x; let dy = sg.w - sg.y; let l2 = dx * dx + dy * dy;
    var t: f32 = 0.0; if (l2 > 0.0) { t = clamp(((px - sg.x) * dx + (py - sg.y) * dy) / l2, 0.0, 1.0); }
    let qx = sg.x + t * dx - px; let qy = sg.y + t * dy - py; best = min(best, qx * qx + qy * qy);
  }
  let inside = (cnt & 1u) == 1u;
  ins[i] = select(0u, 1u, inside); Dg[i] = select(0.0, sqrt(best), inside);
}
@compute @workgroup_size(64)
fn seed(@builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>, @builtin(local_invocation_index) li: u32) {
  let i = (wid.y * nw.x + wid.x) * 64u + li; if (i >= u.nx * u.ny) { return; }
  fA[i] = select(BIG, 0.0, ins[i] == 1u && Dg[i] >= u.r);
}
${edt('edt1', 'fA', 'fB', 'u.nx', 'u.ny', '0u')}
${edt('edt2', 'fB', 'fA', 'u.ny', 'u.nx', 'u.nx * u.ny')}

fn Dc(i: i32, j: i32) -> f32 { return Dg[u32(j) * u.nx + u32(i)]; }
@compute @workgroup_size(64)
fn refine(@builtin(workgroup_id) wid: vec3<u32>, @builtin(num_workgroups) nw: vec3<u32>, @builtin(local_invocation_index) li: u32) {
  let nx = i32(u.nx); let ny = i32(u.ny); let nn = u.nx * u.ny;
  let p = (wid.y * nw.x + wid.x) * 64u + li; if (p >= nn) { return; }
  let ci = i32(p % u.nx); let cj = i32(p / u.nx); let cell = u.cell; let r = u.r;
  if (ins[p] == 0u || Dg[p] >= r) { fA[p] = 0.0; return; }     // fA is reused as the output
  let px = (f32(ci) + 0.5) * cell; let py = (f32(cj) + 0.5) * cell;
  var best: f32 = 1e30;
  for (var dj: i32 = -1; dj <= 1; dj = dj + 1) { for (var di: i32 = -1; di <= 1; di = di + 1) {
    let ii = ci + di; let jj = cj + dj; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) { continue; }
    let col = arg[nn + u32(jj) * u.nx + u32(ii)]; if (col == 0xFFFFFFFFu) { continue; }
    let row = arg[col * u.ny + u32(jj)];
    let sx = i32(col); let sy = i32(row);
    let Dcc = Dc(sx, sy);
    let gx = (Dc(min(sx + 1, nx - 1), sy) - Dc(max(sx - 1, 0), sy)) / (2.0 * cell);
    let gy = (Dc(sx, min(sy + 1, ny - 1)) - Dc(sx, max(sy - 1, 0))) / (2.0 * cell);
    var gl = sqrt(gx * gx + gy * gy); if (gl == 0.0) { gl = 1.0; }
    let t = Dcc - r;
    let ex = (f32(sx) + 0.5) * cell - t * gx / gl; let ey = (f32(sy) + 0.5) * cell - t * gy / gl;
    best = min(best, sqrt((px - ex) * (px - ex) + (py - ey) * (py - ey)));
  } }
  let lo = r - Dg[p];
  if (lo > 0.0) {
    let gx = (Dc(min(ci + 1, nx - 1), cj) - Dc(max(ci - 1, 0), cj)) / (2.0 * cell);
    let gy = (Dc(ci, min(cj + 1, ny - 1)) - Dc(ci, max(cj - 1, 0))) / (2.0 * cell);
    let gl = sqrt(gx * gx + gy * gy);
    if (gl > 0.5) {
      let qx = px + lo * gx / gl; let qy = py + lo * gy / gl;
      let fi = qx / cell - 0.5; let fj = qy / cell - 0.5; let i0 = i32(floor(fi)); let j0 = i32(floor(fj));
      if (i0 >= 0 && j0 >= 0 && i0 + 1 < nx && j0 + 1 < ny) {
        let tx = fi - f32(i0); let ty = fj - f32(j0);
        let Dq = Dc(i0, j0) * (1.0 - tx) * (1.0 - ty) + Dc(i0 + 1, j0) * tx * (1.0 - ty) + Dc(i0, j0 + 1) * (1.0 - tx) * ty + Dc(i0 + 1, j0 + 1) * tx * ty;
        if (Dq >= r - 0.15 * cell) { fA[p] = lo; return; }
      }
    }
  }
  fA[p] = max(max(best, lo), 0.0);
}`;
};

// Layered detection. Never throws: sets GPU.state to 'ok' or 'off' with a message.
async function gpuDetect() {
  const off = (m) => { GPU.state = 'off'; GPU.msg = m; return false; };
  try {
    if (/[?&]gpu=off\b/.test(location.search)) return off('disabled by ?gpu=off');
    if (!('gpu' in navigator) || !navigator.gpu) return off('WebGPU not available in this browser (navigator.gpu missing; needs a secure context such as localhost or https)');
    const adapter = await navigator.gpu.requestAdapter({ powerPreference: 'high-performance' });
    if (!adapter) return off('no WebGPU adapter');
    const info = adapter.info || (adapter.requestAdapterInfo ? await adapter.requestAdapterInfo() : {}) || {};
    if (adapter.isFallbackAdapter || info.isFallbackAdapter) return off('only a software (fallback) adapter available: treated as no GPU');
    GPU.info = info; GPU.adapterName = [info.vendor, info.architecture, info.description || info.device].filter(Boolean).join(' / ') || 'unknown adapter';
    const lim = {};
    for (const k of ['maxStorageBufferBindingSize', 'maxBufferSize']) lim[k] = adapter.limits[k];
    if ((adapter.limits.maxStorageBuffersPerShaderStage || 0) < 7) return off('adapter allows fewer than 7 storage buffers per stage');
    const dev = await adapter.requestDevice({ requiredLimits: lim });
    GPU.dev = dev; GPU.limits = { bind: dev.limits.maxStorageBufferBindingSize, buf: dev.limits.maxBufferSize };
    dev.lost.then((i) => { GPU.lost = true; GPU.state = 'off'; GPU.msg = 'device lost (' + i.reason + '): ' + i.message; if (window.gpuLostHook) window.gpuLostHook(GPU.msg); });
    dev.addEventListener('uncapturederror', (e) => { GPU.lastError = e.error.message; console.error('WebGPU:', e.error.message); });
    const mod = dev.createShaderModule({ code: gpuWGSL() });
    const ci = await mod.getCompilationInfo();
    const errs = ci.messages.filter((m) => m.type === 'error');
    if (errs.length) return off('WGSL compile error: ' + errs[0].message + ' @line ' + errs[0].lineNum);
    const st = (b, t) => ({ binding: b, visibility: GPUShaderStage.COMPUTE, buffer: { type: t } });
    GPU.bgl = dev.createBindGroupLayout({ entries: [st(0, 'uniform'), st(1, 'read-only-storage'), ...[2, 3, 4, 5, 6, 7].map((b) => st(b, 'storage'))] });
    const layout = dev.createPipelineLayout({ bindGroupLayouts: [GPU.bgl] });
    GPU.pipe = {};
    for (const e of ['dmask', 'seed', 'edt1', 'edt2', 'refine']) GPU.pipe[e] = await dev.createComputePipelineAsync({ layout, compute: { module: mod, entryPoint: e } });
    GPU.state = 'ok'; GPU.msg = 'ready: ' + GPU.adapterName;
    return true;
  } catch (e) { return off('WebGPU setup failed: ' + (e && e.message || e)); }
}

function gpuFits(nx, ny) {
  if (GPU.state !== 'ok' || GPU.lost) return false;
  const nn = nx * ny, need = Math.max(2 * nn * 4, 2 * (Math.max((ny + 1) * nx, (nx + 1) * ny)) * 4, nn * 4);
  return need <= Math.min(GPU.limits.bind, GPU.limits.buf);
}

function gpuAlloc(nx, ny) {
  const d = GPU.dev, nn = nx * ny, R = GPU.res;
  if (R && R.nx === nx && R.ny === ny) return R;
  if (R) for (const k of ['u', 'segs', 'Dg', 'ins', 'fA', 'fB', 'scratch', 'arg']) R[k]?.destroy();
  const U = GPUBufferUsage, sto = U.STORAGE | U.COPY_SRC | U.COPY_DST, zoff = Math.max((ny + 1) * nx, (nx + 1) * ny);
  const mk = (size, usage) => d.createBuffer({ size: Math.max(16, size), usage });
  const b = { nx, ny, zoff, u: mk(32, U.UNIFORM | U.COPY_DST), segs: null, segCap: 0, Dg: mk(nn * 4, sto), ins: mk(nn * 4, sto), fA: mk(nn * 4, sto), fB: mk(nn * 4, sto), scratch: mk(2 * zoff * 4, sto), arg: mk(2 * nn * 4, sto) };
  GPU.res = b; return b;
}
function gpuBind(b) {
  const e = (i, buf) => ({ binding: i, resource: { buffer: buf } });
  return GPU.dev.createBindGroup({ layout: GPU.bgl, entries: [e(0, b.u), e(1, b.segs), e(2, b.Dg), e(3, b.ins), e(4, b.fA), e(5, b.fB), e(6, b.scratch), e(7, b.arg)] });
}
async function gpuRead(buf, bytes) {
  const d = GPU.dev, rb = d.createBuffer({ size: bytes, usage: GPUBufferUsage.MAP_READ | GPUBufferUsage.COPY_DST });
  const enc = d.createCommandEncoder(); enc.copyBufferToBuffer(buf, 0, rb, 0, bytes); d.queue.submit([enc.finish()]);
  await rb.mapAsync(GPUMapMode.READ); const out = rb.getMappedRange().slice(0); rb.unmap(); rb.destroy(); return out;
}
function gpuUniform(b, nseg, cell, r) {
  const ab = new ArrayBuffer(32), dv = new DataView(ab);
  dv.setUint32(0, b.nx, true); dv.setUint32(4, b.ny, true); dv.setUint32(8, nseg, true); dv.setUint32(12, b.zoff, true); dv.setFloat32(16, cell, true); dv.setFloat32(20, r, true);
  GPU.dev.queue.writeBuffer(b.u, 0, ab);
}
async function gpuGuard(fn) {   // validation + out-of-memory errors of one GPU job become exceptions; a lost device throws too
  const d = GPU.dev; if (GPU.lost) throw new Error(GPU.msg);
  d.pushErrorScope('validation'); d.pushErrorScope('out-of-memory');
  let res, err = null;
  try { res = await fn(); } catch (e) { err = e; }
  const oom = await d.popErrorScope(), val = await d.popErrorScope();
  if (GPU.lost) throw new Error(GPU.msg);
  if (val) throw new Error('GPU validation error: ' + val.message);
  if (oom) throw new Error('GPU out of memory: ' + oom.message);
  if (err) throw err;
  return res;
}

// grid: {x0,y0,nx,ny,cell}; segs: [[ax,ay,bx,by],...] in world mm. Returns {inside:Uint8Array, D:Float32Array}; the buffers stay resident for gpuErode.
async function gpuGrid(grid, segs) {
  return gpuGuard(async () => {
    const d = GPU.dev, b = gpuAlloc(grid.nx, grid.ny), nn = grid.nx * grid.ny;
    const sf = new Float32Array(segs.length * 4);
    segs.forEach((s, i) => { sf[4 * i] = s[0] - grid.x0; sf[4 * i + 1] = s[1] - grid.y0; sf[4 * i + 2] = s[2] - grid.x0; sf[4 * i + 3] = s[3] - grid.y0; });
    if (b.segCap < segs.length) { b.segs?.destroy(); b.segs = d.createBuffer({ size: Math.max(16, segs.length * 16), usage: GPUBufferUsage.STORAGE | GPUBufferUsage.COPY_DST }); b.segCap = segs.length; }
    d.queue.writeBuffer(b.segs, 0, sf); gpuUniform(b, segs.length, grid.cell, 0);
    const bg = gpuBind(b), enc = d.createCommandEncoder(), groups = Math.ceil(nn / 64), gx = Math.min(groups, 4096), gy = Math.ceil(groups / gx);
    const pass = enc.beginComputePass(); pass.setPipeline(GPU.pipe.dmask); pass.setBindGroup(0, bg); pass.dispatchWorkgroups(gx, gy); pass.end();
    d.queue.submit([enc.finish()]);
    const D = new Float32Array(await gpuRead(b.Dg, nn * 4)), insU = new Uint32Array(await gpuRead(b.ins, nn * 4)), inside = new Uint8Array(nn);
    for (let i = 0; i < nn; i++) inside[i] = insU[i] ? 1 : 0;
    b.gridId = ++GPU.gridCounter; b.nseg = segs.length; b.cell = grid.cell;
    return { inside, D, gridId: b.gridId };
  });
}
// d = distance to the eroded region for the resident grid (see distToEroded); cells outside the profile are 0.
async function gpuErode(gridId, r) {
  return gpuGuard(async () => {
    const d = GPU.dev, b = GPU.res; if (!b || b.gridId !== gridId) throw new Error('GPU grid buffers are stale');
    const nn = b.nx * b.ny; gpuUniform(b, b.nseg, b.cell, r);
    const bg = gpuBind(b), enc = d.createCommandEncoder(), groups = Math.ceil(nn / 64), gx = Math.min(groups, 4096), gy = Math.ceil(groups / gx);
    const run = (e, x, y) => { const p = enc.beginComputePass(); p.setPipeline(GPU.pipe[e]); p.setBindGroup(0, bg); p.dispatchWorkgroups(x, y); p.end(); };
    run('seed', gx, gy); run('edt1', Math.ceil(b.nx / 64), 1); run('edt2', Math.ceil(b.ny / 64), 1); run('refine', gx, gy);
    d.queue.submit([enc.finish()]);
    return new Float32Array(await gpuRead(b.fA, nn * 4));
  });
}
