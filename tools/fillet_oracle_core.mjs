// Core of the fillet oracle: grid rasterisation, distance transforms, the round / chamfer height field. No dependencies, runs in
// node and (inlined by tools/fillet_oracle_build_viewer.mjs) in the browser viewer. See docs/fillet-oracle.md.
// ------------------------------------------------------------------ geometry on a grid
const BIG = 1e20;

// Felzenszwalb 1D squared-distance transform with nearest-seed index. f[i] = 0 at seeds, BIG elsewhere.
function edt1d(f, n, dOut, argOut, v, z) {
  let k = 0;
  v[0] = 0; z[0] = -Infinity; z[1] = Infinity;
  for (let q = 1; q < n; q++) {
    let s;
    for (;;) {
      const p = v[k];
      s = ((f[q] + q * q) - (f[p] + p * p)) / (2 * q - 2 * p);
      if (s <= z[k] && k > 0) k--; else break;
    }
    if (s <= z[k]) { v[k] = q; z[k] = -Infinity; z[k + 1] = Infinity; continue; }
    k++; v[k] = q; z[k] = s; z[k + 1] = Infinity;
  }
  k = 0;
  for (let q = 0; q < n; q++) {
    while (z[k + 1] < q) k++;
    const p = v[k];
    dOut[q] = (q - p) * (q - p) + f[p];
    argOut[q] = p;
  }
}

// Euclidean distance (in cells) from every cell to the nearest seed (seed[i] = 1), plus the nearest seed's cell index.
function edt2d(seed, nx, ny) {
  const n = Math.max(nx, ny);
  const f = new Float64Array(n), d = new Float64Array(n), arg = new Int32Array(n), v = new Int32Array(n), z = new Float64Array(n + 1);
  const g = new Float64Array(nx * ny), gy = new Int32Array(nx * ny);
  for (let x = 0; x < nx; x++) {              // columns
    for (let y = 0; y < ny; y++) f[y] = seed[y * nx + x] ? 0 : BIG;
    edt1d(f, ny, d, arg, v, z);
    for (let y = 0; y < ny; y++) { g[y * nx + x] = d[y]; gy[y * nx + x] = arg[y]; }
  }
  const dist = new Float32Array(nx * ny), near = new Int32Array(nx * ny);
  for (let y = 0; y < ny; y++) {              // rows
    for (let x = 0; x < nx; x++) f[x] = g[y * nx + x];
    edt1d(f, nx, d, arg, v, z);
    for (let x = 0; x < nx; x++) { const sx = arg[x]; dist[y * nx + x] = d[x] >= BIG ? Infinity : Math.sqrt(d[x]); near[y * nx + x] = gy[y * nx + sx] * nx + sx; }
  }
  return { dist, near };
}

// Grid geometry (origin, cell counts) of the padded bounding box of the segments.
export function gridGeom(segs, cell, pad = 1) {
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (const s of segs) { x0 = Math.min(x0, s[0], s[2]); x1 = Math.max(x1, s[0], s[2]); y0 = Math.min(y0, s[1], s[3]); y1 = Math.max(y1, s[1], s[3]); }
  x0 -= pad; y0 -= pad; x1 += pad; y1 += pad;
  return { x0, y0, x1, y1, nx: Math.ceil((x1 - x0) / cell), ny: Math.ceil((y1 - y0) / cell), cell };
}

// Segments [x0,y0,x1,y1] -> grid with inside mask (even-odd) and exact distance to the boundary for inside cells.
// `geom` (optional, from gridGeom or a window of one) fixes the grid instead of the bounding box.
export function rasterise(segs, cell, pad = 1, geom = null) {
  const { x0, y0, x1, y1, nx, ny } = geom ?? gridGeom(segs, cell, pad);
  const cx = (i) => x0 + (i + 0.5) * cell, cy = (j) => y0 + (j + 0.5) * cell;
  const inside = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) {
    const y = cy(j), xs = [];
    for (const [ax, ay, bx, by] of segs) if ((ay <= y) !== (by <= y)) xs.push(ax + (y - ay) / (by - ay) * (bx - ax));
    xs.sort((a, b) => a - b);
    for (let t = 0; t + 1 < xs.length; t += 2) {
      const i0 = Math.max(0, Math.ceil((xs[t] - x0) / cell - 0.5)), i1 = Math.min(nx - 1, Math.floor((xs[t + 1] - x0) / cell - 0.5));
      for (let i = i0; i <= i1; i++) inside[j * nx + i] = 1;
    }
  }
  // exact distance to the polyline by bucketed ring search
  const brute = !!geom?.brute;   // windows: every segment counts, not only those inside the window
  const B = Math.max(1, 20 * cell), bx = brute ? 1 : Math.ceil((x1 - x0) / B), by = brute ? 1 : Math.ceil((y1 - y0) / B);
  const buckets = Array.from({ length: bx * by }, () => []);
  if (!brute) segs.forEach((s, si) => {
    const sx0 = Math.min(s[0], s[2]), sx1 = Math.max(s[0], s[2]), sy0 = Math.min(s[1], s[3]), sy1 = Math.max(s[1], s[3]);
    for (let j = Math.floor((sy0 - y0) / B); j <= Math.floor((sy1 - y0) / B); j++) for (let i = Math.floor((sx0 - x0) / B); i <= Math.floor((sx1 - x0) / B); i++) if (i >= 0 && j >= 0 && i < bx && j < by) buckets[j * bx + i].push(si);
  });
  const D = new Float32Array(nx * ny);
  const segDist2 = (s, px, py) => {
    const dx = s[2] - s[0], dy = s[3] - s[1], l2 = dx * dx + dy * dy;
    let t = l2 > 0 ? ((px - s[0]) * dx + (py - s[1]) * dy) / l2 : 0; t = t < 0 ? 0 : t > 1 ? 1 : t;
    const qx = s[0] + t * dx - px, qy = s[1] + t * dy - py; return qx * qx + qy * qy;
  };
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    if (!inside[j * nx + i]) continue;
    const px = cx(i), py = cy(j), bi = Math.floor((px - x0) / B), bj = Math.floor((py - y0) / B);
    let best = Infinity;
    if (brute) { for (const sg of segs) { const d2 = segDist2(sg, px, py); if (d2 < best) best = d2; } D[j * nx + i] = Math.sqrt(best); continue; }
    for (let ring = 0; ring < Math.max(bx, by); ring++) {
      if (best < Infinity && (ring - 1) * B > Math.sqrt(best)) break;
      for (let jj = bj - ring; jj <= bj + ring; jj++) for (let ii = bi - ring; ii <= bi + ring; ii++) {
        if (Math.max(Math.abs(ii - bi), Math.abs(jj - bj)) !== ring || ii < 0 || jj < 0 || ii >= bx || jj >= by) continue;
        for (const si of buckets[jj * bx + ii]) { const d2 = segDist2(segs[si], px, py); if (d2 < best) best = d2; }
      }
    }
    D[j * nx + i] = Math.sqrt(best);
  }
  return { x0, y0, nx, ny, cell, inside, D };
}

// d = distance to Pr = {D >= r}. The grid EDT gives the nearest seed cells; each seed c is moved to the edge of Pr along the
// outward gradient of D (e = c - (D(c) - r) grad D(c)), and d is the shortest distance from the cell centre to those edge points
// over the nearest seeds of the 3x3 neighbourhood (sub-cell accurate on smooth edges). Lower bound r - D (1-Lipschitz).
export function distToEroded(g, r) {
  const { nx, ny, cell, inside, D, x0, y0 } = g;
  const seed = new Uint8Array(nx * ny); let n = 0;
  for (let i = 0; i < nx * ny; i++) if (inside[i] && D[i] >= r) { seed[i] = 1; n++; }
  const d = new Float32Array(nx * ny);
  if (n === 0) { d.fill(Infinity); return { d, prArea: 0 }; }
  const { near } = edt2d(seed, nx, ny);
  const ex = new Float32Array(nx * ny), ey = new Float32Array(nx * ny);   // edge point of each seed (filled lazily)
  const done = new Uint8Array(nx * ny);
  const edge = (c) => {
    if (done[c]) return;
    done[c] = 1;
    const ci = c % nx, cj = (c - ci) / nx;
    const gx = ((ci + 1 < nx ? D[c + 1] : D[c]) - (ci > 0 ? D[c - 1] : D[c])) / (2 * cell), gy = ((cj + 1 < ny ? D[c + nx] : D[c]) - (cj > 0 ? D[c - nx] : D[c])) / (2 * cell);
    const gl = Math.hypot(gx, gy) || 1, t = D[c] - r;
    ex[c] = x0 + (ci + 0.5) * cell - t * gx / gl; ey[c] = y0 + (cj + 0.5) * cell - t * gy / gl;
  };
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const p = j * nx + i;
    if (!inside[p]) continue;
    if (seed[p]) { d[p] = 0; continue; }
    const px = x0 + (i + 0.5) * cell, py = y0 + (j + 0.5) * cell;
    let best = Infinity;
    for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
      const ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
      const c = near[jj * nx + ii]; edge(c);
      const dd = Math.hypot(px - ex[c], py - ey[c]); if (dd < best) best = dd;
    }
    // the inward normal ray from p reaches Pr after exactly r - D(p) when the point q it ends at is still at depth r:
    // then the distance is exact (the lower bound is attained)
    const lo = r - D[p];
    if (lo > 0) {
      const gx = ((i + 1 < nx ? D[p + 1] : D[p]) - (i > 0 ? D[p - 1] : D[p])) / (2 * cell), gy = ((j + 1 < ny ? D[p + nx] : D[p]) - (j > 0 ? D[p - nx] : D[p])) / (2 * cell), gl = Math.hypot(gx, gy);
      if (gl > 0.5) {
        const qx = px + lo * gx / gl, qy = py + lo * gy / gl, fi = (qx - x0) / cell - 0.5, fj = (qy - y0) / cell - 0.5, i0 = Math.floor(fi), j0 = Math.floor(fj);
        if (i0 >= 0 && j0 >= 0 && i0 + 1 < nx && j0 + 1 < ny) {
          const tx = fi - i0, ty = fj - j0;
          const Dq = D[j0 * nx + i0] * (1 - tx) * (1 - ty) + D[j0 * nx + i0 + 1] * tx * (1 - ty) + D[(j0 + 1) * nx + i0] * (1 - tx) * ty + D[(j0 + 1) * nx + i0 + 1] * tx * ty;
          if (Dq >= r - 0.15 * cell) { d[p] = lo; continue; }
        }
      }
    }
    d[p] = Math.max(best, lo, 0);
  }
  return { d, prArea: n * cell * cell };
}

export function heightfield(g, d, H, r, kind) {
  const z = new Float32Array(g.nx * g.ny).fill(NaN);
  for (let i = 0; i < z.length; i++) {
    if (!g.inside[i]) continue;
    const di = d[i];
    if (kind === 'chamfer') z[i] = H - Math.min(di, r);
    else z[i] = di <= r ? H - r + Math.sqrt(Math.max(0, r * r - di * di)) : H - r;
  }
  return z;
}


// Variant "swept-cut": the removed depth at a point depends on the distance delta to the nearest profile boundary only,
//   round:   depth = r - sqrt(r^2 - (r - delta)^2)   (delta < r, else nothing removed)
//   chamfer: depth = r - delta
// Equal to the opening on smooth rims away from tight tips; at convex corners it gives a mitre (no flat ledge at H - r).
export function heightSwept(g, H, r, kind) {
  const z = new Float32Array(g.nx * g.ny).fill(NaN);
  for (let i = 0; i < z.length; i++) {
    if (!g.inside[i]) continue;
    const dl = g.D[i];
    z[i] = dl >= r ? H : H - (kind === 'chamfer' ? r - dl : r - Math.sqrt(Math.max(0, r * r - (r - dl) * (r - dl))));
  }
  return z;
}
