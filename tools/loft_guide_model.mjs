// Scratch model: which displacement spreading reproduces the oracle's guided loft volume?
// Section at height y: square of half width h(y)=10-5y/30; the guide pulls the contact point
// (h,0) out to the guide curve x_g(y) (natural cubic through (0,10),(15,13),(30,5)); the pull
// fades along the perimeter with a hat of half width L*h.
function natural(xs, ys) {
  const n = xs.length, h = [], a = [], b = [], c = [], d = [];
  for (let i = 0; i < n - 1; i++) h.push(xs[i + 1] - xs[i]);
  const A = Array.from({ length: n }, () => Array(n).fill(0)), r = Array(n).fill(0);
  A[0][0] = 1; A[n - 1][n - 1] = 1;
  for (let i = 1; i < n - 1; i++) { A[i][i - 1] = h[i - 1]; A[i][i] = 2 * (h[i - 1] + h[i]); A[i][i + 1] = h[i]; r[i] = 3 * ((ys[i + 1] - ys[i]) / h[i] - (ys[i] - ys[i - 1]) / h[i - 1]); }
  const cc = Array(n).fill(0);
  for (let i = 0; i < n; i++) { let p = i; for (let k = i + 1; k < n; k++) if (Math.abs(A[k][i]) > Math.abs(A[p][i])) p = k; [A[i], A[p]] = [A[p], A[i]]; [r[i], r[p]] = [r[p], r[i]]; for (let k = i + 1; k < n; k++) { const f = A[k][i] / A[i][i]; for (let j = i; j < n; j++) A[k][j] -= f * A[i][j]; r[k] -= f * r[i]; } }
  for (let i = n - 1; i >= 0; i--) { let s = r[i]; for (let j = i + 1; j < n; j++) s -= A[i][j] * cc[j]; cc[i] = s / A[i][i]; }
  return (x) => { let i = Math.min(Math.max(xs.findIndex((v) => v > x) - 1, 0), n - 2); if (x >= xs[n - 1]) i = n - 2; const t = x - xs[i]; const bi = (ys[i + 1] - ys[i]) / h[i] - h[i] * (2 * cc[i] + cc[i + 1]) / 3; const di = (cc[i + 1] - cc[i]) / (3 * h[i]); return ys[i] + bi * t + cc[i] * t * t + di * t * t * t; };
}
const GV = process.argv[2] ? Number(process.argv[2]) : 13;
const guide = natural([0, 15, 30], [10, GV, 5]);
function area(y, L) {
  const h = 10 - 5 * y / 30, dx = guide(y) - h;
  const pts = [[h, 0, 0], [h, h, 1], [-h, h, 3], [-h, 0, 4], [-h, -h, 3], [h, -h, 1]]; // (x, z, perimeter distance in h units)
  const poly = pts.map(([x, z, d]) => [x + dx * (L === 0 ? (d === 0 ? 1 : 0) : Math.max(0, 1 - d / L)), z]);
  poly[0] = [h + dx, 0];
  let a = 0; for (let i = 0; i < poly.length; i++) { const [x1, z1] = poly[i], [x2, z2] = poly[(i + 1) % poly.length]; a += x1 * z2 - x2 * z1; }
  return Math.abs(a) / 2;
}
for (const L of [1, 1.25, 1.5, 1.75, 2, 3]) {
  let v = 0; const n = 3000;
  for (let i = 0; i < n; i++) v += area((i + 0.5) * 30 / n, L) * 30 / n;
  console.log('L', L, 'volume', v.toFixed(2));
}
