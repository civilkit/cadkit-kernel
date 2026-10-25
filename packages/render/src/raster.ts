// Orthographic software rasteriser: z-buffer + barycentric fill + lambert.
// Highlighted faces get palette colours; everything else is shaded grey.
import type { MeshData } from './mesh.js';

export type ViewName = 'iso' | 'front' | 'top' | 'right';
export type RGB = [number, number, number];

export const PALETTE: RGB[] = [
  [217, 52, 44], [42, 91, 215], [22, 163, 74], [224, 168, 0],
  [124, 58, 237], [236, 114, 183], [0, 153, 153], [200, 80, 20],
];

const BACK: RGB = [16, 22, 30];

function norm(v: number[]): number[] {
  const l = Math.hypot(...v) || 1;
  return v.map((x) => x / l);
}

function sub(a: number[], b: number[]): number[] {
  return a.map((x, i) => x - b[i]);
}

function cross(a: number[], b: number[]): number[] {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function dot(a: number[], b: number[]): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** Camera basis for a named view (world is Z-up). dir points scene -> viewer
 *  (larger projected z wins the depth test); right x up == dir keeps the
 *  image right-handed (unmirrored). */
export function cameraFor(view: ViewName): { right: number[]; up: number[]; dir: number[] } {
  switch (view) {
    case 'front': return { right: [1, 0, 0], up: [0, 1, 0], dir: [0, 0, 1] };
    case 'top': return { right: [-1, 0, 0], up: [0, 0, 1], dir: [0, 1, 0] };
    case 'right': return { right: [0, 1, 0], up: [0, 0, 1], dir: [1, 0, 0] };
    case 'iso': {
      const dir = norm([1, 1, 1]);
      const right = norm(cross([0, 0, 1], dir));
      const up = cross(dir, right);
      return { right, up, dir };
    }
  }
}

export interface Frame {
  w: number;
  h: number;
  rgb: Uint8Array;
}

/** World box to frame instead of the whole mesh (a close-up). */
export interface FitBox {
  min: number[];
  max: number[];
}

export function renderTriangles(
  mesh: MeshData,
  view: ViewName,
  w: number,
  h: number,
  colors: Map<number, RGB>,
  fitBox?: FitBox,
): Frame {
  const { right, up, dir } = cameraFor(view);
  const proj = (p: number[]): [number, number, number] => [dot(p, right), dot(p, up), dot(p, dir)];
  // Fit projected bbox with a margin; fitBox frames a region of the scene
  // instead, so a close-up keeps the whole body (and its occlusion) in view.
  let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
  const consider = (p: number[]): void => {
    const [x, y] = proj(p);
    if (x < x0) x0 = x; if (x > x1) x1 = x;
    if (y < y0) y0 = y; if (y > y1) y1 = y;
  };
  if (fitBox) {
    for (let c = 0; c < 8; c++) {
      const side = [fitBox.min, fitBox.max];
      consider([0, 1, 2].map((a) => side[(c >> a) & 1][a]));
    }
  } else {
    for (let i = 0; i < mesh.positions.length; i += 3) {
      consider([mesh.positions[i], mesh.positions[i + 1], mesh.positions[i + 2]]);
    }
  }
  const span = Math.max(x1 - x0, y1 - y0, 1e-9);
  const sc = Math.min(w, h) * 0.9 / span;
  const cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
  const px = (p: number[]): [number, number, number] => {
    const [x, y, z] = proj(p);
    return [(x - cx) * sc + w / 2, h / 2 - (y - cy) * sc, z];
  };

  const rgb = new Uint8Array(w * h * 3);
  for (let i = 0; i < w * h; i++) { rgb[i * 3] = BACK[0]; rgb[i * 3 + 1] = BACK[1]; rgb[i * 3 + 2] = BACK[2]; }
  const depth = new Float32Array(w * h).fill(-Infinity);
  const light = norm([0.4, -0.6, 0.75]);
  const tris = Math.floor(mesh.positions.length / 9);

  for (let t = 0; t < tris; t++) {
    const v = [0, 1, 2].map((k) => {
      const o = (t * 3 + k) * 3;
      return px([mesh.positions[o], mesh.positions[o + 1], mesh.positions[o + 2]]);
    });
    const n = [
      mesh.normals[t * 9] + mesh.normals[t * 9 + 3] + mesh.normals[t * 9 + 6],
      mesh.normals[t * 9 + 1] + mesh.normals[t * 9 + 4] + mesh.normals[t * 9 + 7],
      mesh.normals[t * 9 + 2] + mesh.normals[t * 9 + 5] + mesh.normals[t * 9 + 8],
    ];
    const nl = norm(n);
    // No backface culling: the z-buffer resolves closed solids, and open
    // sheets stay visible from behind (lit dark by the lambert floor).

    const face = mesh.triFace[t];
    const hi = colors.get(face);
    let r: number, g: number, b: number;
    if (hi) { [r, g, b] = hi; }
    else {
      const l = 0.35 + 0.65 * Math.max(0, dot(nl, light));
      r = g = b = Math.round(200 * l);
    }

    const area = (v[1][0] - v[0][0]) * (v[2][1] - v[0][1]) - (v[2][0] - v[0][0]) * (v[1][1] - v[0][1]);
    if (Math.abs(area) < 1e-9) continue;
    const xMin = Math.max(0, Math.floor(Math.min(v[0][0], v[1][0], v[2][0])));
    const xMax = Math.min(w - 1, Math.ceil(Math.max(v[0][0], v[1][0], v[2][0])));
    const yMin = Math.max(0, Math.floor(Math.min(v[0][1], v[1][1], v[2][1])));
    const yMax = Math.min(h - 1, Math.ceil(Math.max(v[0][1], v[1][1], v[2][1])));
    for (let y = yMin; y <= yMax; y++) {
      for (let x = xMin; x <= xMax; x++) {
        const w0 = ((v[1][0] - x) * (v[2][1] - y) - (v[2][0] - x) * (v[1][1] - y)) / area;
        const w1 = ((v[2][0] - x) * (v[0][1] - y) - (v[0][0] - x) * (v[2][1] - y)) / area;
        const w2 = 1 - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
        const z = w0 * v[0][2] + w1 * v[1][2] + w2 * v[2][2];
        const idx = y * w + x;
        if (z > depth[idx]) {
          depth[idx] = z;
          rgb[idx * 3] = r; rgb[idx * 3 + 1] = g; rgb[idx * 3 + 2] = b;
        }
      }
    }
  }
  return { w, h, rgb };
}
