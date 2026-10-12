// Public render entry: mesh + view + highlighted face ids -> PNG + legend.
// The legend maps chip numbers to persistent face names (the mapping the
// viewer and agents share); chips are drawn on the image, names live in data.
import type { MeshData } from './mesh.js';
import { PALETTE, renderTriangles, type RGB, type ViewName } from './raster.js';
import { encodePng } from './png.js';

// 3x5 digit glyphs.
const DIGITS: Record<string, string[]> = {
  '0': ['111', '101', '101', '101', '111'],
  '1': ['010', '110', '010', '010', '111'],
  '2': ['111', '001', '111', '100', '111'],
  '3': ['111', '001', '111', '001', '111'],
  '4': ['101', '101', '111', '001', '001'],
  '5': ['111', '100', '111', '001', '111'],
  '6': ['111', '100', '111', '101', '111'],
  '7': ['111', '001', '010', '010', '010'],
  '8': ['111', '101', '111', '101', '111'],
  '9': ['111', '101', '111', '001', '111'],
};

function drawDigit(rgb: Uint8Array, w: number, h: number, d: string, x0: number, y0: number, color: RGB): void {
  const g = DIGITS[d];
  if (!g) return;
  for (let gy = 0; gy < 5; gy++) {
    for (let gx = 0; gx < 3; gx++) {
      if (g[gy][gx] !== '1') continue;
      const xx = x0 + gx, yy = y0 + gy;
      if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
      rgb[(yy * w + xx) * 3] = color[0];
      rgb[(yy * w + xx) * 3 + 1] = color[1];
      rgb[(yy * w + xx) * 3 + 2] = color[2];
    }
  }
}

function drawNumber(rgb: Uint8Array, w: number, h: number, n: number, x: number, y: number, color: RGB): void {
  const s = String(n);
  s.split('').forEach((d, i) => drawDigit(rgb, w, h, d, x + i * 4, y, color));
}

export interface RenderOpts {
  width?: number;
  height?: number;
  view?: ViewName;
  /** persistent face ids to highlight (unknown ids are ignored) */
  highlights?: string[];
}

export interface LegendEntry {
  n: number;
  face: string;
  color: RGB;
}

export interface RenderResult {
  png: Buffer;
  width: number;
  height: number;
  legend: LegendEntry[];
}

export function renderMesh(mesh: MeshData, opts: RenderOpts = {}): RenderResult {
  const w = opts.width ?? 640;
  const h = opts.height ?? 480;
  const colors = new Map<number, RGB>();
  const legend: LegendEntry[] = [];
  for (const id of (opts.highlights ?? []).slice(0, 20)) {
    const fi = mesh.faceIds.indexOf(id);
    if (fi < 0 || colors.has(fi)) continue;
    const color = PALETTE[legend.length % PALETTE.length];
    colors.set(fi, color);
    legend.push({ n: legend.length + 1, face: id, color });
  }
  const frame = renderTriangles(mesh, opts.view ?? 'iso', w, h, colors);

  // Legend strip along the bottom: chip + number per highlight.
  if (legend.length) {
    const y0 = h - 20;
    for (let x = 0; x < w; x++) {
      for (let y = y0; y < h; y++) {
        frame.rgb[(y * w + x) * 3] = 10;
        frame.rgb[(y * w + x) * 3 + 1] = 14;
        frame.rgb[(y * w + x) * 3 + 2] = 20;
      }
    }
    legend.forEach((e, i) => {
      const x0 = 8 + i * 64;
      for (let y = 0; y < 12; y++) {
        for (let x = 0; x < 12; x++) {
          const xx = x0 + x, yy = y0 + 4 + y;
          frame.rgb[(yy * w + xx) * 3] = e.color[0];
          frame.rgb[(yy * w + xx) * 3 + 1] = e.color[1];
          frame.rgb[(yy * w + xx) * 3 + 2] = e.color[2];
        }
      }
      drawNumber(frame.rgb, w, h, e.n, x0 + 16, y0 + 6, [255, 255, 255]);
    });
  }
  return { png: encodePng(w, h, frame.rgb), width: w, height: h, legend };
}
