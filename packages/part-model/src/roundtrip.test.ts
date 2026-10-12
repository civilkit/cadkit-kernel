// Round-trip over every sample part: tools/cases/*.ckpart + the bench parts.
// For each file: every feature type (including nested insertPart docs) must be
// in FEATURE_TYPES; rebuilding through the builder must reproduce the document;
// regen of original vs rebuilt must agree (errors, counts, volume).
// largeSampleFull is regen-skipped (115 s; the gate benches it instead).
import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { existsSync, readdirSync, readFileSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { FEATURE_TYPES, type PartDoc } from './index.js';
import { Part } from '../../builder/src/index.js';

const ROOT = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../../../..');

function corpus(): string[] {
  const out: string[] = [];
  const walk = (d: string) => {
    for (const e of readdirSync(d, { withFileTypes: true })) {
      const p = path.join(d, e.name);
      if (e.isDirectory()) walk(p);
      else if (/\.ckpart(\.json)?$/.test(e.name)) out.push(p);
    }
  };
  walk(path.join(ROOT, 'tools/cases'));
  walk(path.join(ROOT, 'tools/bench/benchcad'));
  return out.sort();
}

function eachFeature(doc: PartDoc, fn: (f: { type: string }) => void): void {
  const walk = (feats: Array<{ type: string; part?: PartDoc }>) => {
    for (const f of feats ?? []) {
      fn(f);
      if (f.part && typeof f.part === 'object' && Array.isArray((f.part as PartDoc).features)) {
        walk((f.part as PartDoc).features as Array<{ type: string }>);
      }
    }
  };
  walk(doc.features as Array<{ type: string }>);
}

function rebuild(doc: PartDoc): PartDoc {
  const p = new Part(doc.name, doc.material);
  for (const f of doc.features) p.add({ ...(f as unknown as Record<string, unknown>) });
  p.seq = doc.seq;
  return p.toJSON();
}

// Regen parity skips: measured wall time for original+rebuilt through the
// worker (their types are covered by faster files; the gate benches slow ones).
const SKIP_REGEN: Record<string, string> = {
  'largeSampleFull.ckpart': '115s regen',
  'torsion_spring_000054.ckpart.json': '39s regen',
  'coil_spring_000026.ckpart.json': '22s regen',
};

describe('round-trip', () => {
  it('corpus is non-empty', () => {
    assert.ok(corpus().length >= 30, `only ${corpus().length} files`);
  });

  it('every feature type in every file is known', () => {
    const seen = new Set<string>();
    for (const f of corpus()) {
      const doc = JSON.parse(readFileSync(f, 'utf8')) as PartDoc;
      eachFeature(doc, (feat) => {
        seen.add(feat.type);
        assert.ok(
          (FEATURE_TYPES as readonly string[]).includes(feat.type),
          `${path.basename(f)} uses unknown type ${feat.type}`,
        );
      });
    }
    console.log(`round-trip types covered: ${[...seen].sort().join(' ')}`);
  });

  it('the builder reproduces every document', () => {
    for (const f of corpus()) {
      const doc = JSON.parse(readFileSync(f, 'utf8')) as PartDoc;
      const back = rebuild(doc);
      assert.deepEqual(back.features, doc.features, path.basename(f));
      assert.equal(back.seq, doc.seq, path.basename(f));
      assert.equal(back.name, doc.name, path.basename(f));
    }
  });
});

const wasm = path.join(ROOT, 'target/wasm32-unknown-unknown/release/ck_wasm.wasm');

describe('round-trip regen', { skip: !existsSync(wasm) && 'no wasm built' }, () => {
  it('original and rebuilt agree', async () => {
    const { createWorkerKernel } = await import('../../kernel-client/src/worker.js');
    const k = await createWorkerKernel(wasm);
    try {
      const files = corpus().filter((f) => {
        const why = SKIP_REGEN[path.basename(f)];
        if (why) console.log(`round-trip regen-skipped: ${path.basename(f)} (${why})`);
        return !why;
      });
      assert.ok(files.length >= 30);
      for (const f of files) {
        const t0 = Date.now();
        const doc = JSON.parse(readFileSync(f, 'utf8')) as PartDoc;
        const amid = async (part: PartDoc) => {
          const r = await k.request({ type: 'regen', tag: 't', part });
          if (r.trap || !r.ok || !r.result) throw new Error(`${path.basename(f)}: ${r.trap ?? r.error}`);
          const res = r.result as { errors: unknown; body: { faces?: unknown[]; edges?: unknown[] } | null };
          let volume: number | null = null;
          if (res.body) {
            const m = await k.request({ type: 'massProps', tag: 't' });
            const mr = m.result as { volume?: number } | undefined;
            volume = typeof mr?.volume === 'number' ? mr.volume : null;
          }
          return {
            errors: res.errors,
            faces: res.body?.faces?.length ?? null,
            edges: res.body?.edges?.length ?? null,
            volume,
          };
        };
        const a = await amid(doc);
        const b = await amid(rebuild(doc));
        assert.deepEqual(b.errors, a.errors, `${path.basename(f)} errors`);
        assert.deepEqual({ faces: b.faces, edges: b.edges }, { faces: a.faces, edges: a.edges }, `${path.basename(f)} counts`);
        if (a.volume !== null || b.volume !== null) {
          assert.ok(a.volume !== null && b.volume !== null && Math.abs(a.volume - b.volume) <= 1e-6 * Math.abs(a.volume), `${path.basename(f)} volume ${a.volume} vs ${b.volume}`);
        }
        const dt = Date.now() - t0;
        if (dt > 2000) console.log(`round-trip slow: ${path.basename(f)} ${dt}ms`);
      }
    } finally {
      await k.close();
    }
  });
});
