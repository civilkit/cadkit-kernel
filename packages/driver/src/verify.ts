// The agent's verify loop (brepjs's structure, our ingredients): one call that
// regens, validates, checks expectations and returns "done, or fix this
// feature". Built on the kernel's regen / validate / check requests — no code
// copied from anywhere else.
//
// The report points every problem at the smallest responsible section of the
// agent's code: a feature id plus the field to fix when it is inferable.
import type { PartDoc } from '../../part-model/src/index.js';
import type { KernelReply } from '../../kernel-client/src/protocol.js';

/** Minimal kernel surface verify needs (worker kernel, test fake, later server). */
export interface KernelTransport {
  request(msg: Record<string, unknown>): Promise<KernelReply>;
}

export type RegenError = [featureId: string, err: { code: string; params: Record<string, unknown> }];

export interface ValidateIssue {
  code: string;
  entity: string;
  detail: string;
}

export interface CheckResultItem {
  index: number;
  id?: string;
  kind: string;
  pass: boolean;
  measured?: unknown;
  expected?: unknown;
  detail?: string;
}

export interface RepairHint {
  /** feature id, or '' when the problem is not feature-attributed (check/trap) */
  feature: string;
  code: string;
  hint: string;
}

export interface VerifyReport {
  ok: boolean;
  tag: string;
  ms: number;
  regen: { errors: RegenError[] };
  validate: { ok: boolean; issues: ValidateIssue[] } | null;
  check: { passed: number; failed: number; results: CheckResultItem[] } | null;
  hints: RepairHint[];
}

export interface VerifyOpts {
  tag?: string;
  tol?: number;
  /** check expectations; default [{kind:'valid'}] (validator must be clean) */
  expect?: Array<Record<string, unknown>>;
}

/** Fix guidance per error/issue code. Sources: features field table, ADR 0003. */
const FIX: Record<string, (feature: string, params: Record<string, unknown>) => string> = {
  missingSketch: (f) => `${f}.sketch names a sketch id that does not exist — point it at an existing sketch feature`,
  missingReference: (f) => `${f} references geometry that no longer exists — re-pick the face/edge by name (resolve) and update the ref`,
  nothingToCut: (f) => `${f} removes nothing — move the profile so it overlaps the body, or flip the cut direction`,
  booleanFailed: (f) => `${f}: boolean failed — check the tool and target overlap in a volume, not just a face`,
  shellTooThick: (f) => `${f}: thickness exceeds the thinnest wall — reduce thickness`,
  shellUnsupported: (f) => `${f}: this shape cannot be shelled — remove the feature or change the base solid`,
  holeNoPoints: (f) => `${f}: hole sketch has no points — add centre points to the hole sketch`,
  holeInvalid: (f) => `${f}: hole parameters invalid — check diameter/depth against the stock`,
  ribNeedsOpenSketch: (f) => `${f}: rib profile must be an open chain — open the sketch profile`,
  ribNoContact: (f) => `${f}: rib does not touch the body — extend the profile to contact`,
  knitNotClosed: (f) => `${f}: sheets do not close — close the gap or relax the tolerance`,
  knitNearMiss: (f) => `${f}: sheet edges near-miss — move the boundary or relax the tolerance`,
  fillNotClosed: (f) => `${f}: fill boundary is not closed — close the edge loop`,
  fillContinuity: (f) => `${f}: continuity cannot be met on this loop — drop to G0 or change the loop`,
  draftTooLarge: (f) => `${f}: draft angle too large for these faces — reduce angle`,
  draftParallelFace: (f) => `${f}: neutral plane is parallel to a drafted face — pick another neutral`,
  filletChamfer: (f) => `${f}: blend failed — reduce radius/distance or reorder before other blends`,
  pipeCutRadius: (f, p) => `${f}: pipe radius too large${typeof p.max === 'number' ? ` (max ${p.max})` : ''} — reduce radius`,
  thickenReach: (f) => `${f}: sheet cannot be thickened that far — reduce distance`,
  extendFolds: (f) => `${f}: extension folds the surface — shorten the extension`,
  projectCurveMisses: (f) => `${f}: projected curve misses the faces — move it over the body`,
  mirrorOntoItself: (f) => `${f}: mirror plane maps the body onto itself — move the plane or seeds`,
  referenceChanged: (f) => `${f}: a referenced entity changed shape after an edit — re-resolve the ref`,
  referenceLost: (f) => `${f}: a referenced entity is gone after an edit — pick a replacement ref`,
};

function hintFor(feature: string, code: string, params: Record<string, unknown>): RepairHint {
  const fix = FIX[code];
  return { feature, code, hint: fix ? fix(feature, params) : `${feature || 'part'}: error ${code}${Object.keys(params).length ? ` ${JSON.stringify(params)}` : ''} — see the features field table` };
}

/** Face/edge entity name -> owning feature id (F2.C0 -> F2), else the raw name. */
function ownerOf(entity: string): string {
  const head = entity.split('.')[0];
  return /^F\d+$/.test(head) ? head : entity;
}

export async function verify(
  kernel: KernelTransport,
  part: PartDoc,
  opts: VerifyOpts = {},
): Promise<VerifyReport> {
  const t0 = performance.now();
  const tag = opts.tag ?? 'main';
  const tol = opts.tol ?? 1e-6;
  const expect = opts.expect ?? [{ kind: 'valid' }];
  const hints: RepairHint[] = [];

  const fail = (code: string, hint: string): VerifyReport => ({
    ok: false, tag, ms: Math.round(performance.now() - t0),
    regen: { errors: [] }, validate: null, check: null,
    hints: [{ feature: '', code, hint }],
  });

  let r: KernelReply;
  try {
    r = await kernel.request({ type: 'regen', tag, part });
  } catch (e) {
    return fail('trap', `regen threw: ${String((e as Error).message ?? e)}`);
  }
  if (r.trap) return fail('trap', `regen trapped: ${r.trap}`);
  if (!r.ok) return fail('regenFailed', `regen errored: ${r.error ?? 'unknown'}`);
  const errors = ((r.result as { errors?: RegenError[] }).errors ?? []) as RegenError[];

  let validate: VerifyReport['validate'] = null;
  try {
    const v = await kernel.request({ type: 'validate', tag, tol });
    if (!v.trap && v.ok && v.result) {
      const res = v.result as { ok: boolean; issues: ValidateIssue[] };
      validate = { ok: res.ok, issues: res.issues ?? [] };
    }
  } catch { validate = null; }

  let check: VerifyReport['check'] = null;
  try {
    const c = await kernel.request({ type: 'check', tag, expect });
    if (!c.trap && c.ok && c.result) {
      const res = c.result as { passed: number; failed: number; results: CheckResultItem[] };
      check = { passed: res.passed ?? 0, failed: res.failed ?? 0, results: res.results ?? [] };
    }
  } catch { check = null; }

  for (const [fid, err] of errors) hints.push(hintFor(fid, err.code, err.params ?? {}));
  for (const iss of validate?.issues ?? []) hints.push(hintFor(ownerOf(iss.entity), iss.code, {}));
  for (const res of check?.results ?? []) {
    if (!res.pass) {
      hints.push({
        feature: '',
        code: `expect:${res.kind}`,
        hint: `expectation ${res.kind} failed — measured ${JSON.stringify(res.measured)} vs expected ${JSON.stringify(res.expected)}${res.detail ? ` (${res.detail})` : ''}: adjust the last feature or fix the expectation`,
      });
    }
  }

  const ok = errors.length === 0 && (validate ? validate.ok : true) && (check ? check.failed === 0 : true);
  return { ok, tag, ms: Math.round(performance.now() - t0), regen: { errors }, validate, check, hints };
}
