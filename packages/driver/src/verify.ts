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
  missingAxis: (f) => `${f}.axis is missing — pass a sketch line entity id (the builder defaults to the sketch's first line)`,
  missingReference: (f) => `${f} references geometry that no longer exists — re-pick the face/edge by name (resolve) and update the ref`,
  nothingToCut: (f) => `${f} removes nothing — move the profile so it overlaps the body, or flip the cut direction`,
  cutNoEffect: (f) => `${f} cuts nothing — the tool misses the body: flip it or move it to overlap`,
  booleanFailed: (f, p) =>
    typeof p.hostFace === 'string' && typeof p.toolFace === 'string'
      ? `${f}: boolean failed — the tool wall ${p.toolFace} and the body wall ${p.hostFace} are nearly the same surface (${typeof p.gap === 'number' ? p.gap.toPrecision(2) : '?'} mm apart): make the tool outline exactly that wall (project the body edge of the sketch spline, not a re-fitted copy) or move it clear of the wall; cutting only the inner region leaves the wall out`
      : Array.isArray(p.faces) && p.faces.length
        ? `${f}: boolean failed — the result could not be closed around ${p.faces.slice(0, 4).join(', ')}${Array.isArray(p.at) ? ` (near ${p.at.map((x) => (typeof x === 'number' ? +x.toFixed(2) : x)).join(', ')})` : ''}${typeof p.issue === 'string' ? ` [${p.issue}]` : ''}: those faces touch tangentially or lie within ~1e-3 of each other along a rim — move the tool by a small amount (0.01), unite the bodies that touch each other first, or make the near-coincident faces exactly coincident`
        : `${f}: boolean failed — check the tool and target overlap in a volume, not just a face`,
  shellTooThick: (f, p) =>
    typeof p.detail === 'string' && typeof p.maxThickness === 'number'
      ? `${f}: ${p.detail} — reduce thickness to ${p.maxThickness} mm or less`
      : `${f}: thickness exceeds the thinnest wall — reduce thickness`,
  shellFailed: (f, p) =>
    typeof p.detail === 'string' ? `${f}: ${p.detail}` : `${f}: the shell cavity could not be cut — try a smaller thickness or open another face`,
  moveFaceFailed: (f, p) =>
    typeof p.detail === 'string'
      ? `${f}: ${p.detail}${typeof p.maxThickness === 'number' && p.maxThickness > 0 ? ` — move by at most ${p.maxThickness} mm` : ''}`
      : `${f}: the faces could not be moved — select tangent neighbours too or reduce the distance`,
  shellUnsupported: (f) => `${f}: this shape cannot be shelled — remove the feature or change the base solid`,
  holeNoPoints: (f) => `${f}: hole sketch has no centres — add {type:'point'} entities over centre points (sk.point(x, y)), not circles or bare points`,
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
  projectCurveNoInput: (f) => `${f}: nothing to project — pick a sketch with curves, or a planar curve`,
  projectCurveNoFaces: (f) => `${f}: no faces to project onto — add a body first`,
  projectCurveSeveral: (f, p) => `${f}: the projection falls into ${typeof p.count === 'number' ? p.count : 'several'} separate pieces — pick the faces of one side, set a direction, or draw the sketch so that it lands on one connected stretch of the faces`,
  projectCurvePlane: (f) => `${f}: the curve does not lie in a plane (or is straight), so it has no direction to project along — pick a sketch or a planar curve`,
  projectCurveFailed: (f) => `${f}: the projection could not be computed — try other faces or a simpler sketch`,
  mirrorOntoItself: (f) => `${f}: mirror plane maps the body onto itself — move the plane or seeds`,
  referenceChanged: (f) => `${f}: a referenced entity changed shape after an edit — re-resolve the ref`,
  referenceLost: (f) => `${f}: a referenced entity is gone after an edit — pick a replacement ref`,
};

export function hintFor(feature: string, code: string, params: Record<string, unknown>): RepairHint {
  const fix = FIX[code];
  return { feature, code, hint: fix ? fix(feature, params) : `${feature || 'part'}: error ${code}${Object.keys(params).length ? ` ${JSON.stringify(params)}` : ''} — see the features field table` };
}

/** Every coded fix, sampled for display (powers the authoring guide). */
export function listFixes(): Array<{ code: string; sample: string }> {
  return Object.keys(FIX).map((code) => ({ code, sample: FIX[code]('Fn', {}) }));
}

/** Face/edge entity name -> owning feature id (F2.C0 -> F2, P.C0 -> P), else the raw name. */
function ownerOf(entity: string): string {
  const head = entity.split('.')[0];
  return /^\w+$/.test(head) ? head : entity;
}

export async function verify(
  kernel: KernelTransport,
  part: PartDoc,
  opts: VerifyOpts = {},
): Promise<VerifyReport> {
  const t0 = performance.now();
  const tag = opts.tag ?? 'main';
  const tol = opts.tol ?? 1e-6;
  // Intent checks run by default: explicit expect wins, else the document's own.
  const expect = opts.expect ?? part.expect ?? [{ kind: 'valid' }];
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
  const body = (r.result as { body?: { empty?: boolean } | null }).body ?? null;
  // No body means nothing to check: never report "done" on an empty part.
  const noBody = !body || body.empty === true;
  if (noBody) hints.push({ feature: '', code: 'noBody', hint: 'regen produced no body — the document is empty or every solid feature failed: add or fix a solid feature' });

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

  const ok = !noBody && errors.length === 0 && (validate ? validate.ok : true) && (check ? check.failed === 0 : true);
  return { ok, tag, ms: Math.round(performance.now() - t0), regen: { errors }, validate, check, hints };
}
