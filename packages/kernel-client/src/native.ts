// native.ts: KernelTransport over HTTP to a local native kernel service (crate ck-native, docs/plan-local-kernel-service.md).
//
//   GET    /version              -> { kernel, protocol, git, build, cores }   (field names read tolerantly, see readVersion)
//   POST   /session              -> { session: id }
//   POST   /session/<id>/rpc     body = the same text the wasm kernel gets ({id, msg}); reply = the framed reply bytes, unchanged
//   DELETE /session/<id>
//
// Every request carries `Authorization: Bearer <token>` (never a URL parameter, never logged, never in an error message).
// Works in the browser and in Node (global fetch); no Node imports, so the app bundle can use it.
//
// Failure shape: the same as the wasm transport, `{ ok: false, trap }` for a crash or a transport failure and `{ ok: false, error }`
// for a refusal. A failure that makes the session unusable (service gone, token rejected, session expired, timeout) is also reported
// once through `onFailure`, so the app can switch to the built-in kernel; the session is never silently continued on another kernel.
import { decodeReply, encodeRequest, toKernelReply, KERNEL_PROTOCOL, type KernelReply } from './protocol.js';

export interface ServiceVersion {
  /** the kernel crate version, e.g. "0.1.0" */
  kernel: string;
  protocol: number;
  /** git hash the service was built from */
  git: string;
  /** the build id that identifies the kernel build (equals the wasm's commit string when both come from the same source) */
  build: string;
  cores: number;
}

export interface ExpectedVersion {
  protocol: number;
  /** the built-in kernel's build id (the wasm `commit`), when known: a service built from a different build is refused */
  build?: string;
}

export type FailureKind = 'unreachable' | 'unauthorized' | 'expired' | 'timeout' | 'bad-reply';

export interface NativeFailure {
  kind: FailureKind;
  /** human-readable, safe to show (contains no token) */
  message: string;
}

export interface NativeOptions {
  /** base address, e.g. "http://127.0.0.1:7461" (a bare "127.0.0.1:7461" is read as http) */
  url: string;
  token: string;
  /** the version this client was built against; connect() refuses a service that differs (default: only the protocol number) */
  expect?: ExpectedVersion;
  /** a request that takes longer than this is aborted and reported as a timeout (default 5 minutes; 0 = none) */
  timeoutMs?: number;
  /** a small request every this many ms while idle keeps the service from expiring the session (default 30 s; 0 = off) */
  keepAliveMs?: number;
  fetch?: typeof fetch;
  onFailure?(f: NativeFailure): void;
}

export type ProbeResult =
  | { ok: true; version: ServiceVersion }
  | { ok: false; kind: 'unreachable' | 'unauthorized' | 'invalid' | 'mismatch'; message: string; version?: ServiceVersion };

/** "127.0.0.1:7461" -> "http://127.0.0.1:7461"; trailing slashes dropped. Throws on a value that is not an http(s) address. */
export function normalizeServiceUrl(raw: string): string {
  let s = raw.trim();
  if (!s) throw new Error('empty service address');
  if (!/^[a-z][a-z0-9+.-]*:\/\//i.test(s)) s = 'http://' + s;
  const u = new URL(s);
  if (u.protocol !== 'http:' && u.protocol !== 'https:') throw new Error('the service address must be http or https');
  if (u.username || u.password) throw new Error('the service address must not contain credentials');
  return u.origin + u.pathname.replace(/\/+$/, '');
}

const isLoopback = (url: string): boolean => { try { return /^(127\.|localhost$|\[::1\]$)/.test(new URL(url).hostname); } catch { return false; } };

function headers(token: string, extra?: Record<string, string>): Record<string, string> {
  return { Authorization: `Bearer ${token}`, ...extra };
}

/** fetch options common to every call: cors, no cookies, no cache, and the hint Chrome's Local Network Access check wants. */
function baseInit(url: string, token: string, extra?: Record<string, string>): RequestInit {
  const init: Record<string, unknown> = { mode: 'cors', credentials: 'omit', cache: 'no-store', headers: headers(token, extra) };
  if (isLoopback(url)) init.targetAddressSpace = 'local';
  return init as RequestInit;
}

const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : typeof v === 'string' && /^\d+$/.test(v) ? Number(v) : null);
const str = (v: unknown): string => (typeof v === 'string' ? v : typeof v === 'number' ? String(v) : '');

/** Read the /version document. Accepts camelCase or snake_case names; null when it does not carry a protocol number. */
export function readVersion(j: unknown): ServiceVersion | null {
  if (!j || typeof j !== 'object') return null;
  const o = j as Record<string, unknown>;
  const protocol = num(o.protocol ?? o.protocolVersion ?? o.protocol_version);
  if (protocol === null) return null;
  return {
    kernel: str(o.kernel ?? o.kernelVersion ?? o.kernel_version ?? o.version),
    protocol,
    git: str(o.git ?? o.gitHash ?? o.git_hash),
    build: str(o.build ?? o.buildId ?? o.build_id),
    cores: num(o.cores) ?? 0,
  };
}

const lastToken = (s: string): string => s.trim().split(/\s+/).pop() ?? '';

/** Whether a build id the service reports names the same build as the wasm's commit string ("<branch> <40-hex sha>"): equal, or
 *  hashes where one is a prefix of the other (the service may report a short hash). */
export function sameBuild(id: string, commit: string): boolean {
  if (id === commit) return true;
  const a = lastToken(id), b = lastToken(commit);
  return a.length >= 7 && b.length >= 7 && /^[0-9a-f]+$/i.test(a) && /^[0-9a-f]+$/i.test(b) && (a.toLowerCase().startsWith(b.toLowerCase()) || b.toLowerCase().startsWith(a.toLowerCase()));
}

/** null when the service may be used, otherwise why not. */
export function versionMismatch(v: ServiceVersion, expected: ExpectedVersion): string | null {
  if (v.protocol !== expected.protocol) {
    return `protocol ${v.protocol} (this app speaks ${expected.protocol})`;
  }
  const ids = [v.build, v.git].filter(Boolean);
  if (expected.build && ids.length && !ids.some((id) => sameBuild(id, expected.build!))) {
    return `kernel build ${v.build || v.git} (this app was built with ${lastToken(expected.build).slice(0, 12)})`;
  }
  return null;
}

/** GET /version and compare with what this client expects. Never throws; the message is safe to show. */
export async function probeService(o: Pick<NativeOptions, 'url' | 'token' | 'expect' | 'fetch'>): Promise<ProbeResult> {
  let base: string;
  try { base = normalizeServiceUrl(o.url); } catch (e) { return { ok: false, kind: 'invalid', message: (e as Error).message }; }
  const f = o.fetch ?? fetch;
  let res: Response;
  try {
    res = await f(base + '/version', { ...baseInit(base, o.token), signal: AbortSignal.timeout(5000) });
  } catch {
    return { ok: false, kind: 'unreachable', message: 'no answer from the service' };
  }
  if (res.status === 401 || res.status === 403) return { ok: false, kind: 'unauthorized', message: 'the service refused the token' };
  if (!res.ok) return { ok: false, kind: 'invalid', message: `the service answered ${res.status}` };
  let version: ServiceVersion | null = null;
  try { version = readVersion(await res.json()); } catch { /* not json */ }
  if (!version) return { ok: false, kind: 'invalid', message: 'the address does not look like a kernel service' };
  const why = versionMismatch(version, o.expect ?? { protocol: KERNEL_PROTOCOL });
  if (why) return { ok: false, kind: 'mismatch', message: why, version };
  return { ok: true, version };
}

const REFUSED = (trap: string): KernelReply => ({ ok: false, trap });

export class NativeKernel {
  readonly url: string;
  private readonly token: string;
  private readonly f: typeof fetch;
  private readonly timeoutMs: number;
  private readonly onFailure?: (f: NativeFailure) => void;
  private readonly keepAliveMs: number;
  private sessionP: Promise<string> | null = null;
  private sessionId: string | null = null;
  private nextId = 1;
  private chain: Promise<unknown> = Promise.resolve();
  private readonly inflight = new Set<{ ac: AbortController; why: 'cancel' | 'timeout' | null }>();
  private epoch = 0;   // bumped by cancel(): requests queued before it are dropped
  private failed: NativeFailure | null = null;
  private closed = false;
  private timer: ReturnType<typeof setInterval> | null = null;
  private lastUse = 0;
  version: ServiceVersion | null = null;

  /** Use connect() unless the version was checked already. */
  constructor(o: NativeOptions) {
    this.url = normalizeServiceUrl(o.url);
    this.token = o.token;
    this.f = o.fetch ?? fetch;
    this.timeoutMs = o.timeoutMs ?? 5 * 60_000;
    this.keepAliveMs = o.keepAliveMs ?? 30_000;
    this.onFailure = o.onFailure;
  }

  /** Check the service (version, token), then return a transport. The session itself opens on the first request. */
  static async connect(o: NativeOptions): Promise<NativeKernel> {
    const p = await probeService(o);
    if (!p.ok) throw new NativeConnectError(p.kind, p.message);
    const k = new NativeKernel(o);
    k.version = p.version;
    return k;
  }

  /** The build id of the service (what a status display shows next to the kernel version). */
  get commit(): string { return this.version?.build || this.version?.git || ''; }

  get failure(): NativeFailure | null { return this.failed; }

  private fail(kind: FailureKind, message: string): KernelReply {
    if (!this.failed) {
      this.failed = { kind, message };
      this.stopKeepAlive();
      if (!this.closed) this.onFailure?.(this.failed);
    }
    return REFUSED(`local kernel service: ${message}`);
  }

  private openSession(): Promise<string> {
    return (this.sessionP ??= (async () => {
      const res = await this.f(this.url + '/session', { ...baseInit(this.url, this.token), method: 'POST' });
      if (res.status === 401 || res.status === 403) throw new SessionError('unauthorized', 'the service refused the token');
      if (!res.ok) throw new SessionError('unreachable', `could not open a session (the service answered ${res.status})`);
      const j = (await res.json()) as { session?: unknown };
      const id = typeof j.session === 'string' || typeof j.session === 'number' ? String(j.session) : '';
      if (!id) throw new SessionError('bad-reply', 'the service did not return a session id');
      this.sessionId = id;
      this.startKeepAlive();
      return id;
    })());
  }

  /** One protocol message. Requests are sent one at a time in call order, like the single wasm worker. */
  request(msg: Record<string, unknown>): Promise<KernelReply> {
    const epoch = this.epoch;
    const run = async (): Promise<KernelReply> => {
      if (this.failed) return REFUSED(`local kernel service: ${this.failed.message}`);
      if (this.closed) return REFUSED('local kernel service: closed');
      if (epoch !== this.epoch) return { ok: false, error: 'cancelled' };
      return this.send(msg, epoch);
    };
    const p = this.chain.then(run, run);
    this.chain = p.catch(() => {});
    return p;
  }

  private async send(msg: Record<string, unknown>, epoch: number): Promise<KernelReply> {
    let sid: string;
    try { sid = await this.openSession(); } catch (e) {
      this.sessionP = null;
      if (epoch !== this.epoch) return { ok: false, error: 'cancelled' };
      return e instanceof SessionError ? this.fail(e.kind, e.message) : this.fail('unreachable', 'no answer from the service');
    }
    const tok = { ac: new AbortController(), why: null as 'cancel' | 'timeout' | null };
    this.inflight.add(tok);
    const timer = this.timeoutMs > 0 ? setTimeout(() => { tok.why = 'timeout'; tok.ac.abort(); }, this.timeoutMs) : null;
    this.lastUse = Date.now();
    try {
      const res = await this.f(`${this.url}/session/${encodeURIComponent(sid)}/rpc`, {
        ...baseInit(this.url, this.token, { 'Content-Type': 'application/json' }),
        method: 'POST', body: encodeRequest(this.nextId++, msg), signal: tok.ac.signal,
      });
      if (res.status === 401 || res.status === 403) return this.fail('unauthorized', 'the service refused the token');
      if (res.status === 404) return this.fail('expired', 'the service no longer has this session (restarted or timed out)');
      const bytes = new Uint8Array(await res.arrayBuffer());
      if (!res.ok) {
        // a refusal of this one request (too large, too many requests, a crash inside the kernel): the session goes on
        const text = new TextDecoder().decode(bytes.subarray(0, 300)).replace(/\s+/g, ' ').trim();
        const crash = res.status >= 500;
        return crash ? REFUSED(`kernel crashed (${res.status})${text ? ': ' + text : ''}`) : { ok: false, error: `the service refused the request (${res.status})${text ? ': ' + text : ''}` };
      }
      try {
        return toKernelReply(decodeReply(bytes).value as { ok?: unknown; result?: unknown; error?: unknown });
      } catch {
        return this.fail('bad-reply', 'the service sent a reply that cannot be read');
      }
    } catch {
      if (tok.why === 'cancel') return { ok: false, error: 'cancelled' };
      if (tok.why === 'timeout') return this.fail('timeout', `no reply within ${Math.round(this.timeoutMs / 1000)} s`);
      return this.fail('unreachable', 'the connection to the service was lost');
    } finally {
      if (timer) clearTimeout(timer);
      this.inflight.delete(tok);
    }
  }

  private startKeepAlive(): void {
    if (this.keepAliveMs <= 0 || this.timer) return;
    this.timer = setInterval(() => {
      if (this.failed || this.closed || Date.now() - this.lastUse < this.keepAliveMs) return;
      // any request counts as use; an unknown type is the cheapest reply the kernel gives
      void this.request({ type: 'ping' });
    }, this.keepAliveMs);
    (this.timer as { unref?: () => void }).unref?.();
  }
  private stopKeepAlive(): void { if (this.timer) { clearInterval(this.timer); this.timer = null; } }

  /** Abort what is in flight and drop what is queued (a cancelled preview). The session stays open on the service. */
  cancel(): void {
    this.epoch++;
    for (const t of this.inflight) { t.why = 'cancel'; t.ac.abort(); }
  }

  /** Abort everything and delete the session on the service (best effort). */
  async close(): Promise<void> {
    if (this.closed) return;
    this.closed = true;
    this.stopKeepAlive();
    this.cancel();
    const sid = this.sessionId;
    this.sessionId = null;
    if (!sid) return;
    try {
      await this.f(`${this.url}/session/${encodeURIComponent(sid)}`, { ...baseInit(this.url, this.token), method: 'DELETE', keepalive: true, signal: AbortSignal.timeout(3000) });
    } catch { /* the service may be gone already */ }
  }
}

class SessionError extends Error {
  constructor(readonly kind: FailureKind, message: string) { super(message); }
}

/** connect() could not use the service. `kind` is one of the ProbeResult kinds. */
export class NativeConnectError extends Error {
  constructor(readonly kind: 'unreachable' | 'unauthorized' | 'invalid' | 'mismatch', message: string) { super(message); this.name = 'NativeConnectError'; }
}

export { KERNEL_PROTOCOL };
