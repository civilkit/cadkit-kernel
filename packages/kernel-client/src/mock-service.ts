// mock-service.ts: a stand-in for the local native kernel service (ck-native), for tests of the app side. Node only; never bundled into the app.
// Speaks the same HTTP shape (docs/plan-local-kernel-service.md): GET /version, POST /session, POST /session/<id>/rpc, DELETE /session/<id>,
// checks the bearer token on every request, and answers an rpc with the framed reply bytes. With `wasmPath` every session gets its
// own wasm instance (WasmHost), so the replies are the real kernel's; without it a tiny fake kernel answers.
import http from 'node:http';
import { readFileSync } from 'node:fs';
import type { AddressInfo } from 'node:net';
import { WasmHost } from './wasm-host.js';
import { KERNEL_PROTOCOL } from './protocol.js';
import type { ServiceVersion } from './native.js';

/** Frame a JSON reply the way the kernel does: [u32 json_len][json][pad to 8]. */
export function frameReply(value: unknown): Uint8Array {
  const json = new TextEncoder().encode(JSON.stringify(value));
  const out = new Uint8Array((4 + json.length + 7) & ~7);
  new DataView(out.buffer).setUint32(0, json.length, true);
  out.set(json, 4);
  return out;
}

export interface MockRequest { method: string; path: string; auth: string | undefined; body: string }
export interface RpcOverride { status?: number; body?: Uint8Array | string; delayMs?: number; hang?: boolean }

export interface MockOptions {
  token?: string;
  version?: Partial<ServiceVersion>;
  /** run the real kernel for each session */
  wasmPath?: string;
  port?: number;
  /** how long the server keeps an idle connection open (default 2 minutes; the real service must keep it above the client's idle ping) */
  keepAliveMs?: number;
  /** decide the answer to an rpc by hand (return undefined to let the kernel / fake answer) */
  onRpc?(body: string, session: string, n: number): RpcOverride | undefined | Promise<RpcOverride | undefined>;
}

export interface MockService {
  url: string;
  token: string;
  requests: MockRequest[];
  sessions: Set<string>;
  deleted: string[];
  /** the connections that were cut off by the client while a request was open */
  aborted: number;
  /** drop every open connection and stop listening (the service went away) */
  stop(): Promise<void>;
  /** forget a session (the service restarted or timed it out): its next rpc answers 404 */
  expire(id: string): void;
}

export async function startMockService(o: MockOptions = {}): Promise<MockService> {
  const token = o.token ?? 'test-token-0123456789';
  const mod = o.wasmPath ? new WebAssembly.Module(readFileSync(o.wasmPath)) : null;
  const hosts = new Map<string, WasmHost | null>();
  const sockets = new Set<import('node:net').Socket>();
  const self = { requests: [] as MockRequest[], sessions: new Set<string>(), deleted: [] as string[], aborted: 0 };
  let nextSession = 1;
  const rpcCount = new Map<string, number>();
  const ver: ServiceVersion = { kernel: '0.0.0-mock', protocol: KERNEL_PROTOCOL, git: 'mockgit', build: 'mockbuild', cores: 4, ...o.version };

  const server = http.createServer((req, res) => {
    const chunks: Buffer[] = [];
    req.on('data', (c: Buffer) => chunks.push(c));
    let done = false;
    res.on('close', () => { if (!done) self.aborted++; });
    req.on('end', async () => {
      const body = Buffer.concat(chunks).toString('utf8');
      const path = (req.url ?? '').split('?')[0];
      const auth = req.headers.authorization;
      self.requests.push({ method: req.method ?? '', path, auth, body });
      const send = (status: number, data: Uint8Array | string, type = 'application/octet-stream') => {
        done = true;
        res.writeHead(status, { 'Content-Type': type, 'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Private-Network': 'true' });
        res.end(data);
      };
      if (req.method === 'OPTIONS') {
        done = true;
        res.writeHead(204, { 'Access-Control-Allow-Origin': '*', 'Access-Control-Allow-Headers': 'authorization, content-type', 'Access-Control-Allow-Methods': 'GET, POST, DELETE', 'Access-Control-Allow-Private-Network': 'true' });
        res.end();
        return;
      }
      if (auth !== `Bearer ${token}`) return send(401, 'unauthorized', 'text/plain');
      if (req.method === 'GET' && path === '/version') return send(200, JSON.stringify(ver), 'application/json');
      if (req.method === 'POST' && path === '/session') {
        const id = `s${nextSession++}`;
        self.sessions.add(id);
        hosts.set(id, mod ? new WasmHost(mod) : null);
        return send(200, JSON.stringify({ session: id }), 'application/json');
      }
      const m = /^\/session\/([^/]+)(\/rpc)?$/.exec(path);
      if (!m) return send(404, 'not found', 'text/plain');
      const id = decodeURIComponent(m[1]);
      if (req.method === 'DELETE' && !m[2]) {
        self.deleted.push(id); self.sessions.delete(id); hosts.delete(id);
        return send(200, '', 'text/plain');
      }
      if (req.method === 'POST' && m[2]) {
        if (!self.sessions.has(id)) return send(404, 'unknown session', 'text/plain');
        const n = (rpcCount.get(id) ?? 0) + 1;
        rpcCount.set(id, n);
        const ov = await o.onRpc?.(body, id, n);
        if (ov?.hang) return;
        if (ov?.delayMs) await new Promise((r) => setTimeout(r, ov.delayMs));
        if (ov && (ov.status !== undefined || ov.body !== undefined)) return send(ov.status ?? 200, ov.body ?? '');
        const host = hosts.get(id);
        if (host) {
          try { return send(200, host.callRaw(body)); } catch (e) { return send(500, JSON.stringify({ trap: (e as Error).message }), 'application/json'); }
        }
        const msg = (JSON.parse(body) as { msg?: { type?: string } }).msg ?? {};
        return send(200, frameReply(msg.type === 'ping' ? { ok: false, error: 'unknown request type: ping', ms: 0 } : { ok: true, result: { echo: msg.type ?? null }, ms: 0 }));
      }
      return send(404, 'not found', 'text/plain');
    });
  });
  server.keepAliveTimeout = o.keepAliveMs ?? 120_000;
  server.on('connection', (s) => { sockets.add(s); s.on('close', () => sockets.delete(s)); });
  await new Promise<void>((r) => server.listen(o.port ?? 0, '127.0.0.1', r));
  const port = (server.address() as AddressInfo).port;
  return {
    url: `http://127.0.0.1:${port}`, token, requests: self.requests, sessions: self.sessions, deleted: self.deleted,
    get aborted() { return self.aborted; },
    stop: () => new Promise<void>((r) => { for (const s of sockets) s.destroy(); server.close(() => r()); }),
    expire: (id) => { self.sessions.delete(id); },
  };
}
