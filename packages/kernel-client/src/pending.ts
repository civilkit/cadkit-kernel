// pending.ts: the table of calls in flight on a worker. Once the worker is
// gone (error, exit, messageerror, timeout) every waiting call is answered
// with the same `{ ok: false, trap }` shape a wasm trap uses, and every later
// call is answered at once, so a dead worker never leaves a caller hanging.
import type { KernelReply } from './protocol.js';

export class PendingCalls {
  private readonly waiting = new Map<number, (r: KernelReply) => void>();
  private nextId = 1;
  private deadReason: string | null = null;

  /** Why the worker is unusable, or null while it is alive. */
  get dead(): string | null { return this.deadReason; }
  get size(): number { return this.waiting.size; }

  /** Register a call; returns its id and the promise, or an already-settled promise when dead. */
  open(): { id: number; reply: Promise<KernelReply> } {
    const id = this.nextId++;
    if (this.deadReason !== null) return { id, reply: Promise.resolve({ ok: false, trap: this.deadReason }) };
    return { id, reply: new Promise<KernelReply>((resolve) => { this.waiting.set(id, resolve); }) };
  }

  settle(id: number, reply: KernelReply): void {
    const w = this.waiting.get(id);
    if (!w) return;
    this.waiting.delete(id);
    w(reply);
  }

  /** Mark the worker dead (first reason wins) and answer everything waiting. */
  fail(reason: string): void {
    if (this.deadReason === null) this.deadReason = reason;
    const trap = this.deadReason;
    const all = [...this.waiting.values()];
    this.waiting.clear();
    for (const w of all) w({ ok: false, trap });
  }
}
