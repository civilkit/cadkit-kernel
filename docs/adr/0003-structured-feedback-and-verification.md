# ADR 0003: Structured feedback and independent verification

Status: Accepted; partly implemented.

## Context

A person looks at the viewport and notices a wrong result. A program does not;
it needs the kernel to say what happened, in a form it can act on. And "it
matches the old implementation" is no longer the standard of correctness.

## Decision

1. **Every regen returns per-feature results.** Each feature reports success or
   a structured error `{code, params}` (for example `booleanFailed`,
   `nothingToCut`, `missingReference`, `blendOverflow`). Later features still
   regenerate when they do not depend on the failed one. Error codes are stable
   and documented; messages are never the only carrier of meaning.
2. **Validity is checked independently of the operation that produced the
   geometry.** The `validate` request returns `{ok, issues[{code, entity,
   detail}]}` for closed manifold shells, loop closure, vertex and edge
   consistency, orientation, Euler characteristic and volume sign. It names
   the entity (a persistent name) so a reference to it can be fixed.
3. **Repair is explicit.** `heal` reports what it fixed and what remains;
   `regen` with `autoHeal` is opt-in and records a `healed` warning. A body
   reported clean must still be checked against volume and shape (heal can
   report clean while wrong).
4. **Correctness is measured against independent oracles**, in this order:
   closed-form values (`tools/oracles`), external ground truth from another
   kernel (`tools/bench`), round trips through STEP, and the original kernel as
   a soft reference. The gate separates hard metrics (wire protocol, validity,
   volume, oracles, imports, repairs) from soft ones (identity with the
   reference).
5. **Intent checks.** A caller may attach expectations to a part (volume,
   bounding box, mass, counts of holes) and `regen` or a query reports
   mismatches. This is what lets an agent know a model is right, not merely
   valid.
6. **No traps.** Malformed or extreme input returns a structured error. The
   protocol lint (`tools/lint_protocol.mjs`) fuzzes every request type and is
   a required step before landing.

## Consequences

- Front ends render the same results differently: a person sees highlighted
  features and a message; an agent gets the codes and entity names.
- New features must add error codes, validity coverage and at least one oracle
  or benchmark case before they are considered done.
- Known gaps to track as work: intent checks (item 5) are not implemented;
  `validate` samples curves coarsely (false positives on thin annuli, being
  fixed); validation does not yet cover assemblies.
