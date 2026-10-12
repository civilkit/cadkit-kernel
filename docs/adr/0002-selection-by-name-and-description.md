# ADR 0002: Selecting geometry by persistent name, resolved from description

Status: Accepted for names (implemented); Proposed for description resolution.

## Context

Features refer to faces and edges of earlier features ("fillet these edges",
"sketch on this face"). A person points at them. A program has no pointer, and
both benchmarks and our own experience show that agents most often fail by
picking the wrong face or edge.

Names in the document must survive parameter edits, or references silently
break or, worse, silently point to a different entity.

## Decision

1. **References in the document use persistent names** that are derived from
   how the geometry was made (for example `F2.S.e1`: the side face from sketch
   entity e1 of feature F2; `F2.C0`/`F2.C1`: the end caps; `@` suffixes for
   derived faces). Never positions or indices into a mesh.
2. **Stability is measured, not assumed.** `tools/naming_stability.mjs` edits
   upstream dimensions across 37 cases and checks that every reference still
   resolves to the same kind of geometry. Baseline: 214 of 216 edits stable,
   0 resolved to a wrong entity; the 2 losses are topology-changing edits.
   Keep "wrong entity" at zero and track the metric in the gate.
3. **A resolver turns a description into names** for front ends that cannot
   point. Input: a face or edge description such as
   `{kind: "plane", normal: "+z", extremum: "max", of: "F2"}` or
   `{kind: "cylinder", radius: 5, near: [10, 5, 0]}` or "the edge between
   these two named faces". Output: the matching names with a confidence flag
   and the alternatives if ambiguous. The reply must say when more than one
   entity matches; it never silently picks one.
4. **The inverse is also provided:** given a name, return a human-readable
   description (kind, size, direction, position relative to the body) so a
   person or agent can check that a reference means what they think.
5. **Position hints are secondary.** Where an edge reference carries a world
   position hint, treat it only as a tie-breaker between candidates that are
   otherwise identical in topology.

## Consequences

- Interactive picking records the name at pick time; the viewport highlights
  by name.
- Agents write descriptions, not names, for new references, then store the
  resolved name; stale names produce a structured error naming the lost
  reference and the nearest candidates.
- The known weak point is deleting or reordering an upstream feature, which can
  re-point a name without an error (observed in both this kernel and the
  reference kernel). Add an explicit "references changed" warning for this.

## Open questions

- How to represent "the same entity" across a split (`~1`, `~2` successors) so
  references can follow a split face when that is the intent.
- Whether descriptions need an expression syntax or a small fixed vocabulary.
