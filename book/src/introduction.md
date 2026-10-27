# Introduction

The CadKit kernel is a boundary-representation (B-rep) CAD kernel written in Rust. It compiles to a small WebAssembly
module that runs in a browser worker or in Node, and the same code also runs as a native service. You give it a *part*, a
JSON document that lists the features that build a model, and it rebuilds that part into a solid body with faces, edges,
a display mesh and measurements.

This book describes four things:

- **The part format.** What a `.ckpart` document contains, how features refer to the faces and edges that earlier
  features made, how sketches are solved, and what each feature does.
- **The protocol.** How a host talks to the kernel: the messages it accepts, the binary reply envelope, and every field of
  a regeneration result.
- **The kernel itself.** The crates, the B-rep model, the geometry and operations behind each feature, tessellation, and how
  regeneration is cached.
- **Validation.** How the kernel decides that a body is sound: the topology and geometry checks, the tessellation checks,
  tolerances, healing, and how feature errors are reported so that a failing edit can be understood.

Import and export (STEP, SketchUp, SolidWorks, ACIS SAT, text) and the way the kernel is tested are covered in the last
parts, and the book ends with a crate map and a glossary.

## What the kernel does

A part is a list of features applied in order. A sketch feature holds 2D geometry and constraints; a solid feature such as
`extrude` or `revolve` turns a sketch into material; edit features such as `fillet`, `shell` or `draft` change the body
that exists so far. Regenerating a part runs the list from the top (or up to a chosen feature) and returns the body that
results, together with a status for every feature.

```json
{
  "format": "ck-part",
  "version": 1,
  "features": [
    { "type": "sketch",  "id": "F1", "plane": { "kind": "std", "id": "top" },
      "points": [ { "id": "p1", "x": 0, "y": 0 }, { "id": "p2", "x": 40, "y": 0 },
                  { "id": "p3", "x": 40, "y": 20 }, { "id": "p4", "x": 0, "y": 20 } ],
      "entities": [ { "type": "line", "id": "e1", "p1": "p1", "p2": "p2" },
                    { "type": "line", "id": "e2", "p1": "p2", "p2": "p3" },
                    { "type": "line", "id": "e3", "p1": "p3", "p2": "p4" },
                    { "type": "line", "id": "e4", "p1": "p4", "p2": "p1" } ],
      "constraints": [] },
    { "type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": 10 }
  ]
}
```

That is a complete part: a 40 by 20 rectangle extruded 10 units. [A first part](getting-started/first-part.md) runs it
and reads the result.

The kernel covers sketches with a constraint solver; extrude, revolve, sweep and loft; fillets, chamfers, shells, drafts,
holes and threads; patterns, mirrors and transforms; surfacing features; and assemblies with mates. It reads and writes
STEP and reads SketchUp, SolidWorks and ACIS SAT files.

## Design points worth knowing

- **Everything is data.** A part is plain JSON and a reply is JSON plus binary buffers. A regeneration depends only on the
  part, the options sent with the request, and any fonts loaded into the session; an optional regeneration cache is keyed
  by that content.
- **Deterministic.** The same part regenerated on the same build gives the same result, which is what lets the kernel cache
  and compare results.
- **Errors belong to features.** A feature that fails does not abort the part: the kernel reports an error for that feature
  and goes on with the rest of the list, so one regeneration can return several errors. See
  [feature errors and diagnostics](validation/diagnostics.md).
- **Names are stable.** Faces and edges carry names derived from the feature that made them, so a fillet or a sketch placed on
  a face still finds that face after an upstream edit. See [identifiers and references](format/references.md).

## How to read this book

If you are building a host application, start with [Getting started](getting-started/build.md) and then read
[the protocol](protocol/overview.md). If you are writing or generating parts, read [the part format](format/overview.md)
and keep the [feature index](format/features.md) open. If a body fails a check or a feature reports an error, go to
[validation](validation/overview.md).

Code samples in the book were run against a build of the kernel; the numbers shown are what that build returned. The
runner used for the examples is `book/tools/run.mjs` in the repository.

## Licence

The kernel is released under either the MIT or the Apache-2.0 licence, at your option. Third-party material that ships with
the repository, and its licences, are listed in `THIRD_PARTY.md`.
