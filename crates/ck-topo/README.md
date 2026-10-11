# ck-topo

B-rep topology: the half-edge-style solid modeler core. Depends on ck-geom,
ck-core.

## Modules

| module | contents |
|---|---|
| `body.rs` | the Body: lumps -> faces -> (wires of) coedges -> edges -> vertices; face adjacency; id/key minting (`F4.S.e1@F1.1`, lump indices in oracle) |
| `make.rs` | primitive construction: extrude, revolve (+ make/revolve.rs), from sketches |
| `edit.rs` | local topology edits (imprint, split, delete face, close) |
| `orient.rs` | consistency: outward normals, shell orientation, ccw loops |
| `region.rs` | lump/region classification, containment (inside/outside queries) |
| `check.rs` | validity: open edges, non-manifold, degenerate |
| `mass.rs` | volume, area, centroids, inertia (massProps message) |

## Interface

- Faces carry surface (ck-geom) + boundary loops; the wire `body.faces[]`
  entries (id, kind, planar, meta{feature,entity}, lump) come from here.
- Edge keys (`F2.S.e12@F1.1|F4.S.e1@F1.1#N` in the oracle) = sorted pair of
  face ids + sequence — the exact key format is pinned by the golden.

## Rebuild notes

- The oracle golden's face/edge/lump structure for the flange coupling is
  the reference B-rep. Face counts: 182 faces, 9 lumps for the assembly.
