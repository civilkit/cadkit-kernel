# The sketch solver

The sketch solver moves the points of a sketch until its constraints hold. This chapter describes when the kernel runs it, how it is built, how it reports whether a sketch is fully defined, over-constrained or in conflict, and the read-only queries that expose more of its state.

## When it runs

A regen does not solve a sketch on its own: the stored coordinates are used as written (see [sketches](sketches.md)). The solver runs in two cases:

1. A dimensional constraint has an `expr` whose value differs from its stored `value` by more than 1e-12. The kernel evaluates the expression, writes the new `value` and solves the sketch starting from the stored coordinates. If the solve does not converge the coordinates are left as they were and no error is raised.
2. The regen request has `opts.solveSketches: true`. Every sketch is solved with all its constraints, and a sketch that already satisfies them is not solved again. This is the mode for an editor that drags a point: a constraint `fix` with `value: [x, y]` acts as a temporary pin. The solved coordinates are used for the reply and for the features that follow.

The queries `sketchFreedoms` and `sketchAnalyze` below only analyse the sketch as stored; they move nothing.

## The equation system

- **Unknowns** are the `x` and `y` of every point that is not `fixed`, then the radius `r` of every circle that is not marked `ext`/`from` and of every ellipse, in document order.
- **Equations** are one per arc (the radii `c`-`p1` and `c`-`p2` must agree), then the residuals of each constraint in document order. A constraint with `suppressed: true` contributes none. A constraint contributes one equation (a distance, an angle, a radius), two (coincident points, a fixed point, a collinear pair, symmetry) or several (spline constraints).
- A constraint that names something that does not exist, or has the wrong `refs`, makes the whole system impossible to build. The sketch is then left unsolved without an error.

## The solve

The solver is a damped Gauss-Newton iteration on the minimum-norm step:

- the Jacobian is taken by central differences with step `1e-7 * (1 + |x|)`;
- each iteration solves `(J W J^T + lambda I) y = -f` for the step `dx = W J^T y`, with `lambda = 1e-10 * max diagonal + 1e-14`; `W` is an optional per-unknown weight that makes chosen unknowns resist moving (an editor uses it for the point being dragged);
- the step is halved up to 12 times until the sum of squared residuals decreases;
- at most 60 iterations; it stops when the largest residual is at most `1e-9`, and counts as solved when it is at most `1e-7`.

Because the step is the smallest that fixes the residuals, an under-constrained sketch moves as little as it can: points stay near where they were drawn.

## Status: fully defined, under, over

With `opts.sketchStatus: true` (implied by `solveSketches`) every entry of `sketches` in the reply gains a `status`:

| Field | Meaning |
|---|---|
| `dof` | Degrees of freedom: the number of unknowns minus the rank of the Jacobian. A free point counts 2, a free circle 3. |
| `fullyDefined` | `state` is `"full"` and `solved` is true. |
| `state` | `"over"` if some constraint is implied by earlier ones, otherwise `"full"` when `dof` is 0, otherwise `"under"`. |
| `solved` | The stored coordinates satisfy every constraint (largest residual at most 1e-7), or a solve from them converges. |
| `conflicts` | Ids of constraints that cannot hold together with the ones before them. |
| `redundant` | Ids of constraints that are implied by the ones before them. |
| `residual` | The largest absolute residual at the coordinates in the reply; `-1` if it is not finite or the system cannot be built. |
| `suppressed` | Ids of constraints flagged `suppressed`, in document order. Present only when there are any. |

How blame is assigned: the rows of the Jacobian are taken in solver order and orthogonalised one after the other against the rows already accepted. A row is independent when the part of it outside the span of the earlier rows is more than 1e-6 of its own norm. A dependent row belongs to a constraint that is either `redundant` (its residual at the solution is 1e-6 or less, so it holds anyway) or a `conflict` (it does not hold). The later constraint in document order is always the one blamed, and a constraint is never in both lists. Suppressing the constraint that is blamed removes the conflict.

A draft whose coordinates do not satisfy its constraints reports `solved: true` with a positive `residual` when a solve from them converges, and `solved: false` when the constraints cannot be met.

A fully defined sketch from the previous chapter (regen with `opts.sketchStatus: true`):

```json
{
  "format": "ck-part", "version": 1, "variables": [{"name": "w", "expr": "30"}],
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "O", "x": 0, "y": 0, "fixed": true},
        {"id": "b", "x": 30, "y": 0},
        {"id": "c", "x": 30, "y": 15},
        {"id": "d", "x": 0, "y": 15}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "O", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "O"}
      ],
      "constraints": [
        {"id": "k1", "type": "horizontal", "refs": ["e1"]},
        {"id": "k2", "type": "vertical", "refs": ["e2"]},
        {"id": "k3", "type": "horizontal", "refs": ["e3"]},
        {"id": "k4", "type": "vertical", "refs": ["e4"]},
        {"id": "k5", "type": "distance", "refs": ["O", "b"], "value": 30, "expr": "=w"},
        {"id": "k6", "type": "distance", "refs": ["b", "c"], "value": 15}
      ]
    }
  ]
}
```

Run result: no errors; no solid body. `status` of the sketch: `{"dof":0,"fullyDefined":true,"conflicts":[],"redundant":[],"solved":true,"state":"full","residual":0}`.


Two distances between the same points, one asking for 30 mm and one for 40 mm (regen with `opts.sketchStatus: true`):

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "O", "x": 0, "y": 0, "fixed": true},
        {"id": "b", "x": 30, "y": 0},
        {"id": "c", "x": 30, "y": 15},
        {"id": "d", "x": 0, "y": 15}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "O", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "O"}
      ],
      "constraints": [
        {"id": "k1", "type": "distance", "refs": ["O", "b"], "value": 30},
        {"id": "k2", "type": "distance", "refs": ["O", "b"], "value": 40}
      ]
    }
  ]
}
```

Run result: no errors; no solid body. `status` of the sketch: `{"dof":5,"fullyDefined":false,"conflicts":["k2"],"redundant":[],"solved":false,"state":"over","residual":10}`.


The second constraint is blamed. The sketch stays as it was sent and the regen has no error: conflicting constraints never raise a feature error.

### Live solve

Starting from a sketch whose point `b` is 1 mm off the horizontal line that `k1` demands, `solveSketches` moves it onto the line (and then fits the distance of `k5`). Regen with `opts.solveSketches: true` of:

```json
{
  "format": "ck-part", "version": 1,
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [
        {"id": "O", "x": 0, "y": 0, "fixed": true},
        {"id": "b", "x": 30, "y": 1},
        {"id": "c", "x": 30, "y": 15},
        {"id": "d", "x": 0, "y": 15}
      ],
      "entities": [
        {"type": "line", "id": "e1", "p1": "O", "p2": "b"},
        {"type": "line", "id": "e2", "p1": "b", "p2": "c"},
        {"type": "line", "id": "e3", "p1": "c", "p2": "d"},
        {"type": "line", "id": "e4", "p1": "d", "p2": "O"}
      ],
      "constraints": [
        {"id": "k1", "type": "horizontal", "refs": ["e1"]},
        {"id": "k5", "type": "distance", "refs": ["O", "b"], "value": 30}
      ]
    }
  ]
}
```

Run result: no errors; no solid body. `b` becomes (30, 0) in the reply and `status` is `{"dof":4,"fullyDefined":false,"conflicts":[],"redundant":[],"solved":true,"state":"under","residual":6.306066779870889e-12}`.


Without the option, the same sketch comes back unchanged.

## Queries

Both queries work on the tag of a previous regen (`tag`), or on a part passed in `part`, and reply `null` when there is no body and no `part`.

### `sketchFreedoms`

What can still move in a sketch: the null space of the Jacobian at the stored coordinates, per point, radius and entity. `dof` is the same number as in `status`.

```json
{"type": "sketchFreedoms", "featureId": "F1", "tag": "t"}
```

Reply:

```json
{
  "dof": 1, "solved": true,
  "points": [
    {"id": "O", "free": 0},
    {"id": "b", "free": 0},
    {"id": "c", "free": 1, "dir": [0, 1]},
    {"id": "d", "free": 1, "dir": [0, 1]}
  ],
  "radii": [],
  "entities": [
    {"id": "e1", "kind": "line", "free": 0},
    {"id": "e2", "kind": "line", "free": 1},
    {"id": "e3", "kind": "line", "free": 2},
    {"id": "e4", "kind": "line", "free": 1}
  ]
}
```


For a point, `free` is 0 for a fixed or fully determined point, 1 when it can move along the single direction `dir` (a unit vector in sketch coordinates, with the larger component positive), and 2 when it can move anywhere. For an entity `free` is the sum over its points plus 1 for a free radius. A rectangle with its height missing leaves the top corners free in y only.

### `sketchAnalyze`

A structural analysis of one sketch, for diagnosing profiles that do not close. The options `tolerance` (near-coincident distance, default 0.01 mm) and `tiny` (tiny segment length, default 0.001 mm) tune it; a value of 0 or less switches the check off.

```json
{"type": "sketchAnalyze", "featureId": "F1"}
```

Reply:

```json
{
  "tolerance": 0.01, "tiny": 0.001,
  "findings": [
    {
      "kind": "openEnd", "entities": ["e1"], "points": ["e", "a"], "at": [0, 0.02], "gap": 0.02,
      "nearest": "e4"
    },
    {
      "kind": "openEnd", "entities": ["e4"], "points": ["a", "e"], "at": [0, 0], "gap": 0.02,
      "nearest": "e1"
    }
  ],
  "chains": [{"entities": ["e1", "e2", "e3", "e4"], "closed": false, "construction": false}],
  "profile": {
    "closedChains": 0,
    "reason": {
      "kind": "openEnd", "entities": ["e4"], "points": ["a", "e"], "at": [0, 0], "gap": 0.02,
      "nearest": "e1"
    }
  }
}
```


Finding kinds: `openEnd` (an end touches nothing; `gap` is the distance to the nearest other open end and `nearest` its entity), `nearCoincident` (two points closer than `tolerance` with no coincident constraint between them), `duplicate`, `overlap` (collinear lines whose interiors intersect), `zeroLength`, `tinySegment`, `branch` (more than two ends at a point), `isolatedPoint` and `selfIntersection`. `chains` lists the connected pieces and whether each is closed. `profile.closedChains` counts the closed chains a profile can be built from; when it is 0, `profile.reason` names the biggest `openEnd` gap, `{"kind": "empty"}` for a sketch with no curves, or `{"kind": "constructionOnly"}`.

## Limits

- The spline constraints `g1`, `g2`, `g3`, `onCurve` and `tension` work on open control-point splines only.
- Text entities are fixed geometry; only their anchor point can be constrained.
- The solver reports no feature error for any outcome. A client that needs to know must ask for the `status`.
- The analysis is numerical: the rank test uses a relative threshold, so a nearly singular sketch can be reported as redundant or as under-defined by a small margin.

## See also

- [Sketches](sketches.md)
- [Variables and expressions](expressions.md)
- [Regeneration](../protocol/regen.md)
- [Queries](../protocol/queries.md)
