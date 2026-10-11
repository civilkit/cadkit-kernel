# ck-geom

Curve and surface geometry: the analytic + NURBS layer between math (ck-core)
and topology (ck-topo). No dependencies.

## Modules

| module | contents |
|---|---|
| `curve.rs` | 3D curves: line, circle, ellipse, conic (focus form), sinusoid, (rational) bezier/bspline segments; param, eval, deriv, closest point, split |
| `curve2.rs` | 2D curves (sketch space, pcurve space): line, circle, ellipse, bspline |
| `pcurve.rs` | curves on surfaces (parameter-space curves) |
| `surface.rs` | analytic surfaces: plane, cylinder, cone, sphere, torus + surface of extrusion/revolution, offset (exact analytic derivatives) + B-spline (STEP writer strings in kernel.strings confirm this exact set) |
| `fit.rs` | curve/surface fitting (splines through points, smooth joins) |
| `analysis.rs` | curvature, derivatives, frame propagation |
| `convert.rs` | conversions between representations (analytic <-> NURBS) |
| `nurbs/basis.rs` | B-spline basis functions |
| `nurbs/bspline.rs` | B-spline curves |
| `nurbs/bsurf.rs` | B-spline surfaces |
| `nurbs/rational.rs` | rational (NURBS) weights handling |

## Interface

- Eval returns point + derivatives; classify returns the analytic `kind`
  strings the wire format uses: `plane, cylinder, cone, torus, sphere, spline`
  (see oracle body.faces[].kind).
- Face/edge ids reference geometry owned by ck-topo; this crate is
  representation-only.

## Rebuild notes

- The golden flange part exercises ALL face kinds: 82 plane, 38 cylinder,
  40 cone, 6 torus, 16 spline — use it as the fitting test.
- Error strings (kernel.strings) name the operations: posEllipse, tension,
  meowNURBS over a plane — module hints.
- Curve-kind tag run `line circle ellipse spline curve conic sinusoid
  pcurve`: `conic` and `sinusoid` have no probeable wire record (the oracle
  emits conic fillet sections only as sampled pts), so they are
  GUESS-parameterised — conic in focus form r = p/(1+e·cos t) (covers the
  parabola/hyperbola gap), sinusoid as a sine graph in a frame plane.
- Offsets are exact: `Surface::derivs2` differentiates the unit normal from
  the cross-product field using `Surface::thirds` (third partials, exact
  for analytic surfaces and B-splines). Oracle-pinned by `curvatureAt` on
  `surfaceOffset` faces (offset cylinder R=10 +2 -> k1=1/12).
