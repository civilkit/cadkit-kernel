# Equation curves: a sketch spline from x(t) and y(t)

A curve given by formulas: `x(t)` and `y(t)` over a range of `t`, fitted to within a tolerance by the
sketch's interpolating spline. The document stores the formulas; every regen evaluates them again, so a
change of a part variable moves the curve and an agent only has to write the formula text.
Code: `ck-model/src/sketch/equation.rs`. The formulas use the expression evaluator of dimension fields
(`ck-model/src/expr.rs`); there is no second evaluator.

## Entity (wire)

An ordinary `spline` entity with an `equation` object:

```json
{ "type": "spline", "id": "e1", "pts": [],
  "equation": { "x": "R*cos(t)", "y": "R*sin(t)", "t": [0, 360], "tol": 0.01 } }
```

| Field | Meaning |
|---|---|
| `x`, `y` | formulas of `t` (a leading `=` is allowed). The grammar of dimension expressions: `+ - * / ^`, `sqrt sin cos tan abs`, `min max pow mod hypot atan2 round floor ceil trunc exp ln log10 asin acos atan`, `pi`, units on numbers (`5 mm`, `1 in`). **Trigonometry is in degrees**: a full turn is `t` from 0 to 360 |
| `t` | `[start, end]`, each a number or a formula of the part's variables (`"=360*turns"`); `end` may be less than `start` (the curve runs the other way) |
| `tol` | largest distance in mm between the formula's curve and the spline, default 0.01, allowed 0.00001 to 10 |

Names: `t` is the curve's variable (it hides a part variable called `t`); any other name must be a
part variable (`part.variables`). The result of a formula is in mm.

## What regen does

1. Evaluates the formulas, checks them, and fits the spline (below).
2. Removes the sketch points named `<id>_start`, `<id>_p<n>`, `<id>_end`, whatever their coordinates, and
   adds the fitted points under those names as **fixed** points: `<id>_start` is the first, `<id>_end`
   the last of an open curve (a closed one has no `<id>_end`). The entity's `pts` and `closed` are
   rewritten to match. A line can end on `<id>_start` / `<id>_end` (write the id as its `p1`/`p2`, or a
   `coincident` constraint), and it stays joined when the curve changes.
3. When any generated point moved, the sketch is solved again (the generated points do not move).

A curve whose end is within `tol` of its start is **closed** (a periodic spline, a region of its own). The
echo of the sketch (`sketches[i].sketch`) carries the generated points, so the editor can draw the curve
as any spline; a document saved with them is fine, they are rebuilt.

## The fit (fewest points for the tolerance)

The spline is the kernel's interpolating spline (natural cubic, chord-length parameters; periodic when
closed), the same curve the profile builds. A candidate set of samples is checked span by span: the
formula at four `t` of the span (the two Gauss points, the middle, and one that moves along the
golden ratio from span to span, so a periodic formula sampled at whole periods is not taken for a
straight line) against the spline, and the spline's mid-span point against the formula
(closest-point searches). Two placements are tried and the one with fewer points is
kept (a tie keeps the even one):

* **even**: the fewest evenly spaced `t` within the tolerance. Every count is tried upward from 2 (3 when
  closed): the error is not monotone in the count, because the samples fall in and out of step with a
  periodic formula, so a search back down (solidcraft #9411) can miss the fewest. A check stops at the
  first span that misses, so failing counts are cheap;
* **split**: start with 3 (4) samples and split only the spans that miss the tolerance, round after
  round (fewer points where the curve is flat).

A straight line takes 2 points; a circle of radius 20 at 0.01 mm about 20. At most 200 points.

## Errors and warnings

Errors stop the sketch feature with the code, `entity` (the spline id) and `detail` (a plain message):

| Code | When |
|---|---|
| `equationFormula` | `x` or `y` is missing, empty, over 1024 characters, unreadable, or uses an unknown name (the message names it) |
| `equationNotFinite` | the formula has no finite value somewhere in the range (the message gives the `t`): `sqrt(5 - t)` past 5, `1/(t-5)` at 5 |
| `equationRange` | `t` is not `[start, end]`, an end cannot be read, or start = end |
| `equationTolerance` | `tol` outside 0.00001 to 10, or the curve needs more than 200 points for it (the message gives the distance with 200 points) |
| `equationDegenerate` | the formulas give the same point for every `t` |

Warning (the sketch still builds): `equationSelfIntersects` with `entity` and `detail` (where it crosses)
when the fitted curve crosses itself, as a Lissajous figure does; such an outline is not one region.

## Query: `equationCurve` (stateless)

`{type:"equationCurve", x, y, t:[start, end], tol?, variables?:[{name, expr}]}` returns
`{pts:[[x,y],..], closed, count, deviation, tol, warnings:[{code, detail}]}`, or the error
`"<code>: <message>"`. The sketch editor draws its preview from this; it is the same fit as regen.

## Examples (degrees)

| Curve | x | y | t |
|---|---|---|---|
| circle radius R | `R*cos(t)` | `R*sin(t)` | 0 to 360 |
| ellipse | `30*cos(t)` | `12*sin(t)` | 0 to 360 |
| sine wave, 40 long | `t/9` | `A*sin(t)` | 0 to 360 |
| helix seen from the side (r 10, pitch 6, 2 turns) | `10*cos(t)` | `6*t/360` | 0 to 720 |
| Lissajous 3:2 | `20*sin(3*t + 90)` | `15*sin(2*t)` | 0 to 360 |
| involute of a base circle r | `r*(cos(t) + t*pi/180*sin(t))` | `r*(sin(t) - t*pi/180*cos(t))` | 0 to 40 |
| parabola | `t` | `t*t/40` | -30 to 30 |

Behaviour studied from solidcraft #8353 (equation curve), #9282 (its user page), #9411 (fewest points)
and #9549 (a unit on the variable); no code copied. Differences: our trigonometry is in degrees, as in
every other expression of the kernel; the point count is chosen again on every regen (the spline is one
sketch curve whatever its point count); there is no explicit `y = f(x)` form (write
`x: "t"`).
