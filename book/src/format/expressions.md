# Variables and expressions

Numeric fields in a part can hold an expression instead of a number, and a part can define global variables for expressions to use. This chapter gives the grammar, the units, the functions, the rules for variables and the result of each kind of mistake.

## Where expressions are accepted

- Every numeric feature field that the field tables in this part list as "number or expression": depths, radii, angles, offsets, distances, counts, vector components such as `translate`.
- The `expr` field of a dimensional sketch constraint; the kernel then recomputes the constraint's `value` (see [sketches](sketches.md)).
- The `expr` of a global variable.

Sketch point coordinates and circle radii are plain numbers. A string there is converted when it is a plain number (`"3"` becomes 3) and anything else, including `"=2+3"`, becomes 0. To drive a coordinate from a variable, add a constraint with an `expr`.

## Field values

A numeric field is read as follows:

| JSON value | Result |
|---|---|
| number | that number |
| string starting with `=` (leading spaces allowed) | the value of the expression; a syntax error, an unknown name or a non-finite result is `badExpression` |
| any other string | the string read as a plain number after trimming spaces; `0` if it is not one |
| missing, `null`, boolean, array, object | the default of the field |

| Field value | Value |
|---|---|
| `7` | 7 |
| `"7"` | 7 |
| `" 7 "` | 7 |
| `"2*3"` | 0 |
| `"abc"` | 0 |
| `"1in"` | 0 |
| `true` | 0 |
| `null` | 0 |

(Run as a translation distance; the default of that field is 0.)

A field holding `"=expr"` fails its feature with `badExpression` when the expression is bad; the feature's other checks come after it.

## Grammar

```text
expr   := term (('+' | '-') term)*
term   := power (('*' | '/') power)*
power  := unary ('^' power)?                 right associative
unary  := '(' expr ')' | '-' power | '+' power
        | name                               variable, or pi
        | function '(' expr ')'
        | function2 '(' expr (',' expr)* ')'   the functions of the second table below
        | number [unit]
number := digits ['.' [digits]] | '.' digits, with an optional exponent (e5, E-3)
unit   := mm | cm | m | in | deg | °         case-insensitive
```

- A leading `=` is optional inside variable definitions and constraint `expr`s and is removed. Spaces are allowed between tokens.
- `,` is read as the decimal separator, so `1,5` is 1.5, except directly inside the parentheses of a call to one of the functions of the second table, where it separates the arguments: `max(1,5)` is 5 and `max(1.5, 2)` is 2.
- Variable names are case sensitive. `pi` and the function names are case-insensitive.
- `^` binds tighter than unary minus, so `-2^2` is -4 and `2^3^2` is 512.
- Functions are `sqrt`, `sin`, `cos`, `tan` and `abs`, with one argument in parentheses. Trigonometric functions take degrees.
- More functions, written in lower case and always followed by `(`:

| Function | Arguments | Value |
|---|---|---|
| `min`, `max` | one or more | the smallest, the largest |
| `pow(a, b)` | 2 | `a` to the power `b` |
| `mod(a, b)` | 2 | the remainder of `a / b`, with the sign of `a` |
| `hypot(a, b)` | 2 | `sqrt(a^2 + b^2)` |
| `atan2(y, x)` | 2 | the angle of the point `(x, y)`, in degrees |
| `asin`, `acos`, `atan` | 1 | an angle in degrees |
| `round` | 1 | the nearest whole number, halves away from zero |
| `floor`, `ceil`, `trunc` | 1 | rounded down, up, towards zero |
| `exp`, `ln`, `log10` | 1 | `e` to the power, natural and decimal logarithm |

  A wrong number of arguments, an argument or a result that is not finite (`ln(0)`, `asin(2)`, `mod(1, 0)`), or a name in capitals (`MAX(1, 2)`) is a syntax error. These names are not reserved: a variable may still be called `max` or `round`, and `max * 2` is that variable times 2; only a name followed by `(` is the function.
- Units scale a number to millimetres or degrees: `cm` is 10, `m` is 1000, `in` is 25.4, `mm`, `deg` and `°` are 1. A unit directly after a name or a parenthesis is a syntax error.
- A result that is not finite (`1/0`, `1e400`) is a syntax error.

Values of expressions as the kernel evaluates them (rounded to nine decimals; the variable `w` is 30 here):

| Expression | Value |
|---|---|
| `=2^3` | 8 |
| `=2^3^2` | 512 |
| `=-2^2` | -4 |
| `=10/4` | 2.5 |
| `=(1+2)*3` | 9 |
| `=2*3+4` | 10 |
| `=1e1` | 10 |
| `=.5` | 0.5 |
| `=1,5` | 1.5 |
| `=3 mm` | 3 |
| `=2 cm` | 20 |
| `=1 in` | 25.4 |
| `=3 m` | 3000 |
| `=sqrt(16)` | 4 |
| `=SQRT(16)` | 4 |
| `=abs(-4)` | 4 |
| `=max(3, 1, 2)` | 3 |
| `=mod(-7, 4)` | -3 |
| `=round(-2.5)` | -3 |
| `=atan2(1, 1)` | 45 |
| `=hypot(3, 4)` | 5 |
| `=ln(0)` | `badExpression` |
| `=sin(30)` | 0.5 |
| `=cos(60)*10` | 5 |
| `=tan(45)` | 1 |
| `=pi` | 3.141592654 |
| `=PI*2` | 6.283185307 |
| `=--3` | 3 |
| `=1/0` | `badExpression` |
| `=abc` | `badExpression` |
| `=7)` | `badExpression` |
| `=sqrt 4` | `badExpression` |
| `=3 mx` | `badExpression` |
| `=W` | `badExpression` |

## Global variables

`variables` is a list of definitions in the document:

| Field | Type | Meaning |
|---|---|---|
| `name` | string | Letters, digits and `_`, not starting with a digit. Case sensitive. |
| `expr` | string or number | The value, an expression. A number is accepted and read as its text. |

Variables can use each other in any order. The kernel repeatedly resolves every variable whose expression mentions no variable that is still unresolved. A variable that cannot be resolved does not stop the document:

| Problem | Effect |
|---|---|
| Invalid name, a name equal to `pi`, a function name or a unit (`mm`, `cm`, `m`, `in`, `deg`) in any letter case, or a name already defined | The definition is dropped. For a repeated name the first definition is kept. |
| `expr` does not evaluate | The variable is left undefined. |
| A cycle (`a = b + 1`, `b = a + 1`) | The variables in the cycle are left undefined. |

The problems are not reported on their own. A field that uses an undefined variable fails with `badExpression`, because the name is then unknown.

```json
{
  "format": "ck-part", "version": 1,
  "variables": [{"name": "t", "expr": "4"}, {"name": "d", "expr": "=t*2"}],
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": "=d"}
  ]
}
```

Run result: no errors; 3 faces, volume 2513.2741 mm³. The extrude depth is 8, so the cylinder top is at z = 8.


```json
{
  "format": "ck-part", "version": 1,
  "variables": [{"name": "a", "expr": "b+1"}, {"name": "b", "expr": "a+1"}],
  "features": [
    {
      "type": "sketch", "id": "F1", "plane": {"kind": "std", "id": "front"},
      "points": [{"id": "c", "x": 0, "y": 0}],
      "entities": [{"type": "circle", "id": "e1", "c": "c", "r": 10}], "constraints": []
    },
    {"type": "extrude", "id": "F2", "op": "add", "sketch": "F1", "end": "blind", "depth": "=a"}
  ]
}
```

Run result: errors `F2`: `badExpression`; no solid body.


## See also

- [Overview of the part format](overview.md)
- [Sketches](sketches.md)
- [Features](features.md)
