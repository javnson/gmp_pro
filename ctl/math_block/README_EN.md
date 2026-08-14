# CTL Math Contract

`gmp_math.h` is the single numeric entry point for CTL algorithms. Reusable
components should include it, or a self-contained component header that includes
it, instead of including `gmp_core.h`.

The normal GMP path loads only `gmp_type.h`, `ctl.config.h`, the selected
`ctrl_gt`/`parameter_gt` backend, constants, compact vectors and matrices, and
coordinate transforms. It does not load CSP, device management, lifecycle hooks,
or the GMP runtime framework.

For a standalone embedded integration, define `GMP_CTL_PORTABLE` and provide
`gmp_ctl_portable_config.h`. The same `gmp_math.h` entry then loads the portable
type contract without requiring CSP or the GMP runtime.

Dependency rule:

- Numeric algorithms depend on `ctl/math_block/gmp_math.h`.
- Components add only their direct component dependencies.
- Time, critical-section, logging, and device services explicitly depend on
  `core/base` or `core/dev`; they are not part of the math contract.
- Suites and application entry points may include `gmp_core.h` when they need the
  complete GMP assembly.

The configuration authority remains layered as user configuration, CSP
configuration, then repository defaults through `gmp_type.h`.

## Numeric domains and conversions

`real_gt` describes the optional source/offline precision and defaults to
`double`; a user or CSP may select `long double`. Embedded algorithms should
not normally declare `real_gt` objects. Conversion macros consume constant
expressions directly so the compiler can fold them into the destination type
without creating a wide intermediate object.

`parameter_gt` is `float` or `double`. It owns configuration, tuning, offline
analysis and other slow calculations. Native `+`, `-`, `*` and `/` operators
are used directly; non-linear operations must use `param_abs`, `param_sin`,
`param_cos`, `param_tan`, `param_sqrt`, `param_exp`, `param_ln`, `param_pow`,
`param_atan2` and the other `param_xxx` entry points.

`ctrl_gt` owns the high-frequency control path. It may be floating point or a
fixed-point type such as TI IQmath. Runtime algorithms must use `ctl_xxx`
operations and must not convert to `parameter_gt` on every ISR call.

Algorithms may publish stricter compile-time capabilities. FDRC defines
`CTL_FDRC_SUPPORTED` and is executable only when
`CTL_CTRL_GT_IS_FLOATING_POINT` is nonzero. On a fixed-point backend its step
function returns zero and its enable API keeps learning disabled, even if an
application requests activation.

Cross-domain conversion is limited to four explicit entry points:

```c
parameter_gt p = real2param(0.125);
ctrl_gt literal = real2ctrl(0.125);
ctrl_gt cached = param2ctrl(tuning_value);   /* normally during init */
parameter_gt diagnostic = ctrl2param(output); /* slow diagnostics */
```

`float2ctrl` and `ctrl2float` are deprecated. The descriptive aliases
`parameter2ctrl` and `ctrl2parameter` remain for downstream compatibility;
new CTL code uses the four domain names above. Common numeric and mathematical constants come from `const/` as
the parallel `CTL_CTRL_CONST_*` and `CTL_PARAM_CONST_*` families. Physical
tuning values remain component inputs rather than global mathematical constants.

Zero comparisons use `CTL_CTRL_CONST_EPSILON` or
`CTL_PARAM_CONST_EPSILON`. The control-domain value is one LSB for TI IQmath,
`1e-6` for float backends, and `1e-12` for double. The parameter-domain value
is `1e-6` for float and `1e-12` for double. A component may use a different
threshold only when it represents an algorithmic or physical limit, and that
exception must be named or documented locally.

Real-time code uses `CTL_CTRL_CONST_*` for standard values such as zero, one,
one half, pi, and epsilon. It must not repeat `real2ctrl(1.0)` or an equivalent
conversion at call sites. Parameter-domain formulas likewise use
`CTL_PARAM_CONST_*` for reusable constants. A component-specific threshold or
physical literal that does not belong in the shared constant catalog enters
the parameter domain through `real2param(...)`; it must not be frozen to
`float` with an `f` suffix. Initialization calculations remain in
`parameter_gt` until the final assignment to cached `ctrl_gt` fields.

## Angle-unit contract

Angle units are part of the function contract and must never be inferred:

| Function | Value domain | Angle input/result |
| --- | --- | --- |
| `ctl_sin`, `ctl_cos`, `ctl_tan` | `ctrl_gt` | per-unit turn |
| `ctl_sin_rad`, `ctl_cos_rad` | `ctrl_gt` | radians |
| `param_sin`, `param_cos`, `param_tan` | `parameter_gt` | radians |
| `param_sin_pu`, `param_cos_pu` | `parameter_gt` | per-unit turn |
| `ctl_atan2` | `ctrl_gt` | result in radians |
| `ctl_atan2_pu` | `ctrl_gt` | result in per-unit turns |

One per-unit turn is `2*pi` radians. The radian wrappers for `ctrl_gt` stay in
the selected control backend; in particular, the IQmath path does not fall
back to floating-point libm.

## C++ fixed-size algebra

`gmp_math.hpp` adds heap-free C++ templates without changing the C API:

```cpp
ctl::math::vector_lite<MyNumber, 3> vector;
ctl::math::matrix_lite<MyNumber, 3, 3> matrix;
ctl::math::vector_lite<MyNumber, 3> result = matrix * vector;
```

The element type only needs the arithmetic operations used by the selected
calculation. Storage is inline and dimensions are compile-time constants.

The tests in `ctl/math_block/tests` exercise float/float, float/double,
double/double, IQ24/float and IQ24/double contracts, plus the C++ templates
with a custom arithmetic type. `numeric_type_contract_audit.py` rejects
`parameter_gt` declarations in real-time step functions and concrete libm
calls, deprecated conversions, explicit `real_gt` storage, premature
parameter-to-control conversion at selected init APIs, and repeated standard
literal conversions in authoritative CTL component/framework/suite code.
