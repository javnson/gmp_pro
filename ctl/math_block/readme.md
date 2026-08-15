# CTL math block

**English** | [简体中文](readme_cn.md)

`ctl/math_block/gmp_math.h` is the numeric entry point for reusable CTL C
algorithms. It assembles the selected `ctrl_gt` and `parameter_gt` backends,
conversions, constants, compact vectors/matrices, and coordinate transforms
without loading CSP, devices, or the GMP runtime.

## Numeric domains

- `real_gt` is an optional source/offline precision. Embedded algorithms
  should not normally store it.
- `parameter_gt` is `float` or `double` and is used for configuration, tuning,
  initialization, and slow analysis. Nonlinear operations use `param_*` APIs.
- `ctrl_gt` owns real-time states and calculations. It may be floating point or
  fixed point, so multiplication, division, saturation, nonlinear operations,
  and conversions use the `ctl_*` contract.

The explicit conversion entry points are `real2param`, `real2ctrl`,
`param2ctrl`, and `ctrl2param`. `float2ctrl` and `ctrl2float` are deprecated.

## Angle contract

`ctl_sin`, `ctl_cos`, and `ctl_tan` take per-unit turns. Their `_rad` variants
take radians. `param_sin`, `param_cos`, and `param_tan` take radians, while
`param_sin_pu` and `param_cos_pu` take per-unit turns. `ctl_atan2` returns
radians and `ctl_atan2_pu` returns turns. One turn is `2*pi` radians.

## Contents and validation

The current C tree contains `ctrl_gt`, `parameter_gt`, `const`, `coordinate`,
`vector_lite`, `matrix_lite`, `complex_lite`, and `utilities`. The public
function names are those declared in these headers; older names such as
`ctl_clarke_abc2ab0`, `ctl_park_ab2dq`, and `ctl_svpwm_calc` are not current
APIs.

`gmp_math.hpp` adds heap-free fixed-size C++ vector and matrix templates. Tests
under `ctl/math_block/tests` cover the configured float/double and TI IQmath
contracts plus C++ custom-number templates. See [the detailed numeric
contract](README_EN.md) for backend and audit details.
