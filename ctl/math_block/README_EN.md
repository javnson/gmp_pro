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
