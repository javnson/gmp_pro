# GMP CCTL — C++ Control Template Library

**English** | [简体中文](readme_cn.md)

`cctl` contains experimental and reusable C++ control and power-electronics
objects. It complements the C-oriented `ctl` library and can use Eigen-backed
types for matrix and numerical calculations.

## Current modules

| Directory | Purpose |
| --- | --- |
| [`numerical_solver`](numerical_solver/readme.md) | Numerical equation and solver experiments |
| [`power_electronics_objects`](power_electronics_objects/readme.md) | C++ models of converters and controlled plants |

CCTL is not the default runtime used by current hardware suites. For production
embedded control, start with `ctl/component` and `ctl/suite`; use CCTL where C++
types and host-side numerical models provide a clear benefit.
