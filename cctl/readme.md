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
| [`dsa`](dsa/README.md) | Reusable preallocated SPSC lock-free fixed-record streams |
| `peripheral_if` | TI DSP-style ADC, center-aligned complementary ePWM, eQEP, and fixed-rate divider models |
| `circuit_solver` | Historical Python modified-nodal-analysis experiments |
| `component` | Early C++ interface experiments; not exported by `cctl.hpp` |

ADC, ePWM, and eQEP models in `peripheral_if` accept user parameters through
their static `make()` factories and expose immutable configuration values. The
ADC can retain a context-aware ISR function pointer; `trigger_and_transfer()`
moves result registers before dispatching that interrupt.

CCTL is not the default runtime used by current hardware suites. For production
embedded control, start with `ctl/component` and `ctl/suite`; use CCTL where C++
types and host-side numerical models provide a clear benefit.

## Validated PMSM averaged-simulation core

`cctl.hpp` aggregates the fixed-size vector, Euler/RK4 solvers, averaged three-phase inverter, and PMSM averaged model that currently pass offline tests. The implementation is header-only C++11, has no Eigen dependency, and contains no UDP/TCP or controller coupling.

The standalone validation project is under `tb/pmsm_average_model_test`. It checks solver convergence order, locked-rotor analytical RL response, analytical free coast, abc/dq power invariance, torque, and averaged dead-time voltage error.

`ctl/suite/mcs_pmsm_nt/project/cctl` additionally links the switched MNA power
circuit, `pmsm_cs` current-source motor, these peripheral models, and the existing
PMSM controller in one process. ePWM SOC and the ADC ISR connect the 100 ns
plant step to the 20 kHz controller, while the MCU compute scheduler independently
runs 33 kHz user background work, without network transport. `csp/cctl` owns the
simulation main thread plus file and console service workers; the reusable `dsa`
record ring supplies the 32 MB simulation-to-file channel.
