# CCTL Host Simulation CSP

**English** | [简体中文](README_CN.md) | [API manual](doc/README.md)

This CSP supplies the reusable host runtime for direct CCTL simulations. A
project owns its controller, peripheral mapping, generated circuit, motor or
other plant topology; the CSP owns scheduling and observation infrastructure.

The public `simulation_runtime` API provides `initialize()`, `start()`, one-step
`step()`, non-blocking `interface_transfer()`, `run()`, and `finalize()`. In the
standard GMP lifecycle, the main thread advances one step per `gmp_csp_loop()`;
batched file output and one-second console progress use two workers. The
simulation/file boundary is a preallocated single-producer/single-consumer
lock-free record ring provided by `cctl/dsa`. A full ring drops the new observation record rather than
stalling the numerical solver, and the final summary reports all drops.

Projects should manage concrete timing, ring capacity, output batching, and
pause policy through their SDPE requirement, then populate `simulation_config`
from the generated macros. Select this CSP through `gmp_src_mgr` as
`csp|cctl`; do not list its source manually in CMake.

The CSP now supplies the standard `csp.config.h`, `csp.typedef.h`,
`csp.typedef.hpp`, `csp.general.h`, and `csp.general.hpp` composition. It also
owns `main()` and enters projects through `gmp_base_entry()`. See the API manual
for the lifecycle, scalar-type contract, and every public function.

The common `simulation_system` standardizes projects into chip, peripheral, and
circuit domains. A chip model can use `compute_budget_scheduler` to grant
user/background work at an independent frequency, while ADC control calculation
is triggered only by the peripheral model's conversion-complete interrupt.
