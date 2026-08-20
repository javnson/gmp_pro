# CCTL Host Simulation CSP

This CSP supplies the reusable host runtime for direct CCTL simulations. A
project owns its controller, peripheral mapping, generated circuit, motor or
other plant topology; the CSP owns scheduling and observation infrastructure.

The public `simulation_runtime` API provides `initialize()`, one-step `step()`,
non-blocking `interface_transfer()`, `run()`, and `finalize()`. `run()` starts
three workers: simulation, batched file output, and one-second console progress.
The simulation/file boundary is a preallocated single-producer/single-consumer
lock-free record ring provided by `cctl/dsa`. A full ring drops the new observation record rather than
stalling the numerical solver, and the final summary reports all drops.

Projects should manage concrete timing, ring capacity, output batching, and
pause policy through their SDPE requirement, then populate `simulation_config`
from the generated macros. Select this CSP through `gmp_src_mgr` as
`csp|cctl`; do not list its source manually in CMake.
