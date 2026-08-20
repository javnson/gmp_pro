# Direct CCTL mcs_pmsm_nt simulation

This target links the existing `mcs_pmsm_nt` controller, generated switched MNA
inverter, and `cctl::pmsm_cs` current-source PMSM in one process. It removes the
Windows/Simulink network transport while preserving the normal GMP callback order.

Run `build_test.bat`. The seven-stage flow generates the target SDPE bindings,
deploys the selected sources, generates `gmp_config.cmake`, regenerates the
project-local main circuit, builds, and runs the regression. It locates GMP
exclusively through `GMP_PRO_LOCATION` and builds in
`%TEMP%\gmp_mcs_pmsm_nt_cctl_build`.

`sdpe_mgr/sdpe_requirement.json` owns controller, ADC/eQEP/ePWM, timing, output,
and pause parameters. `gmp_src_mgr/gmp_framework_config.json` selects the GMP
controller dependencies plus `cctl|dsa` and `csp|cctl`; CMake consumes only the
generated source-manager `.cmake` file. The CMake generator resolves `inc_dirs`
from the selected modules and their dependency closure, including in `src_only`
mode where the header-mirror summary may be absent or stale. `hw/PMSM.CIR` is converted into
`hw/generated/PMSM.json` and the fixed-matrix `pmsmcircuit.hpp`, so this target
does not depend on another test case or the `simulate` project.

The plant advances at 100 ns while the 20 kHz controller advances every 500 plant
steps. TI-style ADC, center-aligned complementary ePWM with dead band, and eQEP
models are provided by `cctl/peripheral_if`. The motor accepts load torque through
`pmsm_cs_input::load_torque_nm`. The 4 s regression applies 0.02 N*m after
0.5 s and leaves enough time to check the existing speed PI against its 300 rpm
command.

The reusable `cctl/dsa` SPSC record ring is allocation-free and lock-free after
initialization. The `csp/cctl` host runtime exposes `initialize`, `step`,
`interface_transfer`, `run`, and `finalize`, and owns simulation, batched file
output, and one-second console progress workers. The SDPE defaults are a 32 MB
ring and 1 MB output batches. A full ring drops observations instead of blocking
the numerical solver, and the final summary reports queued, written, and dropped
records, wall time, and realtime factor.

Manual execution pauses through `system("@pause")` by default. CTest passes
`--no-pause`; `--output <path>` overrides the CSV destination.
