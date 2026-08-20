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
mode where the header-mirror summary may be absent or stale. `hw/PMSM.CIR` is
converted once into `hw/generated/PMSM.json`, then into matching fixed and Eigen
solvers under `hw/generated/fixed` and `hw/generated/eigen`. Eigen is resolved
from the GMP installer/vcpkg environment, never from the deprecated third-party
copy.

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

The console begins with `GMP CCTL Motor Simulation Kit`, prints total simulated
time, plant step, step count, and backend, then displays elapsed time, ETA, queue
occupancy, drops, and a 64-character `=`/`>` progress bar.

## Fixed versus Eigen benchmark

`build_test.bat` builds and regresses separate fixed and Eigen executables from
the same handwritten testbench and circuit data. A third CTest compares all
80,000 CSV rows element-by-element with a `1e-8` tolerance; the current maximum
difference is `4.20e-10`. Run
`benchmark_backends.bat [runs]` afterwards for an alternating-order whole-system
benchmark. It retains CSV formatting but sends records to `NUL`, checks the same
physical result signature, and reports mean/median/minimum wall time. A three-run
measurement on the current development machine was 9.067892 s for fixed and
8.059164 s for Eigen, a 1.1252x Eigen advantage. Eigen is therefore recommended
for this desktop simulation; fixed remains the static-storage, dependency-free
baseline for embedded/FPGA-oriented work. Re-run the benchmark on each target.

The controller, circuit, and motor cannot execute as three concurrent exact
stages. Circuit step `n` consumes motor current from step `n-1`, motor step `n`
consumes the newly computed circuit voltage, and circuit step `n+1` immediately
needs that motor result. Control boundaries must also update gates before the
circuit step. Splitting this chain across workers would add a cross-thread
handoff every 100 ns without overlap. A Jacobi-style one-step lag could overlap
work, but changes the numerical model and is not a transparent optimization.
The exact solver therefore remains one numerical worker while file and console
workers run concurrently with it.

Manual execution pauses through `system("@pause")` by default. CTest passes
`--no-pause`; `--output <path>` overrides the CSV destination.
