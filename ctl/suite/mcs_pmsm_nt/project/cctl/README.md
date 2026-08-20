# Direct CCTL mcs_pmsm_nt simulation

This target links the existing `mcs_pmsm_nt` controller, generated switched MNA
inverter, and `cctl::pmsm_cs` current-source PMSM in one process. It removes the
Windows/Simulink network transport. Each control transaction follows
`ePWM SOC -> ADC latch/interrupt -> controller callbacks -> ePWM update`.

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
converted once into `hw/generated/PMSM.json`; the default calculation class is
`hw/generated/eigen/pmsmcircuit.hpp` plus `pmsmcircuit.archive`. The header owns
the fixed-size structure and loader, while the archive owns the deduplicated
Eigen matrix pools; CMake copies the archive beside the executable. Setting `MATRIX_BACKEND` explicitly to
`fixed` or `all` also generates `hw/generated/fixed/pmsmcircuit.hpp`. Eigen is
resolved from the GMP installer/vcpkg environment, never from the deprecated
third-party copy.

The plant advances at 100 ns. The master ePWM emits one ADC SOC per 50 us,
center-aligned carrier on the configured CMPB up-count event (250 TBCLK here).
The testbench requires that event to occur while all three low-side gates conduct.
The ADC only stages analog values between SOC events; a trigger atomically latches
all channels, sets interrupt pending, and immediately dispatches the control ISR.
TI-style ADC, center-aligned complementary ePWM with dead band and compare-event
trigger output, and eQEP models are provided by `cctl/peripheral_if`. The motor accepts load torque through
`pmsm_cs_input::load_torque_nm`. The 4 s regression applies 0.02 N*m after
0.5 s and leaves enough time to check the existing speed PI against its 300 rpm
command.

The netlist exposes seven final conditioning nodes named `VADC_VDC`,
`VADC_VA/VB/VC`, and `VADC_IA/IB/IC`; circuit JSON marks them with
`role=adc_sample_voltage` and an `adc_channel`. Conversion is
`floor(clamp(VADC,0,Vref)/Vref*2^N)`, saturated to `2^N-1` (3.3 V, 12 bit in
this project). Controller scaling matches the netlist's actual analog front end:
1/48 voltage dividers, 0.055 V/A current sensitivity, and 1.65 V current bias.
The handwritten binding also follows the netlist's physical A/B/C shunt routing,
which is labeled `VADC_IC/IB/IA` respectively. CSV records include all seven raw
ADC result codes.

The reusable `cctl/dsa` SPSC record ring is allocation-free and lock-free after
initialization. The `csp/cctl` host runtime exposes `initialize`, `step`,
`interface_transfer`, `run`, and `finalize`, and owns simulation, batched file
output, and one-second console progress workers. The SDPE defaults are a 32 MB
ring and 1 MB output batches. A full ring drops observations instead of blocking
the numerical solver, and the final summary reports queued, written, and dropped
records, wall time, and realtime factor.

The console begins with `GMP CCTL Motor Simulation Kit`, then prints simulation
configuration and the applied process-priority policy. In an interactive terminal,
elapsed time, ETA, simulated time, instantaneous Mstep/s, queue occupancy, drops,
and the progress bar are redrawn at one saved cursor anchor instead of appending
lines. The bar reads the current visible console width on every refresh and fills
the available row after reserving its percentage suffix, so resizing is handled
automatically. Redirected/CTest output emits only the final progress snapshot.

On Windows, SDPE macro `CCTL_SIM_REALTIME_PRIORITY` controls whether the runtime
requests `REALTIME_PRIORITY_CLASS` for the simulation; the project default is on.
`--realtime-priority` forces it on, while `--normal-priority` and
`--no-realtime-priority` turn it off. Failure to acquire the class is reported and
falls back to normal priority without failing the run. The original process class
is restored after simulation. CTest explicitly selects normal priority so an
automated run cannot starve unrelated work on the host.

## Matrix backend selection

The unsuffixed `mcs_pmsm_nt_cctl` target and `build_test.bat` now use Eigen by
default. On the current 23-state, 729-topology model, an earlier three-run normal
priority measurement was 16.141034 s for Eigen and 22.135849 s for fixed, so the
daily generation, build, and regression path no longer pays for both backends.

Fixed support remains opt-in. `build_test.bat --with-fixed` additionally
generates the fixed header, configures `CCTL_BUILD_FIXED_BACKEND=ON`, builds
`mcs_pmsm_nt_cctl_fixed`, and runs its independent closed-loop test. Fixed pools
retain C++17 constant initialization and optional AVX2 for explicit static-storage,
Eigen-free embedded/FPGA-oriented work.

The Eigen archive is read and validated only while constructing `PmsmCircuit`;
simulation steps perform no file I/O. For the current 23-state, 729-topology
model, the JSON is 181,466,029 bytes. The formerly embedded Eigen header was
about 24.66 MB; the split output is about 23 KB of header plus an 8.33 MB archive.
Both JSON and archive are reproducible and excluded from Git. A deployment must
place `pmsmcircuit.archive` in the process working directory or pass an explicit
path to the `PmsmCircuit` constructor.

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
