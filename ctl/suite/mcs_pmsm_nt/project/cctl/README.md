# Direct CCTL mcs_pmsm_nt simulation

This target links the existing `mcs_pmsm_nt` controller, generated switched MNA
inverter, and `cctl::pmsm_cs` current-source PMSM in one process. It removes the
Windows/Simulink network transport. Each control transaction follows
`ePWM SOC -> ADC latch/interrupt -> controller callbacks -> ePWM update`.

## Maintained runtime boundaries

The selected `csp|cctl` module owns the executable `main()` and calls
`gmp_base_entry()`. The ordinary GMP order initializes `setup_peripheral()`,
`ctl_init()`, and project `init()`. The project `init()` only registers build
metadata, the persistent topology, and callbacks with the CSP. The CSP then
starts services in `gmp_csp_post_process()`, advances one plant step per
`gmp_csp_loop()`, and finalizes in `gmp_csp_exit()`. This project must not add
another process entry or initialize the controller again from a plant callback.

Project `xplt/mcu_simulation.hpp/.cpp` aggregates the seven ADC inputs, three
complementary ePWM modules, ADC SOC/interrupt dispatch, and eQEP model. The
C-compatible `xplt.peripheral.*` files contain only controller-visible register
storage, scaling-channel initialization, and the simulated ADC ISR. That ISR is
the only project site that calls `gmp_base_ctl_step()`; it is reached from ePWM
SOC after ADC and encoder registers are latched, never from the plant main loop.
`xplt.ctl_interface.h` maps control callbacks and routes state-machine output
enable/disable through the CSP.
Generic TI-style peripheral primitives remain under `cctl/peripheral_if`.

Chip background code and the control ISR have independent schedules. The SDPE
value `CCTL_SIM_USER_CODE_FREQUENCY_HZ` is currently 33 kHz; the simulated MCU
compute allocator calls user `mainloop()` and controller-background
`ctl_mainloop()` at that rate instead of running an arbitrary number of user
loops per control interrupt. ADC still raises the 20 kHz control interrupt and
exclusively owns `gmp_base_ctl_step()`. A 4 s regression therefore expects
132000 user/background dispatches and 80000 control calculations.

Controller arithmetic uses the CSP default `ctrl_gt=float`, while peripheral
and motor models use `sim_real_gt=double`. See `csp/cctl/doc/README.md` for the
complete scalar and lifecycle contract.

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
this project). The project-private `mcs_pmsm_nt_cctl_inverter` SDPE entity binds
the 2136SINV simulation calibration: 1/48 voltage dividers, 5 mOhm times 11 or
0.055 V/A current sensitivity, and 1.65 V current bias. The netlist's 22 kOhm
feedback and two series 1 kOhm input resistors give an effective gain of 11. Before compilation,
`hw/validate_generated_model.py` recovers these DC gains from the generated
matrices, compares them with the private SDPE parameters, rejects phase-current
cross coupling, and verifies A-to-PWM1/2, B-to-PWM3/4, C-to-PWM5/6 bridge routing.
The physical A/B/C shunt paths now map directly to `VADC_IA/IB/IC`; the handwritten
binding performs no A/C swap. CSV records include all seven raw
ADC result codes.
Open large result files with `tools/cctl_studio/result_viewer/run_result_viewer.bat`;
it loads selected columns in the background, preserves extrema while decimating,
and supports independent multi-panel curves with linked zoom.

The reusable `cctl/dsa` SPSC record ring is allocation-free and lock-free after
initialization. The `csp/cctl` host runtime exposes `initialize`, `start`,
`step`, `interface_transfer`, `run`, and `finalize`. The standard GMP main
thread owns simulation while workers handle batched file output and one-second
console progress. The SDPE defaults are a 32 MB
ring and 1 MB output batches. A full ring drops observations instead of blocking
the numerical solver. The final summary reports queued, written, dropped, peak
ring occupancy, and the output worker's actual formatting/writing busy time.
That busy time occurs on the independent output thread rather than being added
to the numerical hot path.

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

`queue=0` means only that the SPSC ring was empty at the instant of the console
snapshot; the adjacent `staged` value reports records already held in the output
worker's private batch. Neither means the simulation thread writes synchronously.
`--profile` enables
sparse hot-path timing for peripherals, the circuit, motor, housekeeping, and
controller ISR without timing every plant step.

## Matrix backend selection

The unsuffixed `mcs_pmsm_nt_cctl` target and `build_test.bat` use Eigen by
default. Startup now reports `build=Release optimized=yes`. An IDE-built
Debug/`/Od` executable prints an immediate performance warning because Eigen's
small fixed-size expressions can be tens of times slower without optimization.
Performance regressions must use `build_test.bat` or an explicit CMake Release
configuration.

When Visual Studio opens this directory, it reads `CMakePresets.json`. Select
`Windows MSVC Release (recommended)` for performance runs or `Windows MSVC
Debug` for a genuine `/Od` diagnostic build, then build/start
`mcs_pmsm_nt_cctl.exe`. Visual Studio remembers the last workspace selection,
so an existing workspace may remain on the legacy `x64-Debug` entry until
Release is selected once. `mcs_pmsm_nt_cctl.exe --build-info` reports the
configuration without running 40 million steps: Release must say
`build=Release optimized=yes`, while Debug must say `build=Debug optimized=no`.
With a Visual Studio multi-config generator the toolbar configuration is
authoritative; `CMAKE_BUILD_TYPE` does not select it. CMake also deploys the
Eigen archive beside the executable for each selected configuration and uses
that directory as the Visual Studio debugger working directory.

The reusable PMSM supports Euler, midpoint RK2, and classical RK4, retaining RK4
as its general default. Project SDPE owns `CCTL_SIM_PMSM_INTEGRATION_ORDER`.
Because the 100 ns plant step is tiny relative to the millisecond-scale
electrical time constants, this project selects Euler while still updating the
motor every circuit step; it adds no multirate hold or extra coupling delay.
Values 2 and 4 remain available for accuracy comparisons.

On the current machine, the 4 s, 40,000,000-step, 23-state/729-topology Release
Eigen regression takes about 9.4 s. A profiled run to `NUL` takes about 9.11 s at
4.39 Mstep/s, versus 12.15 s with RK4. The last-50-ms mean-speed difference from
RK4 is about 0.003 rpm and the closed-loop regression passes. Formatting and
writing 80,000 records (about 22.45 MB) keeps the asynchronous output worker busy
for only about 0.57 s, so it cannot explain a run lasting hundreds of seconds.

Fixed support remains opt-in. `build_test.bat --with-fixed` additionally
generates the fixed header, configures `CCTL_BUILD_FIXED_BACKEND=ON`, builds
`mcs_pmsm_nt_cctl_fixed`, and runs its independent closed-loop test. Fixed pools
retain C++17 constant initialization and optional AVX2 for explicit static-storage,
Eigen-free embedded/FPGA-oriented work. Its same-configuration regression is
about 12.6 s, so Eigen remains the default.

The Eigen archive is read and validated only while constructing `PmsmCircuit`;
simulation steps perform no file I/O. For the current 23-state, 729-topology
model, compact schema-v2 JSON is about 77.2 MB after pooling runtime matrices. The pools
retain 1,037,408 of 1,604,529 logical coefficients, a 35.35% reduction. The formerly embedded Eigen header was
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
The exact solver therefore remains on the GMP numerical main thread while file
and console workers run concurrently with it.

`hw/generate_code.bat` accepts `DISCRETIZATION_METHOD=forward_euler`,
`backward_euler`, or `rk4`. The affine-LTI RK4 backend is precomputed into one
matrix update, so it adds no runtime stages. This particular circuit is highly
stiff because of its pF-scale conditioning/parasitic capacitances, however, and
RK4 at 100 ns produces a non-finite first step. Backward Euler therefore remains
the project default; RK4 is intended for less-stiff topologies or smaller-step
accuracy experiments.

Manual execution pauses through `system("@pause")` by default. CTest passes
`--no-pause`; `--output <path>` overrides the CSV destination.
