# Hosted CCTL CSP API

**English** | [简体中文](README_CN.md)

This document is the maintenance contract for `csp/cctl`. The CSP owns the
host process entry, GMP lifecycle integration, simulation scalar definition,
main-thread simulation runtime, console progress, asynchronous file output, and the
generic output-enable state. A selected project owns its controller,
`setup_peripheral()`, plant topology, MCU register mapping, SDPE parameters,
records, and result validation.

## Header composition

Include `gmp_core.h` from C and `gmp_core.hpp` from C++. GMP loads these CSP
headers through the normal configuration chain:

- `csp.config.h` marks the build as a finite PC environment, enables the CSP
  controlled-exit hook, and defaults `ctrl_gt` to `float` through
  `USING_FLOAT_FPU`. A project may define the selection earlier in
  `xplt.config.h`.
- `csp.typedef.h` is the C-compatible customization point. It repeats the
  guarded `ctrl_gt` default and otherwise retains GMP portable C types.
- `csp.typedef.hpp` defines `sim_real_gt`. Its default is `double`; define
  `GMP_CCTL_SIM_REAL_TYPE` before GMP headers to select another C++ scalar.
- `csp.general.h` declares services callable by C controller code.
- `csp.general.hpp` wraps the C header and includes both `csp.typedef.hpp` and
  `csp_cctl.hpp`. C++ project code normally receives the whole CSP through
  `gmp_core.hpp`.

`ctrl_gt` and `sim_real_gt` deliberately serve different purposes. `ctrl_gt`
models the arithmetic of the real controller and therefore remains `float` by
default. `sim_real_gt` models the continuous plant and peripherals and defaults
to `double`.

An alternative `sim_real_gt` must be copyable and default constructible, accept
integer and floating constants, and implement assignment, `+`, `-`, unary `-`,
`*`, `/`, and all ordinary comparisons. It must also work with the standard
math operations used by CCTL models: `abs`, `min`, `max`, `isfinite`, `floor`,
`fmod`, `sqrt`, `sin`, `cos`, and `remainder`. Built-in `float`, `double`, and
`long double` satisfy this contract. A custom class must supply compatible math
overloads and conversions; changing only the typedef is not sufficient if a
generated circuit has a fixed `double` scalar.

## C services

- `csp_sl_enable_output()` sets the CSP-owned simulated power-stage enable.
  Call it from the controller state-machine transition that makes PWM outputs
  active. The PMSM project routes `ctl_fast_enable_output()` through it.
- `csp_sl_disable_output()` clears the enable state. Call it on initialization,
  stop, fault, and disabled state transitions before publishing new compares.
- `csp_cctl_output_is_enabled()` returns the current flag for the project MCU
  simulation. A nonzero value permits the ePWM model to drive gates.
- `gmp_hal_wd_feed()`, `gmp_hal_wd_enable()`, and `gmp_hal_wd_disable()` are
  hosted no-ops that satisfy the portable CSP watchdog contract.

## Process and GMP lifecycle

`src/csp_cctl_main.cpp` is the only process entry for a project that selects
`csp|cctl`. A project must not define another `main()`. The CSP captures command
line arguments and calls `gmp_base_entry()`, which preserves the standard order:

```text
gmp_csp_startup
  -> setup_peripheral
  -> ctl_init
  -> init
  -> gmp_csp_post_process
  -> gmp_csp_loop (repeat: chip -> peripheral out -> circuit -> sample/ISR)
  -> gmp_csp_exit
```

`gmp_csp_startup()` parses the common options `--no-pause`,
`--realtime-priority`, `--normal-priority`, `--no-realtime-priority`,
`--profile`, `--build-info`, and `--output <path>`. A project reads the result
through `command_line()` and registers build metadata plus its simulation from
the normal C-linkage `init()` hook. `gmp_csp_post_process()` starts the plant and
the two service workers. Each `gmp_csp_loop()` call advances exactly one plant
step. The core loop asks
`gmp_csp_should_exit()` after each complete background iteration, and
`gmp_csp_exit()` finalizes the plant, joins workers, reports results, restores
priority, and performs the optional pause.

The CCTL CSP defines both `SPECIFY_CSP_MANAGES_USER_MAINLOOP` and
`SPECIFY_CSP_MANAGES_CTL_MAINLOOP`. Plant steps can be much faster than MCU
background tasks, so the GMP core does not call either mainloop at the plant
rate. A project's chip model uses `compute_budget_scheduler` to call
`mainloop()` and `ctl_mainloop()` at an independent SDPE-configured frequency;
its phase accumulator supports non-integer ratios such as 33 kHz on a 10 MHz
plant clock. The real-time `gmp_base_ctl_step()` entry is outside that budget.
Only the peripheral model may invoke it, synchronously from the simulated ADC
ISR after conversion results have been latched.

The CSP implements `gmp_csp_startup()`, `gmp_csp_post_process()`,
`gmp_csp_loop()`, `gmp_csp_exit()`, `gmp_csp_stuck_routine()`, and
`gmp_csp_not_implement()`. Projects do not replace these functions.

## Standard simulation domains

`embedded_chip_simulation`, `peripheral_simulation`, and
`circuit_simulation` model the processor compute/background domain, ADC/PWM/QEP
peripherals, and the electrical/mechanical plant. `simulation_system` composes
them in this deterministic order:

```text
chip background budget
  -> apply peripheral outputs to plant
  -> advance electrical/mechanical circuit
  -> sample plant into peripherals
  -> synchronously dispatch ADC ISR/control when an event is pending
```

Initialization runs chip, peripheral, then circuit; finalization reverses that
order. Peripheral events own interrupts. Neither the chip budget scheduler nor
the circuit step polls or directly invokes the controller ISR.

## Simulation runtime

`simulation_runtime::initialize(config, callbacks)` validates and stores one
run. Required configuration fields are a positive `total_steps`, finite positive
`plant_step_s`, nonzero POD `record_size`, output filename, and record writer.
Use SDPE-generated values for ring size, batch size, progress interval, and
pause policy. CSP command-line policy overrides output path and priority.

Callbacks have these roles:

- `initialize`: prepare the plant after the GMP lifecycle has initialized the
  controller and platform storage;
- `step`: advance one numerical state from `gmp_csp_loop()`; `step_range` is
  retained for the blocking convenience API;
- `finalize`: validate the completed plant run;
- `write_record`: format one copied POD record on the file worker;
- `print_summary`: print project-specific results from `gmp_csp_exit()`.

The framework path uses `start()`, repeated `step()`, and `finalize()`; it never
blocks inside a project entry wrapper. `run()` is a standalone convenience that
performs the same sequence on its calling thread. File output and console
progress are the only worker threads. `finalize()` joins them and is idempotent.

`interface_transfer(record, size)` performs the hot-path nonblocking copy into
the SPSC ring. The record must remain trivially copyable and its size must equal
`simulation_config::record_size`. A full ring drops the new record and returns
`false`; it never stalls the solver.

`completed_steps()`, `buffered_records()`, `config()`, and `summary()` expose
read-only runtime state. `print_summary(stream)` prints common performance and
I/O results. `pause_if_requested(suppress)` applies the configured manual-run
pause after project-specific output has been printed.

Select the module through `gmp_src_mgr` as `csp|cctl`; do not enumerate CSP
sources manually in project CMake files.
