# GMP architecture and source-of-truth map

## Contents

1. Repository layers
2. Runtime and control flow
3. Configuration composition
4. Suite structure
5. Generation and distribution
6. Installation and host tools
7. Simulink/SIL boundary

## 1. Repository layers

| Area | Responsibility | Representative entry points |
| --- | --- | --- |
| Root | Public aggregate headers and user entry BAT files | `gmp_core.h`, `gmp_core.hpp`, `install_gmp*.bat`, `gmp_env.bat` |
| `core/std` | Types, compiler/config composition, lifecycle, weak ports | `gmp.std.h`, `gmp_core_func.h`, `gmp_cport.h`, `src/gmp_std_port.c` |
| `core/dev` | Datalink, tunable variables, PIL, devices and peripheral interfaces | `gmp_datalink`, `gmp_tunable`, `gmp_pil_core` sources |
| `core/pm` | Cooperative tasks, schedulers, workflow/state machines | `function_scheduler.h`, `state_machine.h` |
| `core/mm` | Optional memory management | `block_mem.h` |
| `csp` | Chip/host startup, type definitions and peripheral services | `csp.general.h`, `csp.config.h`, platform `src/` |
| `ctl/math_block` | Numeric primitives compatible with `ctrl_gt` | `gmp_math.h` |
| `ctl/component` | Reusable real-time control modules and interfaces | component headers and selected `src/*.c` |
| `ctl/framework` | Control dispatch and application state frameworks | `ctl_dispatch.h`, CiA 402 support |
| `ctl/hardware_preset` | Physical component/board parameter presets and SDPE fragments | headers plus `sdpe_src/*.json` |
| `ctl/suite` | Complete cross-platform applications | shared `src`, common SDPE, per-target projects |
| `cctl` | Experimental/host-oriented C++ control and plant objects | numerical solvers and power-electronics objects |
| `slib` | MATLAB/Simulink libraries and SIL bridge artifacts | source SLX, MATLAB helpers, Release output |
| `tools` | Installers, SDPE, source/facility generators, SIL/PIL and debugger tools | each tool's canonical directory |

## 2. Runtime and control flow

`gmp_core.h` includes `core/std/gmp.std.h` before `gmp_core_func.h`. Configuration therefore exists before inline lifecycle functions are compiled.

Initialization in `gmp_base_init()` is ordered:

1. `gmp_csp_startup()` unless CSP is disabled.
2. user/platform `setup_peripheral()`.
3. optional GMP banner.
4. controller `ctl_init()` when CTL is enabled.
5. application `init()`.
6. `gmp_csp_post_process()`.

`gmp_base_entry()` then calls `gmp_base_loop()`. Hardware normally loops forever; `SPECIFY_PC_ENVIRONMENT` uses `PC_ENV_MAX_ITERATION`. The background loop calls CSP work, framework state dispatch, user `mainloop()`, and controller `ctl_mainloop()`.

The fast control path is separate. A hardware ISR or the Windows SIL packet loop calls `gmp_base_ctl_step()` from `ctl/framework/ctl_dispatch.h`:

- standard framework: `ctl_input_callback()`, `ctl_dispatch()`, `ctl_output_callback()`;
- nano framework: `ctl_fm_periodic_dispatch()` unless manually dispatched, followed by user `ctl_dispatch()`.

User lifecycle functions have weak/default implementations for supported compilers, but target applications normally provide `setup_peripheral`, `ctl_init`, `init`, `mainloop`, `ctl_mainloop`, and the inline/callback control stages.

## 3. Configuration composition

`core/std/gmp.std.h` composes configuration in this order:

1. GMP option constants from `core/std/cfg/options.cfg.h`.
2. project/platform `xplt.config.h`.
3. CSP `csp.config.h` unless CSP is disabled.
4. GMP defaults from `core/std/cfg/gmp.cfg.h`.
5. validation from `core/std/cfg/validate.cfg.h`.
6. compiler, error-code, CSP type, GMP type, endian and peripheral definitions.
7. CTL config, math, application `ctl_main.h`, `xplt.ctl_interface.h`, and dispatch when CTL is enabled.

This order is why project configuration and include paths are part of the ABI. Review macro definitions across the complete chain before moving or renaming them.

`ctrl_gt` is selectable (float, double, or supported fixed-point mode). Reusable CTL code must use GMP math/conversion helpers rather than assuming native `float` semantics.

## 4. Suite structure

The recommended suite layout is:

```text
ctl/suite/<suite>/
├── src/                         shared controller/application code
├── sdpe_general/                common physical/control parameter input and output
├── project/
│   ├── simulate/                Windows/Visual Studio + Simulink target
│   │   ├── xplt/                host ADC/PWM/buffer binding
│   │   ├── sdpe_mgr/            simulation-specific SDPE layer
│   │   └── gmp_src_mgr/         local module selection and generated source tree
│   └── <hardware-target>/       CCS/STM32/etc. project and platform binding
└── README.md / README_CN.md
```

Shared `src` owns control laws, state machines, tunable dictionaries and application tasks. Target projects own startup, registers, peripheral routing, raw ADC/PWM conversion, compiler settings and platform-specific generated configuration.

`BUILD_LEVEL` is suite-specific. It is a commissioning contract, not a repository-wide enumeration. Read the suite generated settings and controller branches before changing a level.

## 5. Generation and distribution

### Source manager

Canonical data and engines:

- module registry: `tools/facilities_generator/src_mgr/gmp_framework_dic.json`;
- project discovery: `framework_project_discovery.py`;
- header mirror: `framework_sync_inc_v3.py`;
- flattened C/C++ source copy: `framework_sync_src_v3.py`;
- distributed BAT template: `tools/facilities_generator/src_mgr/gmp_src_mgr`;
- fleet distributor: `framework_distribute_tools_v3.py`.

Per-project `gmp_framework_config.json` selects registry modules and `sync_mode`. Header output mirrors paths below `gmp_inc`; source output is flattened below `gmp_src`. The generated `gmp_compiler_includes.txt` is machine-specific and ignored.

Fleet distribution searches roots from `deploy_targets.json`, filters candidates with `git check-ignore`, preserves project configuration, copies canonical scripts, then runs header generation before source generation.

### SDPE

`tools/SDPE_v2` owns the generator/editor and canonical four BAT wrappers. `distribute_sdpe_mgr.py` finds `sdpe_mgr` and `sdpe_general` directories containing `sdpe_requirement.json`, applies Git ignore rules, and atomically refreshes only the BAT wrappers.

Suite SDPE commonly has a general physical/control layer and a target peripheral/build layer. Generated `.h` and `*_matlab_init.m` files are consumed by C and Simulink respectively. They duplicate information available from the GMP library and SDPE requirements, so the main repository may ignore them. Once a user copies a project outside GMP, the standalone repository must commit these generated files so it remains buildable without the parent library's generation workflow.

## 6. Installation and host tools

All entry points register `GMP_PRO_LOCATION` first.

- system mode uses Scoop-managed user applications/Python and user-wide vcpkg integration;
- private mode builds/copies `bin`, uses Python 3.12.10, pinned applications, repository-local vcpkg and a completion marker;
- Visual Studio is optional, but suite-native simulation packages require the x64 C++ workload;
- proxy choice is persisted for the private environment and exported to downloads/vcpkg/Visual Studio children.

After base tools, both modes configure the repository: CCS product registration, facility JSON generation, source-manager distribution plus generation, SDPE wrapper distribution, and standalone project-ignore distribution.

## 7. Simulink/SIL boundary

Read `slib/readme.md` for the detailed library contract.

The Windows SIL CSP receives a packed suite-defined buffer over ASIO UDP, updates platform inputs, invokes `gmp_base_ctl_step`, and transmits outputs. The native buffer types are selected by `gmp_pc_simulink_rx_buffer_t` and `gmp_pc_simulink_tx_buffer_t` macros. Model packing and target binding must be changed as one ABI.

Linux SIL is an official platform direction but is not yet optimized or complete. Preserve the existing Linux core and describe missing helper binaries or validation as implementation gaps, not as a decision to drop Linux.

`gmp_run_model_sdpe_init` derives the suite layout from the saved SLX path and loads exactly one common and one target MATLAB initialization script, in that order.
