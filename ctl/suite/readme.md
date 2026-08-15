# GMP CTL Suite Guide

**English** | [简体中文](readme_cn.md)

`ctl/suite` contains complete cross-platform controller applications assembled
from `ctl/component`, `ctl/math_block`, `core`, and `csp`. The directory root is
an index only: suite directories and these guide documents belong here;
cross-suite generators and validation tools belong under `tools`.

## Suite catalog

| Suite | Application | Status |
| --- | --- | --- |
| [`dps_clllc`](dps_clllc) | Bidirectional isolated CLLLC / DAB control | Active |
| [`dps_fsbb`](dps_fsbb) | Four-switch buck-boost converter | Active |
| [`mcs_acm_nt`](mcs_acm_nt) | Asynchronous motor vector control | Active; SIL plus F280049C PIL project |
| [`mcs_pmsm_nt`](mcs_pmsm_nt) | Current PMSM control template | Recommended |
| [`mcs_pmsm_id`](mcs_pmsm_id) | PMSM parameter identification | Active |
| [`pgs_inv_GFL_inverter`](pgs_inv_GFL_inverter) | Three-phase grid-following inverter | Active |
| [`pgs_inv_GFM_inverter`](pgs_inv_GFM_inverter) | Three-phase grid-forming inverter | Active |
| [`pgs_sinv_2stage`](pgs_sinv_2stage) | Two-stage single-phase plant models | Three SDPE-bound models under `project/simulate`; no native controller build |
| [`pgs_sinv_rc`](pgs_sinv_rc) | Single-phase converter with repetitive control | Active |

## Standard layout

```text
<suite>/
├── src/                         shared controller/application code
├── sdpe_general/                suite-wide SDPE requirements and output
├── project/
│   ├── simulate/                host/SIL project, model and target SDPE layer
│   ├── <c2000-target>/
│   │   ├── C2000Lib/            TI device support, driverlib, headers and linker files
│   │   ├── src/                 GMP, user, SDPE, xplt and PIL files
│   │   │   ├── gmp_src_mgr/
│   │   │   ├── sdpe_mgr/
│   │   │   ├── xplt/
│   │   │   └── pil/             when the target provides PIL support
│   │   └── targetConfigs/       CCS connection files
│   └── <other-target>/          existing native layout; STM32 projects are retained
└── doc/
    └── simulation_result/       archived simulation evidence and data summary
```

The C2000 boundary is strict: TI-provided material belongs in `C2000Lib`, while
all GMP-provided, cross-platform, user, SDPE, source-manager, and PIL material
belongs in `src`. Simulation and hardware targets share the suite-level control
code. The deprecated `rt_trace` module is not part of a suite project.

Keep each C2000 startup assembly file, such as
`f28003x_codestartbranch.asm`, only in `C2000Lib/device_support`. When a SysConfig
file loads the `device_support` module, set
`device_support.useStandardCodeStartBranch = false;` so SysConfig does not add a
second C2000Ware copy. After a layout migration, run a clean/full build so CCS
regenerates its build directory and linker input list.

Generated SDPE outputs are disposable build inputs. Remove stale `.h` and `.m`
outputs before regeneration, then generate `sdpe_general` first and every target
`sdpe_mgr` second. Do not hand-edit generated files.

## Processor-in-the-loop deployment

Hardware projects that provide PIL compose their target requirements with the
common [`gmp_suite_pil.json`](../../tools/SDPE_v2/common_requirements/gmp_suite_pil.json)
contract. UART rate, command allocation, UDP ports, timeouts, and channel masks
are generated from SDPE data without repository-absolute paths.

PIL mode has two safety invariants:

- Hardware interrupts acknowledge peripherals and maintain time, but do not
  advance `gmp_base_ctl_step()` independently.
- Controller output-enable callbacks do not energize the physical power stage;
  Data Link transactions own the control steps.

After changing a target, inspect and regenerate its SDPE layers and build that
target. The repository no longer carries a fleet-wide validator in `ctl/suite`.

## Simulation evidence and model parameters

Stored simulation outputs belong in each suite's `doc/simulation_result`, whose
README records the available files and extracted metrics. Models load exactly
one suite-common and one simulation-target SDPE initialization script. Loading
an initializer is not sufficient: model block parameters must reference the
generated variables instead of duplicating numeric values.

Run the repository audit from its root:

```powershell
python tools/SDPE_v2/audit_suite_models.py --matlab-release R2024b
```

The command refreshes `doc/sdpe_model_parameter_audit.md` in every suite and
returns a non-zero status while ordinary models still contain candidate
hard-coded physical parameters.

## New project workflow

1. Copy the closest maintained suite or target project.
2. Configure common and target SDPE data; generate common before target output.
3. Update `gmp_framework_config.json` and run the local source-manager scripts.
4. Adapt only the target `xplt` layer for ADC, PWM, UART, and fast protection.
5. Validate the simulation target before hardware commissioning.
6. Progress through documented `BUILD_LEVEL` stages with current-limited power.

See the [detailed Chinese suite guide](readme_cn.md) for runtime boundaries and
the current platform inventory.
