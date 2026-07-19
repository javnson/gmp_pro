# GMP CTL Suite Guide

**English** | [简体中文](readme_cn.md)

`ctl/suite` contains complete cross-platform controller applications assembled
from `ctl/component`, `ctl/math_block`, `core`, and `csp`. A suite is the
preferred starting point for an actual product or experiment because it keeps
shared control logic, target adaptation, parameter engineering, simulation, and
build dependencies together.

## Suite catalog

| Suite | Application | Status |
| --- | --- | --- |
| [`dps_clllc`](dps_clllc) | Bidirectional isolated CLLLC / DAB control | Active |
| [`dps_fsbb`](dps_fsbb) | Four-switch buck-boost converter | Active |
| `mcs_acm` | Asynchronous motor control | Legacy layout |
| `mcs_pmsm` | Original PMSM control project | Deprecated for new work |
| [`mcs_pmsm_nt`](mcs_pmsm_nt) | Current PMSM control template | Recommended |
| [`mcs_pmsm_id`](mcs_pmsm_id) | PMSM parameter identification | Active |
| [`pgs_inv_GFL_inverter`](pgs_inv_GFL_inverter) | Three-phase grid-following inverter | Active |
| [`pgs_sinv_rc`](pgs_sinv_rc) | Single-phase converter with repetitive control | Active |

## Recommended layout

```text
<suite>/
├── src/                  shared, platform-independent control code
├── sdpe_general/         common topology and controller parameters
├── project/
│   ├── simulate/         PC controller and Simulink SIL adapter
│   └── <target>/         hardware project and xplt adapter
└── doc/                  verified procedures and archived results
```

Each target normally contains `gmp_src_mgr` for dependency generation,
`sdpe_mgr` for target parameters, and `xplt` for hardware or simulation I/O.
Simulation and hardware targets should share the same suite `src` control code.

## New project workflow

1. Copy the closest maintained suite or target project.
2. Configure common and target SDPE data.
3. Update `gmp_framework_config.json` and run the local source-manager scripts.
4. Adapt only the target `xplt` layer for ADC, PWM, UART, and fast protection.
5. Validate the simulation target before hardware commissioning.
6. Progress through documented `BUILD_LEVEL` stages with current-limited power.

See the [detailed Chinese suite guide](readme_cn.md) for the full architecture,
per-suite status, callbacks, SDPE layering, and migration notes.
