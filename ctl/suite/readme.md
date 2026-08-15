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
| [`mcs_acm_nt`](mcs_acm_nt) | Asynchronous motor vector control | Active; SIL plus F280049C PIL project |
| [`mcs_pmsm_nt`](mcs_pmsm_nt) | Current PMSM control template | Recommended |
| [`mcs_pmsm_id`](mcs_pmsm_id) | PMSM parameter identification | Active |
| [`pgs_inv_GFL_inverter`](pgs_inv_GFL_inverter) | Three-phase grid-following inverter | Active |
| [`pgs_inv_GFM_inverter`](pgs_inv_GFM_inverter) | Three-phase grid-forming inverter | Active |
| `pgs_sinv_2stage` | Two-stage single-phase plant models | Model-only assets under `project/simulation`; no controller project |
| [`pgs_sinv_rc`](pgs_sinv_rc) | Single-phase converter with repetitive control | Active |

## Processor-in-the-loop deployment

The maintained hardware fleet uses the common
[`sdpe_pil`](sdpe_pil/sdpe_requirement.json) transport contract. Each target
combines it with its suite SDPE contract, so UART rate, command allocation, UDP
ports, timeouts, and channel masks are generated without absolute paths. The
`ENABLE_GMP_DL_PIL_SIM` selection is disabled by default and may be covered by
a target-private SDPE selection when PIL is required.

PIL mode has two safety invariants:

- Hardware interrupts acknowledge peripherals and maintain time, but
  `gmp_base_ctl_step()` does not advance the controller.
- Controller output-enable callbacks do not energize the physical power stage;
  control steps are owned by Data Link transactions.

The deployed modern fleet contains 19 hardware projects in `dps_clllc`,
`dps_fsbb`, `mcs_pmsm_id`, `mcs_pmsm_nt`, `pgs_inv_GFL_inverter`,
`pgs_inv_GFM_inverter`, and `pgs_sinv_rc`. Their existing `simulate` projects
remain SIL-only and do not bind the hardware PIL transport contract.

The removed legacy suite names `mcs_acm` and `mcs_pmsm` are **skipped by fleet PIL deployment**. They no longer have repository directories; old paths that
appear in historical manuals are not valid templates. `mcs_acm_nt/f280049c`
has its own isolated PIL evidence, but is not part of the 19-target fleet
validated by `validate_pil_deployment.py`.

After changing a suite target, run `python ctl/suite/validate_pil_deployment.py`
from the repository root. The check covers the maintained-target inventory,
SDPE composition, generated PIL source, channel masks, ISR isolation, and the
SIL/PIL project boundary.

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
