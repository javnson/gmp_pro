# CTL digital-power components

**English** | [简体中文](readme_cn.md)

The current implementation is organized as follows:

| Directory | Implemented scope |
| --- | --- |
| `basic` | `protectoion_strategy.h`, standard SIL data interface, virtual impedance, and `ctl_dp_basic.c` |
| `dcdc` | Buck, boost, CLLLC, FSBB, and shared DCDC core headers/sources |
| `inv` | GFL/GFM control, PQ/droop, transition, virtual impedance, VSM, harmonic/negative/zero-sequence control, voltage control, SRF/DDSRF/DSOGI PLL, and Vienna rectifier header |
| `mppt` | Incremental-conductance and perturb-and-observe algorithms |
| `sinv` | Single-phase inverter core, outer loop, protection, repetitive control, reference generation, SMS-PQ, SPFC, and SOGI PLL variants |

Include the concrete header and sources selected by the source-manager
registry. `ctl/component/digital_power.h` is a legacy umbrella and currently
contains 11 include paths for removed layouts such as `single_phase` and
`three_phase`; it is not a valid public include.

The `inv/tests/host_sim` project covers a focused GFM host simulation, and
`sinv/tests/sinv_qr_contract_test.c` covers a focused single-phase QR contract.
These checks do not imply that every module or hardware target is validated.
Use each suite README for its exact `BUILD_LEVEL`, signal convention, and
evidence.
