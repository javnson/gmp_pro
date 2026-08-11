# PMSM vector-control template

**English** | [简体中文](README_CN.md)

This is the recommended GMP permanent-magnet synchronous motor vector-control template. It shares controller code across C2000, STM32, and PC simulation targets and uses incremental `BUILD_LEVEL` stages to validate a drive from peripheral bring-up through closed-loop motion control.

## Main capabilities

- Field-oriented current and motion control.
- Modulation, encoder and speed processing, ramps, and ADC calibration.
- CiA 402 state-machine integration and GMP data-link support.
- Common SDPE configuration with target-specific peripheral mappings.

Supported project directories currently include `f280039c_Iris_node`,
`f280049c`, `f29h85x_lp_3phgan`, `simulate`, `stm32f405`, `stm32g431`, and
`stm32g474_hrtim`. The F29H85x target requires CCS 21 or newer and uses the
`GMP-Core-C29x` Product plus two-layer SDPE bindings for the LaunchPad and
BOOSTXL-3PHGANINV resources. New PMSM applications should normally start from
this suite and retain the shared `src/` plus per-target `project/` structure.

Begin at the lowest build level and confirm current offsets, phase order, encoder direction, PWM polarity, and protection behavior before enabling the current, speed, or position loops. The [Chinese guide](README_CN.md) contains the detailed architecture, build-level procedure, and platform notes.

## Equal-bandwidth DQ-PI and DQ-LADRC1 simulation record (2026-08-08)

The FOC current loop uses DQ-PI by default. Uncomment `#define ENABLE_FOC_LADRC_CTRL` in `foc_core.h`, or define the same compiler macro, to select DQ-LADRC1. Both selections use `ctl_auto_tuning_foc_core()` and `ctl_init_foc_core()`.

The reproducible comparison is `ctl/component/intrinsic/complex/tests/host_sim/foc_current_loop_compare.c`. Conditions: 20 kHz sampling, `Ld=Lq=50 uH`, `Rs=0.13 ohm`, `24/sqrt(3) V` voltage base, `10 A` current base, a q-axis step from 0 to 0.3 pu, and a 0.9 pu circular limit. PI crossover bandwidth and LADRC `fc` are both 707.355 Hz; LADRC uses `fo=2*fc`. Feedforward and cross-coupling are disabled.

| Controller | Bandwidth (Hz) | 10%-90% rise (ms) | 2% settling (ms) | Overshoot (%) | IAE | Final iq (pu) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DQ-PI | 707.355 | 0.450 | 0.950 | -0.000010 | 0.000052500 | 0.300000 |
| DQ-LADRC1 | 707.355 | 1.000 | 1.700 | -0.000020 | 0.000116859 | 0.300000 |

PI is faster for this nominal first-order plant; both controllers converge without visible overshoot. This is a regression comparison, not a robustness claim under parameter error or disturbance.

The Windows x64 Debug SIL projects for both `mcs_pmsm_nt` and `mcs_pmsm_id` were also built in default PI and `ENABLE_FOC_LADRC_CTRL` configurations. Both `MCS_STD_PMSM_MODEL.slx` models established their UDP link and ran without errors to 0.1 s in each configuration.

## F280049C Processor-in-the-Loop

The F280049C target provides a target-local SDPE feature named
`ENABLE_GMP_DL_PIL_SIM`. It is independent of the shared suite configuration:
disabled builds retain the normal physical ADC/PWM control path, while enabled
builds run controller steps only from validated Data Link PIL requests and keep
the physical gate/PWM outputs forced safe. The UART/UDP endpoints, command base,
masks, and every plant channel mapping are also managed by the target SDPE file.

The reproducible staged workflow, MATLAB runner, safety contract, and recorded
hardware results are in [`project/f280049c/pil`](project/f280049c/pil/README.md).
