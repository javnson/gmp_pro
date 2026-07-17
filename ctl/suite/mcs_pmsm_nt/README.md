# PMSM vector-control template

**English** | [简体中文](README_CN.md)

This is the recommended GMP permanent-magnet synchronous motor vector-control template. It shares controller code across C2000, STM32, and PC simulation targets and uses incremental `BUILD_LEVEL` stages to validate a drive from peripheral bring-up through closed-loop motion control.

## Main capabilities

- Field-oriented current and motion control.
- Modulation, encoder and speed processing, ramps, and ADC calibration.
- CiA 402 state-machine integration and GMP data-link support.
- Common SDPE configuration with target-specific peripheral mappings.

Supported project directories currently include `f280039c_Iris_node`, `f280049c`, `simulate`, `stm32f405`, `stm32g431`, and `stm32g474_hrtim`. New PMSM applications should normally start from this suite and retain the shared `src/` plus per-target `project/` structure.

Begin at the lowest build level and confirm current offsets, phase order, encoder direction, PWM polarity, and protection behavior before enabling the current, speed, or position loops. The [Chinese guide](README_CN.md) contains the detailed architecture, build-level procedure, and platform notes.
