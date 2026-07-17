# Single-phase inverter/rectifier with repetitive control

**English** | [简体中文](readme_cn.md)

This suite is a GMP reference framework for a bidirectional single-phase inverter and active-front-end rectifier. It uses a flat controller architecture and zero-copy signal binding so the same control design can be moved between the F280039C Iris target and the PC simulation target.

## Main capabilities

- Quasi-proportional-resonant and frequency-domain repetitive control.
- SOGI-based single-phase PLL.
- Active/reactive-power commands with soft start.
- Unipolar SPWM, dead-time handling, and staged protection checks.
- CiA 402 state management and GMP data-link integration.

Use the project's `BUILD_LEVEL` stages in order. Validate measurements, polarity, PWM, dead time, and protection with an isolated low-voltage setup before closing current, voltage, power, or repetitive-control loops. Common settings belong in `sdpe_general/`; target-specific bindings remain under `project/`.

The [Chinese guide](readme_cn.md) contains the detailed controller description. Simulation procedures and measured results are recorded in the [UDP/SIL experiment report](doc/README.md).
