# Three-phase grid-following inverter

**English** | [简体中文](README_CN.md)

This suite implements a three-phase, two-level grid-following converter. Its shared controller source is used by F280039C Iris, LaunchXL-F280049C, PC simulation, and STM32G431 projects. The C2000 targets are hardware-validated; simulation supports software verification, while new STM32 deployments should follow the incremental commissioning procedure.

## Control stages

- `BUILD_LEVEL=1`: open-loop voltage generation.
- `BUILD_LEVEL=2`: islanded current-loop validation.
- `BUILD_LEVEL=3`: grid synchronization with PLL and sequence-current control.
- `BUILD_LEVEL=4`: decoupling, damping, and lead compensation.
- `BUILD_LEVEL=5`: complete active/reactive-power control.

The suite uses a two-layer SDPE model: common control settings are kept in `sdpe_general/`, while sampling, PWM, protection, and board mappings remain target-specific. The validated hardware combination includes the Helios three-phase GaN inverter and Harmonia LC filter.

Grid-connected commissioning involves hazardous voltage and energy. Complete isolated low-voltage tests, polarity checks, protection tests, and the lower build levels before connection to a live grid. See the [Chinese guide](README_CN.md) for detailed configuration and validation notes.
