# Three-phase grid-following inverter

**English** | [简体中文](README_CN.md)

This suite implements a three-phase, two-level grid-following converter. Its shared controller source is used by F280039C Iris, LaunchXL-F280049C, PC simulation, and STM32G431 projects. The C2000 targets are hardware-validated; simulation supports software verification, while new STM32 deployments should follow the incremental commissioning procedure.

## Control stages

- `BUILD_LEVEL=1`: open-loop voltage generation.
- `BUILD_LEVEL=2`: islanded current-loop validation.
- `BUILD_LEVEL=3`: grid synchronization with PLL and sequence-current control.
- `BUILD_LEVEL=4`: decoupling, damping, and lead compensation.
- `BUILD_LEVEL=5`: complete active/reactive-power control.
- `BUILD_LEVEL=6`: stand-alone LC capacitor-voltage control. An ordinary PI
  generates the d-q current reference; independently selectable
  circular/square limits act on the complete command and final-output clamping
  corrects the integrators, while the current loop uses its internal reference
  generator. The omega-C coupling
  feed-forward is independently switchable.

`USING_3D_SVPWM` selects four-leg A/B/C/N modulation and enables the tunable
QPR zero-sequence current controller. In level 6, the negative-sequence voltage
loop is enabled as well, providing separate positive-, negative-, and
zero-sequence control paths for unbalanced loads. A hardware target must map
and protect the fourth bridge leg before enabling this option.

The suite uses a two-layer SDPE model: common control settings are kept in `sdpe_general/`, while sampling, PWM, protection, and board mappings remain target-specific. The validated hardware combination includes the Helios three-phase GaN inverter and Harmonia LC filter.

Grid-connected commissioning involves hazardous voltage and energy. Complete isolated low-voltage tests, polarity checks, protection tests, and the lower build levels before connection to a live grid. See the [Chinese guide](README_CN.md) for detailed configuration and validation notes.

The PC project includes automated SDPE/build/SIL scripts in
[`project/simulate`](project/simulate/README.md). They run BUILD_LEVEL 1–6 and
save machine-readable metrics plus waveform images.
