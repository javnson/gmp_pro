# Three-phase grid-forming inverter

**English** | [简体中文](README_CN.md)

This suite reuses `gfl_inv_ctrl_t` as the inner current-control kernel and
adds a stand-alone voltage loop, a replaceable grid-forming outer loop, and a
generic PLL-to-GFM bumpless transition. The common source is shared by the PC,
F280039C Iris, LaunchXL-F280049C, and STM32G431 targets.

## Commissioning levels

1. Open-loop voltage/PWM and sensing check.
2. Closed current loop with the current kernel's internal reference generator.
3. Stand-alone LC capacitor-voltage loop.
4. PLL-oriented grid current loop.
5. PLL synchronization, continuous lock qualification, bumpless transfer to
   the GFM angle and voltage loop, grid separation, and load-step operation.

The voltage loop uses ordinary PI controllers. Optional circular and square
current limits act on the complete d-q command, and
`ctl_pid_clamping_correction_using_real_output()` returns the final limited
actuator result to both integrators. The omega-C capacitor coupling
feed-forward can be disabled independently.

BUILD_LEVEL 5 selects `GFM_CONTROL_TECHNOLOGY` in common SDPE: `1` is P-f/Q-V
droop, `2` is a damped swing-equation VSM, and `3` combines the droop frequency
source with synchronous-frame virtual impedance. Virtual impedance conditions
the voltage reference and does not generate phase by itself. None of the three
outer-loop choices owns the voltage PI.
`inv_gfm_transition` tracks the PLL, initializes the forming angle from it,
normalizes the blended phasor, and ramps both the phasor and current command.

`USING_3D_SVPWM` selects A/B/C/N four-leg modulation and enables the QPR
zero-sequence path. A hardware target must map and protect its neutral leg
before enabling this option.

The automated MATLAB R2024b SIL workflow is documented in
[`project/simulate`](project/simulate/README.md). The current evidence is
software simulation and host testing only; no new GFM hardware validation is
claimed.
