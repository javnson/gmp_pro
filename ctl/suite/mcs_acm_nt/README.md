# AC Induction Motor Vector-Control Suite (MCS_ACM_NT)

**English** | [简体中文](README_CN.md)

This suite is the ACIM counterpart of `mcs_pmsm_nt`. It provides staged `BUILD_LEVEL` commissioning, a standalone ACIM current-loop core, and replaceable sensored and sensorless field-angle modules.

The central interface rule is that shaft position/speed belong to the mechanical loop, while Park/iPark require rotor-flux field angle and synchronous electrical speed. Every angle is turns-per-unit: `1 pu = 360 degrees = 2*pi rad`.

## Components

| Component | Responsibility |
|---|---|
| `current_loop/imfoc_core.h` | ACIM d/q current loop, decoupling, limiting, and transforms |
| `observer/acim_pos_calc.h` | Sensored slip and synchronous rotor-flux angle calculation |
| `observer/acim_fo.h` | Composite flux observer, PLL field angle, slip, and mechanical-speed estimate |
| `observer/ato_pll.h` | Shared turns-PU angle-tracking PLL |
| `interface/encoder_switcher.h` | Observer-agnostic angle transfer with speed hysteresis and debounce |
| `interface/sensorless_handover.h` | Coordinated angle ownership transfer and startup-to-closed-loop Id fade |

`imfoc_core` embeds none of the observers and does not wrap the PMSM FOC core.

## BUILD_LEVEL workflow

| Level | Function |
|---|---|
| 1 | V/f open-loop validation |
| 2 | I/F d/q current-loop validation |
| 3 | Sensored or sensorless real field-angle current loop |
| 4 | Mechanical speed loop |

Generate both common and target SDPE headers before building. The simulation project accepts `/p:BuildLevel=1..4` and `/p:AcimFeedbackMode=1|2` MSBuild overrides. Use `project/simulate/commissioning/run_build_level_sil.m` with `MCS_STD_ACM_MODEL.slx` for repeatable captures.

The SIL campaign has passed Levels 1, 2, sensored Level 3, and sensored Level 4. Sensorless acquisition and loss fallback are operational, but post-handover sensorless closure on the supplied model is not yet accepted for hardware use. Isolated F280049C PIL Level 1 has also passed on hardware; Levels 2 through 4 remain staged work. See [the Chinese commissioning record](doc/commissioning_record_cn.md) for evidence and remaining work.

The observer equations were compared with TI controlSUITE ACIFE/ACISE. GMP deliberately replaces direct `atan2` angle extraction with a PLL for better hardware robustness, while retaining the TI current model, voltage model, trapezoidal integration, and slip-equation lineage.
