# GMP CLLLC / DAB digital-power project

**English** | [简体中文](README_CN.md)

This suite implements the Dioscuri bidirectional isolated converter on the
TMS320F280025C and provides a MATLAB/Simulink SIL commissioning target.  The
same controller source is used by both targets.

## Electrical and control baseline

| Item | Value |
|---|---:|
| Transformer ratio | 1:1 |
| Magnetizing inductance | 120 uH |
| Primary / secondary leakage inductance | 20 uH / 20 uH |
| Primary / secondary resonant capacitance | 120 nF / 120 nF |
| Resonant / operating frequency | 100 kHz / 75--150 kHz |
| Nominal dead time | 200 ns |
| Control interrupt | every 2 PWM periods (50 kHz nominal) |
| GMP system tick | CPU Timer0, 1 kHz, independent of PWM frequency |

The signed modulation command jointly changes switching frequency and the
primary-to-secondary phase shift.  Command magnitude moves from 150 kHz toward
75 kHz and increases phase displacement; command sign selects the requested
power-flow direction.  The four commissioning levels are:

1. `BUILD_LEVEL=1`: open-loop modulation and polarity checkout.
2. `BUILD_LEVEL=2`: primary/input-current closed loop.
3. `BUILD_LEVEL=3`: output-voltage closed loop.
4. `BUILD_LEVEL=4`: parallel CC/CV competition; the lower modulation demand
   wins and the inactive PI integrator is synchronized for anti-windup.

The CLLLC hardware consultant derives conservative current- and voltage-loop PI
parameters from the tank, load, output capacitor, switching and resonant limits.

## Two-layer SDPE configuration

The suite follows the same common/target split as `dps_fsbb`:

- `sdpe_general/sdpe_requirement.json` owns the resonant tank, TMCS1133B5A
  current sensing, AMC1311/divider voltage sensing, per-unit bases, targets,
  loop bandwidths and joint-modulation limits.
- `project/f280025c_dioscuri/src/sdpe_mgr/sdpe_requirement.json` owns
  `BUILD_LEVEL`, six-pair PWM routing, synchronization, ADC trigger, CPU Timer,
  UART and GPIO bindings.

Generate the common layer first, then the target layer.  `ctrl_settings.h` is a
thin compatibility include and no longer owns physical constants.

## Dioscuri F280025C resources

The PCB signal names cross the natural ePWM numbering; the mapping below is
intentional and must not be simplified in software.

| Converter signal | F280025C resource | GPIO / ADC input |
|---|---|---|
| Primary PRI_P / ControlPort1 | ePWM4 A/B | GPIO22 / GPIO23 |
| Primary PRI_N / ControlPort2 | ePWM7 A/B | GPIO28 / GPIO29 |
| Secondary SEC_P / ControlPort4 | ePWM1 A/B | GPIO0 / GPIO1 |
| Secondary SEC_N / ControlPort3 | ePWM2 A/B | GPIO2 / GPIO3 |
| Reserved FSBB pair 1 | ePWM3 A/B | GPIO4 / GPIO5 |
| Reserved FSBB pair 2 | ePWM5 A/B | GPIO8 / GPIO9 |
| Primary current | ADCA SOC0 | A6 |
| Secondary current | ADCA SOC1 | A2 |
| Secondary voltage | ADCA SOC2 | A3 (also shown as C5 on the schematic net) |
| Primary voltage | ADCC SOC0 | C6 |
| USB UART | SCIA | GPIO16 / GPIO17 |

The SDPE target prints all six pairs as `DIOSCURI_PWM_PAIR1_BASE` through
`DIOSCURI_PWM_PAIR6_BASE`.  The four logical CLLLC legs select from these
options, so bridge order—and therefore the physical positive-current
direction—can be corrected without changing controller code.  The four active
legs must be distinct, the sync master must be one of them, and the SysConfig
ADC SOC trigger must match `CLLLC_ADC_TRIGGER_PWM_BASE`.

At `BUILD_LEVEL=1`, write `g_modulation_target_user` through Datalink to vary
the signed hybrid frequency/phase command online.  `g_modulation_command`
remains the read-only command actually applied to the modulator.

`GPIO_ENABLE` is GPIO41 and drives the active-low OE# input of the
SN74LVC8T245PWR.  SysConfig initializes it high.  Firmware keeps OE# high while
configuring PWM, raises it before forcing trips, and only pulls it low after all
four selected trip flags have been cleared.

The UCC21520 hardware dead-time option is disabled on this board, therefore
DBRED and DBFED are always written by the DSP.  The schematic exposes no
unambiguous MCU net for `DSP_ADC_CAVITY_CURT`; hardware currently uses primary
current as the resonant-current feedback fallback.  The SIL model provides an
independent resonant current sensor.

The schematic labels CAN as GPIO31/30, while the selected F280025C 64-pin
device pinmux only permits CANA on GPIO32/33.  SysConfig uses the valid device
mapping; using CAN on this PCB requires checking/reworking that hardware route.

## Build and run

Hardware source synchronization and build:

```powershell
ctl\suite\dps_clllc\sdpe_general\sdpe_generate.bat
ctl\suite\dps_clllc\project\f280025c_dioscuri\src\sdpe_mgr\sdpe_generate.bat
ctl\suite\dps_clllc\project\f280025c_dioscuri\src\gmp_src_mgr\gmp_generate_src.bat
```

Import `project/f280025c_dioscuri` into CCS and build the `Debug`
configuration.  SysConfig must run before the compiler because the SDPE board
entity deliberately binds to the generated peripheral symbols.

The generated image is `Debug/CTL_DP_CLLLC_Dioscuri.out`.  See
[`doc/commissioning.md`](doc/commissioning.md) for the SIL build, model Mask,
signal wiring, validation procedure and present commissioning limits.
