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

Positive modulation uses variable-frequency CLLLC operation.  Negative
modulation uses a signed secondary-bridge phase shift at the resonant frequency
for reverse DAB power transfer.  `BUILD_LEVEL=1` is open loop, level 2 is the
current loop, and level 3 is the voltage/current parallel competition loop.
The CLLLC hardware consultant derives conservative current- and voltage-loop PI
parameters from the tank, load, output capacitor, switching and resonant limits.

## Dioscuri F280025C resources

The PCB signal names cross the natural ePWM numbering; the mapping below is
intentional and must not be simplified in software.

| Converter signal | F280025C resource | GPIO / ADC input |
|---|---|---|
| Primary bridge leg A | ePWM2 A/B | GPIO2 / GPIO3 |
| Primary bridge leg B | ePWM1 A/B | GPIO0 / GPIO1 |
| Secondary bridge leg A | ePWM4 A/B | GPIO22 / GPIO23 |
| Secondary bridge leg B | ePWM3 A/B | GPIO4 / GPIO5 |
| Primary current | ADCA SOC0 | A6 |
| Secondary current | ADCA SOC1 | A2 |
| Secondary voltage | ADCA SOC2 | A3 (also shown as C5 on the schematic net) |
| Primary voltage | ADCC SOC0 | C6 |
| USB UART | SCIA | GPIO16 / GPIO17 |

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
ctl\suite\dps_clllc\project\f280025c_dioscuri\src\gmp_src_mgr\gmp_generate_src.bat
cd ctl\suite\dps_clllc\project\f280025c_dioscuri\Debug
C:\ti\ccs2002\ccs\utils\bin\gmake.exe -j4 all
```

The generated image is `Debug/CTL_DP_CLLLC_Dioscuri.out`.  See
[`doc/commissioning.md`](doc/commissioning.md) for the SIL build, model Mask,
signal wiring, validation procedure and present commissioning limits.
