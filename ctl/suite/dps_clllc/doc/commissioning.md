# CLLLC / DAB commissioning guide

## Safety and first hardware test

Start with a current-limited low-voltage DC supply and a resistive secondary
load.  Confirm all eight gate signals, polarity and dead time with the power
stage disabled before fitting/enabling the MOSFET supply.  The control ISR is
triggered by ePWM1 once every `CLLLC_PWM_CYCLES_PER_CONTROL` switching periods;
CPU Timer0 supplies the independent 1 kHz GMP scheduler tick.  This separation
is mandatory because the CLLLC switching frequency changes at run time.

The startup task automatically requests `CIA402_CMD_ENABLE_OPERATION`.  Each
hardware transition retains a 100 ms delay, allowing ADC offset calibration and
power-stage checks before PWM release.  Current channels use TMCS1133-B5A
scaling (150 mV/A, 1.65 V zero-current bias); voltage channels use the AMC1311
chain scaling of approximately 27.05 mV/V.

## SIL build and model generation

Use MATLAB R2024b.  Do not edit `slib/install_path/R2024b`; the project copies
the installed voltage/current sensor blocks into its generated model without a
library link.

1. Run `project/simulate/gmp_src_mgr/gmp_generate_src.bat`.
2. Build `project/simulate/DigialPower_simulink.vcxproj` as `Debug|x64`.
3. In MATLAB, change to `project/simulate` and run `build_clllc_model`.
4. Open `GMP_STD_CLLLC_MODEL.slx` and inspect the `GMP STD CLLLC Module` Mask.
5. Run `run_clllc_cosim(0.02)` for an interactive run, or
   `run_clllc_validation(0.02)` to save the report artifacts.

The Advanced ePWM Mask groups timebase, complementary dead band and ADC/ISR
sampling.  `TimerClockHz`, `NominalPeriodTicks`, `DeadtimeSeconds`, and
`AdcCyclesPerIsr` participate directly in gate/trigger generation.  The eight
gates are ordered for the two Simscape Universal Bridge blocks as primary
`[Aupper Bupper Alower Blower]`, followed by the same secondary order.

ADC UDP order is fixed as primary voltage, primary current, secondary voltage,
secondary current and resonant current, followed by 19 unused channels.  Sensor
blocks output integer ADC codes rather than engineering units.  Controller
monitor channels return period, dead time, primary voltage/current, secondary
voltage, resonant current and modulation command.

## Current validation result

The 20 ms BUILD_LEVEL 1 run completed a real UDP handshake and automatic CiA402
startup.  PWM was released at approximately 7 ms.  The resonant tank showed a
damped startup transient and the secondary capacitor began charging.  At the
end of the run the measured values were:

| Quantity | Result |
|---|---:|
| Primary voltage | 47.9824 V |
| Primary current | -3.3462 A |
| Secondary voltage | 7.7737 V |
| Resonant current | -3.3462 A |
| Open-loop command | 0.1 pu |

Artifacts are saved under `project/simulate/validation` as JSON, MAT and PNG.
This is a software/SIL commissioning result, not yet a claim of tuned closed-loop
hardware performance.  BUILD_LEVEL 2 and 3 use the implemented auto-tuned
current and voltage/current competition controllers and should be commissioned
only after the open-loop polarity, current scaling and tank parameters have
been verified on the physical module.

