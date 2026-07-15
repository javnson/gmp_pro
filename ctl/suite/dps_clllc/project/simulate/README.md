# CLLLC/DAB SIL simulation

This target shares the controller and physical settings with `f280025c_dioscuri`.
It contains a switching model with a 1:1 transformer, `Lm=120 uH`, symmetric
`Lr=20 uH` and `Cr=120 nF`, and a 75--150 kHz operating range.

1. Build `DigialPower_simulink.vcxproj` as `Debug|x64`.
2. Run `build_clllc_model` after changing the topology or sensor packaging.
3. Run `run_clllc_cosim(0.02)` from MATLAB R2024b, or use
   `run_clllc_validation(0.02)` to regenerate the JSON, MAT and PNG report.

The `Advanced ePWM` masked subsystem decodes four compare values and four phase
values, applies complementary dead time to all eight MOSFET gates, and emits one
ADC/ISR trigger after the Mask-configured number of switching cycles. The five ADC channels are primary
voltage/current, secondary voltage/current, and resonant-tank current.

`BUILD_LEVEL=1` is open-loop frequency modulation, level 2 closes the current
loop, and level 3 enables the parallel voltage/current competition controller.
Negative modulation commands select resonant-frequency DAB phase-shift mode.

The validated BUILD_LEVEL 1 run releases PWM at approximately 7 ms and produces
the commissioning plot in `validation/build_level_1_waveforms.png`.  See the
suite-level `doc/commissioning.md` for wiring, resource mapping and limitations.
