# Native FSBB CCTL simulation

**English** | [简体中文](README_CN.md)

Run `build_test.bat 3 --no-pause` from this directory. The script generates SDPE settings, GMP sources, and the Eigen plant and then builds and runs the CTest experiment. Select `1` for open-loop voltage, `2` for inductor-current feedback, or `3` for cascaded voltage/current feedback. The default is level 3.

The CCTL CSP runs the existing [FSBB controller](../../src/ctl_main.c) and [user scheduler](../../src/user_main.c). A simulated ePWM triggers ADC conversion at 20 kHz. The ADC publishes four channel results before invoking the same GMP control ISR used by the embedded target. The switched circuit advances every 100 ns.

[The plant generator](hw/generate_closed_loop.py) uses the existing [TINA topology](hw/FSBB.CIR), with the shared FSBB parameter set for L, C, and load and the Simulink MOSFET channel resistance. Levels 1 and 2 run at nominal 24 V input for 1.2 s. Level 3 runs for 1.8 s and steps the input from 32 V to 18 V at 1.2 s with a 24 V output reference and a 20 Ω load. See the [run instructions and channel map](README_CN.md).

All three CTest experiments passed on the Windows Release build. Level 3 held about 24.01 V in both modes. During the input step, the voltage dipped to 22.65 V and returned to within ±2% of 24 V after 55.73 ms. [Level 3 waveforms](../../doc/simulation_result/cctl/level_3_waveforms.png)
