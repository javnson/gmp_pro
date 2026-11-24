# FSBB on LaunchXL-F280049C

**English** | [简体中文](README_CN.md)

Import this directory into CCS as `CTL_DP_FSBB_280049C` and build **Debug**. It uses the XDS110 LaunchPad target, C2000Ware 5.04, and the shared [FSBB controller](../../src). Refresh the source-manager and SDPE files after installing GMP or changing the requirements. Generate `../../sdpe_general` first, then `src/sdpe_mgr`, and run `src/gmp_src_mgr/gmp_generate_src.bat` before building.

Physical control starts with the outputs disabled and runs at 20 kHz, with hardware one-shot overcurrent trips on both ePWM modules. The LVFB board supplies `Vout` and `IL`. Set `FSBB_LAUNCHXL_FIXED_VIN` to the measured bench supply voltage, or enable `FSBB_ENABLE_VIN_SAMPLE` for an isolated `Vin` input on BoosterPack pin 24. Verify the build level and protection limits in [target SDPE](src/sdpe_mgr/sdpe_requirement.json) before powered tests. [Wiring and commissioning guide (Chinese)](README_WIRING_CN.md)

For PIL, enable `ENABLE_GMP_DL_PIL_SIM` in [target SDPE](src/sdpe_mgr/sdpe_requirement.json), regenerate, and rebuild for the chosen level. The physical gate remains disabled. A Data Link STEP executes the controller with simulated ADC inputs. The existing Simulink model receives the virtual enable and PWM results from the same GMP bridge used by the PMSM suite. See [PIL commands](README_CN.md#运行-pil).

Level 1 physical and PIL firmware both compiled and linked with TI C2000 22.6.1.LTS. The FSBB model and framed UDP bridge exchanged six samples in MATLAB R2025b using simulated Data Link responses. LaunchPad execution and powered LVFB tests are pending.
