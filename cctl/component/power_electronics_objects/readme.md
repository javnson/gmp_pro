# CCTL power-electronics objects

**English** | [简体中文](readme_cn.md)

This directory contains experimental C++ models of power converters, switching devices, and electrical machines for host-side analysis and simulation.

The currently validated offline-simulation entry points are:

- `inverter/three_phase_average_inverter.hpp`: averaged three-phase, two-level inverter with switch resistance, diode drop, and averaged dead-time error.
- `motor_model/pmsm_average_model.hpp`: coupled PMSM electrical and mechanical model with `id`, `iq`, mechanical speed, and mechanical angle states.

The universal buck-boost, DFIG, legacy induction-motor, and switch-level MOSFET directories remain historical experimental code. New PMSM simulations should start from the averaged model above. These objects are not embedded controller implementations; production control code still belongs under `ctl/component` or a suite's shared `src` directory.
