# CTL motor-control components

**English** | [简体中文](readme_cn.md)

The motor-control tree supplies reusable blocks; complete controller
applications live under `ctl/suite`.

| Directory | Current implementation |
| --- | --- |
| `basic` | decoupling, encoder calibration, motor protection, V/f generation, and voltage reconstruction |
| `consultant` | ACIM, PMSM, inverter, mechanical, nameplate, per-unit, and unit calculations |
| `current_loop` | FOC, induction-motor FOC, DTC-SVM, and PMSM deadbeat predictive current control |
| `distributor` | IPM, SPM, and lookup-table field-weakening/current distribution |
| `interface` | encoder, encoder switching, universal motor interface, sensorless handover, startup excitation, and SIL structures |
| `mechanical_loop` | basic cascaded control, LADRC position/speed, MIT, and SMC mechanical controllers |
| `motion` | trapezoidal, S-curve, SOGI, and knob-position trajectories/controllers |
| `observer` | ACIM flux/position/SMO, ATO PLL, BLDC Hall/ZCD, PMSM ESMO/flux/HFI |
| `param_est` | PMSM stator-resistance MRAS header |
| `pmsm_offline_id` | PMSM offline-identification state machine and sensored implementation |
| `suite_pmsm`, `suite_acim` | reusable complete-controller wrappers retained inside the component tree |

`ctl/component/motor_control.h` is not a valid umbrella include: it still
contains 42 paths from removed layouts and names. Include concrete headers such
as `current_loop/foc_core.h` or `observer/acim_fo.h`, and select the matching
sources through `gmp_framework_dic.json`.

Angles and units are module contracts. Modern motor suites use turns-per-unit
where documented (`1 pu = 2*pi rad`), but callers must verify the selected
header and suite mapping. Host tests under `observer/tests/host_sim` cover a
focused observer set; hardware evidence belongs to the exact suite target and
`BUILD_LEVEL`, not to this whole directory.
