# PMSM vector control and offline parameter identification

**English** | [简体中文](README_CN.md)

This suite extends the GMP PMSM vector-control template with an offline identification engine. It can characterize the motor and prepare current-loop tuning while retaining the multi-platform, incremental `BUILD_LEVEL` workflow used by the standard motor-control suite.

## Identification scope

- Stator resistance and inductance.
- Permanent-magnet flux linkage.
- Inverter dead-time compensation.
- Mechanical inertia and friction.
- Sensored encoder offset and pole-pair calibration with fault diagnostics.
- Directional static/load torque.
- Initial current-loop PI tuning.

The shared implementation is centered on `src/`, including the `pmsm_offline_id_if` interface and the normal `ctl_main` control entry. Project targets include C2000, STM32, and PC simulation variants.

Parameter identification can deliberately energize or rotate the motor. Verify current scaling, phase order, protection limits, rotor freedom, and emergency-stop behavior at a low build level before starting an identification sequence. The [Chinese guide](README_CN.md) documents the state flow, parameters, and detailed operating procedure.

## PC SIL workflow

The simulation target now has two complementary runners that use the same native controller executable and packed UDP ABI:

```powershell
cd ctl\suite\mcs_pmsm_id\project\simulate
python .\run_pmsm_id_sil_fast.py --build
```

The averaged-plant runner completes encoder calibration, electrical identification, constant-Iq acceleration, and PWM-off coast-down, then writes JSON/CSV results. The current regression reaches `COMPLETE (8)` at 14.2535 simulated seconds. It detects four pole pairs and the encoder offset exactly to the simulated encoder LSB; errors are -2.45% inertia, -1.95% viscous friction, and -9.21% static load torque. Electrical-result errors remain within 2.4%.

The encoder safety paths are reproducible with `--encoder-fault random`, `stuck`, and `nonuniform`. They diagnose random jumps, missing shaft motion, and nonuniform electrical-cycle anchors respectively, and physically inhibit PWM on fault.

Use `run_pmsm_id_sil.m` for the detailed Simulink switching plant. Short smoke runs now exercise the encoder preparation stage first. Use a 1 us plant step for final correlation of the model's 1 us inverter dead time and allow enough time for the mechanical coast-down; this high-fidelity path is intentionally much slower. All identification stimuli, timings, speed ratios, and encoder fault thresholds are owned by the common `sdpe_general/sdpe_requirement.json` source. See the [Chinese guide](README_CN.md) for commands, monitor-channel mapping, and model truth values.

## Equal-bandwidth DQ-PI and DQ-LADRC1 simulation record (2026-08-08)

The FOC current loop uses DQ-PI by default. Uncomment `#define ENABLE_FOC_LADRC_CTRL` in `foc_core.h`, or define the same compiler macro, to select DQ-LADRC1. Both selections use `ctl_auto_tuning_foc_core()` and `ctl_init_foc_core()`.

The reproducible comparison is `ctl/component/intrinsic/complex/tests/host_sim/foc_current_loop_compare.c`. Conditions: 20 kHz sampling, `Ld=Lq=50 uH`, `Rs=0.13 ohm`, `24/sqrt(3) V` voltage base, `10 A` current base, a q-axis step from 0 to 0.3 pu, and a 0.9 pu circular limit. PI crossover bandwidth and LADRC `fc` are both 707.355 Hz; LADRC uses `fo=2*fc`. Feedforward and cross-coupling are disabled.

| Controller | Bandwidth (Hz) | 10%-90% rise (ms) | 2% settling (ms) | Overshoot (%) | IAE | Final iq (pu) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| DQ-PI | 707.355 | 0.450 | 0.950 | -0.000010 | 0.000052500 | 0.300000 |
| DQ-LADRC1 | 707.355 | 1.000 | 1.700 | -0.000020 | 0.000116859 | 0.300000 |

PI is faster for this nominal first-order plant; both controllers converge without visible overshoot. This is a regression comparison, not a robustness claim under parameter error or disturbance.

The Windows x64 Debug SIL projects for both `mcs_pmsm_nt` and `mcs_pmsm_id` were also built in default PI and `ENABLE_FOC_LADRC_CTRL` configurations. Both `MCS_STD_PMSM_MODEL.slx` models established their UDP link and ran without errors to 0.1 s in each configuration.
