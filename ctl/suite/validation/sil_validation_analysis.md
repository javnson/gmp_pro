# CTL suite SIL validation analysis

Validation date: 2026-08-15  
Toolchain: MATLAB/Simulink R2024b, Visual Studio x64 Debug SIL targets

## Scope and method

The fleet contains eight Simulink SIL projects and 35 supported `BUILD_LEVEL`
configurations. Each executable configuration is generated through SDPE, its
project-local GMP sources are synchronized, and the target is fully rebuilt to
prevent stale object files from invalidating the result. The Simulink validator
then records:

- native or Simulink assertions and transport failures;
- a step or command-transition response, including rise time, settling time,
  and overshoot where the signal semantics permit it;
- final-window absolute and percentage steady-state error;
- suite-specific safety and state-machine conditions.

The machine-readable fleet results are in `sil_validation_fleet_summary.json`
and `sil_validation_fleet_summary.csv`. Waveforms, MATLAB logs, and per-level
JSON metrics remain under each project's `project/simulate/validation` folder.

## Result summary

| Suite | Levels | Overall pass | Simulation unavailable | Dynamic failures | Steady-state failures |
|---|---:|---:|---:|---:|---:|
| `dps_clllc` | 4 | 1 | 0 | 3 | 3 |
| `dps_fsbb` | 3 | 3 | 0 | 0 | 0 |
| `mcs_acm_nt` | 4 | 0 | 0 | 4 | 2 |
| `mcs_pmsm_id` | 4 | 0 | 4 | 0 | 0 |
| `mcs_pmsm_nt` | 4 | 0 | 0 | 4 | 4 |
| `pgs_inv_GFL_inverter` | 6 | 4 | 0 | 2 | 0 |
| `pgs_inv_GFM_inverter` | 5 | 4 | 0 | 1 | 0 |
| `pgs_sinv_rc` | 5 | 1 | 0 | 4 | 2 |
| **Total** | **35** | **13** | **4** | **14** | **11** |

No runtime assertion remains in a completed SIL simulation. During validation,
ACIM initially exposed a stale generated object that still asserted on a valid
zero-frequency soft start. Source synchronization plus a deterministic rebuild
removed that stale-code failure; all four ACIM levels subsequently completed.

## Findings by suite

### DC/DC suites

- `dps_fsbb` BL1-BL3 pass. Steady-state errors are 4.94%, 0.0031%, and
  0.00061%; settling times are 0.340 s, 0.330 s, and 0.219 s.
- `dps_clllc` BL1 passes. BL2 has 153.37% steady error and sustained resonant
  current/switching oscillation. BL3 output collapses to zero before the end of
  the run (100% error). BL4 reaches only about 8.44 V for a 48 V target
  (82.41% error). These are control/model failures, not SIL transport failures.

### Motor-control suites

- `mcs_acm_nt` now runs at every level without assertion. BL2 and BL3 have
  excellent steady-state current error (0.024% and 0.028%) but fail the present
  transient limits; BL3 settles at 0.599 s, effectively at the 0.6 s stop time.
  BL1 is a frequency ramp rather than a step, so its current step metric is not
  meaningful. BL4 has 92.57% speed error within 0.6 s. The next revision should
  inject explicit post-start steps and use a longer mechanical settling window.
- `mcs_pmsm_nt` completes SIL at BL1-BL4 but produces zero plant current and
  speed. BL1/3/4 show 100% steady error; BL2 remains failed after extending its
  run to 1.5 s. Its voltage command rises to saturation while plant current
  remains zero, indicating an enable/PWM/plant-interface defect rather than
  insufficient simulation duration.
- `mcs_pmsm_id` cannot establish its Simulink UDP session within 5000 ms. BL2
  was rebuilt and directly retested; the native controller remains waiting
  while the S-function times out. Because all levels use the same transport and
  model, the fleet marks BL1-BL4 as simulation unavailable. Existing non-Simulink
  fast tests are not counted as SIL evidence.

### Three-phase inverter suites

- `pgs_inv_GFL_inverter` BL2, BL3, BL4, and BL6 pass. Their steady errors are
  0.00086%, 0.146%, 0.0010%, and 0.560%. BL1 and BL5 meet their steady-state
  criteria but have no valid settling interval under the generic transition
  metric; neither failure is an assertion or steady-state control failure.
- `pgs_inv_GFM_inverter` BL2-BL5 pass. BL4 passes after evaluating the 10 ms
  smoothed dq-voltage envelope (0.144% steady error, 43.9 ms settling), which
  separates switching ripple from the control trajectory. BL5, including PMSG
  synchronization, GFM takeover, load step, inertia and capacity checks, passes
  with 1.27% steady error and 0.868 s settling. BL1 is steady (0.001% error) but
  lacks an explicit command step, so the generic dynamic metric remains invalid.

### Single-phase inverter suite

- `pgs_sinv_rc` BL4 passes (0.0277% error, 0.451 s settling).
- BL2 and BL3 meet steady-state accuracy (0.150% and 0.0673%) but fail dynamic
  overshoot/settling limits.
- BL1 produces 13.63 Vrms against an 8.41 Vrms reference (61.77% error).
- BL5 regulates the mean DC bus accurately (0.0356% error), but the divergence
  diagnostic ends at 2.56 and the transient overshoot is excessive, so both the
  dynamic and suite-specific steady-state gates fail.

## Recommended correction order

1. Repair the PMSM-ID model/controller UDP contract, then rerun BL1-BL4.
2. Trace the PMSM normal-suite output-enable/PWM path because the controller
   commands voltage while the motor electrical state remains stationary.
3. Inspect CLLLC BL2 resonant-loop stability and BL3/BL4 voltage-loop polarity,
   scaling, and saturation.
4. Add explicit validation-only post-start steps to ACIM BL1/BL4, GFL BL1/BL5,
   GFM BL1, and SINV BL2/BL3 so dynamic metrics describe a controlled transition
   rather than startup, a ramp, or a mode handover.
5. Diagnose SINV BL5's nonzero divergence monitor before tuning its transient.

