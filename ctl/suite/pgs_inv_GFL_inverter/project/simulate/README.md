# GFL inverter automated SIL validation

The scripts in this folder follow the same controller-process plus Simulink
plant layout as `pgs_sinv_rc/project/simulate`.

- `run_gfl_cosim(level, stop_time)` runs one selected model.
- `run_gfl_validation(level, stop_time, label)` logs controller signals,
  writes `../../doc/simulation_result/*_metrics.json` and `*_waveforms.png`, and evaluates
  finite/output-enable checks. BUILD_LEVEL 6 additionally checks capacitor
  voltage tracking and the selectable 0.8-pu circular/square current limits.
- `run_build_level_matrix.ps1` regenerates SDPE, rebuilds Debug x64, and runs
  BUILD_LEVEL 1 through 6. It restores the checked-in SDPE selection even if a
  case fails.
- `run_pq_droop_validation.ps1` selects BUILD_LEVEL 5, enables PQ droop,
  rebuilds, validates the droop reference change and closed-loop response, and
  restores both SDPE files afterward.

Run the complete matrix from PowerShell:

```powershell
$env:GMP_PRO_LOCATION = 'E:\lib\gmp_pro'
.\run_build_level_matrix.ps1
.\run_pq_droop_validation.ps1
```

BUILD_LEVEL mapping:

1. stand-alone open-loop voltage and PWM/sensing check;
2. stand-alone d-q current loop using the internal reference generator;
3. grid PLL plus positive/negative-sequence current control;
4. level 3 plus decoupling, active damping and lead compensation;
5. grid P/Q outer loop;
6. stand-alone LC capacitor-voltage ordinary PI outer loop with final vector
   limiting and clamping correction.

`USING_3D_SVPWM` enables the fourth neutral-leg PWM (`pwm_cmp[3]`) and
zero-sequence QPR path. The supplied legacy `.slx` plants expose three bridge
legs, so full four-leg plant validation requires a four-leg plant model; the
3D duty geometry remains covered by the reusable module host test.
