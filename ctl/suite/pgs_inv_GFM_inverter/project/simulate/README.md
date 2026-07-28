# GFM inverter automated SIL validation

The scripts follow the controller-process plus Simulink plant layout used by
`pgs_sinv_rc/project/simulate`.

- `run_gfm_cosim(level, stop_time)` runs one matching controller/plant pair.
- `run_gfm_validation(level, stop_time, label)` logs signals, evaluates the
  level-specific criteria, and writes JSON metrics plus PNG waveforms under
  `validation/`.
- `run_build_level_matrix.ps1` regenerates target SDPE, builds Debug x64, and
  validates BUILD_LEVEL 1 through 5. It restores and rebuilds the checked-in
  BUILD_LEVEL 5 selection even when a case fails.
- `run_gfm_technology_matrix.ps1` holds BUILD_LEVEL 5 and validates droop, VSM,
  and droop-plus-virtual-impedance against the same PMSG island/load-step case.
  It restores the common and target SDPE selections afterward.
- `prepare_gfm_pmsg_grid_model.m` reproducibly builds the BL5 plant from the
  checked-in grid-connected model and the MATLAB R2024b Specialized Power
  Systems permanent-magnet synchronous-machine block operated as a PMSG.

Run the matrix from PowerShell:

```powershell
.\run_build_level_matrix.ps1
.\run_gfm_technology_matrix.ps1
```

BL1-3 use the resistive-load plant, BL4 uses the ideal grid plant, and BL5 uses
the generated PMSG plant. In BL5 the PMSG:

- starts at 50 Hz with four pole pairs and 10 kg·m² inertia;
- receives 120 W mechanical input and carries a matched 120 W base load;
- connects through 10 mH per-phase weak-grid inductance at 0.05 s;
- lets the controller synchronize and finish its 0.10 s transition;
- separates at 0.50 s, leaving the converter in islanded GFM operation.

The converter has a permanent 30 W stabilizing load. A further 120 W load is
connected at 0.70 s. At the configured 0.5-pu voltage and 0.8-pu current-vector
limit, the calculated active-power ceiling is about 277 W, so the 150 W total
load uses about 54% of the available current-limited capacity. The validation
also records PMSG kinetic energy, inertia constant, worst-case RoCoF, voltage
tracking, q-axis voltage, command step, and actual/command current peaks.

`USING_3D_SVPWM` selects the fourth neutral-leg command and zero-sequence QPR
path. The supplied three-leg plants do not constitute a full four-leg power
stage validation.
