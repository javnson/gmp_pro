# SDPE model-parameter audit

Suite: `pgs_inv_GFM_inverter`  
MATLAB load verification: `R2024b`  
Result: **NEEDS MIGRATION**

The audit verifies that each SLX has a path-independent callback to its freshly generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually references at least one variable exported by that generated script. Loading an initializer alone is not counted as parameter use.

| Model | Kind | Callback | SDPE variables used | Status |
| --- | --- | --- | ---: | --- |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_pmsm_grid.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_2022b.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_backup.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_remote.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_resload.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/DP_STD_MDL_DCAC_3ph_2level_resload_2022b.slx` | model | PASS | 0 | NEEDS MIGRATION |
| `project/simulate/gmp_fp_utilities_src_2022b.slx` | library | PASS | 0 | EXEMPT |

## Per-model evidence

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates (first 12 shown): `Amplitude=0`, `Capacitance=1e-6`, `Flux=0.060`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=10.0e-3`, `Inductance=1e-3`, `Inductance=2.5e-3`, `Inductance=695e-6`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_pmsm_grid.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates (first 12 shown): `Amplitude=0`, `Capacitance=1e-6`, `Flux=0.050`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=2.5e-3`, `Inductance=3.0e-3`, `Inductance=695e-6`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=3.0e-3`, `Inductance=695e-6`, `Resistance=0.20`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=100e-6`, `Inductance=1e-3`, `Inductance=695e-6`, `Resistance=0.1`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_2022b.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=100e-6`, `Inductance=1e-3`, `Inductance=695e-6`, `Resistance=0.1`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_backup.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=3e-3`, `Inductance=695e-6`, `Resistance=0.5`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_gridconn_remote.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=100*pi()`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=3e-3`, `Inductance=695e-6`, `Resistance=0.5`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_resload.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=695e-6`, `Resistance=50/3`, `Vf=0.3`.

### `project/simulate/DP_STD_MDL_DCAC_3ph_2level_resload_2022b.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=1/2*pi`, `Gain=1/CMP_MAX/2`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=695e-6`, `Resistance=50/3`, `Vf=0.3`.

### `project/simulate/gmp_fp_utilities_src_2022b.slx`

Block library: load/callback behavior was checked; plant-parameter binding is not applicable.

Numeric physical-parameter candidates: `Gain=-1`, `Gain=-1/2`, `Gain=0.5`, `Gain=1/2*pi`, `Gain=1/2^ADC_BIT`, `Gain=1/ADC_GAIN/ADC_REF`, `Gain=1/CMP_MAX/2`, `Gain=1/PerUnit_Base`, `Gain=2`, `Gain=4`.

## Interpretation

Numeric candidates are review evidence, not an automatic failure by themselves: some are solver settings, ideal-element values, or internal mask defaults whose effective value comes from a symbolic parent mask. A non-library model with zero generated-variable references is unambiguously not consuming SDPE parameters.

The R2024b load check opens every committed SLX with callbacks enabled. It does not claim that a SIL simulation, target build, or hardware test passed.
