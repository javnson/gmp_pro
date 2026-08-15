# SDPE model-parameter audit

Suite: `dps_clllc`  
MATLAB load verification: `R2024b`  
Result: **NEEDS MIGRATION**

The audit verifies that each SLX has a path-independent callback to its freshly generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually references at least one variable exported by that generated script. Loading an initializer alone is not counted as parameter use.

| Model | Kind | Callback | SDPE variables used | Status |
| --- | --- | --- | ---: | --- |
| `project/simulate/GMP_STD_CLLLC_MODEL.slx` | model | PASS | 0 | NEEDS MIGRATION |

## Per-model evidence

### `project/simulate/GMP_STD_CLLLC_MODEL.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=48`, `Capacitance=120e-9`, `Capacitance=1e-6`, `Capacitance=440e-6`, `Gain=2^ADC_BIT`, `Inductance=1e-3`, `Inductance=20e-6`, `Resistance=0.05`, `Resistance=1`, `Resistance=10`, `Ron=0.01`.

## Interpretation

Numeric candidates are review evidence, not an automatic failure by themselves: some are solver settings, ideal-element values, or internal mask defaults whose effective value comes from a symbolic parent mask. A non-library model with zero generated-variable references is unambiguously not consuming SDPE parameters.

The R2024b load check opens every committed SLX with callbacks enabled. It does not claim that a SIL simulation, target build, or hardware test passed.
