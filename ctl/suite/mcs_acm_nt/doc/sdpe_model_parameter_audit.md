# SDPE model-parameter audit

Suite: `mcs_acm_nt`  
MATLAB load verification: `R2024b`  
Result: **NEEDS MIGRATION**

The audit verifies that each SLX has a path-independent callback to its freshly generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually references at least one variable exported by that generated script. Loading an initializer alone is not counted as parameter use.

| Model | Kind | Callback | SDPE variables used | Status |
| --- | --- | --- | ---: | --- |
| `project/simulate/MCS_STD_ACM_MODEL.slx` | model | PASS | 0 | NEEDS MIGRATION |

## Per-model evidence

### `project/simulate/MCS_STD_ACM_MODEL.slx`

No generated SDPE variable is referenced by the stored model. The callback loads SDPE data, but the plant remains numerically configured and must be migrated.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Inductance=1e-3`, `PolePairs=2`, `Resistance=1e6`.

## Interpretation

Numeric candidates are review evidence, not an automatic failure by themselves: some are solver settings, ideal-element values, or internal mask defaults whose effective value comes from a symbolic parent mask. A non-library model with zero generated-variable references is unambiguously not consuming SDPE parameters.

The R2024b load check opens every committed SLX with callbacks enabled. It does not claim that a SIL simulation, target build, or hardware test passed.
