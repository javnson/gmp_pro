# SDPE model-parameter audit

Suite: `dps_fsbb`  
MATLAB load verification: `R2024b`  
Result: **PASS**

The audit verifies that each SLX has a path-independent callback to its freshly generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually references at least one variable exported by that generated script. Loading an initializer alone is not counted as parameter use.

| Model | Kind | Callback | SDPE variables used | Status |
| --- | --- | --- | ---: | --- |
| `project/simulate/MCS_STD_FSBB_MODEL.slx` | model | PASS | 12 | PASS |
| `project/simulate/MCS_STD_FSBB_MODEL_2022b.slx` | model | PASS | 12 | PASS |

## Per-model evidence

### `project/simulate/MCS_STD_FSBB_MODEL.slx`

Referenced generated SDPE variables: `CTRL_ADC_RESOLUTION`, `CTRL_FSBB_IIN_BIAS`, `CTRL_FSBB_IIN_SENSITIVITY`, `CTRL_PWM_CMP_MAX`, `CTRL_PWM_DEADBAND_CMP`, `FSBB_MODEL_DIODE_RON`, `FSBB_MODEL_DIODE_VF`, `FSBB_MODEL_MOSFET_RON`, `FSBB_MODEL_SNUBBER_C`, `FSBB_MODEL_SNUBBER_R`, `FSBB_MODEL_SWITCH_LON`, `FSBB_PARAM_CIN_ESR`.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Inductance=10E-6`, `Inductance=1e-3`.

### `project/simulate/MCS_STD_FSBB_MODEL_2022b.slx`

Referenced generated SDPE variables: `CTRL_ADC_RESOLUTION`, `CTRL_FSBB_IIN_BIAS`, `CTRL_FSBB_IIN_SENSITIVITY`, `CTRL_PWM_CMP_MAX`, `CTRL_PWM_DEADBAND_CMP`, `FSBB_MODEL_DIODE_RON`, `FSBB_MODEL_DIODE_VF`, `FSBB_MODEL_MOSFET_RON`, `FSBB_MODEL_SNUBBER_C`, `FSBB_MODEL_SNUBBER_R`, `FSBB_MODEL_SWITCH_LON`, `FSBB_PARAM_CIN_ESR`.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Inductance=10E-6`, `Inductance=1e-3`.

## Interpretation

Numeric candidates are review evidence, not an automatic failure by themselves: some are solver settings, ideal-element values, or internal mask defaults whose effective value comes from a symbolic parent mask. A non-library model with zero generated-variable references is unambiguously not consuming SDPE parameters.

The R2024b load check opens every committed SLX with callbacks enabled. It does not claim that a SIL simulation, target build, or hardware test passed.
