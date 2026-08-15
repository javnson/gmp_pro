# SDPE model-parameter audit

Suite: `pgs_sinv_2stage`  
MATLAB load verification: `R2024b`  
Result: **PASS**

The audit verifies that each SLX has a path-independent callback to its freshly generated `sdpe_mgr/ctrl_settings_matlab_init.m`, and that the model XML actually references at least one variable exported by that generated script. Loading an initializer alone is not counted as parameter use.

| Model | Kind | Callback | SDPE variables used | Status |
| --- | --- | --- | ---: | --- |
| `project/simulate/PGS_STD_SINV_2STAGE_MODEL_Grid.slx` | model | PASS | 27 | PASS |
| `project/simulate/PGS_STD_SINV_2STAGE_MODEL_Rectifier.slx` | model | PASS | 27 | PASS |
| `project/simulate/PGS_STD_SINV_2STAGE_MODEL_RLOAD.slx` | model | PASS | 27 | PASS |

## Per-model evidence

### `project/simulate/PGS_STD_SINV_2STAGE_MODEL_Grid.slx`

Referenced generated SDPE variables: `CTRL_AC_CURRENT_BIAS`, `CTRL_AC_CURRENT_SENSITIVITY`, `CTRL_AC_INDUCTANCE`, `CTRL_AC_RESISTANCE`, `CTRL_AC_VOLTAGE_BIAS`, `CTRL_AC_VOLTAGE_SENSITIVITY`, `CTRL_ADC_RESOLUTION`, `CTRL_ADC_VOLTAGE_REF`, `CTRL_DCBUS_VOLTAGE`, `CTRL_DC_VOLTAGE_BIAS`, `CTRL_DC_VOLTAGE_SENSITIVITY`, `CTRL_GRID_VOLTAGE_RMS`, `CTRL_PWM_CMP_MAX`, `CTRL_PWM_DEADBAND_CMP`, `SINV_CURRENT_AMPLIFIER_GAIN`, `SINV_CURRENT_SHUNT_OHM`, `SINV_DC_CAPACITANCE_F`, `SINV_DC_CAP_EPR_OHM`, `SINV_DC_CAP_ESR_OHM`, `SINV_FILTER_CAPACITANCE_F`, `SINV_FILTER_CAP_EPR_OHM`, `SINV_FILTER_CAP_ESR_OHM`, `SINV_MODEL_DIODE_RON`, `SINV_MODEL_DIODE_VF`, `SINV_MODEL_MOSFET_RON`, `SINV_PWM_FREQUENCY_HZ`, `SINV_SENSOR_FILTER_HZ`.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Frequency=2*pi*CTRL_GRID_FREQUENCY`, `Gain=2^ADC_BIT`, `Inductance=10e-6`, `Inductance=1e-3`, `Resistance=1`.

### `project/simulate/PGS_STD_SINV_2STAGE_MODEL_Rectifier.slx`

Referenced generated SDPE variables: `CTRL_AC_CURRENT_BIAS`, `CTRL_AC_CURRENT_SENSITIVITY`, `CTRL_AC_INDUCTANCE`, `CTRL_AC_RESISTANCE`, `CTRL_AC_VOLTAGE_BIAS`, `CTRL_AC_VOLTAGE_SENSITIVITY`, `CTRL_ADC_RESOLUTION`, `CTRL_ADC_VOLTAGE_REF`, `CTRL_DC_VOLTAGE_BIAS`, `CTRL_DC_VOLTAGE_SENSITIVITY`, `CTRL_GRID_VOLTAGE_RMS`, `CTRL_PWM_CMP_MAX`, `CTRL_PWM_DEADBAND_CMP`, `SINV_CURRENT_AMPLIFIER_GAIN`, `SINV_CURRENT_SHUNT_OHM`, `SINV_DC_CAPACITANCE_F`, `SINV_DC_CAP_EPR_OHM`, `SINV_DC_CAP_ESR_OHM`, `SINV_FILTER_CAPACITANCE_F`, `SINV_FILTER_CAP_EPR_OHM`, `SINV_FILTER_CAP_ESR_OHM`, `SINV_MODEL_DIODE_RON`, `SINV_MODEL_DIODE_VF`, `SINV_MODEL_MOSFET_RON`, `SINV_PWM_FREQUENCY_HZ`, `SINV_RECTIFIER_RLOAD_OHM`, `SINV_SENSOR_FILTER_HZ`.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Frequency=2*pi*CTRL_GRID_FREQUENCY`, `Gain=2^ADC_BIT`, `Inductance=10e-6`, `Inductance=1e-3`.

### `project/simulate/PGS_STD_SINV_2STAGE_MODEL_RLOAD.slx`

Referenced generated SDPE variables: `CTRL_AC_CURRENT_BIAS`, `CTRL_AC_CURRENT_SENSITIVITY`, `CTRL_AC_INDUCTANCE`, `CTRL_AC_RESISTANCE`, `CTRL_AC_VOLTAGE_BIAS`, `CTRL_AC_VOLTAGE_SENSITIVITY`, `CTRL_ADC_RESOLUTION`, `CTRL_ADC_VOLTAGE_REF`, `CTRL_DCBUS_VOLTAGE`, `CTRL_DC_VOLTAGE_BIAS`, `CTRL_DC_VOLTAGE_SENSITIVITY`, `CTRL_PWM_CMP_MAX`, `CTRL_PWM_DEADBAND_CMP`, `SINV_CURRENT_AMPLIFIER_GAIN`, `SINV_CURRENT_SHUNT_OHM`, `SINV_DC_CAPACITANCE_F`, `SINV_DC_CAP_EPR_OHM`, `SINV_DC_CAP_ESR_OHM`, `SINV_FILTER_CAPACITANCE_F`, `SINV_FILTER_CAP_EPR_OHM`, `SINV_FILTER_CAP_ESR_OHM`, `SINV_MODEL_DIODE_RON`, `SINV_MODEL_DIODE_VF`, `SINV_MODEL_MOSFET_RON`, `SINV_PWM_FREQUENCY_HZ`, `SINV_RLOAD_OHM`, `SINV_SENSOR_FILTER_HZ`.

Numeric physical-parameter candidates: `Amplitude=0`, `Capacitance=1e-6`, `Frequency=0`, `Gain=2^ADC_BIT`, `Inductance=10e-6`, `Inductance=1e-3`, `Resistance=1`.

## Interpretation

Numeric candidates are review evidence, not an automatic failure by themselves: some are solver settings, ideal-element values, or internal mask defaults whose effective value comes from a symbolic parent mask. A non-library model with zero generated-variable references is unambiguously not consuming SDPE parameters.

The R2024b load check opens every committed SLX with callbacks enabled. It does not claim that a SIL simulation, target build, or hardware test passed.
