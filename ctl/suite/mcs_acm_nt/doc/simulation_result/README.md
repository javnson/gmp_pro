# mcs_acm_nt simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 1 | PASS | PASS | FAIL | FAIL | FAIL | 51.035 |
| 2 | PASS | PASS | FAIL | PASS | FAIL | 53.822 |
| 3 | PASS | PASS | FAIL | PASS | FAIL | 50.648 |
| 4 | PASS | PASS | FAIL | FAIL | FAIL | 50.573 |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0 |
| `step_response.response_final` | 0.00354787 |
| `step_response.rise_time_10_90_s` | 0 |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 3496.3 |
| `steady_state_error_percent` | 1.59782e+15 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.1 |
| `step_response.response_final` | 0.0999757 |
| `step_response.rise_time_10_90_s` | 0 |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 465.525 |
| `steady_state_error_percent` | 0.0242753 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.1 |
| `step_response.response_final` | 0.100028 |
| `step_response.rise_time_10_90_s` | 0.0004 |
| `step_response.settling_time_s` | 0.599225 |
| `step_response.overshoot_percent` | 11.3201 |
| `steady_state_error_percent` | 0.0281413 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.206897 |
| `step_response.response_final` | 0.0153799 |
| `step_response.rise_time_10_90_s` | — |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |
| `steady_state_error_percent` | 92.5664 |

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [build_level_1_signals.csv](build_level_1_signals.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [build_level_2_signals.csv](build_level_2_signals.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [build_level_3_signals.csv](build_level_3_signals.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [build_level_4_signals.csv](build_level_4_signals.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_1.csv](commissioning/build_level_1.csv) | 21 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_2.csv](commissioning/build_level_2.csv) | 21 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensored.csv](commissioning/build_level_3_sensored.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless.csv](commissioning/build_level_3_sensorless.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_acquire.csv](commissioning/build_level_3_sensorless_acquire.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_acquire_comp5.csv](commissioning/build_level_3_sensorless_acquire_comp5.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_comp20.csv](commissioning/build_level_3_sensorless_comp20.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_diag.csv](commissioning/build_level_3_sensorless_diag.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_fallback.csv](commissioning/build_level_3_sensorless_fallback.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_gain_schedule.csv](commissioning/build_level_3_sensorless_gain_schedule.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_highspeed.csv](commissioning/build_level_3_sensorless_highspeed.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_highspeed_scale_half.csv](commissioning/build_level_3_sensorless_highspeed_scale_half.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_if.csv](commissioning/build_level_3_sensorless_if.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_leak5.csv](commissioning/build_level_3_sensorless_leak5.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_leaky_gain_schedule.csv](commissioning/build_level_3_sensorless_leaky_gain_schedule.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_seed_comp05.csv](commissioning/build_level_3_sensorless_seed_comp05.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_split_angle.csv](commissioning/build_level_3_sensorless_split_angle.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_split_angle_comp20.csv](commissioning/build_level_3_sensorless_split_angle_comp20.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/build_level_3_sensorless_vm.csv](commissioning/build_level_3_sensorless_vm.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/current_level3_sensored.csv](commissioning/current_level3_sensored.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/current_level3_sensorless.csv](commissioning/current_level3_sensorless.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/current_level3_sensorless_measured_voltage.csv](commissioning/current_level3_sensorless_measured_voltage.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/final_build_level_1.csv](commissioning/final_build_level_1.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/final_build_level_2.csv](commissioning/final_build_level_2.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/final_build_level_3_sensored.csv](commissioning/final_build_level_3_sensored.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/final_build_level_4_sensored.csv](commissioning/final_build_level_4_sensored.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/final_level1.csv](commissioning/final_level1.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [commissioning/udc_diag.csv](commissioning/udc_diag.csv) | 25 | 6 | `Variable`, `Signal`, `Samples`, `Final`, `Minimum`, `Maximum` |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 4 | 12 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

No additional JSON records are archived.

## Supporting artifacts

- [build_level_1_matlab.log](build_level_1_matlab.log)
- [build_level_2_matlab.log](build_level_2_matlab.log)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_4_matlab.log](build_level_4_matlab.log)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
