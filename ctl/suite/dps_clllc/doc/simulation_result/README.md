# dps_clllc simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 1 | PASS | PASS | PASS | PASS | FAIL | — |
| 2 | PASS | PASS | FAIL | FAIL | FAIL | — |
| 3 | PASS | PASS | FAIL | FAIL | FAIL | — |
| 4 | PASS | PASS | FAIL | FAIL | FAIL | — |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `—`, stop time `0.2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `final_primary_voltage_v` | 47.9824 |
| `final_primary_current_a` | -3.87761 |
| `final_secondary_voltage_v` | 31.0948 |
| `final_resonant_current_a` | -3.87761 |
| `final_modulation_command` | 0.4 |
| `steady_state_error_percent` | 0 |
| `step_response.settling_time_s` | 1e-05 |
| `step_response.overshoot_percent` | 0 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `—`, stop time `0.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `final_primary_voltage_v` | 47.9824 |
| `final_primary_current_a` | -2.66843 |
| `final_secondary_voltage_v` | 6.10122 |
| `final_resonant_current_a` | -2.66843 |
| `final_modulation_command` | 0.806481 |
| `steady_state_error_percent` | 153.369 |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 120 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `—`, stop time `0.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `final_primary_voltage_v` | 47.9824 |
| `final_primary_current_a` | 2.71319 |
| `final_secondary_voltage_v` | 0 |
| `final_resonant_current_a` | 2.71319 |
| `final_modulation_command` | 1 |
| `steady_state_error_percent` | 100 |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `—`, stop time `0.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `final_primary_voltage_v` | 47.9824 |
| `final_primary_current_a` | -2.74381 |
| `final_secondary_voltage_v` | 8.44206 |
| `final_resonant_current_a` | -2.74381 |
| `final_modulation_command` | 0.00397751 |
| `steady_state_error_percent` | 82.4124 |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 4 | 11 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

No additional JSON records are archived.

## Supporting artifacts

- [build_level_1_matlab.log](build_level_1_matlab.log)
- [build_level_1_results.mat](build_level_1_results.mat)
- [build_level_1_waveforms.png](build_level_1_waveforms.png)
- [build_level_2_matlab.log](build_level_2_matlab.log)
- [build_level_2_results.mat](build_level_2_results.mat)
- [build_level_2_waveforms.png](build_level_2_waveforms.png)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_3_results.mat](build_level_3_results.mat)
- [build_level_3_waveforms.png](build_level_3_waveforms.png)
- [build_level_4_matlab.log](build_level_4_matlab.log)
- [build_level_4_results.mat](build_level_4_results.mat)
- [build_level_4_waveforms.png](build_level_4_waveforms.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
