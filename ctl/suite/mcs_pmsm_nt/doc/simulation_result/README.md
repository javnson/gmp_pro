# mcs_pmsm_nt simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 2 | PASS | PASS | FAIL | FAIL | FAIL | — |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.0546477 |
| `step_response.response_final` | 0 |
| `step_response.rise_time_10_90_s` | — |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |
| `steady_state_error_percent` | 100 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `—`, stop time `1.5 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.1 |
| `step_response.response_final` | -0.0231752 |
| `step_response.rise_time_10_90_s` | — |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |
| `steady_state_error_percent` | 123.175 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.1 |
| `step_response.response_final` | 0 |
| `step_response.rise_time_10_90_s` | — |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |
| `steady_state_error_percent` | 100 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `—`, stop time `0.6 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `step_response.reference_final` | 0.1 |
| `step_response.response_final` | 0 |
| `step_response.rise_time_10_90_s` | — |
| `step_response.settling_time_s` | — |
| `step_response.overshoot_percent` | 0 |
| `steady_state_error_percent` | 100 |

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 1 | 11 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

No additional JSON records are archived.

## Supporting artifacts

- [build_level_1_matlab.log](build_level_1_matlab.log)
- [build_level_1_waveforms.png](build_level_1_waveforms.png)
- [build_level_2_matlab.log](build_level_2_matlab.log)
- [build_level_2_waveforms.png](build_level_2_waveforms.png)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_3_waveforms.png](build_level_3_waveforms.png)
- [build_level_4_matlab.log](build_level_4_matlab.log)
- [build_level_4_waveforms.png](build_level_4_waveforms.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
