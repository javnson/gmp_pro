# dps_fsbb simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **PASS**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 3 | PASS | PASS | PASS | PASS | PASS | — |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `—`, stop time `0.8 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `vin_final_v` | 23.9771 |
| `vout_final_v` | 11.411 |
| `vout_peak_v` | 11.7949 |
| `il_final_a` | 0.591849 |
| `iout_final_a` | 0.569371 |
| `steady_state_error_percent` | 4.9357 |
| `step_response.settling_time_s` | 0.33965 |
| `step_response.overshoot_percent` | 0 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `—`, stop time `0.8 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `vin_final_v` | 23.9771 |
| `vout_final_v` | 46.6153 |
| `vout_peak_v` | 46.7925 |
| `il_final_a` | 4.97473 |
| `iout_final_a` | 2.32858 |
| `steady_state_error_percent` | 0.00307811 |
| `step_response.settling_time_s` | 0.3299 |
| `step_response.overshoot_percent` | 0.338576 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `—`, stop time `1.5 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `vin_final_v` | 23.9771 |
| `vout_final_v` | 24 |
| `vout_peak_v` | 24.0366 |
| `il_final_a` | 1.34028 |
| `iout_final_a` | 1.19775 |
| `steady_state_error_percent` | 0.000611536 |
| `step_response.settling_time_s` | 0.218848 |
| `step_response.overshoot_percent` | 0.474141 |

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
- [fsbb_internal_wiring.png](fsbb_internal_wiring.png)
- [top_level_wiring.png](top_level_wiring.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
