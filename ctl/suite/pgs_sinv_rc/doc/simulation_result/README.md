# pgs_sinv_rc simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 1 | PASS | PASS | FAIL | FAIL | FAIL | 83.229 |
| 2 | PASS | PASS | FAIL | PASS | FAIL | — |
| 3 | PASS | PASS | FAIL | PASS | FAIL | 87.507 |
| 4 | PASS | PASS | PASS | PASS | PASS | 85.738 |
| 5 | PASS | PASS | FAIL | FAIL | FAIL | 85.705 |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `PGS_STD_SINV_MODEL_RLOAD`, stop time `2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 13.6336 |
| `iac_rms_a` | 1.18066 |
| `vbus_mean_v` | 59.7127 |
| `iref_rms_a` | 0 |
| `current_error_rms_pu` | 0 |
| `active_power_pu` | 0.0330555 |
| `reactive_power_pu` | -0.00409525 |
| `pll_frequency_hz` | 49.8762 |
| `iac_thd_percent` | 2.50815 |
| `fdrc_output_rms_pu` | 0 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `PGS_STD_SINV_MODEL_RLOAD`, stop time `2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 23.2387 |
| `iac_rms_a` | 1.99966 |
| `vbus_mean_v` | 59.1975 |
| `iref_rms_a` | 1.99829 |
| `current_error_rms_pu` | 0.00729723 |
| `active_power_pu` | 0.0958713 |
| `reactive_power_pu` | -0.0118759 |
| `pll_frequency_hz` | 49.8732 |
| `iac_thd_percent` | 1.20417 |
| `fdrc_output_rms_pu` | 0.00909414 |

### [build_level_2_rc_off_final_metrics.json](build_level_2_rc_off_final_metrics.json)

BUILD_LEVEL `2`, model `PGS_STD_SINV_MODEL_RLOAD`, stop time `3 s`, recorded status **—**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 23.2403 |
| `iac_rms_a` | 2.00037 |
| `vbus_mean_v` | 59.1971 |
| `iref_rms_a` | 1.99861 |
| `current_error_rms_pu` | 0.00791914 |
| `active_power_pu` | 0.0958685 |
| `reactive_power_pu` | -0.011871 |
| `pll_frequency_hz` | 49.8749 |
| `iac_thd_percent` | 0.886772 |
| `fdrc_output_rms_pu` | 0 |

### [build_level_2_rc_on_final_metrics.json](build_level_2_rc_on_final_metrics.json)

BUILD_LEVEL `2`, model `PGS_STD_SINV_MODEL_RLOAD`, stop time `3 s`, recorded status **—**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 23.2382 |
| `iac_rms_a` | 2.0002 |
| `vbus_mean_v` | 59.1972 |
| `iref_rms_a` | 1.99846 |
| `current_error_rms_pu` | 0.00789492 |
| `active_power_pu` | 0.0958721 |
| `reactive_power_pu` | -0.0118842 |
| `pll_frequency_hz` | 49.8752 |
| `iac_thd_percent` | 0.876574 |
| `fdrc_output_rms_pu` | 0.00475438 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `PGS_STD_SINV_MODEL_Grid`, stop time `2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 24.1325 |
| `iac_rms_a` | 2.01385 |
| `vbus_mean_v` | 59.1666 |
| `iref_rms_a` | 2.00781 |
| `current_error_rms_pu` | 0.0122092 |
| `active_power_pu` | 0.0999321 |
| `reactive_power_pu` | 0.000164139 |
| `pll_frequency_hz` | 50 |
| `iac_thd_percent` | 7.43246 |
| `fdrc_output_rms_pu` | 0.00423448 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `PGS_STD_SINV_MODEL_Grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 24.141 |
| `iac_rms_a` | 3.00649 |
| `vbus_mean_v` | 58.7327 |
| `iref_rms_a` | 3.00319 |
| `current_error_rms_pu` | 0.0117045 |
| `active_power_pu` | 0.149667 |
| `reactive_power_pu` | 0.000180934 |
| `pll_frequency_hz` | 50 |
| `iac_thd_percent` | 4.59926 |
| `fdrc_output_rms_pu` | 0.004503 |

### [build_level_5_metrics.json](build_level_5_metrics.json)

BUILD_LEVEL `5`, model `PGS_STD_SINV_MODEL_Rectifier`, stop time `2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `vac_rms_v` | 23.997 |
| `iac_rms_a` | 5.16872 |
| `vbus_mean_v` | 60.003 |
| `iref_rms_a` | 5.17086 |
| `current_error_rms_pu` | 0.0129708 |
| `active_power_pu` | -0.255504 |
| `reactive_power_pu` | 0.000377619 |
| `pll_frequency_hz` | 50.0001 |
| `iac_thd_percent` | 3.71898 |
| `fdrc_output_rms_pu` | 0.00475905 |

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 5 | 12 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

No additional JSON records are archived.

## Supporting artifacts

- [build_level_1_matlab.log](build_level_1_matlab.log)
- [build_level_1_waveforms.png](build_level_1_waveforms.png)
- [build_level_2_matlab.log](build_level_2_matlab.log)
- [build_level_2_rc_off_final_waveforms.png](build_level_2_rc_off_final_waveforms.png)
- [build_level_2_rc_on_final_waveforms.png](build_level_2_rc_on_final_waveforms.png)
- [build_level_2_waveforms.png](build_level_2_waveforms.png)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_3_waveforms.png](build_level_3_waveforms.png)
- [build_level_4_matlab.log](build_level_4_matlab.log)
- [build_level_4_waveforms.png](build_level_4_waveforms.png)
- [build_level_5_matlab.log](build_level_5_matlab.log)
- [build_level_5_waveforms.png](build_level_5_waveforms.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
