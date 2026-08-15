# pgs_inv_GFL_inverter simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 1 | PASS | PASS | FAIL | PASS | FAIL | 65.902 |
| 2 | PASS | PASS | PASS | PASS | PASS | — |
| 3 | PASS | PASS | PASS | PASS | PASS | — |
| 4 | PASS | PASS | PASS | PASS | PASS | — |
| 5 | PASS | PASS | FAIL | PASS | FAIL | 70.146 |
| 6 | PASS | PASS | PASS | PASS | PASS | — |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.139709 |
| `zero_current_rms_pu` | 0.000866528 |
| `voltage_d_mean_pu` | 0.453083 |
| `voltage_q_rms_pu` | 0.370614 |
| `current_d_mean_pu` | 0.145619 |
| `current_q_mean_pu` | 0.133891 |
| `pll_error_rms_pu` | 0 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.099895 |
| `zero_current_rms_pu` | 0.000854754 |
| `voltage_d_mean_pu` | 0.381726 |
| `voltage_q_rms_pu` | 0.331778 |
| `current_d_mean_pu` | 0.0999813 |
| `current_q_mean_pu` | 0.0999494 |
| `pll_error_rms_pu` | 0 |
| `current_tracking_error_pu` | 5.38923e-05 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `DP_STD_MDL_DCAC_3ph_2level_gridconn`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0712424 |
| `zero_current_rms_pu` | 0.000867673 |
| `voltage_d_mean_pu` | 0.348643 |
| `voltage_q_rms_pu` | 0.000294793 |
| `current_d_mean_pu` | 0.100135 |
| `current_q_mean_pu` | 9.10718e-06 |
| `pll_error_rms_pu` | 0.000294793 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `DP_STD_MDL_DCAC_3ph_2level_gridconn`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.600016 |
| `zero_current_rms_pu` | 0.000864647 |
| `voltage_d_mean_pu` | 0.356241 |
| `voltage_q_rms_pu` | 0.00025451 |
| `current_d_mean_pu` | 0.600058 |
| `current_q_mean_pu` | 0.600004 |
| `pll_error_rms_pu` | 0.00025451 |

### [build_level_5_metrics.json](build_level_5_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gridconn`, stop time `1 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0661357 |
| `zero_current_rms_pu` | 0.000878475 |
| `voltage_d_mean_pu` | 0.347046 |
| `voltage_q_rms_pu` | 0.000265276 |
| `current_d_mean_pu` | 0.00693474 |
| `current_q_mean_pu` | -0.093138 |
| `pll_error_rms_pu` | 0.000265276 |

### [build_level_5_pq_droop_metrics.json](build_level_5_pq_droop_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gridconn`, stop time `1.2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0661533 |
| `zero_current_rms_pu` | 0.000875323 |
| `voltage_d_mean_pu` | 0.347043 |
| `voltage_q_rms_pu` | 0.000265397 |
| `current_d_mean_pu` | 0.00690367 |
| `current_q_mean_pu` | -0.0931654 |
| `pll_error_rms_pu` | 0.000265397 |

### [build_level_6_3d_metrics.json](build_level_6_3d_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.100372 |
| `zero_current_rms_pu` | 0.000860575 |
| `voltage_d_mean_pu` | 0.507896 |
| `voltage_q_rms_pu` | 0.00602398 |
| `current_d_mean_pu` | 0.14168 |
| `current_q_mean_pu` | 0.0103027 |
| `pll_error_rms_pu` | 0 |

### [build_level_6_final_metrics.json](build_level_6_final_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `0.8 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.101958 |
| `zero_current_rms_pu` | 0.000868217 |
| `voltage_d_mean_pu` | 0.514009 |
| `voltage_q_rms_pu` | 0.00685405 |
| `current_d_mean_pu` | 0.143763 |
| `current_q_mean_pu` | 0.00989193 |
| `pll_error_rms_pu` | 0 |

### [build_level_6_metrics.json](build_level_6_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.100544 |
| `zero_current_rms_pu` | 0.0008604 |
| `voltage_d_mean_pu` | 0.507975 |
| `voltage_q_rms_pu` | 0.00589286 |
| `current_d_mean_pu` | 0.141693 |
| `current_q_mean_pu` | 0.010311 |
| `pll_error_rms_pu` | 0 |

### [voltage_vector_limit_update_metrics.json](voltage_vector_limit_update_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.100544 |
| `zero_current_rms_pu` | 0.0008604 |
| `voltage_d_mean_pu` | 0.507975 |
| `voltage_q_rms_pu` | 0.00589286 |
| `current_d_mean_pu` | 0.141693 |
| `current_q_mean_pu` | 0.010311 |
| `pll_error_rms_pu` | 0 |

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 6 | 12 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

No additional JSON records are archived.

## Supporting artifacts

- [bl2_rerun.err.log](bl2_rerun.err.log)
- [bl2_rerun.log](bl2_rerun.log)
- [bl2_rerun2.err.log](bl2_rerun2.err.log)
- [bl2_rerun2.log](bl2_rerun2.log)
- [bl5_clean.err.log](bl5_clean.err.log)
- [bl5_clean.log](bl5_clean.log)
- [bl5_debug.err.log](bl5_debug.err.log)
- [bl5_debug.log](bl5_debug.log)
- [bl5_rerun.err.log](bl5_rerun.err.log)
- [bl5_rerun.log](bl5_rerun.log)
- [bl5_rerun2.err.log](bl5_rerun2.err.log)
- [bl5_rerun2.log](bl5_rerun2.log)
- [bl6_final.err.log](bl6_final.err.log)
- [bl6_final.log](bl6_final.log)
- [build_level_1_matlab.log](build_level_1_matlab.log)
- [build_level_1_waveforms.png](build_level_1_waveforms.png)
- [build_level_2_matlab.log](build_level_2_matlab.log)
- [build_level_2_waveforms.png](build_level_2_waveforms.png)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_3_waveforms.png](build_level_3_waveforms.png)
- [build_level_4_matlab.log](build_level_4_matlab.log)
- [build_level_4_waveforms.png](build_level_4_waveforms.png)
- [build_level_5_matlab.log](build_level_5_matlab.log)
- [build_level_5_pq_droop_waveforms.png](build_level_5_pq_droop_waveforms.png)
- [build_level_5_waveforms.png](build_level_5_waveforms.png)
- [build_level_6_3d_waveforms.png](build_level_6_3d_waveforms.png)
- [build_level_6_final_waveforms.png](build_level_6_final_waveforms.png)
- [build_level_6_matlab.log](build_level_6_matlab.log)
- [build_level_6_waveforms.png](build_level_6_waveforms.png)
- [matrix_final.err.log](matrix_final.err.log)
- [matrix_final.log](matrix_final.log)
- [matrix_run.err.log](matrix_run.err.log)
- [matrix_run.log](matrix_run.log)
- [voltage_vector_limit_update_waveforms.png](voltage_vector_limit_update_waveforms.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
