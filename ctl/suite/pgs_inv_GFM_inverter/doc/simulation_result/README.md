# pgs_inv_GFM_inverter simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 1 | PASS | PASS | FAIL | PASS | FAIL | 89.733 |
| 2 | PASS | PASS | PASS | PASS | PASS | — |
| 3 | PASS | PASS | PASS | PASS | PASS | — |
| 4 | PASS | PASS | PASS | PASS | PASS | 93.434 |
| 5 | PASS | PASS | PASS | PASS | PASS | — |

## Key recorded metrics

### [build_level_1_metrics.json](build_level_1_metrics.json)

BUILD_LEVEL `1`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0801584 |
| `phase_current_peak_pu` | 0.117658 |
| `voltage_d_mean_pu` | 0.404426 |
| `voltage_q_rms_pu` | 0.0454534 |
| `current_d_mean_pu` | 0.112957 |
| `current_q_mean_pu` | -0.0046776 |
| `frequency_hz` | 49.8753 |

### [build_level_2_metrics.json](build_level_2_metrics.json)

BUILD_LEVEL `2`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0999461 |
| `phase_current_peak_pu` | 0.144525 |
| `voltage_d_mean_pu` | 0.3818 |
| `voltage_q_rms_pu` | 0.331902 |
| `current_d_mean_pu` | 0.100006 |
| `current_q_mean_pu` | 0.100002 |
| `frequency_hz` | 49.8753 |

### [build_level_3_metrics.json](build_level_3_metrics.json)

BUILD_LEVEL `3`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0988871 |
| `phase_current_peak_pu` | 0.142832 |
| `voltage_d_mean_pu` | 0.500023 |
| `voltage_q_rms_pu` | 0.00476787 |
| `current_d_mean_pu` | 0.139049 |
| `current_q_mean_pu` | 0.00975118 |
| `frequency_hz` | 49.8753 |
| `voltage_tracking_error_pu` | 2.3293e-05 |

### [build_level_4_metrics.json](build_level_4_metrics.json)

BUILD_LEVEL `4`, model `DP_STD_MDL_DCAC_3ph_2level_gridconn`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.0712385 |
| `phase_current_peak_pu` | 0.107025 |
| `voltage_d_mean_pu` | 0.348641 |
| `voltage_q_rms_pu` | 0.000293473 |
| `current_d_mean_pu` | 0.100148 |
| `current_q_mean_pu` | 7.23745e-06 |
| `frequency_hz` | 50 |

### [build_level_5_capacity_metrics.json](build_level_5_capacity_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.281829 |
| `phase_current_peak_pu` | 0.414755 |
| `voltage_d_mean_pu` | 0.460345 |
| `voltage_q_rms_pu` | 0.00689408 |
| `current_d_mean_pu` | 0.397997 |
| `current_q_mean_pu` | 0.0228189 |
| `frequency_hz` | 49.9087 |
| `voltage_tracking_error_pu` | 0.0402944 |

### [build_level_5_droop_metrics.json](build_level_5_droop_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.281628 |
| `phase_current_peak_pu` | 0.414699 |
| `voltage_d_mean_pu` | 0.45992 |
| `voltage_q_rms_pu` | 0.00687075 |
| `current_d_mean_pu` | 0.397632 |
| `current_q_mean_pu` | 0.0228136 |
| `frequency_hz` | 49.9089 |
| `voltage_tracking_error_pu` | 0.0400796 |

### [build_level_5_dsogi_probe_metrics.json](build_level_5_dsogi_probe_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid`, stop time `0.8 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 1.65702 |
| `voltage_d_mean_pu` | -0.241711 |
| `voltage_q_rms_pu` | 0.649124 |
| `current_d_mean_pu` | 0.00109112 |
| `current_q_mean_pu` | 0.81797 |
| `frequency_hz` | 0 |
| `voltage_tracking_error_pu` | 0.741711 |

### [build_level_5_initial_metrics.json](build_level_5_initial_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid`, stop time `1.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.00132907 |
| `voltage_d_mean_pu` | 0 |
| `voltage_q_rms_pu` | 0 |
| `current_d_mean_pu` | 0 |
| `current_q_mean_pu` | 0 |
| `frequency_hz` | 0 |
| `voltage_tracking_error_pu` | 0.5 |

### [build_level_5_metrics.json](build_level_5_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.281628 |
| `phase_current_peak_pu` | 0.414699 |
| `voltage_d_mean_pu` | 0.45992 |
| `voltage_q_rms_pu` | 0.00687075 |
| `current_d_mean_pu` | 0.397632 |
| `current_q_mean_pu` | 0.0228136 |
| `frequency_hz` | 49.9089 |
| `voltage_tracking_error_pu` | 0.0400796 |

### [build_level_5_sampling_fix_metrics.json](build_level_5_sampling_fix_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid`, stop time `0.8 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.594582 |
| `voltage_d_mean_pu` | 0.227252 |
| `voltage_q_rms_pu` | 0.281095 |
| `current_d_mean_pu` | 0.00470652 |
| `current_q_mean_pu` | 0.000223488 |
| `frequency_hz` | 0 |
| `voltage_tracking_error_pu` | 0.272748 |

### [build_level_5_short_metrics.json](build_level_5_short_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `0.5 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.042143 |
| `voltage_d_mean_pu` | 0.410633 |
| `voltage_q_rms_pu` | 0.00836154 |
| `current_d_mean_pu` | 0.0588803 |
| `current_q_mean_pu` | 0.0015223 |
| `frequency_hz` | 49.9888 |
| `voltage_tracking_error_pu` | 0.0893671 |

### [build_level_5_srf_fix_metrics.json](build_level_5_srf_fix_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid`, stop time `0.8 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.593919 |
| `voltage_d_mean_pu` | 0.224539 |
| `voltage_q_rms_pu` | 0.290353 |
| `current_d_mean_pu` | 0.0306729 |
| `current_q_mean_pu` | 0.0614758 |
| `frequency_hz` | 0 |
| `voltage_tracking_error_pu` | 0.275461 |

### [build_level_5_tuned_grid_metrics.json](build_level_5_tuned_grid_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_weakgrid`, stop time `1.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.00132907 |
| `voltage_d_mean_pu` | 0 |
| `voltage_q_rms_pu` | 0 |
| `current_d_mean_pu` | 0 |
| `current_q_mean_pu` | 0 |
| `frequency_hz` | 0 |
| `voltage_tracking_error_pu` | 0.5 |

### [build_level_5_virtual_impedance_metrics.json](build_level_5_virtual_impedance_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.281628 |
| `phase_current_peak_pu` | 0.414699 |
| `voltage_d_mean_pu` | 0.45992 |
| `voltage_q_rms_pu` | 0.00687075 |
| `current_d_mean_pu` | 0.397632 |
| `current_q_mean_pu` | 0.0228136 |
| `frequency_hz` | 49.9089 |
| `voltage_tracking_error_pu` | 0.0400796 |

### [build_level_5_vsm_metrics.json](build_level_5_vsm_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `2 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.281521 |
| `phase_current_peak_pu` | 0.414755 |
| `voltage_d_mean_pu` | 0.459956 |
| `voltage_q_rms_pu` | 0.00683134 |
| `current_d_mean_pu` | 0.397655 |
| `current_q_mean_pu` | 0.0228602 |
| `frequency_hz` | 49.9302 |
| `voltage_tracking_error_pu` | 0.0400442 |

### [build_level_5_weak_pmsg_metrics.json](build_level_5_weak_pmsg_metrics.json)

BUILD_LEVEL `5`, model `DP_STD_MDL_DCAC_3ph_2level_gfm_pmsg_grid`, stop time `1.2 s`, recorded status **FAIL**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.121826 |
| `voltage_d_mean_pu` | 0.385002 |
| `voltage_q_rms_pu` | 0.0470098 |
| `current_d_mean_pu` | 0.170639 |
| `current_q_mean_pu` | 0.0168214 |
| `frequency_hz` | 49.968 |
| `voltage_tracking_error_pu` | 0.115683 |

### [build_level_6_3d_metrics.json](build_level_6_3d_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.100372 |
| `voltage_d_mean_pu` | 0.507896 |
| `voltage_q_rms_pu` | 0.00602398 |
| `current_d_mean_pu` | 0.14168 |
| `current_q_mean_pu` | 0.0103027 |
| `voltage_tracking_error_pu` | 0.00789645 |

### [build_level_6_final_metrics.json](build_level_6_final_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `0.8 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.101958 |
| `voltage_d_mean_pu` | 0.514009 |
| `voltage_q_rms_pu` | 0.00685405 |
| `current_d_mean_pu` | 0.143763 |
| `current_q_mean_pu` | 0.00989193 |
| `voltage_tracking_error_pu` | 0.0140086 |

### [build_level_6_metrics.json](build_level_6_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `0.8 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.101958 |
| `voltage_d_mean_pu` | 0.514009 |
| `voltage_q_rms_pu` | 0.00685405 |
| `current_d_mean_pu` | 0.143763 |
| `current_q_mean_pu` | 0.00989193 |
| `voltage_tracking_error_pu` | 0.0140086 |

### [voltage_vector_limit_update_metrics.json](voltage_vector_limit_update_metrics.json)

BUILD_LEVEL `6`, model `DP_STD_MDL_DCAC_3ph_2level_resload`, stop time `1 s`, recorded status **PASS**.

| Metric | Value |
| --- | ---: |
| `phase_current_rms_pu` | 0.100544 |
| `voltage_d_mean_pu` | 0.507975 |
| `voltage_q_rms_pu` | 0.00589286 |
| `current_d_mean_pu` | 0.141693 |
| `current_q_mean_pu` | 0.010311 |
| `voltage_tracking_error_pu` | 0.00797456 |

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
- [build_level_2_waveforms.png](build_level_2_waveforms.png)
- [build_level_3_matlab.log](build_level_3_matlab.log)
- [build_level_3_waveforms.png](build_level_3_waveforms.png)
- [build_level_4_matlab.log](build_level_4_matlab.log)
- [build_level_4_waveforms.png](build_level_4_waveforms.png)
- [build_level_5_capacity_waveforms.png](build_level_5_capacity_waveforms.png)
- [build_level_5_droop_waveforms.png](build_level_5_droop_waveforms.png)
- [build_level_5_dsogi_probe_waveforms.png](build_level_5_dsogi_probe_waveforms.png)
- [build_level_5_initial_waveforms.png](build_level_5_initial_waveforms.png)
- [build_level_5_matlab.log](build_level_5_matlab.log)
- [build_level_5_sampling_fix_waveforms.png](build_level_5_sampling_fix_waveforms.png)
- [build_level_5_short_waveforms.png](build_level_5_short_waveforms.png)
- [build_level_5_srf_fix_waveforms.png](build_level_5_srf_fix_waveforms.png)
- [build_level_5_tuned_grid_waveforms.png](build_level_5_tuned_grid_waveforms.png)
- [build_level_5_virtual_impedance_waveforms.png](build_level_5_virtual_impedance_waveforms.png)
- [build_level_5_vsm_waveforms.png](build_level_5_vsm_waveforms.png)
- [build_level_5_waveforms.png](build_level_5_waveforms.png)
- [build_level_5_weak_pmsg_waveforms.png](build_level_5_weak_pmsg_waveforms.png)
- [build_level_6_3d_waveforms.png](build_level_6_3d_waveforms.png)
- [build_level_6_final_waveforms.png](build_level_6_final_waveforms.png)
- [build_level_6_waveforms.png](build_level_6_waveforms.png)
- [voltage_vector_limit_update_waveforms.png](voltage_vector_limit_update_waveforms.png)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
