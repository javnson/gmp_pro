# mcs_pmsm_id simulation results

This directory is the repository archive for simulation evidence. Validation scripts write here directly; generated controller/SDPE files remain outside this archive.

## Validation matrix

Recorded overall result: **FAIL**.

| BUILD_LEVEL | Build | Simulation | Dynamic | Steady state | Overall | Duration (s) |
| ---: | --- | --- | --- | --- | --- | ---: |
| 2 | PASS | FAIL | FAIL | FAIL | FAIL | — |

## Key recorded metrics

No per-run metric JSON files are present.

## CSV data inventory

| File | Data rows | Columns | Leading fields |
| --- | ---: | ---: | --- |
| [fast_sil/pmsm_id_sil_fast_trace.csv](fast_sil/pmsm_id_sil_fast_trace.csv) | 14254 | 21 | `time_s`, `enable`, `oid_state`, `Rs_ohm`, `Ld_H`, `Lq_H`, … |
| [sil_validation_summary.csv](sil_validation_summary.csv) | 1 | 11 | `suite`, `build_level`, `build_pass`, `simulation_pass`, `runtime_assertion_triggered`, `dynamic_pass`, … |

## Other machine-readable records

### [fast_sil/pmsm_id_sil_fast_result.json](fast_sil/pmsm_id_sil_fast_result.json)

| Field | Value |
| --- | ---: |
| `passed` | PASS |
| `completed` | PASS |
| `final_state` | 8 |
| `completed_at_s` | 14.2535 |
| `simulated_until_s` | 14.2535 |
| `plant.Rs_ohm` | 4.7 |
| `plant.Ld_H` | 0.0085 |
| `plant.Lq_H` | 0.0085 |
| `plant.flux_Wb` | 0.0038197 |
| `plant.inertia_kg_m2` | 0.0005 |
| `plant.viscous_friction_Nm_s` | 0.0001 |
| `plant.load_torque_Nm` | 0.0002 |
| `encoder_truth.pole_pairs` | 4 |
| `encoder_truth.offset_pu` | 0.0999146 |
| `encoder_fault_injection` | none |
| `encoder_fault_code` | 0 |

## Supporting artifacts

- [build_level_2_matlab.log](build_level_2_matlab.log)

A recorded PASS applies only to the stored model, BUILD_LEVEL, parameters and toolchain represented by these files.
