# PMSM vector control and offline parameter identification

**English** | [简体中文](README_CN.md)

This suite extends the GMP PMSM vector-control template with an offline identification engine. It can characterize the motor and prepare current-loop tuning while retaining the multi-platform, incremental `BUILD_LEVEL` workflow used by the standard motor-control suite.

## Identification scope

- Stator resistance and inductance.
- Permanent-magnet flux linkage.
- Inverter dead-time compensation.
- Mechanical inertia and friction.
- Initial current-loop PI tuning.

The shared implementation is centered on `src/`, including the `pmsm_offline_id_if` interface and the normal `ctl_main` control entry. Project targets include C2000, STM32, and PC simulation variants.

Parameter identification can deliberately energize or rotate the motor. Verify current scaling, phase order, protection limits, rotor freedom, and emergency-stop behavior at a low build level before starting an identification sequence. The [Chinese guide](README_CN.md) documents the state flow, parameters, and detailed operating procedure.
