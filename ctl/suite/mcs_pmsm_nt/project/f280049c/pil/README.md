# F280049C PMSM Processor-in-the-Loop

This folder contains the reproducible PIL workflow for `MCS_STD_PMSM_MODEL.slx`. The target SDPE requirement is the single source for the PIL enable switch, BUILD_LEVEL, Data Link command allocation, UART rate, UDP endpoints, masks, and channel mappings.

## Safety boundary

`ENABLE_GMP_DL_PIL_SIM` is a target-local SDPE feature switch. When disabled, the physical ADC ISR, controller dispatch, and PWM path retain their normal behavior. When enabled, only a valid Data Link PIL STEP request can execute `ctl_dispatch()`. The physical ADC ISR continues the GMP system time base but cannot step the controller, and every physical PWM enable request is forced to the safe tripped/gate-disabled state.

Do not use a PIL firmware image for physical power conversion. Disable the SDPE PIL switch, regenerate, and rebuild before returning to a powered inverter.

## Run one stage

1. Select BUILD_LEVEL 1 through 4 and enable `ENABLE_GMP_DL_PIL_SIM` in `sdpe_mgr/sdpe_requirement.json`.
2. Generate the common SDPE output first, then generate this target output and synchronize `gmp_src_mgr`.
3. Build and flash the F280049C image.
4. Start the headless bridge from `tools/gmp_pil_server/gmp_debugger`:

   ```powershell
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/sdpe_mgr/sdpe_requirement.json --port COM5 --trace ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/pil/results/manual/build_level_1/bridge_trace.csv
   ```

5. Run MATLAB:

   ```matlab
   cd('ctl/suite/mcs_pmsm_nt/project/f280049c/pil');
   run_pil_stage(1, 0.05, "results/manual/build_level_1");
   ```

The bridge records every selected ADC channel, every PWM/monitor result, and serial round-trip time in CSV. MATLAB saves the complete `SimulationOutput` and a JSON manifest. A STEP timeout terminates the bridge; it is deliberately not retried because an ambiguous retry could execute the controller twice.

## Standard data contract

- Simulink to bridge: `<d24I16d8i>`, 264 bytes.
- Bridge to Simulink: `<d8I8I16d>`, 200 bytes.
- Target RX: fixed tick and digital words followed by the ADC/panel fields selected by `GMP_PIL_RX_MASK`.
- Target TX: fixed digital word followed by the PWM/DAC/monitor fields selected by `GMP_PIL_TX_MASK`.

`run_pil_stage` validates these Simulink vector sizes before simulation and applies all UDP endpoints from SDPE without modifying the source model.

## Recorded hardware validation

All four commissioning levels were run on a LaunchXL-F280049C on 2026-08-09.
The saved traces, MATLAB results, manifests, logs, and metric summary are in
[`results/20260809_hardware_pil`](results/20260809_hardware_pil/README.md).
Both PIL-enabled and PIL-disabled target builds were compiled; the final
connected-board image is the isolated PIL `BUILD_LEVEL=1` configuration.
