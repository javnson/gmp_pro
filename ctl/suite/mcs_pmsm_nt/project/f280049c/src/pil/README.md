# F280049C PMSM Processor-in-the-Loop

**English** | [简体中文](README_CN.md)

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
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/src/sdpe_mgr/sdpe_requirement.json --port COM5 --trace ../../../ctl/suite/mcs_pmsm_nt/project/f280049c/src/pil/results/manual/build_level_1/bridge_trace.csv
   ```

5. Run MATLAB:

   ```matlab
   cd('ctl/suite/mcs_pmsm_nt/project/f280049c/src/pil');
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

## XDS110 latency

PIL is a causal request/response loop, so its wall-clock speed is limited by
round-trip latency rather than by the 20 kHz execution capability of the control
algorithm. TI reports that XDS110 virtual UART transfers above approximately
230400 baud can aggregate small packets for 14–16 ms. Two such USB directions
explain the approximately 30 ms target round trip observed at 256000 baud.

The F280049C SDPE configuration therefore uses the standard 115200 baud rate,
well below that XDS110 threshold. In PIL builds the Data Link service also runs directly
from the background loop instead of waiting for its normal 2 ms scheduled task;
the scheduled physical-control path is unchanged when PIL is disabled. For
sub-millisecond round trips, use a low-latency USB-UART adapter with a tunable
latency timer or a native USB/bulk transport instead of the XDS110 VCOM path.

The connected-board BUILD_LEVEL 1 validation at 115200 baud completed 100
Simulink-coupled controller steps in 0.8454 s (118.3 steps/s). The target RTT
was 7.24 ms on average, compared with approximately 30 ms at 256000 baud. The
recorded trace, MATLAB output, and timing summary are in
[`results/20260809_latency_115200`](results/20260809_latency_115200/README.md).
