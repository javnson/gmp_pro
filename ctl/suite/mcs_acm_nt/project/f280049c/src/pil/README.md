# F280049C ACIM Processor-in-the-Loop

**English** | [简体中文](README_CN.md)

This folder contains the reproducible PIL workflow for `MCS_STD_ACM_MODEL.slx`. The target SDPE requirement is the single source for the PIL enable switch, BUILD_LEVEL, Data Link command allocation, UART rate, UDP endpoints, masks, and channel mappings.

## Safety boundary

`ENABLE_GMP_DL_PIL_SIM` is a target-local SDPE feature switch. When disabled, the physical ADC ISR, controller dispatch, and PWM path retain their normal behavior. When enabled, only a valid Data Link PIL STEP request can execute `ctl_dispatch()`. The physical ADC ISR continues the GMP system time base but cannot step the controller, and every physical PWM enable request is forced to the safe tripped/gate-disabled state.

Do not use a PIL firmware image for physical power conversion. Disable the SDPE PIL switch, regenerate, and rebuild before returning to a powered inverter.

## Run one stage

1. Select BUILD_LEVEL 1 through 4 and enable `ENABLE_GMP_DL_PIL_SIM` in `sdpe_mgr/sdpe_requirement.json`.
2. Generate the common SDPE output first, then generate this target output and synchronize `gmp_src_mgr`.
3. Build and flash the F280049C image.
4. Start the headless bridge from `tools/gmp_pil_server/gmp_debugger`:

   ```powershell
   python -m apis.examples.pil_bridge --sdpe ../../../ctl/suite/mcs_acm_nt/project/f280049c/src/sdpe_mgr/sdpe_requirement.json --port COM5 --trace ../../../ctl/suite/mcs_acm_nt/project/f280049c/src/pil/results/manual/build_level_1/bridge_trace.csv
   ```

5. Run MATLAB:

   ```matlab
   cd('ctl/suite/mcs_acm_nt/project/f280049c/src/pil');
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

On 2026-08-10, isolated PIL `BUILD_LEVEL=1` was compiled, downloaded, and run on
a LaunchXL-F280049C against the same ACIM model. The 0.005 s run completed exactly
100 controller steps at 20 kHz; its last frame was at 0.004975 s, mean target
round-trip time was about 7.25 ms, and the physical-output enable word remained
zero throughout. This run also verified that the UDP S-function advances the
target only at major solver steps and reuses the cached result at minor steps.
MATLAB output and the manifest are under `results/timing_fix_build_level_1`.
Levels 2 through 4 have not yet been validated on hardware and must be
commissioned in order.

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

This ACIM Level 1 run used 115200 baud and measured about 7.25 ms mean target
round-trip time. PIL wall-clock speed reflects XDS110 request/response latency,
not the target current-loop execution capacity.
