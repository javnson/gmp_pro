# Implementation status

This file records verified state, not intended state.

| Work item | State | Evidence |
|---|---|---|
| Eight `C2000Lib_*` packages | complete | copied from local C2000Ware 5.04.00.00 |
| Official schematics in `hw/` | complete | one official PDF per board |
| Debug target files | complete | official C2000Ware CCXML per board |
| Eight SysConfig files | validated | all provide ADCA INT1, every usable BOOSTXL ADC, every device DAC and six BOOSTXL ePWM pairs |
| Eight enlarged flash linker layouts | compiled | portable `mass_data`, local code-start source and device-appropriate RAM mapping |
| CSP SDPE component library | validated | eight schemas/entities registered by the repository-wide SDPE settings |
| Shared and board SDPE requirements | generated | one common requirement plus eight `src/sdpe_mgr/requirements/<BOARD>` channel selections |
| Shared task/Data Link application | implemented | `user_main` owns scheduling; `user_dl` owns serial Data Link, tunable, memory, scope and optional PIL |
| ADC-to-PWM/DAC control example | implemented | coupled SDPE ADC SOC/result selection mirrored to selected PWM and capability-guarded DAC |
| Classic CAN reference service | implemented | independent 2 ms task, 1 Mbit/s, RX 0x101, TX 0x201; disconnected-bus-safe polling |
| MCAN reference service | pending | required for F28P55X |
| Sixteen CCS configurations | complete | every board has Debug and Release in the formal root project |
| All-configuration compile | passed | all eight Debug and eight Release configurations rebuilt after the 2026-08-16 SDPE/task refactor |
| F280049C flash/runtime test | historical pass | XDS110 physical board evidence predates the 2026-08-16 task/SDPE refactor |
| F280049C Data Link test | historical pass | COM5, 115200 bit/s; echo/stress/tunable/memory/scope before the refactor |
| F280049C physical CAN test | pending | requires a second active 1 Mbit/s CAN node |

Do not interpret an item marked `pending` or `in progress` as implemented.

## F280049C physical evidence (2026-08-15, pre-refactor)

- `startup_task_runs = 1`
- `heartbeat_task_runs = 60`
- `datalink_task_runs = 15000`
- `control_isr_runs = 600019`
- `DSPC2000_SystemTick = 30000`
- `launchpad_adc_raw = 139` at the final halt, with different values observed
  during earlier samples
- with ADC raw 139, the ADC-to-PWM example produced compare value 84 for a
  period of 2500 counts
- Data Link header CRC, payload CRC, FIFO overflow and timeout counters were all
  zero at the final halt
- Data Link smoke test validated a 128-byte memory region, three tunables and a
  400 x 2 float32 scope capture

The formal root-project Debug image was then flashed and probed for 5 seconds:

- `startup_task_runs = 1`
- `heartbeat_task_runs = 10`
- `datalink_task_runs = 2500`
- `control_isr_runs = 100016`
- `DSPC2000_SystemTick = 5000`
- `launchpad_adc_raw = 140`
- `launchpad_pwm_compare = 85`
- all four Data Link error counters remained zero
- the full serial smoke test passed again against that formal image
