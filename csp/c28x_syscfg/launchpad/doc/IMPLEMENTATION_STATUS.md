# Implementation status

This file records verified state, not intended state.

| Work item | State | Evidence |
|---|---|---|
| Eight `C2000Lib_*` packages | complete | copied from local C2000Ware 5.04.00.00 |
| Official schematics in `hw/` | complete | one official PDF per board |
| Debug target files | complete | official C2000Ware CCXML per board |
| F280049C, F280039C and F280025C SysConfig | validated | SysConfig 1.21, zero errors/warnings |
| Remaining five SysConfig files | in progress | require per-board pin and switch audit |
| F280049C and F280039C enlarged flash linkers | compiled | large GMP image linked from local code-start sources |
| F280025C build support | prepared | local code-start source, enlarged Flash command file and EABI DriverLib archive |
| Remaining six flash linker layouts | in progress | must be checked against each device memory map |
| Shared SDPE requirement | generated | current `.h`/`.m` pair generated and validated from one JSON source |
| Shared task/Data Link application | implemented | one-shot, 500 ms heartbeat, 2 ms Data Link, tunable, memory, scope and optional PIL |
| ADC-to-PWM/DAC control example | implemented | SDPE-selected ADC mirrored to PWM; DAC used when available and enabled |
| Classic CAN reference service | implemented | 1 Mbit/s, RX 0x101, TX 0x201; disconnected-bus-safe polling |
| MCAN reference service | pending | required for F28P55X |
| F280049C/F280039C Debug/Release configurations | complete | CCS invokes each root `.syscfg` and builds from the formal root project |
| Remaining twelve CCS configurations | pending | created after every target file validates |
| All-configuration compile | pending | CCS headless build |
| F280049C/F280039C Debug/Release compile | passed | CCS 12.8 headless, C2000 compiler 22.6.1.LTS, zero errors |
| F280049C flash/runtime test | passed | XDS110 physical board, 30-second counter observation |
| F280049C Data Link test | passed | COM5, 115200 bit/s; echo/stress/tunable/memory/scope |
| F280049C physical CAN test | pending | requires a second active 1 Mbit/s CAN node |

Do not interpret an item marked `pending` or `in progress` as implemented.

## F280049C physical evidence (2026-08-15)

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
