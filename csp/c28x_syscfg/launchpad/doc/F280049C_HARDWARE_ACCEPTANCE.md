# LAUNCHXL-F280049C hardware acceptance

Verified on 2026-08-15 with the on-board XDS110 and its application UART on
COM5.  The firmware was built with C2000 compiler 22.6.1.LTS, programmed to
flash through CCS 12.8 DSLite, and executed from flash.

## Runtime acceptance

A 30-second debugger observation produced:

| Observable | Result | Acceptance rule |
|---|---:|---|
| `startup_task_runs` | 1 | one-shot task runs once and disables itself |
| `heartbeat_task_runs` | 60 | 500 ms task period |
| `datalink_task_runs` | 15000 | 2 ms task period |
| `control_isr_runs` | 600019 | approximately 20 kHz ADC/control interrupt |
| `DSPC2000_SystemTick` | 30000 | one GMP tick per millisecond |
| `launchpad_adc_raw` | 139 at final halt | live, non-zero ADC conversion |

At the final halt, Data Link header CRC, payload CRC, FIFO overflow and timeout
error counters were all zero.

A separate five-second run checked the reference control path.  With
`launchpad_adc_raw = 139`, `launchpad_pwm_compare = 84` for the ePWM period of
2500 counts, which agrees with the expected normalized ADC-to-PWM mapping.

## Data Link acceptance

The complete u16 smoke test passed at 115200 bit/s:

- information/discovery request;
- payload echo and a 256-byte stress payload;
- discovery, read, write and restore of three physical tunables;
- discovery, read, write and restore of the 128-byte scratch memory window;
- configuration, arming, status polling and retrieval of the 400 x 2 float32
  sine/cosine scope.

The target uses both the SCIA RX FIFO interrupt and a lightweight foreground
FIFO pump.  Only ingress is pumped in the foreground; frame parsing and all
Data Link services remain in the required 2 ms scheduler task.

## CAN status

Classic CAN is configured for 1 Mbit/s.  The standalone-board test verifies
that a disconnected bus does not stop ADC, scheduling or Data Link operation.
A second active CAN node is still required to accept the 0x101 receive and
0x201 telemetry paths electrically.
