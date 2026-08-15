# CTL intrinsic components

**English** | [简体中文](readme_cn.md)

This directory contains the repository's general-purpose control building
blocks. The current source tree is the inventory authority:

| Group | Implemented headers |
| --- | --- |
| `basic` | `divider.h`, `hysteresis_controller.h`, `saturation.h`, `slope_limiter.h`, `state_sequencer.h` |
| `continuous` | `continuous_pi.h`, `continuous_pid.h`, `continuous_pid_aw.h`, `ladrc1.h`, `ladrc2.h`, `sogi.h`, `track_pid.h` |
| `discrete` | biquad/direct-form/general/FIR filters, PID, SOGI, lead-lag, pole-zero, PR plus tuner, signal generator, and tracking PID |
| `complex` | `dq_pi.h`, `dq_ladrc1.h` |
| `advance` | backstepping, FDRC, fuzzy logic/PID, ILC, IMC, LMS, MRAC, paired LUT, repetitive control, sinc interpolation, SMC, and surface search |
| `protection` | generic protection/slots, ITOC, PT100x, and sag/swell detection |

Include the concrete header and its registered sources. The current
`ctl/component/intrinsic.h` umbrella still references the removed
`continuous/s_function.h` and `discrete/z_function.h`, so it is not a valid
compile contract.

Function names and units differ by module; read the selected header rather
than inferring a universal PID-style API. Registry validation flags in
`gmp_framework_dic.json` are the authoritative record of compile, simulation,
and hardware evidence. Host tests currently exist for the `complex` d/q
controllers; that does not validate every intrinsic module.
