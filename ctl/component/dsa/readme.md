# Dynamic signal analysis components

**English** | [简体中文](readme_cn.md)

This directory provides lightweight instrumentation helpers for real-time control applications.

- `dsa_scope.h` captures selected signals for observation or transport.
- `dsa_trigger.h` defines trigger conditions for controlled capture.
- `sine_analyzer.h` estimates properties of sinusoidal signals.
- `ti_dlog/` contains TI-style data-logging support.

Keep capture buffers bounded and ensure analysis work fits the real-time budget of the calling task.
