# Controller interface components

**English** | [简体中文](readme_cn.md)

This directory defines controller-facing abstractions for ADC, PWM, DAC, modulation, bias, gain, and GMP SIL integration. They isolate reusable control algorithms from target-specific peripheral registers.

Select the smallest interface that represents the required signal path, bind it to the platform implementation during initialization, and keep target register access outside reusable controller modules. Advanced PWM and modulation interfaces must still enforce the target's polarity, dead-time, and protection constraints.
