# CTL Control Template Library - Documentation

## Introduction

The CTL (Control Template Library) is a modular, reusable C-language library designed for digital control systems. It provides a series of validated, independent control modules aimed at simplifying and accelerating the development of embedded control applications.

These modules cover a wide range of common functionalities, from basic signal processing to advanced control strategies, and are designed for easy integration into various microcontroller (MCU) platforms.

## Module Overview

The library consists of the following core module categories, each tailored for specific control requirements:

- **Basic Modules**: Provide fundamental functions for signal processing and generation, serving as the building blocks for more complex systems.
- **Continuous Controllers**: Include classic controllers like PID designed based on continuous-domain (S-domain) theory, which are then discretized for implementation in digital systems.
- **Discrete Controllers**: Offer controllers and filters designed directly in the discrete-domain (Z-domain), forming the core of digital control.
- **Magnitude-based Control**: Contain nonlinear controllers whose decision logic depends directly on the signal's magnitude, typically offering very fast response times.
- **Protection Modules**: Are used to implement system safety features, such as monitoring and responding to over-current, over-voltage, and power quality anomalies.
- **Advanced Controllers**: Implement more complex modern control algorithms, such as adaptive control.

## Developer Guide: API Naming Convention

To maintain code consistency and readability, all functions in this library follow a unified naming convention. This helps developers quickly understand a function's purpose and usage.

**Basic Structure:**

```
ctl_<action>_<target_object>_via_<data_source>_<configurations>
```

**Component Descriptions:**

- **`ctl_`**: A fixed prefix for all library functions.
- **`<action>`**: Describes the function's primary operation.
  - `init`, `setup`: Used for module initialization, typically called only once at startup.
  - `step`: Executes the module's calculations for each control cycle; the core function in real-time loops (e.g., ISR).
  - `get`, `set`: Used to retrieve or modify the internal state or parameters of a module.
  - `input`, `output`: Marks functions called at specific stages (input/output) of the control flow.
- **`<target_object>`**: Specifies the target module the function operates on, e.g., `pid`, `ramp`, `sogi`.
- **`_via_<data_source>`**: (Optional) Specifies the data source or method the function relies on, e.g., `_via_amp_freq` indicates setup via amplitude and frequency parameters.
- **`_<configurations>`**: (Optional) Differentiates between various implementations or configurations of the same module, e.g., `_par` for a parallel topology, `_2` for a second implementation.

**Examples:**

- `ctl_init_pid`, `ctl_set_pid_limit`: Functions from `pid.h`.
- `ctl_init_ramp_generator_via_freq`, `ctl_step_ramp_generator`: Functions from `signal_generator.h`.

## 1. Basic Modules

The `basic` submodule provides a series of essential functions for control systems.

- **1.1 Frequency Divider (`divider.h`)**: Implements a counter-based frequency divider to generate low-frequency trigger signals from high-frequency events, suitable for executing slow tasks within a fast interrupt.
- **1.2 Saturation / Limiter (`saturation.h`)**: Provides standard saturation (limiter) functionality to clamp a signal within specified upper and lower bounds. It also supports a bipolar saturation mode for setting different limits for positive and negative signals.
- **1.3 Slope Limiter (`slope_limiter.h`)**: Also known as a rate limiter, it constrains the rate of change of a signal to prevent abrupt jumps, often used for smooth setpoint transitions.

## 2. Continuous Controllers

This module contains classic controllers designed based on continuous-time theory.

- **2.1 Continuous-Form PID Controller (`continuous_pid.h`)**: Provides standard and anti-windup PID controllers discretized from the continuous PID formula, supporting both series and parallel forms.
- **2.2 Tracking PID Controller (`track_pid.h`)**: A composite controller that combines a frequency divider, a slope limiter, and a continuous PID controller, designed for scenarios requiring smooth setpoint tracking.
- **2.3 Second-Order Generalized Integrator (SOGI) (`sogi.h`)**: Implements a SOGI, which is a frequency-adaptive filter that can generate in-phase and 90°-shifted (quadrature) sinusoidal signals from an input. It is a core component in applications like Phase-Locked Loops (PLLs).
- **2.4 S-Domain Transfer Function (`s_function.h`)**: Provides a generic second-order S-domain transfer function module. Users can quickly design and implement a controller or filter by intuitively specifying pole and zero frequencies in the S-plane.

## 3. Discrete Controllers

This module offers a rich variety of controllers, filters, and signal generators designed directly in the discrete-time domain (Z-domain).

- **3.1 Discrete PID and Tracking PID (`discrete_pid.h`, `track_discrete_pid.h`)**: Provides a standard incremental discrete PID controller, along with a composite tracking PID that integrates slope generation and frequency division.
- **3.2 Resonant and Proportional-Resonant Controllers (`proportional_resonant.h`)**: Implements various resonant controllers (R, PR, QR, QPR) that provide very high gain at a specific resonant frequency. They are key to achieving zero steady-state error in AC signal tracking, especially in power electronics applications like grid-tied inverters.
- **3.3 Discrete Filters and Compensators (`discrete_filter.h`, `pole_zero.h`, `lead_lag.h`)**: Offers an extensive library of linear filters, including 1st/2nd-order IIR filters, configurable general-purpose pole-zero compensators (1P1Z, 2P2Z, 3P3Z), and classic lead-lag compensators for system correction and noise suppression.
- **3.4 Generic Z-Domain Transfer Function (`z_function.h`)**: A general-purpose IIR filter module that allows users to implement any linear time-invariant controller or filter by directly providing the numerator and denominator coefficients of its Z-domain transfer function.
- **3.5 Discrete SOGI (`discrete_sogi.h`)**: The discrete-form version of the Second-Order Generalized Integrator, with functionality similar to the continuous version. It is fundamental for digital PLLs and advanced signal analysis.
- **3.6 Signal Generators (`signal_generator.h`)**: Includes common signal generators for simulation, testing, and system excitation, such as an efficient sine/cosine wave generator based on phasor rotation and a ramp signal (sawtooth) generator.

## 4. Magnitude-based Control

This module contains nonlinear controllers whose control decisions are based directly on the signal's current magnitude, typically resulting in very fast dynamic responses.

- **4.1 Hysteresis Controller (`hysteresis_controller.h`)**: Also known as a "Bang-Bang" controller, its output switches only between two states (0 and 1). The state flips when the input signal crosses the boundaries of a predefined hysteresis band. It is often used for current loop control due to its extremely fast response.
- **4.2 Sliding Mode Controller (SMC) (`smc.h`)**: Implements a Sliding Mode Controller, a highly robust nonlinear control method known for its effectiveness in dealing with parameter uncertainties and external disturbances.

## 5. Protection Modules

This module provides key components for ensuring system safety.

- **5.1 Generic Boundary Protection Monitor (`protection.h`)**: A configurable protection system that can monitor multiple variables simultaneously, checking if they exceed their predefined safe operating limits.
- **5.2 Three-Stage Inverse-Time Overcurrent Protection (`itoc_protection.h`)**: Mimics the behavior of an industrial circuit breaker by providing three levels of overcurrent protection: **instantaneous** trip for very high currents, **short-time delay** trip for moderate fault currents, and **long-time delay** trip for minor overload conditions.
- **5.3 Voltage Sag/Swell Detector (`sag_swell.h`)**: Specialized for detecting power quality issues. It uses an internal SOGI to accurately calculate the fundamental amplitude of the input voltage and determines if a sag or swell event has occurred based on whether the amplitude remains outside nominal thresholds for a specified duration.

## 6. Advanced Controllers

This module provides more complex control algorithms and tools for implementing advanced features like adaptive control.

- **6.1 Look-up Tables and Interpolation (`surf_search.h`)**: Provides the foundational tool for implementing complex nonlinear functions: the Look-Up Table (LUT). It supports 1D and 2D (uniform/non-uniform grid) tables and includes efficient search (binary search) and interpolation (linear/bilinear) algorithms, essential for high-performance nonlinear control.
- **6.2 Fuzzy PID Controller (`fuzzy_pid.h`)**: Implements a fuzzy self-tuning PID controller. It uses the **error (E)** and the **change in error (EC)** as inputs to dynamically adjust the PID parameters `Kp`, `Ki`, and `Kd` by querying predefined 2D look-up tables (i.e., fuzzy rules). This allows the controller to adapt to changes in system dynamics in real-time.

## Conclusion

The CTL Control Template Library provides a comprehensive, modular, and easy-to-use set of tools designed to help you quickly build stable and efficient digital control systems. We hope this documentation helps you to better understand and utilize this library.
