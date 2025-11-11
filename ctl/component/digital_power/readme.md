# GMP CTL DP Digital Power Control Library Documentation

## Introduction

This document serves as the official user guide for the **Digital Power Control Library**. This library is designed to provide power electronics engineers and embedded developers with a powerful, portable, and highly modular firmware toolkit. By encapsulating common power topology control algorithms, advanced grid-tied strategies, protection mechanisms, and hardware interfaces, this library significantly accelerates the development process from simulation to prototype.

### Design Philosophy

- **Modularity**: Every function (e.g., PID, PLL, MPPT) is an independent component, allowing users to freely combine them according to their needs.
- **Portability**: Core algorithms are decoupled from the hardware abstraction layer, facilitating easy migration across different microcontroller (MCU) platforms.
- **High Performance**: The algorithms balance theoretical rigor with real-time performance in an embedded environment.

### Library Structure Overview

The library consists of the following five core sub-modules:

- **1. Basic Module**: Provides the fundamental building blocks for complex systems, including classic DC/DC converter control, universal protection strategies, and simulation interfaces.
- **2. MPPT (Maximum Power Point Tracking) Module**: Offers high-efficiency MPPT algorithms for photovoltaic (PV) systems.
- **3. Single-Phase Converter Module**: Provides control and modulation algorithms for applications like single-phase grid-tied inverters, active front-ends (AFE), and PFC rectifiers.
- **4. Three-Phase Converter Module**: Delivers advanced, d-q decoupling-based control and modulation algorithms for applications such as three-phase grid-tied inverters and motor drives.
- **5. Hardware Preset Module**: Contains preset sensor calibration parameters for specific hardware platforms, decoupling application-level code from hardware specifics.

## 1. Basic Module

The `Basic` module is the cornerstone of the entire library, containing control functions for fundamental power topologies and universal components.

- **1.1 Buck/Boost Converter Controllers (`buck.h`, `boost.h`)**
  - **Function**: Provides a generic **cascaded dual-loop PID controller (outer voltage loop, inner current loop)** for classic Buck and Boost circuits. This control structure is the standard for achieving high dynamic performance and precise steady-state output.
  - **Features**: Supports multiple operating modes, including voltage, current, and open-loop, allowing users to switch flexibly based on application requirements (e.g., constant voltage output, constant current charging).
- **1.2 4-Switch Buck-Boost Duty Cycle Preset (`buckboost.h`)**
  - **Function**: Provides a duty cycle calculation strategy for **4-switch non-inverting Buck-Boost converters**, which require a wide voltage regulation range.
  - **Features**: Achieves a smooth, seamless transition from Buck to Boost mode through a clever four-region operational division.
- **1.3 Universal Protection Strategy (`protection_strategy.h`)**
  - **Function**: Offers a standard **"brick-wall" protection** scheme that can simultaneously monitor output voltage, current, and power.
  - **Features**: When any parameter exceeds its configured safety threshold, a latchable error flag is immediately triggered, providing fundamental and reliable hardware protection for the system.
- **1.4 Software-in-the-Loop (SIL) Interface (`std_sil_dp_interface.h`)**
  - **Function**: Defines the standard data interface for **Software-in-the-Loop (SIL)** simulation.
  - **Features**: Uses the `#pragma pack(1)` directive to enforce 1-byte alignment for data structures, ensuring seamless integration with simulation environments like Matlab/Simulink. This is a key element in achieving Model-Based Design (MBD).

## 2. MPPT (Maximum Power Point Tracking) Module

The `MPPT` module is specifically designed for photovoltaic systems, aiming to track the solar panel's maximum power point in real-time to maximize energy harvesting efficiency.

- **2.1 Perturb and Observe (P&O) (`PnO_algorithm.h`)**
  - **Principle**: Iteratively finds the maximum power point by periodically perturbing the operating voltage of the solar panel and observing the resulting direction of power change.
  - **Features**: Implements an advanced P&O algorithm with an **adaptive step size**. It uses a large step size for fast tracking during rapid irradiance changes and a small step size near the MPP to reduce steady-state oscillations, balancing dynamic response and steady-state efficiency.
- **2.2 Incremental Conductance (INC) (`INC_algorithm.h`)**
  - **Principle**: Based on the mathematical property that `dP/dV=0` at the MPP, it accurately determines the operating point's location by comparing the incremental conductance (`dI/dV`) with the negative instantaneous conductance (`-I/V`).
  - **Features**: Compared to P&O, the INC method can theoretically respond more quickly and accurately to sudden changes in irradiance, making it particularly suitable for variable weather conditions.

## 3. Single-Phase Converter Module

The `Single-Phase Converter Module` provides a complete set of control algorithms for single-phase AC/DC and DC/AC conversion, forming the core for applications like single-phase PFC, grid-tied inverters, and uninterruptible power supplies (UPS).

- **3.1 Core Controller (`single_phase_dc_ac.h`)**
  - **Architecture**: A comprehensive top-level controller for single-phase converters, integrating phase-locking, multi-mode control, and advanced compensation algorithms.
  - **Key Features**:
    - **Harmonic Compensation**: Employs **Quasi-Proportional-Resonant (QPR/QR) controllers** to accurately compensate for multiple high-order harmonics (3rd, 5th, 7th... up to 15th) in the grid current, significantly improving the output current's THD (Total Harmonic Distortion).
    - **Multi-Mode Operation**: Supports various modes, including open-loop, current closed-loop, and voltage closed-loop, and can be configured for either inverter or rectifier operation.
    - **Accurate AC Measurement**: Includes a built-in True RMS calculation module based on a `sine_analyzer` to provide precise feedback for closed-loop control.
- **3.2 Single-Phase PLL (SPLL) (`spll.h`)**
  - **Architecture**: Implements a high-performance single-phase Phase-Locked Loop based on a **Second-Order Generalized Integrator (SOGI)**.
  - **Purpose**: In grid-tied applications, it accurately extracts the fundamental phase and frequency from a grid voltage that may be rich in harmonics, which is a prerequisite for stable grid connection and unity power factor.
- **3.3 Other Components**
  - **Single-Phase PFC Controller (`spfc.h`)**: Provides a concise and efficient dual-loop controller for Boost-type single-phase PFC rectifiers.
  - **Single-Phase Modulation (`sp_modulation.h`)**: Implements a single-phase unipolar SPWM modulation algorithm with **dead-time compensation**, effectively reducing waveform distortion caused by hardware dead-time.

## 4. Three-Phase Converter Module

The `Three-Phase Converter Module`, based on modern AC control theory, offers a feature-rich and high-performance set of control algorithms suitable for complex applications like three-phase grid-tied inverters, active front-ends (AFE), and high-performance motor drives.

- **4.1 Core Controller (`three_phase_dc_ac.h`)**
  - **Architecture**: A top-level controller for three-phase converters based on **d-q synchronous rotating frame decoupling control**, integrating multiple industry-leading advanced control strategies.
  - **Key Features**:
    - **Dual-Sequence Control (Positive/Negative)**: Capable of independently controlling both the positive and negative sequence components of the grid. During grid voltage imbalance, it can actively suppress negative-sequence currents to meet the most stringent grid codes.
    - **Droop Control**: Includes standard P-f and Q-V droop controllers, enabling the inverter to operate stably not only in grid-tied mode but also in island/off-grid mode, and supports paralleling of multiple units.
    - **Multi-Harmonic and Zero-Sequence Compensation**: Accurately compensates for 5th and 7th harmonics in the d-q rotating frame and suppresses 3rd and 9th harmonics in the zero-sequence component, comprehensively improving power quality.
- **4.2 Three-Phase PLL (SRF-PLL) (`pll.h`)**
  - **Architecture**: Implements a standard three-phase Synchronous Reference Frame PLL (SRF-PLL).
  - **Purpose**: This is the most classic and reliable phase-locking method for three-phase systems, providing the controller with a high-precision grid synchronization signal.
- **4.3 Other Components**
  - **Three-Phase Modulation (`tp_modulation.h`)**: Implements a three-phase bridge modulation algorithm with **dead-time compensation**.
  - **Three-Phase Vienna Rectifier (`Vienna.h`)**: This is currently a **placeholder module**, outlining the future implementation of a controller for three-phase Vienna rectifiers, which are used in high-efficiency/high-power PFC applications.

## 5. Hardware Preset Module

The `Hardware Preset Module` provides preset sensor calibration parameters for specific hardware platforms. This design decouples low-level hardware parameters from the high-level control algorithms, allowing the algorithm code to be reused on different hardware without modification.

The formula to convert ADC readings to physical values is:

- `Physical Voltage = (ADC Voltage Reading - Voltage Bias) / Voltage Gain`
- `Physical Current = (ADC Voltage Reading - Current Bias) / Current Gain`

The table below lists the calibration parameters for supported hardware platforms:

| Hardware Solution      | Voltage Gain    | Voltage Bias | Current Gain     | Current Bias |
| ---------------------- | --------------- | ------------ | ---------------- | ------------ |
| Diansai Half-Bridge v1 | `(2.2 / 202.2)` | `0.9966 V`   | `(0.005 * 20.0)` | `0.9 V`      |
| ... (More to be added) | ...             | ...          | ...              | ...          |