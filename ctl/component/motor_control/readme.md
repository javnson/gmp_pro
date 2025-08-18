# CTL (Control Template Library) Motor Control Library - Documentation

## Introduction

The CTL (Control Template Library) Motor Control Library is a high-performance, modular motor control algorithm library designed for embedded systems. It aims to provide a standardized, reusable set of components for the rapid development and deployment of control systems for various motors, such as Permanent Magnet Synchronous Motors (PMSM) and AC Induction Motors (ACM).

The core philosophy of this library is modularity and interface standardization, allowing developers to combine different functional modules like building blocks to suit a wide range of applications, from simple open-loop control to complex, high-performance Field-Oriented Control (FOC).

## Module Overview

The CTL library is divided into several functional layers, each containing a specific set of modules. The main module categories include:

* **Core Modules (Basic Components)**: Provide the most fundamental building blocks for motor control.
* **Consultant Modules**: Used to manage, calculate, and convert motor and driver parameters.
* **Current Controller**: The core for implementing closed-loop current control.
* **Trajectory Planning & Motion Control**: Responsible for generating smooth motion profiles and implementing position/speed closed-loop control.
* **Observer**: Used to estimate motor states, such as speed, position, and flux linkage.
* **Motor Controller**: Highly integrated control schemes for specific motor types.
* **Presets**: Provide pre-defined parameters for specific motors and inverter hardware.

---

## Core Modules (Basic Components)

Core modules are the foundation upon which all advanced controllers are built. They provide independent functionalities such as sensor data processing, coordinate transformations, and PWM generation.

* **Universal Motor Interface (`motor_universal_interface.h`)**: Defines standardized sensor and actuator interfaces (e.g., `rotation_ift`, `velocity_ift`) and unifies them through an aggregator structure `mtr_ift`, decoupling high-level algorithms from underlying hardware.
* **Encoder Processing (`encoder.h`)**: Provides data processing for various encoders (single-turn/multi-turn absolute, auto-turn-counting) and includes a tool for calculating speed by differentiating position.
* **Encoder Calibration (`encoder_calibrate.h`)**: Implements an automated state machine that injects a d-axis current to align the rotor, used for calibrating the electrical zero offset of an absolute position encoder.
* **SVPWM Generation (`svpwm.h`)**: Implements the Space Vector Pulse Width Modulation algorithm, which converts a voltage command vector in the stationary ¦Á-¦Â frame into three-phase PWM duty cycles.
* **Voltage Decoupling (`decouple.h`)**: Provides the cross-coupling feed-forward voltage calculation required for the d-q axis current loops in FOC, supporting both PMSM and ACM.
* **V/F Curve Generation (`vf_generator.h`)**: Used for implementing open-loop V/F (Voltage/Frequency) control, including generators for constant frequency, sloped frequency, and a complete V/F profile.
* **Voltage Reconstruction (`voltage_calculator.h`)**: Reconstructs the three-phase AC voltages and ¦Á-¦Â frame voltages applied to the motor based on the DC bus voltage and SVPWM duty cycles.
* **Software-in-the-Loop (SIL) Interface (`std_sil_motor_interface.h`)**: Defines standard interface structures for data exchange with simulation environments like Simulink, ensuring cross-platform data compatibility with `#pragma pack(1)`.

---

## Consultant Modules

The "Consultant" modules are a key feature of the CTL library. They encapsulate complex parameter calculations, unit conversions, and controller tuning processes into a series of easy-to-use "parameter experts," greatly simplifying the system configuration workflow.

* **Unit & Parameter Conversion (`motor_unit_calculator.h`)**: Provides a series of pre-defined physical constants and unit conversion macros, forming the basis for all parameter calculations.
* **Per-Unit System Consultant (`motor_per_unit_consultant.h`)**: Automatically calculates a complete and consistent set of per-unit base values from the system's fundamental ratings (power, voltage, frequency).
* **Driver Consultant (`motor_driver_consultant.h`)**: Centrally manages parameters related to the motor driver hardware and controller software configuration (e.g., power ratings, control frequencies, ADC parameters) through compile-time macros and a runtime structure.
* **PMSM/ACM Consultant (`pmsm_consultant.h`, `acm_consultant.h`)**: Manages the nameplate and design parameters for specific motor types, providing functions for derived parameter calculation (e.g., torque constant) and automatic PI controller tuning.

---

## Current Controller

The current controller is the core of any high-performance motor drive. This library offers a variety of current control schemes, from classic PI controllers to advanced strategies, along with comprehensive current reference generation logic.

* **FOC Core Current Loop (`motor_current_ctrl.h`)**: A standard FOC current regulator, including coordinate transformations and d-q axis PI controllers, which serves as the foundation for most control strategies.
* **Current Reference Generation Strategies**:
    * **Generic Current Distributor (`current_distributor.h`)**: Decomposes a total current command into Id and Iq components using a lookup table (LUT), enabling custom current distribution strategies.
    * **Maximum Torque Per Ampere (MTPA) (`mtpa.h`, `mtpa_pu.h`)**: Designed for salient-pole motors, it calculates the optimal Id and Iq via an analytical formula to achieve maximum efficiency.
    * **Field Weakening (MTPV) (`mtpv.h`, `mtpv_pu.h`)**: Used for operation above base speed, it injects negative d-axis current to reduce back-EMF and extend the motor's high-speed operating range.
* **Advanced/Alternative Current Controllers**:
    * **Linear Active Disturbance Rejection Controller (LADRC) (`ladrc_current_controller.h`)**: Based on an Extended State Observer (LESO), it actively compensates for system disturbances, offering strong robustness and simple tuning.
    * **Dead-beat Predictive Current Controller (DPCC) (`pmsm_dpcc.h`)**: A model-based predictive controller designed for extremely fast current dynamic response, theoretically capable of eliminating errors in a single cycle.
    * **Direct Torque Control (DTC) (`dtc.h`)**: A high-performance control strategy distinct from FOC. It directly controls flux and torque via hysteresis controllers, offering extremely fast torque response.

---

## Trajectory Planning & Motion Control

This chapter's modules are responsible for generating smooth motion profiles and implementing high-level motion closed-loop control.

* **Motion Trajectory Generators**:
    * **Trapezoidal Velocity Profile (`trapezoidal_trajectory.h`)**: A classic position planner that implements constant acceleration, constant velocity, and constant deceleration phases. Simple to implement.
    * **S-Curve Velocity Profile (`s_curve_trajectory.h`)**: Smooths the velocity profile by controlling jerk (the rate of change of acceleration), resulting in smoother motion and reduced shock.
    * **Sinusoidal (Cycloidal) Profile (`sinusoidal_trajectory.h`)**: Generates a trajectory based on a cycloidal function, providing optimal smoothness where acceleration and jerk are zero at the start and end of motion.
* **Position/Speed Controllers**:
    * **Basic P-Control Position Loop (`basic_pos_loop_p.h`)**: A standard proportional (P) position controller for servo applications.
    * **Haptic Knob Position Loop (`knob_pos_loop.h`)**: Simulates a physical knob with detents, intended for human-machine interaction.
    * **Linear Active Disturbance Rejection Speed Controller (LADRC) (`ladrc_spd_ctrl.h`)**: A robust controller for the speed loop, insensitive to load variations.

---

## Observer

The observer is the "eyes" of an advanced motor control algorithm. Especially in sensorless control, it is responsible for estimating the motor's internal states from easily measurable external electrical quantities.

* **PMSM Observers**:
    * **Sensored Flux Observer (`pmsm.fo.h`)**: Requires a position sensor to accurately calculate stator flux and torque.
    * **Sliding Mode Observer (SMO) (`pmsm.smo.h`)**: A classic sensorless position estimation method that works by tracking the back-EMF, suitable for medium to high speeds.
    * **High-Frequency Injection (HFI) Observer (`pmsm.hfi.h`)**: A sensorless position estimation technique specifically for zero and low-speed operation, which utilizes motor saliency.
* **AC Induction Motor (ACM) Observers**:
    * **Sensored Flux Observer (`acm.fo.h`)**: Estimates rotor flux and torque based on the voltage model.
    * **Rotor Flux Angle Estimator (`acm.pos_calc.h`)**: The core of Indirect Field-Oriented Control (IFOC) for ACMs, determining the flux angle by calculating the slip frequency.

---

## Motor Controller

The motor controller modules are the top-level application wrappers in this library. They integrate the underlying core modules, controllers, observers, and advanced strategies into complete, ready-to-use solutions for specific motor types.

* **Permanent Magnet Synchronous Motor (PMSM) Controllers**:
    * **Standard Sensored FOC Controller (`pmsm_ctrl.h`)**: A basic, three-loop FOC controller that requires an encoder.
    * **MTPA FOC Controller (`pmsm_ctrl_mtpa.h`)**: Integrates the MTPA strategy on top of the standard FOC to improve efficiency.
    * **SMO Sensorless Controller (`pmsm_ctrl_smo.h`)**: Integrates the SMO and an open-loop startup routine for medium- to high-speed sensorless applications.
    * **HFI Sensorless Controller (`pmsm_ctrl_hfi.h`)**: Integrates the HFI observer for zero and low-speed sensorless applications.
* **AC Induction Motor (ACM) Controller**:
    * **Standard Sensored FOC Controller (`acm_sensored_ctrl.h`)**: Implements rotor-flux-oriented IFOC, requiring a speed sensor.

---

## Presets

The presets modules provide a series of pre-defined parameters for specific hardware platforms or algorithms, aiming to achieve a "plug-and-play" rapid development experience.

* **Hardware Presets**: Define key physical parameters for specific inverter boards, primarily the calibration values for the ADC sampling circuits.
    * `GMP_LV_3PH_GAN_INV.h`, `GMP_3PH_213L6SINV_DUAL.h`, `SE_PWR_BD.h`, `TI_3PH_GAN_INV.h`
* **Algorithm Presets**:
    * `CURRENT_DISTRIBUTOR_LUT.h`: Provides a pre-defined lookup table for the **Generic Current Distributor** to implement custom current distribution strategies.
* **Motor Presets**: Provide complete parameter sets for specific motor models.
    * `ACM_4P24V.h`, `GBM2804H_100T.h`, `HBL48ZL400330K.h`, `PMSRM_4P_15KW520V.h`, `TYI_5008_KV335.h`, `TYI_5010_360KV.h`

---

## How to Use the CTL Library

A typical workflow for building a motor control application with this library is as follows:

1.  **Project Setup & Parameter Configuration**:
    * In your project, first include your chosen **Motor Preset** header (e.g., `TYI_5008_KV335.h`) and **Hardware Preset** header (e.g., `GMP_LV_3PH_GAN_INV.h`). These files provide most of the required base parameters via macros.
    * Use these preset macros to initialize the **Consultant Modules** (e.g., `pmsm_consultant_t`, `motor_driver_consultant_t`). The consultants will help you automatically calculate derived parameters and controller gains.

2.  **Select and Initialize the Controller**:
    * Based on your application needs, select a top-level controller from the **Motor Controller** modules (e.g., `pmsm_ctrl.h` if an encoder is used, or `pmsm_ctrl_smo.h` for medium-speed sensorless control).
    * Define an instance of this controller (e.g., `pmsm_bare_controller_t my_motor;`).
    * Create a corresponding initialization struct (e.g., `pmsm_bare_controller_init_t init_params;`) and populate it with the parameters calculated by the **Consultant Modules**.
    * Call the controller's initialization function (e.g., `ctl_init_pmsm_bare_controller(&my_motor, &init_params);`).

3.  **Attach Hardware Interfaces**:
    * In your platform-specific code, initialize hardware drivers for ADC, PWM, encoders, etc., and ensure their data conforms to the interface standards defined in the **Core Modules** (e.g., `tri_adc_ift`, `tri_pwm_ift`, `rotation_ift`).
    * Use the `ctl_attach_*` functions provided by the controller to attach these hardware interface instances. This is the crucial step that connects the software algorithms to the physical hardware.

4.  **Implement the Control Interrupt Service Routine (ISR)**:
    * In a timer-based interrupt, call the CTL library functions in the "**Sample-Calculate-Output**" sequence:
        ```c
        void control_isr() {
            // 1. Sample: Update inputs from ADC and encoder
            update_adc_inputs(&my_motor.mtr_interface);
            update_encoder_inputs(&my_motor.mtr_interface);

            // 2. Calculate: Execute the core controller algorithm
            ctl_step_pmsm_ctrl(&my_motor);

            // 3. Output: Write the results to the PWM registers
            update_pwm_outputs(my_motor.pwm_out);
        }
        ```

5.  **Control Modes and Start/Stop**:
    * In your main loop or state machine, control the motor by calling the controller's API.
    * Before starting, always call the `ctl_clear_*_ctrl()` function to reset internal states.
    * Call `ctl_enable_*_ctrl()` to enable the controller.
    * As needed, call mode-switching functions (e.g., `ctl_pmsm_ctrl_velocity_mode()`) and set new targets (e.g., `ctl_set_pmsm_ctrl_speed()`).

---

## How to Extend the CTL Library

The CTL library is designed to be extensible. If you wish to add new algorithm modules, please follow these principles and steps:

* **Core Design Philosophy**: **Modularity** and **Interface Standardization**. A new module should be a functionally cohesive unit and interact with other modules through standard interfaces.

* **Adhere to Standard Interfaces**:
    * If you are developing a **new observer**, it should provide a standard `rotation_ift` or `velocity_ift` as its output, so that higher-level controllers can seamlessly attach it as a position/speed feedback source.
    * If you are developing a **new hardware abstraction** (e.g., a new ADC driver), it should package its data into a standard interface like `tri_adc_ift` for use by higher-level controllers.

* **Steps to Add a New Module (e.g., a New Observer)**:
    1.  **Create File**: Create a new header file, e.g., `pmsm_my_observer.h`.
    2.  **Define Data Structure**: Define the necessary data structure for the observer in the file, e.g., `pmsm_my_observer_t`.
    3.  **Provide Standard Output**: Your data structure **must** include a standard output interface, for example, `rotation_ift encif;`.
    4.  **Write API**: Provide standard `init`, `setup`, `step`, and `clear` functions. The `step` function is responsible for executing the core algorithm and updating its final estimated position and speed into the `encif` member.
    5.  **Integrate and Test**: In a top-level **Motor Controller**, add an instance of your new observer and attach it to the FOC algorithm using the `ctl_attach_mtr_position()` function for testing.

* **Code Style and Conventions**:
    * All public API functions should be prefixed with `ctl_`.
    * Whenever possible, use `GMP_STATIC_INLINE` for frequently called functions to improve performance.
    * Write clear, Doxygen-style comments for all data structures, functions, and important parameters to enable automatic documentation generation.
