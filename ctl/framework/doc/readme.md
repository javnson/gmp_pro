# CTL Framework Quick Start Guide

In addition to providing a rich set of motor control algorithm modules, the CTL library also offers a high-level application framework called **CTL-Nano** to manage the entire lifecycle and task scheduling of the controller. Understanding this framework is key to successfully using this library.

The framework is primarily defined by two files, `ctl_nano.h` and `ctl_dispatch.h`, and its core ideas are **"State Machine Management"** and **"Time-Division Scheduling"**.

### 1. Core Concept: `ctl_object_nano_t`

The core structure `ctl_object_nano_t`, defined in `ctl_nano.h`, is the heart of the entire control system. You can think of it as a "main control board" that contains:

- **System State Machine (`state_machine`)**: Manages every step of the controller from power-on to operation and fault handling.
- **Timestamps (`isr_tick`, `mainloop_tick`)**: Tracks the execution cadence of high-frequency and low-frequency tasks.
- **Global Handle (`ctl_nano_handle`)**: A global pointer that allows any part of the system to access this "main control board".

### 2. Lifecycle: State Machine (`ctl_nano_state_machine`)

The controller has a well-defined lifecycle, strictly managed by the state machine to ensure safe and orderly system operation:

1. **`CTL_SM_PENDING`**: The initial state after power-on, where all outputs are disabled.
2. **`CTL_SM_CALIBRATE`**: Hardware self-checks, such as ADC offset calibration, are performed in this state.
3. **`CTL_SM_READY`**: Calibration is complete, and the system is waiting for a start command.
4. **`CTL_SM_RUNUP`**: Executes motor startup procedures, like parameter identification or open-loop pre-positioning.
5. **`CTL_SM_ONLINE`**: The main control mode is active, and the motor is under normal closed-loop control.
6. **`CTL_SM_FAULT`**: A critical error has been detected, leading to an immediate shutdown and fault handling.

### 3. Task Scheduling: High-Frequency vs. Low-Frequency Tasks

The CTL framework categorizes control tasks into two types:

- **High-Frequency Tasks (Executed in ISR)**: In `ctl_dispatch.h`, `gmp_base_ctl_step()` is the single entry point you need to call within your **timer interrupt service routine (ISR)**. It is responsible for executing the most time-critical tasks, such as:
  - **Input (`ctl_fmif_input_stage_routine`)**: Reading ADC currents, encoder positions, etc.
  - **Core Calculation (`ctl_fmif_core_stage_routine`)**: Executing FOC, SMO, and other core algorithms.
  - **Output (`ctl_fmif_output_stage_routine`)**: Updating PWM duty cycles.
- **Low-Frequency Tasks (Executed in Main Loop)**: In `ctl_nano.h`, `ctl_fm_state_dispatch()` is the function you need to call within your **main loop (`while(1)`)**. It handles less time-critical tasks, including:
  - **State Machine Dispatching**: Calling the appropriate handler function based on the current state (e.g., `ctl_fmif_sm_online_routine`).
  - **Monitoring (`ctl_fmif_monitor_routine`)**: Sending debug data to a host computer.
  - **Safety Checks (`ctl_fmif_security_routine`)**: Checking for over-current, over-voltage, and other faults.

### 4. How to Use: User's Responsibilities

To integrate the CTL framework into your project, you need to do two things:

1. **Define the Global Handle**: In one of your `.c` files, define and initialize the `ctl_object_nano_t` structure and the global pointer `ctl_nano_handle`.

   ```
   ctl_object_nano_t g_my_controller_obj;
   ctl_object_nano_t *ctl_nano_handle = &g_my_controller_obj;
   ```

2. **Implement `ctl_fmif_\*` Callback Functions**: The framework provides the skeleton; you need to implement the `ctl_fmif_*` functions declared in `ctl_nano.h` according to your specific hardware and application logic. For example, you would write the code to read your MCU's ADC registers in `ctl_fmif_input_stage_routine`, and the logic to handle user button presses to switch motor operating modes in `ctl_fmif_sm_online_routine`.

This approach clearly separates the complex real-time scheduling and state management from your specific application code, allowing you to focus more on implementing the core control algorithms.

### 5. Simplified Mode: Direct Dispatch

If your project does not require the complex state machine management provided by `CTL-Nano`, you can opt for a more direct and simpler dispatch mode.

**Prerequisite**: In your project configuration, **do not** define the `SPECIFY_ENABLE_CTL_FRAMEWORK_NANO` macro.

**How It Works**: In this mode, when you call `gmp_base_ctl_step()` in your ISR, it will no longer execute the `Nano` framework logic. Instead, it will sequentially call three **global callback functions** that you define:

1. `ctl_input_callback()`
2. `ctl_dispatch()`
3. `ctl_output_callback()`

**How to Use: User's Responsibilities** You need to implement these three functions in your code and clearly divide your control flow into these three stages:

1. **Implement `ctl_input_callback()`**:
   - **Purpose**: Responsible for all data **input** and **sampling**.
   - **Content**: Write the code to read hardware peripherals like ADCs, encoders, and Hall sensors here, and update the sampled values into your top-level controller structure.
2. **Implement `ctl_dispatch()`**:
   - **Purpose**: Responsible for the core control algorithm **calculation**.
   - **Content**: Call the `step` function of your chosen top-level controller module here, for example, `ctl_step_pmsm_ctrl()`. This function will perform the FOC calculations and determine the final PWM duty cycles.
3. **Implement `ctl_output_callback()`**:
   - **Purpose**: Responsible for all data **output** and **actuation**.
   - **Content**: Write the code to update the MCU's timer compare registers with the PWM duty cycles calculated by the `step` function.

**Example Code**: Your interrupt service routine will look very clean and simple:

```
// Implement these three callbacks in one of your .c files
void ctl_input_callback(void) {
    // Read ADC and encoder values...
    update_adc_inputs(&g_my_pmsm_controller.mtr_interface);
    update_encoder_inputs(&g_my_pmsm_controller.mtr_interface);
}

void ctl_dispatch(void) {
    // Execute the core FOC calculation
    ctl_step_pmsm_ctrl(&g_my_pmsm_controller);
}

void ctl_output_callback(void) {
    // Write the results to the PWM registers
    update_pwm_outputs(g_my_pmsm_controller.pwm_out);
}

// In your timer interrupt service routine
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        
        // Just call this single function
        gmp_base_ctl_step();
    }
}
```

While this mode lacks the safety guarantees and lifecycle management of the state machine, its simple structure and direct execution path make it ideal for rapid prototyping or projects with relatively simple logic.