/**
 * @file ctl_nano.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines the state machine and core object for the CTL-Nano framework.
 * @details This file provides the top-level structure for managing the lifecycle
 * of the motor controller. It includes a state machine to handle states like
 * pending, calibration, ready, online, and fault. It also defines the main
 * control object that holds all system-level state and parameters.
 *
 * @version 0.2
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_CTL_NANO_H_
#define _FILE_CTL_NANO_H_

// Basic headers
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl_main.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//// Enable GMP CTL Controller Framework Nano
//#ifndef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
//#define SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
//#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

/*---------------------------------------------------------------------------*/
/* CTL-Nano Framework Definitions                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CTL_NANO_FRAMEWORK_CORE CTL-Nano Framework Core
 * @ingroup CTL_FRAMEWORK_CORE
 * @brief Core definitions for the Nano scheduling framework.
 * @{
 */

/**
 * @brief Defines the states for the main controller state machine.
 */
typedef enum _tag_ctl_nano_state_machine
{
    /**
     * @brief System is pending and inactive.
     * In this state, all controller outputs are strictly prohibited,
     * and the control algorithm is bypassed. The user must explicitly
     * enable the system to proceed.
     */
    CTL_SM_PENDING = 0,

    /**
     * @brief System is in the calibration phase.
     * In this state, routines like ADC offset calibration and other
     * hardware security checks should be performed.
     */
    CTL_SM_CALIBRATE = 2,

    /**
     * @brief Controller is ready to be launched.
     * The system has been initialized and calibrated, and is waiting for
     * a command to start operation.
     */
    CTL_SM_READY = 3,

    /**
     * @brief Controller is in the run-up or preparation phase.
     * This state is intended for tasks like motor parameter identification,
     * auto-tuning, or a controlled startup sequence.
     */
    CTL_SM_RUNUP = 4,

    /**
     * @brief Controller is online and fully operational.
     * The main control loop is active, and the motor is under closed-loop control.
     */
    CTL_SM_ONLINE = 5,

    /**
     * @brief Controller has entered a fault state.
     * This state is entered upon detection of a critical error. All outputs
     * should be disabled, and fault handling logic is executed.
     */
    CTL_SM_FAULT = 6
} ctl_nano_state_machine;

/**
 * @brief A security key to verify that the controller object has been initialized.
 * @details This value corresponds to the ASCII characters for "GCTL".
 */
#define GMP_CTL_ENDORSE ((0x4743544C))

/**
 * @brief The core data object for the CTL-Nano framework.
 * @details This structure acts as the main handle for the entire controller,
 * holding all top-level state information, timing data, and configuration flags.
 */
typedef struct _tag_ctl_object_nano
{
    /**
     * @brief Security endorse field. Must equal GMP_CTL_ENDORSE to be considered valid.
     * This helps prevent the use of uninitialized controller objects.
     */
    uint32_t security_endorse;

    // --- Timing and Profiling ---
    uint_least32_t isr_tick;                   ///< Incremented on every control ISR call.
    uint_least32_t mainloop_tick;              ///< Incremented on every main loop dispatch call.
    uint_least32_t control_law_CPU_usage_tick; ///< Tick counter for profiling the core control law execution time.
    uint_least32_t mainloop_CPU_usage_tick;    ///< Tick counter for profiling the main loop execution time.

    // --- State and Configuration ---
    ctl_nano_state_machine state_machine; ///< The current state of the controller.
    ec_gt error_code;                     ///< Holds the last recorded error code.
    uint32_t ctrl_freq;                   ///< The frequency of the control ISR in Hz.
    ctl_divider_t div_monitor;            ///< A divider for scheduling the monitor routine.

    // --- Feature Switches ---
    fast_gt switch_calibrate_stage;  ///< Switch to enable/disable the CALIBRATE stage (Default: On).
    fast_gt switch_runup_stage;      ///< Switch to enable/disable the RUNUP stage (Default: Off).
    fast_gt switch_security_routine; ///< Switch to enable/disable the security monitoring routine.

} ctl_object_nano_t;

/** @} */ // end of CTL_NANO_FRAMEWORK_CORE group

/*---------------------------------------------------------------------------*/
/* User-Implemented Framework Interface (FMIF) Functions                     */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CTL_FMIF User Implementation Interface
 * @brief A set of callback functions that the user must implement to integrate
 * the framework with their specific application and hardware.
 * @{
 */

// ....................................................................//
// High-Frequency Routines (typically called from ISR)
// ....................................................................//

/** @brief User-defined routine for the sensor input stage. */
// void ctl_fmif_input_stage_routine(ctl_object_nano_t *pctl_obj);

/** @brief User-defined routine for the core control algorithm. */
// void ctl_fmif_core_stage_routine(ctl_object_nano_t *pctl_obj);

/** @brief User-defined routine for the actuator output stage. */
// void ctl_fmif_output_stage_routine(ctl_object_nano_t *pctl_obj);

/** @brief User-defined routine for real-time requests (e.g., SPI communication). */
// void ctl_fmif_request_stage_routine(ctl_object_nano_t *pctl_obj);

// ....................................................................//
// Low-Frequency Routines (typically called from main loop)
// ....................................................................//

/** @brief User-defined routine for monitoring and debugging. */
void ctl_fmif_monitor_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for safety checks during online operation. */
fast_gt ctl_fmif_security_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the PENDING state. */
fast_gt ctl_fmif_sm_pending_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the CALIBRATE state. */
fast_gt ctl_fmif_sm_calibrate_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the READY state. */
fast_gt ctl_fmif_sm_ready_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the RUNUP state. */
fast_gt ctl_fmif_sm_runup_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the ONLINE state. */
fast_gt ctl_fmif_sm_online_routine(ctl_object_nano_t* pctl_obj);

/** @brief User-defined routine for the FAULT state. */
fast_gt ctl_fmif_sm_fault_routine(ctl_object_nano_t* pctl_obj);

// ....................................................................//
// Hardware Abstraction Callbacks
// ....................................................................//

/** @brief User-defined function to enable PWM outputs. */
// inline void ctl_fmif_output_enable(ctl_object_nano_t *pctl_obj);

/** @brief User-defined function to disable PWM outputs. */
// inline void ctl_fmif_output_disable(ctl_object_nano_t *pctl_obj);

/** @} */ // end of CTL_FMIF group

/*---------------------------------------------------------------------------*/
/* Framework API Functions                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CTL_FRAMEWORK_API Framework API
 * @brief Functions provided by the framework to be called by the user.
 * @{
 */

/** @brief Dispatches the main state machine; call this in the main loop. */
void ctl_fm_state_dispatch(ctl_object_nano_t* pctl_obj);

/** @brief Inspects the controller object for validity; call after initialization. */
uint32_t ctl_fm_controller_inspection(ctl_object_nano_t* pctl_obj);

/**
 * @brief Initializes the header of the nano control object.
 * @param[out] ctl_obj Pointer to the nano control object.
 * @param[in]  ctrl_freq The frequency of the control ISR in Hz.
 */
void ctl_fm_init_nano_header(ctl_object_nano_t* ctl_obj, uint32_t ctrl_freq);

/**
 * @brief A global handle to the default nano control object.
 * @details The user must define and initialize this pointer in their application code.
 */
extern ctl_object_nano_t* ctl_nano_handle;

/** @} */ // end of CTL_FRAMEWORK_API group

/*---------------------------------------------------------------------------*/
/* Framework Utility Functions                                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CTL_FRAMEWORK_UTILITIES Framework Utilities
 * @brief Helper functions for state management and debugging.
 * @{
 */

/**
 * @brief DANGEROUS: Forces the controller into the ONLINE state.
 * @warning This bypasses all safety checks and startup procedures.
 * Only use for expert debugging purposes.
 * @param[out] ctl_obj Pointer to the nano control object.
 */
void ctl_fm_force_online(ctl_object_nano_t* ctl_obj);

/**
 * @brief Forces the controller into the CALIBRATE state.
 * @param[out] ctl_obj Pointer to the nano control object.
 */
void ctl_fm_force_calibrate(ctl_object_nano_t* ctl_obj);

/**
 * @brief Sets up a default configuration for the nano control object.
 * @param[out] ctl_obj Pointer to the nano control object.
 * @return An error code if setup fails.
 */
ec_gt ctl_setup_default_ctl_nano_obj(ctl_object_nano_t* ctl_obj);

/**
 * @brief Checks if the controller is currently in the ONLINE state.
 * @param[in] ctl_obj Pointer to the nano control object.
 * @return 1 if online, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_fm_is_online(ctl_object_nano_t* ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_ONLINE;
}

/**
 * @brief Checks if the controller is currently in the CALIBRATE state.
 * @param[in] ctl_obj Pointer to the nano control object.
 * @return 1 if in calibration, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_fm_is_calibrate(ctl_object_nano_t* ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_CALIBRATE;
}

/**
 * @brief Checks if the controller is currently in the RUNUP state.
 * @param[in] ctl_obj Pointer to the nano control object.
 * @return 1 if in run-up, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_fm_is_runup(ctl_object_nano_t* ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_RUNUP;
}

/**
 * @brief Changes the state of the controller's state machine.
 * @param[out] ctl_obj Pointer to the nano control object.
 * @param[in]  sm The new state to transition to.
 */
GMP_STATIC_INLINE void ctl_fm_change_state(ctl_object_nano_t* ctl_obj, ctl_nano_state_machine sm)
{
    ctl_obj->state_machine = sm;
}

/** @} */ // end of CTL_FRAMEWORK_UTILITIES group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_NANO_H_
