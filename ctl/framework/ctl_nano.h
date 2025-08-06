/**
 * @file ctl_nano.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// Basic headers
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl_main.h>
#include <gmp_core.h>

#ifndef _FILE_CTL_NANO_H_
#define _FILE_CTL_NANO_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Controller basic State machine
typedef enum _tag_ctl_nano_state_machine
{
    // State machine: system is pending
    // in this stage, all the controller output is strictly prohibited,
    // and the controller is bypassed.
    // User should enable the system then the output may happen.
    CTL_SM_PENDING = 0,

    // State machine: system is initializing
    // in this stage, PWM output with 0 output is permitted.
    // ADC device is ready, and PWM device is ready.
    // All the control law objects will initialize.
    //	CTL_SM_INIT = 1,

    // State machine: system is calibrating
    // in this stage, ADC zero point should be calibrated,
    // hardware security check is done in this stage
    CTL_SM_CALIBRATE = 2,

    // State machine: Controller is ready to launch
    // The state machine is ready for user to launch
    CTL_SM_READY = 3,

    // State machine: Controller is preparing
    // Auto tuning, auto measurement, auto identify is done here.
    // a set of switches will determine which routine may be invoke
    CTL_SM_RUNUP = 4,

    // State machine: Controller is on-line
    CTL_SM_ONLINE = 5,

    // State machine: Controller is in fault.
    CTL_SM_FAULT = 6
} ctl_nano_state_machine;

// endorse for CTL
#define GMP_CTL_ENDORSE ((0x4743544C))

// controller basic objects
typedef struct _tag_ctl_object_nano
{
    // This number must equals to GMP_CTL_ENDORSE, that is 0x47_43_54_4C (GCTL)
    // to ensure the controller is initialized correctly.
    uint32_t security_endorse;

    // ISR tick
    uint_least32_t isr_tick;

    // State Machine tick
    uint_least32_t mainloop_tick;

    // Calculating CPU usage for Control law
    uint_least32_t control_law_CPU_usage_tick;

    // Calculating CPU usage for Control Main loop
    uint_least32_t mainloop_CPU_usage_tick;

    // state machine of the controller
    ctl_nano_state_machine state_machine;

    // controller error code
    ec_gt error_code;

    // Control Period Counter
    //		uint32_t period_count;

    // Controller Frequency, Unit Hz
    uint32_t ctrl_freq;

    // Controller Monitor divider
    ctl_divider_t div_monitor;

    // Switch of calibrate stage
    // Default is On
    fast_gt switch_calibrate_stage;

    // Switch of calibrate stage
    // Default is Off
    fast_gt switch_runup_stage;

    // Switch of security module
    fast_gt switch_security_routine;

} ctl_object_nano_t;

//////////////////////////////////////////////////////////////////////////
// The following functions should implemented by user
// All these function should not be a pending function.

// ....................................................................//
// The following functions may running in Main ISR

// deal with all the adc and other input messages
// void ctl_fmif_input_stage_routine(ctl_object_nano_t *pctl_obj);

// implement all the controller routine
// this is a inline function implemented by user.
// void ctl_fmif_core_stage_routine(ctl_object_nano_t *pctl_obj);

// output all the PWM and other digital or analog signal
// void ctl_fmif_output_stage_routine(ctl_object_nano_t *pctl_obj);

// request other information via peripheral, for instance SPI.
// void ctl_fmif_request_stage_routine(ctl_object_nano_t *pctl_obj);

// ....................................................................//
// The following functions may running in Main Loop

// controller monitor routine
void ctl_fmif_monitor_routine(ctl_object_nano_t *pctl_obj);

// controller security routine.
// This function would only be called when controller is online.
fast_gt ctl_fmif_security_routine(ctl_object_nano_t *pctl_obj);

// controller pending routine
// When `CTL_SM_PENDING` state, this function would be called.
fast_gt ctl_fmif_sm_pending_routine(ctl_object_nano_t *pctl_obj);

// controller calibrate routine
// When `CTL_SM_CALIBRATE` state, this function would be called.
fast_gt ctl_fmif_sm_calibrate_routine(ctl_object_nano_t *pctl_obj);

// controller ready routine
// When `CTL_SM_READY` state,this function would be called.
fast_gt ctl_fmif_sm_ready_routine(ctl_object_nano_t *pctl_obj);

// controller run-up routine
// When `CTL_SM_RUNUP` state,this function would be called.
fast_gt ctl_fmif_sm_runup_routine(ctl_object_nano_t *pctl_obj);

// controller online routine
//  When `CTL_SM_ONLINE` state,this function would be called.
fast_gt ctl_fmif_sm_online_routine(ctl_object_nano_t *pctl_obj);

// controller fault routine
//  When `CTL_SM_FAULT` state,this function would be called.
fast_gt ctl_fmif_sm_fault_routine(ctl_object_nano_t *pctl_obj);

// ....................................................................//
// The following functions may called in Main ISR and Main Loop

// enable PWM output
// inline void ctl_fmif_output_enable(ctl_object_nano_t *pctl_obj);

// disable PWM output
// inline void ctl_fmif_output_disable(ctl_object_nano_t *pctl_obj);

//////////////////////////////////////////////////////////////////////////
// The following function should be called by user on time.

// In Main ISR
// void ctl_fm_periodic_dispatch(ctl_object_nano_t *pctl_obj);

// Controller core
// This function should be called in your controller Main ISR.
// This function may invoke all the other functions related to the controller.
// GMP_STATIC_INLINE
// void ctl_fm_periodic_dispatch(ctl_object_nano_t *pctl_obj);

// In Main Loop
void ctl_fm_state_dispatch(ctl_object_nano_t *pctl_obj);

// In Initialize function
uint32_t ctl_fm_controller_inspection(ctl_object_nano_t *pctl_obj);

//////////////////////////////////////////////////////////////////////////
// Function Prototypes

//// You should call this function to initialize the object
// void ctl_fm_init_nano_header(ctl_object_nano_t *ctl_obj);

//// Then you may call this function to set the details of the objects
// void ctl_fm_setup_nano_header(ctl_object_nano_t *ctl_obj,
//                               uint32_t ctrl_freq // the frequency of the control law, unit Hz
//);

// You should call this function to initialize the object
void ctl_fm_init_nano_header(
    // controller nano object
    ctl_object_nano_t *ctl_obj,
    // the frequency of the control law, unit Hz
    uint32_t ctrl_freq);

extern ctl_object_nano_t *ctl_nano_handle;

//////////////////////////////////////////////////////////////////////////
// Utilities

// DANGEROUS:
// This function may damage your controller.
// Call it iff you are EXPERT.
//
void ctl_fm_force_online(ctl_object_nano_t *ctl_obj);

void ctl_fm_force_calibrate(ctl_object_nano_t *ctl_obj);

// default ctl nano object
ec_gt ctl_setup_default_ctl_nano_obj(ctl_object_nano_t *ctl_obj);

// 1 means true
GMP_STATIC_INLINE
fast_gt ctl_fm_is_online(ctl_object_nano_t *ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_ONLINE;
}

// 1 means true
GMP_STATIC_INLINE
fast_gt ctl_fm_is_calibrate(ctl_object_nano_t *ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_CALIBRATE;
}

// 1 means true
GMP_STATIC_INLINE
fast_gt ctl_fm_is_runup(ctl_object_nano_t *ctl_obj)
{
    return ctl_obj->state_machine == CTL_SM_RUNUP;
}

// change state
GMP_STATIC_INLINE
void ctl_fm_change_state(ctl_object_nano_t *ctl_obj, ctl_nano_state_machine sm)
{
    ctl_obj->state_machine = sm;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_NANO_H_
