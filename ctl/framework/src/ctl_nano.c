/**
 * @file ctl_nano.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <core/base/gmp_base.h>

#include <ctl/framework/ctl_nano.h>

#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

// controller nano object handle entity
ctl_object_nano_t *ctl_nano_handle = NULL;

//////////////////////////////////////////////////////////////////////////
// WEAK FUNCTION DEFINITION

// This is the main file of GMP CTL (controller template library) module,
// The functions provided in this file may be invoked in your controller Main ISR or Main loop.
// User should follow the instruction of the function manual.

#ifdef _MSC_VER

// ....................................................................//
// The following functions may running in Main ISR

#pragma comment(linker, "/alternatename:ctl_fmif_input_stage_routine=default_ctl_fmif_input_stage_routine")
void default_ctl_fmif_input_stage_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_core_stage_routine=default_ctl_fmif_core_stage_routine")
void default_ctl_fmif_core_stage_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_output_stage_routine=default_ctl_fmif_output_stage_routine")
void default_ctl_fmif_output_stage_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_request_stage_routine=default_ctl_fmif_request_stage_routine")
void default_ctl_fmif_request_stage_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

// ....................................................................//
// The following functions may running in Main Loop

#pragma comment(linker, "/alternatename:ctl_fmif_monitor_routine=default_ctl_fmif_monitor_routine")
void default_ctl_fmif_monitor_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_security_routine=default_ctl_fmif_security_routine")
fast_gt default_ctl_fmif_security_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return GMP_EC_OK;
}

// return value:
// 1 change to next progress
// 0 keep the same state
#pragma comment(linker, "/alternatename:ctl_fmif_sm_pending_routine=default_ctl_fmif_sm_pending_routine")
fast_gt default_ctl_fmif_sm_pending_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// return value:
// 1 change to next progress
// 0 keep the same state
#pragma comment(linker, "/alternatename:ctl_fmif_sm_calibrate_routine=default_ctl_fmif_sm_calibrate_routine")
fast_gt default_ctl_fmif_sm_calibrate_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 1;
}

#pragma comment(linker, "/alternatename:ctl_fmif_sm_ready_routine=default_ctl_fmif_sm_ready_routine")
fast_gt default_ctl_fmif_sm_ready_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 0;
}

// Main relay close, power on the main circuit
#pragma comment(linker, "/alternatename:ctl_fmif_sm_runup_routine=default_ctl_fmif_sm_runup_routine")
fast_gt default_ctl_fmif_sm_runup_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
    return 1;
}

#pragma comment(linker, "/alternatename:ctl_fmif_sm_online_routine=default_ctl_fmif_sm_online_routine")
void default_ctl_fmif_sm_online_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_sm_fault_routine=default_ctl_fmif_sm_fault_routine")
void default_ctl_fmif_sm_fault_routine(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

// ....................................................................//
// The following functions may called in Main ISR and Main Loop

#pragma comment(linker, "/alternatename:ctl_fmif_output_enable=default_ctl_fmif_output_enable")
void default_ctl_fmif_output_enable(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#pragma comment(linker, "/alternatename:ctl_fmif_output_disable=default_ctl_fmif_output_disable")
void default_ctl_fmif_output_disable(ctl_object_nano_t *pctl_obj)
{
    // not implement
}

#elif defined __TI_COMPILER_VERSION__
// Unsupport weak function

#else // Other Compiler support WEAK function

// ....................................................................//
// The following functions may running in Main ISR

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_input_stage_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_core_stage_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_output_stage_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_request_stage_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

// ....................................................................//
// The following functions may running in Main Loop

GMP_WEAK_FUNC_PREFIX
void ctl_fmif_monitor_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
}

GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_security_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return GMP_EC_OK;
}

// return value:
// 1 change to next progress
// 0 keep the same state
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_pending_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 0;
}

// return value:
// 1 change to next progress
// 0 keep the same state
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_calibrate_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 1;
}

GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_ready_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 0;
}

// Main relay close, power on the main circuit
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_runup_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 1;
}

GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_online_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 0;
}

GMP_WEAK_FUNC_PREFIX
fast_gt ctl_fmif_sm_fault_routine(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
{
    // not implement
    return 0;
}

// ....................................................................//
// The following functions may called in Main ISR and Main Loop

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_output_enable(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

// GMP_WEAK_FUNC_PREFIX
// void ctl_fmif_output_disable(ctl_object_nano_t *pctl_obj) GMP_WEAK_FUNC_SUFFIX
//{
//     // not implement
// }

#endif // _MSC_VER

//////////////////////////////////////////////////////////////////////////
// Kernal functions

// Controller state machine
// This function should be called in your controller main loop routine.
// This function should implement all the controller state machine transfer.
void ctl_fm_state_dispatch(ctl_object_nano_t *pctl_obj)
{
    // All the real-time controller state machine should implement here.
    pctl_obj->mainloop_tick = pctl_obj->mainloop_tick + 1;

    // Security ensure
    if (pctl_obj->switch_security_routine && pctl_obj->state_machine > CTL_SM_CALIBRATE)
        ctl_fmif_security_routine(pctl_obj);

    // State machine
    switch (pctl_obj->state_machine)
    {
        // Wait user to enable the controller.
    case CTL_SM_PENDING:
        // close output
        ctl_fmif_output_disable(pctl_obj);

        // call pending State machine routine
        if (ctl_fmif_sm_pending_routine(pctl_obj))
        {
            // need to change to the next state
            pctl_obj->state_machine = CTL_SM_CALIBRATE;
        }
        break;

        // The following 3 states, user should define a state machine
        // to determine the output is enable or disable
    case CTL_SM_CALIBRATE:

        // User may define if output is enabler or disable

        // call State machine routine
        if (pctl_obj->switch_calibrate_stage) // enable the calibrate stage
        {
            if (ctl_fmif_sm_calibrate_routine(pctl_obj))
            {
                // need to change to the next state
                pctl_obj->state_machine = CTL_SM_READY;
            }
        }
        else
        {
            // need to change to the next state
            pctl_obj->state_machine = CTL_SM_READY;
        }

        break;
    case CTL_SM_READY:
        // output is disabled
        ctl_fmif_output_disable(pctl_obj);

        // call State machine routine
        if (ctl_fmif_sm_ready_routine(pctl_obj))
        {
            pctl_obj->state_machine = CTL_SM_RUNUP;
        }

        break;
    case CTL_SM_RUNUP:

        // User may define if output is enabler or disable

        // call State machine routine
        if (ctl_fmif_sm_runup_routine(pctl_obj))
        {
            pctl_obj->state_machine = CTL_SM_ONLINE;
        }

        break;

        // The following state mast ensure output is enable,
        // And every thing is ready.
    case CTL_SM_ONLINE:
        // call State machine routine
        ctl_fmif_sm_online_routine(pctl_obj);

        ctl_fmif_output_enable(pctl_obj);
        break;

        // Fault -> Close Output right now.
    case CTL_SM_FAULT:
        ctl_fmif_output_disable(pctl_obj);

        // call State machine routine
        ctl_fmif_sm_fault_routine(pctl_obj);
        break;

        // Error State Machine -> close PWM.
        // and transfer to Fault state.
    default:
        // Error State Machine occurred:
        ctl_fmif_output_disable(pctl_obj);
        pctl_obj->state_machine = CTL_SM_FAULT;
        break;
    }

    // Monitor module caller
    if (ctl_step_divider(&pctl_obj->div_monitor))
    {
        ctl_fmif_monitor_routine(pctl_obj);
    }
}

// Security Check routine
// This function should be called in your controller initialization routine.
// This function may check all the controller parameters.
uint32_t ctl_fm_controller_inspection(ctl_object_nano_t *pctl_obj)
{
    if (ctl_nano_handle == NULL)
    {
        // fatal error default controller nano handle has not specified
        gmp_base_print("Error: User must specify the default controller nano header.\r\n");

        while (1)
            ;
    }

    return GMP_EC_OK;
}

//////////////////////////////////////////////////////////////////////////
// Other functions

//// init controller object
//// User should call this function in init process
//// BEFORE all the other control object is inited.
// void ctl_fm_init_nano_header(ctl_object_nano_t *ctl_obj)
//{
//     ctl_obj->isr_tick = 0;
//
//     ctl_obj->state_machine = CTL_SM_PENDING;
//     //	ctl_obj->state_machine = CTL_SM_READY;
//     ctl_obj->switch_calibrate_stage = 1;
//     ctl_obj->switch_runup_stage = 0;
//     ctl_obj->switch_security_routine = 1;
//
//     // finally set the endorsement
//     ctl_obj->security_endorse = GMP_CTL_ENDORSE;
//
//     ctl_obj->control_law_CPU_usage_tick = 0;
//     ctl_obj->mainloop_CPU_usage_tick = 0;
//
//     // Monitor divider
//     ctl_init_divider(&ctl_obj->div_monitor);
//     ctl_setup_divider(&ctl_obj->div_monitor, 20);
//
//     return;
// }
//
// void ctl_fm_setup_nano_header(ctl_object_nano_t *ctl_obj,
//                               uint32_t ctrl_freq // the frequency of the control law, unit Hz
//)
//{
//     ctl_obj->ctrl_freq = ctrl_freq;
//
//     return;
// }

// init controller object
// User should call this function in init process
// BEFORE all the other control object is inited.
void ctl_fm_init_nano_header(
    // controller nano object
    ctl_object_nano_t *ctl_obj,
    // the frequency of the control law, unit Hz
    uint32_t ctrl_freq)
{
    ctl_obj->isr_tick = 0;

    ctl_obj->state_machine = CTL_SM_PENDING;
    //	ctl_obj->state_machine = CTL_SM_READY;
    ctl_obj->switch_calibrate_stage = 1;
    ctl_obj->switch_runup_stage = 0;
    ctl_obj->switch_security_routine = 1;

    // finally set the endorsement
    ctl_obj->security_endorse = GMP_CTL_ENDORSE;

    ctl_obj->control_law_CPU_usage_tick = 0;
    ctl_obj->mainloop_CPU_usage_tick = 0;

    // Monitor divider
    ctl_init_divider(&ctl_obj->div_monitor, 20);

    ctl_obj->ctrl_freq = ctrl_freq;

    return;
}

void ctl_sm_nano_trnasfer(ctl_object_nano_t *ctl_obj, ctl_nano_state_machine target_state)
{
}

void ctl_fm_force_online(ctl_object_nano_t *ctl_obj)
{
    ctl_obj->state_machine = CTL_SM_ONLINE;
}

void ctl_fm_force_calibrate(ctl_object_nano_t *ctl_obj)
{
    ctl_obj->state_machine = CTL_SM_CALIBRATE;
}

ec_gt ctl_setup_default_ctl_nano_obj(ctl_object_nano_t *ctl_obj)
{
    ctl_nano_handle = ctl_obj;
    return GMP_EC_OK;
}

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
