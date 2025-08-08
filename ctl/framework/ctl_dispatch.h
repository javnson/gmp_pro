/**
 * @file ctl_dispatch.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides the core task dispatcher for the CTL motor control framework.
 * @details This file contains the main entry point function, typically called from a
 * periodic timer interrupt (ISR). It orchestrates the execution of the different
 * stages of the control loop: sensor input, core algorithm execution, and actuator
 * output. The dispatcher is highly configurable via preprocessor macros to support
 * different scheduling strategies.
 *
 * @version 1.0
 * @date 2025-08-07
 *
 * @copyright Copyright GMP(c) 2025
 */

#ifndef _FILE_CTL_DISPATCH_H_
#define _FILE_CTL_DISPATCH_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup CTL_FRAMEWORK_CORE CMP CTL Framework Core
 * @brief The framework of GMP CTL dispatch logic.
 */

/*---------------------------------------------------------------------------*/
/* CTL Framework Dispatcher                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CTL_FRAMEWORK_DISPATCHER CTL Framework Dispatcher
 * @ingroup CTL_FRAMEWORK_CORE
 * @brief The core scheduler for the control loop execution.
 * @{
 */

#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
/**
 * @defgroup CTL_NANO_FRAMEWORK Nano Framework
 * @ingroup CTL_FRAMEWORK_DISPATCHER
 * @brief A lightweight, monolithic scheduling framework.
 * @{
 */

/**
 * @brief Executes the complete control cycle in a single, sequential function call.
 * @details This function is used when the entire control task (input, core, output)
 * is handled within one interrupt context. The execution of each stage can be
 * individually disabled via compile-time macros.
 * @param[in,out] pctl_obj Pointer to the nano framework's control object.
 */
GMP_STATIC_INLINE void ctl_fm_periodic_dispatch(ctl_object_nano_t* pctl_obj)
{
    // Step 0: ISR tick update
    pctl_obj->isr_tick = pctl_obj->isr_tick + 1;

#ifndef GMP_DISABLE_CTL_OBJ_ENDORSE_CHECK
    // Security check to ensure the object handle is valid
    if (pctl_obj->security_endorse == GMP_CTL_ENDORSE)
    {
#endif // GMP_DISABLE_CTL_OBJ_ENDORSE_CHECK

#ifndef GMP_CTL_FRM_NANO_INPUT_STANDALONE
        // Step I: Input stage
        // All analog, digital, and peripheral inputs (e.g., ADC, encoder)
        // should be processed here.
        ctl_fmif_input_stage_routine(pctl_obj);
#endif // GMP_CTL_FRM_NANO_INPUT_STANDALONE

#ifndef GMP_CTL_FRM_NANO_CORE_STANDALONE
        // Step II: Controller core stage
        // All control laws and algorithms should be executed here.
        ctl_fmif_core_stage_routine(pctl_obj);
#endif // GMP_CTL_FRM_NANO_CORE_STANDALONE

#ifndef GMP_CTL_FRM_NANO_OUTPUT_STANDALONE
        // Step III: Output stage
        // All output routines (e.g., PWM, DAC, GPIO) should be implemented here.
        ctl_fmif_output_stage_routine(pctl_obj);
#endif // GMP_CTL_FRM_NANO_OUTPUT_STANDALONE

#ifndef GMP_DISABLE_CTL_OBJ_ENDORSE_CHECK
    }
#endif // GMP_DISABLE_CTL_OBJ_ENDORSE_CHECK

#ifndef GMP_CTL_FRM_NANO_REQUEST_STANDALONE
    // Step IV: Request stage
    // Handles any real-time requests or background tasks.
    ctl_fmif_request_stage_routine(pctl_obj);
#endif // GMP_CTL_FRM_NANO_REQUEST_STANDALONE
}

/** @} */ // end of CTL_NANO_FRAMEWORK group
#endif    // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

/**
 * @brief The main kernel step function for the GMP CTL module.
 * @details This function is the primary entry point intended to be invoked by the
 * user in the main control ISR. It conditionally compiles the appropriate
 * dispatch logic based on the selected framework.
 */
GMP_STATIC_INLINE void gmp_base_ctl_step(void)
{
#ifdef SPECIFY_ENABLE_GMP_CTL
#ifdef SPECIFY_ENABLE_CTL_FRAMEWORK_NANO

#ifndef SPECIFY_CALL_PERIODIC_DISPATCH_MANUALLY
    // Automatically call the nano framework's periodic dispatch function.
    ctl_fm_periodic_dispatch(ctl_nano_handle);
#endif // SPECIFY_CALL_PERIODIC_DISPATCH_MANUALLY

    // Call the user-defined controller implementation
    ctl_dispatch();

#else // Not using Nano Framework, use standard callback-based framework

    // input stage
    ctl_input_callback();

    // call user controller user defined ISR
    ctl_dispatch();

    // output stage
    ctl_output_callback();

#endif // SPECIFY_ENABLE_CTL_FRAMEWORK_NANO
#endif // SPECIFY_ENABLE_GMP_CTL
}

/** @} */ // end of CTL_FRAMEWORK_DISPATCHER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_DISPATCH_H_
