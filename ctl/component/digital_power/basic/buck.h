/**
 * @file buck.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a generic cascaded PID controller for a Buck converter.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _FILE_BUCK_CTRL_H_
#define _FILE_BUCK_CTRL_H_

#include <ctl/component/interface/interface_base.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup buck_controller_api Buck Converter Controller API
 * @brief Contains all configurations, data structures, and functions for the Buck controller.
 * @details
 * # Buck Controller Usage Guide
 *
 * This module implements a standard dual-loop (voltage and current) controller
 * for a DC-DC Buck converter.
 *
 * ## 1. Initialization
 * - Call @ref ctl_init_buck_ctrl to configure the PID parameters for both the
 * outer voltage loop and the inner current loop.
 * - Call @ref ctl_attach_buck_ctrl_input to link the controller to the ADC
 * interfaces for sensing input/output voltages and inductor current.
 *
 * ## 2. Mode Selection
 * The controller supports several operating modes:
 * - **Voltage Mode**: Closed-loop voltage control. Use @ref ctl_buck_ctrl_voltage_mode
 * to enter and @ref ctl_set_buck_ctrl_voltage to set the target output voltage.
 * - **Current Mode**: Closed-loop current control (voltage loop is bypassed).
 * Use @ref ctl_buck_ctrl_current_mode to enter and @ref ctl_set_buck_ctrl_current
 * to set the target inductor current.
 * - **Open Loop Mode**: Directly control the output voltage command. Use
 * @ref ctl_buck_ctrl_openloop_mode to enter and @ref ctl_set_buck_ctrl_voltage_openloop
 * to set the voltage command.
 *
 * ## 3. Execution
 * - Call @ref ctl_step_buck_ctrl periodically in a timer ISR to execute one
 * step of the control logic.
 * - Call @ref ctl_get_buck_ctrl_modulation to retrieve the calculated PWM duty cycle
 * and apply it to your PWM peripheral.
 *
 * ## 4. Control
 * - Use @ref ctl_enable_buck_ctrl and @ref ctl_disable_buck_ctrl for master control.
 * - Use @ref ctl_clear_buck_ctrl to reset PID integrators before enabling.
 * @{
 * @ingroup CTL_DP_LIB
 */

/*---------------------------------------------------------------------------*/
/* Configuration Macros                             */
/*---------------------------------------------------------------------------*/

#define CTL_BUCK_CTRL_OUTPUT_WITHOUT_UIN // You may enable this macro to enter debug mode where duty cycle is not divided by U_in.

/*---------------------------------------------------------------------------*/
/* Data Structures                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Core data structure for the Buck controller.
 */
typedef struct _tag_buck_ctrl_type
{
    /*-- Controller Interfaces --*/
    adc_ift* adc_uo; /**< ADC interface for output capacitor voltage. */
    adc_ift* adc_il; /**< ADC interface for inductor current. */
    adc_ift* adc_ui; /**< ADC interface for input voltage. */

    /*-- Controller Output --*/
    ctrl_gt pwm_out_pu; /**< Final calculated PWM duty cycle (per-unit). */

    /*-- Controller Objects --*/
    ctl_pid_t current_pid; /**< Inner PID controller for the inductor current loop. */
    ctl_pid_t voltage_pid; /**< Outer PID controller for the output voltage loop. */

    /*-- Input Signal Filters --*/
    ctl_low_pass_filter_t lpf_ui; /**< Low-pass filter for the input voltage measurement. */
    ctl_low_pass_filter_t lpf_uo; /**< Low-pass filter for the output voltage measurement. */
    ctl_low_pass_filter_t lpf_il; /**< Low-pass filter for the inductor current measurement. */

    /*-- Saturation Objects --*/
    ctl_saturation_t modulation_saturation; /**< Saturation block to limit the input voltage value. */
    ctrl_gt ui_sat;                         /**< Saturated input voltage value. */

    /*-- Feed-forward Terms --*/
    ctrl_gt current_ff; /**< Feed-forward term for the current loop. */
    ctrl_gt voltage_ff; /**< Feed-forward term for the voltage loop (output of current PID). */

    /*-- Intermediate Variables --*/
    ctrl_gt current_set; /**< Target value for the inner current loop. */
    ctrl_gt voltage_set; /**< Target value for the outer voltage loop. */
    ctrl_gt voltage_out; /**< Intermediate voltage command from the current loop. */

    /*-- Control Flags --*/
    fast_gt flag_enable_system;       /**< Master enable flag for the entire controller. */
    fast_gt flag_enable_current_ctrl; /**< Enable flag for the inner current loop. */
    fast_gt flag_enable_voltage_ctrl; /**< Enable flag for the outer voltage loop. */

} buck_ctrl_t;

/*---------------------------------------------------------------------------*/
/* Function Prototypes                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the Buck controller with specified PID and filter parameters.
 * @param[out] buck Pointer to the Buck controller instance to be initialized.
 * @param[in] v_kp Proportional gain for the voltage PID controller.
 * @param[in] v_Ti Integral time constant for the voltage PID controller.
 * @param[in] v_Td Derivative time constant for the voltage PID controller.
 * @param[in] i_kp Proportional gain for the current PID controller.
 * @param[in] i_Ti Integral time constant for the current PID controller.
 * @param[in] i_Td Derivative time constant for the current PID controller.
 * @param[in] uin_min Minimum input voltage for saturation, to prevent division by zero.
 * @param[in] uin_max Maximum input voltage for saturation.
 * @param[in] fc Cutoff frequency for the low-pass filters on sensor inputs.
 * @param[in] fs Controller execution frequency in Hz.
 */
void ctl_init_buck_ctrl(buck_ctrl_t* buck, parameter_gt v_kp, parameter_gt v_Ti, parameter_gt v_Td, parameter_gt i_kp,
                        parameter_gt i_Ti, parameter_gt i_Td, parameter_gt uin_min, parameter_gt uin_max,
                        parameter_gt fc, parameter_gt fs);

/**
 * @brief Attaches the controller to the physical ADC input interfaces.
 * @param[out] buck Pointer to the Buck controller instance.
 * @param[in] uo Pointer to the ADC interface for the output capacitor voltage.
 * @param[in] il Pointer to the ADC interface for the inductor current.
 * @param[in] uin Pointer to the ADC interface for the input voltage.
 */
void ctl_attach_buck_ctrl_input(buck_ctrl_t* buck, adc_ift* uo, adc_ift* il, adc_ift* uin);

/*---------------------------------------------------------------------------*/
/* Inline Functions                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Clears the internal states and integral terms of the PID controllers.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_buck_ctrl(buck_ctrl_t* buck)
{
    ctl_clear_pid(&buck->current_pid);
    ctl_clear_pid(&buck->voltage_pid);

    buck->current_ff = 0;
    buck->voltage_ff = 0;
}

/**
 * @brief Executes one step of the Buck control logic.
 * @details This function performs the cascaded voltage and current control loop calculations.
 * It should be called periodically at the frequency specified during initialization.
 * @param[in,out] buck Pointer to the Buck controller instance.
 * @return The calculated PWM duty cycle (per-unit).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_buck_ctrl(buck_ctrl_t* buck)
{
    // Filter the raw ADC inputs
    ctl_step_lowpass_filter(&buck->lpf_il, buck->adc_il->value);
    ctl_step_lowpass_filter(&buck->lpf_uo, buck->adc_uo->value);
    ctl_step_lowpass_filter(&buck->lpf_ui, buck->adc_ui->value);

    if (buck->flag_enable_system)
    {
        // Outer voltage loop
        if (buck->flag_enable_voltage_ctrl)
        {
            buck->current_set =
                ctl_step_pid_ser(&buck->voltage_pid, buck->voltage_set - buck->lpf_uo.out) + buck->current_ff;
        }

        // Inner current loop
        if (buck->flag_enable_current_ctrl)
        {
            buck->voltage_out =
                ctl_step_pid_ser(&buck->current_pid, buck->current_set - buck->lpf_il.out) + buck->voltage_ff;
        }

#ifdef CTL_BUCK_CTRL_OUTPUT_WITHOUT_UIN
        // Duty cycle calculation without input voltage feedforward (for debugging)
        buck->pwm_out_pu = buck->voltage_out;
#else
        // Standard duty cycle calculation: D = U_out_cmd / U_in
        buck->ui_sat = ctl_step_saturation(&buck->modulation_saturation, buck->lpf_ui.out);
        buck->pwm_out_pu = ctl_div(buck->voltage_out, buck->ui_sat);

#endif // CTL_BUCK_CTRL_OUTPUT_WITHOUT_UIN
    }
    else
    {
        // For safety, ensure the Buck switch is off by default when disabled.
        buck->pwm_out_pu = float2ctrl(0);
    }

    return buck->pwm_out_pu;
}

/**
 * @brief Switches the controller to open-loop Voltage mode.
 * @details Both PID loops are disabled. The output is determined directly by the value set with @ref ctl_set_buck_ctrl_voltage_openloop.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_buck_ctrl_openloop_mode(buck_ctrl_t* buck)
{
    //buck->flag_enable_system = 1;
    buck->flag_enable_current_ctrl = 0;
    buck->flag_enable_voltage_ctrl = 0;
}

/**
 * @brief Sets the target intermediate voltage command in open-loop mode.
 * @param[in,out] buck Pointer to the Buck controller instance.
 * @param[in] vo The desired intermediate voltage command.
 */
GMP_STATIC_INLINE void ctl_set_buck_ctrl_voltage_openloop(buck_ctrl_t* buck, ctrl_gt vo)
{
    buck->voltage_out = vo;
}

/**
 * @brief Switches the controller to closed-loop Current mode.
 * @details The outer voltage loop is disabled, and the inner current loop directly tracks the user-specified current setpoint.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_buck_ctrl_current_mode(buck_ctrl_t* buck)
{
    //buck->flag_enable_system = 0;
    buck->flag_enable_current_ctrl = 1;
    buck->flag_enable_voltage_ctrl = 0;
}

/**
 * @brief Sets the target inductor current in Current mode.
 * @param[in,out] buck Pointer to the Buck controller instance.
 * @param[in] il The target inductor current.
 */
GMP_STATIC_INLINE void ctl_set_buck_ctrl_current(buck_ctrl_t* buck, ctrl_gt il)
{
    buck->current_set = il;
}

/**
 * @brief Switches the controller to closed-loop Voltage mode.
 * @details Both the inner current loop and outer voltage loop are enabled.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_buck_ctrl_voltage_mode(buck_ctrl_t* buck)
{
    //buck->flag_enable_system = 1;
    buck->flag_enable_current_ctrl = 1;
    buck->flag_enable_voltage_ctrl = 1;
}

/**
 * @brief Sets the target output voltage in Voltage mode.
 * @param[in,out] buck Pointer to the Buck controller instance.
 * @param[in] uo The target output voltage.
 */
GMP_STATIC_INLINE void ctl_set_buck_ctrl_voltage(buck_ctrl_t* buck, ctrl_gt uo)
{
    buck->voltage_set = uo;
}

/**
 * @brief Enables the entire Buck controller system.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_enable_buck_ctrl(buck_ctrl_t* buck)
{
    buck->flag_enable_system = 1;
}

/**
 * @brief Disables the entire Buck controller system.
 * @param[in,out] buck Pointer to the Buck controller instance.
 */
GMP_STATIC_INLINE void ctl_disable_buck_ctrl(buck_ctrl_t* buck)
{
    buck->flag_enable_system = 0;
}

/**
 * @brief Gets the last calculated PWM duty cycle.
 * @param[in] buck Pointer to the Buck controller instance.
 * @return The last calculated PWM duty cycle (per-unit).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_buck_ctrl_modulation(buck_ctrl_t* buck)
{
    return buck->pwm_out_pu;
}

/** @} */ // end of buck_controller_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BUCK_CTRL_H_
