/**
 * @file boost.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a generic cascaded PID controller for a Boost converter.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details
 * # Boost Controller Usage Guide
 *
 * This module implements a standard dual-loop (voltage and current) controller
 * for a DC-DC Boost converter.
 *
 * ## 1. Initialization
 * - Call @ref ctl_init_boost_ctrl to configure the PID parameters for both the
 * outer voltage loop and the inner current loop.
 * - Call @ref ctl_attach_boost_ctrl_input to link the controller to the ADC
 * interfaces for sensing input/output voltages and inductor current.
 *
 * ## 2. Mode Selection
 * The controller supports several operating modes:
 * - **Voltage Mode**: Closed-loop voltage control. Use @ref ctl_boost_ctrl_voltage_mode
 * to enter and @ref ctl_set_boost_ctrl_voltage to set the target output voltage.
 * - **Current Mode**: Closed-loop current control (voltage loop is bypassed).
 * Use @ref ctl_boost_ctrl_current_mode to enter and @ref ctl_set_boost_ctrl_current
 * to set the target inductor current.
 * - **Open Loop Mode**: Directly control the output voltage command. Use
 * @ref ctl_boost_ctrl_openloop_mode to enter and @ref ctl_set_boost_ctrl_voltage_openloop
 * to set the voltage command.
 *
 * ## 3. Execution
 * - Call @ref ctl_step_boost_ctrl periodically in a timer ISR to execute one
 * step of the control logic.
 * - Call @ref ctl_get_boost_ctrl_modulation to retrieve the calculated PWM duty cycle
 * and apply it to your PWM peripheral.
 *
 * ## 4. Control
 * - Use @ref ctl_enable_boost_ctrl and @ref ctl_disable_boost_ctrl for master control.
 * - Use @ref ctl_clear_boost_ctrl to reset PID integrators before enabling.
 */

#ifndef _FILE_BOOST_CTRL_H_
#define _FILE_BOOST_CTRL_H_

#include <ctl/component/interface/interface_base.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup boost_controller_api Boost Converter Controller API
 * @brief Contains all configurations, data structures, and functions for the Boost controller.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Configuration Macros                             */
/*---------------------------------------------------------------------------*/

// #define CTL_BOOST_CTRL_OUTPUT_WITHOUT_UOUT // You may enable this macro to enter debug mode where duty cycle is not divided by U_in.

/**
 * @brief Defines the minimum input voltage to avoid division by zero during duty cycle calculation.
 */
#ifndef CTL_BOOST_CTRL_UIN_MIN
#define CTL_BOOST_CTRL_UIN_MIN ((float2ctrl(0.01)))
#endif // CTL_BOOST_CTRL_UIN_MIN

#define UPPER_BRIDGE (0) /**< Identifier for controlling the upper-side switch. */
#define LOWER_BRIDGE (1) /**< Identifier for controlling the lower-side switch. */

/**
 * @brief Defines which switch in the half-bridge is being controlled by the PWM signal.
 * @details
 * - `UPPER_BRIDGE`: The duty cycle `D` controls the upper switch. Transfer function M = 1/D.
 * - `LOWER_BRIDGE`: The duty cycle `D` controls the lower switch. Transfer function M = 1/(1-D).
 */
#ifndef CTL_BOOST_CTRL_POSITION
#define CTL_BOOST_CTRL_POSITION UPPER_BRIDGE
#endif // CTL_BOOST_CTRL_POSITION

/*---------------------------------------------------------------------------*/
/* Data Structures                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Core data structure for the Boost controller.
 */
typedef struct _tag_boost_ctrl_type
{
    /*-- Controller Interfaces --*/
    adc_ift* adc_uo; /**< ADC interface for output capacitor voltage. */
    adc_ift* adc_il; /**< ADC interface for inductor current. */
    adc_ift* adc_ui; /**< ADC interface for input voltage. */

    /*-- Controller Output --*/
    ctrl_gt pwm_out_pu; /**< Final calculated PWM duty cycle (per-unit). */

    /*-- Controller Objects --*/
    pid_regular_t current_pid; /**< Inner PID controller for the inductor current loop. */
    pid_regular_t voltage_pid; /**< Outer PID controller for the output voltage loop. */

    /*-- Input Signal Filters --*/
    ctl_low_pass_filter_t lpf_ui; /**< Low-pass filter for the input voltage measurement. */
    ctl_low_pass_filter_t lpf_uo; /**< Low-pass filter for the output voltage measurement. */
    ctl_low_pass_filter_t lpf_il; /**< Low-pass filter for the inductor current measurement. */

    /*-- Saturation Objects --*/
    ctl_saturation_t modulation_saturation; /**< Saturation block to limit the intermediate voltage command. */
    ctrl_gt vo_sat;                         /**< Saturated intermediate voltage command. */

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

} boost_ctrl_t;

/*---------------------------------------------------------------------------*/
/* Function Prototypes                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the Boost controller with specified PID and filter parameters.
 * @param[out] boost Pointer to the Boost controller instance to be initialized.
 * @param[in] v_kp Proportional gain for the voltage PID controller.
 * @param[in] v_Ti Integral time constant for the voltage PID controller.
 * @param[in] v_Td Derivative time constant for the voltage PID controller.
 * @param[in] i_kp Proportional gain for the current PID controller.
 * @param[in] i_Ti Integral time constant for the current PID controller.
 * @param[in] i_Td Derivative time constant for the current PID controller.
 * @param[in] vo_min Minimum output voltage for saturation, to prevent division by zero.
 * @param[in] vo_max Maximum output voltage for saturation.
 * @param[in] fc Cutoff frequency for the low-pass filters on sensor inputs.
 * @param[in] fs Controller execution frequency in Hz.
 */
void ctl_init_boost_ctrl(boost_ctrl_t* boost, parameter_gt v_kp, parameter_gt v_Ti, parameter_gt v_Td,
                         parameter_gt i_kp, parameter_gt i_Ti, parameter_gt i_Td, parameter_gt vo_min,
                         parameter_gt vo_max, parameter_gt fc, parameter_gt fs);

/**
 * @brief Attaches the controller to the physical ADC input interfaces.
 * @param[out] boost Pointer to the Boost controller instance.
 * @param[in] uc Pointer to the ADC interface for the output capacitor voltage.
 * @param[in] il Pointer to the ADC interface for the inductor current.
 * @param[in] uin Pointer to the ADC interface for the input voltage.
 */
void ctl_attach_boost_ctrl_input(boost_ctrl_t* boost, adc_ift* uc, adc_ift* il, adc_ift* uin);

/*---------------------------------------------------------------------------*/
/* Inline Functions                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Executes one step of the Boost control logic.
 * @details This function performs the cascaded voltage and current control loop calculations.
 * It should be called periodically at the frequency specified during initialization.
 * @param[in,out] boost Pointer to the Boost controller instance.
 * @return The calculated PWM duty cycle (per-unit).
 */
GMP_STATIC_INLINE
ctrl_gt ctl_step_boost_ctrl(boost_ctrl_t* boost)
{
    // Filter the raw ADC inputs
    ctl_step_lowpass_filter(&boost->lpf_il, boost->adc_il->value);
    ctl_step_lowpass_filter(&boost->lpf_uo, boost->adc_uo->value);
    ctl_step_lowpass_filter(&boost->lpf_ui, boost->adc_ui->value);

    if (boost->flag_enable_system)
    {
        // Outer voltage loop
        if (boost->flag_enable_voltage_ctrl)
        {
            boost->current_set =
                ctl_step_pid_ser(&boost->voltage_pid, boost->voltage_set - boost->lpf_uo.out) + boost->current_ff;
        }

        // Inner current loop
        if (boost->flag_enable_current_ctrl)
        {
            boost->voltage_out =
                ctl_step_pid_ser(&boost->current_pid, boost->current_set - boost->lpf_il.out) + boost->voltage_ff;
        }

#ifdef CTL_BOOST_CTRL_OUTPUT_WITHOUT_UOUT
        // Duty cycle calculation without input voltage feedforward (for debugging)
#if CTL_BOOST_CTRL_POSITION == LOWER_BRIDGE
        boost->pwm_out_pu = float2ctrl(1) - boost->voltage_out;
#elif CTL_BOOST_CTRL_POSITION == UPPER_BRIDGE
        boost->pwm_out_pu = boost->voltage_out;
#endif // CTL_BOOST_CTRL_POSITION
#else  // CTL_BOOST_CTRL_OUTPUT_WITHOUT_UOUT                                                                           \
       // Standard duty cycle calculation with input voltage feedforward
        boost->vo_sat = ctl_step_saturation(&boost->modulation_saturation, boost->voltage_out);
#if CTL_BOOST_CTRL_POSITION == UPPER_BRIDGE
        // D = U_in / U_out_cmd
        boost->pwm_out_pu = ctl_div(boost->lpf_ui.out, boost->vo_sat);
#elif CTL_BOOST_CTRL_POSITION == LOWER_BRIDGE
        // D = 1 - (U_in / U_out_cmd)
        boost->pwm_out_pu = GMP_CONST_1 - ctl_div(boost->lpf_ui.out, boost->vo_sat);
#endif // CTL_BOOST_CTRL_POSITION
#endif // CTL_BOOST_CTRL_OUTPUT_WITHOUT_UOUT
    }
    else
    {
        // For safety, ensure the Boost switch is conductive by default when disabled.
        boost->pwm_out_pu = float2ctrl(1);
    }

    return boost->pwm_out_pu;
}

/**
 * @brief Gets the last calculated PWM duty cycle.
 * @param[in] boost Pointer to the Boost controller instance.
 * @return The last calculated PWM duty cycle (per-unit).
 */
GMP_STATIC_INLINE
ctrl_gt ctl_get_boost_ctrl_modulation(boost_ctrl_t* boost)
{
    return boost->pwm_out_pu;
}

/**
 * @brief Clears the internal states and integral terms of the PID controllers.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_clear_boost_ctrl(boost_ctrl_t* boost)
{
    ctl_clear_pid(&boost->voltage_pid);
    ctl_clear_pid(&boost->current_pid);

    boost->current_ff = 0;
    boost->voltage_ff = 0;
}

/**
 * @brief Switches the controller to closed-loop Voltage mode.
 * @details Both the inner current loop and outer voltage loop are enabled.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_boost_ctrl_voltage_mode(boost_ctrl_t* boost)
{
    boost->flag_enable_system = 1;
    boost->flag_enable_current_ctrl = 1;
    boost->flag_enable_voltage_ctrl = 1;
}

/**
 * @brief Sets the target output voltage in Voltage mode.
 * @param[in,out] boost Pointer to the Boost controller instance.
 * @param[in] v_set The target output voltage.
 */
GMP_STATIC_INLINE
void ctl_set_boost_ctrl_voltage(boost_ctrl_t* boost, ctrl_gt v_set)
{
    boost->voltage_set = v_set;
}

/**
 * @brief Switches the controller to closed-loop Current mode.
 * @details The outer voltage loop is disabled, and the inner current loop directly tracks the user-specified current setpoint.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_boost_ctrl_current_mode(boost_ctrl_t* boost)
{
    boost->flag_enable_system = 1;
    boost->flag_enable_current_ctrl = 1;
    boost->flag_enable_voltage_ctrl = 0;
}

/**
 * @brief Sets the target inductor current in Current mode.
 * @param[in,out] boost Pointer to the Boost controller instance.
 * @param[in] i_set The target inductor current.
 */
GMP_STATIC_INLINE
void ctl_set_boost_ctrl_current(boost_ctrl_t* boost, ctrl_gt i_set)
{
    boost->current_set = i_set;
}

/**
 * @brief Switches the controller to open-loop Voltage mode.
 * @details Both PID loops are disabled. The output is determined directly by the value set with @ref ctl_set_boost_ctrl_voltage_openloop.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_boost_ctrl_openloop_mode(boost_ctrl_t* boost)
{
    boost->flag_enable_system = 1;
    boost->flag_enable_current_ctrl = 0;
    boost->flag_enable_voltage_ctrl = 0;
}

/**
 * @brief Sets the target intermediate voltage command in open-loop mode.
 * @param[in,out] boost Pointer to the Boost controller instance.
 * @param[in] v_set The desired intermediate voltage command.
 */
GMP_STATIC_INLINE
void ctl_set_boost_ctrl_voltage_openloop(boost_ctrl_t* boost, ctrl_gt v_set)
{
    boost->voltage_out = v_set;
}

/**
 * @brief Disables the entire Boost controller system.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_disable_boost_ctrl(boost_ctrl_t* boost)
{
    boost->flag_enable_system = 0;
}

/**
 * @brief Enables the entire Boost controller system.
 * @param[in,out] boost Pointer to the Boost controller instance.
 */
GMP_STATIC_INLINE
void ctl_enable_boost_ctrl(boost_ctrl_t* boost)
{
    boost->flag_enable_system = 1;
}

/** @} */ // end of boost_controller_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BOOST_CTRL_H_
