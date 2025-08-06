/**
 * @file ctl_dp_basic.c
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implementation file for basic digital power controller modules.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains the function definitions for initializing and
 * attaching interfaces for various digital power controllers, including Buck,
 * Boost, and protection modules.
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Buck Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/buck.h>

/**
 * @ingroup buck_controller_api
 * @brief Initializes the Buck controller with specified PID and filter parameters.
 * @param[out] buck Pointer to the Buck controller instance.
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
                        parameter_gt fc, parameter_gt fs)
{
    // Ensure controller is disabled on startup
    ctl_disable_buck_ctrl(buck);

    // Initialize low-pass filters for all sensor inputs
    ctl_init_lp_filter(&buck->lpf_il, fs, fc);
    ctl_init_lp_filter(&buck->lpf_ui, fs, fc);
    ctl_init_lp_filter(&buck->lpf_uo, fs, fc);

    // Initialize saturation block for input voltage to prevent division by zero
    ctl_init_saturation(&buck->modulation_saturation, uin_min, uin_max);

    // Initialize PID controllers
    ctl_init_pid_ser(&buck->current_pid, i_kp, i_Ti, i_Td, fs);
    ctl_init_pid_ser(&buck->voltage_pid, v_kp, v_Ti, v_Td, fs);

    // Clear all internal states
    ctl_clear_buck_ctrl(buck);
}

/**
 * @ingroup buck_controller_api
 * @brief Attaches the controller to the physical ADC input interfaces.
 * @param[out] buck Pointer to the Buck controller instance.
 * @param[in] uo Pointer to the ADC interface for the output capacitor voltage.
 * @param[in] il Pointer to the ADC interface for the inductor current.
 * @param[in] uin Pointer to the ADC interface for the input voltage.
 */
void ctl_attach_buck_ctrl_input(buck_ctrl_t* buck, adc_ift* uo, adc_ift* il, adc_ift* uin)
{
    buck->adc_il = il;
    buck->adc_ui = uin;
    buck->adc_uo = uo;
}

//////////////////////////////////////////////////////////////////////////
// Boost Control
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/boost.h>

/**
 * @ingroup boost_controller_api
 * @brief Initializes the Boost controller with specified PID and filter parameters.
 * @param[out] boost Pointer to the Boost controller instance.
 * @param[in] v_kp Proportional gain for the voltage PID controller.
 * @param[in] v_Ti Integral time constant for the voltage PID controller.
 * @param[in] v_Td Derivative time constant for the voltage PID controller.
 * @param[in] i_kp Proportional gain for the current PID controller.
 * @param[in] i_Ti Integral time constant for the current PID controller.
 * @param[in] i_Td Derivative time constant for the current PID controller.
 * @param[in] vo_min Minimum output voltage command for saturation.
 * @param[in] vo_max Maximum output voltage command for saturation.
 * @param[in] fc Cutoff frequency for the low-pass filters on sensor inputs.
 * @param[in] fs Controller execution frequency in Hz.
 */
void ctl_init_boost_ctrl(boost_ctrl_t* boost, parameter_gt v_kp, parameter_gt v_Ti, parameter_gt v_Td,
                         parameter_gt i_kp, parameter_gt i_Ti, parameter_gt i_Td, parameter_gt vo_min,
                         parameter_gt vo_max, parameter_gt fc, parameter_gt fs)
{
    // Ensure controller is disabled on startup
    ctl_disable_boost_ctrl(boost);

    // Initialize PID controllers with correct parameters
    ctl_init_pid_ser(&boost->current_pid, i_kp, i_Ti, i_Td, fs);
    ctl_init_pid_ser(&boost->voltage_pid, v_kp, v_Ti, v_Td, fs);

    // Initialize low-pass filters for all sensor inputs
    ctl_init_lp_filter(&boost->lpf_il, fs, fc);
    ctl_init_lp_filter(&boost->lpf_ui, fs, fc);
    ctl_init_lp_filter(&boost->lpf_uo, fs, fc);

    // Initialize saturation block for output voltage command
    ctl_init_saturation(&boost->modulation_saturation, vo_min, vo_max);

    // Clear all internal states
    ctl_clear_boost_ctrl(boost);
}

/**
 * @ingroup boost_controller_api
 * @brief Attaches the controller to the physical ADC input interfaces.
 * @param[out] boost Pointer to the Boost controller instance.
 * @param[in] uc_port Pointer to the ADC interface for the output capacitor voltage.
 * @param[in] il_port Pointer to the ADC interface for the inductor current.
 * @param[in] uin_port Pointer to the ADC interface for the input voltage.
 */
void ctl_attach_boost_ctrl_input(boost_ctrl_t* boost, adc_ift* uc_port, adc_ift* il_port, adc_ift* uin_port)
{
    boost->adc_uo = uc_port;
    boost->adc_il = il_port;
    boost->adc_ui = uin_port;
}

//////////////////////////////////////////////////////////////////////////
// Protection Strategy
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/digital_power/basic/protectoion_strategy.h>

/**
 * @ingroup protection_strategy_api
 * @brief Attaches the protection module to the physical ADC input interfaces.
 * @param[out] obj Pointer to the VIP protection instance.
 * @param[in] uo Pointer to the ADC interface for the output voltage.
 * @param[in] io Pointer to the ADC interface for the output current.
 */
void ctl_attach_vip_protection(std_vip_protection_t* obj, adc_ift* uo, adc_ift* io)
{
    obj->adc_io = io;
    obj->adc_uo = uo;
}

/**
 * @ingroup protection_strategy_api
 * @brief Initializes the VIP protection module.
 * @param[out] obj Pointer to the VIP protection instance.
 * @param[in] power_f_cut Cutoff frequency for the power measurement filter.
 * @param[in] voltage_f_cut Cutoff frequency for the voltage measurement filter.
 * @param[in] current_f_cut Cutoff frequency for the current measurement filter.
 * @param[in] v_max Maximum voltage limit in physical units (e.g., Volts).
 * @param[in] v_base Base voltage for per-unit conversion.
 * @param[in] i_max Maximum current limit in physical units (e.g., Amps).
 * @param[in] i_base Base current for per-unit conversion.
 * @param[in] p_max Maximum power limit in physical units (e.g., Watts).
 * @param[in] fs Sampling frequency of the controller in Hz.
 */
void ctl_init_vip_protection(std_vip_protection_t* obj, parameter_gt power_f_cut, parameter_gt voltage_f_cut,
                             parameter_gt current_f_cut, parameter_gt v_max, parameter_gt v_base, parameter_gt i_max,
                             parameter_gt i_base, parameter_gt p_max, parameter_gt fs)
{
    // Set protection thresholds in per-unit values
    obj->voltage_max = float2ctrl(v_max / v_base);
    obj->current_max = float2ctrl(i_max / i_base);
    obj->power_max = float2ctrl(p_max / v_base / i_base);

    // Initialize low-pass filters for measurements
    ctl_init_lp_filter(&obj->power_filter, fs, power_f_cut);
    ctl_init_lp_filter(&obj->voltage_filter, fs, voltage_f_cut);
    ctl_init_lp_filter(&obj->current_filter, fs, current_f_cut);
}
