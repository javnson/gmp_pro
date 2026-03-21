/**
 * @file ctl_motor_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// const f module

#include <ctl/component/motor_control/basic/vf_generator.h>

void ctl_init_const_f_controller(ctl_const_f_controller* ctrl, parameter_gt frequency, parameter_gt isr_freq)
{
    // ctl_setup_ramp_gen(&ctrl->rg, float2ctrl(frequency / isr_freq), 1, 0);

    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, frequency, 1, 0);
}

// Const slope Frequency module

void ctl_init_const_slope_f_controller(
    // controller object
    ctl_slope_f_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);
}

// change target frequency
void ctl_set_slope_f_freq(
    // Const VF controller
    ctl_slope_f_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}

/**
 * @brief Initializes the Constant Slope Frequency Controller (PU).
 * @param[out] ctrl Pointer to the ctl_slope_f_pu_controller object.
 * @param[in] frequency The initial target frequency in Hz.
 * @param[in] freq_slope The maximum rate of frequency change in Hz/s.
 * @param[in] rated_krpm The motor rated speed in krpm (Base value source).
 * @param[in] pole_pairs The motor pole pairs (Base value source).
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_init_const_slope_f_pu_controller(ctl_slope_f_pu_controller* ctrl, parameter_gt frequency,
                                          parameter_gt freq_slope, parameter_gt rated_krpm, parameter_gt pole_pairs,
                                          parameter_gt isr_freq)
{
    // 1. Calculate Base Frequency (Rated Electrical Hz)
    // f_base = (RPM * Poles) / 60
    // rated_krpm is in 1000 RPM
    parameter_gt base_freq_hz = (rated_krpm * 1000.0f * pole_pairs) / 60.0f;

    // Prevent divide by zero if user inputs bad data
    if (base_freq_hz < 0.1f)
        base_freq_hz = 1.0f;

    // 2. Clear Positions
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // 3. Init Ramp Generator
    // The slope will be updated in the step function, init with 0.
    // Range is [0, 1) for electrical angle.
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    // 4. Calculate Conversion Ratio
    // This ratio converts "1.0 pu frequency" into "step size per ISR tick"
    // step = (f_base / f_isr)
    ctrl->ratio_freq_pu_to_step = float2ctrl(base_freq_hz / isr_freq);

    // 5. Initialize Target Frequency in PU
    // target_pu = target_hz / base_hz
    ctrl->target_freq_pu = float2ctrl(frequency / base_freq_hz);

    // 6. Initialize Slope Limiter
    // The limiter needs to limit the change of PU per Tick.
    // Max Change (Hz/s) = freq_slope
    // Max Change (PU/s) = freq_slope / base_freq_hz
    ctrl_gt slope_limit_per_tick = float2ctrl(freq_slope / base_freq_hz);

    ctl_init_slope_limiter(&ctrl->freq_slope, slope_limit_per_tick, -slope_limit_per_tick, isr_freq);

    // Set initial output of limiter to current target (assuming instant start if needed, or 0)
    // Usually start from 0 for soft start
    ctrl->freq_slope.out = 0;
    ctrl->current_freq_pu = 0;
}

// VF controller

void ctl_init_const_vf_controller(
    // controller object
    ctl_const_vf_controller* ctrl,
    // target frequency, Hz
    parameter_gt frequency,
    // frequency slope, Hz/s
    parameter_gt freq_slope,
    // voltage range
    ctrl_gt voltage_bound,
    // Voltage Frequency constant
    // unit p.u./Hz, p.u.
    ctrl_gt voltage_over_frequency, ctrl_gt voltage_bias,
    // ISR frequency
    parameter_gt isr_freq)
{
    ctrl->enc.elec_position = 0;
    ctrl->enc.position = 0;

    // init ramp frequency is 0
    ctl_init_ramp_generator_via_freq(&ctrl->rg, isr_freq, 0, 1, 0);

    ctrl->target_frequency = frequency / isr_freq;
    ctrl->target_voltage = 0;

#if !defined CTRL_GT_IS_FIXED
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq);
#elif defined CTRL_GT_IS_FLOAT
    ctrl->v_over_f = float2ctrl(voltage_over_frequency * isr_freq / (2 ^ GLOBAL_Q));
#else
#error("The system does not specify ctrl_gt is float or fixed. You should define CTRL_GT_IS_FLOAT or CTRL_GT_IS_FIXED.")
#endif // CTRL_GT_IS_XXX
    ctrl->v_bias = voltage_bias;

    ctl_init_slope_limiter(&ctrl->freq_slope, float2ctrl(freq_slope / isr_freq), -float2ctrl(freq_slope / isr_freq),
                           isr_freq);

    ctl_init_saturation(&ctrl->volt_sat, voltage_bound, -voltage_bound);
}

// change target frequency
void ctl_set_const_vf_target_freq(
    // Const VF controller
    ctl_const_vf_controller* ctrl,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // Main ISR frequency
    parameter_gt isr_freq)
{
    ctrl->target_frequency = float2ctrl(target_freq / isr_freq);
}
