/**
 * @file vf_generator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides open-loop V/F (Voltage/Frequency) profile generators for motor control.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_CONST_VF_H_
#define _FILE_CONST_VF_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus

/*---------------------------------------------------------------------------*/
/* Constant Frequency Generator                                              */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ConstFGen Constant Frequency Generator
 * @brief A module to generate a constant frequency signal output.
 * @details This file includes modules for generating constant frequency, sloped frequency,
 * and a full constant V/F profile. These are typically used for simple open-loop
 * control of AC motors.
 * This module uses a ramp generator to produce a continuously incrementing
 * angle (position) at a fixed frequency.
 * @{
 */

/**
 * @brief Data structure for the Constant Frequency Controller.
 */
typedef struct _tag_const_f
{
    /** @brief Encoder output interface, provides position information. */
    rotation_ift enc;

    /** @brief Ramp generator to produce the angle signal. */
    ctl_ramp_generator_t rg;

} ctl_const_f_controller;

/**
 * @brief Initializes the Constant Frequency Controller.
 * @param[out] ctrl Pointer to the ctl_const_f_controller object.
 * @param[in] frequency The desired constant frequency in Hz.
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_init_const_f_controller(ctl_const_f_controller* ctrl, parameter_gt frequency, parameter_gt isr_freq);

/**
 * @brief Executes one step of the constant frequency controller.
 * @details This function calculates the next electrical position based on the fixed
 * frequency and should be called periodically within the control ISR.
 * @param[in,out] ctrl Pointer to the ctl_const_f_controller object.
 */
GMP_STATIC_INLINE void ctl_step_const_f_controller(ctl_const_f_controller* ctrl)
{
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;
}

/** @} */ // end of ConstFGen group

/*---------------------------------------------------------------------------*/
/* Slope Frequency Generator                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup SlopeFGen Slope Frequency Generator
 * @ingroup CTL_MC_COMPONENT
 * @brief A module that generates a frequency signal with a controlled slope.
 *
 * This module changes the output frequency from its current value to a
 * target value gradually, following a defined slope (rate of change).
 * @{
 */

/**
 * @brief Data structure for the Slope Frequency Controller.
 */
typedef struct _tag_slope_f
{
    /** @brief Encoder output interface, provides position information. */
    rotation_ift enc;

    /** @brief Input: The target frequency for the generator in Hz. */
    ctrl_gt target_frequency;

    /** @brief Output: The current instantaneous frequency in Hz. */
    ctrl_gt current_freq;

    /** @brief Ramp generator to produce the angle signal. */
    ctl_ramp_generator_t rg;

    /** @brief Slope limiter to control the rate of frequency change. */
    ctl_slope_limiter_t freq_slope;

} ctl_slope_f_controller;

/**
 * @brief Initializes the Constant Slope Frequency Controller.
 * @param[out] ctrl Pointer to the ctl_slope_f_controller object.
 * @param[in] frequency The initial target frequency in Hz.
 * @param[in] freq_slope The maximum rate of frequency change in Hz/s.
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_init_const_slope_f_controller(ctl_slope_f_controller* ctrl, parameter_gt frequency, parameter_gt freq_slope,
                                       parameter_gt isr_freq);

/**
 * @brief Executes one step of the slope frequency controller.
 * @details This function updates the current frequency based on the slope,
 * and then generates the next electrical angle.
 * @param[in,out] ctrl Pointer to the ctl_slope_f_controller object.
 * @return ctrl_gt The new electrical position (angle).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_slope_f(ctl_slope_f_controller* ctrl)
{
    // Step to the next frequency according to the slope limit
    ctrl->current_freq = ctl_step_slope_limiter(&ctrl->freq_slope, ctrl->target_frequency);

    // Update the ramp generator's slope (i.e., its frequency)
    ctl_set_ramp_generator_slope(&ctrl->rg, ctrl->current_freq);

    // Generate the next angle based on the new frequency
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;

    return ctrl->enc.elec_position;
}

/**
 * @brief Resets the internal state of the frequency slope limiter.
 * @param[in,out] ctrl Pointer to the ctl_slope_f_controller object.
 */
GMP_STATIC_INLINE void ctl_clear_slope_f(ctl_slope_f_controller* ctrl)
{
    ctl_clear_slope_limiter(&ctrl->freq_slope);
}

/**
 * @brief Sets a new target frequency for the slope generator.
 * @param[in,out] ctrl Pointer to the ctl_slope_f_controller object.
 * @param[in] target_freq The new target frequency in Hz.
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_set_slope_f_freq(ctl_slope_f_controller* ctrl, parameter_gt target_freq, parameter_gt isr_freq);

/** @} */ // end of SlopeFGen group

/*---------------------------------------------------------------------------*/
/* Constant V/F Profile Generator                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup ConstVFGen Constant V/F Profile Generator
 * @ingroup CTL_MC_COMPONENT
 * @brief A module to generate a constant Volts/Hertz (V/F) profile.
 *
 * This module is used for open-loop speed control of an AC motor. It generates
 * a variable frequency to control speed and a variable voltage that maintains
 * a constant V/F ratio, which helps in keeping the motor's magnetic flux constant.
 * @{
 */

/**
 * @brief Data structure for the Constant V/F Controller.
 */
typedef struct _tag_const_vf
{
    /** @brief Encoder output interface, provides position information. */
    rotation_ift enc;

    /** @brief Input: The target frequency in Hz. */
    ctrl_gt target_frequency;

    /** @brief Output: The calculated target voltage magnitude (p.u.). */
    ctrl_gt target_voltage;

    /** @brief Output: The current instantaneous frequency in Hz. */
    ctrl_gt current_freq;

    /** @brief A small frequency range around zero where the output voltage is forced to zero. */
    ctrl_gt freq_deadband;

    /**
     * @brief The constant V/F ratio (Volts per Hertz).
     * @details This parameter defines the proportional relationship between voltage and frequency.
     * The underlying electromagnetic principle is:
     * @f[ E = 4.44 \cdot N \cdot \Phi \cdot f = V_{ratio} \cdot f @f]
     * where @f( E @f) is the EMF, @f( N @f) is the number of turns, @f( \Phi @f) is the magnetic flux,
     * and @f( f @f) is the frequency.
     */
    ctrl_gt v_over_f;

    /** @brief A voltage boost applied at low frequencies to compensate for stator resistance (IR drop). */
    ctrl_gt v_bias;

    /** @brief Ramp generator to produce the angle signal. */
    ctl_ramp_generator_t rg;

    /** @brief Slope limiter to control the rate of frequency change. */
    ctl_slope_limiter_t freq_slope;

    /** @brief Saturation block to limit the output voltage magnitude. */
    ctl_saturation_t volt_sat;

} ctl_const_vf_controller;

/**
 * @brief Initializes the Constant V/F Controller.
 * @param[out] ctrl Pointer to the ctl_const_vf_controller object.
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 * @param[in] frequency The initial target frequency in Hz.
 * @param[in] freq_slope The maximum rate of frequency change in Hz/s.
 * @param[in] voltage_bound The maximum output voltage magnitude (p.u.).
 * @param[in] voltage_over_frequency The V/F ratio constant (p.u./Hz).
 * @param[in] voltage_bias The voltage boost at low frequencies (p.u.).
 */
void ctl_init_const_vf_controller(ctl_const_vf_controller* ctrl, parameter_gt isr_freq, parameter_gt frequency,
                                  parameter_gt freq_slope, ctrl_gt voltage_bound, ctrl_gt voltage_over_frequency,
                                  ctrl_gt voltage_bias);

/**
 * @brief Executes one step of the constant V/F controller.
 * @details This function calculates the next frequency and the corresponding voltage
 * magnitude based on the V/F profile. It then generates the next electrical angle.
 * @param[in,out] ctrl Pointer to the ctl_const_vf_controller object.
 * @return ctrl_gt The calculated target voltage amplitude (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_const_vf(ctl_const_vf_controller* ctrl)
{
    // Step to the next frequency according to the slope limit
    ctrl->current_freq = ctl_step_slope_limiter(&ctrl->freq_slope, ctrl->target_frequency);

    // Calculate the absolute value of the frequency for voltage calculation
    ctrl_gt freq_abs = (ctrl->current_freq >= 0) ? ctrl->current_freq : -ctrl->current_freq;

    // Calculate target voltage magnitude based on the V/F profile
    if (freq_abs > ctrl->freq_deadband)
    {
        ctrl->target_voltage = ctl_step_saturation(&ctrl->volt_sat, ctrl->v_bias + ctl_mul(ctrl->v_over_f, freq_abs));
    }
    else // In dead-band
    {
        ctrl->target_voltage = 0;
    }

    // Update the ramp generator's slope (i.e., its frequency)
    ctl_set_ramp_generator_slope(&ctrl->rg, ctrl->current_freq);

    // Generate the next angle based on the new frequency
    ctrl->enc.elec_position = ctl_step_ramp_generator(&ctrl->rg);
    ctrl->enc.position = ctrl->enc.elec_position;

    // Return the calculated voltage magnitude
    return ctrl->target_voltage;
}

/**
 * @brief Sets a new target frequency for the V/F controller.
 * @param[in,out] ctrl Pointer to the ctl_const_vf_controller object.
 * @param[in] target_freq The new target frequency in Hz.
 * @param[in] isr_freq The frequency of the interrupt service routine (ISR) in Hz.
 */
void ctl_set_const_vf_target_freq(ctl_const_vf_controller* ctrl, parameter_gt target_freq, parameter_gt isr_freq);

/** 
 * @} 
 */ // end of ConstVFGen group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONST_VF_H_
