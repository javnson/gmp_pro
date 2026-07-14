/**
 * @file buckboost.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a 4-switch non-inverting Buck-Boost duty cycle calculation.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 * 
 */

#include <ctl/component/interface/pwm_channel.h>
#include <ctl/component/digital_power/dcdc/dcdc_core.h>

#ifndef _FILE_BUCKBOOST_4CH_H_
#define _FILE_BUCKBOOST_4CH_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Comprehensive asset parameters for 4-Switch Buck-Boost auto-tuning.
 */
typedef struct _tag_4switch_buckboost_hardware_t
{
    parameter_gt fs; /**< System sampling and loop execution frequency (Hz). */

    /* Plant Physical Assets */
    parameter_gt v_in_min;  /**< Minimum operational input DC voltage (V). */
    parameter_gt v_in_max;  /**< Maximum operational input DC voltage (V). */
    parameter_gt v_out_min; /**< Minimum operational target output voltage (V). */
    parameter_gt v_out_max; /**< Maximum operational target output voltage (V). */

    parameter_gt L_henry;    /**< Main power inductor value (Henry). */
    parameter_gt R_esr_ohm;  /**< Inductor equivalent series resistance (Ohm). */
    parameter_gt C_farad;    /**< Output filter capacitance value (Farad). */
    parameter_gt R_load_min; /**< Minimum rated equivalent resistive load (Ohm) - [Worst Case Heavy Load]. */

    /* Normalization Bases */
    parameter_gt v_base; /**< ADC Voltage normalization base (V corresponding to 1.0 PU). */
    parameter_gt i_base; /**< ADC Current normalization base (A corresponding to 1.0 PU). */

    /* Operational Constraints */
    parameter_gt slope_v_pu_s; /**< Target voltage soft-start ramp rate (PU/sec). */
    parameter_gt slope_i_pu_s; /**< Target current protection ramp rate (PU/sec). */
    ctrl_gt i_out_max;         /**< Explicit maximum positive current saturation limit (PU). */
    ctrl_gt i_out_min;         /**< Explicit maximum reverse current saturation limit (PU). */
    ctrl_gt v_cmd_max;         /**< Maximum voltage command produced by the inner current loop (PU). */
    ctrl_gt v_cmd_min;         /**< Minimum voltage command produced by the inner current loop (PU). */

    /* Loop Dynamic Goals */
    parameter_gt fc_current_loop; /**< Target cross-over frequency for the inner current loop (Hz). */
    parameter_gt fc_voltage_loop; /**< Target desired cross-over frequency for the outer voltage loop (Hz). */
} ctl_4switch_buckboost_hardware_t;

/*---------------------------------------------------------------------------*/
/* Exported Blueprint Service APIs                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Calculates the absolute worst-case RHPZ frequency occurring in the Boost region of the 4S-BB.
 * @param[in] hw Pointer to the 4-Switch Buck-Boost hardware asset structure.
 * @return parameter_gt The lowest RHPZ frequency in Hz.
 */
parameter_gt ctl_fsbb_calc_worst_rhp_zero(const ctl_4switch_buckboost_hardware_t* hw);

/**
 * @brief Computes the unified single PID coefficient set for the 4-Switch Buck-Boost converter.
 * @param[out] init_config Pointer to the destination configuration profile to be populated.
 * @param[in] hw Pointer to the comprehensive 4S-BB hardware asset boundary structure.
 */
void ctl_dcdc_blueprint_fsbb_cascade(ctl_dcdc_core_init_t* init_config, const ctl_4switch_buckboost_hardware_t* hw);

/*---------------------------------------------------------------------------*/
/* Four-Switch Buck-Boost (FSBB) Modulator                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the FSBB Modulator.
 */
typedef struct _tag_fsbb_modulator_t
{
    pwm_dual_channel_t pwm; //!< Ch0: Buck leg (Q1), Ch1: Boost leg (Q4).
    ctrl_gt duty_max;       //!< Max duty (e.g., 0.95) to guarantee bootstrap charging.
    ctrl_gt duty_min;       //!< Min duty (e.g., 0.05) to guarantee bootstrap charging.
    ctrl_gt m_low;          //!< Lower threshold for transition zone (e.g., 0.90).
    ctrl_gt m_high;         //!< Upper threshold for transition zone (e.g., 1.10).
} fsbb_modulator_t;

/**
 * @brief Initializes the FSBB modulator.
 * @param mod Pointer to the FSBB modulator object.
 * @param m_low Lower threshold for transition zone (V_req / V_in ratio, e.g. 0.9).
 * @param m_high Upper threshold for transition zone (V_req / V_in ratio, e.g. 1.1).
 */
GMP_STATIC_INLINE void ctl_init_fsbb_modulator(fsbb_modulator_t* mod, pwm_gt full_scale, ctrl_gt duty_max,
                                               ctrl_gt duty_min, ctrl_gt m_low, ctrl_gt m_high)
{
    ctl_init_pwm_dual_channel(&mod->pwm, 0, full_scale);
    mod->duty_max = duty_max;
    mod->duty_min = duty_min;
    mod->m_low = m_low;
    mod->m_high = m_high;
}

/**
 * @brief Executes the FSBB modulation step with Transition Zone blending.
 * @details Solves the Non-Inverting Buck-Boost equation: V_out / V_in = D_buck / (1 - D_boost)
 */
GMP_STATIC_INLINE void ctl_step_fsbb_modulator(fsbb_modulator_t* mod, ctrl_gt v_req, ctrl_gt v_in)
{
    ctrl_gt v_in_safe = (v_in > float2ctrl(0.1f)) ? v_in : float2ctrl(0.1f);

    // M represents the demanded voltage gain (V_req / V_in)
    ctrl_gt M = ctl_div(v_req, v_in_safe);

    ctrl_gt d_buck = float2ctrl(0.0f);
    ctrl_gt d_boost = float2ctrl(0.0f);

    if (M <= mod->m_low)
    {
        // --- PURE BUCK MODE ---
        // Boost leg must keep switching at min duty to maintain high-side bootstrap.
        d_boost = mod->duty_min;

        // M = D_buck / (1 - D_boost)  =>  D_buck = M * (1 - D_boost)
        d_buck = ctl_mul(M, float2ctrl(1.0f) - d_boost);
    }
    else if (M >= mod->m_high)
    {
        // --- PURE BOOST MODE ---
        // Buck leg must not stay at 1.0; force to duty_max to maintain its bootstrap.
        d_buck = mod->duty_max;

        // M = D_buck / (1 - D_boost)  =>  D_boost = 1 - (D_buck / M)
        d_boost = float2ctrl(1.0f) - ctl_div(d_buck, M);
    }
    else
    {
        // --- TRANSITION ZONE (BUCK-BOOST MODE) ---
        // Smoothly blend the duty cycles linearly across the transition band.
        // Weight 'w' goes from 0.0 (at m_low) to 1.0 (at m_high)
        ctrl_gt w = ctl_div(M - mod->m_low, mod->m_high - mod->m_low);

        // Linear interpolation for Buck Duty
        ctrl_gt d_buck_start = ctl_mul(mod->m_low, float2ctrl(1.0f) - mod->duty_min);
        ctrl_gt d_buck_end = mod->duty_max;
        d_buck = d_buck_start + ctl_mul(w, d_buck_end - d_buck_start);

        // Linear interpolation for Boost Duty
        ctrl_gt d_boost_start = mod->duty_min;
        ctrl_gt d_boost_end = float2ctrl(1.0f) - ctl_div(mod->duty_max, mod->m_high);
        d_boost = d_boost_start + ctl_mul(w, d_boost_end - d_boost_start);
    }

    // Final hard-clamp for absolute safety
    d_buck = ctl_sat(d_buck, mod->duty_max, mod->duty_min);
    d_boost = ctl_sat(d_boost, mod->duty_max, mod->duty_min);

    // Map to Dual PWM Channels
    ctl_vector2_t raw_duty;
    raw_duty.dat[0] = d_buck;  // Channel 0 -> Buck Leg (Q1)
    raw_duty.dat[1] = d_boost; // Channel 1 -> Boost Leg (Q4)

#if (PWM_MODULATOR_USING_NEGATIVE_LOGIC == 1)
    ctl_step_pwm_dual_channel_inv(&mod->pwm, &raw_duty);
#else
    ctl_step_pwm_dual_channel(&mod->pwm, &raw_duty);
#endif
}

/**
 * @brief Retrieves the actual calculated compare value for the Buck Leg.
 */
GMP_STATIC_INLINE pwm_gt ctl_get_fsbb_buck_cmp(fsbb_modulator_t* mod)
{
    return mod->pwm.value[0];
}

/**
 * @brief Retrieves the actual calculated compare value for the Boost Leg.
 */
GMP_STATIC_INLINE pwm_gt ctl_get_fsbb_boost_cmp(fsbb_modulator_t* mod)
{
    return mod->pwm.value[1];
}



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BUCKBOOST_4CH_H_

/**
 * @}
 */
