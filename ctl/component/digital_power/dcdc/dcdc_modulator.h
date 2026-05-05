/**
 * @defgroup DCDC_MODULATOR DC-DC Modulators
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief Modulators for Buck, Boost, and Four-Switch Buck-Boost (FSBB) topologies.
 * @details These modulators translate the requested voltage (V_req) from the DCDC Core 
 * into actual PWM duty cycles based on topological physical equations, including 
 * strict bootstrap capacitor charging limits and smooth mode transitions.
 */

#ifndef _FILE_CTL_DCDC_MODULATOR_H_
#define _FILE_CTL_DCDC_MODULATOR_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/interface/pwm_channel.h>

/*---------------------------------------------------------------------------*/
/* 1. Buck Modulator                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Buck Modulator.
 */
typedef struct _tag_buck_modulator_t
{
    pwm_channel_t pwm;
    ctrl_gt duty_max; //!< Max duty to ensure bottom switch turns on for bootstrap.
    ctrl_gt duty_min; //!< Min duty to ensure top switch minimum ON time.
} buck_modulator_t;

GMP_STATIC_INLINE void ctl_init_buck_modulator(buck_modulator_t* mod, pwm_gt full_scale, ctrl_gt duty_max,
                                               ctrl_gt duty_min)
{
    ctl_init_pwm_channel(&mod->pwm, 0, full_scale);
    mod->duty_max = duty_max;
    mod->duty_min = duty_min;
}

GMP_STATIC_INLINE pwm_gt ctl_step_buck_modulator(buck_modulator_t* mod, ctrl_gt v_req, ctrl_gt v_in)
{
    ctrl_gt v_in_safe = (v_in > float2ctrl(0.1f)) ? v_in : float2ctrl(0.1f);
    ctrl_gt duty = ctl_div(v_req, v_in_safe);

    // Strict limits for Bootstrap and Min-On time
    duty = ctl_sat(duty, mod->duty_max, mod->duty_min);
    return ctl_step_pwm_channel(&mod->pwm, duty);
}

/*---------------------------------------------------------------------------*/
/* 2. Boost Modulator                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Boost Modulator.
 */
typedef struct _tag_boost_modulator_t
{
    pwm_channel_t pwm;
    ctrl_gt duty_max; //!< Max duty for bottom switch (prevents continuous inductor short).
    ctrl_gt duty_min; //!< Min duty to ensure bootstrap capacitor charges.
} boost_modulator_t;

GMP_STATIC_INLINE void ctl_init_boost_modulator(boost_modulator_t* mod, pwm_gt full_scale, ctrl_gt duty_max,
                                                ctrl_gt duty_min)
{
    ctl_init_pwm_channel(&mod->pwm, 0, full_scale);
    mod->duty_max = duty_max;
    mod->duty_min = duty_min;
}

GMP_STATIC_INLINE pwm_gt ctl_step_boost_modulator(boost_modulator_t* mod, ctrl_gt v_req, ctrl_gt v_out)
{
    ctrl_gt v_out_safe = (v_out > float2ctrl(0.1f)) ? v_out : float2ctrl(0.1f);
    ctrl_gt duty = float2ctrl(1.0f) - ctl_div(v_req, v_out_safe);

    duty = ctl_sat(duty, mod->duty_max, mod->duty_min);
    return ctl_step_pwm_channel(&mod->pwm, duty);
}

/*---------------------------------------------------------------------------*/
/* 3. Four-Switch Buck-Boost (FSBB) Modulator                                */
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

    ctl_step_pwm_dual_channel(&mod->pwm, &raw_duty);
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

#endif // _FILE_CTL_DCDC_MODULATOR_H_
