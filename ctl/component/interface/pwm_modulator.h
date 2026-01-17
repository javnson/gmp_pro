/**
 * @file pwm_modulator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for three-phase bridge modulation with dead-time compensation.
 * @version 1.0
 * @date 2026-01-15
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TP_MODULATION_API Three-Phase Modulation API
 * @{
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief Provides functions for generating three-phase PWM signals from voltage commands,
 * including dead-time compensation based on current direction.
 */

#ifndef _FILE_THREE_PHASE_MODULATION_H_
#define _FILE_THREE_PHASE_MODULATION_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//////////////////////////////////////////////////////////////////////////
// SPWM modulator
//

/**
 * @brief Data structure for the three-phase bridge SPWM modulation module.
 * @note Recommended Value for Current Deadband:
 * - Theoretical Minimum: I_min = 2 * Coss * Vdc / T_dt
 * - Practical Setting: Typically 1% to 3% of the rated current.
 * - Tuning Guide: Set higher if zero-crossing oscillation occurs; set lower if crossover distortion is observed.
 * Ensure value is strictly larger than the ADC noise floor.
 */
typedef struct _tag_spwm_modulator
{
    // --- Input ---
    ctl_vector3_t vab0_out; //!< RO: target output voltage v alpha beta 0 (Input must be Per-Unit: -1.0 to 1.0).
    ctl_vector3_t* iuvw;    //!< RO: Filtered three-phase currents. POSITIVE direction = Flowing OUT of inverter.

    // --- Output Ports ---
    pwm_gt pwm_out[3]; //!< Final PWM compare values {A, B, C}.

    // --- Parameters ---
    pwm_gt pwm_full_scale;           //!< PWM counter period value (100% duty cycle).
    pwm_gt pwm_deadband_comp_val;    //!< Dead-time compensation value (in PWM counts). Added/Subtracted from CMP value.
    ctrl_gt current_deadband;        //!< Base threshold for dead-time compensation (e.g., 0.02pu).
    ctrl_gt current_hysteresis_band; //!< Hysteresis band width (e.g., 0.005pu).

    // --- Internal State ---
    vector3_gt vabc_out;
    signed short last_current_dir[3];         //!< State memory: 1(Pos), -1(Neg), 0(None).
    fast_gt flag_enable_deadband_compensator; //!< Enable dead band compensator.
} spwm_modulator_t;

/**
 * @brief Clears the internal states of the modulation module.
 * @ingroup CTL_TP_MODULATION_API
 * @param[out] mod Pointer to the @ref spwm_modulator_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_spwm_modulator(spwm_modulator_t* mod)
{
    mod->pwm_out[phase_A] = 0;
    mod->pwm_out[phase_B] = 0;
    mod->pwm_out[phase_C] = 0;

    ctl_vector3_clear(&mod->vabc_out);

    mod->last_current_dir[phase_A] = 0;
    mod->last_current_dir[phase_B] = 0;
    mod->last_current_dir[phase_C] = 0;
}

/**
 * @brief Initializes the three-phase bridge modulation module.
 * @ingroup CTL_TP_MODULATION_API
 *
 * @param[out] bridge Pointer to the `three_phase_bridge_modulation_t` structure.
 * @param[in] pwm_full_scale The maximum value of the PWM counter.
 * @param[in] pwm_deadband The total dead-time value in PWM timer counts.
 * @param[in] iuvw a pointer to inverter output current, point to ADC module or main controller.
 * @param[in] current_deadband The current threshold to enable dead-time compensation.
 * @param[in] current_hysteresis The current hysteresis to enable dead-time compensation.
 */
void ctl_init_spwm_modulator(spwm_modulator_t* mod, pwm_gt pwm_full_scale, pwm_gt pwm_deadband_comp_val,
                             ctl_vector3_t* iuvw, ctrl_gt current_deadband, ctrl_gt current_hysteresis);

/**
 * @brief Executes one step of the three-phase modulation algorithm.
 * @ingroup CTL_TP_MODULATION_API
 * @details Converts per-unit voltage commands (-1.0 to 1.0) to PWM compare values,
 * and applies dead-time compensation based on the direction of phase currents.
 *
 * @param[in,out] mod Pointer to the @ref spwm_modulator_t structure.
 */
GMP_STATIC_INLINE void ctl_step_spwm_modulator(spwm_modulator_t* mod)
{
    gmp_base_assert(mod);
    gmp_base_assert(mod->iuvw);
    gmp_base_assert(mod->pwm_out);

    // Temp variable to handle signed calculations safely
    pwm_gt pwm_value;
    int i;

    // Modulation, SPWM
    ctl_ct_iclarke(&mod->vab0_out, &mod->vabc_out);

    // --- Calculate Raw PWM Values ---
    // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
    for (i = 0; i < 3; ++i)
    {
        // Convert voltage command (-1.0 to 1.0) to duty cycle (0 to 1.0), then to raw PWM value.
        pwm_value = pwm_mul(ctl_div2(ctl_add(float2ctrl(1.0f), mod->vabc_out.dat[i])), mod->pwm_full_scale);

        //
        // NOTE: pwm_gt must be a signed int number.
        // Or dead-time compensator would get a wrong result, when low modulation ratio.
        //

        // dead-time compensation
        if (mod->flag_enable_deadband_compensator)
        {
            // ---------------------------------------------------------
            // ZONE 1: Strong Positive Current -> Apply Positive Comp
            // ---------------------------------------------------------
            if (mod->iuvw->dat[i] > mod->current_deadband + mod->current_hysteresis_band)
            {
                pwm_value += mod->pwm_deadband_comp_val;
                mod->last_current_dir[i] = 1;
            }

            // ---------------------------------------------------------
            // ZONE 2: Strong Negative Current -> Apply Negative Comp
            // ---------------------------------------------------------
            else if (mod->iuvw->dat[i] < (-mod->current_deadband - mod->current_hysteresis_band))
            {
                if (pwm_value >= mod->pwm_deadband_comp_val)
                {
                    pwm_value -= mod->pwm_deadband_comp_val;
                }
                else
                {
                    pwm_value = 0; // avoid unsigned number underflow.
                }

                mod->last_current_dir[i] = -1;
            }

            // ---------------------------------------------------------
            // ZONE 3: Inside Deadband (Parasitic Cap Zone) -> No Comp
            // ---------------------------------------------------------
            else if ((mod->iuvw->dat[i] > -mod->current_deadband) && (mod->iuvw->dat[i] < mod->current_deadband))
            {
                mod->last_current_dir[i] = 0;
            }

            // ---------------------------------------------------------
            // ZONE 4: Hysteresis Band -> Keep Last State
            // ---------------------------------------------------------
            else
            {
                // Current is in the gray area:
                // [Deadband, Deadband + Hysteresis] OR [-Deadband - Hysteresis, -Deadband]

                if (mod->last_current_dir[i] == 1)
                {
                    pwm_value += mod->pwm_deadband_comp_val;
                }
                else if (mod->last_current_dir[i] == -1)
                {
                    if (pwm_value >= mod->pwm_deadband_comp_val)
                    {
                        pwm_value -= mod->pwm_deadband_comp_val;
                    }
                    else
                    {
                        pwm_value = 0; // avoid unsigned number underflow.
                    }
                }
                else
                {
                    // last_current_dir == 0
                    // We came from the dead band zone, so we stay 0 (no comp)
                    // until we cross the outer hysteresis limit.
                }
            }
        }

        // Saturate final PWM values to ensure they are within valid range.
        pwm_value = pwm_sat(pwm_value, mod->pwm_full_scale, 0);

        // write back correct number
        mod->pwm_out[i] = pwm_value;
    }
}




#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_MODULATION_H_

/**
 * @}
 */
