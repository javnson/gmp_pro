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
parameter_gt ctl_4sbb_calc_worst_rhp_zero(const ctl_4switch_buckboost_hardware_t* hw);

/**
 * @brief Computes the unified single PID coefficient set for the 4-Switch Buck-Boost converter.
 * @param[out] init_config Pointer to the destination configuration profile to be populated.
 * @param[in] hw Pointer to the comprehensive 4S-BB hardware asset boundary structure.
 */
void ctl_dcdc_blueprint_4sbb_cascade(ctl_dcdc_core_init_t* init_config, const ctl_4switch_buckboost_hardware_t* hw);

/**
 * @brief Defines the maximum duty cycle for the pure Buck operating region.
 * This threshold is used to manage the transition between different operating modes.
 */
#define D_buck_max float2ctrl(0.8)

/**
 * @brief Enumeration for indexing the duty cycles of the Buck and Boost switches.
 */
typedef enum _tag_buckboost_phase
{
    buck_phase = 0, //!< Index for the Buck switch duty cycle in the output vector.
    boost_phase = 1 //!< Index for the Boost switch duty cycle in the output vector.
} ctl_buckboost_phases_t;

/**
 * @brief Calculates the duty cycles for the Buck and Boost switches based on the desired voltage ratio.
 * @ingroup CTL_BUCKBOOST_API
 * @details This function implements a control strategy that divides the converter's operation
 * into four distinct regions to manage the transition between Buck and Boost modes.
 *
 * The four operating regions are:
 * 1.  **Buck Section:** `ratio` in `[0, D_buck_max]`. Only the Buck controller is active.
 * 2.  **Buck-Boost Section 1:** `ratio` in `(D_buck_max, 1]`. The Buck duty is held at maximum while the Boost duty ramps up.
 * 3.  **Buck-Boost Section 2:** `ratio` in `(1, 1/D_buck_max]`. The Boost duty is held constant while the Buck duty changes.
 * 4.  **Boost Section:** `ratio` > `1/D_buck_max`. Only the Boost controller is active.
 *
 * @warning The logic in "Buck-Boost Section 2" appears to contradict the intended behavior. The comment
 * suggests the Buck duty should decrease, but the code causes it to increase. This also creates a
 * discontinuity in the Buck duty cycle at the boundary with the "Boost Section". Please review this
 * logic carefully.
 *
 * @param[in] ratio The desired voltage conversion ratio (Vout / Vin).
 * @param[out] buck_boost_duty A 2-element vector where `dat[0]` will be the Buck duty and `dat[1]` will be the Boost duty.
 */
GMP_STATIC_INLINE void ctl_buckboost_duty_preset(ctrl_gt ratio, ctl_vector2_t* buck_boost_duty)
{
    if (ratio <= D_buck_max)
    {
        // Region 1: Pure Buck mode
        buck_boost_duty->dat[buck_phase] = ratio;
        buck_boost_duty->dat[boost_phase] = 0;
    }
    else if (ratio <= float2ctrl(1))
    {
        // Region 2: Transition from Buck to Boost
        buck_boost_duty->dat[buck_phase] = D_buck_max;
        buck_boost_duty->dat[boost_phase] = ctl_div(ctl_sub(ratio, D_buck_max), ratio);
    }
    else if (ratio <= ctl_div(float2ctrl(1), D_buck_max))
    {
        // Region 3: Transition from Buck to Boost (WARNING: See function description)
        buck_boost_duty->dat[buck_phase] = ctl_mul(D_buck_max, ratio);
        buck_boost_duty->dat[boost_phase] = ctl_sub(float2ctrl(1), D_buck_max);
    }
    else
    {
        // Region 4: Pure Boost mode
        buck_boost_duty->dat[buck_phase] = 0;
        buck_boost_duty->dat[boost_phase] = ctl_div(ctl_sub(ratio, float2ctrl(1)), ratio);
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_BUCKBOOST_4CH_H_

/**
 * @}
 */
