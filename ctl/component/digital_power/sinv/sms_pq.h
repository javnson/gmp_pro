/**
 * @file ctl_sms_pq.h
 * @author GMP Library Contributors
 * @brief Single-Phase Measurement System PQ Single-Phase Instantaneous Active and Reactive Power Calculator.
 * @details Utilizes Orthogonal Signal Generation (OSG) via SOGI to calculate 
 * ripple-free instantaneous power for single-phase systems. Eliminates the 
 * need for sluggish double-line-frequency notch filters.
 * * @version 1.0
 * @copyright Copyright GMP(c) 2024-2026
 */

#ifndef _CTL_SINGLE_PHASE_PQ_H_
#define _CTL_SINGLE_PHASE_PQ_H_

#include <ctl/component/intrinsic/discrete/biquad_filter.h>
#include <ctl/component/intrinsic/discrete/discrete_sogi.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Data structure for the Single-Phase PQ Calculator.
 */
typedef struct _tag_single_phase_pq_t
{
    // --- Outputs ---
    ctrl_gt active_power_p;   //!< Calculated Average Active Power (P)
    ctrl_gt reactive_power_q; //!< Calculated Average Reactive Power (Q)
    ctrl_gt apparent_power_s; //!< Calculated Apparent Power (S)

    // --- Internal State Variables ---
    ctl_vector2_t i_ab; //!< Current in alpha-beta stationary frame

    // --- Core Sub-modules ---
    discrete_sogi_t sogi_i;    //!< SOGI to generate orthogonal current signals (i_alpha, i_beta).
    ctl_biquad_filter_t lpf_p; //!< High-bandwidth Biquad LPF to remove PWM switching noise from P.
    ctl_biquad_filter_t lpf_q; //!< High-bandwidth Biquad LPF to remove PWM switching noise from Q.

} ctl_sms_pq_t;

/*---------------------------------------------------------------------------*/
/* Function Prototypes and Inline Implementations                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the Single-Phase PQ Calculator.
 * @param[out] pq Pointer to the PQ calculator instance.
 * @param[in] grid_freq Nominal grid frequency in Hz (e.g., 50.0f).
 * @param[in] fs Controller execution sampling frequency in Hz.
 * @param[in] lpf_fc Cutoff frequency for the output noise filter in Hz (e.g., 200.0f).
 */
void ctl_init_sms_pq(ctl_sms_pq_t* pq, parameter_gt grid_freq, parameter_gt fs, parameter_gt lpf_fc);

/**
 * @brief Clears the internal states of the PQ calculator.
 * @param[in,out] pq Pointer to the PQ calculator instance.
 */
GMP_STATIC_INLINE void ctl_clear_sms_pq(ctl_sms_pq_t* pq)
{
    ctl_clear_discrete_sogi(&pq->sogi_i);
    ctl_clear_biquad_filter(&pq->lpf_p);
    ctl_clear_biquad_filter(&pq->lpf_q);

    pq->active_power_p = float2ctrl(0.0f);
    pq->reactive_power_q = float2ctrl(0.0f);
    pq->apparent_power_s = float2ctrl(0.0f);

    ctl_vector2_clear(&pq->i_ab);
}

/**
 * @brief Executes one step of the Single-Phase Instantaneous PQ Calculation.
 * * @details 
 * Uses the pre-calculated v_alpha and v_beta from the system's PLL to save DSP cycles.
 * Generates i_alpha and i_beta internally using an independent SOGI.
 * * @param[in,out] pq Pointer to the PQ calculator instance.
 * @param[in] v_alpha Grid voltage alpha component (from PLL's SOGI).
 * @param[in] v_beta Grid voltage beta component (from PLL's SOGI).
 * @param[in] i_ac The raw instantaneous single-phase AC grid current feedback.
 */
GMP_STATIC_INLINE void ctl_step_sms_pq(ctl_sms_pq_t* pq, ctrl_gt v_alpha, ctrl_gt v_beta, ctrl_gt i_ac)
{
    // 1. Generate orthogonal current signals using SOGI
    ctl_step_discrete_sogi(&pq->sogi_i, i_ac);

    // 2. Align coordinate conventions with the PLL module
    // The PLL convention is: alpha = -sogi_ds, beta = sogi_qs
    pq->i_ab.dat[phase_alpha] = -ctl_get_discrete_sogi_ds(&pq->sogi_i);
    pq->i_ab.dat[phase_beta] = ctl_get_discrete_sogi_qs(&pq->sogi_i);

    // 3. Instantaneous Power Theory for Single Phase (using peak values)
    // Note: The 0.5f multiplier is required because SOGI produces peak-amplitude signals.
    // P = 1/2 * (V_alpha * I_alpha + V_beta * I_beta)
    ctrl_gt p_inst = ctl_mul(float2ctrl(0.5f),
                             ctl_mul(v_alpha, pq->i_ab.dat[phase_alpha]) + ctl_mul(v_beta, pq->i_ab.dat[phase_beta]));

    // Q = 1/2 * (V_beta * I_alpha - V_alpha * I_beta)
    ctrl_gt q_inst = ctl_mul(float2ctrl(0.5f),
                             ctl_mul(v_beta, pq->i_ab.dat[phase_alpha]) - ctl_mul(v_alpha, pq->i_ab.dat[phase_beta]));

    // 4. Low-Pass Filtering (Removes switching noise, NOT double-line frequency)
    pq->active_power_p = ctl_step_biquad_filter(&pq->lpf_p, p_inst);
    pq->reactive_power_q = ctl_step_biquad_filter(&pq->lpf_q, q_inst);

    // 5. Apparent Power Calculation (S = sqrt(P^2 + Q^2))
    // Uses intrinsic fast square root if available in your lib
    ctrl_gt p_sq = ctl_mul(pq->active_power_p, pq->active_power_p);
    ctrl_gt q_sq = ctl_mul(pq->reactive_power_q, pq->reactive_power_q);
    pq->apparent_power_s = ctl_sqrt(p_sq + q_sq);
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_SINGLE_PHASE_PQ_H_
