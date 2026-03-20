/**
 * @file pll.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a three-phase Synchronous Reference Frame PLL (SRF-PLL).
 * @version 1.1
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 */

/**
 * @defgroup CTL_PLL_API Phase-Locked Loop (PLL) API
 * @{
 * @ingroup CTL_DP_LIB
 * @brief A standard three-phase SRF-PLL for grid synchronization.
 */

#ifndef _FILE_THREE_PHASE_PLL_DDSRF_H_
#define _FILE_THREE_PHASE_PLL_DDSRF_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifndef CTL_DDSRF_USE_DOUBLE_ANGLE_FORMULA
#define CTL_DDSRF_USE_DOUBLE_ANGLE_FORMULA (0)
#endif // CTL_DDSRF_USE_DOUBLE_ANGLE_FORMULA

/**
 * @brief Data structure for the Decoupled Double Synchronous Reference Frame (DDSRF) PLL.
 * @details This PLL structure handles unbalanced grid conditions by decoupling the
 * positive and negative sequence components.
 */
typedef struct _tag_ddsrf_pll
{
    //
    // Input Variables
    //
    ctrl_gt e_alpha; //!< Input voltage in the alpha-axis of the stationary reference frame.
    ctrl_gt e_beta;  //!< Input voltage in the beta-axis of the stationary reference frame.

    //
    // --- Decoupling Network States ---
    //
    ctl_vector2_t v_pos_raw; //!< Raw voltage in positive sequence frame (contains 2*omega ripple).
    ctl_vector2_t v_neg_raw; //!< Raw voltage in negative sequence frame (contains 2*omega ripple).

    ctl_vector2_t v_pos_dc; //!< Decoupled (Filtered) Positive Sequence Voltage (DC).
    ctl_vector2_t v_neg_dc; //!< Decoupled (Filtered) Negative Sequence Voltage (DC).

    //
    // --- Output Variables ---
    //
    ctrl_gt v_pos_mag; //!< Magnitude of positive sequence voltage.
    ctrl_gt v_neg_mag; //!< Magnitude of negative sequence voltage.

    ctrl_gt theta;        //!< Estimated grid angle, in per-unit format (0 to 1.0 represents 0 to 2*pi).
    ctl_vector2_t phasor; //!< Phasor corresponding to the estimated angle {cos(theta), sin(theta)}.
    ctrl_gt freq_pu;      //!< Estimated grid frequency, in per-unit format.

    //
    // Intermediate Variables
    //
    ctrl_gt e_error; //!< The error signal (q-axis voltage) used by the PI controller.

    ctl_vector2_t phasor_2w; //!< Phasor for 2*theta {cos(2theta), sin(2theta)}.

    //
    // Parameters
    //
    ctrl_gt freq_sf; //!< Scaling factor to convert per-unit frequency to per-unit angle increment per tick.

    //
    // Internal Controller Objects
    //
    ctl_pid_t pid_pll; //!< PI controller for the phase-locking loop. Output is the frequency deviation.

    // Four Low-Pass Filters are required for the decoupling network.
    // Cutoff frequency is typically set to freq_grid / sqrt(2).
    ctl_filter_IIR1_t lpf_pos_d;
    ctl_filter_IIR1_t lpf_pos_q;
    ctl_filter_IIR1_t lpf_neg_d;
    ctl_filter_IIR1_t lpf_neg_q;

} ddsrf_pll_t;

/**
 * @brief Initialize the DDSRF-PLL.
 */
void ctl_init_ddsrf_pll(ddsrf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_ki,
                        parameter_gt f_ctrl, parameter_gt decoupling_fc);

/**
 * @brief Auto-tune and initialize the DDSRF-PLL.
 * @details Bandwidth usually 20-30Hz. Decoupling FC usually f_base/sqrt(2).
 */
void ctl_init_ddsrf_pll_auto_tune(ddsrf_pll_t* pll, parameter_gt f_base, parameter_gt f_ctrl, parameter_gt voltage_mag,
                                  parameter_gt bandwidth_hz);

/**
 * @brief Clears internal states.
 */
GMP_STATIC_INLINE void ctl_clear_ddsrf_pll(ddsrf_pll_t* pll)
{
    pll->e_alpha = 0;
    pll->e_beta = 0;
    pll->e_error = 0;

    // Reset Angle & Freq
    pll->theta = 0;
    pll->freq_pu = float2ctrl(1.0f); // 1.0 p.u.
    ctl_set_phasor_via_angle(pll->theta, &pll->phasor);

    // Reset Decoupling States
    ctl_vector2_clear(&pll->v_pos_raw);
    ctl_vector2_clear(&pll->v_neg_raw);
    ctl_vector2_clear(&pll->v_pos_dc);
    ctl_vector2_clear(&pll->v_neg_dc);

    // Reset Filters
    ctl_clear_filter_iir1(&pll->lpf_pos_d);
    ctl_clear_filter_iir1(&pll->lpf_pos_q);
    ctl_clear_filter_iir1(&pll->lpf_neg_d);
    ctl_clear_filter_iir1(&pll->lpf_neg_q);

    // Reset PID
    ctl_clear_pid(&pll->pid_pll);

    pll->v_pos_mag = 0;
    pll->v_neg_mag = 0;
}

/**
 * @brief Executes one step of the DDSRF-PLL algorithm.
 * @param[in,out] pll Pointer to the DDSRF-PLL structure.
 * @param[in] alpha Measured alpha-axis voltage.
 * @param[in] beta Measured beta-axis voltage.
 * @return Estimated grid angle (theta) in p.u.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_ddsrf_pll(ddsrf_pll_t* pll, ctrl_gt alpha, ctrl_gt beta)
{
    // Store inputs
    pll->e_alpha = alpha;
    pll->e_beta = beta;

    // Prepare vector for optimized park functions
    ctl_vector2_t v_ab_2;
    v_ab_2.dat[phase_alpha] = alpha;
    v_ab_2.dat[phase_beta] = beta;

    // 1. Update Phasor (Theta -> Sin, Cos)
    ctl_set_phasor_via_angle(pll->theta, &pll->phasor);

    // 2. Calculate 2*Theta Phasor for Decoupling
#if CTL_DDSRF_USE_DOUBLE_ANGLE_FORMULA == 1
    // cos(2x) = cos^2(x) - sin^2(x)
    // sin(2x) = 2*sin(x)*cos(x)
    // Note: Using ctl_mul for fixed-point safety
    ctrl_gt cos_sq = ctl_mul(pll->phasor.dat[phasor_cos], pll->phasor.dat[phasor_cos]);
    ctrl_gt sin_sq = ctl_mul(pll->phasor.dat[phasor_sin], pll->phasor.dat[phasor_sin]);

    pll->phasor_2w.dat[phasor_cos] = cos_sq - sin_sq;
    pll->phasor_2w.dat[phasor_sin] = ctl_mul2(ctl_mul(sin_theta, cos_theta));

#else
    // calculate directly.
    ctl_set_phasor_via_angle(ctl_mul2(pll->theta), &pll->phasor_2w);

#endif //CTL_DDSRF_USE_DOUBLE_ANGLE_FORMULA

    ctrl_gt cos_2theta = pll->phasor_2w.dat[phasor_cos];
    ctrl_gt sin_2theta = pll->phasor_2w.dat[phasor_sin];

    // 3. Raw Transformation (Coupled)
    // Positive Sequence Frame (+omega)
    ctl_ct_park2(&v_ab_2, &pll->phasor, &pll->v_pos_raw);

    // Negative Sequence Frame (-omega)
    ctl_ct_park2_neg(&v_ab_2, &pll->phasor, &pll->v_neg_raw);

    // 4. Decoupling Network Calculation
    // Use the *filtered DC values* from the previous step to cancel the *current raw ripple*

    // --- Decouple Positive Sequence ---
    // Remove V_neg component which appears as V_neg_dc * e^(-j2theta)
    // d_pos* = d_pos_raw - (d_neg_dc * cos2t + q_neg_dc * sin2t)
    // q_pos* = q_pos_raw - (q_neg_dc * cos2t - d_neg_dc * sin2t)

    ctrl_gt term_p1 = ctl_mul(pll->v_neg_dc.dat[phase_d], cos_2theta);
    ctrl_gt term_p2 = ctl_mul(pll->v_neg_dc.dat[phase_q], sin_2theta);

    ctrl_gt term_p3 = ctl_mul(pll->v_neg_dc.dat[phase_q], cos_2theta);
    ctrl_gt term_p4 = ctl_mul(pll->v_neg_dc.dat[phase_d], sin_2theta);

    ctrl_gt v_pos_d_decoupled = pll->v_pos_raw.dat[phase_d] - (term_p1 + term_p2);
    ctrl_gt v_pos_q_decoupled = pll->v_pos_raw.dat[phase_q] - (term_p3 - term_p4);

    // --- Decouple Negative Sequence ---
    // Remove V_pos component which appears as V_pos_dc * e^(+j2theta)
    // d_neg* = d_neg_raw - (d_pos_dc * cos2t - q_pos_dc * sin2t)
    // q_neg* = q_neg_raw - (q_pos_dc * cos2t + d_pos_dc * sin2t)

    ctrl_gt term_n1 = ctl_mul(pll->v_pos_dc.dat[phase_d], cos_2theta);
    ctrl_gt term_n2 = ctl_mul(pll->v_pos_dc.dat[phase_q], sin_2theta);

    ctrl_gt term_n3 = ctl_mul(pll->v_pos_dc.dat[phase_q], cos_2theta);
    ctrl_gt term_n4 = ctl_mul(pll->v_pos_dc.dat[phase_d], sin_2theta);

    ctrl_gt v_neg_d_decoupled = pll->v_neg_raw.dat[phase_d] - (term_n1 - term_n2);
    ctrl_gt v_neg_q_decoupled = pll->v_neg_raw.dat[phase_q] - (term_n3 + term_n4);

    // 5. Low Pass Filtering (Extract DC)
    pll->v_pos_dc.dat[phase_d] = ctl_step_filter_iir1(&pll->lpf_pos_d, v_pos_d_decoupled);
    pll->v_pos_dc.dat[phase_q] = ctl_step_filter_iir1(&pll->lpf_pos_q, v_pos_q_decoupled);

    pll->v_neg_dc.dat[phase_d] = ctl_step_filter_iir1(&pll->lpf_neg_d, v_neg_d_decoupled);
    pll->v_neg_dc.dat[phase_q] = ctl_step_filter_iir1(&pll->lpf_neg_q, v_neg_q_decoupled);

    // 6. Loop Control (PI on Positive Q-Axis)
    // Error signal is the decoupled positive sequence q-axis voltage
    pll->e_error = pll->v_pos_dc.dat[phase_q];

    ctl_step_pid_par(&pll->pid_pll, pll->e_error);

    // 7. Update Frequency and Angle
    pll->freq_pu = float2ctrl(1.0f) + pll->pid_pll.out;

    // theta = theta + freq * Ts_gain
    pll->theta += ctl_mul(pll->freq_pu, pll->freq_sf);
    pll->theta = ctrl_mod_1(pll->theta); // Wrap [0, 1.0]

    // 8. Update Output Magnitudes
    // Magnitudes are simply the D-component in SRF if Q is regulated to 0 (for Pos seq)
    // For Neg seq, magnitude is sqrt(d^2 + q^2).
    // Here we simply output the DC components, user can calculate sqrt if needed.
    // Or approximate mag = d for pos seq.
    pll->v_pos_mag = pll->v_pos_dc.dat[phase_d];

    // For negative sequence, we might want the vector magnitude
    // If not computationally expensive:
    // pll->v_neg_mag = sqrt(d*d + q*q);
    // For now, assign d component or implement explicit mag calc based on demand.
    // Simple Approximation:
    pll->v_neg_mag = pll->v_neg_dc.dat[phase_d]; // Placeholder if neg q is small, else needs sqrt

    return pll->theta;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_PLL_H_

/**
 * @}
 */
