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

#ifndef _FILE_THREE_PHASE_PLL_H_
#define _FILE_THREE_PHASE_PLL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/coordinate/coord_trans.h>

/**
 * @brief Data structure for the three-phase PLL controller.
 */
typedef struct _tag_srf_pll_3ph
{
    //
    // Input Variables
    //
    vector2_gt e_ab; //!< Input voltage in the alpha-beta-axis of the stationary reference frame.

    //
    // Output Variables
    //
    ctrl_gt theta;        //!< Estimated grid angle, in per-unit format (0 to 1.0 represents 0 to 2*pi).
    ctl_vector2_t phasor; //!< Phasor corresponding to the estimated angle {cos(theta), sin(theta)}.
    ctrl_gt freq_pu;      //!< Estimated grid frequency, in per-unit format.
    ctrl_gt v_mag;        //!< Output voltage magnitude

    //
    // Intermediate Variables
    //
    ctrl_gt e_error; //!< The error signal (q-axis voltage) used by the PI controller.

    //
    // Parameters
    //
    ctrl_gt freq_sf; //!< Scaling factor to convert per-unit frequency to per-unit angle increment per tick.

    //
    // Internal Controller Objects
    //
    ctl_pid_t pid_pll; //!< PI controller for the phase-locking loop. Output is the frequency deviation.

} srf_pll_t;

/**
 * @brief Initializes the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] pll Pointer to the @ref srf_pll_t structure to be initialized.
 * @param[in] f_base The nominal grid frequency (e.g., 50 or 60 Hz), used as the per-unit base.
 * @param[in] pid_kp Proportional gain for the phase-locking PI controller.
 * @param[in] pid_Ti Integral time constant for the phase-locking PI controller (in seconds).
 * @param[in] pid_Td Derivative time constant (typically 0 for a PI controller).
 * @param[in] f_ctrl The controller's execution frequency (sampling frequency) in Hz.
 */
void ctl_init_sfr_pll_T(srf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_Ti,
                        parameter_gt pid_Td, parameter_gt f_ctrl);

/**
 * @brief Initializes the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] pll Pointer to the @ref srf_pll_t structure to be initialized.
 * @param[in] f_base The nominal grid frequency (e.g., 50 or 60 Hz), used as the per-unit base.
 * @param[in] pid_kp Proportional gain for the phase-locking PI controller.
 * @param[in] pid_ki Integral gain for the phase-locking PI controller (in seconds).
 * @param[in] pid_kd Derivative time constant (typically 0 for a PI controller).
 * @param[in] f_ctrl The controller's execution frequency (sampling frequency) in Hz.
 */
void ctl_init_sfr_pll(srf_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_ki,
                      parameter_gt pid_kd, parameter_gt f_ctrl);

/**
 * @brief Initializes the SRF-PLL by automatically calculating the PI parameters based on the desired bandwidth and damping ratio.
 * @param[out] pll            Pointer to the PLL structure to be initialized.
 * @param[in]  f_base         The nominal grid frequency in Hz (e.g., 50.0 or 60.0).
 * @param[in]  f_ctrl         The control loop execution frequency (sampling frequency) in Hz (e.g., 10000.0).
 * @param[in]  voltage_mag    The magnitude of the input voltage vector (sqrt(alpha^2 + beta^2)). Set this to 1.0 if the input voltages are already normalized (per-unit).
 * @param[in]  bandwidth_hz   The desired bandwidth of the PLL in Hz. Recommended range: 10.0 ~ 30.0 Hz.
 * @param[in]  damping_factor The damping ratio (zeta). Recommended value: 0.707.
 */
void ctl_init_srf_pll_auto_tune(srf_pll_t* pll, parameter_gt f_base, parameter_gt f_ctrl, parameter_gt voltage_mag,
                                parameter_gt bandwidth_hz, parameter_gt damping_factor);

/**
 * @brief Clears the internal states of the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 * @param[out] pll Pointer to the @ref srf_pll_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_pll_3ph(srf_pll_t* pll)
{
    ctl_vector2_clear(&pll->e_ab);

    pll->e_error = 0;
    pll->theta = 0;

    // Initialize frequency to nominal (1.0 p.u.).
    pll->freq_pu = float2ctrl(1.0);

    // Update the phasor based on the cleared angle.
    ctl_set_phasor_via_angle(pll->theta, &pll->phasor);

    // Clear the PI controller states.
    ctl_clear_pid(&pll->pid_pll);
}

/**
 * @brief Executes one step of the three-phase PLL algorithm.
 * @ingroup CTL_PLL_API
 * @details This function performs a Park transform on the input alpha-beta voltages
 * using the estimated angle, calculates the q-axis error, updates the PI controller,
 * and integrates the frequency to get the new angle.
 *
 * @param[out] pll Pointer to the @ref srf_pll_t structure.
 * @param[in] alpha The measured alpha-axis voltage.
 * @param[in] beta The measured beta-axis voltage.
 * @return The newly estimated grid angle (theta) in per-unit format.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pll_3ph(srf_pll_t* pll, ctrl_gt alpha, ctrl_gt beta)
{
    // Store input variables for monitoring.
    ctl_vector2_t vdq_temp;
    pll->e_ab.dat[phase_alpha] = alpha;
    pll->e_ab.dat[phase_beta] = beta;

    // 1. Generate the phasor {sin(theta), cos(theta)} from the current angle estimate.
    ctl_set_phasor_via_angle(pll->theta, &pll->phasor);

    // 2. Perform Park transform to get the q-axis voltage (error signal).
    // Vq = -Valpha * sin(theta) + Vbeta * cos(theta)
    ctl_ct_park2(&pll->e_ab, &pll->phasor, &vdq_temp);
    pll->e_error = vdq_temp.dat[phase_q];
    pll->v_mag = vdq_temp.dat[phase_d];

    // 3. Step the PI controller with the error signal to get the frequency deviation.
    ctl_step_pid_par(&pll->pid_pll, pll->e_error);

    // 4. Update the estimated frequency (nominal frequency + deviation).
    pll->freq_pu = float2ctrl(1.0) + pll->pid_pll.out;

    // 5. Integrate the frequency to update the angle (VCO - Voltage Controlled Oscillator).
    pll->theta += ctl_mul(pll->freq_pu, pll->freq_sf);
    pll->theta = ctrl_mod_1(pll->theta); // Wrap angle between 0 and 1.0.

    return pll->theta;
}

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

/**
 * @brief Data structure for the DSOGI-PLL controller.
 * @details This structure combines two SOGI blocks (for alpha and beta axes)
 * and an SRF-PLL to perform robust grid synchronization under distorted or
 * unbalanced grid conditions.
 */
typedef struct _tag_dsogi_pll
{
    //
    // Internal SOGI blocks
    //
    discrete_sogi_t sogi_alpha; //!< SOGI filter for the Alpha axis.
    discrete_sogi_t sogi_beta;  //!< SOGI filter for the Beta axis.

    //
    // Internal SRF-PLL block
    //
    srf_pll_t srf_pll; //!< The standard SRF-PLL instance.

    //
    // Intermediate Variables (Output of PNSC)
    //
    vector2_gt v_pos_seq; //!< Extracted positive sequence voltage {v_alpha+, v_beta+}.

} dsogi_pll_t;

/**
 * @brief Initializes the DSOGI-PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] dsogi    Pointer to the @ref dsogi_pll_t structure.
 * @param[in]  f_base   Nominal grid frequency (e.g., 50.0).
 * @param[in]  f_ctrl   Control loop frequency (e.g., 10000.0).
 * @param[in]  v_mag    Nominal voltage magnitude (usually 1.0 for per-unit).
 * @param[in]  bw_pll   Desired PLL bandwidth in Hz (e.g., 20.0).
 * @param[in]  k_sogi   SOGI damping factor (typically 1.414 or 1.0).
 */
void ctl_init_dsogi_pll(dsogi_pll_t* dsogi, parameter_gt f_base, parameter_gt f_ctrl, parameter_gt v_mag,
                        parameter_gt bw_pll, parameter_gt k_sogi);

/**
 * @brief Clears the internal states of the DSOGI-PLL controller.
 * @param[out] dsogi Pointer to the @ref dsogi_pll_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_dsogi_pll(dsogi_pll_t* dsogi)
{
    ctl_clear_discrete_sogi(&dsogi->sogi_alpha);
    ctl_clear_discrete_sogi(&dsogi->sogi_beta);
    ctl_clear_pll_3ph(&dsogi->srf_pll);
    ctl_vector2_clear(&dsogi->v_pos_seq);
}

/**
 * @brief Executes one step of the DSOGI-PLL algorithm.
 * @details
 * 1. Filters input voltages using SOGI (Band-Pass & Quadrature generation).
 * 2. Calculates Positive Sequence Components (PNSC).
 * 3. Feeds the clean positive sequence into the SRF-PLL.
 *
 * @param[in,out] dsogi Pointer to the @ref dsogi_pll_t structure.
 * @param[in]     alpha Raw measured alpha-axis voltage.
 * @param[in]     beta  Raw measured beta-axis voltage.
 * @return        The estimated grid angle (theta) in per-unit format.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_dsogi_pll(dsogi_pll_t* dsogi, ctrl_gt alpha, ctrl_gt beta)
{
    ctrl_gt v_alpha_p, v_alpha_q; // Alpha axis: In-phase & Quadrature
    ctrl_gt v_beta_p, v_beta_q;   // Beta axis:  In-phase & Quadrature

    // ---------------------------------------------------------
    // Step 1: Run SOGI Filters
    // ---------------------------------------------------------
    ctl_step_discrete_sogi(&dsogi->sogi_alpha, alpha);
    ctl_step_discrete_sogi(&dsogi->sogi_beta, beta);

    // Retrieve filtered signals
    // p = prime (in-phase/direct), q = quadrature (90 deg lag)
    v_alpha_p = ctl_get_discrete_sogi_ds(&dsogi->sogi_alpha);
    v_alpha_q = ctl_get_discrete_sogi_qs(&dsogi->sogi_alpha);

    v_beta_p = ctl_get_discrete_sogi_ds(&dsogi->sogi_beta);
    v_beta_q = ctl_get_discrete_sogi_qs(&dsogi->sogi_beta);

    // ---------------------------------------------------------
    // Step 2: Positive Sequence Calculation (PNSC)
    // Formula:
    // v_alpha+ = 0.5 * (v_alpha' - q * v_beta')
    // v_beta+  = 0.5 * (q * v_alpha' + v_beta')
    // ---------------------------------------------------------

    // v_pos_alpha = 0.5 * (v_alpha_p - v_beta_q)
    dsogi->v_pos_seq.dat[phase_alpha] = ctl_div2(v_alpha_p - v_beta_q);

    // v_pos_beta  = 0.5 * (v_alpha_q + v_beta_p)
    dsogi->v_pos_seq.dat[phase_beta] = ctl_div2(v_alpha_q + v_beta_p);

    // ---------------------------------------------------------
    // Step 3: Run SRF-PLL with Positive Sequence Voltage
    // ---------------------------------------------------------
    return ctl_step_pll_3ph(&dsogi->srf_pll, dsogi->v_pos_seq.dat[phase_alpha], dsogi->v_pos_seq.dat[phase_beta]);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_PLL_H_

/**
 * @}
 */
