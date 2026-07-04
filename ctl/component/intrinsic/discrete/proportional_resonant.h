/**
 * @file proportional_resonant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Universal Discrete Resonant (R, PR, QR, QPR) Controllers Component.
 * @details This file implements a complete library of parallel discrete resonant 
 * compensators for high-performance AC reference tracking and selective harmonic elimination.
 * @version 0.2
 * @date 2026-06-09
 * * @copyright Copyright GMP(c) 2026
 */

#ifndef _PROPORTIONAL_RESONANT_H_
#define _PROPORTIONAL_RESONANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup resonant_controllers Resonant Controllers
 * @brief Pure and Quasi-Resonant structural algorithms library.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Ideal Resonant (R) Controller                                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Packed Discrete Coefficient Structure for ideal Resonant (R) controller.
 */
typedef struct _tag_ctl_resonant_coef
{
    ctrl_gt b0; /**< Numerator weight for current input e[n]. */
    ctrl_gt b2; /**< Numerator weight for historical input e[n-2]. */
    ctrl_gt a1; /**< Denominator weight for historical output u[n-1]. */
    ctrl_gt a2; /**< Denominator weight for historical output u[n-2]. */
} ctl_resonant_coef_t;

/**
 * @brief Runtime execution instance for the pure Resonant (R) controller.
 * @details Implements an ideal resonant controller which provides infinite gain
 * at the resonant frequency.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = k_r \frac{2s}{s^2 + \omega_r^2} @f]
 *
 * After discretization using the Bilinear Transform, the transfer function is:
 * @f[ G(z) = k_r \frac{2T(1-z^{-2})}{(T^2\omega_r^2+4) - 2(T^2\omega_r^2-4)z^{-1} + (T^2\omega_r^2+4)z^{-2}} @f]
 */
typedef struct _tag_ctl_resonant_controller
{
    // State variables
    ctrl_gt output;   /**< Current execution step result output, u[n]. */
    ctrl_gt input_1;  /**< Historical error input delay buffer, e[n-1]. */
    ctrl_gt input_2;  /**< Historical error input delay buffer, e[n-2]. */
    ctrl_gt output_1; /**< Historical feedback output delay buffer, u[n-1]. */
    ctrl_gt output_2; /**< Historical feedback output delay buffer, u[n-2]. */

    /* Standardized Packed Coefficient Block */
    ctl_resonant_coef_t coef; /**< Standard active discrete parameter block. */
} resonant_ctrl_t;

/**
 * @brief Calculates ideal pure Resonant (R) coefficients using standard Tustin transform.
 * @param[out] coef Pointer to the destination pure resonant coefficient block.
 * @param[in] target_kr Expected resonant gain.
 * @param[in] target_freq_resonant Expected resonant frequency center (Hz).
 * @param[in] fs Sampling and execution frequency (Hz).
 */
void ctl_calc_resonant_ctrl_coef(ctl_resonant_coef_t* coef, parameter_gt target_kr, parameter_gt target_freq_resonant,
                                 parameter_gt fs);

/**
 * @brief Initializes a standalone ideal pure Resonant (R) controller instance.
 * * @param[out] r Pointer to the pure resonant controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency center (Hz).
 * @param[in] fs Sampling and execution frequency (Hz).
 */
void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Clears historical error inputs and feedback output accumulators of the R controller.
 * @param[out] r Pointer to the resonant controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_resonant_controller(resonant_ctrl_t* r)
{
    r->output = float2ctrl(0.0f);
    r->output_1 = float2ctrl(0.0f);
    r->output_2 = float2ctrl(0.0f);
    r->input_1 = float2ctrl(0.0f);
    r->input_2 = float2ctrl(0.0f);
}

/**
 * @brief Executes one step computation of the pure Resonant (R) controller.
 * @param[in,out] r Pointer to the resonant controller instance.
 * @param[in] input The current error tracking sample, e[n].
 * @return ctrl_gt The calculated resonant output, u[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_resonant_controller(resonant_ctrl_t* r, ctrl_gt input)
{
    /* Equation: u[n] = a1*u[n-1] + a2*u[n-2] + b0*e[n] + b2*e[n-2] */
    r->output = ctl_mul(r->coef.a1, r->output_1) + ctl_mul(r->coef.a2, r->output_2) + ctl_mul(r->coef.b0, input) +
                ctl_mul(r->coef.b2, r->input_2);

    /* Update historical delay lines pipelines */
    r->input_2 = r->input_1;
    r->input_1 = input;
    r->output_2 = r->output_1;
    r->output_1 = r->output;

    return r->output;
}

/*---------------------------------------------------------------------------*/
/* Proportional-Resonant (PR) Controller                                     */
/*---------------------------------------------------------------------------*/

/**
 * @brief Runtime execution instance for the parallel Proportional-Resonant (PR) controller.
 * @details Combines a proportional gain with a resonant controller.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = k_p + k_r \frac{2s}{s^2 + \omega_r^2} @f]
 */
typedef struct _tag_ctl_pr_controller
{
    resonant_ctrl_t resonant_part; /**< Imbedded pure resonant core block. */
    ctrl_gt kp;                    /**< Bounded proportional gain constant. */
} pr_ctrl_t;

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs);

/**
 * @brief Clears internal historical assets of the PR controller.
 * @param[out] pr Pointer to the PR controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pr_controller(pr_ctrl_t* pr)
{
    ctl_clear_resonant_controller(&pr->resonant_part);
}

/**
 * @brief Executes one step computation of the Proportional-Resonant (PR) controller.
 * @param[in,out] pr Pointer to the PR controller instance.
 * @param[in] input The current error tracking sample, e[n].
 * @return ctrl_gt The synthesized composite control effort.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pr_controller(pr_ctrl_t* pr, ctrl_gt input)
{
    // u(n) = Kp*e(n) + R(n)
    ctrl_gt p_out = ctl_mul(pr->kp, input);
    ctrl_gt r_out = ctl_step_resonant_controller(&pr->resonant_part, input);
    return p_out + r_out;
}

/*---------------------------------------------------------------------------*/
/* Quasi-Resonant (QR) Controller                                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Packed Discrete Coefficient Structure for Quasi-Resonant (QR) controller.
 */
typedef struct _tag_ctl_qr_coef
{
    ctrl_gt b0; /**< Numerator weight for current input e[n]. */
    ctrl_gt b2; /**< Numerator weight for historical input e[n-2]. */
    ctrl_gt a1; /**< Denominator weight for historical output u[n-1]. */
    ctrl_gt a2; /**< Denominator weight for historical output u[n-2]. */
} ctl_qr_coef_t;

/**
 * @brief Runtime execution instance for the non-ideal Quasi-Resonant (QR) controller.
 * @details A non-ideal resonant controller with finite gain at the resonant
 * frequency, which improves stability and robustness to frequency variations.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = k_r \frac{2\omega_c s}{s^2 + 2\omega_c s + \omega_r^2} @f]
 */
typedef struct _tag_ctl_qr_controller
{
    ctrl_gt output;   /**< Current execution step result output, u[n]. */
    ctrl_gt input_1;  /**< Historical error input delay buffer, e[n-1]. */
    ctrl_gt input_2;  /**< Historical error input delay buffer, e[n-2]. */
    ctrl_gt output_1; /**< Historical feedback output delay buffer, u[n-1]. */
    ctrl_gt output_2; /**< Historical feedback output delay buffer, u[n-2]. */

    ctl_qr_coef_t coef; /**< Standard active discrete parameter block. */
} qr_ctrl_t;

/**
 * @brief Helper function to calculate QR coefficients based on a specific K value.
 * @details Solves the Tustin substitution algebra.
 * Transfer Function: G(s) = Kr * (2*Wc*s) / (s^2 + 2*Wc*s + Wr^2)
 * Substitution: s = K * (1-z^-1)/(1+z^-1)
 * @param[out] coef Pointer to the destination QR coefficient block.
 * @param[in] kr Gain of the resonant term.
 * @param[in] wc Cutoff angular frequency (rad/s), sets the bandwidth.
 * @param[in] wr Resonant center angular frequency (rad/s).
 * @param[in] k_tustin The Tustin mapping factor (2*fs or prewarped K).
 */
void ctl_calc_qr_ctrl_coef(ctl_qr_coef_t* coef, parameter_gt kr, parameter_gt wc, parameter_gt wr,
                           parameter_gt k_tustin);

/**
 * @brief Initializes a quasi-resonant (QR) controller using standard Bilinear Tustin transform.
 * @note Suitable for tracking low-frequency AC references relative to the sampling frequency.
 * * @param[out] qr Pointer to the QR controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant center frequency (Hz).
 * @param[in] freq_cut Cutoff frequency / bandwidth half-width (Hz).
 * @param[in] fs Sampling and execution frequency (Hz).
 */
void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs);

/**
 * @brief Initializes a quasi-resonant (QR) controller with center frequency pre-warping.
 * @note Crucial for selective high-order harmonic elimination loops to correct continuous-to-discrete warping grid effects.
 * * @param[out] qr Pointer to the QR controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant center frequency (Hz).
 * @param[in] freq_cut Cutoff frequency / bandwidth half-width (Hz).
 * @param[in] fs Sampling and execution frequency (Hz).
 */
void ctl_init_qr_controller_prewarped(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                                      parameter_gt fs);

/**
 * @brief Clears historical error inputs and feedback output accumulators of the QR controller.
 * @param[out] qr Pointer to the quasi-resonant controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_qr_controller(qr_ctrl_t* qr)
{
    qr->output = float2ctrl(0.0f);
    qr->output_1 = float2ctrl(0.0f);
    qr->output_2 = float2ctrl(0.0f);
    qr->input_1 = float2ctrl(0.0f);
    qr->input_2 = float2ctrl(0.0f);
}

/**
 * @brief Executes one step computation of the Quasi-Resonant (QR) controller.
 * @param[in,out] qr Pointer to the quasi-resonant controller instance.
 * @param[in] input The current error tracking sample, e[n].
 * @return ctrl_gt The calculated quasi-resonant output, u[n].
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_qr_controller(qr_ctrl_t* qr, ctrl_gt input)
{
    /* Equation: u[n] = a1*u[n-1] + a2*u[n-2] + b0*e[n] + b2*e[n-2] */
    qr->output = ctl_mul(qr->coef.a1, qr->output_1) + ctl_mul(qr->coef.a2, qr->output_2) + ctl_mul(qr->coef.b0, input) +
                 ctl_mul(qr->coef.b2, qr->input_2);

    /* Update historical delay lines pipelines */
    qr->input_2 = qr->input_1;
    qr->input_1 = input;
    qr->output_2 = qr->output_1;
    qr->output_1 = qr->output;

    return qr->output;
}

/*---------------------------------------------------------------------------*/
/* Quasi-Proportional-Resonant (QPR) Controller                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Runtime execution instance for the Quasi-Proportional-Resonant (QPR) controller.
 * @details Combines a proportional gain with a quasi-resonant controller.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = K_p + K_r \frac{2\omega_c s}{s^2 + 2\omega_c s + \omega_r^2} @f]
 */
typedef struct _tag_ctl_qpr_controller
{
    qr_ctrl_t resonant_part; /**< Imbedded quasi-resonant core block. */
    ctrl_gt kp;              /**< Bounded proportional gain constant. */
} qpr_ctrl_t, ctl_qpr_t;

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs);
void ctl_init_qpr_controller_prewarped(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                                       parameter_gt freq_cut, parameter_gt fs);

/**
 * @brief Clears internal historical assets of the QPR controller.
 * @param[out] qpr Pointer to the QPR controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_qpr_controller(qpr_ctrl_t* qpr)
{
    ctl_clear_qr_controller(&qpr->resonant_part);
}

/**
 * @brief Executes one step computation of the Quasi-Proportional-Resonant (QPR) controller.
 * @param[in,out] qpr Pointer to the QPR controller instance.
 * @param[in] input The current error tracking sample, e[n].
 * @return ctrl_gt The synthesized composite control effort.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_qpr_controller(qpr_ctrl_t* qpr, ctrl_gt input)
{
    ctrl_gt p_out = ctl_mul(qpr->kp, input);
    ctrl_gt r_out = ctl_step_qr_controller(&qpr->resonant_part, input);
    return p_out + r_out;
}

/** 
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _PROPORTIONAL_RESONANT_H_
