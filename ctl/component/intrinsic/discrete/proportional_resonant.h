/**
 * @file proportional_resonant.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides discrete Proportional-Resonant (PR) and Quasi-Proportional-Resonant (QPR) controllers.
 * @version 0.1
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _PROPORTIONAL_RESONANT_H_
#define _PROPORTIONAL_RESONANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup resonant_controllers Resonant Controllers
 * @brief A library of discrete resonant controllers for AC signal tracking.
 * @details This file implements several variations of resonant controllers, which are
 * essential for AC signal tracking, particularly in applications like grid-tied
 * inverters. They provide infinite gain at a specific resonant frequency,
 * allowing for zero steady-state error when tracking a sinusoidal reference.
 * Implementations include pure Resonant (R), Proportional-Resonant (PR),
 * Quasi-Resonant (QR), and Quasi-Proportional-Resonant (QPR) controllers.

 * @{
 */

/*---------------------------------------------------------------------------*/
/* Resonant (R) Controller                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a pure Resonant (R) controller.
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
    ctrl_gt output;   //!< Current controller output, u(n).
    ctrl_gt input_1;  //!< Previous input, e(n-1).
    ctrl_gt input_2;  //!< Input from two steps ago, e(n-2).
    ctrl_gt output_1; //!< Previous output, u(n-1).
    ctrl_gt output_2; //!< Output from two steps ago, u(n-2).

    // Coefficients
    ctrl_gt b0; //!< Numerator coefficient for e(n).
    ctrl_gt b2; //!< Numerator coefficient for e(n-2).
    ctrl_gt a1; //!< Denominator coefficient for u(n-1).
    ctrl_gt a2; //!< Denominator coefficient for u(n-2).
} resonant_ctrl_t;

/**
 * @brief Initializes a resonant controller.
 * @param[out] r Pointer to the resonant controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs);

/**
 * @brief Clears the internal states of the resonant controller.
 * @param[out] r Pointer to the resonant controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_resonant_controller(resonant_ctrl_t* r)
{
    r->output = 0;
    r->output_1 = 0;
    r->output_2 = 0;
    r->input_1 = 0;
    r->input_2 = 0;
}

/**
 * @brief Executes one step of the resonant controller.
 * @param[in,out] r Pointer to the resonant controller instance.
 * @param[in] input The current input, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_resonant_controller(resonant_ctrl_t* r, ctrl_gt input)
{
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    r->output =
        ctl_mul(r->a1, r->output_1) + ctl_mul(r->a2, r->output_2) + ctl_mul(r->b0, input) + ctl_mul(r->b2, r->input_2);

    // Update states
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
 * @brief Data structure for a Proportional-Resonant (PR) controller.
 * @details Combines a proportional gain with a resonant controller.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = k_p + k_r \frac{2s}{s^2 + \omega_r^2} @f]
 */
typedef struct _tag_ctl_pr_controller
{
    resonant_ctrl_t resonant_part; //!< The resonant part of the controller.
    parameter_gt kp;               //!< The proportional gain.
} pr_ctrl_t;

/**
 * @brief Initializes a PR controller.
 * @param[out] pr Pointer to the PR controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs);

/**
 * @brief Clears the internal states of the PR controller.
 * @param[out] pr Pointer to the PR controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pr_controller(pr_ctrl_t* pr)
{
    ctl_clear_resonant_controller(&pr->resonant_part);
}

/**
 * @brief Executes one step of the PR controller.
 * @param[in,out] pr Pointer to the PR controller instance.
 * @param[in] input The current input, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pr_controller(pr_ctrl_t* pr, ctrl_gt input)
{
    // u(n) = Kp*e(n) + R(n)
    ctrl_gt p_out = ctl_mul(float2ctrl(pr->kp), input);
    ctrl_gt r_out = ctl_step_resonant_controller(&pr->resonant_part, input);
    return p_out + r_out;
}

/*---------------------------------------------------------------------------*/
/* Quasi-Resonant (QR) Controller                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a Quasi-Resonant (QR) controller.
 * @details A non-ideal resonant controller with finite gain at the resonant
 * frequency, which improves stability and robustness to frequency variations.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = k_r \frac{2\omega_c s}{s^2 + 2\omega_c s + \omega_r^2} @f]
 */
typedef struct _tag_ctl_qr_controller
{
    // State variables
    ctrl_gt output;   //!< Current controller output, u(n).
    ctrl_gt input_1;  //!< Previous input, e(n-1).
    ctrl_gt input_2;  //!< Input from two steps ago, e(n-2).
    ctrl_gt output_1; //!< Previous output, u(n-1).
    ctrl_gt output_2; //!< Output from two steps ago, u(n-2).

    // Coefficients
    ctrl_gt b0; //!< Numerator coefficient for e(n).
    ctrl_gt b2; //!< Numerator coefficient for e(n-2).
    ctrl_gt a1; //!< Denominator coefficient for u(n-1).
    ctrl_gt a2; //!< Denominator coefficient for u(n-2).
} qr_ctrl_t;

/**
 * @brief Initializes a quasi-resonant controller using Standard Tustin.
 * @param[out] qr Pointer to the QR controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz.
 * @param[in] freq_cut Cutoff frequency in Hz, which sets the controller's bandwidth.
 * @param[in] fs Sampling frequency in Hz.
 * @note  Use this only for low frequency resonances relative to Fs.
 */
void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs);

/**
 * @brief Initializes a quasi-resonant controller with Frequency Pre-warping.
 * @details Corrects the frequency warping effect of bilinear transformation at the resonant frequency.
 * Essential for harmonic control (e.g., 6th, 12th harmonics).
 * @param[out] qr Pointer to the QR controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz (Center Frequency).
 * @param[in] freq_cut Cutoff frequency in Hz (Bandwidth/2).
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_qr_controller_prewarped(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                                      parameter_gt fs);

/**
 * @brief Clears the internal states of the QR controller.
 * @param[out] qr Pointer to the QR controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_qr_controller(qr_ctrl_t* qr)
{
    qr->output = 0;
    qr->output_1 = 0;
    qr->output_2 = 0;
    qr->input_1 = 0;
    qr->input_2 = 0;
}

/**
 * @brief Executes one step of the QR controller.
 * @param[in,out] qr Pointer to the QR controller instance.
 * @param[in] input The current input, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_qr_controller(qr_ctrl_t* qr, ctrl_gt input)
{
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    qr->output = ctl_mul(qr->a1, qr->output_1) + ctl_mul(qr->a2, qr->output_2) + ctl_mul(qr->b0, input) +
                 ctl_mul(qr->b2, qr->input_2);

    // Update states
    qr->input_2 = qr->input_1;
    qr->input_1 = input;
    qr->output_2 = qr->output_1;
    qr->output_1 = qr->output;

    return qr->output;
}

/*---------------------------------------------------------------------------*/
/* Quasi-Proportional-Resonant (QPR) Controller                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for a Quasi-Proportional-Resonant (QPR) controller.
 * @details Combines a proportional gain with a quasi-resonant controller.
 *
 * Continuous-time transfer function:
 * @f[ G(s) = K_p + K_r \frac{2\omega_c s}{s^2 + 2\omega_c s + \omega_r^2} @f]
 */
typedef struct _tag_ctl_qpr_controller
{
    qr_ctrl_t resonant_part; //!< The quasi-resonant part of the controller.
    parameter_gt kp;         //!< The proportional gain.
} qpr_ctrl_t;

/**
 * @brief Initializes a QPR controller using Standard Tustin.
 * @param[out] qpr Pointer to the QPR controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz.
 * @param[in] freq_cut Cutoff frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs);

/**
 * @brief Initializes a QPR controller with Frequency Pre-warping.
 * @param[out] qpr Pointer to the QPR controller instance.
 * @param[in] kp Proportional gain.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz.
 * @param[in] freq_cut Cutoff frequency in Hz.
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_qpr_controller_prewarped(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                                       parameter_gt freq_cut, parameter_gt fs);

/**
 * @brief Clears the internal states of the QPR controller.
 * @param[out] qpr Pointer to the QPR controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_qpr_controller(qpr_ctrl_t* qpr)
{
    ctl_clear_qr_controller(&qpr->resonant_part);
}

/**
 * @brief Executes one step of the QPR controller.
 * @param[in,out] qpr Pointer to the QPR controller instance.
 * @param[in] input The current input, e(n).
 * @return ctrl_gt The calculated output, u(n).
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_qpr_controller(qpr_ctrl_t* qpr, ctrl_gt input)
{
    // u(n) = Kp*e(n) + QR(n)
    ctrl_gt p_out = ctl_mul(float2ctrl(qpr->kp), input);
    ctrl_gt r_out = ctl_step_qr_controller(&qpr->resonant_part, input);
    return p_out + r_out;
}

/**
 * @}
 */ // end of resonant_controllers group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _PROPORTIONAL_RESONANT_H_
