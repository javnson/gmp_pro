/**
 * @file pll.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a three-phase Synchronous Reference Frame PLL (SRF-PLL).
 * @version 1.1
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 *
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
typedef struct _tag_pll_3ph
{
    //
    // Input Variables
    //
    ctrl_gt e_alpha; //!< Input voltage in the alpha-axis of the stationary reference frame.
    ctrl_gt e_beta;  //!< Input voltage in the beta-axis of the stationary reference frame.

    //
    // Output Variables
    //
    ctrl_gt theta;        //!< Estimated grid angle, in per-unit format (0 to 1.0 represents 0 to 2*pi).
    ctl_vector2_t phasor; //!< Phasor corresponding to the estimated angle {cos(theta), sin(theta)}.
    ctrl_gt freq_pu;      //!< Estimated grid frequency, in per-unit format.

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

} three_phase_pll_t;

// Forward declaration for the inline function
GMP_STATIC_INLINE void ctl_clear_pll_3ph(three_phase_pll_t* pll);

/**
 * @brief Initializes the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 *
 * @param[out] pll Pointer to the @ref three_phase_pll_t structure to be initialized.
 * @param[in] f_base The nominal grid frequency (e.g., 50 or 60 Hz), used as the per-unit base.
 * @param[in] pid_kp Proportional gain for the phase-locking PI controller.
 * @param[in] pid_Ti Integral time constant for the phase-locking PI controller (in seconds).
 * @param[in] pid_Td Derivative time constant (typically 0 for a PI controller).
 * @param[in] f_ctrl The controller's execution frequency (sampling frequency) in Hz.
 */
void ctl_init_pll_3ph(three_phase_pll_t* pll, parameter_gt f_base, parameter_gt pid_kp, parameter_gt pid_Ti,
                      parameter_gt pid_Td, parameter_gt f_ctrl);

/**
 * @brief Clears the internal states of the three-phase PLL controller.
 * @ingroup CTL_PLL_API
 * @param[out] pll Pointer to the @ref three_phase_pll_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_pll_3ph(three_phase_pll_t* pll)
{
    pll->e_alpha = 0;
    pll->e_beta = 0;
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
 * @param[out] pll Pointer to the @ref three_phase_pll_t structure.
 * @param[in] alpha The measured alpha-axis voltage.
 * @param[in] beta The measured beta-axis voltage.
 * @return The newly estimated grid angle (theta) in per-unit format.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pll_3ph(three_phase_pll_t* pll, ctrl_gt alpha, ctrl_gt beta)
{
    // Store input variables for monitoring.
    pll->e_alpha = alpha;
    pll->e_beta = beta;

    // 1. Generate the phasor {cos(theta), sin(theta)} from the current angle estimate.
    ctl_set_phasor_via_angle(pll->theta, &pll->phasor);

    // 2. Perform Park transform to get the q-axis voltage (error signal).
    // Vq = -Valpha * sin(theta) + Vbeta * cos(theta)
    // CORRECTED: The sign of the second term was wrong, leading to positive feedback.
    pll->e_error = ctl_mul(pll->e_beta, pll->phasor.dat[0]) - ctl_mul(pll->e_alpha, pll->phasor.dat[1]);

    // 3. Step the PI controller with the error signal to get the frequency deviation.
    ctl_step_pid_par(&pll->pid_pll, pll->e_error);

    // 4. Update the estimated frequency (nominal frequency + deviation).
    pll->freq_pu = float2ctrl(1.0) + pll->pid_pll.out;

    // 5. Integrate the frequency to update the angle (VCO - Voltage Controlled Oscillator).
    pll->theta += ctl_mul(pll->freq_pu, pll->freq_sf);
    pll->theta = ctrl_mod_1(pll->theta); // Wrap angle between 0 and 1.0.

    return pll->theta;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_PLL_H_

/**
 * @}
 */
