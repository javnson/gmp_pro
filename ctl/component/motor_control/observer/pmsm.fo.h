/**
 * @file pmsm_fo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a flux and torque observer for a Permanent Magnet Synchronous Motor (PMSM).
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_PMSM_FO_H_
#define _FILE_PMSM_FO_H_

#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup PMSM_FLUX_OBSERVER PMSM Flux and Torque Observer
 * @brief A module for estimating PMSM stator flux and electromagnetic torque.
 *
 * @details This module calculates the stator flux and electromagnetic torque based on the motor's
 * currents and rotor angle. It uses a sensored model, meaning it requires rotor position feedback.
 * The calculations are performed in the stationary ¦Á-¦Â reference frame using per-unit values.

 * This observer uses the following per-unit equations:
 *
 * **Flux Equations:**
 * @f[ \psi_\alpha^* = \omega_b L_s^* I_\alpha^* + \psi_{PM}^* \cos\theta @f]
 * @f[ \psi_\alpha^* = \omega_b L_s^* I_\alpha^* + \psi_{PM}^* \cos\theta @f]
 * @f[ \psi_\beta^* = \omega_b L_s^* I_\beta^* + \psi_{PM}^* \sin\theta @f]
 * @f[ \psi_\beta^* = \omega_b L_s^* I_\beta^* + \psi_{PM}^* \sin\theta @f]
 * @f[ \psi^* = (\psi_\alpha^{*\,2} + \psi_\beta^{*\,2})^{\frac{1}{2}} @f]
 * @f[ \psi^* = \sqrt{(\psi_\alpha^*)^2 + (\psi_\beta^*)^2} @f]
 *
 * **Torque Equation:**
 * @f[ T^* = \frac{1}{\psi_{PM}^*} (\psi_\alpha^* i_\beta^* - \psi_\beta^* i_\alpha^* ) @f]
 * @f[ T^* = \frac{1}{\psi_{PM}^*} (\psi_\alpha^* i_\beta^* - \psi_\beta^* i_\alpha^*) @f]
 *
 */

/*---------------------------------------------------------------------------*/
/* PMSM Flux and Torque Observer                                             */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup PMSM_FLUX_OBSERVER PMSM Flux Observer
 * @brief PMSM Observer of Flux.
 * @{
 */

/**
 * @brief Data structure for the PMSM flux observer.
 */
typedef struct _tag_pmsm_fo_t
{
    //
    // Outputs
    //
    vector2_gt flux;  /**< The estimated stator flux vector [¦Á, ¦Â] in per-unit. */
    ctrl_gt flux_mag; /**< The magnitude of the estimated stator flux in per-unit. */
    ctrl_gt torque;   /**< The estimated electromagnetic torque in per-unit. */

    //
    // Parameters
    //
    ctrl_gt wb_ls;      /**< The per-unit stator inductance multiplied by the base electrical frequency (¦Øb * Ls*). */
    ctrl_gt psi_pm;     /**< The per-unit permanent magnet flux linkage (¦×_PM*). */
    ctrl_gt inv_psi_pm; /**< The pre-calculated inverse of the per-unit PM flux (1 / ¦×_PM*). */

} pmsm_fo_t;

/**
 * @brief Initializes the PMSM flux observer object.
 * @param fo Pointer to the `pmsm_fo_t` object.
 * @param wb_ls_star The per-unit stator inductance multiplied by the base electrical frequency (¦Øb * Ls*).
 * @param psi_pm_star The per-unit permanent magnet flux linkage (¦×_PM*).
 */
void ctl_init_pmsm_fo(pmsm_fo_t* fo, ctrl_gt wb_ls_star, ctrl_gt psi_pm_star);

/**
 * @brief Executes one step of the flux and torque observation.
 * @param fo Pointer to the `pmsm_fo_t` object.
 * @param i_ab_star Pointer to a vector containing the ¦Á and ¦Â stator currents in per-unit.
 * @param sin_cos_theta Pointer to a vector containing the sine (dat[0]) and cosine (dat[1]) of the rotor electrical angle.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_fo(pmsm_fo_t* fo, const vector2_gt* i_ab_star, const vector2_gt* sin_cos_theta)
{
    ctrl_gt i_alpha = i_ab_star->dat[0];
    ctrl_gt i_beta = i_ab_star->dat[1];
    ctrl_gt sin_theta = sin_cos_theta->dat[0];
    ctrl_gt cos_theta = sin_cos_theta->dat[1];

    // Calculate stator flux components (psi_alpha, psi_beta)
    // ¦×_¦Á* = ¦Øb*Ls* * I_¦Á* + ¦×_PM* * cos(¦È)
    fo->flux.dat[0] = ctl_mac(ctl_mul(fo->wb_ls, i_alpha), fo->psi_pm, cos_theta);

    // ¦×_¦Â* = ¦Øb*Ls* * I_¦Â* + ¦×_PM* * sin(¦È)
    fo->flux.dat[1] = ctl_mac(ctl_mul(fo->wb_ls, i_beta), fo->psi_pm, sin_theta);

    // Calculate flux magnitude
    // ¦×* = sqrt( (¦×_¦Á*)^2 + (¦×_¦Â*)^2 )
    fo->flux_mag =
        ctl_sqrt(ctl_add(ctl_mul(fo->flux.dat[0], fo->flux.dat[0]), ctl_mul(fo->flux.dat[1], fo->flux.dat[1])));

    // Calculate electromagnetic torque
    // T* = (1/¦×_PM*) * (¦×_¦Á* * i_¦Â* - ¦×_¦Â* * i_¦Á*)
    ctrl_gt torque_term = ctl_sub(ctl_mul(fo->flux.dat[0], i_beta), ctl_mul(fo->flux.dat[1], i_alpha));
    fo->torque = ctl_mul(fo->inv_psi_pm, torque_term);
}

/**
 * @brief Gets the estimated stator flux vector (¦Á, ¦Â).
 * @param fo Pointer to the `pmsm_fo_t` object.
 * @return A pointer to the `vector2_gt` containing the flux components.
 */
GMP_STATIC_INLINE const vector2_gt* ctl_get_pmsm_fo_flux_vec(const pmsm_fo_t* fo)
{
    return &fo->flux;
}

/**
 * @brief Gets the magnitude of the estimated stator flux.
 * @param fo Pointer to the `pmsm_fo_t` object.
 * @return The flux magnitude in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_pmsm_fo_flux_mag(const pmsm_fo_t* fo)
{
    return fo->flux_mag;
}

/**
 * @brief Gets the estimated electromagnetic torque.
 * @param fo Pointer to the `pmsm_fo_t` object.
 * @return The torque in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_pmsm_fo_torque(const pmsm_fo_t* fo)
{
    return fo->torque;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_FO_H_
