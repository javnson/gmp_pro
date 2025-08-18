/**
 * @file decouple.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides voltage feed-forward decoupling functions for motor control.
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_MTR_CTRL_DECOUPLE_H_
#define _FILE_MTR_CTRL_DECOUPLE_H_

// This header is assumed to contain the definition for ctl_vector2_t and phase_d/q enums.
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Decoupling Control for PMSM                                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_DECOUPLE_PMSM PMSM Decoupling Control
 * @brief Decoupling voltage calculation for Permanent Magnet Synchronous Motors.
 * @details This module contains functions to calculate the cross-coupling voltage
 * terms in the d-q reference frame for both PMSM and Induction Motors. These
 * feed-forward terms are essential for achieving high-performance current control.
 * Functions are provided for both SI unit and per-unit system calculations.
 * @details This function computes the cross-coupling terms that should be added to the
 * PI controller outputs to decouple the d and q axis dynamics.
 * Formulas:
 * @f[ V_{d,ff} = - \omega_e \cdot L_q \cdot i_q            @f]
 * @f[ V_{q,ff} =   \omega_e \cdot (L_d \cdot i_d + \psi_f) @f]
 * @{
 */

/**
 * @brief Calculates PMSM decoupling voltage feed-forward terms using SI units.
 * @param[out] vdq_ff Pointer to the output feed-forward voltage vector (V, or p.u.).
 * @param[in] idq Pointer to the measured/reference current vector (A or p.u.).
 * @param[in] lsd D-axis inductance in Henrys (H, or p.u.).
 * @param[in] lsq Q-axis inductance in Henrys (H, or p.u.).
 * @param[in] omega_e Electrical speed in rad/s or p.u..
 * @param[in] psi_e Permanent magnet flux linkage in Webers (Wb or p.u.).
 */
GMP_STATIC_INLINE void ctl_mtr_pmsm_decouple(ctl_vector2_t* vdq_ff, const ctl_vector2_t* idq, ctrl_gt lsd, ctrl_gt lsq,
                                             ctrl_gt omega_e, ctrl_gt psi_e)
{
    vdq_ff->dat[phase_d] = -idq->dat[phase_q] * lsq * omega_e;
    vdq_ff->dat[phase_q] = (idq->dat[phase_d] * lsd + psi_e) * omega_e;
}

/**
 * @todo add PMSM decoupling calculator by consultant
 */

/** @} */ // end of MC_DECOUPLE_PMSM group

/*---------------------------------------------------------------------------*/
/* Decoupling Control for Induction Motor / RL Filter                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_DECOUPLE_ACM Induction Motor / RL Filter Decoupling Control
 * @ingroup CTL_MC_COMPONENT
 * @brief Decoupling voltage calculation for Induction Motors or generic RL loads.
 * @details This function is applicable to induction motors (using transient inductance)
 * or any generic R-L load in a rotating reference frame.
 * Formulas:
 * @f[ V_{d,ff} = - \omega_e \cdot L_q \cdot i_q @f]
 * @f[ V_{q,ff} =   \omega_e \cdot L_d \cdot i_d @f]
 * @{
 */

/**
 * @brief Calculates decoupling voltage feed-forward terms using SI units or per-unit units.
 * @param[out] vdq_ff Pointer to the output feed-forward voltage vector (V).
 * @param[in] idq Pointer to the measured/reference current vector (A).
 * @param[in] ls_sigma D-axis inductance in Henrys (H). For ACM, this is the transient inductance @f( \sigma L_s @f).
 * @param[in] omega_e Electrical speed in rad/s.
 */
GMP_STATIC_INLINE void ctl_mtr_acm_decouple(ctl_vector2_t* vdq_ff, const ctl_vector2_t* idq, ctrl_gt ls_sigma,
                                            ctrl_gt omega_e)
{
    vdq_ff->dat[phase_d] = -idq->dat[phase_q] * ls_sigma * omega_e;
    vdq_ff->dat[phase_q] = idq->dat[phase_d] * ls_sigma * omega_e;
}

/** @} */ // end of MC_DECOUPLE_ACM group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MTR_CTRL_DECOUPLE_H_
