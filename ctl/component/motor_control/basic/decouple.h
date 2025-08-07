/**
 * @file decouple.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides voltage feed-forward decoupling functions for motor control.
 * @details This module contains functions to calculate the cross-coupling voltage
 * terms in the d-q reference frame for both PMSM and Induction Motors. These
 * feed-forward terms are essential for achieving high-performance current control.
 * Functions are provided for both SI unit and per-unit system calculations.
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
 * @{
 */

/**
 * @brief Calculates PMSM decoupling voltage feed-forward terms using SI units.
 * @details This function computes the cross-coupling terms that should be added to the
 * PI controller outputs to decouple the d and q axis dynamics.
 * Formulas:
 * //tex: V_{d,ff} = - \omega_e \cdot L_q \cdot i_q
 * //tex: V_{q,ff} =   \omega_e \cdot (L_d \cdot i_d + \psi_f)
 * @param[out] vdq_ff Pointer to the output feed-forward voltage vector (V).
 * @param[in] idq Pointer to the measured/reference current vector (A).
 * @param[in] lsd D-axis inductance in Henrys (H).
 * @param[in] lsq Q-axis inductance in Henrys (H).
 * @param[in] omega_e Electrical speed in rad/s.
 * @param[in] psi_e Permanent magnet flux linkage in Webers (Wb).
 */
GMP_STATIC_INLINE void ctl_mtr_pmsm_decouple_si(ctl_vector2_t* vdq_ff, const ctl_vector2_t* idq, ctrl_gt lsd,
                                                ctrl_gt lsq, ctrl_gt omega_e, ctrl_gt psi_e)
{
    vdq_ff->dat[phase_d] = -idq->dat[phase_q] * lsq * omega_e;
    vdq_ff->dat[phase_q] = (idq->dat[phase_d] * lsd + psi_e) * omega_e;
}

/**
 * @brief Calculates PMSM decoupling voltage feed-forward terms using per-unit values.
 * @details The calculation form is identical to the SI version, but all inputs and
 * outputs are in the per-unit system.
 * Formulas:
 * //tex: V_{d,ff,pu} = - \omega_{e,pu} \cdot L_{q,pu} \cdot i_{q,pu}
 * //tex: V_{q,ff,pu} =   \omega_{e,pu} \cdot (L_{d,pu} \cdot i_{d,pu} + \psi_{f,pu})
 * @param[out] vdq_ff_pu Pointer to the output feed-forward voltage vector (p.u.).
 * @param[in] idq_pu Pointer to the measured/reference current vector (p.u.).
 * @param[in] lsd_pu D-axis inductance (p.u.).
 * @param[in] lsq_pu Q-axis inductance (p.u.).
 * @param[in] omega_e_pu Electrical speed (p.u.).
 * @param[in] psi_e_pu Permanent magnet flux linkage (p.u.).
 */
GMP_STATIC_INLINE void ctl_mtr_pmsm_decouple_pu(ctl_vector2_t* vdq_ff_pu, const ctl_vector2_t* idq_pu, ctrl_gt lsd_pu,
                                                ctrl_gt lsq_pu, ctrl_gt omega_e_pu, ctrl_gt psi_e_pu)
{
    // The mathematical form is identical in the per-unit system.
    vdq_ff_pu->dat[phase_d] = -idq_pu->dat[phase_q] * lsq_pu * omega_e_pu;
    vdq_ff_pu->dat[phase_q] = (idq_pu->dat[phase_d] * lsd_pu + psi_e_pu) * omega_e_pu;
}

/** @} */ // end of MC_DECOUPLE_PMSM group

/*---------------------------------------------------------------------------*/
/* Decoupling Control for Induction Motor / RL Filter                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_DECOUPLE_ACM Induction Motor / RL Filter Decoupling Control
 * @brief Decoupling voltage calculation for Induction Motors or generic RL loads.
 * @{
 */

/**
 * @brief Calculates decoupling voltage feed-forward terms using SI units.
 * @details This function is applicable to induction motors (using transient inductance)
 * or any generic R-L load in a rotating reference frame.
 * Formulas:
 * //tex: V_{d,ff} = - \omega_e \cdot L_q \cdot i_q
 * //tex: V_{q,ff} =   \omega_e \cdot L_d \cdot i_d
 * @param[out] vdq_ff Pointer to the output feed-forward voltage vector (V).
 * @param[in] idq Pointer to the measured/reference current vector (A).
 * @param[in] lsd D-axis inductance in Henrys (H). For ACM, this is the transient inductance //tex: \sigma L_s.
 * @param[in] lsq Q-axis inductance in Henrys (H). For ACM, this is the transient inductance //tex: \sigma L_s.
 * @param[in] omega_e Electrical speed in rad/s.
 */
GMP_STATIC_INLINE void ctl_mtr_acm_decouple_si(ctl_vector2_t* vdq_ff, const ctl_vector2_t* idq, ctrl_gt lsd,
                                               ctrl_gt lsq, ctrl_gt omega_e)
{
    vdq_ff->dat[phase_d] = -idq->dat[phase_q] * lsq * omega_e;
    vdq_ff->dat[phase_q] = idq->dat[phase_d] * lsd * omega_e;
}

/**
 * @brief Calculates decoupling voltage feed-forward terms using per-unit values.
 * @details The calculation form is identical to the SI version, but all inputs and
 * outputs are in the per-unit system.
 * Formulas:
 * //tex: V_{d,ff,pu} = - \omega_{e,pu} \cdot L_{q,pu} \cdot i_{q,pu}
 * //tex: V_{q,ff,pu} =   \omega_{e,pu} \cdot L_{d,pu} \cdot i_{d,pu}
 * @param[out] vdq_ff_pu Pointer to the output feed-forward voltage vector (p.u.).
 * @param[in] idq_pu Pointer to the measured/reference current vector (p.u.).
 * @param[in] lsd_pu D-axis inductance (p.u.).
 * @param[in] lsq_pu Q-axis inductance (p.u.).
 * @param[in] omega_e_pu Electrical speed (p.u.).
 */
GMP_STATIC_INLINE void ctl_mtr_acm_decouple_pu(ctl_vector2_t* vdq_ff_pu, const ctl_vector2_t* idq_pu, ctrl_gt lsd_pu,
                                               ctrl_gt lsq_pu, ctrl_gt omega_e_pu)
{
    // The mathematical form is identical in the per-unit system.
    vdq_ff_pu->dat[phase_d] = -idq_pu->dat[phase_q] * lsq_pu * omega_e_pu;
    vdq_ff_pu->dat[phase_q] = idq_pu->dat[phase_d] * lsd_pu * omega_e_pu;
}

/** @} */ // end of MC_DECOUPLE_ACM group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MTR_CTRL_DECOUPLE_H_
