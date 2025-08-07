/**
 * @file mtpv_pu.h
 * @brief Implements a per-unit MTPV (Field Weakening) controller for PMSM.
 * @details This module calculates the optimal d-q axis current references to achieve
 * the maximum possible torque when the motor operates above its base speed and
 * is limited by the available bus voltage. It injects a negative d-axis
 * current to counteract the back-EMF, allowing for higher speed operation.
 * All calculations are performed in the per-unit system.
 *
 * @version 1.0
 * @date 2025-08-06
 *
 * //tex:
 * // The MTPV algorithm operates on the voltage limit circle defined by:
 * // U_{max,pu}^2 = (R_{s,pu} i_{d,pu} - \omega_{e,pu} L_{q,pu} i_{q,pu})^2 + (R_{s,pu} i_{q,pu} + \omega_{e,pu} L_{d,pu} i_{d,pu} + \omega_{e,pu} \psi_{f,pu})^2
 *
 */

#ifndef _FILE_PMSM_MTPV_PU_H_
#define _FILE_PMSM_MTPV_PU_H_

#include <math.h> // For sqrtf and fabsf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MTPV (Field Weakening) Per-Unit Controller                                */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MTPV_CONTROLLER_PU MTPV / Field Weakening Controller (Per-Unit)
 * @brief Calculates optimal Id and Iq references for high-speed operation using per-unit values.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

#ifndef GMP_STATIC_INLINE
#define GMP_STATIC_INLINE static inline
#endif

#ifndef CTRL_GT_DEFINED
#define CTRL_GT_DEFINED
typedef float ctrl_gt;
typedef float parameter_gt;
#endif

/**
 * @brief Main structure for the MTPV per-unit controller.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt id_ref_pu; ///< The calculated field-weakening d-axis current reference (p.u.).
    ctrl_gt iq_ref_pu; ///< The calculated q-axis current reference under voltage limit (p.u.).

    // --- Pre-calculated Per-Unit Parameters ---
    ctrl_gt rs_pu;    ///< Stator Resistance (p.u.).
    ctrl_gt ld_pu;    ///< D-axis Inductance (p.u.).
    ctrl_gt lq_pu;    ///< Q-axis Inductance (p.u.).
    ctrl_gt psi_f_pu; ///< Flux Linkage (p.u.).
    ctrl_gt rs_sq_pu; ///< R_s^2 (p.u.).
    ctrl_gt ld_sq_pu; ///< L_d^2 (p.u.).
    ctrl_gt lq_sq_pu; ///< L_q^2 (p.u.).

} ctl_mtpv_pu_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the MTPV per-unit controller with motor parameters.
 * @param[out] mtpv Pointer to the MTPV controller structure.
 * @param[in]  Rs_pu Stator Resistance (p.u.).
 * @param[in]  Ld_pu D-axis Inductance (p.u.).
 * @param[in]  Lq_pu Q-axis Inductance (p.u.).
 * @param[in]  psi_f_pu Permanent magnet flux linkage (p.u.).
 */
GMP_STATIC_INLINE void ctl_init_mtpv_pu(ctl_mtpv_pu_controller_t* mtpv, parameter_gt Rs_pu, parameter_gt Ld_pu,
                                        parameter_gt Lq_pu, parameter_gt psi_f_pu)
{
    mtpv->rs_pu = (ctrl_gt)Rs_pu;
    mtpv->ld_pu = (ctrl_gt)Ld_pu;
    mtpv->lq_pu = (ctrl_gt)Lq_pu;
    mtpv->psi_f_pu = (ctrl_gt)psi_f_pu;

    // Pre-calculate squared terms for efficiency
    mtpv->rs_sq_pu = mtpv->rs_pu * mtpv->rs_pu;
    mtpv->ld_sq_pu = mtpv->ld_pu * mtpv->ld_pu;
    mtpv->lq_sq_pu = mtpv->lq_pu * mtpv->lq_pu;

    mtpv->id_ref_pu = 0.0f;
    mtpv->iq_ref_pu = 0.0f;
}

/**
 * @brief Executes one step of the MTPV (Field Weakening) calculation in per-unit.
 * @details This function should be called when the commanded voltage from the
 * current loop exceeds the available DC bus voltage.
 * @param[out] mtpv      Pointer to the MTPV controller structure.
 * @param[in]  iq_cmd_pu The original q-axis current command from the speed loop (p.u.).
 * @param[in]  u_max_pu  The maximum available voltage magnitude (p.u.).
 * @param[in]  omega_e_pu The current electrical speed (p.u.).
 */
GMP_STATIC_INLINE void ctl_step_mtpv_pu(ctl_mtpv_pu_controller_t* mtpv, ctrl_gt iq_cmd_pu, ctrl_gt u_max_pu,
                                        ctrl_gt omega_e_pu)
{
    ctrl_gt we_sq = omega_e_pu * omega_e_pu;
    ctrl_gt u_max_sq = u_max_pu * u_max_pu;

    // --- Calculate terms for the quadratic equation of Id: A*id^2 + B*id + C = 0 ---
    ctrl_gt A = we_sq * mtpv->ld_sq_pu + mtpv->rs_sq_pu;
    ctrl_gt B = 2.0f * we_sq * mtpv->ld_pu * mtpv->psi_f_pu;
    ctrl_gt C = we_sq * mtpv->psi_f_pu * mtpv->psi_f_pu + we_sq * mtpv->lq_sq_pu * iq_cmd_pu * iq_cmd_pu +
                mtpv->rs_sq_pu * iq_cmd_pu * iq_cmd_pu + 2.0f * mtpv->rs_pu * omega_e_pu * mtpv->psi_f_pu * iq_cmd_pu -
                u_max_sq;

    // --- Solve the quadratic equation for Id ---
    ctrl_gt discriminant = B * B - 4.0f * A * C;

    if (discriminant >= 0.0f)
    {
        // A valid solution exists. We choose the negative Id for field weakening.
        // The solution with the smaller magnitude (closer to zero) is typically desired.
        mtpv->id_ref_pu = (-B + sqrtf(discriminant)) / (2.0f * A);
    }
    else
    {
        // No real solution, indicates extreme condition where the voltage ellipse
        // cannot be reached even with maximum field weakening.
        // Clamp Id to the characteristic current, which is the theoretical maximum field-weakening current.
        mtpv->id_ref_pu = -mtpv->psi_f_pu / mtpv->ld_pu;
    }

    // The q-axis current is the original command from the speed loop.
    // In a full implementation, Iq might also be adjusted to stay within the overall current limit circle.
    mtpv->iq_ref_pu = iq_cmd_pu;
}

/**
 * @brief Gets the calculated d-axis current reference for field weakening.
 * @param[in] mtpv Pointer to the MTPV controller structure.
 * @return The optimal d-axis current reference (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpv_id_ref_pu(const ctl_mtpv_pu_controller_t* mtpv)
{
    return mtpv->id_ref_pu;
}

/**
 * @brief Gets the q-axis current reference for field weakening.
 * @param[in] mtpv Pointer to the MTPV controller structure.
 * @return The optimal q-axis current reference (p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpv_iq_ref_pu(const ctl_mtpv_pu_controller_t* mtpv)
{
    return mtpv->iq_ref_pu;
}

/** @} */ // end of MTPV_CONTROLLER_PU group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_MTPV_PU_H_
