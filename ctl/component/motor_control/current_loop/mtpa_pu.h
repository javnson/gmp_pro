/**
 * @file mtpa.h
 * @brief Implements a Maximum Torque Per Ampere (MTPA) current distributor.
 * 
 * @version 0.3
 * @date 2025-08-06
 *
 */

#ifndef _FILE_PMSM_MTPA_PU_H_
#define _FILE_PMSM_MTPA_PU_H_

#include <math.h> // For sqrtf and fabsf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MTPA Current Distributor                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MTPA_DISTRIBUTOR_PU MTPA Current Distributor (Per Unit)
 * @brief Calculates optimal Id and Iq references for salient-pole PMSMs.
 * @details This module is designed for salient-pole Permanent Magnet Synchronous
 * Motors (Ld != Lq). It takes a total torque-producing current command (Is)
 * and calculates the optimal d-axis (Id) and q-axis (Iq) current references
 * that produce the maximum torque for a given current magnitude. This improves
 * motor efficiency. Functions are provided for both SI and per-unit systems.
 * The calculation is based on the analytical solution for MTPA, which gives
 * the optimal d-axis current (id) as a function of the total current (is):
 * @f[ i_{d} = \frac{\psi_{f} - \sqrt{\psi_{f}^{2}+8\left(L_{d}-L_{q}\right)^{2}i_{s}^{2}}}{4\left(L_{d}-L_{q}\right)} @f]
 * The q-axis current is then calculated to maintain the current magnitude:
 * @f[ i_q = \sqrt{i_s^2 - i_d^2} @f] 
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

#define MTPA_SALIENT_POLE_THRESHOLD (1e-9f)

/**
 * @brief Main structure for the MTPA current distributor.
 * @details This structure can be initialized with either SI or per-unit values.
 * The step function must match the initialization type.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt id_ref; ///< The calculated optimal d-axis current reference (SI or p.u.).
    ctrl_gt iq_ref; ///< The calculated optimal q-axis current reference (SI or p.u.).

    // --- Pre-calculated Parameters ---
    ctrl_gt psi_f;       ///< Permanent magnet flux linkage (Wb or p.u.).
    ctrl_gt dL;          ///< Inductance difference (Ld - Lq) (H or p.u.).
    ctrl_gt psi_f_sq;    ///< Pre-squared flux linkage.
    ctrl_gt four_dL;     ///< Pre-calculated 4 * dL.
    ctrl_gt eight_dL_sq; ///< Pre-calculated 8 * dL^2.
    uint8_t is_salient;  ///< Flag indicating if the motor is salient enough for MTPA.

} ctl_mtpa_distributor_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @defgroup MTPA_SI SI Unit Implementation
 * @ingroup MTPA_DISTRIBUTOR
 * @brief MTPA functions operating with standard SI units.
 * @{
 */

/**
 * @brief Initializes the MTPA distributor with SI motor parameters.
 * @details This function pre-calculates several constants based on the motor's
 * electrical parameters to speed up the real-time calculations.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  Ld D-axis inductance in Henrys (H).
 * @param[in]  Lq Q-axis inductance in Henrys (H).
 * @param[in]  psi_f Permanent magnet flux linkage in Webers (Wb).
 */
void ctl_init_mtpa_distributor_si(ctl_mtpa_distributor_t* mtpa, parameter_gt Ld, parameter_gt Lq, parameter_gt psi_f);

/**
 * @brief Executes one step of the MTPA current distribution using SI units.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  is_cmd The signed, total torque-producing current command in Amperes (A).
 */
GMP_STATIC_INLINE void ctl_step_mtpa_distributor_si(ctl_mtpa_distributor_t* mtpa, ctrl_gt is_cmd)
{
    if (!mtpa->is_salient)
    {
        mtpa->id_ref = 0.0f;
        mtpa->iq_ref = is_cmd;
        return;
    }

    ctrl_gt is_mag = fabsf(is_cmd);
    ctrl_gt is_mag_sq = is_mag * is_mag;

    ctrl_gt term_under_sqrt = mtpa->psi_f_sq + mtpa->eight_dL_sq * is_mag_sq;
    ctrl_gt sqrt_term = sqrtf(term_under_sqrt);
    mtpa->id_ref = (mtpa->psi_f - sqrt_term) / mtpa->four_dL;

    ctrl_gt id_ref_sq = mtpa->id_ref * mtpa->id_ref;
    ctrl_gt iq_mag_sq = is_mag_sq - id_ref_sq;
    ctrl_gt iq_mag = (iq_mag_sq > 0.0f) ? sqrtf(iq_mag_sq) : 0.0f;

    mtpa->iq_ref = (is_cmd >= 0.0f) ? iq_mag : -iq_mag;
}

/** @} */ // end of MTPA_SI group

/**
 * @defgroup MTPA_PU Per-Unit Implementation
 * @ingroup MTPA_DISTRIBUTOR
 * @brief MTPA functions operating with per-unit (p.u.) values.
 * @{
 */

/**
 * @brief Initializes the MTPA distributor with per-unit motor parameters.
 * @details This function pre-calculates several constants based on the motor's
 * per-unit electrical parameters to speed up the real-time calculations.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  Ld_pu D-axis inductance (p.u.).
 * @param[in]  Lq_pu Q-axis inductance (p.u.).
 * @param[in]  psi_f_pu Permanent magnet flux linkage (p.u.).
 */
void ctl_init_mtpa_distributor_pu(ctl_mtpa_distributor_t* mtpa, parameter_gt Ld_pu, parameter_gt Lq_pu,
                                  parameter_gt psi_f_pu);

/**
 * @brief Executes one step of the MTPA current distribution using per-unit values.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  is_cmd_pu The signed, total torque-producing current command (p.u.).
 */
GMP_STATIC_INLINE void ctl_step_mtpa_distributor_pu(ctl_mtpa_distributor_t* mtpa, ctrl_gt is_cmd_pu)
{
    // The mathematical form is identical in the per-unit system.
    if (!mtpa->is_salient)
    {
        mtpa->id_ref = 0.0f;
        mtpa->iq_ref = is_cmd_pu;
        return;
    }

    ctrl_gt is_mag = fabsf(is_cmd_pu);
    ctrl_gt is_mag_sq = is_mag * is_mag;

    ctrl_gt term_under_sqrt = mtpa->psi_f_sq + mtpa->eight_dL_sq * is_mag_sq;
    ctrl_gt sqrt_term = sqrtf(term_under_sqrt);
    mtpa->id_ref = (mtpa->psi_f - sqrt_term) / mtpa->four_dL;

    ctrl_gt id_ref_sq = mtpa->id_ref * mtpa->id_ref;
    ctrl_gt iq_mag_sq = is_mag_sq - id_ref_sq;
    ctrl_gt iq_mag = (iq_mag_sq > 0.0f) ? sqrtf(iq_mag_sq) : 0.0f;

    mtpa->iq_ref = (is_cmd_pu >= 0.0f) ? iq_mag : -iq_mag;
}

/** @} */ // end of MTPA_PU group

/**
 * @brief Gets the calculated d-axis current reference.
 * @param[in] mtpa Pointer to the MTPA distributor structure.
 * @return The optimal d-axis current reference (SI or p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpa_id_ref(const ctl_mtpa_distributor_t* mtpa)
{
    return mtpa->id_ref;
}

/**
 * @brief Gets the calculated q-axis current reference.
 * @param[in] mtpa Pointer to the MTPA distributor structure.
 * @return The optimal q-axis current reference (SI or p.u.).
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpa_iq_ref(const ctl_mtpa_distributor_t* mtpa)
{
    return mtpa->iq_ref;
}

/** 
 * @} 
 */ // end of MTPA_DISTRIBUTOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_MTPA_H_
