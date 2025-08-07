/**
 * @file mtpa.h
 * @brief Implements a Maximum Torque Per Ampere (MTPA) current distributor.
 * @details This module is designed for salient-pole Permanent Magnet Synchronous
 * Motors (Ld != Lq). It takes a total torque-producing current command (Is)
 * from a higher-level controller (e.g., a speed loop) and calculates the
 * optimal d-axis (Id) and q-axis (Iq) current references that will produce
 * the maximum possible torque for that given total current magnitude. This
 * improves motor efficiency.
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * //tex:
 * // The calculation is based on the analytical solution for MTPA, which gives
 * // the optimal d-axis current (id) as a function of the total current (is):
 * // i_{d} = \frac{\psi_{f} - \sqrt{\psi_{f}^{2}+8\left(L_{d}-L_{q}\right)^{2}i_{s}^{2}}}{4\left(L_{q}-L_{d}\right)}
 * // The q-axis current is then calculated to maintain the current magnitude:
 * // i_q = \sqrt{i_s^2 - i_d^2}
 *
 */

#ifndef _FILE_PMSM_MTPA_H_
#define _FILE_PMSM_MTPA_H_

#include <math.h> // For sqrtf and fabsf

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* MTPA Current Distributor                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MTPA_DISTRIBUTOR MTPA Current Distributor
 * @brief Calculates optimal Id and Iq references for salient-pole PMSMs.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

#ifndef GMP_STATIC_INLINE
#define GMP_STATIC_INLINE static inline
#endif

// Define the standard control data type if not already defined
#ifndef CTRL_GT_DEFINED
#define CTRL_GT_DEFINED
typedef float ctrl_gt;
typedef float parameter_gt;
#endif

#define MTPA_SALIENT_POLE_THRESHOLD (1e-9f)

/**
 * @brief Main structure for the MTPA current distributor.
 */
typedef struct
{
    // --- Outputs ---
    ctrl_gt id_ref; ///< The calculated optimal d-axis current reference.
    ctrl_gt iq_ref; ///< The calculated optimal q-axis current reference.

    // --- Pre-calculated Parameters ---
    ctrl_gt psi_f;       ///< Permanent magnet flux linkage (Wb).
    ctrl_gt dL;          ///< Inductance difference (Ld - Lq).
    ctrl_gt psi_f_sq;    ///< Pre-squared flux linkage.
    ctrl_gt four_dL;     ///< Pre-calculated 4 * (Ld - Lq).
    ctrl_gt eight_dL_sq; ///< Pre-calculated 8 * (Ld - Lq)^2.
    uint8_t is_salient;  ///< Flag indicating if the motor is salient enough for MTPA.

} ctl_mtpa_distributor_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the MTPA distributor with motor parameters.
 * @details This function pre-calculates several constants based on the motor's
 * electrical parameters to speed up the real-time calculations.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  Ld D-axis inductance (H).
 * @param[in]  Lq Q-axis inductance (H).
 * @param[in]  psi_f Permanent magnet flux linkage (Wb).
 */
GMP_STATIC_INLINE void ctl_init_mtpa_distributor(ctl_mtpa_distributor_t* mtpa, parameter_gt Ld, parameter_gt Lq,
                                                 parameter_gt psi_f)
{
    mtpa->psi_f = (ctrl_gt)psi_f;
    mtpa->dL = (ctrl_gt)(Ld - Lq);

    // Check if the motor has significant saliency.
    if (fabsf(mtpa->dL) > MTPA_SALIENT_POLE_THRESHOLD)
    {
        mtpa->is_salient = 1;
        mtpa->psi_f_sq = mtpa->psi_f * mtpa->psi_f;
        mtpa->four_dL = 4.0f * mtpa->dL;
        mtpa->eight_dL_sq = 8.0f * mtpa->dL * mtpa->dL;
    }
    else
    {
        // For non-salient motors, MTPA is not applicable. Id should be zero.
        mtpa->is_salient = 0;
    }

    mtpa->id_ref = 0.0f;
    mtpa->iq_ref = 0.0f;
}

/**
 * @brief Executes one step of the MTPA current distribution.
 * @details Takes the total torque-producing current command and calculates the
 * optimal Id and Iq components.
 * @param[out] mtpa Pointer to the MTPA distributor structure.
 * @param[in]  is_cmd The signed, total torque-producing current command from the speed loop.
 */
GMP_STATIC_INLINE void ctl_step_mtpa_distributor(ctl_mtpa_distributor_t* mtpa, ctrl_gt is_cmd)
{
    if (!mtpa->is_salient)
    {
        // For non-salient motors (Ld=Lq), max torque is always at Id = 0.
        mtpa->id_ref = 0.0f;
        mtpa->iq_ref = is_cmd;
        return;
    }

    ctrl_gt is_mag = fabsf(is_cmd);
    ctrl_gt is_mag_sq = is_mag * is_mag;

    // 1. Calculate the optimal d-axis current using the analytical MTPA formula.
    // Note: The formula is rearranged to be numerically stable for both Ld > Lq and Lq > Ld.
    ctrl_gt term_under_sqrt = mtpa->psi_f_sq + mtpa->eight_dL_sq * is_mag_sq;
    ctrl_gt sqrt_term = sqrtf(term_under_sqrt);
    mtpa->id_ref = (mtpa->psi_f - sqrt_term) / (mtpa->four_dL);

    // 2. Calculate the corresponding q-axis current magnitude.
    ctrl_gt id_ref_sq = mtpa->id_ref * mtpa->id_ref;
    ctrl_gt iq_mag_sq = is_mag_sq - id_ref_sq;
    ctrl_gt iq_mag = (iq_mag_sq > 0.0f) ? sqrtf(iq_mag_sq) : 0.0f;

    // 3. Apply the original sign of the command to the q-axis current.
    mtpa->iq_ref = (is_cmd >= 0.0f) ? iq_mag : -iq_mag;
}

/**
 * @brief Gets the calculated d-axis current reference.
 * @param[in] mtpa Pointer to the MTPA distributor structure.
 * @return The optimal d-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpa_id_ref(const ctl_mtpa_distributor_t* mtpa)
{
    return mtpa->id_ref;
}

/**
 * @brief Gets the calculated q-axis current reference.
 * @param[in] mtpa Pointer to the MTPA distributor structure.
 * @return The optimal q-axis current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtpa_iq_ref(const ctl_mtpa_distributor_t* mtpa)
{
    return mtpa->iq_ref;
}

/** @} */ // end of MTPA_DISTRIBUTOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_MTPA_H_
