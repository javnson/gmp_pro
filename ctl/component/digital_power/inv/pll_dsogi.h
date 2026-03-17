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


#include <ctl/component/intrinsic/discrete/discrete_sogi.h>
#include <ctl/component/digital_power/inv/pll_srf.h>


#ifndef _FILE_THREE_PHASE_PLL_DSOGI_H_
#define _FILE_THREE_PHASE_PLL_DSOGI_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus


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

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_PLL_DSOGI_H_


