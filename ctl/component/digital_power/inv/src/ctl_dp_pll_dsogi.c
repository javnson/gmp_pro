/**
 * @file ctl_dp_three_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for three-phase digital power controller modules.
 * @version 1.05
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for three-phase digital power components,
 * including the Three-Phase PLL and bridge modulation modules.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Three Phase PLL

#include <ctl/component/digital_power/inv/pll_dsogi.h>

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
                        parameter_gt bw_pll, parameter_gt k_sogi)
{
    // 1. Initialize the two SOGI blocks
    // Note: They are initialized at the nominal grid frequency (f_base).
    ctl_init_discrete_sogi(&dsogi->sogi_alpha, k_sogi, f_base, f_ctrl);
    ctl_init_discrete_sogi(&dsogi->sogi_beta, k_sogi, f_base, f_ctrl);

    // 2. Initialize the internal SRF-PLL using Auto-Tune
    // Damping ratio for PLL is typically fixed at 0.707
    ctl_init_srf_pll_auto_tune(&dsogi->srf_pll, f_base, f_ctrl, v_mag, bw_pll, 0.707f);

    // 3. Clear internal states
    ctl_vector2_clear(&dsogi->v_pos_seq);
}
