

#include <gmp_core.h>

#include <ctl/component/intrinsic/continuous/ladrc1.h>

/**
 * @brief Dynamically upgrades the LADRC parameters with safe state retention.
 * * @details
 * Uses physical floating-point calculations to absorb the sample time (h) and 
 * system gain (b0) into the real-time control coefficients.
 */
void ctl_upgrade_ladrc1(ctl_ladrc1_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs)
{
    // 1. Safe Guards
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt b0_safe = (b0 > 1e-9f || b0 < -1e-9f) ? b0 : 1.0f; // Prevent Div by 0

    // 2. Physical Radian Frequencies
    parameter_gt h = 1.0f / fs_safe;
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc;
    parameter_gt wo = CTL_PARAM_CONST_2PI * fo;

    // 3. Absorbed Coefficients Calculation (Float/Parameter space)
    parameter_gt h_b0_phy = h * b0_safe;
    parameter_gt h_beta1_phy = h * 2.0f * wo;
    parameter_gt h_beta2_b0_phy = h * (wo * wo) / b0_safe;
    parameter_gt kp_b0_phy = wc / b0_safe;

    // 4. Assign to structure
    // We only update the coefficients. z1, z2_u, and u_prev are untouched
    // to guarantee perfectly smooth operation during dynamic bandwidth adjustments.
    ladrc->h_b0 = float2ctrl(h_b0_phy);
    ladrc->h_beta1 = float2ctrl(h_beta1_phy);
    ladrc->h_beta2_b0 = float2ctrl(h_beta2_b0_phy);
    ladrc->kp_b0 = float2ctrl(kp_b0_phy);
}

/**
 * @brief Initializes the 1st-order LADRC controller to safe defaults.
 */
void ctl_init_ladrc1(ctl_ladrc1_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs)
{
    // Configure default output limits (¡À1 for PU systems)
    ladrc->out_max = float2ctrl(1.0f);
    ladrc->out_min = float2ctrl(-1.0f);

    // Calculate parameters
    ctl_upgrade_ladrc1(ladrc, b0, fc, fo, fs);

    // Clear history states
    ctl_clear_ladrc1(ladrc);
}
