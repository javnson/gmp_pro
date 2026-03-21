#include <gmp_core.h>

#include <ctl/component/intrinsic/continuous/ladrc2.h>


/**
 * @brief Dynamically upgrades the 2nd-Order LADRC parameters.
 * * @details
 * The poles of the 3rd-order observer are placed at -wo:
 * (s + wo)^3 = s^3 + 3*wo*s^2 + 3*wo^2*s + wo^3 
 * => beta1 = 3*wo, beta2 = 3*wo^2, beta3 = wo^3
 * * The poles of the controller are placed at -wc (Critically damped, zeta = 1):
 * (s + wc)^2 = s^2 + 2*wc*s + wc^2
 * => Kpp = wc^2, Kvp = 2*wc
 */
void ctl_upgrade_ladrc2(ctl_ladrc2_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs)
{
    // 1. Safe Guards
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt b0_safe = b0;
    if (b0 >= 0.0f && b0 < 1e-9f)
        b0_safe = 1e-9f;
    else if (b0 < 0.0f && b0 > -1e-9f)
        b0_safe = -1e-9f;

    // 2. Physical Radian Frequencies
    parameter_gt h = 1.0f / fs_safe;
    parameter_gt wc = CTL_PARAM_CONST_2PI * fc;
    parameter_gt wo = CTL_PARAM_CONST_2PI * fo;

    // 3. Absorbed Coefficients Calculation (Float/Parameter space)
    parameter_gt h_b0_phy = h * b0_safe;
    parameter_gt h_beta1_phy = h * 3.0f * wo;
    parameter_gt h_beta2_phy = h * 3.0f * (wo * wo);
    parameter_gt h_beta3_b0_phy = h * (wo * wo * wo) / b0_safe;

    parameter_gt kpp_b0_phy = (wc * wc) / b0_safe;
    parameter_gt kvp_b0_phy = (2.0f * wc) / b0_safe;
    parameter_gt b0_inv_phy = 1.0f / b0_safe;

    // 4. Assign to structure (States z1, z2, z3_u are preserved for bumpless transition)
    ladrc->h = float2ctrl(h);
    ladrc->h_b0 = float2ctrl(h_b0_phy);
    ladrc->h_beta1 = float2ctrl(h_beta1_phy);
    ladrc->h_beta2 = float2ctrl(h_beta2_phy);
    ladrc->h_beta3_b0 = float2ctrl(h_beta3_b0_phy);

    ladrc->kpp_b0 = float2ctrl(kpp_b0_phy);
    ladrc->kvp_b0 = float2ctrl(kvp_b0_phy);
    ladrc->b0_inv = float2ctrl(b0_inv_phy);
}

/**
 * @brief Initializes the 2nd-order LADRC controller to safe defaults.
 */
void ctl_init_ladrc2(ctl_ladrc2_t* ladrc, parameter_gt b0, parameter_gt fc, parameter_gt fo, parameter_gt fs)
{
    // Configure default output limits (¡À1 for PU systems)
    ladrc->out_max = float2ctrl(1.0f);
    ladrc->out_min = float2ctrl(-1.0f);

    // Calculate parameters
    ctl_upgrade_ladrc2(ladrc, b0, fc, fo, fs);

    // Clear history states
    ctl_clear_ladrc2(ladrc);
}
