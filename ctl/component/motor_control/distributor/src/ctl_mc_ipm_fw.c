

#include <gmp_core.h>
#include <math.h> // For atan2f during initialization

#include <ctl/component/motor_control/distributor/ipm_fw_distributor.h>

//
///**
// * @brief Auto-tunes the IPM base angle and FW PI parameters.
// * * @details
// * **1. MTPA Base Angle Calculation:**
// * For an IPM motor, the optimal d-axis current to maximize torque per ampere is:
// * @f[ i_d = \frac{\psi_f}{2(L_q - L_d)} - \sqrt{ \frac{\psi_f^2}{4(L_q - L_d)^2} + I_m^2 } @f]
// * The positive torque angle in radians is @f$ \alpha = \text{atan2}(i_q, i_d) @f$.
// * For negative torque, to maintain symmetry across the d-axis, the angle is mirrored:
// * @f[ \alpha_{neg\_pu} = 1.0 - \alpha_{pos\_pu} @f]
// * * **2. PI Controller Tuning (Series PI):**
// * @f[ K_p = \frac{\omega_c}{G_{plant}}, \quad K_i = \frac{1}{T_i} \approx \omega_{zero} @f]
// */
//void ctl_autotuning_ipm_fixed_fw(ipm_fw_distributor_init_t* init)
//{
//    // -----------------------------------------------------------------
//    // 1. Calculate Base Angle via MTPA Analytical Solution
//    // -----------------------------------------------------------------
//    parameter_gt dL = init->mtr_Lq - init->mtr_Ld;
//
//    if (dL > 1e-6f && init->i_nom > 1e-3f)
//    {
//        // Calculate optimal id using MTPA formula
//        parameter_gt term1 = init->mtr_psi_f / (2.0f * dL);
//        parameter_gt term2 = sqrtf((term1 * term1) + (init->i_nom * init->i_nom));
//        parameter_gt opt_id = term1 - term2; // opt_id will be negative
//
//        // Ensure mathematically safe calculation for iq
//        parameter_gt iq_sq = (init->i_nom * init->i_nom) - (opt_id * opt_id);
//        parameter_gt opt_iq = (iq_sq > 0.0f) ? sqrtf(iq_sq) : 0.0f;
//
//        // Calculate angle in radians and convert to PU
//        parameter_gt alpha_rad = atan2f(opt_iq, opt_id);
//        init->alpha_pos_base_pu = alpha_rad / CTL_PARAM_CONST_2PI;
//    }
//    else
//    {
//        // Fallback to SPM (id=0) if saliency is negligible or inputs are invalid
//        init->alpha_pos_base_pu = 0.25f; // 90 degrees
//    }
//
//    // Mirror the angle for negative torque (e.g., if pos is 105 deg, neg is 255 deg)
//    // In PU: 1.0 - alpha_pos.
//    // This ensures symmetrical reluctance torque utilization during braking.
//    init->alpha_neg_base_pu = 1.0f - init->alpha_pos_base_pu;
//
//    // -----------------------------------------------------------------
//    // 2. Configure Limits and Margins
//    // -----------------------------------------------------------------
//    init->v_fw_margin = 0.95f;
//
//    // The maximum FW compensation should not push the total angle past 180 deg (0.5 PU)
//    // Limit = 0.5 PU - base_angle_pu
//    init->alpha_max_fw_pu = 0.5f - init->alpha_pos_base_pu;
//    if (init->alpha_max_fw_pu < 0.0f)
//        init->alpha_max_fw_pu = 0.0f;
//
//    // -----------------------------------------------------------------
//    // 3. PI Parameter Calculation (Series PI)
//    // -----------------------------------------------------------------
//    parameter_gt target_bandwidth_rad = 60.0f;
//    parameter_gt plant_gain_estimate = 1.0f;
//
//    init->kp_fw = target_bandwidth_rad / plant_gain_estimate;
//    init->ki_fw = 20.0f;
//}
//
//void ctl_init_ipm_fixed_fw(ipm_fw_distributor_t* dist, const ipm_fw_distributor_init_t* init)
//{
//    if (init->v_base > 1e-6f)
//    {
//        dist->vs_limit_pu = float2ctrl((init->v_nom * init->v_fw_margin) / init->v_base);
//    }
//    else
//    {
//        dist->vs_limit_pu = float2ctrl(0.0f);
//    }
//
//    dist->alpha_pos_base_pu = float2ctrl(init->alpha_pos_base_pu);
//    dist->alpha_neg_base_pu = float2ctrl(init->alpha_neg_base_pu);
//    dist->alpha_max_fw_pu = float2ctrl(init->alpha_max_fw_pu);
//
//    dist->flag_enable_fw = 0;
//
//    // Initialize the Field Weakening PID controller (Series Type)
//    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);
//
//    // Limit maximum angle compensation to prevent going deep into unstable demagnetization zones
//    ctl_set_pid_limit(&dist->fw_pid, init->alpha_max_fw_pu, 0.0f);
//    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_max_fw_pu, 0.0f);
//
//    ctl_clear_const_distributor(dist);
//}
