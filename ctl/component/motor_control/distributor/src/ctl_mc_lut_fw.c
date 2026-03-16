
#include <gmp_core.h>

#include <ctl/component/motor_control/distributor/lut_fw_distributor.h>

/**
 * @brief Auto-tunes the LUT FW parameters.
 * * @details 
 * Unlike the fixed-alpha module, the base angle is provided by the LUT. 
 * The tuning here focuses on the Series PI controller:
 * @f[ K_p = \frac{\omega_c}{G_{plant}}, \quad K_i = \frac{1}{T_i} \approx \omega_{zero} @f]
 * The maximum compensation angle is set heuristically, typically 0.2 PU (72 deg),
 * assuming the LUT base angle is already providing significant demagnetization at high loads.
 */
void ctl_autotuning_lut_fw_distributor(ctl_lut_fw_distributor_init_t* init)
{
    // 1. Configure Limits and Margins
    init->v_fw_margin = 0.95f;

    // For LUT-based control, the base angle typically ranges from 0.25 PU (SPM-like)
    // to ~0.35 PU (Highly salient IPM). We allow a safe additional margin of 0.15~0.2 PU.
    init->alpha_max_fw_pu = 0.20f;

    // 2. PI Parameter Calculation (Series PI)
    parameter_gt target_bandwidth_rad = 60.0f;
    parameter_gt plant_gain_estimate = 1.0f;

    init->kp_fw = target_bandwidth_rad / plant_gain_estimate;
    init->ki_fw = 20.0f;
}

void ctl_init_lut_fw_distributor(ctl_lut_fw_distributor_t* dist, const ctl_lut_fw_distributor_init_t* init)
{
    // Calculate PU Voltage Limit
    if (init->v_base > 1e-6f)
    {
        dist->vs_limit_pu = float2ctrl((init->v_nom * init->v_fw_margin) / init->v_base);
    }
    else
    {
        dist->vs_limit_pu = float2ctrl(0.0f);
    }

    dist->alpha_max_fw_pu = float2ctrl(init->alpha_max_fw_pu);
    dist->flag_enable_fw = 0;

    // Initialize the Field Weakening PID controller (Series Type)
    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);

    // Limit maximum angle compensation
    ctl_set_pid_limit(&dist->fw_pid, init->alpha_max_fw_pu, 0.0f);
    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_max_fw_pu, 0.0f);

    // Initialize the Paired Look-Up Table module
    if (init->lut_table != NULL && init->lut_size > 1)
    {
        ctl_init_paired_lut1d(&dist->im_lut, init->lut_table, init->lut_size);
    }
    else
    {
        // Fail-safe handling for null tables
        dist->im_lut.table = NULL;
        dist->im_lut.size = 0;
    }

    ctl_clear_lut_fw_distributor(dist);
}
