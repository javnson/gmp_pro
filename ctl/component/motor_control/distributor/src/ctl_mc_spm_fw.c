
#include <gmp_core.h>


#include <ctl/component/motor_control/distributor/spm_fw_distributor.h>


/**
 * @brief Auto-tunes the FW PI parameters based on Series-PI formulation.
 * * @details 
 * For a Series PI Controller defined as:
 * $$ C(s) = K_p \left( 1 + \frac{K_i}{s} \right) $$
 * * The plant from "Advance Angle (PU)" to "Voltage Amplitude (PU)" can be approximated 
 * by a static gain $G_{plant}$ at a given high speed. 
 * $$ G_{plant} = \frac{\Delta V_{pu}}{\Delta \alpha_{pu}} $$
 * * We design $K_p$ to cancel the plant gain and establish the voltage loop bandwidth $\omega_c$:
 * $$ K_p = \frac{\omega_c}{G_{plant}} $$
 * * Since the Series PI's $K_i$ term intrinsically acts as the inverse of the integration 
 * time constant ($1 / T_i$), it DOES NOT need to be multiplied by $K_p$ again 
 * in the controller's parameter setup. We set it to match the desired zero frequency:
 * $$ K_i = \frac{1}{T_i} \approx \omega_{zero} $$
 */
void ctl_autotuning_spm_fw_distributor(ctl_spm_fw_distributor_init_t* init)
{
    init->v_fw_margin = 0.95f;
    init->alpha_max_fw = 0.25f;

    parameter_gt target_bandwidth_rad = 60.0f;
    parameter_gt plant_gain_estimate = 1.0f;

    init->kp_fw = target_bandwidth_rad / plant_gain_estimate;
    init->ki_fw = 20.0f;
}

void ctl_init_spm_fw_distributor(ctl_spm_fw_distributor_t* dist, const ctl_spm_fw_distributor_init_t* init)
{
    if (init->v_base > 1e-6f)
    {
        dist->vs_limit_pu = float2ctrl((init->v_nom * init->v_fw_margin) / init->v_base);
    }
    else
    {
        dist->vs_limit_pu = float2ctrl(0.0f);
    }

    dist->alpha_max_fw_pu = float2ctrl(init->alpha_max_fw);
    dist->flag_enable_fw = 0;

    ctl_init_pid(&dist->fw_pid, init->kp_fw, init->ki_fw, 0.0f, init->fs);

    ctl_set_pid_limit(&dist->fw_pid, init->alpha_max_fw, 0.0f);
    ctl_set_pid_int_limit(&dist->fw_pid, init->alpha_max_fw, 0.0f);

    ctl_clear_spm_fw_distributor(dist);
}
