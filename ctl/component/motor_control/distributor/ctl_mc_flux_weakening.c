
#include <gmp_core.h>


#include <ctl/component/motor_control/distributor/flux_weakening.h>

/**
 * @brief Initializes the Flux Weakening Controller.
 * @param[out] fw Pointer to the FW controller instance.
 * @param[in] kp Voltage loop Kp.
 * @param[in] ki Voltage loop Ki.
 * @param[in] margin Voltage reference margin (e.g., 0.95).
 * @param[in] fs Sampling frequency.
 */
void ctl_init_fw_ctrl(ctl_fw_ctrl_t* fw, ctrl_gt kp, ctrl_gt ki, ctrl_gt margin, parameter_gt fs)
{
    // 1. Initialize PI
    // Note: The output of this PI is id_ref.
    // Ideally, when V_err < 0 (Voltage too high), we want id to go Negative.
    // If we define Error = V_ref - V_s:
    //    V_s > V_ref -> Error < 0.
    //    We need negative output. So Kp should be Positive.
    ctl_init_pid(&fw->volt_ctrl, kp, ki, 0, fs);

    // 2. Set PI Limits
    // The FW controller only generates NEGATIVE id current.
    // So Max is 0, Min is -I_smax (will be updated in step if I_smax changes, but init here).
    ctl_set_pid_limit(&fw->volt_ctrl, 0, -1000.0f); // Temporary min, sets in step
    ctl_set_pid_int_limit(&fw->volt_ctrl, 0, -1000.0f);

    fw->v_ref_margin = margin;
    fw->i_smax = 0; // User needs to set this via setter or input
    fw->id_ref_fw = 0;
    fw->iq_max_dynamic = 0;
    fw->flag_in_fw = 0;

    ctl_vector2_clear(&fw->vdq_curr);
}
