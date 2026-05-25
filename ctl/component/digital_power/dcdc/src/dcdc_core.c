#include <gmp_core.h>

#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/intrinsic/basic/slope_limiter.h>

/**
 * @brief Initializes the unified DC-DC core.
 */
void ctl_init_dcdc_core(ctl_dcdc_core_t* core, const ctl_dcdc_core_init_t* init, parameter_gt v_slope_pu,
                        parameter_gt i_slope_pu)
{
    // Init PI Controllers
    ctl_init_pid_Tmode(&core->v_loop_pi, float2ctrl(init->kp_v_pu), float2ctrl(init->kp_v_pu / init->ki_v_pu),
                       float2ctrl(0), float2ctrl(init->fs));
    ctl_init_pid_Tmode(&core->i_loop_pi, float2ctrl(init->kp_i_pu), float2ctrl(init->kp_i_pu / init->ki_i_pu),
                       float2ctrl(0), float2ctrl(init->fs));

    // --- Configure Strict PID Limits (PU) ---
    // 1. Voltage loop outputs Current. Limit it via i_L_max / i_L_min.
    ctrl_gt limit_i_max = float2ctrl(init->i_L_max / init->i_base);
    ctrl_gt limit_i_min = float2ctrl(init->i_L_min / init->i_base);
    ctl_set_pid_limit(&core->v_loop_pi, limit_i_max, limit_i_min);
    ctl_set_pid_int_limit(&core->v_loop_pi, limit_i_max, limit_i_min);

    // 2. Current loop handles Voltage. Limit it via v_req_max / v_req_min.
    // NOTE: Minimum voltage is strictly limited here (e.g., to 0 for unidirectional DC output).
    ctrl_gt limit_v_max = float2ctrl(init->v_req_max / init->v_base);
    ctrl_gt limit_v_min = float2ctrl(init->v_req_min / init->v_base);
    ctl_set_pid_limit(&core->i_loop_pi, limit_v_max, limit_v_min);
    ctl_set_pid_int_limit(&core->i_loop_pi, limit_v_max, limit_v_min);

    // Init Slope Limiters (Units/sec)
    ctl_init_slope_limiter(&core->v_ramp, float2ctrl(v_slope_pu), float2ctrl(-v_slope_pu), float2ctrl(init->fs));
    ctl_init_slope_limiter(&core->i_ramp, float2ctrl(i_slope_pu), float2ctrl(-i_slope_pu), float2ctrl(init->fs));

    core->flag_enable = 0;
    core->flag_enable_load_ff = 0;
    core->load_ff_gain = float2ctrl(1.0f);

    // Null pointers
    core->v_in_fdbk = NULL;
    core->v_out_fdbk = NULL;
    core->i_L_fdbk = NULL;
    core->i_load_fdbk = NULL;
}