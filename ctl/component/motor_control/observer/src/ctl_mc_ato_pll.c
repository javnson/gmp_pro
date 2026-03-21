#include <gmp_core.h>

#include <ctl/component/motor_control/observer/ato_pll.h>

/**
 * @brief Auto-tunes and initializes the Angle Tracking Observer.
 * @details Absorbs all dimension conversions (W_base, 2*PI) and sampling time (Ts) 
 * into pure fixed-point constants for extreme execution efficiency.
 */
void ctl_init_ato_pll(ctl_ato_pll_t* pll, parameter_gt bandwidth_hz, parameter_gt damping_ratio,
                      parameter_gt omega_base, parameter_gt fs, parameter_gt spd_limit_max, parameter_gt spd_limit_min)
{
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;
    parameter_gt w_base = (omega_base > 1e-6f) ? omega_base : 1.0f;
    parameter_gt ts = 1.0f / fs_safe;

    // 1. Physical Radian Frequencies
    parameter_gt wn = CTL_PARAM_CONST_2PI * bandwidth_hz;
    parameter_gt zeta = damping_ratio;

    // 2. Calculate Physical Gains
    parameter_gt kp_phy = 2.0f * zeta * wn;
    parameter_gt ki_phy = wn * wn;

    // 3. Map to PU and absorb Discrete Sampling Time
    parameter_gt pu_scale = CTL_PARAM_CONST_2PI / w_base;
    parameter_gt kp_pu_val = kp_phy * pu_scale;
    parameter_gt ki_pu_val = ki_phy * pu_scale * ts;

    // 4. Initialize underlying PID object
    ctl_init_pid(&pll->pi_ctrl, float2ctrl(kp_pu_val), float2ctrl(ki_pu_val), float2ctrl(0.0f), fs_safe);
    ctl_set_pid_limit(&pll->pi_ctrl, float2ctrl(spd_limit_max), float2ctrl(spd_limit_min));
    // 同样约束积分器的边界，防止深深度饱和
    ctl_set_pid_int_limit(&pll->pi_ctrl, float2ctrl(spd_limit_max), float2ctrl(spd_limit_min));

    // 5. Angle Integration Scale Factor
    pll->sf_w_to_angle = float2ctrl(w_base * ts / CTL_PARAM_CONST_2PI);

    // 6. Clear States
    ctl_clear_ato_pll(pll);
}
