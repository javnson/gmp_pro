
#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// pmsm smo

#include <ctl/component/motor_control/observer/pmsm.smo.h>

void ctl_init_pmsm_smo(
    // SMO handle
    pmsm_smo_t* smo,
    // SMO Initialize object
    const ctl_smo_init_t* init)
{

    ctl_vector2_clear(&smo->e_est);
    ctl_vector2_clear(&smo->z);
    ctl_vector2_clear(&smo->i_est);

    smo->theta_est = 0;

    ctl_set_phasor_via_angle(smo->theta_est, &smo->phasor);

    // smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl));
    smo->k1 = float2ctrl(1.0f / (init->Ld * init->f_ctrl) * init->u_base / init->i_base);
    smo->k2 = float2ctrl(init->Rs / (init->Ld * init->f_ctrl));
    // smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld * init->f_ ctrl));
    smo->k3 = float2ctrl((init->Ld - init->Lq) / (init->Ld));

    smo->k_slide = float2ctrl(init->k_slide);

    ctl_init_lp_filter(&smo->filter_e[0], init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_e[1], init->f_ctrl, init->fc_e);
    ctl_init_lp_filter(&smo->filter_spd, init->f_ctrl, init->fc_omega);

    ctl_init_pid_Tmode(&smo->pid_pll, init->pid_kp, init->pid_Ti, init->pid_Td, init->f_ctrl);
    ctl_set_pid_limit(&smo->pid_pll, init->spd_max_limit, init->spd_min_limit);

    smo->spd_sf = float2ctrl((30.0f / CTL_PARAM_CONST_PI) * init->f_ctrl / init->speed_base_rpm / init->pole_pairs);
    smo->wr_est = 0;

    smo->theta_compensate = float2ctrl(init->speed_base_rpm / 60.0f / init->fc_e * init->pole_pairs);
}

/**
 * @brief Auto-calculates SMO parameters based on motor physics.
 * * @param[out] init The SMO initialization structure to be filled.
 * @param[in] ker_init The motor physical parameters (Resistance, Inductance, etc.).
 */
void ctl_auto_tuning_pmsm_smo(ctl_smo_init_t* init, const mtr_current_init_t* ker_init)
{
    // ============================================================
    // 1. 物理参数传递 (Physical Parameters Pass-through)
    // ============================================================
    init->Rs = ker_init->mtr_Rs;
    init->Ld = ker_init->mtr_Ld;
    init->Lq = ker_init->mtr_Lq;
    init->pole_pairs = (uint16_t)ker_init->pole_pairs;

    // 假设 mtr_current_init_t 中的 spd_base 是 KRPM (千转/分)
    // 需要转换为 RPM 用于初始化计算
    init->speed_base_rpm = ker_init->spd_base * 1000.0f;

    init->f_ctrl = ker_init->fs;
    init->u_base = ker_init->v_base;
    init->i_base = ker_init->i_base;

    // ============================================================
    // 2. 滤波器参数整定 (Filter Tuning)
    // ============================================================

    // 计算电机额定电频率 (Rated Electrical Frequency in Hz)
    // f_elec = (RPM * Poles) / 60
    parameter_gt f_rated_elec = (init->speed_base_rpm * init->pole_pairs) / 60.0f;

    // [关键参数] 反电动势滤波器截止频率 (fc_e)
    // 策略：设置为额定电频率，保证在额定转速下有足够的衰减，同时保留基波
    // 如果电机转速极低，设置一个下限 (例如 5Hz)
    if (f_rated_elec < 10.0f)
        init->fc_e = 10.0f;
    else
        init->fc_e = f_rated_elec;

    // [关键参数] 速度估计滤波器 (fc_omega)
    // 策略：速度信号主要用于监测或速度环反馈，通常带宽较低
    // 设定为 30Hz ~ 50Hz 足够满足大多数速度环需求
    init->fc_omega = 8.0f;

    // ============================================================
    // 3. 滑模增益 (Sliding Gain)
    // ============================================================

    // 策略：在 PU 系统中，额定反电动势幅值约为 1.0。
    // 考虑到负载突变和模型误差，给予 0.85 ~ 1.2 的增益。
    // 只有当估算电流能“追上”实际电流变化率时，SMO 才能工作。
    init->k_slide = 0.85f;

    // ============================================================
    // 4. PLL 参数整定 (PLL Tuning)
    // ============================================================

    // 设定 PLL 自然频率 (Natural Frequency) wn
    // 通常设定为 20Hz ~ 60Hz。
    // 带宽越高，动态越好，但噪声越大。
    parameter_gt pll_bw_hz = 30.0f;
    parameter_gt wn = CTL_PARAM_CONST_2PI * pll_bw_hz;

    // 设定阻尼比 (Damping Ratio) zeta
    parameter_gt zeta = 1.0f; // 临界阻尼，无超调

    //// 计算 PID 参数 (假设是标准位置式/T式 PID: Kp * (1 + 1/(Ti*s)))
    //// Kp 决定响应速度 ( Stiffness )
    //init->pid_kp = float2ctrl(2.0f * zeta * wn);

    //// Ti 决定消除稳态误差的速度 (Integral Time Constant)
    //// Ti = 2*zeta / wn
    //parameter_gt Ti_val = (2.0f * zeta) / wn;
    //init->pid_Ti = float2ctrl(Ti_val);

    // 关键：如果 e_error 是基于 p.u. 的反电动势，
    // 且希望 PLL 输出 w_est (rad/tick)，
    // 你需要将 wn 转换到采样周期尺度 (rad/tick)

    parameter_gt Ts = 1.0f / init->f_ctrl;
    parameter_gt wn_digital = wn * Ts; // 归一化到每步弧度

    // 重新计算 Kp (此时 Kp 应该是一个很小的数，比如 0.01 ~ 0.1 数量级)
    init->pid_kp = float2ctrl(2.0f * zeta * wn_digital);
    init->pid_Ti = float2ctrl(2.0f * zeta / wn_digital * Ts); // 注意 Ti 在 T模式下的定义

    // Td 通常不需要
    init->pid_Td = 0;

    // ============================================================
    // 5. 限制参数 (Limits)
    // ============================================================

    // PLL 输出速度限制 (标幺值)
    // 允许超速 20%
    init->spd_max_limit = float2ctrl(1.2f);
    init->spd_min_limit = float2ctrl(-1.2f);
}
