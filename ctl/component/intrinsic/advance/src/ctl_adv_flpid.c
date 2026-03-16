#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Fuzzy PID controller
#include <ctl/component/intrinsic/advance/fuzzy_pid.h>

void ctl_init_fuzzy_pid(ctl_fuzzy_pid_t* fp, parameter_gt base_kp, parameter_gt base_ti, parameter_gt base_td,
                        ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt e_q_factor, parameter_gt ec_q_factor,
                        ctl_lut2d_t d_kp_lut, ctl_lut2d_t d_ki_lut, ctl_lut2d_t d_kd_lut, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0); // 增加防呆保护

    // 1. 将基于时间常数 (Ti, Td) 的输入转换为独立的 (Ki, Kd) 物理增益
    parameter_gt base_ki = 0.0;
    if (base_ti > 1e-6) // 移除 f 后缀
    {
        base_ki = base_kp / base_ti;
    }
    parameter_gt base_kd = base_kp * base_td;

    // 2. 预先计算离散化的基础增益，并固化为 ctrl_gt 类型！(极速运行的关键)
    fp->base_kp = float2ctrl(base_kp);
    fp->base_ki = float2ctrl(base_ki / fs);
    fp->base_kd = float2ctrl(base_kd * fs);

    // 3. 预先计算离散化需要用到的频率缩放因子
    fp->inv_fs_ctrl = float2ctrl(1.0 / fs);
    fp->fs_ctrl = float2ctrl(fs);

    // 4. 量化因子固化
    fp->e_q_factor = float2ctrl(e_q_factor);
    fp->ec_q_factor = float2ctrl(ec_q_factor);

    fp->last_error = float2ctrl(0.0f);

    // 5. Assign LUTs
    fp->d_kp_lut = d_kp_lut;
    fp->d_ki_lut = d_ki_lut;
    fp->d_kd_lut = d_kd_lut;

    // 6. 极其关键：必须初始化为并联型 (Parallel) PID！
    // 只有并联型的 Kp, Ki, Kd 之间才没有耦合，模糊表的调节才会完全符合预期。
    ctl_init_pid(&fp->pid, base_kp, base_ki, base_kd, fs);

    // 7. Set limits
    ctl_set_pid_limit(&fp->pid, sat_max, sat_min);
}
