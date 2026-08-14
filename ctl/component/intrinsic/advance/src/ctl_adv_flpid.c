#include <ctl/math_block/gmp_math.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Fuzzy PID controller
#include <ctl/component/intrinsic/advance/fuzzy_pid.h>

void ctl_init_fuzzy_pid(ctl_fuzzy_pid_t* fp, parameter_gt base_kp, parameter_gt base_ti, parameter_gt base_td,
                        ctrl_gt sat_max, ctrl_gt sat_min, parameter_gt e_q_factor, parameter_gt ec_q_factor,
                        ctl_lut2d_t d_kp_lut, ctl_lut2d_t d_ki_lut, ctl_lut2d_t d_kd_lut, parameter_gt fs)
{
    gmp_ctl_assert(fs > CTL_PARAM_CONST_ZERO); // Sampling frequency must be positive.

    // 1. Convert Ti and Td into independent Ki and Kd gains.
    parameter_gt base_ki = 0.0f;
    if (base_ti > CTL_PARAM_CONST_EPSILON)
    {
        base_ki = base_kp / base_ti;
    }
    parameter_gt base_kd = base_kp * base_td;

    // 2. Precalculate the discrete gains, then store the final control-domain values.
    fp->base_kp = real2ctrl(base_kp);
    fp->base_ki = real2ctrl(base_ki / fs);
    fp->base_kd = real2ctrl(base_kd * fs);

    // 3. Precalculate the frequency scaling used by discretization.
    fp->inv_fs_ctrl = real2ctrl(1.0f / fs);
    fp->fs_ctrl = real2ctrl(fs);

    // 4. Store the fuzzy-output quantization factors.
    fp->e_q_factor = real2ctrl(e_q_factor);
    fp->ec_q_factor = real2ctrl(ec_q_factor);

    fp->last_error = CTL_CTRL_CONST_ZERO;

    // 5. Assign LUTs
    fp->d_kp_lut = d_kp_lut;
    fp->d_ki_lut = d_ki_lut;
    fp->d_kd_lut = d_kd_lut;

    // 6. Use the parallel PID form so Kp, Ki, and Kd remain independently tunable.
    // This independence is required for the fuzzy rule-table corrections.
    ctl_init_pid(&fp->pid, base_kp, base_ki, base_kd, fs);

    // 7. Set limits
    ctl_set_pid_limit(&fp->pid, sat_max, sat_min);
}
