#include <ctl/math_block/gmp_math.h>

#include <ctl/component/intrinsic/complex/dq_pi.h>

void ctl_init_dq_pi(ctl_dq_pi_t* dq, parameter_gt kp_d, parameter_gt ki_d, parameter_gt kp_q,
                    parameter_gt ki_q, parameter_gt fs)
{
    gmp_ctl_assert(dq);
    gmp_ctl_assert(fs > 0.0f);

    ctl_init_pid(&dq->axis[0], kp_d, ki_d, 0.0f, fs);
    ctl_init_pid(&dq->axis[1], kp_q, ki_q, 0.0f, fs);

    dq->circle_limit_sq = float2ctrl(1.0f);
    dq->rect_limit_max.dat[0] = float2ctrl(1.0f);
    dq->rect_limit_max.dat[1] = float2ctrl(1.0f);
    dq->rect_limit_min.dat[0] = float2ctrl(-1.0f);
    dq->rect_limit_min.dat[1] = float2ctrl(-1.0f);

    dq->flag_enable_feedforward = 0;
    dq->flag_enable_circle_limit = 0;
    dq->flag_enable_rect_limit = 1;
    ctl_clear_dq_pi(dq);
}
