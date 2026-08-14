#include <ctl/math_block/gmp_math.h>

#include <ctl/component/intrinsic/complex/dq_ladrc1.h>

void ctl_init_dq_ladrc1(ctl_dq_ladrc1_t* dq, parameter_gt b0_d, parameter_gt fc_d, parameter_gt fo_d,
                        parameter_gt b0_q, parameter_gt fc_q, parameter_gt fo_q, parameter_gt fs)
{
    gmp_ctl_assert(dq);
    gmp_ctl_assert(fs > 0.0f);

    ctl_init_ladrc1(&dq->axis[0], b0_d, fc_d, fo_d, fs);
    ctl_init_ladrc1(&dq->axis[1], b0_q, fc_q, fo_q, fs);

    dq->circle_limit_sq = float2ctrl(1.0f);
    dq->rect_limit_max.dat[0] = float2ctrl(1.0f);
    dq->rect_limit_max.dat[1] = float2ctrl(1.0f);
    dq->rect_limit_min.dat[0] = float2ctrl(-1.0f);
    dq->rect_limit_min.dat[1] = float2ctrl(-1.0f);

    dq->flag_enable_feedforward = 0;
    dq->flag_enable_circle_limit = 0;
    dq->flag_enable_rect_limit = 1;
    ctl_clear_dq_ladrc1(dq);
}
