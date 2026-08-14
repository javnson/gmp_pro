
#include <ctl/math_block/gmp_math.h>


//////////////////////////////////////////////////////////////////////////
// ILC

#include <ctl/component/intrinsic/advance/ilc.h>

void ctl_init_ilc(ctl_ilc_controller_t* ilc, const ctl_ilc_init_t* init)
{
    ilc->u_k = init->u_k_buffer;
    ilc->u_k_minus_1 = init->u_k_minus_1_buffer;
    ilc->e_k_minus_1 = init->e_k_minus_1_buffer;

    // 修复 1：严格的定点/浮点类型隔离与转换
    ilc->learning_gain = float2ctrl(init->learning_gain);

    ilc->trajectory_length = init->trajectory_length;
    ilc->is_learning = 1; // 默认开启学习

    ctl_clear_ilc(ilc);
}
