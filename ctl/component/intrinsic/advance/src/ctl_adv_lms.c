
#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// LMS filter

#include <ctl/component/intrinsic/advance/lms_filter.h>

fast_gt ctl_init_lms_filter(ctl_lms_filter_t* lms, uint32_t order, parameter_gt mu,
                            ctrl_gt* external_weights, // 架构升级：依赖注入权重数组
                            ctrl_gt* external_buffer)  // 架构升级：依赖注入缓存数组
{
    // 防呆保护
    gmp_base_assert(order > 0);
    gmp_base_assert(external_weights != 0);
    gmp_base_assert(external_buffer != 0);

    lms->order = order;
    lms->weights = external_weights;
    lms->buffer = external_buffer;

    // 安全转入定点域
    lms->mu = float2ctrl(mu);

    ctl_clear_lms_filter(lms);

    return 1; // 成功
}
