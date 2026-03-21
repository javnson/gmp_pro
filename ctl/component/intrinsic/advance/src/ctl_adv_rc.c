#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// repetitive controller
#include <ctl/component/intrinsic/advance/repetitive_controller.h>


fast_gt ctl_init_repetitive_controller(ctl_repetitive_controller_t* rc, parameter_gt fs, parameter_gt f_fund,
                                       parameter_gt q_filter_coeff,
                                       ctrl_gt* external_buffer,
                                       uint32_t max_buffer_capacity)
{

    // 1. 防呆与除零保护
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(f_fund > 0.0f);
    gmp_base_assert(external_buffer != 0); // 确保指针不为空

    // 2. 计算需要的周期点数 N
    uint32_t required_samples = (uint32_t)(fs / f_fund);

    // 3. 内存安全终极防线：防止缓冲区溢出
    if (required_samples > max_buffer_capacity || required_samples == 0)
    {
        // 如果基波频率太低，导致需要的点数超过了用户分配的内存，直接拒绝初始化
        return 0;
    }

   // 4. 安全赋值
    rc->period_samples = required_samples;
    rc->state_buffer = external_buffer;
    rc->q_filter_coeff = float2ctrl(q_filter_coeff);

    // 5. 设置默认限幅
    rc->out_max = float2ctrl(1.0f);
    rc->out_min = float2ctrl(-1.0f);

    // 6. 清理历史状态
    ctl_clear_repetitive_controller(rc);

    return 1; // Success
}
