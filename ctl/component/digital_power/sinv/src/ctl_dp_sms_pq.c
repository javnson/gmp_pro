#include <ctl/component/digital_power/sinv/sms_pq.h>

void ctl_init_sms_pq(ctl_sms_pq_t* pq, parameter_gt grid_freq, parameter_gt fs, parameter_gt lpf_fc)
{
    // 初始化电流 SOGI，阻尼系数通常取 0.707 提供最优暂态响应
    ctl_init_discrete_sogi(&pq->sogi_i, grid_freq, 0.707f, fs);

    // 初始化 Biquad 低通滤波器 (由于 SOGI 已经消除了 100Hz 纹波，这里的 fc 可以设得比较高，比如 200Hz 甚至 500Hz)
    ctl_init_biquad_lpf(&pq->lpf_p, fs, lpf_fc, 0.707f);
    ctl_init_biquad_lpf(&pq->lpf_q, fs, lpf_fc, 0.707f);

    ctl_clear_sms_pq(pq);
}

