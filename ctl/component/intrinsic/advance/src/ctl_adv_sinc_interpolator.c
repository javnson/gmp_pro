
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// sinc interpolator
#include <ctl/component/intrinsic/advance/sinc_interpolator.h>

fast_gt ctl_init_sinc_interpolator(ctl_sinc_interpolator_t* sinc, uint32_t num_taps, uint32_t table_size,
                                   ctrl_gt* external_buffer,
                                   ctrl_gt* external_sinc_table)
{
    uint32_t i, j;

    // 防呆保护
    gmp_base_assert(external_buffer != 0);
    gmp_base_assert(external_sinc_table != 0);
    gmp_base_assert(num_taps > 0);
    gmp_base_assert(table_size > 0);

    sinc->num_taps = num_taps;
    sinc->table_size = table_size;

    sinc->buffer = external_buffer;
    sinc->sinc_table = external_sinc_table;

    // Calculate the coefficients for each fractional delay interval
    for (i = 0; i < table_size; i++)
    {
        // 移除 f 后缀，使用高精度 parameter_gt
        parameter_gt fractional_offset = (parameter_gt)i / (parameter_gt)table_size;

        for (j = 0; j < num_taps; j++)
        {
            parameter_gt center = (parameter_gt)(num_taps - 1) / 2.0f;
            parameter_gt t = (parameter_gt)j - center - fractional_offset;

            parameter_gt sinc_val;
            if (fabsf(t) < 1e-9f) // 避免严格的 0.0 比较
            {
                sinc_val = 1.0f;
            }
            else
            {
                sinc_val = sinf(CTL_PARAM_CONST_PI * t) / (CTL_PARAM_CONST_PI * t);
            }

            // Blackman window
            parameter_gt window_val = 0.42f - 0.5f * cosf(2.0f * CTL_PARAM_CONST_PI * j / (num_taps - 1)) +
                                      0.08f * cosf(4.0f * CTL_PARAM_CONST_PI * j / (num_taps - 1));

            // 修复：必须使用 float2ctrl 宏安全转入控制域，并采用一维展平寻址
            sinc->sinc_table[i * num_taps + j] = float2ctrl(sinc_val * window_val);
        }
    }

    ctl_clear_sinc_interpolator(sinc);
    return 1;
}
