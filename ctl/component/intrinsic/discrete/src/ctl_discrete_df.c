#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Direct Form controller
#include <ctl/component/intrinsic/discrete/direct_form.h>

// TODO: add ctrl2float()function for all parameters

/*---------------------------------------------------------------------------*/
/* DF11 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df11(ctl_df11_t* df, parameter_gt b0, parameter_gt b1, parameter_gt a1)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->a1 = float2ctrl(a1);
    ctl_clear_df11(df);
}

parameter_gt ctl_get_df11_gain(ctl_df11_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w;
    parameter_gt num_imag = -df->b1 * sin_w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w;
    parameter_gt den_imag = -df->a1 * sin_w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df11_phase_lag(ctl_df11_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w);
    parameter_gt sin_w = sinf(w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w;
    parameter_gt num_imag = -df->b1 * sin_w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w;
    parameter_gt den_imag = -df->a1 * sin_w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF22 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df22(ctl_df22_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt a1, parameter_gt a2)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    ctl_clear_df22(df);
}

parameter_gt ctl_get_df22_gain(ctl_df22_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df22_phase_lag(ctl_df22_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF13 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df13(ctl_df13_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->b3 = float2ctrl(b3);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    df->a3 = float2ctrl(a3);
    ctl_clear_df13(df);
}

parameter_gt ctl_get_df13_gain(ctl_df13_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df13_phase_lag(ctl_df13_t* df, parameter_gt fs, parameter_gt f)
{
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}

/*---------------------------------------------------------------------------*/
/* DF23 Implementation                                                       */
/*---------------------------------------------------------------------------*/
void ctl_init_df23(ctl_df23_t* df, parameter_gt b0, parameter_gt b1, parameter_gt b2, parameter_gt b3, parameter_gt a1,
                   parameter_gt a2, parameter_gt a3)
{
    df->b0 = float2ctrl(b0);
    df->b1 = float2ctrl(b1);
    df->b2 = float2ctrl(b2);
    df->b3 = float2ctrl(b3);
    df->a1 = float2ctrl(a1);
    df->a2 = float2ctrl(a2);
    df->a3 = float2ctrl(a3);
    ctl_clear_df23(df);
}

parameter_gt ctl_get_df23_gain(ctl_df23_t* df, parameter_gt fs, parameter_gt f)
{
    // Transfer function is identical to DF13
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt mag_num = sqrtf(num_real * num_real + num_imag * num_imag);
    parameter_gt mag_den = sqrtf(den_real * den_real + den_imag * den_imag);

    return (mag_den < 1e-9f) ? 0.0f : (mag_num / mag_den);
}

parameter_gt ctl_get_df23_phase_lag(ctl_df23_t* df, parameter_gt fs, parameter_gt f)
{
    // Transfer function is identical to DF13
    parameter_gt w = 2.0f * CTL_PARAM_CONST_PI * f / fs;
    parameter_gt cos_w = cosf(w), sin_w = sinf(w);
    parameter_gt cos_2w = cosf(2 * w), sin_2w = sinf(2 * w);
    parameter_gt cos_3w = cosf(3 * w), sin_3w = sinf(3 * w);

    parameter_gt num_real = df->b0 + df->b1 * cos_w + df->b2 * cos_2w + df->b3 * cos_3w;
    parameter_gt num_imag = -df->b1 * sin_w - df->b2 * sin_2w - df->b3 * sin_3w;
    parameter_gt den_real = 1.0f + df->a1 * cos_w + df->a2 * cos_2w + df->a3 * cos_3w;
    parameter_gt den_imag = -df->a1 * sin_w - df->a2 * sin_2w - df->a3 * sin_3w;

    parameter_gt phase_num = atan2f(num_imag, num_real);
    parameter_gt phase_den = atan2f(den_imag, den_real);

    return -(phase_num - phase_den);
}
