#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Lead Lag controller

#include <ctl/component/intrinsic/discrete/lead_lag.h>

void ctl_init_lead(ctrl_lead_t* obj, parameter_gt K_D, parameter_gt tau_D, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0); // 防呆保护

    // Sampling period (Unified to Ts)
    parameter_gt Ts = 1.0 / fs;

    // Denominator term from bilinear transform: (2*tau_D + T)
    parameter_gt den = 2.0f * tau_D + Ts;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_D - Ts) / (2*tau_D + Ts)
    obj->a1 = float2ctrl((2.0 * tau_D - Ts) * inv_den);

    // b0 = (2*tau_D + 2*K_D + Ts) / (2*tau_D + Ts)
    obj->b0 = float2ctrl((2.0 * tau_D + 2.0 * K_D + Ts) * inv_den);

    // b1 = (Ts - 2*tau_D - 2*K_D) / (2*tau_D + Ts)
    obj->b1 = float2ctrl((Ts - 2.0 * tau_D - 2.0 * K_D) * inv_den);

    ctl_clear_lead(obj);
}

void ctl_init_lead_form2(ctrl_lead_t* obj, parameter_gt alpha, parameter_gt T, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    // Sampling period
    parameter_gt Ts = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_D + T)
    parameter_gt den = 2.0f * T + Ts;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*T - Ts) / (2*T + Ts)
    obj->a1 = float2ctrl((2.0f * T - Ts) * inv_den);

    // b0 = (2*alpha*T + T) / (2*T + Ts)
    obj->b0 = float2ctrl((2.0f * alpha * T + Ts) * inv_den);

    // b1 = (T - 2*alpha*T) / (2*T + Ts)
    obj->b1 = float2ctrl((Ts - 2.0f * alpha * T) * inv_den);

    // Clear initial states
    ctl_clear_lead(obj);
}

void ctl_init_lead_form3(ctrl_lead_t* obj, parameter_gt theta_rad, parameter_gt fc, parameter_gt fs)
{
    parameter_gt alpha;
    //parameter_gt theta_rad = angle_deg * (CTL_PARAM_CONST_PI / 180.0f); // 确保输入是弧度或进行转换

    // 1. Calculate Alpha
    parameter_gt sin_val = sinf(theta_rad);

    // Prevent division by zero if theta is 90 degrees (pi/2)
    if (sin_val > 0.9999)
        sin_val = 0.9999;

    alpha = (1.0f + sin_val) / (1.0f - sin_val);

    // 2. Calculate time constant
    // T = 1 / (omega_c * sqrt(alpha))
    parameter_gt omega_c = 2.0f * CTL_PARAM_CONST_PI * fc;
    parameter_gt T = 1.0f / (omega_c * sqrtf(alpha));

    // 3. 调用 Form2 进行离散化
    ctl_init_lead_form2(obj, alpha, T, fs);
}

//void ctl_init_lead_form3(ctrl_lead_t* obj, parameter_gt angle, parameter_gt fc, parameter_gt fs)
//{
//    parameter_gt alpha;
//
//    alpha = (1 + sinf(angle)) / (1 - sinf(angle));
//
//    ctl_init_lead_form2(obj, alpha, 1 / fc, fs);
//}

void ctl_init_lag(ctrl_lag_t* obj, parameter_gt tau_L, parameter_gt tau_P, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    // Sampling period
    parameter_gt Ts = 1.0f / fs;

    // Denominator term from bilinear transform: (2*tau_P + T)
    parameter_gt den = 2.0f * tau_P + Ts;
    parameter_gt inv_den;

    // Avoid division by zero
    if (fabsf(den) < 1e-9f)
    {
        inv_den = 0.0f; // Or handle error appropriately
    }
    else
    {
        inv_den = 1.0f / den;
    }

    // Calculate coefficients based on the discretized transfer function
    // H(z) = (b0 + b1*z^-1) / (1 - a1*z^-1)

    // a1 = (2*tau_P - Ts) / (2*tau_P + Ts)
    obj->a1 = float2ctrl((2.0 * tau_P - Ts) * inv_den);

    // b0 = (2*tau_L + Ts) / (2*tau_P + Ts)
    obj->b0 = float2ctrl((2.0 * tau_L + Ts) * inv_den);

    // b1 = (Ts - 2*tau_L) / (2*tau_P + Ts)
    obj->b1 = float2ctrl((Ts - 2.0 * tau_L) * inv_den);

    // Clear initial states
    ctl_clear_lag(obj);
}

// Note: Implementations for ctl_init_2p2z and other generic pole-zero
// initializers would go here if they were defined.
