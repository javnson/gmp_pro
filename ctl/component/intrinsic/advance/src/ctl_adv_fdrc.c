/**
 * @file rc_controller.c
 * @author GMP Library Contributors
 * @brief Implementation of the Fractional Delay Repetitive Controller.
 * 
 * @copyright Copyright GMP(c) 2024-2026
 */

#include <gmp_core.h>

#include <ctl/component/intrinsic/advance/fdrc.h>

void ctl_init_fdrc(ctl_fdrc_t* obj, ctrl_gt* buffer, uint32_t capacity, parameter_gt fs, parameter_gt f_min,
                   parameter_gt q_fc, parameter_gt k_rc, int32_t phase_lead_k)
{
    // Memory and validity assertions
    gmp_base_assert(buffer != NULL);
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(f_min > 0.0f);

    // Ensure the provided capacity is physically large enough to hold the lowest frequency cycle
    gmp_base_assert(capacity >= CTL_RC_CALC_MIN_CAPACITY(fs, f_min));

    // Configure structural parameters
    obj->buffer = buffer;
    obj->buffer_capacity = capacity;
    obj->phase_lead_k = phase_lead_k;
    obj->k_rc = float2ctrl(k_rc);

    obj->out_max = float2ctrl(1.0f);
    obj->out_min = float2ctrl(-1.0f);

    obj->fs = fs;
    obj->f_min_rated = f_min;

    // Enable learning by default
    ctl_enable_fdrc_integrating(obj);

    // Initialize the internal Q(z) Biquad filter as a Low-Pass Filter with Butterworth response
    ctl_init_biquad_lpf(&obj->q_filter, fs, q_fc, 0.707f);

    // Reset runtime states
    ctl_clear_fdrc(obj);
}
