
#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// ILC

#include <ctl/component/intrinsic/advance/ilc.h>

void ctl_init_ilc(ctl_ilc_controller_t* ilc, const ctl_ilc_init_t* init)
{
    ilc->u_k = init->u_k_buffer;
    ilc->u_k_minus_1 = init->u_k_minus_1_buffer;
    ilc->e_k_minus_1 = init->e_k_minus_1_buffer;

    ilc->learning_gain = (ctrl_gt)init->learning_gain;
    ilc->trajectory_length = init->trajectory_length;

    ctl_clear_ilc(ilc);
}

//////////////////////////////////////////////////////////////////////////
// IMC
#include <ctl/component/intrinsic/advance/imc.h>

int ctl_init_imc(ctl_imc_controller_t* imc, const ctl_imc_init_t* init)
{
    ctrl_gt Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // --- Discretize Plant Model (ZOH) ---
    imc->a_p_d = expf(-(Ts / (ctrl_gt)init->tau_p));
    imc->b_p_d = (ctrl_gt)init->K_p * (1.0f - imc->a_p_d);

    // --- Calculate Dead Time ---
    imc->dead_time_samples = (uint16_t)roundf((ctrl_gt)init->theta_p / Ts);
    if (imc->dead_time_samples >= IMC_MAX_DEAD_TIME_SAMPLES)
    {
        return -1; // Error: Dead time exceeds buffer size
    }

    // --- Discretize IMC Controller Q(s) (Tustin's method) ---
    // Q(s) = (1/Kp) * (tau_p*s + 1) / (lambda*s + 1)
    ctrl_gt lambda = (ctrl_gt)init->lambda;
    ctrl_gt tau_p = (ctrl_gt)init->tau_p;
    ctrl_gt K_p = (ctrl_gt)init->K_p;

    imc->a_q_d = (2.0f * lambda - Ts) / (2.0f * lambda + Ts);
    imc->b0_q_d = (1.0f / K_p) * (2.0f * tau_p + Ts) / (2.0f * lambda + Ts);
    imc->b1_q_d = (1.0f / K_p) * (Ts - 2.0f * tau_p) / (2.0f * lambda + Ts);

    // Reset states
    ctl_clear_imc(imc);
    return 0;
}

//////////////////////////////////////////////////////////////////////////
// LMS filter

#include <ctl/component/intrinsic/advance/lms_filter.h>

fast_gt ctl_init_lms_filter(ctl_lms_filter_t* lms, uint32_t order, parameter_gt mu)
{
    uint32_t i;

    lms->order = order;
    lms->mu = float2ctrl(mu);
    lms->output = 0.0f;
    lms->error = 0.0f;
    lms->buffer_index = 0;

    // --- Key Point Analysis 1: Dynamic Memory Allocation ---
    // The filter's memory requirement depends on its order. Dynamic allocation
    // provides flexibility. Ensure the MCU has enough heap space.
    lms->weights = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));
    lms->buffer = (ctrl_gt*)malloc(order * sizeof(ctrl_gt));

    if (lms->weights == NULL || lms->buffer == NULL)
    {
        // Free any partially allocated memory before returning
        if (lms->weights)
            free(lms->weights);
        if (lms->buffer)
            free(lms->buffer);
        return 0; // Memory allocation failed
    }

    // Initialize weights and buffer to zero
    for (i = 0; i < order; ++i)
    {
        lms->weights[i] = 0.0f;
        lms->buffer[i] = 0.0f;
    }

    return 1; // Success
}

void ctl_destroy_lms_filter(ctl_lms_filter_t* lms)
{
    if (lms->weights != NULL)
    {
        free(lms->weights);
        lms->weights = NULL;
    }
    if (lms->buffer != NULL)
    {
        free(lms->buffer);
        lms->buffer = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////
// MRAC
#include <ctl/component/intrinsic/advance/mrac.h>

void ctl_init_mrac(ctl_mrac_controller_t* mrac, const ctl_mrac_init_t* init)
{
    ctrl_gt Ts = 1.0f / (ctrl_gt)init->f_ctrl;

    // Discretize the reference model using Zero-Order Hold (ZOH)
    mrac->a_m_d = expf(-(ctrl_gt)init->a_m * Ts);
    mrac->b_m_d = (ctrl_gt)init->b_m / (ctrl_gt)init->a_m * (1.0f - mrac->a_m_d);

    // Discretize the adaptation rates
    mrac->gamma_r_d = (ctrl_gt)init->gamma_r * Ts;
    mrac->gamma_y_d = (ctrl_gt)init->gamma_y * Ts;

    // Reset states
    ctl_clear_mrac(mrac);
}
//////////////////////////////////////////////////////////////////////////
// repetitive controller
#include <ctl/component/intrinsic/advance/repetitive_controller.h>
#include <stdlib.h> // Required for malloc and free

fast_gt ctl_init_repetitive_controller(ctl_repetitive_controller_t* rc, parameter_gt fs, parameter_gt f_fund,
                                       parameter_gt q_filter_coeff)
{
    uint32_t i;

    rc->period_samples = (uint32_t)(fs / f_fund);
    rc->q_filter_coeff = float2ctrl(q_filter_coeff);
    rc->output = 0;
    rc->buffer_index = 0;

    // Allocate memory for the state buffer
    rc->state_buffer = (ctrl_gt*)malloc(rc->period_samples * sizeof(ctrl_gt));
    if (rc->state_buffer == NULL)
    {
        return 0; // Memory allocation failed
    }

    // Initialize the buffer to zero
    for (i = 0; i < rc->period_samples; ++i)
    {
        rc->state_buffer[i] = 0;
    }

    return 1; // Success
}

void ctl_destroy_repetitive_controller(ctl_repetitive_controller_t* rc)
{
    if (rc->state_buffer != NULL)
    {
        free(rc->state_buffer);
        rc->state_buffer = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////
// sinc interpolator
#include <ctl/component/intrinsic/advance/sinc_interpolator.h>
#include <stdlib.h> // Required for malloc and free

fast_gt ctl_init_sinc_interpolator(ctl_sinc_interpolator_t* sinc, uint32_t num_taps, uint32_t table_size)
{
    uint32_t i, j;

    sinc->num_taps = num_taps;
    sinc->table_size = table_size;
    sinc->buffer_index = 0;
    sinc->output = 0.0f;

    // Allocate memory for the data buffer
    sinc->buffer = (ctrl_gt*)malloc(num_taps * sizeof(ctrl_gt));
    if (sinc->buffer == NULL)
        return 0;

    // Allocate memory for the look-up table (array of pointers)
    sinc->sinc_table = (ctrl_gt**)malloc(table_size * sizeof(ctrl_gt*));
    if (sinc->sinc_table == NULL)
    {
        free(sinc->buffer);
        return 0;
    }

    // Allocate memory for each row of the table
    for (i = 0; i < table_size; i++)
    {
        sinc->sinc_table[i] = (ctrl_gt*)malloc(num_taps * sizeof(ctrl_gt));
        if (sinc->sinc_table[i] == NULL)
        {
            // Clean up on failure
            for (j = 0; j < i; j++)
                free(sinc->sinc_table[j]);
            free(sinc->sinc_table);
            free(sinc->buffer);
            return 0;
        }
    }

    // --- Key Point Analysis 1: Pre-calculate the Windowed-Sinc Coefficient Table ---
    // This is the core of the module. By pre-calculating all potentially needed FIR
    // filter coefficients at once, it avoids expensive sin/cos calculations in the real-time loop.
    for (i = 0; i < table_size; i++)
    {
        // 'fractional_offset' represents the sub-sample offset (0.0 to 1.0) for the filter currently being calculated.
        float fractional_offset = (float)i / table_size;

        for (j = 0; j < num_taps; j++)
        {
            // 't' is the time-axis variable for the Sinc function, shifted and centered.
            float t = (float)j - (float)(num_taps - 1) / 2.0f - fractional_offset;

            // Calculate the Sinc function value: sin(pi*t) / (pi*t)
            float sinc_val;
            if (t == 0.0f)
            {
                sinc_val = 1.0f;
            }
            else
            {
                sinc_val = sinf(CTL_PARAM_CONST_PI * t) / (CTL_PARAM_CONST_PI * t);
            }

            // Calculate the Blackman window value to smooth the truncation effects of the Sinc function
            // and reduce Gibbs phenomenon in the frequency domain.
            float window_val = 0.42f - 0.5f * cosf(2.0f * CTL_PARAM_CONST_PI * j / (num_taps - 1)) +
                               0.08f * cosf(4.0f * CTL_PARAM_CONST_PI * j / (num_taps - 1));

            // The final filter coefficient is the product of the Sinc value and the window value.
            sinc->sinc_table[i][j] = (ctrl_gt)(sinc_val * window_val);
        }
    }

    // Initialize the data buffer
    ctl_clear_sinc_interpolator(sinc);
    return 1;
}

void ctl_destroy_sinc_interpolator(ctl_sinc_interpolator_t* sinc)
{
    uint32_t i;

    if (sinc->sinc_table != NULL)
    {
        for (i = 0; i < sinc->table_size; i++)
        {
            if (sinc->sinc_table[i] != NULL)
            {
                free(sinc->sinc_table[i]);
            }
        }
        free(sinc->sinc_table);
        sinc->sinc_table = NULL;
    }
    if (sinc->buffer != NULL)
    {
        free(sinc->buffer);
        sinc->buffer = NULL;
    }
}

//////////////////////////////////////////////////////////////////////////
//
#include <ctl/component/intrinsic/advance/paired_lut1d.h>

void ctl_init_paired_lut1d(ctl_paired_lut1d_t* lut, const ctl_lut1d_pair_t* table, uint32_t size)
{
    if (table != NULL && size > 0)
    {
        lut->table = table;
        lut->size = size;
    }
    else
    {
        // Safe default handling for null or empty tables
        lut->table = NULL;
        lut->size = 0;
    }
}
