/**
 * @file peripheral_if_util_init.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// ADC channel

#include <ctl/component/interface/adc_channel.h>

void ctl_init_adc_channel(adc_channel_t* adc_obj, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    adc_obj->raw = 0;
    adc_obj->bias = bias;
    adc_obj->gain = gain;
    adc_obj->iqn = iqn;
    adc_obj->resolution = resolution;
    adc_obj->control_port.value = 0;
}

void ctl_init_adc_dual_channel(dual_adc_channel_t* adc, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    int i = 0;

    for (i = 0; i < 2; ++i)
    {
        adc->raw[i] = 0;
        adc->bias[i] = bias;
        adc->gain[i] = gain;
        adc->control_port.value.dat[i] = 0;
    }

    adc->resolution = resolution;
    adc->iqn = iqn;
}

void ctl_init_tri_adc_channel(tri_adc_channel_t* adc, ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    int i = 0;

    for (i = 0; i < 3; ++i)
    {
        adc->raw[i] = 0;
        adc->bias[i] = bias;
        adc->gain[i] = gain;
        adc->control_port.value.dat[i] = 0;
    }

    adc->resolution = resolution;
    adc->iqn = iqn;
}

void ctl_init_adc_calibrator(adc_bias_calibrator_t* obj, parameter_gt fc, parameter_gt Q, parameter_gt fs)
{
    uint32_t total_period = (uint32_t)(10 * fc);

    // setup the filter
    ctl_init_biquad_lpf(&obj->filter, fs, fc, Q);

    // at least 1000 period
    if (total_period > 1000)
        obj->filter_period = total_period;
    else
        obj->filter_period = 1000;

    // init parameters
    obj->filter_tick = 0;
    obj->raw = 0;

    obj->output_valid = 0;
    obj->enable_filter = 0;
}

void ctl_enable_adc_calibrator(adc_bias_calibrator_t* obj)
{

    ctl_clear_biquad_filter(&obj->filter);

    obj->filter_tick = 0;
    obj->output_valid = 0;
    obj->enable_filter = 1;
}

//////////////////////////////////////////////////////////////////////////
// PWM channel

#include <ctl/component/interface/pwm_channel.h>

void ctl_init_pwm_channel(pwm_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale)

{
    pwm_obj->raw.value = 0;
    pwm_obj->value = 0;
    pwm_obj->full_scale = full_scale;
    pwm_obj->phase = phase;
}

//////////////////////////////////////////////////////////////////////////
// PWM dual channel

// setup pwm object
void ctl_init_pwm_dual_channel(pwm_dual_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale)
{
    fast_gt i;

    for (i = 0; i < 2; ++i)
    {
        pwm_obj->raw.value.dat[i] = 0;
        pwm_obj->value[i] = 0;
    }

    pwm_obj->full_scale = full_scale;
    pwm_obj->phase = phase;
}

//////////////////////////////////////////////////////////////////////////
// PWM tri channel

// setup PWM object
void ctl_init_pwm_tri_channel(pwm_tri_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        pwm_obj->raw.value.dat[i] = 0;
        pwm_obj->value[i] = 0;
    }

    pwm_obj->full_scale = full_scale;
    pwm_obj->phase = phase;
}

//////////////////////////////////////////////////////////////////////////
// ADC ptr channel init functions

#include <ctl/component/interface/adc_ptr_channel.h>

void ctl_init_ptr_adc_channel(
    // ptr_adc object
    ptr_adc_channel_t* adc,
    // pointer to ADC raw data
    adc_gt* adc_target,
    // ADC Channel settings.
    // iqn is valid only when ctrl_gt is a fixed point type.
    ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    adc->raw = adc_target;

    // adc->raw = 0;
    adc->bias = bias;

    adc->gain = gain;
    adc->resolution = resolution;

#if defined CTRL_GT_IS_FIXED
    adc->shift_index = iqn - adc_obj->resolution;
#elif defined CTRL_GT_IS_FLOAT
    adc->per_unit_base = (ctrl_gt)1.0 / (1 << adc->resolution);

    // iqn parameter is not used here
    UNUSED_PARAMETER(iqn);
#endif

    adc->control_port.value = 0;
}

void ctl_init_dual_ptr_adc_channel(
    // ptr_adc object
    dual_ptr_adc_channel_t* adc,
    // pointer to ADC raw data
    adc_gt* adc_target,
    // ADC Channel settings.
    // iqn is valid only when ctrl_gt is a fixed point type.
    ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    int i = 0;

    adc->raw = adc_target;

    for (i = 0; i < 2; ++i)
    {
        adc->bias[i] = bias;
        adc->gain[i] = gain;
        adc->control_port.value.dat[i] = 0;
    }

    adc->resolution = resolution;

#if defined CTRL_GT_IS_FIXED
    adc->shift_index = iqn - adc_obj->resolution;
#elif defined CTRL_GT_IS_FLOAT
    adc->per_unit_base = (ctrl_gt)1.0 / (1 << adc->resolution);

    // iqn parameter is not used here
    UNUSED_PARAMETER(iqn);
#endif
}

void ctl_init_tri_ptr_adc_channel(
    // ptr_adc object
    tri_ptr_adc_channel_t* adc,
    // pointer to ADC raw data
    adc_gt* adc_target,
    // ADC Channel settings.
    // iqn is valid only when ctrl_gt is a fixed point type.
    ctrl_gt gain, ctrl_gt bias, fast_gt resolution, fast_gt iqn)
{
    int i = 0;

    adc->raw = adc_target;

    for (i = 0; i < 3; ++i)
    {
        adc->bias[i] = bias;
        adc->gain[i] = gain;
        adc->control_port.value.dat[i] = 0;
    }

    adc->resolution = resolution;

#if defined CTRL_GT_IS_FIXED
    adc->shift_index = iqn - adc_obj->resolution;
#elif defined CTRL_GT_IS_FLOAT
    adc->per_unit_base = (ctrl_gt)1.0 / (1 << adc->resolution);

    // iqn parameter is not used here
    UNUSED_PARAMETER(iqn);
#endif
}

//////////////////////////////////////////////////////////////////////////
// SPWM modulator

#include <ctl/component/interface/pwm_modulator.h>

void ctl_init_spwm_modulator(spwm_modulator_t* mod, pwm_gt pwm_full_scale, pwm_gt pwm_deadband_comp_val,
                             ctl_vector3_t* _iuvw, ctrl_gt current_deadband, ctrl_gt current_hysteresis)
{
    mod->iuvw = _iuvw;
    mod->pwm_full_scale = pwm_full_scale;
    mod->pwm_deadband_comp_val = pwm_deadband_comp_val;

    mod->current_deadband = current_deadband;
    mod->current_hysteresis_band = current_hysteresis;

    ctl_clear_spwm_modulator(mod);
}

void ctl_init_npc_modulator(npc_modulator_t* mod, pwm_gt pwm_full_scale, pwm_gt pwm_deadband_comp_val,
    ctl_vector3_t* _iuvw, ctrl_gt current_deadband, ctrl_gt current_hysteresis)
{
    mod->iuvw = _iuvw;
    mod->pwm_full_scale = pwm_full_scale;
    mod->pwm_deadband_comp_val = pwm_deadband_comp_val;

    mod->current_deadband = current_deadband;
    mod->current_hysteresis_band = current_hysteresis;

    ctl_clear_npc_modulator(mod);
}
