#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Signal Generator

#include <ctl/component/intrinsic/discrete/signal_generator.h>

void ctl_init_sine_generator(ctl_sine_generator_t* sg,
                             parameter_gt init_angle, // pu
                             parameter_gt step_angle) // pu
{
    sg->ph_cos = float2ctrl(cosf(init_angle));
    sg->ph_sin = float2ctrl(sinf(init_angle));

    sg->ph_sin_delta = float2ctrl(sinf(step_angle));
    sg->ph_cos_delta = float2ctrl(cosf(step_angle));
}

void ctl_init_ramp_generator(ctl_ramp_generator_t* _rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg)
{
    gmp_base_assert(amp_neg < amp_pos);

    _rg->current = float2ctrl(0.0f);

    _rg->maximum = float2ctrl(amp_pos);
    _rg->minimum = float2ctrl(amp_neg);

    _rg->slope = slope;
}

void ctl_init_ramp_generator_via_freq(
    // pointer to ramp generator object
    ctl_ramp_generator_t* _rg,
    // isr frequency, unit Hz
    parameter_gt isr_freq,
    // target frequency, unit Hz
    parameter_gt target_freq,
    // ramp range
    parameter_gt amp_pos, parameter_gt amp_neg)
{
    //gmp_base_assert(target_freq > 0.0); // ·ÀÖ¹³ıÁã
    gmp_base_assert(amp_neg < amp_pos);

    _rg->current = float2ctrl(0);

    _rg->maximum = float2ctrl(amp_pos);
    _rg->minimum = float2ctrl(amp_neg);

    parameter_gt a = isr_freq / target_freq;
    parameter_gt b = amp_pos - amp_neg;

    // _rg->slope = float2ctrl((amp_pos - amp_neg) / (isr_freq / target_freq));
    _rg->slope = float2ctrl(b / a);
}

void ctl_init_square_wave_generator(ctl_square_wave_generator_t* sq, parameter_gt fs, parameter_gt target_freq,
                                    parameter_gt amplitude, parameter_gt offset)
{
    gmp_base_assert(fs > 0.0f);

    sq->high_level = float2ctrl(offset + amplitude);
    sq->low_level = float2ctrl(offset - amplitude);
    sq->phase = float2ctrl(0.0f);

    sq->phase_step = float2ctrl(2.0f * CTL_PARAM_CONST_PI * target_freq / fs);
    sq->output = float2ctrl(sq->high_level);
}

void ctl_init_triangle_wave_generator(ctl_triangle_wave_generator_t* tri, parameter_gt fs, parameter_gt target_freq,
                                      parameter_gt pos_peak, parameter_gt neg_peak)
{
    gmp_base_assert(fs > 0.0f);
    gmp_base_assert(neg_peak < pos_peak);

    tri->pos_peak = float2ctrl(pos_peak);
    tri->neg_peak = float2ctrl(neg_peak);
    // The total peak-to-peak amplitude is traversed twice per period (up and down).
    // So, the time for one ramp (neg to pos) is T/2.
    // Slope = Amplitude / Time = (pos_peak - neg_peak) / ( (1/target_freq) / 2 )
    // Slope per sample = Slope / fs
    tri->slope = float2ctrl(2.0f * (pos_peak - neg_peak) * target_freq / fs);
    tri->output = float2ctrl(neg_peak);
}
