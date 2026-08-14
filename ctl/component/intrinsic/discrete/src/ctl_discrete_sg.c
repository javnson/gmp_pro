#include <ctl/math_block/gmp_math.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Signal Generator

#include <ctl/component/intrinsic/discrete/signal_generator.h>

void ctl_init_sine_generator(ctl_sine_generator_t* sg,
                             parameter_gt init_angle, // pu
                             parameter_gt step_angle) // pu
{
    sg->ph_cos = real2ctrl(param_cos(init_angle));
    sg->ph_sin = real2ctrl(param_sin(init_angle));

    sg->ph_sin_delta = real2ctrl(param_sin(step_angle));
    sg->ph_cos_delta = real2ctrl(param_cos(step_angle));
}

void ctl_init_ramp_generator(ctl_ramp_generator_t* _rg, ctrl_gt slope, parameter_gt amp_pos, parameter_gt amp_neg)
{
    gmp_ctl_assert(amp_neg < amp_pos);

    _rg->current = CTL_CTRL_CONST_ZERO;

    _rg->maximum = real2ctrl(amp_pos);
    _rg->minimum = real2ctrl(amp_neg);

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
    gmp_ctl_assert(isr_freq > 0.0f);
    gmp_ctl_assert(amp_neg < amp_pos);

    _rg->current = CTL_CTRL_CONST_ZERO;

    _rg->maximum = real2ctrl(amp_pos);
    _rg->minimum = real2ctrl(amp_neg);

    if (isr_freq <= 0.0f || amp_neg >= amp_pos)
    {
        _rg->slope = CTL_CTRL_CONST_ZERO;
        return;
    }

    // Frequency is signed: zero keeps the ramp stationary, while a negative
    // value decrements the accumulator and therefore represents reverse rotation.
    _rg->slope = real2ctrl((amp_pos - amp_neg) * target_freq / isr_freq);
}

void ctl_init_square_wave_generator(ctl_square_wave_generator_t* sq, parameter_gt fs, parameter_gt target_freq,
                                    parameter_gt amplitude, parameter_gt offset)
{
    gmp_ctl_assert(fs > 0.0f);

    sq->high_level = real2ctrl(offset + amplitude);
    sq->low_level = real2ctrl(offset - amplitude);
    sq->phase = CTL_CTRL_CONST_ZERO;

    sq->phase_step = real2ctrl(2.0f * CTL_PARAM_CONST_PI * target_freq / fs);
    sq->output = real2ctrl(sq->high_level);
}

void ctl_init_triangle_wave_generator(ctl_triangle_wave_generator_t* tri, parameter_gt fs, parameter_gt target_freq,
                                      parameter_gt pos_peak, parameter_gt neg_peak)
{
    gmp_ctl_assert(fs > 0.0f);
    gmp_ctl_assert(neg_peak < pos_peak);

    tri->pos_peak = real2ctrl(pos_peak);
    tri->neg_peak = real2ctrl(neg_peak);
    // The total peak-to-peak amplitude is traversed twice per period (up and down).
    // So, the time for one ramp (neg to pos) is T/2.
    // Slope = Amplitude / Time = (pos_peak - neg_peak) / ( (1/target_freq) / 2 )
    // Slope per sample = Slope / fs
    tri->slope = real2ctrl(2.0f * (pos_peak - neg_peak) * target_freq / fs);
    tri->output = real2ctrl(neg_peak);
}
