#include <gmp_core.h>

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
