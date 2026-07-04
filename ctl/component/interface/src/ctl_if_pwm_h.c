#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// Single Phase Modulation
//////////////////////////////////////////////////////////////////////////
#include <ctl/component/interface/hpwm_modulator.h>

void ctl_init_single_phase_H_modulation(single_phase_H_modulation_t* bridge, pwm_gt pwm_full_scale, pwm_gt pwm_deadband,
                                        ctrl_gt current_deadband)
{
    bridge->pwm_full_scale = pwm_full_scale;
    // The deadband value is typically applied symmetrically, so we store half of it.
    bridge->pwm_deadband = pwm_deadband / 2;
    bridge->current_deadband = current_deadband;

    bridge->flag_enable_dbcomp = 0;

    ctl_clear_single_phase_H_modulation(bridge);
}
