// Single Phase PFC with uncontrolled rectification

#ifndef _FILE_GMP_CTL_SPFC_H_
#define _FILE_GMP_CTL_SPFC_H_

// This module implement a single phase PLL
// main target is to let current follow voltage
typedef struct _tag_spfc_type
{
    //
    // Input Section
    //

    // grid side voltage
    ctrl_gt rectifier_voltage;

    // DC side current
    ctrl_gt rectifier_current;

    // DC side output voltage
    ctrl_gt dc_voltage;

    //
    // Output Section
    //

    // output boost controller modulation result
    ctrl_gt boost_modulation;

    //
    // Intrinsic Variables
    //

    // voltage reference for DC bus
    ctrl_gt voltage_set;

    // current aptitude command
    ctrl_gt current_set;

    // current reference for inductor
    ctrl_gt current_ref;

    //
    // Parameters
    //

    //
    // Submodules
    //

    // current controller
    pid_regular_t current_ctrl;

    // voltage controller
    pid_regular_t voltage_ctrl;

} spfc_t;

GMP_STATIC_INLINE
void ctl_clear_pfc_ctrl(spfc_t *pfc)
{
    ctl_clear_pid(&pfc->current_ctrl);
    ctl_clear_pid(&pfc->voltage_ctrl);
}

GMP_STATIC_INLINE
ctrl_gt ctl_step_spfc_ctrl(
    // handle of PFC controller
    spfc_t *pfc,
    // voltage after rectifier
    ctrl_gt rectifier_voltage,
    // current after rectifier, that is current on inductor
    ctrl_gt rectifier_current,
    // voltage of DC bus
    ctrl_gt dc_voltage)
{
    // DC Bus voltage controller
    pfc->current_set = ctl_step_pid_ser(&pfc->voltage_ctrl, pfc->voltage_set - dc_voltage);

    // modulation current target with voltage real value
    pfc->current_ref = ctl_mul(pfc->current_set, rectifier_voltage);

    // Current controller
    pfc->boost_modulation = ctl_step_pid_ser(&pfc->current_ctrl, pfc->current_ref - rectifier_current);

    // return modulation
    return pfc->boost_modulation;
}

void ctl_init_spfc_ctrl(
    // handle of PFC controller
    spfc_t *pfc,
    // voltage controller parameters
    parameter_gt voltage_kp, parameter_gt voltage_Ti, parameter_gt voltage_Td,
    // current controller parameters
    parameter_gt current_kp, parameter_gt current_Ti, parameter_gt current_Td,
    // controller frequency
    parameter_gt fs);

#endif // _FILE_GMP_CTL_SPFC_H_
