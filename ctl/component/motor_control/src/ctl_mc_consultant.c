#include <gmp_core.h>

#include <ctl/math_block/const/math_param_const.h>

#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>

void ctl_setup_per_unit_consultant_by_puf(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                          parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                          parameter_gt rated_freq)
{
    pu->pole_pairs = pole_pairs;
    pu->phases = phases;

    pu->base_voltage = rated_voltage_phase_rms;
    pu->base_power = rated_power / pu->phases;
    pu->base_current = rated_power / pu->base_voltage;

    pu->base_inst_current = pu->base_current * CTL_PARAM_CONST_SQRT2;
    pu->base_inst_voltage = pu->base_voltage * CTL_PARAM_CONST_SQRT2;

    pu->base_freq = rated_freq;
    pu->base_omega = rated_freq * 2 * CTL_PARAM_CONST_PI;
    pu->base_speed = pu->base_omega / pole_pairs;
    pu->base_speed_krpm = (parameter_gt)60.0 * rated_freq / pole_pairs;

    pu->base_impedence = pu->base_voltage / pu->base_current;
    pu->base_inductance = pu->base_impedence / pu->base_omega;
    pu->base_capacitance = 1 / pu->base_impedence / pu->base_omega;
    pu->base_flux = pu->base_inst_voltage / pu->base_omega;
    pu->base_torque = pu->base_power * pu->phases / pu->base_speed;
}

// horsepower to SI power
parameter_gt ctl_helper_hp2power(parameter_gt hp)
{
    return (parameter_gt)746.0 * hp * 1000;
}
