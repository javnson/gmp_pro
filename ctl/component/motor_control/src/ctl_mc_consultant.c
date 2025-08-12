#include <gmp_core.h>

#include <ctl/math_block/const/math_param_const.h>

#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>

static void calculate_all_base_values(ctl_per_unit_consultant_t* pu)
{
    if (pu->base_voltage <= 0 || pu->base_power <= 0 || pu->base_freq <= 0 || pu->pole_pairs == 0)
    {
        return;
    }
    // 1. elec base value
    pu->base_omega = 2.0f * M_PI * pu->base_freq;
    pu->base_current = pu->base_power / pu->base_voltage;
    pu->base_impedence = pu->base_voltage / pu->base_current;
    pu->base_inductance = pu->base_impedence / pu->base_omega;
    pu->base_capacitance = 1.0f / (pu->base_impedence * pu->base_omega);
    pu->base_flux = pu->base_voltage / pu->base_omega;
    pu->base_inst_voltage = pu->base_voltage * sqrtf(2.0);
    pu->base_inst_current = pu->base_current * sqrtf(2.0);

    // 2. 机械基值 (基于同步速)
    pu->base_speed = pu->base_omega / pu->pole_pairs;
    parameter_gt total_base_power = pu->base_power * pu->phases;
    pu->base_torque = total_base_power / pu->base_speed;
    parameter_gt base_rpm = pu->base_speed * (60.0f / (2.0f * M_PI));
    pu->base_speed_krpm = base_rpm / 1000.0f;
}

void ctl_init_per_unit_consultant_pmsm(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                       parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                       parameter_gt rated_freq)
{
    if (!pu)
        return;

    pu->pole_pairs = pole_pairs;
    pu->phases = phases;
    pu->base_power = rated_power / (parameter_gt)phases;
    pu->base_voltage = rated_voltage_phase_rms;
    pu->base_freq = rated_freq;

    calculate_all_base_values(pu);
}

void ctl_init_per_unit_consultant_acm(ctl_per_unit_consultant_t* pu, uint32_t pole_pairs, uint32_t phases,
                                      parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                      parameter_gt synchronous_freq, parameter_gt rated_spd_krpm)
{
    if (!pu)
        return;

    pu->pole_pairs = pole_pairs;
    pu->phases = phases;
    pu->base_power = rated_power / (parameter_gt)phases; // 基准功率为每相功率
    pu->base_voltage = rated_voltage_phase_rms;

    pu->base_freq = synchronous_freq;

    calculate_all_base_values(pu);

    pu->base_speed_krpm = rated_spd_krpm;
}
