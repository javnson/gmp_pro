
#ifndef _FILE_MOTOR_PER_UNIT_CONSULTANT_H_
#define _FILE_MOTOR_PER_UNIT_CONSULTANT_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

typedef struct _tag_per_unit_consultant_t
{
    parameter_gt base_voltage; // Vrms: V
    parameter_gt base_power;   // P per phase: W
    parameter_gt base_current; // Irms: A
    uint32_t pole_pairs;       // motor pole pairs
    uint32_t phases;           // motor phases

    parameter_gt base_freq;       // f: Hz
    parameter_gt base_omega;      // omega_e :rad/s
    parameter_gt base_speed;      // omega: rad/s
    parameter_gt base_speed_krpm; // n: krpm

    parameter_gt base_impedence;   // R: Ohm
    parameter_gt base_inductance;  // L: H
    parameter_gt base_capacitance; // C: F
    parameter_gt base_flux;        // phi: Wb
    parameter_gt base_torque;      // T: N`m

    parameter_gt base_inst_current; // instantaneous current: A

    parameter_gt base_inst_voltage; // instantaneous voltage: V

    // parameter_gt base_inertia; // H: kg m^2

} ctl_per_unit_consultant_t;

// init a per unit consultant object
void ctl_setup_per_unit_consultant_by_puf(ctl_per_unit_consultant_t *pu, uint32_t pole_pairs, uint32_t phases,
                                          parameter_gt rated_power, parameter_gt rated_voltage_phase_rms,
                                          parameter_gt rated_freq);

// horsepower to SI power
parameter_gt ctl_helper_hp2power(parameter_gt hp);

// connected in wye, nominal voltage RMS line-to-line -> phase voltage
GMP_STATIC_INLINE
parameter_gt ctl_helper_wye_line_voltage_to_phase(parameter_gt wye_value)
{
    return wye_value / CTL_PARAM_CONST_SQRT3;
}

// connected in delta, nominal voltage RMS line to line -> phase voltage
GMP_STATIC_INLINE
parameter_gt ctl_helper_delta_line_voltage_to_phase(parameter_gt delta_value)
{
    return delta_value;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_PER_UNIT_CONSULTANT_H_
