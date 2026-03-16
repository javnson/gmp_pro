

#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/inverter_consultant.h>

/**
 * @brief Initializes and validates the Inverter hardware model.
 */
void ctl_consultant_inverter_init(ctl_consultant_inverter_t* inv, parameter_gt v_bus_nom, parameter_gt i_max_peak,
                                  parameter_gt f_pwm, parameter_gt f_ctrl, parameter_gt dt_sec, parameter_gt m_max)
{
    // 1. Strict Hardware Validation (Gatekeeper)
    // Prevent lethal configuration errors (e.g., 0Hz control frequency)
    gmp_base_assert(v_bus_nom > 0.0f);
    gmp_base_assert(i_max_peak > 0.0f);
    gmp_base_assert(f_pwm > 0.0f);
    gmp_base_assert(f_ctrl > 0.0f);
    gmp_base_assert(dt_sec >= 0.0f);
    gmp_base_assert(m_max > 0.0f && m_max <= 1.0f); // Modulation index typically bounded

    // 2. Base Parameter Assignment
    inv->nominal_v_bus = v_bus_nom;
    inv->max_i_peak = i_max_peak;
    inv->f_pwm = f_pwm;
    inv->f_ctrl = f_ctrl;
    inv->dead_time_sec = dt_sec;
    inv->max_modulation_idx = m_max;

    // 3. Pre-calculated Derived Constants
    inv->ts_ctrl = 1.0f / f_ctrl;

    // Dead-time error factor: T_dead / T_pwm = T_dead * f_pwm
    inv->v_err_deadtime_fac = dt_sec * f_pwm;

    // 4. Initialize Dynamic States assuming nominal bus voltage initially
    ctl_consultant_inverter_update_vbus(inv, v_bus_nom);
}
