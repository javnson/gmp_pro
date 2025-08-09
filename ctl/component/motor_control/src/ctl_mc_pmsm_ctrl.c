/**
 * @file pmsm_ctrl.c
 * @author javnson (javnson@zju.edu.cn)
 * @brief Implementation file for the PMSM Bare-metal Controller.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This file contains the function definitions for initializing and configuring
 * the PMSM controller.
 */

#include <gmp_core.h>

#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl.h>

/**
 * @ingroup MC_PMSM_FUNCTIONS
 * @brief Configures and initializes a PMSM controller instance based on provided parameters.
 * @details This function sets up the PID controllers and other internal parameters of the controller
 * instance according to the values specified in the `init` structure. It must be called before
 * the controller is used.
 * @param[out] ctrl Pointer to the PMSM controller instance to be initialized.
 * @param[in] init Pointer to the initialization structure containing all configuration parameters.
 */
void ctl_init_pmsm_bare_controller(pmsm_bare_controller_t* ctrl, pmsm_bare_controller_init_t* init)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    // --- Initialize discrete PID controllers ---
    // d-axis current controller
    ctl_init_discrete_pid(&ctrl->current_ctrl[phase_d], init->current_pid_gain, init->current_Ti, init->current_Td,
                          init->fs);
    ctl_set_discrete_pid_limit(&ctrl->current_ctrl[phase_d], init->voltage_limit_max, init->voltage_limit_min);

    // q-axis current controller
    ctl_init_discrete_pid(&ctrl->current_ctrl[phase_q], init->current_pid_gain, init->current_Ti, init->current_Td,
                          init->fs);
    ctl_set_discrete_pid_limit(&ctrl->current_ctrl[phase_q], init->voltage_limit_max, init->voltage_limit_min);

    // Velocity controller
    ctl_init_tracking_pid(&ctrl->spd_ctrl, init->spd_pid_gain, init->spd_Ti, init->spd_Td,
                                init->current_limit_max, init->current_limit_min, init->acc_limit_max,
                                init->acc_limit_min, init->spd_ctrl_div, init->fs);
#else  // using continuous controller
    // --- Initialize continuous PID controllers ---
    // d-axis current controller
    ctl_init_pid_ser(&ctrl->current_ctrl[phase_d], init->current_pid_gain, init->current_Ti, init->current_Td,
                     init->fs);
    ctl_set_pid_limit(&ctrl->current_ctrl[phase_d], init->voltage_limit_max, init->voltage_limit_min);

    // q-axis current controller
    ctl_init_pid_ser(&ctrl->current_ctrl[phase_q], init->current_pid_gain, init->current_Ti, init->current_Td,
                     init->fs);
    ctl_set_pid_limit(&ctrl->current_ctrl[phase_q], init->voltage_limit_max, init->voltage_limit_min);

    // Velocity controller
    ctl_init_tracking_continuous_pid(&ctrl->spd_ctrl, init->spd_pid_gain, init->spd_Ti, init->spd_Td,
                                     init->current_limit_max, init->current_limit_min, init->acc_limit_max,
                                     init->acc_limit_min, init->spd_ctrl_div, init->fs);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

    // --- Clear intermediate variables ---
    ctl_vector3_clear(&ctrl->iab0);
    ctl_vector3_clear(&ctrl->uab0);
    ctl_vector3_clear(&ctrl->idq0);
    ctl_vector3_clear(&ctrl->udq0);

    // --- Clear feed-forward parameters ---
    ctl_vector2_clear(&ctrl->idq_ff);
    ctl_vector2_clear(&ctrl->vdq_ff);

    // --- Clear set-point parameters ---
    ctl_vector3_clear(&ctrl->vab0_set);
    ctl_vector3_clear(&ctrl->vdq_set);
    ctl_vector2_clear(&ctrl->idq_set);
    ctrl->speed_set = 0;
    ctrl->pos_set = 0;
    ctrl->revolution_set = 0;

    // --- Set initial state ---
    // Disable controller and set to a safe default mode
    ctl_disable_pmsm_ctrl(ctrl);
    ctl_pmsm_ctrl_valphabeta_mode(ctrl);
}

/**
 * @ingroup MC_PMSM_FUNCTIONS
 * @brief Attaches the PMSM controller to a three-phase PWM output interface.
 * @details This function links the controller's output to a hardware-specific PWM interface,
 * establishing the path for the final control signal.
 * @param[out] ctrl Pointer to the PMSM controller instance.
 * @param[in] pwm_out Pointer to the three-phase PWM interface instance.
 */
void ctl_attach_pmsm_bare_output(pmsm_bare_controller_t* ctrl, tri_pwm_ift* pwm)
{
    ctrl->pwm_out = pwm;
}
