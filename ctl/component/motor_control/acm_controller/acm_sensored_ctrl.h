/**
 * @file acm_sensored_ctrl.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a complete sensored Field-Oriented Control (FOC) for AC Induction Motors (ACM).
 * @version 0.2
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file provides a comprehensive controller for AC induction motors, including
 * current and speed control loops, coordinate transformations, and flux estimation.
 * It is designed to be highly configurable through preprocessor macros.
 */

#ifndef _FILE_ACM_SENSORED_CTRL_BARE_H_
#define _FILE_ACM_SENSORED_CTRL_BARE_H_

// Necessary support
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef ACM_CTRL_USING_DISCRETE_CTRL
#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>
#else
#include <ctl/component/intrinsic/continuous/track_pid.h>
#endif // ACM_CTRL_USING_DISCRETE_CTRL

#include <ctl/component/motor_control/basic/decouple.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/observer/acm.pos_calc.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup MC_ACM_SENSORED_CONTROLLER Sensored ACM FOC Controller
 * @brief A complete Field-Oriented Controller for AC Induction Motors.
 *
 * @details
 * ### ACM Bare Controller Usage:
 *
 * 1.  **Attach physical interfaces:**
 * - This task should be completed in a platform-specific file.
 * - Use `ctl_attach_acm_sensored_bare_output()` to attach the PWM interface.
 * - Use `ctl_attach_acm_sensored_bare_rotor_postion()` to attach the rotor position interface.
 * - Use `ctl_attach_mtr_adc_channels()` to attach ADC interfaces.
 * - Use `ctl_attach_mtr_position()` to attach the flux position encoder.
 * - Use `ctl_attach_mtr_velocity()` to attach the velocity encoder.
 *
 * 2.  **Initialize the controller structure:**
 * - Fill the `acm_sensored_bare_controller_init_t` structure with motor and controller parameters.
 * - Call `ctl_init_acm_sensored_bare_controller()` to initialize the controller entity.
 *
 * 3.  **Select an operating mode and provide a control target:**
 * The controller supports the following modes:
 * - **Valpha, Vbeta Mode**: Open-loop voltage control in the stationary frame.
 * - Enter with `ctl_acm_sensored_ctrl_valphabeta_mode()`.
 * - Set targets with `ctl_set_acm_sensored_ctrl_valphabeta()`.
 * - **Voltage (Vd-Vq) Mode**: Open-loop voltage control in the rotating frame.
 * - Enter with `ctl_acm_sensored_ctrl_voltage_mode()`.
 * - Set targets with `ctl_set_acm_sensored_ctrl_vdq_ff()`.
 * - **Current (Id-Iq) Mode**: Closed-loop current control.
 * - Enter with `ctl_acm_sensored_ctrl_current_mode()`.
 * - Set targets with `ctl_set_acm_sensored_ctrl_idq_ff()`.
 * - **Velocity Mode**: Closed-loop speed control.
 * - Enter with `ctl_acm_sensored_ctrl_velocity_mode()`.
 * - Set targets with `ctl_set_acm_sensored_ctrl_speed()`.
 * > **Note:** These functions perform an immediate switch. For smooth transitions during runtime, an external algorithm is required.
 *
 * 4.  **Invoke the controller in the main ISR:**
 * - First, call the `mtr_interface` step functions to update sensor inputs.
 * - Then, call `ctl_step_acm_sensored_ctrl()` to execute the control logic.
 * - Finally, call the PWM interface function to output the modulation result.
 *
 * 5.  **Enable / Disable the controller:**
 * - Use `ctl_enable_acm_sensored_ctrl()` to enable the controller.
 * - Use `ctl_disable_acm_sensored_ctrl()` to disable it.
 * > **Note:** Call `ctl_clear_acm_sensored_ctrl()` before enabling the controller to reset its internal states.
 *
 * 6.  **Enable / Disable the flux estimator:**
 * - Use `ctl_enable_acm_sensored_ctrl_flux_est()` to enable flux estimation.
 * - Use `ctl_disable_acm_sensored_ctrl_flux_est()` to disable it.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Configuration Macros                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ACM_CONFIG Configuration Macros
 * @ingroup MC_ACM_SENSORED_CONTROLLER
 * @brief Compile-time configurations for the ACM controller.
 * @{
 */

/**
 * @brief Selects the number of phase currents to measure (2 or 3).
 */
#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES ((2))
#endif

/**
 * @brief Selects the number of phase voltages to measure (0, 2, or 3).
 */
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((3))
#endif

/**
 * @brief Selects the feedforward strategy (0 for manual, 1 for decoupling).
 */
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (0)
#endif

/** @} */ // end of MC_ACM_CONFIG group

/*---------------------------------------------------------------------------*/
/* Data Structures                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ACM_STRUCTS Data Structures
 * @ingroup MC_ACM_SENSORED_CONTROLLER
 * @brief Data structures for the ACM controller and its initialization.
 * @{
 */

/**
 * @brief Main data structure for the sensored ACM controller.
 */
typedef struct _tag_acm_sensored_bare_controller
{
    // --- Interfaces ---
    mtr_ift mtr_interface;   /**< @brief Universal motor interface for sensors (flux position, rotor velocity). */
    rotation_ift* rotor_pos; /**< @brief Direct interface to the rotor position encoder. */
    tri_pwm_ift* pwm_out;    /**< @brief Output interface for PWM signals. */

    // --- Controller Components ---
#ifdef ACM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2]; /**< @brief Discrete PID controllers for d and q axis currents. */
    track_discrete_pid_t spd_ctrl;  /**< @brief Discrete tracking PID controller for speed. */
#else
    ctl_pid_t current_ctrl[2];              /**< @brief Continuous PID controllers for d and q axis currents. */
    ctl_tracking_continuous_pid_t spd_ctrl; /**< @brief Continuous tracking PID controller for speed. */
#endif
    spd_calculator_t spd_enc;    /**< @brief Rotor speed calculator. */
    ctl_im_spd_calc_t flux_calc; /**< @brief Flux speed and angle calculator. */
    ctl_slope_f_controller rg;   /**< @brief Ramp generator for smooth frequency changes. */
    ctrl_gt speed_pu_rg_sf;      /**< @brief Scale factor from speed (p.u.) to ramp generator frequency. */

    // --- Intermediate Variables ---
    vector3_gt iab0; /**< @brief Stationary frame currents (alpha, beta, 0). */
    vector3_gt idq0; /**< @brief Rotating frame currents (d, q, 0). */
    vector3_gt uab0; /**< @brief Stationary frame voltages (alpha, beta, 0). */
    vector3_gt udq0; /**< @brief Rotating frame voltages (d, q, 0). */

    // --- Feedforward & Setpoints ---
    vector2_gt idq_ff;   /**< @brief Feedforward currents (d, q). */
    vector2_gt vdq_ff;   /**< @brief Feedforward voltages (d, q). */
    ctrl_gt speed_set;   /**< @brief Speed reference (p.u.). */
    vector2_gt idq_set;  /**< @brief Current references (d, q). */
    vector3_gt vdq_set;  /**< @brief Voltage references (d, q, 0). */
    vector3_gt vab0_set; /**< @brief Stationary frame voltage references (alpha, beta, 0). */

    // --- Flags ---
    fast16_gt isr_tick;                /**< @brief ISR execution counter. */
    fast_gt flag_enable_controller;    /**< @brief Master enable for the entire controller. */
    fast_gt flag_enable_output;        /**< @brief Enables PWM output generation. */
    fast_gt flag_enable_modulation;    /**< @brief Enables the inverse Park and SVPWM stages. */
    fast_gt flag_enable_current_ctrl;  /**< @brief Enables the current control loop. */
    fast_gt flag_enable_velocity_ctrl; /**< @brief Enables the speed control loop. */
    fast_gt flag_enable_flux_est;      /**< @brief Enables the flux estimator. */

} acm_sensored_bare_controller_t;

/**
 * @brief Initialization structure for the sensored ACM controller.
 */
typedef struct _tag_acm_sensored_bare_controller_init
{
    // --- Controller Timing ---
    parameter_gt fs; /**< @brief Controller execution frequency in Hz. */

    // --- Current Controller Parameters ---
    parameter_gt current_pid_gain;  /**< @brief Proportional gain (Kp) for the current controller. */
    parameter_gt current_Ti;        /**< @brief Integral time constant (Ti) for the current controller. */
    parameter_gt current_Td;        /**< @brief Derivative time constant (Td) for the current controller. */
    parameter_gt voltage_limit_max; /**< @brief Upper saturation limit for the current controller output (voltage). */
    parameter_gt voltage_limit_min; /**< @brief Lower saturation limit for the current controller output (voltage). */

    // --- Speed Controller Parameters ---
    parameter_gt spd_pid_gain;      /**< @brief Proportional gain (Kp) for the speed controller. */
    parameter_gt spd_Ti;            /**< @brief Integral time constant (Ti) for the speed controller. */
    parameter_gt spd_Td;            /**< @brief Derivative time constant (Td) for the speed controller. */
    parameter_gt current_limit_max; /**< @brief Upper saturation limit for the speed controller output (current). */
    parameter_gt current_limit_min; /**< @brief Lower saturation limit for the speed controller output (current). */
    parameter_gt acc_limit_max;     /**< @brief Upper saturation limit for the tracking integral (acceleration). */
    parameter_gt acc_limit_min;     /**< @brief Lower saturation limit for the tracking integral (deceleration). */
    uint32_t spd_ctrl_div;          /**< @brief Execution frequency divider for the speed controller. */

    // --- Motor Parameters ---
    parameter_gt Lr;        /**< @brief Rotor inductance (H). */
    parameter_gt Rr;        /**< @brief Rotor resistance (Ohm). */
    parameter_gt base_freq; /**< @brief Stator base frequency (Hz). */
    parameter_gt base_spd;  /**< @brief Rotor base speed (rpm). */
    uint16_t pole_pairs;    /**< @brief Number of motor pole pairs. */

    // --- Ramp Generator Parameters ---
    parameter_gt target_freq_slope; /**< @brief Frequency slope for the open-loop ramp generator (Hz/s). */

} acm_sensored_bare_controller_init_t;

/** @} */ // end of MC_ACM_STRUCTS group

/*---------------------------------------------------------------------------*/
/* Function Prototypes and Inline Implementations                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ACM_FUNCTIONS Functions
 * @ingroup MC_ACM_SENSORED_CONTROLLER
 * @brief API functions for controlling and configuring the ACM controller.
 * @{
 */

// --- Initialization and Attachment ---

/**
 * @brief Initializes the sensored ACM controller.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  init Pointer to the initialization parameters.
 */
void ctl_init_acm_sensored_bare_controller(acm_sensored_bare_controller_t* ctrl,
                                           acm_sensored_bare_controller_init_t* init);

/**
 * @brief Attaches the PWM output interface to the controller.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  pwm_out Pointer to the PWM interface.
 */
void ctl_attach_acm_sensored_bare_output(acm_sensored_bare_controller_t* ctrl, tri_pwm_ift* pwm_out);

/**
 * @brief Attaches the rotor position encoder interface to the controller.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  rotor_enc Pointer to the rotor position encoder interface.
 */
void ctl_attach_acm_sensored_bare_rotor_postion(acm_sensored_bare_controller_t* ctrl, rotation_ift* rotor_enc);

// --- Core Controller Logic ---

/**
 * @brief Resets the internal states of the PID controllers.
 * @param[in,out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_acm_sensored_ctrl(acm_sensored_bare_controller_t* ctrl)
{
#ifdef ACM_CTRL_USING_DISCRETE_CTRL
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_discrete_track_pid(&ctrl->spd_ctrl);
#else
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_continuous_pid(&ctrl->spd_ctrl);
#endif
}

/**
 * @brief Executes one step of the sensored ACM control algorithm.
 * This function should be called periodically in the main control ISR.
 * @param[in,out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_step_acm_sensored_ctrl(acm_sensored_bare_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;

    ctrl->isr_tick++;

    // Ramp generator
    ctl_step_slope_f(&ctrl->rg);

    // Clarke Transformation
#if MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 3
    ctl_ct_clarke(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#elif MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 2
    ctl_ct_clarke_2ph((ctl_vector2_t*)&ctrl->mtr_interface.iabc->value, (ctl_vector2_t*)&ctrl->iab0);
    ctrl->iab0.dat[phase_0] = 0;
#endif

    // Park Transformation
    etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
    ctl_set_phasor_via_angle(etheta, &phasor);
    ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);

    // Speed calculation
    ctl_step_spd_calc(&ctrl->spd_enc);

    if (ctrl->flag_enable_controller)
    {
        // Flux estimation
        if (ctrl->flag_enable_flux_est)
        {
            ctl_step_im_spd_calc(&ctrl->flux_calc, ctrl->idq0.dat[phase_d], ctrl->idq0.dat[phase_q],
                                 ctl_get_mtr_velocity(&ctrl->mtr_interface));
        }

        // Set speed target
        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->rg.target_frequency = ctl_mul(ctrl->flux_calc.omega_s, ctrl->speed_pu_rg_sf);
        }
        else
        {
            ctrl->rg.target_frequency = ctl_mul(ctrl->speed_set, ctrl->speed_pu_rg_sf);
        }

        // Velocity Controller
        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef ACM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] = ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                     ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#else
            ctrl->idq_set.dat[phase_q] = ctl_step_tracking_continuous_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                          ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#endif
        }
        else
        {
            ctl_vector2_copy(&ctrl->idq_set, &ctrl->idq_ff);
        }

        // Current Controller
        if (ctrl->flag_enable_current_ctrl)
        {
#ifdef ACM_CTRL_USING_DISCRETE_CTRL
            ctrl->vdq_set.dat[phase_d] = ctl_step_discrete_pid(&ctrl->current_ctrl[phase_d],
                                                               ctrl->idq_set.dat[phase_d] - ctrl->idq0.dat[phase_d]) +
                                         ctrl->vdq_ff.dat[phase_d];
            ctrl->vdq_set.dat[phase_q] = ctl_step_discrete_pid(&ctrl->current_ctrl[phase_q],
                                                               ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
                                         ctrl->vdq_ff.dat[phase_q];
#else
            ctrl->vdq_set.dat[phase_d] =
                ctl_step_pid_ser(&ctrl->current_ctrl[phase_d], ctrl->idq_set.dat[phase_d] - ctrl->idq0.dat[phase_d]) +
                ctrl->vdq_ff.dat[phase_d];
            ctrl->vdq_set.dat[phase_q] =
                ctl_step_pid_ser(&ctrl->current_ctrl[phase_q], ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
                ctrl->vdq_ff.dat[phase_q];
#endif
            ctrl->vdq_set.dat[phase_0] = 0;
        }
        else
        {
            ctl_vector2_copy((ctl_vector2_t*)&ctrl->vdq_set, &ctrl->vdq_ff);
            ctrl->vdq_set.dat[phase_0] = 0;
        }

        // Inverse Park Transformation
        if (ctrl->flag_enable_modulation)
        {
            ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
        }

        // SVPWM
        if (ctrl->flag_enable_output)
        {
            ctl_ct_svpwm_calc(&ctrl->vab0_set, &ctrl->pwm_out->value);
        }
        else
        {
            ctl_vector3_set(&ctrl->pwm_out->value, float2ctrl(0.5));
        }
    }
    else
    {
        ctl_vector3_clear(&ctrl->pwm_out->value);
        ctl_clear_acm_sensored_ctrl(ctrl);
    }
}

// --- Enable / Disable Functions ---

/** @brief Enables the entire controller. */
GMP_STATIC_INLINE void ctl_enable_acm_sensored_ctrl(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}
/** @brief Disables the entire controller. */
GMP_STATIC_INLINE void ctl_disable_acm_sensored_ctrl(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}
/** @brief Enables the PWM output. */
GMP_STATIC_INLINE void ctl_enable_acm_sensored_ctrl_output(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
}
/** @brief Disables the PWM output. */
GMP_STATIC_INLINE void ctl_disable_acm_sensored_ctrl_output(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 0;
}
/** @brief Enables the flux estimator. */
GMP_STATIC_INLINE void ctl_enable_acm_sensored_ctrl_flux_est(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_flux_est = 1;
}
/** @brief Disables the flux estimator. */
GMP_STATIC_INLINE void ctl_disable_acm_sensored_ctrl_flux_est(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_flux_est = 0;
}

// --- Mode Control Functions ---

/** @brief Sets the controller to Valpha-Vbeta open-loop mode. */
GMP_STATIC_INLINE void ctl_acm_sensored_ctrl_valphabeta_mode(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the target Valpha and Vbeta voltages. */
GMP_STATIC_INLINE void ctl_set_acm_sensored_ctrl_valphabeta(acm_sensored_bare_controller_t* ctrl, ctrl_gt valpha,
                                                            ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_alpha] = valpha;
    ctrl->vab0_set.dat[phase_beta] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

/** @brief Sets the controller to Vd-Vq open-loop voltage mode. */
GMP_STATIC_INLINE void ctl_acm_sensored_ctrl_voltage_mode(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the feedforward/reference Vd and Vq voltages. */
GMP_STATIC_INLINE void ctl_set_acm_sensored_ctrl_vdq_ff(acm_sensored_bare_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

/** @brief Sets the controller to Id-Iq closed-loop current mode. */
GMP_STATIC_INLINE void ctl_acm_sensored_ctrl_current_mode(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the feedforward/reference Id and Iq currents. */
GMP_STATIC_INLINE void ctl_set_acm_sensored_ctrl_idq_ff(acm_sensored_bare_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

/** @brief Sets the controller to closed-loop velocity mode. */
GMP_STATIC_INLINE void ctl_acm_sensored_ctrl_velocity_mode(acm_sensored_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
}

/** @brief Sets the target speed reference. */
GMP_STATIC_INLINE void ctl_set_acm_sensored_ctrl_speed(acm_sensored_bare_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

/** @} */ // end of MC_ACM_FUNCTIONS group
/** @} */ // end of MC_ACM_SENSORED_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ACM_SENSORED_CTRL_BARE_H_
