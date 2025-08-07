/**
 * @file pmsm_ctrl_smo.h
 * @brief Provides a sensorless PMSM controller using a Sliding Mode Observer (SMO).
 * @details This file implements a full sensorless FOC solution. It includes an
 * open-loop ramp-up generator for starting the motor and a Sliding Mode
 * Observer (SMO) to estimate rotor position and speed for closed-loop control.
 * A key function, `ctl_switch_pmsm_smo_ctrl_using_smo`, handles the bumpless
 * transfer from open-loop startup to sensorless closed-loop operation.
 *
 * //tex:
 * // The SMO estimates the back-EMF based on the motor model:
 * // \frac{d\hat{i}_{\alpha\beta}}{dt} = \frac{1}{L_d} (v_{\alpha\beta} - R_s\hat{i}_{\alpha\beta} - E_{\alpha\beta} - Z_{\alpha\beta})
 * // Where Z is the sliding mode correction term.
 *
 */

#ifndef _FILE_PMSM_CTRL_SMO_H_
#define _FILE_PMSM_CTRL_SMO_H_

// Necessary support
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/observer/pmsm.smo.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>
#else
#include <ctl/component/intrinsic/continuous/track_pid.h>
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

#include <ctl/component/motor_control/basic/decouple.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* PMSM SMO (Sliding Mode Observer) Sensorless Controller                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_SMO_CONTROLLER PMSM SMO Sensorless Controller
 * @brief A module for sensorless FOC of a PMSM using a Sliding Mode Observer.
 * @details This module provides a complete sensorless control strategy, including:
 * - An open-loop V/f ramp generator for reliable motor startup.
 * - A Sliding Mode Observer (SMO) for estimating rotor position and speed.
 * - A bumpless transfer mechanism to switch from startup to closed-loop control.
 * @{
 */

//================================================================================
// Pragma Defines
//================================================================================

/**
 * @brief Configures the number of phase current sensors used for measurement.
 */
#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_CURRENT_MEASUREMENT_PHASES

/**
 * @brief Configures the number of phase voltage sensors used for measurement.
 */
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

/**
 * @brief Selects the feedforward strategy for the controller.
 */
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (1)
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Main structure for the PMSM SMO Sensorless Controller.
 */
typedef struct _tag_pmsm_smo_bare_controller
{
    //--------------------------------------------------------------------------
    // Interfaces
    //--------------------------------------------------------------------------
    mtr_ift mtr_interface; ///< Input interface for motor signals.
    tri_pwm_ift* pwm_out;  ///< Output interface for three-phase PWM.

    //--------------------------------------------------------------------------
    // Controller & Observer Entities
    //--------------------------------------------------------------------------
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2]; ///< Discrete PID controllers for d-q axis currents.
    track_discrete_pid_t spd_ctrl;  ///< Discrete tracking PID controller for speed.
#else                               // use continuous controller
    ctl_pid_t current_ctrl[2];              ///< Continuous PID controllers for d-q axis currents.
    ctl_tracking_continuous_pid_t spd_ctrl; ///< Continuous tracking PID controller for speed.
#endif
    pmsm_smo_t smo;                  ///< Sliding Mode Observer instance.
    ctl_slope_f_controller ramp_gen; ///< Open-loop V/f ramp generator for startup.
    ctrl_gt ramp_freq_spd_set_sf;    ///< Scale factor from ramp frequency (Hz) to speed setpoint (p.u.).

    //--------------------------------------------------------------------------
    // Controller Intermediate Variables
    //--------------------------------------------------------------------------
    vector3_gt iab0; ///< Phase currents in the alpha-beta-zero stationary frame.
    vector3_gt idq0; ///< Phase currents in the d-q-zero rotating frame.
    vector3_gt uab0; ///< Phase voltages in the alpha-beta-zero stationary frame.
    vector3_gt udq0; ///< Phase voltages in the d-q-zero rotating frame.

    //--------------------------------------------------------------------------
    // Controller Feedforward Parameters
    //--------------------------------------------------------------------------
    vector2_gt idq_ff; ///< Feedforward values for d-q axis currents.
    vector2_gt vdq_ff; ///< Feedforward values for d-q axis voltages.

    //--------------------------------------------------------------------------
    // Controller Setpoints
    //--------------------------------------------------------------------------
    ctrl_gt speed_set;   ///< Target speed (p.u.).
    vector2_gt idq_set;  ///< Target d-q axis currents.
    vector3_gt vdq_set;  ///< Target d-q-zero axis voltages.
    vector3_gt vab0_set; ///< Target alpha-beta-zero axis voltages.

    //--------------------------------------------------------------------------
    // State Flags & Counters
    //--------------------------------------------------------------------------
    fast_gt flag_enable_controller;    ///< Master flag to enable or disable the entire controller.
    fast_gt flag_enable_output;        ///< Flag to enable or disable the PWM output.
    fast_gt flag_enable_modulation;    ///< Flag to enable inverse Park transform and modulation.
    fast_gt flag_enable_current_ctrl;  ///< Flag to enable the current control loop.
    fast_gt flag_enable_velocity_ctrl; ///< Flag to enable the velocity control loop.
    fast_gt flag_enable_smo;           ///< Flag to enable the SMO calculations.
    fast_gt flag_switch_cplt;          ///< Flag indicating the switch from open-loop to SMO is complete.

} pmsm_smo_controller_t;

/**
 * @brief Initialization parameters for the PMSM SMO Controller.
 */
typedef struct _tag_pmsm_smo_bare_controller_init
{
    parameter_gt fs; ///< Controller sampling frequency (Hz).

    //--- Current Controller ---
    parameter_gt current_pid_gain;  ///< Proportional gain (Kp) for the current controllers.
    parameter_gt current_Ti;        ///< Integral time constant (Ti) for the current controllers.
    parameter_gt current_Td;        ///< Derivative time constant (Td) for the current controllers.
    parameter_gt voltage_limit_max; ///< Maximum output limit for the current controllers (voltage).
    parameter_gt voltage_limit_min; ///< Minimum output limit for the current controllers (voltage).

    //--- Speed Controller ---
    parameter_gt spd_pid_gain;      ///< Proportional gain (Kp) for the speed controller.
    parameter_gt spd_Ti;            ///< Integral time constant (Ti) for the speed controller.
    parameter_gt spd_Td;            ///< Derivative time constant (Td) for the speed controller.
    parameter_gt current_limit_max; ///< Maximum output limit for the speed controller (q-axis current).
    parameter_gt current_limit_min; ///< Minimum output limit for the speed controller (q-axis current).
    parameter_gt acc_limit_max;     ///< Maximum acceleration limit (p.u./s).
    parameter_gt acc_limit_min;     ///< Minimum deceleration limit (p.u./s).
    uint32_t spd_ctrl_div;          ///< Speed controller execution frequency divider.

    //--- SMO Observer ---
    parameter_gt Rs;             ///< Motor Stator Resistance (Ohm).
    parameter_gt Ld;             ///< Motor d-axis Inductance (H).
    parameter_gt Lq;             ///< Motor q-axis Inductance (H).
    uint16_t pole_pairs;         ///< Number of motor pole pairs.
    parameter_gt u_base;         ///< Base voltage for per-unit conversion (V).
    parameter_gt i_base;         ///< Base current for per-unit conversion (A).
    parameter_gt speed_base_rpm; ///< Base speed for per-unit conversion (RPM).
    parameter_gt smo_fc_e;       ///< Cutoff frequency for back-EMF filter (Hz).
    parameter_gt smo_fc_omega;   ///< Cutoff frequency for speed filter (Hz).
    ctrl_gt smo_kp;              ///< Proportional gain for the SMO's PLL.
    ctrl_gt smo_Ti;              ///< Integral time for the SMO's PLL.
    ctrl_gt smo_Td;              ///< Derivative time for the SMO's PLL.
    ctrl_gt smo_k_slide;         ///< Sliding gain for the SMO.

    //--- Ramp Generator ---
    parameter_gt ramp_target_freq;       ///< Target frequency for open-loop startup (Hz).
    parameter_gt ramp_target_freq_slope; ///< Ramp-up rate for open-loop startup (Hz/s).

} pmsm_smo_controller_init_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the PMSM SMO Controller.
 * @param[out] ctrl Pointer to the controller structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_pmsm_smo_bare_controller(pmsm_smo_controller_t* ctrl, pmsm_smo_controller_init_t* init);

/**
 * @brief Attaches the controller to a three-phase PWM output interface.
 * @param[out] ctrl    Pointer to the controller structure.
 * @param[in]  pwm_out Pointer to the PWM interface to attach.
 */
void ctl_attach_pmsm_smo_bare_output(pmsm_smo_controller_t* ctrl, tri_pwm_ift* pwm_out);

/**
 * @brief Resets the controller and SMO to their initial states for startup.
 * @details This function clears all PID integrators, resets the ramp generator,
 * and crucially, re-attaches the motor interface to the ramp generator's
 * open-loop angle for the next startup sequence.
 * @param[out] ctrl Pointer to the controller structure to clear.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_smo_ctrl(pmsm_smo_controller_t* ctrl)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_discrete_track_pid(&ctrl->spd_ctrl);
#else  // continuous controller
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_continuous_pid(&ctrl->spd_ctrl);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

    // Reset to open-loop V/f control for startup
    ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->ramp_gen.enc);
    ctl_clear_slope_f(&ctrl->ramp_gen);

    ctrl->flag_switch_cplt = 0;
}

/**
 * @brief Executes one step of the sensorless PMSM FOC control loop.
 * @details This function should be called periodically. It runs the open-loop ramp,
 * the FOC loops, and the SMO observer calculations in each step.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_smo_ctrl(pmsm_smo_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;

    if (!ctrl->flag_enable_controller)
    {
        // When disabled, ensure PWM is off and states are cleared.
        if (ctrl->pwm_out)
        {
            ctrl->pwm_out->value.dat[phase_A] = 0;
            ctrl->pwm_out->value.dat[phase_B] = 0;
            ctrl->pwm_out->value.dat[phase_C] = 0;
        }
        ctl_clear_pmsm_smo_ctrl(ctrl);
        return;
    }

    // --- State Estimation & Startup ---
    // Note: Ramp generator and SMO are run unconditionally.
    // The active angle source is managed by ctl_attach_mtr_position.
    ctl_step_slope_f(&ctrl->ramp_gen); // Open-loop ramp for startup
    ctl_step_pmsm_smo(&ctrl->smo, ctrl->vab0_set.dat[phase_alpha], ctrl->vab0_set.dat[phase_beta],
                      ctrl->iab0.dat[phase_alpha], ctrl->iab0.dat[phase_beta]); // SMO for sensorless estimation

    // --- FOC Transformations ---
    // Clark Transformation
#if MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 3
    ctl_ct_clarke(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#elif MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 2
    ctl_ct_clarke_2ph(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#else
#error("Wrong parameter for macro MTR_CTRL_CURRENT_MEASUREMENT_PHASES")
#endif

#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 3
    ctl_ct_clarke(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 2
    ctl_ct_clarke_2ph(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 0
    // If no voltage measurement, use the setpoint voltage for the observer
    ctrl->uab0.dat[phase_alpha] = ctrl->vab0_set.dat[phase_alpha];
    ctrl->uab0.dat[phase_beta] = ctrl->vab0_set.dat[phase_beta];
    ctrl->uab0.dat[phase_0] = ctrl->vab0_set.dat[phase_0];
#else
#error("Wrong parameter for macro MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES")
#endif

    // Park Transformation
    etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
    ctl_set_phasor_via_angle(etheta, &phasor);
    ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);
    ctl_ct_park(&ctrl->uab0, &phasor, &ctrl->udq0);

    // --- Control Loops ---
    // Velocity Controller
    if (ctrl->flag_enable_velocity_ctrl)
    {
        ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
        ctrl->idq_set.dat[phase_q] =
            ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
            ctrl->idq_ff.dat[phase_q];
#else // using continuous controller
        ctrl->idq_set.dat[phase_q] = ctl_step_tracking_continuous_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                      ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                     ctrl->idq_ff.dat[phase_q];
#endif
    }
    else
    {
        ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
        ctrl->idq_set.dat[phase_q] = ctrl->idq_ff.dat[phase_q];
    }

    // Current Controller
    if (ctrl->flag_enable_current_ctrl)
    {
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
        ctrl->vdq_set.dat[phase_d] =
            ctl_step_discrete_pid(&ctrl->current_ctrl[phase_d], ctrl->idq_set.dat[phase_d] - ctrl->idq0.dat[phase_d]) +
            ctrl->vdq_ff.dat[phase_d];
        ctrl->vdq_set.dat[phase_q] =
            ctl_step_discrete_pid(&ctrl->current_ctrl[phase_q], ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
            ctrl->vdq_ff.dat[phase_q];
#else //  using continuous controller
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
        ctrl->vdq_set.dat[phase_d] = ctrl->vdq_ff.dat[phase_d];
        ctrl->vdq_set.dat[phase_q] = ctrl->vdq_ff.dat[phase_q];
        ctrl->vdq_set.dat[phase_0] = 0;
    }

    // --- Output Stage ---
    // Inverse Park Transformation
    if (ctrl->flag_enable_modulation)
    {
        ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
    }

    // SVPWM
    if (ctrl->flag_enable_output && ctrl->pwm_out)
    {
        ctl_ct_svpwm_calc(&ctrl->vab0_set, &ctrl->pwm_out->value);
    }
    else if (ctrl->pwm_out)
    {
        ctrl->pwm_out->value.dat[phase_A] = 0;
        ctrl->pwm_out->value.dat[phase_B] = 0;
        ctrl->pwm_out->value.dat[phase_C] = 0;
    }
}

//--------------------------------------------------------------------------
// Enable / Disable and Mode Setting Functions
//--------------------------------------------------------------------------

/** @brief Enables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_enable_pmsm_smo_ctrl(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}

/** @brief Disables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_disable_pmsm_smo_ctrl(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}

/** @brief Enables the SMO observer calculations. */
GMP_STATIC_INLINE void ctl_enable_pmsm_smo(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_smo = 1;
}

/** @brief Sets the controller to V¦Á¦Â (voltage) mode. */
GMP_STATIC_INLINE void ctl_pmsm_smo_ctrl_valphabeta_mode(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the controller to Vdq (voltage) mode. */
GMP_STATIC_INLINE void ctl_pmsm_smo_ctrl_voltage_mode(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the d-q voltage feedforward (or reference) values. */
GMP_STATIC_INLINE void ctl_set_pmsm_smo_ctrl_vdq_ff(pmsm_smo_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

/** @brief Sets the controller to Idq (current) mode. */
GMP_STATIC_INLINE void ctl_pmsm_smo_ctrl_current_mode(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
}

/** @brief Sets the d-q current feedforward (or reference) values. */
GMP_STATIC_INLINE void ctl_set_pmsm_smo_ctrl_idq_ff(pmsm_smo_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

/** @brief Sets the controller to velocity mode. */
GMP_STATIC_INLINE void ctl_pmsm_smo_ctrl_velocity_mode(pmsm_smo_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
}

/** @brief Sets the target speed for the velocity controller. */
GMP_STATIC_INLINE void ctl_set_pmsm_smo_ctrl_speed(pmsm_smo_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

//--------------------------------------------------------------------------
// SMO Related Functions
//--------------------------------------------------------------------------

/**
 * @brief Switches the controller from open-loop startup to closed-loop SMO control.
 * @details This function performs the critical transition to sensorless operation.
 * It should be called once the motor has reached a sufficient speed for the
 * SMO to provide a reliable angle estimate. It performs a "bumpless transfer"
 * by initializing the PID controllers' states to match the system's state at
 * the moment of switching, preventing torque jolts.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_switch_pmsm_smo_ctrl_using_smo(pmsm_smo_controller_t* ctrl)
{
    if ((ctrl->flag_enable_smo) && (!ctrl->flag_switch_cplt))
    {
        ctl_vector2_t phasor;
        vector3_gt udq0_at_switch;
        vector3_gt idq0_at_switch;

        // 1. Switch angle source from ramp generator to SMO observer
        ctl_attach_mtr_position(&ctrl->mtr_interface, &ctrl->smo.encif);

        // 2. Set the controller to closed-loop velocity mode
        ctl_pmsm_smo_ctrl_velocity_mode(ctrl);
        ctl_set_pmsm_smo_ctrl_speed(ctrl, ctl_mul(ctrl->ramp_gen.target_frequency, ctrl->ramp_freq_spd_set_sf));

        // 3. Calculate current system state for bumpless transfer
        ctl_set_phasor_via_angle(ctrl->smo.encif.elec_position, &phasor);
        ctl_ct_park(&ctrl->vab0_set, &phasor, &udq0_at_switch);
        ctl_ct_park(&ctrl->iab0, &phasor, &idq0_at_switch);

        // 4. Clear any feedforward from the startup phase
        ctrl->idq_ff.dat[phase_q] = 0;
        ctrl->idq_ff.dat[phase_d] = 0;

        // 5. Initialize PID controller internal states for bumpless transfer
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
        // Pre-load the output of the PID controllers from the last cycle
        ctrl->spd_ctrl.pid.output_1 = idq0_at_switch.dat[phase_q];
        ctrl->current_ctrl[phase_d].output_1 = udq0_at_switch.dat[phase_d];
        ctrl->current_ctrl[phase_q].output_1 = udq0_at_switch.dat[phase_q];
#else // using continuous controller                                                                                  \
       // Pre-load the integrator and output of the PID controllers
        ctl_set_pid_integrator(&ctrl->spd_ctrl.pid, idq0_at_switch.dat[phase_q]);
        ctl_set_pid_integrator(&ctrl->current_ctrl[phase_d], udq0_at_switch.dat[phase_d]);
        ctl_set_pid_integrator(&ctrl->current_ctrl[phase_q], udq0_at_switch.dat[phase_q]);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

        ctrl->flag_switch_cplt = 1;
    }
}

/** @} */ // end of PMSM_SMO_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_SMO_H_
