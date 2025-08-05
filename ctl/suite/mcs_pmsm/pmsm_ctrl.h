/**
 * @file pmsm_ctrl.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a cross-platform core implementation of Field-Oriented Control (FOC) for Permanent Magnet Synchronous
 * Motors (PMSM).
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details
 * # PMSM Bare-metal Controller Usage Guide
 *
 * ## 1. Attach Physical Interfaces
 * - Use @ref ctl_attach_pmsm_bare_output to attach the PWM output interface.
 * - Use functions within @ref mtr_interface to attach input interfaces like ADC, encoders, etc.
 *
 * ## 2. Initialize the Controller
 * - Fill the @ref pmsm_bare_controller_init_t struct to specify motor and controller parameters.
 * - Call @ref ctl_init_pmsm_bare_controller to initialize the controller entity.
 *
 * ## 3. Select Operating Mode and Provide a Target
 * This controller supports various operating modes. Enter a mode by calling its corresponding switching function and
 * provide a target value using a set function.
 * - **V¦Á¦Â Mode**: @ref ctl_pmsm_ctrl_valphabeta_mode, @ref ctl_set_pmsm_ctrl_valphabeta
 * - **Voltage (Vdq) Mode**: @ref ctl_pmsm_ctrl_voltage_mode, @ref ctl_set_pmsm_ctrl_vdq_ff
 * - **Current (Idq) Mode**: @ref ctl_pmsm_ctrl_current_mode, @ref ctl_set_pmsm_ctrl_idq_ff
 * - **Velocity Mode**: @ref ctl_pmsm_ctrl_velocity_mode, @ref ctl_set_pmsm_ctrl_speed
 * - **Position Mode**: @ref ctl_pmsm_ctrl_position_mode, @ref set_pmsm_ctrl_position
 *
 * @note The mode switching functions only change internal flags. For smooth transitions during runtime, an additional
 * transition algorithm should be implemented.
 *
 * ## 4. Invoke in the Main Interrupt Service Routine (ISR)
 * - First, call the motor interface's step function to update inputs (e.g., ADC sampling).
 * - Then, call @ref ctl_step_pmsm_ctrl to execute the core FOC calculations.
 * - Finally, call the PWM interface's function to output the modulation result.
 *
 * ## 5. Enable/Disable the Controller
 * - Use @ref ctl_enable_pmsm_ctrl to enable the controller.
 * - Use @ref ctl_disable_pmsm_ctrl to disable the controller.
 * - It is recommended to call @ref ctl_clear_pmsm_ctrl to clear internal states before enabling.
 */

#ifndef _FILE_PMSM_CTRL_BARE_H_
#define _FILE_PMSM_CTRL_BARE_H_

// Necessary support
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>
#else
#include <ctl/component/intrinsic/continuous/track_pid.h>
#endif // PMSM_CTRL_USING_DISCRETE_CTRL

#include <ctl/component/motor_control/basic/decouple.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup pmsm_bare_controller_api PMSM Bare-metal Controller API
 * @brief Contains all configurations, data structures, and functions for the PMSM FOC controller.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Configuration Macros                             */
/*---------------------------------------------------------------------------*/

/**
 * @brief Selects the number of current measurement phases (2 or 3).
 * @details Defaults to 3 phases. Users can define this macro as 2 in a config file to select 2-phase measurement.
 */
#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_CURRENT_MEASUREMENT_PHASES

/**
 * @brief Selects the number of voltage measurement phases or disables it.
 * @details
 * - 0: No voltage sensors.
 * - 2: 2-phase voltage sensors.
 * - 3: 3-phase voltage sensors (Default).
 */
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

/**
 * @brief Selects the feedforward strategy.
 * @details
 * - 0: User mode, feedforward terms are specified manually by the user.
 * - 1: Decoupling mode, feedforward terms are calculated by the controller based on a decoupling algorithm (Default).
 * @note Decoupling mode is only effective when the velocity loop or an outer loop is active; otherwise, it will be
 * bypassed.
 */
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (1)
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY

/*---------------------------------------------------------------------------*/
/* Data Structures                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Core data structure for the PMSM bare-metal controller.
 */
typedef struct _tag_pmsm_bare_controller
{
    /*-- Interfaces --*/
    mtr_ift mtr_interface; /**< Universal motor input interface (ADC, encoder, etc.). */
    tri_pwm_ift* pwm_out;  /**< Three-phase PWM output interface. */

    /*-- Controller Entities --*/
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2]; /**< d/q-axis discrete PID current controllers. */
    track_discrete_pid_t spd_ctrl;  /**< Discrete PID velocity controller. */
#else                               // use continuous controller
    pid_regular_t current_ctrl[2]; /**< d/q-axis continuous PID current controllers. */
    track_pid_t spd_ctrl;          /**< Continuous PID velocity controller. */
#endif

    /*-- Controller Intermediate Variables --*/
    vector3_gt iab0; /**< Current in the 3-phase stationary frame (i_alpha, i_beta, i_0). */
    vector3_gt idq0; /**< Current in the 2-phase rotating frame (i_d, i_q, i_0). */
    vector3_gt uab0; /**< Voltage in the 3-phase stationary frame (v_alpha, v_beta, v_0). */
    vector3_gt udq0; /**< Voltage in the 2-phase rotating frame (v_d, v_q, v_0). */

    /*-- Controller Feed-forward Parameters --*/
    vector2_gt idq_ff; /**< d/q-axis current feedforward values. */
    vector2_gt vdq_ff; /**< d/q-axis voltage feedforward values. */

    /*-- Controller Set-point Parameters --*/
    int32_t revolution_set; /**< Target position revolution count. */
    ctrl_gt pos_set;        /**< Target position fractional part within a revolution [0, 1). */
    ctrl_gt speed_set;      /**< Target velocity (per-unit). */
    vector2_gt idq_set;     /**< d/q-axis current target values. */
    vector3_gt vdq_set;     /**< d/q-axis voltage target values. */
    vector3_gt vab0_set;    /**< ¦Á/¦Â-axis voltage target values. */

    /*-- Controller State & Flags --*/
    fast16_gt isr_tick;                /**< Interrupt counter for frequency division. */
    fast_gt flag_enable_controller;    /**< Master enable flag for the controller. */
    fast_gt flag_enable_output;        /**< PWM output enable flag. */
    fast_gt flag_enable_modulation;    /**< Modulation enable flag (IPark and SVPWM). */
    fast_gt flag_enable_current_ctrl;  /**< Current loop enable flag. */
    fast_gt flag_enable_velocity_ctrl; /**< Velocity loop enable flag. */
    fast_gt flag_enable_position_ctrl; /**< Position loop enable flag. */

} pmsm_bare_controller_t;

/**
 * @brief Initialization parameters structure for the PMSM bare-metal controller.
 */
typedef struct _tag_pmsm_bare_controller_init
{
    /*-- Common Parameters --*/
    parameter_gt fs; /**< Controller operating frequency (Hz). */

    /*-- Current Controller Parameters --*/
    parameter_gt current_pid_gain;  /**< Current loop proportional gain (Kp). */
    parameter_gt current_Ti;        /**< Current loop integral time constant (s). */
    parameter_gt current_Td;        /**< Current loop derivative time constant (s). */
    parameter_gt voltage_limit_max; /**< Current loop output saturation upper limit (Vdq voltage in p.u.). */
    parameter_gt voltage_limit_min; /**< Current loop output saturation lower limit (Vdq voltage in p.u.). */

    /*-- Speed Controller Parameters --*/
    parameter_gt spd_pid_gain;      /**< Velocity loop proportional gain (Kp). */
    parameter_gt spd_Ti;            /**< Velocity loop integral time constant (s). */
    parameter_gt spd_Td;            /**< Velocity loop derivative time constant (s). */
    parameter_gt current_limit_max; /**< Velocity loop output saturation upper limit (Iq current in p.u.). */
    parameter_gt current_limit_min; /**< Velocity loop output saturation lower limit (Iq current in p.u.). */
    parameter_gt acc_limit_max;     /**< Maximum acceleration limit (p.u./s). */
    parameter_gt acc_limit_min;     /**< Minimum acceleration (max deceleration) limit (p.u./s). */
    uint32_t spd_ctrl_div;          /**< Velocity loop execution frequency divider (relative to the main ISR). */

    /*-- Position Controller Parameters (if any) --*/
    // ... (Position loop parameters can be added here)

} pmsm_bare_controller_init_t;

/*---------------------------------------------------------------------------*/
/* Functions                                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief Clears the internal states and integral terms of all PID controllers.
 * @details This function is crucial for ensuring a safe start-up. It resets the integrators
 * of the current and speed controllers to prevent integrator windup from a previous run,
 * which could cause a sudden and uncontrolled motion when the controller is re-enabled.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @note It is highly recommended to call this function before enabling the controller via @ref ctl_enable_pmsm_ctrl.
 */
GMP_STATIC_INLINE
void ctl_clear_pmsm_ctrl(pmsm_bare_controller_t* ctrl)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_discrete_track_pid(&ctrl->spd_ctrl);
#else  // continuous controller
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_track_pid(&ctrl->spd_ctrl);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL
}

/**
 * @brief Executes one step of the PMSM controller calculation.
 * @details This function implements the complete Field-Oriented Control (FOC) pipeline. It should be called
 * periodically in the main control Interrupt Service Routine (ISR). The execution flow is as follows:
 * 1.  **Clarke Transform**: Converts 3-phase currents (and voltages) from ABC to the ¦Á¦Â stationary frame.
 * 2.  **Park Transform**: Converts ¦Á¦Â values to the dq rotating frame using the current rotor angle.
 * 3.  **Cascaded Control Loops**: Executes the position (if enabled), velocity (if enabled), and current control loops.
 * 4.  **Inverse Park Transform**: Converts the calculated dq voltages back to the ¦Á¦Â frame.
 * 5.  **SVPWM**: Generates PWM duty cycles from the ¦Á¦Â voltage vector.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE
void ctl_step_pmsm_ctrl(pmsm_bare_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;
    ctrl_gt vq_limit = float2ctrl(1.0);

    ctrl->isr_tick += 1;

    if (ctrl->flag_enable_controller)
    {
        // Clark Transformation: i_abc -> i_ab
#if MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 3
        ctl_ct_clark(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#elif MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 2
        ctl_ct_clark_2ph(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#else
#error("Wrong parameter for macro MTR_CTRL_CURRENT_MEASUREMENT_PHASES")
#endif

        // Clark Transformation: u_abc -> u_ab
#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 3
        ctl_ct_clark(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 2
        ctl_ct_clark_2ph(&ctrl->mtr_interface.uabc->value, &ctrl->uab0);
#elif MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES == 0
        ctl_vector3_clear(&ctrl->uab0);
#else
#error("Wrong parameter for macro MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES")
#endif

        // Park Transformation
        etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
        ctl_set_phasor_via_angle(etheta, &phasor);
        ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0); // i_ab -> i_dq
#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES != 0
        ctl_ct_park(&ctrl->uab0, &phasor, &ctrl->udq0); // u_ab -> u_dq
#else
        ctl_vector3_clear(&ctrl->udq0);
#endif

        // Position controller
        if (ctrl->flag_enable_position_ctrl)
        {
            // (To be implemented)
        }

        // Velocity Controller
        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] = ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                     ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#else
            ctrl->idq_set.dat[phase_q] =
                ctl_step_track_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                ctrl->idq_ff.dat[phase_q];
#endif

#if MTR_CTRL_FEEDFORWARD_STRATEGY == 1
            // Decoupling logic can be added here
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
            vq_limit = ctl_sqrt(float2ctrl(1.0) - ctl_mul(ctrl->vdq_set.dat[phase_d], ctrl->vdq_set.dat[phase_d]));
            ctl_set_pid_limit(&ctrl->current_ctrl[phase_q], vq_limit, -vq_limit);
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

        // Inverse Park transformation
        if (ctrl->flag_enable_modulation)
        {
            ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
        }

        // SVPWM modulation
        if (ctrl->flag_enable_output)
        {
            ctl_ct_svpwm_calc(&ctrl->vab0_set, &ctrl->pwm_out->value);
        }
        else
        {
            ctrl->pwm_out->value.dat[phase_A] = float2ctrl(0.5);
            ctrl->pwm_out->value.dat[phase_B] = float2ctrl(0.5);
            ctrl->pwm_out->value.dat[phase_C] = float2ctrl(0.5);
        }
    }
    else
    {
        // If controller is disabled, stop PWM and clear states
        ctrl->pwm_out->value.dat[phase_A] = 0;
        ctrl->pwm_out->value.dat[phase_B] = 0;
        ctrl->pwm_out->value.dat[phase_C] = 0;
        ctl_clear_pmsm_ctrl(ctrl);
    }
}

/*-- Enable / Disable Functions --*/

/**
 * @brief Enables the PMSM controller master switch.
 * @details This function allows the main control loop in @ref ctl_step_pmsm_ctrl to execute.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_enable_pmsm_ctrl(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}

/**
 * @brief Disables the PMSM controller master switch.
 * @details When disabled, the main control loop is bypassed, PWM outputs are set to zero, and internal states are
 * cleared.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_disable_pmsm_ctrl(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}

/**
 * @brief Enables the final PWM output stage.
 * @details This allows the calculated duty cycles to be sent to the motor. The control logic continues to run.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_enable_pmsm_ctrl_output(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
}

/**
 * @brief Disables the final PWM output stage.
 * @details This gates the PWM output, effectively letting the motor coast. The control logic continues to run in the
 * background.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_disable_pmsm_ctrl_output(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 0;
}

/*-- Mode Switching and Set-point Functions --*/

/**
 * @brief Switches the controller to V-alpha-beta mode.
 * @details In this mode, the controller bypasses all internal loops and directly applies the user-specified
 * V¦Á and V¦Â voltages to the SVPWM module.
 * - Modulation: Disabled (IPark is bypassed)
 * - Current/Velocity/Position Loops: Disabled
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_valphabeta_mode(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the target voltage vector (V¦Á, V¦Â) in V-alpha-beta mode.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @param[in] valpha The target alpha-component of the voltage vector (per-unit).
 * @param[in] vbeta The target beta-component of the voltage vector (per-unit).
 * @note This function is only effective when the controller is in V-alpha-beta mode.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_valphabeta(pmsm_bare_controller_t* ctrl, ctrl_gt valpha, ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_A] = valpha;
    ctrl->vab0_set.dat[phase_B] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

/**
 * @brief Switches the controller to Voltage (Vdq) mode.
 * @details In this mode, the controller applies the user-specified Vd and Vq voltages after the Inverse Park transform.
 * - Modulation: Enabled
 * - Current/Velocity/Position Loops: Disabled
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_voltage_mode(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the Vd, Vq feedforward/reference values in Voltage (Vdq) mode.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @param[in] vd The target d-axis voltage (per-unit).
 * @param[in] vq The target q-axis voltage (per-unit).
 * @note This function is only effective when the controller is in Voltage (Vdq) mode.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_vdq_ff(pmsm_bare_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

/**
 * @brief Switches the controller to Current (Idq) mode.
 * @details In this mode, the current control loop is active and regulates the motor current to match the user-specified
 * Id and Iq references.
 * - Modulation: Enabled
 * - Current Loop: Enabled
 * - Velocity/Position Loops: Disabled
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_current_mode(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the Id, Iq feedforward/reference values in Current (Idq) mode.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @param[in] id The target d-axis current (per-unit).
 * @param[in] iq The target q-axis current (per-unit).
 * @note This function is only effective when the controller is in Current (Idq) mode.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_idq_ff(pmsm_bare_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

/**
 * @brief Switches the controller to Velocity mode.
 * @details In this mode, the velocity and current control loops are active. The velocity loop generates an Iq reference
 * for the current loop.
 * - Modulation: Enabled
 * - Current Loop: Enabled
 * - Velocity Loop: Enabled
 * - Position Loop: Disabled
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_velocity_mode(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the target velocity in Velocity mode.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @param[in] spd The target velocity (per-unit).
 * @note This function is only effective when the controller is in Velocity mode.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_speed(pmsm_bare_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

/**
 * @brief Switches the controller to Position mode.
 * @details In this mode, all control loops (position, velocity, and current) are active in a cascaded structure.
 * - Modulation: Enabled
 * - Current Loop: Enabled
 * - Velocity Loop: Enabled
 * - Position Loop: Enabled
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_position_mode(pmsm_bare_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;
}

/**
 * @brief Sets the target position in Position mode.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 * @param[in] revolution The integer part of the target position (number of full turns).
 * @param[in] pos The fractional part of the target position [0, 1).
 * @note This function is only effective when the controller is in Position mode.
 */
GMP_STATIC_INLINE void set_pmsm_ctrl_position(pmsm_bare_controller_t* ctrl, int32_t revolution, ctrl_gt pos)
{
    ctrl->revolution_set = revolution;
    ctrl->pos_set = pos;
}

/*-- Initialization Functions --*/

/**
 * @brief Configures and initializes a PMSM controller instance based on provided parameters.
 * @details This function sets up the PID controllers and other internal parameters of the controller
 * instance according to the values specified in the `init` structure. It must be called before
 * the controller is used.
 * @param[out] ctrl Pointer to the PMSM controller instance to be initialized.
 * @param[in] init Pointer to the initialization structure containing all configuration parameters.
 */
void ctl_init_pmsm_bare_controller(pmsm_bare_controller_t* ctrl, pmsm_bare_controller_init_t* init);

/**
 * @brief Attaches the PMSM controller to a three-phase PWM output interface.
 * @details This function links the controller's output to a hardware-specific PWM interface,
 * establishing the path for the final control signal.
 * @param[out] ctrl Pointer to the PMSM controller instance.
 * @param[in] pwm_out Pointer to the three-phase PWM interface instance.
 */
void ctl_attach_pmsm_bare_output(pmsm_bare_controller_t* ctrl, tri_pwm_ift* pwm_out);

/** @} */ // end of pmsm_bare_controller_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_BARE_H_
