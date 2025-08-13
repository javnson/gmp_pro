/**
 * @file pmsm_ctrl.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Provides a cross-platform core implementation of Field-Oriented Control (FOC) for PMSM.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
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
 * @defgroup MC_PMSM_SENSORED_CONTROLLER Sensored PMSM FOC Controller
 * @brief A complete Field-Oriented Controller for Permanent Magnet Synchronous Motors.
 *
 * @details
 * ### PMSM Bare-metal Controller Usage Guide:
 *
 * 1.  **Attach Physical Interfaces:**
 * - Use @ref ctl_attach_pmsm_bare_output to attach the PWM output interface.
 * - Use functions within the `mtr_interface` to attach input interfaces like ADC, encoders, etc.
 *
 * 2.  **Initialize the Controller:**
 * - Fill the @ref pmsm_bare_controller_init_t struct to specify motor and controller parameters.
 * - Call @ref ctl_init_pmsm_bare_controller to initialize the controller entity.
 *
 * 3.  **Select Operating Mode and Provide a Target:**
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
 * 4.  **Invoke in the Main Interrupt Service Routine (ISR):**
 * - First, call the motor interface's step function to update inputs (e.g., ADC sampling).
 * - Then, call @ref ctl_step_pmsm_ctrl to execute the core FOC calculations.
 * - Finally, call the PWM interface's function to output the modulation result.
 *
 * 5.  **Enable/Disable the Controller:**
 * - Use @ref ctl_enable_pmsm_ctrl to enable the controller.
 * - Use @ref ctl_disable_pmsm_ctrl to disable the controller.
 * - It is recommended to call @ref ctl_clear_pmsm_ctrl to clear internal states before enabling.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* Configuration Macros                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_CONFIG Configuration Macros
 * @ingroup MC_PMSM_SENSORED_CONTROLLER
 * @brief Compile-time configurations for the PMSM controller.
 * @{
 */

#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES                                                                            \
    ((3)) /**< @brief Selects the number of current measurement phases (2 or 3). */
#endif

#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES                                                                            \
    ((3)) /**< @brief Selects the number of voltage measurement phases (0, 2, or 3). */
#endif

#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY                                                                                  \
    (1) /**< @brief Selects the feedforward strategy (0 for manual, 1 for decoupling). */
#endif

/** @} */ // end of MC_PMSM_CONFIG group

/*---------------------------------------------------------------------------*/
/* Data Structures                                                           */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_STRUCTS Data Structures
 * @ingroup MC_PMSM_SENSORED_CONTROLLER
 * @brief Data structures for the PMSM controller and its initialization.
 * @{
 */

/**
 * @brief Core data structure for the PMSM bare-metal controller.
 */
typedef struct _tag_pmsm_controller
{
    // --- Interfaces ---
    mtr_ift mtr_interface; /**< @brief Universal motor input interface (ADC, encoder, etc.). */
    tri_pwm_ift* pwm_out;  /**< @brief Three-phase PWM output interface. */

    // --- Controller Components ---
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2];       /**< @brief d/q-axis discrete PID current controllers. */
    ctl_tracking_discrete_pid_t spd_ctrl; /**< @brief Discrete PID velocity controller. */
#else
    ctl_pid_t current_ctrl[2];              /**< @brief d/q-axis continuous PID current controllers. */
    ctl_tracking_continuous_pid_t spd_ctrl; /**< @brief Continuous PID velocity controller. */
#endif

    // --- Intermediate Variables ---
    vector3_gt iab0; /**< @brief Current in the 3-phase stationary frame (i_alpha, i_beta, i_0). */
    vector3_gt idq0; /**< @brief Current in the 2-phase rotating frame (i_d, i_q, i_0). */
    vector3_gt uab0; /**< @brief Voltage in the 3-phase stationary frame (v_alpha, v_beta, v_0). */
    vector3_gt udq0; /**< @brief Voltage in the 2-phase rotating frame (v_d, v_q, v_0). */

    // --- Feed-forward & Setpoints ---
    vector2_gt idq_ff;      /**< @brief d/q-axis current feedforward values. */
    vector2_gt vdq_ff;      /**< @brief d/q-axis voltage feedforward values. */
    int32_t revolution_set; /**< @brief Target position revolution count. */
    ctrl_gt pos_set;        /**< @brief Target position fractional part within a revolution [0, 1). */
    ctrl_gt speed_set;      /**< @brief Target velocity (per-unit). */
    vector2_gt idq_set;     /**< @brief d/q-axis current target values. */
    vector3_gt vdq_set;     /**< @brief d/q-axis voltage target values. */
    vector3_gt vab0_set;    /**< @brief @f( \alpha-\beta@f) -axis voltage target values. */

    // --- State & Flags ---
    fast16_gt isr_tick;                /**< @brief Interrupt counter for frequency division. */
    fast_gt flag_enable_controller;    /**< @brief Master enable flag for the controller. */
    fast_gt flag_enable_output;        /**< @brief PWM output enable flag. */
    fast_gt flag_enable_modulation;    /**< @brief Modulation enable flag (IPark and SVPWM). */
    fast_gt flag_enable_current_ctrl;  /**< @brief Current loop enable flag. */
    fast_gt flag_enable_velocity_ctrl; /**< @brief Velocity loop enable flag. */
    fast_gt flag_enable_position_ctrl; /**< @brief Position loop enable flag. */

} pmsm_controller_t;

/**
 * @brief Initialization parameters structure for the PMSM bare-metal controller.
 */
typedef struct _tag_pmsm_controller_init
{
    parameter_gt fs; /**< @brief Controller operating frequency (Hz). */

    // --- Current Controller Parameters ---
    parameter_gt current_pid_gain;  /**< @brief Current loop proportional gain (Kp). */
    parameter_gt current_Ti;        /**< @brief Current loop integral time constant (s). */
    parameter_gt current_Td;        /**< @brief Current loop derivative time constant (s). */
    parameter_gt voltage_limit_max; /**< @brief Current loop output saturation upper limit (Vdq voltage in p.u.). */
    parameter_gt voltage_limit_min; /**< @brief Current loop output saturation lower limit (Vdq voltage in p.u.). */

    // --- Speed Controller Parameters ---
    parameter_gt spd_pid_gain;      /**< @brief Velocity loop proportional gain (Kp). */
    parameter_gt spd_Ti;            /**< @brief Velocity loop integral time constant (s). */
    parameter_gt spd_Td;            /**< @brief Velocity loop derivative time constant (s). */
    parameter_gt current_limit_max; /**< @brief Velocity loop output saturation upper limit (Iq current in p.u.). */
    parameter_gt current_limit_min; /**< @brief Velocity loop output saturation lower limit (Iq current in p.u.). */
    parameter_gt acc_limit_max;     /**< @brief Maximum acceleration limit (p.u./s). */
    parameter_gt acc_limit_min;     /**< @brief Minimum acceleration (max deceleration) limit (p.u./s). */
    uint32_t spd_ctrl_div;          /**< @brief Velocity loop execution frequency divider (relative to the main ISR). */

} pmsm_controller_init_t;

/** @} */ // end of MC_PMSM_STRUCTS group

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_PMSM_FUNCTIONS Functions
 * @ingroup MC_PMSM_SENSORED_CONTROLLER
 * @brief API functions for controlling and configuring the PMSM controller.
 * @{
 */

// --- Initialization and Attachment ---
void ctl_init_pmsm_controller(pmsm_controller_t* ctrl, pmsm_controller_init_t* init);
void ctl_attach_pmsm_output(pmsm_controller_t* ctrl, tri_pwm_ift* pwm_out);

// --- Core Controller Logic ---

/**
 * @brief Clears the internal states and integral terms of all PID controllers.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_ctrl(pmsm_controller_t* ctrl)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_pid(&ctrl->spd_ctrl);
#else
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_continuous_pid(&ctrl->spd_ctrl);
#endif
}

/**
 * @brief Executes one step of the PMSM controller calculation.
 * @param[in,out] ctrl Pointer to the PMSM controller instance.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_ctrl(pmsm_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;
    ctrl_gt vq_limit = float2ctrl(1.0);

    ctrl->isr_tick++;

    if (ctrl->flag_enable_controller)
    {
        // Clarke Transformation
#if MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 3
        ctl_ct_clarke(&ctrl->mtr_interface.iabc->value, &ctrl->iab0);
#elif MTR_CTRL_CURRENT_MEASUREMENT_PHASES == 2
        ctl_ct_clarke_2ph((ctl_vector2_t*)&ctrl->mtr_interface.iabc->value, (ctl_vector2_t*)&ctrl->iab0);
#endif

        // Park Transformation
        etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
        ctl_set_phasor_via_angle(etheta, &phasor);
        ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);

        // Position controller
        if (ctrl->flag_enable_position_ctrl)
        { /* To be implemented */
        }

        // Velocity Controller
        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] =
                ctl_step_tracking_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
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
            ctl_vector2_copy((ctl_vector2_t*)&ctrl->vdq_set, &ctrl->vdq_ff);
            ctrl->vdq_set.dat[phase_0] = 0;
        }

        // Inverse Park and SVPWM
        if (ctrl->flag_enable_modulation)
        {
            ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
        }
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
        ctl_clear_pmsm_ctrl(ctrl);
    }
}

// --- Enable / Disable Functions ---

/** @brief Enables the master switch for the controller. */
GMP_STATIC_INLINE void ctl_enable_pmsm_ctrl(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}
/** @brief Disables the master switch for the controller. */
GMP_STATIC_INLINE void ctl_disable_pmsm_ctrl(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}
/** @brief Enables the PWM output stage. */
GMP_STATIC_INLINE void ctl_enable_pmsm_ctrl_output(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
}
/** @brief Disables the PWM output stage. */
GMP_STATIC_INLINE void ctl_disable_pmsm_ctrl_output(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 0;
}

// --- Mode Control and Setpoint Functions ---

/** @brief Switches to V¦Á¦Â open-loop mode. */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_valphabeta_mode(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target V¦Á and V¦Â voltages. */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_valphabeta(pmsm_controller_t* ctrl, ctrl_gt valpha, ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_alpha] = valpha;
    ctrl->vab0_set.dat[phase_beta] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

/** @brief Switches to Vdq open-loop voltage mode. */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_voltage_mode(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target Vd and Vq voltages. */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_vdq_ff(pmsm_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

/** @brief Switches to Idq closed-loop current mode. */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_current_mode(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target Id and Iq currents. */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_idq_ff(pmsm_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

/** @brief Switches to closed-loop velocity mode. */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_velocity_mode(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target velocity. */
GMP_STATIC_INLINE void ctl_set_pmsm_ctrl_speed(pmsm_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

/** @brief Switches to closed-loop position mode. */
GMP_STATIC_INLINE void ctl_pmsm_ctrl_position_mode(pmsm_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;
}

/** @brief Sets the target position. */
GMP_STATIC_INLINE void set_pmsm_ctrl_position(pmsm_controller_t* ctrl, int32_t revolution, ctrl_gt pos)
{
    ctrl->revolution_set = revolution;
    ctrl->pos_set = pos;
}

/** @} */ // end of MC_PMSM_FUNCTIONS group
/** @} */ // end of MC_PMSM_SENSORED_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_BARE_H_
