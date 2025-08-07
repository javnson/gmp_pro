/**
 * @file pmsm_ctrl_hfi.h
 * @brief Provides a universal PMSM controller interface based on Field-Oriented Control (FOC).
 * @details This file defines the data structures, functions, and control logic for a
 * generic Permanent Magnet Synchronous Motor (PMSM) controller. It supports
 * various control modes, including voltage, current, velocity, and position control.
 * The implementation can be configured for discrete or continuous PID controllers
 * and supports different hardware setups for current and voltage measurement.
 *
 * Usage involves these steps:
 * 1.  Attach hardware interfaces (ADC, PWM, encoders) using the provided functions.
 * 2.  Initialize the controller structure with motor and control parameters via
 * `ctl_init_pmsm_hfi_hfi_controller`.
 * 3.  Select an operating mode (e.g., `ctl_pmsm_hfi_ctrl_velocity_mode`).
 * 4.  Provide a setpoint for the chosen mode (e.g., `ctl_set_pmsm_hfi_ctrl_speed`).
 * 5.  Call `ctl_step_pmsm_hfi_ctrl` periodically within a main control ISR.
 * 6.  Enable the controller and its output using the enable/disable functions.
 */

#ifndef _FILE_PMSM_CTRL_HFI_H_
#define _FILE_PMSM_CTRL_HFI_H_

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

/*---------------------------------------------------------------------------*/
/* PMSM High-Frequency Injection (HFI) Bare Controller               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_HFI_BARE_CONTROLLER PMSM HFI Bare Controller
 * @brief A module for controlling a PMSM using Field-Oriented Control.
 * @details This module contains the data structures and functions to implement a
 * full FOC controller with cascaded control loops for position, velocity,
 * and current.
 * @{
 */

//================================================================================
// Pragma Defines
//================================================================================

/**
 * @brief Configures the number of phase current sensors used for measurement.
 * @details
 * - **3**: Uses 3 phase current sensors (default).
 * - **2**: Uses 2 phase current sensors.
 */
#ifndef MTR_CTRL_CURRENT_MEASUREMENT_PHASES
#define MTR_CTRL_CURRENT_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_CURRENT_MEASUREMENT_PHASES

/**
 * @brief Configures the number of phase voltage sensors used for measurement.
 * @details
 * - **3**: Uses 3 phase voltage sensors (default).
 * - **2**: Uses 2 phase voltage sensors.
 * - **0**: Disables voltage measurement.
 */
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((3))
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

/**
 * @brief Selects the feedforward strategy for the controller.
 * @details
 * - **1**: Decoupled mode. Feedforward terms are calculated automatically. This is
 * only active when the velocity controller is enabled (default).
 * - **0**: User mode. Feedforward values must be specified manually by the user.
 */
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (1)
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Main structure for the PMSM Bare Controller.
 * @details This structure holds all the state variables, interfaces, controller
 * instances, and flags required for the FOC algorithm.
 */
typedef struct _tag_pmsm_hfi_hfi_controller
{
    //--------------------------------------------------------------------------
    // Interfaces
    //--------------------------------------------------------------------------
    mtr_ift mtr_interface; ///< Input interface for motor signals (currents, voltage, position).
    tri_pwm_ift* pwm_out;  ///< Output interface for three-phase PWM generation.

    //--------------------------------------------------------------------------
    // Controller Entities
    //--------------------------------------------------------------------------
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2];       ///< Discrete PID controllers for d-q axis currents.
    ctl_tracking_discrete_pid_t spd_ctrl; ///< Discrete tracking PID controller for speed.
#else                                     // use continuous controller
    ctl_pid_t current_ctrl[2];              ///< Continuous PID controllers for d-q axis currents.
    ctl_tracking_continuous_pid_t spd_ctrl; ///< Continuous tracking PID controller for speed.
#endif

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
    int32_t revolution_set; ///< Target position (number of full revolutions).
    ctrl_gt pos_set;        ///< Target position within a single revolution (0.0 to 1.0).
    ctrl_gt speed_set;      ///< Target speed (p.u.).
    vector2_gt idq_set;     ///< Target d-q axis currents.
    vector3_gt vdq_set;     ///< Target d-q-zero axis voltages.
    vector3_gt vab0_set;    ///< Target alpha-beta-zero axis voltages.

    //--------------------------------------------------------------------------
    // State Flags & Counters
    //--------------------------------------------------------------------------
    fast16_gt isr_tick;                ///< Counter incremented in each ISR call.
    fast_gt flag_enable_controller;    ///< Master flag to enable or disable the entire controller.
    fast_gt flag_enable_output;        ///< Flag to enable or disable the PWM output.
    fast_gt flag_enable_modulation;    ///< Flag to enable inverse Park transform and modulation.
    fast_gt flag_enable_current_ctrl;  ///< Flag to enable the current control loop.
    fast_gt flag_enable_velocity_ctrl; ///< Flag to enable the velocity control loop.
    fast_gt flag_enable_position_ctrl; ///< Flag to enable the position control loop.

} pmsm_hfi_controller_t;

/**
 * @brief Initialization parameters for the PMSM Bare Controller.
 */
typedef struct _tag_pmsm_hfi_controller_init
{
    parameter_gt fs; ///< Controller sampling frequency (Hz).

    //--- Current Controller ---
    parameter_gt current_pid_gain;  ///< Proportional gain (Kp) for the current controller.
    parameter_gt current_Ti;        ///< Integral time constant (Ti) for the current controller.
    parameter_gt current_Td;        ///< Derivative time constant (Td) for the current controller.
    parameter_gt voltage_limit_max; ///< Maximum output limit for the current controller (voltage).
    parameter_gt voltage_limit_min; ///< Minimum output limit for the current controller (voltage).

    //--- Speed Controller ---
    parameter_gt spd_pid_gain;      ///< Proportional gain (Kp) for the speed controller.
    parameter_gt spd_Ti;            ///< Integral time constant (Ti) for the speed controller.
    parameter_gt spd_Td;            ///< Derivative time constant (Td) for the speed controller.
    parameter_gt current_limit_max; ///< Maximum output limit for the speed controller (q-axis current).
    parameter_gt current_limit_min; ///< Minimum output limit for the speed controller (q-axis current).
    parameter_gt acc_limit_max;     ///< Maximum acceleration limit (p.u./s).
    parameter_gt acc_limit_min;     ///< Minimum deceleration limit (p.u./s).
    uint32_t spd_ctrl_div;          ///< Speed controller execution frequency divider.

} pmsm_hfi_controller_init_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the PMSM HFI Controller.
 * @param[out] ctrl Pointer to the controller structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_pmsm_hfi_controller(pmsm_hfi_controller_t* ctrl, pmsm_hfi_controller_init_t* init);

/**
 * @brief Attaches the controller to a three-phase PWM output interface.
 * @param[out] ctrl    Pointer to the controller structure.
 * @param[in]  pwm_out Pointer to the PWM interface to attach.
 */
void ctl_attach_pmsm_hfi_output(pmsm_hfi_controller_t* ctrl, tri_pwm_ift* pwm_out);

/**
 * @brief Resets all internal states and integrators of the PID controllers.
 * @details This function should be called before enabling the controller to prevent
 * unexpected behavior from stale data.
 * @param[out] ctrl Pointer to the controller structure to clear.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_hfi_ctrl(pmsm_hfi_controller_t* ctrl)
{
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_discrete_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_pid(&ctrl->spd_ctrl);
#else  // continuous controller
    ctl_clear_pid(&ctrl->current_ctrl[phase_d]);
    ctl_clear_pid(&ctrl->current_ctrl[phase_q]);
    ctl_clear_tracking_continuous_pid(&ctrl->spd_ctrl);
#endif // PMSM_CTRL_USING_DISCRETE_CTRL
}

/**
 * @brief Executes one step of the PMSM FOC HFI control loop.
 * @details This function should be called at a fixed frequency, typically within a
 * timer-based ISR. It performs Clarke and Park transforms, executes the
 * cascaded control loops (current, velocity, position), and calculates
 * the required PWM duty cycles via SVPWM.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_hfi_ctrl(pmsm_hfi_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;
    ctrl_gt vq_limit = float2ctrl(1.0);

    ctrl->isr_tick += 1;

    if (ctrl->flag_enable_controller)
    {
        // Clark Transformation: (abc -> alpha-beta)
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
        ctl_vector3_clear(&ctrl->uab0);
#else
#error("Wrong parameter for macro MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES")
#endif

        // Park Transformation: (alpha-beta -> d-q)
        etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
        ctl_set_phasor_via_angle(etheta, &phasor);
        ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);

#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES != 0
        ctl_ct_park(&ctrl->uab0, &phasor, &ctrl->udq0);
#else
        ctl_vector3_clear(&ctrl->udq0); // Should clear udq0, not uab0
#endif

        // Position Controller
        if (ctrl->flag_enable_position_ctrl)
        {
            // Position control logic is not yet implemented.
        }

        // Velocity Controller
        if (ctrl->flag_enable_velocity_ctrl)
        {
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] =
                ctl_step_tracking_pid(&ctrl->spd_ctrl, ctrl->speed_set, ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                ctrl->idq_ff.dat[phase_q];
#else // using continuous controller
            ctrl->idq_set.dat[phase_q] = ctl_step_tracking_continuous_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                          ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#endif

#if MTR_CTRL_FEEDFORWARD_STRATEGY == 1
            // Decoupling logic is commented out.
            // ctl_mtr_pmsm_decouple(&ctrl->vdq_set, &ctrl->idq_set, ctrl->Ld, ctrl->Lq,
            //                       ctl_get_mtr_velocity(ctrl->mtr_interface), ctrl->psi_e);
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

            // Note: Dynamic saturation for Vq is commented out in discrete implementation.
            //vq_limit = ctl_sqrt(float2ctrl(1.0) - ctl_mul(ctrl->vdq_set.dat[phase_d], ctrl->vdq_set.dat[phase_d]));
            //ctl_set_discrete_pid_limit(&ctrl->current_ctrl[phase_q], vq_limit, -vq_limit);

            ctrl->vdq_set.dat[phase_q] = ctl_step_discrete_pid(&ctrl->current_ctrl[phase_q],
                                                               ctrl->idq_set.dat[phase_q] - ctrl->idq0.dat[phase_q]) +
                                         ctrl->vdq_ff.dat[phase_q];
#else //  using continuous controller
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

        // Inverse Park Transformation & Modulation
        if (ctrl->flag_enable_modulation)
        {
            ctl_ct_ipark(&ctrl->vdq_set, &phasor, &ctrl->vab0_set);
        }
        // else: vab0_set will be used directly from user input.

        // SVPWM Output Stage
        if (ctrl->flag_enable_output)
        {
            ctl_ct_svpwm_calc(&ctrl->vab0_set, &ctrl->pwm_out->value);
        }
        else
        {
            // Set PWM to 50% duty cycle for a high-impedance state.
            ctrl->pwm_out->value.dat[phase_A] = float2ctrl(0.5);
            ctrl->pwm_out->value.dat[phase_B] = float2ctrl(0.5);
            ctrl->pwm_out->value.dat[phase_C] = float2ctrl(0.5);
        }
    }
    else // Controller is disabled
    {
        // Set PWM to 50% duty cycle for a high-impedance state.
        ctrl->pwm_out->value.dat[phase_A] = float2ctrl(0.5);
        ctrl->pwm_out->value.dat[phase_B] = float2ctrl(0.5);
        ctrl->pwm_out->value.dat[phase_C] = float2ctrl(0.5);

        ctl_clear_pmsm_hfi_ctrl(ctrl);
    }
}

//--------------------------------------------------------------------------
// Enable / Disable Functions
//--------------------------------------------------------------------------

/** @brief Enables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_enable_pmsm_hfi_ctrl(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}

/** @brief Disables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_disable_pmsm_hfi_ctrl(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}

/** @brief Enables PWM signal output. */
GMP_STATIC_INLINE void ctl_enable_pmsm_hfi_ctrl_output(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
}

/** @brief Disables PWM signal output. */
GMP_STATIC_INLINE void ctl_disable_pmsm_hfi_ctrl_output(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 0;
}

//--------------------------------------------------------------------------
// V_alpha_beta Mode Functions
//--------------------------------------------------------------------------

/**
 * @brief Sets the controller to V_alpha_beta mode.
 * @details In this mode, the controller bypasses all control loops and directly
 * applies the user-specified alpha-beta voltages to the SVPWM stage.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_pmsm_hfi_ctrl_valphabeta_mode(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the target alpha and beta voltage components.
 * @details This function is only effective in V_alpha_beta mode.
 * @param[out] ctrl   Pointer to the controller structure.
 * @param[in]  valpha The target alpha-axis voltage.
 * @param[in]  vbeta  The target beta-axis voltage.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_hfi_ctrl_valphabeta(pmsm_hfi_controller_t* ctrl, ctrl_gt valpha, ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_A] = valpha;
    ctrl->vab0_set.dat[phase_B] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

//--------------------------------------------------------------------------
// Vdq (Voltage) Mode Functions
//--------------------------------------------------------------------------

/**
 * @brief Sets the controller to Vdq (voltage) mode.
 * @details In this mode, the user provides d-q voltage setpoints which are fed
 * to the inverse Park transform. Current and velocity loops are bypassed.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_pmsm_hfi_ctrl_voltage_mode(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the d-q voltage feedforward (or reference) values.
 * @details In Vdq mode, these values are the primary setpoints. In other modes,
 * they act as feedforward terms.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  vd   The target d-axis voltage.
 * @param[in]  vq   The target q-axis voltage.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_hfi_ctrl_vdq_ff(pmsm_hfi_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

//--------------------------------------------------------------------------
// Idq (Current) Mode Functions
//--------------------------------------------------------------------------

/**
 * @brief Sets the controller to Idq (current) mode.
 * @details Activates the current control loop. The user provides d-q current
 * setpoints. The velocity loop is bypassed.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_pmsm_hfi_ctrl_current_mode(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the d-q current feedforward (or reference) values.
 * @details In Idq mode, these values are the primary setpoints. In velocity mode,
 * they act as feedforward terms.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  id   The target d-axis current.
 * @param[in]  iq   The target q-axis current.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_hfi_ctrl_idq_ff(pmsm_hfi_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

//--------------------------------------------------------------------------
// Velocity Mode Functions
//--------------------------------------------------------------------------

/**
 * @brief Sets the controller to velocity mode.
 * @details Activates the cascaded velocity and current control loops.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_pmsm_hfi_ctrl_velocity_mode(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 0;
}

/**
 * @brief Sets the target speed for the velocity controller.
 * @param[out] ctrl Pointer to the controller structure.
 * @param[in]  spd  The target speed in per-unit format.
 */
GMP_STATIC_INLINE void ctl_set_pmsm_hfi_ctrl_speed(pmsm_hfi_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

//--------------------------------------------------------------------------
// Position Mode Functions
//--------------------------------------------------------------------------

/**
 * @brief Sets the controller to position mode.
 * @details Activates all control loops: position, velocity, and current.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_pmsm_hfi_ctrl_position_mode(pmsm_hfi_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;
}

/**
 * @brief Sets the target position for the position controller.
 * @param[out] ctrl       Pointer to the controller structure.
 * @param[in]  revolution The target number of full revolutions.
 * @param[in]  pos        The fractional target position within the revolution (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_set_pmsm_hfi_ctrl_position(pmsm_hfi_controller_t* ctrl, int32_t revolution, ctrl_gt pos)
{
    ctrl->revolution_set = revolution;
    ctrl->pos_set = pos;
}

/** @} */ // end of PMSM_HFI_BARE_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_HFI_H_
