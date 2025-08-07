/**
 * @file pmsm_ctrl_mtpa.h
 * @brief Provides a PMSM controller with Maximum Torque Per Ampere (MTPA) strategy.
 * @details This file builds upon the basic FOC controller by incorporating an MTPA
 * current distributor. The speed controller's output (total current magnitude)
 * is optimally distributed into d-axis and q-axis current references to maximize
 * torque for a given current, which is particularly useful for Interior PMSMs (IPMSM).
 *
 * The controller supports the same operating modes as the base controller but enhances
 * the velocity control mode with the MTPA algorithm.
 *
 * //tex:
 * // The MTPA condition for an IPMSM is given by:
 * // i_d = \frac{-\psi_{pm} + \sqrt{\psi_{pm}^2 + 4(L_d - L_q)^2 i_q^2}}{2(L_d - L_q)}
 *
 */

#ifndef _FILE_PMSM_CTRL_MTPA_H_
#define _FILE_PMSM_CTRL_MTPA_H_

// Necessary support
#include <ctl/component/interface/interface_base.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/component/motor_control/current_loop/current_distributor.h>
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
/* PMSM MTPA (Maximum Torque Per Ampere) Controller                          */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup PMSM_MTPA_CONTROLLER PMSM MTPA Controller
 * @brief A module for controlling a PMSM using FOC with an MTPA strategy.
 * @details This module extends the standard FOC controller by adding a current
 * distributor that calculates the optimal Id and Iq references from the
 * speed controller's output to achieve maximum torque per ampere.
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
 * - **3**: Uses 3 phase voltage sensors.
 * - **2**: Uses 2 phase voltage sensors.
 * - **0**: Disables voltage measurement (default).
 */
#ifndef MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES
#define MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES ((0))
#endif // MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES

/**
 * @brief Selects the feedforward strategy for the controller.
 * @details
 * - **1**: Decoupled mode. Feedforward terms are calculated automatically (default).
 * - **0**: User mode. Feedforward values must be specified manually by the user.
 */
#ifndef MTR_CTRL_FEEDFORWARD_STRATEGY
#define MTR_CTRL_FEEDFORWARD_STRATEGY (1)
#endif // MTR_CTRL_FEEDFORWARD_STRATEGY

/**
 * @brief Enables the MTPA current distributor.
 * @details When defined, the output of the speed controller is interpreted as a
 * total current magnitude and is split into Id and Iq components by the
 * MTPA algorithm.
 */
#define PMSM_CTRL_USING_CURRENT_DISTRIBUTOR

//================================================================================
// Type Defines
//================================================================================

/**
 * @brief Main structure for the PMSM MTPA Controller.
 * @details This structure holds all the state variables, interfaces, controller
 * instances, and flags required for the MTPA FOC algorithm.
 */
typedef struct _tag_pmsm_mtpa_bare_controller
{
    //--------------------------------------------------------------------------
    // Interfaces
    //--------------------------------------------------------------------------
    mtr_ift mtr_interface; ///< Input interface for motor signals.
    tri_pwm_ift* pwm_out;  ///< Output interface for three-phase PWM.

    //--------------------------------------------------------------------------
    // Controller Entities
    //--------------------------------------------------------------------------
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
    discrete_pid_t current_ctrl[2]; ///< Discrete PID controllers for d-q axis currents.
    track_discrete_pid_t spd_ctrl;  ///< Discrete tracking PID controller for speed.
#else                               // use continuous controller
    ctl_pid_t current_ctrl[2];              ///< Continuous PID controllers for d-q axis currents.
    ctl_tracking_continuous_pid_t spd_ctrl; ///< Continuous tracking PID controller for speed.
#endif

#ifdef PMSM_CTRL_USING_CURRENT_DISTRIBUTOR
    ctl_current_distributor_t distributor; ///< Pointer to the MTPA current distributor module.
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
    ctrl_gt im_set;         ///< Target magnitude of current (p.u.).
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

} pmsm_mtpa_controller_t;

/**
 * @brief Initialization parameters for the PMSM MTPA Controller.
 */
typedef struct _tag_pmsm_mtpa_bare_controller_init
{
    parameter_gt fs; ///< Controller sampling frequency (Hz).

    //--- Current Controller ---
    parameter_gt current_d_pid_gain; ///< Proportional gain (Kp) for the d-axis current controller.
    parameter_gt current_q_pid_gain; ///< Proportional gain (Kp) for the q-axis current controller.
    parameter_gt current_d_Ti;       ///< Integral time constant (Ti) for the d-axis current controller.
    parameter_gt current_q_Ti;       ///< Integral time constant (Ti) for the q-axis current controller.
    parameter_gt current_d_Td;       ///< Derivative time constant (Td) for the d-axis current controller.
    parameter_gt current_q_Td;       ///< Derivative time constant (Td) for the q-axis current controller.
    parameter_gt voltage_limit_max;  ///< Maximum output limit for the current controllers (voltage).
    parameter_gt voltage_limit_min;  ///< Minimum output limit for the current controllers (voltage).

    //--- Speed Controller ---
    parameter_gt spd_pid_gain;      ///< Proportional gain (Kp) for the speed controller.
    parameter_gt spd_Ti;            ///< Integral time constant (Ti) for the speed controller.
    parameter_gt spd_Td;            ///< Derivative time constant (Td) for the speed controller.
    parameter_gt current_limit_max; ///< Maximum output limit for the speed controller (total current).
    parameter_gt current_limit_min; ///< Minimum output limit for the speed controller (total current).
    parameter_gt acc_limit_max;     ///< Maximum acceleration limit (p.u./s).
    parameter_gt acc_limit_min;     ///< Minimum deceleration limit (p.u./s).
    uint32_t spd_ctrl_div;          ///< Speed controller execution frequency divider.

} pmsm_mtpa_bare_controller_init_t;

//================================================================================
// Function Prototypes
//================================================================================

/**
 * @brief Initializes the PMSM MTPA Controller.
 * @param[out] ctrl Pointer to the controller structure to initialize.
 * @param[in]  init Pointer to the structure containing initialization parameters.
 */
void ctl_init_pmsm_mtpa_bare_controller(pmsm_mtpa_controller_t* ctrl, pmsm_mtpa_bare_controller_init_t* init);

/**
 * @brief Attaches the controller to a three-phase PWM output interface.
 * @param[out] ctrl    Pointer to the controller structure.
 * @param[in]  pwm_out Pointer to the PWM interface to attach.
 */
void ctl_attach_pmsm_mtpa_bare_output(pmsm_mtpa_controller_t* ctrl, tri_pwm_ift* pwm_out);

/**
 * @brief Attaches an MTPA current distributor module to the controller.
 * @param[out] ctrl        Pointer to the controller structure.
 * @param[in]  distributor Pointer to the initialized current distributor module.
 */
void ctl_attach_idq_distributor(pmsm_mtpa_controller_t* ctrl, ctl_current_distributor_t* distributor);

/**
 * @brief Resets all internal states and integrators of the PID controllers.
 * @param[out] ctrl Pointer to the controller structure to clear.
 */
GMP_STATIC_INLINE void ctl_clear_pmsm_mtpa_ctrl(pmsm_mtpa_controller_t* ctrl)
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
}

/**
 * @brief Executes one step of the PMSM FOC control loop with MTPA.
 * @param[out] ctrl Pointer to the controller structure.
 */
GMP_STATIC_INLINE void ctl_step_pmsm_mtpa_ctrl(pmsm_mtpa_controller_t* ctrl)
{
    ctl_vector2_t phasor;
    ctrl_gt etheta;
    ctrl_gt vq_limit = float2ctrl(1.0);

    ctrl->isr_tick += 1;

    if (ctrl->flag_enable_controller)
    {
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
        ctl_vector3_clear(&ctrl->uab0);
#else
#error("Wrong parameter for macro MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES")
#endif

        // Park Transformation
        etheta = ctl_get_mtr_elec_theta(&ctrl->mtr_interface);
        ctl_set_phasor_via_angle(etheta, &phasor);
        ctl_ct_park(&ctrl->iab0, &phasor, &ctrl->idq0);

#if MTR_CTRL_VOLTAGE_MEASUREMENT_PHASES != 0
        ctl_ct_park(&ctrl->uab0, &phasor, &ctrl->udq0);
#else
        ctl_vector3_clear(&ctrl->udq0);
#endif

        // Position Controller
        if (ctrl->flag_enable_position_ctrl)
        {
            // Position control logic is not yet implemented.
        }

        // Velocity Controller
        if (ctrl->flag_enable_velocity_ctrl)
        {
#if defined(PMSM_CTRL_USING_CURRENT_DISTRIBUTOR)
            // With MTPA, speed controller outputs total current magnitude
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->im_set = ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                ctl_get_mtr_velocity(&ctrl->mtr_interface));
#else // using continuous controller
            ctrl->im_set = ctl_step_tracking_continuous_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                     ctl_get_mtr_velocity(&ctrl->mtr_interface));
#endif
            // Distributor calculates optimal id and iq
            ctl_step_current_distributor(&ctrl->distributor,ctrl->im_set);
            ctrl->idq_set.dat[phase_q] = ctl_get_distributor_iq_ref(&ctrl->distributor) + ctrl->idq_ff.dat[phase_q];
            ctrl->idq_set.dat[phase_d] = ctl_get_distributor_id_ref(&ctrl->distributor) + ctrl->idq_ff.dat[phase_d];
#else
            // Without MTPA, speed controller outputs only iq
            ctrl->idq_set.dat[phase_d] = ctrl->idq_ff.dat[phase_d];
#ifdef PMSM_CTRL_USING_DISCRETE_CTRL
            ctrl->idq_set.dat[phase_q] = ctl_step_discrete_track_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                     ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#else  // using continuous controller
            ctrl->idq_set.dat[phase_q] = ctl_step_tracking_continuous_pid(&ctrl->spd_ctrl, ctrl->speed_set,
                                                                          ctl_get_mtr_velocity(&ctrl->mtr_interface)) +
                                         ctrl->idq_ff.dat[phase_q];
#endif // PMSM_CTRL_USING_DISCRETE_CTRL
#endif // PMSM_CTRL_USING_CURRENT_DISTRIBUTOR

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

        // SVPWM Output Stage
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
    else // Controller is disabled
    {
        ctrl->pwm_out->value.dat[phase_A] = 0;
        ctrl->pwm_out->value.dat[phase_B] = 0;
        ctrl->pwm_out->value.dat[phase_C] = 0;
        ctl_clear_pmsm_mtpa_ctrl(ctrl);
    }
}

//--------------------------------------------------------------------------
// Enable / Disable and Mode Setting Functions
// (These functions are identical in structure to the HFI controller)
//--------------------------------------------------------------------------

/** @brief Enables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_enable_pmsm_mtpa_ctrl(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 1;
}

/** @brief Disables the entire PMSM controller. */
GMP_STATIC_INLINE void ctl_disable_pmsm_mtpa_ctrl(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_controller = 0;
}

/** @brief Enables PWM signal output. */
GMP_STATIC_INLINE void ctl_enable_pmsm_mtpa_ctrl_output(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
}

/** @brief Disables PWM signal output. */
GMP_STATIC_INLINE void ctl_disable_pmsm_mtpa_ctrl_output(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 0;
}

/** @brief Sets the controller to V_alpha_beta mode. */
GMP_STATIC_INLINE void ctl_pmsm_mtpa_ctrl_valphabeta_mode(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 0;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target alpha and beta voltage components. */
GMP_STATIC_INLINE void ctl_set_pmsm_mtpa_ctrl_valphabeta(pmsm_mtpa_controller_t* ctrl, ctrl_gt valpha, ctrl_gt vbeta)
{
    ctrl->vab0_set.dat[phase_A] = valpha;
    ctrl->vab0_set.dat[phase_B] = vbeta;
    ctrl->vab0_set.dat[phase_0] = 0;
}

/** @brief Sets the controller to Vdq (voltage) mode. */
GMP_STATIC_INLINE void ctl_pmsm_mtpa_ctrl_voltage_mode(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 0;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the d-q voltage feedforward (or reference) values. */
GMP_STATIC_INLINE void ctl_set_pmsm_mtpa_ctrl_vdq_ff(pmsm_mtpa_controller_t* ctrl, ctrl_gt vd, ctrl_gt vq)
{
    ctrl->vdq_ff.dat[phase_d] = vd;
    ctrl->vdq_ff.dat[phase_q] = vq;
}

/** @brief Sets the controller to Idq (current) mode. */
GMP_STATIC_INLINE void ctl_pmsm_mtpa_ctrl_current_mode(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 0;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the d-q current feedforward (or reference) values. */
GMP_STATIC_INLINE void ctl_set_pmsm_mtpa_ctrl_idq_ff(pmsm_mtpa_controller_t* ctrl, ctrl_gt id, ctrl_gt iq)
{
    ctrl->idq_ff.dat[phase_d] = id;
    ctrl->idq_ff.dat[phase_q] = iq;
}

/** @brief Sets the controller to velocity mode (with MTPA). */
GMP_STATIC_INLINE void ctl_pmsm_mtpa_ctrl_velocity_mode(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 0;
}

/** @brief Sets the target speed for the velocity controller. */
GMP_STATIC_INLINE void ctl_set_pmsm_mtpa_ctrl_speed(pmsm_mtpa_controller_t* ctrl, ctrl_gt spd)
{
    ctrl->speed_set = spd;
}

/** @brief Sets the controller to position mode. */
GMP_STATIC_INLINE void ctl_pmsm_mtpa_ctrl_position_mode(pmsm_mtpa_controller_t* ctrl)
{
    ctrl->flag_enable_output = 1;
    ctrl->flag_enable_modulation = 1;
    ctrl->flag_enable_current_ctrl = 1;
    ctrl->flag_enable_velocity_ctrl = 1;
    ctrl->flag_enable_position_ctrl = 1;
}

/** @brief Sets the target position for the position controller. */
GMP_STATIC_INLINE void ctl_set_pmsm_mtpa_ctrl_position(pmsm_mtpa_controller_t* ctrl, int32_t revolution, ctrl_gt pos)
{
    ctrl->revolution_set = revolution;
    ctrl->pos_set = pos;
}

/** @} */ // end of PMSM_MTPA_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_CTRL_MTPA_H_
