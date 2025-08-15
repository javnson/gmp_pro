/**
 * @file motor_current_ctrl.h
 * @brief Implements a generic FOC (Field-Oriented Control) current controller.
 *
 * @version 0.3
 * @date 2025-08-06
 *
 */

#ifndef _FILE_MOTOR_CURRENT_CTRL_H_
#define _FILE_MOTOR_CURRENT_CTRL_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* FOC Current Controller                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CURRENT_CONTROLLER FOC Current Controller
 * @brief The core current regulation loop for an FOC motor controller.
 * @details This module forms the core of an FOC system. It takes three-phase
 * current measurements, a rotor angle, and d-q axis current references,
 * then performs the necessary coordinate transformations (Clarke, Park) and
 * PI control to generate the required d-q axis voltages. These voltages are
 * then transformed back to the stationary frame (Inverse Park) to be used by
 * a PWM modulator.
 * The controller implements the PMSM voltage equations in the d-q frame:
 * @f[ v_d = R_s i_d + L_d \frac{di_d}{dt} - \omega_e L_q i_q @f]
 * @f[ v_q = R_s i_q + L_q \frac{di_q}{dt} + \omega_e L_d i_d + \omega_e \psi_f @f]
 * The PI controllers regulate i_d and i_q to their reference values.
 * @{
 */

//================================================================================
// Type Defines & Macros
//================================================================================

/**
 * @brief Main structure for the FOC current controller.
 */
typedef struct
{
    // --- Controller Entities ---
    ctl_pid_t idq_ctrl[2]; ///< PI controllers for d-axis and q-axis currents.

    // --- Inputs (updated each cycle) ---
    ctl_vector2_t idq_ref; ///< The d-q axis current reference vector [id_ref, iq_ref]^T.
    ctl_vector2_t vdq_ff;  ///< The d-q axis voltage feedforward vector [vd_ff, vq_ff]^T.

    // --- Outputs & Intermediate Variables ---
    ctl_vector3_t iab0; ///< The 3-phase currents in the alpha-beta stationary frame.
    ctl_vector3_t idq0; ///< The 3-phase currents in the d-q rotating frame.
    ctl_vector3_t vdq0; ///< The calculated d-q axis output voltages.
    ctl_vector3_t vab0; ///< The final alpha-beta voltages to be sent to the modulator.

    // --- State ---
    fast_gt flag_enable_controller; ///< Flag to enable or disable the PI controller action.

} ctl_current_controller_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the current controller structure to safe defaults.
 * @param[out] cc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_current_controller(ctl_current_controller_t* cc)
{
    ctl_clear_pid(&cc->idq_ctrl[0]);
    ctl_clear_pid(&cc->idq_ctrl[1]);
    ctl_vector2_clear(&cc->idq_ref);
    ctl_vector2_clear(&cc->vdq_ff);
    ctl_vector3_clear(&cc->iab0);
    ctl_vector3_clear(&cc->idq0);
    ctl_vector3_clear(&cc->vdq0);
    ctl_vector3_clear(&cc->vab0);
}

/**
 * @brief Sets up the parameters for the d-q axis PI controllers.
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  kp Proportional gain.
 * @param[in]  Ti Integral time constant.
 * @param[in]  Td Derivative time constant (usually 0 for a PI controller).
 * @param[in]  out_max Maximum output limit (voltage).
 * @param[in]  out_min Minimum output limit (voltage).
 * @param[in]  fs Controller execution frequency (Hz).
 */
void ctl_init_current_controller(ctl_current_controller_t* cc, ctrl_gt kp, ctrl_gt Ti, ctrl_gt Td, ctrl_gt out_max,
                                 ctrl_gt out_min, parameter_gt fs);

/**
 * @brief Sets the d-q axis current reference (target).
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  id_ref The target d-axis current.
 * @param[in]  iq_ref The target q-axis current.
 */
GMP_STATIC_INLINE void ctl_set_current_ref(ctl_current_controller_t* cc, ctrl_gt id_ref, ctrl_gt iq_ref)
{
    cc->idq_ref.dat[0] = id_ref;
    cc->idq_ref.dat[1] = iq_ref;
}

/**
 * @brief Sets the d-q axis voltage feedforward terms.
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  vd_ff The d-axis voltage feedforward term.
 * @param[in]  vq_ff The q-axis voltage feedforward term.
 */
GMP_STATIC_INLINE void ctl_set_voltage_ff(ctl_current_controller_t* cc, ctrl_gt vd_ff, ctrl_gt vq_ff)
{
    cc->vdq_ff.dat[0] = vd_ff;
    cc->vdq_ff.dat[1] = vq_ff;
}

/**
 * @brief Executes one step of the FOC current control loop.
 * @param[out] cc      Pointer to the current controller structure.
 * @param[in]  iabc    Pointer to the measured 3-phase currents.
 * @param[in]  theta   The current electrical angle of the rotor (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_step_current_controller(ctl_current_controller_t* cc, const ctl_vector3_t* _iabc,
                                                   ctrl_gt theta)
{
    ctl_vector2_t phasor;
    ctl_set_phasor_via_angle(theta, &phasor);

    // 1. Clarke Transform: 3-phase currents to alpha-beta stationary frame.
    ctl_ct_clarke(_iabc, &cc->iab0);

    // 2. Park Transform: Stationary frame currents to d-q rotating frame.
    ctl_ct_park(&cc->iab0, &phasor, &cc->idq0);

    // 3. Execute PI controllers if enabled.
    if (cc->flag_enable_controller)
    {
        // Calculate error and step the PI controllers
        ctrl_gt err_d = cc->idq_ref.dat[0] - cc->idq0.dat[0];
        ctrl_gt err_q = cc->idq_ref.dat[1] - cc->idq0.dat[1];
        cc->vdq0.dat[0] = ctl_step_pid_ser(&cc->idq_ctrl[0], err_d);
        cc->vdq0.dat[1] = ctl_step_pid_ser(&cc->idq_ctrl[1], err_q);
    }
    else
    {
        cc->vdq0.dat[0] = 0.0f;
        cc->vdq0.dat[1] = 0.0f;
    }

    // 4. Add feedforward voltages.
    cc->vdq0.dat[0] += cc->vdq_ff.dat[0];
    cc->vdq0.dat[1] += cc->vdq_ff.dat[1];
    cc->vdq0.dat[2] = 0.0f; // Zero-sequence component is always zero.

    // 5. Inverse Park Transform: d-q voltages back to alpha-beta stationary frame.
    ctl_ct_ipark(&cc->vdq0, &phasor, &cc->vab0);
}

/**
 * @brief Enables the PI controller action.
 * @param[out] cc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_enable_current_controller(ctl_current_controller_t* cc)
{
    cc->flag_enable_controller = 1;
}

/**
 * @brief Disables the PI controller action.
 * @details When disabled, the controller output will be zero, but feedforward terms will still be applied.
 * @param[out] cc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_disable_current_controller(ctl_current_controller_t* cc)
{
    cc->flag_enable_controller = 0;
}

/** 
 *@} 
 */ // end of CURRENT_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_CURRENT_CTRL_H_
