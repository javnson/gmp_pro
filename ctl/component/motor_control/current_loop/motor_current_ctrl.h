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

#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>

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

#ifndef MC_CURRENT_SAMPLE_PHASE_MODE
#define MC_CURRENT_SAMPLE_PHASE_MODE (3)
#endif // MC_CURRENT_SAMPLE_PHASE_MODE

/**
 * @brief Main structure for the FOC current controller.
 */
typedef struct _tag_current_controller
{
    uint32_t isr_tick; //!< Controller Tick

    // --- Inputs (updated each cycle) ---
    tri_adc_ift* adc_iuvw; //!< The current sampled.
    adc_ift* adc_udc;      //!< DC Bus voltage.

    rotation_ift* pos_if; //!< @brief Standard rotation input interface.
    velocity_ift* spd_if; //!< @brief Standard velocity input interface.

    ctl_vector2_t* phasor_input; //!< input rotor phasor

    // --- Outputs & Intermediate Variables ---
    ctl_vector3_t iab0; //!< The 3-phase currents in the alpha-beta stationary frame.

    //
    // --- Feed-forward & Parameters ---
    //
    ctrl_gt coef_ff_decouple[2]; //!< CTRL: current feed-foreword

    //
    // --- Setpoints & Intermediate Variables (Read/Write) ---
    //
    ctl_vector2_t idq_ref; //!< The d-q axis current reference vector [id_ref, iq_ref]^T.
    ctl_vector2_t vdq_ff;  //!< The d-q axis voltage feedforward vector [vd_ff, vq_ff]^T.

    //
    // --- Measurement & Internal State Variables (Read-Only) ---
    //
    ctl_vector2_t phasor;   //!< phasor to park/ipark transform
    ctrl_gt udc;            //!< Udc after filter.
    ctl_vector3_t iuvw;     //!< sampled current after filter.
    ctl_vector3_t idq0;     //!< The 3-phase currents in the d-q rotating frame.
    ctl_vector3_t vdq0;     //!< The calculated d-q axis output voltages.
    ctl_vector3_t vab0;     //!< The final alpha-beta voltages to be sent to the modulator.
    ctl_vector2_t v_dec;    //!< Decoupling
    ctl_vector3_t vdq_comp; //!< vdq after compensator
    ctl_vector3_t vdq_out;  //!< vdq after compensator

    //
    // --- Controller Entities ---
    //
    ctl_filter_IIR1_t filter_iabc[3]; //!< CTRL: current input filter
    ctl_filter_IIR1_t filter_udc;     //!< CTRL: current input filter
    ctl_pid_t idq_ctrl[2];            //!< PI controllers for d-axis and q-axis currents.
    ctrl_lead_t lead_compensator[2];  //!< output lead compensator

    // --- State ---
    fast_gt flag_enable_current_ctrl; //!< Flag to enable or disable the PI controller action.
    fast_gt
        flag_enable_theta_calc; //!< Flag to enable theta -> phasor calculation. If this flag is disabled using phasor directly.
    fast_gt flag_enable_lead_compensator; //!< Enables the output lead compensator.
    fast_gt flag_enable_decouple;         //!< Enable decoupling
    fast_gt flag_enable_bus_compensation; //!< Enable V bus compensation

} mtr_current_ctrl_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the current controller structure to safe defaults.
 * @param[out] mc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_mtr_current_ctrl(mtr_current_ctrl_t* mc)
{
    ctl_clear_filter_iir1(&mc->filter_iabc[phase_U]);
    ctl_clear_filter_iir1(&mc->filter_iabc[phase_V]);
    ctl_clear_filter_iir1(&mc->filter_iabc[phase_W]);

    ctl_clear_pid(&mc->idq_ctrl[phase_d]);
    ctl_clear_pid(&mc->idq_ctrl[phase_q]);

    ctl_clear_lead(&mc->lead_compensator[phase_d]);
    ctl_clear_lead(&mc->lead_compensator[phase_q]);

    //ctl_vector2_clear(&mc->idq_ref);
    ctl_vector2_clear(&mc->vdq_ff);
    ctl_vector3_clear(&mc->iuvw);
    ctl_vector3_clear(&mc->iab0);
    ctl_vector3_clear(&mc->idq0);
    ctl_vector3_clear(&mc->vdq0);
    ctl_vector3_clear(&mc->vab0);
}

// 注意电压基值应当按照变流器输出最大电压即Udc/SQRT(3)来计算
// 电流基值应当按照变流器最大允许输出电流来计算，这样最合理

typedef struct _tag_mtr_current_ctrl
{
    // [fatal] the following information is key parameter for auto-tuning.
    parameter_gt fs;         //!< Controller execution frequency (Hz).
    parameter_gt v_base;     //!< Base voltage for per-unit conversion (V).
    parameter_gt i_base;     //!< Base current for per-unit conversion (A).
    parameter_gt freq_base;  //!< Nominal motor elec-frequency (e.g., 50 or 100 Hz).
    parameter_gt spd_base;   //!< Nominal motor speed base, krpm.
    parameter_gt pole_pairs; //!< pole pairs

    // [fatal] the following information is key parameter for auto-tuning.
    parameter_gt mtr_Ld; //!< motor inductor of d
    parameter_gt mtr_Lq; //!< motor inductor of q
    parameter_gt mtr_Rs; //!< motor resistor of stator

    // the following parameters would be calculated by auto-tuning
    parameter_gt current_adc_fc; //!< Current ADC filter cut frequency (Hz).
    parameter_gt voltage_adc_fc; //!< Voltage ADC filter cut frequency (Hz).

    // the following parameters would be calculated by auto-tuning.
    parameter_gt current_loop_bw;   //!< Current loop bandwidth frequency (Hz).
    parameter_gt current_phase_lag; //!< Current loop output compensate angle (rad).

    // the following parameters would be calculated by auto PI tuning.
    parameter_gt kpd;
    parameter_gt kid;
    parameter_gt kpq;
    parameter_gt kiq;

} mtr_current_init_t;

/**
 * @brief Auto-tuning motor driver parameters.
 * @param[in,out] init Pointer to the `mtr_current_init_t` structure.
 */
void ctl_auto_tuning_mtr_current_ctrl(mtr_current_init_t* init);

/**
 * @brief Sets up the parameters for the d-q axis PI controllers.
 * @param[out] mc Pointer to the current controller structure.
 * @param[in]  init initialize object for Motor controller.
 */
void ctl_init_mtr_current_ctrl(mtr_current_ctrl_t* mc, mtr_current_init_t* init);

/**
 * @brief Executes one step of the FOC current control loop.
 * @param[out] mc      Pointer to the current controller structure.
 * @param[in]  theta   The current electrical angle of the rotor (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_step_current_controller(mtr_current_ctrl_t* mc)
{
    // enable theta input or phasor input
    if (mc->flag_enable_theta_calc)
        ctl_set_phasor_via_angle(mc->pos_if->elec_position, &mc->phasor);
    else
        ctl_vector2_copy(&mc->phasor, mc->phasor_input);

        // input current filter
#if MC_CURRENT_SAMPLE_PHASE_MODE == 3
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iabc[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iabc[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    mc->iuvw.dat[phase_W] = ctl_step_filter_iir1(&mc->filter_iabc[phase_W], mc->adc_iuvw->value.dat[phase_C]);

    // 1. Clarke Transform: 3-phase currents to alpha-beta stationary frame.
    ctl_ct_clarke(&mc->iuvw, &mc->iab0);

#elif MC_CURRENT_SAMPLE_PHASE_MODE == 2
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iabc[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iabc[phase_V], mc->adc_iuvw->value.dat[phase_B]);

    // 1. Clarke Transform: 3-phase currents to alpha-beta stationary frame.
    ctl_ct_clarke_2ph((ctl_vector2_t*)&mc->iuvw, (ctl_vector2_t*)&mc->iab0);
    mc->iab0.dat[phase_0] = 0;

#endif // GFL_CURRENT_SAMPLE_PHASE_MODE

    // input DC Bus Voltage filter
    mc->udc = ctl_step_filter_iir1(&mc->filter_udc, mc->adc_udc->value);

    // 2. Park Transform: Stationary frame currents to d-q rotating frame.
    ctl_ct_park(&mc->iab0, &mc->phasor, &mc->idq0);

    // 3. Execute PI controllers if enabled.
    if (mc->flag_enable_current_ctrl)
    {
        // Calculate error and step the PI controllers
        ctrl_gt err_d = mc->idq_ref.dat[phase_d] - mc->idq0.dat[phase_d];
        ctrl_gt err_q = mc->idq_ref.dat[phase_q] - mc->idq0.dat[phase_q];

        mc->vdq0.dat[phase_d] = ctl_step_pid_ser(&mc->idq_ctrl[phase_d], err_d);
        mc->vdq0.dat[phase_q] = ctl_step_pid_ser(&mc->idq_ctrl[phase_q], err_q);

        // decoupling
        mc->v_dec.dat[phase_d] = -mc->spd_if->speed * mc->coef_ff_decouple[phase_d] * mc->idq0.dat[phase_q];
        mc->v_dec.dat[phase_q] = mc->spd_if->speed * mc->coef_ff_decouple[phase_q] * mc->idq0.dat[phase_d];

        if (mc->flag_enable_decouple)
        {
            mc->vdq0.dat[phase_d] += mc->v_dec.dat[phase_d];
            mc->vdq0.dat[phase_q] += mc->v_dec.dat[phase_q];
        }
    }
    else
    {
        mc->vdq0.dat[0] = 0.0f;
        mc->vdq0.dat[1] = 0.0f;
    }

    // --- 3d. lead compensator ---
    if (mc->flag_enable_lead_compensator)
    {
        mc->vdq_comp.dat[phase_d] = ctl_step_lead(&mc->lead_compensator[phase_d], mc->vdq0.dat[phase_d]);
        mc->vdq_comp.dat[phase_q] = ctl_step_lead(&mc->lead_compensator[phase_q], mc->vdq0.dat[phase_q]);
    }
    else
    {
        ctl_vector3_copy(&mc->vdq_comp, &mc->vdq_out);
    }

    //    // 4. Add feed forward voltages.
    //    mc->vdq_out_comp.dat[0] += mc->vdq_ff.dat[0];
    //    mc->vdq_out_comp.dat[1] += mc->vdq_ff.dat[1];
    //    mc->vdq_out_comp.dat[2] = 0.0f; // Zero-sequence component is always zero.

    // D. Bus Voltage Compensation
    if (mc->flag_enable_bus_compensation)
    {
        ctrl_gt v_scale;
        if (mc->udc > 0.1f)             // prevent div 0
            v_scale = 1.732f / mc->udc; // udc is per unit value
        else
            v_scale = 1.732f;

        mc->vdq_out.dat[0] = (mc->vdq_comp.dat[0] + mc->vdq_ff.dat[0]) * v_scale;
        mc->vdq_out.dat[1] = (mc->vdq_comp.dat[1] + mc->vdq_ff.dat[1]) * v_scale;
    }

    // saturation
    mc->vdq_out.dat[0] = ctl_sat(mc->vdq_comp.dat[0], 1.0f, -1.0f);
    mc->vdq_out.dat[1] = ctl_sat(mc->vdq_comp.dat[1], 1.0f, -1.0f);

    // 5. IPark: d-q -> alpha-beta
    ctl_ct_ipark(&mc->vdq_out, &mc->phasor, &mc->vab0);
}

/**
 * @brief Enables the PI controller action.
 * @param[out] cc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_enable_mtr_current_ctrl(mtr_current_ctrl_t* mc)
{
    mc->flag_enable_current_ctrl = 1;
}

/**
 * @brief Sets the d-q axis current reference (target).
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  id_ref The target d-axis current.
 * @param[in]  iq_ref The target q-axis current.
 */
GMP_STATIC_INLINE void ctl_set_mtr_current_ctrl_ref(mtr_current_ctrl_t* mc, ctrl_gt id_ref, ctrl_gt iq_ref)
{
    mc->idq_ref.dat[0] = id_ref;
    mc->idq_ref.dat[1] = iq_ref;
}

/**
 * @brief Sets the d-q axis voltage feed forward terms.
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  vd_ff The d-axis voltage feed forward term.
 * @param[in]  vq_ff The q-axis voltage feed forward term.
 */
GMP_STATIC_INLINE void ctl_set_mtr_current_ctrl_vdq_ff(mtr_current_ctrl_t* mc, ctrl_gt vd_ff, ctrl_gt vq_ff)
{
    mc->vdq_ff.dat[0] = vd_ff;
    mc->vdq_ff.dat[1] = vq_ff;
}

/**
 * @brief Disables the PI controller action.
 * @details When disabled, the controller output will be zero, but feedforward terms will still be applied.
 * @param[out] cc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_disable_mtr_current_ctrl(mtr_current_ctrl_t* mc)
{
    mc->flag_enable_current_ctrl = 0;
}

GMP_STATIC_INLINE void ctl_attach_mtr_current_ctrl_port(mtr_current_ctrl_t* mc, tri_adc_ift* _iabc, adc_ift* _udc,
                                                        rotation_ift* _pos_if, velocity_ift* _vec_if)
{
    mc->adc_iuvw = _iabc;
    mc->adc_udc = _udc;
    mc->pos_if = _pos_if;
    mc->spd_if = _vec_if;
}

GMP_STATIC_INLINE void ctl_attach_mtr_current_ctrl_phasor(mtr_current_ctrl_t* mc, ctl_vector2_t* _phasor)
{
    mc->phasor_input = _phasor;
}

/** 
 *@} 
 */ // end of CURRENT_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_CURRENT_CTRL_H_
