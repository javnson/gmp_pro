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

#ifndef MC_CURRENT_OUTPUT_LIMIT_BY_SQRT
#define MC_CURRENT_OUTPUT_LIMIT_BY_SQRT (0)
#endif // MC_CURRENT_OUTPUT_LIMIT_BY_SQRT

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

    ctl_vector2_t vdq_ff; //!< The d-q axis voltage feed forward vector [vd_ff, vq_ff]^T.

    // --- Outputs & Intermediate Variables ---
    ctl_vector3_t vab0; //!< The final alpha-beta voltages to be sent to the modulator.

    //
    // --- Setpoints & Intermediate Variables (Read/Write) ---
    //
    ctl_vector2_t idq_ref; //!< The d-q axis current reference vector [id_ref, iq_ref]^T.
    ctl_vector2_t
        vdq_ref; //!< The d-q axis voltage reference vector [id_ref, iq_ref]^T, provided by user or current controller.

    //
    // --- Measurement & Internal State Variables (Read-Only) ---
    //
    ctrl_gt udc;          //!< Udc after filter.
    ctl_vector3_t iuvw;   //!< sampled current after filter.
    ctl_vector2_t phasor; //!< phasor to park/ipark transform

    ctl_vector3_t iab0; //!< The 3-phase currents in the alpha-beta stationary frame.
    ctl_vector3_t idq0; //!< The 3-phase currents in the d-q rotating frame.

    ctl_vector2_t vdq_ctrl_out; //!< PID Controller result
    ctl_vector2_t vdq_decouple; //!< Decoupling

    ctl_vector3_t vdq_out;                 //!< vdq output = vdq_ref + vdq_ff
    ctl_vector2_t vdq_out_bus_compensator; //!< vdq after Udc compensator
    ctl_vector2_t vdq_out_sat;             //!< vdq output after saturation

    ctrl_gt max_vq_mag; //!< Calculate by id

    ctl_vector3_t vdq0; //!< The calculated d-q axis output voltages.

    //ctl_vector3_t vdq_comp; //!< vdq after compensator
    //ctl_vector3_t vdq_out;  //!< vdq after compensator

    //
    // --- Controller Entities ---
    //
    ctl_filter_IIR1_t filter_iuvw[3]; //!< CTRL: current input filter
    ctl_filter_IIR1_t filter_udc;     //!< CTRL: DC bus voltage input filter

    ctl_pid_t idq_ctrl[2];           //!< PI controllers for d-axis and q-axis currents.
    ctrl_lead_t lead_compensator[2]; //!< output lead compensator

    //
    // --- Feed-forward & Parameters ---
    //
    ctrl_gt coef_ff_decouple[2]; //!< CTRL: current feed-foreword
    ctrl_gt max_vs_mag;          //!< output voltage limit
    ctrl_gt max_dcbus_voltage;   //!< output voltage under 1 pu Vbus, default is 1.732 when Vbus = Vbase.

    // --- State ---
    fast_gt flag_enable_current_ctrl; //!< Flag to enable or disable the PI controller action.
    fast_gt
        flag_enable_theta_calc; //!< Flag to enable theta -> phasor calculation. If this flag is disabled using phasor directly.
    fast_gt flag_enable_lead_compensator; //!< Enables the output lead compensator.
    fast_gt flag_enable_decouple;         //!< Enable decoupling
    fast_gt flag_enable_bus_compensation; //!< Enable V bus compensation
    fast_gt flag_enable_vdq_feedforward;  //!<  Enable vdq feed forward

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
    // 1. Clear Filters
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_U]); // 修正命名 iabc -> iuvw
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_V]);
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_W]);
    ctl_clear_filter_iir1(&mc->filter_udc);

    // 2. Clear Controllers
    ctl_clear_pid(&mc->idq_ctrl[phase_d]);
    ctl_clear_pid(&mc->idq_ctrl[phase_q]);

    ctl_clear_lead(&mc->lead_compensator[phase_d]);
    ctl_clear_lead(&mc->lead_compensator[phase_q]);

    // 3. Clear State Vectors & Intermediate Variables
    ctl_vector2_clear(&mc->vdq_ff);

    ctl_vector3_clear(&mc->iuvw);
    ctl_vector3_clear(&mc->iab0);
    ctl_vector3_clear(&mc->idq0);

    ctl_vector2_clear(&mc->vdq_ctrl_out);
    ctl_vector2_clear(&mc->vdq_decouple);
    ctl_vector3_clear(&mc->vdq_out);
    ctl_vector2_clear(&mc->vdq_out_bus_compensator);
    ctl_vector2_clear(&mc->vdq_out_sat);

    ctl_vector3_clear(&mc->vdq0);
    ctl_vector3_clear(&mc->vab0);

    mc->udc = 0;
    mc->isr_tick = 0;
}

// 注意电压基值应当按照变流器输出最大电压即Udc/SQRT(3)来计算
// 电流基值应当按照变流器最大允许输出电流来计算，这样最合理

typedef struct _tag_mtr_current_ctrl
{
    // [fatal] the following information is key parameter for auto-tuning.
    parameter_gt fs;            //!< Controller execution frequency (Hz).
    parameter_gt v_bus;         //!< DC Bus voltage for V bus compensator (V).
    parameter_gt v_phase_limit; //!< Phase voltage limitation(Vrms).
    parameter_gt v_base;        //!< Base voltage for per-unit conversion (V).
    parameter_gt i_base;        //!< Base current for per-unit conversion (A).
    parameter_gt freq_base;     //!< Nominal motor elec-frequency (e.g., 50 or 100 Hz).
    parameter_gt spd_base;      //!< Nominal motor speed base, krpm.
    parameter_gt pole_pairs;    //!< pole pairs

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
    mc->isr_tick += 1;

    //
    // 1. enable theta input or phasor input
    //
    if (mc->flag_enable_theta_calc)
    {
        ctl_set_phasor_via_angle(mc->pos_if->elec_position, &mc->phasor);
    }
    else
    {
        gmp_base_assert(mc->phasor_input);
        ctl_vector2_copy(&mc->phasor, mc->phasor_input);
    }

    //
    // 2. input filter
    //
#if MC_CURRENT_SAMPLE_PHASE_MODE == 3
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);
    mc->iuvw.dat[phase_W] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_W], mc->adc_iuvw->value.dat[phase_C]);

    // 1. Clarke Transform: 3-phase currents to alpha-beta stationary frame.
    ctl_ct_clarke(&mc->iuvw, &mc->iab0);

#elif MC_CURRENT_SAMPLE_PHASE_MODE == 2
    mc->iuvw.dat[phase_U] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_U], mc->adc_iuvw->value.dat[phase_A]);
    mc->iuvw.dat[phase_V] = ctl_step_filter_iir1(&mc->filter_iuvw[phase_V], mc->adc_iuvw->value.dat[phase_B]);

    // 1. Clarke Transform: 3-phase currents to alpha-beta stationary frame.
    ctl_ct_clarke_2ph((ctl_vector2_t*)&mc->iuvw, (ctl_vector2_t*)&mc->iab0);
    mc->iab0.dat[phase_0] = 0;

#endif // GFL_CURRENT_SAMPLE_PHASE_MODE

    // input DC Bus Voltage filter
    mc->udc = ctl_step_filter_iir1(&mc->filter_udc, mc->adc_udc->value);

    //
    // 3. Park Transform: alpha-beta -> d-q
    //
    ctl_ct_park(&mc->iab0, &mc->phasor, &mc->idq0);

    //
    // 4. Controller kernel, current loop
    //
    if (mc->flag_enable_current_ctrl)
    {
        //
        // 4.1 Calculate error and step the PI controllers
        //
        ctrl_gt err_d = mc->idq_ref.dat[phase_d] - mc->idq0.dat[phase_d];
        ctrl_gt err_q = mc->idq_ref.dat[phase_q] - mc->idq0.dat[phase_q];

        // d axis controller limited by Vs,max, q axis controller limited by vd
        //ctl_set_pid_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
        //ctl_set_pid_limit(&mc->idq_ctrl[phase_q], mc->max_vq_mag, -mc->max_vq_mag);

        mc->vdq_ctrl_out.dat[phase_d] = ctl_step_pid_ser(&mc->idq_ctrl[phase_d], err_d);
        mc->vdq_ctrl_out.dat[phase_q] = ctl_step_pid_ser(&mc->idq_ctrl[phase_q], err_q);

        //
        // 4.2 Calculate feed forward decoupling
        //
        if (mc->flag_enable_decouple)
        {
            // decoupling
            mc->vdq_decouple.dat[phase_d] =
                -ctl_mul(mc->spd_if->speed, ctl_mul(mc->coef_ff_decouple[phase_d], mc->idq0.dat[phase_q]));
            mc->vdq_decouple.dat[phase_q] =
                ctl_mul(mc->spd_if->speed, ctl_mul(mc->coef_ff_decouple[phase_q], mc->idq0.dat[phase_d]));

            mc->vdq_ctrl_out.dat[phase_d] += mc->vdq_decouple.dat[phase_d];
            mc->vdq_ctrl_out.dat[phase_q] += mc->vdq_decouple.dat[phase_q];
        }

        //
        // 4.3 lead compensator
        //
        if (mc->flag_enable_lead_compensator)
        {
            mc->vdq_ref.dat[phase_d] = ctl_step_lead(&mc->lead_compensator[phase_d], mc->vdq0.dat[phase_d]);
            mc->vdq_ref.dat[phase_q] = ctl_step_lead(&mc->lead_compensator[phase_q], mc->vdq0.dat[phase_q]);
        }
        else
        {
            ctl_vector2_copy(&mc->vdq_ref, &mc->vdq_ctrl_out);
        }
    }

    //
    // 5. vdq feed forward
    //
    if (mc->flag_enable_vdq_feedforward)
    {
        ctl_vector2_add((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref, &mc->vdq_ff);
    }
    else
    {
        ctl_vector2_copy((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref);
    }

    //
    // 6. Bus Voltage Compensation, and vdq feed forward
    //
    if (mc->flag_enable_bus_compensation)
    {
        ctrl_gt v_scale;
        if (mc->udc > float2ctrl(0.5f))                // prevent div 0
            v_scale = mc->max_dcbus_voltage / mc->udc; // udc is per unit value
        else
            v_scale = mc->max_dcbus_voltage;

        mc->vdq_out_bus_compensator.dat[phase_d] = ctl_mul(mc->vdq_out.dat[phase_d], v_scale);
        mc->vdq_out_bus_compensator.dat[phase_q] = ctl_mul(mc->vdq_out.dat[phase_q], v_scale);
    }
    else
    {
        ctl_vector2_copy(&mc->vdq_out_bus_compensator, (ctl_vector2_t*)&mc->vdq_out);
    }

    //
    // 6. output Circular Saturation
    //

    // 6.1 Saturation output
    mc->vdq_out_sat.dat[phase_d] = ctl_sat(mc->vdq_out_bus_compensator.dat[phase_d], mc->max_vs_mag, -mc->max_vs_mag);

    // q axis controller is limited by d axis output
#if MC_CURRENT_OUTPUT_LIMIT_BY_SQRT == 1
    ctrl_gt val_sq =
        ctl_mul(mc->max_vs_mag, mc->max_vs_mag) - ctl_mul(mc->vdq_out_sat.dat[phase_d], mc->vdq_out_sat.dat[phase_d]);

    if (val_sq < 0)
        mc->max_vq_mag = 0;
    else
        mc->max_vq_mag = ctl_sqrt(val_sq);
#else  // default case
    mc->max_vq_mag = mc->max_vs_mag - mc->vdq_out_sat.dat[phase_d];
#endif // MC_CURRENT_OUTPUT_LIMIT_BY_SQRT

    mc->vdq_out_sat.dat[phase_q] =
        ctl_sat(mc->vdq_out.dat[phase_q] + mc->vdq_ff.dat[phase_q], mc->max_vq_mag, -mc->max_vq_mag);

    // 6.2 PID Anti-Windup Back-calculation
    if (mc->flag_enable_current_ctrl)
    {
        // 逻辑：PID real output = vdq_out_sat - all feed forward items

        // --- D Axis Correction ---
        ctrl_gt v_pid_d_real = mc->vdq_out_sat.dat[phase_d];
        if (mc->flag_enable_decouple)
            v_pid_d_real -= mc->vdq_decouple.dat[phase_d];
        if (mc->flag_enable_vdq_feedforward)
            v_pid_d_real -= mc->vdq_ff.dat[phase_d];

        ctl_pid_clamping_correction_using_real_output(&mc->idq_ctrl[phase_d], v_pid_d_real);

        // --- Q Axis Correction ---
        ctrl_gt v_pid_q_real = mc->vdq_out_sat.dat[phase_q];
        if (mc->flag_enable_decouple)
            v_pid_q_real -= mc->vdq_decouple.dat[phase_q];
        if (mc->flag_enable_vdq_feedforward)
            v_pid_q_real -= mc->vdq_ff.dat[phase_q];

        ctl_pid_clamping_correction_using_real_output(&mc->idq_ctrl[phase_q], v_pid_q_real);
    }

    mc->vdq_out.dat[phase_0] = 0;

    // 7. iPark: d-q -> alpha-beta
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
 * @brief Sets the d-q axis voltage reference terms.
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  vd_ff The d-axis voltage feed forward term.
 * @param[in]  vq_ff The q-axis voltage feed forward term.
 */
GMP_STATIC_INLINE void ctl_set_mtr_current_ctrl_vdq_ref(mtr_current_ctrl_t* mc, ctrl_gt vd_ff, ctrl_gt vq_ff)
{
    mc->vdq_ref.dat[0] = vd_ff;
    mc->vdq_ref.dat[1] = vq_ff;
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
