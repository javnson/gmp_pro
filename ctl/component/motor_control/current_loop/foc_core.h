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

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/motor_control/interface/motor_universal_interface.h>

/* Uncomment to use DQ-LADRC1. When it is not defined, DQ-PI is used. */
// #define ENABLE_FOC_LADRC_CTRL

#ifndef ENABLE_FOC_LADRC_CTRL
#include <ctl/component/intrinsic/complex/dq_pi.h>
#else
#include <ctl/component/intrinsic/complex/dq_ladrc1.h>
#endif
#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>

#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>

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
 * The selected DQ controller regulates i_d and i_q to their references.
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

    ctl_vector2_t vdq_ctrl_out; //!< DQ controller result before feedforward and limiting.
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

#ifndef ENABLE_FOC_LADRC_CTRL
    ctl_dq_pi_t idq_ctrl; //!< DQ-PI current controller (default).
#else
    ctl_dq_ladrc1_t idq_ctrl; //!< DQ-LADRC1 current controller.
#endif
    ctrl_lead_t lead_compensator[2]; //!< output lead compensator

    //
    // --- Feed-forward & Parameters ---
    //
    ctrl_gt coef_ff_decouple[2]; //!< CTRL: current feed-foreword
    ctrl_gt max_vs_mag;          //!< Circular output voltage limit.
    ctrl_gt max_vs_mag_sq;       //!< Cached squared circular limit.
    ctrl_gt max_vs_rect;         //!< Per-axis rectangular output voltage limit.
    ctrl_gt max_dcbus_voltage;   //!< output voltage under 1 pu Vbus, default is 1.732 when Vbus = Vbase.

    // --- State ---
    fast_gt flag_enable_current_ctrl; //!< Enables or disables closed-loop current control.
    fast_gt
        flag_enable_theta_calc; //!< Flag to enable theta -> phasor calculation. If this flag is disabled using phasor directly.
    fast_gt flag_enable_lead_compensator; //!< Enables the output lead compensator.
    fast_gt flag_enable_decouple;         //!< Enable decoupling
    fast_gt flag_enable_bus_compensation; //!< Enable V bus compensation
    fast_gt flag_enable_vdq_feedforward;  //!<  Enable vdq feed forward

} mc_foc_core_t;

//================================================================================
// Function Prototypes & Definitions
//================================================================================

/**
 * @brief Initializes the current controller structure to safe defaults.
 * @param[out] mc Pointer to the current controller structure.
 */
GMP_STATIC_INLINE void ctl_clear_foc_core(mc_foc_core_t* mc)
{
    // 1. Clear Filters
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_U]);
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_V]);
    ctl_clear_filter_iir1(&mc->filter_iuvw[phase_W]);
    ctl_clear_filter_iir1(&mc->filter_udc);

    // 2. Clear Controllers
#ifndef ENABLE_FOC_LADRC_CTRL
    ctl_clear_dq_pi(&mc->idq_ctrl);
#else
    ctl_clear_dq_ladrc1(&mc->idq_ctrl);
#endif

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

// Use the converter maximum output voltage, Udc/SQRT(3), as the voltage base.
// Use the converter maximum permitted output current as the current base.

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

    // the following parameters would be calculated by LADRC1 auto-tuning.
    parameter_gt ladrc_b0d;
    parameter_gt ladrc_fcd;
    parameter_gt ladrc_fod;
    parameter_gt ladrc_b0q;
    parameter_gt ladrc_fcq;
    parameter_gt ladrc_foq;

} mc_foc_init_t;

/**
 * @brief Auto-tuning motor driver parameters.
 * @param[in,out] init Pointer to the `mtr_current_init_t` structure.
 */
void ctl_auto_tuning_foc_core(mc_foc_init_t* init);
void ctl_auto_tuning_foc_core_pi(mc_foc_init_t* init);
void ctl_auto_tuning_foc_core_ladrc1(mc_foc_init_t* init);

/**
 * @brief Initializes the compile-time selected d-q current controller.
 * @param[out] mc Pointer to the current controller structure.
 * @param[in]  init initialize object for Motor controller.
 */
void ctl_init_foc_core(mc_foc_core_t* mc, mc_foc_init_t* init);
#ifndef ENABLE_FOC_LADRC_CTRL
void ctl_init_foc_core_pi(mc_foc_core_t* mc, const mc_foc_init_t* init);
#else
void ctl_init_foc_core_ladrc1(mc_foc_core_t* mc, const mc_foc_init_t* init);
#endif

/**
 * @brief Initializes the basic motor current controller (FOC Core).
 * @details Sets up IIR filters for currents and bus voltage, initializes the selected DQ controller,
 * for d/q axes, configures lead compensators, sets voltage limits, and establishes safe 
 * default execution flags.
 * * @param[out] mc        Pointer to the motor current controller instance.
 * @param[in]  kp        PI proportional gain, or LADRC b0 when LADRC is selected.
 * @param[in]  ki        PI integral gain, or LADRC fc when LADRC is selected.
 * @param[in]  max_vs_pu Maximum stator voltage magnitude in per-unit (e.g., 0.577f for SVPWM).
 * @param[in]  fs        Sampling/execution frequency of the control loop (Hz).
 */
void ctl_init_foc_core_basic(mc_foc_core_t* mc, parameter_gt kp, parameter_gt ki, parameter_gt max_vs_pu,
                                     parameter_gt fs);

/**
 * @brief Sets the output voltage saturation limits for the FOC controller.
 * @details Configures both circular and rectangular voltage limits. The circular limit
 * restricts the overall maximum voltage vector magnitude, while the rectangular limit
 * bounds the individual d/q axis controller outputs. PI integral limits are scaled
 * to 80% of the rectangular limit.
 * @param[in,out] mc                   Pointer to the motor current controller instance.
 * @param[in]     volt_rect_saturation Rectangular saturation limit for d/q axes in per-unit (pu).
 * @param[in]     volt_cir_saturation  Circular saturation limit for voltage vector magnitude in per-unit (pu).
 */
void ctl_set_foc_core_saturation(mc_foc_core_t* mc, parameter_gt volt_rect_saturation, parameter_gt volt_cir_saturation);

/**
 * @brief Executes one step of the FOC current control loop.
 * @param[out] mc      Pointer to the current controller structure.
 * @param[in]  theta   The current electrical angle of the rotor (0.0 to 1.0).
 */
GMP_STATIC_INLINE void ctl_step_foc_core(mc_foc_core_t* mc)
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
        gmp_ctl_assert(mc->phasor_input);
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

    // 4. Compose decoupling and user feedforward before the DQ controller.
    ctl_vector2_clear(&mc->vdq_decouple);
    if (mc->flag_enable_decouple)
    {
        mc->vdq_decouple.dat[phase_d] =
            -ctl_mul(mc->spd_if->speed, ctl_mul(mc->coef_ff_decouple[phase_d], mc->idq0.dat[phase_q]));
        mc->vdq_decouple.dat[phase_q] =
            ctl_mul(mc->spd_if->speed, ctl_mul(mc->coef_ff_decouple[phase_q], mc->idq0.dat[phase_d]));
    }

    ctl_vector2_t total_ff;
    ctl_vector2_copy(&total_ff, &mc->vdq_decouple);
    if (mc->flag_enable_vdq_feedforward)
        ctl_vector2_add(&total_ff, &total_ff, &mc->vdq_ff);

    // 5. Closed-loop control. The DQ module owns feedforward, vector limits,
    // and anti-windup/observer applied-command feedback as one atomic pipeline.
    if (mc->flag_enable_current_ctrl)
    {
#ifndef ENABLE_FOC_LADRC_CTRL
        ctl_step_dq_pi(&mc->idq_ctrl, &mc->idq_ref, (ctl_vector2_t*)&mc->idq0, &total_ff, &mc->vdq_ref);
        ctl_vector2_copy(&mc->vdq_ctrl_out, &mc->idq_ctrl.ctrl_out);
#else
        ctl_step_dq_ladrc1(&mc->idq_ctrl, &mc->idq_ref, (ctl_vector2_t*)&mc->idq0, &total_ff, &mc->vdq_ref);
        ctl_vector2_copy(&mc->vdq_ctrl_out, &mc->idq_ctrl.ctrl_out);
#endif
    }
    else
    {
        // Open-loop voltage command keeps the same feedforward semantics.
        ctl_vector2_add((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref, &total_ff);
        ctl_vector2_sat_circle_sq((ctl_vector2_t*)&mc->vdq_out, (ctl_vector2_t*)&mc->vdq_out,
                                  mc->max_vs_mag_sq);
        ctl_vector2_t limit_max = {{mc->max_vs_rect, mc->max_vs_rect}};
        ctl_vector2_t limit_min = {{-mc->max_vs_rect, -mc->max_vs_rect}};
        ctl_vector2_sat_rect((ctl_vector2_t*)&mc->vdq_out, (ctl_vector2_t*)&mc->vdq_out, &limit_max, &limit_min);
        ctl_vector2_copy(&mc->vdq_ref, (ctl_vector2_t*)&mc->vdq_out);
    }

    // 6. Optional phase-lead and bus-voltage compensation.
    if (mc->flag_enable_lead_compensator)
    {
        mc->vdq_out.dat[phase_d] = ctl_step_lead(&mc->lead_compensator[phase_d], mc->vdq_ref.dat[phase_d]);
        mc->vdq_out.dat[phase_q] = ctl_step_lead(&mc->lead_compensator[phase_q], mc->vdq_ref.dat[phase_q]);
    }
    else
        ctl_vector2_copy((ctl_vector2_t*)&mc->vdq_out, &mc->vdq_ref);

    ctrl_gt v_scale = CTL_CTRL_CONST_1;
    if (mc->flag_enable_bus_compensation)
    {
        if (mc->udc > CTL_CTRL_CONST_1_OVER_2)
            v_scale = mc->max_dcbus_voltage / mc->udc;
        else
            v_scale = mc->max_dcbus_voltage;
    }
    mc->vdq_out_bus_compensator.dat[phase_d] = ctl_mul(mc->vdq_out.dat[phase_d], v_scale);
    mc->vdq_out_bus_compensator.dat[phase_q] = ctl_mul(mc->vdq_out.dat[phase_q], v_scale);

    // Guard the actual modulator command as well (notably after bus compensation).
    ctl_vector2_copy(&mc->vdq_out_sat, &mc->vdq_out_bus_compensator);
    ctl_vector2_sat_circle_sq(&mc->vdq_out_sat, &mc->vdq_out_sat, mc->max_vs_mag_sq);
    ctl_vector2_t limit_max = {{mc->max_vs_rect, mc->max_vs_rect}};
    ctl_vector2_t limit_min = {{-mc->max_vs_rect, -mc->max_vs_rect}};
    ctl_vector2_sat_rect(&mc->vdq_out_sat, &mc->vdq_out_sat, &limit_max, &limit_min);

    mc->vdq_out.dat[phase_d] = mc->vdq_out_sat.dat[phase_d];
    mc->vdq_out.dat[phase_q] = mc->vdq_out_sat.dat[phase_q];
    mc->vdq_out.dat[phase_0] = CTL_CTRL_CONST_ZERO;

    // 7. iPark: d-q -> alpha-beta, using the command actually applied.
    ctl_ct_ipark(&mc->vdq_out, &mc->phasor, &mc->vab0);
}

/**
 * @brief Sets the d-q axis current reference (target).
 * @param[out] cc Pointer to the current controller structure.
 * @param[in]  id_ref The target d-axis current.
 * @param[in]  iq_ref The target q-axis current.
 */
GMP_STATIC_INLINE void ctl_set_foc_core_idq_ref(mc_foc_core_t* mc, ctrl_gt id_ref, ctrl_gt iq_ref)
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
GMP_STATIC_INLINE void ctl_set_foc_core_vdq_ref(mc_foc_core_t* mc, ctrl_gt vd_ff, ctrl_gt vq_ff)
{
    mc->vdq_ref.dat[0] = vd_ff;
    mc->vdq_ref.dat[1] = vq_ff;
}

/**
 * @brief Enables closed-loop DQ current control.
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_enable_foc_core_current_ctrl(mc_foc_core_t* mc)
{
    mc->flag_enable_current_ctrl = 1;
}

/**
 * @brief Disables closed-loop DQ current control.
 * @details When disabled, explicit voltage
 * commands or feedforward terms (if enabled) will still be applied to the plant.
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_disable_foc_core_current_ctrl(mc_foc_core_t* mc)
{
    mc->flag_enable_current_ctrl = 0;
}

/**
 * @brief Attaches the primary hardware and sensor interfaces to the FOC core.
 * @details Binds the physical ADC channels for phase currents and bus voltage, 
 * as well as the position and velocity feedback interfaces.
 * @param[in,out] mc       Pointer to the FOC core structure.
 * @param[in]     _iabc    Pointer to the 3-phase current ADC interface.
 * @param[in]     _udc     Pointer to the DC bus voltage ADC interface.
 * @param[in]     _pos_if  Pointer to the rotor position interface.
 * @param[in]     _vec_if  Pointer to the rotor velocity interface.
 */
GMP_STATIC_INLINE void ctl_attach_foc_core_port(mc_foc_core_t* mc, tri_adc_ift* _iabc, adc_ift* _udc,
                                                rotation_ift* _pos_if, velocity_ift* _vec_if)
{
    mc->adc_iuvw = _iabc;
    mc->adc_udc = _udc;
    mc->pos_if = _pos_if;
    mc->spd_if = _vec_if;
}

/**
 * @brief Dynamically attaches or updates the position encoder interface.
 * @details Highly useful for seamless (bumpless) transitions between open-loop V/F, 
 * sensorless observers (SMO), and physical encoders on the fly.
 * @param[in,out] mc       Pointer to the FOC core structure.
 * @param[in]     _pos_if  Pointer to the new rotor position interface.
 */
GMP_STATIC_INLINE void ctl_attach_foc_core_pos_enc(mc_foc_core_t* mc, rotation_ift* _pos_if)
{
    mc->pos_if = _pos_if;
}

/**
 * @brief Attaches an external phasor (sine/cosine vector) input to the FOC core.
 * @details Allows bypassing standard trigonometric (sin/cos) calculations by 
 * providing pre-calculated phasor values, optimizing execution speed.
 * @param[in,out] mc       Pointer to the FOC core structure.
 * @param[in]     _phasor  Pointer to the external phasor structure (vector2).
 */
GMP_STATIC_INLINE void ctl_attach_foc_core_phasor(mc_foc_core_t* mc, ctl_vector2_t* _phasor)
{
    mc->phasor_input = _phasor;
}

/**
 * @brief Disables the d-q axis cross-coupling voltage compensation.
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_disable_foc_core_decouple(mc_foc_core_t* mc)
{
    mc->flag_enable_decouple = 0;
}

/**
 * @brief Enables the d-q axis cross-coupling voltage compensation.
 * @details Improves current control dynamic response at high speeds by actively cancelling 
 * out the cross-coupling back-EMF terms (w * Lq * Iq and -w * Ld * Id).
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_enable_foc_core_decouple(mc_foc_core_t* mc)
{
    mc->flag_enable_decouple = 1;
}

/**
 * @brief Disables the Vd/Vq voltage feedforward control.
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_disable_foc_core_vdq_ff(mc_foc_core_t* mc)
{
    mc->flag_enable_vdq_feedforward = 0;
}

/**
 * @brief Enables the Vd/Vq voltage feedforward control.
 * @details Adds pre-calculated open-loop feedforward voltages directly to the 
 * DQ controller outputs. Essential for high-performance dynamic tracking.
 * @param[in,out] mc Pointer to the FOC core structure.
 */
GMP_STATIC_INLINE void ctl_enable_foc_core_vdq_ff(mc_foc_core_t* mc)
{
    mc->flag_enable_vdq_feedforward = 1;
}

/** 
 *@} 
 */ // end of CURRENT_CONTROLLER group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_CURRENT_CTRL_H_
