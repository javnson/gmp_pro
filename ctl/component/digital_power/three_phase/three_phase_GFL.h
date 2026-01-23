/**
 * @file three_phase_GFL.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset three-phase DC/AC grid following inverter controller.
 * @version 1.0
 * @date 2026-01-11
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TOPOLOGY_GFL_INV_H_API Three-Phase GFL Inverter Topology API (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */

#ifndef _FILE_THREE_PHASE_GFL_
#define _FILE_THREE_PHASE_GFL_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/math_block/coordinate/coord_trans.h>

#include <ctl/component/intrinsic/basic/saturation.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/lead_lag.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

#include <ctl/component/digital_power/three_phase/pll.h>

// --- Compilation-time Configuration Macros ---
#ifndef GFL_CURRENT_SAMPLE_PHASE_MODE
/**
 * @brief Configures the current sampling method.
 * - **3**: Sample all three phase currents (Ia, Ib, Ic).
 * - **2**: Sample two phase currents (Ia, Ib), and calculate the third.
 */
#define GFL_CURRENT_SAMPLE_PHASE_MODE (3)
#endif // GFL_CURRENT_SAMPLE_PHASE_MODE

#ifndef GFL_VOLTAGE_SAMPLE_PHASE_MODE
/**
 * @brief Configures the voltage sampling method.
 * - **3**: Sample all three phase voltages (Va, Vb, Vc).
 * - **2**: Sample two phase voltages (Va, Vb), and calculate the third.
 * - **1**: Sample two line-to-line voltages (Vab, Vbc).
 */
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (3)
#endif // GFL_VOLTAGE_SAMPLE_PHASE_MODE

#ifndef GFL_CAPACITOR_CURRENT_CALCULATE_MODE
/**
 * @brief Configures the voltage sampling method.
 * - **4**: TODO calculate Ic based on Uuvw and Iabc
 * - **3**: Capacitor Voltage is measured, calculate Ic = C d/dt(Vc)
 * - **2**: Sample inverter current Iuvw and grid current Iabc, calculate Ic = Iuvw - Iabc.
 * - **1**: Sample Capacitor current directly.
 */
#define GFL_CAPACITOR_CURRENT_CALCULATE_MODE (3)
#endif // GFL_VOLTAGE_SAMPLE_PHASE_MODE

/**
 * @brief Main data structure for the three-phase inverter controller.
 */
typedef struct _tag_gfl_inv_ctrl_type
{

    uint32_t isr_tick; //!< Main ISR tick counter.

    //
    // --- Input Ports (ADC Interfaces) ---
    //
    adc_ift* adc_udc; //!< DC Bus voltage.
    adc_ift* adc_idc; //!< DC Bus current.

    ctl_vector2_t* phasor_ext; //!< input a phasor for park and ipark

    // Grid side feedback
    tri_adc_ift* adc_vabc; //!< grid phase voltage ADCs {Va, Vb, Vc}, this voltage will use as pll input.
    tri_adc_ift* adc_iabc; //!< grid phase current ADCs {Ia, Ib, Ic}, this current will use as current control port.

    //
    // --- Output Ports ---
    //
    ctl_vector3_t vab0_out; //!< RO: Total modulation voltage in alpha-beta frame.

    //
    // --- Setpoints & Intermediate Variables (Read/Write) ---
    //
    ctl_vector2_t idq_set; //!< R/W: Current command in d-q frame (for current mode).
    ctl_vector2_t vdq_out; //!< R/W: Output of positive-sequence current controller.

    ctl_vector2_t vdq_ff_external;  //!< W/R: vdq feed forward from external.
    ctl_vector3_t vab0_ff_external; //!< W/R: v alpha beta feed forward form external

    //
    // --- Measurement & Internal State Variables (Read-Only) ---
    //
    ctrl_gt angle;        //!< RO: Estimated grid angle.
    ctl_vector2_t phasor; //!< RO: Phasor {cos, sin} corresponding to the grid angle.

    ctl_vector3_t iabc; //!< RO: grid current Filtered three-phase currents.
    ctl_vector3_t iab0; //!< RO: grid current Clarke transformed currents {alpha, beta, zero}.
    ctl_vector2_t idq;  //!< RO: Park transformed currents {d, q}.

    ctl_vector3_t vabc; //!< RO: Filtered three-phase voltages.
    ctl_vector3_t vab0; //!< RO: Clarke transformed voltages {alpha, beta, zero}.
    ctl_vector2_t vdq;  //!< RO: Park transformed voltages {d, q}.

    ctl_vector2_t vdq_ff_decouple; //!< RO: vdq feed forward of decoupling.
    ctl_vector2_t vdq_ff_damping;  //!< RO: vdq damping feed forward.

    ctl_vector2_t vdq_out_comp; //!< RO: output result after output compensator.
    ctl_vector2_t vab_pos;      //!< RO: Positive-sequence modulation voltage in alpha-beta frame.

    //
    // --- Controller Objects ---
    //

    // input filter
    ctl_filter_IIR1_t filter_udc;     //!< CTRL: DC bus current input filter
    ctl_filter_IIR1_t filter_idc;     //!< CTRL: DC bus voltage input filter
    ctl_filter_IIR1_t filter_iabc[3]; //!< CTRL: current input filter
    ctl_filter_IIR1_t filter_uabc[3]; //!< CTRL: voltage input filter

    // current controller
    ctl_pid_t pid_idq[2];     //!< CTRL: idq PID controller
    ctrl_gt coef_ff_decouple; //!< CTRL: current feed-foreword

    // active damping
    ctl_filter_IIR2_t filter_damping[2]; //!< CTRL: active capacitor damping
    ctrl_gt coef_ff_damping;          //!< damping gain
    vector2_gt vdq_last;              //!< (vdq - vdq_last) to calculate differential

    // output lead compensator
    ctrl_lead_t lead_compensator[2];

    // PLL & RG
    srf_pll_t pll;           //!< Three-phase PLL for grid synchronization.
    ctl_ramp_generator_t rg; //!< Ramp generator for open-loop/free-run operation.

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable_system;           //!< Master enable for the entire controller.
    fast_gt flag_enable_pll;              //!< Enables the three-phase PLL.
    fast_gt flag_enable_offgrid;          //!< 1 to use ramp generator angle (free run), 0 to use PLL angle.
    fast_gt flag_enable_current_ctrl;     //!< Enables the inner current control loop.
    fast_gt flag_enable_decouple;         //!< Enables inductor decoupling feed-forward.
    fast_gt flag_enable_active_damping;   //!< Enables the virtual resistance of capacitor.
    fast_gt flag_enable_lead_compensator; //!< Enables the output lead compensator.
    fast_gt flag_enable_external_phasor;  //!< Enables the external phasor input.

} gfl_inv_ctrl_t;

/**
 * @brief Clear GFL inverter parameters, without PLL.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] inv Pointer to the gfl_inv_ctrl_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_gfl_inv(gfl_inv_ctrl_t* inv)
{
    int i = 0;

    for (i = 0; i < 3; ++i)
    {
        ctl_clear_filter_iir1(&inv->filter_iabc[i]);
        ctl_clear_filter_iir1(&inv->filter_uabc[i]);
    }

    ctl_clear_filter_iir1(&inv->filter_idc);
    ctl_clear_filter_iir1(&inv->filter_udc);

    ctl_clear_pid(&inv->pid_idq[phase_d]);
    ctl_clear_pid(&inv->pid_idq[phase_q]);

    ctl_clear_biquad_filter(&inv->filter_damping[phase_d]);
    ctl_clear_biquad_filter(&inv->filter_damping[phase_q]);

    ctl_vector2_clear(&inv->vdq_last);

    ctl_clear_lead(&inv->lead_compensator[phase_d]);
    ctl_clear_lead(&inv->lead_compensator[phase_q]);

    // TODO: clear intermediate variables
    ctl_vector2_clear(&inv->vdq_ff_external);
    ctl_vector3_clear(&inv->vab0_ff_external);
}

/**
 * @brief Auto-tuning GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] inv Pointer to the gfl_inv_ctrl_t structure.
 */
GMP_STATIC_INLINE void ctl_clear_gfl_inv_with_PLL(gfl_inv_ctrl_t* inv)
{
    ctl_clear_gfl_inv(inv);
    ctl_clear_pll_3ph(&inv->pll);

    inv->flag_enable_system = 0;
}

/**
 * @brief Main data structure for the three-phase inverter controller.
 */
typedef struct _tag_gfl_inv_ctrl_init
{
    // [fatal] the following information is key parameter for auto-tuning.
    parameter_gt fs;        //!< Controller execution frequency (Hz).
    parameter_gt v_base;    //!< Base voltage for per-unit conversion (V).
    parameter_gt i_base;    //!< Base current for per-unit conversion (A).
    parameter_gt freq_base; //!< Nominal grid frequency (e.g., 50 or 60 Hz).

    // [fatal] the following information is key parameter for auto-tuning.
    parameter_gt grid_filter_L; //!< Grid filter inductor parameters
    parameter_gt grid_filter_C; //!< Grid filter inductor parameters

    // the following parameters would be calculated by auto-tuning
    parameter_gt current_adc_fc; //!< Current ADC filter cut frequency (Hz).
    parameter_gt voltage_adc_fc; //!< Voltage ADC filter cut frequency (Hz).

    // the following parameters would be calculated by auto-tuning.
    parameter_gt current_loop_bw;   //!< Current loop bandwidth frequency (Hz).
    parameter_gt current_loop_zero; // !< Current loop Zero frequency (Hz).
    parameter_gt current_phase_lag; //!< Current loop output compensate angle (rad).

    // the following parameters would be calculated by auto-tuning.
    parameter_gt active_damping_resister;    //!< active resister, Ohm.
    parameter_gt active_damping_center_freq; //!< active damping center frequency, Hz.
    parameter_gt active_damping_filter_q;    //!< active damping filter Q

    parameter_gt kp_pll; //!< PLL controller parameters
    parameter_gt ki_pll; //!< PLL controller parameters

} gfl_inv_ctrl_init_t;

/**
 * @brief Auto-tuning GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_auto_tuning_gfl_inv(gfl_inv_ctrl_init_t* init);

/**
 * @brief update GFL inverter controller parameters by init structure.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_update_gfl_inv_coeff(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init);

/**
 * @brief Init GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_init_gfl_inv(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init);

/**
 * @brief Attach GFL inverter with input/output interface.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[in,out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] adc_udc Pointer to the udc ADC interface structure.
 * @param[in] adc_idc Pointer to the idc ADC interface structure.
 * @param[in] adc_iabc Pointer to the tri-channel ADC current interface structure.
 * @param[in] adc_vabc Pointer to the tri-channel ADC voltage interface structure.
 */
void ctl_attach_gfl_inv(gfl_inv_ctrl_t* inv, adc_ift* adc_idc, adc_ift* adc_udc, tri_adc_ift* adc_iabc,
                        tri_adc_ift* adc_vabc);

/**
 * @brief Executes one step of the  GFL three-phase inverter control algorithm.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] ctrl Pointer to the `inv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_step_gfl_inv_ctrl(gfl_inv_ctrl_t* gfl)
{
    // assert critical pointer
    gmp_base_assert(gfl);
    gmp_base_assert(gfl->adc_iabc);
    gmp_base_assert(gfl->adc_vabc);

    gfl->isr_tick += 1;

    // --- 1. Input Filtering and Coordinate Transformation ---
    ctl_step_filter_iir1(&gfl->filter_udc, gfl->adc_udc->value);
    ctl_step_filter_iir1(&gfl->filter_idc, gfl->adc_idc->value);

    // current sensor
#if GFL_CURRENT_SAMPLE_PHASE_MODE == 3
    gfl->iabc.dat[phase_A] = ctl_step_filter_iir1(&gfl->filter_iabc[phase_A], gfl->adc_iabc->value.dat[phase_A]);
    gfl->iabc.dat[phase_B] = ctl_step_filter_iir1(&gfl->filter_iabc[phase_B], gfl->adc_iabc->value.dat[phase_B]);
    gfl->iabc.dat[phase_C] = ctl_step_filter_iir1(&gfl->filter_iabc[phase_C], gfl->adc_iabc->value.dat[phase_C]);

    ctl_ct_clarke(&gfl->iabc, &gfl->iab0);

#elif GFL_CURRENT_SAMPLE_PHASE_MODE == 2
    gfl->iabc.dat[phase_A] = ctl_step_filter_iir1(&gfl->filter_iabc[phase_A], gfl->adc_iabc->value.dat[phase_A]);
    gfl->iabc.dat[phase_B] = ctl_step_filter_iir1(&gfl->filter_iabc[phase_B], gfl->adc_iabc->value.dat[phase_B]);
    gfl->iabc.dat[phase_C] = 0;

    ctl_ct_clarke_2ph((ctl_vector2_t*)&gfl->iabc, (ctl_vector2_t*)&gfl->iab0);
    gfl->iab0.dat[phase_0] = 0;

#endif // GFL_CURRENT_SAMPLE_PHASE_MODE

    // voltage sensor
#if GFL_VOLTAGE_SAMPLE_PHASE_MODE == 3
    gfl->vabc.dat[phase_A] = ctl_step_filter_iir1(&gfl->filter_uabc[phase_A], gfl->adc_vabc->value.dat[phase_A]);
    gfl->vabc.dat[phase_B] = ctl_step_filter_iir1(&gfl->filter_uabc[phase_B], gfl->adc_vabc->value.dat[phase_B]);
    gfl->vabc.dat[phase_C] = ctl_step_filter_iir1(&gfl->filter_uabc[phase_C], gfl->adc_vabc->value.dat[phase_C]);

    ctl_ct_clarke(&gfl->vabc, &gfl->vab0);

#elif GFL_VOLTAGE_SAMPLE_PHASE_MODE == 2
    gfl->vabc.dat[phase_A] = ctl_step_filter_iir1(&gfl->lpf_vabc[phase_A], gfl->adc_vabc->value.dat[phase_A]);
    gfl->vabc.dat[phase_B] = ctl_step_filter_iir1(&gfl->lpf_vabc[phase_B], gfl->adc_vabc->value.dat[phase_B]);
    gfl->vabc.dat[phase_C] = 0;

    ctl_ct_clarke_2ph((ctl_vector2_t*)&gfl->vabc, (ctl_vector2_t*)&gfl->vab0);
    gfl->vab0.dat[phase_0] = 0;

#elif GFL_VOLTAGE_SAMPLE_PHASE_MODE == 1
    gfl->vabc.dat[phase_UAB] = ctl_step_filter_iir1(&gfl->lpf_vabc[phase_UAB], gfl->adc_vabc->value.dat[phase_UAB]);
    gfl->vabc.dat[phase_UBC] = ctl_step_filter_iir1(&gfl->lpf_vabc[phase_UBC], gfl->adc_vabc->value.dat[phase_UBC]);
    gfl->vabc.dat[phase_0] = 0;
    ctl_ct_clarke_from_line((ctl_vector2_t*)&gfl->vabc, (ctl_vector2_t*)&gfl->vab0);
    gfl->vab0.dat[phase_0] = 0;

#endif // GFL_VOLTAGE_SAMPLE_PHASE_MODE

    // --- 2. Grid Synchronization (PLL) ---
    if (gfl->flag_enable_pll)
        ctl_step_pll_3ph(&gfl->pll, gfl->vab0.dat[phase_alpha], gfl->vab0.dat[phase_beta]);

    // --- 3. Main Control Logic ---
    if (gfl->flag_enable_system)
    {
        // --- 3a. Angle and Phasor Generation ---
        if (gfl->flag_enable_offgrid)
        {
            gfl->angle = ctl_step_ramp_generator(&gfl->rg);
            ctl_set_phasor_via_angle(gfl->angle, &gfl->phasor);
        }
        // using external phasor
        else if (gfl->flag_enable_external_phasor)
        {
            gmp_base_assert(gfl->phasor_ext);
            ctl_vector2_copy(&gfl->phasor, gfl->phasor_ext);
        }
        // using PLL phasor
        else
        {
            gfl->angle = gfl->pll.theta;
            ctl_vector2_copy(&gfl->phasor, &gfl->pll.phasor);
        }

        // --- 3b. Park Transformation ---
        ctl_ct_park2((ctl_vector2_t*)&gfl->iab0, &gfl->phasor, &gfl->idq);
        ctl_ct_park2((ctl_vector2_t*)&gfl->vab0, &gfl->phasor, &gfl->vdq);

        // --- 3c. current controller ---
        // if current controller is disabled the vdq_out would output directly.
        if (gfl->flag_enable_current_ctrl)
        {
            gfl->vdq_out.dat[phase_d] =
                ctl_step_pid_ser(&gfl->pid_idq[phase_d], gfl->idq_set.dat[phase_d] - gfl->idq.dat[phase_d]);
            gfl->vdq_out.dat[phase_q] =
                ctl_step_pid_ser(&gfl->pid_idq[phase_q], gfl->idq_set.dat[phase_q] - gfl->idq.dat[phase_q]);

            // decouple
            if (gfl->flag_enable_decouple)
            {
                gfl->vdq_ff_decouple.dat[phase_d] = gfl->coef_ff_decouple * gfl->idq.dat[phase_q];
                gfl->vdq_ff_decouple.dat[phase_q] = -gfl->coef_ff_decouple * gfl->idq.dat[phase_d];
            }
            else
            {
                gfl->vdq_ff_decouple.dat[phase_d] = 0;
                gfl->vdq_ff_decouple.dat[phase_q] = 0;
            }

            // active damping
            if (gfl->flag_enable_active_damping)
            {
                // 计算微分量 (代表电容电流 trend)
                    ctrl_gt diff_d = gfl->vdq.dat[phase_d] - gfl->vdq_last.dat[phase_d];
                    ctrl_gt diff_q = gfl->vdq.dat[phase_q] - gfl->vdq_last.dat[phase_q];

                // damping filter
                gfl->vdq_ff_damping.dat[phase_d] =
                        ctl_step_biquad_filter(&gfl->filter_damping[phase_d],
                                               gfl->coef_ff_damping * diff_d);
                gfl->vdq_ff_damping.dat[phase_q] =
                        ctl_step_biquad_filter(&gfl->filter_damping[phase_q],
                                               gfl->coef_ff_damping * diff_q);

                ctl_vector2_copy(&gfl->vdq_last, &gfl->vdq);
            }
            else
            {
                gfl->vdq_ff_damping.dat[phase_d] = 0;
                gfl->vdq_ff_damping.dat[phase_q] = 0;
            }

            // mix all vdq up
            gfl->vdq_out.dat[phase_d] += gfl->vdq_ff_decouple.dat[phase_d] + gfl->vdq_ff_damping.dat[phase_d];
            gfl->vdq_out.dat[phase_q] += gfl->vdq_ff_decouple.dat[phase_q] + gfl->vdq_ff_damping.dat[phase_q];
        }

        // --- 3d. lead compensator ---
        if (gfl->flag_enable_lead_compensator)
        {
            gfl->vdq_out_comp.dat[phase_d] = ctl_step_lead(&gfl->lead_compensator[phase_d], gfl->vdq_out.dat[phase_d]);
            gfl->vdq_out_comp.dat[phase_q] = ctl_step_lead(&gfl->lead_compensator[phase_q], gfl->vdq_out.dat[phase_q]);
        }
        else
        {
            ctl_vector2_copy(&gfl->vdq_out_comp, &gfl->vdq_out);
        }

        // --- 3e. output inverse park Transformation ---
        ctl_ct_ipark2(&gfl->vdq_out_comp, &gfl->phasor, &gfl->vab_pos);

        gfl->vab0_out.dat[phase_alpha] = gfl->vab_pos.dat[phase_alpha] + gfl->vab0_ff_external.dat[phase_alpha];
        gfl->vab0_out.dat[phase_beta] = gfl->vab_pos.dat[phase_beta] + gfl->vab0_ff_external.dat[phase_beta];
        gfl->vab0_out.dat[phase_0] = gfl->vab0_ff_external.dat[phase_0];
    }
    else
    {
        gfl->vab0_out.dat[phase_alpha] = 0;
        gfl->vab0_out.dat[phase_beta] = 0;
        gfl->vab0_out.dat[phase_0] = 0;
    }
}

//////////////////////////////////////////////////////////////////////////
// Mode Setting Functions
//////////////////////////////////////////////////////////////////////////

/** @brief Sets the controller to open-loop mode. */
GMP_STATIC_INLINE void ctl_set_gfl_inv_openloop_mode(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_system = 0;
    inv->flag_enable_pll = 0;
    inv->flag_enable_offgrid = 1;
    inv->flag_enable_current_ctrl = 0;
    inv->flag_enable_decouple = 0;
    inv->flag_enable_active_damping = 0;
    inv->flag_enable_lead_compensator = 0;
}

/** @brief Sets the open-loop output voltage in the d-q frame. */
GMP_STATIC_INLINE void ctl_set_gfl_inv_voltage_openloop(gfl_inv_ctrl_t* inv, ctrl_gt vd, ctrl_gt vq)
{
    inv->vdq_out.dat[phase_d] = vd;
    inv->vdq_out.dat[phase_q] = vq;
}

/** @brief Sets the controller to current closed-loop mode. */
GMP_STATIC_INLINE void ctl_set_gfl_inv_current_mode(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_system = 0;
    inv->flag_enable_pll = 0;
    inv->flag_enable_offgrid = 1;
    inv->flag_enable_current_ctrl = 1;
    inv->flag_enable_decouple = 0;
    inv->flag_enable_active_damping = 0;
    inv->flag_enable_lead_compensator = 0;
}

/** @brief Sets the target current in the d-q frame. */
GMP_STATIC_INLINE void ctl_set_gfl_inv_current(gfl_inv_ctrl_t* inv, ctrl_gt id, ctrl_gt iq)
{
    inv->idq_set.dat[phase_d] = id;
    inv->idq_set.dat[phase_q] = iq;
}

//////////////////////////////////////////////////////////////////////////
// Utility Functions
//////////////////////////////////////////////////////////////////////////

/** @brief Calculates the output active power (P). */
GMP_STATIC_INLINE ctrl_gt ctl_get_gfl_inv_Pout(gfl_inv_ctrl_t* inv)
{
    return ctl_add(ctl_mul(inv->vdq.dat[phase_d], inv->idq.dat[phase_d]),
                   ctl_mul(inv->vdq.dat[phase_q], inv->idq.dat[phase_q]));
}

/** @brief Calculates the output reactive power (Q). */
GMP_STATIC_INLINE ctrl_gt ctl_get_gfl_inv_Qout(gfl_inv_ctrl_t* inv)
{
    return ctl_sub(ctl_mul(inv->vdq.dat[phase_q], inv->idq.dat[phase_d]),
                   ctl_mul(inv->vdq.dat[phase_d], inv->idq.dat[phase_q]));
}

/** @brief Sets the angle source to the internal ramp generator (freerun). */
GMP_STATIC_INLINE void ctl_set_gfl_inv_freerun(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_offgrid = 1;
}

/** @brief Sets the angle source to the PLL (grid-tied). */
GMP_STATIC_INLINE void ctl_set_gfl_inv_grid_connect(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_offgrid = 0;
}

/** @brief Enable inverter decoupling */
GMP_STATIC_INLINE void ctl_enable_gfl_inv_decouple(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_decouple = 0;
}

/** @brief Enable inverter active damping */
GMP_STATIC_INLINE void ctl_enable_gfl_inv_active_damp(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_active_damping = 0;
}

/** @brief Enable inverter lead compensator */
GMP_STATIC_INLINE void ctl_enable_gfl_inv_lead_compensator(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_active_damping = 0;
}

/** @brief Get PLL error */
GMP_STATIC_INLINE ctrl_gt ctl_get_gfl_pll_error(gfl_inv_ctrl_t* inv)
{
    return inv->pll.e_error;
}

/** @brief Enable GFL controller */
GMP_STATIC_INLINE void ctl_enable_gfl_inv(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_system = 1;
}

/** @brief Disable GFL controller */
GMP_STATIC_INLINE void ctl_disable_gfl_inv(gfl_inv_ctrl_t* inv)
{
    inv->flag_enable_system = 0;
}

//////////////////////////////////////////////////////////////////////////
// PQ controller
//

/**
 * @brief P-Q Grid-Following Power Controller.
 * * This controller sits on top of the Current Controller.
 * It regulates Active Power (P) and Reactive Power (Q) by adjusting 
 * the d-axis and q-axis current references.
 * * Topology:
 * P_ref ---(-)--> [PID_P] ----> Id_ref
 * Q_ref ---(-)--> [PID_Q] ----> Iq_ref
 */
typedef struct _tag_gfl_pq_ctrl
{
    //
    // --- Input Interfaces (Pointers) ---
    //
    ctl_vector2_t* vdq_meas; //!< PTR: Feedback grid voltage vector (d,q), output voltage is positive.
    ctl_vector2_t* idq_meas; //!< PTR: Feedback grid current vector (d,q), output current is positive.

    //
    // --- Output Interface ---
    //
    ctl_vector2_t idq_set_out; //!< RO: Calculated current command {Id*, Iq*} to be sent to inner loop.

    //
    // --- Setpoints (User Settings) ---
    //
    ctl_vector2_t pq_set; //!< WR: Power Setpoints {P_ref, Q_ref} in pu.

    //
    // --- Measurement & State (Read-Only) ---
    //
    ctl_vector2_t pq_meas; //!< RO: Calculated instantaneous Active/Reactive Power.
    ctrl_gt s_mag_sq;      //!< RO: Magnitude squared of apparent power (debugging).

    //
    // --- Controllers & Limits ---
    //
    ctl_pid_t pid_p; //!< CTRL: PID for Active Power. Output is Id_ref.
    ctl_pid_t pid_q; //!< CTRL: PID for Reactive Power. Output is Iq_ref.

    ctrl_gt max_i2_mag; //!< PARAM: Maximum allowable current magnitude square (current limit protection).

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable; //!< 1: Enable PQ control (Closed Loop), 0: Disable (Output 0 or Hold).

} gfl_pq_ctrl_t;

/**
 * @brief Initialize the PQ controller with parameters.
 * @param[out] pq Pointer to the PQ controller instance.
 * @param[in] init Initialization parameters.
 */
void ctl_init_gfl_pq(gfl_pq_ctrl_t* pq, parameter_gt p_kp, parameter_gt p_ki, parameter_gt q_kp, parameter_gt q_ki,
                     parameter_gt i_out_max, parameter_gt fs);

/**
 * @brief Reset the PQ controller (clear integrators).
 * @param[in,out] pq Pointer to the PQ controller instance.
 */
void ctl_clear_gfl_pq(gfl_pq_ctrl_t* pq);

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq(gfl_pq_ctrl_t* pq, ctl_vector2_t* vdq, ctl_vector2_t* idq);

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq_to_core(gfl_pq_ctrl_t* pq, gfl_inv_ctrl_t* core);

/**
 * @brief Execute one step of the PQ control loop.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @note This should run at a slower rate than the current loop (e.g., 1kHz - 5kHz).
 */
GMP_STATIC_INLINE void ctl_step_gfl_pq(gfl_pq_ctrl_t* pq)
{
    // Safety check for pointers
    if (!pq->vdq_meas || !pq->idq_meas)
        return;

    // Local variables for readability
    ctrl_gt vd = pq->vdq_meas->dat[phase_d];
    ctrl_gt vq = pq->vdq_meas->dat[phase_q];
    ctrl_gt id = pq->idq_meas->dat[phase_d];
    ctrl_gt iq = pq->idq_meas->dat[phase_q];

    // -----------------------------------------------------------
    // 1. Calculate Instantaneous Power (Per-Unit assumption)
    //    P = vd*id + vq*iq
    //    Q = vq*id - vd*iq  (Standard convention, verify with your grid standard)
    // -----------------------------------------------------------
    // Note: If Vq is strictly regulated to 0 by PLL, P ~= Vd*Id, Q ~= -Vd*Iq

    pq->pq_meas.dat[0] = ctl_mul(vd, id) + ctl_mul(vq, iq); // Active Power P
    pq->pq_meas.dat[1] = ctl_mul(vq, id) - ctl_mul(vd, iq); // Reactive Power Q

    // -----------------------------------------------------------
    // 2. Main Control Loop
    // -----------------------------------------------------------
    if (pq->flag_enable)
    {
        // --- Active Power Control (P -> Id) ---
        // Error = Setpoint - Measure
        ctrl_gt p_err = pq->pq_set.dat[0] - pq->pq_meas.dat[0];

        // PID Output is Id reference
        // Source convention: P>0 means discharging to grid.
        pq->idq_set_out.dat[phase_d] = ctl_step_pid_ser(&pq->pid_p, p_err);

        // --- Reactive Power Control (Q -> Iq) ---
        // Error = Setpoint - Measure
        ctrl_gt q_err = pq->pq_set.dat[1] - pq->pq_meas.dat[1];

        // PID Output is Iq reference
        // Source convention Q>0 means discharging inductive reactive power to grid.
        pq->idq_set_out.dat[phase_q] = ctl_step_pid_ser(&pq->pid_q, q_err);

        // -----------------------------------------------------------
        // 3. Current Limiting (Circular Saturation)
        //    Prevent the reference from exceeding converter capability.
        // -----------------------------------------------------------
        ctrl_gt id_ref = pq->idq_set_out.dat[phase_d];
        ctrl_gt iq_ref = pq->idq_set_out.dat[phase_q];

        ctrl_gt i_mag_sq = ctl_mul(id_ref, id_ref) + ctl_mul(iq_ref, iq_ref);
        ctrl_gt i2_limit = pq->max_i2_mag;

        if (i_mag_sq > i2_limit)
        {
            // Simple scaling to keep vector direction but limit magnitude
            ctrl_gt scaler = ctl_sqrt(ctl_div(i2_limit, i_mag_sq));
            pq->idq_set_out.dat[phase_d] *= ctl_mul(scaler, pq->idq_set_out.dat[phase_d]);
            pq->idq_set_out.dat[phase_q] *= ctl_mul(scaler, pq->idq_set_out.dat[phase_q]);
        }
    }
    else
    {
        // If disabled, reset integrators and zero output
        pq->idq_set_out.dat[phase_d] = 0;
        pq->idq_set_out.dat[phase_q] = 0;
    }
}

/** @brief Enable PQ controller */
GMP_STATIC_INLINE void ctl_enable_gfl_pq_ctrl(gfl_pq_ctrl_t* pq)
{
    pq->flag_enable = 1;
}

/** @brief Disable PQ controller */
GMP_STATIC_INLINE void ctl_disable_gfl_pq_ctrl(gfl_pq_ctrl_t* pq)
{
    pq->flag_enable = 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_GFL_

/**
 * @}
 */
