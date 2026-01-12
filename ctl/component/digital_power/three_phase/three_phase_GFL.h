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
                      //    adc_ift* adc_iabc[3]; //!< Array of pointers to phase current ADCs {Ia, Ib, Ic}.
                      //    adc_ift* adc_vabc[3]; //!< Array of pointers to phase voltage ADCs {Va, Vb, Vc}.

    // Grid side feedback
    tri_adc_ift* adc_vabc; //!< grid phase voltage ADCs {Va, Vb, Vc}, this voltage will use as pll input.
    tri_adc_ift* adc_iabc; //!< grid phase current ADCs {Ia, Ib, Ic}, this current will use as current control port.

    // inverter side feedback
    //tri_adc_ift* adc_vuvw; //!< inverter phase voltage ADCs {Vu, Vv, Vw}, this voltage will use as observer input.
    //tri_adc_ift* adc_iuvw; //!< inverter phase current ADCs {Iu, Iv, Iw}, this current will use as active damping.

    //
    // --- Output Ports ---
    //
    tri_pwm_ift*
        pwm_out; /**< @brief Final PWM duty cycles {A, B, C} in per-unit format. Three-phase PWM output interface. */

    //
    // --- Feed-forward & Parameters ---
    //
    //ctrl_gt omega_L;        //!< Feed-forward decoupling term: `2*pi*f*L` in per-unit.
    //ctrl_gt free_run_slope; //!< Default slope for the ramp generator.

    //
    // --- Setpoints & Intermediate Variables (Read/Write) ---
    //
    ctl_vector2_t idq_set;          //!< R/W: Current command in d-q frame (for current mode).
    ctl_vector2_t idq_ff;           //!< R/W: Current feed-forward term in d-q frame.
    ctl_vector2_t vdq_out;          //!< R/W: Output of positive-sequence current controller.
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
    ctl_vector3_t vab0_out;     //!< RO: Total modulation voltage in alpha-beta frame.
    //ctl_vector3_t abc_out;      //!< RO: Final three-phase modulation signals before scaling.

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
    ctl_filter_IIR2_t filter_damping; //!< CTRL: active capacitor damping
    ctrl_gt coef_ff_damping;          //!< damping gain
    vector2_gt vdq_last;              //!< (vdq - vdq_last) to calculate differential

    // output lead compensator
    ctrl_lead_t lead_compensator;

    // PLL & RG
    three_phase_pll_t pll;   //!< Three-phase PLL for grid synchronization.
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

    ctl_clear_biquad_filter(&inv->filter_damping);
    ctl_vector2_clear(&inv->vdq_last);

    ctl_clear_lead(&inv->lead_compensator);

    // TODO: clear intermediate variables
    
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
void ctl_auto_tuning_gfl_inv(gfl_inv_ctrl_init_t* init)
{
    parameter_gt T_res;
    parameter_gt LC_res_Hz;

    parameter_gt freq1;
    parameter_gt freq2;

    parameter_gt control_delay;
    parameter_gt filter_delay;

    assert(init->grid_filter_L > 1e-9f);

    // Select proper ADC digital filter cut frequency
    init->current_adc_fc = init->fs / 3;
    init->voltage_adc_fc = init->fs / 3;

    // Only L filter grid connector
    if (init->grid_filter_C < 1e-9f)
    {
        init->current_loop_bw = init->fs / 10;
        init->active_damping_resister = 0;
    }
    // LC filter grid connector
    else
    {
        // Calculate LC filter resonant frequency
        T_res = CTL_PARAM_CONST_2PI * sqrtf(init->grid_filter_L * init->grid_filter_C);
        LC_res_Hz = 1.0f / T_res;

        // select current loop BW
        freq1 = LC_res_Hz / 3;
        freq2 = init->fs / 10;
        init->current_loop_bw = fminf(freq1, freq2);

        // Calculate LC filter characteristic impedance, damping ratio is 0.5
        init->active_damping_resister = sqrtf(init->grid_filter_L / init->grid_filter_C);

        // calculate active_damping_center_freq and active_damping_Q
        init->active_damping_center_freq = LC_res_Hz;
        init->active_damping_filter_q = 1.0f;
    }

    // select current loop zero
    init->current_loop_zero = init->current_loop_bw / 10;

    // controller delay
    control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * 1.5f / init->fs;

    // Create a LPF object and calculate phase lag
    ctl_filter_IIR1_t temp_filter;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_zero);

    init->current_phase_lag = control_delay + filter_delay;

    // select PLL bandwidth is freq_base / 3
    init->kp_pll = 2.0f * 0.707f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI;
    init->ki_pll = init->freq_base / 3.0f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI * CTL_PARAM_CONST_2PI;
}

/**
 * @brief update GFL inverter controller parameters by init structure.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_update_gfl_inv_coeff(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init)
{
    int i = 0;

    for (i = 0; i < 3; ++i)
    {
        ctl_init_filter_iir1_lpf(&inv->filter_iabc[i], init->fs, init->current_adc_fc);
        ctl_init_filter_iir1_lpf(&inv->filter_uabc[i], init->fs, init->voltage_adc_fc);
    }

    parameter_gt kp_dq =
        init->grid_filter_L * CTL_PARAM_CONST_2PI * init->current_loop_bw * init->i_base / init->v_base;
    parameter_gt ki_dq = kp_dq * CTL_PARAM_CONST_2PI * init->current_loop_zero;

    ctl_init_pid(&inv->pid_idq[phase_d], kp_dq, ki_dq, 0, init->fs);
    ctl_init_pid(&inv->pid_idq[phase_q], kp_dq, ki_dq, 0, init->fs);

    ctl_init_lead_form3(&inv->lead_compensator, init->current_phase_lag, init->current_loop_bw, init->fs);

    ctl_init_pll_3ph(&inv->pll, init->freq_base, init->kp_pll, init->ki_pll, 0, init->fs);

    ctl_init_ramp_generator_via_freq(&inv->rg, init->fs, init->freq_base, 1, 0);

    // Only L filter grid connector
    if (init->grid_filter_C < 1e-9f)
    {
        // close active damping
        inv->flag_enable_active_damping = 0;
        inv->coef_ff_damping = 0;
    }
    // LC filter grid connector
    else
    {
#if GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 1 || GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 2
        // active damping voltage = damping_gain * iCdq
        inv->coef_ff_damping = float2ctrl(init->active_damping_resister * init->i_base / init->v_base);
#elif GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 3
        // active damping voltage = damping_gain * (udq - udq_last)
        inv->coef_ff_damping = float2ctrl(init->active_damping_resister * init->grid_filter_C * init->fs);
#elif GFL_CAPACITOR_CURRENT_CALCULATE_MODE == 4
        // TODO FIX HERE
#endif // GFL_CAPACITOR_CURRENT_CALCULATE_MODE

        ctl_init_biquad_bpf(&inv->filter_damping, init->fs, init->active_damping_center_freq,
                            init->active_damping_filter_q);
    }

    // decoupling feed-forward
    inv->coef_ff_decouple = CTL_PARAM_CONST_2PI * init->freq_base * init->i_base / init->v_base;
}

/**
 * @brief Init GFL inverter parameters.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] inv Pointer to the gfl_inv_ctrl_t structure.
 * @param[in] init Pointer to the `gfl_inv_ctrl_init_t` structure.
 */
void ctl_init_gfl_inv(gfl_inv_ctrl_t* inv, gfl_inv_ctrl_init_t* init)
{
    ctl_update_gfl_inv_coeff(inv, init);
    ctl_clear_gfl_inv_with_PLL(inv);
}


/**
 * @brief Executes one step of the  GFL three-phase inverter control algorithm.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] ctrl Pointer to the `inv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_step_gfl_inv_ctrl(gfl_inv_ctrl_t* gfl)
{
    // assert critical pointer
    gmp_base_assert(gfl);
    gmp_base_assert(gfl->pwm_out);
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
                gfl->vdq_ff_damping.dat[phase_d] =
                    gfl->coef_ff_damping * (gfl->vdq.dat[phase_d] - gfl->vdq_last.dat[phase_d]);
                gfl->vdq_ff_damping.dat[phase_q] =
                    gfl->coef_ff_damping * (gfl->vdq.dat[phase_q] - gfl->vdq_last.dat[phase_q]);

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
            gfl->vdq_out_comp.dat[phase_d] = ctl_step_lead(&gfl->lead_compensator, gfl->vdq_out.dat[phase_d]);
            gfl->vdq_out_comp.dat[phase_q] = ctl_step_lead(&gfl->lead_compensator, gfl->vdq_out.dat[phase_q]);
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

    // --- 3f. iClarke and output ---
    ctl_ct_iclarke(&gfl->vab0_out, &gfl->pwm_out->value);
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_GFL_

/**
 * @}
 */
