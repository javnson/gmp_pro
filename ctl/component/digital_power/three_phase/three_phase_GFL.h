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
#define GFL_VOLTAGE_SAMPLE_PHASE_MODE (1)
#endif // GFL_VOLTAGE_SAMPLE_PHASE_MODE

#ifndef GFL_CAPACITOR_CURRENT_CALCULATE_MODE
/**
 * @brief Configures the voltage sampling method.
 * - **4**: TODO calculate Ic based on Uuvw and Iabc
 * - **3**: Capacitor Voltage is measured, calculate Ic = C d/dt(Vc)
 * - **2**: Sample inverter current Iuvw and grid current Iabc, calculate Ic = Iuvw - Iabc.
 * - **1**: Sample Capacitor current directly.
 */
#define GFL_CAPACITOR_CURRENT_CALCULATE_MODE (1)
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
    ctrl_gt omega_L;        //!< Feed-forward decoupling term: `2*pi*f*L` in per-unit.
    ctrl_gt free_run_slope; //!< Default slope for the ramp generator.

    //
    // --- Setpoints & Intermediate Variables (Read/Write) ---
    //
    ctl_vector2_t vdq_set;     //!< R/W: Voltage command in d-q frame (for voltage mode openloop).
    ctrl_gt rg_freq_pu;        //!< R/W: Ramp generator frequency in per-unit (for freerun mode).
    ctl_vector2_t idq_set;     //!< R/W: Current command in d-q frame (for current mode).
    ctl_vector2_t idq_ff;      //!< R/W: Current feed-forward term in d-q frame.
    ctl_vector2_t vdq_pos_out; //!< R/W: Output of positive-sequence current controller.

    //
    // --- Measurement & Internal State Variables (Read-Only) ---
    //
    ctl_vector3_t iabc; //!< RO: grid current Filtered three-phase currents.
    ctl_vector3_t iab0; //!< RO: grid current Clarke transformed currents {alpha, beta, zero}.
    ctl_vector2_t idq;  //!< RO: Park transformed currents {d, q}.

    ctl_vector3_t vabc; //!< RO: Filtered three-phase voltages.
    ctl_vector3_t vab0; //!< RO: Clarke transformed voltages {alpha, beta, zero}.
    ctl_vector2_t vdq;  //!< RO: Park transformed voltages {d, q}.

    ctl_vector2_t vab_pos; //!< RO: Positive-sequence modulation voltage in alpha-beta frame.
    ctl_vector2_t vab_out; //!< RO: Total modulation voltage in alpha-beta frame.
    ctl_vector3_t abc_out; //!< RO: Final three-phase modulation signals before scaling.

    ctrl_gt angle;        //!< RO: Estimated grid angle.
    ctl_vector2_t phasor; //!< RO: Phasor {cos, sin} corresponding to the grid angle.

    //
    // --- Controller Objects ---
    //

    // input filter
    ctl_filter_IIR1_t filter_iabc[3]; //!< CTRL: current input filter
    ctl_filter_IIR2_t filter_uabc[3]; //!< CTRL: voltage input filter

    // current controller
    ctl_pid_t pid_idq;

    // active damping

    // output lead compensator
    ctrl_lead_t lead_compensator;

    // PLL & RG
    three_phase_pll_t pll;   //!< Three-phase PLL for grid synchronization.
    ctl_ramp_generator_t rg; //!< Ramp generator for open-loop/freerun operation.

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable_system;           //!< Master enable for the entire controller.
    fast_gt flag_enable_current_ctrl;     //!< Enables the inner current control loop.
    fast_gt flag_enable_lead_compensator; //!< Enables the output lead compensator.
    fast_gt flag_enable_decouple;         //!< Enables inductor decoupling feed-forward.
    fast_gt flag_enable_active_damping;   //!< Enables the virtual resistance of capcitor.
    fast_gt flag_enable_pll;              //!< Enables the three-phase PLL.
    fast_gt flag_enable_offgrid;          //!< 1 to use ramp generator angle (free run), 0 to use PLL angle.

} gfl_inv_ctrl_t;

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
    parameter_gt current_loop_bw;   //!< Currnet loop bandwidth frequency (Hz).
    parameter_gt current_loop_zero; // !< Current loop Zero frequency (Hz).
    parameter_gt current_phase_lag; //!< Current loop output compensate angle (rad).

    // the following parameters would be calculated by auto-tuning.
    parameter_gt active_damping_resister; //!< active resister, Ohm.

    parameter_gt kp_pll; //!< PLL controller parameters
    parameter_gt ki_pll; //!< PLL controller parameters

} gfl_inv_ctrl_init_t;

/**
 * @brief Executes one step of the three-phase inverter control algorithm.
 * @ingroup CTL_TOPOLOGY_GFL_INV_H_API
 * @param[out] ctrl Pointer to the `inv_ctrl_t` structure.
 */
void ctl_auto_tuning_gfl_inv(gfl_inv_ctrl_init_t* init)
{
    parameter_gt LC_res;
    parameter_gt LC_res_Hz;

    parameter_gt freq1;
    parameter_gt freq2;

    parameter_gt control_delay;
    parameter_gt filter_delay;

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
        // Calculate LC filter resonent frequency
        LC_res = CTL_PARAM_CONST_2PI * sqrtf(init->grid_filter_L * init->grid_filter_C);
        LC_res_Hz = 1.0f / LC_res;

        // select current loop BW
        freq1 = LC_res_Hz / 3;
        freq2 = init->fs / 10;
        init->current_loop_bw = fminf(freq1, freq2);

        // Calculate LC filt<er characteristic impedance, damping ratio is 0.5
        init->active_damping_resister = sqrtf(init->grid_filter_L / init->grid_filter_C);
    }

    // select current loop zero
    init->current_loop_zero = init->current_loop_bw / 10;

    // controller delay
    control_delay = CTL_PARAM_CONST_2PI * init->current_loop_bw * 1.5f / init->fs;

    // Create a LPF object and calcualte phase lag
    ctl_filter_IIR1_t temp_filter;
    ctl_init_filter_iir1_lpf(&temp_filter, init->fs, init->current_adc_fc);
    filter_delay = ctl_get_filter_iir1_phase_lag(&temp_filter, init->fs, init->current_loop_zero);

    init->current_phase_lag = control_delay + filter_delay;

    // select PLL bandwidth is freq_base / 3
    init->kp_pll = 2.0f * 0.707f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI;
    init->ki_pll = init->freq_base / 3.0f * init->freq_base / 3.0f * CTL_PARAM_CONST_2PI * CTL_PARAM_CONST_2PI;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_GFL_

/**
 * @}
 */
