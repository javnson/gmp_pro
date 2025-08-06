/**
 * @file single_phase_dc_ac.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset single-phase DC/AC inverter controller.
 * @version 1.0
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2025
 *
 * @defgroup CTL_TOPOLOGY_SINV_H_API Single-Phase Inverter Topology API (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive single-phase inverter, including harmonic compensation and multiple
 * operating modes.
 */

#ifndef _FILE_SINGLE_PHASE_DC_AC_H_
#define _FILE_SINGLE_PHASE_DC_AC_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#include <ctl/component/digital_power/single_phase/spll.h>
#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

// --- Compilation-time Configuration Macros ---

/**
 * @brief Selects the modulation strategy for the H-bridge.
 * - **Enabled**: Unipolar modulation where PWM is generated on one phase leg only.
 * - **Disabled**: Bipolar modulation where both phase legs are driven symmetrically.
 */
// #define CTL_SINV_CTRL_UNIPOLAR_MODULATION

/**
 * @brief Enables the use of the Sine Analyzer module for true RMS calculations.
 * @note This is the recommended method for accurate AC measurements. If disabled,
 * a fallback mechanism would be required, but the original one was flawed.
 */
#define CTL_SINV_CTRL_ENABLE_SINE_ANALYZER

#ifdef CTL_SINV_CTRL_ENABLE_SINE_ANALYZER
#include <ctl/component/dsa/sine_analyzer.h>
#endif // CTL_SINV_CTRL_ENABLE_SINE_ANALYZER

/**
 * @brief Enumeration for single-phase leg indexing.
 */
typedef enum _tag_single_phase_name
{
    PHASE_N = 0, //!< Index for the 'N' phase leg (typically the lower or reference leg).
    PHASE_L = 1  //!< Index for the 'L' phase leg (typically the upper or active leg).
} single_phase_name_num;

/**
 * @brief Main data structure for the single-phase inverter controller.
 */
typedef struct _tag_sinv_ctrl_type
{
    //
    // --- Input Ports (ADC Interfaces) ---
    //
    adc_ift* adc_udc;   //!< DC Bus voltage.
    adc_ift* adc_idc;   //!< DC Bus current (positive for inverter power flow).
    adc_ift* adc_il;    //!< Inductor current.
    adc_ift* adc_ugrid; //!< Grid voltage.
    adc_ift* adc_igrid; //!< Grid current (positive for power output).

    //
    // --- Output Ports ---
    //
    ctrl_gt sinv_pwm_pu[2]; //!< Final PWM duty cycles in per-unit format {N-phase, L-phase}.

    //
    // --- Setpoints & Intermediate Variables ---
    //
    ctrl_gt ig_set;       //!< Current command amplitude (envelope).
    ctrl_gt ig_ref;       //!< Instantaneous AC current reference.
    ctrl_gt v_set;        //!< Voltage command (DC for rectifier, AC RMS for inverter).
    ctrl_gt pf_set;       //!< Power factor setpoint, `pf_set = cos(phi)`.
    ctrl_gt modulation;   //!< The final modulation signal before PWM mapping.
    ctrl_gt target_phase; //!< The instantaneous phase reference, including power factor angle.
    ctrl_gt vg_rms;       //!< Measured grid voltage RMS value.
    ctrl_gt ig_rms;       //!< Measured grid current RMS value.

    //
    // --- Controller Objects ---
    //
    ctl_low_pass_filter_t lpf_udc;   //!< LPF for DC voltage measurement.
    ctl_low_pass_filter_t lpf_idc;   //!< LPF for DC current measurement.
    ctl_low_pass_filter_t lpf_il;    //!< LPF for inductor current measurement.
    ctl_low_pass_filter_t lpf_ugrid; //!< LPF for grid voltage measurement.
    ctl_low_pass_filter_t lpf_igrid; //!< LPF for grid current measurement.

    ctl_single_phase_pll spll; //!< Single-phase PLL for grid synchronization.
    ctl_src_rg_t rg;           //!< Ramp generator for open-loop/freerun operation.

    // --- Core and Harmonic Controllers ---
    pid_regular_t voltage_pid; //!< PID controller for the voltage loop.
    qpr_ctrl_t sinv_qpr_base;  //!< QPR controller for the fundamental current loop.
    qr_ctrl_t sinv_qr_3;       //!< QR controller for 3rd harmonic compensation.
    qr_ctrl_t sinv_qr_5;       //!< QR controller for 5th harmonic compensation.
    qr_ctrl_t sinv_qr_7;       //!< QR controller for 7th harmonic compensation.
    qr_ctrl_t sinv_qr_9;       //!< QR controller for 9th harmonic compensation.
    qr_ctrl_t sinv_qr_11;      //!< QR controller for 11th harmonic compensation.
    qr_ctrl_t sinv_qr_13;      //!< QR controller for 13th harmonic compensation.
    qr_ctrl_t sinv_qr_15;      //!< QR controller for 15th harmonic compensation.

#ifdef CTL_SINV_CTRL_ENABLE_SINE_ANALYZER
    sine_analyzer_t ac_current_measure; //!< Sine analyzer for grid current.
    sine_analyzer_t ac_voltage_measure; //!< Sine analyzer for grid voltage.
#endif

    //
    // --- Control Flags ---
    //
    fast_gt flag_enable_system;         //!< Master enable for the entire controller.
    fast_gt flag_enable_current_ctrl;   //!< Enables the inner current control loop.
    fast_gt flag_enable_harmonic_ctrl;  //!< Enables the additional harmonic compensators.
    fast_gt flag_enable_voltage_ctrl;   //!< Enables the outer voltage control loop.
    fast_gt flag_enable_ac_measurement; //!< Enables the AC RMS measurement modules.
    fast_gt flag_rectifier_mode;        //!< 0 for Inverter (DC->AC), 1 for Rectifier (AC->DC).
    fast_gt flag_enable_ramp;           //!< Enables the internal ramp generator.
    fast_gt flag_angle_freerun;         //!< 1 to use ramp generator angle, 0 to use PLL angle.
    fast_gt flag_enable_spll;           //!< Enables the single-phase PLL.

} sinv_ctrl_t;

/**
 * @brief Initialization parameters for the single-phase inverter controller.
 */
typedef struct _tag_single_phase_converter_init_t
{
    // --- System Frequencies ---
    parameter_gt base_freq; //!< Nominal grid frequency (e.g., 50 or 60 Hz).
    parameter_gt f_ctrl;    //!< Controller execution frequency (sampling frequency) in Hz.

    // --- Controller Gains ---
    parameter_gt v_ctrl_kp; //!< Voltage loop: Proportional gain.
    parameter_gt v_ctrl_Ti; //!< Voltage loop: Integral time (s).
    parameter_gt v_ctrl_Td; //!< Voltage loop: Derivative time (s).

    parameter_gt i_ctrl_kp;       //!< Current loop (QPR): Proportional gain.
    parameter_gt i_ctrl_kr;       //!< Current loop (QPR): Resonant gain.
    parameter_gt i_ctrl_cut_freq; //!< Current loop (QPR): Cutoff frequency for the resonant part (Hz).

    parameter_gt pll_ctrl_kp;       //!< PLL loop: Proportional gain.
    parameter_gt pll_ctrl_Ti;       //!< PLL loop: Integral time (s).
    parameter_gt pll_ctrl_cut_freq; //!< PLL loop: Cutoff frequency for the SOGI LPF (Hz).

    // --- Harmonic Compensation Gains (QR) ---
    parameter_gt harm_ctrl_kr_3;        //!< 3rd harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_3;  //!< 3rd harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_5;        //!< 5th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_5;  //!< 5th harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_7;        //!< 7th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_7;  //!< 7th harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_9;        //!< 9th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_9;  //!< 9th harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_11;       //!< 11th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_11; //!< 11th harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_13;       //!< 13th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_13; //!< 13th harmonic: Cutoff frequency (Hz).
    parameter_gt harm_ctrl_kr_15;       //!< 15th harmonic: Resonant gain.
    parameter_gt harm_ctrl_cut_freq_15; //!< 15th harmonic: Cutoff frequency (Hz).

    // --- Filter Parameters ---
    parameter_gt adc_filter_fc; //!< Cutoff frequency for all ADC input filters (Hz).

} sinv_init_t;

// --- Function Declarations & Inline Implementations ---

// Forward declarations for functions defined in the corresponding .c file
void ctl_upgrade_sinv_param(sinv_ctrl_t* sinv, sinv_init_t* init);
void ctl_init_sinv_ctrl(sinv_ctrl_t* sinv, sinv_init_t* init);
void ctl_attach_sinv_with_adc(sinv_ctrl_t* sinv, adc_ift* udc, adc_ift* idc, adc_ift* il, adc_ift* ugrid,
                              adc_ift* igrid);

/**
 * @brief Clears the states of the main controllers (PID, QPR, QR).
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_clear_sinv(sinv_ctrl_t* sinv)
{
    ctl_clear_pid(&sinv->voltage_pid);
    ctl_clear_qpr_controller(&sinv->sinv_qpr_base);
    ctl_clear_qr_controller(&sinv->sinv_qr_3);
    ctl_clear_qr_controller(&sinv->sinv_qr_5);
    ctl_clear_qr_controller(&sinv->sinv_qr_7);
    // Note: Clearing of higher order harmonics can be added if necessary.
}

/**
 * @brief Clears the states of the main controllers and the PLL.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_clear_sinv_with_pll(sinv_ctrl_t* sinv)
{
    ctl_clear_sinv(sinv);
    ctl_clear_single_phase_pll(&sinv->spll);
}

/**
 * @brief Executes one step of the single-phase inverter control algorithm.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @warning The fallback RMS calculation (when SINE_ANALYZER is disabled) in the original
 * code was flawed due to direct float comparison. It has been removed. It is highly
 * recommended to keep `CTL_SINV_CTRL_ENABLE_SINE_ANALYZER` enabled.
 *
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 * @return The final modulation signal before being mapped to PWM outputs.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv(sinv_ctrl_t* sinv)
{
    // --- 1. ADC Input Filtering ---
    ctl_step_lowpass_filter(&sinv->lpf_idc, sinv->adc_idc->value);
    ctl_step_lowpass_filter(&sinv->lpf_udc, sinv->adc_udc->value);
    ctl_step_lowpass_filter(&sinv->lpf_il, sinv->adc_il->value);
    ctl_step_lowpass_filter(&sinv->lpf_ugrid, sinv->adc_ugrid->value);
    ctl_step_lowpass_filter(&sinv->lpf_igrid, sinv->adc_igrid->value);

    // --- 2. Grid Synchronization (PLL) ---
    if (sinv->flag_enable_spll)
    {
        ctl_step_single_phase_pll(&sinv->spll, ctl_get_lowpass_filter_result(&sinv->lpf_ugrid));
    }

    // --- 3. AC Measurement ---
    if (sinv->flag_enable_ac_measurement)
    {
#ifdef CTL_SINV_CTRL_ENABLE_SINE_ANALYZER
        sinv->vg_rms =
            ctl_step_sine_analyzer(&sinv->ac_voltage_measure, ctl_get_lowpass_filter_result(&sinv->lpf_ugrid));
        sinv->ig_rms =
            ctl_step_sine_analyzer(&sinv->ac_current_measure, ctl_get_lowpass_filter_result(&sinv->lpf_igrid));
#endif
    }

    if (sinv->flag_enable_system)
    {
        // --- 4. Angle Reference Generation ---
        if (sinv->flag_enable_ramp)
        {
            ctl_step_ramp_gen(&sinv->rg);
        }

        ctrl_gt base_angle = sinv->flag_angle_freerun ? ctl_get_ramp_gen_output(&sinv->rg) : sinv->spll.theta;

        // Calculate phase shift for power factor
        ctrl_gt phi = ctl_acos(sinv->pf_set);
        sinv->target_phase = ctl_sin(base_angle + phi);

        // --- 5. Outer Voltage Loop ---
        if (sinv->flag_enable_voltage_ctrl)
        {
            if (sinv->flag_rectifier_mode)
            { // Rectifier (AC->DC)
                sinv->ig_set =
                    -ctl_step_pid_ser(&sinv->voltage_pid, sinv->v_set - ctl_get_lowpass_filter_result(&sinv->lpf_udc));
            }
            else
            { // Inverter (DC->AC)
                sinv->ig_set = ctl_step_pid_ser(&sinv->voltage_pid, sinv->v_set - sinv->vg_rms);
            }
        }

        // --- 6. Inner Current Loop ---
        if (sinv->flag_enable_current_ctrl)
        {
            sinv->ig_ref = ctl_mul(sinv->target_phase, sinv->ig_set);
            ctrl_gt current_error = sinv->ig_ref - ctl_get_lowpass_filter_result(&sinv->lpf_igrid);

            sinv->modulation = ctl_step_qpr_controller(&sinv->sinv_qpr_base, current_error);

            if (sinv->flag_enable_harmonic_ctrl)
            {
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_3, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_5, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_7, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_9, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_11, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_13, -current_error);
                sinv->modulation += ctl_step_qr_controller(&sinv->sinv_qr_15, -current_error);
            }
        }
        else
        { // Open-loop current
            sinv->ig_ref = ctl_mul(sinv->target_phase, sinv->ig_set);
            sinv->modulation = sinv->ig_ref;
        }

        // --- 7. Modulation Stage ---
#ifdef CTL_SINV_CTRL_UNIPOLAR_MODULATION
        // Unipolar SPWM
        sinv->sinv_pwm_pu[PHASE_L] = sinv->modulation;
        sinv->sinv_pwm_pu[PHASE_N] = -sinv->modulation;
#else
        // Bipolar SPWM
        sinv->sinv_pwm_pu[PHASE_L] = ctl_div2(ctl_add(sinv->modulation, float2ctrl(1.0f)));
        sinv->sinv_pwm_pu[PHASE_N] = ctl_div2(ctl_sub(float2ctrl(1.0f), sinv->modulation));
#endif
    }
    else
    {
        // Disable output if system is not enabled
        sinv->sinv_pwm_pu[PHASE_L] = 0;
        sinv->sinv_pwm_pu[PHASE_N] = 0;
    }

    return sinv->modulation;
}

// --- Controller State and Setpoint Functions ---

/**
 * @brief Sets the controller to open-loop inverter mode.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @note This function configures flags for open-loop operation but leaves the system
 * disabled. Call `ctl_enable_sinv_ctrl()` to start operation.
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_set_sinv_openloop_inverter(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 0;
    sinv->flag_enable_current_ctrl = 0;
    sinv->flag_enable_harmonic_ctrl = 0;
    sinv->flag_enable_voltage_ctrl = 0;
    sinv->flag_rectifier_mode = 0; // Inverter mode
    sinv->flag_enable_ramp = 1;
    sinv->flag_angle_freerun = 1; // Use ramp generator
    sinv->flag_enable_spll = 1;
    sinv->flag_enable_ac_measurement = 1;
}

/**
 * @brief Sets the controller to current closed-loop inverter mode.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @note This function configures flags for current-controlled operation but leaves the system
 * disabled. Call `ctl_enable_sinv_ctrl()` to start operation.
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_set_sinv_current_closeloop_inverter(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 0;
    sinv->flag_enable_current_ctrl = 1;
    sinv->flag_enable_harmonic_ctrl = 0;
    sinv->flag_enable_voltage_ctrl = 0;
    sinv->flag_rectifier_mode = 0; // Inverter mode
    sinv->flag_enable_ramp = 1;
    sinv->flag_angle_freerun = 1; // Use ramp generator
    sinv->flag_enable_spll = 1;
    sinv->flag_enable_ac_measurement = 1;
}

/**
 * @brief Sets the controller to voltage closed-loop inverter mode.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @note This function configures flags for voltage-controlled operation but leaves the system
 * disabled. Call `ctl_enable_sinv_ctrl()` to start operation.
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_set_sinv_voltage_closeloop_inverter(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 0;
    sinv->flag_enable_current_ctrl = 1;
    sinv->flag_enable_harmonic_ctrl = 0;
    sinv->flag_enable_voltage_ctrl = 1;
    sinv->flag_rectifier_mode = 0; // Inverter mode
    sinv->flag_enable_ramp = 1;
    sinv->flag_angle_freerun = 1; // Use ramp generator
    sinv->flag_enable_spll = 1;
    sinv->flag_enable_ac_measurement = 1;
}

/**
 * @brief Sets the controller to voltage closed-loop rectifier mode.
 * @ingroup CTL_TOPOLOGY_SINV_H_API
 * @note This function configures flags for rectifier operation but leaves the system
 * disabled. Call `ctl_enable_sinv_ctrl()` to start operation.
 * @param[out] sinv Pointer to the `sinv_ctrl_t` structure.
 */
GMP_STATIC_INLINE void ctl_set_sinv_voltage_closeloop_rectifier(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 0;
    sinv->flag_enable_current_ctrl = 1;
    sinv->flag_enable_harmonic_ctrl = 0;
    sinv->flag_enable_voltage_ctrl = 1;
    sinv->flag_rectifier_mode = 1; // Rectifier mode
    sinv->flag_enable_ramp = 0;
    sinv->flag_angle_freerun = 0; // Use PLL
    sinv->flag_enable_spll = 1;
    sinv->flag_enable_ac_measurement = 1;
}

/** @brief Sets the target current amplitude. */
GMP_STATIC_INLINE void ctl_set_sinv_current_ref(sinv_ctrl_t* sinv, ctrl_gt i_target)
{
    sinv->ig_set = i_target;
}

/** @brief Sets the target voltage (DC or AC RMS). */
GMP_STATIC_INLINE void ctl_set_sinv_voltage_ref(sinv_ctrl_t* sinv, ctrl_gt v_target)
{
    sinv->v_set = v_target;
}

/** @brief Sets the target power factor. */
GMP_STATIC_INLINE void ctl_set_sinv_power_factor(sinv_ctrl_t* sinv, ctrl_gt pf)
{
    sinv->pf_set = pf;
}

/** @brief Enables the main system controller. */
GMP_STATIC_INLINE void ctl_enable_sinv_ctrl(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 1;
}

/** @brief Disables the main system controller, stopping PWM output. */
GMP_STATIC_INLINE void ctl_disable_sinv_ctrl(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_system = 0;
}

/** @brief Sets the angle source to the internal ramp generator (freerun). */
GMP_STATIC_INLINE void ctl_set_sinv_freerun(sinv_ctrl_t* sinv)
{
    sinv->flag_angle_freerun = 1;
}

/** @brief Sets the angle source to the PLL (grid-tied). */
GMP_STATIC_INLINE void ctl_set_sinv_pll(sinv_ctrl_t* sinv)
{
    sinv->flag_angle_freerun = 0;
}

/** @brief Enables the harmonic compensation controllers. */
GMP_STATIC_INLINE void ctl_enable_sinv_harm_ctrl(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_harmonic_ctrl = 1;
}

/** @brief Disables the harmonic compensation controllers. */
GMP_STATIC_INLINE void ctl_disable_sinv_harm_ctrl(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_harmonic_ctrl = 0;
}

/** @brief Checks if the controller is in inverter mode. */
GMP_STATIC_INLINE fast_gt ctl_is_sinv_inverter_mode(sinv_ctrl_t* sinv)
{
    return (sinv->flag_rectifier_mode == 0);
}

/** @brief Enables the AC RMS measurement modules. */
GMP_STATIC_INLINE void ctl_enable_sinv_ac_measurement(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_ac_measurement = 1;
}

/** @brief Disables the AC RMS measurement modules. */
GMP_STATIC_INLINE void ctl_disable_sinv_ac_measurement(sinv_ctrl_t* sinv)
{
    sinv->flag_enable_ac_measurement = 0;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SINGLE_PHASE_DC_AC_H_

/**
 * @}
 */
