/**
 * @file ctl_sinv_protect.h
 * @author GMP Library Contributors
 * @brief Single-Phase Inverter Protection Manager.
 * 
 * @details
 * Assembles highly-optimized specialized protection nodes into a cohesive 
 * protection manager for Single-Phase Inverters or Active Front Ends (AFE).
 * Features Fast (ISR-level) and Slow (Task-level) evaluation pipelines,
 * Mask-based Error/Warning routing, and First-Fault capture for blackbox diagnostics.
 * 
 * @version 1.0
 * @copyright Copyright GMP(c) 2024-2026
 */

#include <ctl/math_block/gmp_math.h>
#include <ctl/component/intrinsic/protection/protection_slot.h>
#include <ctl/component/intrinsic/protection/pt100x.h>

#ifndef _CTL_SINV_PROTECT_H_
#define _CTL_SINV_PROTECT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*===========================================================================*/
/* 1. System Specific Fault Bitmasks (SINV Domain)                           */
/*===========================================================================*/

#define SINV_PROT_BIT_NONE         (0x00000000UL)
#define SINV_PROT_BIT_HW_TZ        (0x00000001UL) //!< Hardware trip-zone shutdown.
#define SINV_PROT_BIT_DC_OVP_FAST  (0x00000002UL) //!< Fast DC-bus overvoltage.
#define SINV_PROT_BIT_AC_OCP_FAST  (0x00000004UL) //!< Fast AC-side overcurrent.
#define SINV_PROT_BIT_CTRL_DIVERGE (0x00000008UL) //!< Fast controller-divergence detection.

#define SINV_PROT_BIT_AC_OVP_RMS   (0x00000010UL) //!< Slow AC RMS overvoltage.
#define SINV_PROT_BIT_AC_UVP_RMS   (0x00000020UL) //!< Slow AC RMS undervoltage.
#define SINV_PROT_BIT_PLL_FREQ_ERR (0x00000040UL) //!< Slow PLL frequency fault for anti-islanding.

#define SINV_PROT_BIT_IGBT_TEMP_OVP    (0x00000080UL) //!< Slow power-module overtemperature from PT100/PT1000.
#define SINV_PROT_BIT_IGBT_THERMAL_I2T (0x00000100UL) //!< Slow inverse-time power-module thermal overload.

/*===========================================================================*/
/* 2. Protection Manager Structure                                           */
/*===========================================================================*/

/**
 * @brief Main Protection Manager for Single-Phase Inverter.
 */
typedef struct _tag_sinv_protect_t
{
    // --- Policy and status flags ---
    uint32_t current_status;  //!< Raw aggregate of all active limit violations.
    uint32_t first_error;     //!< Latched first fatal fault for postmortem diagnostics.
    uint32_t active_errors;   //!< Fatal faults selected by error_mask; these cause a trip.
    uint32_t active_warnings; //!< Warnings selected by warning_mask; these do not trip.

    uint32_t error_mask;   //!< A set bit classifies the corresponding fault as fatal.
    uint32_t warning_mask; //!< A set bit classifies the corresponding fault as a warning.

    // --- Fast protection nodes executed in the control ISR ---
    ctl_prot_single_t node_dc_ovp_fast;  //!< DC-bus overvoltage with a high threshold.
    ctl_prot_single_t node_ac_ocp_fast;  //!< AC overcurrent with a symmetric magnitude threshold.
    ctl_prot_single_t node_ctrl_diverge; //!< Controller divergence with a symmetric magnitude threshold.

    // --- Slow protection nodes executed in a 1 ms or 10 ms task ---
    // OVP and UVP use separate nodes because their required debounce times can differ significantly.
    ctl_prot_single_t node_ac_ovp_rms; //!< AC RMS overvoltage with a high threshold.
    ctl_prot_single_t node_ac_uvp_rms; //!< AC RMS undervoltage with a low threshold.

    ctl_prot_window_t node_pll_freq; //!< Grid-frequency deviation with a two-sided window.

    ctl_prot_pt_sensor_t node_igbt_temp; //!< IGBT temperature sensor with resistance conversion.
    ctl_prot_thermal_t node_igbt_i2t;    //!< IGBT I2t integral thermal model.

} ctl_sinv_protect_t;

/*===========================================================================*/
/* Initialization Structure                                                  */
/*===========================================================================*/

/**
 * @brief Auto-configuration structure for the SINV Protection Manager.
 * @details Users populate physical thresholds here. If any threshold is left 
 * as 0.0f, the initialization function will apply safe, industry-standard defaults.
 */
typedef struct _tag_sinv_prot_init_t
{
    // --- Policy routing masks ---
    uint32_t error_mask;   //!< Which faults should trip the system?
    uint32_t warning_mask; //!< Which faults are just warnings?

    // --- Fast Protection Thresholds (ISR Level) ---
    parameter_gt v_bus_max;  //!< DC Bus Over-Voltage Limit (e.g., 420.0 V).
    parameter_gt i_ac_max;   //!< AC Peak Over-Current Limit (e.g., 30.0 A).
    parameter_gt v_ctrl_max; //!< Max Controller output PU to detect divergence (e.g., 1.5).

    // --- Slow Protection Thresholds (Task Level) ---
    parameter_gt v_ac_rms_max; //!< AC RMS Over-Voltage Limit (e.g., 260.0 V).
    parameter_gt v_ac_rms_min; //!< AC RMS Under-Voltage Limit (e.g., 180.0 V).

    parameter_gt freq_grid_nom; //!< Nominal grid frequency (50.0 or 60.0 Hz).
    parameter_gt freq_dev_max;  //!< Max allowed frequency deviation (e.g., 0.5 Hz).

    // --- Thermal & Sensor Thresholds ---
    parameter_gt igbt_temp_max; //!< Max IGBT temperature in Celsius (e.g., 85.0 C).
    parameter_gt pt_adc_gain;   //!< ADC to Ohms conversion gain for PT100/1000.
    parameter_gt pt_r0;         //!< Base resistance (100.0 or 1000.0).

    parameter_gt i_ac_rated_rms; //!< Nominal continuous AC current for I2t.
    parameter_gt i2t_limit;      //!< Maximum I2t thermal integral limit.

} ctl_sinv_prot_init_t;

/*===========================================================================*/
/* 3. API Definitions                                                        */
/*===========================================================================*/

/**
 * @brief Initializes the SINV Protection Manager with smart defaults.
 * 
 * @param[out] prot Pointer to the protection manager.
 * @param[in]  init Pointer to the configuration structure.
 */
void ctl_init_sinv_protect(ctl_sinv_protect_t* prot, const ctl_sinv_prot_init_t* init);

/**
 * @brief Deep-reset the protection manager.
 * @details Use this during startup and after a CiA 402 FAULT_RESET (0x0080) command.
 * Clears fault flags, debounce counters, first-fault records, and the thermal integrator.
 * 
 * @param[out] prot Pointer to the protection manager.
 */
GMP_STATIC_INLINE void ctl_reset_sinv_protect(ctl_sinv_protect_t* prot)
{
    // 1. Clear global routing status.
    prot->current_status = 0;
    prot->first_error = 0;
    prot->active_errors = 0;
    prot->active_warnings = 0;

    // 2. Clear node debounce and first-fault history.
    prot->node_dc_ovp_fast.current_count = 0;
    prot->node_dc_ovp_fast.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_ac_ocp_fast.current_count = 0;
    prot->node_ac_ocp_fast.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_ctrl_diverge.current_count = 0;
    prot->node_ctrl_diverge.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_ac_ovp_rms.current_count = 0;
    prot->node_ac_ovp_rms.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_ac_uvp_rms.current_count = 0;
    prot->node_ac_uvp_rms.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_pll_freq.current_count = 0;
    prot->node_pll_freq.fault_record_val = CTL_CTRL_CONST_ZERO;

    prot->node_igbt_temp.current_count = 0;
    prot->node_igbt_temp.fault_record_val = CTL_CTRL_CONST_ZERO;

    // 3. Reset the thermal integrator.
    // A system reset starts a new software thermal-integration interval.
    prot->node_igbt_i2t.thermal_acc = CTL_CTRL_CONST_ZERO;
    prot->node_igbt_i2t.fault_record_val = CTL_CTRL_CONST_ZERO;
}

/**
 * @brief Execute the fast protection pipeline.
 * @details Call from the control ISR; the path performs only bounded arithmetic and comparisons.
 * 
 * @param prot Pointer to the manager.
 * @param v_bus_inst Instantaneous DC-bus voltage.
 * @param i_ac_inst Instantaneous AC current.
 * @param ctrl_v_ref Controller voltage command used for divergence detection.
 * @return uint32_t Active fatal-fault mask; a nonzero value requires immediate PWM shutdown.
 */
GMP_STATIC_INLINE uint32_t ctl_step_sinv_protect_fast(ctl_sinv_protect_t* prot, ctrl_gt v_bus_inst, ctrl_gt i_ac_inst,
                                                      ctrl_gt ctrl_v_ref)
{
    uint32_t inst_status = 0;

    // 1. Evaluate fast protection nodes.
    inst_status |= ctl_step_prot_single_high_peak(&prot->node_dc_ovp_fast, v_bus_inst);
    inst_status |= ctl_step_prot_single_sym_peak(&prot->node_ac_ocp_fast, i_ac_inst);
    inst_status |= ctl_step_prot_single_sym_peak(&prot->node_ctrl_diverge, ctrl_v_ref);

    // 2. Aggregate status and apply policy routing.
    prot->current_status |= inst_status;
    prot->active_errors |= (inst_status & prot->error_mask);
    prot->active_warnings |= (inst_status & prot->warning_mask);

    // 3. Capture the first fatal fault.
    if (prot->active_errors != 0 && prot->first_error == 0)
    {
        prot->first_error = prot->active_errors;
    }

    return prot->active_errors;
}

/**
 * @brief Execute the slow protection pipeline.
 * @details Call from a background or RTOS task for RMS, temperature, and thermal calculations.
 * 
 * @param prot Pointer to the manager.
 * @param v_ac_rms AC RMS voltage.
 * @param pll_freq_hz Grid frequency reported by the PLL in hertz.
 * @param temp_adc_pu Temperature-sensor ADC value in raw or per-unit form.
 * @param i_ac_rms AC RMS current used by the I2t integrator.
 */
GMP_STATIC_INLINE void ctl_task_sinv_protect_slow(ctl_sinv_protect_t* prot, ctrl_gt v_ac_rms, ctrl_gt pll_freq_hz,
                                                  ctrl_gt temp_adc_pu, ctrl_gt i_ac_rms)
{
    uint32_t task_status = 0;

    // 1. Evaluate slow protection nodes.
    task_status |= ctl_step_prot_single_high_peak(&prot->node_ac_ovp_rms, v_ac_rms);
    task_status |= ctl_step_prot_single_low_snap(&prot->node_ac_uvp_rms, v_ac_rms);

    task_status |= ctl_step_prot_window_snap(&prot->node_pll_freq, pll_freq_hz);

    task_status |= ctl_step_prot_pt_sensor(&prot->node_igbt_temp, temp_adc_pu);
    task_status |= ctl_step_prot_thermal_i2t(&prot->node_igbt_i2t, i_ac_rms);

    // 2. Aggregate status and apply policy routing.
    prot->current_status |= task_status;
    prot->active_errors |= (task_status & prot->error_mask);
    prot->active_warnings |= (task_status & prot->warning_mask);

    // 3. Capture the first fatal fault.
    if (prot->active_errors != 0 && prot->first_error == 0)
    {
        prot->first_error = prot->active_errors;
    }
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_SINV_PROTECT_H_
