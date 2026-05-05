/**
 * @file ctl_main.h
 * @author GMP Library Contributors
 * @brief Top-level Controller for Single-Phase Inverter/AFE.
 * @version 1.0
 * @date 2026-05-05
 *
 * @copyright Copyright GMP(c) 2024-2026
 *
 */

#ifndef _FILE_CTL_MAIN_H_
#define _FILE_CTL_MAIN_H_

#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/intrinsic/discrete/slope_f_pu.h>
#include <ctl/framework/cia402_state_machine.h>
#include <xplt.peripheral.h>

// SINV Control Modules
#include <ctl/component/digital_power/sinv/sinv_protect.h>
#include <ctl/component/digital_power/sinv/sinv_rc_core.h>
#include <ctl/component/digital_power/sinv/sinv_ref_gen.h>
#include <ctl/component/digital_power/sinv/sms_pq.h>
#include <ctl/component/digital_power/sinv/spll_sogi.h>
#include <ctl/component/interface/hpwm_modulator.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// controller modules with extern

extern volatile fast_gt flag_system_running;
extern volatile fast_gt flag_error;

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

// ADC interfaces (Values mapped automatically in step_adc_channel)
extern adc_channel_t adc_v_grid;
extern adc_channel_t adc_i_ac;
extern adc_channel_t adc_v_bus;

// State machine & Protection
extern cia402_sm_t cia402_sm;
extern ctl_sinv_protect_t protection;

// Modulator
extern single_phase_H_modulation_t hpwm;

// Controller body: SINV Core
extern spll_sogi_t pll;
extern ctl_sms_pq_t pq_meter;
extern ctl_sinv_ref_gen_t ref_gen;
extern ctl_sinv_rc_core_t rc_core;

// User Setpoints
extern ctrl_gt g_p_ref_user;
extern ctrl_gt g_q_ref_user;

// Final output duty cycle for the modulator
extern pwm_gt pwm_cmp_L;
extern pwm_gt pwm_cmp_N;

//=================================================================================================
// function prototype
void clear_all_controllers(void);
void ctl_init(void);
void ctl_mainloop(void);
fast_gt ctl_exec_adc_calibration(void);

//=================================================================================================
// controller process

/**
 * @brief periodic callback function things.
 * @details Executed at the highest ISR frequency (e.g., 20kHz).
 */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // ADC input will handled by input process

    // ADC calibrator routine
    if (flag_enable_adc_calibrator)
    {
        // For Single Phase, we calibrate AC voltage and AC current
        if (index_adc_calibrator == 1)
            ctl_step_adc_calibrator(&adc_calibrator, adc_v_grid.control_port.value);
        else if (index_adc_calibrator == 0)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_ac.control_port.value);
    }
    // normal controller routine
    else
    {
        // 1. Grid Synchronization (PLL)
        ctl_step_single_phase_pll(&pll, adc_v_grid.control_port.value);

        // 2. Real-time PQ Measurement
        ctl_step_sms_pq(&pq_meter, adc_v_grid.control_port.value, adc_i_ac.control_port.value, &pll.phasor);

        // 3. Command Generation (P/Q to I_ref)
        if (cia402_sm.state_word.bits.operation_enabled)
        {
            // The internal slope limiters inside ref_gen handle the soft-start automatically
            ctl_step_sinv_ref_gen_pq(&ref_gen, g_p_ref_user, g_q_ref_user, pll.v_mag, &pll.phasor);
        }
        else
        {
            ctl_clear_sinv_ref_gen(&ref_gen);
        }

        // 4. Inner Current Controller (RC Core)
        rc_core.flag_enable_ctrl = cia402_sm.state_word.bits.operation_enabled;

        // Pass I_ref from ref generator. Fdbk ptrs (ADC) are already zero-copy bound in init()
        ctl_step_sinv_rc_core(&rc_core, ref_gen.i_ref_inst);

        // 5. Fast Protection Callback (ISR Level)
        if (ctl_step_sinv_protect_fast(&protection, adc_v_bus.control_port.value, adc_i_ac.control_port.value,
                                       rc_core.v_out_ref))
        {
            cia402_fault_request(&cia402_sm);
        }

        // 6. Dead-time Compensation & PWM Modulation
        if (cia402_sm.state_word.bits.operation_enabled)
        {
            ctl_step_single_phase_H_modulation(&hpwm, rc_core.v_out_ref, adc_i_ac.control_port.value);
        }
        else
        {
            ctl_clear_single_phase_H_modulation(&hpwm);
        }

        // 7. Write outputs for hardware timers
        pwm_cmp_L = ctl_get_single_phase_modulation_L_phase(&hpwm);
        pwm_cmp_N = ctl_get_single_phase_modulation_N_phase(&hpwm);
    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
