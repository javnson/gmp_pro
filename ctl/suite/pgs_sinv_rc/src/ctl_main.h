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

#include <xplt.peripheral.h>

#include <core/pm/function_scheduler.h>

#include <core/dev/pil_core.h>

#include <ctl/component/interface/adc_channel.h>
#include <ctl/framework/cia402_state_machine.h>

// SINV Control Modules
#include <ctl/component/digital_power/sinv/sinv_protect.h>
#include <ctl/component/digital_power/sinv/sinv_rc_core.h>
#include <ctl/component/digital_power/sinv/sinv_ref_gen.h>
#include <ctl/component/digital_power/sinv/sinv_outer_loop.h>
#include <ctl/component/digital_power/sinv/sms_pq.h>
#include <ctl/component/digital_power/sinv/spll_sogi.h>
#include <ctl/component/interface/hpwm_modulator.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// extern controller modules

// System framework
extern cia402_sm_t cia402_sm;

// Control Law Core
extern spll_sogi_t pll;
extern ctl_sms_pq_t pq_meter;
extern ctl_sinv_ref_gen_t ref_gen;
extern ctl_sinv_rc_core_t rc_core;
extern ctl_sinv_outer_loop_t outer_loop;
extern ctl_ramp_generator_t rg;

// Input channel
extern adc_channel_t adc_v_grid;
extern adc_channel_t adc_i_ac;
extern adc_channel_t adc_v_bus;

// Output channel
extern single_phase_H_modulation_t hpwm;

// Protection module
extern ctl_sinv_protect_t protection;

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;

// User commands
extern ctrl_gt g_p_ref_user;
extern ctrl_gt g_q_ref_user;
extern ctrl_gt g_vbus_ref_user;

extern ctrl_gt openloop_v_ref;
extern vector2_gt phasor;

//=================================================================================================
// function prototype
void clear_all_controllers(void);
void ctl_init(void);
void ctl_mainloop(void);
fast_gt ctl_exec_adc_calibration(void);
fast_gt ctl_exec_dc_voltage_ready(void);
fast_gt ctl_check_pll_locked(void);
fast_gt ctl_check_compliance(void);
fast_gt ctl_fault_recover_routine(void);

//=================================================================================================
// Background Controller Tasks

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
        // Calibrate only the zero-current channel. Grid voltage can be present
        // during startup and therefore is not a valid zero-offset source.
        ctl_step_adc_calibrator(&adc_calibrator, adc_i_ac.control_port.value);
    }
    // normal controller routine
    else
    {
        // 1. Grid Synchronization (PLL)
        ctl_step_single_phase_pll(&pll, adc_v_grid.control_port.value);

        // 2. Real-time PQ Measurement
        ctl_step_sms_pq(&pq_meter, pll.uab.dat[phase_alpha], pll.uab.dat[phase_beta], adc_i_ac.control_port.value);

        // 3. Command generation for the selected commissioning level.
        if (cia402_sm.state_word.bits.operation_enabled)
        {
#if BUILD_LEVEL <= 2
            ctl_step_ramp_generator(&rg);
            ctl_set_phasor_via_angle(rg.current, &phasor);
#endif
#if BUILD_LEVEL == 1
            ctl_clear_sinv_ref_gen(&ref_gen);
#elif BUILD_LEVEL == 2
            ref_gen.i_ref_inst = ctl_mul(float2ctrl(SINV_LEVEL2_CURRENT_REF_PEAK_PU),
                                         phasor.dat[phasor_sin]);
#elif BUILD_LEVEL == 3
            ctl_step_sinv_ref_gen_pq(&ref_gen, g_p_ref_user, g_q_ref_user, ctl_abs(pll.v_mag), &pll.phasor);
#elif BUILD_LEVEL == 4
            ctl_step_sinv_ref_gen_pq(&ref_gen,
                ctl_step_sinv_power_loop(&outer_loop, g_p_ref_user, pq_meter.active_power_p),
                g_q_ref_user, ctl_abs(pll.v_mag), &pll.phasor);
#elif BUILD_LEVEL == 5
            ctl_step_sinv_ref_gen_pq(&ref_gen,
                ctl_step_sinv_dc_bus_loop(&outer_loop, g_vbus_ref_user,
                    adc_v_bus.control_port.value, float2ctrl(-1.0f)),
                float2ctrl(0.0f), ctl_abs(pll.v_mag), &pll.phasor);
#endif
        }
        else
        {
            ctl_clear_sinv_ref_gen(&ref_gen);
            ctl_clear_sinv_outer_loop(&outer_loop);
        }

        // 4. Inner Current Controller (RC Core)
#if BUILD_LEVEL == 1
        rc_core.flag_enable_ctrl = 0;
#else
        rc_core.flag_enable_ctrl = cia402_sm.state_word.bits.operation_enabled;
#endif

#if BUILD_LEVEL >= 3
        // PLL frequency is per-unit; FDRC requires the actual fundamental in Hz.
        rc_core.fundamental_freq = CTRL_GRID_FREQUENCY * ctrl2float(pll.frequency);
#else
        rc_core.fundamental_freq = CTRL_GRID_FREQUENCY;
#endif

        // Pass I_ref from ref generator. Fdbk ptrs (ADC) are already zero-copy bound in init()
        ctl_step_sinv_rc_core(&rc_core, ref_gen.i_ref_inst);

        // 5. Fast Protection Callback (ISR Level)
        if (ctl_step_sinv_protect_fast(&protection, adc_v_bus.control_port.value, adc_i_ac.control_port.value,
                                       rc_core.v_out_unsat))
        {
            cia402_fault_request(&cia402_sm);
        }

        // PWM Modulation
        if (cia402_sm.state_word.bits.operation_enabled)
        {
#if BUILD_LEVEL == 1
            ctl_step_single_phase_H_modulation(&hpwm, ctl_mul(openloop_v_ref, phasor.dat[phasor_sin]), adc_i_ac.control_port.value);
#else
            ctl_step_single_phase_H_modulation(&hpwm, rc_core.v_out_ref, adc_i_ac.control_port.value);
#endif // BUILD_LEVEL


        }
        else
        {
            ctl_clear_single_phase_H_modulation(&hpwm);
        }

    }
}

#ifdef __cplusplus
}
#endif // _cplusplus

#endif // _FILE_CTL_MAIN_H_
