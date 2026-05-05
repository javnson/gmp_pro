/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief Implementation of the Top-level Single-Phase Inverter Control Pipeline.
 *
 * @copyright Copyright GMP(c) 2024-2026
 *
 */

#include <gmp_core.h>

#include "ctl_main.h"

//=================================================================================================
// global controller variables

// state machine
cia402_sm_t cia402_sm;
ctl_sinv_protect_t protection;

// modulator
single_phase_H_modulation_t hpwm;

// controller body
spll_sogi_t pll;
ctl_sms_pq_t pq_meter;
ctl_sinv_ref_gen_t ref_gen;
ctl_sinv_rc_core_t rc_core;

ctrl_gt g_p_ref_user = float2ctrl(0.0f);
ctrl_gt g_q_ref_user = float2ctrl(0.0f);

volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// adc calibrator flags
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
volatile fast_gt index_adc_calibrator = 0;

// IO and offsets
ctrl_gt adc_v_grid = float2ctrl(0.0f);
ctrl_gt adc_i_ac = float2ctrl(0.0f);
ctrl_gt adc_v_bus = float2ctrl(0.0f);

ctrl_gt adc_offset_v_grid = float2ctrl(0.0f);
ctrl_gt adc_offset_i_ac = float2ctrl(0.0f);

pwm_gt pwm_cmp_L = 0;
pwm_gt pwm_cmp_N = 0;

// FDRC Static Buffer
#define CONTROLLER_FREQUENCY (20000.0f)
#define FDRC_ARRAY_SIZE      (420) // 20kHz / 50Hz
static ctrl_gt fdrc_buffer[FDRC_ARRAY_SIZE];

//=================================================================================================
// CTL initialize routine

void ctl_init(void)
{
    // stop here and wait for user start the controller
    ctl_fast_disable_output();

    // Init RC Core
    ctl_sinv_rc_init_t rc_init = {0};
    rc_init.fs = CONTROLLER_FREQUENCY;
    rc_init.freq_grid = 50.0f;
    rc_init.v_base = 311.0f;
    rc_init.i_base = 20.0f;
    rc_init.L_ac = 0.003f;
    rc_init.R_ac = 0.1f;
    ctl_auto_tuning_sinv_rc(&rc_init);
    ctl_init_sinv_rc_core(&rc_core, &rc_init, fdrc_buffer, FDRC_ARRAY_SIZE);

    // init SPWM modulator
    // Max PWM Timer = 2500, Deadband = 50, Current Zero-crossing threshold = 0.5A
    ctl_init_single_phase_H_modulation(&spwm, 2500, 50, float2ctrl(0.5f));
    spwm.flag_enable_dbcomp = 1; // ¿ªÆôËÀÇø²¹³¥

    // Init PLL and PQ
    ctl_single_phase_pll_init_t pll_init = {0};
    pll_init.fs = CONTROLLER_FREQUENCY;
    pll_init.freq_grid = 50.0f;
    pll_init.v_base = 311.0f;
    ctl_auto_tuning_single_phase_pll(&pll_init);
    ctl_init_single_phase_pll(&pll, &pll_init);
    ctl_init_sms_pq(&pq_meter, CONTROLLER_FREQUENCY, 50.0f);
    ctl_init_sinv_ref_gen(&ref_gen, 40.0f, 0.1f);

    // ramp generators for soft-start
    ctl_init_const_slope_f_pu_controller(&rg_p, 0.0f, 1.0f, 1.0f, 1, CONTROLLER_FREQUENCY);
    ctl_init_const_slope_f_pu_controller(&rg_q, 0.0f, 1.0f, 1.0f, 1, CONTROLLER_FREQUENCY);

    // init and config CiA402 standard state machine
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100;

    // init and config Protection module
    ctl_sinv_prot_init_t prot_init = {0};
    prot_init.error_mask =
        SINV_PROT_BIT_HW_TZ | SINV_PROT_BIT_DC_OVP_FAST | SINV_PROT_BIT_AC_OCP_FAST | SINV_PROT_BIT_CTRL_DIVERGE;
    prot_init.warning_mask = SINV_PROT_BIT_AC_OVP_RMS | SINV_PROT_BIT_AC_UVP_RMS | SINV_PROT_BIT_PLL_FREQ_ERR;
    ctl_init_sinv_protect(&protection, &prot_init);

    // init ADC Calibrator
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);
    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }
}

//=================================================================================================
// CTL endless loop routine

void ctl_mainloop(void)
{
    cia402_dispatch(&cia402_sm);

    // Process User commands
    ctl_set_slope_f_pu_target(&rg_p, g_p_ref_user);
    ctl_set_slope_f_pu_target(&rg_q, g_q_ref_user);

    // FDRC Delayed Engagement Logic
    if (cia402_sm.state_word.bits.operation_enabled && cia402_sm.current_state_counter > 100)
    {
        rc_core.flag_enable_fdrc = 1;
    }
    else
    {
        rc_core.flag_enable_fdrc = 0;
    }
}

//=================================================================================================
// CiA402 default callback routine & Background Tasks

gmp_task_status_t tsk_protect(gmp_task_t* tsk)[cite:6]
{
    GMP_UNUSED_VAR(tsk);

    // Execute Slow Protection (RMS, Temp, Grid Freq)
    ctl_task_sinv_protect_slow(&protection, pq_meter.v_rms, pll.frequency,
                               float2ctrl(25.0f), // Mock temp
                               pq_meter.i_rms);

    if (protection.active_errors != 0)
    {
        cia402_fault_request(&cia402_sm);
    }

    return GMP_TASK_DONE;
}

void clear_all_controllers(void)
{
    ctl_clear_single_phase_pll(&pll);
    ctl_clear_sinv_rc_core(&rc_core);
    ctl_clear_sinv_ref_gen(&ref_gen);
    ctl_clear_slope_f_pu(&rg_p);
    ctl_clear_slope_f_pu(&rg_q);
    ctl_clear_single_phase_H_modulation(&spwm);
}

// ADC Calibration State Machine
fast_gt ctl_exec_adc_calibration(void)
{
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {
            if (index_adc_calibrator == 0)
            {
                adc_offset_i_ac += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                ctl_clear_adc_calibrator(&adc_calibrator);
                ctl_enable_adc_calibrator(&adc_calibrator);
            }
            else if (index_adc_calibrator == 1)
            {
                adc_offset_v_grid += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                flag_enable_adc_calibrator = 0;
                clear_all_controllers();
            }
        }
        return 0; // ADC calibrate is not complete
    }
    return 1; // skip calibrate routine
}

void ctl_enable_pwm(void)
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
}
