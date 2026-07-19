/** @file ctl_main.c @brief CLLLC/DAB controller initialization and slow path. */
#include <gmp_core.h>
#include "ctl_main.h"

cia402_sm_t cia402_sm;
ctl_dcdc_core_t dcdc_core;
clllc_modulator_t clllc_mod;
adc_channel_t adc_v_primary;
adc_channel_t adc_i_primary;
adc_channel_t adc_v_secondary;
adc_channel_t adc_i_secondary;
adc_channel_t adc_i_resonant;
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 0;
volatile fast_gt index_adc_calibrator = 0;
ctrl_gt g_v_out_ref_user = float2ctrl(CLLLC_VOLTAGE_TARGET_PU);
ctrl_gt g_i_limit_user = float2ctrl(CLLLC_CURRENT_TARGET_PU);
ctrl_gt g_modulation_target_user = float2ctrl(0.10f);
ctrl_gt g_modulation_command = float2ctrl(0.0f);

void ctl_init(void)
{
    ctl_clllc_hardware_t hw = {0};
    ctl_dcdc_core_init_t init_cfg = {0};

    ctl_fast_disable_output();
    hw.fs = CONTROLLER_FREQUENCY;
    hw.f_res_hz = CLLLC_F_RESONANT_HZ;
    hw.f_sw_min_hz = CLLLC_F_MIN_HZ;
    hw.f_sw_max_hz = CLLLC_F_MAX_HZ;
    hw.lm_h = CLLLC_LM_H;
    hw.lr_primary_h = CLLLC_LR_PRIMARY_H;
    hw.lr_secondary_h = CLLLC_LR_SECONDARY_H;
    hw.cr_primary_f = CLLLC_CR_PRIMARY_F;
    hw.cr_secondary_f = CLLLC_CR_SECONDARY_F;
    hw.transformer_ratio = CLLLC_TRANSFORMER_NS_NP;
    hw.c_out_f = CLLLC_COUT_F;
    hw.r_load_min_ohm = CLLLC_RLOAD_MIN_OHM;
    hw.tank_esr_ohm = CLLLC_TANK_ESR_OHM;
    hw.v_base = CTRL_VOLTAGE_BASE;
    hw.i_base = CTRL_CURRENT_BASE;
    hw.slope_v_pu_s = CLLLC_VOLTAGE_SLOPE_PU_S;
    hw.slope_i_pu_s = CLLLC_CURRENT_SLOPE_PU_S;
    hw.fc_current_loop = CLLLC_CURRENT_LOOP_BW_HZ;
    hw.fc_voltage_loop = CLLLC_VOLTAGE_LOOP_BW_HZ;
    hw.i_limit_max = float2ctrl(1.0f);
    hw.i_limit_min = -float2ctrl(1.0f);
    hw.modulation_max = float2ctrl(1.0f);
    hw.modulation_min = -float2ctrl(1.0f);

    ctl_dcdc_blueprint_clllc_parallel(&init_cfg, &hw);
    ctl_init_dcdc_core(&dcdc_core, &init_cfg);
    ctl_attach_dcdc_core(&dcdc_core, &adc_v_primary.control_port,
                         &adc_v_secondary.control_port,
                         &adc_i_resonant.control_port,
                         &adc_i_primary.control_port);
    dcdc_core.v_target = g_v_out_ref_user;
    dcdc_core.i_target = g_i_limit_user;
    dcdc_core.mode = (BUILD_LEVEL == 1) ? CTL_DCDC_MODE_OPENLOOP :
                     ((BUILD_LEVEL == 2) ? CTL_DCDC_MODE_CURRENTLOOP : CTL_DCDC_MODE_VOLTAGELOOP);

    ctl_init_clllc_modulator(&clllc_mod, CLLLC_TIMER_CLOCK_HZ,
                             CLLLC_F_RESONANT_HZ, CLLLC_F_MIN_HZ,
                             CLLLC_F_MAX_HZ, CLLLC_DEADBAND_S,
                             float2ctrl(CLLLC_MAX_PHASE_SHIFT_PU));
    init_cia402_state_machine(&cia402_sm);
    /* Keep the complete automatic commissioning sequence configurable.
       Hardware uses the conservative 100 ms default per stage; SIL reduces
       it to 2 ms so a short switching simulation still traverses all states. */
    cia402_sm.minimum_transit_delay[0] = CTRL_STARTUP_DELAY_MS;
    cia402_sm.minimum_transit_delay[1] = CTRL_STARTUP_DELAY_MS;
    cia402_sm.minimum_transit_delay[2] = CTRL_STARTUP_DELAY_MS;
    cia402_sm.minimum_transit_delay[3] = CTRL_STARTUP_DELAY_MS;
    ctl_init_adc_calibrator(&adc_calibrator, 20.0f, 0.707f, CONTROLLER_FREQUENCY);
}

void ctl_mainloop(void)
{
    dcdc_core.v_target = g_v_out_ref_user;
    dcdc_core.i_target = g_i_limit_user;
    cia402_dispatch(&cia402_sm);
}

void clear_all_controllers(void) { ctl_clear_dcdc_core(&dcdc_core); }
void ctl_enable_pwm(void) { clear_all_controllers(); ctl_fast_enable_output(); }
void ctl_disable_pwm(void) { ctl_fast_disable_output(); }

fast_gt ctl_exec_adc_calibration(void)
{
    /* Voltage channels are unipolar and must not be zero-offset calibrated.
       The TMCS1133 current bias is calibrated from the disabled bridge. */
    if (!flag_enable_adc_calibrator)
        return 1;
    if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) &&
        ctl_is_adc_calibrator_result_valid(&adc_calibrator))
    {
        adc_i_primary.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
        adc_i_secondary.bias = adc_i_primary.bias;
        adc_i_resonant.bias = adc_i_primary.bias;
        flag_enable_adc_calibrator = 0;
        clear_all_controllers();
        return 1;
    }
    return 0;
}

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);
    return GMP_TASK_DONE;
}

void gmp_pil_sim_step(const gmp_sim_rx_buf_t* rx, gmp_sim_tx_buf_t* tx)
{
#if defined ENABLE_GMP_DL_PIL_SIM
    ctl_input_callback_pil(rx);
    ctl_dispatch();
    ctl_output_callback_pil(tx);
#else
    GMP_UNUSED_VAR(rx); GMP_UNUSED_VAR(tx);
#endif
}
