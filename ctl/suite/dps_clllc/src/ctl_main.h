/** @file ctl_main.h @brief CLLLC/DAB controller top level. */
#ifndef DPS_CLLLC_CTL_MAIN_H
#define DPS_CLLLC_CTL_MAIN_H

#include "ctrl_settings.h"
#include <core/pm/function_scheduler.h>
#include <core/dev/pil_core.h>
#include <ctl/framework/cia402_state_machine.h>
#include <ctl/component/interface/adc_channel.h>
#include <ctl/component/digital_power/dcdc/clllc.h>

#ifdef __cplusplus
extern "C" {
#endif

extern cia402_sm_t cia402_sm;
extern ctl_dcdc_core_t dcdc_core;
extern clllc_modulator_t clllc_mod;

extern adc_channel_t adc_v_primary;
extern adc_channel_t adc_i_primary;
extern adc_channel_t adc_v_secondary;
extern adc_channel_t adc_i_secondary;
extern adc_channel_t adc_i_resonant;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;
extern ctrl_gt g_v_out_ref_user;
extern ctrl_gt g_i_limit_user;
extern ctrl_gt g_modulation_command;

void ctl_init(void);
void ctl_mainloop(void);
void ctl_enable_pwm(void);
void ctl_disable_pwm(void);
void clear_all_controllers(void);
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

GMP_STATIC_INLINE void ctl_dispatch(void)
{
    ctrl_gt command;

    /* The hardware has no dedicated cavity-current MCU connection.  Primary
       bridge current is the safe hardware fallback; SIL supplies the separate
       resonant-current channel through its input callback. */
#if !defined SPECIFY_PC_ENVIRONMENT
    adc_i_resonant.control_port.value = adc_i_primary.control_port.value;
#endif

#if (BUILD_LEVEL == 1)
    ctl_dcdc_internal_ingest_and_filter(&dcdc_core);
    command = float2ctrl(CLLLC_OPEN_LOOP_COMMAND_PU);
#elif (BUILD_LEVEL == 2)
    command = ctl_step_dcdc_output_current_loop(&dcdc_core);
#else
    command = ctl_step_dcdc_parallel(&dcdc_core);
#endif
    g_modulation_command = command;
    ctl_step_clllc_modulator(&clllc_mod, command);
}

#ifdef __cplusplus
}
#endif
#endif
