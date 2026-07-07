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

//=================================================================================================
// include Necessary control modules

#include "ctrl_settings.h"

#include <core/pm/function_scheduler.h>

#include <core/dev/pil_core.h>

#include <ctl/framework/cia402_state_machine.h>

#include <ctl/component/interface/adc_channel.h>

#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/digital_power/dcdc/fsbb.h>

//#include <ctl/component/system/dcdc_protect.h> // 假设有对应的 DCDC 保护模块

#ifndef _FILE_CTRL_MAIN_H_
#define _FILE_CTRL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

//=================================================================================================
// extern controller modules

// System framework
extern cia402_sm_t cia402_sm;

// Control Law Core
extern ctl_dcdc_core_t dcdc_core;

// Input channel
extern adc_channel_t adc_v_in;
extern adc_channel_t adc_v_out;
extern adc_channel_t adc_i_L;
extern adc_channel_t adc_i_load;

// Output channel
extern fsbb_modulator_t fsbb_mod;

// Protection module
//extern ctl_dcdc_protect_t protection;

// ADC Calibrator
extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

// User commands
extern ctrl_gt g_v_out_ref_user;
extern ctrl_gt g_i_limit_user;

extern ctrl_gt v_req;

//=================================================================================================
// function prototype

void ctl_init(void);
void ctl_mainloop(void);

void ctl_enable_pwm(void);
void ctl_disable_pwm(void);
void clear_all_controllers(void);

//=================================================================================================
// Background Controller Tasks

// Protection Tasks
gmp_task_status_t tsk_protect(gmp_task_t* tsk);

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
        if (index_adc_calibrator == 3)
            ctl_step_adc_calibrator(&adc_calibrator, adc_v_in.control_port.value);
        else if (index_adc_calibrator == 2)
            ctl_step_adc_calibrator(&adc_calibrator, adc_v_out.control_port.value);
        if (index_adc_calibrator == 1)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_L.control_port.value);
        else if (index_adc_calibrator == 0)
            ctl_step_adc_calibrator(&adc_calibrator, adc_i_load.control_port.value);
    }
    // normal controller routine
    else
    {
#if (BUILD_LEVEL >= 2)
        // 1. 核心控制解算：输入真实反馈，输出所需的等效电压 V_req
        ctrl_gt v_req = ctl_step_dcdc_core_cascade_generic(&dcdc_core);

        // 2. FSBB 调制器映射：将 V_req 转化为 4 个管子的无缝占空比
        //        ctl_step_fsbb_modulator(&fsbb_mod, v_req, adc_v_in.control_port.value);

#elif (BUILD_LEVEL == 1)
        // 开环固定占空比发波测试
        //        ctl_step_fsbb_modulator(&fsbb_mod, float2ctrl(0.5f) * adc_v_in.control_port.value, adc_v_in.control_port.value);
        ctl_step_fsbb_modulator(&fsbb_mod, v_req, 0.5f);
#endif
    }
}

#ifdef __cplusplus
}
#endif

#endif // _FILE_CTRL_MAIN_H_
