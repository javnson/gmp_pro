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

#ifndef _FILE_CTRL_MAIN_H_
#define _FILE_CTRL_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "ctrl_settings.h"

#include <core/pm/function_scheduler.h>

#include <ctl/component/digital_power/dcdc/dcdc_core.h>
#include <ctl/component/digital_power/dcdc/fsbb.h>
//#include <ctl/component/digital_power/dcdc/dcdc_modulator.h>

// 引入框架组件
#include <ctl/component/interface/adc_channel.h>
#include <ctl/framework/cia402_state_machine.h>
//#include <ctl/component/system/dcdc_protect.h> // 假设有对应的 DCDC 保护模块

#include <core/dev/pil_core.h>

//=================================================================================================
// 全局变量声明 (Global Variables Export)

// 1. 系统框架模块
extern cia402_sm_t cia402_sm;
//extern ctl_dcdc_protect_t protection;

// 2. 算法核心与调制器
extern ctl_dcdc_core_t dcdc_core;
extern fsbb_modulator_t fsbb_mod;

// 3. 用户设定值 (User Setpoints)
extern ctrl_gt g_v_out_ref_user;
extern ctrl_gt g_i_limit_user;

// 4. ADC 采样通道 (物理接口)
extern adc_channel_t adc_v_in;
extern adc_channel_t adc_v_out;
extern adc_channel_t adc_i_L;
extern adc_channel_t adc_i_load; // 可选的负载电流前馈

// 5. 状态标志
extern volatile fast_gt flag_system_running;
extern volatile fast_gt flag_error;

extern adc_bias_calibrator_t adc_calibrator;
extern volatile fast_gt flag_enable_adc_calibrator;
extern volatile fast_gt index_adc_calibrator;

extern ctrl_gt v_req;

//=================================================================================================
// 核心 API 声明 (Core API)

/**
 * @brief 初始化整个 FSBB 系统的控制参数与底层绑定
 */
void ctl_init(void);

/**
 * @brief 系统的慢速后台主循环 (处理状态机、参数下发)
 */
void ctl_mainloop(void);

/**
 * @brief 硬件发波使能动作
 */
void ctl_enable_pwm(void);

/**
 * @brief 硬件发波封锁动作
 */
void ctl_disable_pwm(void);

/**
 * @brief 清理所有控制器的历史积分状态，防止启动冲击
 */
void clear_all_controllers(void);

//=================================================================================================
// 后台任务声明 (Background Tasks)

/**
 * @brief 慢速保护与监控任务 (如过温、均值过载保护)
 */
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
