//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
// User should implement the Main ISR of the controller tasks.
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#ifndef _FILE_XPLT_CTL_INTERFACE_H_
#define _FILE_XPLT_CTL_INTERFACE_H_

#include <xplt.peripheral.h>

#include "ctrl_main.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// 1. Controller Hardware Input Callback (读外设)

/**
 * @brief 在控制算法执行前调用，负责从硬件寄存器读取原始 ADC 值并标幺化
 */
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // 从 DSP 的 ADC 结果寄存器中读取原始值 (0~4095)
    // 这里的 ADCARESULT_BASE 等宏请根据您的实际硬件引脚分配进行替换

    ctl_step_adc_channel(&adc_v_in,   ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0));
    ctl_step_adc_channel(&adc_v_out,  ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0));
    ctl_step_adc_channel(&adc_i_L,    ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0));
    ctl_step_adc_channel(&adc_i_load, ADC_readResult(ADCDRESULT_BASE, ADC_SOC_NUMBER0));
}

//=================================================================================================
// 2. Controller Dispatch (算控制律)

/**
 * @brief 控制算法的核心步进逻辑 (20kHz)
 * @details 负责处理 ADC 校准、双闭环运算、以及 FSBB 占空比映射
 */
GMP_STATIC_INLINE void ctl_dispatch(void)
{
    // 如果系统正在执行 ADC 零点校准，则阻塞主控制流
    if (flag_enable_adc_calibrator)
    {
        // 调用定义在 ctrl_main.c 中的校准状态机
        // ctl_exec_adc_calibration();
        return;
    }

#if (BUILD_LEVEL == 1)
    // --- Level 1: 硬件验证 (开环) ---
    // 强制输出 V_req = 0.5 * V_in，验证 Buck 桥臂是否输出 50% 占空比
    ctrl_gt v_req_openloop = ctl_mul(adc_v_in.control_port.value, float2ctrl(0.5f));
    ctl_step_fsbb_modulator(&fsbb_mod, v_req_openloop, adc_v_in.control_port.value);

#elif (BUILD_LEVEL >= 2)
    // --- Level 2 & 3: 闭环运行 ---
    if (cia402_sm.state_word.bits.operation_enabled)
    {
        // 1. 执行 FSBB 统一双环内核，计算所需的物理节点电压 V_req
        ctrl_gt v_req = ctl_step_dcdc_fsbb(&dcdc_core);

        // 2. 将 V_req 丢给调制器，自动映射出 Buck 和 Boost 的无缝占空比
        ctl_step_fsbb_modulator(&fsbb_mod, v_req, adc_v_in.control_port.value);
    }
    else
    {
        // 停机状态：强行令目标电压为 0，调制器输出最低安全占空比
        ctl_step_fsbb_modulator(&fsbb_mod, float2ctrl(0.0f), adc_v_in.control_port.value);
    }
#endif
}

//=================================================================================================
// 3. Controller Hardware Output Callback (写外设)

/**
 * @brief 在控制算法执行后调用，负责将计算结果写入 PWM 和 DAC 寄存器
 */
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // 1. 写入 Buck 桥臂 ePWM (降压管)
    EPWM_setCounterCompareValue(PHASE_BUCK_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_buck_cmp(&fsbb_mod));

    // 2. 写入 Boost 桥臂 ePWM (升压管)
    EPWM_setCounterCompareValue(PHASE_BOOST_BASE, EPWM_COUNTER_COMPARE_A, ctl_get_fsbb_boost_cmp(&fsbb_mod));

    // 3. 调试 DAC 输出 (将内部标幺值转为 12-bit DAC 数据，带有 2048 中心偏置)
#if BUILD_LEVEL >= 1
    // 观测实际输出电压与目标电压的跟踪情况
    DAC_setShadowValue(IRIS_DACA_BASE, (uint16_t)(adc_v_out.control_port.value * 2048.0f + 2048.0f));
    DAC_setShadowValue(IRIS_DACB_BASE, (uint16_t)(dcdc_core.v_out_ref * 2048.0f + 2048.0f));
#endif // BUILD_LEVEL
}

//=================================================================================================
// 4. Hardware Enable / Disable Actions

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

/**
 * @brief 硬件级快开使能
 */
GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    // 清除 PWM 模块的硬件封锁(TZ)标志
    EPWM_clearTripZoneFlag(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // 清理历史积分器，防止启动浪涌
    clear_all_controllers();

    // 拉高硬件驱动芯片的 EN 引脚
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    // 点亮运行指示灯
    GPIO_WritePin(CONTROLLER_LED, 0);
}

/**
 * @brief 硬件级快关封锁
 */
GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    // 强制触发 PWM 的 TZ(Trip Zone)，瞬间全关断，硬件级无延迟
    EPWM_forceTripZoneEvent(PHASE_BUCK_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_BOOST_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // 拉低驱动芯片的 EN 引脚
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    // 熄灭运行指示灯
    GPIO_WritePin(CONTROLLER_LED, 1);
}

//=================================================================================================
// 5. PIL (Processor-in-the-Loop) Simulation Interfaces

typedef enum _tag_dcdc_adc_index_items
{
    DCDC_ADC_ID_VIN   = 0,
    DCDC_ADC_ID_VOUT  = 1,
    DCDC_ADC_ID_IL    = 2,
    DCDC_ADC_ID_ILOAD = 3,

    DCDC_ADC_SENSOR_NUMBER = 4
} dcdc_adc_index_items;

/**
 * @brief PIL 仿真环境的输入注入
 */
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // 将仿真器发来的数字注入到 ADC 通道中
    ctl_step_adc_channel(&adc_v_in,   rx->adc_result[DCDC_ADC_ID_VIN]);
    ctl_step_adc_channel(&adc_v_out,  rx->adc_result[DCDC_ADC_ID_VOUT]);
    ctl_step_adc_channel(&adc_i_L,    rx->adc_result[DCDC_ADC_ID_IL]);
    ctl_step_adc_channel(&adc_i_load, rx->adc_result[DCDC_ADC_ID_ILOAD]);
}

/**
 * @brief PIL 仿真环境的输出抓取
 */
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    // 将计算好的占空比发回仿真器
    tx->pwm_cmp[0] = ctl_get_fsbb_buck_cmp(&fsbb_mod);
    tx->pwm_cmp[1] = ctl_get_fsbb_boost_cmp(&fsbb_mod);

    // 监测信号发回仿真示波器
    tx->monitor[0] = adc_v_out.control_port.value;
    tx->monitor[1] = dcdc_core.v_out_ref;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_XPLT_CTL_INTERFACE_H_
