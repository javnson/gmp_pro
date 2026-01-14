/**
 * @file cia402_state_machine.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 0.1
 * @date 2026-01-14
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#include <gmp_core.h>

#include <ctl/framework/cia402_state_machine.h>

#ifndef _MSC_VER

void ctl_enable_pwm();
void ctl_disable_pwm();
void cel_enable_main_contactor();
void cel_disable_main_contactor();
void ctl_enable_precharge_relay();
void ctl_disable_precharge_relay();
fast_gt ctl_exec_adc_calibration(void);
fast_gt ctl_exec_dc_voltage_ready(void);
void ctl_release_brake();
void ctl_restore_brake();
fast_gt ctl_check_encoder(void);
fast_gt ctl_exec_rotor_alignment(void);
fast_gt ctl_check_pll_locked(void);

    // =============================================================
// 1. 功率级执行 (Power Stage & Actuators)
// =============================================================

/**
 * @brief 硬件PWM输出使能/禁止
 * @note 在 Operation Enabled 状态下为 true，其他状态为 false
 * @param enable true: 开启PWM驱动; false: 封锁PWM（高阻态或特定电平）
 */
GMP_WEAK_FUNC_PREFIX
void ctl_enable_pwm() GMP_WEAK_FUNC_SUFFIX
{
    // Default: Do nothing
}

GMP_WEAK_FUNC_PREFIX
void ctl_disable_pwm() GMP_WEAK_FUNC_SUFFIX
{
    // Default: Do nothing
}

/**
 * @brief 主接触器/直流继电器控制
 * @note 通常在 Ready to Switch On 阶段闭合
 * @param close true: 吸合; false: 断开
 */
GMP_WEAK_FUNC_PREFIX
void cel_enable_main_contactor() GMP_WEAK_FUNC_SUFFIX
{
}

GMP_WEAK_FUNC_PREFIX
void cel_disable_main_contactor() GMP_WEAK_FUNC_SUFFIX
{
}

/**
 * @brief 预充电继电器控制
 * @note 在 Switch On Disabled -> Ready to Switch On 过渡期间使用
 */
GMP_WEAK_FUNC_PREFIX
void ctl_enable_precharge_relay() GMP_WEAK_FUNC_SUFFIX
{
}

GMP_WEAK_FUNC_PREFIX
void ctl_disable_precharge_relay() GMP_WEAK_FUNC_SUFFIX
{
}

// =============================================================
// 2. 采样与校准 (Sensing & Calibration)
// =============================================================

/**
 * @brief 执行ADC偏置校准
 * @note 通常在 Not Ready 或 Switch On Disabled 状态下调用
 * @return true: 校准完成且成功; false: 失败或正在进行中
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_exec_adc_calibration(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1; // 默认直接返回成功
}

/**
 * @brief 检查直流母线电压是否在允许范围内
 * @note 用于 Ready to Switch On 的准入条件
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_exec_dc_voltage_ready(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1;
}

// =============================================================
// 3. 机械制动 (Mechanical Brake)
// =============================================================

/**
 * @brief 机械抱闸控制 (Holding Brake)
 * @note 电机场景特有。通常 Operation Enabled 时 release (true)，否则 engage (false)
 * @param release true: 松开抱闸(允许转动); false: 抱死(制动)
 */
GMP_WEAK_FUNC_PREFIX
void ctl_release_brake() GMP_WEAK_FUNC_SUFFIX
{
}

GMP_WEAK_FUNC_PREFIX
void ctl_restore_brake() GMP_WEAK_FUNC_SUFFIX
{
}

// =============================================================
// 4. 编码器与对齐 (Position & Alignment)
// =============================================================

/**
 * @brief 检查编码器/位置传感器状态
 * @note 在 Ready to Switch On 之前必须通过
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_check_encoder(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1;
}

/**
 * @brief 启动转子初始位置检测 (IPD / Alignment)
 * @note 针对同步电机。通常在 Switched On -> Operation Enabled 瞬间触发
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_exec_rotor_alignment(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1;
}

// =============================================================
// 5. 电网同步 (Grid Synchronization)
// =============================================================

/**
 * @brief 启动/复位锁相环 (PLL)
 * @note 在 Ready to Switch On 状态下必须启动 PLL
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_check_pll_locked(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1;
}

/**
 * @brief 检查电网电压/频率是否符合安规 (Grid Code)
 * @note 整个运行周期都需要检查
 */
GMP_WEAK_FUNC_PREFIX
fast_gt ctl_check_compliance(void) GMP_WEAK_FUNC_SUFFIX
{
    return 1;
}

// =============================================================
// 6. 交流侧操作 (AC Side Operations)
// =============================================================

/**
 * @brief 交流并网继电器/断路器控制
 * @note 在 Ready -> Switched On 跳转时闭合
 * @param close true: 并网; false: 解列
 */
GMP_WEAK_FUNC_PREFIX
void ctl_enable_grid_relay() GMP_WEAK_FUNC_SUFFIX
{
}

GMP_WEAK_FUNC_PREFIX
void ctl_disable_grid_relay() GMP_WEAK_FUNC_SUFFIX
{
}

#else // in MSVC environment

#pragma comment(linker, "/alternatename:ctl_enable_pwm=default_ctl_void_void_func")
#pragma comment(linker, "/alternatename:ctl_disable_pwm=default_ctl_void_void_func")

#pragma comment(linker, "/alternatename:cel_enable_main_contactor=default_ctl_void_void_func")
#pragma comment(linker, "/alternatename:cel_disable_main_contactor=default_ctl_void_void_func")

#pragma comment(linker, "/alternatename:ctl_enable_precharge_relay=default_ctl_void_void_func")
#pragma comment(linker, "/alternatename:ctl_disable_precharge_relay=default_ctl_void_void_func")

#pragma comment(linker, "/alternatename:ctl_exec_adc_calibration=default_ctl_fast_gt_void_func_with1_ret")
#pragma comment(linker, "/alternatename:ctl_exec_dc_voltage_ready=default_ctl_fast_gt_void_func_with1_ret")

#pragma comment(linker, "/alternatename:ctl_release_brake=default_ctl_void_void_func")
#pragma comment(linker, "/alternatename:ctl_restore_brake=default_ctl_void_void_func")

#pragma comment(linker, "/alternatename:ctl_check_encoder=default_ctl_fast_gt_void_func_with1_ret")
#pragma comment(linker, "/alternatename:ctl_exec_rotor_alignment=default_ctl_fast_gt_void_func_with1_ret")

#pragma comment(linker, "/alternatename:ctl_check_pll_locked=default_ctl_fast_gt_void_func_with1_ret")
#pragma comment(linker, "/alternatename:ctl_check_compliance=default_ctl_fast_gt_void_func_with1_ret")

#pragma comment(linker, "/alternatename:ctl_enable_grid_relay=default_ctl_void_void_func")
#pragma comment(linker, "/alternatename:ctl_disable_grid_relay=default_ctl_void_void_func")

void default_ctl_void_void_func(void)
{
    // not implement
}

fast_gt default_ctl_fast_gt_void_func_with1_ret(void)
{
    return 1; // 默认直接返回成功
}

#endif // _MSC_VER
