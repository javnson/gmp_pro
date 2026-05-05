/**
 * @file ctrl_main.c
 * @author GMP Library Contributors
 * @brief Top-level implementation for the Four-Switch Buck-Boost (FSBB) Converter.
 */

#include "ctrl_main.h"

//=================================================================================================
// 1. 全局变量定义与实例化 (Global Instantiation)

cia402_sm_t cia402_sm;
ctl_dcdc_protect_t protection;

ctl_dcdc_core_t dcdc_core;
fsbb_modulator_t fsbb_mod;

// 用户指令 (默认启动输出 0V)
ctrl_gt g_v_out_ref_user = float2ctrl(0.0f);
ctrl_gt g_i_limit_user = float2ctrl(10.0f); // 默认限制 10A 标幺值

volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// ADC 物理通道
adc_channel_t adc_v_in;
adc_channel_t adc_v_out;
adc_channel_t adc_i_L;
adc_channel_t adc_i_load;

//=================================================================================================
// 2. 初始化程序 (Initialization)

void ctl_init(void)
{
    // 启动前强制封锁 PWM，确保安全
    ctl_fast_disable_output();

    // --- 2.1 FSBB 双环内核自整定与初始化 ---

    ctl_dcdc_core_init_t core_init = {0};

    // 填入硬件物理参数 (从 ctrl_settings.h 的宏获取)
    core_init.fs = CONTROLLER_FREQUENCY;
    core_init.L_main = 22.0e-6f; // 22uH 主电感
    core_init.C_out = 470.0e-6f; // 470uF 输出电容
    core_init.r_L = 0.015f;      // 15mOhm 电感 ESR (极零对消)
    core_init.r_C = 0.005f;      // 5mOhm 电容 ESR

    // 填入运行工况 (用于推导最恶劣的 RHPZ 并压制外环带宽)
    core_init.v_in_nom = 48.0f;  // 典型输入 48V
    core_init.v_in_min = 18.0f;  // 最小输入 18V (最深 Boost 模式)
    core_init.v_out_nom = 60.0f; // 典型输出 60V
    core_init.i_out_max = 20.0f; // 最大负载电流 20A

    // 系统基准值 (标幺化)
    core_init.v_base = CTRL_VOLTAGE_BASE;
    core_init.i_base = CTRL_CURRENT_BASE;

    // 安全物理极限
    core_init.i_L_max = 25.0f;   // 绝对最大电感电流 25A
    core_init.i_L_min = -5.0f;   // 允许轻微反向电流 (适用于同步整流轻载)
    core_init.v_req_max = 80.0f; // 最大等效调制电压 80V
    core_init.v_req_min = 0.0f;  // 最小调制电压 0V

    // 执行参数自动整定 (SI 转 PU，带 RHPZ 压制保护)
    ctl_auto_tuning_dcdc_fsbb(&core_init);

    // 初始化内核：配置电压爬坡斜率(如 50V/s) 和 电流斜率(如 500A/s)
    ctl_init_dcdc_core(&dcdc_core, &core_init, float2ctrl(50.0f / CTRL_VOLTAGE_BASE),
                       float2ctrl(500.0f / CTRL_CURRENT_BASE));

    // 绑定物理 ADC 接口 (零拷贝)
    ctl_attach_dcdc_core(&dcdc_core, &adc_v_in.control_port, &adc_v_out.control_port, &adc_i_L.control_port, NULL);

    // --- 2.2 FSBB 调制器初始化 ---

    // 设定自举电容安全占空比 [0.05, 0.95]
    // 设定平滑过渡区 m_low = 0.9, m_high = 1.1 (即 Vin 的 ±10% 区间内四管齐动)
    ctl_init_fsbb_modulator(&fsbb_mod, CTRL_PWM_CMP_MAX, float2ctrl(0.95f), float2ctrl(0.05f), float2ctrl(0.90f),
                            float2ctrl(1.10f));

    // --- 2.3 保护与状态机初始化 ---

    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 20; // 延时 20ms 后切入运行

    ctl_dcdc_prot_init_t prot_init = {0};
    prot_init.v_in_max = 75.0f;
    prot_init.v_out_max = 70.0f;
    prot_init.i_L_max = 28.0f; // 硬件快保护阈值 > 控制器限幅 25A
    ctl_init_dcdc_protect(&protection, &prot_init);
}

//=================================================================================================
// 3. 后台主循环 (Background Task)

void ctl_mainloop(void)
{
    // 状态机核心调度
    cia402_dispatch(&cia402_sm);

    // --- 用户参数下发 ---
    // 仅在设备处于运行状态时，更新核心目标值
    if (cia402_sm.state_word.bits.operation_enabled)
    {
        dcdc_core.v_out_set_user = g_v_out_ref_user;
    }
    else
    {
        // 停机状态下，目标电压归零
        dcdc_core.v_out_set_user = float2ctrl(0.0f);
    }

    // --- BUILD_LEVEL 分级测试 ---
#if (BUILD_LEVEL == 1)
    // Level 1: 纯开环发波测试 (验证调制器死区与极性)
    // 强制内核输出固定调制电压 (例如输入的一半，纯 Buck)
    dcdc_core.flag_enable = 0;
#endif

#if (BUILD_LEVEL == 2)
    // Level 2: 闭环运行 (无负载前馈)
    dcdc_core.flag_enable = 1;
    dcdc_core.flag_enable_load_ff = 0;
#endif

#if (BUILD_LEVEL == 3)
    // Level 3: 高性能全开 (启用负载电流前馈，应对剧烈负载跳变)
    dcdc_core.flag_enable = 1;
    dcdc_core.flag_enable_load_ff = 1;
#endif
}

//=================================================================================================
// 4. 保护与回调接口

gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // 执行慢保护检测 (如平均功率限制、过温保护)
    // ... (依据硬件具体配置)

    if (protection.active_errors != 0)
    {
        cia402_fault_request(&cia402_sm);
    }
    return GMP_TASK_DONE;
}

void clear_all_controllers(void)
{
    // 复位内核积分器，防止上次停机残留的积分项导致启动浪涌
    ctl_clear_dcdc_core(&dcdc_core);
}

void ctl_enable_pwm(void)
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
}
