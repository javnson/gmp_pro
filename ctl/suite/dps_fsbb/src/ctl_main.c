/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief Top-level implementation for the Four-Switch Buck-Boost (FSBB) Converter.
 * @details Implements a dual-loop PU controller with smooth mode transitions.
 */

#include "ctrl_main.h"
#include <gmp_core.h>

//=================================================================================================
// 1. 全局变量定义与实例化 (Global Instantiation)

// 系统框架与保护
cia402_sm_t cia402_sm;
ctl_dcdc_protect_t protection;

// 算法核心与调制器
ctl_dcdc_core_t dcdc_core;
fsbb_modulator_t fsbb_mod;

// 用户指令与设定值 (标幺化值)
ctrl_gt g_v_out_ref_user = float2ctrl(0.0f);
ctrl_gt g_i_limit_user = float2ctrl(0.5f); // 默认限制 0.5 PU (约 15A)

// 标志位
volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// ADC 物理通道实例
adc_channel_t adc_v_in;
adc_channel_t adc_v_out;
adc_channel_t adc_i_L;
adc_channel_t adc_i_load;

// ADC 偏置校准器
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
volatile fast_gt index_adc_calibrator = 0;

//=================================================================================================
// 2. 初始化程序 (Initialization)

void ctl_init(void)
{
    // 初始状态强制封锁 PWM
    ctl_fast_disable_output();

    // --- 2.1 FSBB 算法核心自整定 ---

    ctl_dcdc_core_init_t core_init = {0};

    // 配置物理参数 (SI Units)
    core_init.v_base = CTRL_VOLTAGE_BASE; // 80V
    core_init.i_base = CTRL_CURRENT_BASE; // 30A
    core_init.fs = CONTROLLER_FREQUENCY;
    core_init.L_main = 22.0e-6f; // 22uH
    core_init.C_out = 470.0e-6f; // 470uF
    core_init.r_L = 0.015f;      // 15mR ESR
    core_init.r_C = 0.005f;      // 5mR ESR

    // 运行点配置 (用于计算 RHPZ 安全带宽)
    core_init.v_in_nom = CTRL_VIN_NOM; // 24V
    core_init.v_in_min = 12.0f;        // 最深 Boost 模式下的输入
    core_init.v_out_nom = 48.0f;
    core_init.i_out_max = 15.0f;

    // 保护限幅配置 (SI 转 PU)
    core_init.i_L_max = 25.0f;   // 25A 峰值限流
    core_init.i_L_min = -2.0f;   // 允许微量反向电流
    core_init.v_req_max = 75.0f; // 调制电压上限
    core_init.v_req_min = 0.0f;

    // 执行针对 FSBB 的自动整定 (自动压制 RHPZ 下的电压环带宽)
    ctl_auto_tuning_dcdc_fsbb(&core_init);

    // 初始化内核 (注入斜率: 电压 50V/s, 电感电流 500A/s)
    ctl_init_dcdc_core(&dcdc_core, &core_init, float2ctrl(50.0f / CTRL_VOLTAGE_BASE),
                       float2ctrl(500.0f / CTRL_CURRENT_BASE));

    // 绑定 ADC 指针 (零拷贝接入)
    ctl_attach_dcdc_core(&dcdc_core, &adc_v_in.control_port, &adc_v_out.control_port, &adc_i_L.control_port,
                         &adc_i_load.control_port);

    // --- 2.2 调制器初始化 (带过渡区) ---

    // 配置占空比限制 [0.05, 0.95] 保证自举电容充电
    // 过渡区设置在 Vin 的 90% 到 110% 之间
    ctl_init_fsbb_modulator(&fsbb_mod, CTRL_PWM_CMP_MAX, float2ctrl(0.95f), float2ctrl(0.05f), float2ctrl(0.90f),
                            float2ctrl(1.10f));

    // --- 2.3 系统框架初始化 ---

    // CiA 402 状态机
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // 稳定运行 100ms 后才正式使能指令

    // 保护模块
    ctl_dcdc_prot_init_t prot_init = {0};
    prot_init.v_out_max = CTRL_PROT_VOUT_MAX;
    prot_init.i_L_max = CTRL_PROT_IL_MAX;
    ctl_init_dcdc_protect(&protection, &prot_init);

    // ADC 校准器 (20个周期平均，0.707置信度)
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);
    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }
}

//=================================================================================================
// 3. 后台主循环 (Low-Frequency Logic)

void ctl_mainloop(void)
{
    // 调度 CiA402 状态机
    cia402_dispatch(&cia402_sm);

    // 根据状态机决定输出目标
    if (cia402_sm.state_word.bits.operation_enabled)
    {
        // 允许运行：下发用户指令，并限制电感电流参考值
        dcdc_core.v_out_set_user = g_v_out_ref_user;
        ctl_set_pid_limit(&dcdc_core.v_loop_pi, g_i_limit_user, -g_i_limit_user);
    }
    else
    {
        // 待机或故障：目标清零
        dcdc_core.v_out_set_user = float2ctrl(0.0f);
    }

    // --- BUILD_LEVEL 动态管理 ---
#if (BUILD_LEVEL == 1)
    dcdc_core.flag_enable = 0; // 内核不运行，依靠 interface 中的开环逻辑
#else
    dcdc_core.flag_enable = 1;
#endif

#if (BUILD_LEVEL == 3)
    dcdc_core.flag_enable_load_ff = 1; // 开启负载电流前馈，应对动态跳变
#else
    dcdc_core.flag_enable_load_ff = 0;
#endif
}

//=================================================================================================
// 4. 回调函数与任务实现

/**
 * @brief 处理 ADC 校准的自动流程
 */
fast_gt ctl_exec_adc_calibration(void)
{
    if (!flag_enable_adc_calibrator)
        return 1;

    if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
    {
        // 顺序校准：电感电流 -> 输出电压 -> 输入电压
        if (index_adc_calibrator == 0)
        {
            adc_i_L.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else if (index_adc_calibrator == 1)
        {
            adc_v_out.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            index_adc_calibrator++;
            ctl_clear_adc_calibrator(&adc_calibrator);
            ctl_enable_adc_calibrator(&adc_calibrator);
        }
        else
        {
            adc_v_in.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
            flag_enable_adc_calibrator = 0; // 校准结束
            clear_all_controllers();
        }
    }
    return 0;
}

/**
 * @brief 保护监控任务 (10ms 周期)
 */
gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // 监控慢速保护量（如过热、平均过载）
    if (protection.active_errors != 0)
    {
        cia402_fault_request(&cia402_sm);
    }
    return GMP_TASK_DONE;
}

void clear_all_controllers(void)
{
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
