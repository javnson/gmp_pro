/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief 20kW Single-Phase Inverter/AFE Top-level Implementation.
 */

#include "ctl_main.h"

//=================================================================================================
// 1. 全局变量定义与实例化 (Flat Architecture)

// 系统框架模块
cia402_sm_t cia402_sm;
ctl_sinv_protect_t protection;
single_phase_H_modulation_t hpwm;

// 算法核心模块
spll_sogi_t pll;
ctl_sms_pq_t pq_meter;
ctl_sinv_ref_gen_t ref_gen;
ctl_sinv_rc_core_t rc_core;

// 用户与系统交互变量
ctrl_gt g_p_ref_user = float2ctrl(0.0f);
ctrl_gt g_q_ref_user = float2ctrl(0.0f);
volatile fast_gt flag_system_running = 0;
volatile fast_gt flag_error = 0;

// ADC 采样通道定义
adc_channel_t adc_v_grid;
adc_channel_t adc_i_ac;
adc_channel_t adc_v_bus;

// ADC 校准标志
adc_bias_calibrator_t adc_calibrator;
volatile fast_gt flag_enable_adc_calibrator = 1;
volatile fast_gt index_adc_calibrator = 0;

// 定时器比较值缓存
pwm_gt pwm_cmp_L = 0;
pwm_gt pwm_cmp_N = 0;

// FDRC 重复控制器静态内存
#define CONTROLLER_FREQUENCY (20000.0f)
#define FDRC_ARRAY_SIZE      (400) // 20kHz / 50Hz = 400 points
static ctrl_gt fdrc_buffer[FDRC_ARRAY_SIZE];

//=================================================================================================
// 2. 状态机响应函数：锁相环状态检查

/**
 * @brief 检测 PLL 是否满足并网收敛要求
 * @return 1: 锁相成功且稳定; 0: 锁定中或频率异常
 */
fast_gt ctl_check_pll_locked(void)
{
    // 准入条件：
    // 1. 电网电压幅值在 0.8pu ~ 1.2pu 之间 (防止断路器未闭合或严重欠压)
    // 2. PLL 内部频率误差趋于 0 (freq_error 接近 0)
    ctrl_gt v_mag_pu = pll.v_mag;
    ctrl_gt f_err_abs = ctl_abs(pll.freq_error);

    if ((v_mag_pu > float2ctrl(0.8f)) && (v_mag_pu < float2ctrl(1.2f)))
    {
        if (f_err_abs < float2ctrl(0.005f)) // 频率偏差小于 0.25Hz (对于50Hz)
        {
            return 1;
        }
    }
    return 0;
}

//=================================================================================================
// 3. 初始化程序

void ctl_init(void)
{
    // 停止 PWM 输出，进入高阻态安全状态
    ctl_fast_disable_output();

    // --- 3.1 ADC 物理通道初始化 (Gain, Bias, Resolution, IQN) ---
    // 假设 Vbus 满量程 600V, Vac 满量程 450V, Iac 满量程 50A
    ctl_init_adc_channel(&adc_v_bus, float2ctrl(600.0f), float2ctrl(0.0f), 12, 24);
    ctl_init_adc_channel(&adc_v_grid, float2ctrl(450.0f), float2ctrl(0.0f), 12, 24);
    ctl_init_adc_channel(&adc_i_ac, float2ctrl(50.0f), float2ctrl(0.0f), 12, 24);

    // --- 3.2 算法核心参数整定与初始化 ---

    // 电流环 (QPR) 初始化
    ctl_sinv_rc_init_t rc_init = {0};
    rc_init.fs = CONTROLLER_FREQUENCY;
    rc_init.freq_grid = 50.0f;
    rc_init.v_base = 311.0f; // 220V RMS
    rc_init.i_base = 20.0f;
    rc_init.v_bus = 400.0f;
    rc_init.L_ac = 0.003f;
    rc_init.R_ac = 0.1f;
    ctl_auto_tuning_sinv_rc(&rc_init);
    ctl_init_sinv_rc_core(&rc_core, &rc_init, fdrc_buffer, FDRC_ARRAY_SIZE);

    // 绑定物理 ADC 指针到控制器
    ctl_attach_sinv_rc(&rc_core, &adc_v_bus.control_port, &adc_v_grid.control_port, &adc_i_ac.control_port);

    // H桥单极性调制器初始化 (周期=2500, 死区=50)
    ctl_init_single_phase_H_modulation(&hpwm, 2500, 50, float2ctrl(0.5f));
    hpwm.flag_enable_dbcomp = 1;

    // PLL 与 PQ 计算初始化
    // 采用 50Hz/s 的有功爬坡限制和 100Var/s 的无功爬坡限制 (安规软启动)
    ctl_init_single_phase_pll(&pll, 10.0f, 0.02f, 20.0f, 50.0f, CONTROLLER_FREQUENCY);
    ctl_init_sms_pq(&pq_meter, CONTROLLER_FREQUENCY, 50.0f);
    ctl_init_sinv_ref_gen(&ref_gen, 40.0f, 0.1f, 10.0f, 20.0f, CONTROLLER_FREQUENCY);

    // --- 3.3 框架模块初始化 ---

    // CiA 402 状态机
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // Operation Enabled 保持 100ms 后才正式切入负载

    // 保护模块初始化
    ctl_sinv_prot_init_t prot_init = {0};
    prot_init.error_mask =
        SINV_PROT_BIT_HW_TZ | SINV_PROT_BIT_DC_OVP_FAST | SINV_PROT_BIT_AC_OCP_FAST | SINV_PROT_BIT_CTRL_DIVERGE;
    prot_init.v_bus_max = 420.0f;
    prot_init.i_ac_max = 30.0f;
    ctl_init_sinv_protect(&protection, &prot_init);

    // ADC 自动偏置校准器
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);
    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }

    // --- BUILD_LEVEL 控制逻辑 ---
#if (BUILD_LEVEL == 1)
    // 离网负载校验模式：不需要 FDRC，通常工作在 V/f 模式或极简电流环
    rc_core.flag_enable_fdrc = 0;
    rc_core.flag_enable_lead_comp = 0;
#endif

#if (BUILD_LEVEL == 2)
    // 并网 PQ 控制模式：禁止 FDRC，只用 QPR 保证基本运行
    rc_core.flag_enable_fdrc = 0;
    rc_core.flag_enable_lead_comp = 1;
#endif
}

//=================================================================================================
// 4. 后台任务调度 (低频任务)

void ctl_mainloop(void)
{
    // 状态机核心调度 (处理启停、故障复位)
    cia402_dispatch(&cia402_sm);

#if (BUILD_LEVEL == 3)
    // 并网性能优化模式：延时切入 FDRC
    if (cia402_sm.state_word.bits.operation_enabled && cia402_sm.current_state_counter > 200)
    {
        rc_core.flag_enable_fdrc = 1;
    }
    else
    {
        rc_core.flag_enable_fdrc = 0;
    }
    rc_core.flag_enable_lead_comp = 1;
#endif
}

//=================================================================================================
// 5. 回调函数实现

void clear_all_controllers(void)
{
    ctl_clear_single_phase_pll(&pll);
    ctl_clear_sinv_rc_core(&rc_core);
    ctl_clear_sinv_ref_gen(&ref_gen);
    ctl_clear_single_phase_H_modulation(&hpwm);
}

void ctl_enable_pwm(void)
{
    ctl_fast_enable_output();
}

void ctl_disable_pwm(void)
{
    ctl_fast_disable_output();
}
