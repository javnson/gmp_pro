/**
 * @file ctl_main.c
 * @author GMP Library Contributors
 * @brief 20kW Single-Phase Inverter/AFE Top-level Implementation.
 */

#include <gmp_core.h>

#include "ctl_main.h"
#include <ctrl_settings.h>

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

// ADC 采样通道定义 (硬件增益和偏置在 setup_peripheral 中配置)
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

// FDRC 重复控制器静态内存 (根据控制频率自动计算数组长度)
#define FDRC_ARRAY_SIZE ((int)(CONTROLLER_FREQUENCY / 50.0f) + 10)
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
    // 2. PLL 内部频率误差必须小于系统设定的容忍度 (例如 0.005 PU)
    ctrl_gt v_mag_pu = pll.v_mag;
    ctrl_gt f_err_abs = ctl_abs(pll.freq_error);

    if ((v_mag_pu > float2ctrl(0.8f)) && (v_mag_pu < float2ctrl(1.2f)))
    {
        if (f_err_abs < CTRL_SPLL_EPSILON)
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

    // 注: ADC 通道的增益与偏置初始化 (ctl_init_adc_channel)
    // 已移至 xplt.peripheral.c 的 setup_peripheral() 中，以确保硬件配置彻底解耦。

    // --- 3.1 算法核心参数整定与初始化 ---

    // 电流环 (QPR) 初始化
    ctl_sinv_rc_init_t rc_init = {0};
    rc_init.fs = CONTROLLER_FREQUENCY;
    rc_init.freq_grid = 50.0f;
    rc_init.v_base = CTRL_VOLTAGE_BASE;
    rc_init.i_base = CTRL_CURRENT_BASE;
    rc_init.v_bus = CTRL_DCBUS_VOLTAGE;
    rc_init.L_ac = 0.003f; // 根据实际滤波器电感修改
    rc_init.R_ac = 0.1f;   // 寄生电阻估算
    ctl_auto_tuning_sinv_rc(&rc_init);
    ctl_init_sinv_rc_core(&rc_core, &rc_init, fdrc_buffer, FDRC_ARRAY_SIZE);

    // 绑定物理 ADC 指针到控制器 (零拷贝数据拉取)
    ctl_attach_sinv_rc(&rc_core, &adc_v_bus.control_port, &adc_v_grid.control_port, &adc_i_ac.control_port);

    // H桥单极性调制器初始化
    ctl_init_single_phase_H_modulation(&hpwm, CTRL_PWM_CMP_MAX + 1, CTRL_PWM_DEADBAND_CMP, float2ctrl(0.5f));
    hpwm.flag_enable_dbcomp = 1; // 开启死区补偿

    // PLL 与 PQ 计算初始化
    ctl_init_single_phase_pll(&pll, 10.0f, 0.02f, 20.0f, 50.0f, CONTROLLER_FREQUENCY);
    ctl_init_sms_pq(&pq_meter, CONTROLLER_FREQUENCY, 50.0f);

    // 指令发生器初始化 (限幅保护: I_max, V_min, P_slope, Q_slope)
    // 爬坡斜率设定为: 有功 10.0 PU/s， 无功 20.0 PU/s
    ctl_init_sinv_ref_gen(&ref_gen, CTRL_CURRENT_BASE * 1.5f, 0.1f, 10.0f, 20.0f, CONTROLLER_FREQUENCY);

    // --- 3.2 框架模块初始化 ---

    // CiA 402 状态机
    init_cia402_state_machine(&cia402_sm);
    cia402_sm.minimum_transit_delay[3] = 100; // Operation Enabled 保持 100ms 后才正式切入负载

    // 保护模块初始化
    ctl_sinv_prot_init_t prot_init = {0};
    prot_init.error_mask =
        SINV_PROT_BIT_HW_TZ | SINV_PROT_BIT_DC_OVP_FAST | SINV_PROT_BIT_AC_OCP_FAST | SINV_PROT_BIT_CTRL_DIVERGE;
    prot_init.warning_mask = SINV_PROT_BIT_AC_OVP_RMS | SINV_PROT_BIT_AC_UVP_RMS | SINV_PROT_BIT_PLL_FREQ_ERR;
    prot_init.v_bus_max = CTRL_PROT_VBUS_MAX;
    prot_init.i_ac_max = CTRL_MAX_HW_CURRENT * 0.9f; // 硬件极限的 90%
    ctl_init_sinv_protect(&protection, &prot_init);

    // ADC 自动偏置校准器
    ctl_init_adc_calibrator(&adc_calibrator, 20, 0.707f, CONTROLLER_FREQUENCY);
    if (flag_enable_adc_calibrator)
    {
        ctl_enable_adc_calibrator(&adc_calibrator);
    }

    // --- 3.3 BUILD_LEVEL 控制逻辑 ---
#if (BUILD_LEVEL == 1)
    // 离网负载校验模式：不需要 FDRC 和前馈
    rc_core.flag_enable_fdrc = 0;
    rc_core.flag_enable_lead_comp = 0;
#endif

#if (BUILD_LEVEL == 2)
    // 并网 PQ 控制模式：禁止 FDRC，开启电压前馈
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
    // 当并网稳定运行超过 200 个控制周期后，开启重复控制器消除畸变
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
// 5. 背景回调函数实现 (慢速保护与 ADC 校准)

/**
 * @brief 慢速保护后台任务
 * @note 放置在 RTOS 的低频 Task 中 (例如 1ms/10ms 周期)
 */
gmp_task_status_t tsk_protect(gmp_task_t* tsk)
{
    GMP_UNUSED_VAR(tsk);

    // 执行慢保护检测 (交流有效值过欠压、温度、电网频率)
    ctl_task_sinv_protect_slow(&protection, pq_meter.v_rms, pll.frequency,
                               float2ctrl(25.0f), // Mock temp: 实际应接入板载温度传感器
                               pq_meter.i_rms);

    if (protection.active_errors != 0)
    {
        cia402_fault_request(&cia402_sm);
    }

    return GMP_TASK_DONE;
}

/**
 * @brief ADC 上电零点偏置校准器
 * @note  阻塞控制算法，直至交流电压/电流通道零点校准完毕
 */
fast_gt ctl_exec_adc_calibration(void)
{
    if (flag_enable_adc_calibrator)
    {
        if (ctl_is_adc_calibrator_cmpt(&adc_calibrator) && ctl_is_adc_calibrator_result_valid(&adc_calibrator))
        {
            if (index_adc_calibrator == 0) // 通道 0: AC 电流零偏校准
            {
                adc_i_ac.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                ctl_clear_adc_calibrator(&adc_calibrator);
                ctl_enable_adc_calibrator(&adc_calibrator);
            }
            else if (index_adc_calibrator == 1) // 通道 1: 交流电压零偏校准
            {
                adc_v_grid.bias += ctl_get_adc_calibrator_result(&adc_calibrator);
                index_adc_calibrator += 1;

                flag_enable_adc_calibrator = 0;
                clear_all_controllers(); // 校准完毕，清理滤波器内残余历史值
            }
        }
        return 0; // 校准尚未完成，拦截主控制流程
    }
    return 1; // 校准已完成或被跳过，放行主控制流程
}

//=================================================================================================
// 6. 控制器复位与使能动作

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
