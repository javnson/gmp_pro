/**
 * @file ctl_sinv_protect.h
 * @author GMP Library Contributors
 * @brief Single-Phase Inverter Protection Manager.
 * 
 * @details
 * Assembles highly-optimized specialized protection nodes into a cohesive 
 * protection manager for Single-Phase Inverters or Active Front Ends (AFE).
 * Features Fast (ISR-level) and Slow (Task-level) evaluation pipelines,
 * Mask-based Error/Warning routing, and First-Fault capture for blackbox diagnostics.
 * 
 * @version 1.0
 * @copyright Copyright GMP(c) 2024-2026
 */

#include <ctl/component/intrinsic/protection/protection_slot.h>
#include <ctl/component/intrinsic/protection/pt100x.h>

#ifndef _CTL_SINV_PROTECT_H_
#define _CTL_SINV_PROTECT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*===========================================================================*/
/* 1. System Specific Fault Bitmasks (SINV Domain)                           */
/*===========================================================================*/

#define SINV_PROT_BIT_NONE         (0x00000000UL)
#define SINV_PROT_BIT_HW_TZ        (0x00000001UL) //!< 硬件底层触发封波 (Trip-Zone)
#define SINV_PROT_BIT_DC_OVP_FAST  (0x00000002UL) //!< 直流母线瞬态过压 (Fast)
#define SINV_PROT_BIT_AC_OCP_FAST  (0x00000004UL) //!< 交流侧瞬态过流 (Fast)
#define SINV_PROT_BIT_CTRL_DIVERGE (0x00000008UL) //!< 控制器算法发散 (Fast)

#define SINV_PROT_BIT_AC_OVP_RMS   (0x00000010UL) //!< 交流侧有效值过压 (Slow)
#define SINV_PROT_BIT_AC_UVP_RMS   (0x00000020UL) //!< 交流侧有效值欠压 (Slow)
#define SINV_PROT_BIT_PLL_FREQ_ERR (0x00000040UL) //!< 锁相环电网频率异常 (Slow - 防孤岛)

#define SINV_PROT_BIT_IGBT_TEMP_OVP    (0x00000080UL) //!< 功率模块过温 (Slow - PT100/1000)
#define SINV_PROT_BIT_IGBT_THERMAL_I2T (0x00000100UL) //!< 功率模块反时限热过载 (Slow - I2t)

/*===========================================================================*/
/* 2. Protection Manager Structure                                           */
/*===========================================================================*/

/**
 * @brief Main Protection Manager for Single-Phase Inverter.
 */
typedef struct _tag_sinv_protect_t
{
    // --- 策略与状态字 (Policy & Status Flags) ---
    uint32_t current_status;  //!< 当前正处于越限状态的所有故障位图的原始聚合
    uint32_t first_error;     //!< 首发致命故障锁存 (用于黑匣子溯源)
    uint32_t active_errors;   //!< 经过 error_mask 过滤后的致命故障 (导致跳闸)
    uint32_t active_warnings; //!< 经过 warning_mask 过滤后的告警 (不跳闸，仅降额或报信)

    uint32_t error_mask;   //!< 设为 1 的位表示该异常被认定为致命 Error
    uint32_t warning_mask; //!< 设为 1 的位表示该异常被认定为轻微 Warning

    // --- 极速保护节点 (Fast Nodes - Executed in 20kHz ISR) ---
    ctl_prot_single_t node_dc_ovp_fast;  //!< 母线过压 (单边高阈值)
    ctl_prot_single_t node_ac_ocp_fast;  //!< 交流过流 (对称绝对值阈值)
    ctl_prot_single_t node_ctrl_diverge; //!< 算法发散 (对称绝对值阈值)

    // --- 慢速保护节点 (Slow Nodes - Executed in 1ms/10ms Task) ---
    // 注意：OVP 和 UVP 拆分为两个 Single 节点，因为过压和欠压的消抖时间(安规要求)往往差异极大
    ctl_prot_single_t node_ac_ovp_rms; //!< 交流有效值过压 (单边高)
    ctl_prot_single_t node_ac_uvp_rms; //!< 交流有效值欠压 (单边低)

    ctl_prot_window_t node_pll_freq; //!< 电网频率漂移 (双边窗口保护)

    ctl_prot_pt_sensor_t node_igbt_temp; //!< IGBT 温度传感器 (含阻值温度转换)
    ctl_prot_thermal_t node_igbt_i2t;    //!< IGBT I2t 积分热模型

} ctl_sinv_protect_t;

/*===========================================================================*/
/* Initialization Structure                                                  */
/*===========================================================================*/

/**
 * @brief Auto-configuration structure for the SINV Protection Manager.
 * @details Users populate physical thresholds here. If any threshold is left 
 * as 0.0f, the initialization function will apply safe, industry-standard defaults.
 */
typedef struct _tag_sinv_prot_init_t
{
    // --- 策略路由掩码 ---
    uint32_t error_mask;   //!< Which faults should trip the system?
    uint32_t warning_mask; //!< Which faults are just warnings?

    // --- Fast Protection Thresholds (ISR Level) ---
    parameter_gt v_bus_max;  //!< DC Bus Over-Voltage Limit (e.g., 420.0 V).
    parameter_gt i_ac_max;   //!< AC Peak Over-Current Limit (e.g., 30.0 A).
    parameter_gt v_ctrl_max; //!< Max Controller output PU to detect divergence (e.g., 1.5).

    // --- Slow Protection Thresholds (Task Level) ---
    parameter_gt v_ac_rms_max; //!< AC RMS Over-Voltage Limit (e.g., 260.0 V).
    parameter_gt v_ac_rms_min; //!< AC RMS Under-Voltage Limit (e.g., 180.0 V).

    parameter_gt freq_grid_nom; //!< Nominal grid frequency (50.0 or 60.0 Hz).
    parameter_gt freq_dev_max;  //!< Max allowed frequency deviation (e.g., 0.5 Hz).

    // --- Thermal & Sensor Thresholds ---
    parameter_gt igbt_temp_max; //!< Max IGBT temperature in Celsius (e.g., 85.0 C).
    parameter_gt pt_adc_gain;   //!< ADC to Ohms conversion gain for PT100/1000.
    parameter_gt pt_r0;         //!< Base resistance (100.0 or 1000.0).

    parameter_gt i_ac_rated_rms; //!< Nominal continuous AC current for I2t.
    parameter_gt i2t_limit;      //!< Maximum I2t thermal integral limit.

} ctl_sinv_prot_init_t;

/*===========================================================================*/
/* 3. API Definitions                                                        */
/*===========================================================================*/

/**
 * @brief Initializes the SINV Protection Manager with smart defaults.
 * 
 * @param[out] prot Pointer to the protection manager.
 * @param[in]  init Pointer to the configuration structure.
 */
void ctl_init_sinv_protect(ctl_sinv_protect_t* prot, const ctl_sinv_prot_init_t* init);

/**
 * @brief 深度复位保护管理器 (Deep Reset)
 * @details 专用于开机初始化，以及 CiA 402 收到 FAULT_RESET (0x0080) 指令时。
 * 清除所有报警标志、消抖计数器、黑匣子记录和热积分器。
 * 
 * @param[out] prot Pointer to the protection manager.
 */
GMP_STATIC_INLINE void ctl_reset_sinv_protect(ctl_sinv_protect_t* prot)
{
    // 1. 清空全局路由标志
    prot->current_status = 0;
    prot->first_error = 0;
    prot->active_errors = 0;
    prot->active_warnings = 0;

    // 2. 深度清空底层节点的历史包袱 (消抖计数与黑匣子)
    prot->node_dc_ovp_fast.current_count = 0;
    prot->node_dc_ovp_fast.fault_record_val = float2ctrl(0.0f);

    prot->node_ac_ocp_fast.current_count = 0;
    prot->node_ac_ocp_fast.fault_record_val = float2ctrl(0.0f);

    prot->node_ctrl_diverge.current_count = 0;
    prot->node_ctrl_diverge.fault_record_val = float2ctrl(0.0f);

    prot->node_ac_ovp_rms.current_count = 0;
    prot->node_ac_ovp_rms.fault_record_val = float2ctrl(0.0f);

    prot->node_ac_uvp_rms.current_count = 0;
    prot->node_ac_uvp_rms.fault_record_val = float2ctrl(0.0f);

    prot->node_pll_freq.current_count = 0;
    prot->node_pll_freq.fault_record_val = float2ctrl(0.0f);

    prot->node_igbt_temp.current_count = 0;
    prot->node_igbt_temp.fault_record_val = float2ctrl(0.0f);

    // 3. 特殊清理：热量积分器复位
    // 系统复位时，默认物理设备的热量也会散去(或者至少把软件积分器清零，重新积分)
    prot->node_igbt_i2t.thermal_acc = float2ctrl(0.0f);
    prot->node_igbt_i2t.fault_record_val = float2ctrl(0.0f);
}

/**
 * @brief 高频极速保护执行管道 (Fast Pipeline)
 * @details 放置于控制 ISR 中 (如 20kHz)。仅执行纯数学、无延迟的绝对值比较。
 * 
 * @param prot Pointer to the manager.
 * @param v_bus_inst 瞬时直流母线电压
 * @param i_ac_inst 瞬时交流电网电流
 * @param ctrl_v_ref 控制算法输出的指令电压 (用于检测算法发散)
 * @return uint32_t 返回 active_errors。如果不为 0，ISR 应立即硬件封波！
 */
GMP_STATIC_INLINE uint32_t ctl_step_sinv_protect_fast(ctl_sinv_protect_t* prot, ctrl_gt v_bus_inst, ctrl_gt i_ac_inst,
                                                      ctrl_gt ctrl_v_ref)
{
    uint32_t inst_status = 0;

    // 1. 挂载快保护算子 (并行执行)
    inst_status |= ctl_step_prot_single_high_peak(&prot->node_dc_ovp_fast, v_bus_inst);
    inst_status |= ctl_step_prot_single_sym_peak(&prot->node_ac_ocp_fast, i_ac_inst);
    inst_status |= ctl_step_prot_single_sym_peak(&prot->node_ctrl_diverge, ctrl_v_ref);

    // 2. 状态聚合与策略路由
    prot->current_status |= inst_status;
    prot->active_errors |= (inst_status & prot->error_mask);
    prot->active_warnings |= (inst_status & prot->warning_mask);

    // 3. 首发故障锁存 (First-Error Capture)
    if (prot->active_errors != 0 && prot->first_error == 0)
    {
        prot->first_error = prot->active_errors;
    }

    return prot->active_errors;
}

/**
 * @brief 低频慢速保护执行管道 (Slow Pipeline)
 * @details 放置于后台任务或 RTOS 线程中 (如 1ms/10ms)。执行有效值比较、温度计算及热积分。
 * 
 * @param prot Pointer to the manager.
 * @param v_ac_rms 交流电压有效值
 * @param pll_freq_hz 锁相环实时输出的电网频率 (Hz)
 * @param temp_adc_pu 温度传感器 ADC 采样的标幺值或原始值
 * @param i_ac_rms 交流电流有效值 (用于 I2t 积分)
 */
GMP_STATIC_INLINE void ctl_task_sinv_protect_slow(ctl_sinv_protect_t* prot, ctrl_gt v_ac_rms, ctrl_gt pll_freq_hz,
                                                  ctrl_gt temp_adc_pu, ctrl_gt i_ac_rms)
{
    uint32_t task_status = 0;

    // 1. 挂载慢保护算子
    task_status |= ctl_step_prot_single_high_peak(&prot->node_ac_ovp_rms, v_ac_rms);
    task_status |= ctl_step_prot_single_low_snap(&prot->node_ac_uvp_rms, v_ac_rms);

    task_status |= ctl_step_prot_window_snap(&prot->node_pll_freq, pll_freq_hz);

    task_status |= ctl_step_prot_pt_sensor(&prot->node_igbt_temp, temp_adc_pu);
    task_status |= ctl_step_prot_thermal_i2t(&prot->node_igbt_i2t, i_ac_rms);

    // 2. 状态聚合与策略路由
    prot->current_status |= task_status;
    prot->active_errors |= (task_status & prot->error_mask);
    prot->active_warnings |= (task_status & prot->warning_mask);

    // 3. 首发故障锁存 (First-Error Capture)
    if (prot->active_errors != 0 && prot->first_error == 0)
    {
        prot->first_error = prot->active_errors;
    }
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_SINV_PROTECT_H_
