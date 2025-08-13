/**
 * @file offline_motor_param_est.h
 * @author Javnson & Gemini
 * @brief Top-level framework for offline PMSM parameter identification. (Parameterized)
 * @version 0.6
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_OFFLINE_MOTOR_PARAM_EST_H_
#define _FILE_OFFLINE_MOTOR_PARAM_EST_H_

// 包含所有必需的底层模块头文件
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/intrinsic/filter/discrete_filter.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/current_controller/motor_current_ctrl.h>
#include <ctl/component/motor_control/generator/vf_generator.h>
#include <ctl/component/motor_control/interface/motor_universal_interface.h>
#include <ctl/math_block/vector/vector3.h>

// 假设的系统核心头文件
// #include "gmp_core.h"
// typedef uint32_t time_gt;
// time_gt gmp_core_get_systemtick(void);
// #define GMP_STATIC_INLINE static inline

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup OFFLINE_ESTIMATION Offline Parameter Estimation
 * @brief A module to automatically identify key PMSM parameters.
 * @{
 */

//================================================================================
// 1. 枚举与配置 (Enums & Configurations)
//================================================================================

typedef enum
{
    ENCODER_TYPE_NONE,     /**< 无编码器 */
    ENCODER_TYPE_ABSOLUTE, /**< 绝对值编码器 (如磁编、光编) */
    ENCODER_TYPE_QEP       /**< 增量式QEP编码器 */
} ctl_encoder_type_e;

typedef enum
{
    OFFLINE_MAIN_STATE_IDLE,
    OFFLINE_MAIN_STATE_RS,
    OFFLINE_MAIN_STATE_L,
    OFFLINE_MAIN_STATE_FLUX,
    OFFLINE_MAIN_STATE_J,
    OFFLINE_MAIN_STATE_DONE,
    OFFLINE_MAIN_STATE_ERROR
} ctl_offline_est_main_state_e;
typedef enum
{
    OFFLINE_SUB_STATE_INIT,
    OFFLINE_SUB_STATE_QEP_INDEX_SEARCH,
    OFFLINE_SUB_STATE_EXEC,
    OFFLINE_SUB_STATE_CALC,
    OFFLINE_SUB_STATE_DONE
} ctl_offline_est_sub_state_e;

//================================================================================
// 2. 主结构体定义 (Main Structure Definition)
//================================================================================

typedef struct ctl_offline_est_s
{
    /*-------------------- 配置 (Configuration) --------------------*/
    ctl_encoder_type_e encoder_type; /**< 用户指定的编码器类型 */
    parameter_gt isr_freq_hz;        /**< 控制中断的频率 (Hz) */
    parameter_gt rs_test_current_pu; /**< Rs测试中使用的电流标幺值 */
    uint16_t qep_search_elec_cycles; /**< QEP index搜索时旋转的电周期数 */

    /*-------------------- 模块接口 (Module Interfaces) --------------------*/
    mtr_ift* mtr_interface;                   /**< 指向通用电机传感器接口的指针 */
    ctl_per_unit_consultant_t* pu_consultant; /**< 指向标幺值系统顾问的指针 */
    ctl_vector3_t vab_command;                /**< 输出给PWM模块的alpha-beta电压指令 */

    /*-------------------- 控制模块 (Control Modules) --------------------*/
    ctl_current_controller_t current_ctrl;
    ctl_slope_f_controller speed_profile_gen;
    ctl_sine_generator_t hfi_signal_gen;
    ctl_low_pass_filter_t measure_flt[3]; /**< 0: Vd, 1: Id, 2: Encoder Pos */

    /*-------------------- 状态与标志位 (State & Flags) --------------------*/
    ctl_offline_est_main_state_e main_state;
    ctl_offline_est_sub_state_e sub_state;
    fast_gt flag_start_estimation;
    fast_gt flag_estimation_done;
    fast_gt flag_error_detected;
    time_gt task_start_time;

    /*-------------------- 中间变量 (Intermediate Variables) --------------------*/
    // Rs & Encoder 辨识变量
    uint16_t sample_count;                       /**< 用于计算均值和标准差的采样计数 */
    parameter_gt V_sum, I_sum, Pos_sum;          /**< 测量值的累加和 */
    parameter_gt V_sq_sum, I_sq_sum, Pos_sq_sum; /**< 测量值平方的累加和 */
    parameter_gt rs_line_results[3];             /**< 存储三相线电阻的测量结果 */
    parameter_gt enc_offset_results[3];          /**< 存储三步定位的编码器位置 */
    uint16_t step_index;                         /**< 当前在定位的第几步 */

    /*-------------------- 最终辨识结果 (Final Identified Parameters) --------------------*/
    ctl_pmsm_dsn_consultant_t pmsm_params;     /**< 注意: pmsm_params.Rs 现在是平均相电阻 */
    ctl_vector3_t Rs_line_to_line;             /**< 存储测量的三相线电阻 (U-V, V-W, W-U) */
    parameter_gt encoder_offset;               /**< 编码器零点偏置 (电角度, 0-1.0) */
    parameter_gt current_noise_std_dev;        /**< 评估出的电流信号标准差 (A) */
    parameter_gt position_consistency_std_dev; /**< 评估出的位置传感器步进一致性标准差 (rad_mech) */

} ctl_offline_est_t;

//================================================================================
// 3. 核心函数声明与实现 (Core Function Declarations & Implementations)
//================================================================================

// --- 前向声明状态处理函数 ---
static void est_loop_handle_rs(ctl_offline_est_t* est);
static void est_loop_handle_l(ctl_offline_est_t* est);
static void est_loop_handle_flux(ctl_offline_est_t* est);
static void est_loop_handle_j(ctl_offline_est_t* est);

GMP_STATIC_INLINE fast_gt est_is_delay_elapsed_ms(time_gt start_tick, uint32_t delay_ms)
{
    time_gt current_tick = gmp_core_get_systemtick();
    return ((current_tick - start_tick) >= delay_ms);
}

/**
 * @brief 初始化离线参数辨识模块.
 */
GMP_STATIC_INLINE void ctl_init_offline_est(ctl_offline_est_t* est, mtr_ift* mtr_if, ctl_per_unit_consultant_t* pu_cons,
                                            ctl_encoder_type_e enc_type, parameter_gt isr_freq, parameter_gt current_kp,
                                            parameter_gt current_ki, parameter_gt rs_curr_pu, uint16_t qep_cycles)
{
    est->mtr_interface = mtr_if;
    est->pu_consultant = pu_cons;

    // 保存配置
    est->encoder_type = enc_type;
    est->isr_freq_hz = isr_freq;
    est->rs_test_current_pu = rs_curr_pu;
    est->qep_search_elec_cycles = qep_cycles;

    // 初始化模块
    ctl_init_current_controller(&est->current_ctrl, current_kp, current_ki, 0, ctl_consult_base_peak_voltage(pu_cons),
                                -ctl_consult_base_peak_voltage(pu_cons), isr_freq);
    for (int i = 0; i < 3; ++i)
        ctl_clear_lowpass_filter(&est->measure_flt[i]);

    // 初始化状态
    est->main_state = OFFLINE_MAIN_STATE_IDLE;
    est->sub_state = OFFLINE_SUB_STATE_INIT;
    est->flag_start_estimation = 0;
    est->flag_estimation_done = 0;
    est->flag_error_detected = 0;

    // 清空结果
    ctl_init_pmsm_dsn_consultant(&est->pmsm_params, 0, 0, 0, 0, 0, 0, 0);
    ctl_vector3_clear(&est->vab_command);
    ctl_vector3_clear(&est->Rs_line_to_line);
    est->encoder_offset = 0.0f;
    est->current_noise_std_dev = 0.0f;
    est->position_consistency_std_dev = 0.0f;
}

/**
 * @brief 离线参数辨识模块的实时步进函数 (在ISR中调用).
 */
GMP_STATIC_INLINE void ctl_step_offline_est(ctl_offline_est_t* est)
{
    if (est->main_state > OFFLINE_MAIN_STATE_IDLE && est->main_state < OFFLINE_MAIN_STATE_DONE)
    {
        const ctl_vector3_t* iabc = ctl_get_mtr_current(est->mtr_interface);
        ctrl_gt theta = ctl_get_mtr_elec_theta(est->mtr_interface);
        ctl_step_current_controller(&est->current_ctrl, iabc, theta);
        ctl_vector3_copy(&est->vab_command, &est->current_ctrl.vab0);
    }
    else
    {
        ctl_vector3_clear(&est->vab_command);
    }
}

/**
 * @brief 离线参数辨识模块的后台循环函数 (在主循环中调用).
 */
GMP_STATIC_INLINE fast_gt ctl_loop_offline_est(ctl_offline_est_t* est)
{
    if (est->flag_start_estimation && est->main_state == OFFLINE_MAIN_STATE_IDLE)
    {
        est->main_state = OFFLINE_MAIN_STATE_RS;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        est->flag_start_estimation = 0;
    }

    switch (est->main_state)
    {
    case OFFLINE_MAIN_STATE_RS:
        est_loop_handle_rs(est);
        break;
    case OFFLINE_MAIN_STATE_L:
        est_loop_handle_l(est);
        break;
    case OFFLINE_MAIN_STATE_FLUX:
        est_loop_handle_flux(est);
        break;
    case OFFLINE_MAIN_STATE_J:
        est_loop_handle_j(est);
        break;
    case OFFLINE_MAIN_STATE_DONE:
        est->flag_estimation_done = 1;
        break;
    case OFFLINE_MAIN_STATE_ERROR:
        break;
    case OFFLINE_MAIN_STATE_IDLE:
    default:
        break;
    }

    return est->flag_estimation_done;
}

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_OFFLINE_MOTOR_PARAM_EST_H_
