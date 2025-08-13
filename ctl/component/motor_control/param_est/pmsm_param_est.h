/**
 * @file offline_motor_param_est.h
 * @author Javnson & Gemini
 * @brief Top-level framework for offline PMSM parameter identification. (J Interface)
 * @version 0.8
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
    parameter_gt l_hfi_v_pu;         /**< L辨识时注入的高频电压标幺值 */
    parameter_gt l_hfi_freq_hz;      /**< L辨识时注入的高频电压频率 (Hz) */
    parameter_gt l_hfi_rot_freq_hz;  /**< L辨识时高频矢量的旋转频率 (Hz) */
    parameter_gt j_test_iq_pu;       /**< J辨识时施加的q轴电流标幺值 */

    /*-------------------- 模块接口 (Module Interfaces) --------------------*/
    mtr_ift* mtr_interface;                   /**< 指向通用电机传感器接口的指针 */
    ctl_per_unit_consultant_t* pu_consultant; /**< 指向标幺值系统顾问的指针 */
    ctl_vector3_t vab_command;                /**< 输出给PWM模块的alpha-beta电压指令 */

    /*-------------------- 控制模块 (Control Modules) --------------------*/
    ctl_current_controller_t current_ctrl;
    ctl_slope_f_controller speed_profile_gen;
    ctl_sine_generator_t hfi_signal_gen;
    ctl_low_pass_filter_t measure_flt[4]; /**< 0:Vd, 1:Id, 2:Pos, 3:I_hfi_mag */

    /*-------------------- 状态与标志位 (State & Flags) --------------------*/
    ctl_offline_est_main_state_e main_state;
    ctl_offline_est_sub_state_e sub_state;
    fast_gt flag_start_estimation;
    fast_gt flag_estimation_done;
    fast_gt flag_error_detected;
    time_gt task_start_time;

    /*-------------------- 中间变量 (Intermediate Variables) --------------------*/
    // 通用累加器
    uint32_t sample_count;
    parameter_gt sum_x, sum_y, sum_xy, sum_x2; /**< 用于线性回归的累加器 */
    uint16_t step_index;

    // Rs & Encoder 辨识变量
    parameter_gt rs_step_results[6];
    parameter_gt enc_offset_results[6];

    // Ld/Lq 辨识变量
    parameter_gt hfi_i_max, hfi_i_min;
    parameter_gt hfi_theta_d, hfi_theta_q;

    // J 辨识变量
    parameter_gt avg_torque; /**< J辨识期间的平均转矩 */

    /*-------------------- 最终辨识结果 (Final Identified Parameters) --------------------*/
    ctl_pmsm_dsn_consultant_t pmsm_params;
    ctl_vector3_t Rs_line_to_line;
    parameter_gt encoder_offset;
    parameter_gt current_noise_std_dev;
    parameter_gt position_consistency_std_dev;

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
                                            parameter_gt current_ki, parameter_gt rs_curr_pu, uint16_t qep_cycles,
                                            parameter_gt l_v_pu, parameter_gt l_freq_hz, parameter_gt l_rot_freq_hz,
                                            parameter_gt j_iq_pu)
{
    est->mtr_interface = mtr_if;
    est->pu_consultant = pu_cons;

    // 保存配置
    est->encoder_type = enc_type;
    est->isr_freq_hz = isr_freq;
    est->rs_test_current_pu = rs_curr_pu;
    est->qep_search_elec_cycles = qep_cycles;
    est->l_hfi_v_pu = l_v_pu;
    est->l_hfi_freq_hz = l_freq_hz;
    est->l_hfi_rot_freq_hz = l_rot_freq_hz;
    est->j_test_iq_pu = j_iq_pu;

    // 初始化模块
    ctl_init_current_controller(&est->current_ctrl, current_kp, current_ki, 0, ctl_consult_base_peak_voltage(pu_cons),
                                -ctl_consult_base_peak_voltage(pu_cons), isr_freq);
    for (int i = 0; i < 4; ++i)
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

        // 根据不同状态，FOC的theta有不同来源
        ctrl_gt theta = 0.0f;
        if (est->main_state == OFFLINE_MAIN_STATE_L)
        {
            theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);
        }
        else if (est->main_state == OFFLINE_MAIN_STATE_RS)
        {
            // TODO: 在Rs辨识中设置固定的theta
        }
        else
        {
            theta = ctl_get_mtr_elec_theta(est->mtr_interface);
        }

        // HFI注入时，电压由前馈产生
        if (est->main_state == OFFLINE_MAIN_STATE_L && est->sub_state == OFFLINE_SUB_STATE_EXEC)
        {
            ctl_step_sine_generator(&est->hfi_signal_gen);
            parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(est->pu_consultant, est->l_hfi_v_pu);
            parameter_gt v_hfi_inst = ctl_get_sine_generator_sin(&est->hfi_signal_gen) * v_hfi_peak;
            ctl_set_voltage_ff(&est->current_ctrl, v_hfi_inst, 0.0f);
        }

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
