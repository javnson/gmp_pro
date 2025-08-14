/**
 * @file offline_motor_param_est.h
 * @author Javnson
 * @brief Top-level framework for offline PMSM parameter identification. 
 * @version 0.8 (Fixed)
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_OFFLINE_MOTOR_PARAM_EST_H_
#define _FILE_OFFLINE_MOTOR_PARAM_EST_H_

// 包含所有必需的底层模块头文件
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/math_block/vector_lite/vector3.h>

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
    uint16_t qep_search_elec_cycles; /**< QEP index搜索时旋转的电周期数 */

    // Rs configuration
    parameter_gt rs_test_current_pu; /**< Rs测试中使用的电流标幺值 */

    // Ldq configuration
    parameter_gt l_hfi_v_pu;        /**< Ldq辨识时注入的高频电压标幺值 */
    parameter_gt l_hfi_freq_hz;     /**< Ldq辨识时注入的高频电压频率 (Hz) */
    parameter_gt l_hfi_rot_freq_hz; /**< Ldq辨识时高频矢量的旋转频率 (Hz) */

    // FIX: 增加磁链和惯量测试所需的配置参数
    parameter_gt flux_test_iq_pu; /**< Flux测试中用于产生转矩的Iq标幺值 */
    parameter_gt j_test_iq_pu;    /**< 惯量J测试中使用的Iq标幺值 */

    /*-------------------- 模块接口 (Module Interfaces) --------------------*/
    mtr_ift* mtr_interface;                   /**< 指向通用电机传感器接口的指针 */
    ctl_per_unit_consultant_t* pu_consultant; /**< 指向标幺值系统顾问的指针 */
    ctl_vector3_t vab_command;                /**< 输出给PWM模块的alpha-beta电压指令 */

    /*-------------------- 控制模块 (Control Modules) --------------------*/
    ctl_current_controller_t current_ctrl;    /**< motor current controller */
    ctl_slope_f_controller speed_profile_gen; /**< rotor angle generator */
    ctl_sine_generator_t hfi_signal_gen;      /**< HFI sine generator */
    ctl_low_pass_filter_t measure_flt[4];     /**< 0:Vd, 1:Id, 2:Pos, 3:I_hfi_mag */

    /*-------------------- 状态与标志位 (State & Flags) --------------------*/
    ctl_offline_est_main_state_e main_state; /**< main state machine. */
    ctl_offline_est_sub_state_e sub_state;   /**< sub state machine. */
    fast_gt flag_start_estimation;           /**< controller flag start estimation. */
    fast_gt
        flag_estimation_done; /**< output flag complete estimation, the @ref ctl_offline_est_t::pmsm_params is valid. */
    fast_gt flag_error_detected; /**< output flag error */
    fast_gt flag_enable_rs;      /**< Rs and encoder off-line estimate is enabled */
    /**
     * @brief Ldq off-line estimate method choose and switch.
     * 0: disable Ldq estimate, 
     * 1: enable Ldq estimate and use DC bias offset (FIXME: NOT IMPLEMENTED)
     * 2: enable Ldq estimate and use High frequency rotation vector injection method
     */
    fast_gt flag_enable_ldq;
    fast_gt flag_enable_psif;    /**< @f( \psi_f @f) off-line is enabled */
    fast_gt flag_enable_inertia; /**< inertia J estimate is enabled */
    time_gt task_start_time;     /**< a variable to log the start time*/

    /*-------------------- 中间变量 (Intermediate Variables) --------------------*/
    // Rs & Encoder 辨识变量
    uint16_t sample_count;
    parameter_gt V_sum, I_sum, Pos_sum; /**< SI单位 */
    parameter_gt V_sq_sum, I_sq_sum;    /**< SI单位 */
    parameter_gt rs_step_results[6];    /**< 存储六步法每一步的线电阻测量结果,SI单位 */
    parameter_gt enc_offset_results[6]; /**< 存储六步法每一步的编码器位置 */
    uint16_t step_index;
    // FIX: 增加一个变量, 用于在后台任务和ISR之间传递Rs辨识所需的目标角度
    parameter_gt rs_test_angle_pu; /**< Rs辨识中, 用于锁定转子的目标电角度 (单位: PU) */

    // Ld/Lq 辨识变量
    parameter_gt hfi_i_max, hfi_i_min;
    parameter_gt hfi_theta_d, hfi_theta_q;

    // J 辨识变量 (部分变量复用自Rs辨识)
    parameter_gt sum_x, sum_y, sum_xy, sum_x2;
    parameter_gt avg_torque;

    /*-------------------- 最终辨识结果 (Final Identified Parameters) --------------------*/
    ctl_pmsm_dsn_consultant_t pmsm_params; /**< output: PMSM parameters */
    ctl_vector3_t Rs_line_to_line;         /**< output: PMSM Rs (3phase), judging if motor is connected correctly. */
    parameter_gt encoder_offset;           /**< output: encoder offset */
    parameter_gt current_noise_std_dev;    /**< output: standard deviation of current，SI单位 */
    parameter_gt position_consistency_std_dev; /**< output: standard deviation of encoder */

} ctl_offline_est_t;

//================================================================================
// 3. 核心函数声明与实现 (Core Function Declarations & Implementations)
//================================================================================

// --- 前向声明状态处理函数 ---
void est_loop_handle_rs(ctl_offline_est_t* est);
void est_loop_handle_l(ctl_offline_est_t* est);
void est_loop_handle_flux(ctl_offline_est_t* est);
void est_loop_handle_j(ctl_offline_est_t* est);
static void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est);
static void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est);

/**
 * @brief 初始化离线参数辨识模块.
 * @note FIX: 合并了原头文件中重复的两个初始化函数, 包含所有配置参数.
 */
GMP_STATIC_INLINE void ctl_init_offline_est(ctl_offline_est_t* est, mtr_ift* mtr_if, ctl_per_unit_consultant_t* pu_cons,
                                            ctl_encoder_type_e enc_type, parameter_gt isr_freq, parameter_gt current_kp,
                                            parameter_gt current_ki, parameter_gt rs_curr_pu, uint16_t qep_cycles,
                                            parameter_gt l_v_pu, parameter_gt l_freq_hz, parameter_gt l_rot_freq_hz,
                                            parameter_gt flux_iq_pu, parameter_gt j_iq_pu)
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
    est->flux_test_iq_pu = flux_iq_pu;
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
    est->rs_test_angle_pu = 0.0f;
}

/**
 * @brief 离线参数辨识模块的实时步进函数 (在ISR中调用).
 * @note FIX: 完全重写了此函数的逻辑, 以确保在不同辨识阶段使用正确的电角度theta.
 * 这是整个修复方案的核心部分.
 */
GMP_STATIC_INLINE void ctl_step_offline_est(ctl_offline_est_t* est)
{
    ctl_vector3_t* iabc = ctl_get_mtr_current(est->mtr_interface);
    ctrl_gt theta = 0.0f;

    // 只在辨识进行时执行
    if (est->main_state > OFFLINE_MAIN_STATE_IDLE && est->main_state < OFFLINE_MAIN_STATE_DONE)
    {
        // 只有在工作阶段执行电机控制器和测量滤波器等
        // 根据当前状态选择电机角度的来源
        // 根据当前状态选择执行特定的滤波器
        if (est->sub_state == OFFLINE_SUB_STATE_EXEC)
        {
            switch (est->main_state)
            {
            case OFFLINE_MAIN_STATE_RS:
                // Rs辨识时, 使用后台循环中设定的固定角度来锁定转子.
                theta = est->rs_test_angle_pu;

                //
                if (gmp_base_get_diff_system_tick(est->task_start_time) > RS_STABILIZE_TIME_MS)
                {
                    // get current measurement result
                    ctrl_gt Id = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[0]);
                    est->I_sum += ctrl2float(Id);
                    est->I_sq_sum += ctrl2float(ctl_mul(Id, Id));

                    // get voltage measurement result
                    ctrl_gt Vd = ctl_step_lowpass_filter(&est->measure_flt[0], est->current_ctrl.vdq0.dat[0]);
                    est->V_sum += ctrl2float(Vd);
                    est->V_sq_sum += ctrl2float(ctl_mul(Vd, Vd));

                    // if encoder is existed, get encoder result
                    if (est->encoder_type != ENCODER_TYPE_NONE)
                    {
                        est->Pos_sum += ctrl2float(
                            ctl_step_lowpass_filter(&est->measure_flt[2], ctl_get_mtr_elec_theta(est->mtr_interface)));
                    }

                    est->sample_count++;
                }

                break;

            case OFFLINE_MAIN_STATE_L:
                // L辨识时, 根据具体方法确定角度来源
                if (est->flag_enable_ldq == 2)
                { // 方法2: 旋转HFI法

                    // 角度由慢速旋转的斜坡发生器提供
                    theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);

                    ctrl_gt v_hfi_inst;

                    // 当在测量和收敛阶段时，控制器正常运行
                    if (gmp_base_get_diff_system_tick(est->task_start_time) <
                        LDQ_STABILIZE_TIME_MS + LDQ_MEASUREMENT_TIME_MS)
                    {
                        // 发生信号
                        ctl_step_sine_generator(&est->hfi_signal_gen);
                        // 在d轴上注入高频电压, q轴前馈为0,
                        // 为什么这里是乘法而不是加法？
                        v_hfi_inst = ctl_mul(ctl_get_sine_generator_sin(&est->hfi_signal_gen), est->l_hfi_v_pu);

                        // 获取电流环计算出的alpha-beta电流 (这是对实际电流的反馈)
                        ctrl_gt i_alpha = est->current_ctrl.iab0.dat[0];
                        ctrl_gt i_beta = est->current_ctrl.iab0.dat[1];

                        // 计算高频电流响应的幅值，对幅值进行低通滤波，得到其包络
                        ctrl_gt i_hfi_mag =
                            ctl_step_lowpass_filter(&est->measure_flt[3], ctl_vector2_mag_sq(est->current_ctrl.iab0));

                        // 先等待一段时间，等到稳定（控制器收敛、滤波器收敛）之后再读取电流的最大最小值，只是在一段时间内执行
                        if (gmp_base_get_diff_system_tick(est->task_start_time) > LDQ_STABILIZE_TIME_MS)
                        {
                            // 寻找电流响应的极大值(对应q轴)和极小值(对应d轴)
                            if (i_hfi_mag > est->hfi_i_max)
                            {
                                est->hfi_i_max = i_hfi_mag;
                                est->hfi_theta_q =
                                    ctl_get_ramp_generator_output(&est->speed_profile_gen.rg); // 电流最大处是q轴
                            }
                            if (i_hfi_mag < est->hfi_i_min)
                            {
                                est->hfi_i_min = i_hfi_mag;
                                est->hfi_theta_d =
                                    ctl_get_ramp_generator_output(&est->speed_profile_gen.rg); // 电流最小处是d轴
                            }
                        }
                    }
                    // 在减速阶段，电压给定要给小
                    else if (gmp_base_get_diff_system_tick(est->task_start_time) <
                             LDQ_STABILIZE_TIME_MS + LDQ_MEASUREMENT_TIME_MS + LDQ_DECELERATE_TIME_MS)
                    {
                        v_hfi_inst = est->l_hfi_v_pu;
                    }
                    // 停止后，关闭电压给定
                    else
                    {
                        v_hfi_inst = 0;
                    }

                    // 在d轴上注入高频电压, q轴前馈为0
                    ctl_set_voltage_ff(&est->current_ctrl, v_hfi_inst, 0);

                    // 步进旋转发生器, ISR会使用其输出的角度
                    ctl_step_slope_f(&est->speed_profile_gen);
                }
                else
                { // 方法1: 直流偏置HFI法
                    // 角度固定为0, 使d轴对齐alpha轴
                    theta = 0.0f;
                }

                break;

            case OFFLINE_MAIN_STATE_FLUX:
                // 磁链辨识时, 电机开环旋转, 角度由速度发生器提供
                theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);
                break;

            case OFFLINE_MAIN_STATE_J:
                // 惯量辨识时, 电机需要速度反馈, 角度从编码器获取
                // 此测试强制要求有编码器
                theta = ctl_get_mtr_elec_theta(est->mtr_interface);
                break;

            default:
                // 其他状态或默认情况, 不输出电压
                ctl_vector3_clear(&est->vab_command);

                break;
            }

            // --- 3. 执行电流环控制器 ---
            ctl_step_current_controller(&est->current_ctrl, iabc, theta);
            ctl_vector3_copy(&est->vab_command, &est->current_ctrl.vab0);
        }
        // 当不处于工作状态时，不输出电压
        else
        {
            ctl_vector3_clear(&est->vab_command);
        }
    }
    // 如果不处于辨识状态，关闭输出
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
        // FIX: 根据使能标志位决定起始状态
        if (est->flag_enable_rs)
        {
            est->main_state = OFFLINE_MAIN_STATE_RS;
        }
        else if (est->flag_enable_ldq)
        {
            est->main_state = OFFLINE_MAIN_STATE_L;
        }
        else if (est->flag_enable_psif)
        {
            est->main_state = OFFLINE_MAIN_STATE_FLUX;
        }
        else if (est->flag_enable_inertia)
        {
            est->main_state = OFFLINE_MAIN_STATE_J;
        }

        est->sub_state = OFFLINE_SUB_STATE_INIT;
        est->flag_start_estimation = 0;
    }

    // 需要为每一个if条件增加一个else，当没有使能时状态机切换到下一个状态。
    switch (est->main_state)
    {
    case OFFLINE_MAIN_STATE_RS:
        if (est->flag_enable_rs)
            est_loop_handle_rs(est);
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_L;
            est->sub_state = OFFLINE_SUB_STATE_INIT;
        }
        break;
    case OFFLINE_MAIN_STATE_L:
        if (est->flag_enable_ldq)
            est_loop_handle_l(est);
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_FLUX;
            est->sub_state = OFFLINE_SUB_STATE_INIT;
        }
        break;
    case OFFLINE_MAIN_STATE_FLUX:
        if (est->flag_enable_psif)
            est_loop_handle_flux(est);
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_J;
            est->sub_state = OFFLINE_SUB_STATE_INIT;
        }
        break;
    case OFFLINE_MAIN_STATE_J:
        if (est->flag_enable_inertia)
            est_loop_handle_j(est);
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_DONE;
            est->sub_state = OFFLINE_SUB_STATE_INIT;
        }
        break;
    case OFFLINE_MAIN_STATE_DONE:
        est->flag_estimation_done = 1;
        break;
    case OFFLINE_MAIN_STATE_ERROR:
        est->flag_error_detected = 1;
        // 遇到错误时可以停止辨识,并保持当前的状态为ERROR
        break;
    case OFFLINE_MAIN_STATE_IDLE:
    default:
        break;
    }

    return est->flag_estimation_done;
}

/** 
 * @} 
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_OFFLINE_MOTOR_PARAM_EST_H_
