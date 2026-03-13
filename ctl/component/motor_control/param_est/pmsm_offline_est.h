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
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/component/motor_control/basic/vf_generator.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>
#include <ctl/math_block/vector_lite/vector3.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup OFFLINE_ESTIMATION Offline Parameter Estimation
 * 
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

//typedef struct ctl_offline_est_s
//{
//    // input ports
//    mtr_ift* mtr_interface;                   /**< 指向通用电机传感器接口的指针 */
//    ctl_per_unit_consultant_t* pu_consultant; /**< 指向标幺值系统顾问的指针 */
//
//    // output ports
//    ctl_vector3_t vab_command; /**< 输出给PWM模块的alpha-beta电压指令 */
//
//    // key configuration
//    ctl_encoder_type_e encoder_type; /**< 用户指定的编码器类型 */
//    parameter_gt fs;                 /**< 控制中断的频率 (Hz) */
//    uint16_t qep_search_elec_cycles; /**< QEP index搜索时旋转的电周期数 */
//
//    // inner controller
//    mtr_current_init_t current_ctrl;    /**< motor current controller */
//    ctl_slope_f_controller speed_profile_gen; /**< rotor angle generator */
//    ctl_sine_generator_t hfi_signal_gen;      /**< HFI sine generator */
//    ctl_low_pass_filter_t measure_flt[4];     /**< 0:Vd, 1:Id, 2:Pos, 3:I_hfi_mag */
//
//    // system state machine
//    ctl_offline_est_main_state_e main_state; /**< main state machine. */
//    ctl_offline_est_sub_state_e sub_state;   /**< sub state machine. */
//
//    fast_gt flag_start;      /**< controller flag start estimation. */
//    fast_gt flag_complete;   /**< output flag complete estimation, the @ref ctl_offline_est_t::pmsm_params is valid. */
//    fast_gt flag_error;      /**< output flag error */
//    time_gt task_start_time; /**< a variable to log the start time*/
//
//    // intermediate variables
//    uint16_t sample_count;
//    parameter_gt V_sum, I_sum, Pos_sum; /**< SI单位 */
//    parameter_gt V_sq_sum, I_sq_sum;    /**< SI单位 */
//    uint16_t step_index;
//
//    parameter_gt sum_x, sum_y, sum_xy, sum_x2;
//    parameter_gt avg_torque;
//
//    struct
//    {
//        // Enable flag
//        fast_gt flag_enable; /**< Rs and encoder off-line estimate is enabled */
//
//        // Rs configuration
//        ctrl_gt test_current_pu;    /**< Rs测试中使用的电流标幺值 */
//        ctrl_gt idel_current_pu;    /**< idling period motor current set p.u. */
//        parameter_gt idel_speed_hz; /**< idling period motor elec-speed in Hz */
//
//        // Rs measurement time constant
//        time_gt idling_time;    /**< idling time, during this period, the rotor will be drive for several turns*/
//        time_gt stabilize_time; /**< Rs稳定时间 */
//        time_gt measure_time;   /**< Rs测量时间*/
//
//        fast_gt flag_idling_cmpt; /**< the flag of completing idling period. */
//
//        // intermediate variables
//        parameter_gt step_results[6];       /**< 存储六步法每一步的线电阻测量结果,SI单位 */
//        parameter_gt enc_offset_results[6]; /**< 存储六步法每一步的编码器位置 */
//        ctrl_gt test_angle_pu;              /**< angle of current test*/
//
//        // output variables
//        ctl_vector3_t Rs_line_to_line;      /**< output: PMSM Rs (3phase), judging if motor is connected correctly. */
//        parameter_gt current_noise_std_dev; /**< output: standard deviation of current, in SI unit */
//        parameter_gt position_consistency_std_dev; /**< output: standard deviation of encoder */
//
//    } rs_est;
//
//    struct
//    {
//        // Enable flag
//        /**
//         * @brief Ldq off-line estimate method choose and switch.
//         * 0: disable Ldq estimate,
//         * 1: enable Ldq estimate and use DC bias offset (FIXME: NOT IMPLEMENTED)
//         * 2: enable Ldq estimate and use High frequency rotation vector injection method
//         */
//        fast_gt flag_enable;
//
//        // Ldq configurations
//        ctrl_gt hfi_v_pu;             /**< Ldq辨识时注入的高频电压标幺值 */
//        parameter_gt hfi_freq_hz;     /**< Ldq辨识时注入的高频电压频率 (Hz) */
//        parameter_gt hfi_rot_freq_hz; /**< Ldq辨识时高频矢量的旋转频率 (Hz) */
//
//        // Ldq measurement intermediate variables
//        parameter_gt hfi_i_max, hfi_i_min;
//        parameter_gt hfi_theta_d, hfi_theta_q;
//
//        // Ldq measurement time constant
//        time_gt stabilize_time; /**< Ldq测量稳定时间 */
//        time_gt measure_time;   /**< Ldq测量时间 */
//        time_gt ending_time;    /**< Ldq收尾时间*/
//    } ldq_est;
//
//    struct
//    {
//        // Enable flag
//        fast_gt flag_enable; /**< @f( \psi_f @f) off-line is enabled */
//
//        // psi_f configurations
//        parameter_gt flux_test_iq_pu; /**< Flux测试中用于产生转矩的Iq标幺值 */
//
//        // psi_f measurement intermediate variables
//
//        // psi_f measurement time constant
//
//    } psif_est;
//
//    struct
//    {
//        // Enable flag
//        fast_gt flag_enable; /**< @f( \psi_f @f) off-line is enabled */
//
//        // J estimate configurations
//        parameter_gt test_iq_pu; /**< 惯量J测试中使用的Iq标幺值 */
//
//        // J measurement intermediate variables
//
//        // J measurement time constant
//
//    } inertia_est;
//
//    /*-------------------- 最终辨识结果 (Final Identified Parameters) --------------------*/
//    ctl_pmsm_dsn_consultant_t pmsm_params; /**< output: PMSM parameters */
//    parameter_gt encoder_offset;           /**< output: encoder offset */
//    parameter_gt encoder_offset_ldq;       /**< output: encoder offset measured by Ldq period. */
//
//} ctl_offline_est_t;
//
////================================================================================
//// 3. 核心函数声明与实现 (Core Function Declarations & Implementations)
////================================================================================
//
//// --- 前向声明状态处理函数 ---
//void est_loop_handle_rs(ctl_offline_est_t* est);
//void est_loop_handle_l(ctl_offline_est_t* est);
//void est_loop_handle_flux(ctl_offline_est_t* est);
//void est_loop_handle_j(ctl_offline_est_t* est);
//void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est);
//void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est);
//
///**
// * @brief 初始化离线参数辨识模块.
// * @note FIX: 合并了原头文件中重复的两个初始化函数, 包含所有配置参数.
// */
//GMP_STATIC_INLINE void ctl_init_offline_est(
//    // est object
//    ctl_offline_est_t* est,
//    // input interface
//    mtr_ift* mtr_if, ctl_per_unit_consultant_t* pu_cons,
//    // encoder type
//    ctl_encoder_type_e enc_type,
//    // controller frequency
//    parameter_gt isr_freq,
//    // current PID controller
//    parameter_gt current_kp, parameter_gt current_Ti, parameter_gt out_max_pu)
//{
//    int i;
//
//    // input ports
//    est->mtr_interface = mtr_if;
//    est->pu_consultant = pu_cons;
//
//    // output ports
//    ctl_vector3_clear(&est->vab_command);
//
//    // key config
//    est->encoder_type = enc_type;
//    est->fs = isr_freq;
//    est->qep_search_elec_cycles = 20;
//
//    // inner controller
//    ctl_init_current_controller(
//        // current controller
//        &est->current_ctrl,
//        // pi param
//        current_kp, current_Ti, 0, out_max_pu, -out_max_pu,
//        // fs
//        isr_freq);
//
//    for (i = 0; i < 4; ++i)
//        ctl_clear_lowpass_filter(&est->measure_flt[i]);
//
//    // disable all the measurements
//    est->rs_est.flag_enable = 0;
//    est->ldq_est.flag_enable = 0;
//    est->psif_est.flag_enable = 0;
//    est->inertia_est.flag_enable = 0;
//
//    // system state machines
//    est->main_state = OFFLINE_MAIN_STATE_IDLE;
//    est->sub_state = OFFLINE_SUB_STATE_INIT;
//    est->flag_start = 0;
//    est->flag_complete = 0;
//    est->flag_error = 0;
//
//    // clear intermediate variables
//
//    // clear results
//    ctl_init_pmsm_dsn_consultant(&est->pmsm_params, 0, 0, 0, 0, 0, 0, 0);
//    est->encoder_offset = 0.0f;
//}
//
///**
// * @brief This function will config and enable Rs measurement.
// * @param stabilize_time the time of stabilize, unit Tick.
// * @param measure_time the time of measurement, unit Tick.
// * @param encoder_type encoder type, 0 no encoder is installed, 1 absolute encoder is installed, 2 QEP encoder is installed.
// */
//GMP_STATIC_INLINE void ctl_config_offline_est_Rs(
//    // handle of est
//    ctl_offline_est_t* est,
//    // time settings,idling period should greater than 200ms, 100ms is acceleration time and deceleration time
//    time_gt idling_time, time_gt stabilize_time, time_gt measure_time,
//    // config for idling period
//    fast_gt enable_idling,
//    // test current set
//    parameter_gt idel_speed_hz, parameter_gt idel_current_pu, parameter_gt test_current_pu)
//{
//    int i;
//
//    est->rs_est.flag_enable = 1;
//
//    est->rs_est.test_current_pu = float2ctrl(test_current_pu);
//    est->rs_est.idel_speed_hz = idel_speed_hz;
//    est->rs_est.idel_current_pu = float2ctrl(idel_current_pu);
//
//    est->rs_est.idling_time = idling_time;
//    est->rs_est.stabilize_time = stabilize_time;
//    est->rs_est.measure_time = measure_time;
//
//    if (enable_idling == 1)
//        est->rs_est.flag_idling_cmpt = 0;
//    else
//        est->rs_est.flag_idling_cmpt = 1;
//
//    for (i = 0; i < 6; ++i)
//    {
//        est->rs_est.step_results[i] = 0;
//        est->rs_est.enc_offset_results[i] = 0;
//    }
//
//    ctl_vector3_clear(&est->rs_est.Rs_line_to_line);
//    est->rs_est.current_noise_std_dev = 0;
//    est->rs_est.position_consistency_std_dev = 0;
//}
//
///**
// * @brief This function will config and enable Ldq measurement.
// */
//GMP_STATIC_INLINE void ctl_config_offline_est_Ldq_rotate(
//    // handle of est
//    ctl_offline_est_t* est,
//    // time settings,stabilize period should greater than 200ms, 100ms is acceleration time and deceleration time
//    time_gt stabilize_time, time_gt measure_time,
//    // inject voltage p.u., inject freq[Hz], ratate freq[Hz]
//    parameter_gt v_inj, parameter_gt freq_inj, parameter_gt freq_ratate)
//{
//    est->ldq_est.flag_enable = 1;
//
//    est->ldq_est.hfi_v_pu = float2ctrl(v_inj);
//    est->ldq_est.hfi_freq_hz = freq_inj;
//    est->ldq_est.hfi_rot_freq_hz = freq_ratate;
//
//    est->ldq_est.hfi_i_max = -1.0f;
//    est->ldq_est.hfi_i_min = 1e9; // 一个很大的初始值
//    est->ldq_est.hfi_theta_d = 0.0f;
//    est->ldq_est.hfi_theta_q = 0.0f;
//
//    est->ldq_est.stabilize_time = stabilize_time;
//    est->ldq_est.measure_time = measure_time;
//    est->ldq_est.ending_time = 1000;
//}
//
///**
// * @brief 离线参数辨识模块的实时步进函数 (在ISR中调用).
// * @note FIX: 完全重写了此函数的逻辑, 以确保在不同辨识阶段使用正确的电角度theta.
// * 这是整个修复方案的核心部分.
// */
//GMP_STATIC_INLINE void ctl_step_offline_est(ctl_offline_est_t* est)
//{
//    ctl_vector3_t* iabc = ctl_get_mtr_current(est->mtr_interface);
//    ctrl_gt theta = 0;
//
//    // 只在辨识进行时执行
//    if (est->main_state > OFFLINE_MAIN_STATE_IDLE && est->main_state < OFFLINE_MAIN_STATE_DONE)
//    {
//        // 只有在工作阶段执行电机控制器和测量滤波器等
//        // 根据当前状态选择电机角度的来源
//        // 根据当前状态选择执行特定的滤波器
//        if (est->sub_state == OFFLINE_SUB_STATE_EXEC)
//        {
//            switch (est->main_state)
//            {
//            case OFFLINE_MAIN_STATE_RS:
//
//                // idling period for QEP first index signal is coming.
//                if (est->rs_est.flag_idling_cmpt == 0)
//                {
//                    theta = est->speed_profile_gen.rg.current;
//                    break;
//                }
//
//                // Rs measurement period the target angle is fixed.
//                theta = est->rs_est.test_angle_pu;
//
//                // after stabilize time
//                if (gmp_base_get_diff_system_tick(est->task_start_time) > est->rs_est.stabilize_time)
//                {
//                    // get voltage measurement result
//                    ctrl_gt Vd = ctl_step_lowpass_filter(&est->measure_flt[0], est->current_ctrl.vdq0.dat[0]);
//                    est->V_sum += ctrl2float(Vd);
//                    est->V_sq_sum += ctrl2float(ctl_mul(Vd, Vd));
//
//                    // get current measurement result
//                    ctrl_gt Id = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[0]);
//                    est->I_sum += ctrl2float(Id);
//                    est->I_sq_sum += ctrl2float(ctl_mul(Id, Id));
//
//                    // if encoder is existed, get encoder result
//                    if (est->encoder_type != ENCODER_TYPE_NONE)
//                    {
//                        est->Pos_sum += ctrl2float(
//                            ctl_step_lowpass_filter(&est->measure_flt[2], ctl_get_mtr_elec_theta(est->mtr_interface)));
//                    }
//
//                    est->sample_count++;
//                }
//
//                break;
//
//            case OFFLINE_MAIN_STATE_L:
//                // L辨识时, 根据具体方法确定角度来源
//                if (est->ldq_est.flag_enable == 2)
//                { // 方法2: 旋转HFI法
//
//                    // motor theta is provided by slope generator
//                    theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);
//
//                    ctrl_gt v_hfi_inst;
//
//                    // 当在测量和收敛阶段时，控制器正常运行
//                    if (gmp_base_get_diff_system_tick(est->task_start_time) <
//                        est->ldq_est.stabilize_time + est->ldq_est.measure_time)
//                    {
//                        // 发生信号
//                        ctl_step_sine_generator(&est->hfi_signal_gen);
//                        // 在d轴上注入高频电压, q轴前馈为0,
//                        // 为什么这里是乘法而不是加法？
//                        v_hfi_inst = ctl_mul(ctl_get_sine_generator_sin(&est->hfi_signal_gen), est->ldq_est.hfi_v_pu);
//
//                        // 获取电流环计算出的alpha-beta电流 (这是对实际电流的反馈)
//                        //ctrl_gt i_alpha = est->current_ctrl.iab0.dat[0];
//                        //ctrl_gt i_beta = est->current_ctrl.iab0.dat[1];
//
//                        // 计算高频电流响应的幅值，对幅值进行低通滤波，得到其包络
//                        ctrl_gt i_hfi_mag = ctl_step_lowpass_filter(
//                            &est->measure_flt[3], ctl_vector2_mag((ctl_vector2_t*)&est->current_ctrl.iab0));
//
//                        // 先等待一段时间，等到稳定（控制器收敛、滤波器收敛）之后再读取电流的最大最小值，只是在一段时间内执行
//                        if (gmp_base_get_diff_system_tick(est->task_start_time) > est->ldq_est.stabilize_time)
//                        {
//                            // 寻找电流响应的极大值(对应q轴)和极小值(对应d轴)
//                            if (i_hfi_mag > est->ldq_est.hfi_i_max)
//                            {
//                                est->ldq_est.hfi_i_max = i_hfi_mag;
//                                est->ldq_est.hfi_theta_q =
//                                    ctl_get_ramp_generator_output(&est->speed_profile_gen.rg); // 电流最大处是q轴
//                            }
//                            if (i_hfi_mag < est->ldq_est.hfi_i_min)
//                            {
//                                est->ldq_est.hfi_i_min = i_hfi_mag;
//                                est->ldq_est.hfi_theta_d =
//                                    ctl_get_ramp_generator_output(&est->speed_profile_gen.rg); // 电流最小处是d轴
//                            }
//                        }
//                    }
//                    // 在减速阶段，电压给定要给小
//                    else if (gmp_base_get_diff_system_tick(est->task_start_time) <
//                             (est->ldq_est.stabilize_time + est->ldq_est.measure_time + est->ldq_est.ending_time))
//                    {
//                        v_hfi_inst = est->ldq_est.hfi_v_pu;
//                    }
//                    // 停止后，关闭电压给定
//                    else
//                    {
//                        v_hfi_inst = 0;
//                    }
//
//                    // 在d轴上注入高频电压, q轴前馈为0
//                    ctl_set_voltage_ff(&est->current_ctrl, v_hfi_inst, 0);
//
//                    // 步进旋转发生器, ISR会使用其输出的角度
//                    ctl_step_slope_f(&est->speed_profile_gen);
//                }
//                else
//                {
//                    // 方法1: 直流偏置HFI法
//                    // 角度固定为0, 使d轴对齐alpha轴
//                    theta = 0.0f;
//                }
//
//                break;
//
//            case OFFLINE_MAIN_STATE_FLUX:
//                // 磁链辨识时, 电机开环旋转, 角度由速度发生器提供
//                theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);
//                break;
//
//            case OFFLINE_MAIN_STATE_J:
//                // 惯量辨识时, 电机需要速度反馈, 角度从编码器获取
//                // 此测试强制要求有编码器
//                theta = ctl_get_mtr_elec_theta(est->mtr_interface);
//                break;
//
//            default:
//                // 其他状态或默认情况, 不输出电压
//                ctl_vector3_clear(&est->vab_command);
//
//                break;
//            }
//
//            // --- 3. 执行电流环控制器 ---
//            ctl_step_current_controller(&est->current_ctrl, iabc, theta);
//            ctl_vector3_copy(&est->vab_command, &est->current_ctrl.vab0);
//        }
//        // 当不处于工作状态时，不输出电压
//        else
//        {
//            ctl_vector3_clear(&est->vab_command);
//        }
//    }
//    // 如果不处于辨识状态，关闭输出
//    else
//    {
//        ctl_vector3_clear(&est->vab_command);
//    }
//}
//
///**
// * @brief 离线参数辨识模块的后台循环函数 (在主循环中调用).
// */
//GMP_STATIC_INLINE fast_gt ctl_loop_offline_est(ctl_offline_est_t* est)
//{
//    if (est->flag_start && est->main_state == OFFLINE_MAIN_STATE_IDLE)
//    {
//        // FIX: 根据使能标志位决定起始状态
//        if (est->rs_est.flag_enable)
//        {
//            est->main_state = OFFLINE_MAIN_STATE_RS;
//        }
//        else if (est->ldq_est.flag_enable)
//        {
//            est->main_state = OFFLINE_MAIN_STATE_L;
//        }
//        else if (est->psif_est.flag_enable)
//        {
//            est->main_state = OFFLINE_MAIN_STATE_FLUX;
//        }
//        else if (est->inertia_est.flag_enable)
//        {
//            est->main_state = OFFLINE_MAIN_STATE_J;
//        }
//
//        est->sub_state = OFFLINE_SUB_STATE_INIT;
//        est->flag_start = 0;
//    }
//
//    // 需要为每一个if条件增加一个else，当没有使能时状态机切换到下一个状态。
//    switch (est->main_state)
//    {
//    case OFFLINE_MAIN_STATE_RS:
//        if (est->rs_est.flag_enable)
//            est_loop_handle_rs(est);
//        else
//        {
//            est->main_state = OFFLINE_MAIN_STATE_L;
//            est->sub_state = OFFLINE_SUB_STATE_INIT;
//        }
//        break;
//    case OFFLINE_MAIN_STATE_L:
//        if (est->ldq_est.flag_enable)
//            est_loop_handle_l(est);
//        else
//        {
//            est->main_state = OFFLINE_MAIN_STATE_FLUX;
//            est->sub_state = OFFLINE_SUB_STATE_INIT;
//        }
//        break;
//    case OFFLINE_MAIN_STATE_FLUX:
//        if (est->psif_est.flag_enable)
//            est_loop_handle_flux(est);
//        else
//        {
//            est->main_state = OFFLINE_MAIN_STATE_J;
//            est->sub_state = OFFLINE_SUB_STATE_INIT;
//        }
//        break;
//    case OFFLINE_MAIN_STATE_J:
//        if (est->inertia_est.flag_enable)
//            est_loop_handle_j(est);
//        else
//        {
//            est->main_state = OFFLINE_MAIN_STATE_DONE;
//            est->sub_state = OFFLINE_SUB_STATE_INIT;
//        }
//        break;
//    case OFFLINE_MAIN_STATE_DONE:
//        est->flag_complete = 1;
//        break;
//    case OFFLINE_MAIN_STATE_ERROR:
//        est->flag_error = 1;
//        // 遇到错误时可以停止辨识,并保持当前的状态为ERROR
//        break;
//    case OFFLINE_MAIN_STATE_IDLE:
//    default:
//        break;
//    }
//
//    // ERROR 这里需要增加过流保护模块，当电机电流过大需要进入错误模式。
//
//    return est->flag_complete;
//}

/** 
 * @} 
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_OFFLINE_MOTOR_PARAM_EST_H_
