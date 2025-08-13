/**
 * @file offline_motor_param_est_handlers.c
 * @author Javnson & Gemini
 * @brief State machine handler implementations for the offline parameter estimator.
 * @version 0.4
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#include "offline_motor_param_est.h"
#include <math.h>   // For sqrtf
#include <stdlib.h> // For fabsf

// 定义三步法注入磁场的电角度 (U->V, V->W, W->U)
static const float THREE_STEP_ANGLES_DEG[3] = {30.0f, 150.0f, 270.0f};

// 定义参数
#define RS_STABILIZE_TIME_MS (1000) // 每步稳定1000ms
#define RS_MEASURE_TIME_MS   (500)  // 稳定后测量500ms

/**
 * @brief Rs和编码器偏置辨识的主循环处理函数.
 */
static void est_loop_handle_rs(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 配置低通滤波器 (例如，5Hz截止频率)
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 5.0f); // Vd
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 5.0f); // Id
        ctl_init_lp_filter(&est->measure_flt[2], est->isr_freq_hz, 5.0f); // Position

        // 2. 配置电流控制器注入目标电流
        parameter_gt id_ref = ctl_consult_Ipeak_to_phy(pu, est->rs_test_current_pu);
        ctl_set_current_ref(&est->current_ctrl, id_ref, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        est->step_index = 0;

        // 3. 根据编码器类型决定下一步
        if (est->encoder_type == ENCODER_TYPE_QEP)
        {
            est->sub_state = OFFLINE_SUB_STATE_QEP_INDEX_SEARCH;
            // TODO: 配置速度发生器以低速旋转
        }
        else
        {
            est->sub_state = OFFLINE_SUB_STATE_EXEC;
        }

        est->task_start_time = gmp_core_get_systemtick();
        break;
    }

    case OFFLINE_SUB_STATE_QEP_INDEX_SEARCH: {
        // TODO: 实现寻找QEP index的逻辑
        // 1. 使用 vf_generator 低速旋转电机 est->qep_search_elec_cycles 圈
        // 2. 硬件应在此过程中捕获到index信号
        // 3. 旋转完成后，切换到 EXEC 状态
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 注入特定方向的电流 ---
        float target_angle_pu = THREE_STEP_ANGLES_DEG[est->step_index] / 360.0f;
        // 通过设置Park变换的角度来旋转物理磁场
        // TODO: 需配合顶层应用设置FOC的参考角度theta
        // 假设 ctl_step_current_controller 的 theta 输入由外部更新为 target_angle_pu

        // --- 2. 等待电流稳定 ---
        if (!est_is_delay_elapsed_ms(est->task_start_time, RS_STABILIZE_TIME_MS))
        {
            return; // 等待
        }

        // --- 3. 稳定后，在指定时间内进行测量和累加 ---
        if (!est_is_delay_elapsed_ms(est->task_start_time, RS_STABILIZE_TIME_MS + RS_MEASURE_TIME_MS))
        {
            parameter_gt Vd = est->current_ctrl.vdq0.dat[0];
            parameter_gt Id = est->current_ctrl.idq0.dat[0];
            parameter_gt Pos = ctl_get_mtr_elec_theta(est->mtr_interface);

            Vd = ctl_step_lowpass_filter(&est->measure_flt[0], Vd);
            Id = ctl_step_lowpass_filter(&est->measure_flt[1], Id);
            if (est->encoder_type != ENCODER_TYPE_NONE)
            {
                Pos = ctl_step_lowpass_filter(&est->measure_flt[2], Pos);
            }

            est->V_sum += Vd;
            est->I_sum += Id;
            est->V_sq_sum += Vd * Vd;
            est->I_sq_sum += Id * Id;
            if (est->encoder_type != ENCODER_TYPE_NONE)
            {
                est->Pos_sum += Pos;
            }
            est->sample_count++;
            return;
        }

        // --- 4. 测量时间结束，计算本次结果并进入下一步 ---
        if (est->sample_count > 0)
        {
            parameter_gt V_mean = est->V_sum / est->sample_count;
            parameter_gt I_mean = est->I_sum / est->sample_count;

            est->rs_line_results[est->step_index] = (I_mean > 1e-3) ? (V_mean / I_mean) : 0.0f;

            if (est->encoder_type != ENCODER_TYPE_NONE)
            {
                est->enc_offset_results[est->step_index] = est->Pos_sum / est->sample_count;
            }

            parameter_gt I_std_dev = sqrtf(fabsf((est->I_sq_sum / est->sample_count) - (I_mean * I_mean)));
            est->current_noise_std_dev += I_std_dev;
        }

        // --- 5. 准备进入下一步 ---
        est->step_index++;
        if (est->step_index >= 3)
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        else
        {
            est->sample_count = 0;
            est->V_sum = 0;
            est->I_sum = 0;
            est->Pos_sum = 0;
            est->V_sq_sum = 0;
            est->I_sq_sum = 0;
            est->task_start_time = gmp_core_get_systemtick();
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算最终电阻值
        est->Rs_line_to_line.dat[0] = est->rs_line_results[0]; // R_uv
        est->Rs_line_to_line.dat[1] = est->rs_line_results[1]; // R_vw
        est->Rs_line_to_line.dat[2] = est->rs_line_results[2]; // R_wu
        // 平均相电阻 = 平均线电阻 / 2
        est->pmsm_params.Rs = (est->rs_line_results[0] + est->rs_line_results[1] + est->rs_line_results[2]) / 6.0f;

        // 2. 计算平均电流噪声
        est->current_noise_std_dev /= 3.0f;

        // 3. 计算编码器偏置和健康度
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // 以第一步(U->V)为基准，理论电气角度为30度
            parameter_gt base_angle_pu = THREE_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->enc_offset_results[0];
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;

            // 检查步进一致性
            parameter_gt diff1 = fabsf(est->enc_offset_results[1] - est->enc_offset_results[0]);
            parameter_gt diff2 = fabsf(est->enc_offset_results[2] - est->enc_offset_results[1]);
            // 理论电气角度差120度 (0.333 pu)
            parameter_gt err1 = fabsf(diff1 - 120.0f / 360.0f);
            parameter_gt err2 = fabsf(diff2 - 120.0f / 360.0f);
            // 此处可以计算标准差，或用一个更简单的误差平均值
            est->position_consistency_std_dev = (err1 + err2) / 2.0f;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        ctl_disable_current_controller(&est->current_ctrl);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        est->main_state = OFFLINE_MAIN_STATE_L;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

/**
 * @brief 电感辨识的主循环处理函数.
 */
static void est_loop_handle_l(ctl_offline_est_t* est)
{
    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 初始化高频注入信号发生器
        parameter_gt step_angle = est->l_hfi_freq_hz / est->isr_freq_hz;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 2. 初始化慢速旋转发生器
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, est->l_hfi_rot_freq_hz,
                                          est->l_hfi_rot_freq_hz * 2.0f, // 0.5s 加速时间
                                          est->isr_freq_hz);

        // 3. 配置电流控制器为开环电压模式
        ctl_disable_current_controller(&est->current_ctrl);

        // 4. 初始化滤波器和测量变量
        ctl_init_lp_filter(&est->measure_flt[3], est->isr_freq_hz, est->l_hfi_rot_freq_hz * 5.0f); // 滤波HFI电流响应
        est->hfi_i_max = -1.0f;
        est->hfi_i_min = 1e9; // 一个很大的初始值
        est->hfi_theta_d = 0.0f;
        est->hfi_theta_q = 0.0f;

        // 5. 记录开始时间并切换状态
        est->task_start_time = gmp_core_get_systemtick();
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // 慢速旋转注入矢量
        ctl_step_slope_f(&est->speed_profile_gen);

        // 获取电流环计算出的alpha-beta电流 (这是对实际电流的反馈)
        parameter_gt i_alpha = est->current_ctrl.iab0.dat[0];
        parameter_gt i_beta = est->current_ctrl.iab0.dat[1];

        // 计算高频电流响应的幅值
        parameter_gt i_hfi_mag = ctl_sqrt(i_alpha * i_alpha + i_beta * i_beta);

        // 对幅值进行低通滤波，得到其包络
        i_hfi_mag = ctl_step_lowpass_filter(&est->measure_flt[3], i_hfi_mag);

        // 获取当前的注入角度
        parameter_gt current_theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);

        // 寻找电流响应的极大值和极小值
        if (i_hfi_mag > est->hfi_i_max)
        {
            est->hfi_i_max = i_hfi_mag;
            est->hfi_theta_d = current_theta;
        }
        if (i_hfi_mag < est->hfi_i_min)
        {
            est->hfi_i_min = i_hfi_mag;
            est->hfi_theta_q = current_theta;
        }

        // 检查是否旋转了足够长的时间 (例如，旋转1.5圈以确保稳定)
        uint32_t rotation_time_ms = (uint32_t)(1.5f / est->l_hfi_rot_freq_hz * 1000.0f);
        if (est_is_delay_elapsed_ms(est->task_start_time, rotation_time_ms))
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算阻抗
        parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(est->pu_consultant, est->l_hfi_v_pu);
        parameter_gt z_d = (est->hfi_i_max > 1e-3) ? (v_hfi_peak / est->hfi_i_max) : 0.0f;
        parameter_gt z_q = (est->hfi_i_min > 1e-3) ? (v_hfi_peak / est->hfi_i_min) : 0.0f;

        // 2. 计算电感 (忽略电阻, Z ≈ ωL)
        parameter_gt omega_hfi = CTL_PARAM_CONST_2PI * est->l_hfi_freq_hz;
        if (omega_hfi > 1e-3)
        {
            est->pmsm_params.Ld = z_d / omega_hfi;
            est->pmsm_params.Lq = z_q / omega_hfi;
        }

        // 3. 如果有编码器，可以进一步校准编码器零偏
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // d轴方向是 hfi_theta_d，此时转子的真实d轴应该对齐这个方向
            // 读取此时编码器的真实位置
            parameter_gt actual_pos = ctl_get_mtr_elec_theta(est->mtr_interface);
            // 新的零偏 = 理论d轴角 - 实际d轴角
            est->encoder_offset = est->hfi_theta_d - actual_pos;
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. 关闭电机励磁
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl); // 恢复闭环模式

        // 2. 切换到下一个主状态
        est->main_state = OFFLINE_MAIN_STATE_FLUX;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

/**
 * @brief 磁链辨识的主循环处理函数.
 */
static void est_loop_handle_flux(ctl_offline_est_t* est)
{
    // ... 此处将实现磁链辨识的子状态机逻辑 ...
}

/**
 * @brief 惯量辨识的主循环处理函数.
 */
static void est_loop_handle_j(ctl_offline_est_t* est)
{
    // ... 此处将实现惯量辨识的子状态机逻辑 ...
}
