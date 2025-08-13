/**
 * @file offline_motor_param_est_handlers.c
 * @author Javnson & Gemini
 * @brief State machine handler implementations for the offline parameter estimator.
 * @version 0.5
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#include "offline_motor_param_est.h"
#include <math.h>   // For sqrtf
#include <stdlib.h> // For fabsf

// 定义六步法注入磁场的电角度 (U->V, W->V, W->U, V->U, V->W, U->W)
// 对应于 30, 90, 150, 210, 270, 330 度
static const float SIX_STEP_ANGLES_DEG[6] = {30.0f, 90.0f, 150.0f, 210.0f, 270.0f, 330.0f};

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
        // 1. 配置低通滤波器
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

        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_QEP_INDEX_SEARCH: {
        // TODO: 实现寻找QEP index的逻辑
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 注入特定方向的电流 ---
        float target_angle_pu = SIX_STEP_ANGLES_DEG[est->step_index] / 360.0f;
        // TODO: 此处需要一个机制来设置FOC的参考角度theta
        // 假设 ctl_step_current_controller 的 theta 输入由外部更新为 target_angle_pu
        // 这是一个简化的占位符，实际应用中可能需要 est->current_ctrl.theta_override = target_angle_pu;

        // --- 2. 等待电流稳定 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, RS_STABILIZE_TIME_MS))
        {
            return;
        }

        // --- 3. 稳定后，在指定时间内进行测量和累加 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, RS_STABILIZE_TIME_MS + RS_MEASURE_TIME_MS))
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

            est->rs_step_results[est->step_index] = (I_mean > 1e-3) ? (V_mean / I_mean) : 0.0f;

            if (est->encoder_type != ENCODER_TYPE_NONE)
            {
                est->enc_offset_results[est->step_index] = est->Pos_sum / est->sample_count;
            }

            parameter_gt I_std_dev = sqrtf(fabsf((est->I_sq_sum / est->sample_count) - (I_mean * I_mean)));
            est->current_noise_std_dev += I_std_dev;
        }

        // --- 5. 准备进入下一步 ---
        est->step_index++;
        if (est->step_index >= 6)
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
            est->task_start_time = gmp_base_get_system_tick();
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算三组线电阻的平均值
        // R_uv (30 deg) vs R_vu (210 deg)
        est->Rs_line_to_line.dat[0] = (est->rs_step_results[0] + est->rs_step_results[3]) / 2.0f;
        // R_wv (90 deg) vs R_vw (270 deg) - 注意W在V前面
        est->Rs_line_to_line.dat[1] = (est->rs_step_results[1] + est->rs_step_results[4]) / 2.0f;
        // R_wu (150 deg) vs R_uw (330 deg)
        est->Rs_line_to_line.dat[2] = (est->rs_step_results[2] + est->rs_step_results[5]) / 2.0f;

        // 2. 计算最终的平均相电阻 R_phase = R_line / 2
        est->pmsm_params.Rs =
            (est->Rs_line_to_line.dat[0] + est->Rs_line_to_line.dat[1] + est->Rs_line_to_line.dat[2]) / 6.0f;

        // 3. 计算平均电流噪声
        est->current_noise_std_dev /= 6.0f;

        // 4. 计算编码器偏置和健康度
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // 以第一步(U->V)为基准，理论电气角度为30度
            parameter_gt base_angle_pu = SIX_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->enc_offset_results[0];
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;

            // 检查步进一致性 (理论步进60度)
            parameter_gt step_diff_sum = 0;
            parameter_gt step_diff_sq_sum = 0;
            parameter_gt sixty_deg_pu = 60.0f / 360.0f;
            for (int i = 0; i < 5; ++i)
            {
                parameter_gt diff = est->enc_offset_results[i + 1] - est->enc_offset_results[i];
                if (diff < 0)
                    diff += 1.0f; // 处理回绕
                parameter_gt error = diff - sixty_deg_pu;
                step_diff_sum += error;
                step_diff_sq_sum += error * error;
            }
            parameter_gt mean_error = step_diff_sum / 5.0f;
            est->position_consistency_std_dev = sqrtf(fabsf(step_diff_sq_sum / 5.0f - mean_error * mean_error));
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
 * @brief 电感辨识的主分发函数.
 * @note  为了支持此功能, 需要在 ctl_offline_est_t 中增加一个枚举成员
 * `ctl_l_est_method_e l_est_method;` 并在初始化时由用户指定.
 */
void est_loop_handle_l(ctl_offline_est_t* est)
{
    // // 假设默认使用直流偏置法，实际应用中应由配置决定
    // if (est->l_est_method == L_EST_METHOD_DC_BIAS_HFI) {
    //     est_loop_handle_l_dcbias_hfi(est);
    // } else {
    //     est_loop_handle_l_rotating_hfi(est);
    // }

    // 当前直接调用直流偏置HFI法
    est_loop_handle_l_dcbias_hfi(est);
}


/**
 * @brief 电感辨识的主循环处理函数.
 */
void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est)
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
        est->task_start_time = gmp_base_get_system_tick();
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
        if (gmp_base_is_delay_elapsed(est->task_start_time, rotation_time_ms))
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
 * @brief [方法二] 使用直流偏置HFI法辨识电感.
 * @note  为了支持此功能, 需要在 ctl_offline_est_t 中增加以下成员:
 * `ctl_filter_IIR1_t hpf_v, hpf_i;`
 * `parameter_gt hfi_v_rms[2], hfi_i_rms[2];`
 * 并在 ctl_step_offline_est 中增加逻辑以在L辨识期间将theta固定为0.
 */
static void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 初始化高通滤波器用于提取交流分量
        parameter_gt hpf_fc = est->l_hfi_freq_hz / 10.0f; // e.g., 1/10 of HFI frequency
        // ctl_init_filter_iir1_hpf(&est->hpf_v, est->isr_freq_hz, hpf_fc);
        // ctl_init_filter_iir1_hpf(&est->hpf_i, est->isr_freq_hz, hpf_fc);

        // 2. 初始化高频注入信号发生器
        parameter_gt step_angle = est->l_hfi_freq_hz / est->isr_freq_hz;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 3. 配置电流控制器以施加直流偏置，锁定转子d轴到alpha轴
        parameter_gt id_ref = ctl_consult_Ipeak_to_phy(pu, est->rs_test_current_pu);
        ctl_set_current_ref(&est->current_ctrl, id_ref, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);
        // **重要**: 此时需确保 ctl_step_offline_est 中传入Park变换的theta固定为0

        // 4. 初始化变量
        est->step_index = 0; // 0 for d-axis, 1 for q-axis
        est->sum_x2 = 0;     // V_ac^2
        est->sum_y2 = 0;     // I_ac^2
        est->sample_count = 0;

        // 5. 进入执行状态
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 等待转子对齐并稳定 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, L_DCBIAS_ALIGN_TIME_MS))
        {
            return;
        }

        // --- 2. 注入高频信号并持续测量 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, L_DCBIAS_ALIGN_TIME_MS + L_DCBIAS_MEASURE_TIME_MS))
        {
            // 产生高频电压信号
            ctl_step_sine_generator(&est->hfi_signal_gen);
            parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(pu, est->l_hfi_v_pu);
            parameter_gt v_hfi_inst = ctl_get_sine_generator_sin(&est->hfi_signal_gen) * v_hfi_peak;

            parameter_gt V_ac = 0, I_ac = 0;
            if (est->step_index == 0)
            { // 测量 d 轴
                ctl_set_voltage_ff(&est->current_ctrl, v_hfi_inst, 0.0f);
                // V_ac = ctl_step_filter_iir1(&est->hpf_v, est->current_ctrl.vdq0.dat[0]);
                // I_ac = ctl_step_filter_iir1(&est->hpf_i, est->current_ctrl.idq0.dat[0]);
            }
            else
            { // 测量 q 轴
                ctl_set_voltage_ff(&est->current_ctrl, 0.0f, v_hfi_inst);
                // V_ac = ctl_step_filter_iir1(&est->hpf_v, est->current_ctrl.vdq0.dat[1]);
                // I_ac = ctl_step_filter_iir1(&est->hpf_i, est->current_ctrl.idq0.dat[1]);
            }

            // 累加平方和以计算RMS
            est->sum_x2 += V_ac * V_ac;
            est->sum_y2 += I_ac * I_ac;
            est->sample_count++;
            return;
        }

        // --- 3. 测量结束，保存结果并准备下一步 ---
        if (est->sample_count > 0)
        {
            // est->hfi_v_rms[est->step_index] = ctl_sqrt(est->sum_x2 / est->sample_count);
            // est->hfi_i_rms[est->step_index] = ctl_sqrt(est->sum_y2 / est->sample_count);
        }

        est->step_index++;
        if (est->step_index >= 2)
        { // d 和 q 轴都测量完毕
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        else
        {                                                     // 准备测量下一个轴
            est->sub_state = OFFLINE_SUB_STATE_EXEC;          // 保持在EXEC状态
            est->task_start_time = gmp_base_get_system_tick(); // 重置计时器
            est->sample_count = 0;
            est->sum_x2 = 0;
            est->sum_y2 = 0;
            // ctl_clear_filter_iir1(&est->hpf_v);
            // ctl_clear_filter_iir1(&est->hpf_i);
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算阻抗
        // parameter_gt z_d = (est->hfi_i_rms[0] > 1e-6) ? (est->hfi_v_rms[0] / est->hfi_i_rms[0]) : 0.0f;
        // parameter_gt z_q = (est->hfi_i_rms[1] > 1e-6) ? (est->hfi_v_rms[1] / est->hfi_i_rms[1]) : 0.0f;
        parameter_gt z_d = 0, z_q = 0; // 占位符

        // 2. 计算电感 (更精确的方法应考虑电阻: Z^2 = R^2 + (ωL)^2)
        parameter_gt omega_hfi = CTL_PARAM_CONST_2PI * est->l_hfi_freq_hz;
        if (omega_hfi > 1e-3)
        {
            parameter_gt Rs = est->pmsm_params.Rs;
            // est->pmsm_params.Ld = ctl_sqrt(fmaxf(0.0f, z_d*z_d - Rs*Rs)) / omega_hfi;
            // est->pmsm_params.Lq = ctl_sqrt(fmaxf(0.0f, z_q*z_q - Rs*Rs)) / omega_hfi;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. 停止电机
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);

        // 2. 切换到下一个主状态
        est->main_state = OFFLINE_MAIN_STATE_FLUX;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}



// 定义参数
#define FLUX_RAMP_UP_TIME_S    (2.0f) // 斜坡升速时间 (秒)
#define FLUX_STABILIZE_TIME_MS (1000) // 每个速度点的稳定时间
#define FLUX_MEASURE_TIME_MS   (1000) // 每个速度点的测量时间
#define FLUX_TEST_POINTS       (4)    // 磁链测试的速度点数量

// 定义测试速度点 (以额定速度的百分比表示)
static const float FLUX_TEST_SPEED_PU[FLUX_TEST_POINTS] = {0.25f, 0.5f, 0.75f, 1.0f};

/**
 * @brief 磁链辨识的主循环处理函数.
 */
void est_loop_handle_flux(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 配置电流控制器为 Id=0 模式
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 2. 配置滤波器
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 20.0f); // Uq
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 20.0f); // Omega_e

        // 3. 初始化变量
        est->step_index = 0; // 用于遍历速度点
        est->V_sum = 0;      // 用作 Uq*Omega_e 的累加和
        est->I_sum = 0;      // 用作 Omega_e^2 的累加和
        est->Pos_sum = 0;    // 用作 Uq 的累加和
        est->V_sq_sum = 0;   // 用作 Omega_e 的累加和
        est->sample_count = 0;

        // 4. 进入执行状态
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 设置当前目标速度并斜坡升速 ---
        parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
        parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
        parameter_gt ramp_time_s = FLUX_RAMP_UP_TIME_S;

        // 配置速度发生器
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz, target_freq_hz / ramp_time_s,
                                          est->isr_freq_hz);

        // --- 2. 等待电机达到目标速度并稳定 ---
        uint32_t wait_time_ms = (uint32_t)(ramp_time_s * 1000) + FLUX_STABILIZE_TIME_MS;
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms))
        {
            // 在ISR中，FOC会跟随 speed_profile_gen 的输出角度旋转
            ctl_step_slope_f(&est->speed_profile_gen);
            return;
        }

        // --- 3. 在稳定状态下测量指定时间 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms + FLUX_MEASURE_TIME_MS))
        {
            // 获取q轴电压和电角速度
            parameter_gt Uq = est->current_ctrl.vdq0.dat[1];
            parameter_gt Omega_e = ctl_get_mtr_velocity(est->mtr_interface) * ctl_consult_pole_pairs(pu) *
                                   CTL_PARAM_CONST_2PI; // 假设速度单位为 rps

            // 滤波
            Uq = ctl_step_lowpass_filter(&est->measure_flt[0], Uq);
            Omega_e = ctl_step_lowpass_filter(&est->measure_flt[1], Omega_e);

            // 累加用于线性回归计算 (最小二乘法)
            // ψf = Σ(Uq*ωe) / Σ(ωe^2)
            est->V_sum += Uq * Omega_e;
            est->I_sum += Omega_e * Omega_e;
            est->sample_count++;
            return;
        }

        // --- 4. 准备下一个速度点 ---
        est->step_index++;
        if (est->step_index >= FLUX_TEST_POINTS)
        {
            // 所有速度点测试完成
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        else
        {
            // 重置计时器，进入下一个速度点的测试
            est->task_start_time = gmp_base_get_system_tick();
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 使用最小二乘法计算斜率，即磁链
        // Formula: slope = Σ(xy) / Σ(x^2), where x=ωe, y=Uq
        if (est->I_sum > 1e-3)
        { // 确保分母不为零
            est->pmsm_params.flux = est->V_sum / est->I_sum;
        }
        else
        {
            est->pmsm_params.flux = 0.0f; // 错误情况
            est->main_state = OFFLINE_MAIN_STATE_ERROR;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. 让电机停下来
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, 0.0f,
                                          FLUX_TEST_SPEED_PU[FLUX_TEST_POINTS - 1] * ctl_consult_base_frequency(pu) /
                                              FLUX_RAMP_UP_TIME_S,
                                          est->isr_freq_hz);
        // TODO: 此处应有一个等待电机停止的延时

        // 2. 切换到下一个主状态
        est->main_state = OFFLINE_MAIN_STATE_J;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

/**
 * @brief 惯量辨识的主循环处理函数.
 */
void est_loop_handle_j(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 检查磁链是否已辨识
        if (est->pmsm_params.flux < 1e-6)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // 无法计算转矩
            return;
        }

        // 2. 配置电流控制器为 Id=0, Iq=0
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 3. 配置滤波器
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 50.0f); // Speed
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 50.0f); // Iq

        // 4. 初始化变量
        est->sum_x = 0;
        est->sum_y = 0;
        est->sum_xy = 0;
        est->sum_x2 = 0;
        est->avg_torque = 0;
        est->sample_count = 0;

        // 5. 进入执行状态
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 等待一小段时间以确保电机静止 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, J_STABILIZE_TIME_MS))
        {
            return;
        }

        // --- 2. 施加转矩阶跃并持续测量 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, J_STABILIZE_TIME_MS + J_TORQUE_STEP_TIME_MS))
        {
            // 设置目标Iq
            parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->j_test_iq_pu);
            ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);

            // 获取并滤波测量值
            parameter_gt speed_rps = ctl_get_mtr_velocity(est->mtr_interface); // 假设单位为 rps
            parameter_gt speed_rads = ctl_step_lowpass_filter(&est->measure_flt[0], speed_rps * CTL_PARAM_CONST_2PI);
            parameter_gt iq_measured = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[1]);

            // 获取当前时间 (秒)
            parameter_gt time_s = (gmp_base_get_system_tick() - (est->task_start_time + J_STABILIZE_TIME_MS)) / 1000.0f;

            // 累加用于线性回归和平均转矩计算
            est->sum_x += time_s;               // Σt
            est->sum_y += speed_rads;           // Σω
            est->sum_xy += time_s * speed_rads; // Σ(t*ω)
            est->sum_x2 += time_s * time_s;     // Σ(t^2)
            est->avg_torque += 1.5f * est->pmsm_params.pole_pair * est->pmsm_params.flux * iq_measured;
            est->sample_count++;

            return;
        }

        // --- 3. 测量时间结束，进入计算状态 ---
        est->sub_state = OFFLINE_SUB_STATE_CALC;
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        if (est->sample_count > 10)
        { // 确保有足够的数据点
            // 1. 计算平均转矩
            est->avg_torque /= est->sample_count;

            // 2. 计算角加速度 (线性回归斜率)
            // alpha = (N*Σ(tω) - Σt*Σω) / (N*Σ(t^2) - (Σt)^2)
            parameter_gt N = est->sample_count;
            parameter_gt numerator = N * est->sum_xy - est->sum_x * est->sum_y;
            parameter_gt denominator = N * est->sum_x2 - est->sum_x * est->sum_x;

            if (fabsf(denominator) > 1e-9)
            {
                parameter_gt alpha = numerator / denominator;
                // 3. 计算惯量 J = Te / alpha
                if (fabsf(alpha) > 1e-3)
                {
                    est->pmsm_params.inertia = est->avg_torque / alpha;
                }
                else
                {
                    est->main_state = OFFLINE_MAIN_STATE_ERROR; // 加速度过小
                }
            }
            else
            {
                est->main_state = OFFLINE_MAIN_STATE_ERROR; // 数据错误
            }
        }
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // 采样点不足
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. 停止电机
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);

        // 2. 切换到最终完成状态
        est->main_state = OFFLINE_MAIN_STATE_DONE;
        est->sub_state = OFFLINE_SUB_STATE_INIT; // 重置子状态
        break;
    }
    }
}
