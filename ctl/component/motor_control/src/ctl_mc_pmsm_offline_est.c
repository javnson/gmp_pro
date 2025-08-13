/**
 * @file offline_motor_param_est_handlers.c
 * @author Javnson & Gemini
 * @brief State machine handler implementations for the offline parameter estimator.
 * @version 0.3
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#include "offline_motor_param_est.h"
#include <math.h>   // For sqrtf
#include <stdlib.h> // For fabsf

// 定义三步法注入磁场的电角度 (U->V, V->W, W->U)
// 分别对应 30, 150, 270 度
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
    // ... 此处将实现电感辨识的子状态机逻辑 ...
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
