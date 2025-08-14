/**
 * @file offline_motor_param_est_handlers.c
 * @author Javnson 
 * @brief State machine handler implementations for the offline parameter estimator.
 * @version 0.8 (Fixed)
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#include <gmp_core.h>

#include <ctl/component/motor_control/param_est/pmsm_offline_est.h>

#include <math.h>   // For sqrtf
#include <stdlib.h> // For fabsf

// 定义六步法注入磁场的电角度 (U->V, W->V, W->U, V->U, V->W, U->W)
// 对应于 30, 90, 150, 210, 270, 330 度 (单位: 度)
static const float SIX_STEP_ANGLES_DEG[6] = {30.0f, 90.0f, 150.0f, 210.0f, 270.0f, 330.0f};

// 定义各阶段的延时参数
#define RS_STABILIZE_TIME_MS     (1000) // Rs辨识：每步稳定时间1000ms
#define RS_MEASURE_TIME_MS       (500)  // Rs辨识：稳定后测量时间500ms
#define L_DCBIAS_ALIGN_TIME_MS   (1500) // L辨识(直流偏置法)：转子对齐时间
#define L_DCBIAS_MEASURE_TIME_MS (1000) // L辨识(直流偏置法)：测量时间
#define FLUX_RAMP_UP_TIME_S      (2.0f) // 磁链辨识：斜坡升速时间 (秒)
#define FLUX_STABILIZE_TIME_MS   (1000) // 磁链辨识：每个速度点的稳定时间
#define FLUX_MEASURE_TIME_MS     (1000) // 磁链辨识：每个速度点的测量时间
#define J_STABILIZE_TIME_MS      (500)  // 惯量辨识：静止稳定时间
#define J_TORQUE_STEP_TIME_MS    (1000) // 惯量辨识：转矩阶跃和测量时间

// 有一个重要问题需要在计算逻辑中修正，目前的计算时基于实际值控制的，但是实际情况是基于标幺值控制的。
// 所以需要调整电机电流控制器的代码，让电机的电流控制器的输出正确标幺/换算。

// 在每一个步骤的开始阶段需要初始化电机控制器。

// 先改电机的电流控制器

// 电机工作在J观测时保持使用虚拟角度，而不是实际角度，因为实际角度的闭环不一定能够稳定运行，这一条需要加一个标志位。

// 上面这些宏需要作为参数写在类中，只不过这些参数在初始化时都给出默认值。

// offline estimate module for PMSM, idling accelerate time, in [ms] unit.
#define CTL_MC_OFFLINE_EST_IDLING_ACCELERATE_TIME (100)

/**
 * @brief Rs和编码器偏置辨识的主循环处理函数.
 */
static void est_loop_handle_rs(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 配置低通滤波器和斜坡发生器
        ctl_init_const_slope_f_controller(
            &est->speed_profile_gen, est->rs_est.idel_speed_hz,
            est->rs_est.idel_speed_hz * 1000.0f / CTL_MC_OFFLINE_EST_IDLING_ACCELERATE_TIME, est->fs);

        ctl_init_lp_filter(&est->measure_flt[0], est->fs, 5.0f); // Vd
        ctl_init_lp_filter(&est->measure_flt[1], est->fs, 5.0f); // Id
        ctl_init_lp_filter(&est->measure_flt[2], est->fs, 5.0f); // Position

        // 2. provide a zero current (Id = 0, Iq = 0)
        ctl_set_current_ref(&est->current_ctrl, 0, 0);
        ctl_clear_current_controller(&est->current_ctrl);
        ctl_enable_current_controller(&est->current_ctrl);

        est->step_index = 0;

        // 3. 根据编码器类型决定下一步 (QEP index搜索功能暂未实现)
        if (est->encoder_type == ENCODER_TYPE_QEP)
        {
            // est->sub_state = OFFLINE_SUB_STATE_QEP_INDEX_SEARCH;
            // TODO: 配置速度发生器以低速旋转寻找Z脉冲
            est->sub_state = OFFLINE_SUB_STATE_EXEC; // 暂时跳过
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
        // Step I idling period
        if (est->rs_est.flag_idling_cmpt == 0)
        {
            // after idling period set complete flag
            if (gmp_base_get_diff_system_tick(est->task_start_time) > est->rs_est.idling_time)
            {
                est->rs_est.flag_idling_cmpt = 1;
                // clear task_start_time
                est->task_start_time = gmp_base_get_system_tick();
                ctl_set_current_ref(&est->current_ctrl, 0, 0);
            }
            // decelerate period
            else if (gmp_base_get_diff_system_tick(est->task_start_time) >
                     (est->rs_est.idling_time - CTL_MC_OFFLINE_EST_IDLING_ACCELERATE_TIME))
            {
                ctl_set_slope_f_freq(&est->speed_profile_gen, 0, est->fs);
                ctl_set_current_ref(&est->current_ctrl, 0, 0);
            }
            // accelerate period
            else
            {
                ctl_set_slope_f_freq(&est->speed_profile_gen, est->rs_est.idel_speed_hz, est->fs);
                ctl_set_current_ref(&est->current_ctrl, est->rs_est.idel_current_pu, 0);
            }
        }

        // Step II during measuring period, provide a fixed angle and a fixed current
        est->rs_est.test_angle_pu = float2ctrl(SIX_STEP_ANGLES_DEG[est->step_index] / 360.0f);
        ctl_set_current_ref(&est->current_ctrl, est->rs_est.test_current_pu, 0.0f);

        // Step III waiting for measurement is complete
        if (!gmp_base_is_delay_elapsed(est->task_start_time, est->rs_est.stabilize_time + est->rs_est.measure_time))
        {
            return;
        }

        // Step IV complete period
        if (est->sample_count > 0)
        {
            parameter_gt V_mean = est->V_sum / est->sample_count;
            parameter_gt I_mean = est->I_sum / est->sample_count;

            if (fabsf(I_mean) > 1e-3)
                est->rs_est.step_results[est->step_index] = V_mean / I_mean * ctl_consult_base_impedance(pu);
            else
                est->rs_est.step_results[est->step_index] = 0.0f;

            if (est->encoder_type != ENCODER_TYPE_NONE)
                est->rs_est.enc_offset_results[est->step_index] = est->Pos_sum / est->sample_count;

            est->rs_est.current_noise_std_dev +=
                sqrtf(fabsf((est->I_sq_sum / est->sample_count) - (I_mean * I_mean))) * ctl_consult_base_current(pu);
        }

        // Step V step to next position
        est->step_index++;
        if (est->step_index >= 6)
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        else
        {
            // 清零累加器, 为下一次测量做准备
            est->sample_count = 0;
            est->V_sum = 0;
            est->I_sum = 0;
            est->Pos_sum = 0;
            est->V_sq_sum = 0;
            est->I_sq_sum = 0;
            est->task_start_time = gmp_base_get_system_tick(); // 重置计时器
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算三组线电阻的平均值
        est->rs_est.Rs_line_to_line.dat[0] =
            (est->rs_est.step_results[0] + est->rs_est.step_results[3]) / 2.0f; // U-V vs V-U
        est->rs_est.Rs_line_to_line.dat[1] =
            (est->rs_est.step_results[1] + est->rs_est.step_results[4]) / 2.0f; // W-V vs V-W
        est->rs_est.Rs_line_to_line.dat[2] =
            (est->rs_est.step_results[2] + est->rs_est.step_results[5]) / 2.0f; // W-U vs U-W

        // 2. 计算最终的平均相电阻 R_phase = R_line_avg / 2
        est->pmsm_params.Rs = (est->rs_est.Rs_line_to_line.dat[0] + est->rs_est.Rs_line_to_line.dat[1] +
                               est->rs_est.Rs_line_to_line.dat[2]) /
                              6.0f;

        // 3. 计算平均电流噪声
        est->rs_est.current_noise_std_dev /= 6.0f;

        // 4. 计算编码器偏置和健康度（标准差）
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // 以第一步(U->V)为基准，理论电气角度为30度 (0.0833 PU)
            // TODO 理论上应该使用平均值来实现
            parameter_gt base_angle_pu = SIX_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->rs_est.enc_offset_results[0];

            // 归一化到 [0, 1.0)
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;
            if (est->encoder_offset >= 1.0f)
                est->encoder_offset -= 1.0f;

            // 检查步进一致性 (理论步进60度)
            parameter_gt step_diff_sum = 0;
            parameter_gt step_diff_sq_sum = 0;
            parameter_gt sixty_deg_pu = 60.0f / 360.0f;
            for (int i = 0; i < 5; ++i)
            {
                parameter_gt diff = est->rs_est.enc_offset_results[i + 1] - est->rs_est.enc_offset_results[i];
                if (diff < -0.5f)
                    diff += 1.0f; // 处理负向回绕
                if (diff > 0.5f)
                    diff -= 1.0f; // 处理正向回绕
                parameter_gt error = diff - sixty_deg_pu;
                step_diff_sum += error;
                step_diff_sq_sum += error * error;
            }
            parameter_gt mean_error = step_diff_sum / 5.0f;
            est->rs_est.position_consistency_std_dev = sqrtf(fabsf(step_diff_sq_sum / 5.0f - mean_error * mean_error));
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        ctl_disable_current_controller(&est->current_ctrl);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);

        // 切换到下一个主状态
        if (est->ldq_est.flag_enable)
            est->main_state = OFFLINE_MAIN_STATE_L;
        else if (est->psif_est.flag_enable)
            est->main_state = OFFLINE_MAIN_STATE_FLUX;
        else if (est->inertia_est.flag_enable)
            est->main_state = OFFLINE_MAIN_STATE_J;
        else
            est->main_state = OFFLINE_MAIN_STATE_DONE;
        // reset sub state
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

/**
 * @brief 电感辨识的主分发函数.
 * @note FIX: 明确根据flag_enable_ldq选择方法. 由于直流偏置法未实现, 
 * 若选择该方法则直接报错, 默认执行已实现的旋转HFI法.
 */
void est_loop_handle_l(ctl_offline_est_t* est)
{
    // NOTE: 本模块提供了两种电感辨识方法.
    // 方法1 (flag_enable_ldq=1): 直流偏置HFI法 (est_loop_handle_l_dcbias_hfi)
    //      此方法当前实现不完整, 核心的IIR滤波器和RMS计算逻辑被注释掉了.
    //      在修复前不应使用.
    // 方法2 (flag_enable_ldq=2): 旋转矢量HFI法 (est_loop_handle_l_rotating_hfi)
    //      此方法已正确实现.
    if (est->ldq_est.flag_enable == 1)
    {
        // est_loop_handle_l_dcbias_hfi(est); // 不调用未完成的函数
        est->main_state = OFFLINE_MAIN_STATE_ERROR; // 直接标记错误
    }
    else
    {
        est_loop_handle_l_rotating_hfi(est);
    }
}

/**
 * @brief [方法一] 使用旋转HFI法辨识电感.
 */
static void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est)
{
    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. 初始化高频注入正弦信号发生器
        parameter_gt step_angle = est->ldq_est.hfi_freq_hz / est->fs;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 2. 初始化慢速旋转发生器
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, est->ldq_est.hfi_rot_freq_hz,
                                          est->ldq_est.hfi_rot_freq_hz * 10.0f, // 0.1s 加速时间
                                          est->fs);

        // 3. 配置电流控制器为开环电压模式 (通过在ISR中设置前馈电压实现)
        ctl_disable_current_controller(&est->current_ctrl);

        // 4. 初始化滤波器和测量变量
        ctl_init_lp_filter(&est->measure_flt[3], est->fs,
                           est->ldq_est.hfi_rot_freq_hz * 5.0f); // 滤波HFI电流响应

        est->ldq_est.hfi_i_max = -1.0f;
        est->ldq_est.hfi_i_min = 1e9; // 一个很大的初始值
        est->ldq_est.hfi_theta_d = 0.0f;
        est->ldq_est.hfi_theta_q = 0.0f;

        // 5. 记录开始时间并切换状态
        est->task_start_time = gmp_base_get_system_tick();
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {

        // 在减速阶段完成后，切换到下一个状态
        if (gmp_base_get_diff_system_tick(est->task_start_time) >
            (est->ldq_est.stabilize_time + est->ldq_est.measure_time + est->ldq_est.ending_time))
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        // 在完成测量之后，需要将目标转速设置为0
        else if (gmp_base_get_diff_system_tick(est->task_start_time) >
                 (est->ldq_est.stabilize_time + est->ldq_est.measure_time))
        {
            // 将目标转速设置为0即可。
            ctl_set_slope_f_freq(&est->speed_profile_gen, 0, est->fs);
        }

        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. 计算阻抗
        parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(est->pu_consultant, ctrl2float(est->ldq_est.hfi_v_pu));

        // Z_d = V_hfi / I_hfi_min, Z_q = V_hfi / I_hfi_max
        parameter_gt z_d = (est->ldq_est.hfi_i_min > 1e-3) ? (v_hfi_peak / est->ldq_est.hfi_i_min) : 0.0f;
        parameter_gt z_q = (est->ldq_est.hfi_i_max > 1e-3) ? (v_hfi_peak / est->ldq_est.hfi_i_max) : 0.0f;

        // 2. 计算电感 (忽略电阻)
        //tex:
        //$$ Z \approx \omega L $$
        parameter_gt omega_hfi = CTL_PARAM_CONST_2PI * est->ldq_est.hfi_freq_hz;
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
            est->encoder_offset_ldq = est->ldq_est.hfi_theta_d - actual_pos;
            if (est->encoder_offset_ldq < 0.0f)
                est->encoder_offset_ldq += 1.0f;
            if (est->encoder_offset_ldq >= 1.0f)
                est->encoder_offset_ldq -= 1.0f;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. 关闭电机励磁
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl); // 恢复闭环模式

        // 2. 切换到下一个主状态
        if (est->psif_est.flag_enable)
        {
            est->main_state = OFFLINE_MAIN_STATE_FLUX;
        }
        else if (est->inertia_est.flag_enable)
        {
            est->main_state = OFFLINE_MAIN_STATE_J;
        }
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_DONE;
        }
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

#define LDQ_STABILIZE_TIME_MS   (1000) // 示例: HFI稳定时间
#define LDQ_MEASUREMENT_TIME_MS (2000) // 示例: 测量时间 (例如旋转一圈)
#define LDQ_DECELERATE_TIME_MS  (500)  // 示例: 减速时间

/**
 * @brief [方法二] 使用直流偏置HFI法辨识电感.
 * @note  FIXME: 此函数当前实现不完整, 无法正常工作.
 * 1. 依赖于未实现的IIR高通滤波器 (ctl_init_filter_iir1_hpf, ctl_step_filter_iir1).
 * 2. 核心的RMS值计算和阻抗/电感计算逻辑被注释掉了.
 * 在完成上述修复前, 请勿使用此方法.
 */
static void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est)
{
    // 由于此方法未完成，保留原代码结构但添加警告
    // 实际项目中应实现IIR滤波器和相关计算
    // ...
    // 为避免执行错误逻辑，直接切换到错误状态
    est->main_state = OFFLINE_MAIN_STATE_ERROR;
}

// 定义磁链测试的速度点 (以额定频率的百分比表示)
#define FLUX_TEST_POINTS (4)
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

        // 1. FIX: 配置电流控制器为Id=0, Iq=一个小的正值以产生转矩驱动电机旋转.
        //    原代码Iq=0, 电机没有转矩, 将无法跟随速度曲线旋转.
        parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->flux_test_iq_pu);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);
        ctl_enable_current_controller(&est->current_ctrl);

        // 2. 配置滤波器
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 20.0f); // Uq
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 20.0f); // Omega_e

        // 3. 初始化变量
        est->step_index = 0; // 用于遍历速度点
        // 复用变量用于最小二乘法: y = Uq, x = Omega_e
        est->V_sum = 0; // 用作 Σ(Uq*Omega_e)
        est->I_sum = 0; // 用作 Σ(Omega_e^2)
        est->sample_count = 0;

        // 4. 进入执行状态, 并立即配置第一个速度点
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();

        // 配置第一个速度点的速度发生器
        parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
        parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz, target_freq_hz / FLUX_RAMP_UP_TIME_S,
                                          est->isr_freq_hz);
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. 持续步进速度发生器, 为ISR提供开环旋转角度 ---
        ctl_step_slope_f(&est->speed_profile_gen);

        // --- 2. 等待电机达到目标速度并稳定 ---
        uint32_t wait_time_ms = (uint32_t)(FLUX_RAMP_UP_TIME_S * 1000) + FLUX_STABILIZE_TIME_MS;
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms))
        {
            return; // 在斜坡和稳定期间, 不进行测量
        }

        // --- 3. 在稳定状态下测量指定时间 ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms + FLUX_MEASURE_TIME_MS))
        {
            // 获取q轴电压和电角速度
            parameter_gt Uq = est->current_ctrl.vdq0.dat[1] * ctl_consult_base_voltage(pu);
            // 速度应从速度发生器的目标频率获取, 因为这是开环测试, 真实速度未知或不准
            parameter_gt Omega_e = est->speed_profile_gen.current_freq * est->fs * CTL_PARAM_CONST_2PI;

            // 滤波
            Uq = ctl_step_lowpass_filter(&est->measure_flt[0], Uq);
            Omega_e = ctl_step_lowpass_filter(&est->measure_flt[1], Omega_e);

            // 累加用于线性回归计算 (最小二乘法)
            // 磁链方程
            //tex:
            // $$U_q = R_s I_q + \omega_e \psi_f $$

            // 当Iq恒定时,
            //tex:
            // $$ U_q' = U_q - R_s I_q  = \omega_e \psi_f$$
            // $$ \psi_f = \frac{\Sigma{U_q'\omega_e}} {\Sigma{\omega_e^2}}$$
            parameter_gt iq_measured = est->current_ctrl.idq0.dat[1];
            parameter_gt Uq_prime = Uq - est->pmsm_params.Rs * iq_measured;
            est->V_sum += Uq_prime * Omega_e;
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

            // 1. 让电机停下来
            ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
            // (电机将在无力矩下自然减速停止)
        }
        else
        {
            // 重置计时器和速度发生器, 进入下一个速度点的测试
            est->task_start_time = gmp_base_get_system_tick();
            parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
            parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
            ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz,
                                              target_freq_hz / FLUX_RAMP_UP_TIME_S, est->isr_freq_hz);
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 使用最小二乘法计算斜率，即磁链
        //tex:
        // $$ slope = \frac{\Sigma{xy}}{\Sigma{x^2}}, x = \omega_e , y = Uq' $$
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

        // 2. 切换到下一个主状态
        if (est->flag_enable_inertia)
        {
            est->main_state = OFFLINE_MAIN_STATE_J;
        }
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_DONE;
        }
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
        // 1. FIX: 检查依赖项: 磁链和编码器
        if (est->pmsm_params.flux < 1e-6)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // 无法计算转矩
            return;
        }
        if (est->encoder_type == ENCODER_TYPE_NONE)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // 无编码器无法测量速度
            return;
        }

        // 2. 配置电流控制器为 Id=0, Iq=0 (初始状态)
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 3. 配置滤波器
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 50.0f); // Speed
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 50.0f); // Iq

        // 4. 初始化变量 (用于线性回归: y=speed, x=time)
        est->sum_x = 0;  // Σt
        est->sum_y = 0;  // Σω
        est->sum_xy = 0; // Σ(t*ω)
        est->sum_x2 = 0; // Σ(t^2)
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
            // 设置目标Iq以产生转矩
            parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->j_test_iq_pu);
            ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);

            // 获取并滤波测量值
            parameter_gt speed_rps = ctl_get_mtr_velocity(est->mtr_interface); // 假设单位为 rps
            parameter_gt speed_rads = ctl_step_lowpass_filter(&est->measure_flt[0], speed_rps * CTL_PARAM_CONST_2PI);
            parameter_gt iq_measured = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[1]);

            // 获取当前时间 (秒)
            parameter_gt time_s = (gmp_base_get_system_tick() - (est->task_start_time + J_STABILIZE_TIME_MS)) / 1000.0f;
            if (time_s < 0)
                time_s = 0;

            // 累加用于线性回归和平均转矩计算
            est->sum_x += time_s;
            est->sum_y += speed_rads;
            est->sum_xy += time_s * speed_rads;
            est->sum_x2 += time_s * time_s;
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

            // 2. 计算角加速度 (线性回归斜率: alpha)
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
                    est->pmsm_params.inertia = fabsf(est->avg_torque / alpha);
                }
                else
                {
                    est->main_state = OFFLINE_MAIN_STATE_ERROR; // 加速度过小, 无法计算
                }
            }
            else
            {
                est->main_state = OFFLINE_MAIN_STATE_ERROR; // 数据错误, 无法计算回归
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
