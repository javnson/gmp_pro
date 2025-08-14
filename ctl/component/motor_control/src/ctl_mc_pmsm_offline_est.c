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

// ����������ע��ų��ĵ�Ƕ� (U->V, W->V, W->U, V->U, V->W, U->W)
// ��Ӧ�� 30, 90, 150, 210, 270, 330 �� (��λ: ��)
static const float SIX_STEP_ANGLES_DEG[6] = {30.0f, 90.0f, 150.0f, 210.0f, 270.0f, 330.0f};

// ������׶ε���ʱ����
#define RS_STABILIZE_TIME_MS     (1000) // Rs��ʶ��ÿ���ȶ�ʱ��1000ms
#define RS_MEASURE_TIME_MS       (500)  // Rs��ʶ���ȶ������ʱ��500ms
#define L_DCBIAS_ALIGN_TIME_MS   (1500) // L��ʶ(ֱ��ƫ�÷�)��ת�Ӷ���ʱ��
#define L_DCBIAS_MEASURE_TIME_MS (1000) // L��ʶ(ֱ��ƫ�÷�)������ʱ��
#define FLUX_RAMP_UP_TIME_S      (2.0f) // ������ʶ��б������ʱ�� (��)
#define FLUX_STABILIZE_TIME_MS   (1000) // ������ʶ��ÿ���ٶȵ���ȶ�ʱ��
#define FLUX_MEASURE_TIME_MS     (1000) // ������ʶ��ÿ���ٶȵ�Ĳ���ʱ��
#define J_STABILIZE_TIME_MS      (500)  // ������ʶ����ֹ�ȶ�ʱ��
#define J_TORQUE_STEP_TIME_MS    (1000) // ������ʶ��ת�ؽ�Ծ�Ͳ���ʱ��

// ��һ����Ҫ������Ҫ�ڼ����߼���������Ŀǰ�ļ���ʱ����ʵ��ֵ���Ƶģ�����ʵ������ǻ��ڱ���ֵ���Ƶġ�
// ������Ҫ������������������Ĵ��룬�õ���ĵ����������������ȷ����/���㡣

// ��ÿһ������Ŀ�ʼ�׶���Ҫ��ʼ�������������

// �ȸĵ���ĵ���������

// ���������J�۲�ʱ����ʹ������Ƕȣ�������ʵ�ʽǶȣ���Ϊʵ�ʽǶȵıջ���һ���ܹ��ȶ����У���һ����Ҫ��һ����־λ��

// ������Щ����Ҫ��Ϊ����д�����У�ֻ������Щ�����ڳ�ʼ��ʱ������Ĭ��ֵ��

// offline estimate module for PMSM, idling accelerate time, in [ms] unit.
#define CTL_MC_OFFLINE_EST_IDLING_ACCELERATE_TIME (100)

/**
 * @brief Rs�ͱ�����ƫ�ñ�ʶ����ѭ��������.
 */
static void est_loop_handle_rs(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. ���õ�ͨ�˲�����б�·�����
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

        // 3. ���ݱ��������;�����һ�� (QEP index����������δʵ��)
        if (est->encoder_type == ENCODER_TYPE_QEP)
        {
            // est->sub_state = OFFLINE_SUB_STATE_QEP_INDEX_SEARCH;
            // TODO: �����ٶȷ������Ե�����תѰ��Z����
            est->sub_state = OFFLINE_SUB_STATE_EXEC; // ��ʱ����
        }
        else
        {
            est->sub_state = OFFLINE_SUB_STATE_EXEC;
        }

        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_QEP_INDEX_SEARCH: {
        // TODO: ʵ��Ѱ��QEP index���߼�
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
            // �����ۼ���, Ϊ��һ�β�����׼��
            est->sample_count = 0;
            est->V_sum = 0;
            est->I_sum = 0;
            est->Pos_sum = 0;
            est->V_sq_sum = 0;
            est->I_sq_sum = 0;
            est->task_start_time = gmp_base_get_system_tick(); // ���ü�ʱ��
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. ���������ߵ����ƽ��ֵ
        est->rs_est.Rs_line_to_line.dat[0] =
            (est->rs_est.step_results[0] + est->rs_est.step_results[3]) / 2.0f; // U-V vs V-U
        est->rs_est.Rs_line_to_line.dat[1] =
            (est->rs_est.step_results[1] + est->rs_est.step_results[4]) / 2.0f; // W-V vs V-W
        est->rs_est.Rs_line_to_line.dat[2] =
            (est->rs_est.step_results[2] + est->rs_est.step_results[5]) / 2.0f; // W-U vs U-W

        // 2. �������յ�ƽ������� R_phase = R_line_avg / 2
        est->pmsm_params.Rs = (est->rs_est.Rs_line_to_line.dat[0] + est->rs_est.Rs_line_to_line.dat[1] +
                               est->rs_est.Rs_line_to_line.dat[2]) /
                              6.0f;

        // 3. ����ƽ����������
        est->rs_est.current_noise_std_dev /= 6.0f;

        // 4. ���������ƫ�úͽ����ȣ���׼�
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // �Ե�һ��(U->V)Ϊ��׼�����۵����Ƕ�Ϊ30�� (0.0833 PU)
            // TODO ������Ӧ��ʹ��ƽ��ֵ��ʵ��
            parameter_gt base_angle_pu = SIX_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->rs_est.enc_offset_results[0];

            // ��һ���� [0, 1.0)
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;
            if (est->encoder_offset >= 1.0f)
                est->encoder_offset -= 1.0f;

            // ��鲽��һ���� (���۲���60��)
            parameter_gt step_diff_sum = 0;
            parameter_gt step_diff_sq_sum = 0;
            parameter_gt sixty_deg_pu = 60.0f / 360.0f;
            for (int i = 0; i < 5; ++i)
            {
                parameter_gt diff = est->rs_est.enc_offset_results[i + 1] - est->rs_est.enc_offset_results[i];
                if (diff < -0.5f)
                    diff += 1.0f; // ���������
                if (diff > 0.5f)
                    diff -= 1.0f; // �����������
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

        // �л�����һ����״̬
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
 * @brief ��б�ʶ�����ַ�����.
 * @note FIX: ��ȷ����flag_enable_ldqѡ�񷽷�. ����ֱ��ƫ�÷�δʵ��, 
 * ��ѡ��÷�����ֱ�ӱ���, Ĭ��ִ����ʵ�ֵ���תHFI��.
 */
void est_loop_handle_l(ctl_offline_est_t* est)
{
    // NOTE: ��ģ���ṩ�����ֵ�б�ʶ����.
    // ����1 (flag_enable_ldq=1): ֱ��ƫ��HFI�� (est_loop_handle_l_dcbias_hfi)
    //      �˷�����ǰʵ�ֲ�����, ���ĵ�IIR�˲�����RMS�����߼���ע�͵���.
    //      ���޸�ǰ��Ӧʹ��.
    // ����2 (flag_enable_ldq=2): ��תʸ��HFI�� (est_loop_handle_l_rotating_hfi)
    //      �˷�������ȷʵ��.
    if (est->ldq_est.flag_enable == 1)
    {
        // est_loop_handle_l_dcbias_hfi(est); // ������δ��ɵĺ���
        est->main_state = OFFLINE_MAIN_STATE_ERROR; // ֱ�ӱ�Ǵ���
    }
    else
    {
        est_loop_handle_l_rotating_hfi(est);
    }
}

/**
 * @brief [����һ] ʹ����תHFI����ʶ���.
 */
static void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est)
{
    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. ��ʼ����Ƶע�������źŷ�����
        parameter_gt step_angle = est->ldq_est.hfi_freq_hz / est->fs;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 2. ��ʼ��������ת������
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, est->ldq_est.hfi_rot_freq_hz,
                                          est->ldq_est.hfi_rot_freq_hz * 10.0f, // 0.1s ����ʱ��
                                          est->fs);

        // 3. ���õ���������Ϊ������ѹģʽ (ͨ����ISR������ǰ����ѹʵ��)
        ctl_disable_current_controller(&est->current_ctrl);

        // 4. ��ʼ���˲����Ͳ�������
        ctl_init_lp_filter(&est->measure_flt[3], est->fs,
                           est->ldq_est.hfi_rot_freq_hz * 5.0f); // �˲�HFI������Ӧ

        est->ldq_est.hfi_i_max = -1.0f;
        est->ldq_est.hfi_i_min = 1e9; // һ���ܴ�ĳ�ʼֵ
        est->ldq_est.hfi_theta_d = 0.0f;
        est->ldq_est.hfi_theta_q = 0.0f;

        // 5. ��¼��ʼʱ�䲢�л�״̬
        est->task_start_time = gmp_base_get_system_tick();
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {

        // �ڼ��ٽ׶���ɺ��л�����һ��״̬
        if (gmp_base_get_diff_system_tick(est->task_start_time) >
            (est->ldq_est.stabilize_time + est->ldq_est.measure_time + est->ldq_est.ending_time))
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        // ����ɲ���֮����Ҫ��Ŀ��ת������Ϊ0
        else if (gmp_base_get_diff_system_tick(est->task_start_time) >
                 (est->ldq_est.stabilize_time + est->ldq_est.measure_time))
        {
            // ��Ŀ��ת������Ϊ0���ɡ�
            ctl_set_slope_f_freq(&est->speed_profile_gen, 0, est->fs);
        }

        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. �����迹
        parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(est->pu_consultant, ctrl2float(est->ldq_est.hfi_v_pu));

        // Z_d = V_hfi / I_hfi_min, Z_q = V_hfi / I_hfi_max
        parameter_gt z_d = (est->ldq_est.hfi_i_min > 1e-3) ? (v_hfi_peak / est->ldq_est.hfi_i_min) : 0.0f;
        parameter_gt z_q = (est->ldq_est.hfi_i_max > 1e-3) ? (v_hfi_peak / est->ldq_est.hfi_i_max) : 0.0f;

        // 2. ������ (���Ե���)
        //tex:
        //$$ Z \approx \omega L $$
        parameter_gt omega_hfi = CTL_PARAM_CONST_2PI * est->ldq_est.hfi_freq_hz;
        if (omega_hfi > 1e-3)
        {
            est->pmsm_params.Ld = z_d / omega_hfi;
            est->pmsm_params.Lq = z_q / omega_hfi;
        }

        // 3. ����б����������Խ�һ��У׼��������ƫ
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // d�᷽���� hfi_theta_d����ʱת�ӵ���ʵd��Ӧ�ö����������
            // ��ȡ��ʱ����������ʵλ��
            parameter_gt actual_pos = ctl_get_mtr_elec_theta(est->mtr_interface);
            // �µ���ƫ = ����d��� - ʵ��d���
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
        // 1. �رյ������
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl); // �ָ��ջ�ģʽ

        // 2. �л�����һ����״̬
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

#define LDQ_STABILIZE_TIME_MS   (1000) // ʾ��: HFI�ȶ�ʱ��
#define LDQ_MEASUREMENT_TIME_MS (2000) // ʾ��: ����ʱ�� (������תһȦ)
#define LDQ_DECELERATE_TIME_MS  (500)  // ʾ��: ����ʱ��

/**
 * @brief [������] ʹ��ֱ��ƫ��HFI����ʶ���.
 * @note  FIXME: �˺�����ǰʵ�ֲ�����, �޷���������.
 * 1. ������δʵ�ֵ�IIR��ͨ�˲��� (ctl_init_filter_iir1_hpf, ctl_step_filter_iir1).
 * 2. ���ĵ�RMSֵ������迹/��м����߼���ע�͵���.
 * ����������޸�ǰ, ����ʹ�ô˷���.
 */
static void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est)
{
    // ���ڴ˷���δ��ɣ�����ԭ����ṹ����Ӿ���
    // ʵ����Ŀ��Ӧʵ��IIR�˲�������ؼ���
    // ...
    // Ϊ����ִ�д����߼���ֱ���л�������״̬
    est->main_state = OFFLINE_MAIN_STATE_ERROR;
}

// ����������Ե��ٶȵ� (�ԶƵ�ʵİٷֱȱ�ʾ)
#define FLUX_TEST_POINTS (4)
static const float FLUX_TEST_SPEED_PU[FLUX_TEST_POINTS] = {0.25f, 0.5f, 0.75f, 1.0f};

/**
 * @brief ������ʶ����ѭ��������.
 */
void est_loop_handle_flux(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {

        // 1. FIX: ���õ���������ΪId=0, Iq=һ��С����ֵ�Բ���ת�����������ת.
        //    ԭ����Iq=0, ���û��ת��, ���޷������ٶ�������ת.
        parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->flux_test_iq_pu);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);
        ctl_enable_current_controller(&est->current_ctrl);

        // 2. �����˲���
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 20.0f); // Uq
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 20.0f); // Omega_e

        // 3. ��ʼ������
        est->step_index = 0; // ���ڱ����ٶȵ�
        // ���ñ���������С���˷�: y = Uq, x = Omega_e
        est->V_sum = 0; // ���� ��(Uq*Omega_e)
        est->I_sum = 0; // ���� ��(Omega_e^2)
        est->sample_count = 0;

        // 4. ����ִ��״̬, ���������õ�һ���ٶȵ�
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();

        // ���õ�һ���ٶȵ���ٶȷ�����
        parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
        parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz, target_freq_hz / FLUX_RAMP_UP_TIME_S,
                                          est->isr_freq_hz);
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. ���������ٶȷ�����, ΪISR�ṩ������ת�Ƕ� ---
        ctl_step_slope_f(&est->speed_profile_gen);

        // --- 2. �ȴ�����ﵽĿ���ٶȲ��ȶ� ---
        uint32_t wait_time_ms = (uint32_t)(FLUX_RAMP_UP_TIME_S * 1000) + FLUX_STABILIZE_TIME_MS;
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms))
        {
            return; // ��б�º��ȶ��ڼ�, �����в���
        }

        // --- 3. ���ȶ�״̬�²���ָ��ʱ�� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms + FLUX_MEASURE_TIME_MS))
        {
            // ��ȡq���ѹ�͵���ٶ�
            parameter_gt Uq = est->current_ctrl.vdq0.dat[1] * ctl_consult_base_voltage(pu);
            // �ٶ�Ӧ���ٶȷ�������Ŀ��Ƶ�ʻ�ȡ, ��Ϊ���ǿ�������, ��ʵ�ٶ�δ֪��׼
            parameter_gt Omega_e = est->speed_profile_gen.current_freq * est->fs * CTL_PARAM_CONST_2PI;

            // �˲�
            Uq = ctl_step_lowpass_filter(&est->measure_flt[0], Uq);
            Omega_e = ctl_step_lowpass_filter(&est->measure_flt[1], Omega_e);

            // �ۼ��������Իع���� (��С���˷�)
            // ��������
            //tex:
            // $$U_q = R_s I_q + \omega_e \psi_f $$

            // ��Iq�㶨ʱ,
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

        // --- 4. ׼����һ���ٶȵ� ---
        est->step_index++;
        if (est->step_index >= FLUX_TEST_POINTS)
        {
            // �����ٶȵ�������
            est->sub_state = OFFLINE_SUB_STATE_CALC;

            // 1. �õ��ͣ����
            ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
            // (�����������������Ȼ����ֹͣ)
        }
        else
        {
            // ���ü�ʱ�����ٶȷ�����, ������һ���ٶȵ�Ĳ���
            est->task_start_time = gmp_base_get_system_tick();
            parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
            parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
            ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz,
                                              target_freq_hz / FLUX_RAMP_UP_TIME_S, est->isr_freq_hz);
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // ʹ����С���˷�����б�ʣ�������
        //tex:
        // $$ slope = \frac{\Sigma{xy}}{\Sigma{x^2}}, x = \omega_e , y = Uq' $$
        if (est->I_sum > 1e-3)
        { // ȷ����ĸ��Ϊ��
            est->pmsm_params.flux = est->V_sum / est->I_sum;
        }
        else
        {
            est->pmsm_params.flux = 0.0f; // �������
            est->main_state = OFFLINE_MAIN_STATE_ERROR;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {

        // 2. �л�����һ����״̬
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
 * @brief ������ʶ����ѭ��������.
 */
void est_loop_handle_j(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. FIX: ���������: �����ͱ�����
        if (est->pmsm_params.flux < 1e-6)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // �޷�����ת��
            return;
        }
        if (est->encoder_type == ENCODER_TYPE_NONE)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // �ޱ������޷������ٶ�
            return;
        }

        // 2. ���õ���������Ϊ Id=0, Iq=0 (��ʼ״̬)
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 3. �����˲���
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 50.0f); // Speed
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 50.0f); // Iq

        // 4. ��ʼ������ (�������Իع�: y=speed, x=time)
        est->sum_x = 0;  // ��t
        est->sum_y = 0;  // ����
        est->sum_xy = 0; // ��(t*��)
        est->sum_x2 = 0; // ��(t^2)
        est->avg_torque = 0;
        est->sample_count = 0;

        // 5. ����ִ��״̬
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. �ȴ�һС��ʱ����ȷ�������ֹ ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, J_STABILIZE_TIME_MS))
        {
            return;
        }

        // --- 2. ʩ��ת�ؽ�Ծ���������� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, J_STABILIZE_TIME_MS + J_TORQUE_STEP_TIME_MS))
        {
            // ����Ŀ��Iq�Բ���ת��
            parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->j_test_iq_pu);
            ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);

            // ��ȡ���˲�����ֵ
            parameter_gt speed_rps = ctl_get_mtr_velocity(est->mtr_interface); // ���赥λΪ rps
            parameter_gt speed_rads = ctl_step_lowpass_filter(&est->measure_flt[0], speed_rps * CTL_PARAM_CONST_2PI);
            parameter_gt iq_measured = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[1]);

            // ��ȡ��ǰʱ�� (��)
            parameter_gt time_s = (gmp_base_get_system_tick() - (est->task_start_time + J_STABILIZE_TIME_MS)) / 1000.0f;
            if (time_s < 0)
                time_s = 0;

            // �ۼ��������Իع��ƽ��ת�ؼ���
            est->sum_x += time_s;
            est->sum_y += speed_rads;
            est->sum_xy += time_s * speed_rads;
            est->sum_x2 += time_s * time_s;
            est->avg_torque += 1.5f * est->pmsm_params.pole_pair * est->pmsm_params.flux * iq_measured;
            est->sample_count++;

            return;
        }

        // --- 3. ����ʱ��������������״̬ ---
        est->sub_state = OFFLINE_SUB_STATE_CALC;
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        if (est->sample_count > 10)
        { // ȷ�����㹻�����ݵ�
            // 1. ����ƽ��ת��
            est->avg_torque /= est->sample_count;

            // 2. ����Ǽ��ٶ� (���Իع�б��: alpha)
            // alpha = (N*��(t��) - ��t*����) / (N*��(t^2) - (��t)^2)
            parameter_gt N = est->sample_count;
            parameter_gt numerator = N * est->sum_xy - est->sum_x * est->sum_y;
            parameter_gt denominator = N * est->sum_x2 - est->sum_x * est->sum_x;

            if (fabsf(denominator) > 1e-9)
            {
                parameter_gt alpha = numerator / denominator;
                // 3. ������� J = Te / alpha
                if (fabsf(alpha) > 1e-3)
                {
                    est->pmsm_params.inertia = fabsf(est->avg_torque / alpha);
                }
                else
                {
                    est->main_state = OFFLINE_MAIN_STATE_ERROR; // ���ٶȹ�С, �޷�����
                }
            }
            else
            {
                est->main_state = OFFLINE_MAIN_STATE_ERROR; // ���ݴ���, �޷�����ع�
            }
        }
        else
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // �����㲻��
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. ֹͣ���
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);

        // 2. �л����������״̬
        est->main_state = OFFLINE_MAIN_STATE_DONE;
        est->sub_state = OFFLINE_SUB_STATE_INIT; // ������״̬
        break;
    }
    }
}
