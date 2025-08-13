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

// ����������ע��ų��ĵ�Ƕ� (U->V, V->W, W->U)
static const float THREE_STEP_ANGLES_DEG[3] = {30.0f, 150.0f, 270.0f};

// �������
#define RS_STABILIZE_TIME_MS (1000) // ÿ���ȶ�1000ms
#define RS_MEASURE_TIME_MS   (500)  // �ȶ������500ms

/**
 * @brief Rs�ͱ�����ƫ�ñ�ʶ����ѭ��������.
 */
static void est_loop_handle_rs(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. ���õ�ͨ�˲��� (���磬5Hz��ֹƵ��)
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 5.0f); // Vd
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 5.0f); // Id
        ctl_init_lp_filter(&est->measure_flt[2], est->isr_freq_hz, 5.0f); // Position

        // 2. ���õ���������ע��Ŀ�����
        parameter_gt id_ref = ctl_consult_Ipeak_to_phy(pu, est->rs_test_current_pu);
        ctl_set_current_ref(&est->current_ctrl, id_ref, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        est->step_index = 0;

        // 3. ���ݱ��������;�����һ��
        if (est->encoder_type == ENCODER_TYPE_QEP)
        {
            est->sub_state = OFFLINE_SUB_STATE_QEP_INDEX_SEARCH;
            // TODO: �����ٶȷ������Ե�����ת
        }
        else
        {
            est->sub_state = OFFLINE_SUB_STATE_EXEC;
        }

        est->task_start_time = gmp_core_get_systemtick();
        break;
    }

    case OFFLINE_SUB_STATE_QEP_INDEX_SEARCH: {
        // TODO: ʵ��Ѱ��QEP index���߼�
        // 1. ʹ�� vf_generator ������ת��� est->qep_search_elec_cycles Ȧ
        // 2. Ӳ��Ӧ�ڴ˹����в���index�ź�
        // 3. ��ת��ɺ��л��� EXEC ״̬
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. ע���ض�����ĵ��� ---
        float target_angle_pu = THREE_STEP_ANGLES_DEG[est->step_index] / 360.0f;
        // ͨ������Park�任�ĽǶ�����ת����ų�
        // TODO: ����϶���Ӧ������FOC�Ĳο��Ƕ�theta
        // ���� ctl_step_current_controller �� theta �������ⲿ����Ϊ target_angle_pu

        // --- 2. �ȴ������ȶ� ---
        if (!est_is_delay_elapsed_ms(est->task_start_time, RS_STABILIZE_TIME_MS))
        {
            return; // �ȴ�
        }

        // --- 3. �ȶ�����ָ��ʱ���ڽ��в������ۼ� ---
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

        // --- 4. ����ʱ����������㱾�ν����������һ�� ---
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

        // --- 5. ׼��������һ�� ---
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
        // 1. �������յ���ֵ
        est->Rs_line_to_line.dat[0] = est->rs_line_results[0]; // R_uv
        est->Rs_line_to_line.dat[1] = est->rs_line_results[1]; // R_vw
        est->Rs_line_to_line.dat[2] = est->rs_line_results[2]; // R_wu
        // ƽ������� = ƽ���ߵ��� / 2
        est->pmsm_params.Rs = (est->rs_line_results[0] + est->rs_line_results[1] + est->rs_line_results[2]) / 6.0f;

        // 2. ����ƽ����������
        est->current_noise_std_dev /= 3.0f;

        // 3. ���������ƫ�úͽ�����
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // �Ե�һ��(U->V)Ϊ��׼�����۵����Ƕ�Ϊ30��
            parameter_gt base_angle_pu = THREE_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->enc_offset_results[0];
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;

            // ��鲽��һ����
            parameter_gt diff1 = fabsf(est->enc_offset_results[1] - est->enc_offset_results[0]);
            parameter_gt diff2 = fabsf(est->enc_offset_results[2] - est->enc_offset_results[1]);
            // ���۵����ǶȲ�120�� (0.333 pu)
            parameter_gt err1 = fabsf(diff1 - 120.0f / 360.0f);
            parameter_gt err2 = fabsf(diff2 - 120.0f / 360.0f);
            // �˴����Լ����׼�����һ�����򵥵����ƽ��ֵ
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
 * @brief ��б�ʶ����ѭ��������.
 */
static void est_loop_handle_l(ctl_offline_est_t* est)
{
    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. ��ʼ����Ƶע���źŷ�����
        parameter_gt step_angle = est->l_hfi_freq_hz / est->isr_freq_hz;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 2. ��ʼ��������ת������
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, est->l_hfi_rot_freq_hz,
                                          est->l_hfi_rot_freq_hz * 2.0f, // 0.5s ����ʱ��
                                          est->isr_freq_hz);

        // 3. ���õ���������Ϊ������ѹģʽ
        ctl_disable_current_controller(&est->current_ctrl);

        // 4. ��ʼ���˲����Ͳ�������
        ctl_init_lp_filter(&est->measure_flt[3], est->isr_freq_hz, est->l_hfi_rot_freq_hz * 5.0f); // �˲�HFI������Ӧ
        est->hfi_i_max = -1.0f;
        est->hfi_i_min = 1e9; // һ���ܴ�ĳ�ʼֵ
        est->hfi_theta_d = 0.0f;
        est->hfi_theta_q = 0.0f;

        // 5. ��¼��ʼʱ�䲢�л�״̬
        est->task_start_time = gmp_core_get_systemtick();
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // ������תע��ʸ��
        ctl_step_slope_f(&est->speed_profile_gen);

        // ��ȡ�������������alpha-beta���� (���Ƕ�ʵ�ʵ����ķ���)
        parameter_gt i_alpha = est->current_ctrl.iab0.dat[0];
        parameter_gt i_beta = est->current_ctrl.iab0.dat[1];

        // �����Ƶ������Ӧ�ķ�ֵ
        parameter_gt i_hfi_mag = ctl_sqrt(i_alpha * i_alpha + i_beta * i_beta);

        // �Է�ֵ���е�ͨ�˲����õ������
        i_hfi_mag = ctl_step_lowpass_filter(&est->measure_flt[3], i_hfi_mag);

        // ��ȡ��ǰ��ע��Ƕ�
        parameter_gt current_theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);

        // Ѱ�ҵ�����Ӧ�ļ���ֵ�ͼ�Сֵ
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

        // ����Ƿ���ת���㹻����ʱ�� (���磬��ת1.5Ȧ��ȷ���ȶ�)
        uint32_t rotation_time_ms = (uint32_t)(1.5f / est->l_hfi_rot_freq_hz * 1000.0f);
        if (est_is_delay_elapsed_ms(est->task_start_time, rotation_time_ms))
        {
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. �����迹
        parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(est->pu_consultant, est->l_hfi_v_pu);
        parameter_gt z_d = (est->hfi_i_max > 1e-3) ? (v_hfi_peak / est->hfi_i_max) : 0.0f;
        parameter_gt z_q = (est->hfi_i_min > 1e-3) ? (v_hfi_peak / est->hfi_i_min) : 0.0f;

        // 2. ������ (���Ե���, Z �� ��L)
        parameter_gt omega_hfi = CTL_PARAM_CONST_2PI * est->l_hfi_freq_hz;
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
            est->encoder_offset = est->hfi_theta_d - actual_pos;
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;
        }

        est->sub_state = OFFLINE_SUB_STATE_DONE;
        break;
    }

    case OFFLINE_SUB_STATE_DONE: {
        // 1. �رյ������
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl); // �ָ��ջ�ģʽ

        // 2. �л�����һ����״̬
        est->main_state = OFFLINE_MAIN_STATE_FLUX;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}

/**
 * @brief ������ʶ����ѭ��������.
 */
static void est_loop_handle_flux(ctl_offline_est_t* est)
{
    // ... �˴���ʵ�ִ�����ʶ����״̬���߼� ...
}

/**
 * @brief ������ʶ����ѭ��������.
 */
static void est_loop_handle_j(ctl_offline_est_t* est)
{
    // ... �˴���ʵ�ֹ�����ʶ����״̬���߼� ...
}
