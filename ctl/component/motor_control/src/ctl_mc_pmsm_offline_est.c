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

// ����������ע��ų��ĵ�Ƕ� (U->V, W->V, W->U, V->U, V->W, U->W)
// ��Ӧ�� 30, 90, 150, 210, 270, 330 ��
static const float SIX_STEP_ANGLES_DEG[6] = {30.0f, 90.0f, 150.0f, 210.0f, 270.0f, 330.0f};

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
        // 1. ���õ�ͨ�˲���
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

        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_QEP_INDEX_SEARCH: {
        // TODO: ʵ��Ѱ��QEP index���߼�
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. ע���ض�����ĵ��� ---
        float target_angle_pu = SIX_STEP_ANGLES_DEG[est->step_index] / 360.0f;
        // TODO: �˴���Ҫһ������������FOC�Ĳο��Ƕ�theta
        // ���� ctl_step_current_controller �� theta �������ⲿ����Ϊ target_angle_pu
        // ����һ���򻯵�ռλ����ʵ��Ӧ���п�����Ҫ est->current_ctrl.theta_override = target_angle_pu;

        // --- 2. �ȴ������ȶ� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, RS_STABILIZE_TIME_MS))
        {
            return;
        }

        // --- 3. �ȶ�����ָ��ʱ���ڽ��в������ۼ� ---
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

        // --- 4. ����ʱ����������㱾�ν����������һ�� ---
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

        // --- 5. ׼��������һ�� ---
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
        // 1. ���������ߵ����ƽ��ֵ
        // R_uv (30 deg) vs R_vu (210 deg)
        est->Rs_line_to_line.dat[0] = (est->rs_step_results[0] + est->rs_step_results[3]) / 2.0f;
        // R_wv (90 deg) vs R_vw (270 deg) - ע��W��Vǰ��
        est->Rs_line_to_line.dat[1] = (est->rs_step_results[1] + est->rs_step_results[4]) / 2.0f;
        // R_wu (150 deg) vs R_uw (330 deg)
        est->Rs_line_to_line.dat[2] = (est->rs_step_results[2] + est->rs_step_results[5]) / 2.0f;

        // 2. �������յ�ƽ������� R_phase = R_line / 2
        est->pmsm_params.Rs =
            (est->Rs_line_to_line.dat[0] + est->Rs_line_to_line.dat[1] + est->Rs_line_to_line.dat[2]) / 6.0f;

        // 3. ����ƽ����������
        est->current_noise_std_dev /= 6.0f;

        // 4. ���������ƫ�úͽ�����
        if (est->encoder_type != ENCODER_TYPE_NONE)
        {
            // �Ե�һ��(U->V)Ϊ��׼�����۵����Ƕ�Ϊ30��
            parameter_gt base_angle_pu = SIX_STEP_ANGLES_DEG[0] / 360.0f;
            est->encoder_offset = base_angle_pu - est->enc_offset_results[0];
            if (est->encoder_offset < 0.0f)
                est->encoder_offset += 1.0f;

            // ��鲽��һ���� (���۲���60��)
            parameter_gt step_diff_sum = 0;
            parameter_gt step_diff_sq_sum = 0;
            parameter_gt sixty_deg_pu = 60.0f / 360.0f;
            for (int i = 0; i < 5; ++i)
            {
                parameter_gt diff = est->enc_offset_results[i + 1] - est->enc_offset_results[i];
                if (diff < 0)
                    diff += 1.0f; // �������
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
 * @brief ��б�ʶ�����ַ�����.
 * @note  Ϊ��֧�ִ˹���, ��Ҫ�� ctl_offline_est_t ������һ��ö�ٳ�Ա
 * `ctl_l_est_method_e l_est_method;` ���ڳ�ʼ��ʱ���û�ָ��.
 */
void est_loop_handle_l(ctl_offline_est_t* est)
{
    // // ����Ĭ��ʹ��ֱ��ƫ�÷���ʵ��Ӧ����Ӧ�����þ���
    // if (est->l_est_method == L_EST_METHOD_DC_BIAS_HFI) {
    //     est_loop_handle_l_dcbias_hfi(est);
    // } else {
    //     est_loop_handle_l_rotating_hfi(est);
    // }

    // ��ǰֱ�ӵ���ֱ��ƫ��HFI��
    est_loop_handle_l_dcbias_hfi(est);
}


/**
 * @brief ��б�ʶ����ѭ��������.
 */
void est_loop_handle_l_rotating_hfi(ctl_offline_est_t* est)
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
        est->task_start_time = gmp_base_get_system_tick();
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
        if (gmp_base_is_delay_elapsed(est->task_start_time, rotation_time_ms))
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
 * @brief [������] ʹ��ֱ��ƫ��HFI����ʶ���.
 * @note  Ϊ��֧�ִ˹���, ��Ҫ�� ctl_offline_est_t ���������³�Ա:
 * `ctl_filter_IIR1_t hpf_v, hpf_i;`
 * `parameter_gt hfi_v_rms[2], hfi_i_rms[2];`
 * ���� ctl_step_offline_est �������߼�����L��ʶ�ڼ佫theta�̶�Ϊ0.
 */
static void est_loop_handle_l_dcbias_hfi(ctl_offline_est_t* est)
{
    ctl_per_unit_consultant_t* pu = est->pu_consultant;

    switch (est->sub_state)
    {
    case OFFLINE_SUB_STATE_INIT: {
        // 1. ��ʼ����ͨ�˲���������ȡ��������
        parameter_gt hpf_fc = est->l_hfi_freq_hz / 10.0f; // e.g., 1/10 of HFI frequency
        // ctl_init_filter_iir1_hpf(&est->hpf_v, est->isr_freq_hz, hpf_fc);
        // ctl_init_filter_iir1_hpf(&est->hpf_i, est->isr_freq_hz, hpf_fc);

        // 2. ��ʼ����Ƶע���źŷ�����
        parameter_gt step_angle = est->l_hfi_freq_hz / est->isr_freq_hz;
        ctl_init_sine_generator(&est->hfi_signal_gen, 0.0f, step_angle);

        // 3. ���õ�����������ʩ��ֱ��ƫ�ã�����ת��d�ᵽalpha��
        parameter_gt id_ref = ctl_consult_Ipeak_to_phy(pu, est->rs_test_current_pu);
        ctl_set_current_ref(&est->current_ctrl, id_ref, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);
        // **��Ҫ**: ��ʱ��ȷ�� ctl_step_offline_est �д���Park�任��theta�̶�Ϊ0

        // 4. ��ʼ������
        est->step_index = 0; // 0 for d-axis, 1 for q-axis
        est->sum_x2 = 0;     // V_ac^2
        est->sum_y2 = 0;     // I_ac^2
        est->sample_count = 0;

        // 5. ����ִ��״̬
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. �ȴ�ת�Ӷ��벢�ȶ� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, L_DCBIAS_ALIGN_TIME_MS))
        {
            return;
        }

        // --- 2. ע���Ƶ�źŲ��������� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, L_DCBIAS_ALIGN_TIME_MS + L_DCBIAS_MEASURE_TIME_MS))
        {
            // ������Ƶ��ѹ�ź�
            ctl_step_sine_generator(&est->hfi_signal_gen);
            parameter_gt v_hfi_peak = ctl_consult_Vpeak_to_phy(pu, est->l_hfi_v_pu);
            parameter_gt v_hfi_inst = ctl_get_sine_generator_sin(&est->hfi_signal_gen) * v_hfi_peak;

            parameter_gt V_ac = 0, I_ac = 0;
            if (est->step_index == 0)
            { // ���� d ��
                ctl_set_voltage_ff(&est->current_ctrl, v_hfi_inst, 0.0f);
                // V_ac = ctl_step_filter_iir1(&est->hpf_v, est->current_ctrl.vdq0.dat[0]);
                // I_ac = ctl_step_filter_iir1(&est->hpf_i, est->current_ctrl.idq0.dat[0]);
            }
            else
            { // ���� q ��
                ctl_set_voltage_ff(&est->current_ctrl, 0.0f, v_hfi_inst);
                // V_ac = ctl_step_filter_iir1(&est->hpf_v, est->current_ctrl.vdq0.dat[1]);
                // I_ac = ctl_step_filter_iir1(&est->hpf_i, est->current_ctrl.idq0.dat[1]);
            }

            // �ۼ�ƽ�����Լ���RMS
            est->sum_x2 += V_ac * V_ac;
            est->sum_y2 += I_ac * I_ac;
            est->sample_count++;
            return;
        }

        // --- 3. ������������������׼����һ�� ---
        if (est->sample_count > 0)
        {
            // est->hfi_v_rms[est->step_index] = ctl_sqrt(est->sum_x2 / est->sample_count);
            // est->hfi_i_rms[est->step_index] = ctl_sqrt(est->sum_y2 / est->sample_count);
        }

        est->step_index++;
        if (est->step_index >= 2)
        { // d �� q �ᶼ�������
            est->sub_state = OFFLINE_SUB_STATE_CALC;
        }
        else
        {                                                     // ׼��������һ����
            est->sub_state = OFFLINE_SUB_STATE_EXEC;          // ������EXEC״̬
            est->task_start_time = gmp_base_get_system_tick(); // ���ü�ʱ��
            est->sample_count = 0;
            est->sum_x2 = 0;
            est->sum_y2 = 0;
            // ctl_clear_filter_iir1(&est->hpf_v);
            // ctl_clear_filter_iir1(&est->hpf_i);
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // 1. �����迹
        // parameter_gt z_d = (est->hfi_i_rms[0] > 1e-6) ? (est->hfi_v_rms[0] / est->hfi_i_rms[0]) : 0.0f;
        // parameter_gt z_q = (est->hfi_i_rms[1] > 1e-6) ? (est->hfi_v_rms[1] / est->hfi_i_rms[1]) : 0.0f;
        parameter_gt z_d = 0, z_q = 0; // ռλ��

        // 2. ������ (����ȷ�ķ���Ӧ���ǵ���: Z^2 = R^2 + (��L)^2)
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
        // 1. ֹͣ���
        ctl_set_voltage_ff(&est->current_ctrl, 0.0f, 0.0f);
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);

        // 2. �л�����һ����״̬
        est->main_state = OFFLINE_MAIN_STATE_FLUX;
        est->sub_state = OFFLINE_SUB_STATE_INIT;
        break;
    }
    }
}



// �������
#define FLUX_RAMP_UP_TIME_S    (2.0f) // б������ʱ�� (��)
#define FLUX_STABILIZE_TIME_MS (1000) // ÿ���ٶȵ���ȶ�ʱ��
#define FLUX_MEASURE_TIME_MS   (1000) // ÿ���ٶȵ�Ĳ���ʱ��
#define FLUX_TEST_POINTS       (4)    // �������Ե��ٶȵ�����

// ��������ٶȵ� (�Զ�ٶȵİٷֱȱ�ʾ)
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
        // 1. ���õ���������Ϊ Id=0 ģʽ
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 2. �����˲���
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 20.0f); // Uq
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 20.0f); // Omega_e

        // 3. ��ʼ������
        est->step_index = 0; // ���ڱ����ٶȵ�
        est->V_sum = 0;      // ���� Uq*Omega_e ���ۼӺ�
        est->I_sum = 0;      // ���� Omega_e^2 ���ۼӺ�
        est->Pos_sum = 0;    // ���� Uq ���ۼӺ�
        est->V_sq_sum = 0;   // ���� Omega_e ���ۼӺ�
        est->sample_count = 0;

        // 4. ����ִ��״̬
        est->sub_state = OFFLINE_SUB_STATE_EXEC;
        est->task_start_time = gmp_base_get_system_tick();
        break;
    }

    case OFFLINE_SUB_STATE_EXEC: {
        // --- 1. ���õ�ǰĿ���ٶȲ�б������ ---
        parameter_gt target_speed_pu = FLUX_TEST_SPEED_PU[est->step_index];
        parameter_gt target_freq_hz = target_speed_pu * ctl_consult_base_frequency(pu);
        parameter_gt ramp_time_s = FLUX_RAMP_UP_TIME_S;

        // �����ٶȷ�����
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, target_freq_hz, target_freq_hz / ramp_time_s,
                                          est->isr_freq_hz);

        // --- 2. �ȴ�����ﵽĿ���ٶȲ��ȶ� ---
        uint32_t wait_time_ms = (uint32_t)(ramp_time_s * 1000) + FLUX_STABILIZE_TIME_MS;
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms))
        {
            // ��ISR�У�FOC����� speed_profile_gen ������Ƕ���ת
            ctl_step_slope_f(&est->speed_profile_gen);
            return;
        }

        // --- 3. ���ȶ�״̬�²���ָ��ʱ�� ---
        if (!gmp_base_is_delay_elapsed(est->task_start_time, wait_time_ms + FLUX_MEASURE_TIME_MS))
        {
            // ��ȡq���ѹ�͵���ٶ�
            parameter_gt Uq = est->current_ctrl.vdq0.dat[1];
            parameter_gt Omega_e = ctl_get_mtr_velocity(est->mtr_interface) * ctl_consult_pole_pairs(pu) *
                                   CTL_PARAM_CONST_2PI; // �����ٶȵ�λΪ rps

            // �˲�
            Uq = ctl_step_lowpass_filter(&est->measure_flt[0], Uq);
            Omega_e = ctl_step_lowpass_filter(&est->measure_flt[1], Omega_e);

            // �ۼ��������Իع���� (��С���˷�)
            // ��f = ��(Uq*��e) / ��(��e^2)
            est->V_sum += Uq * Omega_e;
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
        }
        else
        {
            // ���ü�ʱ����������һ���ٶȵ�Ĳ���
            est->task_start_time = gmp_base_get_system_tick();
        }
        break;
    }

    case OFFLINE_SUB_STATE_CALC: {
        // ʹ����С���˷�����б�ʣ�������
        // Formula: slope = ��(xy) / ��(x^2), where x=��e, y=Uq
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
        // 1. �õ��ͣ����
        ctl_init_const_slope_f_controller(&est->speed_profile_gen, 0.0f,
                                          FLUX_TEST_SPEED_PU[FLUX_TEST_POINTS - 1] * ctl_consult_base_frequency(pu) /
                                              FLUX_RAMP_UP_TIME_S,
                                          est->isr_freq_hz);
        // TODO: �˴�Ӧ��һ���ȴ����ֹͣ����ʱ

        // 2. �л�����һ����״̬
        est->main_state = OFFLINE_MAIN_STATE_J;
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
        // 1. �������Ƿ��ѱ�ʶ
        if (est->pmsm_params.flux < 1e-6)
        {
            est->main_state = OFFLINE_MAIN_STATE_ERROR; // �޷�����ת��
            return;
        }

        // 2. ���õ���������Ϊ Id=0, Iq=0
        ctl_set_current_ref(&est->current_ctrl, 0.0f, 0.0f);
        ctl_enable_current_controller(&est->current_ctrl);

        // 3. �����˲���
        ctl_init_lp_filter(&est->measure_flt[0], est->isr_freq_hz, 50.0f); // Speed
        ctl_init_lp_filter(&est->measure_flt[1], est->isr_freq_hz, 50.0f); // Iq

        // 4. ��ʼ������
        est->sum_x = 0;
        est->sum_y = 0;
        est->sum_xy = 0;
        est->sum_x2 = 0;
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
            // ����Ŀ��Iq
            parameter_gt iq_ref = ctl_consult_Ipeak_to_phy(pu, est->j_test_iq_pu);
            ctl_set_current_ref(&est->current_ctrl, 0.0f, iq_ref);

            // ��ȡ���˲�����ֵ
            parameter_gt speed_rps = ctl_get_mtr_velocity(est->mtr_interface); // ���赥λΪ rps
            parameter_gt speed_rads = ctl_step_lowpass_filter(&est->measure_flt[0], speed_rps * CTL_PARAM_CONST_2PI);
            parameter_gt iq_measured = ctl_step_lowpass_filter(&est->measure_flt[1], est->current_ctrl.idq0.dat[1]);

            // ��ȡ��ǰʱ�� (��)
            parameter_gt time_s = (gmp_base_get_system_tick() - (est->task_start_time + J_STABILIZE_TIME_MS)) / 1000.0f;

            // �ۼ��������Իع��ƽ��ת�ؼ���
            est->sum_x += time_s;               // ��t
            est->sum_y += speed_rads;           // ����
            est->sum_xy += time_s * speed_rads; // ��(t*��)
            est->sum_x2 += time_s * time_s;     // ��(t^2)
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

            // 2. ����Ǽ��ٶ� (���Իع�б��)
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
                    est->pmsm_params.inertia = est->avg_torque / alpha;
                }
                else
                {
                    est->main_state = OFFLINE_MAIN_STATE_ERROR; // ���ٶȹ�С
                }
            }
            else
            {
                est->main_state = OFFLINE_MAIN_STATE_ERROR; // ���ݴ���
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
