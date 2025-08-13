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

// �������б���ĵײ�ģ��ͷ�ļ�
#include <ctl/component/intrinsic/discrete/signal_generator.h>
#include <ctl/component/intrinsic/filter/discrete_filter.h>
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/current_controller/motor_current_ctrl.h>
#include <ctl/component/motor_control/generator/vf_generator.h>
#include <ctl/component/motor_control/interface/motor_universal_interface.h>
#include <ctl/math_block/vector/vector3.h>

// �����ϵͳ����ͷ�ļ�
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
// 1. ö�������� (Enums & Configurations)
//================================================================================

typedef enum
{
    ENCODER_TYPE_NONE,     /**< �ޱ����� */
    ENCODER_TYPE_ABSOLUTE, /**< ����ֵ������ (��űࡢ���) */
    ENCODER_TYPE_QEP       /**< ����ʽQEP������ */
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
// 2. ���ṹ�嶨�� (Main Structure Definition)
//================================================================================

typedef struct ctl_offline_est_s
{
    /*-------------------- ���� (Configuration) --------------------*/
    ctl_encoder_type_e encoder_type; /**< �û�ָ���ı��������� */
    parameter_gt isr_freq_hz;        /**< �����жϵ�Ƶ�� (Hz) */
    parameter_gt rs_test_current_pu; /**< Rs������ʹ�õĵ�������ֵ */
    uint16_t qep_search_elec_cycles; /**< QEP index����ʱ��ת�ĵ������� */
    parameter_gt l_hfi_v_pu;         /**< L��ʶʱע��ĸ�Ƶ��ѹ����ֵ */
    parameter_gt l_hfi_freq_hz;      /**< L��ʶʱע��ĸ�Ƶ��ѹƵ�� (Hz) */
    parameter_gt l_hfi_rot_freq_hz;  /**< L��ʶʱ��Ƶʸ������תƵ�� (Hz) */
    parameter_gt j_test_iq_pu;       /**< J��ʶʱʩ�ӵ�q���������ֵ */

    /*-------------------- ģ��ӿ� (Module Interfaces) --------------------*/
    mtr_ift* mtr_interface;                   /**< ָ��ͨ�õ���������ӿڵ�ָ�� */
    ctl_per_unit_consultant_t* pu_consultant; /**< ָ�����ֵϵͳ���ʵ�ָ�� */
    ctl_vector3_t vab_command;                /**< �����PWMģ���alpha-beta��ѹָ�� */

    /*-------------------- ����ģ�� (Control Modules) --------------------*/
    ctl_current_controller_t current_ctrl;
    ctl_slope_f_controller speed_profile_gen;
    ctl_sine_generator_t hfi_signal_gen;
    ctl_low_pass_filter_t measure_flt[4]; /**< 0:Vd, 1:Id, 2:Pos, 3:I_hfi_mag */

    /*-------------------- ״̬���־λ (State & Flags) --------------------*/
    ctl_offline_est_main_state_e main_state;
    ctl_offline_est_sub_state_e sub_state;
    fast_gt flag_start_estimation;
    fast_gt flag_estimation_done;
    fast_gt flag_error_detected;
    time_gt task_start_time;

    /*-------------------- �м���� (Intermediate Variables) --------------------*/
    // ͨ���ۼ���
    uint32_t sample_count;
    parameter_gt sum_x, sum_y, sum_xy, sum_x2; /**< �������Իع���ۼ��� */
    uint16_t step_index;

    // Rs & Encoder ��ʶ����
    parameter_gt rs_step_results[6];
    parameter_gt enc_offset_results[6];

    // Ld/Lq ��ʶ����
    parameter_gt hfi_i_max, hfi_i_min;
    parameter_gt hfi_theta_d, hfi_theta_q;

    // J ��ʶ����
    parameter_gt avg_torque; /**< J��ʶ�ڼ��ƽ��ת�� */

    /*-------------------- ���ձ�ʶ��� (Final Identified Parameters) --------------------*/
    ctl_pmsm_dsn_consultant_t pmsm_params;
    ctl_vector3_t Rs_line_to_line;
    parameter_gt encoder_offset;
    parameter_gt current_noise_std_dev;
    parameter_gt position_consistency_std_dev;

} ctl_offline_est_t;

//================================================================================
// 3. ���ĺ���������ʵ�� (Core Function Declarations & Implementations)
//================================================================================

// --- ǰ������״̬������ ---
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
 * @brief ��ʼ�����߲�����ʶģ��.
 */
GMP_STATIC_INLINE void ctl_init_offline_est(ctl_offline_est_t* est, mtr_ift* mtr_if, ctl_per_unit_consultant_t* pu_cons,
                                            ctl_encoder_type_e enc_type, parameter_gt isr_freq, parameter_gt current_kp,
                                            parameter_gt current_ki, parameter_gt rs_curr_pu, uint16_t qep_cycles,
                                            parameter_gt l_v_pu, parameter_gt l_freq_hz, parameter_gt l_rot_freq_hz,
                                            parameter_gt j_iq_pu)
{
    est->mtr_interface = mtr_if;
    est->pu_consultant = pu_cons;

    // ��������
    est->encoder_type = enc_type;
    est->isr_freq_hz = isr_freq;
    est->rs_test_current_pu = rs_curr_pu;
    est->qep_search_elec_cycles = qep_cycles;
    est->l_hfi_v_pu = l_v_pu;
    est->l_hfi_freq_hz = l_freq_hz;
    est->l_hfi_rot_freq_hz = l_rot_freq_hz;
    est->j_test_iq_pu = j_iq_pu;

    // ��ʼ��ģ��
    ctl_init_current_controller(&est->current_ctrl, current_kp, current_ki, 0, ctl_consult_base_peak_voltage(pu_cons),
                                -ctl_consult_base_peak_voltage(pu_cons), isr_freq);
    for (int i = 0; i < 4; ++i)
        ctl_clear_lowpass_filter(&est->measure_flt[i]);

    // ��ʼ��״̬
    est->main_state = OFFLINE_MAIN_STATE_IDLE;
    est->sub_state = OFFLINE_SUB_STATE_INIT;
    est->flag_start_estimation = 0;
    est->flag_estimation_done = 0;
    est->flag_error_detected = 0;

    // ��ս��
    ctl_init_pmsm_dsn_consultant(&est->pmsm_params, 0, 0, 0, 0, 0, 0, 0);
    ctl_vector3_clear(&est->vab_command);
    ctl_vector3_clear(&est->Rs_line_to_line);
    est->encoder_offset = 0.0f;
    est->current_noise_std_dev = 0.0f;
    est->position_consistency_std_dev = 0.0f;
}

/**
 * @brief ���߲�����ʶģ���ʵʱ�������� (��ISR�е���).
 */
GMP_STATIC_INLINE void ctl_step_offline_est(ctl_offline_est_t* est)
{
    if (est->main_state > OFFLINE_MAIN_STATE_IDLE && est->main_state < OFFLINE_MAIN_STATE_DONE)
    {
        const ctl_vector3_t* iabc = ctl_get_mtr_current(est->mtr_interface);

        // ���ݲ�ͬ״̬��FOC��theta�в�ͬ��Դ
        ctrl_gt theta = 0.0f;
        if (est->main_state == OFFLINE_MAIN_STATE_L)
        {
            theta = ctl_get_ramp_generator_output(&est->speed_profile_gen.rg);
        }
        else if (est->main_state == OFFLINE_MAIN_STATE_RS)
        {
            // TODO: ��Rs��ʶ�����ù̶���theta
        }
        else
        {
            theta = ctl_get_mtr_elec_theta(est->mtr_interface);
        }

        // HFIע��ʱ����ѹ��ǰ������
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
 * @brief ���߲�����ʶģ��ĺ�̨ѭ������ (����ѭ���е���).
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
