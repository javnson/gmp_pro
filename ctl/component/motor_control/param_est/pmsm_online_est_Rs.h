/**
 * @file online_rs_estimator.h
 * @author Javnson & Gemini
 * @brief Implements an online stator resistance (Rs) estimator using an MRAS scheme.
 * @version 1.0
 * @date 2025-08-13
 *
 * @copyright Copyright GMP(c) 2025
 *
 */

#ifndef _FILE_ONLINE_RS_ESTIMATOR_H_
#define _FILE_ONLINE_RS_ESTIMATOR_H_

#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ONLINE_RS_ESTIMATOR Online Rs Estimator (MRAS)
 * @brief An online estimator for stator resistance based on the Model Reference Adaptive System principle.
 * @details This module continuously updates the estimated stator resistance (Rs) to track changes
 * due to temperature variations during motor operation. It uses the d-axis voltage equation
 * as the reference model and a PI controller as the adaptation mechanism.
 * @{
 */

//================================================================================
// 1. ���ṹ�嶨�� (Main Structure Definition)
//================================================================================

typedef struct
{
    // --- �ڲ�ģ�� (Internal Modules) ---
    ctl_pid_t adapt_pi; /**< ��������Ӧ���Ƶ�PI������ */

    // --- ���ӵĵ������ (Linked Motor Parameters) ---
    const ctl_pmsm_dsn_consultant_t* pmsm_params; /**< ָ�����Ld, Lq�Ȳ����Ľṹ�� */

    // --- ״̬���� (State Variables) ---
    parameter_gt id_prev; /**< �洢��һʱ�̵�d����������ڼ���΢�� */
    parameter_gt Ts;      /**< �������� (s) */

    // --- ��� (Output) ---
    parameter_gt Rs_estimated; /**< ���߱�ʶ���ĵ���ֵ (Ohm) */

} ctl_online_rs_est_t;

//================================================================================
// 2. ���ĺ��� (Core Functions)
//================================================================================

/**
 * @brief ��ʼ�����ߵ����ʶ��.
 * @param est Pointer to the online estimator structure.
 * @param pmsm Pointer to the motor design parameters (must contain initial Rs, Ld, Lq).
 * @param isr_freq_hz The frequency of the control ISR in Hz.
 * @param adapt_kp The proportional gain for the adaptation PI controller.
 * @param adapt_ki The integral gain for the adaptation PI controller.
 */
GMP_STATIC_INLINE void ctl_init_online_rs_est(ctl_online_rs_est_t* est, const ctl_pmsm_dsn_consultant_t* pmsm,
                                              parameter_gt isr_freq_hz, parameter_gt adapt_kp, parameter_gt adapt_ki)
{
    est->pmsm_params = pmsm;
    est->Ts = 1.0f / isr_freq_hz;
    est->id_prev = 0.0f;

    // ʹ�����߱�ʶ��Rs��Ϊ��ʼֵ
    est->Rs_estimated = pmsm->Rs;

    // ��ʼ��PI������
    // ������ƿ��Ը��ݵ����Ԥ�ڱ仯��Χ���趨
    parameter_gt rs_max = pmsm->Rs * 2.0f; // ���磬��������Ϊ��ʼֵ��2��
    parameter_gt rs_min = pmsm->Rs * 0.5f; // ���磬��������Ϊ��ʼֵ��0.5��
    ctl_init_pid(&est->adapt_pi, adapt_kp, adapt_ki, 0.0f, rs_max, rs_min, isr_freq_hz);

    // ��PI�������ĳ�ʼ�������Ϊ��ʼ����ֵ
    ctl_set_pid_integral(&est->adapt_pi, est->Rs_estimated);
}

/**
 * @brief ���ߵ����ʶ���Ĳ������� (Ӧ��ISR����FOCͬ������).
 * @param est Pointer to the online estimator structure.
 * @param ud The commanded d-axis voltage from the current controller.
 * @param id The measured d-axis current.
 * @param iq The measured q-axis current.
 * @param omega_e The measured electrical angular velocity (rad/s).
 * @return The latest estimated stator resistance (Ohm).
 */
GMP_STATIC_INLINE parameter_gt ctl_step_online_rs_est(ctl_online_rs_est_t* est, parameter_gt ud, parameter_gt id,
                                                      parameter_gt iq, parameter_gt omega_e)
{
    // --- 1. �������΢���� (did/dt) ---
    // ʹ�ú����ֽ���: d_id/dt �� (id[k] - id[k-1]) / Ts
    parameter_gt did_dt = (id - est->id_prev) / est->Ts;

    // --- 2. ��ȡ����ĵ������ ---
    parameter_gt Ld = est->pmsm_params->Ld;
    parameter_gt Lq = est->pmsm_params->Lq;

    // --- 3. �����ο�ģ�ͺͿɵ�ģ�� ---
    // �ο�ģ����� (Reference Model Output): ud_ref = ud - Ld*did/dt + ��e*Lq*iq
    parameter_gt ud_ref = ud - Ld * did_dt + omega_e * Lq * iq;

    // �ɵ�ģ����� (Adjustable Model Output): ud_adj = Rs_hat * id
    parameter_gt ud_adj = est->Rs_estimated * id;

    // --- 4. ������� ---
    parameter_gt error = ud_ref - ud_adj;

    // --- 5. ��������Ӧ���� (PI������) ---
    // Ϊ�˱�֤�ȶ��ԣ�ͨ��ʹ�� id ��Ϊ����Ȩ��
    // ��Ϊ�򻯣��ڵ��ٻ�id�仯���������£���ֱ��ʹ��error
    // һ�����еķ�����ʹ��id�ķ��Ż��ֵ�����޷�
    if (fabsf(id) > 1e-3) // ����d�������Ϊ��ʱ���£����ⲻ�ȶ�
    {
        est->Rs_estimated = ctl_step_pid_ser(&est->adapt_pi, error);
    }

    // --- 6. ����״̬���� ---
    est->id_prev = id;

    return est->Rs_estimated;
}

/** @} */ // end of ONLINE_RS_ESTIMATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ONLINE_RS_ESTIMATOR_H_
