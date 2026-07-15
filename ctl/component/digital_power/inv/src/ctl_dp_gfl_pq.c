/**
 * @file ctl_dp_three_phase.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implementation for three-phase digital power controller modules.
 * @version 1.05
 * @date 2025-08-05
 *
 * @copyright Copyright GMP(c) 2024
 *
 * @brief Initialization functions for three-phase digital power components,
 * including the Three-Phase PLL and bridge modulation modules.
 */

#include <gmp_core.h>
#include <math.h>

//////////////////////////////////////////////////////////////////////////
// Three Phase GFL Converter
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/inv/gfl_pq_ctrl.h>

void ctl_init_gfl_pq(gfl_pq_ctrl_t* pq, parameter_gt p_kp, parameter_gt p_ki, parameter_gt q_kp, parameter_gt q_ki,
                     parameter_gt i_out_max, parameter_gt fs)
{
    pq->flag_enable = 0;
    ctl_init_pid(&pq->pid_p, p_kp, p_ki, 0, fs);
    ctl_init_pid(&pq->pid_q, q_kp, q_ki, 0, fs);
    ctrl_gt current_limit = float2ctrl(i_out_max);
    pq->max_i2_mag = ctl_mul(current_limit, current_limit);
    ctl_set_pid_limit(&pq->pid_p, current_limit, -current_limit);
    ctl_set_pid_limit(&pq->pid_q, current_limit, -current_limit);
    ctl_set_pid_int_limit(&pq->pid_p, current_limit, -current_limit);
    ctl_set_pid_int_limit(&pq->pid_q, current_limit, -current_limit);
    ctl_vector2_clear(&pq->pq_set);
    ctl_vector2_clear(&pq->pq_meas);
    ctl_vector2_clear(&pq->idq_set_out);
    pq->s_mag_sq = 0;
}

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq(gfl_pq_ctrl_t* pq, ctl_vector2_t* vdq, ctl_vector2_t* idq)
{
    pq->vdq_meas = vdq;
    pq->idq_meas = idq;
}

/**
 * @brief Attach feedback pointers to the PQ controller.
 * @param[in,out] pq Pointer to the PQ controller instance.
 * @param[in] vdq Pointer to the inner loop's Vdq measurement.
 * @param[in] idq Pointer to the inner loop's Idq measurement.
 */
void ctl_attach_gfl_pq_to_core(gfl_pq_ctrl_t* pq, gfl_inv_ctrl_t* core)
{
    gmp_base_assert(core);
    gmp_base_assert(pq);

    pq->vdq_meas = &core->vdq;
    pq->idq_meas = &core->idq;
}
