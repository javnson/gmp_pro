/**
 * @file sogi_traj.h
 * @brief Implements a State-Variable Filter (SOGI) based Trajectory Planner.
 * @details Achieves C2 continuous smooth trajectories with ZERO numerical differentiation noise.
 *
 * @version 1.0
 * @date 2024-10-26
 */

#ifndef _FILE_SOGI_TRAJECTORY_H_
#define _FILE_SOGI_TRAJECTORY_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/discrete/discrete_sogi.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* SOGI (State Variable Filter) Trajectory Planner                           */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initialization parameters for the SOGI Trajectory Planner.
 */
typedef struct _tag_sogi_planner_init
{
    parameter_gt fs_motion;  //!< Execution frequency of the motion layer (Hz).
    parameter_gt omega_base; //!< Base mechanical speed (rad/s) for PU conversion.

    parameter_gt max_vel_pu;   //!< Maximum cruising velocity (PU, 1.0 = omega_base).
    parameter_gt max_accel_pu; //!< Maximum acceleration (PU/s).

    parameter_gt tracking_err_limit; //!< Max allowed following error (Revolutions).
    parameter_gt fault_time_ms;      //!< Time threshold to trigger divergence fault (ms).
} ctl_sogi_planner_init_t;

/**
 * @brief Main structure for the SOGI trajectory planner.
 */
typedef struct _tag_sogi_planner
{
    // --- Interfaces & Shared Resources ---
    rotation_ift* pos_if;      //!< Real position feedback.
    ctl_divider_t* div_shared; //!< Shared system divider for TDM.

    // --- SOGI Core (The Pre-Filter) ---
    discrete_sogi_t sogi_core; //!< SOGI object used as a 2nd-order critically damped LPF.

    // --- Pre-calculated Math Constants ---
    ctrl_gt max_vel_limit;  //!< Absolute limit for PU velocity.
    ctrl_gt scale_v_to_rev; //!< Convert PU speed to Delta Revs per tick.
    ctrl_gt omega_0_pu;     //!< SOGI natural frequency. Used to extract Accel: A = w0 * D(s).

    // Exact Analytical Braking Constants for a Critically Damped System
    // S_brake = K_v * |V| + K_a * (A * sgn(V))
    ctrl_gt coef_brake_v; //!< Braking coefficient for current velocity state.
    ctrl_gt coef_brake_a; //!< Braking coefficient for current acceleration state.

    ctrl_gt arrival_tol_revs; //!< Tolerance for exact target snap.
    ctrl_gt arrival_tol_vel;  //!< Tolerance for zero-velocity snap.

    // --- State Variables (Ideal Planned States) ---
    int32_t planner_revs;   //!< Ideal planned position (Full revolutions).
    ctrl_gt planner_angle;  //!< Ideal planned position (Fractional angle 0~1).
    ctrl_gt planner_vel_pu; //!< Ideal planned velocity (PU).
    ctrl_gt planner_acc_pu; //!< Ideal planned acceleration (PU/s) extracted NOISE-FREE from SOGI.

    // --- Target ---
    int32_t target_revs;
    ctrl_gt target_angle;

    // --- Protection States ---
    ctrl_gt tracking_err_limit;
    uint32_t divergence_cnt;
    uint32_t divergence_limit;

    // --- Flags ---
    fast_gt flag_enable;
    fast_gt flag_fault_divergence;
    fast_gt flag_target_reached;

} ctl_sogi_planner_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_sogi_planner(ctl_sogi_planner_t* planner, const ctl_sogi_planner_init_t* init);

/**
 * @brief Safely calculates pure position difference (Target - Feedback).
 */
GMP_STATIC_INLINE ctrl_gt ctl_calc_pos_err_4param(int32_t t_revs, ctrl_gt t_ang, int32_t f_revs, ctrl_gt f_ang)
{
    int32_t d_revs = t_revs - f_revs;
    if (d_revs > 100)
        return float2ctrl(100.0f);
    if (d_revs < -100)
        return float2ctrl(-100.0f);
    return float2ctrl((float)d_revs) + (t_ang - f_ang);
}

GMP_STATIC_INLINE void ctl_attach_sogi_planner(ctl_sogi_planner_t* planner, rotation_ift* pos_if,
                                               ctl_divider_t* div_shared)
{
    gmp_base_assert(pos_if != NULL);
    gmp_base_assert(div_shared != NULL);
    gmp_base_assert(div_shared->target >= 3);

    planner->pos_if = pos_if;
    planner->div_shared = div_shared;
}

/**
 * @brief Bumpless Sync: Snaps the planner's states to the actual motor feedback.
 */
GMP_STATIC_INLINE void ctl_sync_sogi_planner(ctl_sogi_planner_t* planner)
{
    if (planner->pos_if)
    {
        planner->planner_revs = planner->pos_if->revolutions;
        planner->planner_angle = planner->pos_if->position;
        planner->target_revs = planner->planner_revs;
        planner->target_angle = planner->planner_angle;
    }
    planner->planner_vel_pu = float2ctrl(0.0f);
    planner->planner_acc_pu = float2ctrl(0.0f);

    // Clear SOGI historical states
    ctl_clear_discrete_sogi(&planner->sogi_core);

    planner->divergence_cnt = 0;
    planner->flag_fault_divergence = 0;
    planner->flag_target_reached = 1;
}

GMP_STATIC_INLINE void ctl_set_sogi_target(ctl_sogi_planner_t* planner, int32_t target_revs, ctrl_gt target_angle)
{
    planner->target_revs = target_revs;
    planner->target_angle = target_angle;
    planner->flag_target_reached = 0;
}

/**
 * @brief Executes Time-Division Multiplexed (TDM) SOGI Trajectory Planning.
 */
GMP_STATIC_INLINE void ctl_step_sogi_planner_tdm(ctl_sogi_planner_t* planner)
{
    if (!planner->flag_enable || planner->flag_fault_divergence)
        return;

    uint32_t cur_slot = ctl_divider_get_cnt(planner->div_shared);

    // ========================================================================
    // SLOT 1: Motion Planning (Bang-Bang into SOGI Filter)
    // ========================================================================
    if (cur_slot == 1)
    {
        ctrl_gt dist_to_go = ctl_calc_pos_err_4param(planner->target_revs, planner->target_angle, planner->planner_revs,
                                                     planner->planner_angle);

        ctrl_gt abs_v =
            (planner->planner_vel_pu > float2ctrl(0.0f)) ? planner->planner_vel_pu : -planner->planner_vel_pu;
        ctrl_gt abs_dist = (dist_to_go > float2ctrl(0.0f)) ? dist_to_go : -dist_to_go;

        // 1. Exact Arrival Snapping
        if (abs_dist <= planner->arrival_tol_revs && abs_v <= planner->arrival_tol_vel)
        {
            planner->planner_vel_pu = float2ctrl(0.0f);
            planner->planner_acc_pu = float2ctrl(0.0f);
            planner->planner_revs = planner->target_revs;
            planner->planner_angle = planner->target_angle;

            ctl_clear_discrete_sogi(&planner->sogi_core);
            planner->flag_target_reached = 1;
            return;
        }

        // 2. Analytical Braking Check for Critically Damped System
        // Braking Dist = K_v * |V| + K_a * (A * sgn(V))
        ctrl_gt accel_sgn_v =
            (planner->planner_vel_pu >= float2ctrl(0.0f)) ? planner->planner_acc_pu : -planner->planner_acc_pu;
        ctrl_gt s_brake = ctl_mul(planner->coef_brake_v, abs_v) + ctl_mul(planner->coef_brake_a, accel_sgn_v);

        // Safeguard against over-braking estimation during direction reversal
        if (s_brake < float2ctrl(0.0f))
            s_brake = float2ctrl(0.0f);

        ctrl_gt raw_speed_ref = float2ctrl(0.0f);
        ctrl_gt direction = (dist_to_go > float2ctrl(0.0f)) ? float2ctrl(1.0f) : float2ctrl(-1.0f);

        ctrl_gt dir_judge = ctl_mul(dist_to_go, planner->planner_vel_pu);
        fast_gt moving_towards = (dir_judge >= float2ctrl(0.0f)) ? 1 : 0;

        if (moving_towards && (abs_dist <= s_brake))
        {
            raw_speed_ref = float2ctrl(0.0f); // Enter braking zone -> Command Zero to Filter
        }
        else
        {
            raw_speed_ref = ctl_mul(direction, planner->max_vel_limit); // Cruise/Chase
        }

        // 3. Step SOGI Core (The Magic Happens Here)
        ctl_step_discrete_sogi(&planner->sogi_core, raw_speed_ref);

        // 4. Extract NOISE-FREE States directly from SOGI topology
        // Q(s) is the low-pass filtered velocity
        planner->planner_vel_pu = ctl_get_discrete_sogi_qs(&planner->sogi_core);

        // D(s) is the band-pass state. Mathematically, Accel = omega_0 * D(s). Pure algebraic calculation!
        ctrl_gt sogi_ds = ctl_get_discrete_sogi_ds(&planner->sogi_core);
        planner->planner_acc_pu = ctl_mul(sogi_ds, planner->omega_0_pu);

        // 5. Integrate Velocity to Position (Convert Speed PU to Delta Revs)
        ctrl_gt delta_angle = ctl_mul(planner->planner_vel_pu, planner->scale_v_to_rev);
        planner->planner_angle += delta_angle;

        while (planner->planner_angle >= float2ctrl(1.0f))
        {
            planner->planner_angle -= float2ctrl(1.0f);
            planner->planner_revs++;
        }
        while (planner->planner_angle < float2ctrl(0.0f))
        {
            planner->planner_angle += float2ctrl(1.0f);
            planner->planner_revs--;
        }
    }
    // ========================================================================
    // SLOT 2: Divergence Protection
    // ========================================================================
    else if (cur_slot == 2)
    {
        ctrl_gt following_error = ctl_calc_pos_err_4param(planner->planner_revs, planner->planner_angle,
                                                          planner->pos_if->revolutions, planner->pos_if->position);

        ctrl_gt abs_err = (following_error > float2ctrl(0.0f)) ? following_error : -following_error;

        if (abs_err > planner->tracking_err_limit)
        {
            planner->divergence_cnt++;
            if (planner->divergence_cnt >= planner->divergence_limit)
            {
                planner->flag_fault_divergence = 1;
                planner->planner_vel_pu = float2ctrl(0.0f);
                planner->planner_acc_pu = float2ctrl(0.0f);
                ctl_clear_discrete_sogi(&planner->sogi_core);
            }
        }
        else
        {
            if (planner->divergence_cnt > 0)
                planner->divergence_cnt--;
        }
    }
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SOGI_TRAJECTORY_H_
