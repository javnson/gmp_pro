/**
 * @file trapezoidal_trajectory.h
 * @brief Implements an industrial-grade Trapezoidal Trajectory Planner (Speed PU Base).
 *
 * @version 1.3
 * @date 2024-10-26
 */

#ifndef _FILE_TRAPEZOIDAL_TRAJECTORY_H_
#define _FILE_TRAPEZOIDAL_TRAJECTORY_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Trapezoidal Trajectory Planner                                            */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initialization parameters for the Trapezoidal Planner.
 */
typedef struct _tag_trap_planner_init
{
    parameter_gt fs_motion;  //!< Execution frequency of the motion layer (Hz).
    parameter_gt omega_base; //!< Base mechanical speed (rad/s) for PU conversion.

    // --- Speed PU Based Configurations ---
    parameter_gt max_vel_pu;   //!< Maximum cruising velocity (PU, 1.0 = omega_base).
    parameter_gt max_accel_pu; //!< Maximum acceleration (PU/s, 1.0 = omega_base/s).

    parameter_gt tracking_err_limit; //!< Max allowed following error (Revolutions).
    parameter_gt fault_time_ms;      //!< Time threshold to trigger divergence fault (ms).
} ctl_trap_planner_init_t;

/**
 * @brief Main structure for the trapezoidal trajectory planner.
 */
typedef struct _tag_trap_planner
{
    // --- Interfaces & Shared Resources ---
    rotation_ift* pos_if;      //!< Real position feedback (Used for divergence protection).
    ctl_divider_t* div_shared; //!< Shared system divider for TDM.

    // --- Core Math Coefficients (Pre-calculated for Fixed-Point) ---
    ctrl_gt max_vel_limit;       //!< Absolute limit for PU velocity.
    ctrl_gt max_accel_step;      //!< Max acceleration per step (PU/tick).
    ctrl_gt scale_v_to_rev;      //!< Scale factor: PU velocity to delta Revolutions per tick.
    ctrl_gt coef_brake_s_to_vsq; //!< The 'C' constant: C * S_revs = V_pu^2 (For braking check).
    ctrl_gt arrival_tol_revs;    //!< Dynamic tolerance for exact arrival (eliminates deadband).

    // --- State Variables (Ideal Planned States) ---
    int32_t planner_revs;   //!< Ideal planned position (Full revolutions).
    ctrl_gt planner_angle;  //!< Ideal planned position (Fractional angle 0~1).
    ctrl_gt planner_vel_pu; //!< Ideal planned velocity (PU based on omega_base).

    // --- Target ---
    int32_t target_revs;  //!< Target position (Full revolutions).
    ctrl_gt target_angle; //!< Target position (Fractional angle 0~1).

    // --- Protection States ---
    ctrl_gt tracking_err_limit; //!< Divergence limit (Revs).
    uint32_t divergence_cnt;    //!< Fault debounce counter.
    uint32_t divergence_limit;  //!< Fault debounce limit (ticks).

    // --- Flags ---
    fast_gt flag_enable;           //!< Enable execution.
    fast_gt flag_fault_divergence; //!< Fault: Real motor cannot follow the trajectory!
    fast_gt flag_target_reached;   //!< Status: Target has been reached exactly.

} ctl_trap_planner_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_trap_planner(ctl_trap_planner_t* planner, const ctl_trap_planner_init_t* init);

/**
 * @brief Internal Helper: Safely calculates pure position difference (Target - Feedback).
 */
GMP_STATIC_INLINE ctrl_gt ctl_calc_pure_pos_err(int32_t t_revs, ctrl_gt t_ang, int32_t f_revs, ctrl_gt f_ang)
{
    int32_t d_revs = t_revs - f_revs;
    if (d_revs > 100)
        return float2ctrl(100.0f);
    if (d_revs < -100)
        return float2ctrl(-100.0f);
    return float2ctrl((float)d_revs) + (t_ang - f_ang);
}

/**
 * @brief Attaches feedback and shared divider to the planner.
 */
GMP_STATIC_INLINE void ctl_attach_trap_planner(ctl_trap_planner_t* planner, rotation_ift* pos_if,
                                               ctl_divider_t* div_shared)
{
    gmp_base_assert(pos_if != NULL);
    gmp_base_assert(div_shared != NULL);
    gmp_base_assert(div_shared->target >= 3);

    planner->pos_if = pos_if;
    planner->div_shared = div_shared;
}

/**
 * @brief Bumpless Sync: Snaps the planner's ideal states to the actual motor feedback.
 */
GMP_STATIC_INLINE void ctl_sync_trap_planner(ctl_trap_planner_t* planner)
{
    if (planner->pos_if)
    {
        planner->planner_revs = planner->pos_if->revolutions;
        planner->planner_angle = planner->pos_if->position;
        planner->target_revs = planner->planner_revs;
        planner->target_angle = planner->planner_angle;
    }
    planner->planner_vel_pu = float2ctrl(0.0f);
    planner->divergence_cnt = 0;
    planner->flag_fault_divergence = 0;
    planner->flag_target_reached = 1;
}

/**
 * @brief Sets a new absolute target position.
 */
GMP_STATIC_INLINE void ctl_set_trap_target(ctl_trap_planner_t* planner, int32_t target_revs, ctrl_gt target_angle)
{
    planner->target_revs = target_revs;
    planner->target_angle = target_angle;
    planner->flag_target_reached = 0;
}

/**
 * @brief Executes Time-Division Multiplexed (TDM) Trajectory Planning & Protection.
 */
GMP_STATIC_INLINE void ctl_step_trap_planner_tdm(ctl_trap_planner_t* planner)
{
    if (!planner->flag_enable || planner->flag_fault_divergence)
        return;

    uint32_t cur_slot = ctl_divider_get_cnt(planner->div_shared);

    // ========================================================================
    // SLOT 1: Motion Planning (Ideal Trajectory Generation)
    // ========================================================================
    if (cur_slot == 1)
    {
        // 1. Calculate safe distance to go (Revolutions)
        ctrl_gt dist_to_go = ctl_calc_pure_pos_err(planner->target_revs, planner->target_angle, planner->planner_revs,
                                                   planner->planner_angle);

        // Exact Arrival Logic
        ctrl_gt abs_dist = (dist_to_go > float2ctrl(0.0f)) ? dist_to_go : -dist_to_go;
        ctrl_gt abs_vel =
            (planner->planner_vel_pu > float2ctrl(0.0f)) ? planner->planner_vel_pu : -planner->planner_vel_pu;

        if (abs_dist <= planner->arrival_tol_revs && abs_vel <= planner->max_accel_step)
        {
            planner->planner_vel_pu = float2ctrl(0.0f);
            planner->planner_revs = planner->target_revs;
            planner->planner_angle = planner->target_angle;
            planner->flag_target_reached = 1;
            return;
        }

        // 2. Braking Check (Pure Multiplication based on Speed PU)
        ctrl_gt direction = (dist_to_go > float2ctrl(0.0f)) ? float2ctrl(1.0f) : float2ctrl(-1.0f);

        ctrl_gt v_sq = ctl_mul(planner->planner_vel_pu, planner->planner_vel_pu);
        ctrl_gt braking_v_sq = ctl_mul(planner->coef_brake_s_to_vsq, abs_dist);

        ctrl_gt dir_judge = ctl_mul(dist_to_go, planner->planner_vel_pu);
        fast_gt moving_towards = (dir_judge >= float2ctrl(0.0f)) ? 1 : 0;

        ctrl_gt accel = float2ctrl(0.0f);
        if (moving_towards && (v_sq >= braking_v_sq))
        {
            accel = ctl_mul(-direction, planner->max_accel_step); // Brake!
        }
        else
        {
            accel = ctl_mul(direction, planner->max_accel_step); // Accelerate!
        }

        // 3. Integrate Velocity (Speed PU domain) and Saturate
        planner->planner_vel_pu += accel;
        planner->planner_vel_pu = ctl_sat(planner->planner_vel_pu, planner->max_vel_limit, -planner->max_vel_limit);

        // 4. Integrate Position (Convert Speed PU to Delta Revs)
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
    // SLOT 2: Divergence Protection (Real-time Follow Error Check)
    // ========================================================================
    else if (cur_slot == 2)
    {
        ctrl_gt following_error = ctl_calc_pure_pos_err(planner->planner_revs, planner->planner_angle,
                                                        planner->pos_if->revolutions, planner->pos_if->position);

        ctrl_gt abs_err = (following_error > float2ctrl(0.0f)) ? following_error : -following_error;

        if (abs_err > planner->tracking_err_limit)
        {
            planner->divergence_cnt++;
            if (planner->divergence_cnt >= planner->divergence_limit)
            {
                planner->flag_fault_divergence = 1;
                planner->planner_vel_pu = float2ctrl(0.0f);
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

#endif // _FILE_TRAPEZOIDAL_TRAJECTORY_H_
