/**
 * @file scurve_traj.h
 * @brief Implements an industrial-grade Jerk-limited S-Curve Trajectory Planner (Speed PU Base).
 *
 * @version 1.0
 * @date 2024-10-26
 */

#ifndef _FILE_SCURVE_TRAJECTORY_H_
#define _FILE_SCURVE_TRAJECTORY_H_

#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/math_block/gmp_math.h>
#include <gmp_core.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* S-Curve (Jerk-Limited) Trajectory Planner                                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initialization parameters for the S-Curve Planner.
 */
typedef struct _tag_scurve_planner_init
{
    parameter_gt fs_motion;  //!< Execution frequency of the motion layer (Hz).
    parameter_gt omega_base; //!< Base mechanical speed (rad/s) for PU conversion.

    // --- Speed PU Based Configurations ---
    parameter_gt max_vel_pu;   //!< Maximum cruising velocity (PU, 1.0 = omega_base).
    parameter_gt max_accel_pu; //!< Maximum acceleration (PU/s).
    parameter_gt max_jerk_pu;  //!< Maximum jerk (rate of change of accel) (PU/s^2).

    parameter_gt tracking_err_limit; //!< Max allowed following error (Revolutions).
    parameter_gt fault_time_ms;      //!< Time threshold to trigger divergence fault (ms).
} ctl_scurve_planner_init_t;

/**
 * @brief Main structure for the S-Curve trajectory planner.
 */
typedef struct _tag_scurve_planner
{
    // --- Interfaces & Shared Resources ---
    rotation_ift* pos_if;      //!< Real position feedback (Used for divergence protection).
    ctl_divider_t* div_shared; //!< Shared system divider for TDM.

    // --- Core Math Coefficients (Pre-calculated for Fixed-Point) ---
    ctrl_gt dt;              //!< Integration time step (1/fs).
    ctrl_gt max_vel_limit;   //!< Absolute limit for PU velocity.
    ctrl_gt max_accel_limit; //!< Absolute limit for PU acceleration.
    ctrl_gt max_jerk_step;   //!< Max acceleration change per step (PU/s per tick).

    ctrl_gt scale_v_to_rev; //!< Scale factor: PU velocity to delta Revolutions per tick.

    // Braking Math Constants
    ctrl_gt coef_k1_vsq;   //!< K1: For V^2 term in braking distance.
    ctrl_gt coef_k2_v;     //!< K2: For V term in braking distance.
    ctrl_gt coef_k3_flare; //!< K3: For A^2 term to trigger accel ramp-down.

    ctrl_gt arrival_tol_revs; //!< Dynamic position tolerance for exact arrival.
    ctrl_gt arrival_tol_vel;  //!< Dynamic velocity tolerance.

    // --- State Variables (Ideal Planned States) ---
    int32_t planner_revs;   //!< Ideal planned position (Full revolutions).
    ctrl_gt planner_angle;  //!< Ideal planned position (Fractional angle 0~1).
    ctrl_gt planner_vel_pu; //!< Ideal planned velocity (PU based on omega_base).
    ctrl_gt planner_acc_pu; //!< Ideal planned acceleration (PU/s).

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

} ctl_scurve_planner_t;

//================================================================================
// Function Prototypes & Inline Definitions
//================================================================================

void ctl_init_scurve_planner(ctl_scurve_planner_t* planner, const ctl_scurve_planner_init_t* init);

/**
 * @brief Safely calculates pure position difference (Target - Feedback).
 * @details Replaces pos_ext, directly using 4 standard parameters. Clamps to prevent overflow.
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

/**
 * @brief Attaches feedback and shared divider to the planner.
 */
GMP_STATIC_INLINE void ctl_attach_scurve_planner(ctl_scurve_planner_t* planner, rotation_ift* pos_if,
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
GMP_STATIC_INLINE void ctl_sync_scurve_planner(ctl_scurve_planner_t* planner)
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
    planner->divergence_cnt = 0;
    planner->flag_fault_divergence = 0;
    planner->flag_target_reached = 1;
}

/**
 * @brief Sets a new absolute target position.
 */
GMP_STATIC_INLINE void ctl_set_scurve_target(ctl_scurve_planner_t* planner, int32_t target_revs, ctrl_gt target_angle)
{
    planner->target_revs = target_revs;
    planner->target_angle = target_angle;
    planner->flag_target_reached = 0;
}

/**
 * @brief Executes Time-Division Multiplexed (TDM) S-Curve Trajectory Planning.
 */
GMP_STATIC_INLINE void ctl_step_scurve_planner_tdm(ctl_scurve_planner_t* planner)
{
    if (!planner->flag_enable || planner->flag_fault_divergence)
        return;

    uint32_t cur_slot = ctl_divider_get_cnt(planner->div_shared);

    // ========================================================================
    // SLOT 1: Motion Planning (Jerk-Limited S-Curve Generation)
    // ========================================================================
    if (cur_slot == 1)
    {
        // 1. Get Distances and States
        ctrl_gt dist_to_go = ctl_calc_pos_err_4param(planner->target_revs, planner->target_angle, planner->planner_revs,
                                                     planner->planner_angle);

        ctrl_gt abs_v =
            (planner->planner_vel_pu > float2ctrl(0.0f)) ? planner->planner_vel_pu : -planner->planner_vel_pu;
        ctrl_gt abs_a =
            (planner->planner_acc_pu > float2ctrl(0.0f)) ? planner->planner_acc_pu : -planner->planner_acc_pu;
        ctrl_gt abs_dist = (dist_to_go > float2ctrl(0.0f)) ? dist_to_go : -dist_to_go;

        // 2. Exact Arrival Snapping (No Deadband)
        if (abs_dist <= planner->arrival_tol_revs && abs_v <= planner->arrival_tol_vel)
        {
            planner->planner_acc_pu = float2ctrl(0.0f);
            planner->planner_vel_pu = float2ctrl(0.0f);
            planner->planner_revs = planner->target_revs;
            planner->planner_angle = planner->target_angle;
            planner->flag_target_reached = 1;
            return;
        }

        // 3. Physics Check: Are we moving towards the target?
        ctrl_gt dir_judge = ctl_mul(dist_to_go, planner->planner_vel_pu);
        fast_gt moving_towards = (dir_judge >= float2ctrl(0.0f)) ? 1 : 0;
        ctrl_gt direction = (dist_to_go > float2ctrl(0.0f)) ? float2ctrl(1.0f) : float2ctrl(-1.0f);

        // 4. Exact S-Curve Braking Distance: S = K1*V^2 + K2*|V|
        ctrl_gt s_brake = ctl_mul(planner->coef_k1_vsq, ctl_mul(abs_v, abs_v)) + ctl_mul(planner->coef_k2_v, abs_v);

        // 5. Flare Check (When to ramp Accel to 0): V_flare = K3*A^2
        ctrl_gt v_flare = ctl_mul(planner->coef_k3_flare, ctl_mul(abs_a, abs_a));

        // 6. Determine Target Acceleration
        ctrl_gt target_acc = float2ctrl(0.0f);
        if (moving_towards && (abs_dist <= s_brake))
        {
            // Braking Zone
            if (abs_v <= v_flare)
            {
                target_acc = float2ctrl(0.0f); // Final Flare: Jerk will naturally pull Accel to 0
            }
            else
            {
                target_acc = ctl_mul(-direction, planner->max_accel_limit); // Full Braking
            }
        }
        else
        {
            // Cruising or Dynamic Reversal Zone (Point 5 validated)
            target_acc = ctl_mul(direction, planner->max_accel_limit);
        }

        // 7. Apply Jerk Limiter (Slew Rate on Acceleration)
        ctrl_gt delta_acc = target_acc - planner->planner_acc_pu;
        delta_acc = ctl_sat(delta_acc, planner->max_jerk_step, -planner->max_jerk_step);
        planner->planner_acc_pu += delta_acc;

        // 8. Integrate Acceleration to Velocity
        planner->planner_vel_pu += ctl_mul(planner->planner_acc_pu, planner->dt);

        // Velocity Saturation & Anti-Windup
        if (planner->planner_vel_pu > planner->max_vel_limit)
        {
            planner->planner_vel_pu = planner->max_vel_limit;
            planner->planner_acc_pu = float2ctrl(0.0f);
        }
        else if (planner->planner_vel_pu < -planner->max_vel_limit)
        {
            planner->planner_vel_pu = -planner->max_vel_limit;
            planner->planner_acc_pu = float2ctrl(0.0f);
        }

        // 9. Integrate Velocity to Position (Scale to Revs)
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
    // SLOT 2: Divergence Protection (Counting Mechanism)
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

#endif // _FILE_SCURVE_TRAJECTORY_H_
