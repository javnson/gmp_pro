
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// SOGI curve trajectory

#include <ctl/component/motor_control/motion/sogi_traj.h>

/**
 * @brief Initializes the SOGI-based planner.
 * @details Computes the natural frequency of SOGI based on A_max and V_max, 
 * configures it as a critically damped filter, and resolves the braking coefficients.
 */
void ctl_init_sogi_planner(ctl_sogi_planner_t* planner, const ctl_sogi_planner_init_t* init)
{
    // 1. Safe Guards
    parameter_gt fs_safe = (init->fs_motion > 1e-6f) ? init->fs_motion : 1000.0f;
    parameter_gt omega_base = (init->omega_base > 1e-6f) ? init->omega_base : 1.0f;

    // 2. Physics-Based Bandwidth Calculation
    // For a critically damped 2nd-order system, max slope (Accel) is reached at t=1/w0.
    // A_max = V_max * w0 / e  =>  w0 = (A_max * e) / V_max
    parameter_gt e_const = 2.7182818f;
    parameter_gt w0 = (init->max_accel_pu * e_const) / init->max_vel_pu;

    // SOGI init uses fn (Hz)
    parameter_gt fn = w0 / CTL_PARAM_CONST_2PI;

    // 3. Initialize underlying SOGI as Critically Damped LPF
    // The damping ratio zeta = 1.0. In SOGI transfer function, denominator is s^2 + k*w0*s + w0^2.
    // So k = 2*zeta = 2.0.
    ctl_init_discrete_sogi(&planner->sogi_core, 2.0f, fn, fs_safe);

    // 4. Base Configurations
    planner->max_vel_limit = float2ctrl(init->max_vel_pu);
    planner->omega_0_pu = float2ctrl(w0); // Store w0 to extract Accel = w0 * D(s)

    // 1.0 PU Velocity = omega_base rad/s = (omega_base / 2*PI) revs/s
    parameter_gt scale_time_to_rev = omega_base / CTL_PARAM_CONST_2PI;
    // Integrate in discrete time: Delta Revs = V_pu * scale * (1/fs)
    planner->scale_v_to_rev = float2ctrl(scale_time_to_rev / fs_safe);

    // 5. Perfect Exact Analytical Braking
    // The total area under the velocity decay curve for a critically damped system is:
    // S_stop(t->inf) = (2/w0)*V_0 + (1/w0^2)*A_0
    // We convert this pure PU Area into Revolutions
    parameter_gt k_v = (2.0f / w0) * scale_time_to_rev;
    parameter_gt k_a = (1.0f / (w0 * w0)) * scale_time_to_rev;

    planner->coef_brake_v = float2ctrl(k_v);
    planner->coef_brake_a = float2ctrl(k_a);

    // 6. Arrival Tolerance (Cut off infinite decay tail)
    planner->arrival_tol_vel = float2ctrl(init->max_vel_pu * 0.005f);       // 0.5% vel cutoff
    planner->arrival_tol_revs = float2ctrl(k_v * init->max_vel_pu * 0.01f); // 1% distance cutoff

    // 7. Protection Settings
    planner->tracking_err_limit = float2ctrl(init->tracking_err_limit);
    planner->divergence_limit = (uint32_t)(init->fault_time_ms * fs_safe / 1000.0f);
    if (planner->divergence_limit < 1)
        planner->divergence_limit = 1;

    // 8. Clear States
    ctl_sync_sogi_planner(planner);
    planner->flag_enable = 0;
}
