

#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/mech_consultant.h>

/**
 * @brief Initializes and validates the 1-Mass rigid mechanical model.
 */
void ctl_consultant_mech1_init(ctl_consultant_mech1_t* mech, parameter_gt j_tot, parameter_gt b_vis)
{
    // 1. Strict Physical Validation
    gmp_base_assert(j_tot > 0.0f);  // Inertia MUST be positive
    gmp_base_assert(b_vis >= 0.0f); // Friction cannot be negative (would imply active energy injection)

    // 2. Base Parameter Assignment
    mech->J_total = j_tot;
    mech->B_viscous = b_vis;

    // 3. Intrinsic Properties Derivation
    if (b_vis > 1e-6f)
    {
        mech->tau_m = j_tot / b_vis;
    }
    else
    {
        // If viscous friction is basically zero, the time constant is infinite
        mech->tau_m = 1e6f;
    }
}

/**
 * @brief Initializes and validates the 2-Mass flexible mechanical model.
 * @details Automatically calculates the resonance and anti-resonance frequencies
 * based on the physical properties of the transmission system.
 */
void ctl_consultant_mech2_init(ctl_consultant_mech2_t* mech, parameter_gt j_m, parameter_gt j_l, parameter_gt k_s,
                               parameter_gt c_d)
{
    // 1. Strict Physical Validation
    gmp_base_assert(j_m > 0.0f);
    gmp_base_assert(j_l >= 0.0f); // Load could theoretically be zero if disconnected
    gmp_base_assert(k_s > 0.0f);  // Stiffness must exist for a 2-mass system
    gmp_base_assert(c_d >= 0.0f);

    // 2. Base Parameter Assignment
    mech->J_motor = j_m;
    mech->J_load = j_l;
    mech->K_stiff = k_s;
    mech->C_damp = c_d;

    // 3. Intrinsic Properties Derivation
    mech->J_total = j_m + j_l;

    // Avoid division by zero if there is no load connected
    parameter_gt jl_safe = (j_l > 1e-6f) ? j_l : 1e-6f;

    // Anti-Resonance Frequency (w_ares)
    // The frequency at which the load acts as a dynamic vibration absorber for the motor.
    // Formula: w_ares = sqrt(K_s / J_l)
    mech->w_ares_rads = sqrtf(k_s / jl_safe);
    mech->f_ares_hz = mech->w_ares_rads / CTL_PARAM_CONST_2PI;

    // Resonance Frequency (w_res)
    // The frequency at which the energy violently oscillates between motor and load.
    // Formula: w_res = sqrt(K_s * (1/J_m + 1/J_l))
    mech->w_res_rads = sqrtf(k_s * (1.0f / j_m + 1.0f / jl_safe));
    mech->f_res_hz = mech->w_res_rads / CTL_PARAM_CONST_2PI;
}
