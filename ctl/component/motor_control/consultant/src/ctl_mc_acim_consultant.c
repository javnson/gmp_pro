
#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/acim_consultant.h>

/**
 * @brief Initializes and validates the IM physical model.
 * @details Pre-calculates the complex cross-coupling equivalent parameters 
 * necessary for IFOC tuning and MPC predictive matrices.
 */
void ctl_consultant_im_init(ctl_consultant_im_t* motor, uint32_t pp, parameter_gt rs, parameter_gt rr, parameter_gt ls,
                            parameter_gt lr, parameter_gt lm)
{
    // 1. Strict Physical Validation (Gatekeeper)
    // Induction motors rely heavily on accurate inductances. They must be strictly positive.
    gmp_base_assert(pp > 0);
    gmp_base_assert(rs > 0.0f);
    gmp_base_assert(rr > 0.0f);
    gmp_base_assert(ls > 0.0f);
    gmp_base_assert(lr > 0.0f);
    gmp_base_assert(lm > 0.0f);

    // Physics constraint: Mutual inductance cannot exceed self-inductance
    gmp_base_assert(ls > lm);
    gmp_base_assert(lr > lm);

    // 2. Base Parameter Assignment
    motor->pole_pairs = pp;
    motor->Rs = rs;
    motor->Rr = rr;
    motor->Ls = ls;
    motor->Lr = lr;
    motor->Lm = lm;

    // 3. Derived Intrinsic Properties Calculation
    // Rotor time constant (tau_r). Dictates how fast rotor flux builds up.
    motor->tau_r = lr / rr;

    // Lm^2 / Lr is arguably the most frequently used constant in IM vector control.
    motor->Lm_sq_over_Lr = (lm * lm) / lr;

    // Total Leakage Factor (sigma)
    motor->sigma = 1.0f - (motor->Lm_sq_over_Lr / ls);

    // Stator Transient Inductance (sigma * Ls).
    // This is the actual "plant inductance" seen by the high-frequency current loop.
    // Kp of the current loop MUST be tuned using this value, NOT Ls.
    motor->sigma_Ls = motor->sigma * ls;

    // Equivalent Stator Resistance for PI Integral Tuning.
    motor->R_eq = rs + rr * (lm * lm) / (lr * lr);
}
