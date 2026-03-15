
#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/pu_consultant.h>

/**
 * @brief Initializes the PMSM PU model.
 */
void ctl_consultant_pu_pmsm_init(ctl_consultant_pu_pmsm_t* pu, parameter_gt v_base, parameter_gt i_base,
                                 parameter_gt w_base, uint32_t pole_pairs)
{
    // Validate fundamental bases
    gmp_base_assert(v_base > 0.0f);
    gmp_base_assert(i_base > 0.0f);
    gmp_base_assert(w_base > 0.0f);
    gmp_base_assert(pole_pairs > 0);

    // 1. Assign Fundamental Bases
    pu->V_base = v_base;
    pu->I_base = i_base;
    pu->W_base = w_base;

    // 2. Derive Electrical Bases
    pu->Z_base = v_base / i_base;
    pu->L_base = pu->Z_base / w_base;
    pu->Flux_base = v_base / w_base;

    // 3. Derive Mechanical Bases
    // Peak power = 3/2 * V_peak * I_peak (assuming amplitude invariant Park transform)
    pu->P_base = 1.5f * v_base * i_base;

    // Torque = Power / Mechanical Speed
    // W_mech = W_elec / pole_pairs
    parameter_gt w_mech_base = w_base / (parameter_gt)pole_pairs;
    pu->T_base = pu->P_base / w_mech_base;
}

/**
 * @brief Initializes the IM PU model with secondary base derivation.
 */
void ctl_consultant_pu_im_init(ctl_consultant_pu_im_t* pu, parameter_gt v_base, parameter_gt i_base,
                               parameter_gt w_base, uint32_t pole_pairs, parameter_gt turns_ratio)
{
    gmp_base_assert(v_base > 0.0f);
    gmp_base_assert(i_base > 0.0f);
    gmp_base_assert(w_base > 0.0f);
    gmp_base_assert(pole_pairs > 0);
    gmp_base_assert(turns_ratio > 0.0f);

    // 1. Primary (Stator) Bases
    pu->V_s_base = v_base;
    pu->I_s_base = i_base;
    pu->W_base = w_base;

    pu->Z_s_base = v_base / i_base;
    pu->L_s_base = pu->Z_s_base / w_base;
    pu->Flux_s_base = v_base / w_base;

    // 2. Turns Ratio
    pu->turns_ratio = turns_ratio;

    // 3. Secondary (Rotor) Bases Derivation
    // Based on ideal transformer theory mapping:
    // Vr = Vs / k, Ir = Is * k
    pu->V_r_base = pu->V_s_base / turns_ratio;
    pu->I_r_base = pu->I_s_base * turns_ratio;

    // Zr = Vr / Ir = (Vs/k) / (Is*k) = Zs / k^2
    parameter_gt k_sq = turns_ratio * turns_ratio;
    pu->Z_r_base = pu->Z_s_base / k_sq;
    pu->L_r_base = pu->L_s_base / k_sq;
    pu->Flux_r_base = pu->Flux_s_base / turns_ratio;

    // 4. Mechanical Bases
    pu->P_base = 1.5f * v_base * i_base;
    parameter_gt w_mech_base = w_base / (parameter_gt)pole_pairs;
    pu->T_base = pu->P_base / w_mech_base;
}
