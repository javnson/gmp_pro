

#include <gmp_core.h>

#include <ctl/component/motor_control/current_loop/pmsm_dbptc.h>

/**
 * @brief Initializes the DB-PTC model, collapsing all physical equations into pure PU constants.
 * * @details
 * This function performs the heavy lifting of mapping the continuous time equations 
 * into a discrete, strictly Per-Unit bounded space.
 * * Physical Forward Euler:
 * i_d(k+1) = (1 - Rs*Ts/Ld)*i_d(k) + (Ts/Ld)*We*Lq*i_q(k) + (Ts/Ld)*u_d(k)
 * * Per-Unit Base Mapping:
 * I_pu = I / I_base, U_pu = U / V_base, We_pu = We / W_base
 */
void ctl_init_pmsm_dbptc(pmsm_dbptc_ctrl_t* mc, const pmsm_dbptc_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt Ts = 1.0f / fs_safe;

    parameter_gt omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;

    // 1. Matrix A Constants (Self-decay)
    parameter_gt a11 = 1.0f - (init->mtr_Rs * Ts) / init->mtr_Ld;
    parameter_gt a22 = 1.0f - (init->mtr_Rs * Ts) / init->mtr_Lq;

    // 2. Matrix A Cross-Coupling Constants (Absorbing W_base)
    // Kw_d * We_pu = (Ts/Ld) * (We_pu * W_base) * Lq
    parameter_gt kw_d = (Ts * omega_base_elec * init->mtr_Lq) / init->mtr_Ld;
    parameter_gt kw_q = (Ts * omega_base_elec * init->mtr_Ld) / init->mtr_Lq;

    // 3. Matrix B Constants (Control Gain mapping U_pu to I_pu)
    // i_pu = ... + (Ts/Ld) * (U_pu * V_base) / I_base
    parameter_gt b11 = (Ts * init->v_base) / (init->mtr_Ld * init->i_base);
    parameter_gt b22 = (Ts * init->v_base) / (init->mtr_Lq * init->i_base);

    // 4. Matrix E Constant (Back-EMF Disturbance)
    // emf_pu = (Ts/Lq) * (We_pu * W_base) * Flux / I_base
    parameter_gt k_emf = (Ts * omega_base_elec * init->mtr_Flux) / (init->mtr_Lq * init->i_base);

    // --- Transfer to Fixed-Point Struct ---
    mc->A11 = float2ctrl(a11);
    mc->A22 = float2ctrl(a22);

    mc->Kw_d = float2ctrl(kw_d);
    mc->Kw_q = float2ctrl(kw_q);

    mc->B11 = float2ctrl(b11);
    mc->B22 = float2ctrl(b22);

    // Pre-calculate inverse to completely eliminate division in ISR
    mc->inv_B11 = float2ctrl(1.0f / b11);
    mc->inv_B22 = float2ctrl(1.0f / b22);

    mc->K_emf = float2ctrl(k_emf);

    // --- Protection Configuration ---
    // Calculate circular max voltage limit (e.g. Vbus / sqrt(3) / Vbase)
    mc->v_max_pu = float2ctrl((init->v_phase_limit * 1.4142f) / init->v_base);

    // Store I_max_sq to avoid square root during limit check when not saturated
    mc->i_max_pu_sq = float2ctrl(init->i_max_pu * init->i_max_pu);

    // Clear States
    mc->ud_prev = float2ctrl(0.0f);
    mc->uq_prev = float2ctrl(0.0f);
    ctl_vector2_clear(&mc->vdq_out);
    ctl_vector2_clear(&mc->idq_ref);

    mc->flag_enable = 0;
}
