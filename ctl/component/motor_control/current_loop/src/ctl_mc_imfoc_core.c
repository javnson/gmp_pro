
#include <gmp_core.h>

#include <ctl/component/motor_control/current_loop/imfoc_core.h>


/**
 * @brief Auto-tunes and initializes the IM IFOC controller.
 * @details 
 * **1. Transient Inductance (The Plant):**
 * For an IM, the current loop sees the transient inductance:
 * $\sigma L_s = L_s - \frac{L_m^2}{L_r}$
 * * **2. PI Tuning:**
 * $K_p = \sigma L_s \cdot \text{BW}$
 * $K_i = R_{eq} \cdot \text{BW}$ (where $R_{eq} = R_s + R_r (\frac{L_m}{L_r})^2$)
 * * **3. PU Slip Constant:**
 * $\omega_{slip} = \frac{R_r}{L_r} \frac{I_q}{I_d}$. 
 * We absorb the sampling time to calculate the per-tick angle increment:
 * $K_{slip\_calc} = \frac{R_r}{L_r \cdot 2\pi \cdot f_s}$
 */
/**
 * @brief Auto-tunes and initializes the IM IFOC controller using raw parameters.
 */
void ctl_autotune_and_init_im_ifoc(im_ifoc_ctrl_t* mc, const im_ifoc_init_t* init)
{
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt lr_safe = (init->mtr_Lr > 1e-9f) ? init->mtr_Lr : 1.0f;

    // 1. IM Physical Equivalents
    parameter_gt sigma_ls = init->mtr_Ls - (init->mtr_Lm * init->mtr_Lm) / lr_safe;
    parameter_gt req = init->mtr_Rs + init->mtr_Rr * (init->mtr_Lm * init->mtr_Lm) / (lr_safe * lr_safe);

    // 2. PI Gains Calculation (Plant: sigma*Ls)
    parameter_gt bw_rad = CTL_PARAM_CONST_2PI * init->current_loop_bw;
    parameter_gt scale_kp = init->i_base / init->v_base;

    parameter_gt kp_pu = (sigma_ls * bw_rad) * scale_kp;
    parameter_gt ki_pu = (req * bw_rad) * scale_kp;

    // 3. Initialize Independent Slip & Position Calculator
    ctl_im_pos_calc_init_t calc_init;
    parameter_gt tau_r = lr_safe / init->mtr_Rr;

    // Spd base is KRPM -> rad/s
    parameter_gt w_base = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;

    calc_init.sf_lpf_kr = (1.0f / fs_safe) / tau_r;
    calc_init.sf_slip_const = 1.0f / (tau_r * w_base);
    calc_init.sf_mech_to_elec = 1.0f; // Assumes mechanical speed is already normalized to W_base
    calc_init.sf_w_to_angle = (w_base / fs_safe) / CTL_PARAM_CONST_2PI;
    calc_init.i_md_min_limit_pu = 0.05f;

    ctl_init_im_pos_calc(&mc->pos_calc, &calc_init);

    // 4. Decoupling Feedforward Constants (PU Space)
    parameter_gt scale_fac = w_base * init->i_base / init->v_base;
    mc->sf_dec_lsigma = float2ctrl(sigma_ls * scale_fac);
    mc->sf_dec_backemf = float2ctrl(((init->mtr_Lm * init->mtr_Lm) / lr_safe) * scale_fac);

    // 5. Limits & PID Init
    mc->max_vs_mag = float2ctrl((init->v_phase_limit * 1.4142f) / init->v_base);
    mc->max_dcbus_voltage = float2ctrl(init->v_bus / init->v_base);

    ctl_init_pid(&mc->idq_ctrl[phase_d], float2ctrl(kp_pu), float2ctrl(ki_pu), float2ctrl(0.0f), fs_safe);
    ctl_init_pid(&mc->idq_ctrl[phase_q], float2ctrl(kp_pu), float2ctrl(ki_pu), float2ctrl(0.0f), fs_safe);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);

    // 6. Init Filters
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_U], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_V], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_W], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_udc, fs_safe, fs_safe / 5.0f);

    // 7. Clear States
    mc->isr_tick = 0;
    ctl_vector2_clear(&mc->idq_ref);
    ctl_vector2_clear(&mc->vdq_ctrl_out);
    ctl_vector2_clear(&mc->vdq_decouple);
    ctl_vector3_clear(&mc->vdq_out);
    ctl_vector2_clear(&mc->vdq_out_sat);

    mc->flag_enable_current_ctrl = 0;
    mc->flag_enable_decouple = 1;
    mc->flag_enable_bus_compensation = 1;
}

/**
 * @brief Advanced initialization function utilizing the Consultant models.
 */
void ctl_autotune_and_init_im_ifoc_consultant(im_ifoc_ctrl_t* mc, const ctl_consultant_im_t* motor,
                                              const ctl_consultant_pu_im_t* pu, parameter_gt fs, parameter_gt v_bus,
                                              parameter_gt v_phase_limit, parameter_gt current_loop_bw)
{
    parameter_gt fs_safe = (fs > 1e-6f) ? fs : 10000.0f;

    // 1. Delegate Slip & Angle Calculus Initialization to the Consultant
    ctl_init_im_pos_calc_consultant(&mc->pos_calc, motor, pu, fs_safe);

    // 2. PI Gains Auto-tuning (Leveraging derived R_eq and sigma_Ls from the Consultant)
    parameter_gt bw_rad = CTL_PARAM_CONST_2PI * current_loop_bw;

    // Kp_pu = (sigma_Ls * BW) * (I_base / V_base) = (sigma_Ls / L_base) * (BW / W_base)
    parameter_gt kp_pu = (motor->sigma_Ls / pu->L_s_base) * (bw_rad / pu->W_base);

    // Ki_pu = (R_eq * BW) * (I_base / V_base) = (R_eq / Z_s_base) * (BW / W_base) * W_base
    // Simplified: (R_eq / Z_s_base) * bw_rad. Need to multiply by Ts for T-mode integration
    parameter_gt ki_pu = (motor->R_eq / pu->Z_s_base) * bw_rad;

    // 3. Decoupling Constants
    // In PU: Decoupling = omega_e_pu * (sigma_Ls / L_base) * Iq_pu
    mc->sf_dec_lsigma = float2ctrl(motor->sigma_Ls / pu->L_s_base);

    // Decoupling EMF = omega_e_pu * ((Lm^2/Lr) / L_base) * Imd_pu
    mc->sf_dec_backemf = float2ctrl(motor->Lm_sq_over_Lr / pu->L_s_base);

    // 4. Limits & Controllers Setup
    mc->max_vs_mag = float2ctrl((v_phase_limit * 1.4142f) / pu->V_s_base);
    mc->max_dcbus_voltage = float2ctrl(v_bus / pu->V_s_base);

    ctl_init_pid(&mc->idq_ctrl[phase_d], float2ctrl(kp_pu), float2ctrl(ki_pu), float2ctrl(0.0f), fs_safe);
    ctl_init_pid(&mc->idq_ctrl[phase_q], float2ctrl(kp_pu), float2ctrl(ki_pu), float2ctrl(0.0f), fs_safe);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);

    // 5. Init Filters
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_U], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_V], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_W], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_udc, fs_safe, fs_safe / 5.0f);

    // 6. Clear States
    mc->isr_tick = 0;
    ctl_vector2_clear(&mc->idq_ref);
    ctl_vector2_clear(&mc->vdq_ctrl_out);
    ctl_vector2_clear(&mc->vdq_decouple);
    ctl_vector3_clear(&mc->vdq_out);
    ctl_vector2_clear(&mc->vdq_out_sat);

    mc->flag_enable_current_ctrl = 0;
    mc->flag_enable_decouple = 1;
    mc->flag_enable_bus_compensation = 1;
}
