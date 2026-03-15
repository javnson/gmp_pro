
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
void ctl_autotune_and_init_im_ifoc(im_ifoc_ctrl_t* mc, const im_ifoc_init_t* init)
{
    // Protect against zero division
    parameter_gt fs_safe = (init->fs > 1e-6f) ? init->fs : 10000.0f;
    parameter_gt lr_safe = (init->mtr_Lr > 1e-9f) ? init->mtr_Lr : 1.0f;

    // 1. IM Physical Equivalents
    // Transient Inductance: sigma * Ls = Ls - (Lm^2 / Lr)
    parameter_gt sigma_ls = init->mtr_Ls - (init->mtr_Lm * init->mtr_Lm) / lr_safe;

    // Equivalent Resistance: Rs + Rr*(Lm/Lr)^2
    parameter_gt req = init->mtr_Rs + init->mtr_Rr * (init->mtr_Lm * init->mtr_Lm) / (lr_safe * lr_safe);

    // 2. PI Gains Calculation
    parameter_gt bw_rad = CTL_PARAM_CONST_2PI * init->current_loop_bw;
    parameter_gt kp_phy = sigma_ls * bw_rad;
    parameter_gt ki_phy = req * bw_rad;

    // Convert to PU (I_base / V_base)
    parameter_gt scale_kp = init->i_base / init->v_base;
    parameter_gt kp_pu = kp_phy * scale_kp;
    parameter_gt ki_pu = ki_phy * scale_kp;

    // 3. Slip Angle Integration Constant (PU)
    // omega_slip = (Rr/Lr) * (Iq/Id). Delta_angle_pu = omega_slip / (2*PI*fs)
    parameter_gt coef_slip = init->mtr_Rr / (lr_safe * CTL_PARAM_CONST_2PI * fs_safe);

    // Rotor time constant LPF for i_mu (dt / tau_r)
    parameter_gt coef_imu = init->mtr_Rr / (lr_safe * fs_safe);

    // 4. Decoupling Feedforward Constants
    parameter_gt omega_base_elec = (init->spd_base * 1000.0f) * CTL_PARAM_CONST_PI / 30.0f * init->pole_pairs;
    parameter_gt scale_fac = omega_base_elec * init->i_base / init->v_base;

    parameter_gt coef_dec_lsig = sigma_ls * scale_fac;
    parameter_gt coef_dec_emf = ((init->mtr_Lm * init->mtr_Lm) / lr_safe) * scale_fac;

    // ------------------------------------------------------------------------
    // Apply Configurations to Struct
    // ------------------------------------------------------------------------
    mc->coef_slip_calc = float2ctrl(coef_slip);
    mc->coef_imu_lpf = float2ctrl(coef_imu);
    mc->coef_dec_lsigma = float2ctrl(coef_dec_lsig);
    mc->coef_dec_backemf = float2ctrl(coef_dec_emf);

    mc->max_vs_mag = float2ctrl((init->v_phase_limit * 1.4142f) / init->v_base);
    mc->max_dcbus_voltage = float2ctrl(init->v_bus / init->v_base);

    // Init PID controllers
    ctl_init_pid(&mc->idq_ctrl[phase_d], kp_pu, ki_pu, 0.0f, fs_safe);
    ctl_init_pid(&mc->idq_ctrl[phase_q], kp_pu, ki_pu, 0.0f, fs_safe);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_d], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);
    ctl_set_pid_int_limit(&mc->idq_ctrl[phase_q], mc->max_vs_mag, -mc->max_vs_mag);

    // Init Filters
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_U], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_V], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_iuvw[phase_W], fs_safe, fs_safe / 3.0f);
    ctl_init_filter_iir1_lpf(&mc->filter_udc, fs_safe, fs_safe / 5.0f);

    // Clear States
    mc->slip_angle_pu = float2ctrl(0.0f);
    mc->sync_angle_pu = float2ctrl(0.0f);
    mc->i_mu_pu = float2ctrl(0.01f); // Avoid starting exactly at 0
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
