
#include <gmp_core.h>

#include <ctl/component/motor_control/consultant/nameplate_consultant.h>


// Math conversion constants
#define CONST_SQRT2       1.41421356f
#define CONST_SQRT3       1.73205081f
#define CONST_RPM_TO_RADS 0.104719755f // (2 * PI) / 60

/**
 * @brief Parses the PMSM nameplate and generates the physical and PU models.
 */
void ctl_nameplate_build_pmsm(const ctl_nameplate_pmsm_t* np, ctl_consultant_pmsm_t* out_motor,
                              ctl_consultant_pu_pmsm_t* out_pu)
{
    // 1. Resolve Flux Linkage (Weber)
    parameter_gt resolved_flux = np->flux_weber;

    // If flux is 0 or not provided, try to calculate from Ke or Kt
    if (resolved_flux <= 1e-6f)
    {
        if (np->ke_vrms_krpm > 1e-6f)
        {
            resolved_flux = ctl_consultant_pmsm_flux_from_Ke(np->pole_pairs, np->ke_vrms_krpm);
        }
        else if (np->kt_nm_arms > 1e-6f)
        {
            resolved_flux = ctl_consultant_pmsm_flux_from_Kt_Arms(np->pole_pairs, np->kt_nm_arms);
        }
    }

    // 2. Build the Physical Motor Model
    ctl_consultant_pmsm_init(out_motor, np->pole_pairs, np->rs_ohm, np->ld_henry, np->lq_henry, resolved_flux);

    // 3. Convert RMS / L-L Nameplate ratings to Peak Phase Bases for PU
    // V_base = V_rms(L-L) * sqrt(2) / sqrt(3) -> Amplitude invariant phase peak
    parameter_gt v_base_peak = np->rated_voltage_vrms * CONST_SQRT2 / CONST_SQRT3;

    // I_base = I_rms * sqrt(2) -> Phase peak
    parameter_gt i_base_peak = np->rated_current_arms * CONST_SQRT2;

    // W_base = RPM * (2PI/60) * pole_pairs -> Electrical angular velocity
    parameter_gt w_base_elec = np->rated_speed_rpm * CONST_RPM_TO_RADS * np->pole_pairs;

    // 4. Build the PU Model
    ctl_consultant_pu_pmsm_init(out_pu, v_base_peak, i_base_peak, w_base_elec, np->pole_pairs);
}

/**
 * @brief Parses the IM nameplate and generates the physical and PU models.
 */
void ctl_nameplate_build_im(const ctl_nameplate_im_t* np, ctl_consultant_im_t* out_motor,
                            ctl_consultant_pu_im_t* out_pu)
{
    // 1. Automatic Pole Pair Deduction from rated Hz and RPM
    // Sync Speed Ns = 60 * f / p => p = 60 * f / Ns
    // Since N_rated is slightly less than Ns due to slip, we round to nearest integer.
    parameter_gt p_float = (60.0f * np->rated_freq_hz) / np->rated_speed_rpm;

    // Simple rounding logic to nearest integer pole pairs (e.g., 1, 2, 3...)
    uint32_t deduced_pole_pairs = (uint32_t)(p_float + 0.2f);
    if (deduced_pole_pairs < 1)
        deduced_pole_pairs = 1;

    // 2. Build the Physical Motor Model
    ctl_consultant_im_init(out_motor, deduced_pole_pairs, np->rs_ohm, np->rr_ohm, np->ls_henry, np->lr_henry,
                           np->lm_henry);

    // 3. Convert RMS / L-L Nameplate ratings to Peak Phase Bases for PU
    parameter_gt v_base_peak = np->rated_voltage_vrms * CONST_SQRT2 / CONST_SQRT3;
    parameter_gt i_base_peak = np->rated_current_arms * CONST_SQRT2;

    // W_base uses synchronous speed (derived from rated frequency)
    // w_base_elec = 2 * PI * f
    parameter_gt w_base_elec = np->rated_freq_hz * CTL_PARAM_CONST_2PI;

    // 4. Build the PU Model
    // Assuming equivalent circuit parameters (Rs, Rr, Ls, Lr) are already referred
    // to the stator by the auto-tuning process, so turns_ratio = 1.0f.
    ctl_consultant_pu_im_init(out_pu, v_base_peak, i_base_peak, w_base_elec, deduced_pole_pairs, 1.0f);
}

/**
 * @brief Utility: Calculates nominal IM slip frequency from nameplate.
 */
parameter_gt ctl_nameplate_im_calc_rated_slip_rads(const ctl_nameplate_im_t* np)
{
    // Deduced pole pairs
    parameter_gt p_float = (60.0f * np->rated_freq_hz) / np->rated_speed_rpm;
    uint32_t pp = (uint32_t)(p_float + 0.2f);
    if (pp < 1)
        pp = 1;

    // Synchronous mechanical speed (rpm) = 60 * f / p
    parameter_gt sync_rpm = (60.0f * np->rated_freq_hz) / (parameter_gt)pp;

    // Slip rpm = Sync_rpm - Rated_rpm
    parameter_gt slip_rpm = sync_rpm - np->rated_speed_rpm;

    // Convert slip mechanical rpm to electrical rad/s
    return slip_rpm * CONST_RPM_TO_RADS * (parameter_gt)pp;
}
