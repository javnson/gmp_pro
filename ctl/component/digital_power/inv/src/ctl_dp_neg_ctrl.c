
#include <gmp_core.h>



//////////////////////////////////////////////////////////////////////////
// Three Phase Converter negative sequence controller
//////////////////////////////////////////////////////////////////////////

#include <ctl/component/digital_power/inv/inv_neg_ctrl.h>
#include <ctl/component/digital_power/inv/gfl_core.h>

/**
 * @brief Auto-tuning Negative Sequence Controller parameters.
 */
void ctl_auto_tuning_neg_inv(inv_neg_ctrl_init_t* neg_init, const gfl_inv_ctrl_init_t* _gfl_init)
{
    gmp_base_assert(neg_init);
    gmp_base_assert(_gfl_init);

    // 1. Copy Basic System Parameters
    neg_init->fs = _gfl_init->fs;
    neg_init->freq_base = _gfl_init->freq_base;

    // 2. Tuning Separation Filter (Biquad LPF)
    // Goal: Attenuate 2*freq_base (100Hz) ripple significantly.
    // Strategy: Set fc at approx freq_base / 2.5 (e.g., 20Hz for 50Hz grid).
    // This offers a trade-off between ripple attenuation and delay.
    neg_init->seq_filter_fc = _gfl_init->freq_base / 2.5f;
    //neg_init->seq_filter_q = 0.707f; // Butterworth response
    neg_init->seq_filter_q = 5.0f; // Butterworth response

    // 3. Tuning Current Loop (Inner Loop)
    // Constraint: Bandwidth must be lower than filter fc to maintain stability margin.
    // Strategy: Set BW at filter_fc / 2, it's too small.
    //parameter_gt neg_current_bw = neg_init->seq_filter_fc / 2.0f;

    // Set BW at fs/50
    parameter_gt neg_current_bw = neg_init->fs / 200.0f;

    // Set BW at 50Hz
    //parameter_gt neg_current_bw = 50.0f;

    // Calculate PI gains based on Plant Model: V = L * di/dt
    // Kp_pu = (2*pi*BW * L) * (I_base / V_base)
    // Note: This Kp is calculated conceptually here, but physically in update_coeff.
    // We store the tuning target implicitly via the update function logic,
    // but here we define the limits and integration speeds.

    // For auto-tuning function, we usually just set reasonable defaults if the struct
    // expects raw Kp/Ki. However, looking at ctl_update_gfl_inv_coeff, it calculates Kp inside.
    // BUT inv_neg_ctrl_init_t stores raw Kp/Ki. So we calculate them here.

    parameter_gt L_val = _gfl_init->grid_filter_L;
    parameter_gt kp_c_si = CTL_PARAM_CONST_2PI * neg_current_bw * L_val;

    // Convert to P.U.
    neg_init->kp_current = kp_c_si * _gfl_init->i_base / _gfl_init->v_base;

    // Ki: Place zero at low frequency (e.g., BW / 10) to eliminate steady state error
    // Ki = Kp * 2*pi * f_zero
    neg_init->ki_current = CTL_PARAM_CONST_2PI * (neg_current_bw / 10.0f);

    // Default Limits
    neg_init->limit_current_out = 0.8f; // 0.8 p.u. voltage compensation limit

    // 4. Tuning Voltage Loop (Outer Loop) - Optional
    // Constraint: Must be significantly slower than current loop.
    // Strategy: Set BW at Current_BW / 5.
    parameter_gt neg_voltage_bw = neg_current_bw / 5.0f;

    // Plant Model: I = C * dv/dt (Capacitor dominates)
    // If no C (L filter), voltage control is invalid, but we calculate anyway.
    parameter_gt C_val = (_gfl_init->grid_filter_C > 1e-9f) ? _gfl_init->grid_filter_C : 1e-5f; // prevent zero

    parameter_gt kp_v_si = CTL_PARAM_CONST_2PI * neg_voltage_bw * C_val;

    // Convert to P.U. (Note: Output is Current Ref, Input is Voltage)
    // Gain units: A/V. P.U. conversion factor is V_base / I_base.
    neg_init->kp_voltage = kp_v_si * _gfl_init->v_base / _gfl_init->i_base;

    // Ki
    neg_init->ki_voltage = CTL_PARAM_CONST_2PI * (neg_voltage_bw / 5.0f);

    neg_init->limit_voltage_out = 0.5f; // 0.5 p.u. current injection limit
}

/**
 * @brief Update Negative Sequence Controller coefficients.
 */
void ctl_update_neg_inv_coeff(inv_neg_ctrl_t* neg, const inv_neg_ctrl_init_t* neg_init)
{
    int i;

    gmp_base_assert(neg);
    gmp_base_assert(neg_init);
    // _gfl_init is used here if we needed L/C again, but since neg_init already
    // contains the calculated Kp/Ki, we rely on neg_init.

    // 1. Init Separation Filters (Biquad LPF)
    // These filter the raw dq currents/voltages to extract DC negative sequence.
    for (i = 0; i < 2; ++i)
    {
        //ctl_init_biquad_lpf(&neg->filter_idqn[i], neg_init->fs, neg_init->seq_filter_fc, neg_init->seq_filter_q);
        //ctl_init_biquad_lpf(&neg->filter_vdqn[i], neg_init->fs, neg_init->seq_filter_fc, neg_init->seq_filter_q);
        ctl_init_biquad_notch(&neg->filter_idqn[i], neg_init->fs, neg_init->freq_base * 2, neg_init->seq_filter_q);
        ctl_init_biquad_notch(&neg->filter_vdqn[i], neg_init->fs, neg_init->freq_base * 2, neg_init->seq_filter_q);
    }

    // 2. Init Current Loop PIDs
    // D-axis and Q-axis share the same parameters
    ctl_init_pid(&neg->pid_idqn[phase_d], neg_init->kp_current, neg_init->ki_current, 0, neg_init->fs);
    ctl_init_pid(&neg->pid_idqn[phase_q], neg_init->kp_current, neg_init->ki_current, 0, neg_init->fs);

    ctl_set_pid_limit(&neg->pid_idqn[phase_d], neg_init->limit_current_out, -neg_init->limit_current_out);
    ctl_set_pid_limit(&neg->pid_idqn[phase_q], neg_init->limit_current_out, -neg_init->limit_current_out);

    // 3. Init Voltage Loop PIDs
    ctl_init_pid(&neg->pid_vdqn[phase_d], neg_init->kp_voltage, neg_init->ki_voltage, 0, neg_init->fs);
    ctl_init_pid(&neg->pid_vdqn[phase_q], neg_init->kp_voltage, neg_init->ki_voltage, 0, neg_init->fs);

    ctl_set_pid_limit(&neg->pid_vdqn[phase_d], neg_init->limit_voltage_out, -neg_init->limit_voltage_out);
    ctl_set_pid_limit(&neg->pid_vdqn[phase_q], neg_init->limit_voltage_out, -neg_init->limit_voltage_out);
}

/**
 * @brief Initialize Negative Sequence Controller.
 */
void ctl_init_neg_inv(inv_neg_ctrl_t* neg, const inv_neg_ctrl_init_t* neg_init)
{
    // Apply coefficients
    ctl_update_neg_inv_coeff(neg, neg_init);

    // Clear states
    ctl_clear_neg_inv(neg);

    // Default disable flags (Safety first)
    neg->flag_enable_negative_current_ctrl = 0;
    neg->flag_enable_negative_voltage_ctrl = 0;

    // Default setpoints
    ctl_vector2_clear(&neg->idqn_set);
    ctl_vector2_clear(&neg->vdqn_set);
}
