/**
 * @file three_phase_GFM.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Header-only library for a preset three-phase DC/AC grid forming inverter controller.
 * @version 1.0
 * @date 2026-01-11
 *
 * @copyright Copyright GMP(c) 2025
 */

/** 
 * @defgroup CTL_TOPOLOGY_INV_H_API Three-Phase GFM Inverter Topology API (Header)
 * @{
 * @ingroup CTL_DP_LIB
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */

#include <ctl/component/digital_power/three_phase/three_phase_GFL.h>

#ifndef _FILE_THREE_PHASE_GFM_
#define _FILE_THREE_PHASE_GFM_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Grid-Forming Droop Controller (Outer Loop).
 * @details 
 * Generates:
 * 1. Angle (Theta) -> Drives the Park transforms.
 * 2. Current Reference (Idq_ref) -> Drives the inner Current Loop.
 */
typedef struct _tag_gfm_droop_ctrl
{
    //
    // --- Input Ports (Pointers) ---
    //
    ctl_vector2_t* vdq_meas; //!< RO: Measured Capacitor Voltage (dq).
    ctl_vector2_t* idq_meas; //!< RO: Measured Output/Grid Current (dq).
    ctrl_gt* pll_angle;      //!< RO: Pointer to GFL PLL angle for pre-sync tracking.

    //
    // --- Output Ports ---
    //
    ctl_vector2_t idq_ref_out; //!< RW: Output Current Reference (Send to Inner Loop).
    ctrl_gt theta_out;         //!< RW: Generated Phase Angle (Send to Park Transforms).
    ctl_vector2_t phasor_out;  //!< RW: Generated Phasor {cos, sin}.

    //
    // --- Internal States & Calculations ---
    //

    // 1. Power Calculation & Droop
    ctl_vector2_t pq_inst;       //!< Instantaneous P (dat[0]) & Q (dat[1]).
    ctl_filter_IIR1_t lpf_pq[2]; //!< Low Pass Filter for P and Q.
    ctl_vector2_t pq_avg;        //!< Filtered P (dat[0]) & Q (dat[1]).

    ctrl_gt droop_freq_ref;  //!< Frequency Reference (output of P-f droop).
    ctrl_gt droop_v_mag_ref; //!< Voltage Magnitude Reference (output of Q-V droop).

    // 2. Virtual Impedance
    ctl_vector2_t vdq_vir_imp;  //!< Voltage drop across virtual impedance.
    ctl_vector2_t vdq_virt_ref; //!< Final Voltage Ref (V_nom - V_droop - V_vir).

    // 3. Voltage Control Loop
    ctl_pid_t pid_vdq[2];          //!< Voltage D-axis and Q-axis PI.
    ctl_vector2_t vdq_ff_decouple; //!< Decoupling terms (omega * C * v).

    // Safety
    ctl_saturation_t sat_id; //!< Saturation for Id ref.
    ctl_saturation_t sat_iq; //!< Saturation for Iq ref.

    //
    // --- Setpoints ---
    //
    ctl_vector2_t pq_set; //!< P setpoint (dat[0]), Q setpoint (dat[1]).
    ctrl_gt v_nom;        //!< Nominal Voltage Amplitude (V0).
    ctrl_gt f_nom;        //!< Nominal Frequency (f0).

    //
    // --- Coefficients (Calculated during Init) ---
    //
    ctrl_gt coef_vdq_ff;   //!< Decoupling coeff (omega_nom * C).
    ctrl_gt coef_vir_res;  //!< Virtual Resistance.
    ctrl_gt coef_vir_ind;  //!< Virtual Inductance (omega_nom * L_vir).
    ctrl_gt coef_droop[2]; //!< [0]: Kp (P-f), [1]: Kq (Q-V).
    ctrl_gt Ts;            //!< Sampling period.
    ctrl_gt omega_nom;     //!< Nominal angular frequency.

    //
    // --- Flags ---
    //
    fast_gt flag_enable_system;       //!< 1: Run Droop & Integrate Angle. 0: Track PLL & Reset Integrators.
    fast_gt flag_enable_voltage_loop; //!< Enable Voltage PI controller.
    fast_gt flag_enable_voltage_ff;   //!< Enable Voltage decoupling feedforward.
    fast_gt flag_enable_virtual_imp;  //!< Enable Virtual impedance.
    fast_gt flag_enable_droop;        //!< Enable Voltage/Power Droop control.

} gfm_droop_ctrl_t;

// Clear GFM droop controller
GMP_STATIC_INLINE void ctl_clear_gfm_droop_ctrl(gfm_droop_ctrl_t* gfm)
{
    ctl_vector2_clear(&gfm->pq_inst);
    ctl_vector2_clear(&gfm->pq_avg);

    ctl_clear_filter_iir1(&gfm->lpf_pq[0]);
    ctl_clear_filter_iir1(&gfm->lpf_pq[1]);

    ctl_clear_pid(&gfm->pid_vdq[phase_d]);
    ctl_clear_pid(&gfm->pid_vdq[phase_q]);

    gfm->theta_out = 0.0f;
}

/**
 * @brief Initialization parameters for GFM Droop Controller.
 */
typedef struct _tag_gfm_droop_init
{
    parameter_gt fs;        //!< Sampling frequency.
    parameter_gt freq_base; //!< Nominal Grid Frequency (e.g., 50Hz).
    parameter_gt v_base;    //!< Nominal Voltage Amplitude (Peak Phase Voltage).
    parameter_gt s_base;    //!< System rated power (VA).
    parameter_gt i_base;    //!< System rated current (A), derived usually.

    // --- Droop Parameters (P.U. or SI) ---
    // Recommendation: Input e.g. 0.05 (5%) for 100% load.
    parameter_gt droop_p_percent; //!< Frequency droop percentage at rated P (e.g., 0.01 for 1%).
    parameter_gt droop_q_percent; //!< Voltage droop percentage at rated Q (e.g., 0.05 for 5%).
    parameter_gt power_lpf_fc;    //!< Cutoff frequency for Power calculation LPF (Hz).

    // --- Virtual Impedance ---
    parameter_gt rv_virtual; //!< Virtual Resistance (Ohms).
    parameter_gt lv_virtual; //!< Virtual Inductance (Henrys).

    // --- Voltage Loop Parameters ---
    parameter_gt voltage_loop_bw; //!< Voltage loop bandwidth (Hz).
    parameter_gt cf_capacitance;  //!< Filter Capacitor value (F).
    parameter_gt current_limit;   //!< Output current reference limit (p.u.).

} gfm_droop_init_t;

// init gfm init parameters by GFL init struct.
void ctl_auto_tuning_gfm_droop_ctrl(gfm_droop_init_t* gfm_init, gfl_inv_ctrl_init_t* gfl)
{
    gmp_base_assert(gfm_init);
    gmp_base_assert(gfl_init);

    // 1. Copy Basic System Params from GFL
    gfm_init->fs = gfl_init->fs;
    gfm_init->freq_base = gfl_init->freq_base;
    gfm_init->v_base = gfl_init->v_base;
    gfm_init->i_base = gfl_init->i_base;

    // Calculate S_base (VA)
    // S_base = V_base * I_base
    // Let's assume P.U. consistency:
    gfm_init->s_base = gfl_init->v_base * gfl_init->i_base;

    // 2. Filter Capacitor
    gfm_init->cf_capacitance = gfl_init->grid_filter_C;

    // 3. Tuning Voltage Loop
    // Bandwidth: Typically 1/5 to 1/10 of Current Loop Bandwidth
    gfm_init->voltage_loop_bw = gfl_init->current_loop_bw / 5.0f;

    // Kp Calculation: C * 2*pi*BW
    // Physical Kp = C * 2*pi*BW
    // P.U. Kp = Physical_Kp * (V_base / I_base)
    // (Input is Voltage err, Output is Current ref)
    parameter_gt kp_phys = gfm_init->cf_capacitance * CTL_PARAM_CONST_2PI * gfm_init->voltage_loop_bw;
    gfm_init->kp_voltage = kp_phys * (gfm_init->v_base / gfm_init->i_base);

    // Ki Calculation: Place zero to cancel pole or set suitable stiffness
    // Typically zero at BW/10 or BW/5
    gfm_init->ki_voltage = gfm_init->kp_voltage * CTL_PARAM_CONST_2PI * (gfm_init->voltage_loop_bw / 5.0f);

    // 4. Default Droop Settings
    gfm_init->droop_p_percent = 0.01f; // 1% freq drop at rated load
    gfm_init->droop_q_percent = 0.05f; // 5% volt drop at rated load
    gfm_init->power_lpf_fc = 2.0f;     // 2Hz Power Filter (Simulated Inertia)

    // 5. Default Virtual Impedance (Low Voltage Microgrid usually Resistive)
    gfm_init->rv_virtual = 0.0f;
    gfm_init->lv_virtual = 0.0f;

    gfm_init->current_limit = 1.2f; // 1.2 p.u. current limit
}

void ctl_init_gfm_droop(gfm_droop_ctrl_t* gfm, const gfm_droop_init_t* init)
{
    gmp_base_assert(gfm);
    gmp_base_assert(init);

    // Set Setpoints Defaults
    gfm->f_nom = init->freq_base; // 50Hz
    gfm->v_nom = 1.0f;            // 1.0 p.u. voltage
    gfm->pq_set.dat[0] = 0.0f;    // 0 Power setpoint
    gfm->pq_set.dat[1] = 0.0f;

    gfm->Ts = 1.0f / init->fs;
    gfm->omega_nom = CTL_PARAM_CONST_2PI * init->freq_base;

    // 1. Calculate Droop Coeffs
    // Kp_droop = (Delta_f_max) / S_base
    // Delta_f_max = f_nom * droop_percent
    // If working in P.U., S_base = 1.0.
    // Let's assume P/Q inputs to struct are in P.U.
    gfm->coef_droop[0] = (init->droop_p_percent);
    gfm->coef_droop[1] = (init->droop_q_percent);

    // 2. Virtual Impedance Coeffs
    // V_drop = I * Z.
    // Need to convert physical Ohms/Henrys to P.U. impedance?
    // Or if I is P.U., Z should be P.U.
    // Z_base = V_base / I_base.
    parameter_gt z_base = init->v_base / init->i_base;
    gfm->coef_vir_res = init->rv_virtual / z_base;
    gfm->coef_vir_ind = (init->lv_virtual * gfm->omega_nom) / z_base;

    // 3. Feedforward Coeff
    // omega * C * V_base / I_base (to convert V p.u. to I p.u.)
    // V_pu * V_base = V_volts. Current = w * C * V_volts. I_pu = Current / I_base.
    // Coeff = w * C * V_base / I_base = w * C * Z_base.
    gfm->coef_vdq_ff = gfm->omega_nom * init->cf_capacitance * z_base;

    // 4. Init Controllers
    ctl_init_filter_iir1_lpf(&gfm->lpf_pq[0], init->fs, init->power_lpf_fc);
    ctl_init_filter_iir1_lpf(&gfm->lpf_pq[1], init->fs, init->power_lpf_fc);

    ctl_init_pid(&gfm->pid_vdq[phase_d], init->kp_voltage, init->ki_voltage, 0, init->fs);
    ctl_init_pid(&gfm->pid_vdq[phase_q], init->kp_voltage, init->ki_voltage, 0, init->fs);

    ctl_init_saturation(&gfm->sat_id, -init->current_limit, init->current_limit);
    ctl_init_saturation(&gfm->sat_iq, -init->current_limit, init->current_limit);

    // Clear States
    ctl_clear_gfm_droop_ctrl(gfm);

    // Flags
    gfm->flag_enable_system = 0;
    gfm->flag_enable_droop = 1;
    gfm->flag_enable_voltage_loop = 1;
    gfm->flag_enable_voltage_ff = 1;
    gfm->flag_enable_virtual_imp = 0; // Default off
}

/**
 * @brief Executes one step of the GFM Droop Controller.
 */
GMP_STATIC_INLINE void ctl_step_gfm_droop(gfm_droop_ctrl_t* gfm)
{
    // Assert inputs
    gmp_base_assert(gfm->vdq_meas);
    gmp_base_assert(gfm->idq_meas);
    gmp_base_assert(gfm->pll_angle);

    // --- 1. Power Calculation ---
    // P = 1.5 * (Vd*Id + Vq*Iq)
    // Q = 1.5 * (Vq*Id - Vd*Iq)
    // Assuming Amplitude Invariant transformation coefficient inside the calc or pre-scaled inputs.
    // Here we use standard P.U. or SI calculation.
    // If inputs are P.U., 1.5 factor is usually absorbed in base definition or not needed if P_base = V_base*I_base.
    // Let's assume standard math:
    ctrl_gt vd = gfm->vdq_meas->dat[phase_d];
    ctrl_gt vq = gfm->vdq_meas->dat[phase_q];
    ctrl_gt id = gfm->idq_meas->dat[phase_d];
    ctrl_gt iq = gfm->idq_meas->dat[phase_q];

    // Simple Power Calc (Adjust 1.5 factor based on your Park Transform convention)
    // Using 1.0 here assuming P.U. system consistency.
    gfm->pq_inst.dat[0] = ctl_mul(1.5f, ctl_mul(vd, id) + ctl_mul(vq, iq));
    gfm->pq_inst.dat[1] = ctl_mul(1.5f, ctl_mul(vq, id) - ctl_mul(vd, iq));

    // Filter Power
    gfm->pq_avg.dat[0] = ctl_step_filter_iir1(&gfm->lpf_pq[0], gfm->pq_inst.dat[0]);
    gfm->pq_avg.dat[1] = ctl_step_filter_iir1(&gfm->lpf_pq[1], gfm->pq_inst.dat[1]);

    // --- 2. Droop Control ---
    if (gfm->flag_enable_droop)
    {
        // f = f0 - kp * (P - P0)
        gfm->droop_freq_ref = gfm->f_nom - ctl_mul(gfm->coef_droop[0], (gfm->pq_avg.dat[0] - gfm->pq_set.dat[0]));

        // V = V0 - kq * (Q - Q0)
        gfm->droop_v_mag_ref = gfm->v_nom - ctl_mul(gfm->coef_droop[1], (gfm->pq_avg.dat[1] - gfm->pq_set.dat[1]));
    }
    else
    {
        gfm->droop_freq_ref = gfm->f_nom;
        gfm->droop_v_mag_ref = gfm->v_nom;
    }

    // --- 3. Angle Generation & Pre-sync Logic ---
    if (gfm->flag_enable_system)
    {
        // GFM Mode: Integrate frequency
        ctrl_gt omega = CTL_PARAM_CONST_2PI * gfm->droop_freq_ref;
        gfm->theta_out += omega * gfm->Ts;

        // Wrap Angle
        if (gfm->theta_out > CTL_PARAM_CONST_2PI)
            gfm->theta_out -= CTL_PARAM_CONST_2PI;
        else if (gfm->theta_out < 0.0f)
            gfm->theta_out += CTL_PARAM_CONST_2PI;
    }
    else
    {
        // Pre-Sync / Standby Mode: Track PLL
        // Reset Integrator state to match Grid
        gfm->theta_out = *gfm->pll_angle;

        // Optional: Pre-load P/Q filters or PID integrators to avoid bumps if needed
        // For simplest voltage start, we just track angle.
    }

    // Update Phasor (Shared with Park Transforms)
    ctl_set_phasor_via_angle(gfm->theta_out, &gfm->phasor_out);

    // --- 4. Virtual Impedance ---
    // V_virt_d = R*id - wL*iq
    // V_virt_q = R*iq + wL*id
    if (gfm->flag_enable_virtual_imp)
    {
        gfm->vdq_vir_imp.dat[phase_d] = ctl_mul(gfm->coef_vir_res, id) - ctl_mul(gfm->coef_vir_ind, iq);
        gfm->vdq_vir_imp.dat[phase_q] = ctl_mul(gfm->coef_vir_res, iq) + ctl_mul(gfm->coef_vir_ind, id);
    }
    else
    {
        gfm->vdq_vir_imp.dat[phase_d] = 0.0f;
        gfm->vdq_vir_imp.dat[phase_q] = 0.0f;
    }

    // Calculate Reference Voltage (aligned to D-axis)
    // Vd_ref = V_droop - V_vir_d
    // Vq_ref = 0       - V_vir_q
    gfm->vdq_virt_ref.dat[phase_d] = gfm->droop_v_mag_ref - gfm->vdq_vir_imp.dat[phase_d];
    gfm->vdq_virt_ref.dat[phase_q] = -gfm->vdq_vir_imp.dat[phase_q];

    // --- 5. Voltage Control Loop ---
    if (gfm->flag_enable_voltage_loop)
    {
        // Error
        ctrl_gt err_vd = gfm->vdq_virt_ref.dat[phase_d] - vd;
        ctrl_gt err_vq = gfm->vdq_virt_ref.dat[phase_q] - vq;

        // PID Calculation
        ctrl_gt id_ref = ctl_step_pid_ser(&gfm->pid_vdq[phase_d], err_vd);
        ctrl_gt iq_ref = ctl_step_pid_ser(&gfm->pid_vdq[phase_q], err_vq);

        // Feedforward Decoupling
        // Id_ff = - w*C*Vq + Id_load (Measured Id is load current)
        // Iq_ff = + w*C*Vd + Iq_load
        if (gfm->flag_enable_voltage_ff)
        {
            gfm->vdq_ff_decouple.dat[phase_d] = -ctl_mul(gfm->coef_vdq_ff, vq); // -wC*vq
            gfm->vdq_ff_decouple.dat[phase_q] = ctl_mul(gfm->coef_vdq_ff, vd);  // +wC*vd

            // Add decoupling + Feedforward Load Current (id, iq)
            id_ref += gfm->vdq_ff_decouple.dat[phase_d] + id;
            iq_ref += gfm->vdq_ff_decouple.dat[phase_q] + iq;
        }

        // Saturation (Current Limit)
        gfm->idq_ref_out.dat[phase_d] = ctl_step_saturation(&gfm->sat_id, id_ref);
        gfm->idq_ref_out.dat[phase_q] = ctl_step_saturation(&gfm->sat_iq, iq_ref);
    }
    else
    {
        // Loop Disabled: Output 0 or hold
        gfm->idq_ref_out.dat[phase_d] = 0.0f;
        gfm->idq_ref_out.dat[phase_q] = 0.0f;
    }
}

/**
 * @brief Checks if the GFM Voltage is ready to engage (Pre-sync check).
 * @return 1 if voltage magnitude is within tolerance of V_nom, 0 otherwise.
 */
GMP_STATIC_INLINE fast_gt ctl_if_gfm_voltage_ready(gfm_droop_ctrl_t* gfm)
{
    // In Pre-Sync (System disabled), Theta is tracking PLL.
    // We just need to check if the Grid Voltage Amplitude matches our Target V_nom.

    // Calculate Grid Voltage Magnitude
    ctrl_gt vd = gfm->vdq_meas->dat[phase_d];
    ctrl_gt vq = gfm->vdq_meas->dat[phase_q];
    ctrl_gt v_mag_meas = ctl_sqrt(ctl_mul(vd, vd) + ctl_mul(vq, vq));

    // Check difference
    ctrl_gt err = ctl_abs(v_mag_meas - gfm->v_nom);

    // Threshold: e.g., 5% of V_nom.
    // This threshold could be a configurable parameter, hardcoded here for simplicity or added to struct.
    ctrl_gt threshold = gfm->v_nom * 0.05f;

    return (err < threshold) ? 1 : 0;
}

void ctl_attach_gfm_droop_with_gfl_core(gfm_droop_ctrl_t* gfm, gfl_inv_ctrl_t* gfl)
{
    gfm->vdq_meas = &gfl->vdq;
    gfm->idq_meas = &gfl->idq;
    gfm->pll_angle = &gfl->pll.theta;

    gfl->phasor_ext = &gfm->phasor_out;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_THREE_PHASE_GFM_

/**
 * @}
 */
