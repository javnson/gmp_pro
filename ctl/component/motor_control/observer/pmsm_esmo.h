/**
 * @file pmsm.smo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements the Extended-EMF Sliding Mode Observer (ESMO) for PMSM.
 * @details This module provides a highly optimized, fixed-point sensorless
 * position and speed estimator. It utilizes the Extended-EMF (EEMF) model to
 * unify SPM and IPM control, completely eliminating trigonometric functions
 * in the observer plant. It strictly adheres to Per-Unit (PU) normalization
 * and features a non-blocking Angle Tracking Observer (ATO) with robust
 * anti-overflow divergence protection.
 *
 * @version 0.2
 * @date 2025-08-06
 *
 * @copyright Copyright GMP(c) 2025
 *
 */


#include <ctl/component/intrinsic/continuous/continuous_pid.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>
#include <ctl/component/motor_control/interface/encoder.h>
#include <ctl/component/motor_control/observer/ato_pll.h>
#include <ctl/math_block/coordinate/coord_trans.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/component/motor_control/current_loop/foc_core.h>

#ifndef _FILE_PMSM_SMO_H_
#define _FILE_PMSM_SMO_H_


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

	/*---------------------------------------------------------------------------*/
	/* PMSM Sliding Mode Observer (SMO) Position Estimator                       */
	/*---------------------------------------------------------------------------*/

	/**
	 * @defgroup PMSM_SMO PMSM SMO Estimator
	 * @brief A module for sensorless position and speed estimation using SMO.
	 * @details Implements the observer model, sliding control law, filtering, and PLL
	 * required to track the rotor angle of a PMSM.
	 *
	 * This module implements an SMO to estimate the rotor position and speed of a
	 * Permanent Magnet Synchronous Motor. It uses the motor's voltage and current
	 * measurements to drive an observer model. A sliding control law generates a
	 * correction term (z) that, when filtered, represents the motor's back-EMF.
	 * A Phase-Locked Loop (PLL) then tracks the back-EMF signal to extract the
	 * rotor angle and speed.
	 *
	 * The SMO is based on the PMSM voltage model in the stationary @f(\alpha-\beta@f) frame.
	 * The observer estimates the stator currents:
	 * @f[ \frac{d\hat{i}_{\alpha\beta}}{dt} = \frac{1}{L_s}(v_{\alpha\beta} - R_s\hat{i}_{\alpha\beta} - z_{\alpha\beta}) @f]
	 * The sliding control law, z, forces the estimated current to track the measured current:
	 * @f[ z_{\alpha\beta} = k_{slide} \cdot \text{sign}(\hat{i}_{\alpha\beta} - i_{\alpha\beta}) @f]
	 * When in the sliding mode, the filtered value of z is equal to the back-EMF:
	 * @f[ E_{\alpha\beta} \approx \bar{z}_{\alpha\beta} @f]
	 * @{
	 */

	 /**
	  * @brief Main structure for the PMSM ESMO controller.
	  * @details Encapsulates all state variables, pre-calculated physical constants,
	  * scale factors, and safety mechanisms required for the observer.
	  */
	typedef struct _tag_pmsm_esmo_t
	{
		// --- Standard Outputs (User Interfaces) ---
		rotation_ift pos_out; //!< Standard position interface providing estimated electrical angle.
		velocity_ift spd_out; //!< Standard velocity interface providing estimated electrical speed.

		// --- Core Sub-modules ---
		ctl_ato_pll_t ato_pll;         //!< Angle Tracking Observer (Software PLL) for zero-lag tracking.
		ctl_filter_IIR1_t filter_e[2]; //!< Low-pass filters for extracting Back-EMF components (alpha, beta).
		ctrl_gt output_angle_bias;     //!< Output angle fixed, angle in PU.

		// --- State Variables ---
		ctl_vector2_t i_est;  //!< Estimated stator current vector [alpha, beta] in PU.
		ctl_vector2_t e_est;  //!< Estimated Back-EMF vector [alpha, beta] in PU.
		ctl_vector2_t z_slid; //!< Sliding control law output vector [alpha, beta] (Voltage correction).
		ctl_vector2_t phasor; //!< Estimated rotor phasor [cos(theta), sin(theta)].

		// --- Pre-calculated Physical Constants (Plant Model) ---
		ctrl_gt k1; //!< Voltage integration coefficient: (Ts * V_base) / (Ld * I_base).
		ctrl_gt k2; //!< Current decay coefficient: (Rs * Ts) / Ld.
		ctrl_gt k3; //!< Saliency cross-coupling coefficient: (Ld - Lq) / Ld.

		// --- Scale Factors & Margins (Optimized for Fixed-Point) ---
		ctrl_gt sf_w_to_rad_tick; //!< Scale factor: converts PU speed to rad/tick for cross-coupling.
		ctrl_gt sf_wc_inv;        //!< Scale factor: 1 / W_cutoff for analytical phase delay compensation.
		ctrl_gt k_slide;          //!< Sliding gain determining the boundary reaching capability.
		ctrl_gt z_margin;         //!< Boundary layer margin for the continuous saturation function.
		ctrl_gt sf_z_margin_inv;  //!< Pre-calculated inverse of the margin (1 / z_margin) to avoid division.

		// --- Safety Mechanism ---
		ctrl_gt current_err_limit; //!< Maximum allowed absolute current tracking error (PU) before triggering divergence.
		uint32_t diverge_cnt;      //!< Divergence debounce counter with anti-overflow saturation.
		uint32_t diverge_limit;    //!< Divergence debounce limit (ticks) to confirm Loss-of-Lock.

		// --- Flags ---
		fast_gt flag_enable;          //!< Master enable flag for the observer execution.
		fast_gt flag_observer_locked; //!< Status flag: 1 if tracking is stable, 0 if observer has diverged.
		fast_gt flag_lpf_compensate;  //!< Status flag: 1 if enable LPF compensate, 0 if LPF compensate is disabled.
		fast_gt flag_enable_bias;     //!< Status flag: 1 if enable angle fixed angle, 0 if fixed angle bias is disabled.

	} ctl_pmsm_esmo_t;

	/**
	 * @brief Raw initialization structure for the ESMO.
	 * @details Used when the user prefers to supply bare physical parameters directly
	 * rather than using the high-level Consultant objects.
	 */
	typedef struct _tag_pmsm_esmo_init_t
	{
		// --- Motor Physical Parameters ---
		parameter_gt Rs;           //!< Stator phase resistance (Ohm).
		parameter_gt Ld;           //!< D-axis synchronous inductance (H).
		parameter_gt Lq;           //!< Q-axis synchronous inductance (H).
		parameter_gt flux_linkage; //!< Permanent magnet flux linkage (Wb).

		// --- Per-Unit Base Values ---
		parameter_gt V_base; //!< Base Phase Voltage Peak (V).
		parameter_gt I_base; //!< Base Phase Current Peak (A).
		parameter_gt W_base; //!< Base Electrical Angular Velocity (rad/s).

		// --- Execution & Tuning Parameters ---
		parameter_gt fs;            //!< Controller execution frequency (Hz).
		parameter_gt fc_emf;        //!< Cutoff frequency for Back-EMF LPF (Hz).
		parameter_gt ato_bw_hz;     //!< Bandwidth for the ATO/PLL (Hz).
		parameter_gt fault_time_ms; //!< Divergence confirmation debounce time (ms).

		// --- Margins & Limits ---
		parameter_gt current_err_limit_pu; //!< Max absolute current tracking error (PU) before divergence (e.g., 0.3f).
		parameter_gt z_margin_pu;          //!< Boundary layer margin for sliding control (e.g., 0.05f).

	} ctl_pmsm_esmo_init_t;

	//================================================================================
	// Function Prototypes & Inline Definitions
	//================================================================================

	/**
	 * @brief Core initialization function using the bare physical parameters.
	 * @param[out] esmo Pointer to the ESMO instance.
	 * @param[in]  init Pointer to the raw initialization structure.
	 */
	void ctl_init_pmsm_esmo(ctl_pmsm_esmo_t* esmo, const ctl_pmsm_esmo_init_t* init);

	/**
	 * @brief Auto-tunes and populates the ESMO init structure using motor base parameters.
	 * @details Translates physical motor parameters into the ESMO initialization format
	 * and automatically calculates the optimal bandwidths and cutoff frequencies
	 * for the back-EMF filter and the Angle Tracking Observer (ATO).
	 * * @param[out] esmo_init Pointer to the ESMO init structure to be populated.
	 * @param[in]  cur_init  Pointer to the generic motor and current loop base configuration.
	 * @param[in]  flux_linkage Permanent magnet flux linkage in Webers (Wb).
	 */
	void ctl_autotune_esmo_init_from_mtr(ctl_pmsm_esmo_init_t* esmo_init,
		const mtr_current_init_t* cur_init,
		parameter_gt flux_linkage);

	/**
	 * @brief Auto-tunes and initializes the ESMO using rigid physics from the Consultants.
	 * @details Absorbs all dimension conversions (W_base, I_base, etc.) and sampling time (Ts)
	 * into pure fixed-point constants for extreme execution efficiency.
	 * @param[out] esmo Pointer to the ESMO structure to be initialized.
	 * @param[in]  motor Pointer to the PMSM physical model consultant.
	 * @param[in]  pu Pointer to the Per-Unit base model consultant.
	 * @param[in]  fs Controller execution frequency (Hz).
	 * @param[in]  fc_emf Cutoff frequency for the Back-EMF extraction low-pass filter (Hz).
	 * @param[in]  ato_bw_hz Bandwidth for the ATO/PLL tracking loop (Hz).
	 * @param[in]  fault_time_ms Divergence confirmation debounce time (ms).
	 */
	void ctl_init_pmsm_esmo_consultant(ctl_pmsm_esmo_t* esmo, const ctl_consultant_pmsm_t* motor, const ctl_consultant_pu_pmsm_t* pu,
		parameter_gt fs, parameter_gt fc_emf, parameter_gt ato_bw_hz, parameter_gt fault_time_ms);

	/**
	 * @brief Safely clears all history states, integrators, and health flags.
	 * @details Does not alter the enable/disable state. Essential for bumpless restarts.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_clear_pmsm_esmo(ctl_pmsm_esmo_t* esmo)
	{
		ctl_vector2_clear(&esmo->i_est);
		ctl_vector2_clear(&esmo->e_est);
		ctl_vector2_clear(&esmo->z_slid);

		ctl_clear_filter_iir1(&esmo->filter_e[0]);
		ctl_clear_filter_iir1(&esmo->filter_e[1]);
		ctl_clear_ato_pll(&esmo->ato_pll);

		esmo->diverge_cnt = 0;
		esmo->flag_observer_locked = 0;
		esmo->pos_out.elec_position = float2ctrl(0.0f);
		esmo->spd_out.speed = float2ctrl(0.0f);
	}

	/**
	 * @brief Enables the ESMO execution.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_enable_pmsm_esmo(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_enable = 1;
	}

	/**
	 * @brief Disables the ESMO execution.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_disable_pmsm_esmo(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_enable = 0;
	}

	/**
	 * @brief Enables the ESMO compensator.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_enable_pmsm_esmo_compensate(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_lpf_compensate = 1;
	}

	/**
	 * @brief Disables the ESMO compensator.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_disable_pmsm_esmo_compensate(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_lpf_compensate = 0;
	}

	/**
	 * @brief Enables the ESMO compensator.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_enable_pmsm_esmo_bias(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_enable_bias = 1;
	}

	/**
	 * @brief Disables the ESMO compensator.
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 */
	GMP_STATIC_INLINE void ctl_disable_pmsm_esmo_bias(ctl_pmsm_esmo_t* esmo)
	{
		esmo->flag_enable_bias = 0;
	}


	/**
	 * @brief Executes one high-frequency step of the Extended-EMF Sliding Mode Observer.
	 * @details Solves the physics-accurate difference equations, applies the sliding control law,
	 * assesses tracking health, and updates the Phase-Locked Loop (ATO).
	 * @param[in,out] esmo Pointer to the ESMO instance.
	 * @param[in] v_alpha Applied alpha-axis stator voltage (PU).
	 * @param[in] v_beta Applied beta-axis stator voltage (PU).
	 * @param[in] i_alpha Measured alpha-axis stator current (PU).
	 * @param[in] i_beta Measured beta-axis stator current (PU).
	 */
	GMP_STATIC_INLINE void ctl_step_pmsm_esmo(ctl_pmsm_esmo_t* esmo, ctrl_gt v_alpha, ctrl_gt v_beta, ctrl_gt i_alpha,
		ctrl_gt i_beta)
	{
		if (!esmo->flag_enable)
			return;

		// ========================================================================
		// 1. Current Estimation Plant (Difference Equations)
		// ========================================================================
		ctrl_gt wr_tick = ctl_mul(esmo->ato_pll.elec_speed_pu, esmo->sf_w_to_rad_tick);
		ctrl_gt cross_term = ctl_mul(wr_tick, esmo->k3);

		ctrl_gt delta_i_alpha = ctl_mul(esmo->k1, v_alpha - esmo->z_slid.dat[0]) - ctl_mul(esmo->k2, esmo->i_est.dat[0]) -
			ctl_mul(cross_term, esmo->i_est.dat[1]);

		ctrl_gt delta_i_beta = ctl_mul(esmo->k1, v_beta - esmo->z_slid.dat[1]) - ctl_mul(esmo->k2, esmo->i_est.dat[1]) +
			ctl_mul(cross_term, esmo->i_est.dat[0]);

		esmo->i_est.dat[0] += delta_i_alpha;
		esmo->i_est.dat[1] += delta_i_beta;

		// ========================================================================
		// 2. Sliding Control Law (Continuous Saturation)
		// ========================================================================
		ctrl_gt err_alpha = esmo->i_est.dat[0] - i_alpha;
		ctrl_gt err_beta = esmo->i_est.dat[1] - i_beta;

		// Saturation eliminates high-frequency chattering inherent to standard Sign() functions.
		ctrl_gt sat_alpha = ctl_sat(err_alpha, esmo->z_margin, -esmo->z_margin);
		ctrl_gt sat_beta = ctl_sat(err_beta, esmo->z_margin, -esmo->z_margin);

		esmo->z_slid.dat[0] = ctl_mul(esmo->k_slide, ctl_mul(sat_alpha, esmo->sf_z_margin_inv));
		esmo->z_slid.dat[1] = ctl_mul(esmo->k_slide, ctl_mul(sat_beta, esmo->sf_z_margin_inv));

		// ========================================================================
		// 3. Back-EMF Extraction
		// ========================================================================
		esmo->e_est.dat[0] = ctl_step_filter_iir1(&esmo->filter_e[0], esmo->z_slid.dat[0]);
		esmo->e_est.dat[1] = ctl_step_filter_iir1(&esmo->filter_e[1], esmo->z_slid.dat[1]);

		// ========================================================================
		// 4. Observer Health Assessment (Loss-of-Lock Protection)
		// ========================================================================
		ctrl_gt abs_err_alpha = (err_alpha > float2ctrl(0.0f)) ? err_alpha : -err_alpha;
		ctrl_gt abs_err_beta = (err_beta > float2ctrl(0.0f)) ? err_beta : -err_beta;

		// Fast O(1) Absolute Threshold Check for divergence
		if ((abs_err_alpha > esmo->current_err_limit) || (abs_err_beta > esmo->current_err_limit))
		{
			// Saturation constraint to prevent counter overflow
			if (esmo->diverge_cnt < esmo->diverge_limit)
			{
				esmo->diverge_cnt++;
			}
			if (esmo->diverge_cnt >= esmo->diverge_limit)
			{
				esmo->flag_observer_locked = 0;
			}
		}
		else
		{
			if (esmo->diverge_cnt > 0)
			{
				esmo->diverge_cnt--;
			}
			if (esmo->diverge_cnt == 0)
			{
				esmo->flag_observer_locked = 1;
			}
		}

		// ========================================================================
		// 5. Phase Error Generation
		// ========================================================================
		// Math Formulation for PLL error signal:
		// Actual Back-EMF: E_alpha = -|E|*sin(theta), E_beta = |E|*cos(theta)
		// Target error   : err = sin(theta - theta_hat)
		//                      = sin(theta)cos(theta_hat) - cos(theta)sin(theta_hat)
		// Substitute EMF : err = (-E_alpha/|E|)*cos(theta_hat) - (E_beta/|E|)*sin(theta_hat)
		//
		// Dropping the |E| magnitude (absorbed by PLL PI gains), the error voltage is:
		// e_err_voltage  = -E_alpha * cos(theta_hat) - E_beta * sin(theta_hat)
		//
		// Array Mapping:
		// e_est.dat[0]  -> E_alpha
		// e_est.dat[1]  -> E_beta
		// phasor.dat[0] -> sin(theta_hat)
		// phasor.dat[1] -> cos(theta_hat)
		// ========================================================================
		ctl_set_phasor_via_angle(esmo->ato_pll.elec_angle_pu, &esmo->phasor);

		ctrl_gt e_err_voltage =
			-ctl_mul(esmo->e_est.dat[0], esmo->phasor.dat[1]) - ctl_mul(esmo->e_est.dat[1], esmo->phasor.dat[0]);


		// ========================================================================
		// 6. ATO/PLL Step & Analytical Phase Compensation
		// ========================================================================
		ctl_step_ato_pll(&esmo->ato_pll, e_err_voltage);

		ctrl_gt comp_angle_pu = esmo->ato_pll.elec_angle_pu;

		if (esmo->flag_lpf_compensate)
		{
			// Calculate LPF-induced phase lag: Phi_lag = atan(W_elec / W_cutoff)
			ctrl_gt tan_phi = ctl_mul(esmo->ato_pll.elec_speed_pu, esmo->sf_wc_inv);
			ctrl_gt phase_lag_rad = ctl_atan2(tan_phi, CTL_CTRL_CONST_1);

			// Apply forward compensation to cancel out the LPF delay
			comp_angle_pu += ctl_mul(phase_lag_rad, CTL_CTRL_CONST_1_OVER_2PI);
		}

		if (esmo->ato_pll.elec_speed_pu < float2ctrl(0.0f))
		{
			// 加上 180 度 (在标幺值 PU 下，180 度就是 0.5f)
			comp_angle_pu += float2ctrl(0.5f);

			if (esmo->flag_enable_bias)
			{
				comp_angle_pu += esmo->output_angle_bias;
			}
		}
		else
		{
			if (esmo->flag_enable_bias)
			{
				comp_angle_pu -= esmo->output_angle_bias;
			}
		}


		// ========================================================================
		// 7. Output to Top-Level Interfaces
		// ========================================================================
		esmo->pos_out.elec_position = ctrl_mod_1(comp_angle_pu + float2ctrl(1.0f));
		esmo->spd_out.speed = esmo->ato_pll.elec_speed_pu;
	}

	/** @} */ // end of PMSM_SMO group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PMSM_SMO_H_
