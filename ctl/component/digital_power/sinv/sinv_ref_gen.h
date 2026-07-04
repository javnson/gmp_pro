/**
 * @file ctl_sinv_ref_gen.h
 * @author GMP Library Contributors
 * @brief Single-Phase Inverter Current Reference Generator.
 * * @details
 * Generates the instantaneous AC current reference for grid-tied single-phase 
 * inverters or active front ends (AFE). Supports multiple power dispatch modes 
 * (PQ, S-Phi, P-Phi) and features robust grid-fault protection including 
 * low-voltage anti-windup (division-by-zero prevention) and circular current limiting.
 * * @version 1.0
 * @copyright Copyright GMP(c) 2024-2026
 */

#include <ctl/component/intrinsic/basic/slope_limiter.h>

#ifndef _CTL_SINV_REF_GEN_H_
#define _CTL_SINV_REF_GEN_H_

#ifdef __cplusplus
extern "C"
{
#endif


/*---------------------------------------------------------------------------*/
/* Single-Phase Reference Generator                                          */
/*---------------------------------------------------------------------------*/

/**
 * @brief Data structure for the Single-Phase Reference Generator.
 */
typedef struct _tag_sinv_ref_gen_t
{
    // --- Output ---
    ctrl_gt i_ref_inst; //!< Instantaneous AC current reference i(t) to be fed to the QPR/Current loop.

    // --- Internal States (for monitoring) ---
    ctrl_gt i_p_mag; //!< Actual active current amplitude after limitation.
    ctrl_gt i_q_mag; //!< Actual reactive current amplitude after limitation.

    // --- Protection Parameters ---
    ctrl_gt i_max;     //!< Maximum allowed peak current (Hardware limit).
    ctrl_gt i_max_sq;  //!< Pre-calculated squared max current (i_max * i_max) to save DSP cycles.
    ctrl_gt v_mag_min; //!< Minimum grid voltage threshold to prevent division-by-zero during sags.

    // --- PQ slope limiter ---
    ctl_slope_limiter_t p_slope_lim; //!< Slope limiter for Active Power (P)
    ctl_slope_limiter_t q_slope_lim; //!< Slope limiter for Reactive Power (Q)

    // --- Flags ---
    fast_gt flag_over_current; //!< Flag indicating the requested power exceeded hardware current limits.

} ctl_sinv_ref_gen_t;

/*---------------------------------------------------------------------------*/
/* Initialization Functions                                                  */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initializes the Reference Generator with safety limits and slope restrictions.
 * 
 * @param[out] gen Pointer to the generator instance.
 * @param[in]  i_max Maximum allowed peak current magnitude (e.g., 1.5 * rated peak).
 * @param[in]  v_mag_min Minimum voltage magnitude to allow power calculation (e.g., 0.1 pu).
 * @param[in]  p_slope Max rate of change for Active Power (Units/sec). Pass large value to disable.
 * @param[in]  q_slope Max rate of change for Reactive Power (Units/sec). Pass large value to disable.
 * @param[in]  fs System sampling frequency (Hz) to calculate per-step delta.
 */
void ctl_init_sinv_ref_gen(ctl_sinv_ref_gen_t* gen, parameter_gt i_max, parameter_gt v_mag_min, parameter_gt p_slope,
                           parameter_gt q_slope, parameter_gt fs);

/**
 * @brief Clears the internal states and resets slope limiters of the generator.
 * @param[out] gen Pointer to the generator instance.
 */
GMP_STATIC_INLINE void ctl_clear_sinv_ref_gen(ctl_sinv_ref_gen_t* gen)
{
    gen->i_ref_inst = float2ctrl(0.0f);
    gen->i_p_mag = float2ctrl(0.0f);
    gen->i_q_mag = float2ctrl(0.0f);
    gen->flag_over_current = 0;

    ctl_clear_slope_limiter(&gen->p_slope_lim);
    ctl_clear_slope_limiter(&gen->q_slope_lim);
}

/*---------------------------------------------------------------------------*/
/* Core Execution Step Functions (Overloaded for different dispatch targets) */
/*---------------------------------------------------------------------------*/

/**
 * @brief Base Step Function: Generates current ref based on Active (P) & Reactive (Q) Power.
 * 
 * @param[in,out] gen Pointer to the generator instance.
 * @param[in] _p_ref Target active power command (before slope limiting).
 * @param[in] _q_ref Target reactive power command (before slope limiting).
 * @param[in] v_mag Instantaneous grid voltage magnitude (from PLL).
 * @param[in] phasor Pointer to the synchronized grid phasor [sin(theta), cos(theta)] (from PLL).
 * @return ctrl_gt The instantaneous AC current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_ref_gen_pq(ctl_sinv_ref_gen_t* gen, ctrl_gt _p_ref, ctrl_gt _q_ref,
                                                   ctrl_gt v_mag, const ctl_vector2_t* phasor)
{
    // 1. Voltage Sag Protection (Anti Division-by-Zero)
    ctrl_gt v_safe = (v_mag > gen->v_mag_min) ? v_mag : gen->v_mag_min;

    // Apply slope limits to power commands
    ctrl_gt p_ref = ctl_step_slope_limiter(&gen->p_slope_lim, _p_ref);
    ctrl_gt q_ref = ctl_step_slope_limiter(&gen->q_slope_lim, _q_ref);

    // 2. Calculate ideal current amplitudes: I = 2 * Power / V_mag
    ctrl_gt two_over_v = ctl_div(float2ctrl(2.0f), v_safe);
    ctrl_gt ip = ctl_mul(p_ref, two_over_v);
    ctrl_gt iq = ctl_mul(q_ref, two_over_v);

    // 3. Circular Current Limiting (Prioritizes angle retention, scales down magnitude)
    ctrl_gt i_sq = ctl_mul(ip, ip) + ctl_mul(iq, iq);
    if (i_sq > gen->i_max_sq)
    {
        // set over current flags
        gen->flag_over_current = 1;

        // scale = I_max / sqrt(Ip^2 + Iq^2)
        ctrl_gt scale = ctl_div(gen->i_max, ctl_sqrt(i_sq));
        ip = ctl_mul(ip, scale);
        iq = ctl_mul(iq, scale);
    }
    else
    {
        gen->flag_over_current = 0;
    }

    // Save actual limited magnitudes for monitoring
    gen->i_p_mag = ip;
    gen->i_q_mag = iq;

    // 4. Synthesize Instantaneous Current Reference
    // i_ref(t) = Ip * sin(theta) + Iq * cos(theta)
    gen->i_ref_inst = ctl_mul(ip, phasor->dat[phasor_sin]) + ctl_mul(iq, phasor->dat[phasor_cos]);

    return gen->i_ref_inst;
}

/**
 * @brief Overload 1: Generates current ref based on Apparent Power (S) and Power Factor Angle (Phi).
 * 
 * @param[in,out] gen Pointer to the generator instance.
 * @param[in] s_ref Target apparent power magnitude.
 * @param[in] phi_pu Target power factor angle in per-unit (0 to 1.0 represents 0 to 2*pi).
 * @param[in] v_mag Grid voltage magnitude.
 * @param[in] phasor Pointer to the synchronized grid phasor.
 * @return ctrl_gt The instantaneous AC current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_ref_gen_s_phi(ctl_sinv_ref_gen_t* gen, ctrl_gt s_ref, ctrl_gt phi_pu,
                                                      ctrl_gt v_mag, const ctl_vector2_t* phasor)
{
    // P = S * cos(phi)
    // Q = S * sin(phi)
    ctrl_gt p_ref = ctl_mul(s_ref, ctl_cos(phi_pu));
    ctrl_gt q_ref = ctl_mul(s_ref, ctl_sin(phi_pu));

    return ctl_step_sinv_ref_gen_pq(gen, p_ref, q_ref, v_mag, phasor);
}

/**
 * @brief Overload 2: Generates current ref based on Active Power (P) and Power Factor Angle (Phi).
 * 
 * @param[in,out] gen Pointer to the generator instance.
 * @param[in] p_ref Target active power.
 * @param[in] phi_pu Target power factor angle in per-unit (0 to 1.0 represents 0 to 2*pi).
 * @param[in] v_mag Grid voltage magnitude.
 * @param[in] phasor Pointer to the synchronized grid phasor.
 * @return ctrl_gt The instantaneous AC current reference.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_sinv_ref_gen_p_phi(ctl_sinv_ref_gen_t* gen, ctrl_gt p_ref, ctrl_gt phi_pu,
                                                      ctrl_gt v_mag, const ctl_vector2_t* phasor)
{
    // Q = P * tan(phi) = P * (sin(phi) / cos(phi))
    ctrl_gt sin_phi = ctl_sin(phi_pu);
    ctrl_gt cos_phi = ctl_cos(phi_pu);

    // Prevent division by zero if PF is exactly 0 (phi = 90 deg)
    ctrl_gt safe_cos = (ctl_abs(cos_phi) > float2ctrl(0.001f)) ? cos_phi : float2ctrl(0.001f);

    ctrl_gt tan_phi = ctl_div(sin_phi, safe_cos);
    ctrl_gt q_ref = ctl_mul(p_ref, tan_phi);

    return ctl_step_sinv_ref_gen_pq(gen, p_ref, q_ref, v_mag, phasor);
}

#ifdef __cplusplus
}
#endif

#endif // _CTL_SINV_REF_GEN_H_
