/**
 * @file im.fo.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Implements a flux and torque observer for an Induction Motor (IM).
 * @details This module estimates the rotor flux and electromagnetic torque of an induction
 * motor using a voltage model. It integrates the stator back-EMF to find the stator flux,
 * then calculates the rotor flux and torque. A low-pass filter is used instead of a pure
 * integrator to mitigate DC drift issues at low speeds.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_IM_FO_H_
#define _FILE_IM_FO_H_

#include <ctl/math_block/vector_lite/vector2.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup IM_FLUX_OBSERVER Induction Motor Flux and Torque Observer
 * @brief A module for estimating IM rotor flux and electromagnetic torque.
 *
 * This observer is based on the voltage model in the stationary (¦Á-¦Â) reference frame.
 *
 * **Stator Flux Estimation (using a Low-Pass Filter):**
 * @f[ \psi_{s(\alpha\beta)} = \frac{1}{s + \omega_c}(V_{s(\alpha\beta)} - R_s I_{s(\alpha\beta)}) @f]
 * @f[ \vec{\psi}_s = \frac{1}{s + \omega_c}(\vec{V}_s - R_s \vec{I}_s) @f]
 *
 * **Rotor Flux Calculation:**
 * @f[ \psi_{r(\alpha\beta)} = \frac{L_r}{L_m}(\psi_{s(\alpha\beta)} - \sigma L_s I_{s(\alpha\beta)}) @f]
 * @f[ \vec{\psi}_r = \frac{L_r}{L_m}(\vec{\psi}_s - \sigma L_s \vec{I}_s) @f]
 * where @f[ \sigma = 1 - \frac{L_m^2}{L_s L_r} @f]
 *
 * **Torque Equation:**
 * @f[ T_e = \frac{3}{2}P \frac{L_m}{L_r} (\psi_{r\alpha} i_{s\beta} - \psi_{r\beta} i_{s\alpha}) @f]
 * @f[ T_e = \frac{3}{2}P \frac{L_m}{L_r} (\psi_{r\alpha} i_{s\beta} - \psi_{r\beta} i_{s\alpha}) @f]
 *
 */

/*---------------------------------------------------------------------------*/
/* Induction Motor Flux and Torque Observer                                  */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup IM_FLUX_OBSERVER
 * @{
 */

/**
 * @brief Data structure for the Induction Motor flux observer.
 */
typedef struct _tag_im_fo_t
{
    //
    // Outputs
    //
    vector2_gt psi_r;  /**< The estimated rotor flux vector [¦Á, ¦Â] in per-unit. */
    ctrl_gt psi_r_mag; /**< The magnitude of the estimated rotor flux in per-unit. */
    ctrl_gt torque;    /**< The estimated electromagnetic torque in per-unit. */

    //
    // Parameters
    //
    ctrl_gt rs;           /**< The per-unit stator resistance (Rs*). */
    ctrl_gt k_lpf;        /**< The coefficient for the low-pass filter integrator (1 - ¦Øc*Ts). */
    ctrl_gt k_ts;         /**< The coefficient for the integrator input (Ts). */
    ctrl_gt k_rotor_flux; /**< The coefficient for rotor flux calculation (Lr / Lm). */
    ctrl_gt k_sigma_ls;   /**< The coefficient for rotor flux calculation (¦Ò * Ls). */
    ctrl_gt k_torque;     /**< The coefficient for torque calculation ( (3/2)*P*(Lm/Lr) ). */

    //
    // Internal State Variables
    //
    vector2_gt psi_s; /**< The internal state for the estimated stator flux vector [¦Á, ¦Â]. */

} im_fo_t;

/**
 * @brief Initializes the IM flux observer object.
 * @param fo Pointer to the `im_fo_t` object.
 * @param rs_star Per-unit stator resistance.
 * @param ls_star Per-unit stator inductance.
 * @param lr_star Per-unit rotor inductance.
 * @param lm_star Per-unit magnetizing inductance.
 * @param pole_pairs Number of motor pole pairs.
 * @param ts_s Controller sample time in seconds.
 * @param wc_rps Cutoff frequency for the low-pass filter integrator in rad/s.
 */
void ctl_init_im_fo(im_fo_t* fo, ctrl_gt rs_star, ctrl_gt ls_star, ctrl_gt lr_star, ctrl_gt lm_star, ctrl_gt pole_pairs,
                    ctrl_gt ts_s, ctrl_gt wc_rps);

/**
 * @brief Executes one step of the flux and torque observation for an IM.
 * @param fo Pointer to the `im_fo_t` object.
 * @param v_ab_star Pointer to a vector containing the ¦Á and ¦Â stator voltages in per-unit.
 * @param i_ab_star Pointer to a vector containing the ¦Á and ¦Â stator currents in per-unit.
 */
GMP_STATIC_INLINE void ctl_step_im_fo(im_fo_t* fo, const vector2_gt* v_ab_star, const vector2_gt* i_ab_star)
{
    // Calculate stator back-EMF
    ctrl_gt bemf_alpha = ctl_sub(v_ab_star->dat[0], ctl_mul(fo->rs, i_ab_star->dat[0]));
    ctrl_gt bemf_beta = ctl_sub(v_ab_star->dat[1], ctl_mul(fo->rs, i_ab_star->dat[1]));

    // Estimate stator flux using a low-pass filter integrator
    // ¦×s[k] = k_lpf * ¦×s[k-1] + k_ts * BEMF[k]
    fo->psi_s.dat[0] = ctl_add(ctl_mul(fo->k_lpf, fo->psi_s.dat[0]), ctl_mul(fo->k_ts, bemf_alpha));
    fo->psi_s.dat[1] = ctl_add(ctl_mul(fo->k_lpf, fo->psi_s.dat[1]), ctl_mul(fo->k_ts, bemf_beta));

    // Calculate rotor flux
    // ¦×r_¦Á = k_rotor_flux * (¦×s_¦Á - k_sigma_ls * is_¦Á)
    fo->psi_r.dat[0] = ctl_mul(fo->k_rotor_flux, ctl_sub(fo->psi_s.dat[0], ctl_mul(fo->k_sigma_ls, i_ab_star->dat[0])));
    fo->psi_r.dat[1] = ctl_mul(fo->k_rotor_flux, ctl_sub(fo->psi_s.dat[1], ctl_mul(fo->k_sigma_ls, i_ab_star->dat[1])));

    // Calculate rotor flux magnitude
    fo->psi_r_mag =
        ctl_sqrt(ctl_add(ctl_mul(fo->psi_r.dat[0], fo->psi_r.dat[0]), ctl_mul(fo->psi_r.dat[1], fo->psi_r.dat[1])));

    // Calculate electromagnetic torque
    // Te = k_torque * (¦×r_¦Á * is_¦Â - ¦×r_¦Â * is_¦Á)
    ctrl_gt torque_term =
        ctl_sub(ctl_mul(fo->psi_r.dat[0], i_ab_star->dat[1]), ctl_mul(fo->psi_r.dat[1], i_ab_star->dat[0]));
    fo->torque = ctl_mul(fo->k_torque, torque_term);
}

/**
 * @brief Gets the estimated rotor flux vector (¦Á, ¦Â).
 * @param fo Pointer to the `im_fo_t` object.
 * @return A pointer to the `vector2_gt` containing the rotor flux components.
 */
GMP_STATIC_INLINE const vector2_gt* ctl_get_im_fo_flux_vec(const im_fo_t* fo)
{
    return &fo->psi_r;
}

/**
 * @brief Gets the magnitude of the estimated rotor flux.
 * @param fo Pointer to the `im_fo_t` object.
 * @return The rotor flux magnitude in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_im_fo_flux_mag(const im_fo_t* fo)
{
    return fo->psi_r_mag;
}

/**
 * @brief Gets the estimated electromagnetic torque.
 * @param fo Pointer to the `im_fo_t` object.
 * @return The torque in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_im_fo_torque(const im_fo_t* fo)
{
    return fo->torque;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_IM_FO_H_
