
#include <gmp_core.h>

//////////////////////////////////////////////////////////////////////////
// PR / QPR controller

#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

void ctl_init_resonant_controller(resonant_ctrl_t* r, parameter_gt kr, parameter_gt freq_resonant, parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt T = 1.0f / fs;
    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wr_sq_T_sq = wr * wr * T * T;

    // Based on the bilinear transformation of G(s) = kr * (2s) / (s^2 + wr^2)
    // The resulting difference equation is:
    // u(n) = a1*u(n-1) + a2*u(n-2) + b0*e(n) + b2*e(n-2)
    parameter_gt den = wr_sq_T_sq + 4.0f;
    parameter_gt inv_den = 1.0f / den;

    r->b0 = float2ctrl(kr * 2.0f * T * inv_den);
    r->b2 = float2ctrl(-kr * 2.0f * T * inv_den);
    r->a1 = float2ctrl(2.0f * (4.0f - wr_sq_T_sq) * inv_den);
    r->a2 = float2ctrl(-1.0f); // This simplifies from -(4 + T^2*wr^2 - 8)/(4+T^2*wr^2) if no damping

    ctl_clear_resonant_controller(r);
}

void ctl_init_pr_controller(pr_ctrl_t* pr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                            parameter_gt fs)
{
    pr->kp = float2ctrl(kp);
    ctl_init_resonant_controller(&pr->resonant_part, kr, freq_resonant, fs);
}

/**
 * @brief Helper function to calculate QR coefficients based on a specific K value.
 * @details Solves the Tustin substitution algebra.
 * Transfer Function: G(s) = Kr * (2*Wc*s) / (s^2 + 2*Wc*s + Wr^2)
 * Sub: s = K * (1-z^-1)/(1+z^-1)
 */
static void _ctl_calc_qr_coeffs(qr_ctrl_t* qr, parameter_gt kr, parameter_gt wc, parameter_gt wr, parameter_gt k_tustin)
{
    parameter_gt k_sq = k_tustin * k_tustin;
    parameter_gt wr_sq = wr * wr;

    // Common Denominator (D0)
    // D0 = k^2 + 2*wc*k + wr^2
    parameter_gt D0 = k_sq + (2.0f * wc * k_tustin) + wr_sq;

    // Check for stability/singularity
    if (D0 < 1e-9f)
        D0 = 1e-9f;
    parameter_gt inv_D0 = 1.0f / D0;

    // --- Numerator Coefficients ---
    // Num = 2 * Kr * Wc * K * (1 - z^-2)
    // b0 = (2 * Kr * Wc * K) / D0
    qr->b0 = (2.0f * kr * wc * k_tustin) * inv_D0;

    // b1 = 0 (Theoretical property of QR Tustin transform)

    // b2 = -b0
    qr->b2 = -qr->b0;

    // --- Denominator Coefficients ---
    // Denom = D0 + (2*wr^2 - 2*k^2)z^-1 + (k^2 - 2*wc*k + wr^2)z^-2
    // Difference Eq: y[n] = b0*x[n] + ... - A1*y[n-1] - A2*y[n-2]
    // User's step function uses ADDITION: y = a1*y1 + a2*y2 ...
    // So we must store NEGATIVE coefficients.

    // Real A1 = (2*wr^2 - 2*k^2) / D0
    // Stored a1 = -A1 = (2*k^2 - 2*wr^2) / D0
    qr->a1 = (2.0f * k_sq - 2.0f * wr_sq) * inv_D0;

    // Real A2 = (k^2 - 2*wc*k + wr^2) / D0
    // Stored a2 = -A2 = (2*wc*k - k^2 - wr^2) / D0
    // Note: Simplifies to -(k^2 - 2*wc*k + wr^2) / D0
    qr->a2 = (2.0f * wc * k_tustin - k_sq - wr_sq) * inv_D0;
}

/**
 * @brief Initializes a quasi-resonant controller using Standard Tustin.
 * @note  Use this only for low frequency resonances relative to Fs.
 */
void ctl_init_qr_controller(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                            parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    // Standard Tustin K = 2 * Fs
    parameter_gt k_val = 2.0f * fs;

    _ctl_calc_qr_coeffs(qr, kr, wc, wr, k_val);
    ctl_clear_qr_controller(qr);
}

/**
 * @brief Initializes a quasi-resonant controller with Frequency Pre-warping.
 * @details Corrects the frequency warping effect of bilinear transformation at the resonant frequency.
 * Essential for harmonic control (e.g., 6th, 12th harmonics).
 * * @param[out] qr Pointer to the QR controller instance.
 * @param[in] kr Gain of the resonant term.
 * @param[in] freq_resonant Resonant frequency in Hz (Center Frequency).
 * @param[in] freq_cut Cutoff frequency in Hz (Bandwidth/2).
 * @param[in] fs Sampling frequency in Hz.
 */
void ctl_init_qr_controller_prewarped(qr_ctrl_t* qr, parameter_gt kr, parameter_gt freq_resonant, parameter_gt freq_cut,
                                      parameter_gt fs)
{
    gmp_base_assert(fs > 0.0);

    parameter_gt wr = CTL_PARAM_CONST_2PI * freq_resonant;
    parameter_gt wc = CTL_PARAM_CONST_2PI * freq_cut;

    // --- Pre-warping Calculation ---
    // Target angle in discrete domain: Wd = Wr * Ts
    // Tustin mapping: Wa = (2/Ts) * tan(Wd / 2)
    // We replace the standard K (2/Ts) with K_pre = Wr / tan(Wr * Ts / 2)

    // Half angle normalized: (2*pi*f_res) / (2*fs) = pi * f_res / fs
    parameter_gt half_angle = CTL_PARAM_CONST_PI * freq_resonant / fs;

    // Safety for DC or Nyquist
    if (half_angle < 1e-6f)
        half_angle = 1e-6f;
    if (half_angle > (CTL_PARAM_CONST_PI / 2.0f - 1e-6f))
        half_angle = (CTL_PARAM_CONST_PI / 2.0f - 1e-6f);

    parameter_gt tan_val = tanf(half_angle);

    // The "Pre-warped" K value
    parameter_gt k_pre = wr / tan_val;

    _ctl_calc_qr_coeffs(qr, kr, wc, wr, k_pre);
    ctl_clear_qr_controller(qr);
}

void ctl_init_qpr_controller(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                             parameter_gt freq_cut, parameter_gt fs)
{
    qpr->kp = float2ctrl(kp);
    ctl_init_qr_controller(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

void ctl_init_qpr_controller_prewarped(qpr_ctrl_t* qpr, parameter_gt kp, parameter_gt kr, parameter_gt freq_resonant,
                                       parameter_gt freq_cut, parameter_gt fs)
{
    qpr->kp = float2ctrl(kp);
    ctl_init_qr_controller_prewarped(&qpr->resonant_part, kr, freq_resonant, freq_cut, fs);
}

