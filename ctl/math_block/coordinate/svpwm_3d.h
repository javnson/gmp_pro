/**
 * @file svpwm_3d.h
 * @brief Four-leg three-dimensional SVPWM for three-phase four-wire converters.
 *
 * The input is an alpha-beta-zero phase-to-neutral voltage reference normalized
 * to the DC-link voltage. The output order is A, B, C, N and represents the
 * four bridge-leg duty ratios.
 */

#ifndef _FILE_GMP_MATH_SVPWM_3D_H_
#define _FILE_GMP_MATH_SVPWM_3D_H_

#include <ctl/math_block/coordinate/Clarke.h>
#include <ctl/math_block/vector_lite/vector4.h>

#if __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Calculates centered four-leg pole commands without the 0.5 duty bias.
 * @details The neutral leg is included in the min/max common-mode selection.
 * For references inside the linear modulation region, all four results are in
 * [-0.5, 0.5] and the requested phase-to-neutral voltages are preserved:
 * @f[
 * T_A-T_N=v_A,\quad T_B-T_N=v_B,\quad T_C-T_N=v_C.
 * @f]
 * @param[in] ab0 Alpha-beta-zero phase-to-neutral reference, normalized by Vdc.
 * @param[out] Tabcn Centered A/B/C/N pole commands.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_3d(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector4_t* Tabcn)
{
    ctl_vector3_t vabc;
    ctrl_gt vmax;
    ctrl_gt vmin;
    ctrl_gt vcom;

    ctl_ct_iclarke(ab0, &vabc);

    /* The unshifted neutral-leg pole command is zero. */
    vmax = real2ctrl(0.0f);
    vmin = real2ctrl(0.0f);

    if (vabc.dat[phase_A] > vmax)
        vmax = vabc.dat[phase_A];
    if (vabc.dat[phase_A] < vmin)
        vmin = vabc.dat[phase_A];

    if (vabc.dat[phase_B] > vmax)
        vmax = vabc.dat[phase_B];
    if (vabc.dat[phase_B] < vmin)
        vmin = vabc.dat[phase_B];

    if (vabc.dat[phase_C] > vmax)
        vmax = vabc.dat[phase_C];
    if (vabc.dat[phase_C] < vmin)
        vmin = vabc.dat[phase_C];

    vcom = ctl_div2(vmax + vmin);

    Tabcn->dat[phase_A] = vabc.dat[phase_A] - vcom;
    Tabcn->dat[phase_B] = vabc.dat[phase_B] - vcom;
    Tabcn->dat[phase_C] = vabc.dat[phase_C] - vcom;
    Tabcn->dat[phase_N] = -vcom;
}

/**
 * @brief Calculates A/B/C/N four-leg duty ratios with a 0.5 carrier bias.
 * @note A reference is in the linear region when
 * `max(vA,vB,vC,0)-min(vA,vB,vC,0) <= 1`. No overmodulation clipping is
 * performed, matching ctl_ct_svpwm_calc().
 * @param[in] ab0 Alpha-beta-zero phase-to-neutral reference, normalized by Vdc.
 * @param[out] Tabcn A/B/C/N duty ratios.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_3d_calc(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector4_t* Tabcn)
{
    ctl_ct_svpwm_3d(ab0, Tabcn);

    Tabcn->dat[phase_A] += CTL_CTRL_CONST_1_OVER_2;
    Tabcn->dat[phase_B] += CTL_CTRL_CONST_1_OVER_2;
    Tabcn->dat[phase_C] += CTL_CTRL_CONST_1_OVER_2;
    Tabcn->dat[phase_N] += CTL_CTRL_CONST_1_OVER_2;
}

#if __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_MATH_SVPWM_3D_H_
