
#include <ctl/math_block/coordinate/coordinate.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>

#ifndef _FILE_GMP_MATH_SVPWM_H_
#define _FILE_GMP_MATH_SVPWM_H_

#if __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Calculates SVPWM duty cycles from alpha-beta reference voltages using the common-mode injection method.
 * @f[ U_a = U_\alpha @f]
 * @f[ U_b = -U_\alpha /2 + \sqrt{3}/2 U_\beta @f]
 * @f[ U_c = -U_\alpha /2 - \sqrt{3}/2 U_\beta @f]
 * @param[in] ab0 Pointer to the input alpha-beta reference voltage vector.
 * @param[out] Tabc Pointer to the output vector containing the SVPWM duty cycles for phases A, B, and C.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_calc(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* Tabc)
{
    ctrl_gt Ua, Ub, Uc, Umax, Umin, Ucom;
    ctrl_gt Ualpha_tmp = -ctl_div2(ab0->dat[phase_alpha]);
    ctrl_gt Ubeta_tmp = ctl_mul(ab0->dat[phase_beta], CTL_CTRL_CONST_SQRT_3_OVER_2);

    Ua = ab0->dat[phase_alpha];
    Ub = Ualpha_tmp + Ubeta_tmp;
    Uc = Ualpha_tmp - Ubeta_tmp;

    if (Ua > Ub)
    {
        Umax = Ua;
        Umin = Ub;
    }
    else
    {
        Umax = Ub;
        Umin = Ua;
    }
    if (Uc > Umax)
        Umax = Uc;
    else if (Uc < Umin)
        Umin = Uc;

    Ucom = ctl_div2(Umax + Umin);
    Tabc->dat[phase_A] = Ua - Ucom + CTL_CTRL_CONST_1_OVER_2;
    Tabc->dat[phase_B] = Ub - Ucom + CTL_CTRL_CONST_1_OVER_2;
    Tabc->dat[phase_C] = Uc - Ucom + CTL_CTRL_CONST_1_OVER_2;
}

/**
 * @brief Calculates SVPWM duty cycles from alpha-beta reference voltages using the common-mode injection method, without bias.
 * @f[ U_a = U_\alpha @f]
 * @f[ U_b = -U_\alpha /2 + \sqrt{3}/2 U_\beta @f]
 * @f[ U_c = -U_\alpha /2 - \sqrt{3}/2 U_\beta @f]
 * @param[in] ab0 Pointer to the input alpha-beta reference voltage vector.
 * @param[out] Tabc Pointer to the output vector containing the SVPWM duty cycles for phases A, B, and C.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* Tabc)
{
    ctrl_gt Ua, Ub, Uc, Umax, Umin, Ucom;
    ctrl_gt Ualpha_tmp = -ctl_div2(ab0->dat[phase_alpha]);
    ctrl_gt Ubeta_tmp = ctl_mul(ab0->dat[phase_beta], CTL_CTRL_CONST_SQRT_3_OVER_2);

    Ua = ab0->dat[phase_alpha];
    Ub = Ualpha_tmp + Ubeta_tmp;
    Uc = Ualpha_tmp - Ubeta_tmp;

    if (Ua > Ub)
    {
        Umax = Ua;
        Umin = Ub;
    }
    else
    {
        Umax = Ub;
        Umin = Ua;
    }
    if (Uc > Umax)
        Umax = Uc;
    else if (Uc < Umin)
        Umin = Uc;

    Ucom = ctl_div2(Umax + Umin);
    Tabc->dat[phase_A] = Ua - Ucom;
    Tabc->dat[phase_B] = Ub - Ucom;
    Tabc->dat[phase_C] = Uc - Ucom;
}

/**
 * @brief Calculates SVPWM duty cycles from alpha-beta reference voltages using a sector-based algorithm.
 * @param[in] ab0 Pointer to the input alpha-beta reference voltage vector.
 * @param[out] Tabc Pointer to the output vector containing the SVPWM duty cycles for phases A, B, and C.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_calc_theorem(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* Tabc)
{
    ctrl_gt X, Y, Z, T1, T2, Ta, Tb, Tc;
    uint16_t N;
    ctrl_gt Uabc[3] = {0};

    Uabc[0] = ab0->dat[phase_beta];
    Uabc[1] = ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);
    Uabc[2] = -ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);

    N = ((Uabc[0] > 0)) + ((Uabc[1] > 0) << 1) + ((Uabc[2] > 0) << 2);
    X = ctl_mul(CTL_CTRL_CONST_SQRT_3, ab0->dat[phase_beta]);
    Y = ctl_mul(CTL_CTRL_CONST_3_OVER_2, ab0->dat[phase_alpha]) +
        ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);
    Z = -ctl_mul(CTL_CTRL_CONST_3_OVER_2, ab0->dat[phase_alpha]) +
        ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);

    switch (N)
    {
    case 1:
        T1 = Z;
        T2 = Y;
        break;
    case 2:
        T1 = Y;
        T2 = -X;
        break;
    case 3:
        T1 = -Z;
        T2 = X;
        break;
    case 4:
        T1 = -X;
        T2 = Z;
        break;
    case 5:
        T1 = X;
        T2 = -Y;
        break;
    case 6:
        T1 = -Y;
        T2 = -Z;
        break;
    default:
        T1 = 0;
        T2 = 0;
        break;
    }

    if ((T1 + T2) > CTL_CTRL_CONST_1)
    {
        T1 = ctl_div(T1, (T1 + T2));
        T2 = CTL_CTRL_CONST_1 - T1;
    }

    Ta = ctl_div4(CTL_CTRL_CONST_1 - T1 - T2);
    Tb = Ta + ctl_div2(T1);
    Tc = Tb + ctl_div2(T2);
    Ta = -ctl_mul2(Ta) + CTL_CTRL_CONST_1_OVER_2;
    Tb = -ctl_mul2(Tb) + CTL_CTRL_CONST_1_OVER_2;
    Tc = -ctl_mul2(Tc) + CTL_CTRL_CONST_1_OVER_2;

    switch (N)
    {
    case 1:
        Tabc->dat[phase_A] = Tb;
        Tabc->dat[phase_B] = Ta;
        Tabc->dat[phase_C] = Tc;
        break;
    case 2:
        Tabc->dat[phase_A] = Ta;
        Tabc->dat[phase_B] = Tc;
        Tabc->dat[phase_C] = Tb;
        break;
    case 3:
        Tabc->dat[phase_A] = Ta;
        Tabc->dat[phase_B] = Tb;
        Tabc->dat[phase_C] = Tc;
        break;
    case 4:
        Tabc->dat[phase_A] = Tc;
        Tabc->dat[phase_B] = Tb;
        Tabc->dat[phase_C] = Ta;
        break;
    case 5:
        Tabc->dat[phase_A] = Tc;
        Tabc->dat[phase_B] = Ta;
        Tabc->dat[phase_C] = Tb;
        break;
    case 6:
        Tabc->dat[phase_A] = Tb;
        Tabc->dat[phase_B] = Tc;
        Tabc->dat[phase_C] = Ta;
        break;
    default:
        ctl_vector3_clear(Tabc);
        break;
    }
}

#if __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_MATH_SVPWM_H_

