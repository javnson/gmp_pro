/**
 * @file coord_trans.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides coordinate transformation functions for motor control.
 * @version 1.1
 * @date 2025-07-23
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file contains essential coordinate transformations used in Field-Oriented
 * Control (FOC), including Clarke, Park, and their inverse transformations, as
 * well as SVPWM calculation routines.
 */

#ifndef _FILE_COORD_TRANS_H_
#define _FILE_COORD_TRANS_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define GMP_CTL_OUTPUT_TAG

/*---------------------------------------------------------------------------*/
/* Coordinate Axis Definitions                                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_COORD_AXIS_DEFINES Coordinate Axis Definitions
 * @ingroup CTL_MATH
 * @brief Enumerations for indexing different coordinate system axes.
 * @{
 */

/** @brief Enumeration for three-phase system axes (U, V, W). */
enum UVW_ASIX_ENUM
{
    phase_U = 0,
    phase_V = 1,
    phase_W = 2
};

/** @brief Enumeration for three-phase system axes (A, B, C). */
enum ABC_ASIX_ENUM
{
    phase_A = 0,
    phase_B = 1,
    phase_C = 2
};

/** @brief Enumeration for rotating reference frame axes (d, q, 0). */
enum DQ_ASIC_ENUM
{
    phase_d = 0,
    phase_q = 1,
    phase_0 = 2
};

/** @brief Enumeration for stationary reference frame axes (alpha, beta). */
enum ALPHA_BETA_ENUM
{
    phase_alpha = 0,
    phase_beta = 1
};

/** @brief Enumeration for line voltage phases (Uab, Ubc). */
enum LINE_VOLTAGE_ENUM
{
    phase_UAB = 0,
    phase_UBC = 1
};

/** @brief Enumeration for phasor components (sin, cos). */
enum PHASOR_ENUM
{
    phasor_sin = 0,
    phasor_cos = 1
};

/** @} */ // end of MC_COORD_AXIS_DEFINES group

/*---------------------------------------------------------------------------*/
/* Coordinate Transformation Functions                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_COORD_TRANSFORMATIONS Coordinate Transformations
 * @ingroup CTL_MATH
 * @brief A collection of functions for coordinate system transformations.
 * @{
 */

/**
 * @brief Generates a 2D phasor (sin, cos) from a given angle.
 * @param[in] angle The input angle in per-unit (0 to 1 represents 0 to 2pi).
 * @param[out] phasor Pointer to the output 2D vector to store the phasor.
 */
GMP_STATIC_INLINE void ctl_set_phasor_via_angle(ctrl_gt angle, GMP_CTL_OUTPUT_TAG ctl_vector2_t* phasor)
{
    phasor->dat[phasor_sin] = ctl_sin(angle);
    phasor->dat[phasor_cos] = ctl_cos(angle);
}

/**
 * @brief Performs the Clarke transformation from a 3-phase (ABC) to a 2-phase stationary (alpha-beta) reference frame.
 * @f[ i_\alpha = \frac{2}{3} i_a - \frac{1}{3} i_b - \frac{1}{3} i_c @f]
 * @f[ i_\beta = \frac{1}{\sqrt{3}} i_b - \frac{1}{\sqrt{3}} i_c @f]
 * @f[ i_0 = \frac{1}{3} (i_a + i_b + i_c) @f]
 * @param[in] abc Pointer to the input 3-phase vector.
 * @param[out] ab Pointer to the output alpha-beta-0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_clarke(ctl_vector3_t* abc, GMP_CTL_OUTPUT_TAG ctl_vector3_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(CTL_CTRL_CONST_ABC2AB_ALPHA, abc->dat[phase_A] - ctl_div2(abc->dat[phase_B] + abc->dat[phase_C]));
    ab->dat[phase_beta] = ctl_mul(CTL_CTRL_CONST_ABC2AB_BETA, (abc->dat[phase_B] - abc->dat[phase_C]));
    ab->dat[phase_0] = ctl_mul(CTL_CTRL_CONST_ABC2AB_GAMMA, abc->dat[phase_A] + abc->dat[phase_B] + abc->dat[phase_C]);
}

/**
 * @brief Performs the Clarke transformation from 2-phase currents (A, B) assuming a balanced three-phase system (ia+ib+ic=0).
 * @f[ i_\alpha = i_a @f]
 * @f[ i_\beta = \frac{1}{\sqrt{3}} (i_a + 2i_b) @f]
 * @param[in] ab0 Pointer to the input vector containing phase A and B currents.
 * @param[out] ab Pointer to the output alpha-beta vector.
 */
GMP_STATIC_INLINE void ctl_ct_clarke_2ph(ctl_vector2_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] = ab0->dat[phase_A];
    ab->dat[phase_beta] = ctl_mul(CTL_CTRL_CONST_ABC2AB_BETA, (ab0->dat[phase_A] + ctl_mul2(ab0->dat[phase_B])));
}

/**
 * @brief Performs the Clarke transformation from line voltages (Uab, Ubc).
 * @f[ U_\alpha = \frac{1}{3}(2 U_{ab} + U_{bc}) @f]
 * @f[ U_\beta = \frac{1}{\sqrt{3}} U_{bc} @f]
 * @param[in] u_line Pointer to the input vector containing line voltages Uab and Ubc.
 * @param[out] ab Pointer to the output alpha-beta vector.
 */
GMP_STATIC_INLINE void ctl_ct_clarke_from_line(ctl_vector2_t* u_line, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] = ctl_mul(CTL_CTRL_CONST_ABC2AB_GAMMA, ctl_mul2(u_line->dat[phase_UAB]) + u_line->dat[phase_UBC]);
    ab->dat[phase_beta] = ctl_mul(CTL_CTRL_CONST_ABC2AB_BETA, u_line->dat[phase_UBC]);
}

/**
 * @brief Performs the Park transformation from a stationary (alpha-beta) to a rotating (dq) reference frame.
 * @f[ i_d = i_\alpha \cos(\theta) + i_\beta \sin(\theta) @f]
 * @f[ i_q = -i_\alpha \sin(\theta) + i_\beta \cos(\theta) @f]
 * @f[ i_0 = i_0 @f]
 * @param[in] ab Pointer to the input alpha-beta-0 vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] dq0 Pointer to the output dq0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_park(ctl_vector3_t* ab, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector3_t* dq0)
{
    dq0->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    dq0->dat[phase_q] =
        -ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
    dq0->dat[phase_0] = ab->dat[phase_0];
}

/**
 * @brief Performs the Park transformation for 2D vectors (alpha-beta to dq).
 * @param[in] ab Pointer to the input alpha-beta vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] dq Pointer to the output dq vector.
 */
GMP_STATIC_INLINE void ctl_ct_park2(ctl_vector2_t* ab, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector2_t* dq)
{
    dq->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    dq->dat[phase_q] =
        -ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
}

/**
 * @brief Performs the negative sequence Park transformation.
 * @f[ i_d = i_\alpha \cos(\theta) - i_\beta \sin(\theta) @f]
 * @f[ i_q = i_\alpha \sin(\theta) + i_\beta \cos(\theta) @f]
 * @param[in] ab Pointer to the input alpha-beta-0 vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] dq0_neg Pointer to the output negative sequence dq0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_park_neg(ctl_vector3_t* ab, ctl_vector2_t* phasor,
                                       GMP_CTL_OUTPUT_TAG ctl_vector3_t* dq0_neg)
{
    dq0_neg->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) - ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    dq0_neg->dat[phase_q] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
    dq0_neg->dat[phase_0] = ab->dat[phase_0];
}

/**
 * @brief Performs the negative sequence Park transformation for 2D vectors.
 * @param[in] ab Pointer to the input alpha-beta vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] dq0 Pointer to the output negative sequence dq vector.
 */
GMP_STATIC_INLINE void ctl_ct_park2_neg(ctl_vector2_t* ab, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector2_t* dq0)
{
    dq0->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) - ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    dq0->dat[phase_q] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
}

/**
 * @brief Performs the inverse Park transformation from a rotating (dq) to a stationary (alpha-beta) reference frame.
 * @f[ i_\alpha = i_d \cos(\theta) - i_q \sin(\theta) @f]
 * @f[ i_\beta = i_d \sin(\theta) + i_q \cos(\theta) @f]
 * @f[ i_0 = i_0 @f]
 * @param[in] dq0 Pointer to the input dq0 vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] ab Pointer to the output alpha-beta-0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_ipark(ctl_vector3_t* dq0, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector3_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_cos]) - ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_sin]);
    ab->dat[phase_beta] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_cos]);
    ab->dat[phase_0] = dq0->dat[phase_0];
}

/**
 * @brief Performs the inverse Park transformation for 2D vectors (dq to alpha-beta).
 * @param[in] dq Pointer to the input dq vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] ab Pointer to the output alpha-beta vector.
 */
GMP_STATIC_INLINE void ctl_ct_ipark2(ctl_vector2_t* dq, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_cos]) - ctl_mul(dq->dat[phase_q], phasor->dat[phasor_sin]);
    ab->dat[phase_beta] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_cos]);
}

/**
 * @brief Performs the negative sequence inverse Park transformation.
 * @f[ i_\alpha = i_d \cos(\theta) + i_q \sin(\theta) @f]
 * @f[ i_\beta = -i_d \sin(\theta) + i_q \cos(\theta) @f]
 * @param[in] dq0 Pointer to the input dq0 vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] ab Pointer to the output alpha-beta-0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_ipark_neg(ctl_vector3_t* dq0, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector3_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_cos]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_sin]);
    ab->dat[phase_beta] =
        -ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_cos]);
    ab->dat[phase_0] = dq0->dat[phase_0];
}

/**
 * @brief Performs the negative sequence inverse Park transformation for 2D vectors.
 * @param[in] dq Pointer to the input dq vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] ab Pointer to the output alpha-beta vector.
 */
GMP_STATIC_INLINE void ctl_ct_ipark2_neg(ctl_vector2_t* dq, ctl_vector2_t* phasor, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_cos]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_sin]);
    ab->dat[phase_beta] =
        -ctl_mul(dq->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_cos]);
}

/**
 * @brief Performs the inverse Clarke transformation from a 2-phase stationary (alpha-beta) to a 3-phase (ABC) reference frame.
 * @f[ i_a = i_\alpha + i_0 @f]
 * @f[ i_b = -0.5 i_\alpha + \frac{\sqrt{3}}{2} i_\beta + i_0 @f]
 * @f[ i_c = -0.5 i_\alpha - \frac{\sqrt{3}}{2} i_\beta + i_0 @f]
 * @param[in] ab0 Pointer to the input alpha-beta-0 vector.
 * @param[out] abc Pointer to the output 3-phase vector.
 */
GMP_STATIC_INLINE void ctl_ct_iclarke(ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* abc)
{
    ctrl_gt neg_half_alpha = -ctl_div2(ab0->dat[phase_alpha]);
    ctrl_gt beta_term = ctl_mul(CTL_CTRL_CONST_AB2ABC_ALPHA, ab0->dat[phase_beta]);
    abc->dat[phase_A] = ab0->dat[phase_alpha] + ab0->dat[phase_0];
    abc->dat[phase_B] = neg_half_alpha + beta_term + ab0->dat[phase_0];
    abc->dat[phase_C] = neg_half_alpha - beta_term + ab0->dat[phase_0];
}

/**
 * @brief Performs the inverse Clarke transformation for 2D vectors (alpha-beta to ABC), assuming zero sequence is zero.
 * @f[ i_a = i_\alpha @f]
 * @f[ i_b = -0.5 i_\alpha + \frac{\sqrt{3}}{2} i_\beta @f]
 * @f[ i_c = -0.5 i_\alpha - \frac{\sqrt{3}}{2} i_\beta @f]
 * @param[in] ab Pointer to the input alpha-beta vector.
 * @param[out] abc Pointer to the output 3-phase vector.
 */
GMP_STATIC_INLINE void ctl_ct_iclarke2(ctl_vector2_t* ab, GMP_CTL_OUTPUT_TAG ctl_vector3_t* abc)
{
    ctrl_gt neg_half_alpha = -ctl_div2(ab->dat[phase_alpha]);
    ctrl_gt beta_term = ctl_mul(CTL_CTRL_CONST_AB2ABC_ALPHA, ab->dat[phase_beta]);
    abc->dat[phase_A] = ab->dat[phase_alpha];
    abc->dat[phase_B] = neg_half_alpha + beta_term;
    abc->dat[phase_C] = neg_half_alpha - beta_term;
}

/**
 * @brief Transforms stationary frame (alpha-beta) voltages to line voltages (Uab, Ubc).
 * @f[ U_{ab} = U_\alpha @f]
 * @f[ U_{bc} = \frac{1}{2}(-U_\alpha + \sqrt{3}U_\beta) @f]
 * @param[in] ab Pointer to the input alpha-beta vector.
 * @param[out] u_line Pointer to the output vector containing line voltages.
 */
GMP_STATIC_INLINE void ctl_ct_iclarke_to_line(ctl_vector3_t* ab, GMP_CTL_OUTPUT_TAG ctl_vector2_t* u_line)
{
    u_line->dat[phase_UAB] = ab->dat[phase_alpha];
    u_line->dat[phase_UBC] = ctl_div2(-ab->dat[phase_alpha] + ctl_mul(CTL_CTRL_CONST_SQRT_3, ab->dat[phase_beta]));
}

/**
 * @brief Calculates SVPWM duty cycles from alpha-beta reference voltages using the common-mode injection method.
 * @f[ U_a = U_\alpha @f]
 * @f[ U_b = -U_\alpha /2 + \sqrt{3}/2 U_\beta @f]
 * @f[ U_c = -U_\alpha /2 - \sqrt{3}/2 U_\beta @f]
 * @param[in] ab0 Pointer to the input alpha-beta reference voltage vector.
 * @param[out] Tabc Pointer to the output vector containing the SVPWM duty cycles for phases A, B, and C.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_calc(ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* Tabc)
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
 * @brief Calculates SVPWM duty cycles from alpha-beta reference voltages using a sector-based algorithm.
 * @param[in] ab0 Pointer to the input alpha-beta reference voltage vector.
 * @param[out] Tabc Pointer to the output vector containing the SVPWM duty cycles for phases A, B, and C.
 */
GMP_STATIC_INLINE void ctl_ct_svpwm_calc_theorem(ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* Tabc)
{
    ctrl_gt X, Y, Z, T1, T2, Ta, Tb, Tc;
    uint16_t N;
    ctrl_gt Uabc[3] = {0};

    Uabc[0] = ab0->dat[phase_beta];
    Uabc[1] = ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);
    Uabc[2] = -ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);

    N = ((Uabc[0] > 0)) + ((Uabc[1] > 0) << 1) + ((Uabc[2] > 0) << 2);
    X = ctl_mul(CTL_CTRL_CONST_SQRT_3, ab0->dat[phase_beta]);
    Y = ctl_mul(CTL_CTRL_CONST_3_OVER_2, ab0->dat[phase_alpha]) + ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);
    Z = -ctl_mul(CTL_CTRL_CONST_3_OVER_2, ab0->dat[phase_alpha]) + ctl_mul(CTL_CTRL_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);

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

/** @} */ // end of MC_COORD_TRANSFORMATIONS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_COORD_TRANS_H_
