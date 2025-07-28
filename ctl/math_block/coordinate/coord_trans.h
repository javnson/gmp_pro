/**
 * @file coord_trans.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief
 * @version 1.0
 * @date 2025-07-23
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

// This file provide the coordinate transformations

#ifndef _FILE_COORD_TRANS_H_
#define _FILE_COORD_TRANS_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#define GMP_CTL_OUTPUT_TAG

enum UVW_ASIX_ENUM
{
    phase_U = 0,
    phase_V = 1,
    phase_W = 2
};

enum ABC_ASIX_ENUM
{
    phase_A = 0,
    phase_B = 1,
    phase_C = 2
};

enum DQ_ASIC_ENUM
{
    phase_d = 0,
    phase_q = 1,
    phase_0 = 2
};

enum ALPHA_BETA_ENUM
{
    phase_alpha = 0,
    phase_beta = 1
};
enum LINE_VOLTAGE_ENUM
{
    phase_UAB = 0,
    phase_UBC = 1
};

enum PHASOR_ENUM
{
    phasor_sin = 0,
    phasor_cos = 1
};

// phasor
// dat[0]: sin
// dat[1]: cos

// ABC
// dat[0]: A
// dat[1]: B
// dat[2]: C

// AB
// dat[0]: alpha
// dat[1]: beta
// dat[2]: 0

// dq0
// dat[0]: d
// dat[1]: q
// dat[2]: 0

// clang-format off
// 

// angle to 2-d phasor
GMP_STATIC_INLINE
void ctl_set_phasor_via_angle(
    // angle
    ctrl_gt angle,
    // output phasor
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *phasor)
{
    phasor->dat[phasor_sin] = ctl_sin(angle);
    phasor->dat[phasor_cos] = ctl_cos(angle);
}

// Clark coordinate axes transform
// ABC to alpha-beta
// Constant amplitude transformation
GMP_STATIC_INLINE
void ctl_ct_clark(
    // ABC
    ctl_vector3_t *abc,
    // output alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *ab)
{
    ctrl_gt k_alpha = GMP_CONST_ABC2AB_ALPHA; // 2/3
    ctrl_gt k_beta = GMP_CONST_ABC2AB_BETA;   // 1/sqrt(3)
    ctrl_gt k_gamma = GMP_CONST_ABC2AB_GAMMA; // 1/3

    //tex:
    // $$i_\alpha = 2/3\times i_a - 1/3 \times i_b - 1/3 \times i_c $$
    ab->dat[phase_alpha] = ctl_mul(k_alpha, abc->dat[phase_A] - ctl_div2(abc->dat[phase_B] + abc->dat[phase_C]));
    //tex:
    // $$i_\beta = \sqrt{3}/3\times i_b - \sqrt{3}/3\times i_c $$
    ab->dat[phase_beta] = ctl_mul(k_beta, (abc->dat[phase_B] - abc->dat[phase_C]));
    //tex:
    // $$i_\beta = 1/3\times i_a + 1/3\times i_b + 1/3\times i_c $$
    ab->dat[phase_0] = ctl_mul(k_gamma, abc->dat[phase_A] + abc->dat[phase_B] + abc->dat[phase_C]);
}

// Clark coordinate axes transform
// AB0(0 means C doesn't measure)
GMP_STATIC_INLINE
void ctl_ct_clark_2ph(
    // A and B phase voltage/current
    ctl_vector2_t *ab0,
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *ab)
{
    ctrl_gt k_alpha = GMP_CONST_AB02AB_ALPHA;

    //tex:
    //  $$i_\alpha = i_a $$
    ab->dat[phase_alpha] = ab0->dat[phase_A];
    //tex:
    // $$i_\beta = 2/\sqrt{3}\times (i_a/2 + i_b) $$
    ab->dat[phase_beta] = (ctrl_gt)ctl_mul(ctl_div2(ab0->dat[phase_A]) + ab0->dat[phase_B], k_alpha);
}

// Clark coordinate axes transform
//  uAB & uBC is line voltage
GMP_STATIC_INLINE
void ctl_ct_clark_from_line(
    // uAB, uBC
    ctl_vector2_t *u_line, 
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *ab)
{
    ctrl_gt k_beta = GMP_CONST_ABC2AB_BETA;   // 1/sqrt(3)
    ctrl_gt k_gamma = GMP_CONST_ABC2AB_GAMMA; // 1/3

    //tex:
    // $$U_\alpha = \frac{1}{3}(2 U_{ab} + U_{bc})$$
    ab->dat[phase_alpha] = ctl_mul(k_gamma, ctl_mul2(u_line->dat[phase_UAB]) + u_line->dat[phase_UBC]);

    //tex:
    // $$U_\beta = \frac{1}{\sqrt{3}} U_{bc}$$
    ab->dat[phase_beta] = ctl_mul(k_beta, u_line->dat[phase_UBC]);
}

// park coordinate axes transform
// alpha-beta-0 to dq0
GMP_STATIC_INLINE
void ctl_ct_park(
    // alpha beta 0
    ctl_vector3_t *ab, 
    // phasor
    ctl_vector2_t *phasor, 
    // dq0
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *dq0)
{
    //tex:
    // $$i_d = i_\alpha \times cos\;(\theta) + i_\beta \times sin\;(\theta) $$
    dq0->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    //tex:
    // $$i_q = - i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
    dq0->dat[phase_q] =
        -ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
    //tex:
    // $$i_0 = i_0$$
    dq0->dat[phase_0] = ab->dat[phase_0];
}

// park coordinate axes transform
// alpha-beta to dq
GMP_STATIC_INLINE
void ctl_ct_park2(
    // alpha beta
    ctl_vector2_t *ab,
    // phasor
    ctl_vector2_t *phasor,
    // dq0
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *dq0)
{
    //tex:
    // $$i_d = i_\alpha \times cos\;(\theta) + i_\beta \times sin\;(\theta) $$
    dq0->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    //tex:
    // $$i_q = - i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
    dq0->dat[phase_q] =
        -ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
}

// park coordinate axes transform
// alpha-beta-0 to dq0
GMP_STATIC_INLINE
void ctl_ct_park_neg(
    // alpha beta 0
    ctl_vector3_t *ab, 
    // phasor
    ctl_vector2_t *phasor, 
    // dq0
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *dq0_neg)
{
    //tex:
    // $$i_d = i_\alpha \times cos\;(\theta) - i_\beta \times sin\;(\theta) $$
    dq0_neg->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) - ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    //tex:
    // $$i_q =  i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
    dq0_neg->dat[phase_q] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
    //tex:
    // $$i_0 = i_0$$
    dq0_neg->dat[phase_0] = ab->dat[phase_0];
}

// park coordinate axes transform
// alpha-beta to dq
GMP_STATIC_INLINE
void ctl_ct_park2_neg(
    // alpha beta
    ctl_vector2_t *ab,
    // phasor
    ctl_vector2_t *phasor,
    // dq0
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *dq0)
{
    //tex:
    // $$i_d = i_\alpha \times cos\;(\theta) - i_\beta \times sin\;(\theta) $$
    dq0->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) - ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    //tex:
    // $$i_q = i_\alpha \times sin\;(\theta) + i_\beta \times cos\;(\theta)$$
    dq0->dat[phase_q] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
}

// ipark coordinate axes transform
// DQ to alpha_beta
GMP_STATIC_INLINE
void ctl_ct_ipark(
    // dq0
    ctl_vector3_t *dq0,
    // phasor
    ctl_vector2_t *phasor,
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *ab)
{
    //tex:
    // $$i_\alpha = i_d \times \cos\;(\theta) - i_q \times \sin\;(\theta) $$
    ab->dat[phase_alpha] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_cos]) - ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_sin]);
    //tex:
    // $$ i_\beta = i_d \times \sin\;(\theta) + i_q \times \cos\;(\theta) $$
    ab->dat[phase_beta] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_cos]);
    //tex:
    // $$i_0 = i_0$$
    ab->dat[phase_0] = dq0->dat[phase_0];
}

// ipark coordinate axes transform
// DQ to alpha_beta
GMP_STATIC_INLINE
void ctl_ct_ipark2(
    // dq0
    ctl_vector2_t *dq,
    // phasor
    ctl_vector2_t *phasor,
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *ab)
{
    //tex:
    // $$i_\alpha = i_d \times \cos\;(\theta) - i_q \times \sin\;(\theta) $$
    ab->dat[phase_alpha] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_cos]) - ctl_mul(dq->dat[phase_q], phasor->dat[phasor_sin]);
    //tex:
    // $$ i_\beta = i_d \times \sin\;(\theta) + i_q \times \cos\;(\theta) $$
    ab->dat[phase_beta] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_cos]);
}

// ipark coordinate axes transform
// DQ to alpha_beta
GMP_STATIC_INLINE
void ctl_ct_ipark_neg(
    // dq0
    ctl_vector3_t *dq0,
    // phasor
    ctl_vector2_t *phasor,
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *ab)
{
    //tex:
    // $$i_\alpha = i_d \times \cos\;(\theta) + i_q \times \sin\;(\theta) $$
    ab->dat[phase_alpha] =
        ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_cos]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_sin]);
    //tex:
    // $$ i_\beta = -i_d \times \sin\;(\theta) + i_q \times \cos\;(\theta) $$
    ab->dat[phase_beta] =
        -ctl_mul(dq0->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq0->dat[phase_q], phasor->dat[phasor_cos]);
    //tex:
    // $$i_0 = i_0$$
    ab->dat[phase_0] = dq0->dat[phase_0];
}

// ipark coordinate axes transform
// DQ to alpha_beta
GMP_STATIC_INLINE
void ctl_ct_ipark2_neg(
    // dq0
    ctl_vector2_t *dq,
    // phasor
    ctl_vector2_t *phasor,
    // alpha beta
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *ab)
{
    //tex:
    // $$i_\alpha = i_d \times \cos\;(\theta) + i_q \times \sin\;(\theta) $$
    ab->dat[phase_alpha] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_cos]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_sin]);
    //tex:
    // $$ i_\beta = -i_d \times \sin\;(\theta) + i_q \times \cos\;(\theta) $$
    ab->dat[phase_beta] =
        -ctl_mul(dq->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_cos]);
}


// iClarke coordinate axes transform
GMP_STATIC_INLINE
void ctl_ct_iclark(
    // alpha beta 0
    ctl_vector3_t *ab0,
    // ABC
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *abc)
{
    // calculate intermediate terms for efficiency and clarity.
    ctrl_gt neg_half_alpha = -ctl_div2(ab0->dat[phase_alpha]);
    ctrl_gt beta_term = ctl_mul(GMP_CONST_AB2ABC_ALPHA, ab0->dat[phase_beta]);

    //tex:
    // $$i_a = i_\alpha + i_0$$
    abc->dat[phase_A] = ab0->dat[phase_alpha] + ab0->dat[phase_0];

    //tex:
    // $$i_b = -0.5 \times i_\alpha + \frac{\sqrt{3}}{2} \times i_\beta + i_0$$
    abc->dat[phase_B] = neg_half_alpha + beta_term + ab0->dat[phase_0];

    //tex:
    // $$i_c = -0.5 \times i_\alpha - \frac{\sqrt{3}}{2} \times i_\beta + i_0$$
    abc->dat[phase_C] = neg_half_alpha - beta_term + ab0->dat[phase_0];
}

// iClark coordinate axes transform
// sine modulation
GMP_STATIC_INLINE
void ctl_ct_iclark2(
    // alpha beta
    ctl_vector2_t *ab,
    // ABC
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *abc)
{
    // calculate intermediate terms for efficiency and clarity.
    ctrl_gt neg_half_alpha = -ctl_div2(ab->dat[phase_alpha]);
    ctrl_gt beta_term = ctl_mul(GMP_CONST_AB2ABC_ALPHA, ab->dat[phase_beta]);

    //tex:
    // $$i_a = i_\alpha$$
    abc->dat[phase_A] = ab->dat[phase_alpha];

    //tex:
    // $$i_b = -0.5 \times i_\alpha + \frac{\sqrt{3}}{2} \times i_\beta$$
    abc->dat[phase_B] = neg_half_alpha + beta_term;

    //tex:
    // $$i_c = -0.5 \times i_\alpha - \frac{\sqrt{3}}{2} \times i_\beta$$
    abc->dat[phase_C] = neg_half_alpha - beta_term;
}

// Clark transformation to Uab and Ubc
GMP_STATIC_INLINE
void ctl_ct_iclark_to_line(
    // alpha & beta
    ctl_vector3_t *ab,
    // AB & BC line voltage
    GMP_CTL_OUTPUT_TAG ctl_vector2_t *u_line)
{
    // Constants required for the transformation
    const ctrl_gt k_three_halves = GMP_CONST_3_OVER_2;     // 3/2
    const ctrl_gt k_sqrt3_div_2 = GMP_CONST_SQRT_3_OVER_2; // sqrt(3)/2
    const ctrl_gt k_sqrt3 = GMP_CONST_SQRT_3;              // sqrt(3)

    //tex:
    // $$U_{ab} = \frac{3}{2} U_\alpha - \frac{\sqrt{3}}{2} U_\beta$$
    u_line->dat[phase_UAB] =
        ctl_sub(ctl_mul(k_three_halves, ab->dat[phase_alpha]), ctl_mul(k_sqrt3_div_2, ab->dat[phase_beta]));

    //tex:
    // $$U_{bc} = \sqrt{3} U_\beta$$
    u_line->dat[phase_UBC] = ctl_mul(k_sqrt3, ab->dat[phase_beta]);
}

//
// clang-format on

// uab0(alpha-beta-0) to uabc SVPWM modulation result
GMP_STATIC_INLINE
void ctl_ct_svpwm_calc(
    // alpha - beta
    ctl_vector3_t *ab0,
    // modulation result
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *Tabc)
{
    ctrl_gt Ua, Ub, Uc; // Uabc three phase parameters
    ctrl_gt Umax, Umin, Ucom;

    ctrl_gt Ualpha_tmp = -ctl_div2(ab0->dat[phase_alpha]);
    ctrl_gt Ubeta_tmp = ctl_mul(ab0->dat[phase_beta], GMP_CONST_SQRT_3_OVER_2);

    // tex:
    //  $$
    //  U_a = U_\alpha, \\
    // U_b = -U_\alpha /2 + \sqrt{3}/2\cdot U_\beta, \\
    // U_c = -U_\alpha /2 - \sqrt{3}/2\cdot U_\beta
    //  $$

    Ua = ab0->dat[phase_alpha];
    Ub = Ualpha_tmp + Ubeta_tmp;
    Uc = Ualpha_tmp - Ubeta_tmp;

    // find Vmax & Vmin
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

    // get common mode term
    Ucom = ctl_div2(Umax + Umin);

    // get SVPWM modulation result
    Tabc->dat[phase_A] = Ua - Ucom + GMP_CONST_1_OVER_2;
    Tabc->dat[phase_B] = Ub - Ucom + GMP_CONST_1_OVER_2;
    Tabc->dat[phase_C] = Uc - Ucom + GMP_CONST_1_OVER_2;
}

// uab0(alpha-beta-0) to uabc SVPWM modulation result
// This an implementation based on theorem.
GMP_STATIC_INLINE
void ctl_ct_svpwm_calc_theorem(
    // alpha beta
    ctl_vector3_t *ab0,
    // modulation result
    GMP_CTL_OUTPUT_TAG ctl_vector3_t *Tabc)
{
    // u2s: Ualpha ,Ubeta
    ctrl_gt X, Y, Z, T1, T2, Ta, Tb, Tc;
    uint16_t N;
    ctrl_gt Uabc[3] = {0};

    Uabc[0] = ab0->dat[phase_beta];
    Uabc[1] = ctl_mul(GMP_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);
    Uabc[2] = -ctl_mul(GMP_CONST_SQRT_3_OVER_2, ab0->dat[phase_alpha]) - ctl_div2(ab0->dat[phase_beta]);

    N = ((Uabc[0] > 0)) + ((Uabc[1] > 0) << 1) + ((Uabc[2] > 0) << 2);
    X = ctl_mul(GMP_CONST_SQRT_3, ab0->dat[phase_beta]);
    Y = ctl_mul(GMP_CONST_3_OVER_2, ab0->dat[phase_alpha]) + ctl_mul(GMP_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);
    Z = -ctl_mul(GMP_CONST_3_OVER_2, ab0->dat[phase_alpha]) + ctl_mul(GMP_CONST_SQRT_3_OVER_2, ab0->dat[phase_beta]);

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
    if ((T1 + T2) > GMP_CONST_1)
    {
        T1 = ctl_div(T1, (T1 + T2));
        T2 = GMP_CONST_1 - T1;
    }
    Ta = ctl_div4(GMP_CONST_1 - T1 - T2);
    Tb = Ta + ctl_div2(T1);
    Tc = Tb + ctl_div2(T2);

    Ta *= 2;
    Tb *= 2;
    Tc *= 2;

    Ta = -Ta;
    Tb = -Tb;
    Tc = -Tc;

    Ta += GMP_CONST_1_OVER_2;
    Tb += GMP_CONST_1_OVER_2;
    Tc += GMP_CONST_1_OVER_2;

    switch (N)
    {
    case 0:
        Tabc->dat[phase_A] = 0;
        Tabc->dat[phase_B] = 0;
        Tabc->dat[phase_C] = 0;
        break;
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
        Tabc->dat[phase_A] = 0;
        Tabc->dat[phase_B] = 0;
        Tabc->dat[phase_C] = 0;
        break;
    }

    return;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_COORD_TRANS_H_
