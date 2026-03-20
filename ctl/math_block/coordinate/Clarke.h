
#include <ctl/math_block/coordinate/coordinate.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>

#ifndef _FILE_GMP_MATH_CLARKE_H_
#define _FILE_GMP_MATH_CLARKE_H_

#if __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Performs the Clarke transformation from a 3-phase (ABC) to a 2-phase stationary (alpha-beta) reference frame.
 * @f[ i_\alpha = \frac{2}{3} i_a - \frac{1}{3} i_b - \frac{1}{3} i_c @f]
 * @f[ i_\beta = \frac{1}{\sqrt{3}} i_b - \frac{1}{\sqrt{3}} i_c @f]
 * @f[ i_0 = \frac{1}{3} (i_a + i_b + i_c) @f]
 * @param[in] abc Pointer to the input 3-phase vector.
 * @param[out] ab Pointer to the output alpha-beta-0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_clarke(const ctl_vector3_t* abc, GMP_CTL_OUTPUT_TAG ctl_vector3_t* ab)
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
GMP_STATIC_INLINE void ctl_ct_clarke_2ph(const ctl_vector2_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
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
GMP_STATIC_INLINE void ctl_ct_clarke_from_line(const ctl_vector2_t* u_line, GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(CTL_CTRL_CONST_ABC2AB_GAMMA, ctl_mul2(u_line->dat[phase_UAB]) + u_line->dat[phase_UBC]);
    ab->dat[phase_beta] = ctl_mul(CTL_CTRL_CONST_ABC2AB_BETA, u_line->dat[phase_UBC]);
}

/**
 * @brief Performs the inverse Clarke transformation from a 2-phase stationary (alpha-beta) to a 3-phase (ABC) reference frame.
 * @f[ i_a = i_\alpha + i_0 @f]
 * @f[ i_b = -0.5 i_\alpha + \frac{\sqrt{3}}{2} i_\beta + i_0 @f]
 * @f[ i_c = -0.5 i_\alpha - \frac{\sqrt{3}}{2} i_\beta + i_0 @f]
 * @param[in] ab0 Pointer to the input alpha-beta-0 vector.
 * @param[out] abc Pointer to the output 3-phase vector.
 */
GMP_STATIC_INLINE void ctl_ct_iclarke(const ctl_vector3_t* ab0, GMP_CTL_OUTPUT_TAG ctl_vector3_t* abc)
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
GMP_STATIC_INLINE void ctl_ct_iclarke2(const ctl_vector2_t* ab, GMP_CTL_OUTPUT_TAG ctl_vector3_t* abc)
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
GMP_STATIC_INLINE void ctl_ct_iclarke_to_line(const ctl_vector3_t* ab, GMP_CTL_OUTPUT_TAG ctl_vector2_t* u_line)
{
    u_line->dat[phase_UAB] = ab->dat[phase_alpha];
    u_line->dat[phase_UBC] = ctl_div2(-ab->dat[phase_alpha] + ctl_mul(CTL_CTRL_CONST_SQRT_3, ab->dat[phase_beta]));
}

#if __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_MATH_CLARKE_H_
