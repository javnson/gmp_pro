
#include <ctl/math_block/coordinate/coordinate.h>
#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>

#ifndef _FILE_GMP_MATH_PARK_H_
#define _FILE_GMP_MATH_PARK_H_

#if __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @brief Performs the Park transformation from a stationary (alpha-beta) to a rotating (dq) reference frame.
 * @f[ i_d = i_\alpha \cos(\theta) + i_\beta \sin(\theta) @f]
 * @f[ i_q = -i_\alpha \sin(\theta) + i_\beta \cos(\theta) @f]
 * @f[ i_0 = i_0 @f]
 * @param[in] ab Pointer to the input alpha-beta-0 vector.
 * @param[in] phasor Pointer to the phasor vector (sin, cos of the angle).
 * @param[out] dq0 Pointer to the output dq0 vector.
 */
GMP_STATIC_INLINE void ctl_ct_park(const ctl_vector3_t* ab, const ctl_vector2_t* phasor,
                                   GMP_CTL_OUTPUT_TAG ctl_vector3_t* dq0)
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
GMP_STATIC_INLINE void ctl_ct_park2(const ctl_vector2_t* ab, const ctl_vector2_t* phasor,
                                    GMP_CTL_OUTPUT_TAG ctl_vector2_t* dq)
{
    dq->dat[phase_d] =
        ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_cos]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_sin]);
    dq->dat[phase_q] =
        -ctl_mul(ab->dat[phase_alpha], phasor->dat[phasor_sin]) + ctl_mul(ab->dat[phase_beta], phasor->dat[phasor_cos]);
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
GMP_STATIC_INLINE void ctl_ct_ipark(const ctl_vector3_t* dq0, const ctl_vector2_t* phasor,
                                    GMP_CTL_OUTPUT_TAG ctl_vector3_t* ab)
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
GMP_STATIC_INLINE void ctl_ct_ipark2(const ctl_vector2_t* dq, const ctl_vector2_t* phasor,
                                     GMP_CTL_OUTPUT_TAG ctl_vector2_t* ab)
{
    ab->dat[phase_alpha] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_cos]) - ctl_mul(dq->dat[phase_q], phasor->dat[phasor_sin]);
    ab->dat[phase_beta] =
        ctl_mul(dq->dat[phase_d], phasor->dat[phasor_sin]) + ctl_mul(dq->dat[phase_q], phasor->dat[phasor_cos]);
}

#if __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_MATH_PARK_H_

