/**
 * @file interface_base.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines the basic data interface types for control peripherals like ADC, DAC, and PWM.
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_CTL_INTERFACE_BASE_H_
#define _FILE_CTL_INTERFACE_BASE_H_

#include <ctl/math_block/vector_lite/vector2.h>
#include <ctl/math_block/vector_lite/vector3.h>
#include <ctl/math_block/vector_lite/vector4.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup GMP_CTL_INTERFACES GMP CTL interface
 * @brief Basic data structures for interfacing with control peripherals.
 * @details This module provides a hardware abstraction layer by defining universal
 * interface structures for single and multi-channel peripherals. Controller algorithms
 * should use these interfaces to remain independent of specific hardware implementations.
 */

/**
 * @defgroup GMP_CTL_COMMON_INTERFACES GMP CTL basic interface
 * @ingroup GMP_CTL_INTERFACES
 * @brief Basic data interface, DAC, ADC, PWM, CAP, for interfacing with control peripherals.
 */

/**
 * @defgroup PERIPHERAL_INTERFACES Peripheral Interface Base
 * @ingroup GMP_CTL_INTERFACES
 * @brief Basic data structures for interfacing with control peripherals.
 *
 * This module defines the fundamental types used to pass data to and from
 * peripherals like ADCs, DACs, PWMs, and Capture units.
 */

/*---------------------------------------------------------------------------*/
/* Single Channel Interfaces                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup PERIPHERAL_INTERFACES
 * @{
 */

/**
 * @brief Base structure for a single-channel peripheral interface.
 * @details The `value` member holds a per-unit representation of the data.
 * - For ADC/CAP: Output result (scaled and biased).
 * - For DAC/PWM: Input target value.
 */
typedef struct _tag_channel_base_t
{
    ctrl_gt value;
} adc_ift, dac_ift, pwm_ift, cap_ift;

/**
 * @brief Gets the result from a single ADC channel interface.
 * @param adc Pointer to the `adc_ift` object.
 * @return The ADC value in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adc_result(adc_ift* adc)
{
    return adc->value;
}

/**
 * @brief Sets the target for a single PWM channel interface.
 * @param pwm Pointer to the `pwm_ift` object.
 * @param target The target value in per-unit.
 */
GMP_STATIC_INLINE void ctl_set_pwm_target(pwm_ift* pwm, ctrl_gt target)
{
    pwm->value = target;
}

/**
 * @brief Sets the target for a single DAC channel interface.
 * @param dac Pointer to the `dac_ift` object.
 * @param target The target value in per-unit.
 */
GMP_STATIC_INLINE void ctl_set_dac_target(dac_ift* dac, ctrl_gt target)
{
    dac->value = target;
}

/*---------------------------------------------------------------------------*/
/* Dual Channel Interfaces                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Base structure for a dual-channel peripheral interface.
 */
typedef struct _tag_dual_channel_base_t
{
    vector2_gt value;
} dual_adc_ift, dual_dac_ift, dual_pwm_ift;

/**
 * @brief Gets the differential result from a dual ADC interface (ch0 - ch1).
 * @param adc Pointer to the `dual_adc_ift` object.
 * @return The differential value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_adc_diff_result(dual_adc_ift* adc)
{
    return adc->value.dat[0] - adc->value.dat[1];
}

/**
 * @brief Gets the result from a dual ADC interface as a 2D vector.
 * @param adc Pointer to the `dual_adc_ift` object.
 * @param vec Pointer to a `vector2_gt` to store the result.
 */
GMP_STATIC_INLINE void ctl_get_dual_adc_result_vec(dual_adc_ift* adc, vector2_gt* vec)
{
    ctl_vector2_copy(vec, &adc->value);
}

/**
 * @brief Sets the targets for a dual PWM interface.
 * @param pwm Pointer to the `dual_pwm_ift` object.
 * @param phase1 Target value for the first channel.
 * @param phase2 Target value for the second channel.
 */
GMP_STATIC_INLINE void ctl_set_dual_pwm(dual_pwm_ift* pwm, ctrl_gt phase1, ctrl_gt phase2)
{
    pwm->value.dat[0] = phase1;
    pwm->value.dat[1] = phase2;
}

/**
 * @brief Sets the targets for a dual PWM interface using a 2D vector.
 * @param pwm Pointer to the `dual_pwm_ift` object.
 * @param vec Pointer to a `vector2_gt` containing the target values.
 */
GMP_STATIC_INLINE void ctl_set_dual_pwm_vec(dual_pwm_ift* pwm, vector2_gt* vec)
{
    ctl_vector2_copy(&pwm->value, vec);
}

/**
 * @brief Sets the targets for a dual DAC interface.
 * @param dac Pointer to the `dual_dac_ift` object.
 * @param phase1 Target value for the first channel.
 * @param phase2 Target value for the second channel.
 */
GMP_STATIC_INLINE void ctl_set_dual_dac(dual_dac_ift* dac, ctrl_gt phase1, ctrl_gt phase2)
{
    dac->value.dat[0] = phase1;
    dac->value.dat[1] = phase2;
}

/*---------------------------------------------------------------------------*/
/* Triple Channel Interfaces                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @brief Base structure for a three-channel peripheral interface.
 */
typedef struct _tag_tri_channel_base_t
{
    vector3_gt value;
} tri_adc_ift, tri_dac_ift, tri_pwm_ift;

/*---------------------------------------------------------------------------*/
/* Quad Channel Interfaces                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Base structure for a four-channel peripheral interface.
 */
typedef struct _tag_quad_channel_base_t
{
    vector4_gt value;
} quad_adc_ift, quad_dac_ift, quad_pwm_ift;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_BASE_H_
