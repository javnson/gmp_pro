/**
 * @file pwm_channel.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides modules for processing and scaling PWM channel outputs.
 * @version 0.2
 * @date 2024-10-01
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_PWM_CHANNEL_H_
#define _FILE_PWM_CHANNEL_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup PWM_CHANNEL_SINGLE Single PWM Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing a single PWM output channel.
 * @details This file contains structures and functions to convert a logical control
 * value (typically a per-unit duty cycle) into a raw integer value suitable for
 * a hardware PWM timer's compare register.
 */

/**
 * @defgroup PWM_CHANNEL_DUAL Dual PWM Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing a pair of PWM output channels.
 */

/**
 * @defgroup PWM_CHANNEL_TRI Triple PWM Channel
 * @ingroup GMP_CTL_COMMON_INTERFACES
 * @brief A module for processing three PWM output channels, typically for motor control.
 */

/*---------------------------------------------------------------------------*/
/* Single PWM Channel                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup PWM_CHANNEL_SINGLE
 * @{
 */

/**
 * @brief Data structure for a single PWM channel.
 */
typedef struct _tag_pwm_channel
{
    pwm_ift raw;       /**< INPUT: Raw duty cycle data from the control algorithm (per-unit). */
    pwm_gt full_scale; /**< The full-scale value of the PWM timer (e.g., the period register value). */
    pwm_gt phase;      /**< A phase shift or offset added to the final PWM value. */
    pwm_gt value;      /**< OUTPUT: The calculated raw compare value for the PWM timer. */
} pwm_channel_t;

void ctl_init_pwm_channel(pwm_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale);

/**
 * @brief Calculates the PWM compare value from a per-unit input.
 * @details The output is saturated between 0 and `full_scale`.
 * Assumes the input `raw` is a unipolar value from 0.0 to 1.0.
 * @param pwm_obj Pointer to the PWM channel object.
 * @param raw The raw per-unit duty cycle input.
 * @return The calculated PWM compare value.
 */
GMP_STATIC_INLINE pwm_gt ctl_step_pwm_channel(pwm_channel_t* pwm_obj, ctrl_gt raw)
{
    if (raw <= 0)
        pwm_obj->raw.value = 0;
    else
        pwm_obj->raw.value = raw;

    pwm_obj->value = pwm_mul(pwm_obj->raw.value, pwm_obj->full_scale) + pwm_obj->phase;
    pwm_obj->value = pwm_sat(pwm_obj->value, pwm_obj->full_scale, 0);
    return pwm_obj->value;
}

/**
 * @brief Calculates the PWM compare value with wrapping (modulo) behavior.
 * @details Useful for applications requiring phase wrapping, like resolvers.
 * @param pwm_obj Pointer to the PWM channel object.
 * @param raw The raw per-unit duty cycle input.
 * @return The calculated PWM compare value, wrapped by `full_scale`.
 */
GMP_STATIC_INLINE pwm_gt ctl_step_pwm_channel_warp(pwm_channel_t* pwm_obj, ctrl_gt raw)
{
    pwm_obj->raw.value = raw;
    pwm_obj->value = pwm_mul(pwm_obj->raw.value, pwm_obj->full_scale) + pwm_obj->phase;
    pwm_obj->value = pwm_obj->value % pwm_obj->full_scale;
    return pwm_obj->value;
}

/**
 * @brief Calculates the inverted (complementary) PWM compare value.
 * @details Output = full_scale - (raw * full_scale). Useful for complementary PWM pairs.
 * @param pwm_obj Pointer to the PWM channel object.
 * @param raw The raw per-unit duty cycle input.
 * @return The calculated inverted PWM compare value.
 */
GMP_STATIC_INLINE pwm_gt ctl_step_pwm_channel_inv(pwm_channel_t* pwm_obj, ctrl_gt raw)
{
    pwm_obj->raw.value = raw;
    pwm_obj->value = pwm_obj->full_scale - pwm_mul(pwm_obj->raw.value, pwm_obj->full_scale);
    pwm_obj->value = pwm_sat(pwm_obj->value, pwm_obj->full_scale, 0);
    return pwm_obj->value;
}

/**
 * @brief Gets a pointer to the control port interface of the PWM channel.
 * @param pwm Pointer to the PWM channel object.
 * @return Pointer to the `pwm_ift` control port.
 */
GMP_STATIC_INLINE pwm_ift* ctl_get_pwm_channel_ctrl_port(pwm_channel_t* pwm)
{
    return &pwm->raw;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Dual PWM Channel                                                          */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup PWM_CHANNEL_DUAL
 * @{
 */

/**
 * @brief Data structure for a dual PWM channel.
 */
typedef struct _tag_pwm_dual_channel
{
    dual_pwm_ift raw;  /**< INPUT: Raw duty cycle data for two channels (per-unit). */
    pwm_gt full_scale; /**< The full-scale value of the PWM timer. */
    pwm_gt phase;      /**< A phase shift or offset added to the final PWM values. */
    pwm_gt value[2];   /**< OUTPUT: The calculated raw compare values for the two PWM channels. */
} pwm_dual_channel_t;

void ctl_init_pwm_dual_channel(pwm_dual_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale);

/**
 * @brief Calculates the compare values for a dual PWM channel.
 * @param pwm_obj Pointer to the dual PWM channel object.
 * @param raw Pointer to a 2D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_dual_channel(pwm_dual_channel_t* pwm_obj, ctl_vector2_t* raw)
{
    int i;

    for (i = 0; i < 2; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale) + pwm_obj->phase;
        pwm_obj->value[i] = pwm_sat(pwm_obj->value[i], pwm_obj->full_scale, 0);
    }
}

/**
 * @brief Calculates the compare values for a dual PWM channel with wrapping behavior.
 * @param pwm_obj Pointer to the dual PWM channel object.
 * @param raw Pointer to a 2D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_dual_channel_warp(pwm_dual_channel_t* pwm_obj, ctl_vector2_t* raw)
{
    int i;

    for (i = 0; i < 2; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale) + pwm_obj->phase;
        pwm_obj->value[i] = pwm_obj->value[i] % pwm_obj->full_scale;
    }
}

/**
 * @brief Calculates the inverted compare values for a dual PWM channel.
 * @param pwm_obj Pointer to the dual PWM channel object.
 * @param raw Pointer to a 2D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_dual_channel_inv(pwm_dual_channel_t* pwm_obj, ctl_vector2_t* raw)
{
    int i;

    for (i = 0; i < 2; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_obj->full_scale - pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale);
        pwm_obj->value[i] = pwm_sat(pwm_obj->value[i], pwm_obj->full_scale, 0);
    }
}

/**
 * @brief Gets a pointer to the control port interface of the dual PWM channel.
 * @param pwm Pointer to the dual PWM channel object.
 * @return Pointer to the `dual_pwm_ift` control port.
 */
GMP_STATIC_INLINE dual_pwm_ift* ctl_get_dual_pwm_channel_ctrl_port(pwm_dual_channel_t* pwm)
{
    return &pwm->raw;
}

/**
 * @}
 */

/*---------------------------------------------------------------------------*/
/* Triple PWM Channel                                                        */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup PWM_CHANNEL_TRI
 * @{
 */

/**
 * @brief Data structure for a three-phase PWM channel.
 */
typedef struct _tag_pwm_tri_channel
{
    tri_pwm_ift raw;   /**< INPUT: Raw duty cycle data for three channels (per-unit). */
    pwm_gt full_scale; /**< The full-scale value of the PWM timer. */
    pwm_gt phase;      /**< A phase shift or offset added to the final PWM values. */
    pwm_gt value[3];   /**< OUTPUT: The calculated raw compare values for the three PWM channels. */
} pwm_tri_channel_t;

void ctl_init_pwm_tri_channel(pwm_tri_channel_t* pwm_obj, pwm_gt phase, pwm_gt full_scale);

/**
 * @brief Calculates the compare values for a three-phase PWM channel, without input raw.
 */
GMP_STATIC_INLINE void ctl_calc_pwm_tri_channel(pwm_tri_channel_t* pwm_obj)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        pwm_obj->value[i] = pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale) + pwm_obj->phase;
        pwm_obj->value[i] = pwm_sat(pwm_obj->value[i], pwm_obj->full_scale, 0);
    }
}

/**
 * @brief Calculates the compare values for a three-phase PWM channel.
 * @param pwm_obj Pointer to the triple PWM channel object.
 * @param raw Pointer to a 3D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_tri_channel(pwm_tri_channel_t* pwm_obj, ctl_vector3_t* raw)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale) + pwm_obj->phase;
        pwm_obj->value[i] = pwm_sat(pwm_obj->value[i], pwm_obj->full_scale, 0);
    }
}

/**
 * @brief Calculates the compare values for a three-phase PWM channel with wrapping behavior.
 * @param pwm_obj Pointer to the triple PWM channel object.
 * @param raw Pointer to a 3D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_tri_channel_warp(pwm_tri_channel_t* pwm_obj, ctl_vector3_t* raw)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale) + pwm_obj->phase;
        pwm_obj->value[i] = pwm_obj->value[i] % pwm_obj->full_scale;
    }
}

/**
 * @brief Calculates the inverted compare values for a three-phase PWM channel.
 * @param pwm_obj Pointer to the triple PWM channel object.
 * @param raw Pointer to a 3D vector containing the raw per-unit duty cycles.
 */
GMP_STATIC_INLINE void ctl_step_pwm_tri_channel_inv(pwm_tri_channel_t* pwm_obj, ctl_vector3_t* raw)
{
    int i;

    for (i = 0; i < 3; ++i)
    {
        pwm_obj->raw.value.dat[i] = raw->dat[i];
        pwm_obj->value[i] = pwm_obj->full_scale - pwm_mul(pwm_obj->raw.value.dat[i], pwm_obj->full_scale);
        pwm_obj->value[i] = pwm_sat(pwm_obj->value[i], pwm_obj->full_scale, 0);
    }
}

/**
 * @brief Gets a pointer to the control port interface of the triple PWM channel.
 * @param pwm Pointer to the triple PWM channel object.
 * @return Pointer to the `tri_pwm_ift` control port.
 */
GMP_STATIC_INLINE tri_pwm_ift* ctl_get_tri_pwm_channel_ctrl_port(pwm_tri_channel_t* pwm)
{
    return &pwm->raw;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_PWM_CHANNEL_H_
