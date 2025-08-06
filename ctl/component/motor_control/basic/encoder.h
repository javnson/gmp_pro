/**
 * @file encoder.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines structures and functions for handling various types of motor encoders.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This module provides functionalities for processing data from absolute, multi-turn,
 * and auto-counting encoders, as well as calculating motor speed from position data.
 */

#ifndef _FILE_ENCODER_H_
#define _FILE_ENCODER_H_

#include "core/std/gmp_cport.h"
#include <ctl/component/intrinsic/basic/divider.h>
#include <ctl/component/intrinsic/discrete/discrete_filter.h>
#include <ctl/component/motor_control/basic/motor_universal_interface.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup MC_ENCODER Encoder Interface
 * @brief This module contains interfaces for various position and speed encoders.
 */

/*---------------------------------------------------------------------------*/
/* Absolute Position Encoder                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_POS_ENCODER Absolute Position Encoder
 * @ingroup MC_ENCODER
 * @brief Handles single-turn absolute position encoders.
 * @{
 */

/**
 * @brief Data structure for an absolute position encoder.
 */
typedef struct _tag_ctl_pos_encoder_t
{
    rotation_ift encif;  /**< @brief Standard rotation interface for output. */
    uint32_t raw;        /**< @brief Raw data from the position encoder. */
    ctrl_gt offset;      /**< @brief Position offset in per-unit. `position + offset` gives the true position. */
    uint16_t pole_pairs; /**< @brief Number of motor pole pairs to calculate electrical position. */
    uint32_t
        position_base; /**< @brief The base value for a full mechanical revolution (e.g., 2^16 for a 16-bit encoder). */
} pos_encoder_t;

/**
 * @brief Initializes the absolute position encoder structure.
 * @param[out] enc Pointer to the position encoder structure.
 * @param[in] poles Number of motor pole pairs.
 * @param[in] position_base The base value for a full mechanical revolution.
 */
void ctl_init_pos_encoder(pos_encoder_t* enc, uint16_t poles, uint32_t position_base);

/**
 * @brief Processes a new raw encoder value to calculate mechanical and electrical position.
 * @param[in,out] enc Pointer to the position encoder structure.
 * @param[in] raw The new raw value from the encoder hardware.
 * @return The calculated electrical position in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_pos_encoder(pos_encoder_t* enc, uint32_t raw)
{
    enc->raw = raw;
    enc->encif.position = ctl_div(raw, enc->position_base);

    ctrl_gt elec_pos = enc->pole_pairs * (enc->encif.position + CTL_CTRL_CONST_1 - enc->offset);
    ctrl_gt elec_pos_pu = ctrl_mod_1(elec_pos);

    enc->encif.elec_position = elec_pos_pu;
    return enc->encif.elec_position;
}

/**
 * @brief Sets the mechanical offset for the position encoder.
 * @param[in,out] enc Pointer to the position encoder structure.
 * @param[in] offset The desired offset in per-unit.
 */
GMP_STATIC_INLINE void ctl_set_pos_encoder_offset(pos_encoder_t* enc, ctrl_gt offset)
{
    enc->offset = offset;
}

/** @} */ // end of MC_POS_ENCODER group

/*---------------------------------------------------------------------------*/
/* Multi-turn Position Encoder                                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_MULTITURN_ENCODER Multi-turn Position Encoder
 * @ingroup MC_ENCODER
 * @brief Handles multi-turn absolute position encoders where the turn count is provided externally.
 * @{
 */

/**
 * @brief Data structure for a multi-turn position encoder.
 */
typedef struct _tag_ctl_pos_multiturn_encoder_t
{
    rotation_ift encif;     /**< @brief Standard rotation interface for output. */
    uint32_t raw;           /**< @brief Raw data from the position encoder. */
    ctrl_gt offset;         /**< @brief Position offset in per-unit. */
    uint16_t pole_pairs;    /**< @brief Number of motor pole pairs. */
    uint32_t position_base; /**< @brief The base value for a full mechanical revolution. */
} pos_multiturn_encoder_t;

/**
 * @brief Initializes the multi-turn position encoder structure.
 * @param[out] enc Pointer to the multi-turn encoder structure.
 * @param[in] poles Number of motor pole pairs.
 * @param[in] position_base The base value for a full mechanical revolution.
 */
void ctl_init_multiturn_pos_encoder(pos_multiturn_encoder_t* enc, uint16_t poles, uint32_t position_base);

/**
 * @brief Processes a new raw encoder value and an external revolution count.
 * @param[in,out] enc Pointer to the multi-turn encoder structure.
 * @param[in] raw The new raw value from the encoder hardware.
 * @param[in] revolutions The externally tracked revolution count.
 * @return The calculated electrical position in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_multiturn_pos_encoder(pos_multiturn_encoder_t* enc, uint32_t raw,
                                                         int32_t revolutions)
{
    enc->raw = raw;
    enc->encif.position = ctl_div(raw, enc->position_base);

    ctrl_gt elec_pos = enc->pole_pairs * (enc->encif.position + CTL_CTRL_CONST_1 - enc->offset);
    ctrl_gt elec_pos_pu = ctrl_mod_1(elec_pos);

    enc->encif.elec_position = elec_pos_pu;
    enc->encif.revolutions = revolutions;
    return enc->encif.elec_position;
}

/** @} */ // end of MC_MULTITURN_ENCODER group

/*---------------------------------------------------------------------------*/
/* Auto-turn Counting Position Encoder                                       */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_AUTOTURN_ENCODER Auto-turn Counting Position Encoder
 * @ingroup MC_ENCODER
 * @brief Handles single-turn encoders and automatically tracks full revolutions.
 * @{
 */

/**
 * @brief Data structure for an auto-turn counting position encoder.
 */
typedef struct _tag_ctl_pos_autoturn_encoder_t
{
    rotation_ift encif;     /**< @brief Standard rotation interface for output. */
    uint32_t raw;           /**< @brief Raw data from the position encoder. */
    ctrl_gt offset;         /**< @brief Position offset in per-unit. */
    uint16_t pole_pairs;    /**< @brief Number of motor pole pairs. */
    ctrl_gt last_pos;       /**< @brief Last recorded mechanical position, used for revolution counting. */
    uint32_t position_base; /**< @brief The base value for a full mechanical revolution. */
} pos_autoturn_encoder_t;

/**
 * @brief Initializes the auto-turn position encoder structure.
 * @param[out] enc Pointer to the auto-turn encoder structure.
 * @param[in] poles Number of motor pole pairs.
 * @param[in] position_base The base value for a full mechanical revolution.
 */
void ctl_init_autoturn_pos_encoder(pos_autoturn_encoder_t* enc, uint16_t poles, uint32_t position_base);

/**
 * @brief Processes a new raw encoder value and updates the revolution count based on position change.
 * @param[in,out] enc Pointer to the auto-turn encoder structure.
 * @param[in] raw The new raw value from the encoder hardware.
 * @return The calculated electrical position in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_step_autoturn_pos_encoder(pos_autoturn_encoder_t* enc, uint32_t raw)
{
    enc->raw = raw;
    enc->encif.position = ctl_div(raw, enc->position_base);

    ctrl_gt elec_pos = (enc->encif.position + CTL_CTRL_CONST_1 - enc->offset) * enc->pole_pairs;
    ctrl_gt elec_pos_pu = ctrl_mod_1(elec_pos);
    enc->encif.elec_position = elec_pos_pu;

    // Check for revolution crossing
    if (enc->encif.position - enc->last_pos > CTL_CTRL_CONST_1_OVER_2)
    {
        enc->encif.revolutions -= 1; // Negative direction rollover
    }
    if (enc->last_pos - enc->encif.position > CTL_CTRL_CONST_1_OVER_2)
    {
        enc->encif.revolutions += 1; // Positive direction rollover
    }

    enc->last_pos = enc->encif.position;
    return enc->encif.elec_position;
}

/**
 * @brief Sets the mechanical offset for the auto-turn encoder from a raw value.
 * @param[in,out] enc Pointer to the auto-turn encoder structure.
 * @param[in] raw The raw encoder value that corresponds to the zero position.
 */
GMP_STATIC_INLINE void ctl_set_autoturn_pos_encoder_offset(pos_autoturn_encoder_t* enc, uint32_t raw)
{
    enc->offset = float2ctrl((ctrl_gt)raw / enc->position_base);
}

/** @} */ // end of MC_AUTOTURN_ENCODER group

/*---------------------------------------------------------------------------*/
/* Speed Calculator from Position                                            */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_SPEED_CALCULATOR Speed Calculator
 * @ingroup MC_ENCODER
 * @brief Calculates motor speed by differentiating position over time.
 * @{
 */

/**
 * @brief Data structure for the speed calculator.
 */
typedef struct _tag_speed_calculator_t
{
    velocity_ift encif;               /**< @brief Standard velocity interface for output. */
    rotation_ift* pos_encif;          /**< @brief Pointer to the source position encoder interface. */
    ctrl_gt old_position;             /**< @brief Last recorded position (p.u.) for differentiation. */
    ctrl_gt scale_factor;             /**< @brief Factor to convert position delta to speed in p.u. */
    ctl_low_pass_filter_t spd_filter; /**< @brief Low-pass filter for the calculated speed. */
    ctl_divider_t div;                /**< @brief Divider to control the execution frequency of the calculation. */
} spd_calculator_t;

/**
 * @brief Initializes the speed calculator.
 * @param[out] sc Pointer to the speed calculator structure.
 * @param[in] pos_encif Pointer to the source position encoder interface.
 * @param[in] control_law_freq Frequency of the main control loop in Hz.
 * @param[in] speed_calc_div Division factor for the speed calculation frequency (relative to control_law_freq).
 * @param[in] rated_speed_rpm The rated speed of the motor in RPM, used as the base for per-unit speed.
 * @param[in] pole_pairs Number of motor pole pairs.
 * @param[in] speed_filter_fc Cutoff frequency for the speed low-pass filter in Hz.
 */
void ctl_init_spd_calculator(spd_calculator_t* sc, rotation_ift* pos_encif, parameter_gt control_law_freq,
                             uint32_t speed_calc_div, parameter_gt rated_speed_rpm, uint16_t pole_pairs,
                             parameter_gt speed_filter_fc);

/**
 * @brief Executes one step of the speed calculation.
 *
 * This function should be called periodically. It calculates the speed based on
 * the change in position since the last call, filters it, and updates the output.
 * The internal divider determines the actual execution rate.
 *
 * @param[in,out] sc Pointer to the speed calculator structure.
 */
GMP_STATIC_INLINE void ctl_step_spd_calc(spd_calculator_t* sc)
{
    if (ctl_step_divider(&sc->div))
    {
        ctrl_gt current_pos = ctl_get_encoder_position(sc->pos_encif);
        ctrl_gt delta = current_pos - sc->old_position;

        // Correct for position rollover
        if (delta < -CTL_CTRL_CONST_1_OVER_2)
        {
            delta += CTL_CTRL_CONST_1;
        }
        else if (delta > CTL_CTRL_CONST_1_OVER_2)
        {
            delta -= CTL_CTRL_CONST_1;
        }

        ctrl_gt new_spd = ctl_mul(delta, sc->scale_factor);
        sc->encif.speed = ctl_step_lowpass_filter(&sc->spd_filter, new_spd);

        sc->old_position = current_pos;
    }
}

/** @} */ // end of MC_SPEED_CALCULATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_ENCODER_H_
