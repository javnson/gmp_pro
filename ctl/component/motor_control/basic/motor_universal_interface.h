/**
 * @file motor_universal_interface.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a universal interface for motor controllers and sensors.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_MOTOR_UNIVERSAL_INTERFACE_H_
#define _FILE_MOTOR_UNIVERSAL_INTERFACE_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup MC_INTERFACE Motor Control Interface
 * @brief Standardized interfaces for motor sensors and controllers.
 * @details This file implements a set of standardized, abstract interfaces for various
 * motor sensors (position, speed, torque, etc.) and a universal structure
 * to aggregate these interfaces for a single motor controller.
 * 
 */

/*---------------------------------------------------------------------------*/
/* Rotation Sensor Interface                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_ROTATION_IF Rotation Sensor Interface
 * @ingroup MC_INTERFACE
 * @brief Defines the standard interface for a rotation/position sensor.
 * @{
 */

/**
 * @brief Standard data structure for a rotation encoder interface.
 *
 * Any structure representing a position sensor should have these members
 * at its beginning to allow for generic casting and access.
 */
typedef struct _tag_rotation_encoder_t
{
    ctrl_gt position;      /**< @brief Mechanical position of the encoder, in per-unit (p.u.). */
    ctrl_gt elec_position; /**< @brief Electrical position of the motor, in per-unit (p.u.). */
    int32_t revolutions;   /**< @brief Mechanical revolution count of the motor. */
} rotation_ift;

/**
 * @brief Gets the mechanical position from a rotation interface.
 * @param enc Pointer to the rotation interface structure.
 * @return Mechanical position in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_encoder_position(rotation_ift* enc)
{
    return enc->position;
}

/**
 * @brief Gets the electrical position from a rotation interface.
 * @note There is a typo in the original function name (`postion` instead of `position`).
 * @param enc Pointer to the rotation interface structure.
 * @return Electrical position in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_encoder_elec_postion(rotation_ift* enc)
{
    return enc->elec_position;
}

/** @} */ // end of MC_ROTATION_IF group

/*---------------------------------------------------------------------------*/
/* Velocity Sensor Interface                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VELOCITY_IF Velocity Sensor Interface
 * @ingroup MC_INTERFACE
 * @brief Defines the standard interface for a velocity/speed sensor.
 * @{
 */

/**
 * @brief Standard data structure for a speed encoder interface.
 */
typedef struct _tag_speed_encoder_t
{
    ctrl_gt speed; /**< @brief Mechanical speed output, in per-unit (p.u.). */
} velocity_ift;

/**
 * @brief Gets the speed from a velocity interface.
 * @param enc Pointer to the velocity interface structure.
 * @return Mechanical speed in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_encoder_speed(velocity_ift* enc)
{
    return enc->speed;
}

/** @} */ // end of MC_VELOCITY_IF group

/*---------------------------------------------------------------------------*/
/* Torque Sensor Interface                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_TORQUE_IF Torque Sensor Interface
 * @ingroup MC_INTERFACE
 * @brief Defines the standard interface for a torque sensor.
 * @{
 */

/**
 * @brief Standard data structure for a torque sensor interface.
 */
typedef struct _tag_torque_sensor_interface_t
{
    ctrl_gt torque; /**< @brief Mechanical torque output, in per-unit (p.u.). */
} torque_ift;

/** @} */ // end of MC_TORQUE_IF group

/*---------------------------------------------------------------------------*/
/* Interface Type Casting Macros                                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_INTERFACE_CASTING Interface Casting Macros
 * @ingroup MC_INTERFACE
 * @brief Macros for safely casting generic pointers to specific interface types.
 * @{
 */

#define CTL_POSITION_IF(X) ((rotation_ift*)X) /**< @brief Casts a void pointer to a position interface pointer. */
#define CTL_SPEED_IF(X)    ((velocity_ift*)X) /**< @brief Casts a void pointer to a velocity interface pointer. */

/** @} */ // end of MC_INTERFACE_CASTING group

/*---------------------------------------------------------------------------*/
/* Universal Motor Interface                                                 */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_UNIVERSAL_IF Universal Motor Interface
 * @ingroup MC_INTERFACE
 * @brief An aggregator structure that holds pointers to all sensor interfaces for a motor.
 * @{
 */

/**
 * @brief Universal motor interface structure.
 *
 * This structure contains pointers to all sensor inputs for a single motor,
 * providing a unified access point for control algorithms.
 */
typedef struct _tag_universal_mtr_if_t
{
    tri_adc_ift* uabc;      /**< @brief Pointer to the three-phase voltage sensor interface. */
    tri_adc_ift* iabc;      /**< @brief Pointer to the three-phase current sensor interface. */
    adc_ift* udc;           /**< @brief Pointer to the DC bus voltage sensor interface. */
    adc_ift* idc;           /**< @brief Pointer to the DC bus current sensor interface. */
    rotation_ift* position; /**< @brief Pointer to the position encoder interface. */
    velocity_ift* velocity; /**< @brief Pointer to the speed encoder interface. */
    torque_ift* torque;     /**< @brief Pointer to the torque sensor interface. */
} mtr_ift;

// --- Accessor Functions ---

/**
 * @brief Gets a single-phase current value.
 * @param mtr Pointer to the universal motor interface.
 * @param phase The phase index (0 for U, 1 for V, 2 for W).
 * @return The current value for the specified phase.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_current_single(mtr_ift* mtr, uint32_t phase)
{
    gmp_base_assert(mtr->iabc);
    gmp_base_assert(phase < 3); // Corrected from 4 to 3 for 3-phase systems
    return mtr->iabc->value.dat[phase];
}

/**
 * @brief Gets a pointer to the three-phase current vector.
 * @param mtr Pointer to the universal motor interface.
 * @return Pointer to the vector3_gt structure containing phase currents.
 */
GMP_STATIC_INLINE vector3_gt* ctl_get_mtr_current(mtr_ift* mtr)
{
    gmp_base_assert(mtr->iabc);
    return &mtr->iabc->value;
}

/**
 * @brief Gets the DC bus voltage.
 * @param mtr Pointer to the universal motor interface.
 * @return The DC bus voltage value.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_dc_voltage(mtr_ift* mtr)
{
    gmp_base_assert(mtr->udc);
    return mtr->udc->value;
}

/**
 * @brief Gets the motor's electrical angle (theta).
 * @param mtr Pointer to the universal motor interface.
 * @return The electrical angle in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_elec_theta(mtr_ift* mtr)
{
    gmp_base_assert(mtr->position);
    return mtr->position->elec_position;
}

/**
 * @brief Gets the motor's mechanical angle (theta).
 * @param mtr Pointer to the universal motor interface.
 * @return The mechanical angle in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_theta(mtr_ift* mtr)
{
    gmp_base_assert(mtr->position);
    return mtr->position->position;
}

/**
 * @brief Gets the motor's revolution count.
 * @param mtr Pointer to the universal motor interface.
 * @return The total number of full revolutions.
 */
GMP_STATIC_INLINE int32_t ctl_get_mtr_revolution(mtr_ift* mtr)
{
    gmp_base_assert(mtr->position);
    return mtr->position->revolutions;
}

/**
 * @brief Gets the motor's velocity.
 * @param mtr Pointer to the universal motor interface.
 * @return The mechanical velocity in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_velocity(mtr_ift* mtr)
{
    gmp_base_assert(mtr->velocity);
    return mtr->velocity->speed;
}

/**
 * @brief Gets the motor's torque.
 * @param mtr Pointer to the universal motor interface.
 * @return The measured torque in per-unit.
 */
GMP_STATIC_INLINE ctrl_gt ctl_get_mtr_torque(mtr_ift* mtr)
{
    gmp_base_assert(mtr->torque);
    return mtr->torque->torque;
}

// --- Attachment Functions ---

/**
 * @brief Attaches a three-phase current sensor interface to the universal motor interface.
 * @param mtr Pointer to the universal motor interface.
 * @param iabc Pointer to the three-phase ADC interface for current sensing.
 */
GMP_STATIC_INLINE void ctl_attach_mtr_current(mtr_ift* mtr, tri_adc_ift* iabc)
{
    mtr->iabc = iabc;
}

/**
 * @brief Attaches a position encoder interface to the universal motor interface.
 * @param mtr Pointer to the universal motor interface.
 * @param pos Pointer to the rotation interface.
 */
GMP_STATIC_INLINE void ctl_attach_mtr_position(mtr_ift* mtr, rotation_ift* pos)
{
    mtr->position = pos;
}

/**
 * @brief Attaches a velocity sensor interface to the universal motor interface.
 * @param mtr Pointer to the universal motor interface.
 * @param vel Pointer to the velocity interface.
 */
GMP_STATIC_INLINE void ctl_attach_mtr_velocity(mtr_ift* mtr, velocity_ift* vel)
{
    mtr->velocity = vel;
}

/**
 * @brief Attaches a torque sensor interface to the universal motor interface.
 * @param mtr Pointer to the universal motor interface.
 * @param torque Pointer to the torque interface.
 */
GMP_STATIC_INLINE void ctl_attach_mtr_torque(mtr_ift* mtr, torque_ift* torque)
{
    mtr->torque = torque;
}

/*---------------------------------------------------------------------------*/
/* Current Controller interface                                              */
/*---------------------------------------------------------------------------*/

/**
 * @brief Universal motor current interface structure.
 */
typedef struct _tag_universal_mtr_current_if_t
{
    vector2_gt* idq;
} mtr_current_ift;

/** @} */ // end of MC_UNIVERSAL_IF group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_MOTOR_UNIVERSAL_INTERFACE_H_
