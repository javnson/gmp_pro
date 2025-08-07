/**
 * @file gmp_sil_std_interface.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a universal standard interface for GMP Software-in-the-Loop (SIL) simulations.
 * @details This file specifies a data structure intended to be the primary interface
 * between a control algorithm and a simulation environment (e.g., MATLAB/Simulink).
 * It provides channels for PWM outputs, data outputs, and monitoring internal variables.
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_GMP_SIL_STD_INTERFACE_H_
#define _FILE_GMP_SIL_STD_INTERFACE_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup SIL_INTERFACE Software-in-the-Loop (SIL) Interface
 * @ingroup GMP_CTL_INTERFACES
 * @brief Defines the standard data structure for SIL simulation environments.
 *
 * This module contains the data structures used to link the control algorithm
 * with an external simulation platform.
 */

/*---------------------------------------------------------------------------*/
/* SIL Standard Interface                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup SIL_INTERFACE
 * @{
 */

/**
 * @brief Defines the standard data exchange structure for SIL simulation.
 * @details This structure is typically mapped to the inputs and outputs of a
 * simulation block (e.g., an S-Function in Simulink).
 */
typedef struct _tag_sil_std_if
{
    double enable; /**< Simulation enable flag. Typically 1.0 for enabled, 0.0 for disabled. */

    uint32_t pwm_channel
        [24]; /**< Array for PWM channel outputs. The values can represent raw timer compare values or scaled duty cycles. */

    double dout
        [24]; /**< Array for general-purpose data outputs from the controller. Useful for logging or triggering events in the simulation. */

    double monitor_port
        [24]; /**< Array for monitoring internal controller variables. Can be used to observe states, intermediate calculations, etc. */

} ctl_sil_std_ift;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_SIL_STD_INTERFACE_H_
