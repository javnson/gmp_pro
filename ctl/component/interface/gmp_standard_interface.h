/**
 * @file gmp_standard_interface.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines a universal standard interface for GMP-based controllers.
 * @details This file specifies the data structures for transmitting (Tx) and
 * receiving (Rx) data between the controller and an external system. This is
 * essential for hardware integration, debugging, and communication.
 * @version 0.1
 * @date 2024-10-01
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_GMP_STANDARD_INTERFACE_H_
#define _FILE_GMP_STANDARD_INTERFACE_H_

#include <ctl/component/interface/interface_base.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup STD_INTERFACE Standard Controller Interface
 * @brief Defines standard data structures for controller data exchange (Tx/Rx).
 *
 * This module contains the data structures for the main data pathways into and
 * out of the controller.
 */

/*---------------------------------------------------------------------------*/
/* Standard Controller Interface                                             */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup STD_INTERFACE
 * @{
 */

/**
 * @brief Data structure for the controller's transmission (Tx) port.
 * @details This structure holds all data that the controller sends to the outside world.
 */
typedef struct _tag_std_ctrl_tx_port
{
    double enable;            /**< Controller enable status output. */
    uint32_t pwm_channel[24]; /**< Array for PWM channel outputs (e.g., raw timer values). */
    double dout[24];          /**< Array for general-purpose digital or analog data outputs. */
    double monitor_port[24];  /**< Array for monitoring internal controller variables. */
} ctl_std_tx_port_t;

/**
 * @brief Data structure for the controller's reception (Rx) port.
 * @details This structure holds all data that the controller receives from the outside world.
 */
typedef struct _tag_std_ctrl_rx_port
{
    double time;              /**< Current system time or simulation time, typically in seconds. */
    uint32_t adc_channel[24]; /**< Array for raw ADC channel inputs. */
    double din[24];           /**< Array for general-purpose digital or analog data inputs. */
    double sensor_port[24];   /**< Array for other processed sensor data or external commands. */
} ctl_std_rx_port_t;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_STANDARD_INTERFACE_H_
