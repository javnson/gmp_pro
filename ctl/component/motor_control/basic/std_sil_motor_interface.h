/**
 * @file std_sil_motor_interface.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Defines the standard data interface for Software-in-the-Loop (SIL) simulation.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This file specifies the data structures used for communication between the
 * embedded controller and a simulation environment like Simulink. The use of
 * `#pragma pack(1)` ensures that the memory layout of these structures is
 * consistent across different platforms, preventing data corruption issues.
 */

#ifndef _FILE_STD_SIL_MOTOR_INTERFACE_H_
#define _FILE_STD_SIL_MOTOR_INTERFACE_H_

/*---------------------------------------------------------------------------*/
/* Software-in-the-Loop (SIL) Interface                                      */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_SIL_INTERFACE Software-in-the-Loop (SIL) Interface
 * @ingroup MC_INTERFACE
 * @brief Data structures for communication with a simulation environment.
 *
 * These structures define the exact format of data packets sent to (TX) and
 * received from (RX) a host simulator. The `#pragma pack(1)` directive is
 * critical to ensure byte-level compatibility between the embedded target
 * and the simulation host (e.g., a PC running Simulink).
 * @{
 */

/**
 * @brief Specifies the byte alignment for the following structure to 1 byte.
 * This is crucial for cross-platform data exchange to avoid padding issues.
 */
#pragma pack(1)

/**
 * @brief Data structure for the Transmit (TX) buffer sent from the controller to the simulator.
 */
typedef struct _tag_mtr_sil_tx_buf
{
    double enable;    /**< @brief Enable signal for the simulation model. */
    uint32_t tabc[3]; /**< @brief PWM compare register values or duty cycles for the three phases. */
    double monitor_port
        [8]; /**< @brief An array of 8 general-purpose monitoring variables to observe internal controller states. */
} mtr_sil_tx_buf_t;

/**
 * @brief Restores the default byte alignment.
 */
#pragma pack()

/**
 * @brief Specifies the byte alignment for the following structure to 1 byte.
 */
#pragma pack(1)

/**
 * @brief Data structure for the Receive (RX) buffer sent from the simulator to the controller.
 */
typedef struct _tag_mtr_sil_rx_buf
{
    double time;        /**< @brief Simulation time, typically in seconds. */
    uint32_t iabc[3];   /**< @brief Simulated three-phase current feedback values. */
    uint32_t uabc[3];   /**< @brief Simulated three-phase voltage feedback values. */
    uint32_t idc;       /**< @brief Simulated DC bus current feedback. */
    uint32_t udc;       /**< @brief Simulated DC bus voltage feedback. */
    uint32_t encoder;   /**< @brief Simulated raw encoder position feedback. */
    int32_t revolution; /**< @brief Simulated encoder revolution count. */
    double panel
        [4]; /**< @brief An array of 4 general-purpose variables for sending control inputs from the simulator panel. */
} mtr_sil_rx_buf_t;

/**
 * @brief Restores the default byte alignment.
 */
#pragma pack()

/** @} */ // end of MC_SIL_INTERFACE group

#endif // _FILE_STD_SIL_MOTOR_INTERFACE_H_
