/**
 * @file std_sil_dp_interface.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Defines the standard data interface for Software-in-the-Loop (SIL) simulation.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
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
 * @defgroup sil_interface_api Software-in-the-Loop (SIL) Interface API
 * @brief Data structures for communication with a simulation environment.
 * @details This file specifies the data structures used for communication between the
 * embedded controller code and a simulation environment like Simulink. The `#pragma pack(1)`
 * directive is used to ensure that the memory layout of these structures is identical
 * on both the controller and the simulation host, preventing data corruption due to
 * memory alignment differences.
 * @{
 * @ingroup CTL_DP_LIB
 */

/**
 * @brief Specifies the data structure for data transmitted FROM the controller TO the simulation environment.
 * @details This structure is packed to 1-byte alignment to ensure compatibility.
 */
#pragma pack(1)
typedef struct _tag_dp_sil_tx_buf
{
    double enable;       /**< Master enable signal sent to the simulation. */
    uint32_t pwm_cmp[8]; /**< Array of PWM compare register values. */
    uint32_t dac[8];     /**< Array of Digital-to-Analog Converter output values. */
    double monitor[16];  /**< Array of general-purpose monitoring variables. */
} dp_sil_tx_buf_t;
#pragma pack()

/**
 * @brief Specifies the data structure for data received BY the controller FROM the simulation environment.
 * @details This structure is packed to 1-byte alignment to ensure compatibility.
 */
#pragma pack(1)
typedef struct _tag_dp_sil_rx_buf
{
    double time;             /**< Simulation time provided by the host. */
    uint32_t adc_result[24]; /**< Array of simulated Analog-to-Digital Converter results. */
    double panel[16];        /**< Array of simulated user panel inputs (e.g., knobs, switches). */
    int32_t digital[8];      /**< Array of simulated digital inputs. */
} dp_sil_rx_buf_t;
#pragma pack()

/** @} */ // end of sil_interface_api group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_SIL_STD_INTERFACE_H_
