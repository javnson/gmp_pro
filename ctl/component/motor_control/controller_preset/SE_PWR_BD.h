/**
 * @file SE_PWR_BD.h
 * @brief Defines hardware-specific parameters for the SE Power Board.
 * @details This file contains pre-defined constants related to the analog-to-digital
 * converter (ADC) scaling for current and voltage sensing on this specific hardware.
 *
 * @version 1.0
 * @date 2025-08-06
 */

#ifndef _FILE_SE_PWR_BD_H_
#define _FILE_SE_PWR_BD_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Hardware-Specific Board Parameters                                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_BOARD_PARAMETERS_SE_PWR SE Power Board Hardware Parameters
 * @brief Contains definitions specific to the SE Power Board hardware.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* ADC Scaling Parameters                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_ADC_SCALING_SE_PWR ADC Scaling and Reference Values
 * @ingroup HW_BOARD_PARAMETERS_SE_PWR
 * @brief Defines the scaling factors and reference voltages for ADC conversion.
 * @{
 */

/**
 * @brief The reference voltage for the ADC peripheral.
 * @details This is the voltage corresponding to the maximum ADC raw value.
 * Unit: Volts (V)
 */
#define ADC_REFERENCE_V ((0.32))

/**
 * @brief The maximum measurable current.
 * @details This is the physical current value in Amperes that corresponds to the
 * full-scale ADC input voltage (i.e., ADC_REFERENCE_V).
 * Unit: Amperes (A)
 */
#define ADC_FULLSCALE_CURRENT_A ((40.0))

/**
 * @brief The maximum measurable DC bus voltage.
 * @details This is the physical voltage value in Volts that corresponds to the
 * full-scale ADC input voltage (i.e., ADC_REFERENCE_V).
 * Unit: Volts (V)
 */
#define ADC_FULLSCALE_VOLTAGE_V ((617.18))

/**
 * @brief The ADC voltage level corresponding to zero current.
 * @details This value represents the offset in the current sensing circuit.
 * Unit: Volts (V)
 */
#define ADC_CURRENT_OFFSET_V ((0.16))

/**
 * @brief The ADC voltage level corresponding to zero DC bus voltage.
 * @details This value represents the offset in the voltage sensing circuit.
 * For a unipolar voltage sensor, this is typically zero.
 * Unit: Volts (V)
 */
#define ADC_VOLTAGE_OFFSET_V ((0.0))

/** @} */ // end of HW_ADC_SCALING_SE_PWR group
/** @} */ // end of HW_BOARD_PARAMETERS_SE_PWR group

#ifdef __cplusplus
}
#endif // __cplusplus

/*
 * Engineering Notes: Current Calibration Data
 * ===========================================
 * This table provides historical context for the current sensing design across different power ratings.
 *
 * PWR_BD:            100W      400W      750W      1000W     1500W     2kW       3kW       5kW       test
 * Rs(ohm):           0.025     0.015     0.0075    0.005     0.004     0.0025    0.002     0.0015    0.006
 * SD_FULLDATA:       320
 * m_iqIuMeasGain:    0.04266   0.07111   0.14222   0.21333   0.26666   0.42666   0.53333   0.71111   0.17777
 * uiIu_usr_sc:       1280      2133      4266      6400      8000      12800     16000     21333     5333
 * uiIu_usr_sc_min:   1216      2026.35   4052.7    6080      7600      12160     15200     20266.35  5066.35
 * uiIu_usr_sc_max:   1344      2239.65   4479.3    6720      8400      13440     16800     22399.65  5599.65
 */

#endif // _FILE_SE_PWR_BD_H_
