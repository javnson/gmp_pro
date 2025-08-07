/**
 * @file TI_3PH_GAN_INV.h
 * @brief Defines hardware-specific parameters for the TI 3-Phase GaN Inverter board.
 * @details This file contains pre-defined constants related to the analog-to-digital
 * converter (ADC) scaling for current and voltage sensing on this specific hardware.
 *
 * @version 1.0
 * @date 2025-08-06
 */

#ifndef _FILE_TI_3PH_GAN_INV_H_
#define _FILE_TI_3PH_GAN_INV_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Hardware-Specific Board Parameters                                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_BOARD_PARAMETERS_TI_GAN TI GaN Inverter Hardware Parameters
 * @ingroup CTL_MC_PRESET
 * @brief Contains definitions specific to the TI 3-Phase GaN Inverter hardware.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* ADC Scaling Parameters                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_ADC_SCALING_TI_GAN ADC Scaling and Reference Values
 * @ingroup HW_BOARD_PARAMETERS_TI_GAN
 * @brief Defines the scaling factors and reference voltages for ADC conversion.
 * @{
 */

/**
 * @brief The reference voltage for the ADC peripheral.
 * @details This is the voltage corresponding to the maximum ADC raw value.
 * Unit: Volts (V)
 */
#define ADC_REFERENCE_V ((3.3))

/**
 * @brief The maximum measurable current.
 * @details This is the physical current value in Amperes that corresponds to the
 * full-scale ADC input voltage (i.e., ADC_REFERENCE_V).
 * Unit: Amperes (A)
 */
#define ADC_FULLSCALE_CURRENT_A ((16.5))

/**
 * @brief The maximum measurable DC bus voltage.
 * @details This is the physical voltage value in Volts that corresponds to the
 * full-scale ADC input voltage (i.e., ADC_REFERENCE_V).
 * Unit: Volts (V)
 */
#define ADC_FULLSCALE_VOLTAGE_V ((78.199))

/**
 * @brief The ADC voltage level corresponding to zero current.
 * @details This value represents the offset in the current sensing circuit.
 * For a bipolar current sensor, this is typically half of the ADC reference voltage.
 * Unit: Volts (V)
 */
#define ADC_CURRENT_OFFSET_V ((1.65))

/**
 * @brief The ADC voltage level corresponding to zero DC bus voltage.
 * @details This value represents the offset in the voltage sensing circuit.
 * For a unipolar voltage sensor, this is typically zero.
 * Unit: Volts (V)
 */
#define ADC_VOLTAGE_OFFSET_V ((0.0))

/** @} */ // end of HW_ADC_SCALING_TI_GAN group
/** @} */ // end of HW_BOARD_PARAMETERS_TI_GAN group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_TI_3PH_GAN_INV_H_
