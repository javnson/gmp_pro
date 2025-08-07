/**
 * @file GMP_LV_3PH_GAN_INV.h
 * @brief Defines hardware-specific parameters for the GMP LV 3-Phase GaN Inverter board.
 * @details This file contains pre-defined constants related to the analog-to-digital
 * converter (ADC) scaling for current and voltage sensing. These values are
 * crucial for converting raw ADC readings into real-world physical units (Amperes and Volts).
 *
 * @version 1.0
 * @date 2025-08-06
 */

#ifndef _FILE_GMP_LV_3PH_GAN_INV_
#define _FILE_GMP_LV_3PH_GAN_INV_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Hardware-Specific Board Parameters                                        */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_BOARD_PARAMETERS Inverter Hardware Parameters
 * @brief Contains definitions specific to the GMP LV 3-Phase GaN Inverter hardware.
 * @{
 */

/*---------------------------------------------------------------------------*/
/* ADC Scaling Parameters                                                    */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup HW_ADC_SCALING ADC Scaling and Reference Values
 * @ingroup HW_BOARD_PARAMETERS
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
#define ADC_FULLSCALE_CURRENT_A ((65.0))

/**
 * @brief The maximum measurable DC bus voltage.
 * @details This is the physical voltage value in Volts that corresponds to the
 * full-scale ADC input voltage (i.e., ADC_REFERENCE_V).
 * Unit: Volts (V)
 */
#define ADC_FULLSCALE_VOLTAGE_V ((201.3))

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

/** @} */ // end of HW_ADC_SCALING group
/** @} */ // end of HW_BOARD_PARAMETERS group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_LV_3PH_GAN_INV_
