/**
 * @file gmp_lvhb_v1.h
 * @author javnson (javnson@zju.edu.cn)
 * @brief Defines hardware-specific parameters for the Diansai Half-Bridge v1 board.
 * @version 1.05
 * @date 2025-05-28
 *
 * @copyright Copyright (c) 2025
 *
 * @details This header file contains calibration constants for the voltage and
 * current sensors on the Diansai v1 hardware platform, dated 2025-07-20. These
 * macros are used to convert raw ADC values into physical units (Volts and Amperes).
 */

#ifndef _FILE_DIANSAI_HALF_BRIDGE_V1_H_
#define _FILE_DIANSAI_HALF_BRIDGE_V1_H_

/**
 * @brief The gain of the voltage sensor circuit.
 * @details This value typically represents the voltage divider ratio (e.g., R2 / (R1 + R2)).
 * The formula to convert ADC reading to voltage is: V_real = (ADC_V - V_bias) / V_gain.
 */
#define DSV1_VOLTAGE_SENSOR_GAIN ((float)(2.2 / 202.2))

/**
 * @brief The bias voltage of the voltage sensor op-amp circuit, in Volts.
 * @details This is the output voltage of the sensor circuit when the input voltage is zero.
 */
#define DSV1_VOLTAGE_SENSOR_BIAS ((float)(0.9966))

/**
 * @brief The gain of the current sensor circuit.
 * @details This value is typically the product of the shunt resistor value and the
 * amplifier gain (e.g., R_shunt * G_amp).
 * The formula to convert ADC reading to current is: I_real = (ADC_V - I_bias) / I_gain.
 */
#define DSV1_CURRENT_SENSOR_GAIN ((float)(0.005 * 20.0))

/**
 * @brief The bias voltage of the current sensor op-amp circuit, in Volts.
 * @details This is the output voltage of the sensor circuit when the input current is zero.
 * For bidirectional sensors, this is typically half of the ADC reference voltage.
 */
#define DSV1_CURRENT_SENSOR_BIAS ((float)(0.9))

#endif // _FILE_DIANSAI_HALF_BRIDGE_V1_H_
