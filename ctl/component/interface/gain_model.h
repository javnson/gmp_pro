/**
 * @file gain_model.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief Provides helper functions to calculate ADC gain values for common sensor configurations.
 * @details The gain is a crucial parameter for converting a per-unit ADC value back
 * into a physical quantity (e.g., Volts, Amperes). These models simplify the process
 * of determining the correct gain based on hardware components.
 * @version 0.2
 * @date 2025-03-21
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_GAIN_MODEL_H_
#define _FILE_GAIN_MODEL_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup ADC_GAIN_MODELS ADC Gain Calculation Models
 * @brief A collection of functions to determine the correct gain for sensor signal conditioning.
 *
 * These models calculate the gain required by the ADC channel modules to scale the
 * normalized ADC output into a meaningful physical unit.
 */

/*---------------------------------------------------------------------------*/
/* ADC Gain Calculation Models                                               */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup ADC_GAIN_MODELS
 * @{
 */

/**
 * @brief A generic function to calculate the required system gain.
 * @details This function calculates the gain needed to convert a per-unit ADC value
 * to a final value scaled by a physical base.
 * The formula is:
 * //tex: \text{gain} = \frac{V_{\text{ref}}}{G_{\text{sensor}} \cdot \text{Base}_{\text{physical}}}
 * @f$
 * \text{gain} = \frac{V_{\text{ref}}}{G_{\text{sensor}} \cdot \text{Base}_{\text{physical}}}
 * @f$
 * @param Vref The ADC reference voltage (e.g., 3.3V).
 * @param G_sensor The hardware gain of the analog front-end (e.g., the ratio of a voltage divider or the V/A sensitivity of a current sensor).
 * @param Base_physical The maximum physical value that corresponds to 1.0 p.u. in the control system (e.g., 400V or 50A).
 * @return The calculated gain for the ADC channel module.
 */
GMP_STATIC_INLINE parameter_gt ctl_gain_calc_generic(parameter_gt Vref, parameter_gt G_sensor,
                                                     parameter_gt Base_physical)
{
    gmp_base_assert(G_sensor != 0);
    gmp_base_assert(Base_physical != 0);
    return Vref / G_sensor / Base_physical;
}

/**
 * @brief Calculates gain for a voltage sensor using a resistive divider.
 * @details The sensor hardware consists of two resistors, R1 and R2, forming a voltage divider.
 * The hardware gain of this stage is @f$ G_{\text{sensor}} = \frac{R_2}{R_1 + R_2} @f$.
 * @param Vref The ADC reference voltage.
 * @param Vbase The maximum voltage to be measured (the physical base).
 * @param R1 The resistance of the resistor connected between the source voltage and the ADC input.
 * @param R2 The resistance of the resistor connected between the ADC input and ground.
 * @return The calculated gain for the ADC channel module.
 */
GMP_STATIC_INLINE parameter_gt ctl_gain_calc_volt_divider(parameter_gt Vref, parameter_gt Vbase, parameter_gt R1,
                                                          parameter_gt R2)
{
    parameter_gt R_total = R1 + R2;
    gmp_base_assert(R_total != 0);
    parameter_gt G_sensor = R2 / R_total;
    return ctl_gain_calc_generic(Vref, G_sensor, Vbase);
}

/**
 * @brief Calculates gain for a current sensor using a shunt resistor and an operational amplifier.
 * @details The sensor measures current via the voltage drop across a shunt resistor, which is then
 * amplified by an op-amp. The hardware gain is @f$ G_{\text{sensor}} = R_{\text{shunt}} \cdot G_{\text{opamp}} @f$ (in V/A).
 * @param Vref The ADC reference voltage.
 * @param Ibase The maximum current to be measured (the physical base).
 * @param R_shunt The resistance of the shunt resistor in Ohms.
 * @param G_opamp The voltage gain of the operational amplifier.
 * @return The calculated gain for the ADC channel module.
 */
GMP_STATIC_INLINE parameter_gt ctl_gain_calc_shunt_amp(parameter_gt Vref, parameter_gt Ibase, parameter_gt R_shunt,
                                                       parameter_gt G_opamp)
{
    parameter_gt G_sensor = R_shunt * G_opamp;
    return ctl_gain_calc_generic(Vref, G_sensor, Ibase);
}

/**
 * @brief Calculates gain for a Hall-effect current sensor.
 * @details This model is for integrated current sensors (e.g., ACS712) that have a specified
 * sensitivity. The hardware gain is the sensor's sensitivity.
 * @param Vref The ADC reference voltage.
 * @param Ibase The maximum current to be measured (the physical base).
 * @param sensitivity The sensor's sensitivity in Volts per Ampere (V/A).
 * @return The calculated gain for the ADC channel module.
 */
GMP_STATIC_INLINE parameter_gt ctl_gain_calc_hall_sensor(parameter_gt Vref, parameter_gt Ibase,
                                                         parameter_gt sensitivity)
{
    // G_sensor is the sensitivity
    return ctl_gain_calc_generic(Vref, sensitivity, Ibase);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GAIN_MODEL_H_
