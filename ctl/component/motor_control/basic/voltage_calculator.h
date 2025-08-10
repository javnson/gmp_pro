/**
 * @file voltage_calculator.h
 * @author Javnson (javnson@zju.edu.cn)
 * @brief A module to calculate phase voltages based on DC bus voltage and PWM duty cycles.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 * This module reconstructs the three-phase AC voltages (Ua, Ub, Uc) and the
 * stationary reference frame voltages (Ualpha, Ubeta) from the measured DC bus
 * voltage and the commanded SVPWM duty cycle values.
 */

#ifndef _FILE_VOLTAGE_CALCULATOR_H_
#define _FILE_VOLTAGE_CALCULATOR_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Voltage Calculator from PWM                                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_VOLTAGE_CALCULATOR Voltage Calculator
 * @brief Calculates AC phase voltages from DC bus voltage and SVPWM signals.
 * @details The phase voltages are calculated as:
 * @f[
 * U_x = \frac{U_{bus}}{\sqrt{3}} \cdot (2 \cdot T_x - T_y - T_z)
 * @f]
 * where $$T_x, T_y, T_z$$ are the SVPWM timings.
 *
 * The Clarke transformation is:
 * @f[
 * U_\alpha = U_a \\
 * U_\beta = \frac{1}{\sqrt{3}} (U_a + 2 \cdot U_b)
 * @f]
 *

 * @{
 */

/**
 * @brief Data structure for the voltage calculator.
 */
typedef struct _tag_voltage_calculator
{
    // Inputs
    ctrl_gt ubus;             /**< @brief Input: DC bus voltage, in per-unit (p.u.). */
    ctl_vector3_t svpwm_uabc; /**< @brief Input: Reference SVPWM modulation signals for each phase. */

    // Outputs
    ctrl_gt u_alpha;    /**< @brief Output: Calculated voltage in the stationary alpha-axis. */
    ctrl_gt u_beta;     /**< @brief Output: Calculated voltage in the stationary beta-axis. */
    ctl_vector3_t uabc; /**< @brief Output: Calculated three-phase voltages (Ua, Ub, Uc). */

} ctl_volt_calculate_t;

/**
 * @brief Calculates Uabc and Ualpha/Ubeta from DC bus voltage and SVPWM duty cycles.
 *
 * This function reconstructs the applied phase voltages based on the SVPWM modulation
 * signals and the measured DC bus voltage. It then performs a Clarke transformation
 * to obtain the alpha and beta components.
 *
 * @param[in,out] volt_calc Pointer to the voltage calculator structure. The function
 * reads `ubus` and `svpwm_uabc` and writes the results to `uabc`, `u_alpha`, and `u_beta`.
 * NOTE: The original function passed the struct by value, which was a critical error.
 * It has been corrected to pass by pointer.
 */
GMP_STATIC_INLINE void ctl_step_voltage_calculator(ctl_volt_calculate_t* volt_calc)
{
    // NOTE: Assumes phase_U, phase_V, phase_W are defined elsewhere (e.g., an enum).
    ctrl_gt temp = ctl_mul(volt_calc->ubus, GMP_CONST_1_OVER_SQRT3);

    /* Scale the incoming modulation functions with the DC bus voltage value */
    /* and calculate the 3-phase voltages */
    // NOTE: Corrected a typo `v.svpwm_uabc` to `volt_calc->svpwm_uabc`.
    volt_calc->uabc.dat[phase_U] =
        ctl_mul(temp, (ctl_mul2(volt_calc->svpwm_uabc.dat[phase_U]) - volt_calc->svpwm_uabc.dat[phase_V] -
                       volt_calc->svpwm_uabc.dat[phase_W]));
    volt_calc->uabc.dat[phase_V] =
        ctl_mul(temp, (ctl_mul2(volt_calc->svpwm_uabc.dat[phase_V]) - volt_calc->svpwm_uabc.dat[phase_U] -
                       volt_calc->svpwm_uabc.dat[phase_W]));
    volt_calc->uabc.dat[phase_W] =
        ctl_mul(temp, (ctl_mul2(volt_calc->svpwm_uabc.dat[phase_W]) - volt_calc->svpwm_uabc.dat[phase_V] -
                       volt_calc->svpwm_uabc.dat[phase_U]));

    /* Voltage transformation (a,b,c) -> (Alpha,Beta) - Clarke Transform */
    volt_calc->u_alpha = volt_calc->uabc.dat[phase_U];
    volt_calc->u_beta =
        ctl_mul((volt_calc->uabc.dat[phase_U] + ctl_mul2(volt_calc->uabc.dat[phase_V])), GMP_CONST_1_OVER_SQRT3);
}

/** @} */ // end of MC_VOLTAGE_CALCULATOR group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_VOLTAGE_CALCULATOR_H_
