/**
 * @file svpwm.h
 * @author Javnson (javnson@zju.edu.cn), GUO Qichen
 * @brief Implements the Space Vector Pulse Width Modulation (SVPWM) technique.
 * @version 0.1
 * @date 2024-09-30
 *
 * @copyright Copyright GMP(c) 2024
 *
 */

#ifndef _FILE_SVPWM_H_
#define _FILE_SVPWM_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Space Vector Pulse Width Modulation (SVPWM)                               */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup MC_SVPWM Space Vector Pulse Width Modulation (SVPWM)
 * @brief This module contains the structures and functions for SVPWM calculations.
 *
 * The SVPWM module converts a two-phase orthogonal stationary reference frame
 * vector @f((U_\alpha, U_\beta)@f) into three-phase PWM signals.
 * The process is typically divided into two stages:
 * 1.  **Calculation Stage**: Transforms the @f((U_\alpha, U_\beta)@f) vector into intermediate
 * duty cycle timings (Ta, Tb, Tc).
 * 2.  **Modulation Stage**: Converts the intermediate timings into final PWM
 * compare values for the timer peripheral.
 * @{
 */

/**
 * @brief Data structure for the SVPWM channel.
 *
 * This structure holds all the necessary variables for a single SVPWM instance,
 * including inputs, outputs, and parameters.
 */
typedef struct _tag_svpwm_channel_t
{
    // Input variables
    ctrl_gt Ualpha; /**< @brief Input voltage component in the stationary alpha-axis ($$U_\alpha$$). */
    ctrl_gt Ubeta;  /**< @brief Input voltage component in the stationary beta-axis ($$U_\beta$$). */

    // Output stage I variables
    ctrl_gt T[3]; /**< @brief Intermediate three-phase duty cycle timings (Ta, Tb, Tc) ranging from -0.5 to 0.5. */

    // Parameters
    pwm_gt pwm_period; /**< @brief The time base period of the PWM timer (e.g., timer counts for 100% duty cycle). */

    // Output stage II variables
    pwm_gt pwm_cmp[3]; /**< @brief Final PWM compare values for the three output channels. */

} ctl_svpwm_channel_t;

/**
 * @brief Initializes the SVPWM channel structure.
 * @param[in,out] svpwm Pointer to the SVPWM channel structure to initialize.
 * @param[in] pwm_period The PWM time base period.
 */
void ctl_init_svpwm(ctl_svpwm_channel_t* svpwm, pwm_gt pwm_period);

/**
 * @brief Sets the input voltages ($$U_\alpha$$, $$U_\beta$$) for the SVPWM calculation.
 * @param[in,out] svpwm Pointer to the SVPWM channel structure.
 * @param[in] ab Pointer to a vector containing the alpha and beta voltage components.
 */
GMP_STATIC_INLINE void ctl_set_svpwm_via_ab(ctl_svpwm_channel_t* svpwm, ctl_vector3_t* ab)
{
    svpwm->Ualpha = ab->dat[0];
    svpwm->Ubeta = ab->dat[1];
}

/**
 * @brief (Stage I) Calculates three-phase duty ratios from the alpha-beta vector.
 *
 * This function implements an SVPWM algorithm by calculating an equivalent
 * three-phase system and injecting a common-mode voltage to center the signals.
 * This method is equivalent to centering the pulse widths in the PWM period.
 *
 * The transformation from ($$U_\alpha$$, $$U_\beta$$) to the intermediate phase voltages
 * ($$U_a$$, $$U_b$$, $$U_c$$) is as follows:
 * @f[
 * U_a = U_\alpha, \\
 * U_b = -U_\alpha /2 + \sqrt{3}/2\cdot U_\beta, \\
 * U_c = -U_\alpha /2 - \sqrt{3}/2\cdot U_\beta,
 * @f]
 *
 * @param[in,out] svpwm Pointer to the SVPWM channel structure. The inputs @ref ctl_svpwm_channel_t::Ualpha and
 * @ref ctl_svpwm_channel_t::Ubeta are used, and the result is stored in the @ref ctl_svpwm_channel_t::T array.
 */
GMP_STATIC_INLINE void ctl_svpwm_calc(ctl_svpwm_channel_t* svpwm)
{
    ctrl_gt Ua, Ub, Uc; // Uabc three phase parameters
    ctrl_gt Umax, Umin, Ucom;

    ctrl_gt Ualpha_tmp = -ctl_div2(svpwm->Ualpha);
    ctrl_gt Ubeta_tmp = ctl_mul(svpwm->Ubeta, GMP_CONST_SQRT_3_OVER_2);

    Ua = svpwm->Ualpha;
    Ub = Ualpha_tmp + Ubeta_tmp;
    Uc = Ualpha_tmp - Ubeta_tmp;

    // find Vmax & Vmin
    if (Ua > Ub)
    {
        Umax = Ua;
        Umin = Ub;
    }
    else
    {
        Umax = Ub;
        Umin = Ua;
    }

    if (Uc > Umax)
        Umax = Uc;
    else if (Uc < Umin)
        Umin = Uc;

    // get common mode term
    Ucom = ctl_div2(Umax + Umin);

    // get SVPWM modulation result
    svpwm->T[0] = Ua - Ucom;
    svpwm->T[1] = Ub - Ucom;
    svpwm->T[2] = Uc - Ucom;
}

/**
 * @brief (Stage I) Calculates three-phase duty ratios using a sector-based algorithm.
 *
 * This function provides an alternative implementation for the Stage I calculation.
 * It determines the sector based on the reference vector and calculates the dwell
 * times for the adjacent space vectors (T1, T2). These timings are then used to
 * generate the final duty cycle ratios for each phase.
 *
 * @param[in,out] svpwm Pointer to the SVPWM channel structure. The inputs  @ref ctl_svpwm_channel_t::Ualpha and
 * @ref ctl_svpwm_channel_t::Ubeta are used, and the result is stored in the @ref ctl_svpwm_channel_t::T array.
 */
GMP_STATIC_INLINE void ctl_svpwm_calc2(ctl_svpwm_channel_t* svpwm)
{
    // u2s: Ualpha ,Ubeta
    ctrl_gt X, Y, Z, T1, T2, Ta, Tb, Tc;
    uint16_t N;
    ctrl_gt Uabc[3] = {0};

    Uabc[0] = svpwm->Ubeta;
    Uabc[1] = ctl_mul(GMP_CONST_SQRT_3_OVER_2, svpwm->Ualpha) - ctl_div2(svpwm->Ubeta);
    Uabc[2] = -ctl_mul(GMP_CONST_SQRT_3_OVER_2, svpwm->Ualpha) - ctl_div2(svpwm->Ubeta);

    N = ((Uabc[0] > 0)) + ((Uabc[1] > 0) << 1) + ((Uabc[2] > 0) << 2);
    X = ctl_mul(GMP_CONST_SQRT_3, svpwm->Ubeta);
    Y = ctl_mul(GMP_CONST_3_OVER_2, svpwm->Ualpha) + ctl_mul(GMP_CONST_SQRT_3_OVER_2, svpwm->Ubeta);
    Z = -ctl_mul(GMP_CONST_3_OVER_2, svpwm->Ualpha) + ctl_mul(GMP_CONST_SQRT_3_OVER_2, svpwm->Ubeta);

    switch (N)
    {
    case 1:
        T1 = Z;
        T2 = Y;
        break;
    case 2:
        T1 = Y;
        T2 = -X;
        break;
    case 3:
        T1 = -Z;
        T2 = X;
        break;
    case 4:
        T1 = -X;
        T2 = Z;
        break;
    case 5:
        T1 = X;
        T2 = -Y;
        break;
    case 6:
        T1 = -Y;
        T2 = -Z;
        break;
    default:
        T1 = 0;
        T2 = 0;
        break; // Default case added for safety
    }

    if ((T1 + T2) > GMP_CONST_1)
    {
        T1 = ctl_div(T1, (T1 + T2));
        T2 = GMP_CONST_1 - T1;
    }
    Ta = ctl_div4(GMP_CONST_1 - T1 - T2);
    Tb = Ta + ctl_div2(T1);
    Tc = Tb + ctl_div2(T2);

    Ta *= 2;
    Tb *= 2;
    Tc *= 2;

    Ta = -Ta;
    Tb = -Tb;
    Tc = -Tc;

    Ta += GMP_CONST_1_OVER_2;
    Tb += GMP_CONST_1_OVER_2;
    Tc += GMP_CONST_1_OVER_2;

    switch (N)
    {
    case 0:
        svpwm->T[0] = 0;
        svpwm->T[1] = 0;
        svpwm->T[2] = 0;
        break;
    case 1:
        svpwm->T[0] = Tb;
        svpwm->T[1] = Ta;
        svpwm->T[2] = Tc;
        break;
    case 2:
        svpwm->T[0] = Ta;
        svpwm->T[1] = Tc;
        svpwm->T[2] = Tb;
        break;
    case 3:
        svpwm->T[0] = Ta;
        svpwm->T[1] = Tb;
        svpwm->T[2] = Tc;
        break;
    case 4:
        svpwm->T[0] = Tc;
        svpwm->T[1] = Tb;
        svpwm->T[2] = Ta;
        break;
    case 5:
        svpwm->T[0] = Tc;
        svpwm->T[1] = Ta;
        svpwm->T[2] = Tb;
        break;
    case 6:
        svpwm->T[0] = Tb;
        svpwm->T[1] = Tc;
        svpwm->T[2] = Ta;
        break;
    default:
        svpwm->T[0] = 0;
        svpwm->T[1] = 0;
        svpwm->T[2] = 0;
        break;
    }

    return;
}

/**
 * @brief (Stage II) Generates final PWM compare values from duty cycle timings.
 *
 * This function converts the intermediate duty cycle timings (@ref ctl_svpwm_channel_t::T array, nominally
 * in the range [-0.5, 0.5]) into absolute PWM compare values. It does this by
 * shifting the range to [0, 1.0] and then scaling by the PWM period.
 *
 * @param[in,out] svpwm Pointer to the SVPWM channel structure. The @ref ctl_svpwm_channel_t::T array and
 * @ref ctl_svpwm_channel_t::pwm_period are used, and the result is stored in @ref ctl_svpwm_channel_t::pwm_cmp.
 */
GMP_STATIC_INLINE void ctl_svpwm_modulation(ctl_svpwm_channel_t* svpwm)
{
    int i = 0;
    ctrl_gt pwm_data;
    pwm_gt pwm_output;

    for (i = 0; i < 3; ++i)
    {
        // Shift duty cycle from [-0.5, 0.5] to [0, 1.0]
        pwm_data = svpwm->T[i] + float2ctrl(0.5f);
        // Prevent data error
        pwm_data = pwm_data < 0 ? 0 : pwm_data;
        // Scale to PWM period
        pwm_output = (pwm_gt)ctl_mul(pwm_data, svpwm->pwm_period);
        // Clamp output to the maximum period value
        svpwm->pwm_cmp[i] = pwm_output > svpwm->pwm_period ? svpwm->pwm_period : pwm_output;
    }
}

/**
 * @brief (Stage II) Generates final PWM compare values (with inverse modulation).
 *
 * @note There appears to be a logical error in this function. The inverse
 * modulation calculation is immediately overwritten. See analysis for details.
 *
 * @param[in,out] svpwm Pointer to the SVPWM channel structure.
 */
GMP_STATIC_INLINE void ctl_svpwm_inv_modulation(ctl_svpwm_channel_t* svpwm)
{
    int i = 0;
    ctrl_gt pwm_data;
    pwm_gt pwm_output;

    for (i = 0; i < 3; ++i)
    {
        pwm_data = svpwm->T[i] + float2ctrl(0.5f);

        // Prevent data error
        pwm_data = pwm_data < 0 ? 0 : pwm_data;
        // Scale to PWM period
        pwm_output = (pwm_gt)ctl_mul(pwm_data, svpwm->pwm_period);
        // Clamp output to the maximum period value
        svpwm->pwm_cmp[i] = pwm_output > svpwm->pwm_period ? svpwm->pwm_period : pwm_output;
    }
}

/** 
 * @} 
 */ // end of MC_SVPWM group

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_SVPWM_H_
