
/**
 * @file consultant_nameplate.h
 * @brief Implements the Nameplate Consultant (Factory Builder) for PMSM and IM.
 *
 * @version 1.0
 * @date 2024-10-27
 */

#include <ctl/component/motor_control/consultant/acim_consultant.h>
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>
#include <ctl/component/motor_control/consultant/pu_consultant.h>

#ifndef _FILE_CONSULTANT_NAMEPLATE_H_
#define _FILE_CONSULTANT_NAMEPLATE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Consultant: Motor Nameplate (Factory Builder)                             */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CONSULTANT_NAMEPLATE Nameplate Consultant
 * @brief Parses human-readable motor datasheets and builds rigorous physical models.
 * @details Handles all engineering unit conversions (Vrms to Vpeak, Line-to-Line 
 * to Phase, RPM to rad/s, Ke/Kt to Flux). Exports fully configured PMSM, IM, 
 * and PU consultant structures.
 * @{
 */

//================================================================================
// 1. PMSM Nameplate Definition
//================================================================================

typedef struct _tag_nameplate_pmsm
{
    // --- Datasheet / Nameplate Ratings ---
    parameter_gt rated_power_w;      //!< Rated mechanical power (W).
    parameter_gt rated_voltage_vrms; //!< Rated Line-to-Line voltage (Vrms).
    parameter_gt rated_current_arms; //!< Rated phase current (Arms).
    parameter_gt rated_speed_rpm;    //!< Rated speed (rpm).
    uint32_t pole_pairs;             //!< Number of pole pairs (p).

    // --- Engineering Constants (At least one must be provided if flux is unknown) ---
    parameter_gt ke_vrms_krpm; //!< Back-EMF constant (Vrms L-L / krpm).
    parameter_gt kt_nm_arms;   //!< Torque constant (Nm / Arms).
    parameter_gt flux_weber;   //!< PM Flux linkage (Weber). Direct input if known.

    // --- Equivalent Circuit Parameters ---
    parameter_gt rs_ohm;   //!< Stator phase resistance (Ohm).
    parameter_gt ld_henry; //!< D-axis inductance (H).
    parameter_gt lq_henry; //!< Q-axis inductance (H).

} ctl_nameplate_pmsm_t;

//================================================================================
// 2. IM Nameplate Definition
//================================================================================

typedef struct _tag_nameplate_im
{
    // --- Datasheet / Nameplate Ratings ---
    parameter_gt rated_power_w;      //!< Rated mechanical power (W).
    parameter_gt rated_voltage_vrms; //!< Rated Line-to-Line voltage (Vrms).
    parameter_gt rated_current_arms; //!< Rated phase current (Arms).
    parameter_gt rated_freq_hz;      //!< Rated stator electrical frequency (Hz).
    parameter_gt rated_speed_rpm;    //!< Rated mechanical speed (rpm).
    parameter_gt power_factor;       //!< Nominal power factor (cos_phi).

    // --- Equivalent Circuit Parameters (Usually from Auto-tuning) ---
    parameter_gt rs_ohm;   //!< Stator resistance (Ohm).
    parameter_gt rr_ohm;   //!< Rotor resistance (Ohm).
    parameter_gt ls_henry; //!< Stator self-inductance (H).
    parameter_gt lr_henry; //!< Rotor self-inductance (H).
    parameter_gt lm_henry; //!< Mutual inductance (H).

} ctl_nameplate_im_t;

//================================================================================
// Function Prototypes & Factory APIs
//================================================================================

/**
 * @brief Builds the rigorous PMSM physical model and PU base model from a nameplate.
 * @details Automatically resolves missing Flux linkage from Ke or Kt if necessary,
 * and converts RMS Line-to-Line ratings into Peak Phase bases for PU normalization.
 * * @param[in]  np         Pointer to the populated PMSM nameplate data.
 * @param[out] out_motor  Pointer to the destination PMSM physical model.
 * @param[out] out_pu     Pointer to the destination PU base model.
 */
void ctl_nameplate_build_pmsm(const ctl_nameplate_pmsm_t* np, ctl_consultant_pmsm_t* out_motor,
                              ctl_consultant_pu_pmsm_t* out_pu);

/**
 * @brief Builds the rigorous IM physical model and PU base model from a nameplate.
 * @details Automatically deduces the number of pole pairs from rated Hz and RPM,
 * calculates nominal slip, and converts RMS ratings into PU bases.
 * * @param[in]  np         Pointer to the populated IM nameplate data.
 * @param[out] out_motor  Pointer to the destination IM physical model.
 * @param[out] out_pu     Pointer to the destination PU base model.
 */
void ctl_nameplate_build_im(const ctl_nameplate_im_t* np, ctl_consultant_im_t* out_motor,
                            ctl_consultant_pu_im_t* out_pu);

/**
 * @brief Utility: Calculates nominal IM slip frequency from nameplate.
 * @return parameter_gt Nominal slip frequency (rad/s electrical).
 */
parameter_gt ctl_nameplate_im_calc_rated_slip_rads(const ctl_nameplate_im_t* np);

/** @} */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CONSULTANT_NAMEPLATE_H_
