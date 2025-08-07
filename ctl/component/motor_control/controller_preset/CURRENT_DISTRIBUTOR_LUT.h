/**
 * @file CURRENT_DISTRIBUTOR_LUT.h
 * @brief Defines a lookup table (LUT) for a custom current distribution strategy.
 * @details This module provides a pre-defined table that maps a total current
 * magnitude to a specific d-q axis current angle (alpha). This is often used
 * for control strategies like that of a Synchronous Reluctance Motor (SynRM)
 * or Interior Permanent Magnet (IPM) motor, where the optimal torque angle
 * changes with the current level.
 *
 * @version 1.0
 * @date 2025-08-06
 */

#ifndef _FILE_CURRENT_DISTRIBUTOR_LUT_H_
#define _FILE_CURRENT_DISTRIBUTOR_LUT_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------*/
/* Current Distributor Lookup Table                                          */
/*---------------------------------------------------------------------------*/

/**
 * @defgroup CURRENT_DISTRIBUTOR_LUT Current Distributor LUT
 * @ingroup CTL_MC_PRESET
 * @brief Provides a lookup table for mapping current magnitude to a control angle.
 * @{
 */

//================================================================================
// Type Defines & Data
//================================================================================

#ifndef CTRL_GT_DEFINED
#define CTRL_GT_DEFINED
typedef float ctrl_gt;
#endif

/**
 * @brief Defines the structure for a single entry in the current distribution LUT.
 */
typedef struct
{
    ctrl_gt im;    /**< @brief The input current magnitude (p.u. or Amperes). */
    ctrl_gt alpha; /**< @brief The output control angle, normalized to [0, 1.0] representing [0, 360бу]. */
} current_lookup_table_t;

/**
 * @brief The current distribution lookup table.
 * @details This table maps a current magnitude (`im`) to a corresponding
 * advance angle (`alpha`). The angle determines the distribution of the total
 * current into d-axis and q-axis components.
 *
 * The `alpha` value is stored as a normalized per-unit value, where 1.0 corresponds to 360 degrees.
 * For example, an angle of 110бу is stored as 110/360 = 0.3056.
 */
static const current_lookup_table_t CURRENT_DISTRIBUTION_LUT[] = {
    {0.0000f, 0.3056f}, // im = 0.0,  alpha = 110.0бу
    {0.0623f, 0.3222f}, // im = 2.0,  alpha = 116.0бу
    {0.1246f, 0.3394f}, // im = 4.0,  alpha = 122.2бу
    {0.1870f, 0.3500f}, // im = 6.0,  alpha = 126.0бу
    {0.2493f, 0.3533f}, // im = 8.0,  alpha = 127.2бу
    {0.3116f, 0.3589f}, // im = 10.0, alpha = 129.2бу
    {0.3739f, 0.3639f}, // im = 12.0, alpha = 131.0бу
    {0.4363f, 0.3700f}, // im = 14.0, alpha = 133.2бу
    {0.4986f, 0.3772f}, // im = 16.0, alpha = 135.8бу
    {0.5609f, 0.3844f}, // im = 18.0, alpha = 138.4бу
    {0.6232f, 0.3883f}, // im = 20.0, alpha = 139.8бу
    {0.6856f, 0.3944f}, // im = 22.0, alpha = 142.0бу
    {0.7479f, 0.3978f}, // im = 24.0, alpha = 143.2бу
    {0.8102f, 0.4000f}, // im = 26.0, alpha = 144.0бу
    {0.8725f, 0.4017f}, // im = 28.0, alpha = 144.6бу
    {0.9349f, 0.4050f}, // im = 30.0, alpha = 145.8бу
    {0.9972f, 0.4078f}, // im = 32.0, alpha = 146.8бу
    {1.0595f, 0.4094f}, // im = 34.0, alpha = 147.4бу
    {1.1218f, 0.4106f}, // im = 36.0, alpha = 147.8бу
    {1.1842f, 0.4117f}, // im = 38.0, alpha = 148.2бу
    {1.2465f, 0.4133f}, // im = 40.0, alpha = 148.8бу
    {1.3088f, 0.4144f}, // im = 42.0, alpha = 149.2бу
    {1.3711f, 0.4156f}, // im = 44.0, alpha = 149.6бу
    {1.4335f, 0.4167f}, // im = 46.0, alpha = 150.0бу
    {1.4958f, 0.4183f}, // im = 48.0, alpha = 150.6бу
    {1.5581f, 0.4211f}  // im = 50.0, alpha = 151.6бу
};

//================================================================================
// Helper Macros
//================================================================================

/**
 * @brief Calculates the total number of entries in the LUT.
 */
#define CURRENT_DISTRIBUTION_LUT_SIZE (sizeof(CURRENT_DISTRIBUTION_LUT) / sizeof(current_lookup_table_t))

/**
 * @brief Calculates the step size between current magnitude entries in the LUT.
 * @details This assumes the table has a fixed, uniform step size.
 */
#define CURRENT_DISTRIBUTION_LUT_STEP (CURRENT_DISTRIBUTION_LUT[1].im - CURRENT_DISTRIBUTION_LUT[0].im)

/**
 * @brief A default or fixed advance angle, normalized to [0, 1.0].
 * @details Represents 135 degrees. This might be used as a fallback value.
 */
#define CURRENT_DISTRIBUTION_ALPHA (135.0f / 360.0f)

/** @} */ // end of CURRENT_DISTRIBUTOR_LUT group

#ifdef __cplusplus
}
#endif

#endif // _FILE_CURRENT_DISTRIBUTOR_LUT_H_
