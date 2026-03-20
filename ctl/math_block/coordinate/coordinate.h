
#include <ctl/math_block/coordinate/coordinate.h>
#include <ctl/math_block/vector_lite/vector2.h>

#ifndef _FILE_GMP_MATH_COORDINATE_H_
#define _FILE_GMP_MATH_COORDINATE_H_

#if __cplusplus
extern "C"
{
#endif // __cplusplus

/*---------------------------------------------------------------------------*/
/* Coordinate Axis Definitions                                               */
/*---------------------------------------------------------------------------*/

#define GMP_CTL_OUTPUT_TAG

/**
 * @defgroup MC_COORD_AXIS_DEFINES Coordinate Axis Definitions
 * @ingroup CTL_MATH
 * @brief Enumerations for indexing different coordinate system axes.
 * @{
 */

/** @brief Enumeration for three-phase system axes (U, V, W). */
enum UVW_ASIX_ENUM
{
    phase_U = 0,
    phase_V = 1,
    phase_W = 2,
    phase_N = 3
};

/** @brief Enumeration for three-phase system axes (A, B, C). */
enum ABC_ASIX_ENUM
{
    phase_A = 0,
    phase_B = 1,
    phase_C = 2
};

/** @brief Enumeration for rotating reference frame axes (d, q, 0). */
enum DQ_ASIC_ENUM
{
    phase_d = 0,
    phase_q = 1,
    phase_0 = 2
};

/** @brief Enumeration for stationary reference frame axes (alpha, beta). */
enum ALPHA_BETA_ENUM
{
    phase_alpha = 0,
    phase_beta = 1
};

/** @brief Enumeration for line voltage phases (Uab, Ubc). */
enum LINE_VOLTAGE_ENUM
{
    phase_UAB = 0,
    phase_UBC = 1
};

/** @brief Enumeration for phasor components (sin, cos). */
enum PHASOR_ENUM
{
    phasor_sin = 0,
    phasor_cos = 1
};

/** @} */ // end of MC_COORD_AXIS_DEFINES group

/**
 * @brief Generates a 2D phasor (sin, cos) from a given angle.
 * @param[in] angle The input angle in per-unit (0 to 1 represents 0 to 2pi).
 * @param[out] phasor Pointer to the output 2D vector to store the phasor.
 */
GMP_STATIC_INLINE void ctl_set_phasor_via_angle(const ctrl_gt angle, GMP_CTL_OUTPUT_TAG ctl_vector2_t* phasor)
{
    phasor->dat[phasor_sin] = ctl_sin(angle);
    phasor->dat[phasor_cos] = ctl_cos(angle);
}

#if __cplusplus
}
#endif // __cplusplus

#endif // _FILE_GMP_MATH_COORDINATE_H_


