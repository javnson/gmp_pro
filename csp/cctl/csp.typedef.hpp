/**
 * @file csp.typedef.hpp
 * @brief C++ simulation scalar customization for the hosted CCTL CSP.
 */

#ifndef GMP_CSP_CCTL_TYPEDEF_HPP
#define GMP_CSP_CCTL_TYPEDEF_HPP

extern "C"
{
#include <csp.typedef.h>
}

/**
 * @brief Scalar used by continuous plant and peripheral simulation models.
 *
 * Define GMP_CCTL_SIM_REAL_TYPE before including GMP headers to select another
 * compatible scalar.  Controller code remains governed independently by
 * ctrl_gt and defaults to float.
 */
#ifndef GMP_CCTL_SIM_REAL_TYPE
#define GMP_CCTL_SIM_REAL_TYPE double
#endif

using sim_real_gt = GMP_CCTL_SIM_REAL_TYPE;

#endif /* GMP_CSP_CCTL_TYPEDEF_HPP */
