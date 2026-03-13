/**
 * @file error_code.c
 * @author Javnson (javnson@zju.edu.cn)
 * @brief 
 * @version 0.1
 * @date 2024-09-30
 * 
 * @copyright Copyright GMP(c) 2024
 * 
 */

#include <gmp_core.h>


//////////////////////////////////////////////////////////////////////////
// Step I: implement the global variables
// 

// This variable may save the last error code (any return value will include).
// These code may help developer locate faults rapidly.
ec_gt g_gmp_last_ret;


// This variable may save the last error (fatal and error) code.
// These code may help developer locate faults rapidly.
ec_gt g_gmp_last_error;


// This variable may save the last fatal (only fatal) code.
// These code may help developer locate faults rapidly.
ec_gt g_gmp_last_fatal;


//////////////////////////////////////////////////////////////////////////
// Step II: include the error code show function
//

#include <core/std/ec/erro_code.show.inl>

