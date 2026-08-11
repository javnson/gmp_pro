/**
 * @file GMP_SIL_Core.cpp
 * @brief MATLAB MEX and Simulink build entry for the unified GMP SIL core.
 *
 * The S-function implementation is kept in the transport tools tree.  The
 * Simulink library receives only the compiled MEX artifact, so there is no
 * second source copy to maintain under slib.
 */

#if defined(_MSC_VER)
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "mswsock.lib")
#endif

#include <tools/gmp_sil/sil_helper/mdl_gmp_sil_core.cpp>
