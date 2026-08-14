/**
 * @file csp.general.hpp
 * @brief C++ services supplied by the hosted Linux CSP.
 */

#ifndef GMP_LINUX_SIMULINK_CSP_GENERAL_HPP
#define GMP_LINUX_SIMULINK_CSP_GENERAL_HPP

extern "C"
{
#include <csp.general.h>
}

#include <ctrl_rt_trace.h>

extern trace_rt_context_t trace_rt_context;

#endif /* GMP_LINUX_SIMULINK_CSP_GENERAL_HPP */
