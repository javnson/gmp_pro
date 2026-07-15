/**
 * @file sdpe_demo_bindings.h
 * @brief SDPE project bindings for SDPE Demo Current Sensor Project.
 * @note Built-in SDPE manager demo project. It binds a concrete current sensor entity to a small project requirement header.
 */

#ifndef _PROJECT_SDPE_DEMO_BINDINGS_H_
#define _PROJECT_SDPE_DEMO_BINDINGS_H_

#include "hardware_preset/current_sensor/tmcs1133_b2a.h"

#ifdef __cplusplus
extern "C"
{
#endif

// User project prefix code
/* Add demo project includes here if required. */

// Project metadata
#define SDPE_PROJECT_ID "sdpe_demo_current_sensor_project"
#define SDPE_PROJECT_SUITE "sdpe_demo"
#define SDPE_PROJECT_VERSION "0.1.0"

// Selection macros
// Enable current sensor demo bindings.
#define SDPE_DEMO_ENABLE_CURRENT_SENSOR

// Option macros
// Demo build level selection.
// Options: (1), (2), (3)
#define SDPE_DEMO_BUILD_LEVEL (1)

// Requirement bindings
/**
 * @brief Current sensor sensitivity used by the demo ADC scaling logic.
 */
#define CTRL_DEMO_CURRENT_SENSITIVITY TMCS1133_B2A_SENSITIVITY_V_PER_A

/**
 * @brief Current sensor bias voltage.
 */
#define CTRL_DEMO_CURRENT_BIAS TMCS1133_B2A_BIAS_V

/**
 * @brief Current sensor measurable current range.
 */
#define CTRL_DEMO_CURRENT_RANGE_A TMCS1133_B2A_RANGE_A

/**
 * @brief Manual constant example. This value is not owned by a hardware entity.
 */
#define CTRL_DEMO_ADC_REF_V (3.3f)

// User project tail code
/* Add demo project tail macros here if required. */

#ifdef __cplusplus
}
#endif

#endif // _PROJECT_SDPE_DEMO_BINDINGS_H_
