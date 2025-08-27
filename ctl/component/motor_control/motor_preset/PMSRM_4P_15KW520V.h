

#include <ctl/component/motor_control/basic/motor_unit_calculator.h>

#ifndef _FILE_PMSRM_4P_15KW520V_H_
#define _FILE_PMSRM_4P_15KW520V_H_

//
// Information Source: Lin Minyi (lammanyee@zju.edu.cn)
//

//
// Standard Encoder Interface
// RS485 absolute Encoder, Ruiying
//


// PMSM Motor Type
#define MOTOR_TYPE PMSM_MOTOR

//////////////////////////////////////////////////////////////////////////
// Basic parameters
//
// Number of pole pairs
#define MOTOR_PARAM_POLE_PAIRS ((2))

// Stator Resistance, unit Ohm
#define MOTOR_PARAM_RS ((0.1215))

// Stator Inductance, unit H
#define MOTOR_PARAM_LD ((3.03e-3))
#define MOTOR_PARAM_LQ ((20.19e-3))

// Inertia, unit kg*m^2
#define MOTOR_PARAM_INERTIA ((0.018))

//// Friction unit nNms
//#define MOTOR_PARAM_FRICTION ((755.0))

// constant velocity of a motor, unit rpm/V
#define MOTOR_PARAM_KV ((MOTOR_PARAM_CALCULATE_KV_BY_FLUX(MOTOR_PARAM_FLUX, MOTOR_PARAM_POLE_PAIRS)))

// Flux linkage, unit Wb
#define MOTOR_PARAM_FLUX ((0.44))

// Back EMF Voltage, unit V/krpm
#define MOTOR_PARAM_EMF ((MOTOR_PARAM_CALCULATE_EMF_BY_FLUX(MOTOR_PARAM_FLUX, MOTOR_PARAM_POLE_PAIRS)))

//////////////////////////////////////////////////////////////////////////
// Rated parameters

// Rated voltage, unit V
#define MOTOR_PARAM_RATED_VOLTAGE ((500))

// Rated Current, unit A
#define MOTOR_PARAM_RATED_CURRENT ((32.09))

// no load current, unit A
#define MOTOR_PARAM_NO_LOAD_CURRENT ((0.01))

// Rated Frequency, unit Hz
#define MOTOR_PARAM_RATED_FREQUENCY ((100))

//////////////////////////////////////////////////////////////////////////
// Maximum parameters

// Maximum Torque, unit Nm
#define MOTOR_PARAM_MAX_TORQUE ((47.8))

// Maximum Speed, unit rpm
#define MOTOR_PARAM_MAX_SPEED ((3000))


// Maximum DC bus voltage, unit V
#define MOTOR_PARAM_MAX_DC_VOLTAGE ((537))  // 3-Phase

// Maximum Phase Current RMS, unit A
#define MOTOR_PARAM_MAX_PH_CURRENT ((20))
//#define MOTOR_PARAM_MAX_PH_CURRENT ((1.5 * 32.09))

#endif // _FILE_PMSRM_4P_15KW520V_H_
