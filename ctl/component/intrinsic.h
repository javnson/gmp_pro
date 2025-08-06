/**
 * @defgroup CTL_INTRINSIC_LIB  Control Template Library Intrinsic
 * @brief The Basic modules in Control Template Library
 */

//
// ----------------- Second Level Groups -----------------
//

/**
 * @defgroup CTL_INTRINSIC_BASIC Basic Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Fundamental, reusable building blocks for control template library.
 */

/**
 * @defgroup CTL_INTRINSIC_CONTINUOUS Continuous Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Fundamental, reusable building continuous blocks for control template library.
 */

/**
 * @defgroup CTL_INTRINSIC_DISCRETE Discrete Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Fundamental, reusable building discrete blocks for control template library.
 */

/**
 * @defgroup CTL_INTRINSIC_LEBESGUE Lebesgue Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Fundamental, reusable building Lebesgue blocks for control template library.
 */

//
// ----------------- Discrete Components -----------------
//

/**
 * @defgroup discrete_filter_api Discrete Filter Library
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A collection of common discrete filters for signal processing.
 */
#include <ctl/component/intrinsic/discrete/discrete_filter.h>

/**
 * @defgroup discrete_pid_controller Discrete PID Controller
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A collection of common discrete filters for signal processing.
 */
#include <ctl/component/intrinsic/discrete/discrete_pid.h>

/**
 * @defgroup discrete_sogi SOGI-based Quadrature Signal Generator
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief Implements a SOGI to generate in-phase and quadrature-phase signals.
 */
#include <ctl/component/intrinsic/discrete/discrete_sogi.h>

/**
 * @defgroup lead_lag_compensators Lead-Lag Compensators
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A library of discrete IIR filters for control loop compensation.
 */
#include <ctl/component/intrinsic/discrete/lead_lag.h>

/**
 * @defgroup pole_zero_compensators Pole-Zero Compensators
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A library of discrete IIR filters for control loop compensation.
 */
#include <ctl/component/intrinsic/discrete/pole_zero.h>

/**
 * @defgroup resonant_controllers Resonant Controllers
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A library of discrete resonant controllers for AC signal tracking.
 */
#include <ctl/component/intrinsic/discrete/proportional_resonant.h>

/**
 * @defgroup signal_generators Signal Generators
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A library of modules for generating standard test waveforms.
 */
#include <ctl/component/intrinsic/discrete/signal_generator.h>

/**
 * @defgroup tracking_pid Tracking PID Controller
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A composite PID controller for smooth setpoint tracking.
 */
#include <ctl/component/intrinsic/discrete/track_discrete_pid.h>

/**
 * @defgroup z_transfer_function Generic Z-Domain Transfer Function
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief An IIR filter module to implement any given Z-domain transfer function.
 */
#include <ctl/component/intrinsic/discrete/z_function.h>




// 把所有intrinsic的header全部引入进来

// TODO: 为每一个头文件(模块)补充一个简单的说明，方便用户可以快速在这个文件中找到需要的模块

// basic includes

// #include <ctl/ctl_core.h>

#ifndef GMP_CTL_DISABLE_COM_INTRINSIC

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_INTERFACE

#include <ctl/component/intrinsic/interface/adc_channel.h>

#include <ctl/component/intrinsic/interface/dac_channel.h>

#include <ctl/component/intrinsic/interface/pwm_channel.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_INTERFACE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_DISCRETE

#include <ctl/component/intrinsic/discrete/divider.h>

#include <ctl/component/intrinsic/discrete/feed_forward.h>

#include <ctl/component/intrinsic/discrete/discrete_filter.h>

#include <ctl/component/intrinsic/discrete/hcc.h>

#include <ctl/component/intrinsic/discrete/pid.h>

#include <ctl/component/intrinsic/discrete/pll.h>

#include <ctl/component/intrinsic/discrete/pole_zero.h>

#include <ctl/component/intrinsic/discrete/saturation.h>

#include <ctl/component/intrinsic/discrete/slope_lim.h>

#include <ctl/component/intrinsic/discrete/stimulate.h>

// #include <ctl/component/intrinsic/discrete/z_function.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_DISCRETE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_PROTECT

#include <ctl/component/intrinsic/protection/fusing.h>

#include <ctl/component/intrinsic/protection/protection.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_PROTECT

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_ADVANCE

#include <ctl/component/intrinsic/advance/fuzzy_pid.h>

#include <ctl/component/intrinsic/advance/surf_search.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_ADVANCE

//////////////////////////////////////////////////////////////////////////
#ifndef GMP_CTL_DISABLE_COM_INTRINSIC_COMBO

#include <ctl/component/intrinsic/combo/track_pid.h>

#endif // GMP_CTL_DISABLE_COM_INTRINSIC_COMBO

#endif // GMP_CTL_DISABLE_COM_INTRINSIC

