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

/**
 * @defgroup CTL_INTRINSIC_ADVANCE Advance Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Advance, reusable building blocks for control template library.
 */

/**
 * @defgroup CTL_INTRINSIC_PROTECTION Protection Components
 * @ingroup CTL_INTRINSIC_LIB
 * @brief Fundamental, reusable building protection blocks for control template library.
 */

//
// ----------------- Basic Components -----------------
//

/**
 * @defgroup frequency_divider Frequency Divider
 * @ingroup CTL_INTRINSIC_BASIC
 * @brief A simple counter-based module to divide a clock or event frequency.
 */
#include <ctl/component/intrinsic/basic/divider.h>

/**
 * @defgroup saturation_blocks Saturation (Limiter) Blocks
 * @ingroup CTL_INTRINSIC_BASIC
 * @brief A collection of signal limiting modules.
 */
#include <ctl/component/intrinsic/basic/saturation.h>

/**
 * @defgroup slope_limiter Slope Limiter
 * @ingroup CTL_INTRINSIC_BASIC
 * @brief A module to constrain the rate of change of a signal.
 */
#include <ctl/component/intrinsic/basic/slope_limiter.h>

/**
 * @defgroup hysteresis_controller Hysteresis Controller
 * @ingroup CTL_INTRINSIC_BASIC
 * @brief A nonlinear controller that switches output based on a hysteresis band.
 */
#include <ctl/component/intrinsic/basic/hysteresis_controller.h>

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
 * @defgroup df_controllers Direct Form Controllers
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A library of standard direct form IIR filter implementations.
 */
#include <ctl/component/intrinsic/discrete/direct_form.h>

/**
 * @defgroup BIQUAD_filter_api Discrete Filter Library
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A collection of common discrete filters for signal processing.
 */
#include <ctl/component/intrinsic/discrete/biquad_filter.h>

/**
 * @defgroup fir_filter FIR Filter
 * @ingroup CTL_INTRINSIC_DISCRETE
 * @brief A generic FIR filter for custom coefficients.
 */
#include <ctl/component/intrinsic/discrete/fir_filter.h>

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

//
// ----------------- Continuous Components -----------------
//

/**
 * @defgroup continuous_pid_controllers Continuous-Form PID Controllers
 * @ingroup CTL_INTRINSIC_CONTINUOUS
 * @brief A library of discrete PID controllers based on the continuous-time formula.
 */
#include <ctl/component/intrinsic/continuous/continuous_pid.h>

/**
 * @defgroup tracking_continuous_pid Tracking Continuous PID Controller
 * @ingroup CTL_INTRINSIC_CONTINUOUS
 * @brief A composite PID controller for smooth setpoint tracking using a continuous-form PID.
 */
#include <ctl/component/intrinsic/continuous/track_pid.h>

/**
 * @defgroup continuous_sogi Continuous-Form SOGI
 * @ingroup CTL_INTRINSIC_CONTINUOUS
 * @brief A SOGI implementation based on discretized continuous-time state equations.
 */
#include <ctl/component/intrinsic/continuous/sogi.h>

/**
 * @defgroup s_transfer_function Generic S-Domain Transfer Function
 * @ingroup CTL_INTRINSIC_CONTINUOUS
 * @brief An IIR filter module designed from S-domain pole-zero locations.
 */
#include <ctl/component/intrinsic/continuous/s_function.h>

//
// ----------------- Advance Components -----------------
//

/**
 * @defgroup lookup_tables Look-Up Tables (LUT)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A library for 1D and 2D data searching and interpolation.
 */
#include <ctl/component/intrinsic/advance/surf_search.h>

/**
 * @defgroup fuzzy_pid_controller Fuzzy PID Controller
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A self-tuning PID controller using fuzzy logic look-up tables.
 */
#include <ctl/component/intrinsic/advance/fuzzy_pid.h>

/**
 * @defgroup repetitive_controller Repetitive Controller (RC)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief An internal model-based controller for eliminating periodic errors.
 */
#include <ctl/component/intrinsic/advance/repetitive_controller.h>

/**
 * @defgroup BACKSTEPPING_CONTROLLER Backstepping Controller
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A nonlinear controller based on systematic Lyapunov design.
 */
#include <ctl/component/intrinsic/advance/back_stepping.h>

/**
 * @defgroup ILC_CONTROLLER Iterative Learning Controller (ILC)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A controller that improves performance on repetitive tasks by learning from past errors.
 */
#include <ctl/component/intrinsic/advance/ilc.h>

/**
 * @defgroup IMC_CONTROLLER Internal Model Controller (IMC)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A robust model-based controller for SISO systems.
 */
#include <ctl/component/intrinsic/advance/imc.h>

/**
 * @defgroup lms_adaptive_filter LMS Adaptive Filter
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A self-tuning filter that minimizes the mean square error.
 */
#include <ctl/component/intrinsic/advance/lms_filter.h>

/**
 * @defgroup ADAPTIVE_CONTROLLER Model Reference Adaptive Controller (MRAC)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief An adaptive controller for SISO systems with unknown or varying parameters.
 */
#include <ctl/component/intrinsic/advance/mrac.h>

/**
 * @defgroup sinc_interpolator Sinc Interpolator
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A high-quality resampling and fractional delay module.
 */
#include <ctl/component/intrinsic/advance/sinc_interpolator.h>

/**
 * @defgroup sliding_mode_controller Sliding Mode Controller (SMC)
 * @ingroup CTL_INTRINSIC_ADVANCE
 * @brief A nonlinear robust controller based on a sliding surface.
 */
#include <ctl/component/intrinsic/advance/smc.h>

//
// ----------------- Protection Components -----------------
//

/**
 * @defgroup protection_monitor Protection Monitor
 * @ingroup CTL_INTRINSIC_PROTECTION
 * @brief A module for checking multiple variables against their boundaries.
 */
#include <ctl/component/intrinsic/protection/protection.h>

/**
 * @defgroup voltage_event_detector Voltage Sag/Swell Detector
 * @ingroup CTL_INTRINSIC_PROTECTION
 * @brief A module for detecting voltage sag and swell power quality events.
 */
#include <ctl/component/intrinsic/protection/sag_swell.h>

/**
 * @defgroup trip_protector Three-Stage Trip Protector
 * @ingroup CTL_INTRINSIC_PROTECTION
 * @brief An inverse-time overcurrent protection module.
 */
#include <ctl/component/intrinsic/protection/itoc_protection.h>
