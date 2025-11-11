/**
 * @defgroup CTL_DP_LIB Digital Power Control Library
 * @brief The whole Digital Power Control Library. The root module for the entire DP library.
 */

//
// ----------------- Second Level Groups -----------------
//

/**
 * @defgroup CTL_COMPONENTS Digital Power Components
 * @ingroup CTL_DP_LIB
 * @brief Fundamental, reusable building blocks for digital power applications.
 */

/**
 * @defgroup CTL_TOPOLOGIES Digital Power Topology Presets
 * @ingroup CTL_DP_LIB
 * @brief Pre-configured controllers for common power electronics topologies.
 */

//
// ----------------- Basic Components -----------------
//

/**
 * @defgroup buck_controller_api Buck Controller API
 * @ingroup CTL_TOPOLOGIES
 * @brief Functions and data structures for Buck converter control.
 */
#include <ctl/component/digital_power/buck.h>

/**
 * @defgroup boost_controller_api Boost Controller API
 * @ingroup CTL_TOPOLOGIES
 * @brief Functions and data structures for Boost converter control.
 */
#include <ctl/component/digital_power/boost.h>

/**
 * @defgroup CTL_BUCKBOOST_API 4-Switch Buck-Boost API
 * @ingroup CTL_TOPOLOGIES
 * @brief Provides a duty cycle calculation strategy for a 4-switch Buck-Boost converter,
 * covering four distinct operating regions for smooth transitions.
 */
#include <ctl/component/digital_power/buckboost.h>

/**
 * @defgroup protection_strategy_api Protection Strategy API
 * @ingroup CTL_COMPONENTS
 * @brief Functions and data structures for standard VIP (Voltage, Current, Power) protection.
 */
#include <ctl/component/digital_power/basic/protectoion_strategy.h>

/**
 * @defgroup sil_interface_api Software-in-the-Loop (SIL) Interface API
 * @ingroup CTL_COMPONENTS
 * @brief Data structures for communication with a simulation environment.
 */
#include <ctl/component/digital_power/basic/std_sil_dp_interface.h>

//
// ----------------- Hardware preset -----------------
//

//
// ----------------- MPPT Algorithm -----------------
//

/**
 * @defgroup mppt_api MPPT(Maximum Power Point Tracking) API
 * @ingroup CTL_COMPONENTS
 * @brief Maximum Power Point Tracking algorithm.
 */
#include <ctl/component/digital_power/mppt/INC_algorithm.h>
#include <ctl/component/digital_power/mppt/PnO_algorithm.h>

//
// ----------------- Single Phase inverter -----------------
//

/**
 * @defgroup CTL_TOPOLOGY_SINV_H_API Single-Phase Inverter Topology API
 * @ingroup CTL_TOPOLOGIES
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive single-phase inverter, including harmonic compensation and multiple
 * operating modes.
 */
#include <ctl/component/digital_power/single_phase/single_phase_dc_ac.h>

/**
 * @defgroup spfc_api Single Phase PFC API
 * @ingroup CTL_TOPOLOGIES
 * @brief Controller for single-phase Power Factor Correction.
 */
#include <ctl/component/digital_power/single_phase/spfc.h>

/**
 * @defgroup spll_api Single Phase PLL API
 * @ingroup CTL_COMPONENTS
 * @brief A Phase-Locked Loop for single-phase grid synchronization.
 */
#include <ctl/component/digital_power/single_phase/spll.h>

/**
 * @defgroup sp_modulation_api Single-Phase Modulation API
 * @ingroup CTL_COMPONENTS
 * @brief Generates PWM signals for a single-phase H-bridge inverter.
 */
#include <ctl/component/digital_power/single_phase/sp_modulation.h>

//
// ----------------- Three Phase inverter -----------------
//

/**
 * @defgroup CTL_TOPOLOGY_INV_H_API Three-Phase Inverter Topology API
 * @ingroup CTL_TOPOLOGIES
 * @brief Defines the data structures, control flags, and function interfaces for a
 * comprehensive three-phase inverter, including harmonic compensation, droop control,
 * and multiple operating modes.
 */
#include <ctl/component/digital_power/three_phase/three_phase_dc_ac.h>

/**
 * @defgroup CTL_VIENNA_PFC_API Vienna Rectifier PFC API
 * @ingroup CTL_TOPOLOGIES
 * @brief Control functions and data structures for a three-phase Vienna PFC.
 */
#include <ctl/component/digital_power/three_phase/Vienna.h>

/**
 * @defgroup CTL_PLL_API Three Phase Phase-Locked Loop (PLL) API
 * @ingroup CTL_COMPONENTS
 * @brief A standard three-phase SRF-PLL for grid synchronization.
 */
#include <ctl/component/digital_power/three_phase/pll.h>

/**
 * @defgroup CTL_TP_MODULATION_API Three-Phase Modulation API
 * @ingroup CTL_COMPONENTS
 * @brief Provides functions for generating three-phase PWM signals from voltage commands,
 * including dead-time compensation based on current direction.
 */
#include <ctl/component/digital_power/three_phase/pll.h>
