/**
 * @defgroup CTL_MC_LIB Motor Control Library
 * @brief The whole Motor Control Library. The root module for the entire MC library.
 */

//
// ----------------- Second Level Groups -----------------
//

/**
 * @defgroup CTL_MC_COMPONENT Motor Controller Components
 * @ingroup CTL_MC_LIB
 * @brief Fundamental, reusable building blocks for motor control template library.
 */

/**
 * @defgroup CTL_MC_OBSERVER Motor Controller Observer & Estimate
 * @ingroup CTL_MC_LIB
 * @brief Fundamental, reusable building blocks for motor control template library.
 */

/**
 * @defgroup CTL_MC_SUITE Motor Controller Suite
 * @ingroup CTL_MC_LIB
 * @brief Pre-configured controllers for motor control.
 */

/**
 * @defgroup CTL_MC_PRESET Motor - Motor Controller Preset
 * @ingroup CTL_MC_LIB
 * @brief Motor & Motor controller hardware preset.
 */

//
// ----------------- Basic Components -----------------
//

/**
 * @defgroup MC_DECOUPLE_PMSM PMSM Decoupling Control
 * @ingroup CTL_MC_COMPONENT
 * @brief Decoupling voltage calculation for Permanent Magnet Synchronous Motors.
 */
#include <ctl/component/motor_control/basic/decouple.h>

/**
 * @defgroup MC_ENCODER Encoder Interface
 * @ingroup CTL_MC_COMPONENT
 * @brief This module contains interfaces for various position and speed encoders.
 */
#include <ctl/component/motor_control/basic/encoder.h>

/**
 * @defgroup EncoderCalibrate Absolute Position Encoder Calibration
 * @ingroup CTL_MC_COMPONENT
 * @brief A module to automatically calibrate the electrical offset of a position encoder.
 */
#include <ctl/component/motor_control/basic/encoder_calibrate.h>

/**
 * @defgroup MC_INTERFACE Motor Control Interface
 * @ingroup CTL_MC_COMPONENT
 * @brief Standardized interfaces for motor sensors and controllers.
 */
#include <ctl/component/motor_control/basic/motor_universal_interface.h>
#include <ctl/component/motor_control/basic/std_sil_motor_interface.h>

/**
 * @defgroup MC_SVPWM Space Vector Pulse Width Modulation (SVPWM)
 * @ingroup CTL_MC_COMPONENT
 * @brief This module contains the structures and functions for SVPWM calculations.
 */
#include <ctl/component/motor_control/basic/svpwm.h>

/**
 * @defgroup ConstFGen Constant Frequency Generator
 * @ingroup CTL_MC_COMPONENT
 * @brief A module to generate a constant frequency signal output.
 */
#include <ctl/component/motor_control/basic/vf_generator.h>

/**
 * @defgroup MC_VOLTAGE_CALCULATOR Voltage Calculator
 * @ingroup CTL_MC_COMPONENT
 * @brief Calculates AC phase voltages from DC bus voltage and SVPWM signals.
 */
#include <ctl/component/motor_control/basic/voltage_calculator.h>

//
// ----------------- Consultant for Motor driver -----------------
//

/**
 * @defgroup MC_IM_CONSULTANT Induction Motor Design Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief A structure to hold all design parameters for an induction motor.
 */
#include <ctl/component/motor_control/consultant/acm_consultant.h>

/**
 * @defgroup MC_DRIVER_CONSULTANT Motor Driver Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief Defines parameters and structures related to the motor driver hardware and software configuration.
 */
#include <ctl/component/motor_control/consultant/motor_driver_consultant.h>

/**
 * @defgroup MOTOR_PER_UNIT_SYSTEM Motor Per-Unit System Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief A module to establish base quantities for per-unit calculations in motor control.
 */
#include <ctl/component/motor_control/consultant/motor_per_unit_consultant.h>

/**
 * @defgroup MC_DEFINES Motor Control Definitions
 * @ingroup CTL_MC_COMPONENT
 * @brief A collection of common definitions, constants, and macros for motor control.
 */
#include <ctl/component/motor_control/consultant/motor_unit_calculator.h>

/**
 * @defgroup MC_PMSM_NAMEPLATE PMSM Nameplate Consultant
 * @ingroup CTL_MC_COMPONENT
 * @brief Holds the rated (nameplate) parameters of a PMSM.
 */
#include <ctl/component/motor_control/consultant/pmsm_consultant.h>

//
// ----------------- Current Controller -----------------
//

/**
 * @defgroup CURRENT_DISTRIBUTOR LUT Current Distributor
 * @ingroup CTL_MC_COMPONENT
 * @brief Distributes a total current command into Id and Iq components via a LUT.
 */
#include <ctl/component/motor_control/current_loop/current_distributor.h>

/**
 * @defgroup DTC_CONTROLLER Direct Torque Controller (DTC)
 * @ingroup CTL_MC_COMPONENT
 * @brief A high-performance controller based on direct flux and torque regulation.
 */
#include <ctl/component/motor_control/current_loop/dtc.h>

/**
 * @defgroup LADRC_CURRENT_PU LADRC Current Controller (Per-Unit)
 * @ingroup CTL_MC_COMPONENT
 * @brief A robust, per-unit current controller using Active Disturbance Rejection.
 */
#include <ctl/component/motor_control/current_loop/ladrc_current_controller.h>

/**
 * @defgroup CURRENT_CONTROLLER Dual PI FOC Current Controller
 * @ingroup CTL_MC_COMPONENT
 * @brief The core current regulation loop for an FOC motor controller.
 */
#include <ctl/component/motor_control/current_loop/motor_current_ctrl.h>

/**
 * @defgroup MTPA_DISTRIBUTOR MTPA Current Distributor
 * @ingroup CTL_MC_COMPONENT
 * @brief Calculates optimal Id and Iq references for salient-pole PMSMs.
 */
#include <ctl/component/motor_control/current_loop/mtpa.h>

/**
 * @defgroup MTPA_DISTRIBUTOR_PU MTPA Current Distributor (Per Unit)
 * @ingroup CTL_MC_COMPONENT
 * @brief Calculates optimal Id and Iq references for salient-pole PMSMs.
 */
#include <ctl/component/motor_control/current_loop/mtpa_pu.h>

/**
 * @defgroup MTPV_CONTROLLER MTPV / Field Weakening Controller
 * @ingroup CTL_MC_COMPONENT
 * @brief Calculates optimal Id and Iq references for high-speed operation.
 */
#include <ctl/component/motor_control/current_loop/mtpv.h>

/**
 * @defgroup MTPV_CONTROLLER_PU MTPV / Field Weakening Controller (Per-Unit)
 * @ingroup CTL_MC_COMPONENT
 * @brief Calculates optimal Id and Iq references for high-speed operation using per-unit values.
 */
#include <ctl/component/motor_control/current_loop/mtpv_pu.h>

/**
 * @defgroup DPCC_CONTROLLER Dead-beat Predictive Current Controller
 * @ingroup CTL_MC_COMPONENT
 * @brief A model-based current controller for fast dynamic response.
 */
#include <ctl/component/motor_control/current_loop/PMSM_DPCC.h>

/**
 * @defgroup MPC_CONTROLLER FCS-MPC Current Controller
 * @ingroup CTL_MC_COMPONENT
 * @brief A model-predictive current controller for fast dynamic response.
 */
#include <ctl/component/motor_control/current_loop/pmsm_mpc.h>

/**
 * @defgroup MPC_CONTROLLER_ACM FCS-MPC Current & Flux Controller for ACM
 * @ingroup CTL_MC_COMPONENT
 * @brief A model-predictive controller for fast torque and flux response in ACMs.
 */
#include <ctl/component/motor_control/current_loop/acm_mpc.h>

//
// ----------------- Motion Controller -----------------
//

/**
 * @defgroup POSITION_CONTROLLER Basic Position Controller
 * @ingroup CTL_MC_COMPONENT
 * @brief A simple P-controller for a position loop.
 */
#include <ctl/component/motor_control/motion/basic_pos_loop_p.h>

/**
 * @defgroup KNOB_POSITION_LOOP Haptic Knob Position Loop
 * @ingroup CTL_MC_COMPONENT
 * @brief A position controller that simulates discrete detents.
 */
#include <ctl/component/motor_control/motion/knob_pos_loop.h>

/**
 * @defgroup LADRC_SPEED_PU LADRC Speed Controller (Per-Unit)
 * @ingroup CTL_MC_COMPONENT
 * @brief A robust, per-unit speed controller using Active Disturbance Rejection.
 */
#include <ctl/component/motor_control/motion/ladrc_spd_ctrl.h>

/**
 * @defgroup nladrc_controller Nonlinear ADRC
 * @ingroup CTL_MC_COMPONENT
 * @brief An advanced controller for systems with uncertainties and disturbances.
 */
#include <ctl/component/motor_control/motion/nladrc_spd_ctrl.h>

/**
 * @defgroup SINUSOIDAL_PLANNER Sinusoidal Trajectory Planner
 * @ingroup CTL_MC_COMPONENT
 * @brief Generates cycloidal profiles for extremely smooth motion.
 */
#include <ctl/component/motor_control/motion/sinusoidal_trajectory.h>

/**
 * @defgroup S_CURVE_PLANNER S-Curve Trajectory Planner
 * @ingroup CTL_MC_COMPONENT
 * @brief Generates smooth S-shaped velocity profiles.
 */
#include <ctl/component/motor_control/motion/s_curve_trajectory.h>

/**
 * @defgroup TRAPEZOIDAL_PLANNER Trapezoidal Trajectory Planner
 * @ingroup CTL_MC_COMPONENT
 * @brief Generates trapezoidal velocity profiles for position control.
 */
#include <ctl/component/motor_control/motion/trapezoidal_trajectory.h>

//
// ----------------- Motor Observer -----------------
//

/**
 * @defgroup ACM_POS_CALC ACM Position Calculator
 * @ingroup CTL_MC_OBSERVER
 * @brief A module for estimating rotor flux angle in an AC Induction Motor.
 * @details Implements the core calculations for indirect field-oriented control (IFOC).
 */
#include <ctl/component/motor_control/observer/acm.pos_calc.h>

/**
 * @defgroup IM_FLUX_OBSERVER Induction Motor Flux and Torque Observer
 * @ingroup CTL_MC_OBSERVER
 * @brief A module for estimating IM rotor flux and electromagnetic torque.
 */
#include <ctl/component/motor_control/observer/acm.fo.h>

/**
 * @defgroup ACM_SMO ACM Sliding Mode Observer
 * @ingroup CTL_MC_OBSERVER
 * @brief A sensorless module for estimating speed and flux angle of an ACM.
 */
#include <ctl/component/motor_control/observer/acm.smo.h>

/**
 * @defgroup BLDC_HALL_ESTIMATOR BLDC Hall Sensor Estimator
 * @ingroup CTL_MC_OBSERVER
 * @brief A module for calculating smooth speed and position from Hall sensors.
 */
#include <ctl/component/motor_control/observer/bldc.hall.h>

/**
 * @addtogroup PMSM_FLUX_OBSERVER PMSM Flux Observer
 * @ingroup CTL_MC_OBSERVER
 * @brief PMSM Observer of Flux.
 */
#include <ctl/component/motor_control/observer/pmsm.fo.h>

/**
 * @defgroup PMSM_HFI PMSM HFI Estimator
 * @ingroup CTL_MC_OBSERVER
 * @brief A module for sensorless position and speed estimation using HFI.
 * @details Implements the signal injection, demodulation, filtering, and PLL
 * required to track the rotor angle of a salient PMSM at low speeds.
 */
#include <ctl/component/motor_control/observer/pmsm.hfi.h>

/**
 * @defgroup PMSM_SMO PMSM SMO Estimator
 * @ingroup CTL_MC_OBSERVER
 * @brief A module for sensorless position and speed estimation using SMO.
 * @details Implements the observer model, sliding control law, filtering, and PLL
 * required to track the rotor angle of a PMSM.
 */
#include <ctl/component/motor_control/observer/pmsm.smo.h>

//
// ----------------- Motor Estimate -----------------
//

/**
 * @defgroup ONLINE_RS_ESTIMATOR Online Rs Estimator (MRAS)
 * @ingroup CTL_MC_OBSERVER
 * @brief An online estimator for stator resistance based on the Model Reference Adaptive System principle.
 * @details This module continuously updates the estimated stator resistance (Rs) to track changes
 * due to temperature variations during motor operation. It uses the d-axis voltage equation
 * as the reference model and a PI controller as the adaptation mechanism.
 */
#include <ctl/component/motor_control/param_est/pmsm.online_est_Rs.h>

/**
 * @defgroup OFFLINE_ESTIMATION Offline Parameter Estimation
 * @ingroup CTL_MC_OBSERVER
 * @brief A module to automatically identify key PMSM parameters.
 */
#include <ctl/component/motor_control/param_est/pmsm_offline_est.h>


//
// ----------------- PMSM Controller Suite -----------------
//

/**
 * @defgroup MC_PMSM_SENSORED_CONTROLLER Sensored PMSM FOC Controller
 * @ingroup CTL_MC_SUITE
 * @brief A complete Field-Oriented Controller for Permanent Magnet Synchronous Motors.
 */
#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl.h>

/**
 * @defgroup PMSM_HFI_BARE_CONTROLLER PMSM HFI Controller
 * @ingroup CTL_MC_SUITE
 * @brief A module for controlling a PMSM using Field-Oriented Control.
 * @details This module contains the data structures and functions to implement a
 * full FOC controller with cascaded control loops for position, velocity,
 * and current.
 */
#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl_hfi.h>

/**
 * @defgroup PMSM_MTPA_CONTROLLER PMSM MTPA Controller
 * @ingroup CTL_MC_SUITE
 * @brief A module for controlling a PMSM using FOC with an MTPA strategy.
 */
#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl_mtpa.h>

/**
 * @defgroup PMSM_SMO_CONTROLLER PMSM SMO Sensorless Controller
 * @ingroup CTL_MC_SUITE
 * @brief A module for sensorless FOC of a PMSM using a Sliding Mode Observer.
 */
#include <ctl/component/motor_control/pmsm_controller/pmsm_ctrl_smo.h>

/**
 * @defgroup MC_ACM_SENSORED_CONTROLLER Sensored ACM FOC Controller
 * @ingroup CTL_MC_SUITE
 * @brief A complete Field-Oriented Controller for AC Induction Motors.
 */
#include <ctl/component/motor_control/acm_controller/acm_sensored_ctrl.h>

//
// ----------------- Motor / Inverter Preset parameters -----------------
//

#if 0

/**
 * @defgroup ACM_4P24V_PARAMETERS Motor Parameters (ACM 4-Pole 24V)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified AC Induction Motor.
 */
#include <ctl/component/motor_control/motor_preset/ACM_4P24V.h>

/**
 * @defgroup PMSM_GBM2804H_PARAMETERS Motor Parameters (GBM2804H-100T)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSM.
 */
#include <ctl/component/motor_control/motor_preset/GBM2804H_100T.h>

/**
 * @defgroup PMSM_HBL48ZL400330K_PARAMETERS Motor Parameters (HBL48ZL400330K)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSM.
 */
#include <ctl/component/motor_control/motor_preset/HBL48ZL400330K.h>

/**
 * @defgroup PMSRM_4P_15KW_PARAMETERS Motor Parameters (PMSRM 4-Pole 15kW)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSRM.
 */
#include <ctl/component/motor_control/motor_preset/PMSRM_4P_15KW520V.h>


/**
 * @defgroup PMSM_TYI5008_PARAMETERS Motor Parameters (TYI-5008 KV335)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSM.
 */
#include <ctl/component/motor_control/motor_preset/TYI_5008_KV335.h>

/**
 * @defgroup PMSM_TYI5010_PARAMETERS Motor Parameters (TYI-5010 KV360)
 * @ingroup CTL_MC_PRESET
 * @brief Contains all parameter definitions for the specified PMSM.
 */
#include <ctl/component/motor_control/motor_preset/TYI_5010_360KV.h>

/**
 * @defgroup CURRENT_DISTRIBUTOR_LUT Current Distributor LUT
 * @ingroup CTL_MC_PRESET
 * @brief Provides a lookup table for mapping current magnitude to a control angle.
 */
#include <ctl/component/motor_control/controller_preset/CURRENT_DISTRIBUTOR_LUT.h>

/**
 * @defgroup HW_BOARD_PARAMETERS Inverter Hardware Parameters
 * @ingroup CTL_MC_PRESET
 * @brief Contains definitions specific to the GMP LV 3-Phase GaN Inverter hardware.
 */
#include <ctl/component/motor_control/controller_preset/GMP_LV_3PH_GAN_INV.h>

/**
 * @defgroup HW_BOARD_PARAMETERS_2136SINV 2136SINV Inverter Hardware Parameters
 * @ingroup CTL_MC_PRESET
 * @brief Contains definitions specific to the GMP 3PH 2136SINV DUAL hardware.
 */
#include <ctl/component/motor_control/controller_preset/GMP_3PH_2136SINV_DUAL.h>

/**
 * @defgroup HW_BOARD_PARAMETERS_SE_PWR SE Power Board Hardware Parameters
 * @ingroup CTL_MC_PRESET
 * @brief Contains definitions specific to the SE Power Board hardware.
 */
#include <ctl/component/motor_control/controller_preset/SE_PWR_BD.h>

/**
 * @defgroup HW_BOARD_PARAMETERS_TI_GAN TI GaN Inverter Hardware Parameters
 * @ingroup CTL_MC_PRESET
 * @brief Contains definitions specific to the TI 3-Phase GaN Inverter hardware.
 */
#include <ctl/component/motor_control/controller_preset/TI_3PH_GAN_INV.h>

#endif
