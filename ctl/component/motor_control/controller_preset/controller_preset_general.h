/**
 * @file controller_preset_general.h
 * @brief controller preset general configurations.
 */

#ifndef _FILE_CONTROLLER_PRESET_GENERAL_H_
#define _FILE_CONTROLLER_PRESET_GENERAL_H_

/**
 * @defgroup SENSOR_ENUMS Sensor Configuration Enumerations
 * @brief These enumerations are defined for documentation purposes only and are not compiled.
 * Use their integer values for configuration.
 * @{
 */

/**
     * @defgroup SENSOR_CAPABILITY General Sensing Capability
     * @ingroup SENSOR_ENUMS
     * @{
     */
enum
{
    SENSOR_NONE = 0, ///< Indicates that this sensing capability is not available.
};
/** @} */

/**
     * @defgroup SENSOR_TYPES Current Sensing Types
     * @ingroup SENSOR_ENUMS
     * @{
     */
enum
{
    SENSOR_TYPE_SHUNT = 1,  ///< Shunt resistor-based sensing (requires resistance, gain, bias).
    SENSOR_TYPE_HALL = 2,   ///< Hall-effect sensor-based sensing (requires sensitivity, zero-offset).
    SENSOR_TYPE_DIRECT = 3, ///< Other sensors with direct voltage output (same as Hall).
};
/** @} */

/**
     * @defgroup CS_TOPOLOGIES Current Sensing Topologies
     * @ingroup SENSOR_ENUMS
     * @{
     */
enum
{
    CS_TOPOLOGY_LOW_SIDE = 1,  ///< Low-side sensing on inverter legs.
    CS_TOPOLOGY_HIGH_SIDE = 2, ///< High-side sensing on inverter legs.
    CS_TOPOLOGY_INLINE = 3,    ///< In-phase (inline) sensing.
};
/** @} */

/**
     * @defgroup VS_TYPES Voltage Sensing Types
     * @ingroup SENSOR_ENUMS
     * @{
     */
enum
{
    VS_TYPE_PHASE_GND = 1, ///< "Phase voltage" relative to inverter ground (GND).
    VS_TYPE_LINE_LINE = 2, ///< Line-to-line voltage (e.g., V_ab, V_bc).
};
/** @} */

/**
     * @defgroup THERMAL_TYPES Thermal Sensing Types
     * @ingroup SENSOR_ENUMS
     * @{
     */
enum
{
    THERMAL_SENSOR_NTC = 1, ///< NTC thermistor (requires B-value, nominal resistance, etc.).
    THERMAL_SENSOR_PTC = 2, ///< PTC thermistor.
    THERMAL_SENSOR_IC = 3,  ///< IC sensor with linear voltage output (requires sensitivity and offset).
};
/** @} */

/** @} */

#endif // _FILE_CONTROLLER_PRESET_GENERAL_H_
