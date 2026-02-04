// mdl_asio_helper.cpp - Byte packing, transmit via UDP (ASIO), unpacking.

// Source code for GMP_SIL_Core

// Sfunction Parameters:
// NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, NumberPackedPorts, PackedDataTypes, PackedDataSizes,
// Alignment, HostAddr, TargetPort

// Driver function:
//
//% source : MaskUnpackedDataTypes, MaskUnpackedDataSizes
//% source : MaskPackedDataTypes, MaskUnpackedDataSizes
//% source:
// MaskAlignment[NumberUnpackedPorts, UnpackedDataTypes, UnpackedDataSizes, ...
//    NumberPackedPorts, PackedDataTypes, PackedDataSizes, ...
//    Alignment] = mdl_gmp_simulink_core(...
//    MaskUnpackedDataTypes, MaskUnpackedDataSizes, ...
//    MaskPackedDataTypes, MaskPackedDataSizes, ...
//    MaskAlignment);

//% source : MaskHostAddr
//% source : MaskMsgTxPort, MaskMsgRxPort
//% source : MaskCmdTxPort,
// MaskCmdRxPort[HostAddr, TargetPort] =
//    mdl_gmp_simulink_connection(MaskHostAddr, ...
//    MaskMsgTxPort, MaskMsgRxPort, ...
//    MaskCmdTxPort, MaskCmdRxPort);

// S function basic informations.
#define S_FUNCTION_LEVEL 2

#undef S_FUNCTION_NAME
#define S_FUNCTION_NAME GMP_SIL_Core

#define DRIVER "GMP_SIL_Core"

// Headers
#include "fixedpoint.h"
#include "simstruc.h"
#include <stddef.h>
#include <stdlib.h>
#include <string>

#include <codecvt>
#include <fstream>
#include <locale>
#include <memory>
#include <stdint.h>
#include <string>
#include <vector>

#if !defined __linux__

// Need to define WIN32 because each version of windows has slightly different ways of handling networking
#include <SDKDDKVer.h>
#endif // __linux__

// ASIO library
#define ASIO_STANDALONE
#include <asio.hpp>

using udp = asio::ip::udp;

// json library
#include <nlohmann/json.hpp>

// default timeout 10s (10_000 ms)
#define GMP_ASIO_UDP_LINK_TIMEOUT (10000)
#define ASIO_UDP_HELPER_SERVER_MODE

// udp helper
//#include <tools/gmp_sil/udp_helper/asio_udp_helper.hpp>
#include <tools/gmp_sil/udp_helper_v2/asio_udp_helper.hpp>

using json = nlohmann::json;

#include "mex.h"

// Level-2 MATLAB® S-functions must implement the following callback methods:
// + setup
//     Specifies the sizes of various parameters in the SimStruct,
//   such as the number of output ports for the block.
// + Outputs
//     Calculates the output of the block.
// + Terminate
//     Performs any actions required at the termination of the simulation.
//   If no actions are required, this function can be implemented as a stub.

// Module Parameter List
enum
{
    S_NUMBER_UNPACKED_PORTS = 0, // Number of unpacked ports
    S_UNPACKED_DATA_TYPES,       // Vector of data types for unpacked data
    S_UNPACKED_DATA_SIZES,       // Vector of data sizes (2 per port) for unpacked data [unpack only]

    S_NUMBER_PACKED_PORTS, // Number of packed ports
    S_PACKED_DATA_TYPES,   // Data types of packed data [pack only]
    S_PACKED_DATA_SIZES,   // Data size of packed data

    S_ALIGNMENT,           // Byte alignment
    S_NETWORK_HOST,        // NETWORK param
    S_NETWORK_TARGET_PORT, // NETWORK port parameter
    NUM_S_PARAMS           // Number of parameters passed from the mask
};

static char_T msg[256];                         // Generic message buffer
static int_T j;                                 // Generic counter
static DTypeId typeIdint64 = INVALID_DTYPE_ID;  // registered data type id for int64
static DTypeId typeIduint64 = INVALID_DTYPE_ID; // registered data type id for uint64

// General purpose macros
#define GetVectorParam(param, index) (mxGetPr(ssGetSFcnParam(S, param))[index])
#define GetScalarParam(param)        (GetVectorParam(param, 0))
#define GetParamSize(param)          (mxGetN(ssGetSFcnParam(S, param)))

// data_type index -> length
static int_T GetDataBytes(DTypeId data_type)
{
    if (data_type == SS_DOUBLE)
        return 8;
    else if (data_type == SS_SINGLE)
        return 4;
    else if (data_type == SS_INT8)
        return 1;
    else if (data_type == SS_UINT8)
        return 1;
    else if (data_type == SS_INT16)
        return 2;
    else if (data_type == SS_UINT16)
        return 2;
    else if (data_type == SS_INT32)
        return 4;
    else if (data_type == SS_UINT32)
        return 4;
    else if (data_type == SS_BOOLEAN)
        return 1;
    else if (data_type == typeIdint64)
        return 8;
    else if (data_type == typeIduint64)
        return 8;
    else
        return -1;
}

// validate data type
static int_T VerifyDataType(DTypeId data_type)
{
    if (GetDataBytes(data_type) == -1)
        return -1;
    else
        return 0;
}

// memory alignment
static int_T CalculatePad(int_T bytes, uint_T alignment)
{
    return (bytes % alignment) ? alignment - (bytes % alignment) : 0;
}

// input port size calculate
// input port validate
static int_T CalculatePackedWidth(SimStruct* S)
{
    int_T port_width = 1;  // Width of unpacked port;
    DTypeId port_type = 0; // Data type of unpacked port;
    int_T port_bytes;      // Size of unpacked port in bytes;
    int_T total_bytes = 0;

    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS); j++)
    {
        // Get input port width and data type
        port_width = ssGetInputPortWidth(S, j);
        port_type = ssGetInputPortDataType(S, j);

        if ((port_width == DYNAMICALLY_SIZED) || (port_type == DYNAMICALLY_TYPED))
            return -1;

        port_bytes = port_width * GetDataBytes(port_type);
        total_bytes += port_bytes + CalculatePad(port_bytes, (uint_T)GetScalarParam(S_ALIGNMENT));
    }

    // return (total_bytes / GetDataBytes(port_type)) + ((total_bytes % GetDataBytes(port_type)) ? 1 : 0);
    return total_bytes;
}

// output port size calculate
// output port validate
static int_T CalculateUnpackedWidth(SimStruct* S)
{
    int_T port_width = 1;  // Width of unpacked port;
    DTypeId port_type = 0; // Data type of unpacked port;
    int_T port_bytes;      // Size of unpacked port in bytes;
    int_T total_bytes = 0;

    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS); j++)
    {
        port_width = ssGetOutputPortWidth(S, j);
        port_type = ssGetOutputPortDataType(S, j);

        if ((port_width == DYNAMICALLY_SIZED) || (port_type == DYNAMICALLY_TYPED))
            return -1;

        port_bytes = port_width * GetDataBytes(port_type);
        total_bytes += port_bytes + CalculatePad(port_bytes, (uint_T)GetScalarParam(S_ALIGNMENT));
    }

    // return (total_bytes / GetDataBytes(port_type)) + ((total_bytes % GetDataBytes(port_type)) ? 1 : 0);
    return total_bytes;
}

// Specify the number of inputs, outputs, states, parameters, and other characteristics of the C MEX S-function
// This is the first S-function callback methods that the Simulink engine calls.
// This method performs the following tasks:
// + Specify the number of parameters that this S-function supports, using @ssSetNumSFcnParams.
// + Specify the number of states that this function has, using @ssSetNumContStates and @ssSetNumDiscStates.
// + Configure the block's input ports, including:
//   - Specify the number of input ports that this S-function has, using @ssSetNumInputPorts.
//   - Specify the dimensions of the input ports, using @ssSetInputPortDimensionInfo.
//   - For each input port, specify whether it has direct feedthrough, using ssSetInputPortDirectFeedThrough.
// + Configure the block's output ports, including:
//   - Specify the number of output ports that the block has, using ssSetNumOutputPorts.
//   - Specify the dimensions of the output ports. @mdlSetOutputPortDimensionInfo
//   If your S-function outputs are discrete (for example, the outputs only take specific values such as 0, 1, and 2),
//   specify SS_OPTION_DISCRETE_VALUED_OUTPUT.
// + Set the number of sample times (i.e., sample rates) at which the block operates.
//  There are two ways of specifying sample times:
//   - Port-based sample times
//   - Block-based sample times
// + Set the size of the block's work vectors,
//   using ssSetNumRWork, ssSetNumIWork, ssSetNumPWork, ssSetNumModes, ssSetNumNonsampledZCs.
// + Set the simulation options that this block implements, using ssSetOptions.
//
static void mdlInitializeSizes(SimStruct* S)
{
    int_T packed_data_type;   // Data type id of pack port
    int_T unpacked_data_type; // Data type id of unpack port
    DTypeId data_type;        // A temporary DTypeId variable to store the data type for the corresponding type id
    int_T dimension_m;        // Size of unpack ports m dimension
    int_T dimension_n;        // Size of unpack ports n dimension
    int_T unpacked_ports;     // Number of unpack ports
    int_T packed_ports;       // Number of pack ports
    uint_T alignment;         // Byte alignment

    // Sanity check the correct number of arguments is being passed in
    // Check if param is over range
    ssSetNumSFcnParams(S, NUM_S_PARAMS);

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S))
    {
        sprintf(msg, "%s: %d input args expected, %d passed", DRIVER, NUM_S_PARAMS, ssGetSFcnParamsCount(S));
        ssSetErrorStatus(S, msg);
        return;
    }

    packed_data_type = (int_T)GetScalarParam(S_PACKED_DATA_TYPES);
    packed_ports = (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS);

    unpacked_ports = (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS);

    // GetStringParam

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    //////////////////////////////////////////////////////////////////////////
    // Packing Section
    // Set Input Port
    if (packed_ports != (int_T)GetParamSize(S_PACKED_DATA_TYPES))
    {
        sprintf(msg, "%s: Vector sizes must match %d %d", DRIVER, /*Direction[direction].c_str(),*/ unpacked_ports,
                (int_T)GetParamSize(S_PACKED_DATA_TYPES));
        ssWarning(S, msg);
    }

    ssSetNumInputPorts(S, packed_ports);

    for (j = 0; j < packed_ports; j++)
    {
        unpacked_data_type = (int_T)GetVectorParam(S_PACKED_DATA_TYPES, j);
        if (unpacked_data_type == 9)
        { // int64
            data_type = typeIdint64;
        }
        else if (unpacked_data_type == 10)
        { // uint64
            data_type = typeIduint64;
        }
        else
        {
            data_type = (DTypeId)unpacked_data_type;
        }

        if (VerifyDataType(data_type) == -1)
        {
            sprintf(msg, "%s: Invalid input port data type %d port %d", DRIVER, /* Direction[direction].c_str(),*/
                    unpacked_data_type, j);
            ssWarning(S, msg);
        }

        ssSetInputPortDataType(S, j, data_type);
        ssSetInputPortDimensionInfo(S, j, DYNAMIC_DIMENSION);
        ssSetInputPortDirectFeedThrough(S, j, 1);
        ssSetInputPortRequiredContiguous(S, j, 1);
    }

    //////////////////////////////////////////////////////////////////////////
    // Unpacking Section
    // Set output Ports
    if ((unpacked_ports != (int_T)GetParamSize(S_UNPACKED_DATA_TYPES)) ||
        ((unpacked_ports * 2) != (int_T)GetParamSize(S_UNPACKED_DATA_SIZES)))
    {
        sprintf(msg, "%s: Vector sizes must match %d %d %d", DRIVER, /*Direction[direction].c_str(),*/ unpacked_ports,
                (int_T)GetParamSize(S_UNPACKED_DATA_TYPES), (int_T)GetParamSize(S_UNPACKED_DATA_SIZES));
        ssWarning(S, msg);
    }

    ssSetNumOutputPorts(S, unpacked_ports);

    for (j = 0; j < unpacked_ports; j++)
    {
        unpacked_data_type = (int_T)GetVectorParam(S_UNPACKED_DATA_TYPES, j);
        if (unpacked_data_type == 9)
        { // int64
            data_type = typeIdint64;
        }
        else if (unpacked_data_type == 10)
        { // uint64
            data_type = typeIduint64;
        }
        else
        {
            data_type = (DTypeId)unpacked_data_type;
        }

        if (VerifyDataType(data_type) == -1)
        {
            sprintf(msg, "%s: Invalid output port data type %d port %d", DRIVER, /*Direction[direction].c_str(),*/
                    unpacked_data_type, j);
            ssWarning(S, msg);
        }

        ssSetOutputPortDataType(S, j, data_type);

        // unpack has two dimensions to determine the unpacked data
        dimension_m = (int_T)GetVectorParam(S_UNPACKED_DATA_SIZES, j * 2);
        dimension_n = (int_T)GetVectorParam(S_UNPACKED_DATA_SIZES, (j * 2) + 1);

        if ((dimension_m <= 0) || (dimension_n <= 0))
        {
            sprintf(msg, "%s: Invalid output port data dimensions %d %d port %d", DRIVER,
                    /*Direction[direction].c_str(),*/ dimension_m, dimension_n, j);
            ssWarning(S, msg);
        }
        if (dimension_m == 1)
            ssSetOutputPortVectorDimension(S, j, dimension_n);
        else
            ssSetOutputPortMatrixDimensions(S, j, dimension_m, dimension_n);
    }

    //////////////////////////////////////////////////////////////////////////
    // Alignment Validate
    alignment = (uint_T)GetScalarParam(S_ALIGNMENT);
    if ((alignment != 1) && (alignment != 2) && (alignment != 4) && (alignment != 8))
    {
        sprintf(msg, "%s: Invalid data alignment %d", DRIVER, /*Direction[direction].c_str(),*/ alignment);
        ssWarning(S, msg);
    }

    //////////////////////////////////////////////////////////////////////////
    // Network target validate

    if ((int_T)GetParamSize(S_NETWORK_HOST) != 4)
    {
        if ((alignment != 1) && (alignment != 2) && (alignment != 4) && (alignment != 8))
        {
            sprintf(msg, "%s: Invalid target host IP parameter.", DRIVER);
            ssWarning(S, msg);
        }
    }

    if ((int_T)GetParamSize(S_NETWORK_TARGET_PORT) != 4)
    {
        if ((alignment != 1) && (alignment != 2) && (alignment != 4) && (alignment != 8))
        {
            sprintf(msg, "%s: Invalid target port parameter.", DRIVER);
            ssWarning(S, msg);
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Set the number of sample times (i.e., sample rates) at which the block operates.

    // Specify the number of sample times that an S-Function block has
    ssSetNumSampleTimes(S, 1);

    //////////////////////////////////////////////////////////////////////////
    // Set the size of the block's work vectors,

    // integer work vector
    ssSetNumIWork(S, unpacked_ports * 2 + packed_ports * 2);
    // floating-point work vector
    ssSetNumRWork(S, 0);
    // Specify the size of a block's pointer work vector
    // 1 for udp helper
    // 2 for tranBuffer
    // 3 for recvBuffer
    ssSetNumPWork(S, 3);
    // Specify the size of the block's mode vector
    ssSetNumModes(S, 0);
    // Specify the number of states for which a block detects zero crossings that occur between sample points
    ssSetNumNonsampledZCs(S, 0);

    // Parameter tunable disable
    for (j = 0; j < NUM_S_PARAMS; j++)
        ssSetSFcnParamTunable(S, j, SS_PRM_NOT_TUNABLE);

    // Set this S-function as runtime thread-safe for multicore execution
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);

    ssSetOptions(
        S, SS_OPTION_DISALLOW_CONSTANT_SAMPLE_TIME     // Disallow inheritance of Inf sample time
               | SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE // Improve performance of run-time exception-free S-functions
               | SS_OPTION_WORKS_WITH_CODE_REUSE       // Specify this S-function supports code reuse
    );

    // unknown
    ssSetSimStateCompliance(S, DISALLOW_SIM_STATE);
    ssSupportsMultipleExecInstances(S, true);
}

// Specify the sample rates at which this C MEX S-function operates
// This method should specify the sample time and offset time for each sample rate
// at which this S-function operates via the following paired macros
// + ssSetSampleTime(S, sampleTimeIndex, sample_time)
// + ssSetOffsetTime(S, offsetTimeIndex, offset_time)
static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
}

// Set the dimensions of the signals accepted by an input port
// The Simulink engine calls this method during dimension propagation with candidate dimensions dimsInfo for port.
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct* S, int_T portIndex, const DimsInfo_T* dimsInfo)
{
    // Width of packed port including pad
    int_T packed_width;

    if ((dimsInfo->numDims != 1) && (dimsInfo->numDims != 2))
    {
        sprintf(msg, "%s: Invalid inherited input port dimension size %d port %d", DRIVER,
                /*Direction[direction].c_str(),*/ dimsInfo->numDims, portIndex);
        ssSetErrorStatus(S, msg);
        return;
    }

    ssSetInputPortDimensionInfo(S, portIndex, dimsInfo);

    packed_width = CalculatePackedWidth(S);
    if (packed_width == -1)
        return;

    // if (ssGetOutputPortWidth(S, 0) == DYNAMICALLY_SIZED)
    //     ssSetOutputPortVectorDimension(S, 0, packed_width);

    // if (ssGetOutputPortWidth(S, 0) != packed_width)
    //{
    //     sprintf(msg, "%s: Inherited input port width %d does not match inherited output port width %d", DRIVER,
    //             /*Direction[direction].c_str(),*/ packed_width, ssGetOutputPortWidth(S, 0));
    //     ssSetErrorStatus(S, msg);
    // }

    ////////////////////////////////////////////////////////////////////////////

    //    if ((dimsInfo->numDims != 1) && (dimsInfo->numDims != 2))
    //{
    //    sprintf(msg, "%s: Invalid inherited input port dimension size %d port %d", DRIVER,
    //            /*Direction[direction].c_str(),*/ dimsInfo->numDims, portIndex);
    //    ssSetErrorStatus(S, msg);
    //    return;
    //}

    // if (ssGetOutputPortWidth(S, portIndex) == DYNAMICALLY_SIZED)
    //     ssSetOutputPortVectorDimension(S, portIndex, dimsInfo->width);

    // if (ssGetOutputPortWidth(S, portIndex) != dimsInfo->width)
    //{
    //     sprintf(msg, "%s: Invalid inherited output port data width %d port %d (%d)", DRIVER,
    //             /*Direction[direction].c_str(),*/ dimsInfo->width, portIndex, ssGetOutputPortWidth(S, portIndex));
    //     ssSetErrorStatus(S, msg);
    //     return;
    // }

    //    break;
    // case BYTE_UNPACKING:
    // if (packed_width != ssGetInputPortWidth(S, 0))
    //{
    //    sprintf(msg, "%s(%s): Invalid inherited input port width %d (%d)", DRIVER, Direction[direction].c_str(),
    //            ssGetInputPortWidth(S, 0), packed_width);
    //    ssSetErrorStatus(S, msg);
    //    return;
    //}
    //    break;
    //}
}

#define MDL_SET_INPUT_PORT_DATA_TYPE
static void mdlSetInputPortDataType(SimStruct* S, int_T portIndex, DTypeId dType)
{
    // int_T packed_width; // Width of packed port including pad
    //  uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
    //  uint_T debug = (uint_T)GetScalarParam(S_DEBUG);

    // if (debug & DEBUG_ENTRY)
    //     printf("%s(%d) %d %d\n", DRIVER, __LINE__, portIndex, dType);

    if (VerifyDataType(dType))
    {
        sprintf(msg, "%s: Invalid inherited input port data type %d port %d", DRIVER, /*Direction[direction].c_str(),*/
                dType, portIndex);
        ssSetErrorStatus(S, msg);
        return;
    }

    // switch (direction)
    //{
    // case BYTE_PACKING:
    if (ssGetInputPortDataType(S, portIndex) != dType)
    {
        sprintf(msg, "%s: Invalid inherited input port data type %d (%d)", DRIVER, /*Direction[direction].c_str(),*/
                dType, ssGetInputPortDataType(S, portIndex));
        ssSetErrorStatus(S, msg);
        return;
    }
    //    break;
    // case BYTE_UNPACKING:
    //    ssSetInputPortDataType(S, 0, dType);
    //    packed_width = CalculatePackedWidth(S);
    //    if (packed_width == -1)
    //        return;
    //    if (packed_width != ssGetInputPortWidth(S, 0))
    //    {
    //        sprintf(msg, "%s(%s): Invalid inherited input port width %d for data type %d (%d)", DRIVER,
    //                Direction[direction].c_str(), ssGetInputPortWidth(S, 0), dType, packed_width);
    //        ssSetErrorStatus(S, msg);
    //        return;
    //    }
    //    break;
    //}
}

// Set the dimensions of the signals accepted by an output port
// The Simulink engine calls this method with candidate dimensions dimsInfo for port.
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct* S, int_T portIndex, const DimsInfo_T* dimsInfo)
{
    // uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
    // uint_T debug = (uint_T)GetScalarParam(S_DEBUG);

    // if (debug & DEBUG_ENTRY)
    //{
    //     printf("%s(%d) %d %d %d   ", DRIVER, __LINE__, portIndex, dimsInfo->width, dimsInfo->numDims);
    //     for (j = 0; j < dimsInfo->numDims; j++)
    //         printf("%d ", dimsInfo->dims[j]);
    //     printf("\n");
    // }

    if ((dimsInfo->numDims != 1) && (dimsInfo->numDims != 2))
    {
        sprintf(msg, "%s: Invalid inherited output port dimension size %d port %d", DRIVER,
                /*Direction[direction].c_str(),*/ dimsInfo->numDims, portIndex);
        ssSetErrorStatus(S, msg);
        return;
    }

    if (ssGetOutputPortWidth(S, portIndex) == DYNAMICALLY_SIZED)
        ssSetOutputPortVectorDimension(S, portIndex, dimsInfo->width);

    if (ssGetOutputPortWidth(S, portIndex) != dimsInfo->width)
    {
        sprintf(msg, "%s: Invalid inherited output port data width %d port %d (%d)", DRIVER,
                /*Direction[direction].c_str(),*/ dimsInfo->width, portIndex, ssGetOutputPortWidth(S, portIndex));
        ssSetErrorStatus(S, msg);
        return;
    }
}

#define MDL_SET_OUTPUT_PORT_DATA_TYPE
static void mdlSetOutputPortDataType(SimStruct* S, int_T portIndex, DTypeId dType)
{
    // uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
    // uint_T debug = (uint_T)GetScalarParam(S_DEBUG);

    // if (debug & DEBUG_ENTRY)
    //     printf("%s(%d) %d %d\n", DRIVER, __LINE__, portIndex, dType);

    if (ssGetOutputPortDataType(S, portIndex) != dType)
    {
        sprintf(msg, "%s: Invalid inherited output port data type %d port %d (%d)", DRIVER,
                /*Direction[direction].c_str(),*/ dType, portIndex, ssGetOutputPortDataType(S, portIndex));
        ssSetErrorStatus(S, msg);
        return;
    }
}

#define MDL_SET_DEFAULT_PORT_DIMENSION_INFO
static void mdlSetDefaultPortDimensionInfo(SimStruct* S)
{
    // int_T packed_width; // Width of packed port including pad
    //  uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
    // uint_T debug = (uint_T)GetScalarParam(S_DEBUG);

    // if (debug & DEBUG_ENTRY)
    //     printf("%s(%d)\n", DRIVER, __LINE__);

    // switch (direction)
    //{
    // case BYTE_PACKING:
    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS); j++)
        if (ssGetInputPortWidth(S, j) == DYNAMICALLY_SIZED)
            ssSetInputPortVectorDimension(S, j, 1);

    // if (ssGetOutputPortWidth(S, 0) == DYNAMICALLY_SIZED)
    //{
    //     packed_width = CalculatePackedWidth(S);
    //     if (packed_width == -1)
    //         ssSetOutputPortVectorDimension(S, 0, 1);
    //     else
    //         ssSetOutputPortVectorDimension(S, 0, packed_width);
    // }
    //    break;
    // case BYTE_UNPACKING:
    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS); j++)
        if (ssGetOutputPortWidth(S, j) == DYNAMICALLY_SIZED)
            ssSetOutputPortVectorDimension(S, j, 1);

    // if (ssGetInputPortWidth(S, 0) == DYNAMICALLY_SIZED)
    //{
    //     packed_width = CalculatePackedWidth(S);
    //     if (packed_width == -1)
    //         ssSetInputPortVectorDimension(S, 0, 1);
    //     else
    //         ssSetInputPortVectorDimension(S, 0, packed_width);
    // }
    //    break;
    //}
}

#define MDL_SET_DEFAULT_PORT_DATA_TYPES
static void mdlSetDefaultPortDataTypes(SimStruct* S)
{
    // uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
    // uint_T debug = (uint_T)GetScalarParam(S_DEBUG);

    // if (debug & DEBUG_ENTRY)
    //     printf("%s(%d)\n", DRIVER, __LINE__);

    // switch (direction)
    //{
    // case BYTE_PACKING:
    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS); j++)
        if (ssGetInputPortDataType(S, j) == DYNAMICALLY_TYPED)
            ssSetInputPortDataType(S, j, SS_UINT8);
    if (ssGetOutputPortDataType(S, 0) == DYNAMICALLY_TYPED)
        ssSetOutputPortDataType(S, 0, SS_UINT8);
    //    break;
    // case BYTE_UNPACKING:
    for (j = 0; j < (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS); j++)
        if (ssGetOutputPortDataType(S, j) == DYNAMICALLY_TYPED)
            ssSetOutputPortDataType(S, j, SS_UINT8);
    if (ssGetInputPortDataType(S, 0) == DYNAMICALLY_TYPED)
        ssSetInputPortDataType(S, 0, SS_UINT8);
    //    break;
    //}
}

// Initialize the state vectors of this C MEX S-function
// The Simulink® engine invokes this optional method at the beginning of a simulation.
// The method performs initialization activities that this S-function requires only once,
// such as setting up user data or initializing states.
#define MDL_START
static void mdlStart(SimStruct* S)
{
    int_T desired_packed_width; // Calculated width of packed port including pad
    // int_T actual_packed_width;    // Actual width of packed port including pad
    int_T desired_unpacked_width; // Calculated width of packed port including pad
    // int_T actual_unpacked_width;  // Actual width of packed port including pad
    uint_T alignment = (uint_T)GetScalarParam(S_ALIGNMENT);
    int_T unpacked_ports = (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS);
    int_T packed_ports = (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS);

    // input pack width validate, 需要恢复
    desired_packed_width = CalculatePackedWidth(S);
    // if (desired_packed_width == -1)
    //{
    //     sprintf(msg, "%s: Port types and dimensions not fully specified", DRIVER /*, Direction[direction].c_str()*/);
    //     ssSetErrorStatus(S, msg);
    //     return;
    // }
    // actual_packed_width = ssGetInputPortWidth(S, 0);
    // if (desired_packed_width != actual_packed_width)
    //{
    //     sprintf(msg, "%s: Unable to resolve port dimensions (%d %d)", DRIVER, /* Direction[direction].c_str(),*/
    //             desired_packed_width, actual_packed_width);
    //     ssSetErrorStatus(S, msg);
    //     return;
    // }

    // output pack width validate
    desired_unpacked_width = CalculateUnpackedWidth(S);

    sprintf(msg, "%s: unpack malloc:%d, packed malloc:%d\r\n", DRIVER /*, Direction[direction].c_str()*/,
            desired_unpacked_width, desired_packed_width);
    ssPrintf(msg);

    // if (desired_unpacked_width == -1)
    //{
    //     sprintf(msg, "%s: Port types and dimensions not fully specified", DRIVER /*, Direction[direction].c_str()*/);
    //     ssSetErrorStatus(S, msg);
    //     return;
    // }
    // actual_unpacked_width = ssGetOutputPortWidth(S, 0);
    // if (desired_unpacked_width != actual_unpacked_width)
    //{
    //     sprintf(msg, "%s: Unable to resolve port dimensions (%d %d)", DRIVER, /* Direction[direction].c_str(),*/
    //             desired_packed_width, actual_packed_width);
    //     ssSetErrorStatus(S, msg);
    //     return;
    // }

    // setting up user data or initializing states:
    //
    // Calculate input data offset and length
    // assume input data index is j, from 0 to (unpacked_ports -1)
    // offset of the input data: unpacked_ports + j
    // length of the input data: j
    ssSetIWorkValue(S, unpacked_ports + 0, 0);
    for (j = 0; j < unpacked_ports; j++)
    {
        // Calculate data length
        ssSetIWorkValue(S, j, GetDataBytes(ssGetOutputPortDataType(S, j)) * ssGetOutputPortWidth(S, j));

        // Sum up offset
        if (j > 0)
            ssSetIWorkValue(S, unpacked_ports + j,
                            ssGetIWorkValue(S, j - 1) + ssGetIWorkValue(S, unpacked_ports + j - 1) +
                                CalculatePad(ssGetIWorkValue(S, j - 1), alignment));
    }

    // Calculate output data offset and length
    // assume input data index is j, from 0 to (packed_ports -1)
    // offset of the input data: unpack_bias + packed_ports + j
    // length of the input data: unpack_bias + j
    int_T unpack_bias = unpacked_ports * 2;
    ssSetIWorkValue(S, unpack_bias + packed_ports + 0, 0);
    for (j = 0; j < packed_ports; j++)
    {
        // Calculate data length
        ssSetIWorkValue(S, unpack_bias + j, GetDataBytes(ssGetInputPortDataType(S, j)) * ssGetInputPortWidth(S, j));

        // Sum up offset
        if (j > 0)
            ssSetIWorkValue(S, unpack_bias + packed_ports + j,
                            ssGetIWorkValue(S, unpack_bias + j - 1) +
                                ssGetIWorkValue(S, unpack_bias + packed_ports + j - 1) +
                                CalculatePad(ssGetIWorkValue(S, unpack_bias + j - 1), alignment));
    }

    uint8_T* tranBuffer = (uint8_T*)ssGetPWorkValue(S, 1);
    uint8_T* recvBuffer = (uint8_T*)ssGetPWorkValue(S, 2);

    if (tranBuffer != nullptr || recvBuffer != nullptr)
    {
        delete[] tranBuffer;
        delete[] recvBuffer;
    }

    // Transmit data memory assignment here
    tranBuffer = new uint8_T[desired_packed_width];
    recvBuffer = new uint8_T[desired_unpacked_width];

    if (tranBuffer == nullptr || recvBuffer == nullptr)
    {
        delete[] tranBuffer;
        delete[] recvBuffer;

        tranBuffer = nullptr;
        recvBuffer = nullptr;

        sprintf(msg, "%s: Memory allocation error", DRIVER);
        ssSetErrorStatus(S, msg);
        return;
    }

    ssSetPWorkValue(S, 1, tranBuffer);
    ssSetPWorkValue(S, 2, recvBuffer);

    // Setup Network Communication
    sprintf(msg, "%s: mdlStart is invoked\r\n", DRIVER);
    ssPrintf(msg);

    uint32_t ip_addr[4];
    for (j = 0; j < 4; ++j)
    {
        ip_addr[j] = (int_T)GetVectorParam(S_NETWORK_HOST, j);
        if (ip_addr[j] > 255)
        {
            sprintf(msg, "%s: ASIO helper, Wrong IP host address: %d", DRIVER, ip_addr[j]);
            ssSetErrorStatus(S, msg);
        }
    }
    std::stringstream ipaddr;
    ipaddr << ip_addr[0] << "." << ip_addr[1] << "." << ip_addr[2] << "." << ip_addr[3];

    sprintf(msg, "%s: Host IP address:%s.\r\n", DRIVER, ipaddr.str().c_str());
    ssPrintf(msg);

    uint32_t recv_port = (int_T)GetVectorParam(S_NETWORK_TARGET_PORT, 1);
    uint32_t trans_port = (int_T)GetVectorParam(S_NETWORK_TARGET_PORT, 0);
    uint32_t cmd_recv_port = (int_T)GetVectorParam(S_NETWORK_TARGET_PORT, 3);
    uint32_t cmd_trans_port = (int_T)GetVectorParam(S_NETWORK_TARGET_PORT, 2);

    sprintf(msg, "%s: Host port:transmit port: %d, receive port: %d, cmd transmit port: %d, cmd receive port: %d.\r\n",
            DRIVER, trans_port, recv_port, cmd_trans_port, cmd_recv_port);
    ssPrintf(msg);

    // std::string target_ip = "127.0.0.1";
    //  recv_port = 12301;
    //  trans_port = 12300;
    //  cmd_recv_port = 12303;
    //  cmd_trans_port = 12302;

    asio_udp_helper* udp_helper = (asio_udp_helper*)ssGetPWorkValue(S, 0);
    if (udp_helper != nullptr)
    {
        delete udp_helper;
        sprintf(msg, "%s: UDP Helper object hasn't release.", DRIVER);
        ssWarning(S, msg);
    }

    // udp_helper = new asio_udp_helper(target_ip, recv_port, trans_port, cmd_recv_port, cmd_trans_port);
    udp_helper = new asio_udp_helper(ipaddr.str(), recv_port, trans_port, cmd_recv_port, cmd_trans_port);

    if (udp_helper == nullptr)
    {
        sprintf(msg, "%s: ASIO helper cannot create a instance!", DRIVER);
        delete udp_helper;
        udp_helper = nullptr;

        ssSetPWorkValue(S, 0, udp_helper);
        sprintf(msg, "%s: Cannot alloc memory", DRIVER);
        ssSetErrorStatus(S, msg);
    }

    // Generate Configuration files
    json network_config;

    // CMD has been exchange in m file setup
    network_config["target_address"] = ipaddr.str();
    // network_config["receive_port"] = recv_port;
    // network_config["transmit_port"] = trans_port;
    // network_config["command_recv_port"] = cmd_recv_port;
    // network_config["command_trans_port"] = cmd_trans_port;

    // exchange order to fit controller.
    network_config["receive_port"] = trans_port;
    network_config["transmit_port"] = recv_port;
    network_config["command_recv_port"] = cmd_trans_port;
    network_config["command_trans_port"] = cmd_recv_port;

    // show .json content
    std::stringstream json_src;
    json_src << network_config;

    sprintf(msg, "\r\n  %s\r\n\n", json_src.str().c_str());
    ssPrintf(msg);

    sprintf(msg, "\r\n%s: Network Script is generated.\r\n", DRIVER);
    ssPrintf(msg);

    // BUG FIX: add std::ios::trunc flag to clear the whole file.
    std::fstream network_json_file("network.json", std::ios::in | std::ios::out | std::ios::binary | std::ios::trunc);
    // network_json_file.clear();

    if (!network_json_file.is_open())
    {
        network_json_file.open("network.json", std::ios::out | std::ios::app);

        if (!network_json_file.is_open())
        {
            sprintf(msg, "%s: Cannot create or open `network.json`, network script is not generated.\r\n", DRIVER);
            ssWarning(S, msg);
        }
        network_json_file.close();

        // restart
        network_json_file.open("network.json", std::ios::in | std::ios::out | std::ios::binary);

        // Seek to the head of the file
        network_json_file.seekp(std::ios_base::beg);

        network_json_file << network_config;
        network_json_file.close();

        sprintf(msg, "%s: network.json is generated, this script is for controller program!\r\n", DRIVER);
        ssPrintf(msg);
    }
    else
    {
        // Seek to the head of the file
        network_json_file.seekp(std::ios_base::beg);

        network_json_file << network_config;
        network_json_file.close();

        sprintf(msg, "%s: network.json is generated, this script is for controller program!\r\n", DRIVER);
        ssPrintf(msg);
    }

    // Connect to Target
    try
    {
        sprintf(msg, "%s: Connecting to target: %s.\r\n", DRIVER, ipaddr.str().c_str());
        ssPrintf(msg);

        // Create connection
        udp_helper->connect_to_target();

        // Set Overtime
        udp_helper->set_overtime();
    }
    //catch (boost::system::system_error &e)
    catch (asio::system_error& e)
    {
        udp_helper->release_connect();

        delete udp_helper;
        udp_helper = nullptr;

        ssSetPWorkValue(S, 0, udp_helper);

        sprintf(msg, "%s: ASIO helper, Cannot connect to Target! Source exception: %s", DRIVER, e.what());
        ssSetErrorStatus(S, msg);
        return;
    }

    ssSetPWorkValue(S, 0, udp_helper);

    // Send CMD Start to Controller Host
    std::string cmd("Start");

    udp_helper->send_cmd(cmd.c_str(), (uint32_t)cmd.length());
}

// Compute the signals that this block emits
// The Simulink engine invokes this required method at each simulation time step.
// The method should compute the S-function's outputs at the current time step
// and store the results in the S-function's output signal arrays.
static void mdlOutputs(SimStruct* S, int_T tid)
{
    int_T unpacked_ports = (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS);
    int_T packed_ports = (int_T)GetScalarParam(S_NUMBER_PACKED_PORTS);
    int_T unpack_bias = unpacked_ports * 2;

    uint8_T* tranBuffer = (uint8_T*)ssGetPWorkValue(S, 1);
    uint8_T* recvBuffer = (uint8_T*)ssGetPWorkValue(S, 2);

    // Get transmit(input, pack) data
    //
    for (j = 0; j < packed_ports; j++)
    {
        // origin:ssGetOutputPortSignal(S, 0), offset: ssGetIWorkValue(S, unpack_bias + packed_ports + j)
        memcpy((void*)((uint8_T*)tranBuffer + ssGetIWorkValue(S, unpack_bias + packed_ports + j)),
               // input port data, size of data
               (const void*)ssGetInputPortSignal(S, j), ssGetIWorkValue(S, unpack_bias + j));
    }

    // Get udp handle
    asio_udp_helper* udp_helper;
    udp_helper = (asio_udp_helper*)ssGetPWorkValue(S, 0);

    // Communication

    // Test Code:
    // int_T desired_packed_width = CalculatePackedWidth(S);
    // memcpy(recvBuffer, tranBuffer, desired_packed_width * 8);

    // transmit
    int_T desired_packed_width = CalculatePackedWidth(S);
    try
    {
        udp_helper->send_msg((char*)tranBuffer, desired_packed_width);
    }
    //catch (boost::system::system_error &e)
    catch (asio::system_error& e)
    {
        udp_helper->release_connect();

        delete udp_helper;
        udp_helper = nullptr;

        ssSetPWorkValue(S, 0, udp_helper);

        sprintf(msg, "%s: ASIO helper, Cannot transmit message! Source exception: %s", DRIVER, e.what());
        ssSetErrorStatus(S, msg);
        return;
    }

    // receive
    int_T desired_unpacked_width = CalculateUnpackedWidth(S);

    try
    {
        if (udp_helper->recv_msg((char*)recvBuffer, desired_unpacked_width))
        {
            sprintf(msg, "%s: ASIO helper, Receive timeout, please start SIL Controller first!", DRIVER);
            ssSetErrorStatus(S, msg);
        }
    }
    //catch (boost::system::system_error &e)
    catch (asio::system_error& e)
    {
        udp_helper->release_connect();

        delete udp_helper;
        udp_helper = nullptr;

        ssSetPWorkValue(S, 0, udp_helper);

        sprintf(msg, "%s: ASIO helper, Cannot transmit message! Source exception: %s", DRIVER, e.what());
        ssSetErrorStatus(S, msg);
        return;
    }

    // Set output data
    //
    for (j = 0; j < unpacked_ports; j++)
    {
        memcpy((void*)ssGetOutputPortSignal(S, j),
               (const void*)((const uint8_T*)recvBuffer + ssGetIWorkValue(S, unpacked_ports + j)),
               ssGetIWorkValue(S, j));
    }

    return;
}

// Generate code generation data for a C MEX S-function
// #define MDL_RTW
// static void mdlRTW(SimStruct *S)
//{
//    int32_T *bytes; // Number of bytes to copy
//    int32_T *index; // Offset into packed data
//    int_T unpacked_ports = (int_T)GetScalarParam(S_NUMBER_UNPACKED_PORTS);
//    // uint_T direction = (uint_T)GetScalarParam(S_DIRECTION);
//    // uint_T debug = (uint_T)GetScalarParam(S_DEBUG);
//
//    // if (debug & DEBUG_ENTRY)
//    //     printf("%s(%d)\n", DRIVER, __LINE__);
//
//    if ((bytes = (int32_T *)calloc(unpacked_ports, sizeof(int32_T))) == NULL ||
//        (index = (int32_T *)calloc(unpacked_ports, sizeof(int32_T))) == NULL)
//    {
//        free(bytes);
//        free(index);
//        sprintf(msg, "%s: Memory allocation error", DRIVER);
//        ssSetErrorStatus(S, msg);
//        return;
//    }
//
//    for (j = 0; j < unpacked_ports; j++)
//    {
//        bytes[j] = ssGetIWorkValue(S, j);
//        index[j] = ssGetIWorkValue(S, unpacked_ports + j);
//    }
//
//    // ssWriteRTWParamSettings(S, 3, // 这里修改了参数设置
//    //                         SSWRITE_VALUE_DTYPE_NUM, "Direction", &direction, DTINFO(SS_INT32, 0),
//    //                         SSWRITE_VALUE_DTYPE_VECT, "Bytes", bytes, unpacked_ports, DTINFO(SS_INT32, 0),
//    //                         SSWRITE_VALUE_DTYPE_VECT, "Index", index, unpacked_ports, DTINFO(SS_INT32, 0));
//
//    ssWriteRTWParamSettings(S, 2, // 这里修改了参数设置
//                            SSWRITE_VALUE_DTYPE_VECT, "Bytes", bytes, unpacked_ports, DTINFO(SS_INT32, 0),
//                            SSWRITE_VALUE_DTYPE_VECT, "Index", index, unpacked_ports, DTINFO(SS_INT32, 0));
//
//    free(bytes);
//    free(index);
//}

// Perform any actions required at termination of the simulation
// This method performs any actions, such as freeing of memory,
// that must be performed when the simulation is terminated or when an S-Function block is destroyed
// (e.g., when it is deleted from a model).
// This method is called at the end of every simulation in Fast Restart mode.
static void mdlTerminate(SimStruct* S)
{
    // release memory alloc here
    uint8_T* tranBuffer = (uint8_T*)ssGetPWorkValue(S, 1);
    uint8_T* recvBuffer = (uint8_T*)ssGetPWorkValue(S, 2);

    delete[] tranBuffer;
    delete[] recvBuffer;

    tranBuffer = nullptr;
    recvBuffer = nullptr;

    ssSetPWorkValue(S, 1, tranBuffer);
    ssSetPWorkValue(S, 2, recvBuffer);

    // Send CMD Stop to Controller Host
    asio_udp_helper* udp_helper = (asio_udp_helper*)ssGetPWorkValue(S, 0);

    std::string cmd("Stop");
    udp_helper->send_cmd(cmd.c_str(), (uint32_t)cmd.length());

    // release network handle here
    sprintf(msg, "%s: mdlStop is invoked\r\n", DRIVER);
    ssPrintf(msg);

    if (udp_helper != nullptr)
        udp_helper->release_connect();

    if (udp_helper != nullptr)
        delete udp_helper;

    udp_helper = nullptr;

    ssSetPWorkValue(S, 0, udp_helper);

    sprintf(msg, "%s: Bye!\r\n", DRIVER);
    ssPrintf(msg);
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif // MATLAB_MEX_FILE
