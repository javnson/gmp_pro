// #############################################################################
//  $Copyright:
//  Copyright (C) 2017-2024 Texas Instruments Incorporated - http://www.ti.com/
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  $
// #############################################################################

/**
 * @file dlog_4ch_f.h
 * @brief Header file for a 4-channel data logging module (floating-point).
 * @details This module provides functionality to log data from four different
 * sources into separate buffers based on a trigger condition. The trigger is
 * a rising edge on the first input channel crossing a specified threshold.
 * The data logging can be down-sampled using a prescaler.
 * @version 1.0
 * @author Texas Instruments
 *
 */

#ifndef DLOG_4CH_F_H
#define DLOG_4CH_F_H

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

/**
 * @defgroup DATA_LOGGER_4CH 4-Channel Data Logger
 * @brief A module for triggered, multi-channel data acquisition.
 */

/*---------------------------------------------------------------------------*/
/* 4-Channel Data Logger                                                     */
/*---------------------------------------------------------------------------*/

/**
 * @addtogroup DATA_LOGGER_4CH
 * @{
 */

/**
 * @brief Data structure for the 4-channel data logger.
 */
typedef struct
{
    float32_t* input_ptr1;  /**< Pointer to the first data source (also the trigger source). */
    float32_t* input_ptr2;  /**< Pointer to the second data source. */
    float32_t* input_ptr3;  /**< Pointer to the third data source. */
    float32_t* input_ptr4;  /**< Pointer to the fourth data source. */
    float32_t* output_ptr1; /**< Pointer to the output buffer for channel 1. */
    float32_t* output_ptr2; /**< Pointer to the output buffer for channel 2. */
    float32_t* output_ptr3; /**< Pointer to the output buffer for channel 3. */
    float32_t* output_ptr4; /**< Pointer to the output buffer for channel 4. */
    float32_t prev_value;   /**< The previous value of the trigger source (input_ptr1). */
    float32_t trig_value;   /**< The threshold value for the rising-edge trigger. */
    int16_t status;         /**< The current state of the logger (0: Idle, 1: Armed/Waiting for trigger, 2: Logging). */
    int16_t pre_scalar;     /**< The prescaler for data logging (logs one sample every `pre_scalar` calls). */
    int16_t skip_count;     /**< Internal counter for the prescaler. */
    int16_t size;           /**< The size of the output buffers. */
    int16_t count;          /**< The current index for writing into the output buffers. */
} DLOG_4CH_F;

/**
 * @brief Initializes the 4-channel data logger object.
 * @param v Pointer to the `DLOG_4CH_F` object.
 */
static inline void DLOG_4CH_F_init(DLOG_4CH_F* v)
{
    v->input_ptr1 = 0;
    v->input_ptr2 = 0;
    v->input_ptr3 = 0;
    v->input_ptr4 = 0;
    v->output_ptr1 = 0;
    v->output_ptr2 = 0;
    v->output_ptr3 = 0;
    v->output_ptr4 = 0;
    v->prev_value = 0;
    v->trig_value = 0;
    v->status = 0;
    v->pre_scalar = 0;
    v->skip_count = 0;
    v->size = 0;
    v->count = 0;
}

/**
 * @brief Executes one step of the data logging logic (inline function version).
 * @details This function checks for the trigger condition and logs data if the trigger is active.
 * It should be called periodically at the desired sampling rate.
 * @param v Pointer to the `DLOG_4CH_F` object.
 */
static inline void DLOG_4CH_F_FUNC(DLOG_4CH_F* v)
{
    switch (v->status)
    {
    case 1: // Wait for trigger
        if (((*v->input_ptr1) > v->trig_value) && (v->prev_value < v->trig_value))
        {
            // Rising edge detected, start logging data.
            v->status = 2;
        }
        break;

    case 2: // Logging active
        v->skip_count++;
        if (v->skip_count == v->pre_scalar)
        {
            v->skip_count = 0;
            v->output_ptr1[v->count] = *v->input_ptr1;
            v->output_ptr2[v->count] = *v->input_ptr2;
            v->output_ptr3[v->count] = *v->input_ptr3;
            v->output_ptr4[v->count] = *v->input_ptr4;
            v->count++;

            if (v->count == v->size)
            {
                // Buffer is full, reset and wait for the next trigger.
                v->count = 0;
                v->status = 1;
            }
        }
        break;
    }

    v->prev_value = *v->input_ptr1;
}

/**
 * @brief Macro version of the data logging logic for higher performance.
 * @details This macro implements the same functionality as `DLOG_4CH_F_FUNC` but avoids
 * the function call overhead, potentially leading to faster execution.
 * @param v A `DLOG_4CH_F` object (not a pointer).
 */
#define DLOG_4CH_F_MACRO(v)                                                                                            \
    switch (v.status)                                                                                                  \
    {                                                                                                                  \
    case 1: /* wait for trigger*/                                                                                      \
        if (*v.input_ptr1 > v.trig_value && v.prev_value < v.trig_value)                                               \
        {                                                                                                              \
            /* rising edge detected start logging data*/                                                               \
            v.status = 2;                                                                                              \
        }                                                                                                              \
        break;                                                                                                         \
    case 2:                                                                                                            \
        v.skip_count++;                                                                                                \
        if (v.skip_count == v.pre_scalar)                                                                              \
        {                                                                                                              \
            v.skip_count = 0;                                                                                          \
            v.output_ptr1[v.count] = *v.input_ptr1;                                                                    \
            v.output_ptr2[v.count] = *v.input_ptr2;                                                                    \
            v.output_ptr3[v.count] = *v.input_ptr3;                                                                    \
            v.output_ptr4[v.count] = *v.input_ptr4;                                                                    \
            v.count++;                                                                                                 \
            if (v.count == v.size)                                                                                     \
            {                                                                                                          \
                v.count = 0;                                                                                           \
                v.status = 1;                                                                                          \
            }                                                                                                          \
        }                                                                                                              \
        break;                                                                                                         \
    default:                                                                                                           \
        v.status = 0;                                                                                                  \
    }                                                                                                                  \
    v.prev_value = *v.input_ptr1;

/**
 * @}
 */

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* DLOG_4CH_F_H_ */
