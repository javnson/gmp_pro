//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all declarations of controller objects in this file.
//
// User should implement the Main ISR of the controller tasks.
//
// User should ensure that all the controller codes here is platform-independent.
//
// WARNING: This file must be kept in the include search path during compilation.
//

#include "ctl_main.h" // Includes SINV modules and ADC structures
#include <xplt.peripheral.h>

#ifndef _FILE_CTL_INTERFACE_H_
#define _FILE_CTL_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

//=================================================================================================
// Controller interface

// Input Callback
GMP_STATIC_INLINE void ctl_input_callback(void)
{
    // Fetch raw ADC data from hardware registers and process through the ADC channels
    // Note: Ensure the INV_VGRID_RESULT_BASE, INV_IAC_RESULT_BASE, etc.,
    // are correctly mapped to your hardware macros in xplt.peripheral.h

    ctl_step_adc_channel(&adc_v_grid, ADC_readResult(INV_VGRID_RESULT_BASE, INV_VGRID));
    ctl_step_adc_channel(&adc_i_ac, ADC_readResult(INV_IAC_RESULT_BASE, INV_IAC));
    ctl_step_adc_channel(&adc_v_bus, ADC_readResult(INV_VBUS_RESULT_BASE, INV_VBUS));
}

// Output Callback
GMP_STATIC_INLINE void ctl_output_callback(void)
{
    // Write ePWM peripheral CMP for H-Bridge (Phase L and Phase N)
    EPWM_setCounterCompareValue(PHASE_L_BASE, EPWM_COUNTER_COMPARE_A, pwm_cmp_L);
    EPWM_setCounterCompareValue(PHASE_N_BASE, EPWM_COUNTER_COMPARE_A, pwm_cmp_N);

    // DAC Monitor Port (Offset by 2048 for bipolar signals on a 12-bit DAC)
#if BUILD_LEVEL >= 1
    // Monitor Grid Voltage and AC Current via DAC
    DAC_setShadowValue(IRIS_DACA_BASE, (uint16_t)(adc_v_grid.control_port.value * 2048.0f + 2048.0f));
    DAC_setShadowValue(IRIS_DACB_BASE, (uint16_t)(adc_i_ac.control_port.value * 2048.0f + 2048.0f));
#endif // BUILD_LEVEL
}

// function prototype
void GPIO_WritePin(uint16_t gpioNumber, uint16_t outVal);

// Enable System Controller Output
GMP_STATIC_INLINE void ctl_fast_enable_output(void)
{
    // Clear any Trip Zone (TZ) flag for Phase L and Phase N
    EPWM_clearTripZoneFlag(PHASE_L_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_clearTripZoneFlag(PHASE_N_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // Reset algorithm history to prevent sudden jumps
    clear_all_controllers();

    // Hardware PWM gate driver enable
    GPIO_WritePin(PWM_ENABLE_PORT, 1);

    // Turn ON Controller LED (assuming Active-Low LED)
    GPIO_WritePin(CONTROLLER_LED, 0);
}

// Disable System Controller Output
GMP_STATIC_INLINE void ctl_fast_disable_output(void)
{
    // Force Trip Zone (TZ) event to hardware-block PWM outputs immediately
    EPWM_forceTripZoneEvent(PHASE_L_BASE, EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(PHASE_N_BASE, EPWM_TZ_FORCE_EVENT_OST);

    // Hardware PWM gate driver disable
    GPIO_WritePin(PWM_ENABLE_PORT, 0);

    // Turn OFF Controller LED (assuming Active-Low LED)
    GPIO_WritePin(CONTROLLER_LED, 1);
}

//=================================================================================================
// Controller interface for PIL (Processor-in-the-Loop) simulation

typedef enum _tag_adc_index_items
{
    INV_ADC_ID_VBUS = 0,
    INV_ADC_ID_VGRID = 1,
    INV_ADC_ID_IAC = 2,

    INV_ADC_SENSOR_NUMBER = 3
} inv_adc_index_items;

// Input Callback for PIL Simulation
GMP_STATIC_INLINE void ctl_input_callback_pil(const gmp_sim_rx_buf_t* rx)
{
    // Inject simulated ADC raw data directly into the ADC channel processing pipeline
    ctl_step_adc_channel(&adc_v_bus, rx->adc_result[INV_ADC_ID_VBUS]);
    ctl_step_adc_channel(&adc_v_grid, rx->adc_result[INV_ADC_ID_VGRID]);
    ctl_step_adc_channel(&adc_i_ac, rx->adc_result[INV_ADC_ID_IAC]);
}

// Output Callback for PIL Simulation
GMP_STATIC_INLINE void ctl_output_callback_pil(gmp_sim_tx_buf_t* tx)
{
    //
    // PWM channel (Send calculated compare values back to the simulator)
    //
    tx->pwm_cmp[0] = pwm_cmp_L;
    tx->pwm_cmp[1] = pwm_cmp_N;

    //
    // Monitor Data (Send controller states to simulator scope)
    //

    // Scope 1: Feed back the measured AC current
    tx->monitor[0] = adc_i_ac.control_port.value;

    // Scope 2: Feed back the measured Grid Voltage
    tx->monitor[1] = adc_v_grid.control_port.value;
}

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _FILE_CTL_INTERFACE_H_
