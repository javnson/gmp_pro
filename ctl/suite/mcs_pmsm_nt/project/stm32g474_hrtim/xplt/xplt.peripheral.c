//
// THIS IS A DEMO SOURCE CODE FOR GMP LIBRARY.
//
// User should add all definitions of peripheral objects in this file.
//
// User should implement the peripheral objects initialization in setup_peripheral function.
//
// This file is platform-related.
//

// GMP basic core header
#include <gmp_core.h>

// user main header
#include "user_main.h"
#include <xplt.peripheral.h>


//////////////////////////////////////////////////////////////////////////
// definitions of peripheral
//

// ADC DMA buffer
uint32_t adc1_res[ADC1_SEQ_SIZE] = {0};
uint32_t adc2_res[ADC2_SEQ_SIZE] = {0};

// inverter side voltage feedback
tri_ptr_adc_channel_t uuvw;
adc_gt uuvw_src[3];

// inverter side current feedback
tri_ptr_adc_channel_t iuvw;
adc_gt iuvw_src[3];

// DC bus current & voltage feedback
ptr_adc_channel_t udc;
adc_gt udc_src;
ptr_adc_channel_t idc;
adc_gt idc_src;

// Encoder Interface
// ext_as5048a_encoder_t pos_enc;
uint32_t counter;

// a local small cache size, capable of covering the depth of the hardware FIFO (typically 16 bytes)
#define ISR_LOCAL_BUF_SIZE MCS_UART_RX_BUFFER_SIZE
uint8_t rxBuf[ISR_LOCAL_BUF_SIZE];

static void configure_motor_hrtim_channel(uint32_t timer_index, uint32_t output1, uint32_t output2)
{
    HRTIM_TimeBaseCfgTypeDef timebase = {0};
    HRTIM_TimerCtlTypeDef control = {0};
    HRTIM_TimerCfgTypeDef timer = {0};
    HRTIM_CompareCfgTypeDef compare = {0};
    HRTIM_DeadTimeCfgTypeDef deadtime = {0};
    HRTIM_OutputCfgTypeDef output = {0};

    timebase.Period = CTRL_PWM_CMP_MAX;
    timebase.RepetitionCounter = 0;
    timebase.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    timebase.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, timer_index, &timebase) != HAL_OK)
        Error_Handler();

    control.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
    control.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
    control.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
    if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, timer_index, &control) != HAL_OK)
        Error_Handler();

    timer.InterruptRequests = HRTIM_TIM_IT_NONE;
    timer.DMARequests = HRTIM_TIM_DMA_NONE;
    timer.DMASize = 1;
    timer.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
    timer.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
    timer.StartOnSync = HRTIM_SYNCSTART_DISABLED;
    timer.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
    timer.DACSynchro = HRTIM_DACSYNC_NONE;
    timer.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    timer.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
    timer.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    timer.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
    timer.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    timer.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
    timer.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
    timer.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    timer.DelayedProtectionMode = (timer_index <= HRTIM_TIMERINDEX_TIMER_C) ?
                                  HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED :
                                  HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
    timer.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
    timer.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
    timer.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
    timer.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, timer_index, &timer) != HAL_OK)
        Error_Handler();

    compare.CompareValue = CTRL_PWM_CMP_MAX / 2;
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, timer_index, HRTIM_COMPAREUNIT_1, &compare) != HAL_OK)
        Error_Handler();

    deadtime.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    deadtime.RisingValue = CTRL_PWM_DEADBAND_CMP;
    deadtime.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    deadtime.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    deadtime.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
    deadtime.FallingValue = CTRL_PWM_DEADBAND_CMP;
    deadtime.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    deadtime.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    deadtime.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
    if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, timer_index, &deadtime) != HAL_OK)
        Error_Handler();

    output.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
    output.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    output.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    output.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    output.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_index, output1, &output) != HAL_OK)
        Error_Handler();
    output.SetSource = HRTIM_OUTPUTSET_NONE;
    output.ResetSource = HRTIM_OUTPUTRESET_NONE;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, timer_index, output2, &output) != HAL_OK)
        Error_Handler();
}

static void configure_motor_hrtim(void)
{
    HRTIM_ADCTriggerCfgTypeDef adc_trigger = {0};
    configure_motor_hrtim_channel(MCS_HRTIM_PHASE_U_TIMER_INDEX, MCS_HRTIM_PHASE_U_OUTPUT1, MCS_HRTIM_PHASE_U_OUTPUT2);
    configure_motor_hrtim_channel(MCS_HRTIM_PHASE_V_TIMER_INDEX, MCS_HRTIM_PHASE_V_OUTPUT1, MCS_HRTIM_PHASE_V_OUTPUT2);
    configure_motor_hrtim_channel(MCS_HRTIM_PHASE_W_TIMER_INDEX, MCS_HRTIM_PHASE_W_OUTPUT1, MCS_HRTIM_PHASE_W_OUTPUT2);
    adc_trigger.UpdateSource = MCS_HRTIM_ADC_UPDATE_SOURCE;
    adc_trigger.Trigger = MCS_HRTIM_ADC_TRIGGER_EVENT;
    if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &adc_trigger) != HAL_OK)
        Error_Handler();
}

static void configure_motor_adc(void)
{
    ADC_InjectionConfTypeDef injected = {0};
    injected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
    injected.InjectedSingleDiff = ADC_DIFFERENTIAL_ENDED;
    injected.InjectedOffsetNumber = ADC_OFFSET_NONE;
    injected.InjectedOffset = 0;
    injected.InjectedNbrOfConversion = 3;
    injected.InjectedDiscontinuousConvMode = DISABLE;
    injected.AutoInjectedConv = DISABLE;
    injected.QueueInjectedContext = DISABLE;
    injected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_HRTIM_TRG2;
    injected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    injected.InjecOversamplingMode = DISABLE;

    injected.InjectedChannel = ADC_CHANNEL_1;
    injected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
        Error_Handler();
    injected.InjectedChannel = ADC_CHANNEL_6;
    injected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
        Error_Handler();
    injected.InjectedChannel = ADC_CHANNEL_8;
    injected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected) != HAL_OK)
        Error_Handler();
}

/////////////////////////////////////////////////////////////////////////
// peripheral setup function
//

// User should setup all the peripheral in this function.
void setup_peripheral(void)
{
    // Setup Debug Uart
    debug_uart = MCS_UART_HANDLE;

    gmp_base_print(TEXT_STRING("Hello World!\r\n"));

		HAL_Delay(1);
	

    // inverter side ADC
    ctl_init_tri_ptr_adc_channel(
        &uuvw, uuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_tri_ptr_adc_channel(
        &iuvw, iuvw_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_INVERTER_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &udc, &udc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_SENSITIVITY, CTRL_VOLTAGE_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_VOLTAGE_BIAS),
        // ADC resolution, IQN
        12, 24);

    ctl_init_ptr_adc_channel(
        &idc, &idc_src,
        // ADC gain, ADC bias
        ctl_gain_calc_generic(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_SENSITIVITY, CTRL_CURRENT_BASE),
        ctl_bias_calc_via_Vref_Vbias(CTRL_ADC_VOLTAGE_REF, CTRL_DC_CURRENT_BIAS),
        // ADC resolution, IQN
        12, 24);


    //ctl_init_autoturn_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, ((uint32_t)1 << 14) - 1);
//    ctl_init_as5048a_pos_encoder(&pos_enc, MOTOR_PARAM_POLE_PAIRS, SPI_ENCODER_BASE, SPI_ENCODER_NCS);

		// init TIM3 for QEP encoder
		HAL_TIM_Encoder_Start(MCS_ENCODER_TIMER_HANDLE, TIM_CHANNEL_ALL);

    //
    // attach
    //
#if BUILD_LEVEL <= 2
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &rg.enc, &spd_enc.encif);
#else  // BUILD_LEVEL
    ctl_attach_foc_core_port(&mtr_ctrl, &iuvw.control_port, &udc.control_port, &pos_enc.encif, &spd_enc.encif);
#endif // BUILD_LEVEL


    configure_motor_adc();
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    configure_motor_hrtim();
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
                                  MCS_HRTIM_PHASE_U_OUTPUTS |
                                  MCS_HRTIM_PHASE_V_OUTPUTS |
                                  MCS_HRTIM_PHASE_W_OUTPUTS);
    HAL_HRTIM_WaveformCountStart(&hhrtim1,
                                 MCS_HRTIM_PHASE_U_TIMER_ID |
                                 MCS_HRTIM_PHASE_V_TIMER_ID |
                                 MCS_HRTIM_PHASE_W_TIMER_ID);

		// Enable DAC channels
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
		HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
		
		// Enable UART RX DMA
		HAL_UART_Receive_DMA(debug_uart, rxBuf, ISR_LOCAL_BUF_SIZE);



}

//////////////////////////////////////////////////////////////////////////
// interrupt functions and callback functions here

// ADC interrupt
// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
// {
//     if (hadc == &hadc1)
//     {
//         gmp_base_ctl_step();
//     }
// }

/**
  * @brief  Injected conversion complete callback in non blocking mode
  * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
    {
        gmp_base_ctl_step();
        counter++;
    }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		// Index
    if(GPIO_Pin == GPIO_PIN_9)
    {
        __HAL_TIM_SET_COUNTER(MCS_ENCODER_TIMER_HANDLE, 0);
        
    }
}

void send_monitor_data(void)
{}

extern gmp_datalink_t dl;

void flush_dl_tx_buffer(void)
{
    gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_hdr_ptr(&dl),
                       gmp_dev_dl_get_tx_hw_hdr_size(&dl), 10);
    if (gmp_dev_dl_get_tx_hw_pld_size(&dl) > 0)
    {
        gmp_hal_uart_write(debug_uart, gmp_dev_dl_get_tx_hw_pld_ptr(&dl),
                           gmp_dev_dl_get_tx_hw_pld_size(&dl), 10);
    }
}

void flush_dl_rx_buffer(void)
{
    static volatile uint16_t last_read_pos = 0;
    uint16_t current_pos = ISR_LOCAL_BUF_SIZE - __HAL_DMA_GET_COUNTER(debug_uart->hdmarx);
    uint16_t len1 = 0, len2 = 0;
    uint16_t start1 = last_read_pos;

    if (current_pos == last_read_pos)
        return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (current_pos > last_read_pos)
        len1 = current_pos - last_read_pos;
    else
    {
        len1 = ISR_LOCAL_BUF_SIZE - last_read_pos;
        len2 = current_pos;
    }
    last_read_pos = current_pos;
    __set_PRIMASK(primask);

    if (len1 > 0)
        gmp_dev_dl_push_str(&dl, (data_gt*)&rxBuf[start1], len1);
    if (len2 > 0)
        gmp_dev_dl_push_str(&dl, (data_gt*)&rxBuf[0], len2);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == MCS_UART_INSTANCE)
        flush_dl_rx_buffer();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == MCS_UART_INSTANCE)
        flush_dl_rx_buffer();
}
