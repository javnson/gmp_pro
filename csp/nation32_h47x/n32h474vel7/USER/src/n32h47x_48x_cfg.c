/**
 * @file n32h47x_48x_cfg.c
 * @author N32cube
 */

 #include "n32h47x_48x_cfg.h"

/* NTFx CODE START */
__IO uint32_t mwTick;
void SysTick_Delayms(uint32_t Delayms)
{
    uint32_t tickstart = mwTick;
    uint32_t wait=Delayms;
    /* Add 1 to guarantee minimum wait */
    if (wait < 0xFFFFFFFFU)
    {
        wait +=1;
    }
    while ((mwTick - tickstart) < wait)
    {
    }
}
 /**
 *@name  DMA_SetPerMemAddr.
 *@brief Set peripher address and memory address of DMA
 *@param DMAChx (The input parameters must be the following values):
 *          - DMA1_CH1
 *          - DMA1_CH2
 *          - DMA1_CH3
 *          - DMA1_CH4
 *          - DMA1_CH5
 *          - DMA1_CH6
 *          - DMA1_CH7
 *          - DMA1_CH8
 *          - DMA2_CH1
 *          - DMA2_CH2
 *          - DMA2_CH3
 *          - DMA2_CH4
 *          - DMA2_CH5
 *          - DMA2_CH6
 *          - DMA2_CH7
 *          - DMA2_CH8
 *@param periphAddr   peripher address
 *@param memAddr   memory address
 *@param bufSize   buff size
 *@return status
 */
 void DMA_SetPerMemAddr(DMA_ChannelType* DMAChx, uint32_t periphAddr,uint32_t memAddr,uint32_t bufSize )
 {
     /* DMAy Channelx TXNUM Configuration */
    /* Write to DMAy Channelx TXNUM */
    DMAChx->TXNUM = bufSize;

    /* DMAy Channelx PADDR Configuration */
    /* Write to DMAy Channelx PADDR */
    DMAChx->PADDR = periphAddr;

    /* DMAy Channelx MADDR Configuration */
    /* Write to DMAy Channelx MADDR */
    DMAChx->MADDR = memAddr;
 }

 /**
 *@name    RCC_InitSHRTPLL.
 *@brief     Initialze the SHRTPLL clock source and multiplication factor.
 *@param   SHRTPLL_source(SHRTPLL entry clock source):
 *   		  - RCC_SHRTPLL_SRC_HSI         HSI oscillator clock selected as SHRTPLL clock entry
 *   		  - RCC_SHRTPLL_SRC_HSE         HSE oscillator clock selected as SHRTPLL clock entry
 *@param   CLKF(SHRTPLL Multiple coefficient)
 *@param   CLKR(SHRTPLL division coefficient)
 *@param   BWTRIM(Bandwidth adjustment)
 *@return  none: 
 *note    SHRTPLL = SHRTPLL_source*(CLKF[25:0]/16384)/(CLKR[5:0] +1)/4
 */
void RCC_InitSHRTPLL(uint32_t SHRTPLL_source, uint32_t CLKF, uint32_t CLKR, uint32_t BWTRIM)
{
    uint32_t temp_value1,temp_value2,temp_value3;  


    /* Enable SHRTPLL module power */
    RCC->SHRTPLLCTRL3 |= RCC_SHRTPLL_ENABLE;
    /* Select Clock Source */
    temp_value3 = RCC->SHRTPLLCTRL3;
    temp_value3 &= RCC_SHRTPLL_SRC_MASK;
    temp_value3 |= SHRTPLL_source;
    RCC->SHRTPLLCTRL3 = temp_value3;
    /* Configure the work mode to normal */
    RCC->SHRTPLLCTRL1 &= (~RCC_SHRTPLLMODE_PD);
    /* Enable Saturation and fraction accumulation function */
    RCC->SHRTPLLCTRL1 |= (RCC_SHRTPLL_SATEN|RCC_SHRTPLL_FRACEN);
    
    /* get the SHRTPLL value */
    temp_value1 = RCC->SHRTPLLCTRL1;
    temp_value2 = RCC->SHRTPLLCTRL2;
    temp_value3 = RCC->SHRTPLLCTRL3;

    /* Clear CLKF[25:0] bits */
    temp_value1 &= RCC_SHRTPLL_CLKF_MASK;
    /* Clear CLKR[5:0] bits */
    temp_value2 &= RCC_SHRTPLL_CLKR_MASK;
    /* Clear BWTRIM[11:0] bits */
    temp_value3 &= RCC_SHRTPLL_BWTRIM_MASK;

    /* Set CLKF[25:0] bits */
    temp_value1 |= (uint32_t)CLKF;
    /* Set CLKR[5:0] bits */
    temp_value2 |= ((uint32_t)CLKR<<26);
    /* Set BWTRIM[11:0] bits */
    temp_value3 |= ((uint32_t)BWTRIM<<12);

    /* Store the new value */
    RCC->SHRTPLLCTRL1  = temp_value1;
    RCC->SHRTPLLCTRL2  = temp_value2;
    RCC->SHRTPLLCTRL3  = temp_value3;
    
    /* Reset SHRTPLL */
    RCC_EnableAHBPeriphReset(RCC_AHB_PERIPHRST_SHRTPLL);

}
/* NTFx CODE END */
     
/* NTFx CODE START */
/**
 *@brief Initializes the clock tree
 *@param null
 *@return status
 */
bool RCC_Configuration(void)
{
    ErrorStatus ClockStatus;
    RCC_DeInit();
     
    RCC_EnableHsi(ENABLE);
    /* Wait till HSI is ready */
    ClockStatus = RCC_WaitHsiStable();
    if (ClockStatus != SUCCESS) return false;
     
    RCC_ConfigHse(RCC_HSE_ENABLE);
    /* Wait till HSE is ready */
    ClockStatus = RCC_WaitHseStable();
    if (ClockStatus != SUCCESS) return false;
     
    RCC_ConfigLse(RCC_LSE_ENABLE);
    /* Wait till LSE is ready */
    ClockStatus = RCC_WaitLseStable();
    if (ClockStatus != SUCCESS) return false;
     
    RCC_ConfigPll(RCC_PLL_SRC_HSE,RCC_PLL_PRE_2,RCC_PLL_MUL_60,RCC_PLLOUT_DIV_1);
    /* Enable PLL */
    RCC_EnablePll(ENABLE);
    /* Wait till PLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDF) != SET);
     
    /* Enable phase reset to filter waveform burrs */
    RCC_EnableSHRTPLLPHAReset(ENABLE);
    /* Configures the SHRTPLL clock source and multiplication factor */
    RCC_InitSHRTPLL(RCC_SHRTPLL_SRC_HSI,2048000,0,61);//Fin:8000000,Fout:250000000
    //if(RCC_ConfigSHRTPll(RCC_SHRTPLL_SRC_HSI,8000000,250000000, ENABLE) != SUCCESS)
    //  return false;
    /* Wait till SHRTPLL is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_SHRTPLLRDF) != SET);
    /* Disable phase reset to filter waveform burrs */
    RCC_EnableSHRTPLLPHAReset(DISABLE);
     
    /* Select PLL clock 10 divisions*/
    RCC_ConfigUSBPLLPresClk(RCC_USBPLLCLK_SRC_PLL,RCC_USBPLLCLK_DIV10);
    RCC->CFG3 |= RCC_CFG3_USBFSTM;
    /* Select PLL division clock to USBHS PLL source*/
    RCC_ConfigUSBHSClk(RCC_USBHS_CLKSRC_PLLPRES);
    RCC_ConfigHclk(RCC_SYSCLK_DIV1);
    RCC_ConfigPclk2(RCC_HCLK_DIV2);
    RCC_ConfigPclk1(RCC_HCLK_DIV2);
     
    /* Disable Prefetch Buffer */
    FLASH_PrefetchBufSet(FLASH_PrefetchBuf_DIS);
    FLASH_iCacheCmd(FLASH_iCache_EN);
    /* Flash wait state */
    FLASH_SetLatency(FLASH_LATENCY_5);
    /* Enable PWR peripheral clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
    /* Enable access to backup registers */
    PWR_BackupAccessEnable(ENABLE);
     
    /*selsect HSI as RCC ADC1M CLK Source*/
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);  
    RCC_ConfigAdcPllClk(RCC_ADCPLLCLK_SRC_USBHS240M, RCC_ADCPLLCLK_DIV1, RCC_ADCPLLCLK_DISABLE);
    RCC_ConfigAdcHclk(RCC_ADCHCLK_DIV3);
     
    /*config RNG clock*/
    RCC_ConfigTrng1mClk(RCC_TRNG1MCLK_SRC_HSI, RCC_TRNG1MCLK_DIV8);
    RCC_EnableTrng1mClk(ENABLE);
    RCC_ConfigRngcClk(RCC_RNGCCLK_SYSCLK_DIV1);
    /*config GTIM8/9/10 clock*/
    RCC_ConfigGTimClk(RCC_GTIM_CLKSRC_PCLK);
     
    /*config RTC clock*/
    RCC_ConfigRtcClk(RCC_RTCCLK_SRC_LSE);
    RCC_EnableRtcClk(ENABLE);
     
    /*config FDCAN clock*/
    RCC_ConfigFDCANPllClk(RCC_FDCAN_PLLSRC_DIV1);
    RCC_ConfigFDCANClksrc(RCC_FDCAN_CLKSRC_PLL);
    /*config TRACE clock*/
    RCC_ConfigTraceClk(RCC_TRACECLK_DIV1);
     
    /* Select PLL as system clock source */
    RCC_ConfigSysclk(RCC_SYSCLK_SRC_PLL);
    /* Wait till PLL is used as system clock source */
    while (RCC_GetSysclkSrc() != RCC_CFG_SCLKSTS_PLL) ;
    /*  Configure the SysTick to have interrupt in 1ms time basis*/
    SysTick_Config(240000);

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the NVIC
 *@param null
 *@return status
 */
bool NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;
    /*Configure the preemption priority and subpriority:
    - 4 bits for pre-emption priority: possible value are 0..15
    - 0 bits for subpriority: possible value are 0
    - Lower values gives higher priority
    */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    NVIC_SetPriority(SysTick_IRQn,NVIC_EncodePriority(4,15,0));
    
    /*Set GTIM10_IRQ  interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   =GTIM10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        =0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the DMA
 *@param null
 *@return status
 */
bool DMA_Configuration(void)
{

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the GPIO
 *@param null
 *@return status
 */
bool GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;
    GPIO_InitStruct(&GPIO_InitStructure);
     
    /* Enable the GPIO clock*/
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_GPIOB | RCC_AHB_PERIPHEN_GPIOA | RCC_AHB_PERIPHEN_GPIOH | RCC_AHB_PERIPHEN_GPIOC | RCC_AHB_PERIPHEN_GPIOE | RCC_AHB_PERIPHEN_GPIOF, ENABLE);
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_AFIO,ENABLE);
    
     
    GPIO_SetBits(GPIOA,GPIO_PIN_3);
    GPIO_SetBits(GPIOB,GPIO_PIN_4);
     
    /*Initialize Out_PP GPIO */
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DS_2mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitStructure.Pin            = GPIO_PIN_3;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
     
    GPIO_InitStructure.Pin            = GPIO_PIN_4;
    GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure);
     
    /*Initialize AF_PP GPIO */
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Pull      = GPIO_NO_PULL;
    GPIO_InitStructure.GPIO_Slew_Rate = GPIO_SLEW_RATE_SLOW;
    GPIO_InitStructure.GPIO_Current   = GPIO_DS_2mA;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6;
    GPIO_InitStructure.Pin            = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
     
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF11;
    GPIO_InitStructure.Pin            = GPIO_PIN_9 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitPeripheral(GPIOC,&GPIO_InitStructure);
     
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF12;
    GPIO_InitStructure.Pin            = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
     
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF10;
    GPIO_InitStructure.Pin            = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure);
     
    /*Initialize analog GPIO */
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_ANALOG;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitStructure.Pin            = GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
     
    GPIO_InitStructure.Pin            = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOB,&GPIO_InitStructure);
     
    GPIO_InitStructure.Pin            = GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOC,&GPIO_InitStructure);
     
    GPIO_InitStructure.Pin            = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_10 | GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_9;
    GPIO_InitPeripheral(GPIOE,&GPIO_InitStructure);
     
    GPIO_InitStructure.Pin            = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOF,&GPIO_InitStructure);
     
    /*Initialize input GPIO */
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull      = GPIO_PULL_UP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_NO_AF;
    GPIO_InitStructure.Pin            = GPIO_PIN_2;
    GPIO_InitPeripheral(GPIOA,&GPIO_InitStructure);
     
    GPIO_ConfigPinRemap(0,0,GPIO_RMP_SWJ_SWD);
    

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the USART
 *@param null
 *@return status
 */
bool USART_Configuration(void)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    /* EnableUART5clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_UART5,ENABLE);
    /* EnableUSART1|UART7clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1|RCC_APB2_PERIPH_UART7,ENABLE);
     
     
    /*********initialize the USART1************/
    USART_DeInit(USART1);
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO; 
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE; 
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX; 
    USART_InitStructure.OverSampling        = USART_16OVER ; 
    /* Configure USART1 */
    USART_Init(USART1, &USART_InitStructure);
     
     
    /* Enable the USART1 */
    USART_Enable(USART1, ENABLE);
     
    /*********initialize the UART5************/
    USART_DeInit(UART5);
    /* Configure UART5 */
    USART_Init(UART5, &USART_InitStructure);
     
     
    /* Enable the UART5 */
    USART_Enable(UART5, ENABLE);
     
    /*********initialize the UART7************/
    USART_DeInit(UART7);
    /* Configure UART7 */
    USART_Init(UART7, &USART_InitStructure);
     
     
    /* Enable the UART7 */
    USART_Enable(UART7, ENABLE);

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the FDCAN
 *@param null
 *@return status
 */
bool FDCAN_Configuration(void)
{

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the I2C
 *@param null
 *@return status
 */
bool I2C_Configuration(void)
{
     
    /* EnableI2C1clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_I2C1,ENABLE);
    
    //Use reference:
    //I2C_GenerateStart(I2C1, ENABLE);
    //I2C_SendAddr7bit(I2C1, I2C_SLAVE_ADDR, I2C_DIRECTION_SEND); 
    //I2C_SendData(I2C1, *sendBufferPtr++);
    //I2C_GenerateStop(I2C1, ENABLE);
     

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the DAC
 *@param null
 *@return status
 */
bool DAC_Configuration(void)
{
    DAC_InitType DAC_InitStructure;
    DAC_StructInit(&DAC_InitStructure);
    /* Enable DAC12|DAC34 clock */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_DAC12|RCC_APB1_PERIPH_DAC34,ENABLE);
     
    /* Config DAC12 frequency and prescaler factor */
    DAC_ConfigClkPrescaler(DAC12,119);
    DAC_SetHighFrequencyMode(DAC12,DAC_HIGH_FREQ_MODE_DISABLE);
     
    /* Config DAC34 frequency and prescaler factor */
    DAC_ConfigClkPrescaler(DAC34,119);
    DAC_SetHighFrequencyMode(DAC34,DAC_HIGH_FREQ_MODE_DISABLE);
     
    /*init DAC1 */
    DAC_InitStructure.DAC_Trigger                       = DAC_Trigger_Software;
    DAC_InitStructure.DAC_Trigger2                      = DAC_Trigger2_Software;
    DAC_InitStructure.DAC_WaveGeneration                = DAC_WaveGeneration_None;
    DAC_InitStructure.DAC_OutputBuffer                  = ENABLE;
    DAC_InitStructure.DAC_TriggerEnable                 = DISABLE;
    DAC_InitStructure.DAC_DMADoubleDataMode             = DISABLE;
    DAC_InitStructure.DAC_SignedFormat                  = DISABLE;
    DAC_InitStructure.DAC_ConnectOnChipPeripheral       = DISABLE;
    DAC_InitStructure.DAC_ConnectExternalPin            = ENABLE;
    DAC_Init(DAC1, &DAC_InitStructure);
    /* Enable DAC1. */
    DAC_Enable(DAC1, ENABLE);
     
    /*init DAC2 */
    DAC_Init(DAC2, &DAC_InitStructure);
    /* Enable DAC2. */
    DAC_Enable(DAC2, ENABLE);
     
    /*init DAC3 */
    DAC_Init(DAC3, &DAC_InitStructure);
    /* Enable DAC3. */
    DAC_Enable(DAC3, ENABLE);
     
    /*init DAC4 */
    DAC_Init(DAC4, &DAC_InitStructure);
    /* Enable DAC4. */
    DAC_Enable(DAC4, ENABLE);

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the RTC
 *@param null
 *@return status
 */
bool RTC_Configuration(void)
{

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the ADC
 *@param null
 *@return status
 */
bool ADC_Configuration(void)
{
    ADC_InitType ADC_InitStructure;
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC1 ,ENABLE);
    ADC_EnableCH1PositiveEndConnetPGA_P(ADC1,DISABLE);
    ADC_EnableCH1NegtiveEndConnetPGA_N(ADC1,DISABLE);
    ADC_EnableCH2PositiveEndConnetPGA_N(ADC1,DISABLE);
    
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_InitStructure.Resolution = ADC_DATA_RES_6BIT;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_SelectClockMode(ADC1,ADC_CLOCK_MODE_AHB);
    /* Config ADC1  regular channels */
    ADC_ConfigRegularChannel(ADC1,ADC_CH_1,1,ADC_SAMP_TIME_CYCLES_1_5);
    /* Enable ADC1 */
    ADC_Enable(ADC1, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatus(ADC1,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_CalibrationOperation(ADC1,ADC_CALIBRATION_SINGLE_MODE);
    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC1,ADC_CALIBRATION_SINGLE_MODE))
        ;
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC2 ,ENABLE);
    ADC_EnableCH1PositiveEndConnetPGA_P(ADC2,DISABLE);
    ADC_EnableCH1NegtiveEndConnetPGA_N(ADC2,DISABLE);
    ADC_EnableCH2PositiveEndConnetPGA_N(ADC2,DISABLE);
    
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 2;
    ADC_InitStructure.Resolution = ADC_DATA_RES_6BIT;
    ADC_Init(ADC2, &ADC_InitStructure);
    ADC_SelectClockMode(ADC2,ADC_CLOCK_MODE_AHB);
    /* Config ADC2  regular channels */
    ADC_ConfigRegularChannel(ADC2,ADC_CH_11,1,ADC_SAMP_TIME_CYCLES_1_5);
    ADC_ConfigRegularChannel(ADC2,ADC_CH_12,2,ADC_SAMP_TIME_CYCLES_1_5);
    /* Enable ADC2 */
    ADC_Enable(ADC2, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatus(ADC2,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC2 calibration */
    ADC_CalibrationOperation(ADC2,ADC_CALIBRATION_SINGLE_MODE);
    /* Check the end of ADC2 calibration */
    while (ADC_GetCalibrationStatus(ADC2,ADC_CALIBRATION_SINGLE_MODE))
        ;
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC3 ,ENABLE);
    ADC_EnableCH1PositiveEndConnetPGA_P(ADC3,DISABLE);
    ADC_EnableCH1NegtiveEndConnetPGA_N(ADC3,DISABLE);
    ADC_EnableCH2PositiveEndConnetPGA_N(ADC3,DISABLE);
    
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_InitStructure.Resolution = ADC_DATA_RES_6BIT;
    ADC_Init(ADC3, &ADC_InitStructure);
    ADC_SelectClockMode(ADC3,ADC_CLOCK_MODE_AHB);
    /* Config ADC3  regular channels */
    ADC_ConfigRegularChannel(ADC3,ADC_CH_2,1,ADC_SAMP_TIME_CYCLES_1_5);
    /* Enable ADC3 */
    ADC_Enable(ADC3, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatus(ADC3,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC3 calibration */
    ADC_CalibrationOperation(ADC3,ADC_CALIBRATION_SINGLE_MODE);
    /* Check the end of ADC3 calibration */
    while (ADC_GetCalibrationStatus(ADC3,ADC_CALIBRATION_SINGLE_MODE))
        ;
    RCC_EnableAHB1PeriphClk(RCC_AHB_PERIPHEN_ADC4 ,ENABLE);
    ADC_EnableCH1PositiveEndConnetPGA_P(ADC4,DISABLE);
    ADC_EnableCH1NegtiveEndConnetPGA_N(ADC4,DISABLE);
    ADC_EnableCH2PositiveEndConnetPGA_N(ADC4,DISABLE);
    
    ADC_InitStruct(&ADC_InitStructure);
    ADC_InitStructure.WorkMode = ADC_WORKMODE_INDEPENDENT;
    ADC_InitStructure.MultiChEn = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIG_REG_CONV_SOFTWARE;
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = 1;
    ADC_InitStructure.Resolution = ADC_DATA_RES_6BIT;
    ADC_Init(ADC4, &ADC_InitStructure);
    ADC_SelectClockMode(ADC4,ADC_CLOCK_MODE_AHB);
    /* Config ADC4  regular channels */
    ADC_ConfigRegularChannel(ADC4,ADC_CH_6,1,ADC_SAMP_TIME_CYCLES_1_5);
    /* Enable ADC4 */
    ADC_Enable(ADC4, ENABLE);
    /* Check ADC Ready */
    while(ADC_GetFlagStatus(ADC4,ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC4 calibration */
    ADC_CalibrationOperation(ADC4,ADC_CALIBRATION_SINGLE_MODE);
    /* Check the end of ADC4 calibration */
    while (ADC_GetCalibrationStatus(ADC4,ADC_CALIBRATION_SINGLE_MODE))
        ;

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the CORDIC
 *@param null
 *@return status
 */
bool CORDIC_Configuration(void)
{
    /* Enable CORDIC clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_CORDIC, ENABLE);

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the CRC
 *@param null
 *@return status
 */
bool CRC_Configuration(void)
{
    /* Enable CRC clock */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPHEN_CRC, ENABLE);
    CRC32_ResetCrc();
     
    //Use reference:
    // Use this function to get CRC value : uint32_t CRC32_CalcBufCrc(const uint32_t pBuffer[],  uint32_t BufferLength) 

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the FMAC
 *@param null
 *@return status
 */
bool FMAC_Configuration(void)
{

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the SHRTIM
 *@param null
 *@return status
 */
bool SHRTIM_Configuration(void)
{
    /* Initializes SHRTIM1 */
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_MASTER, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_MASTER, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_MASTER);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_MASTER);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_MASTER);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetUpdateTrig(SHRTIM1,SHRTIM_TIMER_MASTER,SHRTIM_UPDATETRIG_NONE);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_MASTER, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_MASTER);
     
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_A, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_A, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_COUNTING_MODE_UP_DOWN);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_A);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_A);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_A);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_A, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_A);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_A);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TA1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TA2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_A);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_B, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_B, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_COUNTING_MODE_UP_DOWN);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_B);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_B);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_B);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_B, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_B);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_B);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TB1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TB2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_B);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_C, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_C, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_COUNTING_MODE_UP_DOWN);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_C);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_C);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_C);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_C, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_C);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_C);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TC1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TC2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_C);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_D, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_D, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_COUNTING_MODE_UP_DOWN);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_D);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_D);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_D);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_D, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_D);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_D);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TD1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TD2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_D);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_E, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_E, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_COUNTING_MODE_UP);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_E);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_E);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_E);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_E, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_E);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_E);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TE1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TE2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_E);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
     
    SHRTIM_TIM_SetPrescaler(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_PRESCALERRATIO_MUL32);
    SHRTIM_TIM_SetPeriod(SHRTIM1, SHRTIM_TIMER_F, 96);
    SHRTIM_TIM_SetRepetition(SHRTIM1, SHRTIM_TIMER_F, 0);
    SHRTIM_TIM_SetCounterMode(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_MODE_CONTINUOUS);
    SHRTIM_TIM_SetCountingMode(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_COUNTING_MODE_UP);
    SHRTIM_TIM_SetInterleavedMode(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_INTERLEAVED_MODE_DISABLED);
    SHRTIM_TIM_DisableStartOnSync(SHRTIM1,SHRTIM_TIMER_F);
    SHRTIM_TIM_DisableResetOnSync(SHRTIM1,SHRTIM_TIMER_F);
    SHRTIM_TIM_SetDACTrig(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_DACTRIG_NONE);
    SHRTIM_TIM_DisablePreload(SHRTIM1,SHRTIM_TIMER_F);
    SHRTIM_TIM_SetUpdateGating(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_UPDATEGATING_INDEPENDENT);
    SHRTIM_TIM_SetBurstModeOption(SHRTIM1, SHRTIM_TIMER_F, SHRTIM_BURSTMODE_MAINTAINCLOCK);
    SHRTIM_TIM_DisablePushPullMode(SHRTIM1,SHRTIM_TIMER_F);
    SHRTIM_TIM_DisableResyncUpdate(SHRTIM1,SHRTIM_TIMER_F);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TF1,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    /* IDLEM1 : 'idle mode Enable' is abeyance-'空闲模式使能'暂时无效 */
    SHRTIM_OUT_Config(SHRTIM1,SHRTIM_OUTPUT_TF2,SHRTIM_OUT_POSITIVE_POLARITY | SHRTIM_OUT_IDLELEVEL_INACTIVE | SHRTIM_OUT_FAULTSTATE_INACTIVE | SHRTIM_OUT_CHOPPERMODE_DISABLED | SHRTIM_OUT_BM_ENTRYMODE_REGULAR);
    SHRTIM_ForceUpdate(SHRTIM1, SHRTIM_TIMER_F);
    /* Use its enable : SHRTIM_EnableOutput(SHRTIM_Module *SHRTIMx, uint32_t Outputs) */
    SHRTIM_SetADCTrigSrc(SHRTIM1,SHRTIM_ADCTRIG1_SOURCE_GROUP1,SHRTIM_ADTG13_SOURCE_GROUP1_MPRD);
    SHRTIM_SetADCTrigUpdate(SHRTIM1,SHRTIM_ADCTRIG_1,SHRTIM_ADCTRIG_UPDATE_TIMER_A);
    SHRTIM_SetADCPostScaler(SHRTIM1, SHRTIM_ADCTRIG_1, 0);
    /*Use its enable :  SHRTIM_TIM_CounterEnable(SHRTIM_Module *SHRTIMx, uint32_t Timers) */

/* NTFx CODE END */

    return true;
}
/* NTFx CODE START */
/**
 *@brief Initializes the TIM
 *@param null
 *@return status
 */
bool TIM_Configuration(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    /* GTIM10 configuration */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GTIM10, ENABLE);
    /* GTIM10 base configuration */
    TIM_InitTimBaseStruct(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Period = 59999;
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.ClkDiv = TIM_CLK_DIV4;
    TIM_TimeBaseStructure.CounterMode = TIM_CNT_MODE_UP;
    TIM_TimeBaseStructure.RepetCnt = 0;
    TIM_InitTimeBase(GTIM10, &TIM_TimeBaseStructure);
    TIM_ConfigArPreload(GTIM10,DISABLE);
    /* Events on the trigger input (TRGI) are deferred to allow perfect synchronization between the current timer (via TRGO) and its slave timers. */
    TIM_SelectMasterSlaveMode(GTIM10,TIM_MASTER_SLAVE_MODE_DISABLE);
    TIM_SelectOutputTrig(GTIM10,TIM_TRGO_SRC_RESET);
    TIM_SelectOutputTrig2(GTIM10,TIM_TRGO2_SRC_RESET);
    /* Clear GTIM10 interrupt flag*/
    TIM_ClearFlag(GTIM10,TIM_FLAG_UPDATE);
    /* Enable GTIM10 interrupt*/
    TIM_ConfigInt(GTIM10,TIM_INT_UPDATE,ENABLE);
     
    /* GTIM10 enable counter */
    TIM_Enable(GTIM10, ENABLE);

/* NTFx CODE END */

    return true;
}
