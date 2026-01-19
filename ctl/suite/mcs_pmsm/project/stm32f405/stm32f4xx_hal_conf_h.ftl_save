[#ftl]
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_hal_conf_template.h
  * @author  MCD Application Team
  * @brief   HAL configuration template file. 
  *          This file should be copied to the application folder and renamed
  *          to stm32f4xx_hal_conf.h.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */ 
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the HAL driver 
  */
#define HAL_MODULE_ENABLED  

  [#assign allModules = ["AES", "CRYP","ADC","CAN","CRC", "CAN_LEGACY","DAC","DCMI","DMA2D","ETH","ETH_LEGACY","NAND","NOR","PCCARD","SRAM","SDRAM","HASH","I2C","I2S","IWDG","LTDC","RNG","RTC","SAI","SD","MMC","SPI","TIM","UART","USART","IRDA","SMARTCARD","SMBUS","WWDG","PCD","HCD", "DSI","QSPI","QUADSPI","CEC","FMPI2C","FMPSMBUS","SPDIFRX", "DFSDM","LPTIM"]]
[#assign isUsed=false]
[#list allModules as module]
[#if isModuleUsed(module)]
[#if (module = "AES" || module = "CRYP") && !isUsed]
[#assign isUsed=true]
[#compress]#define HAL_${module?replace("QUADSPI","QSPI")?replace("AES","CRYP")}_MODULE_ENABLED[/#compress]
[#else]
[#compress]#define HAL_${module?replace("QUADSPI","QSPI")?replace("AES","CRYP")}_MODULE_ENABLED[/#compress]
[#assign isUsed=false]
[/#if]
[#else]
[#if isUsed && (module = "CRYP")]
[#assign isUsed=false]
[#continue]
[#else]
[#if module != "AES"]
[#compress]/* #define HAL_${module?replace("QUADSPI","QSPI")}_MODULE_ENABLED */[/#compress]
[#assign isUsed=true]
[/#if]
[/#if]
[/#if]
[/#list]
  [#function isModuleUsed moduleName]
	[#assign used=false]
	[#list modules as usedModule]
		[#if usedModule==moduleName]
			[#assign used=true]
			[#return true]
		[/#if]
	[/#list]
	[#return used]
  [/#function]
#define HAL_GPIO_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED

/* ########################## HSE/HSI Values adaptation ##################### */
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).  
  */
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    [#if hse_value??]${hse_value}[#else]25000000[/#if]U /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    [#if HSE_Timout??]${HSE_Timout}[#if HSE_Timout?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]5000U[/#if]   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL). 
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE) 
 #define LSI_VALUE  [#if lsi_value??]${lsi_value}[#else]32000[/#if]U       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/
/**
  * @brief External Low Speed oscillator (LSE) value.
  */
#if !defined  (LSE_VALUE)
 #define LSE_VALUE  [#if lse_value??]${lse_value}[#else]32768[/#if]U    /*!< Value of the External Low Speed oscillator in Hz */
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    [#if LSE_Timout??]${LSE_Timout}[#if LSE_Timout?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]5000U[/#if]   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/**
  * @brief External clock source for I2S peripheral
  *        This value is used by the I2S HAL module to compute the I2S clock source 
  *        frequency, this source is inserted directly through I2S_CKIN pad. 
  */
#if !defined  (EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    [#if external_clock_value??]${external_clock_value}[#else]12288000[/#if]U /*!< Value of the External audio frequency in Hz*/
#endif /* EXTERNAL_CLOCK_VALUE */

/* Tip: To avoid modifying this file each time you need to use different HSE,
   ===  you can define the HSE value in your toolchain compiler preprocessor. */

/* ########################### System Configuration ######################### */
/**
  * @brief This is the HAL system configuration section
  */
[#if advancedSettings??][#assign advancedSettings = advancedSettings[0]][/#if]     
#define  VDD_VALUE		      [#if vdd_value??]${vdd_value}[#if vdd_value?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]3300U[/#if] /*!< Value of VDD in mv */
#define  TICK_INT_PRIORITY            [#if TICK_INT_PRIORITY??]${TICK_INT_PRIORITY}[#if TICK_INT_PRIORITY?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0FU[/#if]   /*!< tick interrupt priority */            
#define  USE_RTOS                     [#if advancedSettings?? && advancedSettings.USE_RTOS??]${advancedSettings.USE_RTOS}[#else]0[/#if]U     
#define  PREFETCH_ENABLE              [#if PREFETCH_ENABLE??]${PREFETCH_ENABLE}[#else]1[/#if]U
#define  INSTRUCTION_CACHE_ENABLE     [#if INSTRUCTION_CACHE_ENABLE??]${INSTRUCTION_CACHE_ENABLE}[#else]1[/#if]U
#define  DATA_CACHE_ENABLE            [#if DATA_CACHE_ENABLE??]${DATA_CACHE_ENABLE}[#else]1[/#if]U

#define  USE_HAL_ADC_REGISTER_CALLBACKS         0U /* ADC register callback disabled       */
#define  USE_HAL_CAN_REGISTER_CALLBACKS         0U /* CAN register callback disabled       */
#define  USE_HAL_CEC_REGISTER_CALLBACKS         0U /* CEC register callback disabled       */
#define  USE_HAL_CRYP_REGISTER_CALLBACKS        0U /* CRYP register callback disabled      */
#define  USE_HAL_DAC_REGISTER_CALLBACKS         0U /* DAC register callback disabled       */
#define  USE_HAL_DCMI_REGISTER_CALLBACKS        0U /* DCMI register callback disabled      */
#define  USE_HAL_DFSDM_REGISTER_CALLBACKS       0U /* DFSDM register callback disabled     */
#define  USE_HAL_DMA2D_REGISTER_CALLBACKS       0U /* DMA2D register callback disabled     */
#define  USE_HAL_DSI_REGISTER_CALLBACKS         0U /* DSI register callback disabled       */
#define  USE_HAL_ETH_REGISTER_CALLBACKS         0U /* ETH register callback disabled       */
#define  USE_HAL_HASH_REGISTER_CALLBACKS        0U /* HASH register callback disabled      */
#define  USE_HAL_HCD_REGISTER_CALLBACKS         0U /* HCD register callback disabled       */
#define  USE_HAL_I2C_REGISTER_CALLBACKS         0U /* I2C register callback disabled       */
#define  USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U /* FMPI2C register callback disabled    */
#define  USE_HAL_FMPSMBUS_REGISTER_CALLBACKS    0U /* FMPSMBUS register callback disabled  */
#define  USE_HAL_I2S_REGISTER_CALLBACKS         0U /* I2S register callback disabled       */
#define  USE_HAL_IRDA_REGISTER_CALLBACKS        0U /* IRDA register callback disabled      */
#define  USE_HAL_LPTIM_REGISTER_CALLBACKS       0U /* LPTIM register callback disabled     */
#define  USE_HAL_LTDC_REGISTER_CALLBACKS        0U /* LTDC register callback disabled      */
#define  USE_HAL_MMC_REGISTER_CALLBACKS         0U /* MMC register callback disabled       */
#define  USE_HAL_NAND_REGISTER_CALLBACKS        0U /* NAND register callback disabled      */
#define  USE_HAL_NOR_REGISTER_CALLBACKS         0U /* NOR register callback disabled       */
#define  USE_HAL_PCCARD_REGISTER_CALLBACKS      0U /* PCCARD register callback disabled    */
#define  USE_HAL_PCD_REGISTER_CALLBACKS         0U /* PCD register callback disabled       */
#define  USE_HAL_QSPI_REGISTER_CALLBACKS        0U /* QSPI register callback disabled      */
#define  USE_HAL_RNG_REGISTER_CALLBACKS         0U /* RNG register callback disabled       */
#define  USE_HAL_RTC_REGISTER_CALLBACKS         0U /* RTC register callback disabled       */
#define  USE_HAL_SAI_REGISTER_CALLBACKS         0U /* SAI register callback disabled       */
#define  USE_HAL_SD_REGISTER_CALLBACKS          0U /* SD register callback disabled        */
#define  USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U /* SMARTCARD register callback disabled */
#define  USE_HAL_SDRAM_REGISTER_CALLBACKS       0U /* SDRAM register callback disabled     */
#define  USE_HAL_SRAM_REGISTER_CALLBACKS        0U /* SRAM register callback disabled      */
#define  USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U /* SPDIFRX register callback disabled   */
#define  USE_HAL_SMBUS_REGISTER_CALLBACKS       0U /* SMBUS register callback disabled     */
#define  USE_HAL_SPI_REGISTER_CALLBACKS         0U /* SPI register callback disabled       */
#define  USE_HAL_TIM_REGISTER_CALLBACKS         0U /* TIM register callback disabled       */
#define  USE_HAL_UART_REGISTER_CALLBACKS        0U /* UART register callback disabled      */
#define  USE_HAL_USART_REGISTER_CALLBACKS       0U /* USART register callback disabled     */
#define  USE_HAL_WWDG_REGISTER_CALLBACKS        0U /* WWDG register callback disabled      */

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
[#if !fullAssert??]/*[/#if] #define USE_FULL_ASSERT    1U [#if !fullAssert??]*/[/#if]

/* ################## Ethernet peripheral configuration ##################### */

/* Section 1 : Ethernet peripheral configuration */

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   2U
#define MAC_ADDR1   0U
#define MAC_ADDR2   0U
#define MAC_ADDR3   0U
#define MAC_ADDR4   0U
#define MAC_ADDR5   0U

/* Definition of the Ethernet driver buffers size and count */   
#define ETH_RX_BUF_SIZE                [#if RxBuffLen??]${RxBuffLen}[#else]ETH_MAX_PACKET_SIZE[/#if] /* buffer size for receive               */
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE /* buffer size for transmit              */
#define ETH_RXBUFNB                    [#if ETH_RXBUFNB??]${ETH_RXBUFNB}[#if ETH_RXBUFNB?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]4U[/#if]       /* 4 Rx buffers of size ETH_RX_BUF_SIZE  */
#define ETH_TXBUFNB                    [#if ETH_TXBUFNB??]${ETH_TXBUFNB}[#if ETH_TXBUFNB?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]4U[/#if]       /* 4 Tx buffers of size ETH_TX_BUF_SIZE  */

/* Section 2: PHY configuration section */

/* [#if BspComponentName??][#if BspComponentName=="DP83848"]${BspComponentName}[#else][#if BspComponentName=="LAN8742"]${BspComponentName}A[/#if][/#if] PHY[#else]DP83848_PHY_ADDRESS[/#if] Address*/ 
#define [#if BspComponentName??][#if BspComponentName=="DP83848"]${BspComponentName}[#else][#if BspComponentName=="LAN8742"]${BspComponentName}A[/#if][/#if]_PHY_ADDRESS[#else]DP83848_PHY_ADDRESS[/#if]           [#if BspComponentName??][#if BspComponentName=="DP83848"]0x01U[#else][#if BspComponentName=="LAN8742"]0x00U[/#if][/#if][/#if]
/* PHY Reset delay these values are based on a 1 ms Systick interrupt*/ 
#define PHY_RESET_DELAY                 [#if PHY_RESET_DELAY??]${PHY_RESET_DELAY}[#else]0x000000FF[/#if]U
/* PHY Configuration delay */
#define PHY_CONFIG_DELAY                [#if PHY_CONFIG_DELAY??]${PHY_CONFIG_DELAY}[#else]0x00000FFF[/#if]U

#define PHY_READ_TO                     [#if PHY_READ_TO??]${PHY_READ_TO}[#if PHY_READ_TO?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000FFFFU[/#if]
#define PHY_WRITE_TO                    [#if PHY_WRITE_TO??]${PHY_WRITE_TO}[#if PHY_WRITE_TO?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000FFFFU[/#if]

/* Section 3: Common PHY Registers */

#define PHY_BCR                         ((uint16_t)[#if PHY_BCR??]${PHY_BCR}[#if PHY_BCR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000U[/#if])    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)[#if PHY_BSR??]${PHY_BSR}[#if PHY_BSR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])    /*!< Transceiver Basic Status Register    */
 
#define PHY_RESET                       ((uint16_t)[#if PHY_RESET??]${PHY_RESET}[#if PHY_RESET?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x8000U[/#if])  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)[#if PHY_LOOPBACK??]${PHY_LOOPBACK}[#if PHY_LOOPBACK?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x4000U[/#if])  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)[#if PHY_FULLDUPLEX_100M??]${PHY_FULLDUPLEX_100M}[#if PHY_FULLDUPLEX_100M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2100U[/#if])  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)[#if PHY_HALFDUPLEX_100M??]${PHY_HALFDUPLEX_100M}[#if PHY_HALFDUPLEX_100M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2000U[/#if])  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)[#if PHY_FULLDUPLEX_10M??]${PHY_FULLDUPLEX_10M}[#if PHY_FULLDUPLEX_10M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0100U[/#if])  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)[#if PHY_HALFDUPLEX_10M??]${PHY_HALFDUPLEX_10M}[#if PHY_HALFDUPLEX_10M?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0000U[/#if])  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)[#if PHY_AUTONEGOTIATION??]${PHY_AUTONEGOTIATION}[#if PHY_AUTONEGOTIATION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x1000U[/#if])  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)[#if PHY_RESTART_AUTONEGOTIATION??]${PHY_RESTART_AUTONEGOTIATION}[#if PHY_RESTART_AUTONEGOTIATION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0200U[/#if])  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)[#if PHY_POWERDOWN??]${PHY_POWERDOWN}[#if PHY_POWERDOWN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0800U[/#if])  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)[#if PHY_ISOLATE??]${PHY_ISOLATE}[#if PHY_ISOLATE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0400U[/#if])  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)[#if PHY_AUTONEGO_COMPLETE??]${PHY_AUTONEGO_COMPLETE}[#if PHY_AUTONEGO_COMPLETE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0020U[/#if])  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)[#if PHY_LINKED_STATUS??]${PHY_LINKED_STATUS}[#if PHY_LINKED_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0004U[/#if])  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)[#if PHY_JABBER_DETECTION??]${PHY_JABBER_DETECTION}[#if PHY_JABBER_DETECTION?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0002U[/#if])  /*!< Jabber condition detected            */
  
/* Section 4: Extended PHY Registers */
[#--Common define--]
#define PHY_SR                          ((uint16_t)[#if BspComponentName??][#if BspComponentName=="DP83848"]0x10U[#else][#if BspComponentName=="LAN8742"]0x001FU[/#if][/#if][/#if])    /*!< PHY status register Offset                      */
[#if BspComponentName?? && BspComponentName=="DP83848"]
#define PHY_MICR                        ((uint16_t)[#if PHY_MICR??]${PHY_MICR}[#if PHY_MICR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x11U[/#if])    /*!< MII Interrupt Control Register                  */
#define PHY_MISR                        ((uint16_t)[#if PHY_MISR??]${PHY_MISR}[#if PHY_MISR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x12U[/#if])    /*!< MII Interrupt Status and Misc. Control Register */
#n#define PHY_LINK_STATUS                 ((uint16_t)[#if PHY_LINK_STATUS??]${PHY_LINK_STATUS}[#if PHY_LINK_STATUS?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])  /*!< PHY Link mask                                   */
[#else]
#n
[/#if]
[#--Common define--]
#define PHY_SPEED_STATUS                ((uint16_t)[#if BspComponentName??][#if BspComponentName=="DP83848"]0x0002U[#else][#if BspComponentName=="LAN8742"]0x0004U[/#if][/#if][/#if])  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)[#if BspComponentName??][#if BspComponentName=="DP83848"]0x0004U[#else][#if BspComponentName=="LAN8742"]0x0010U[/#if][/#if][/#if])  /*!< PHY Duplex mask                                 */
[#if BspComponentName?? && BspComponentName=="DP83848"]
#n#define PHY_MICR_INT_EN                 ((uint16_t)[#if PHY_MICR_INT_EN??]${PHY_MICR_INT_EN}[#if PHY_MICR_INT_EN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0002U[/#if])  /*!< PHY Enable interrupts                           */
#define PHY_MICR_INT_OE                 ((uint16_t)[#if PHY_MICR_INT_OE??]${PHY_MICR_INT_OE}[#if PHY_MICR_INT_OE?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0001U[/#if])  /*!< PHY Enable output interrupt events              */
#n#define PHY_MISR_LINK_INT_EN            ((uint16_t)[#if PHY_MISR_LINK_INT_EN??]${PHY_MISR_LINK_INT_EN}[#if PHY_MISR_LINK_INT_EN?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0020U[/#if])  /*!< Enable Interrupt on change of link status       */
#define PHY_LINK_INTERRUPT              ((uint16_t)[#if PHY_LINK_INTERRUPT??]${PHY_LINK_INTERRUPT}[#if PHY_LINK_INTERRUPT?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x2000U[/#if])  /*!< PHY link status interrupt mask                  */
[/#if]
[#if BspComponentName?? && BspComponentName=="LAN8742"]
#n#define PHY_ISFR                        ((uint16_t)[#if PHY_ISFR??]${PHY_ISFR}[#if PHY_ISFR?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x1DU[/#if])    /*!< PHY Interrupt Source Flag register Offset   */
#define PHY_ISFR_INT4                   ((uint16_t)[#if PHY_ISFR_INT4??]${PHY_ISFR_INT4}[#if PHY_ISFR_INT4?matches("(0x[0-9]*[a-f]*[A-F]*)|([0-9]*)")]U[/#if][#else]0x0010U[/#if])  /*!< PHY Link down inturrupt       */  
[/#if]

/* ################## SPI peripheral configuration ########################## */

/* CRC FEATURE: Use to activate CRC feature inside HAL SPI Driver
* Activated: CRC code is present inside driver
* Deactivated: CRC code cleaned from driver
*/

#define USE_SPI_CRC                     [#if CRC_SPI??]${CRC_SPI}[#else]1U[/#if]

/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif /* HAL_RCC_MODULE_ENABLED */

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif /* HAL_GPIO_MODULE_ENABLED */

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif /* HAL_EXTI_MODULE_ENABLED */

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif /* HAL_DMA_MODULE_ENABLED */
   
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif /* HAL_CORTEX_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif /* HAL_ADC_MODULE_ENABLED */

#ifdef HAL_CAN_MODULE_ENABLED
  #include "stm32f4xx_hal_can.h"
#endif /* HAL_CAN_MODULE_ENABLED */

#ifdef HAL_CAN_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_can_legacy.h"
#endif /* HAL_CAN_LEGACY_MODULE_ENABLED */

#ifdef HAL_CRC_MODULE_ENABLED
  #include "stm32f4xx_hal_crc.h"
#endif /* HAL_CRC_MODULE_ENABLED */

#ifdef HAL_CRYP_MODULE_ENABLED
  #include "stm32f4xx_hal_cryp.h" 
#endif /* HAL_CRYP_MODULE_ENABLED */

#ifdef HAL_DMA2D_MODULE_ENABLED
  #include "stm32f4xx_hal_dma2d.h"
#endif /* HAL_DMA2D_MODULE_ENABLED */

#ifdef HAL_DAC_MODULE_ENABLED
  #include "stm32f4xx_hal_dac.h"
#endif /* HAL_DAC_MODULE_ENABLED */

#ifdef HAL_DCMI_MODULE_ENABLED
  #include "stm32f4xx_hal_dcmi.h"
#endif /* HAL_DCMI_MODULE_ENABLED */

#ifdef HAL_ETH_MODULE_ENABLED
  #include "stm32f4xx_hal_eth.h"
#endif /* HAL_ETH_MODULE_ENABLED */

#ifdef HAL_ETH_LEGACY_MODULE_ENABLED
  #include "stm32f4xx_hal_eth_legacy.h"
#endif /* HAL_ETH_LEGACY_MODULE_ENABLED */

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif /* HAL_FLASH_MODULE_ENABLED */
 
#ifdef HAL_SRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sram.h"
#endif /* HAL_SRAM_MODULE_ENABLED */

#ifdef HAL_NOR_MODULE_ENABLED
  #include "stm32f4xx_hal_nor.h"
#endif /* HAL_NOR_MODULE_ENABLED */

#ifdef HAL_NAND_MODULE_ENABLED
  #include "stm32f4xx_hal_nand.h"
#endif /* HAL_NAND_MODULE_ENABLED */

#ifdef HAL_PCCARD_MODULE_ENABLED
  #include "stm32f4xx_hal_pccard.h"
#endif /* HAL_PCCARD_MODULE_ENABLED */ 
  
#ifdef HAL_SDRAM_MODULE_ENABLED
  #include "stm32f4xx_hal_sdram.h"
#endif /* HAL_SDRAM_MODULE_ENABLED */

#ifdef HAL_HASH_MODULE_ENABLED
 #include "stm32f4xx_hal_hash.h"
#endif /* HAL_HASH_MODULE_ENABLED */

#ifdef HAL_I2C_MODULE_ENABLED
 #include "stm32f4xx_hal_i2c.h"
#endif /* HAL_I2C_MODULE_ENABLED */

#ifdef HAL_SMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_smbus.h"
#endif /* HAL_SMBUS_MODULE_ENABLED */

#ifdef HAL_I2S_MODULE_ENABLED
 #include "stm32f4xx_hal_i2s.h"
#endif /* HAL_I2S_MODULE_ENABLED */

#ifdef HAL_IWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_iwdg.h"
#endif /* HAL_IWDG_MODULE_ENABLED */

#ifdef HAL_LTDC_MODULE_ENABLED
 #include "stm32f4xx_hal_ltdc.h"
#endif /* HAL_LTDC_MODULE_ENABLED */

#ifdef HAL_PWR_MODULE_ENABLED
 #include "stm32f4xx_hal_pwr.h"
#endif /* HAL_PWR_MODULE_ENABLED */

#ifdef HAL_RNG_MODULE_ENABLED
 #include "stm32f4xx_hal_rng.h"
#endif /* HAL_RNG_MODULE_ENABLED */

#ifdef HAL_RTC_MODULE_ENABLED
 #include "stm32f4xx_hal_rtc.h"
#endif /* HAL_RTC_MODULE_ENABLED */

#ifdef HAL_SAI_MODULE_ENABLED
 #include "stm32f4xx_hal_sai.h"
#endif /* HAL_SAI_MODULE_ENABLED */

#ifdef HAL_SD_MODULE_ENABLED
 #include "stm32f4xx_hal_sd.h"
#endif /* HAL_SD_MODULE_ENABLED */

#ifdef HAL_SPI_MODULE_ENABLED
 #include "stm32f4xx_hal_spi.h"
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_TIM_MODULE_ENABLED
 #include "stm32f4xx_hal_tim.h"
#endif /* HAL_TIM_MODULE_ENABLED */

#ifdef HAL_UART_MODULE_ENABLED
 #include "stm32f4xx_hal_uart.h"
#endif /* HAL_UART_MODULE_ENABLED */

#ifdef HAL_USART_MODULE_ENABLED
 #include "stm32f4xx_hal_usart.h"
#endif /* HAL_USART_MODULE_ENABLED */

#ifdef HAL_IRDA_MODULE_ENABLED
 #include "stm32f4xx_hal_irda.h"
#endif /* HAL_IRDA_MODULE_ENABLED */

#ifdef HAL_SMARTCARD_MODULE_ENABLED
 #include "stm32f4xx_hal_smartcard.h"
#endif /* HAL_SMARTCARD_MODULE_ENABLED */

#ifdef HAL_WWDG_MODULE_ENABLED
 #include "stm32f4xx_hal_wwdg.h"
#endif /* HAL_WWDG_MODULE_ENABLED */

#ifdef HAL_PCD_MODULE_ENABLED
 #include "stm32f4xx_hal_pcd.h"
#endif /* HAL_PCD_MODULE_ENABLED */

#ifdef HAL_HCD_MODULE_ENABLED
 #include "stm32f4xx_hal_hcd.h"
#endif /* HAL_HCD_MODULE_ENABLED */
   
#ifdef HAL_DSI_MODULE_ENABLED
 #include "stm32f4xx_hal_dsi.h"
#endif /* HAL_DSI_MODULE_ENABLED */

#ifdef HAL_QSPI_MODULE_ENABLED
 #include "stm32f4xx_hal_qspi.h"
#endif /* HAL_QSPI_MODULE_ENABLED */

#ifdef HAL_CEC_MODULE_ENABLED
 #include "stm32f4xx_hal_cec.h"
#endif /* HAL_CEC_MODULE_ENABLED */

#ifdef HAL_FMPI2C_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpi2c.h"
#endif /* HAL_FMPI2C_MODULE_ENABLED */

#ifdef HAL_FMPSMBUS_MODULE_ENABLED
 #include "stm32f4xx_hal_fmpsmbus.h"
#endif /* HAL_FMPSMBUS_MODULE_ENABLED */

#ifdef HAL_SPDIFRX_MODULE_ENABLED
 #include "stm32f4xx_hal_spdifrx.h"
#endif /* HAL_SPDIFRX_MODULE_ENABLED */

#ifdef HAL_DFSDM_MODULE_ENABLED
 #include "stm32f4xx_hal_dfsdm.h"
#endif /* HAL_DFSDM_MODULE_ENABLED */

#ifdef HAL_LPTIM_MODULE_ENABLED
 #include "stm32f4xx_hal_lptim.h"
#endif /* HAL_LPTIM_MODULE_ENABLED */

#ifdef HAL_MMC_MODULE_ENABLED
 #include "stm32f4xx_hal_mmc.h"
#endif /* HAL_MMC_MODULE_ENABLED */

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
 