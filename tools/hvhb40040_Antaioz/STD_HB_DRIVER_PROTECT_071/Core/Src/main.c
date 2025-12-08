/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eeprom_at24c04.h"
#include <string.h> // 用于 memcmp
#include <stdio.h>  // printf is valid
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// NTC Beta parameter
#define NTC_BETA            3950.0f
// rated temperature
#define NTC_T_NOMINAL_K     (25.0f + 273.15f)
// rated resistance at rated temperature
#define NTC_R_NOMINAL       10000.0f

// divider r fixed
#define DIVIDER_R_FIXED     10000.0f

// ADC MAX value
#define ADC_MAX             4095.0f

// K to C bias
#define KELVIN_TO_CELSIUS   273.15f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// fan rpm calculation
volatile uint32_t uwFanFrequency = 0;
volatile uint32_t uwFanRPM = 0;

// Capture Result
volatile uint32_t g_pulse_width = 0; // pulse time
volatile uint32_t g_period = 0;      // period

// ready flag for pulse
volatile uint8_t g_data_ready_flag = 0;

// conter frequency, Hz
const uint32_t TIM3_COUNTER_CLK_HZ = 100000;

// fan speed feedback
float fan_spd_fbk_krpm;

// fan speed set
float fan_spd_set_pu = 0.5f;

// current temperature in C
float current_temp_C;

// ADC buffer
typedef enum _enum_ADC_channels
{
  ADC_CH_PSUP_5V = 0,
  ADC_CH_PSUP_12V = 1,
  ADC_CH_CURRENT = 2,
  ADC_CH_VOLTAGE = 3,
  ADC_CH_TEMP = 4
} adc_channel_enum;

volatile uint16_t adc_dma_buffer[5];

// Error Registers
// 1 = error occurred, 0 = no error.
uint32_t flag_error_over_cuurent;
uint32_t flag_error_over_voltage;
uint32_t flag_error_psup_not_ready;
uint32_t flag_error_fan_not_ready;

// EEPROM
volatile HAL_StatusTypeDef write_status;
volatile HAL_StatusTypeDef read_status;

// tick count
volatile uint32_t loop_tick;
volatile uint32_t isr_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_CRC_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void update_fault_state(void);
float update_fan_speed(void);
float convert_adc_to_temperature(uint16_t adc_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // Init error flags
  flag_error_over_cuurent = 1;
  flag_error_over_voltage = 1;
  flag_error_psup_not_ready = 1;
  flag_error_fan_not_ready = 1;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

#if 0
// --- EEPROM Verify Start ---

	// EEPROM Test buffer
#define TEST_BUFFER_SIZE 32
	uint8_t write_buffer[TEST_BUFFER_SIZE];
	uint8_t read_buffer[TEST_BUFFER_SIZE];

// fill buffer 0, 1, 2, ... 31
for(int i = 0; i < TEST_BUFFER_SIZE; i++)
    write_buffer[i] = i; 


// 2. 选择一个跨边界的地址
// 我们从地址 0x0F0 (240) 开始写入 32 字节
// 这将测试:
// - 写入 0x0F0 - 0x0FF (240-255) -> 跨 16 字节页
// - 写入 0x100 - 0x10F (256-271) -> 跨 256 字节设备地址
uint16_t test_address = 0x0F0;

// Write EEPROM
//printf("Writing 32 bytes to address 0x%03X...\r\n", test_address);
write_status = EEPROM_Write_Buffer(test_address, write_buffer, TEST_BUFFER_SIZE);

if (write_status != HAL_OK)
{
//    printf("Write FAILED! Status: %d\r\n", write_status);
    while(1); // Stop here if meets error
}

// clear buffer
memset(read_buffer, 0, TEST_BUFFER_SIZE);

// read back EEPROM
//printf("Reading 32 bytes from address 0x%03X...\r\n", test_address);
read_status = EEPROM_Read_Buffer(test_address, read_buffer, TEST_BUFFER_SIZE);

if (read_status != HAL_OK)
{
    //printf("Read FAILED! Status: %d\r\n", read_status);
    while(1); // Stop here if meets error
}

// Compare data
//printf("Comparing data...\r\n");
if (memcmp(write_buffer, read_buffer, TEST_BUFFER_SIZE) == 0)
{
//    printf("--- TEST PASSED! ---\r\n");
//    printf("Data written matches data read.\r\n");
}
else
{
//    printf("--- TEST FAILED! ---\r\n");
//    printf("Data mismatch detected.\r\n");
    for(int i = 0; i < TEST_BUFFER_SIZE; i++)
    {
        if(write_buffer[i] != read_buffer[i])
        {
//            printf("Mismatch at index %d: Wrote %d, Read %d\r\n", i, write_buffer[i], read_buffer[i]);
        }
    }
}

printf("EEPROM Test Passed.\r\n");
// --- EEPROM Verify Done ---

#endif // EEPROM Verify

  // Calibrate ADC
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_Delay(1000);

  // Start TIM3, Start FAN Capture
  HAL_TIM_Base_Start(&htim3);

  // rising edge capture
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
	
	// falling edge capture
	if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
	
	// Start TIM 6, ADC trigger
	HAL_TIM_Base_Start(&htim6);
	
	// Start Conversion
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, 5);
	
	// Start TIM 1, for FAN PWM
	HAL_TIM_Base_Start(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    // update loop tick
    loop_tick++;

    // update fault state
    update_fault_state();

    // LED blink
		if(isr_tick / 10000 % 2 == 1)
			HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, 0);
		else
			HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, 1);

    // Temperature
    current_temp_C = convert_adc_to_temperature(adc_dma_buffer[4]);
		
		if(current_temp_C > 60)
			flag_error_fan_not_ready = 1;
		else
			flag_error_fan_not_ready = 0;

	  // calculate fan speed
		fan_spd_fbk_krpm = update_fan_speed();
		
		// PSUP judge
		// PSUP_Monitor_12V = 0x83E
    // PSUP_Monitor_5V  = 0x830
		
    // feed watchdog here
		
		
		// for now no error will be detected
		  flag_error_over_cuurent = 0;
      flag_error_over_voltage = 0;
      flag_error_psup_not_ready = 0;
      flag_error_fan_not_ready = 0;

		


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B17DB5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 6400;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 3000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 639;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 6399;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_LED_GPIO_Port, SYS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_FAN_NOT_READY_GPIO_Port, GPIO_OUT_FAN_NOT_READY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_PSUP_NOT_READY_GPIO_Port, GPIO_OUT_PSUP_NOT_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_OVER_VOLTAGE_GPIO_Port, GPIO_OUT_OVER_VOLTAGE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_OUT_OVER_CURRENT_GPIO_Port, GPIO_OUT_OVER_CURRENT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SYS_LED_Pin */
  GPIO_InitStruct.Pin = SYS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SYS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT_FAN_NOT_READY_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_FAN_NOT_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_OUT_FAN_NOT_READY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT_PSUP_NOT_READY_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_PSUP_NOT_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIO_OUT_PSUP_NOT_READY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT_OVER_VOLTAGE_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_OVER_VOLTAGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_OUT_OVER_VOLTAGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_OUT_OVER_CURRENT_Pin */
  GPIO_InitStruct.Pin = GPIO_OUT_OVER_CURRENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIO_OUT_OVER_CURRENT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#if 0
/**
  * @brief  重定向 C 库的 _write 函数到 UART1
  * @param  file: 文件描述符, 1 表示 stdout
  * @param  ptr:  要发送的数据缓冲区指针
  * @param  len:  数据长度
  * @retval int:  成功发送的字节数
  */
int _write(int file, char *ptr, int len)
{
  // 检查文件描述符是否为 stdout (标准输出)
  if (file == 1)
  {
    // 使用 HAL 库将数据通过 UART1 发送出去
    // huart1 是 CubeMX 自动生成的 UART1 句柄
    // (uint8_t*)ptr 是数据
    // len 是长度
    // HAL_MAX_DELAY 表示使用阻塞模式发送，直到发送完成或超时
    if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY) == HAL_OK)
    {
      return len; // 返回成功发送的字节数
    }
  }

  // 对于其他文件描述符 (如 stderr) 或发送失败，返回 -1
  return -1;
}

#endif // _write

/**
 * @brief  重定向 C 库的 fputc 函数到 UART1 (Keil MDK-ARM)
 * @param  ch:   要发送的字符
 * @param  f:    文件指针 (未使用)
 * @retval int:  返回发送的字符
 */
int fputc(int ch, FILE *f)
{
  // 使用 HAL 库将单个字符 (ch) 通过 UART1 发送出去
  // huart1 是 CubeMX 自动生成的 UART1 句柄
  // (uint8_t *)&ch 是将 int 类型的 ch 转换为 uint8_t 指针
  // 1 是长度
  // HAL_MAX_DELAY 表示使用阻塞模式发送
  if (HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY) == HAL_OK)
  {
    return ch; // 发送成功，返回该字符
  }
  else
  {
    return -1; // 发送失败
  }
}

/**
 * @brief  ���벶���жϻص�����
 * @param  htim: ��ʱ�����
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  // TIM3 FAN TECH capture result
  if (htim->Instance == TIM3)
  {
   
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
			// Rising edge capture result
      g_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      // falling edge capture result
      g_pulse_width = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    }
		
  }
}

// ADC result
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    isr_tick++;


  }
}

void update_fault_state(void)
{
  if (!flag_error_over_cuurent)
    HAL_GPIO_WritePin(GPIO_OUT_OVER_CURRENT_GPIO_Port, GPIO_OUT_OVER_CURRENT_Pin, 0);
  else
    HAL_GPIO_WritePin(GPIO_OUT_OVER_CURRENT_GPIO_Port, GPIO_OUT_OVER_CURRENT_Pin, 1);

  if (!flag_error_over_voltage)
    HAL_GPIO_WritePin(GPIO_OUT_FAN_NOT_READY_GPIO_Port, GPIO_OUT_FAN_NOT_READY_Pin, 0);
  else
    HAL_GPIO_WritePin(GPIO_OUT_FAN_NOT_READY_GPIO_Port, GPIO_OUT_FAN_NOT_READY_Pin, 1);

  if (!flag_error_psup_not_ready)
    HAL_GPIO_WritePin(GPIO_OUT_PSUP_NOT_READY_GPIO_Port, GPIO_OUT_PSUP_NOT_READY_Pin, 0);
  else
    HAL_GPIO_WritePin(GPIO_OUT_PSUP_NOT_READY_GPIO_Port, GPIO_OUT_PSUP_NOT_READY_Pin, 1);

  if (!flag_error_fan_not_ready)
    HAL_GPIO_WritePin(GPIO_OUT_OVER_VOLTAGE_GPIO_Port, GPIO_OUT_OVER_VOLTAGE_Pin, 0);
  else
    HAL_GPIO_WritePin(GPIO_OUT_OVER_VOLTAGE_GPIO_Port, GPIO_OUT_OVER_VOLTAGE_Pin, 1);
}

float update_fan_speed(void)
{
	// BUG REPORT
	// when fan is stop the value will keep the last number which is a wrong number.
	
	// protect fan spd target set
	if(fan_spd_set_pu > 1.0f)
		fan_spd_set_pu = 1.0f;
	if(fan_spd_set_pu < 0.0f)
		fan_spd_set_pu = 0.0f;
	
	// update fan spd set
	uint32_t fan_target = 6400 * fan_spd_set_pu;
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, fan_target);

	return 3000.0f / g_period;
}

/**
 * @brief convert ADC result to C
 * @param adc_value 12-bit source value (0-4095)
 * @return temperature in C
 */
float convert_adc_to_temperature(uint16_t adc_value)
{
    float r_ntc;
    float temperature_k, temperature_c;

    // --- 1. 处理 ADC 边缘情况 ---

    // ADC = 0, R_ntc 趋近于 0 (短路), log(0) 无意义
    // 返回一个极高温度值表示错误
    if (adc_value < 1) 
    {
        return 999.0f; // 表示短路
    }

    // ADC = 4095, R_ntc 趋近于无穷大 (开路)
    // (ADC_MAX - adc_value) 会导致除以零
    // 返回一个极低温度值表示错误
    if (adc_value >= (uint16_t)ADC_MAX) 
    {
        return -273.15f; // 表示开路
    }

    // --- 2. ADC 读数 -> NTC 电阻 ---
    // R_ntc = R_fixed * (adc_value / (4095 - adc_value))
    r_ntc = DIVIDER_R_FIXED * ( (float)adc_value / (ADC_MAX - (float)adc_value) );

    // --- 3. NTC 电阻 -> 温度 ---
    
    // 计算 1/T = 1/T0 + (1/B) * ln(R_ntc / R0)
    float inv_t = (1.0f / NTC_T_NOMINAL_K) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_R_NOMINAL);

    // T (开尔文) = 1 / (1/T)
    temperature_k = 1.0f / inv_t;

    // 转换为摄氏度
    temperature_c = temperature_k - KELVIN_TO_CELSIUS;

    return temperature_c;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
