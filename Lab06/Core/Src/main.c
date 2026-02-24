/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PPR 20
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* ============================================================
   TASK 4: Input Capture Variables
   ============================================================ */
volatile uint32_t ic_val1       = 0;   // First captured timer value
volatile uint32_t ic_val2       = 0;   // Second captured timer value
volatile uint8_t  ic_edge_count = 0;   // Counts how many edges have been captured
volatile uint8_t  ic_ready      = 0;   // Flag: 1 = two edges captured, RPM ready to calc

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
   TASK 3: Measure Motor Speed Using Encoder and POLLING
   ------------------------------------------------------------
   - Uses PA4 as encoder input (GPIO polling)
   - Uses TIM3 as microsecond timer (prescaler=47 @ 48MHz → 1 tick = 1us)
   - Detects two consecutive falling edges by polling
   - Calculates frequency and RPM
   - Prints RPM over UART2
   NOTE: THIS TASK IS COMMENTED OUT. SEE TASK 4 BELOW FOR ACTIVE CODE.
   ============================================================ */

/*
// --- TASK 3: Print RPM via UART ---
void print_RPM(float rpm)
{
    char buffer[50];
    int rpm_int = (int)rpm;
    int len = sprintf(buffer, "Right Motor RPM: %d\r\n", rpm_int);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

// --- TASK 3: Read RPM using polling on PA4 ---
float read_RPM(void)
{
    uint32_t ticks;
    float frequency, rpm;

    // Step 1: Wait for pin to go HIGH (known state)
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);

    // Step 2: Wait for FIRST falling edge (HIGH -> LOW)
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET);

    // Step 3: Reset and start timer at first falling edge
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    // Step 4: Wait for pin to go HIGH again
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);

    // Step 5: Wait for SECOND falling edge (HIGH -> LOW)
    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET);

    // Step 6: Read ticks (1 tick = 1 us)
    ticks = __HAL_TIM_GET_COUNTER(&htim3);
    if(ticks == 0) ticks = 1; // prevent division by zero

    // Frequency (Hz) = 1,000,000 / ticks  (since 1 tick = 1us)
    frequency = 1000000.0f / (float)ticks;

    // RPM = (60 x frequency) / PPR
    rpm = (60.0f * frequency) / PPR;

    return rpm;
}
*/

/* ============================================================
   TASK 4: Measure Motor Speed Using Encoder and INPUT CAPTURE
   ------------------------------------------------------------
   - TIM2 Channel 1 configured for Input Capture on PA0
     (connect encoder signal D4 to PA0 instead of PA4)
   - TIM2 prescaler set so 1 tick = 1us (prescaler = 47 @ 48MHz)
   - Rising edge interrupt fires on each pulse
   - ISR records two consecutive edge timestamps
   - Main loop calculates RPM from the difference
   ============================================================ */

// --- TASK 4: Input Capture interrupt callback ---
// This function is called automatically by HAL every time TIM2 CH1 captures a rising edge
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        if (ic_edge_count == 0)
        {
            // First rising edge: record timestamp
            ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ic_edge_count = 1;
        }
        else if (ic_edge_count == 1)
        {
            // Second rising edge: record timestamp
            ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            ic_edge_count = 0;  // Reset for next measurement
            ic_ready = 1;       // Signal main loop that data is ready
        }
    }
}

// --- TASK 4: Calculate and print RPM from captured values ---
void calculate_and_print_RPM(void)
{
    uint32_t ticks;
    float frequency;
    int rpm_int;
    char buffer[50];
    int len;

    // Handle timer overflow (if second capture < first capture)
    if (ic_val2 > ic_val1)
    {
        ticks = ic_val2 - ic_val1;
    }
    else
    {
        // Timer has overflowed (period = 65535 for 16-bit TIM2)
        ticks = (65535 - ic_val1) + ic_val2 + 1;
    }

    if (ticks == 0) ticks = 1; // prevent division by zero

    // Frequency (Hz) = 1,000,000 / ticks  (1 tick = 1us)
    frequency = 1000000.0f / (float)ticks;

    // RPM = (60 x frequency) / PPR
    rpm_int = (int)((60.0f * frequency) / PPR);

    len = sprintf(buffer, "Right Motor RPM: %d\r\n", rpm_int);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */

  /* --- Motor Direction Setup --- */
  // Right motor direction (PA5 = forward, PA6 = reverse off)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  // Left motor direction (PC4 = forward, PC5 = reverse off)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /* --- TASK 3: Start TIM3 for microsecond polling timer (KEPT but unused now) --- */
  HAL_TIM_Base_Start(&htim3);

  /* --- TASK 4: Start TIM2 Channel 1 Input Capture with Interrupt --- */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* --- Motor PWM Start --- */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Left motor PWM on CH2
  // NOTE: CH1 is now used for Input Capture (encoder), not PWM for right motor
  // Right motor encoder signal must come into PA0 (TIM2 CH1)

  /* --- Set PWM Duty Cycle (50%) --- */
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2147483647); // 50% of max period

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* ============================================================
         TASK 3: POLLING METHOD (COMMENTED OUT)
         ============================================================ */
      /*
      float rpm = read_RPM();
      print_RPM(rpm);
      HAL_Delay(500);
      */

      /* ============================================================
         TASK 4: INPUT CAPTURE METHOD (ACTIVE)
         - ic_ready flag is set inside the ISR when two edges are captured
         - Main loop just waits for the flag then calculates RPM
         ============================================================ */
      if (ic_ready == 1)
      {
          ic_ready = 0;                   // Clear flag
          calculate_and_print_RPM();      // Calculate and print RPM
          HAL_Delay(500);                 // Wait before next print
      }
  }
  /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * -------------------------------------------------------
  * TASK 4 CHANGE: TIM2 Channel 1 is now configured for
  * INPUT CAPTURE (rising edge, interrupt enabled) on PA0.
  * TIM2 Channel 2 remains as PWM output.
  * Prescaler = 47 → 1 tick = 1us at 48MHz clock.
  * -------------------------------------------------------
  * YOU MUST RECONFIGURE TIM2 IN STM32CubeMX:
  *   - TIM2 CH1: Input Capture Direct Mode, Rising Edge
  *   - Enable TIM2 global interrupt in NVIC
  *   - Prescaler = 47
  *   - Period = 65535
  * Then regenerate code and replace this init function.
  * -------------------------------------------------------
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig   = {0};
  TIM_MasterConfigTypeDef sMasterConfig       = {0};
  TIM_IC_InitTypeDef sConfigIC                = {0};
  TIM_OC_InitTypeDef sConfigOC               = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 47;          // 48MHz / (47+1) = 1MHz → 1 tick = 1us
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 65535;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)      { Error_Handler(); }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)         { Error_Handler(); }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)        { Error_Handler(); }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) { Error_Handler(); }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) { Error_Handler(); }

  /* CH1: Input Capture — Rising Edge — for encoder pulse timing */
  sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter    = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }

  /* CH2: PWM output — for motor speed control */
  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * Used in Task 3 (polling) as microsecond timer. Kept for reference.
  */
static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 47;   // 1 tick = 1us at 48MHz
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 65535;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance          = USART2;
  huart2.Init.BaudRate     = 115200;
  huart2.Init.WordLength   = UART_WORDLENGTH_8B;
  huart2.Init.StopBits     = UART_STOPBITS_1;
  huart2.Init.Parity       = UART_PARITY_NONE;
  huart2.Init.Mode         = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USB Initialization Function
  */
static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance                  = USB;
  hpcd_USB_FS.Init.dev_endpoints        = 8;
  hpcd_USB_FS.Init.speed                = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface           = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable     = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin  = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* PA4: kept as input (Task 3 polling pin, unused in Task 4) */
  /* PA7: encoder input (unused in Task 4)                     */
  GPIO_InitStruct.Pin  = GPIO_PIN_4|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PA5, PA6: Motor direction outputs */
  GPIO_InitStruct.Pin   = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PC4, PC5: Motor direction outputs */
  GPIO_InitStruct.Pin   = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* NOTE: PA0 (TIM2 CH1) is configured automatically by HAL_TIM_MspPostInit
     as Alternate Function when HAL_TIM_IC_Init is called. No manual config needed. */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */