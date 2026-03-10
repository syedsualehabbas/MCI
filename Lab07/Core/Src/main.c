/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 7 Task 2 - Signal Filtering Using CMSIS DSP
  ******************************************************************************
  * Hardware:
  * - Function generator -> PA0 (ADC1_IN1)
  * - UART2 PA2(TX)/PA3(RX) @ 115200 -> Python plotter
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILTER_LEN  10
#define ADC_REF_MV  3300U
#define ADC_MAX_VAL 4095U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef  hadc1;
I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */
static char      uart_buf[128];
static float32_t filter_buf[FILTER_LEN];
static uint8_t   filter_idx  = 0;
static volatile uint16_t adc_raw   = 0;
static volatile uint8_t  adc_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */
static void      UART_Printf(const char *fmt, ...);
static void      Filter_Init(void);
static float32_t Filter_Update(float32_t sample);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void UART_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(uart_buf, sizeof(uart_buf), fmt, ap);
    va_end(ap);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
}

static void Filter_Init(void)
{
    filter_idx = 0;
    arm_fill_f32(0.0f, filter_buf, FILTER_LEN);
}

static float32_t Filter_Update(float32_t new_sample)
{
    float32_t result;
    filter_buf[filter_idx] = new_sample;
    filter_idx = (filter_idx + 1) % FILTER_LEN;
    arm_mean_f32(filter_buf, FILTER_LEN, &result);
    return result;
}

/* ADC conversion complete callback - fires after every conversion */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_raw   = (uint16_t)HAL_ADC_GetValue(hadc);
        adc_ready = 1;
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  Filter_Init();
  HAL_ADC_Start_IT(&hadc1);
  UART_Printf("# LAB7 TASK2 START\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
      if (adc_ready)
      {
          adc_ready = 0;

          /* Convert raw count to mV */
          uint32_t raw_mv = ((uint32_t)adc_raw * ADC_REF_MV) / ADC_MAX_VAL;

          /* Convert to float volts and filter */
          float32_t voltage_v   = (float32_t)adc_raw * (3.3f / 4095.0f);
          float32_t filtered_v  = Filter_Update(voltage_v);
          uint32_t  filtered_mv = (uint32_t)(filtered_v * 1000.0f);

          /* Send to Python: raw_mV,filtered_mV */
          UART_Printf("%lu,%lu\r\n", raw_mv, filtered_mv);

          /* Small delay so UART can keep up with continuous ADC */
          HAL_Delay(10);
      }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                   | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_USART2
                                     | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection  = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef   multimode = {0};
  ADC_ChannelConfTypeDef sConfig   = {0};

  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode    = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) Error_Handler();

  sConfig.Channel      = ADC_CHANNEL_1;   /* PA0 = ADC1_IN1 */
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance              = I2C1;
  hi2c1.Init.Timing           = 0x2000090E;
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 7;
  hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode          = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) Error_Handler();
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance                    = USART2;
  huart2.Init.BaudRate               = 115200;
  huart2.Init.WordLength             = UART_WORDLENGTH_8B;
  huart2.Init.StopBits               = UART_STOPBITS_1;
  huart2.Init.Parity                 = UART_PARITY_NONE;
  huart2.Init.Mode                   = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling           = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_USB_PCD_Init(void)
{
  hpcd_USB_FS.Instance                     = USB;
  hpcd_USB_FS.Init.dev_endpoints           = 8;
  hpcd_USB_FS.Init.speed                   = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface              = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable        = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

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

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) {}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif