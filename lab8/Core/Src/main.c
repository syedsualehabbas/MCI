/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Lab 8 - Task 1 (commented) + Task 2 (commented) + Task 3 (active)
  *
  *  TASK 1 (commented out):
  *    Reads WHO_AM_I register (0x0F) from I3G4250D gyroscope via SPI polling.
  *    Expected return: 0xD3 (blue board). Prints result over UART.
  *
  *  TASK 2 (commented out):
  *    Reads temperature register (0x26) via SPI interrupt mode.
  *    Transmits signed temperature value over UART every ~150ms.
  *
  *  TASK 3 (active):
  *    Reads temperature + angular velocity (X, Y, Z) via SPI polling.
  *    Converts raw values to dps using sensitivity = 8.75 mdps/LSB.
  *    Output format: "<temp>, <x_dps>, <y_dps>, <z_dps>\n"
  *    Works with Lab8Task3_plot.py Python script.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */

/* ── Gyroscope Register Addresses ─────────────────────────────────────── */
#define GYRO_WHO_AM_I_ADDR   0x0F   /* Task 1: WHO_AM_I register          */
#define I3G4250D_WHO_AM_I    0xD3   /* Task 1: Expected ID - blue board   */
#define L3GD20_WHO_AM_I      0xD4   /* Task 1: Expected ID - green board  */

#define CTRL_REG1            0x20
#define CTRL_REG1_VAL        0x8F   /* PD=1, ODR=400Hz, Zen=Yen=Xen=1    */

#define CTRL_REG4            0x23
#define CTRL_REG4_VAL        0x00   /* FS = ±245 dps (default)            */

#define OUT_TEMP             0x26   /* Temperature output register        */
#define OUT_X_L              0x28   /* X-axis low byte (auto-inc from here)*/

/* ── SPI Command Bits ──────────────────────────────────────────────────── */
#define SPI_READ             0x80   /* Bit7=1 => read                     */
#define SPI_AUTO_INC         0x40   /* Bit6=1 => auto-increment address   */

/* ── Chip Select Pin ───────────────────────────────────────────────────── */
#define GYRO_CS_PIN          GPIO_PIN_3
#define GYRO_CS_PORT         GPIOE

/* USER CODE END PD */

/* USER CODE BEGIN PM */
#define GYRO_CS_LOW()   HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET)
/* USER CODE END PM */

I2C_HandleTypeDef  hi2c1;
SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart2;
PCD_HandleTypeDef  hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* ── Task 2: SPI state machine (commented out) ─────────────────────────── */
// typedef enum {
//     SPI_STATE_IDLE = 0,
//     SPI_STATE_TX_DONE,
//     SPI_STATE_RX_DONE
// } SPI_State_t;
// volatile SPI_State_t spiState = SPI_STATE_IDLE;
// uint8_t t2_tx_buf[1];
// uint8_t t2_rx_buf[1];
// volatile int8_t t2_temperature = 0;

/* ── Task 3: buffers ────────────────────────────────────────────────────── */
static uint8_t rx_buf[6];
static int8_t  temp_raw;
static int16_t x_raw, y_raw, z_raw;
static char    uart_buf[64];

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_PCD_Init(void);

/* USER CODE BEGIN PFP */

// ── Task 1 prototype (commented out) ──────────────────────────────────────
// static uint8_t Gyro_ReadRegister(uint8_t reg_addr);

// ── Task 2 prototypes (commented out) ─────────────────────────────────────
// void gyro_read_temp_start(void);
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

// ── Task 3 prototypes ──────────────────────────────────────────────────────
static void    gyro_write_reg(uint8_t reg, uint8_t val);
static uint8_t gyro_read_reg(uint8_t reg);
static void    gyro_init(void);
static void    gyro_read_all(void);
static int     write_dps(char *buf, int32_t raw);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* ════════════════════════════════════════════════════════════════════════
 * TASK 1 - WHO_AM_I Read via SPI Polling (COMMENTED OUT)
 * ════════════════════════════════════════════════════════════════════════
 * Usage in USER CODE BEGIN 2:
 *   uint8_t id = Gyro_ReadRegister(GYRO_WHO_AM_I_ADDR);
 *   sprintf(uart_buf, "WHO_AM_I = 0x%02X\r\n", id);
 *   HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);
 * ════════════════════════════════════════════════════════════════════════ */
// static uint8_t Gyro_ReadRegister(uint8_t reg_addr)
// {
//     uint8_t cmd     = SPI_READ | (reg_addr & 0x3F);
//     uint8_t rx_data = 0x00;
//     GYRO_CS_LOW();
//     HAL_SPI_Transmit(&hspi1, &cmd, 1, 100);
//     HAL_SPI_Receive(&hspi1, &rx_data, 1, 100);
//     GYRO_CS_HIGH();
//     return rx_data;
// }

/* ════════════════════════════════════════════════════════════════════════
 * TASK 2 - Temperature via SPI Interrupt Mode (COMMENTED OUT)
 * ════════════════════════════════════════════════════════════════════════ */
// void gyro_read_temp_start(void)
// {
//     t2_tx_buf[0] = SPI_READ | OUT_TEMP;
//     spiState     = SPI_STATE_IDLE;
//     GYRO_CS_LOW();
//     HAL_SPI_Transmit_IT(&hspi1, t2_tx_buf, 1);
// }
//
// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     if (hspi->Instance == SPI1)
//     {
//         spiState = SPI_STATE_TX_DONE;
//         HAL_SPI_Receive_IT(&hspi1, t2_rx_buf, 1);
//     }
// }
//
// void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     if (hspi->Instance == SPI1)
//     {
//         GYRO_CS_HIGH();
//         t2_temperature = (int8_t)t2_rx_buf[0];
//         spiState       = SPI_STATE_RX_DONE;
//     }
// }

/* ════════════════════════════════════════════════════════════════════════
 * TASK 3 - Angular Velocity + Temperature via SPI Polling (ACTIVE)
 * ════════════════════════════════════════════════════════════════════════ */

static void gyro_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x3F, val };
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
}

static uint8_t gyro_read_reg(uint8_t reg)
{
    uint8_t cmd  = SPI_READ | (reg & 0x3F);
    uint8_t data = 0;
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd,  1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, &data, 1, HAL_MAX_DELAY);
    GYRO_CS_HIGH();
    return data;
}

static void gyro_init(void)
{
    gyro_write_reg(CTRL_REG1, CTRL_REG1_VAL);  /* Power on, enable axes  */
    gyro_write_reg(CTRL_REG4, CTRL_REG4_VAL);  /* ±245 dps full scale    */
}

static void gyro_read_all(void)
{
    /* Read temperature */
    temp_raw = (int8_t)gyro_read_reg(OUT_TEMP);

    /* Burst read 6 bytes: OUT_X_L through OUT_Z_H using auto-increment */
    uint8_t cmd = SPI_READ | SPI_AUTO_INC | OUT_X_L;  /* 0xE8 */
    GYRO_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd,   1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, rx_buf, 6, HAL_MAX_DELAY);
    GYRO_CS_HIGH();

    /* Merge low and high bytes (little-endian) */
    x_raw = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    y_raw = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    z_raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
}

/**
 * @brief Convert raw gyro value to dps string without floats.
 *        Sensitivity = 8.75 mdps/LSB
 *        raw * 875 / 100 = value in units of 0.01 dps
 *        e.g. raw=141 -> 141*875/100=1233 -> "12.33" dps
 */
static int write_dps(char *buf, int32_t raw)
{
    int32_t scaled   = (raw * 875L) / 100L;  /* units of 0.01 dps */
    int     negative = (scaled < 0);
    if (negative) scaled = -scaled;

    int32_t whole = scaled / 100;
    int32_t frac  = scaled % 100;

    if (negative)
        return sprintf(buf, "-%ld.%02ld", (long)whole, (long)frac);
    else
        return sprintf(buf,  "%ld.%02ld", (long)whole, (long)frac);
}

/* USER CODE END 0 */

int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();
    MX_USB_PCD_Init();

    /* USER CODE BEGIN 2 */

    // ── Task 1 (commented out) ─────────────────────────────────────────
    // uint8_t who_am_i = Gyro_ReadRegister(GYRO_WHO_AM_I_ADDR);
    // if (who_am_i == I3G4250D_WHO_AM_I)
    //     sprintf(uart_buf, "WHO_AM_I = 0x%02X --> I3G4250D (PASS)\r\n", who_am_i);
    // else if (who_am_i == L3GD20_WHO_AM_I)
    //     sprintf(uart_buf, "WHO_AM_I = 0x%02X --> L3GD20 (PASS)\r\n", who_am_i);
    // else
    //     sprintf(uart_buf, "WHO_AM_I = 0x%02X --> UNKNOWN\r\n", who_am_i);
    // HAL_UART_Transmit(&huart2, (uint8_t*)uart_buf, strlen(uart_buf), 100);

    // ── Task 2 (commented out) ─────────────────────────────────────────
    // GYRO_CS_HIGH();
    // HAL_Delay(10);
    // gyro_init_t2();   /* Task 2 used its own init writing only CTRL_REG1=0x0F */
    // HAL_Delay(100);
    // -- while loop was: gyro_read_temp_start(); wait spiState; transmit; delay

    // ── Task 3 init ────────────────────────────────────────────────────
    GYRO_CS_HIGH();
    HAL_Delay(10);
    gyro_init();
    HAL_Delay(100);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */

        gyro_read_all();

        char xs[12], ys[12], zs[12];
        write_dps(xs, x_raw);
        write_dps(ys, y_raw);
        write_dps(zs, z_raw);

        int len = sprintf(uart_buf, "%d, %s, %s, %s\n",
                          (int)temp_raw, xs, ys, zs);

        HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, (uint16_t)len, HAL_MAX_DELAY);

        HAL_Delay(100);
    }
    /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit     = {0};

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
                                       | RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection    = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0x00201D2B;
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
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;    /* CPOL=1 */
    hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;      /* CPHA=1 */
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 7;
    hspi1.Init.CRCLength         = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
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

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                           | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin
                           | LD6_Pin, GPIO_PIN_RESET);

    /* CS starts HIGH - gyroscope deselected */
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin  = DRDY_Pin | MEMS_INT3_Pin | MEMS_INT4_Pin
                         | MEMS_INT1_Pin | MEMS_INT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin | LD4_Pin | LD3_Pin | LD5_Pin
                          | LD7_Pin | LD9_Pin | LD10_Pin | LD8_Pin | LD6_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
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